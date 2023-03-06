use micro_sp::*;

// use r2r::ur_controller_msgs::action::URControl;
use std::sync::{Arc, Mutex};

pub fn extract_goal_from_state(state: &State) -> Predicate {
    match state.state.get("runner_goal") {
        Some(g_spvalue) => match &g_spvalue.val {
            SPValue::String(g_value) => match pred_parser::pred(&g_value, &state) {
                Ok(goal_predicate) => goal_predicate,
                Err(_) => Predicate::TRUE,
            },
            _ => Predicate::TRUE,
            // TODO: log errors...
        },
        None => Predicate::TRUE,
    }
}

// ugh fix this, make it recursive...
pub async fn update_shared_state(
    // var: &str,
    // val: SPValue,
    // updates: Vec<(&str, SPValue)>,
    update: (&str, SPValue),
    shared_state: &Arc<Mutex<State>>,
) -> State {
    let shared_state_local = shared_state.lock().unwrap().clone();
    // let mut updated_state = State::new();
    let updated_state = shared_state_local.update(update.0, update.1.to_owned());
    // updates
    //     .iter()
    //     .for_each(|(var, val)| updated_state = shared_state_local.update(var, val.to_owned()));
    // let updated_state = shared_state_local.update(var, val);
    *shared_state.lock().unwrap() = updated_state.clone();
    updated_state
}

pub async fn ticker(
    node_id: &str,
    // ur_action_client: &r2r::ActionClient<URControl::Action>,
    model: &Model,
    shared_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
) -> Result<(), Box<dyn std::error::Error>> {
    // wait for the measured values to update the state
    // tokio::time::sleep(std::time::Duration::from_millis(5000)).await;
    let shared_state_local = shared_state.lock().unwrap().clone();
    let mut old_state = shared_state_local.clone();
    loop {
        let shared_state_local = shared_state.lock().unwrap().clone();
        let replan = shared_state_local.get_value("runner_replan");
        let replanned = shared_state_local.get_value("runner_replanned");

        match (replan, replanned) {
            (SPValue::Bool(true), SPValue::Bool(true)) => {
                update_shared_state(("runner_replan", false.to_spvalue()), shared_state).await;
                update_shared_state(("runner_replanned", false.to_spvalue()), shared_state).await
            }
            (SPValue::Bool(true), SPValue::Bool(false)) => {
                let new_state =
                    update_shared_state(("runner_replanned", true.to_spvalue()), shared_state)
                        .await;
                let goal = extract_goal_from_state(&new_state);
                println!("REPLAN TRIGGERED!");
                let new_plan =
                    bfs_operation_planner(new_state.clone(), goal, model.operations.clone(), 30);
                match new_plan.found {
                    false => {
                        r2r::log_warn!(node_id, "No plan was found.");
                        new_state
                    }
                    true => match new_plan.length == 0 {
                        true => {
                            r2r::log_warn!(node_id, "We are already in the goal.");
                            new_state
                        }
                        false => {
                            r2r::log_info!(node_id, "A new plan was found: {:?}.", new_plan.plan);
                            update_shared_state(
                                ("runner_plan", new_plan.plan.to_spvalue()),
                                shared_state,
                            )
                            .await
                        }
                    },
                }
            }
            (SPValue::Bool(false), _) => {
                update_shared_state(("runner_replanned", false.to_spvalue()), shared_state).await
            }
            (_, _) => shared_state_local,
        };

        let new_state = tick_the_runner(node_id, &model, &shared_state).await;

        if new_state != old_state {
            println!("STATE: {}", new_state);
        }

        old_state = new_state.clone();

        *shared_state.lock().unwrap() = new_state.clone();

        timer.tick().await?;
    }
}

async fn tick_the_runner(node_id: &str, model: &Model, shared_state: &Arc<Mutex<State>>) -> State {
    //Result<(), Box<dyn std::error::Error>> {
    let mut state = shared_state.lock().unwrap().clone();

    let old_state = state.clone();

    // Here you can execute the free transitions by checking if they are enabled and then do take_running on them
    model.transitions.iter().for_each(|t| {
        if t.clone().eval_running(&state) {
            state = t.clone().take_running(&state);
        } else {
            state = state.clone()
        }
    });

    // TODO: allow unknown in domain of all variables
    let the_plan = state.get_value("runner_plan");
    match the_plan {
        SPValue::Array(_, plan) => match plan.is_empty() {
            true => {
                update_shared_state(("runner_plan_status", SPValue::Unknown), shared_state).await;
                update_shared_state(("runner_plan", SPValue::Unknown), shared_state).await;
                update_shared_state(("runner_plan_current_step", SPValue::Unknown), shared_state)
                    .await
            }
            // we have not started executing the plan so we start at position 0 in the plan
            false => match state.get_value("runner_plan_current_step") {
                SPValue::Unknown => {
                    update_shared_state(("runner_plan_current_step", 0.to_spvalue()), shared_state)
                        .await
                }
                SPValue::Int32(current_step_in_plan) => match plan.len()
                    <= current_step_in_plan as usize
                {
                    true => {
                        // we are done with the plan and will stop executing and we also
                        // reset the current plan so we do not tries to run the same plan again
                        update_shared_state(
                            ("runner_plan_status", "done".to_spvalue()),
                            shared_state,
                        )
                        .await;
                        update_shared_state(("runner_plan", SPValue::Unknown), shared_state).await;
                        update_shared_state(
                            ("runner_plan_current_step", SPValue::Unknown),
                            shared_state,
                        )
                        .await
                    }
                    false => {
                        let current_op_name = match plan[current_step_in_plan as usize].clone() {
                            SPValue::String(op_name) => op_name.to_string(),
                            _ => panic!("no sucnext_stateh op name"),
                        };
                        let current_op_state = state.get_value(&current_op_name);
                        let current_op = model
                            .operations
                            .iter()
                            .find(|op| op.name == current_op_name)
                            .unwrap();

                        let next_step_in_plan = current_step_in_plan + 1;

                        if current_op_state == "initial".to_spvalue()
                            && current_op.clone().eval_running(&state)
                        {
                            // The operation can be started
                            current_op.clone().start_running(&state)
                        } else if current_op_state == "initial".to_spvalue()
                            && !current_op.clone().eval_running(&state)
                        {
                            // The operation should be started but is not enabled
                            state.update(
                                "runner_plan_status",
                                format!("Waiting for {current_op_name} to be enabled.")
                                    .to_spvalue(),
                            )
                        } else if current_op.clone().can_be_completed(&state) {
                            // the operation has completed and we can take a step in the plan
                            let next_state = current_op.clone().complete_running(&state);
                            let next_state = next_state
                                .update("runner_plan_current_step", next_step_in_plan.to_spvalue());
                            next_state.update(
                                "runner_plan_status",
                                format!("Completed step {current_step_in_plan}.").to_spvalue(),
                            )
                        } else if current_op_state == "executing".to_spvalue() {
                            state.update(
                                "runner_plan_status",
                                format!("Waiting for {current_op_name} to complete.").to_spvalue(),
                            )
                        } else {
                            state.update(
                                "runner_plan_status",
                                format!("Doing nothing.").to_spvalue(),
                            )
                        }

                        // *shared_state.lock().unwrap() = next_state.clone();
                        // next_state // if I need to publish the state
                    }
                },
                _ => old_state.clone(),
            },
        },
        SPValue::Unknown => old_state.clone(),
        _ => panic!("runner_plan should be Array type."),
    }

    // let new_state = shared_state.lock().unwrap().clone();

    // if next_state != old_state {
    // println!("STATE: {}", next_state);
    // }

    // *shared_state.lock().unwrap() = next_state.clone();

    // Ok(())
}
