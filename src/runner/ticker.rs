// use micro_sp::*;
// use ordered_float::OrderedFloat;
// use std::sync::Arc;

// use std::time::{SystemTime, UNIX_EPOCH};

// pub fn extract_goal_from_state(state: &State) -> Predicate {
//     match state.state.get("runner_goal") {
//         Some(g_spvalue) => match &g_spvalue.val {
//             SPValue::String(g_value) => match pred_parser::pred(&g_value, &state) {
//                 Ok(goal_predicate) => goal_predicate,
//                 Err(_) => Predicate::TRUE,
//             },
//             _ => Predicate::TRUE,
//         },
//         None => Predicate::TRUE,
//     }
// }

// pub fn reset_all_operations(state: &State) -> State {
//     let state = state.clone();
//     let mut mut_state = state.clone();
//     state.state.iter().for_each(|(k, _)| {
//         if k.starts_with("op_") {
//             mut_state = mut_state.update(&k, "initial".to_spvalue());
//         }
//     });
//     mut_state
// }

// pub async fn ticker(
//     node_id: &str,
//     model: &Model,
//     // shared_state: &Arc<Mutex<State>>,
//     sharded_mutex: &Arc<ShardedMutex>,
//     mut timer: r2r::Timer,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     // wait for the measured values to update the state
//     // tokio::time::sleep(std::time::Duration::from_millis(5000)).await;
//     let sharded_mutex_ref = &**sharded_mutex;
//     let arc_mutex = sharded_mutex_ref.collect_all();
//     let mut old_state = arc_mutex.lock().unwrap().clone();

//     loop {
//         // should be ok to do this until data is only read?
//         let sharded_mutex_ref = &**sharded_mutex;
//         let arc_mutex = sharded_mutex_ref.collect_all();
//         let shsl = arc_mutex.lock().unwrap().clone();

//         let runner_replan_shard = sharded_mutex.lock("runner_replan").clone();
//         let replan = match runner_replan_shard.get_value("runner_replan") {
//             SPValue::Bool(value) => value,
//             _ => false,
//         };
//         let runner_replanned_shard = sharded_mutex.lock("runner_replanned").clone();
//         let replanned = match runner_replanned_shard.get_value("runner_replanned") {
//             SPValue::Bool(value) => value,
//             _ => false,
//         };
//         let runner_replan_counter_shard = sharded_mutex.lock("runner_replan_counter").clone();
//         let replan_counter = match runner_replan_counter_shard.get_value("runner_replan_counter") {
//             SPValue::Int32(value) => value,
//             _ => 0,
//         };

//         let new_state = match (replan, replanned) {
//             (true, true) => {
//                 // *sharded_mutex.lock("runner_replan") =
//                 //     runner_replan_shard.update("runner_replan", false.to_spvalue());
//                 // *sharded_mutex.lock("runner_replanned") =
//                 //     runner_replanned_shard.update("runner_replanned", false.to_spvalue());
//                 shsl.update("runner_replanned", false.to_spvalue())
//                     .update("runner_replan", false.to_spvalue())
//             }
//             (true, false) => {
//                 let goal = extract_goal_from_state(&shsl);
//                 // *sharded_mutex.lock("runner_replanned") =
//                 //     runner_replan_shard.update("runner_replanned", true.to_spvalue());
//                 // *sharded_mutex.lock("runner_replan_counter") = runner_replanned_shard
//                 //     .update("runner_replan_counter", (replan_counter + 1).to_spvalue());
//                 let new_state = shsl
//                     .update("runner_replanned", true.to_spvalue())
//                     .update("runner_replan_counter", (replan_counter + 1).to_spvalue());
//                 let new_state = reset_all_operations(&new_state);
//                 r2r::log_warn!(node_id, "Re-plan triggered in the following state:");
//                 println!("{}", new_state);
//                 let new_plan =
//                     bfs_operation_planner(new_state.clone(), goal, model.operations.clone(), 30);
//                 match new_plan.found {
//                     false => {
//                         r2r::log_warn!(node_id, "No plan was found.");
//                         new_state
//                     }
//                     true => match new_plan.length == 0 {
//                         true => {
//                             r2r::log_warn!(node_id, "We are already in the goal.");
//                             new_state
//                         }
//                         false => {
//                             r2r::log_warn!(node_id, "A new plan was found: {:?}.", new_plan.plan);
//                             // let runner_plan_shard = sharded_mutex.lock("runner_plan").clone();
//                             // *sharded_mutex.lock("runner_plan") =
//                             //     runner_plan_shard.update("runner_plan", new_plan.plan.to_spvalue());
//                             new_state.update("runner_plan", new_plan.plan.to_spvalue())
//                         }
//                     },
//                 }
//             }
//             (false, _) => {
//                 *sharded_mutex.lock("runner_replanned") =
//                     runner_replanned_shard.update("runner_replanned", false.to_spvalue());
//                 shsl.update("runner_replanned", false.to_spvalue())
//             }
//         };

//         let new_state = tick_the_runner(node_id, &model, &new_state).await;

//         if new_state != old_state {
//             println!("{}", new_state);
//         }

//         // old_state = new_state.clone();
//         // let mut to_update = vec![
//         //     "runner_replan",
//         //     "runner_replanned",
//         //     "runner_replan_counter",
//         //     "runner_plan",
//         //     "runner_plan_status",
//         //     "runner_plan_current_step",
//         //     "runner_goal",
//         // ];
//         // let mut also_to_update = vec![];
//         // new_state.state.iter().for_each(|(k, _)| {
//         //     if k.starts_with("op_") {
//         //         also_to_update.push(k.as_str())
//         //     }
//         //     if k.starts_with("deadline_") {
//         //         also_to_update.push(k.as_str())
//         //     }
//         //     if k.starts_with("timeouts_") {
//         //         also_to_update.push(k.as_str())
//         //     }
//         //     if k.starts_with("timestamp_") {
//         //         also_to_update.push(k.as_str())
//         //     }
//         // });

//         // to_update.extend(also_to_update);

//         // update only the values that are changed by this ticker
//         for key in new_state.state.keys() {
//             if !vec!("scanner_request_state", "fail_counter_scanner").contains(&key.as_str()) {
//                 println!("TOUPDATEKEY: {key}");
//                 let shard = sharded_mutex.lock(key).clone();
//                 let new_shard = shard.update(key, new_state.get_value(key));
//                 *sharded_mutex.lock(key) = new_shard;
//             }
//         }
//         timer.tick().await?;
//     }
// }

// async fn tick_the_runner(node_id: &str, model: &Model, shared_state: &State) -> State {
//     let mut shsl = shared_state.clone();

//     for t in &model.auto_transitions {
//         if t.clone().eval_running(&shsl) {
//             r2r::log_warn!(node_id, "Taking the free transition: {}.", t.name);
//             shsl = t.clone().take_running(&shsl);
//         }
//     }

//     match shsl.get_value("runner_plan") {
//         SPValue::Array(_, plan) => match plan.is_empty() {
//             true => shsl
//                 .update("runner_plan_status", SPValue::Unknown)
//                 .update("runner_plan", SPValue::Unknown)
//                 .update("runner_plan_current_step", SPValue::Unknown),

//             // we have not started executing the plan so we start at position 0 in the plan
//             false => match shsl.get_value("runner_plan_current_step") {
//                 SPValue::Unknown => shsl.update("runner_plan_current_step", 0.to_spvalue()),
//                 SPValue::Int32(curr_step) => match plan.len() <= curr_step as usize {
//                     // we are done with the plan and will stop executing and we also
//                     // reset the current plan so we do not tries to run the same plan again
//                     true => shsl
//                         .update("runner_plan_status", "done".to_spvalue())
//                         .update("runner_goal", SPValue::Unknown)
//                         .update("runner_plan", SPValue::Unknown)
//                         .update("runner_plan_current_step", SPValue::Unknown),

//                     false => {
//                         let current_op_name = match plan[curr_step as usize].clone() {
//                             SPValue::String(op_name) => op_name.to_string(),
//                             _ => panic!("no such op name"),
//                         };
//                         let current_op_state = shsl.get_value(&current_op_name);
//                         let current_op = model
//                             .operations
//                             .iter()
//                             .find(|op| op.name == current_op_name)
//                             .unwrap();

//                         let next_step_in_plan = curr_step + 1;

//                         if current_op_state == "initial".to_spvalue() {
//                             if current_op.clone().eval_running(&shsl) {
//                                 // The operation can be started

//                                 let start = SystemTime::now();
//                                 let since_the_epoch = start
//                                     .duration_since(UNIX_EPOCH)
//                                     .expect("Time went backwards")
//                                     .as_secs_f64();
//                                 let shsl = shsl.update(
//                                     &format!("timestamp_{}", current_op_name),
//                                     since_the_epoch.to_spvalue(),
//                                 );
//                                 current_op.clone().start_running(&shsl)
//                             } else {
//                                 // The operation can be started but is not enabled
//                                 shsl.update(
//                                     "runner_plan_status",
//                                     format!("Waiting for {current_op_name} to be enabled.")
//                                         .to_spvalue(),
//                                 )
//                             }
//                         } else if current_op_state == "executing".to_spvalue() {
//                             if current_op.clone().can_be_completed(&shsl) {
//                                 // complete the operation and take a step in the plan
//                                 let shsl = current_op.clone().complete_running(&shsl);
//                                 shsl.update(
//                                     "runner_plan_current_step",
//                                     next_step_in_plan.to_spvalue(),
//                                 )
//                                 .update(
//                                     "runner_plan_status",
//                                     format!("Completed step {curr_step}.").to_spvalue(),
//                                 )
//                             } else {
//                                 // the operation is still executing, check if operation timeout is exceeded
//                                 let timestamp_current_op = match shsl
//                                     .get_value(&format!("timestamp_{}", current_op_name))
//                                 {
//                                     SPValue::Float64(OrderedFloat(timestamp)) => timestamp,
//                                     _ => 0.0,
//                                 };
//                                 let deadline_current_op = match shsl
//                                     .get_value(&format!("deadline_{}", current_op_name))
//                                 {
//                                     SPValue::Float64(OrderedFloat(deadline)) => deadline,
//                                     _ => 0.0,
//                                 };
//                                 let start = SystemTime::now();
//                                 let since_the_epoch = start
//                                     .duration_since(UNIX_EPOCH)
//                                     .expect("Time went backwards")
//                                     .as_secs_f64();
//                                 if (since_the_epoch - timestamp_current_op) > deadline_current_op {
//                                     let nr_timeouts = match shsl
//                                         .get_value(&format!("timeouts_{}", current_op_name))
//                                     {
//                                         SPValue::Int32(nr_timeouts) => nr_timeouts,
//                                         _ => 0,
//                                     };
//                                     shsl.update(
//                                         "runner_plan_status",
//                                         format!("Operation {current_op_name} timed out.")
//                                             .to_spvalue(),
//                                     )
//                                     .update(
//                                         &format!("timestamp_{}", current_op_name),
//                                         since_the_epoch.to_spvalue(),
//                                     )
//                                     .update(
//                                         &format!("timeouts_{}", current_op_name),
//                                         (nr_timeouts + 1).to_spvalue(),
//                                     )
//                                 } else {
//                                     shsl.update(
//                                         "runner_plan_status",
//                                         format!("Waiting for {current_op_name} to complete.")
//                                             .to_spvalue(),
//                                     )
//                                 }
//                             }
//                         } else {
//                             // this shouldn't really happen
//                             shsl.update(
//                                 "runner_plan_status",
//                                 format!("Doing nothing.").to_spvalue(),
//                             )
//                         }
//                     }
//                 },
//                 _ => shsl.clone(),
//             },
//         },
//         SPValue::Unknown => shsl.clone(),
//         _ => panic!("runner_plan should be Array type."),
//     }
// }

use micro_sp::*;
use ordered_float::OrderedFloat;
use std::sync::{Arc, Mutex};

use std::time::{SystemTime, UNIX_EPOCH};

pub fn extract_goal_from_state(state: &State) -> Predicate {
    match state.state.get("runner_goal") {
        Some(g_spvalue) => match &g_spvalue.val {
            SPValue::String(g_value) => match pred_parser::pred(&g_value, &state) {
                Ok(goal_predicate) => goal_predicate,
                Err(_) => Predicate::TRUE,
            },
            _ => Predicate::TRUE,
        },
        None => Predicate::TRUE,
    }
}

pub fn reset_all_operations(state: &State) -> State {
    let state = state.clone();
    let mut mut_state = state.clone();
    state.state.iter().for_each(|(k, _)| {
        if k.starts_with("op_") {
            mut_state = mut_state.update(&k, "initial".to_spvalue());
        }
    });
    mut_state
}

pub async fn ticker(
    node_id: &str,
    model: &Model,
    shared_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
) -> Result<(), Box<dyn std::error::Error>> {
    // wait for the measured values to update the state
    // tokio::time::sleep(std::time::Duration::from_millis(5000)).await;
    let shared_state_local = shared_state.lock().unwrap().clone();
    let mut old_state = shared_state_local.clone();
    loop {
        let shsl = shared_state.lock().unwrap().clone();
        let replan = shsl.get_value("runner_replan");
        let replanned = shsl.get_value("runner_replanned");
        let replan_counter = match shsl.get_value("runner_replan_counter") {
            SPValue::Int32(value) => value,
            _ => 0,
        };

        let new_state = match (replan, replanned) {
            (SPValue::Bool(true), SPValue::Bool(true)) => shsl
                .update("runner_replan", false.to_spvalue())
                .update("runner_replanned", false.to_spvalue()),
            (SPValue::Bool(true), SPValue::Bool(false)) => {
                let goal = extract_goal_from_state(&shsl);
                let new_state = shsl
                    .update("runner_replanned", true.to_spvalue())
                    .update("runner_replan_counter", (replan_counter + 1).to_spvalue());
                let new_state = reset_all_operations(&new_state);
                r2r::log_warn!(node_id, "Re-plan triggered in the following state:");
                println!("{}", new_state);
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
                            r2r::log_warn!(node_id, "A new plan was found: {:?}.", new_plan.plan);
                            new_state.update("runner_plan", new_plan.plan.to_spvalue())
                        }
                    },
                }
            }
            (SPValue::Bool(false), _) => shsl.update("runner_replanned", false.to_spvalue()),
            (_, _) => shsl,
        };

        let new_state = tick_the_runner(node_id, &model, &new_state).await;

        if new_state != old_state {
            println!("{}", new_state);
        }

        old_state = new_state.clone();

        *shared_state.lock().unwrap() = new_state.clone();

        timer.tick().await?;
    }
}

async fn tick_the_runner(node_id: &str, model: &Model, shared_state: &State) -> State {
    let mut shsl = shared_state.clone();

    for t in &model.auto_transitions {
        if t.clone().eval_running(&shsl) {
            r2r::log_warn!(node_id, "Taking the free transition: {}.", t.name);
            shsl = t.clone().take_running(&shsl);
        }
    }

    match shsl.get_value("runner_plan") {
        SPValue::Array(_, plan) => match plan.is_empty() {
            true => shsl
                .update("runner_plan_status", SPValue::Unknown)
                .update("runner_plan", SPValue::Unknown)
                .update("runner_plan_current_step", SPValue::Unknown),

            // we have not started executing the plan so we start at position 0 in the plan
            false => match shsl.get_value("runner_plan_current_step") {
                SPValue::Unknown => shsl.update("runner_plan_current_step", 0.to_spvalue()),
                SPValue::Int32(curr_step) => match plan.len() <= curr_step as usize {
                    // we are done with the plan and will stop executing and we also
                    // reset the current plan so we do not tries to run the same plan again
                    true => shsl
                        .update("runner_plan_status", "done".to_spvalue())
                        .update("runner_goal", SPValue::Unknown)
                        .update("runner_plan", SPValue::Unknown)
                        .update("runner_plan_current_step", SPValue::Unknown),

                    false => {
                        let current_op_name = match plan[curr_step as usize].clone() {
                            SPValue::String(op_name) => op_name.to_string(),
                            _ => panic!("no such op name"),
                        };
                        let current_op_state = shsl.get_value(&current_op_name);
                        let current_op = model
                            .operations
                            .iter()
                            .find(|op| op.name == current_op_name)
                            .unwrap();

                        let next_step_in_plan = curr_step + 1;

                        if current_op_state == "initial".to_spvalue() {
                            if current_op.clone().eval_running(&shsl) {
                                // The operation can be started

                                let start = SystemTime::now();
                                let since_the_epoch = start
                                    .duration_since(UNIX_EPOCH)
                                    .expect("Time went backwards")
                                    .as_secs_f64();
                                let shsl = shsl.update(
                                    &format!("timestamp_{}", current_op_name),
                                    since_the_epoch.to_spvalue(),
                                );
                                current_op.clone().start_running(&shsl)
                            } else {
                                // The operation can be started but is not enabled
                                shsl.update(
                                    "runner_plan_status",
                                    format!("Waiting for {current_op_name} to be enabled.")
                                        .to_spvalue(),
                                )
                            }
                        } else if current_op_state == "executing".to_spvalue() {
                            if current_op.clone().can_be_completed(&shsl) {
                                // complete the operation and take a step in the plan
                                let shsl = current_op.clone().complete_running(&shsl);
                                shsl.update(
                                    "runner_plan_current_step",
                                    next_step_in_plan.to_spvalue(),
                                )
                                .update(
                                    "runner_plan_status",
                                    format!("Completed step {curr_step}.").to_spvalue(),
                                )
                            } else {
                                // the operation is still executing, check if operation timeout is exceeded
                                let timestamp_current_op = match shsl
                                    .get_value(&format!("timestamp_{}", current_op_name))
                                {
                                    SPValue::Float64(OrderedFloat(timestamp)) => timestamp,
                                    _ => 0.0,
                                };
                                let deadline_current_op = match shsl
                                    .get_value(&format!("deadline_{}", current_op_name))
                                {
                                    SPValue::Float64(OrderedFloat(deadline)) => deadline,
                                    _ => 0.0,
                                };
                                let start = SystemTime::now();
                                let since_the_epoch = start
                                    .duration_since(UNIX_EPOCH)
                                    .expect("Time went backwards")
                                    .as_secs_f64();
                                if (since_the_epoch - timestamp_current_op) > deadline_current_op {
                                    let nr_timeouts = match shsl
                                        .get_value(&format!("timeouts_{}", current_op_name))
                                    {
                                        SPValue::Int32(nr_timeouts) => nr_timeouts,
                                        _ => 0,
                                    };
                                    shsl.update(
                                        "runner_plan_status",
                                        format!("Operation {current_op_name} timed out.")
                                            .to_spvalue(),
                                    )
                                    .update(
                                        &format!("timestamp_{}", current_op_name),
                                        since_the_epoch.to_spvalue(),
                                    )
                                    .update(
                                        &format!("timeouts_{}", current_op_name),
                                        (nr_timeouts + 1).to_spvalue(),
                                    )
                                } else {
                                    shsl.update(
                                        "runner_plan_status",
                                        format!("Waiting for {current_op_name} to complete.")
                                            .to_spvalue(),
                                    )
                                }
                            }
                        } else {
                            // this shouldn't really happen
                            shsl.update(
                                "runner_plan_status",
                                format!("Doing nothing.").to_spvalue(),
                            )
                        }
                    }
                },
                _ => shsl.clone(),
            },
        },
        SPValue::Unknown => shsl.clone(),
        _ => panic!("runner_plan should be Array type."),
    }
}
