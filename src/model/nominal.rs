use micro_sp::*;
use std::error::Error;
use redis::aio::MultiplexedConnection;

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let auto_transitions = vec![];
    let sops = vec![];
    let mut operations = vec![];

    operations.push(Operation::new(
        "gantry_unlock",
        None,
        None,
        None, 
        None,
        false,
        Vec::from([Transition::parse(
            "start_gantry_unlock",
            "var:gantry_request_state == initial \
                && var:gantry_request_trigger == false",
            "true",
             
            vec![
                &format!("var:gantry_command_command <- unlock"),
                "var:gantry_request_trigger <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "complete_gantry_unlock",
            "true",
            "var:gantry_request_state == succeeded",
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_locked_estimated <- false",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    ));

    operations.push(Operation::new(
        "gantry_lock",
        None,
        None,
        None, 
        None,
        false,
        Vec::from([Transition::parse(
            "start_gantry_lock",
            "var:gantry_request_state == initial \
                && var:gantry_request_trigger == false",
            "true",
             
            vec![
                &format!("var:gantry_command_command <- lock"),
                "var:gantry_request_trigger <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "complete_gantry_lock",
            "true",
            "var:gantry_request_state == succeeded",
             
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_locked_estimated <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    ));

    operations.push(Operation::new(
        "gantry_calibrate",
        None,
        None,
        None, 
        None,
        false,
        Vec::from([Transition::parse(
            "start_gantry_calibrate",
            "var:gantry_locked_estimated == false \
                && var:gantry_request_state == initial \
                && var:gantry_request_trigger == false",
            "true",
             
            vec![
                &format!("var:gantry_command_command <- calibrate"),
                "var:gantry_request_trigger <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "complete_gantry_calibrate",
            "true",
            "var:gantry_request_state == succeeded",
             
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_calibrated_estimated <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    ));

    for pos in vec!["home", "pipe_blue_box", "plate_pipe_box"] {
        operations.push(Operation::new(
            &format!("gantry_move_to_{}", pos),
            None,
            None,
            None, 
            None,
            false,
            Vec::from([Transition::parse(
                &format!("start_gantry_move_to_{}", pos),
                "var:gantry_request_state == initial \
                    && var:gantry_request_trigger == false \
                    && var:gantry_locked_estimated == false \
                    && var:gantry_calibrated_estimated == true",
                "true",
                vec![
                    &format!("var:gantry_command_command <- move"),
                    &format!("var:gantry_position_command <- {pos}"),
                    &format!("var:gantry_speed_command <- 0.5"),
                    "var:gantry_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_gantry_move_to_{}", pos),
                "true",
                &format!("var:gantry_request_state == succeeded"),
                 
                vec![
                    "var:gantry_request_trigger <- false",
                    "var:gantry_request_state <- initial",
                    &format!("var:gantry_position_estimated <- {pos}"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    for pos in vec![
        "a",
        "b",
        "c",
        "d",
        "pipe_blue_box",
        "plate_pipe_box",
        "gripper_tool_rack",
        "suction_tool_rack",
    ] {
        operations.push(Operation::new(
            &format!("robot_move_to_{}", pos),
            None,
            None,
            None, 
            None,
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_move_to_{}", pos),
                "var:robot_request_state == initial \
                && var:robot_request_trigger == false \
                && var:gantry_locked_estimated == true \
                && var:gantry_calibrated_estimated == true",
                "true",
                vec![
                    &format!("var:robot_command_command <- move"),
                    &format!("var:robot_position_command <- {pos}"),
                    &format!("var:robot_speed_command <- 0.5"),
                    "var:robot_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_robot_move_to_{}", pos),
                "true",
                &format!("var:robot_request_state == succeeded"),
                 
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_position_estimated <- {pos}"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    for tool in vec!["gripper_tool", "suction_tool"] {
        operations.push(Operation::new(
            &format!("robot_check_for_{tool}_mounted"),
            None,
            None,
            None, 
            None,
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_check_for_{tool}_mounted"),
                &format!(
                    "(var:robot_mounted_checked == false || var:robot_mounted_checked == UNKNOWN_bool) \
                    && var:robot_request_state == initial \
                    && var:robot_request_trigger == false \
                    && var:robot_mounted_estimated == UNKNOWN_string"
                ),
                "true",
                vec![
                    &format!("var:robot_command_command <- check_mounted_tool"),
                    "var:robot_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([
                Transition::parse(
                    &format!("complete_robot_check_for_{tool}_mounted"),
                    "true",
                    &format!("var:robot_request_state == succeeded && var:robot_mounted_one_time_measured == {tool}"),
                    vec![
                        "var:robot_request_trigger <- false",
                        "var:robot_request_state <- initial",
                        "var:robot_mounted_checked <- true",
                        &format!("var:robot_mounted_estimated <- {tool}")
                    ],
                    Vec::<&str>::new(),
                    &state,
                ),
                Transition::parse(
                    &format!("complete_robot_check_for_{tool}_mounted_2"),
                    "true",
                    &format!("var:robot_request_state == succeeded && var:robot_mounted_one_time_measured != {tool}"),
                     
                    vec![
                        "var:robot_request_trigger <- false",
                        "var:robot_request_state <- initial",
                        "var:robot_mounted_checked <- true",
                        &format!("var:robot_mounted_estimated <- var:robot_mounted_one_time_measured"),
                        &format!("var:{sp_id}_replan_trigger <- true"),
                        &format!("var:{sp_id}_replanned <- false"),
                    ],
                    Vec::<&str>::new(),
                    &state,
                )
            ]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    for tool in vec!["gripper_tool", "suction_tool"] {
        operations.push(Operation::new(
            &format!("robot_mount_{}", tool),
            None,
            None,
            None, 
            None,
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_mount_{}", tool),
                &format!(
                    "var:robot_request_state == initial \
                    && var:robot_request_trigger == false \
                    && var:robot_position_estimated == {tool}_rack \
                    && var:robot_mounted_estimated == none \
                    && var:gantry_locked_estimated == true",
                ),
                "true",
                vec![
                    &format!("var:robot_command_command <- mount"),
                    "var:robot_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_robot_mount_{}", tool),
                "true",
                &format!("var:robot_request_state == succeeded"),
                 
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_mounted_estimated <- {tool}"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    for tool in vec!["gripper_tool", "suction_tool"] {
        operations.push(Operation::new(
            &format!("robot_unmount_{tool}"),
            None,
            None,
            None, 
            None,
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_unmount_{tool}"),
                &format!(
                    "var:robot_request_state == initial \
                    && var:robot_request_trigger == false \
                    && var:robot_position_estimated == {tool}_rack \
                    && var:robot_mounted_estimated == {tool} \
                    && var:gantry_locked_estimated == true"
                ),
                "true",
                 
                vec![
                    &format!("var:robot_command_command <- unmount"),
                    "var:robot_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_robot_unmount_{tool}"),
                "true",
                &format!("var:robot_request_state == succeeded"),
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_mounted_estimated <- none"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    let model = Model::new(sp_id, auto_transitions, vec!(), sops, operations);

    (model, state)
}

pub async fn run_emultaion(sp_id: &str, mut con: MultiplexedConnection) -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    let goal = "var:robot_mounted_estimated == suction_tool";

    if let Some(state) = StateManager::get_full_state(&mut con).await {
        let plan_state = state.get_string_or_default_to_unknown(
            &format!("{}_plan_state", sp_id),
            &format!("{}_emulation_test", sp_id),
        );

        if PlanState::from_str(&plan_state) == PlanState::Failed
            || PlanState::from_str(&plan_state) == PlanState::Completed
            || PlanState::from_str(&plan_state) == PlanState::Initial
            || PlanState::from_str(&plan_state) == PlanState::UNKNOWN
        {
            let new_state = state
                // Optional to test what happens when... (look in the Emulation msg for details)
                .update("gantry_emulate_execution_time", 1.to_spvalue())
                .update("gantry_emulated_execution_time", 0.to_spvalue())
                .update("robot_emulate_execution_time", 1.to_spvalue())
                .update("robot_emulated_execution_time", 0.to_spvalue())
                .update("gantry_emulate_failure_rate", 0.to_spvalue())
                .update("gantry_emulated_failure_rate", 50.to_spvalue())
                .update("gantry_emulate_failure_cause", 2.to_spvalue())
                .update(
                    "gantry_emulated_failure_cause",
                    vec!["violation", "collision", "detected_drift"].to_spvalue(),
                )
                .update(
                    &format!("{sp_id}_current_goal_state"),
                    CurrentGoalState::Initial.to_spvalue(),
                )
                .update(
                    &format!("{sp_id}_current_goal_predicate"),
                    goal.to_spvalue(),
                )
                .update(&format!("{sp_id}_replan_trigger"), true.to_spvalue())
                .update(&format!("{sp_id}_replanned"), false.to_spvalue());

            let modified_state = state.get_diff_partial_state(&new_state);
            StateManager::set_state(&mut con, &modified_state).await;
        }
    }

    // TODO
    // Measure operation and plan execution times, and measure total failure rates...
    // Print out plan done or plan failed when done or failed...
    // Generate a RUN report contining time, timeouts, failures, recoveries, paths taken, replans, etc.

    Ok(())
}
