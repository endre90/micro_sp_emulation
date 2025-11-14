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
                .update("gantry_emulated_execution_time", 200.to_spvalue())
                .update("robot_emulate_execution_time", 1.to_spvalue())
                .update("robot_emulated_execution_time", 200.to_spvalue())
                .update("robot_emulate_mounted_tool", true.to_spvalue())
                .update("robot_emulated_mounted_tool", "gripper_tool".to_spvalue())
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


#[tokio::test]
#[serial_test::serial]
async fn test_replan() -> Result<(), Box<dyn Error>> {
    use regex::Regex;
    use testcontainers::{ImageExt, core::ContainerPort, runners::AsyncRunner};
    use testcontainers_modules::redis::Redis;

    let _container = Redis::default()
        .with_mapped_port(6379, ContainerPort::Tcp(6379))
        .start()
        .await
        .unwrap();

    let log_target = "micro_sp_emulation::test_replan";
    micro_sp::initialize_env_logger();
    let sp_id = "micro_sp".to_string();

    let coverability_tracking = false;

    let state = crate::model::state::state();

    let runner_vars = generate_runner_state_variables(&sp_id);
    let state = state.extend(runner_vars, true);

    let (model, state) = crate::model::replan::model(&sp_id, &state);

    let op_vars = generate_operation_state_variables(&model, coverability_tracking);
    let state = state.extend(op_vars, true);

    let connection_manager = ConnectionManager::new().await;
    StateManager::set_state(&mut connection_manager.get_connection().await, &state).await;
    let con_arc = std::sync::Arc::new(connection_manager);

    log::info!(target: &log_target, "Spawning emulators.");

    let con_clone = con_arc.clone();
    let robot_handle = tokio::task::spawn(async move {
        crate::emulators::robot::robot_emulator(&con_clone)
            .await
            .unwrap()
    });

    let con_clone = con_arc.clone();
    let gantry_handle = tokio::task::spawn(async move {
        crate::emulators::gantry::gantry_emulator(&con_clone)
            .await
            .unwrap()
    });

    log::info!(target: &log_target, "Spawning Micro SP.");
    let con_clone = con_arc.clone();
    let sp_id_clone = sp_id.clone();
    let sp_handle =
        tokio::task::spawn(async move { main_runner(&sp_id_clone, model, &con_clone).await });

    log::info!(target: &log_target, "Spawning test task.");
    let con_clone = con_arc.clone();
    let con_local = con_clone.get_connection().await;
    let sp_id_clone = sp_id.clone();
    let emulation_handle = tokio::task::spawn(async move {
        crate::model::replan::run_emultaion(&sp_id_clone, con_local)
            .await
            .unwrap()
    });

    log::info!(target: &log_target, "Test started. Polling for condition...");

    let max_wait = std::time::Duration::from_secs(30);
    let polling_logic = async {
        loop {
            let mut connection = con_arc.get_connection().await;
            match StateManager::get_full_state(&mut connection).await {
                Some(state) => match state.get_string_or_default_to_unknown(&format!("robot_mounted_estimated"), &log_target).as_str() {
                    "suction_tool" => {
                        // Wait before aborting the handles so that the operation can cycle through all states
                        tokio::time::sleep(std::time::Duration::from_secs(2)).await;
                        break;
                    }
                    _ => (),
                },
                None => log::error!(target: &log_target, "Failed to get full state."),
            }

            tokio::time::sleep(std::time::Duration::from_millis(
                crate::EMULATOR_TICK_INTERVAL,
            ))
            .await;
        }
    };

    if let Err(_) = tokio::time::timeout(max_wait, polling_logic).await {
        panic!("Test timed out after {:?} waiting for condition.", max_wait);
    }

    log::info!(target: &log_target, "Condition met. Cleaning up tasks.");

    robot_handle.abort();
    gantry_handle.abort();
    sp_handle.abort();
    emulation_handle.abort();

    log::info!(target: &log_target, "Fetching diagnostics trace for assertions.");
    let mut connection = con_arc.get_connection().await;
    match StateManager::get_sp_value(
        &mut connection,
        &format!("{}_diagnostics_operations", &sp_id),
    )
    .await
    {
        Some(diagnostics_sp_value) => {
            if let SPValue::String(StringOrUnknown::String(diagnostics_string)) =
                diagnostics_sp_value
            {
                if let Ok(diagnostics) =
                    serde_json::from_str::<Vec<Vec<OperationLog>>>(&diagnostics_string)
                {
                    let formatted = format_log_rows(&diagnostics);
                    println!("{}", formatted);

                    colored::control::set_override(false);
                    let result = format_log_rows(&diagnostics);

                    colored::control::unset_override();

                    let result_lines: Vec<&str> = result.trim().lines().collect();

                    let expected_patterns = vec![
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_robot_check_for_suction_tool_mounted\s*\|$",
                        r"^\| ----------------------------------------\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_gantry_calibrate\s*\|$",
                        r"^\| --------------------\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_gantry_lock\s*\|$",
                        r"^\| ---------------\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_robot_move_to_gripper_tool_rack\s*\|$",
                        r"^\| -----------------------------------\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_robot_unmount_gripper_tool\s*\|$",
                        r"^\| ------------------------------\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_robot_move_to_suction_tool_rack\s*\|$",
                        r"^\| -----------------------------------\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Current: op_robot_mount_suction_tool\s*\|$",
                        r"^\| ----------------------------\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                    ];

                    assert_eq!(
                        result_lines.len(),
                        expected_patterns.len(),
                        "Assertion failed: Wrong number of lines.\nActual Output:\n{}",
                        result
                    );

                    // Line-by-line regex match
                    for (i, (result_line, pattern_str)) in
                        result_lines.iter().zip(expected_patterns).enumerate()
                    {
                        let pattern = Regex::new(pattern_str).unwrap();

                        assert!(
                            pattern.is_match(result_line),
                            "Assertion failed: Line {} did not match.\n  Expected pattern: {}\n  Actual line:      {}",
                            i + 1,
                            pattern_str,
                            result_line
                        );
                    }
                } else {
                    assert!(false)
                }
            } else {
                assert!(false)
            }
        }
        None => assert!(false),
    }

    log::info!(target: &log_target, "Assertions passed. Test complete.");

    Ok(())
}
