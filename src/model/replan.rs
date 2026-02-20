use micro_sp::{running::goal_runner::goal_string_to_sp_value, *};
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
                        &format!("var:{sp_id}_replan_for_same_goal <- true"),
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
    let goal = "var:robot_mounted_estimated == suction_tool".to_string();
    let uq_goal = goal_string_to_sp_value(&goal, running::goal_runner::GoalPriority::Normal);
    let scheduled_goals = vec![uq_goal].to_spvalue();

    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    if let Some(state) = StateManager::get_full_state(&mut con).await {
        let new_state = state
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
            .update("gantry_locked_estimated", true.to_spvalue())
            .update(&format!("{sp_id}_scheduled_goals"), scheduled_goals);

        let modified_state = state.get_diff_partial_state(&new_state);
        StateManager::set_state(&mut con, &modified_state).await;
    }

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
    let number_of_timers = 1;

    let runner_vars = generate_runner_state_variables(&sp_id, number_of_timers, "emulator");
    let state = state.extend(runner_vars, true);

    let (model, state) = crate::model::replan::model(&sp_id, &state);

    let op_vars = generate_operation_state_variables(&model, coverability_tracking, "emulator");
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
        tokio::task::spawn(async move { main_runner(&sp_id_clone, model, number_of_timers, &con_clone).await });

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

    log::info!(target: &log_target, "Fetching logger trace for assertions.");
    let mut connection = con_arc.get_connection().await;
    match StateManager::get_sp_value(
        &mut connection,
        &format!("{}_logger_planned_operations", &sp_id),
    )
    .await
    {
        Some(logger_sp_value) => {
            if let SPValue::String(StringOrUnknown::String(logger_string)) =
                logger_sp_value
            {
                if let Ok(logger) =
                    serde_json::from_str::<Vec<Vec<OperationLog>>>(&logger_string)
                {
                    let formatted = format_log_rows(&logger);
                    println!("{}", formatted);

                    colored::control::set_override(false);
                    let result = format_log_rows(&logger);

                    colored::control::unset_override();

                    let result_lines: Vec<&str> = result.trim().lines().collect();

                    let expected_patterns = vec![
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -7: op_robot_chec\.\.\.ted_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -6: op_gantry_unlock_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -5: op_gantry_calibrate_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -4: op_gantry_lock_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -3: op_robot_move\.\.\.ack_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -2: op_robot_unmo\.\.\.ool_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -1: op_robot_move\.\.\.ack_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Latest: op_robot_moun\.\.\.ool_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
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
