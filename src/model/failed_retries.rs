use micro_sp::{running::goal_runner::goal_string_to_sp_value, *};
use redis::aio::MultiplexedConnection;
use std::error::Error;

use crate::{EMULATE_EXACT_EXECUTION_TIME, EMULATE_EXACT_FAILURE_CAUSE, EMULATE_FAILURE_ALWAYS};

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let auto_transitions = vec![];
    let sops = vec![];
    let mut operations = vec![];

    let failed = bv!(&&format!("failed"));
    let state = state.add(assign!(failed, SPValue::Bool(BoolOrUnknown::Bool(false))));

    let bypassed = bv!(&&format!("bypassed"));
    let state = state.add(assign!(bypassed, SPValue::Bool(BoolOrUnknown::Bool(false))));

    operations.push(Operation::new(
        "gantry_unlock",
        None,
        None,
        Some(2), // If there is to be a failure retry, we need a failure transition
        None,    // If there is to be a timeout retry, we need a timeout transition
        true,    // We can add bypass transitions, but usually not necessary to bypass
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
        Vec::from([Transition::parse(
            "failed_gantry_unlock",
            "true",
            "var:gantry_request_state == failed",
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([Transition::parse(
            "bypass_gantry_unlock",
            "true",
            "true",
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:bypassed <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
    ));

    operations.push(Operation::new(
        "gantry_calibrate",
        None,
        None,
        Some(2),
        None,
        false,
        Vec::from([Transition::parse(
            "start_gantry_calibrate",
            "(var:gantry_locked_estimated == false || var:bypassed == true) \
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
        Vec::from([Transition::parse(
            "failed_gantry_unlock",
            "true",
            "var:gantry_request_state == failed",
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:failed <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    ));

    let model = Model::new(sp_id, auto_transitions, vec![], sops, operations);

    (model, state)
}

pub async fn run_emultaion(
    sp_id: &str,
    mut con: MultiplexedConnection,
) -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    let goal = "var:gantry_calibrated_estimated == true".to_string();
    let uq_goal = goal_string_to_sp_value(&goal, running::goal_runner::GoalPriority::Normal);
    let scheduled_goals = vec![uq_goal].to_spvalue();

    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    if let Some(state) = StateManager::get_full_state(&mut con).await {
        let new_state = state
            .update(
                "gantry_emulate_execution_time",
                EMULATE_EXACT_EXECUTION_TIME.to_spvalue(),
            )
            .update("gantry_emulated_execution_time", 300.to_spvalue())
            .update(
                "gantry_emulate_failure_rate",
                EMULATE_FAILURE_ALWAYS.to_spvalue(),
            )
            .update(
                "gantry_emulate_failure_cause",
                EMULATE_EXACT_FAILURE_CAUSE.to_spvalue(),
            )
            .update(
                "gantry_emulated_failure_cause",
                vec!["collision"].to_spvalue(),
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
async fn test_failed_retries() -> Result<(), Box<dyn Error>> {
    use regex::Regex;
    use testcontainers::{ImageExt, core::ContainerPort, runners::AsyncRunner};
    use testcontainers_modules::redis::Redis;

    let _container = Redis::default()
        .with_mapped_port(6379, ContainerPort::Tcp(6379))
        .start()
        .await
        .unwrap();

    let log_target = "micro_sp_emulation::test_failed_retries";
    micro_sp::initialize_env_logger();
    let sp_id = "micro_sp".to_string();

    let coverability_tracking = false;

    let state = crate::model::state::state();

    let runner_vars = generate_runner_state_variables(&sp_id);
    let state = state.extend(runner_vars, true);

    let (model, state) = crate::model::failed_retries::model(&sp_id, &state);

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
        crate::model::failed_retries::run_emultaion(&sp_id_clone, con_local)
            .await
            .unwrap()
    });

    log::info!(target: &log_target, "Test started. Polling for condition...");

    let max_wait = std::time::Duration::from_secs(30);
    let polling_logic = async {
        loop {
            let mut connection = con_arc.get_connection().await;
            match StateManager::get_full_state(&mut connection).await {
                Some(state) => match state.get_bool_or_unknown(&format!("failed"), &log_target) {
                    BoolOrUnknown::Bool(true) => {
                        // Wait before aborting the handles so that the operation can cycle through all states
                        tokio::time::sleep(std::time::Duration::from_secs(4)).await;
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
            if let SPValue::String(StringOrUnknown::String(logger_string)) = logger_sp_value {
                if let Ok(logger) = serde_json::from_str::<Vec<Vec<OperationLog>>>(&logger_string) {
                    let formatted = format_log_rows(&logger);
                    println!("{}", formatted);

                    colored::control::set_override(false);
                    let result = format_log_rows(&logger);

                    colored::control::unset_override();

                    let result_lines: Vec<&str> = result.trim().lines().collect();

                    let expected_patterns = vec![
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -1: op_gantry_unlock_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Failing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Failed\s+\] Retrying \d/\d\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Failing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Failed\s+\] Retrying \d/\d\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Failing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Failed\s+\] Bypassing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Bypassed\s+\] Bypassed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Latest: op_gantry_calibrate_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Failing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Failed\s+\] Retrying \d/\d\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Failing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Failed\s+\] Retrying \d/\d\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Failing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Failed\s+\] Fatal failure\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Fatal\s+\] Unrecoverable\s*\|$",
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
