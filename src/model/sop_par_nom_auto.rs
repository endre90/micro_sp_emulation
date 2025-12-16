use micro_sp::*;
use redis::aio::MultiplexedConnection;
use std::error::Error;

use crate::{DONT_EMULATE_FAILURE, EMULATE_EXACT_EXECUTION_TIME};

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let auto_transitions = vec![];
    let mut sops = vec![];
    let operations = vec![];
    let mut auto_operations = vec![];

    let done = bv!(&&format!("done"));
    let state = state.add(assign!(done, SPValue::Bool(BoolOrUnknown::Bool(false))));

    let auto_opertion_move_ababa = sop_operation_move_robot_and_gantry(sp_id, &state);
    auto_operations.push(auto_opertion_move_ababa);

    let sop_struct = SOPStruct {
        id: format!("sop_move_robot_and_gantry").to_string(),
        sop: SOP::Parallel(vec![
            robot_move_to_pos("a", &state),
            gantry_move_to_pos("x", &state),
        ]),
    };

    sops.push(sop_struct);

    let model = Model::new(sp_id, auto_transitions, auto_operations, sops, operations);

    (model, state)
}

fn sop_operation_move_robot_and_gantry(sp_id: &str, state: &State) -> Operation {
    Operation::new(
        &format!("sop_move_robot_and_gantry"),
        None,
        None,
        None,
        None,
        false,
        Vec::from([Transition::parse(
            &format!("start_sop_move_robot_and_gantry"),
            &format!("var:done == false"),
            "true",
            vec![
                &format!("var:{sp_id}_sop_enabled <- true"),
                &format!("var:{sp_id}_sop_state <- initial"),
                &format!("var:{sp_id}_sop_id <- sop_move_robot_and_gantry"),
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            &format!("complete_sop_move_robot_and_gantry"),
            "true",
            &format!("var:{sp_id}_sop_state == completed"),
            vec!["var:done <- true"],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    )
}

fn robot_move_to_pos(pos: &str, state: &State) -> SOP {
    SOP::Operation(Box::new(Operation::new(
        &format!("robot_move_to_{}", pos),
        None,
        None,
        None,
        None,
        false,
        Vec::from([Transition::parse(
            &format!("start_robot_move_to_{pos}"),
            &format!(
                "var:robot_request_state == initial \
                && var:robot_request_trigger == false"
            ),
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
            &format!("complete_robot_move_to_{pos}"),
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
    )))
}

fn gantry_move_to_pos(pos: &str, state: &State) -> SOP {
    SOP::Operation(Box::new(Operation::new(
        &format!("gantry_move_to_{}", pos),
        None,
        None,
        None,
        None,
        false,
        Vec::from([Transition::parse(
            &format!("start_gantry_move_to_{pos}"),
            &format!(
                "var:gantry_request_state == initial \
                && var:gantry_request_trigger == false"
            ),
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
            &format!("complete_gantry_move_to_{pos}"),
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
    )))
}

pub async fn run_emultaion(
    // sp_id: &str,
    mut con: MultiplexedConnection,
) -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    if let Some(state) = StateManager::get_full_state(&mut con).await {
        let new_state = state
            // Optional to test what happens when... (look in the Emulation msg for details)
            .update(
                "gantry_emulate_execution_time",
                EMULATE_EXACT_EXECUTION_TIME.to_spvalue(),
            )
            .update("gantry_emulated_execution_time", 0.to_spvalue())
            .update(
                "gantry_emulate_failure_rate",
                DONT_EMULATE_FAILURE.to_spvalue(),
            )
            .update(
                "robot_emulate_execution_time",
                EMULATE_EXACT_EXECUTION_TIME.to_spvalue(),
            )
            .update("robot_emulated_execution_time", 500.to_spvalue())
            .update(
                "robot_emulate_failure_rate",
                DONT_EMULATE_FAILURE.to_spvalue(),
            );

        let modified_state = state.get_diff_partial_state(&new_state);
        StateManager::set_state(&mut con, &modified_state).await;
    }

    Ok(())
}

#[tokio::test]
#[serial_test::serial]
async fn test_sop_par_nom_auto() -> Result<(), Box<dyn Error>> {
    use regex::Regex;
    use testcontainers::{ImageExt, core::ContainerPort, runners::AsyncRunner};
    use testcontainers_modules::redis::Redis;

    let _container = Redis::default()
        .with_mapped_port(6379, ContainerPort::Tcp(6379))
        .start()
        .await
        .unwrap();

    let log_target = "micro_sp_emulation::test_sop_par_nom_auto";
    micro_sp::initialize_env_logger();
    let sp_id = "micro_sp".to_string();

    let coverability_tracking = false;

    let state = crate::model::state::state();

    let runner_vars = generate_runner_state_variables(&sp_id);
    let state = state.extend(runner_vars, true);

    let (model, state) = crate::model::sop_par_nom_auto::model(&sp_id, &state);

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
    let sp_handle = tokio::task::spawn(async move {
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
        main_runner(&sp_id_clone, model, &con_clone).await
    });

    log::info!(target: &log_target, "Spawning test task.");
    let con_clone = con_arc.clone();
    let con_local = con_clone.get_connection().await;
    // let sp_id_clone = sp_id.clone();
    let emulation_handle = tokio::task::spawn(async move {
        crate::model::sop_par_nom_auto::run_emultaion(con_local)
            .await
            .unwrap()
    });

    log::info!(target: &log_target, "Test started. Polling for condition...");

    let max_wait = std::time::Duration::from_secs(15);
    let polling_logic = async {
        loop {
            let mut connection = con_arc.get_connection().await;
            match StateManager::get_full_state(&mut connection).await {
                Some(state) => match state.get_bool_or_unknown(&format!("done"), &log_target) {
                    BoolOrUnknown::Bool(true) => {
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

    log::info!(target: &log_target, "Fetching SOP logger trace aggregate.");
    let mut connection = con_arc.get_connection().await;
    match StateManager::get_sp_value(
        &mut connection,
        &format!("{}_logger_sop_operations_agg", &sp_id),
    )
    .await
    {
        Some(logger_sp_value) => {
            if let SPValue::String(StringOrUnknown::String(logger_string)) = logger_sp_value {
                println!("adsf: {}", logger_string);
                if let Ok(logger) =
                    serde_json::from_str::<Vec<Vec<Vec<OperationLog>>>>(&logger_string)
                {
                    for row in logger {
                        let formatted = format_log_rows(&row);
                        println!("{}", formatted);

                        colored::control::set_override(false);
                        // let result = format_log_rows(&row);

                        colored::control::unset_override();
                    }
                } else {
                    println!("fail1");
                    assert!(false)
                }
            } else {
                println!("fail2");
                assert!(false)
            }
        }
        None => assert!(false),
    }

    log::info!(target: &log_target, "Fetching SOP logger trace for assertions.");
    let mut connection = con_arc.get_connection().await;
    match StateManager::get_sp_value(
        &mut connection,
        &format!("{}_logger_sop_operations", &sp_id),
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
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_robot_move_to_a_\w+\s*\|$",
                        r"^\| -{30}\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_robot_move_to_b_\w+\s*\|$",
                        r"^\| -{30}\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_robot_move_to_a_\w+\s*\|$",
                        r"^\| -{30}\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Past -\d: op_robot_move_to_b_\w+\s*\|$",
                        r"^\| -{30}\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Waiting to be completed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing operation\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Operation completed\.\s*\|$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Current: op_robot_move_to_a_\w+\s*\|$",
                        r"^\| -{30}\s*\|$",
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

    log::info!(target: &log_target, "Fetching auto op logger trace for assertions.");
    let mut connection = con_arc.get_connection().await;
    match StateManager::get_sp_value(
        &mut connection,
        &format!("{}_logger_automatic_operations", &sp_id),
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
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Current: op_sop_robot_move_ababa\s*\|$",
                        r"^\| ------------------------\s*\|$",
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
