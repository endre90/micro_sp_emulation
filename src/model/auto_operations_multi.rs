use micro_sp::*;

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let auto_transitions = vec![];
    let sops = vec![];
    let operations = vec![];
    let mut auto_operations = vec![];

    let counter = iv!(&&format!("counter"));
    let state = state.add(
        assign!(counter, SPValue::Int64(IntOrUnknown::Int64(0))),
        "emulator",
    );

    for timer_id in vec![("1", 900), ("2", 1500), ("3", 2200)] {
        auto_operations.push(Operation::new(
            &format!("emulate_auto_timer_{}", timer_id.0),
            None,
            None,
            None,
            None,
            true,
            Vec::from([Transition::parse(
                &format!("start_sleep_{}", timer_id.0),
                &format!(
                    "var:counter < 3 && var:micro_sp_timer_{}_request_state == initial \
                    && var:micro_sp_timer_{}_request_trigger == false",
                    timer_id.0, timer_id.0
                ),
                "true",
                vec![
                    &format!("var:micro_sp_timer_{}_request_trigger <- true", timer_id.0),
                    &format!(
                        "var:micro_sp_timer_{}_duration_ms <- {}",
                        timer_id.0, timer_id.1
                    ),
                    &format!("var:micro_sp_timer_{}_command <- sleep", timer_id.0),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_sleep_{}", timer_id.0),
                "true",
                &format!(
                    "var:micro_sp_timer_{}_request_state == succeeded",
                    timer_id.0
                ),
                vec![
                    &format!("var:micro_sp_timer_{}_request_trigger <- false", timer_id.0),
                    &format!("var:micro_sp_timer_{}_request_state <- initial", timer_id.0),
                    "var:counter += 1",
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

    let model = Model::new(sp_id, auto_transitions, auto_operations, sops, operations);

    (model, state)
}

#[tokio::test]
#[serial_test::serial]

async fn test_auto_operations() -> Result<(), Box<dyn std::error::Error>> {
    use regex::Regex;
    use testcontainers::{ImageExt, core::ContainerPort, runners::AsyncRunner};
    use testcontainers_modules::redis::Redis;

    let _container = Redis::default()
        .with_mapped_port(6379, ContainerPort::Tcp(6379))
        .start()
        .await
        .unwrap();

    let log_target = "micro_sp_emulation::test_auto_operations";
    micro_sp::initialize_env_logger();
    let sp_id = "micro_sp".to_string();

    let coverability_tracking = false;

    let state = crate::model::state::state();
    let state = state
        .update(
            "robot_emulate_execution_time",
            crate::EMULATE_EXACT_EXECUTION_TIME.to_spvalue(),
        )
        .update("robot_emulated_execution_time", 500.to_spvalue())
        .update("robot_position_estimated", "b".to_spvalue())
        .update(
            "robot_emulate_failure_rate",
            crate::DONT_EMULATE_FAILURE.to_spvalue(),
        );

    let number_of_timers = 3;
    let runner_vars = generate_runner_state_variables(&sp_id, number_of_timers, "emulator");
    let state = state.extend(runner_vars, true);

    let (model, state) = crate::model::auto_operations_multi::model(&sp_id, &state);

    let op_vars = generate_operation_state_variables(&model, coverability_tracking, "emulator");
    let state = state.extend(op_vars, true);

    // println!("{}", state);

    let connection_manager = ConnectionManager::new().await;
    StateManager::set_state(&mut connection_manager.get_connection().await, &state).await;
    let con_arc = std::sync::Arc::new(connection_manager);

    // log::info!(target: &log_target, "Spawning emulators.");

    // let con_clone = con_arc.clone();
    // let robot_handle = tokio::task::spawn(async move {
    //     crate::emulators::robot::robot_emulator(&con_clone)
    //         .await
    //         .unwrap()
    // });

    // let con_clone = con_arc.clone();
    // let gantry_handle = tokio::task::spawn(async move {
    //     crate::emulators::gantry::gantry_emulator(&con_clone)
    //         .await
    //         .unwrap()
    // });

    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    log::info!(target: &log_target, "Spawning Micro SP.");
    let con_clone = con_arc.clone();
    let sp_id_clone = sp_id.clone();
    let sp_handle = tokio::task::spawn(async move {
        main_runner(&sp_id_clone, model, number_of_timers, &con_clone).await
    });

    log::info!(target: &log_target, "Test started. Polling for condition...");

    let max_wait = std::time::Duration::from_secs(15);
    let polling_logic = async {
        loop {
            let mut connection = con_arc.get_connection().await;
            match StateManager::get_full_state(&mut connection).await {
                Some(state) => match state.get_int_or_unknown(&format!("counter"), &log_target) {
                    IntOrUnknown::Int64(x) => {
                        if x > 3 {
                            tokio::time::sleep(std::time::Duration::from_secs(2)).await;
                            // let state =
                            //     StateManager::get_full_state(&mut connection).await.unwrap();
                            // println!("DONE STATE SUCC: {}", state);
                            break;
                        }
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

    // robot_handle.abort();
    // gantry_handle.abort();
    sp_handle.abort();
    // emulation_handle.abort();

    log::info!(target: &log_target, "Fetching logger trace for assertions.");
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
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -4: op_emulate_au\.\.\.r_1_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -3: op_emulate_au\.\.\.r_2_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -2: op_emulate_au\.\.\.r_3_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Done -1: op_emulate_au\.\.\.r_1_[\w]+\s*\|$",
                        r"^\| -+\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Initial\s+\] Starting\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Executing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Executing\s+\] Completing\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3} \| Completed\s+\] Completed\s*\|$",
                        r"^\+--------------------------------------------\+$",
                        r"^\+--------------------------------------------\+$",
                        r"^\| Latest: op_emulate_au\.\.\.r_2_[\w]+\s*\|$",
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
