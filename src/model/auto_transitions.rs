use micro_sp::*;

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let mut auto_transitions = vec![];
    let sops = vec![];
    let operations = vec![];

    let counter = iv!(&&format!("counter"));
    let state = state.add(
        assign!(counter, SPValue::Int64(IntOrUnknown::Int64(0))),
        "emulator",
    );

    let lights_on = bv!(&&format!("lights_on"));
    let state = state.add(
        assign!(lights_on, SPValue::Bool(BoolOrUnknown::Bool(false))),
        "emulator",
    );

    auto_transitions.push(Transition::parse(
        "turn_lights_on",
        "var:counter < 3",
        "var:lights_on == false",
        Vec::from([]),
        vec!["var:lights_on <- true", "var:counter += 1"],
        &state,
    ));

    auto_transitions.push(Transition::parse(
        "turn_lights_off",
        "true",
        "var:lights_on == true",
        Vec::from([]),
        vec!["var:lights_on <- false"],
        &state,
    ));

    let model = Model::new(sp_id, auto_transitions, vec![], sops, operations);

    (model, state)
}

#[tokio::test]
#[serial_test::serial]
async fn test_auto_transitions() -> Result<(), Box<dyn std::error::Error>> {
    use regex::Regex;
    use testcontainers::{ImageExt, core::ContainerPort, runners::AsyncRunner};
    use testcontainers_modules::redis::Redis;

    let _container = Redis::default()
        .with_mapped_port(6379, ContainerPort::Tcp(6379))
        .start()
        .await
        .unwrap();

    let log_target = "micro_sp_emulation::test_auto_transitions";
    micro_sp::initialize_env_logger();
    let sp_id = "micro_sp".to_string();

    let coverability_tracking = false;

    let state = crate::model::state::state();

    let number_of_timers = 1;
    let runner_vars = generate_runner_state_variables(&sp_id, number_of_timers, "emulator");
    let state = state.extend(runner_vars, true);

    let (model, state) = crate::model::auto_transitions::model(&sp_id, &state);

    let op_vars = generate_operation_state_variables(&model, coverability_tracking, "emulator");
    let state = state.extend(op_vars, true);

    let connection_manager = ConnectionManager::new().await;
    StateManager::set_state(&mut connection_manager.get_connection().await, &state).await;
    let con_arc = std::sync::Arc::new(connection_manager);

    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

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

    // log::info!(target: &log_target, "Spawning test task.");
    // let con_clone = con_arc.clone();
    // let con_local = con_clone.get_connection().await;
    // let sp_id_clone = sp_id.clone();

    log::info!(target: &log_target, "Test started. Polling for condition...");

    let max_wait = std::time::Duration::from_secs(30);
    let polling_logic = async {
        loop {
            let mut connection = con_arc.get_connection().await;
            match StateManager::get_full_state(&mut connection).await {
                Some(state) => match state.get_int_or_unknown(&format!("counter"), &log_target) {
                    IntOrUnknown::Int64(3) => {
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

    log::info!(target: &log_target, "Fetching logger trace for assertions.");
    let mut connection = con_arc.get_connection().await;
    match StateManager::get_sp_value(
        &mut connection,
        &format!("{}_logger_automatic_transitions", &sp_id),
    )
    .await
    {
        Some(logger_sp_value) => {
            if let SPValue::String(StringOrUnknown::String(logger_string)) = logger_sp_value {
                if let Ok(logger) = serde_json::from_str::<Vec<TransitionMsg>>(&logger_string) {
                    let formatted = format_transition_log(&logger);
                    println!("{}", formatted);

                    colored::control::set_override(false);
                    let result = format_transition_log(&logger);

                    colored::control::unset_override();

                    let result_lines: Vec<&str> = result.trim().lines().collect();

                    let expected_patterns = vec![
                        r"^\+------------------------------------------------------\+$",
                        r"^\| Automatic transitions\s*\|$",
                        r"^\| ---------------------\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3}\] turn_lights_on_[\w]+: Executed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3}\] turn_lights_off_[\w]+: Executed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3}\] turn_lights_on_[\w]+: Executed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3}\] turn_lights_off_[\w]+: Executed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3}\] turn_lights_on_[\w]+: Executed\.\s*\|$",
                        r"^\| \[\d{2}:\d{2}:\d{2}\.\d{3}\] turn_lights_off_[\w]+: Executed\.\s*\|$",
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
