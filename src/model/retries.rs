use micro_sp::*;
use redis::aio::MultiplexedConnection;
use std::error::Error;

use crate::{DONT_EMULATE_FAILURE, DONT_EMULATE_FAILURE_CAUSE, EMULATE_EXACT_EXECUTION_TIME};

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let auto_transitions = vec![];
    let sops = vec![];
    let mut operations = vec![];

    let bypassed = bv!(&&format!("bypassed"));
    let state = state.add(assign!(bypassed, SPValue::Bool(BoolOrUnknown::Bool(false))));

    operations.push(Operation::new(
        "gantry_unlock",
        Some(500),
        None, 
        Some(5), // If there is to be a failure retry, we need a failure transition
        Some(5), // If there is to be a timeout retry, we need a timeout transition
        true, // We can add bypass transitions, but usually not necessary
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
            "fail_gantry_unlock",
            "true",
            "var:gantry_request_state == failed",
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "timeout_gantry_unlock",
            "true",
            "true",
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "bypass_gantry_unlock",
            "true",
            "true",
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:bypassed <- true"
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
        None,
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
        Vec::from([]),
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
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    let goal = "var:gantry_calibrated_estimated == true";

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
                .update(
                    "gantry_emulate_execution_time",
                    EMULATE_EXACT_EXECUTION_TIME.to_spvalue(),
                )
                .update("gantry_emulated_execution_time", 10000.to_spvalue())
                .update(
                    "gantry_emulate_failure_rate",
                    DONT_EMULATE_FAILURE.to_spvalue(),
                )
                .update("gantry_emulated_failure_rate", 0.to_spvalue())
                .update(
                    "gantry_emulate_failure_cause",
                    DONT_EMULATE_FAILURE_CAUSE.to_spvalue(),
                )
                .update("gantry_calibrated_estimated", false.to_spvalue())
                .update("gantry_locked_estimated", true.to_spvalue())
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
