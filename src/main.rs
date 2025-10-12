use redis::aio::MultiplexedConnection;
use std::error::Error;
use std::sync::Arc;
use std::time::Duration;

use micro_sp::*;
use micro_sp_emulation::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let sp_id = "micro_sp".to_string();

    // Enable coverability tracking:
    let coverability_tracking = false;

    let state = model::state::state();

    // Add the variables that keep track of the runner state
    let runner_vars = generate_runner_state_variables(&sp_id);
    let state = state.extend(runner_vars, true);

    let (model, state) = model::minimal::minimal_model(&sp_id, &state);

    let op_vars = generate_operation_state_variables(&model, coverability_tracking);
    let state = state.extend(op_vars, true);

    let connection_manager = ConnectionManager::new().await;
    StateManager::set_state(&mut connection_manager.get_connection().await, &state).await;
    let con_arc = Arc::new(connection_manager);

    log::info!(target: "micro_sp_emulator", "Spawning emulators.");

    let con_clone = con_arc.clone();
    tokio::task::spawn(async move { robot_emulator(&con_clone).await.unwrap() });

    let con_clone = con_arc.clone();
    tokio::task::spawn(async move { gantry_emulator(&con_clone).await.unwrap() });

    log::info!(target: "micro_sp_emulator", "Spawning Micro SP.");

    let con_clone = con_arc.clone();
    let sp_id_clone = sp_id.clone();
    tokio::task::spawn(async move { main_runner(&sp_id_clone, model, &con_clone).await });

    log::info!(target: "micro_sp_emulator", "Spawning test task.");

    let con_clone = con_arc.clone();
    let con_local = con_clone.get_connection().await;
    let sp_id_clone = sp_id.clone();
    tokio::task::spawn(async move { perform_test(&sp_id_clone, con_local).await.unwrap() });

    log::info!(target: "micro_sp_emulator", "Node started.");

    loop {
        tokio::time::sleep(Duration::from_millis(EMULATOR_TICK_INTERVAL)).await;
    }
}

async fn perform_test(sp_id: &str, mut con: MultiplexedConnection) -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    // log::info!(target: NODE_ID, "Setting goal: var:robot_mounted_estimated == suction_tool");
    // let goal = "var:robot_mounted_estimated == suction_tool";
    let goal = "var:blinked == true";
    // && var:blinked == true

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

    // r2r::log_warn!(NODE_ID, "All tests are finished. Generating report...");

    // TODO
    // Measure operation and plan execution times, and measure total failure rates...
    // Print out plan done or plan failed when done or failed...

    Ok(())
}
