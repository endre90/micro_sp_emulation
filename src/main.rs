use std::error::Error;
use std::sync::Arc;
use std::time::Duration;

use micro_sp::*;
use micro_sp_emulation::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    micro_sp::initialize_env_logger();
    let sp_id = "sp1".to_string();

    // Enable coverability tracking:
    let coverability_tracking = false;

    let state = model::state::state();

    // // --- FIX: APPLY INITIAL CONFIGURATION HERE ---
    // // Apply the settings directly to the initial state before anything spawns
    // let state = state
    //     .update("gantry_emulate_execution_time", EMULATE_EXACT_EXECUTION_TIME.to_spvalue())
    //     .update("gantry_emulated_execution_time", 3000.to_spvalue())
    //     // .update("done", false.to_spvalue())
    //     .update("gantry_emulate_failure_rate", DONT_EMULATE_FAILURE.to_spvalue())
    //     .update("robot_emulate_execution_time", EMULATE_EXACT_EXECUTION_TIME.to_spvalue())
    //     .update("robot_emulated_execution_time", 6000.to_spvalue())
    //     .update("robot_emulate_failure_rate", DONT_EMULATE_FAILURE.to_spvalue());


    // Add the variables that keep track of the runner state
    let runner_vars = generate_runner_state_variables(&sp_id);
    let state = state.extend(runner_vars, true);

    let (model, state) = model::sop_parallel::model(&sp_id, &state);

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

    // let con_clone = con_arc.clone();
    // let con_local = con_clone.get_connection().await;
    // let sp_id_clone = sp_id.clone();
    // tokio::task::spawn(async move { model::emergency::run_emultaion(&sp_id_clone, con_local).await.unwrap() });

    let con_clone = con_arc.clone(); // <-- Pass the Arc
    // let sp_id_clone = sp_id.clone();
    tokio::task::spawn(async move {
        // Get a fresh connection *inside* the new task
        let con_local = con_clone.get_connection().await;

        // Now run the logic
        let result = crate::model::sop_parallel::run_emultaion(con_local).await;
        if let Err(e) = result {
            // Log any errors instead of just unwrapping
            log::error!("run_emultaion task failed: {:?}", e);
        }
    });

    log::info!(target: "micro_sp_emulator", "Node started.");

    loop {
        tokio::time::sleep(Duration::from_millis(EMULATOR_TICK_INTERVAL)).await;
    }
}
