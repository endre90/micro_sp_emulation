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

    let (model, state) = model::nominal::minimal_model(&sp_id, &state);

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
    tokio::task::spawn(async move { model::nominal::run_emultaion(&sp_id_clone, con_local).await.unwrap() });

    log::info!(target: "micro_sp_emulator", "Node started.");

    loop {
        tokio::time::sleep(Duration::from_millis(EMULATOR_TICK_INTERVAL)).await;
    }
}

