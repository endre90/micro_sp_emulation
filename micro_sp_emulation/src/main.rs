use std::error::Error;
use std::sync::{Arc, Mutex};
use tokio::sync::{mpsc, oneshot};

use r2r::micro_sp_emulation_msgs::srv::{TriggerGantry, TriggerRobot};
use r2r::QosProfile;

use micro_sp::*;
use micro_sp_emulation::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // Generates unique IDs
    // let alphabet: [char; 8] = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'];
    // let sp_id = nanoid::nanoid!(6, &alphabet);

    let sp_id = "micro_sp".to_string();

    // Enable coverability tracking:
    let coverability_tracking = false;

    // Setup the node
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, NODE_ID, "")?;
    let arc_node = Arc::new(Mutex::new(node));

    let state = models::minimal::state::state();

    // Add the variables that keep track of the runner state
    let runner_vars = generate_runner_state_variables(&sp_id);
    let state = state.extend(runner_vars, true);

    let (model, state) = models::minimal::model::minimal_model(&sp_id, &state);

    let op_vars = generate_operation_state_variables(&model, coverability_tracking);
    let state = state.extend(op_vars, true);



    let (tx, rx) = mpsc::channel(100); // Experiment with buffer size
    log::info!(target: "micro_sp", "Spawning state manager.");
    tokio::task::spawn(async move { redis_state_manager(rx, state).await.unwrap() });

    // tokio::time::sleep(std::time::Duration::from_millis(2000)).await;

    log::info!(target: NODE_ID, "Spawning emulators.");

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    tokio::task::spawn(async move { spawn_gantry_emulator_server(arc_node_clone).await.unwrap() });

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    tokio::task::spawn(async move { spawn_robot_emulator_server(arc_node_clone).await.unwrap() });

    log::info!(target: NODE_ID, "Spawning interfaces.");

    let gantry_client = arc_node
        .lock()
        .unwrap()
        .create_client::<TriggerGantry::Service>(
            "/gantry_emulator_service",
            QosProfile::default(),
        )?;
    let gantry_timer = arc_node
        .lock()
        .unwrap()
        .create_wall_timer(std::time::Duration::from_millis(CLIENT_TICKER_RATE))?;
    let robot_client = arc_node
        .lock()
        .unwrap()
        .create_client::<TriggerRobot::Service>("/robot_emulator_service", QosProfile::default())?;
    let robot_timer = arc_node
        .lock()
        .unwrap()
        .create_wall_timer(std::time::Duration::from_millis(CLIENT_TICKER_RATE))?;
    let waiting_for_gantry = r2r::Node::is_available(&gantry_client)?;
    let waiting_for_robot = r2r::Node::is_available(&robot_client)?;

    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let handle = std::thread::spawn(move || loop {
        arc_node_clone
            .lock()
            .unwrap()
            .spin_once(std::time::Duration::from_millis(1000));
    });

    log::warn!(target: "gantry_interface", "Waiting for the Gantry resource to be online.");
    waiting_for_gantry.await?;
    log::warn!(target: "robot_interface", "Waiting for the Robot resource to be online.");
    waiting_for_robot.await?;
    log::info!(target: "gantry_interface", "Gantry reource online.");
    log::info!(target: "robot_interface", "Robot reource online.");

    let tx_clone = tx.clone();
    tokio::task::spawn(async move {
        gantry_client_ticker(&gantry_client, gantry_timer, tx_clone)
            .await
            .unwrap()
    });

    // let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let tx_clone = tx.clone();
    tokio::task::spawn(async move {
        robot_client_ticker(&robot_client, robot_timer, tx_clone)
            .await
            .unwrap()
    });

    log::info!(target: NODE_ID, "Spawning Micro SP.");

    let tx_clone = tx.clone();
    let sp_id_clone = sp_id.clone();
    tokio::task::spawn(async move { main_runner(&sp_id_clone, model, tx_clone).await });

    log::info!(target: NODE_ID, "Spawning test task.");

    let tx_clone = tx.clone();
    let sp_id_clone = sp_id.clone();
    tokio::task::spawn(async move { perform_test(&sp_id_clone, tx_clone).await.unwrap() });

    log::info!(target: NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

async fn perform_test(
    sp_id: &str,
    command_sender: mpsc::Sender<StateManagement>,
) -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    // log::info!(target: NODE_ID, "Setting goal: var:robot_mounted_estimated == suction_tool");
    // let goal = "var:robot_mounted_estimated == suction_tool";
    let goal = "var:blinked == true";
    // && var:blinked == true

    let (response_tx, response_rx) = oneshot::channel();
    command_sender
        .send(StateManagement::GetState(response_tx))
        .await?; // TODO: maybe we can just ask for values from the guard
    let state = response_rx.await?;

    let plan_state = state.get_string_or_default_to_unknown(
        &format!("{}_tester", sp_id),
        &format!("{}_plan_state", sp_id),
    );

    if PlanState::from_str(&plan_state) == PlanState::Failed
        || PlanState::from_str(&plan_state) == PlanState::Completed
        || PlanState::from_str(&plan_state) == PlanState::Initial
        || PlanState::from_str(&plan_state) == PlanState::UNKNOWN
    {
        let new_state = state
            // Optional to test what happens when... (look in the Emulation msg for details)
            .update("gantry_emulate_execution_time", 2.to_spvalue())
            .update("gantry_emulated_execution_time", 10.to_spvalue())
            .update("gantry_emulate_failure_rate", 2.to_spvalue())
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
        command_sender
            .send(StateManagement::SetPartialState(modified_state))
            .await?;
    }

    // r2r::log_warn!(NODE_ID, "All tests are finished. Generating report...");

    // TODO
    // Measure operation and plan execution times, and measure total failure rates...
    // Print out plan done or plan failed when done or failed...

    Ok(())
}
