use micro_sp::Model;
use r2r::micro_sp_emulation_msgs::srv::TriggerGripper;
use r2r::micro_sp_emulation_msgs::srv::TriggerScan;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "micro_sp_runner";
pub static TICKER_RATE: u64 = 100;
pub static PUBLISHER_RATE: u64 = 100;

mod models;
mod runner;
use models::scan_grip_rob_model::*;
use runner::gripper_client_ticker::*;
use runner::scanner_client_ticker::*;
use runner::ticker::*;

mod tests;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let ticker_timer = node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;

    // test
    let m = scan_grip_rob_model();
    let model = Model::new(&m.0, m.1, m.2, m.3, m.4);

    let shared_state = Arc::new(Mutex::new(model.state.clone()));

    let scanner_client = node.create_client::<TriggerScan::Service>("scanner_service")?;
    let gripper_client = node.create_client::<TriggerGripper::Service>("gripper_service")?;

    let waiting_for_scanner_server = node.is_available(&scanner_client)?;
    let waiting_for_gripper_server = node.is_available(&gripper_client)?;

    let shared_state_clone = shared_state.clone();
    let scanner_timer = node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;
    let gripper_timer = node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    tokio::task::spawn(async move {
        match scanner_client_ticker(
            &scanner_client,
            waiting_for_scanner_server,
            &shared_state_clone,
            scanner_timer,
            NODE_ID,
        )
        .await
        {
            Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
        };
    });

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        match gripper_client_ticker(
            &gripper_client,
            waiting_for_gripper_server,
            &shared_state_clone,
            gripper_timer,
            NODE_ID,
        )
        .await
        {
            Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
        };
    });

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        // wait for the measured values to update the state
        // tokio::time::sleep(std::time::Duration::from_millis(5000)).await;
        let result = ticker(
            NODE_ID,
            &model,
            &shared_state_clone,
            ticker_timer,
        )
        .await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Simple Controller Service call succeeded."),
            Err(e) => r2r::log_error!(
                NODE_ID,
                "Simple Controller Service call failed with: {}.",
                e
            ),
        };
    });

    handle.join().unwrap();

    r2r::log_warn!(NODE_ID, "Node started.");

    Ok(())
}