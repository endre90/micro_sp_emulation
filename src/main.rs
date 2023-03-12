use micro_sp::Model;
use r2r::micro_sp_emulation_msgs::srv::SetState;
use r2r::micro_sp_emulation_msgs::srv::TriggerGripper;
use r2r::micro_sp_emulation_msgs::srv::TriggerScan;
use r2r::std_msgs::msg::String;
use r2r::QosProfile;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "micro_sp_runner";
pub static TICKER_RATE: u64 = 100;
pub static PUBLISHER_RATE: u64 = 100;

mod models;
use models::scan_grip_rob_model::*;

mod runner;
use runner::gripper_client_ticker::*;
use runner::scanner_client_ticker::*;
use runner::set_state_server::*;
use runner::state_publisher::*;
use runner::ticker::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let ticker_timer = node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;

    let m = scan_grip_rob_model();
    let model = Model::new(&m.0, m.1, m.2, m.3, m.4);

    let shared_state = Arc::new(Mutex::new(model.state.clone()));

    let publisher_timer = node.create_wall_timer(std::time::Duration::from_millis(PUBLISHER_RATE))?;
    let state_publisher = node.create_publisher::<String>("state", QosProfile::default())?;

    let set_state_service = node.create_service::<SetState::Service>("set_state")?;

    let scanner_client = node.create_client::<TriggerScan::Service>("scanner_service")?;
    let gripper_client = node.create_client::<TriggerGripper::Service>("gripper_service")?;

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        let result = set_state_server(set_state_service, &shared_state_clone, NODE_ID).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Scanner service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Scanner service call failed with: '{}'.", e),
        };
    });

    let waiting_for_scanner_server = node.is_available(&scanner_client)?;
    let waiting_for_gripper_server = node.is_available(&gripper_client)?;

    let scanner_timer = node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;
    let gripper_timer = node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    let shared_state_clone = shared_state.clone();
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
        let result = ticker(NODE_ID, &model, &shared_state_clone, ticker_timer).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Simple Controller Service call succeeded."),
            Err(e) => r2r::log_error!(
                NODE_ID,
                "Simple Controller Service call failed with: {}.",
                e
            ),
        };
    });

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        let result = state_publisher_callback(state_publisher, publisher_timer, &shared_state_clone, NODE_ID).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "State publisher succeeded."),
            Err(e) => r2r::log_error!(
                NODE_ID,
                "State publisher failed with: {}.",
                e
            ),
        };
    });

    handle.join().unwrap();

    r2r::log_warn!(NODE_ID, "Node started.");

    Ok(())
}
