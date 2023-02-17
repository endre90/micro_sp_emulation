use micro_sp::bfs_operation_planner;
use r2r::QosProfile;
use r2r::micro_sp_emulation_msgs::msg::GantryIncoming;
use r2r::micro_sp_emulation_msgs::msg::GantryOutgoing;
use r2r::micro_sp_emulation_msgs::msg::GripperIncoming;
use r2r::micro_sp_emulation_msgs::msg::GripperOutgoing;
use r2r::micro_sp_emulation_msgs::srv::TriggerScan;
use r2r::micro_sp_emulation_msgs::action::URCommand;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "micro_sp_runner";
pub static TICKER_RATE: u64 = 100;
pub static PUBLISHER_RATE: u64 = 100;

mod runner;
// use runner::dummy_pub_sub_model::*;
use runner::gantry_pub_sub_ticker::*;
use runner::gripper_pub_sub_ticker::*;
use runner::robot_action_ticker::*;
use runner::scanner_client_ticker::*;
use runner::ticker::*;
use runner::rita_model::*;


#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    // let ur_action_client = node.create_action_client::<URControl::Action>("ur_control")?;
    // let waiting_for_ur_action_client = node.is_available(&ur_action_client)?;

    let ticker_timer =
        node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;

    let gantry_subscriber = node.subscribe::<GantryIncoming>(
        "gantry_incoming",
        QosProfile::default().reliable()
    )?;

    let gripper_subscriber = node.subscribe::<GripperIncoming>(
        "gripper_incoming",
        QosProfile::default().reliable()
    )?;

    // test
    let model = rita_model();
    // let plan = bfs_operation_planner(model.state.clone(), extract_goal_from_state(&model.state.clone()), model.operations.clone(), 50);
    // for p in plan.plan {
    //     println!("{}", p);
    // }

    let shared_state = Arc::new(Mutex::new(model.state.clone()));

    let robot_action_client = node.create_action_client::<URCommand::Action>("robot_action")?;
    let waiting_for_robot_action_server = node.is_available(&robot_action_client)?;

    let scanner_client = node.create_client::<TriggerScan::Service>("scanner_service")?;
    let waiting_for_scanner_server = node.is_available(&scanner_client)?;

    let shared_state_clone = shared_state.clone();
    let robot_action_timer =
        node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;
    tokio::task::spawn(async move {
        match robot_action_ticker(&robot_action_client, waiting_for_robot_action_server, &shared_state_clone, robot_action_timer, NODE_ID).await {
            Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
        };
    });

    let shared_state_clone = shared_state.clone();
    let scanner_timer =
        node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;
    tokio::task::spawn(async move {
        match scanner_client_ticker(&scanner_client, waiting_for_scanner_server, &shared_state_clone, scanner_timer, NODE_ID).await {
            Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
        };
    });
    
    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        match gantry_pub_sub_subscriber_callback(&shared_state_clone, gantry_subscriber, NODE_ID).await {
            Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
        };
    });

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        match gripper_pub_sub_subscriber_callback(&shared_state_clone, gripper_subscriber, NODE_ID).await {
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
            // &ur_action_client,
            &model,
            &shared_state_clone.clone(),
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

    let gantry_publisher_timer =
        node.create_wall_timer(std::time::Duration::from_millis(PUBLISHER_RATE))?;
    let gantry_publisher =
        node.create_publisher::<GantryOutgoing>("gantry_outgoing", QosProfile::default().reliable())?;

    let gripper_publisher_timer =
        node.create_wall_timer(std::time::Duration::from_millis(PUBLISHER_RATE))?;
    let gripper_publisher =
        node.create_publisher::<GripperOutgoing>("gripper_outgoing", QosProfile::default().reliable())?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(20));
    });

    // wait for the measured values to update the state
    // tokio::time::sleep(std::time::Duration::from_millis(5000)).await;

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        let result =
            gantry_pub_sub_publisher_callback(&shared_state_clone, gantry_publisher, gantry_publisher_timer, NODE_ID).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Publisher succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Publisher failed with: {}.", e),
        };
    });

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        // wait for the measured values to update the state
        // tokio::time::sleep(std::time::Duration::from_millis(5000)).await;
        let result =
            gripper_pub_sub_publisher_callback(&shared_state_clone, gripper_publisher, gripper_publisher_timer, NODE_ID).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Publisher succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Publisher failed with: {}.", e),
        };
    });

    handle.join().unwrap();

    r2r::log_warn!(NODE_ID, "Node started.");

    Ok(())
}