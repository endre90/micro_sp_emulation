use micro_sp::Model;
use micro_sp::ShardedMutex;
use micro_sp::State;
// use micro_sp::{State, ShardedMutex};
use r2r::micro_sp_emulation_msgs::srv::TriggerGripper;
use r2r::micro_sp_emulation_msgs::srv::TriggerScan;
use std::collections::HashMap;
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

    // Still can't figure this out... 
    // let sharded_mutex = Arc::new(ShardedMutex::new(model.state.state.len()));
    // for (key, value) in model.state.state.clone() {
    //     let mut shard = sharded_mutex.lock(key.as_str()).clone();
    //     // println!("ADDING: {:?} to {:?}", key, shard);
    //     shard.insert(key.clone(), value);
    //     *sharded_mutex.lock(key.clone().as_str()) = shard; //HashMap::from([(key.clone(), shard)]);
    // }

    // println!("SHARDED {:?}", sharded_mutex);

    let shared_state = Arc::new(Mutex::new(model.state.clone()));

    // let robot_action_client = node.create_action_client::<URCommand::Action>("robot_action")?;
    // let waiting_for_robot_action_server = node.is_available(&robot_action_client)?;

    let scanner_client = node.create_client::<TriggerScan::Service>("scanner_service")?;
    let gripper_client = node.create_client::<TriggerGripper::Service>("gripper_service")?;

    let waiting_for_scanner_server = node.is_available(&scanner_client)?;
    let waiting_for_gripper_server = node.is_available(&gripper_client)?;

    // let shared_state_clone = shared_state.clone();
    // let robot_action_timer =
    //     node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;
    // tokio::task::spawn(async move {
    //     match robot_action_ticker(&robot_action_client, waiting_for_robot_action_server, &shared_state_clone, robot_action_timer, NODE_ID).await {
    //         Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
    //         Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
    //     };
    // });

    let shared_state_clone = shared_state.clone();
    // let sharded_mutex_clone = sharded_mutex.clone();
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
    // let sharded_mutex_clone = sharded_mutex.clone();
    tokio::task::spawn(async move {
        // wait for the measured values to update the state
        // tokio::time::sleep(std::time::Duration::from_millis(5000)).await;
        let result = ticker(
            NODE_ID,
            // &ur_action_client,
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

// proptest! {
//     #![proptest_config(ProptestConfig::with_cases(10))]
//     #[test]
//     fn my_behavior_model_works(gantry_act_val in prop_oneof!("a", "b")) {

//         let m = rita_model();
//         // let model = Model::new(&m.0, m.1, m.2, m.3, m.4);
//         // let gantry_act = v_measured!("gantry_act", vec!("a", "b", "atr"));
//         let new_state = m.1.update("gantry_act", gantry_act_val.to_spvalue());

//         let model = Model::new(
//             "asdf",
//             new_state.clone(),
//             m.2,
//             m.3,
//             vec!()
//         );

//         let plan = bfs_operation_planner(model.state.clone(), extract_goal_from_state(&model.state.clone()), model.operations.clone(), 50);
//         for p in plan.plan {
//             println!("{}", p);
//         }

//         // let mut runner = TestRunner::default();
//         // let config = ProptestConfig::with_cases(10); // Set the number of test cases to 10
//         // runner.set_config(config);

//         prop_assert!(plan.found);
//         // prop_assert!(!model.is_empty());
//         // prop_assert!(model.last_value().is_some());
//     }
// }

// proptest! {
//     #![proptest_config(ProptestConfig::with_cases(10))]
//     #[test]
//     fn test_the_gripper_mcdc(gantry_act_val in prop_oneof!("a", "b")) {

//     let ctx = r2r::Context::create()?;
//     let mut node = r2r::Node::create(ctx, "gripper_test", "")?;

//     let ticker_timer =
//         node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;

//     let gantry_subscriber = node.subscribe::<GantryIncoming>(
//         "gantry_incoming",
//         QosProfile::default().reliable()
//     )?;

//     let gripper_subscriber = node.subscribe::<GripperIncoming>(
//         "gripper_incoming",
//         QosProfile::default().reliable()
//     )?;

//         prop_assert!(plan.found);
//         // prop_assert!(!model.is_empty());
//         // prop_assert!(model.last_value().is_some());
//     }
// }
