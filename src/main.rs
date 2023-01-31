use r2r::QosProfile;
use r2r::micro_sp_emulation_msgs::msg::DummyIncoming;
use r2r::micro_sp_emulation_msgs::msg::DummyOutgoing;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "micro_sp_runner";
pub static TICKER_RATE: u64 = 100;
pub static PUBLISHER_RATE: u64 = 100;

mod runner;
use runner::dummy_pub_sub_model::*;
use runner::dummy_pub_sub_ticker::*;
use runner::ticker::*;


#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    // let ur_action_client = node.create_action_client::<URControl::Action>("ur_control")?;
    // let waiting_for_ur_action_client = node.is_available(&ur_action_client)?;

    let ticker_timer =
    node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;

    let subscriber = node.subscribe::<DummyIncoming>(
        "dummy_incoming",
        QosProfile::best_effort(QosProfile::default()),
    )?;

    let publisher_timer =
        node.create_wall_timer(std::time::Duration::from_millis(PUBLISHER_RATE))?;
    let publisher =
        node.create_publisher::<DummyOutgoing>("dummy_outgoing", QosProfile::default())?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!(NODE_ID, "Waiting for the UR Action Service...");
    // waiting_for_ur_action_client.await?;
    r2r::log_info!(NODE_ID, "UR Action available.");

    let model = the_model();
    let shared_state = Arc::new(Mutex::new(model.state.clone()));
    let shared_state_clone = shared_state.clone();
   
    tokio::task::spawn(async move {
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

    
    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        match dummy_pub_sub_subscriber_callback(&shared_state_clone, subscriber, NODE_ID).await {
            Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
        };
    });

    
    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        let result =
            dummy_pub_sub_publisher_callback(&shared_state_clone, publisher, publisher_timer, NODE_ID).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Publisher succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Publisher failed with: {}.", e),
        };
    });

    handle.join().unwrap();

    r2r::log_warn!(NODE_ID, "Node started.");

    Ok(())
}