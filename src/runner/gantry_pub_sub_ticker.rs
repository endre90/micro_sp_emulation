use futures::{Stream, StreamExt};
use micro_sp::SPValue;
use r2r::micro_sp_emulation_msgs::msg::GantryIncoming;
use r2r::micro_sp_emulation_msgs::msg::GantryOutgoing;
use micro_sp::*;
use std::sync::{Arc, Mutex};


pub async fn gantry_pub_sub_subscriber_callback(
    shared_state: &Arc<Mutex<State>>,
    mut subscriber: impl Stream<Item = GantryIncoming> + Unpin,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let shared_state_local = shared_state.lock().unwrap().clone();
                // will have to happen for each field, but can be generated
                let shared_state_local =
                    shared_state_local.update("gantry_act", message.gantry_act.to_spvalue());
                *shared_state.lock().unwrap() = shared_state_local;
            }
            None => r2r::log_error!(node_id, "Subscriber did not get the message?"),
        }
    }
}

pub async fn gantry_pub_sub_publisher_callback(
    shared_state: &Arc<Mutex<State>>,
    publisher: r2r::Publisher<GantryOutgoing>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let shared_state_local = shared_state.lock().unwrap().clone();
        let msg = GantryOutgoing {
            gantry_ref: match shared_state_local.get_value("gantry_ref") {
                SPValue::String(value) => value,
                _ => "unknown".to_string()
            }
        };
        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => ()
        }
        timer.tick().await?;
    }
}