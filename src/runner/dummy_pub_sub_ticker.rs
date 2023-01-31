use futures::{Stream, StreamExt};
use micro_sp::SPValue;
use r2r::micro_sp_emulation_msgs::msg::DummyIncoming;
use r2r::micro_sp_emulation_msgs::msg::DummyOutgoing;
use micro_sp::*;
use std::sync::{Arc, Mutex};


pub async fn dummy_pub_sub_subscriber_callback(
    shared_state: &Arc<Mutex<State>>,
    mut subscriber: impl Stream<Item = DummyIncoming> + Unpin,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let shared_state_local = shared_state.lock().unwrap().clone();
                // will have to hapen for each field, but can be generated
                let shared_state_local =
                    shared_state_local.update("act_pos", message.act_pos.to_spvalue());
                *shared_state.lock().unwrap() = shared_state_local;
            }
            None => r2r::log_error!(node_id, "Subscriber did not get the message?"),
        }
    }
}

pub async fn dummy_pub_sub_publisher_callback(
    shared_state: &Arc<Mutex<State>>,
    publisher: r2r::Publisher<DummyOutgoing>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let shared_state_local = shared_state.lock().unwrap().clone();
        let msg = DummyOutgoing {
            ref_pos: match shared_state_local.get_value("ref_pos") {
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