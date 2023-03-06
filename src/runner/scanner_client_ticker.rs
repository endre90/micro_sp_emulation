use futures::{Future, Stream, StreamExt};
use micro_sp::SPValue;
use micro_sp::*;
use r2r::{micro_sp_emulation_msgs::srv::TriggerScan, Error};
use std::sync::{Arc, Mutex};

// pub async fn update_shared_state(
//     // var: &str,
//     // val: SPValue,
//     // updates: Vec<(&str, SPValue)>,
//     update: (&str, SPValue),
//     shared_state: &Arc<Mutex<State>>,
// ) -> State {
//     let shared_state_local = shared_state.lock().unwrap().clone();
//     // let mut updated_state = State::new();
//     let updated_state = shared_state_local.update(update.0, update.1.to_owned());
//     // updates
//     //     .iter()
//     //     .for_each(|(var, val)| updated_state = shared_state_local.update(var, val.to_owned()));
//     // let updated_state = shared_state_local.update(var, val);
//     *shared_state.lock().unwrap() = updated_state.clone();
//     updated_state
// }

pub async fn scanner_client_ticker(
    scanner_client: &r2r::Client<TriggerScan::Service>,
    wait_for_server: impl Future<Output = Result<(), Error>>,
    shared_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_warn!(node_id, "Waiting for the scanner server...");
    wait_for_server.await?;
    r2r::log_info!(node_id, "Scanner server available.");
    loop {
        let shared_state_local = shared_state.lock().unwrap().clone();
        let scanner_trigger = match shared_state_local.get_value("scanner_trigger") {
            SPValue::Bool(value) => value,
            _ => false,
        };
        let scanner_state = match shared_state_local.get_value("scanner_state") {
            SPValue::String(value) => value,
            _ => "initial".to_string(),
        };
        if scanner_trigger {
            if scanner_state == "initial".to_string() {
                // update_shared_state(("scanner_trigger", false.to_spvalue()), shared_state).await;
                let request = TriggerScan::Request {
                    point_cloud_path: "some_path".to_string(),
                    parameters: "some_parameters".to_string(),
                };

                // maybe an adversary could fail this at 50/50 as well?
                let response = scanner_client
                    .request(&request)
                    .expect("Could not send scan request.")
                    .await
                    .expect("Cancelled.");

                match response.success {
                    true => {
                        // update_shared_state(("scanner_state", "succeeded".to_spvalue()), shared_state).await;
                        // update_shared_state(("scanner_trigger", false.to_spvalue()), shared_state).await;
                        // r2r::log_info!(node_id, "Scanner service state: succeeded.");
                        let shared_state_local =
                            shared_state_local.update("scanner_state", "succeeded".to_spvalue());
                        // let shared_state_local =
                        //     shared_state_local.update("scanner_trigger", false.to_spvalue());
                        *shared_state.lock().unwrap() = shared_state_local;
                        // *shared_state.lock().unwrap() = shared_state_local;
                    }
                    false => {
                        // r2r::log_info!(node_id, "Scanner service state: failed.");
                        let shared_state_local =
                            shared_state_local.update("scanner_state", "failed".to_spvalue());
                        // let shared_state_local =
                        //     shared_state_local.update("scanner_trigger", false.to_spvalue());
                        *shared_state.lock().unwrap() = shared_state_local;
                        // update_shared_state(("scanner_trigger", false.to_spvalue()), shared_state).await;
                        // let shared_state_local =
                        //     shared_state_local.update("scanner_state", "failed".to_spvalue());
                        // *shared_state.lock().unwrap() = shared_state_local;
                    }
                }
            }
        } else {
            // update_shared_state(("scanner_trigger", false.to_spvalue()), shared_state).await;
            // r2r::log_info!(node_id, "Scanner service state: initial.");
            let shared_state_local =
                shared_state_local.update("scanner_state", "initial".to_spvalue());
            *shared_state.lock().unwrap() = shared_state_local;
        }

        timer.tick().await?;
    }
}
