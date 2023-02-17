use futures::{Future, Stream, StreamExt};
use micro_sp::SPValue;
use micro_sp::*;
use r2r::{micro_sp_emulation_msgs::srv::TriggerScan, Error};
use std::sync::{Arc, Mutex};

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
        if scanner_trigger {
            let request = TriggerScan::Request {
                point_cloud_path: "some_path".to_string(),
                parameters: "some_parameters".to_string(),
            };
            let response = scanner_client
                .request(&request)
                .expect("Could not send tf Lookup request.")
                .await
                .expect("Cancelled.");

            match response.success {
                true => {
                    // r2r::log_info!(node_id, "Scanner service state: succeeded.");
                    let shared_state_local =
                        shared_state_local.update("scanner_state", "succeeded".to_spvalue());
                    *shared_state.lock().unwrap() = shared_state_local;
                },
                false => {
                    // r2r::log_info!(node_id, "Scanner service state: failed.");
                    let shared_state_local =
                        shared_state_local.update("scanner_state", "failed".to_spvalue());
                    *shared_state.lock().unwrap() = shared_state_local;
                },
            }
        } else {
            // r2r::log_info!(node_id, "Scanner service state: initial.");
            let shared_state_local =
                shared_state_local.update("scanner_state", "initial".to_spvalue());
            *shared_state.lock().unwrap() = shared_state_local;
        }

        timer.tick().await?;
    }
}
