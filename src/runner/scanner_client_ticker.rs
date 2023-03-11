use futures::Future;
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
    r2r::log_warn!(node_id, "Scanner server available.");

    loop {
        let shsl = shared_state.lock().unwrap().clone();
        let scanner_request_trigger = match shsl.get_value("scanner_request_trigger") {
            SPValue::Bool(value) => value,
            _ => false,
        };
        let scanner_request_state = match shsl.get_value("scanner_request_state") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        if scanner_request_trigger {
            if scanner_request_state == "initial".to_string() {
                let request = TriggerScan::Request {
                    point_cloud_path: "/path/to/file".to_string(),
                    parameters: "parameters".to_string(),
                };

                let response = scanner_client
                    .request(&request)
                    .expect("Could not send scan request.")
                    .await
                    .expect("Cancelled.");

                let shsl = shared_state.lock().unwrap().clone();

                *shared_state.lock().unwrap() = match response.success {
                    true => shsl.update("scanner_request_state", "succeeded".to_spvalue()),
                    false => {
                        let fail_counter_scanner = match shsl.get_value("fail_counter_scanner") {
                            SPValue::Int32(value) => value,
                            _ => 0,
                        };
                        shsl.update("scanner_request_state", "failed".to_spvalue())
                            .update(
                                "fail_counter_scanner",
                                (fail_counter_scanner + 1).to_spvalue(),
                            )
                    }
                }
            }
        }
        timer.tick().await?;
    }
}
