use futures::Future;
use micro_sp::SPValue;
use micro_sp::*;
use r2r::{micro_sp_emulation_msgs::srv::TriggerGantry, Error};
use std::sync::{Arc, Mutex};

pub async fn gantry_client_ticker(
    gantry_cient: &r2r::Client<TriggerGantry::Service>,
    wait_for_server: impl Future<Output = Result<(), Error>>,
    shared_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_warn!(node_id, "Waiting for the gantry server...");
    wait_for_server.await?;
    r2r::log_warn!(node_id, "Gantry server available.");

    loop {
        let shsl = shared_state.lock().unwrap().clone();
        let gantry_request_trigger = match shsl.get_value("gantry_request_trigger") {
            SPValue::Bool(value) => value,
            _ => false,
        };
        let gantry_command = match shsl.get_value("gantry_command") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        let gantry_request_state = match shsl.get_value("gantry_request_state") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        if gantry_request_trigger {
            if gantry_request_state == "initial".to_string() {
                let request = TriggerGantry::Request {
                    position: gantry_command.to_string(),
                };

                let response = gantry_cient
                    .request(&request)
                    .expect("Could not send gantry request.")
                    .await
                    .expect("Cancelled.");

                let shsl = shared_state.lock().unwrap().clone();

                *shared_state.lock().unwrap() = match response.success {
                    true => shsl
                        .update("gantry_request_state", "succeeded".to_spvalue())
                        .update("gantry_actual_state", response.state.to_spvalue()),

                    false => {
                        let scanner_fail_counter = match shsl.get_value("fail_counter_gantry") {
                            SPValue::Int32(value) => value,
                            _ => 0,
                        };
                        shsl.update("gantry_request_state", "failed".to_spvalue())
                            .update(
                                "fail_counter_gantry",
                                (scanner_fail_counter + 1).to_spvalue(),
                            )
                    }
                };
            }
        }

        timer.tick().await?;
    }
}
