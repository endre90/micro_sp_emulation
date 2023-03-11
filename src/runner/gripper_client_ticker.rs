use futures::Future;
use micro_sp::SPValue;
use micro_sp::*;
use r2r::{micro_sp_emulation_msgs::srv::TriggerGripper, Error};
use std::sync::{Arc, Mutex};

pub async fn gripper_client_ticker(
    gripper_cient: &r2r::Client<TriggerGripper::Service>,
    wait_for_server: impl Future<Output = Result<(), Error>>,
    shared_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_warn!(node_id, "Waiting for the gripper server...");
    wait_for_server.await?;
    r2r::log_warn!(node_id, "Gripper server available.");

    loop {
        let shsl = shared_state.lock().unwrap().clone();
        let gripper_request_trigger = match shsl.get_value("gripper_request_trigger") {
            SPValue::Bool(value) => value,
            _ => false,
        };
        let gripper_command = match shsl.get_value("gripper_command") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        let gripper_request_state = match shsl.get_value("gripper_request_state") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        if gripper_request_trigger {
            if gripper_request_state == "initial".to_string() {
                let request = TriggerGripper::Request {
                    command: gripper_command.to_string(),
                };

                let response = gripper_cient
                    .request(&request)
                    .expect("Could not send gripper request.")
                    .await
                    .expect("Cancelled.");

                let shsl = shared_state.lock().unwrap().clone();

                *shared_state.lock().unwrap() = match response.success {
                    true => shsl
                        .update("gripper_request_state", "succeeded".to_spvalue())
                        .update("gripper_actual_state", response.state.to_spvalue()),

                    false => {
                        let scanner_fail_counter = match shsl.get_value("fail_counter_gripper") {
                            SPValue::Int32(value) => value,
                            _ => 0,
                        };
                        shsl.update("gripper_request_state", "failed".to_spvalue())
                            .update(
                                "fail_counter_gripper",
                                (scanner_fail_counter + 1).to_spvalue(),
                            )
                    }
                };
            }
        }

        timer.tick().await?;
    }
}
