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

    let shared_state_local = shared_state.lock().unwrap().clone();
    let gripper_request_fail_counter = iv_runner!("gripper_request_fail_counter");
    let failed_to_grip_counter = iv_runner!("failed_to_grip_counter");
    let shared_state_local =
        shared_state_local.add(assign!(gripper_request_fail_counter, 0.to_spvalue()));
    let shared_state_local =
        shared_state_local.add(assign!(failed_to_grip_counter, 0.to_spvalue()));
    *shared_state.lock().unwrap() = shared_state_local;

    loop {
        let shared_state_local = shared_state.lock().unwrap().clone();
        let gripper_request_trigger = match shared_state_local.get_value("gripper_request_trigger")
        {
            SPValue::Bool(value) => value,
            _ => false,
        };
        let gripper_command = match shared_state_local.get_value("gripper_command") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        let gripper_request_state = match shared_state_local.get_value("gripper_request_state") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        if gripper_request_trigger {
            if gripper_request_state == "initial".to_string() {
                let request = TriggerGripper::Request {
                    command: gripper_command.to_string(),
                };

                // maybe an adversary could fail this at 50/50 as well?
                let response = gripper_cient
                    .request(&request)
                    .expect("Could not send gripper request.")
                    .await
                    .expect("Cancelled.");

                let shared_state_local = match response.success {
                    true => {
                        let shared_state_local = shared_state_local
                            .update("gripper_request_state", "succeeded".to_spvalue());
                        shared_state_local
                            .update("gripper_actual_state", response.state.to_spvalue())
                    }
                    false => {
                        let shared_state_local = shared_state_local
                            .update("gripper_request_state", "failed".to_spvalue());
                        let scanner_fail_counter =
                            match shared_state_local.get_value("gripper_fail_counter") {
                                SPValue::Int32(value) => value,
                                _ => 0,
                            };
                        shared_state_local.update(
                            "gripper_fail_counter",
                            (scanner_fail_counter + 1).to_spvalue(),
                        )
                    }
                };
                *shared_state.lock().unwrap() = shared_state_local;
            }
        }

        timer.tick().await?;
    }
}
