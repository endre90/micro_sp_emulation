use futures::Future;
use micro_sp::SPValue;
use micro_sp::*;
use r2r::{micro_sp_emulation_msgs::srv::TriggerRobot, Error};
use std::sync::{Arc, Mutex};

pub async fn robot_client_ticker(
    robot_cient: &r2r::Client<TriggerRobot::Service>,
    wait_for_server: impl Future<Output = Result<(), Error>>,
    shared_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_warn!(node_id, "Waiting for the robot server...");
    wait_for_server.await?;
    r2r::log_warn!(node_id, "Robot server available.");

    loop {
        let shsl = shared_state.lock().unwrap().clone();
        let robot_request_trigger = match shsl.get_value("robot_request_trigger") {
            SPValue::Bool(value) => value,
            _ => false,
        };
        let robot_command = match shsl.get_value("robot_command") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        let robot_position = match shsl.get_value("robot_position") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        let robot_request_state = match shsl.get_value("robot_request_state") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        if robot_request_trigger {
            if robot_request_state == "initial".to_string() {
                let request = TriggerRobot::Request {
                    command: robot_command.to_string(),
                    position: robot_position.to_string(),
                };

                let response = robot_cient
                    .request(&request)
                    .expect("Could not send robot request.")
                    .await
                    .expect("Cancelled.");

                let shsl = shared_state.lock().unwrap().clone();

                *shared_state.lock().unwrap() = match response.success {
                    true => match robot_command.as_str() {
                        "mount" | "unmount" => {
                            shsl.update("robot_request_state", "succeeded".to_spvalue())
                        }
                        "move" => shsl
                            .update("robot_request_state", "succeeded".to_spvalue())
                            .update("robot_actual_state", request.position.to_spvalue()),
                        _ => shsl,
                    },

                    false => {
                        let robot_fail_counter = match shsl.get_value("fail_counter_robot") {
                            SPValue::Int32(value) => value,
                            _ => 0,
                        };
                        shsl.update("robot_request_state", "failed".to_spvalue())
                            .update("fail_counter_robot", (robot_fail_counter + 1).to_spvalue())
                    }
                };
            }
        }

        timer.tick().await?;
    }
}
