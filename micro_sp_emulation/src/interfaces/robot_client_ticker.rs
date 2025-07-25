use std::sync::Arc;

use micro_sp::*;
use r2r::micro_sp_emulation_msgs::msg::Emulation;
use r2r::micro_sp_emulation_msgs::srv::TriggerRobot;
use redis::AsyncCommands;

pub async fn robot_client_ticker(
    client: &r2r::Client<TriggerRobot::Service>,
    mut timer: r2r::Timer,
    connection_manager: &Arc<ConnectionManager>,
) -> Result<(), Box<dyn std::error::Error>> {
    let log_target = "robot_interface";
    log::info!(target: log_target, "Online.");

    let keys: Vec<String> = vec![
        "robot_request_trigger",
        "robot_request_state",
        "robot_total_fail_counter",
        "robot_subsequent_fail_counter",
        "robot_command_command",
        "robot_speed_command",
        "robot_position_command",
        "robot_position_estimated",
        "robot_mounted_one_time_measured",
        "robot_emulate_execution_time",
        "robot_emulated_execution_time",
        "robot_emulate_failure_rate",
        "robot_emulated_failure_rate",
        "robot_emulate_failure_cause",
        "robot_emulated_failure_cause",
    ]
    .iter()
    .map(|k| k.to_string())
    .collect();

    loop {
        timer.tick().await?;
        let mut con = connection_manager.get_connection().await;

        if let Err(e) = con.set::<_, _, ()>("heartbeat", "alive").await {
            handle_redis_error(&e, &log_target, connection_manager).await;
            continue;
        }
        let state = match redis_get_state_for_keys(&mut con, &keys).await {
            Some(s) => s,
            None => continue,
        };
        let mut request_trigger =
            state.get_bool_or_default_to_false(log_target, "robot_request_trigger");
        let mut request_state =
            state.get_string_or_default_to_unknown(log_target, "robot_request_state");
        let mut total_fail_counter =
            state.get_int_or_default_to_zero(log_target, "robot_total_fail_counter");
        let mut subsequent_fail_counter =
            state.get_int_or_default_to_zero(log_target, "robot_subsequent_fail_counter");
        let robot_command_command =
            state.get_string_or_default_to_unknown(log_target, "robot_command_command");
        let robot_speed_command = state.get_float_or_default_to_zero(log_target, "robot_speed_command");
        let robot_position_command =
            state.get_string_or_default_to_unknown(log_target, "robot_position_command");
        let mut robot_position_estimated =
            state.get_string_or_default_to_unknown(log_target, "robot_position_estimated");
        // let mut robot_mounted_estimated =
        //     state.get_or_default_string(target, "robot_mounted_estimated");
        // let mut robot_locked_estimated = state.get_bool(target, "robot_locked_estimated");
        let mut robot_mounted_one_time_measured =
            state.get_string_or_default_to_unknown(log_target, "robot_mounted_one_time_measured");
        let emulate_execution_time =
            state.get_int_or_default_to_zero(log_target, "robot_emulate_execution_time");
        let emulated_execution_time =
            state.get_int_or_default_to_zero(log_target, "robot_emulated_execution_time");
        let emulate_failure_rate =
            state.get_int_or_default_to_zero(log_target, "robot_emulate_failure_rate");
        let emulated_failure_rate =
            state.get_int_or_default_to_zero(log_target, "robot_emulated_failure_rate");
        let emulate_failure_cause =
            state.get_int_or_default_to_zero(log_target, "robot_emulate_failure_cause");
        let emulated_failure_cause_sp_value =
            state.get_array_or_default_to_empty(log_target, "robot_emulated_failure_cause");

        let emulated_failure_cause: Vec<String> = emulated_failure_cause_sp_value
            .iter()
            .filter(|val| val.is_string())
            .map(|y| y.to_string())
            .collect();

        if request_trigger {
            request_trigger = false;
            if request_state == ServiceRequestState::Initial.to_string() {
                log::info!(target:
                    "robot_interface",
                    "Requesting to {}.",
                    robot_command_command
                );
                let request = TriggerRobot::Request {
                    command: robot_command_command.clone(),
                    speed: robot_speed_command as f32,
                    position: robot_position_command.clone(),
                    emulated_response: Emulation {
                        emulate_execution_time: emulate_execution_time as u8,
                        emulated_execution_time: emulated_execution_time as i32,
                        emulate_failure_rate: emulate_failure_rate as u8,
                        emulated_failure_rate: emulated_failure_rate as i32,
                        emulate_failure_cause: emulate_failure_cause as u8,
                        emulated_failure_cause,
                    },
                };

                match client.request(&request) {
                    Ok(future) => match future.await {
                        Ok(response) => match robot_command_command.as_str() {
                            "move" => {
                                if response.success {
                                    log::info!(target:
                                        "robot_interface",
                                        "Requested move to '{}' succeeded.",
                                        robot_position_command
                                    );
                                    request_state = ServiceRequestState::Succeeded.to_string();
                                    robot_position_estimated = robot_position_command;
                                    subsequent_fail_counter = 0;
                                } else {
                                    log::error!(target:
                                        "robot_interface",
                                        "Requested move to '{}' failed.",
                                        robot_position_command
                                    );
                                    request_state = ServiceRequestState::Failed.to_string();
                                    subsequent_fail_counter = subsequent_fail_counter + 1;
                                    total_fail_counter = total_fail_counter + 1;
                                }
                            }
                            "pick" => {
                                if response.success {
                                    log::info!(target: "robot_interface", "Requested pick succeeded.");
                                    request_state = ServiceRequestState::Succeeded.to_string();
                                    subsequent_fail_counter = 0;
                                } else {
                                    log::error!(target: "robot_interface", "Requested pick failed.");
                                    request_state = ServiceRequestState::Failed.to_string();
                                    subsequent_fail_counter = subsequent_fail_counter + 1;
                                    total_fail_counter = total_fail_counter + 1;
                                }
                            }
                            "place" => {
                                if response.success {
                                    log::info!(target: "robot_interface", "Requested place succeeded.");
                                    request_state = ServiceRequestState::Succeeded.to_string();
                                    subsequent_fail_counter = 0;
                                } else {
                                    log::error!(target: "robot_interface", "Requested place failed.");
                                    request_state = ServiceRequestState::Failed.to_string();
                                    subsequent_fail_counter = subsequent_fail_counter + 1;
                                    total_fail_counter = total_fail_counter + 1;
                                }
                            }
                            "mount" => {
                                if response.success {
                                    log::info!(target: "robot_interface", "Requested mount succeeded.");
                                    request_state = ServiceRequestState::Succeeded.to_string();
                                    subsequent_fail_counter = 0;
                                } else {
                                    log::error!(target: "robot_interface", "Requested mount failed.");
                                    request_state = ServiceRequestState::Failed.to_string();
                                    subsequent_fail_counter = subsequent_fail_counter + 1;
                                    total_fail_counter = total_fail_counter + 1;
                                }
                            }
                            "unmount" => {
                                if response.success {
                                    log::info!(target: "robot_interface",
                                        "Requested unmount succeeded."
                                    );
                                    request_state = ServiceRequestState::Succeeded.to_string();
                                    subsequent_fail_counter = 0;
                                } else {
                                    log::error!(target: "robot_interface", "Requested unmount failed.");
                                    request_state = ServiceRequestState::Failed.to_string();
                                    subsequent_fail_counter = subsequent_fail_counter + 1;
                                    total_fail_counter = total_fail_counter + 1;
                                }
                            }
                            "check_mounted_tool" => {
                                if response.success {
                                    log::info!(target: "robot_interface",
                                        "Requested check_mounted_tool succeeded."
                                    );
                                    request_state = ServiceRequestState::Succeeded.to_string();
                                    robot_mounted_one_time_measured = response.checked_mounted_tool;
                                    subsequent_fail_counter = 0;
                                } else {
                                    log::error!(target: "robot_interface",
                                        "Requested check_mounted_tool failed."
                                    );
                                    request_state = ServiceRequestState::Failed.to_string();
                                    robot_mounted_one_time_measured = "UNKNOWN".to_string();
                                    subsequent_fail_counter = subsequent_fail_counter + 1;
                                    total_fail_counter = total_fail_counter + 1;
                                }
                            }
                            _ => {
                                log::error!(target: "robot_interface",
                                    "Requested command '{}' is invalid.",
                                    robot_command_command
                                );
                                request_state = ServiceRequestState::Failed.to_string();
                                subsequent_fail_counter = subsequent_fail_counter + 1;
                                total_fail_counter = total_fail_counter + 1;
                            }
                        },
                        Err(e) => {
                            log::error!(target: "robot_interface", "Request failed with: {e}.");
                            request_state = ServiceRequestState::Failed.to_string();
                            subsequent_fail_counter = subsequent_fail_counter + 1;
                            total_fail_counter = total_fail_counter + 1;
                        }
                    },
                    Err(e) => {
                        log::error!(target: "robot_interface", "Request failed with: {e}.");
                        request_state = ServiceRequestState::Failed.to_string();
                        subsequent_fail_counter = subsequent_fail_counter + 1;
                        total_fail_counter = total_fail_counter + 1;
                    }
                };
            }
        }

        let new_state = state
            .update("robot_request_trigger", request_trigger.to_spvalue())
            .update("robot_request_state", request_state.to_spvalue())
            .update("robot_total_fail_counter", total_fail_counter.to_spvalue())
            .update(
                "robot_subsequent_fail_counter",
                subsequent_fail_counter.to_spvalue(),
            )
            .update(
                "robot_position_estimated",
                robot_position_estimated.to_spvalue(),
            )
            .update(
                "robot_mounted_one_time_measured",
                robot_mounted_one_time_measured.to_spvalue(),
            );

        let modified_state = state.get_diff_partial_state(&new_state);
        redis_set_state(&mut con, modified_state).await;
    }
}
