use std::sync::Arc;

use micro_sp::*;
use r2r::micro_sp_emulation_msgs::msg::Emulation;
use r2r::micro_sp_emulation_msgs::srv::TriggerGantry;
use redis::AsyncCommands;

pub async fn gantry_client_ticker(
    client: &r2r::Client<TriggerGantry::Service>,
    mut timer: r2r::Timer,
    connection_manager: &Arc<ConnectionManager>,
) -> Result<(), Box<dyn std::error::Error>> {
    let log_target = "gantry_interface";
    log::info!(target: log_target, "Online.");

    let keys: Vec<String> = vec![
        "gantry_request_trigger",
        "gantry_request_state",
        "gantry_total_fail_counter",
        "gantry_subsequent_fail_counter",
        "gantry_command_command",
        "gantry_speed_command",
        "gantry_position_command",
        "gantry_position_estimated",
        "gantry_calibrated_estimated",
        "gantry_locked_estimated",
        "gantry_emulate_execution_time",
        "gantry_emulated_execution_time",
        "gantry_emulate_failure_rate",
        "gantry_emulated_failure_rate",
        "gantry_emulate_failure_cause",
        "gantry_emulate_failure_cause",
        "gantry_emulated_failure_cause",
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
                state.get_bool_or_default_to_false(log_target, "gantry_request_trigger");
            let mut request_state =
                state.get_string_or_default_to_unknown(log_target, "gantry_request_state");
            let mut total_fail_counter =
                state.get_int_or_default_to_zero(log_target, "gantry_total_fail_counter");
            let mut subsequent_fail_counter =
                state.get_int_or_default_to_zero(log_target, "gantry_subsequent_fail_counter");
            let gantry_command_command =
                state.get_string_or_default_to_unknown(log_target, "gantry_command_command");
            let gantry_speed_command =
                state.get_float_or_default_to_zero(log_target, "gantry_speed_command");
            let gantry_position_command =
                state.get_string_or_default_to_unknown(log_target, "gantry_position_command");
            let mut gantry_position_estimated =
                state.get_string_or_default_to_unknown(log_target, "gantry_position_estimated");
            let mut gantry_calibrated_estimated =
                state.get_bool_or_default_to_false(log_target, "gantry_calibrated_estimated");
            let mut gantry_locked_estimated =
                state.get_bool_or_default_to_false(log_target, "gantry_locked_estimated");
            let emulate_execution_time =
                state.get_int_or_default_to_zero(log_target, "gantry_emulate_execution_time");
            let emulated_execution_time =
                state.get_int_or_default_to_zero(log_target, "gantry_emulated_execution_time");
            let emulate_failure_rate =
                state.get_int_or_default_to_zero(log_target, "gantry_emulate_failure_rate");
            let emulated_failure_rate =
                state.get_int_or_default_to_zero(log_target, "gantry_emulated_failure_rate");
            let emulate_failure_cause =
                state.get_int_or_default_to_zero(log_target, "gantry_emulate_failure_cause");
            let emulated_failure_cause_sp_value =
                state.get_array_or_default_to_empty(log_target, "gantry_emulated_failure_cause");

            let emulated_failure_cause: Vec<String> = emulated_failure_cause_sp_value
                .iter()
                .filter(|val| val.is_string())
                .map(|y| y.to_string())
                .collect();

            if request_trigger {
                request_trigger = false; // Consume the trigger
                if request_state == ServiceRequestState::Initial.to_string() {
                    log::info!(target: log_target, "Requesting to {}.", gantry_command_command);
                    let request = TriggerGantry::Request {
                        command: gantry_command_command.clone(),
                        speed: gantry_speed_command as f32,
                        position: gantry_position_command.clone(),
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
                            Ok(response) => match gantry_command_command.as_str() {
                                "move" => {
                                    if response.success {
                                        log::info!(target:
                                            "gantry_interface",
                                            "Requested move to '{}' succeeded.",
                                            gantry_position_command
                                        );
                                        request_state = ServiceRequestState::Succeeded.to_string();
                                        gantry_position_estimated = gantry_position_command;
                                        subsequent_fail_counter = 0;
                                    } else {
                                        log::error!(target:
                                            "gantry_interface",
                                            "Requested move to '{}' failed.",
                                            gantry_position_command
                                        );
                                        request_state = ServiceRequestState::Failed.to_string();
                                        subsequent_fail_counter = subsequent_fail_counter + 1;
                                        total_fail_counter = total_fail_counter + 1;
                                    }
                                }
                                "calibrate" => {
                                    if response.success {
                                        log::info!(target:
                                            "gantry_interface",
                                            "Requested calibration succeeded."
                                        );
                                        request_state = ServiceRequestState::Succeeded.to_string();
                                        gantry_calibrated_estimated = true;
                                        subsequent_fail_counter = 0;
                                    } else {
                                        log::error!(target:
                                            "gantry_interface",
                                            "Requested calibration failed."
                                        );
                                        request_state = ServiceRequestState::Failed.to_string();
                                        subsequent_fail_counter = subsequent_fail_counter + 1;
                                        total_fail_counter = total_fail_counter + 1;
                                    }
                                }
                                "lock" => {
                                    if response.success {
                                        log::info!(target: "gantry_interface", "Requested lock succeeded.");
                                        request_state = ServiceRequestState::Succeeded.to_string();
                                        gantry_locked_estimated = true;
                                        subsequent_fail_counter = 0;
                                    } else {
                                        log::error!(target: "gantry_interface", "Requested lock failed.");
                                        request_state = ServiceRequestState::Failed.to_string();
                                        subsequent_fail_counter = subsequent_fail_counter + 1;
                                        total_fail_counter = total_fail_counter + 1;
                                    }
                                }
                                "unlock" => {
                                    if response.success {
                                        log::info!(target:
                                            "gantry_interface",
                                            "Requested unlock succeeded."
                                        );
                                        request_state = ServiceRequestState::Succeeded.to_string();
                                        gantry_locked_estimated = false;
                                        subsequent_fail_counter = 0;
                                    } else {
                                        log::error!(target: "gantry_interface", "Requested unlock failed.");
                                        request_state = ServiceRequestState::Failed.to_string();
                                        subsequent_fail_counter = subsequent_fail_counter + 1;
                                        total_fail_counter = total_fail_counter + 1;
                                    }
                                }
                                _ => {
                                    log::error!(target:
                                        "gantry_interface",
                                        "Requested command '{}' is invalid.",
                                        gantry_command_command
                                    );
                                    request_state = ServiceRequestState::Failed.to_string();
                                    subsequent_fail_counter = subsequent_fail_counter + 1;
                                    total_fail_counter = total_fail_counter + 1;
                                }
                            },
                            Err(e) => {
                                log::error!(target: "gantry_interface", "Request failed with: {e}.");
                                request_state = ServiceRequestState::Failed.to_string();
                                subsequent_fail_counter = subsequent_fail_counter + 1;
                                total_fail_counter = total_fail_counter + 1;
                            }
                        },
                        Err(e) => {
                            log::error!(target: "gantry_interface", "Request failed with: {e}.");
                            request_state = ServiceRequestState::Failed.to_string();
                            subsequent_fail_counter = subsequent_fail_counter + 1;
                            total_fail_counter = total_fail_counter + 1;
                        }
                    };
                }
            }

            let new_state = state
                .update("gantry_request_trigger", request_trigger.to_spvalue())
                .update("gantry_request_state", request_state.to_spvalue())
                .update("gantry_total_fail_counter", total_fail_counter.to_spvalue())
                .update(
                    "gantry_subsequent_fail_counter",
                    subsequent_fail_counter.to_spvalue(),
                )
                .update(
                    "gantry_position_estimated",
                    gantry_position_estimated.to_spvalue(),
                )
                .update(
                    "gantry_calibrated_estimated",
                    gantry_calibrated_estimated.to_spvalue(),
                )
                .update(
                    "gantry_locked_estimated",
                    gantry_locked_estimated.to_spvalue(),
                );

            let modified_state = state.get_diff_partial_state(&new_state);
            redis_set_state(&mut con, modified_state).await;
        }
}

// pub async fn gantry_client_ticker(
//     client: &r2r::Client<TriggerGantry::Service>,
//     mut timer: r2r::Timer,
//     mut con: MultiplexedConnection,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     let target = "gantry_interface";
//     log::info!(target: target, "Online.");

//     loop {
//         if let Some(state) = redis_get_state(&mut con).await {
//             let mut request_trigger =
//                 state.get_bool_or_default_to_false(target, "gantry_request_trigger");
//             let mut request_state =
//                 state.get_string_or_default_to_unknown(target, "gantry_request_state");
//             let mut total_fail_counter =
//                 state.get_int_or_default_to_zero(target, "gantry_total_fail_counter");
//             let mut subsequent_fail_counter =
//                 state.get_int_or_default_to_zero(target, "gantry_subsequent_fail_counter");
//             let gantry_command_command =
//                 state.get_string_or_default_to_unknown(target, "gantry_command_command");
//             let gantry_speed_command =
//                 state.get_float_or_default_to_zero(target, "gantry_speed_command");
//             let gantry_position_command =
//                 state.get_string_or_default_to_unknown(target, "gantry_position_command");
//             let mut gantry_position_estimated =
//                 state.get_string_or_default_to_unknown(target, "gantry_position_estimated");
//             let mut gantry_calibrated_estimated =
//                 state.get_bool_or_default_to_false(target, "gantry_calibrated_estimated");
//             let mut gantry_locked_estimated =
//                 state.get_bool_or_default_to_false(target, "gantry_locked_estimated");
//             let emulate_execution_time =
//                 state.get_int_or_default_to_zero(target, "gantry_emulate_execution_time");
//             let emulated_execution_time =
//                 state.get_int_or_default_to_zero(target, "gantry_emulated_execution_time");
//             let emulate_failure_rate =
//                 state.get_int_or_default_to_zero(target, "gantry_emulate_failure_rate");
//             let emulated_failure_rate =
//                 state.get_int_or_default_to_zero(target, "gantry_emulated_failure_rate");
//             let emulate_failure_cause =
//                 state.get_int_or_default_to_zero(target, "gantry_emulate_failure_cause");
//             let emulated_failure_cause_sp_value =
//                 state.get_array_or_default_to_empty(target, "gantry_emulated_failure_cause");

//             let emulated_failure_cause: Vec<String> = emulated_failure_cause_sp_value
//                 .iter()
//                 .filter(|val| val.is_string())
//                 .map(|y| y.to_string())
//                 .collect();

//             if request_trigger {
//                 request_trigger = false;
//                 if request_state == ServiceRequestState::Initial.to_string() {
//                     log::info!(target:
//                         "gantry_interface",
//                         "Requesting to {}.",
//                         gantry_command_command
//                     );
//                     let request = TriggerGantry::Request {
//                         command: gantry_command_command.clone(),
//                         speed: gantry_speed_command as f32,
//                         position: gantry_position_command.clone(),
//                         emulated_response: Emulation {
//                             emulate_execution_time: emulate_execution_time as u8,
//                             emulated_execution_time: emulated_execution_time as i32,
//                             emulate_failure_rate: emulate_failure_rate as u8,
//                             emulated_failure_rate: emulated_failure_rate as i32,
//                             emulate_failure_cause: emulate_failure_cause as u8,
//                             emulated_failure_cause,
//                         },
//                     };

//                     match client.request(&request) {
//                         Ok(future) => match future.await {
//                             Ok(response) => match gantry_command_command.as_str() {
//                                 "move" => {
//                                     if response.success {
//                                         log::info!(target:
//                                             "gantry_interface",
//                                             "Requested move to '{}' succeeded.",
//                                             gantry_position_command
//                                         );
//                                         request_state = ServiceRequestState::Succeeded.to_string();
//                                         gantry_position_estimated = gantry_position_command;
//                                         subsequent_fail_counter = 0;
//                                     } else {
//                                         log::error!(target:
//                                             "gantry_interface",
//                                             "Requested move to '{}' failed.",
//                                             gantry_position_command
//                                         );
//                                         request_state = ServiceRequestState::Failed.to_string();
//                                         subsequent_fail_counter = subsequent_fail_counter + 1;
//                                         total_fail_counter = total_fail_counter + 1;
//                                     }
//                                 }
//                                 "calibrate" => {
//                                     if response.success {
//                                         log::info!(target:
//                                             "gantry_interface",
//                                             "Requested calibration succeeded."
//                                         );
//                                         request_state = ServiceRequestState::Succeeded.to_string();
//                                         gantry_calibrated_estimated = true;
//                                         subsequent_fail_counter = 0;
//                                     } else {
//                                         log::error!(target:
//                                             "gantry_interface",
//                                             "Requested calibration failed."
//                                         );
//                                         request_state = ServiceRequestState::Failed.to_string();
//                                         subsequent_fail_counter = subsequent_fail_counter + 1;
//                                         total_fail_counter = total_fail_counter + 1;
//                                     }
//                                 }
//                                 "lock" => {
//                                     if response.success {
//                                         log::info!(target: "gantry_interface", "Requested lock succeeded.");
//                                         request_state = ServiceRequestState::Succeeded.to_string();
//                                         gantry_locked_estimated = true;
//                                         subsequent_fail_counter = 0;
//                                     } else {
//                                         log::error!(target: "gantry_interface", "Requested lock failed.");
//                                         request_state = ServiceRequestState::Failed.to_string();
//                                         subsequent_fail_counter = subsequent_fail_counter + 1;
//                                         total_fail_counter = total_fail_counter + 1;
//                                     }
//                                 }
//                                 "unlock" => {
//                                     if response.success {
//                                         log::info!(target:
//                                             "gantry_interface",
//                                             "Requested unlock succeeded."
//                                         );
//                                         request_state = ServiceRequestState::Succeeded.to_string();
//                                         gantry_locked_estimated = false;
//                                         subsequent_fail_counter = 0;
//                                     } else {
//                                         log::error!(target: "gantry_interface", "Requested unlock failed.");
//                                         request_state = ServiceRequestState::Failed.to_string();
//                                         subsequent_fail_counter = subsequent_fail_counter + 1;
//                                         total_fail_counter = total_fail_counter + 1;
//                                     }
//                                 }
//                                 _ => {
//                                     log::error!(target:
//                                         "gantry_interface",
//                                         "Requested command '{}' is invalid.",
//                                         gantry_command_command
//                                     );
//                                     request_state = ServiceRequestState::Failed.to_string();
//                                     subsequent_fail_counter = subsequent_fail_counter + 1;
//                                     total_fail_counter = total_fail_counter + 1;
//                                 }
//                             },
//                             Err(e) => {
//                                 log::error!(target: "gantry_interface", "Request failed with: {e}.");
//                                 request_state = ServiceRequestState::Failed.to_string();
//                                 subsequent_fail_counter = subsequent_fail_counter + 1;
//                                 total_fail_counter = total_fail_counter + 1;
//                             }
//                         },
//                         Err(e) => {
//                             log::error!(target: "gantry_interface", "Request failed with: {e}.");
//                             request_state = ServiceRequestState::Failed.to_string();
//                             subsequent_fail_counter = subsequent_fail_counter + 1;
//                             total_fail_counter = total_fail_counter + 1;
//                         }
//                     };
//                 }
//             }
//             let new_state = state
//                 .update("gantry_request_trigger", request_trigger.to_spvalue())
//                 .update("gantry_request_state", request_state.to_spvalue())
//                 .update("gantry_total_fail_counter", total_fail_counter.to_spvalue())
//                 .update(
//                     "gantry_subsequent_fail_counter",
//                     subsequent_fail_counter.to_spvalue(),
//                 )
//                 .update(
//                     "gantry_position_estimated",
//                     gantry_position_estimated.to_spvalue(),
//                 )
//                 .update(
//                     "gantry_calibrated_estimated",
//                     gantry_calibrated_estimated.to_spvalue(),
//                 )
//                 .update(
//                     "gantry_locked_estimated",
//                     gantry_locked_estimated.to_spvalue(),
//                 );

//             let modified_state = state.get_diff_partial_state(&new_state);
//             redis_set_state(&mut con, modified_state).await;
//         }
//         timer.tick().await?;
//     }
// }
