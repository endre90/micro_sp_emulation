use futures::{Stream, StreamExt, Future};
use micro_sp::SPValue;
use r2r::{micro_sp_emulation_msgs::action::URCommand, Error};
use micro_sp::*;
use std::sync::{Arc, Mutex};

pub async fn robot_action_ticker(
    robot_action_client: &r2r::ActionClient<URCommand::Action>,
    wait_for_server: impl Future<Output = Result<(), Error>>,
    shared_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_warn!(node_id, "Waiting for the robot action server...");
    wait_for_server.await?;
    r2r::log_info!(node_id, "Robot action server available.");
    loop {
        let shared_state_local = shared_state.lock().unwrap().clone();
        let ur_action_trigger = match shared_state_local.get_value("ur_action_trigger") {
            SPValue::Bool(value) => value,
            _ => false
        };
        // let ur_action_state = match shared_state_local.get_value("ur_action_state") {
        //     SPValue::String(value) => value,
        //     _ => "".to_string()
        // };

        if ur_action_trigger {
            match shared_state_local.get_value("ur_command") {
                SPValue::String(ur_command) => {
                    let goal = URCommand::Goal {
                        ur_command,
                        ur_goal_frame: shared_state_local.get_value("ur_goal_frame").to_string(),
                        ur_tcp: shared_state_local.get_value("ur_tcp").to_string(),
                    };
                    let (_, result, _) = match robot_action_client.send_goal_request(goal) {
                        Ok(x) => match x.await {
                            Ok(y) => y,
                            Err(e) => {
                                r2r::log_info!(node_id, "Could not send goal request.");
                                return Err(Box::new(e));
                            }
                        },
                        Err(e) => {
                            r2r::log_info!(node_id, "Did not get goal.");
                            return Err(Box::new(e));
                        }
                    };
                    match result.await {
                        Ok((status, _)) => match status {
                            r2r::GoalStatus::Aborted => {
                                r2r::log_info!(node_id, "Robot action goal state: aborted.");
                                let shared_state_local =
                                    shared_state_local.update("ur_action_state", "aborted".to_spvalue());
                                    *shared_state.lock().unwrap() = shared_state_local;
                            }
                            r2r::GoalStatus::Accepted => {
                                r2r::log_info!(node_id, "Robot action goal state: accepted.");
                                let shared_state_local =
                                    shared_state_local.update("ur_action_state", "accepted".to_spvalue());
                                    *shared_state.lock().unwrap() = shared_state_local;
                            }
                            r2r::GoalStatus::Canceled => {
                                r2r::log_info!(node_id, "Robot action goal state: cancelled.");
                                let shared_state_local =
                                    shared_state_local.update("ur_action_state", "cancelled".to_spvalue());
                                    *shared_state.lock().unwrap() = shared_state_local;
                            }
                            r2r::GoalStatus::Canceling => {
                                r2r::log_info!(node_id, "Robot action goal state: cancelling.");
                                let shared_state_local =
                                    shared_state_local.update("ur_action_state", "cancelling".to_spvalue());
                                    *shared_state.lock().unwrap() = shared_state_local;
                            }
                            r2r::GoalStatus::Executing => {
                                r2r::log_info!(node_id, "Robot action goal state: executing.");
                                let shared_state_local =
                                    shared_state_local.update("ur_action_state", "executing".to_spvalue());
                                    *shared_state.lock().unwrap() = shared_state_local;
                            }
                            r2r::GoalStatus::Succeeded => {
                                r2r::log_info!(node_id, "Robot action goal state: succeeded.");
                                let shared_state_local =
                                    shared_state_local.update("ur_action_state", "succeeded".to_spvalue());
                                    *shared_state.lock().unwrap() = shared_state_local;
                            }
                            r2r::GoalStatus::Unknown => {
                                r2r::log_info!(node_id, "Robot action goal state: unknown.");
                                let shared_state_local =
                                    shared_state_local.update("ur_action_state", "unknown".to_spvalue());
                                    *shared_state.lock().unwrap() = shared_state_local;
                            }
                        },
                        Err(e) => {
                            r2r::log_error!(node_id, "Robot action failed with: {:?}", e,);
                            r2r::log_error!(node_id, "Robot action goal state: failed.");
                            let shared_state_local =
                                shared_state_local.update("ur_action_state", "failed".to_spvalue());
                                *shared_state.lock().unwrap() = shared_state_local;
                        }
                    }
                },
                _ => ()
            };
        } else {
            r2r::log_info!(node_id, "Robot action goal state: initial.");
            let shared_state_local =
                shared_state_local.update("ur_action_state", "initial".to_spvalue());
            *shared_state.lock().unwrap() = shared_state_local;
        }
        timer.tick().await?;
    }
}