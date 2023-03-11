use futures::{Future, Stream, StreamExt};
use micro_sp::SPValue;
use micro_sp::*;
use r2r::{micro_sp_emulation_msgs::action::URCommand, uuid::Uuid, ActionClientGoal, Error};
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
    // let mut goal_handle: ActionClientGoal<URCommand::Action> = ActionClientGoal::
    let mut goal_handle_status = None;
    loop {
        println!("GOAL HANDLE STATUS: {:?}", goal_handle_status);
        let shared_state_local = shared_state.lock().unwrap().clone();
        let ur_action_trigger = match shared_state_local.get_value("ur_action_trigger") {
            SPValue::Bool(value) => value,
            _ => false,
        };

        if !ur_action_trigger && goal_handle_status != None {
            goal_handle_status = None;
            let shared_state_local = shared_state_local
                .update("ur_action_state", "initial".to_spvalue());
            *shared_state.lock().unwrap() = shared_state_local;
        } else if !ur_action_trigger {
            let shared_state_local = shared_state_local
                .update("ur_action_state", "initial".to_spvalue());
            *shared_state.lock().unwrap() = shared_state_local;
        } else if ur_action_trigger && goal_handle_status == None {
            match shared_state_local.get_value("ur_command") {
                SPValue::String(ur_command) => {
                    let goal = URCommand::Goal {
                        ur_command,
                        ur_goal_frame: shared_state_local.get_value("ur_goal_frame").to_string(),
                        ur_tcp: shared_state_local.get_value("ur_tcp").to_string(),
                    };
                    let (goal_handle, result, feedback) =
                        match robot_action_client.send_goal_request(goal) {
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

                    goal_handle_status = match goal_handle.get_status() {
                        Ok(status) => match status {
                            r2r::GoalStatus::Accepted => {
                                let shared_state_local = shared_state_local
                                    .update("ur_action_state", "executing".to_spvalue());
                                *shared_state.lock().unwrap() = shared_state_local;
                                Some("accepted")
                            },
                            _ => {
                                let shared_state_local = shared_state_local
                                    .update("ur_action_state", "failed".to_spvalue());
                                *shared_state.lock().unwrap() = shared_state_local;
                                None
                            }
                        }
                        Err(_) => None
                    };

                    match result.await {
                        Ok((status, msg)) => match status {
                            r2r::GoalStatus::Aborted => {
                                r2r::log_info!(node_id, "Goal succesfully aborted with: {:?}", msg);
                                // let _ = g.publish_feedback(URCommand::Feedback {
                                //     current_state: "Goal succesfully aborted.".into(),
                                // });
                                // Ok(())
                            }
                            _ => {
                                r2r::log_info!(
                                    node_id,
                                    "Executing the Simple Robot Simulator action succeeded."
                                );
                                let shared_state_local = shared_state_local
                                    .update("ur_action_state", "succeeded".to_spvalue());
                                *shared_state.lock().unwrap() = shared_state_local;
                                // let _ = g.publish_feedback(URCommand::Feedback {
                                //     current_state: "Executing the Simple Robot Simulator action succeeded.".into(),
                                // });
                                // Ok(())
                            }
                        },
                        Err(e) => {
                            // r2r::log_error!(
                            //     node_id,
                            //     "Simple Robot Simulator action failed with: {:?}",
                            //     e,
                            // );
                            // let _ = g.publish_feedback(URCommand::Feedback {
                            //     current_state: "Simple Robot Simulator action failed. Aborting.".into(),
                            // });
                            // return Err(Box::new(e));
                        }
                    }

                    // match result.await {
                    //     Ok((status, _)) => match status {
                    //         r2r::GoalStatus::Aborted => {
                    //             // r2r::log_info!(node_id, "Robot action goal state: aborted.");
                    //             let shared_state_local = shared_state_local
                    //                 .update("ur_action_state", "aborted".to_spvalue());
                    //             *shared_state.lock().unwrap() = shared_state_local;
                    //         }
                    //         r2r::GoalStatus::Accepted => {
                    //             r2r::log_info!(node_id, "Robot action goal state: accepted.");
                    //             let shared_state_local = shared_state_local
                    //                 .update("ur_action_state", "accepted".to_spvalue());
                    //             *shared_state.lock().unwrap() = shared_state_local;
                    //             // "accepted"
                    //         }
                    //         r2r::GoalStatus::Canceled => {
                    //             // r2r::log_info!(node_id, "Robot action goal state: cancelled.");
                    //             let shared_state_local = shared_state_local
                    //                 .update("ur_action_state", "cancelled".to_spvalue());
                    //             *shared_state.lock().unwrap() = shared_state_local;
                    //             // "cancelled"
                    //         }
                    //         r2r::GoalStatus::Canceling => {
                    //             // r2r::log_info!(node_id, "Robot action goal state: cancelling.");
                    //             let shared_state_local = shared_state_local
                    //                 .update("ur_action_state", "cancelling".to_spvalue());
                    //             *shared_state.lock().unwrap() = shared_state_local;
                    //             // "cancelling"
                    //         }
                    //         r2r::GoalStatus::Executing => {
                    //             // r2r::log_info!(node_id, "Robot action goal state: executing.");
                    //             let shared_state_local = shared_state_local
                    //                 .update("ur_action_state", "executing".to_spvalue());
                    //             *shared_state.lock().unwrap() = shared_state_local;
                    //             // "executing"
                    //         }
                    //         r2r::GoalStatus::Succeeded => {
                    //             // r2r::log_info!(node_id, "Robot action goal state: succeeded.");
                    //             let shared_state_local = shared_state_local
                    //                 .update("ur_action_state", "succeeded".to_spvalue());
                    //             *shared_state.lock().unwrap() = shared_state_local;
                    //             // "succeeded"
                    //         }
                    //         r2r::GoalStatus::Unknown => {
                    //             // r2r::log_info!(node_id, "Robot action goal state: unknown.");
                    //             let shared_state_local = shared_state_local
                    //                 .update("ur_action_state", "unknown".to_spvalue());
                    //             *shared_state.lock().unwrap() = shared_state_local;
                    //             // "unknown"
                    //         }
                    //     },
                    //     Err(e) => {
                    //         r2r::log_error!(node_id, "Robot action failed with: {:?}", e,);
                    //         // r2r::log_error!(node_id, "Robot action goal state: failed.");
                    //         let shared_state_local =
                    //             shared_state_local.update("ur_action_state", "failed".to_spvalue());
                    //         *shared_state.lock().unwrap() = shared_state_local;
                    //     }
                    // }
                }
                _ => (),
            };
        } else {
            // r2r::log_info!(node_id, "Robot action goal state: initial.");
            let shared_state_local =
                shared_state_local.update("ur_action_state", "initial".to_spvalue());
            *shared_state.lock().unwrap() = shared_state_local;
        }
        timer.tick().await?;
    }
}
