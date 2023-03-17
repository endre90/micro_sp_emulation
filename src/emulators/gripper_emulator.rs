use futures::{Stream, StreamExt};
use r2r::micro_sp_emulation_msgs::srv::TriggerGripper;
use r2r::ServiceRequest;
use std::error::Error;
use rand::Rng;

pub static NODE_ID: &'static str = "gripper_emulator";

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let gripper_service = node.create_service::<TriggerGripper::Service>("gripper_service")?;

    tokio::task::spawn(async move {
        let result = gripper_server(gripper_service).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Gripper service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Gripper service call failed with: '{}'.", e),
        };
    });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(20));
    });

    r2r::log_warn!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

pub async fn gripper_server(
    mut service: impl Stream<Item = ServiceRequest<TriggerGripper::Service>> + Unpin,
) -> Result<(), Box<dyn std::error::Error>> {


    loop {
        match service.next().await {
            Some(request) => {

                let delay: u64 = {
                    let mut rng = rand::thread_rng();
                    rng.gen_range(0..3000)
                };

                // simulate random task execution time
                tokio::time::sleep(std::time::Duration::from_millis(delay)).await;

                match request.message.command.as_str() {
                    "open" => {
                        r2r::log_info!(NODE_ID, "Got request to open.");
                        // Adversary: 50/50 that we succeed or abort
                        if rand::random::<bool>() {
                            let response = TriggerGripper::Response {
                                state: "opened".to_string(),
                                success: true,
                                info: "Opening succeeded.".to_string(),
                            };
                            r2r::log_warn!(NODE_ID, "Opening succeeded.");
                            request
                                .respond(response)
                                .expect("Could not send service response.");
                            continue;
                        } else {
                            let response = TriggerGripper::Response {
                                state: "unknown".to_string(),
                                success: false,
                                info: "Opening failed.".to_string(),
                            };
                            r2r::log_error!(NODE_ID, "Opening failed.");
                            request
                                .respond(response)
                                .expect("Could not send service response.");
                            continue;
                        }
                    }


                    "close" => {
                        r2r::log_info!(NODE_ID, "Got request to close.");
                        // Adversary: 50/50 that we succeed or abort
                        let response = if rand::random::<bool>() {
                            // Adversary: 50/50 that we grip or close completely (i.e. fail gripping)
                            if rand::random::<bool>() {
                                r2r::log_warn!(NODE_ID, "Closing succeeded - gripping.");
                                TriggerGripper::Response {
                                    state: "gripping".to_string(),
                                    success: true,
                                    info: "Closing succeeded - gripping.".to_string(),
                                }
                            } else {
                                r2r::log_warn!(NODE_ID, "Closing succeeded - closed.");
                                TriggerGripper::Response {
                                    state: "closed".to_string(),
                                    success: true,
                                    info: "Closing succeeded - closed.".to_string(),
                                }
                            }
                        } else {
                            r2r::log_error!(NODE_ID, "Closing failed.");
                            TriggerGripper::Response {
                                state: "unknown".to_string(),
                                success: false,
                                info: "Closing failed.".to_string(),
                            }
                        };
                        request
                            .respond(response)
                            .expect("Could not send service response.");
                        continue;
                    }
                    _ => {
                        let response = TriggerGripper::Response {
                            state: "unknown".to_string(),
                            success: false,
                            info: "Unknown command.".to_string(),
                        };
                        request
                            .respond(response)
                            .expect("Could not send service response.");
                        continue;
                    }
                }
            }

            None => (),
        }
    }
}
