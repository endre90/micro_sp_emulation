use futures::{Stream, StreamExt};
use r2r::micro_sp_emulation_msgs::srv::TriggerRobot;
use r2r::ServiceRequest;
use rand::Rng;
use std::error::Error;

pub static NODE_ID: &'static str = "robot_emulator";

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let robot_service = node.create_service::<TriggerRobot::Service>("robot_service")?;

    tokio::task::spawn(async move {
        let result = robot_server(robot_service).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "robot service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "robot service call failed with: '{}'.", e),
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

pub async fn robot_server(
    mut service: impl Stream<Item = ServiceRequest<TriggerRobot::Service>> + Unpin,
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
                    "mount" | "unmount" => {
                        r2r::log_info!(NODE_ID, "Got request to {}.", request.message.command);
                        let cmd = request.message.command.clone();
                        // Adversary: 50/50 that we succeed or abort
                        if rand::random::<bool>() {
                            let response = TriggerRobot::Response {
                                state: "blah".to_string(),
                                success: true,
                                info: format!("{cmd} succeeded.").to_string(),
                            };
                            r2r::log_warn!(NODE_ID, "{}", &format!("{cmd} succeeded."));
                            request
                                .respond(response)
                                .expect("Could not send service response.");
                            continue;
                        } else {
                            let response = TriggerRobot::Response {
                                state: "blah".to_string(),
                                success: false,
                                info: format!("{cmd} failed.").to_string(),
                            };
                            r2r::log_error!(NODE_ID, "{}", format!("{cmd} failed."));
                            request
                                .respond(response)
                                .expect("Could not send service response.");
                            continue;
                        }
                    }
                    "move" => {
                        r2r::log_info!(
                            NODE_ID,
                            "Got request to move to {}.",
                            request.message.position
                        );
                        let pos = request.message.position.clone();
                        // Adversary: 50/50 that we succeed or abort
                        if rand::random::<bool>() {
                            let response = TriggerRobot::Response {
                                state: "blah".to_string(),
                                success: true,
                                info: format!("Moving to {pos} succeeded.").to_string(),
                            };
                            r2r::log_warn!(NODE_ID, "{}", format!("Moving to {pos} succeeded."));
                            request
                                .respond(response)
                                .expect("Could not send service response.");
                            continue;
                        } else {
                            let response = TriggerRobot::Response {
                                state: "blah".to_string(),
                                success: false,
                                info: format!("Moving to {pos} failed.").to_string(),
                            };
                            r2r::log_error!(NODE_ID, "{}", format!("Moving to {pos} failed."));
                            request
                                .respond(response)
                                .expect("Could not send service response.");
                            continue;
                        }
                    }
                    _ => (),
                }
            }

            None => (),
        }
    }
}
