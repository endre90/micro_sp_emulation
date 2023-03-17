use futures::{Stream, StreamExt};
use r2r::micro_sp_emulation_msgs::srv::TriggerGantry;
use r2r::ServiceRequest;
use std::error::Error;
use rand::Rng;

pub static NODE_ID: &'static str = "gantry_emulator";

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let gantry_service = node.create_service::<TriggerGantry::Service>("gantry_service")?;

    tokio::task::spawn(async move {
        let result = gantry_server(gantry_service).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "gantry service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "gantry service call failed with: '{}'.", e),
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

pub async fn gantry_server(
    mut service: impl Stream<Item = ServiceRequest<TriggerGantry::Service>> + Unpin,
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

                r2r::log_info!(NODE_ID, "Got request to go to {}.", request.message.position );
                // Adversary: 50/50 that we succeed or abort
                if rand::random::<bool>() {
                    let response = TriggerGantry::Response {
                        state: request.message.position.to_string(),
                        success: true,
                        info: "Moving succeeded.".to_string(),
                    };
                    r2r::log_warn!(NODE_ID, "Moving succeeded.");
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                } else {
                    let response = TriggerGantry::Response {
                        state: "unknown".to_string(),
                        success: false,
                        info: "Moving failed.".to_string(),
                    };
                    r2r::log_error!(NODE_ID, "Moving failed.");
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                }
            }

            None => (),
        }
    }
}
