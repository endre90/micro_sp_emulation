use futures::{Stream, StreamExt};
use r2r::micro_sp_emulation_msgs::srv::TriggerScan;
use r2r::ServiceRequest;
use std::error::Error;
use rand::Rng;

pub static NODE_ID: &'static str = "scanner_emulator";

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let scanner_service = node.create_service::<TriggerScan::Service>("scanner_service")?;

    tokio::task::spawn(async move {
        let result = scanner_server(scanner_service).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Scanner service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Scanner service call failed with: '{}'.", e),
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

pub async fn scanner_server(
    mut service: impl Stream<Item = ServiceRequest<TriggerScan::Service>> + Unpin,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => {
                r2r::log_info!(NODE_ID, "Got request to scan.");
                r2r::log_debug!(NODE_ID, "Got request to scan: {:?}", request.message);

                let delay: u64 = {
                    let mut rng = rand::thread_rng();
                    rng.gen_range(0..3000)
                };
                
                // simulate random task execution time
                tokio::time::sleep(std::time::Duration::from_millis(delay)).await;
            
                // Adversary: 50/50 that we succeed or abort
                if rand::random::<bool>() {
                    let response = TriggerScan::Response {
                        success: true,
                        info: "Scanning succeeded.".to_string(),
                    };
                    r2r::log_warn!(NODE_ID, "Scanning succeeded.");
                    request
                        .respond(response)
                        .expect("Could not send service response.");
                    continue;
                } else {
                    let response = TriggerScan::Response {
                        success: false,
                        info: "Scanning failed.".to_string(),
                    };
                    r2r::log_error!(NODE_ID, "Scanning failed.");
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
