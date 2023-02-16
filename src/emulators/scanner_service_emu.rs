use futures::{Stream, StreamExt};
use micro_sp::SPValue;
use r2r::micro_sp_emulation_msgs::srv::TriggerScan;

use micro_sp::*;
use r2r::{QosProfile, ServiceRequest};
use std::error::Error;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "scanner_service_emu";
// pub static PUBLISHER_RATE: u64 = 1000;
// pub static STATE_UPDATE_RATE: u64 = 1000;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let scanner_service =
        node.create_service::<TriggerScan::Service>("scanner_service")?;

    tokio::task::spawn(async move {
        let result = scanner_server(
            scanner_service,
        )
        .await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Scanner service call succeeded."),
            Err(e) => r2r::log_error!(
                NODE_ID,
                "Scanner service call failed with: '{}'.",
                e
            ),
        };
    });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));
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
                println!("Got request: {:?}", request.message);
                let response = TriggerScan::Response { 
                    success: true, 
                    info: "just succeeding...".to_string()
                };
                println!("just succeeding...");
                request
                    .respond(response)
                    .expect("Could not send service response.");
                continue;
                }
            
            None => (),
        }
    }
}