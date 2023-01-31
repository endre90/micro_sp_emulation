pub static NODE_ID: &'static str = "ur_resource_emulator";

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let action = node.create_action_server::<URControlEmulator::Action>("ur_control_emulator")?;

    tokio::task::spawn(async move {
        let result = ur_resource_emulator_action_server(action).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "ur_resource_emulator action call succeeded."),
            Err(e) => {
                r2r::log_error!(
                    NODE_ID,
                    "ur_resource_emulator action call failed with: {}.",
                    e
                )
            }
        };
    });

    handle.join().unwrap();

    r2r::log_warn!(NODE_ID, "Node started.");

    Ok(())
}

// Based on the received message, this function looks into the model to find which operation
// is currently being executed. It does so by comparing the values from precondition actions
// with the values in the received message. When a match was found, this responds with values
// from the matched operation's postcondition guard. 
// TODO: maybe introduce probabilities, failed actions, wrong responses, and delays.
async fn ur_resource_emulator_action_server(
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<URControl::Action>> + Unpin,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match requests.next().await {
            Some(request) => {
                let (mut g, mut _cancel) =
                    request.accept().expect("Could not accept goal request.");
                let g_clone = g.clone();
                match execute_urscript(g_clone, &urc_client, &tf_lookup_client, &templates).await {
                    Ok(ok) => {
                        g.succeed(URControl::Result { success: ok })
                            .expect("Could not send result.");
                        continue;
                    }
                    Err(e) => {
                        let _ = g.abort(URControl::Result { success: false });
                        continue;
                    }
                }
            }
            None => (),
        }
    }
}
