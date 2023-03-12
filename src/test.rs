use micro_sp::*;
use r2r::micro_sp_emulation_msgs::srv::SetState;
use rand::seq::SliceRandom;

use serde_json::Value;
use std::collections::HashMap;

pub static NODE_ID: &'static str = "micro_sp_test";
pub static TICKER_RATE: u64 = 100;
pub static NR_TEST_CASES: u64 = 10;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let client = node.create_client::<SetState::Service>("/set_state")?;
    let waiting_for_server = node.is_available(&client)?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!(NODE_ID, "Waiting for micro_sp to become available...");
    waiting_for_server.await?;
    r2r::log_info!(NODE_ID, "micro_sp available.");

    tokio::task::spawn(async move {
        let result = test_process(&client).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Set State Service call succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Set State Service call failed with: '{}'.", e),
        };
    });

    handle.join().unwrap();

    r2r::log_warn!(NODE_ID, "Node started.");

    Ok(())
}

async fn test_process(
    client: &r2r::Client<SetState::Service>,
) -> Result<(), Box<dyn std::error::Error>> {
    Ok(for _ in 0..NR_TEST_CASES {
        let state = HashMap::from([
            (
                "gripper_actual_state".to_string(),
                vec!["opened", "closed", "gripping", "unknown"]
                    .choose(&mut rand::thread_rng())
                    .unwrap()
                    .to_spvalue(),
            ),
            (
                "scanned_a".to_string(),
                vec![true, false]
                    .choose(&mut rand::thread_rng())
                    .unwrap()
                    .to_spvalue(),
            ),
        ]);
        println!("state: {:?}", state);
        let _res = client_callback(client, state).await;
    })
}

async fn client_callback(
    client: &r2r::Client<SetState::Service>,
    state: HashMap<String, SPValue>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut map = serde_json::Map::<String, Value>::new();
    state.iter().for_each(|(k, v)| {
        let _ = map.insert(k.to_string(), match v {
            SPValue::Array(_, val) => Value::from(format!("array__{:?}", val)),
            SPValue::Bool(val) => Value::from(format!("bool___{}", val)),
            SPValue::Float64(val) => Value::from(format!("float__{}", val)),
            SPValue::String(val) => Value::from(format!("string_{}", val)),
            SPValue::Int32(val) => Value::from(format!("int____{}", val)),
            SPValue::Time(val) => Value::from(format!("time___{:?}", val)),
            SPValue::Unknown => Value::from(format!("unknown")),
        });
    });

    let state = serde_json::Value::Object(map).to_string();

    let request = SetState::Request { state };

    let response = client
        .request(&request)
        .expect("Could not send Set State request.")
        .await
        .expect("Cancelled.");

    r2r::log_info!(NODE_ID, "Request to set state sent.");

    match response.success {
        true => r2r::log_info!(NODE_ID, "Succesfully set state."),
        false => {
            r2r::log_error!(NODE_ID, "Failed to set state.",);
        }
    }
    Ok(())
}
