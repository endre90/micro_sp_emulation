use futures::{Stream, StreamExt};
use micro_sp::*;
use ordered_float::OrderedFloat;
use r2r::{micro_sp_emulation_msgs::srv::SetState, QosProfile};
use r2r::std_msgs::msg::String as StringMsg;
use rand::seq::SliceRandom;
use std::sync::{Arc, Mutex};

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

    let state_listener = node.subscribe::<StringMsg>("state", QosProfile::default())?;
    let shared_state = Arc::new(Mutex::new(HashMap::new()));   

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!(NODE_ID, "Waiting for micro_sp to become available...");
    waiting_for_server.await?;
    r2r::log_info!(NODE_ID, "micro_sp available.");

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        match state_listener_callback(state_listener, &shared_state_clone, NODE_ID)
            .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Extra tf listener failed with: '{}'.", e),
        };
    });

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

pub async fn state_listener_callback(
    mut subscriber: impl Stream<Item = StringMsg> + Unpin,
    shared_state: &Arc<Mutex<HashMap<String, SPValue>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let state_change = match serde_json::from_str(&message.data) {
                    Ok::<HashMap<String, String>, _>(map) => map,
                    _ => HashMap::new(),
                };

                let mut shsl = shared_state.lock().unwrap().clone();
                state_change.iter().for_each(|(k, v)| {
                    let mut value = v.clone();
                    let to_update_prim = value.split_off(7);
                    let to_update = match value.as_str() {
                        "array__" => to_update_prim.to_spvalue(),
                        "bool___" => to_update_prim.parse::<bool>().unwrap_or_default().to_spvalue(),
                        "float__" => to_update_prim.parse::<OrderedFloat<f64>>().unwrap_or_default().to_spvalue(),
                        "string_" => to_update_prim.to_spvalue(),
                        "int____" => to_update_prim.parse::<i32>().unwrap_or_default().to_spvalue(),
                        // "time___" => SPValue::Time(SystemTime::now()),
                        "unknown" => SPValue::Unknown,
                        _ => panic!("can't parse that... {:?}", value)
                    };
                    shsl.insert(k.to_owned(), to_update);
                });
                *shared_state.lock().unwrap() = shsl;
            }
            None => {
                r2r::log_error!(node_id, "Subscriber did not get the message?");
            }
        }
    }
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
