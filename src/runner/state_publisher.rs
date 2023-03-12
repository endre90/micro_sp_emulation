use micro_sp::{State, SPValue};
use r2r::std_msgs::msg::String;
use serde_json::Value;
use std::sync::{Arc, Mutex};

pub async fn state_publisher_callback(
    publisher: r2r::Publisher<String>,
    mut timer: r2r::Timer,
    shared_state: &Arc<Mutex<State>>,
    node_id: &str
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let shsl = shared_state.lock().unwrap().clone();
        let mut map = serde_json::Map::new();
        shsl.state.iter().for_each(|(k, v)| {
            let _ = map.insert(k.to_string(), match &v.val {
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
        let state_msg = String {
            data: state
        };

        match publisher.publish(&state_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(node_id, "Publisher failed to send a message with: '{}'", e);
            }
        };
        timer.tick().await?;
    }
}