use crate::*;
use micro_sp::*;
use micro_sp::SPValue;
use r2r::{std_msgs::msg::String as StringMsg, QosProfile};
use serde_json::Value;
use std::sync::{Arc, Mutex};
use tokio::sync::{mpsc, oneshot};

pub async fn spawn_state_publisher(
    arc_node: Arc<Mutex<r2r::Node>>,
    command_sender: mpsc::Sender<StateManagement>,
) -> Result<(), Box<dyn std::error::Error>> {
    let publisher = arc_node
        .lock()
        .unwrap()
        .create_publisher::<StringMsg>("/state", QosProfile::default())?;

    let mut timer = arc_node
        .lock()
        .unwrap()
        .create_wall_timer(std::time::Duration::from_millis(PUBLISHER_TICKER_RATE))?;

    r2r::log_info!("state_publisher", "Spawned.");

    loop {
        let (response_tx, response_rx) = oneshot::channel();
        command_sender
            .send(StateManagement::GetState(response_tx))
            .await?;
        let state = response_rx.await?;
        let mut map = serde_json::Map::new();
        state.state.iter().for_each(|(k, v)| {
            let _ = map.insert(
                k.to_string(),
                match &v.val {
                    SPValue::Array(a) => Value::from(match a {
                        ArrayOrUnknown::Array(a_val) => {
                            let items_str = a_val
                                .iter()
                                .map(|item| item.to_string())
                                .collect::<Vec<_>>()
                                .join(", ");
                            format!("[{}]", items_str)
                        }
                        ArrayOrUnknown::UNKNOWN => "UNKNOWN".to_string(),
                    }),
                    SPValue::Bool(b) => Value::from(match b {
                        BoolOrUnknown::Bool(b_val) => format!("{}", b_val),
                        BoolOrUnknown::UNKNOWN => "UNKNOWN".to_string(),
                    }),
                    SPValue::Float64(f) => Value::from(match f {
                        FloatOrUnknown::Float64(f_val) => format!("{}", f_val.into_inner()),
                        FloatOrUnknown::UNKNOWN => "UNKNOWN".to_string(),
                    }),
                    SPValue::String(s) => Value::from(match s {
                        StringOrUnknown::String(s_val) => format!("{}", s_val),
                        StringOrUnknown::UNKNOWN => "UNKNOWN".to_string(),
                    }),
                    SPValue::Int64(i) => Value::from(match i {
                        IntOrUnknown::Int64(i_val) => format!("{}", i_val),
                        IntOrUnknown::UNKNOWN => "UNKNOWN".to_string(),
                    }),
                    SPValue::Time(t) => Value::from(match t {
                        TimeOrUnknown::Time(t_val) => format!("{:?}", t_val.elapsed().unwrap_or_default()),
                        TimeOrUnknown::UNKNOWN => "UNKNOWN".to_string(),
                    }),
                    SPValue::Map(_) => Value::from("map_hack".to_string()),
                    SPValue::Transform(_) => Value::from("transform_hack".to_string()),
                },
            );
        });

        let state = serde_json::Value::Object(map).to_string();
        let state_msg = StringMsg { data: state };

        match publisher.publish(&state_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!("state_publisher", "Failed to send a message with: '{}'", e);
            }
        };
        timer.tick().await?;
    }
}

// pub async fn state_publisher(
//     publisher: r2r::Publisher<StringMsg>,
//     mut timer: r2r::Timer,
//     shared_state: &Arc<(Mutex<State>, Vec<AtomicUsize>)>,//HashMap<String, AtomicUsize>)>,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     loop {
//         let shsl = shared_state.0.lock().unwrap().clone();
//         let mut map = serde_json::Map::new();
//         shsl.state.iter().for_each(|(k, v)| {
//             let _ = map.insert(k.to_string(), match &v.val {
//                 SPValue::Array(_, val) => Value::from(format!("{:?}", val)),
//                 SPValue::Bool(val) => Value::from(format!("{}", val)),
//                 SPValue::Float64(val) => Value::from(format!("{}", val)),
//                 SPValue::String(val) => Value::from(format!("{}", val)),
//                 SPValue::Int64(val) => Value::from(format!("{}", val)),
//                 SPValue::Time(val) => Value::from(format!("{:?}", val)),
//                 SPValue::UNKNOWN => Value::from(format!("UNKNOWN")),
//             });
//         });

//         let state = serde_json::Value::Object(map).to_string();
//         let state_msg = StringMsg {
//             data: state
//         };

//         match publisher.publish(&state_msg) {
//             Ok(()) => (),
//             Err(e) => {
//                 r2r::log_error!("state_publisher", "Failed to send a message with: '{}'", e);
//             }
//         };
//         timer.tick().await?;
//     }
// }
