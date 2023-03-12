use futures::{Stream, StreamExt};
use micro_sp::{State, SPValue, ToSPValue};
use ordered_float::OrderedFloat;
use r2r::micro_sp_emulation_msgs::srv::SetState;
use r2r::ServiceRequest;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

pub async fn set_state_server(
    mut service: impl Stream<Item = ServiceRequest<SetState::Service>> + Unpin,
    shared_state: &Arc<Mutex<State>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match service.next().await {
            Some(request) => {
                r2r::log_warn!(node_id, "Got request to set state.");
                let state_change = match serde_json::from_str(&request.message.state) {
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
                    shsl = shsl.update(&k, to_update)
                });

                *shared_state.lock().unwrap() = shsl;

                // Here I have to wait until the test is completed, i.e. 
                // runner_plan_state is one of done, failed or aborted

                let response = SetState::Response {
                    success: true,
                    info: "State change succeeded.".to_string(),
                };
                r2r::log_warn!(node_id, "State change succeeded.");
                request
                    .respond(response)
                    .expect("Could not send service response.");
                continue;
            }

            None => (),
        }
    }
}
