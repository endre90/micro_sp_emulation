use futures::{Stream, StreamExt};
use micro_sp::*;
use ordered_float::OrderedFloat;
use r2r::std_msgs::msg::String as StringMsg;
use r2r::{micro_sp_emulation_msgs::srv::SetState, QosProfile};
use rand::seq::SliceRandom;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

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
        match state_listener_callback(state_listener, &shared_state_clone, NODE_ID).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "State listener failed with: '{}'.", e),
        };
    });

    let shared_state_clone = shared_state.clone();
    tokio::task::spawn(async move {
        let result = some_random_test_process(&client, &shared_state_clone).await;
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
                        "bool___" => to_update_prim
                            .parse::<bool>()
                            .unwrap_or_default()
                            .to_spvalue(),
                        "float__" => to_update_prim
                            .parse::<OrderedFloat<f64>>()
                            .unwrap_or_default()
                            .to_spvalue(),
                        "string_" => to_update_prim.to_spvalue(),
                        "int____" => to_update_prim
                            .parse::<i32>()
                            .unwrap_or_default()
                            .to_spvalue(),
                        // "time___" => SPValue::Time(SystemTime::now()),
                        "unknown" => SPValue::Unknown,
                        _ => panic!("can't parse that... {:?}", value),
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

// only perform test when plan state is failed, done, or aborted
async fn some_random_test_process(
    client: &r2r::Client<SetState::Service>,
    shared_state: &Arc<Mutex<HashMap<String, SPValue>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut test_case = 0;
    let now = Instant::now();
    while test_case < NR_TEST_CASES {
        tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
        let shsl = shared_state.lock().unwrap().clone();
        match shsl.get("runner_plan_state") {
            Some(runner_plan_state) => {
                match runner_plan_state {
                    SPValue::String(plan_state) => {
                        if plan_state == "failed" || plan_state == "done" || plan_state == "aborted"
                        {
                            test_case = test_case + 1;
                            let state = HashMap::from([
                            (
                                "runner_goal".to_string(),
                                vec![
                                    "var:scanned_a == true", 
                                    // "var:scanned_b == true", 
                                    // "var:scanned_a == true && var:scanned_b == true", 
                                    "var:gripper_actual_state == closed && var:robot_actual_state == agv",
                                    // aha ok, then unmount gripper will not be here, so we'll add it later
                                    // "var:mounted == scanner && var:item_a_pose == gripper",
                                    "var:item_a_pose == agv && var:gripper_actual_state == box_a",
                                    // "var:item_b_pose == agv",
                                    //  && var:item_a_pose == gripper"
                                    // aha ok, then unmount gripper will not be here, so we'll add it later
                                    // "var:scanned_a == true && var:gripper_actual_state == closed", 
                                    // "var:gripper_actual_state == closed",
                                    // "var:robot_actual_state == agv && var:mounted == gripper"
                                    "var:item_a_pose == gripper",
                                    // "var:item_b_pose == gripper"
                                    ]
                                    .choose(&mut rand::thread_rng())
                                    .unwrap()
                                    .to_spvalue(),
                            ),
                            (
                                "item_a_pose".to_string(),
                                vec!["box_a", "box_b", "gripper", "agv"]
                                    .choose(&mut rand::thread_rng())
                                    .unwrap()
                                    .to_spvalue(),
                            ),
                            // (
                            //     "item_b_pose".to_string(),
                            //     vec!["box_a", "box_b", "gripper", "agv"]
                            //         .choose(&mut rand::thread_rng())
                            //         .unwrap()
                            //         .to_spvalue(),
                            // ),
                            (
                                "scanned_a".to_string(),
                                vec![false, true]
                                    .choose(&mut rand::thread_rng())
                                    .unwrap()
                                    .to_spvalue(),
                            ),
                            // (
                            //     "scanned_b".to_string(),
                            //     vec![false, true]
                            //         .choose(&mut rand::thread_rng())
                            //         .unwrap()
                            //         .to_spvalue(),
                            // ),
                            (
                                "gripper_actual_state".to_string(),
                                vec!["opened", "closed", "gripping", "unknown"]
                                    .choose(&mut rand::thread_rng())
                                    .unwrap()
                                    .to_spvalue(),
                            ),
                            (
                                "gantry_actual_state".to_string(),
                                vec!["box_a", "agv", "unknown"]
                                    .choose(&mut rand::thread_rng())
                                    .unwrap()
                                    .to_spvalue(),
                            ),
                            (
                                "robot_actual_state".to_string(),
                                vec!(
                                    // "home",
                                    "toolbox_gripper",
                                    "toolbox_scanner",
                                    "box_a",
                                    // "box_b",
                                    "item_a",
                                    // "item_b",
                                    "agv"
                                )
                                    .choose(&mut rand::thread_rng())
                                    .unwrap()
                                    .to_spvalue(),
                            ),
                            ("runner_replanned".to_string(), false.to_spvalue()),
                            ("runner_replan".to_string(), true.to_spvalue()),
                            ("runner_plan".to_string(), SPValue::Unknown),
                            ("runner_plan_state".to_string(), "empty".to_spvalue()),
                            ("runner_plan_info".to_string(), "Tester injected a new state.".to_spvalue())
                            
                        ]);
                            println!("TEST CASE: {}", test_case);
                            println!("PLAN STATE: {}", plan_state);
                            let _res = client_callback(client, state).await;
                        } else {
                            ()
                        }
                    }
                    _ => (),
                }
            }
            None => (),
        }
    }
    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
    let shsl = shared_state.lock().unwrap().clone();

    let mut nr_of_operations = 0;
    let nr_of_autos = match shsl.get("nr_autos").unwrap() {
        SPValue::Int32(value) => *value as f32, 
        _ => 0.0 as f32
    };
    let mut nr_of_taken_autos = 0;
    let mut nr_disabled = 0;
    let mut nr_executing = 0;
    let mut nr_failed = 0;
    let mut nr_timedout = 0;
    let mut nr_completed = 0;
    println!("Calculating SCBM-IAS...");
    shsl.iter().for_each(|(k, v)| {
        if k.starts_with("op_") {
            nr_of_operations = nr_of_operations + 1
        } else if k.starts_with("diabled_") {
            match v {
                SPValue::Int32(number) => if *number != 0 as i32 {
                    nr_disabled = nr_disabled + 1
                },
                _ => ()
            }           
        } else if k.starts_with("fail_") {
            match v {
                SPValue::Int32(number) => if *number != 0 as i32 {
                    nr_failed = nr_failed + 1
                },
                _ => ()
            }           
        } else if k.starts_with("executing_") {
            match v {
                SPValue::Int32(number) => if *number != 0 as i32 {
                    nr_executing = nr_executing + 1
                },
                _ => ()
            }           
        } else if k.starts_with("timedout_") {
            match v {
                SPValue::Int32(number) => if *number != 0 as i32 {
                    nr_timedout = nr_timedout + 1
                },
                _ => ()
            }           
        } else if k.starts_with("completed_") {
            match v {
                SPValue::Int32(number) => if *number != 0 as i32 {
                    nr_completed = nr_completed + 1
                },
                _ => ()
            }    
        } else if k.starts_with("taken_auto") {
            match v {
                SPValue::Int32(number) => if *number != 0 as i32 {
                    nr_of_taken_autos = nr_of_taken_autos + 1
                },
                _ => ()
            }        
         
        } else {

        }
    });
    let total = (nr_completed + nr_disabled + nr_executing + nr_failed + nr_timedout + nr_of_taken_autos) as f32/((5.0*nr_of_operations as f32) + nr_of_autos);
    println!("============================================");
    // println!("CSBM-IAS report: {:.2}%", total);
    println!("Coverabilty report: CSBM-IAS:...{:.2}%", total*100.0);
    println!("Testing duration:...............{:?}", now.elapsed());
    println!("============================================");
    println!("Visited disabled state:.........{} out of {}", nr_disabled, nr_of_operations);
    println!("Visited executing state:........{} out of {}", nr_executing, nr_of_operations);
    println!("Visited failed state:...........{} out of {}", nr_failed, nr_of_operations);
    println!("Visited timedout state:.........{} out of {}", nr_timedout, nr_of_operations);
    println!("Visited completed state:........{} out of {}", nr_completed, nr_of_operations);
    println!("Automatic transitions taken:....{} out of {}", nr_of_taken_autos, nr_of_autos);
    println!("============================================");
   
    Ok(())
}

// some things we can test:
// 1. if the goal is that the object is to be scanned, the object should eventually be scanned
// 2. if scanning fails, the object should be re-scanned (merge 2 and 4)
// 3. if scanning fails 5 times in total, the goal should be aborted // skip this one
// 4. if scanning fails 3 times in a row, the goal should be aborted
// 5. if scanning times out, try to re-scanned again
// 6. if scanning times out 2 times in a row, the goal should be aborted

// some testing strategies:
// random is random, exhaustive is exhaustive, but...
// strategic testing is for the previous properties:
// 1. define a set of simple goals where the object is scanned in all of them. Now we have to make 
//      an association that if the scan request succeeds, the test will be finished sooner. 
//      Maybe I can do that manually, i.e. to set the goal, initial state and also the emulator parameters...
// 2. force failure of scanning, if rescan is triggeredm the test succeeds...
// so on...
// async fn scanner_test_process(
//     client: &r2r::Client<SetState::Service>,
//     shared_state: &Arc<Mutex<HashMap<String, SPValue>>>,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     let mut test_case = 0;
//     Ok(while test_case < NR_TEST_CASES {
//         tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
//         let shsl = shared_state.lock().unwrap().clone();
//         match shsl.get("runner_plan_state") {
//             Some(runner_plan_state) => {
//                 match runner_plan_state {
//                     SPValue::String(plan_state) => {
//                         if plan_state == "failed" || plan_state == "done" || plan_state == "aborted"
//                         {
//                             test_case = test_case + 1;
//                             let state = HashMap::from([
//                             // should all be achieevable goals    
//                             (
//                                 "runner_goal".to_string(),
//                                 vec!["var:scanned_a == true", "var:scanned_a == true && var:gripper_actual_state == closed", "var:scanned_a == true && var:gripper_actual_state == opened"]
//                                     .choose(&mut rand::thread_rng())
//                                     .unwrap()
//                                     .to_spvalue(),
//                             ),
//                             (
//                                 "gripper_actual_state".to_string(),
//                                 vec!["opened", "closed", "gripping", "unknown"]
//                                     .choose(&mut rand::thread_rng())
//                                     .unwrap()
//                                     .to_spvalue(),
//                             ),
//                             (
//                                 "scanned_a".to_string(),
//                                 vec![true, false]
//                                     .choose(&mut rand::thread_rng())
//                                     .unwrap()
//                                     .to_spvalue(),
//                             ),
//                             ("runner_replanned".to_string(), false.to_spvalue()),
//                             ("runner_replan".to_string(), true.to_spvalue()),
//                             ("runner_plan".to_string(), SPValue::Unknown),
//                             ("runner_plan_state".to_string(), "empty".to_spvalue()),
//                             ("runner_plan_info".to_string(), "Tester injected a new state.".to_spvalue())
                            
//                         ]);
//                             println!("TEST CASE: {}", test_case);
//                             println!("PLAN STATE: {}", plan_state);
//                             let _res = client_callback(client, state).await; // 2 seconds delay in the call
//                             // let shsl = shared_state.lock().unwrap().clone();

//                             // TESTS:
//                             // 1. if the goal is that the object is to be scanned, the object should eventually be scanned

//                             // MAYBE I DO NEED AN ACTION FOR TESTING AFTER ALL...
//                             // assert_eq!(shsl.get("runner_plan_state").unwrap().clone(), "done".to_spvalue());

//                         } else {
//                             ()
//                         }
//                     }
//                     _ => (),
//                 }
//             }
//             None => (),
//         }
//     })
// }

async fn client_callback(
    client: &r2r::Client<SetState::Service>,
    state: HashMap<String, SPValue>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut map = serde_json::Map::<String, Value>::new();
    state.iter().for_each(|(k, v)| {
        let _ = map.insert(
            k.to_string(),
            match v {
                SPValue::Array(_, val) => Value::from(format!("array__{:?}", val)),
                SPValue::Bool(val) => Value::from(format!("bool___{}", val)),
                SPValue::Float64(val) => Value::from(format!("float__{}", val)),
                SPValue::String(val) => Value::from(format!("string_{}", val)),
                SPValue::Int32(val) => Value::from(format!("int____{}", val)),
                SPValue::Time(val) => Value::from(format!("time___{:?}", val)),
                SPValue::Unknown => Value::from(format!("unknown")),
            },
        );
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
