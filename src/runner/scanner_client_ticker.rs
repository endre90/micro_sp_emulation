// use futures::Future;
// // use micro_sp::SPValue;
// // use micro_sp::ShardedMutex;
// use micro_sp::*;
// use r2r::{micro_sp_emulation_msgs::srv::TriggerScan, Error};
// use std::sync::{Arc, Mutex};

// pub async fn scanner_client_ticker(
//     scanner_client: &r2r::Client<TriggerScan::Service>,
//     wait_for_server: impl Future<Output = Result<(), Error>>,
//     // shared_state: &Arc<Mutex<State>>,
//     sharded_mutex: &Arc<ShardedMutex>, //&Arc<Mutex<State>>,
//     mut timer: r2r::Timer,
//     node_id: &str,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     r2r::log_warn!(node_id, "Waiting for the scanner server...");
//     wait_for_server.await?;
//     r2r::log_warn!(node_id, "Scanner server available.");

//     loop {
//         let mut scanner_request_state_shard = sharded_mutex.lock("scanner_request_state").clone();
//         let scanner_request_state =
//             match scanner_request_state_shard.get("scanner_request_state").unwrap().clone().val {
//                 SPValue::String(value) => value,
//                 _ => "unknown".to_string(),
//             };

//         // *sharded_mutex.lock("scanner_request_state") = scanner_request_state_shard.update("scanner_request_state", "succeeded".to_spvalue());
//         // *scanner_request_state_shard.lock() = scanner_request_state_shard.update("scanner_request_state", "succeeded".to_spvalue());

//         // let shards = *sharded_mutex.clone();
//         // for shard in shards {
//         //     let asdf = shard.lock().unwrap();
//         // }
//         // let shards = *sharded_mutex.clone();
//         // let shard = sharded_mutex[hash("scanner_request_trigger") % sharded_mutex.len()];
//         // let shsl = shared_state.lock().unwrap().clone();
//         // let mut data = sharded_mutex.lock(&key);
//         //     *data.entry(key).or_insert(false) = true;
//         let scanner_request_trigger_shard = sharded_mutex.lock("scanner_request_trigger").clone();
//         let scanner_request_trigger =
//             match scanner_request_trigger_shard.get("scanner_request_trigger").unwrap().clone().val {
//                 SPValue::Bool(value) => value,
//                 _ => false,
//             };
//         // let scanner_request_trigger = shared_mutex.lock("scanner_request_trigger").clone().get("scanner_request_trigger").unwrap().val;
//         // let scanner_request_trigger = match shsl.get_value("scanner_request_trigger") {
//         //     SPValue::Bool(value) => value,
//         //     _ => false,
//         // };
//         // let scanner_request_state = match shsl.get_value("scanner_request_state") {
//         //     SPValue::String(value) => value,
//         //     _ => "unknown".to_string(),
//         // };

//         // let scanner_request_state_shard = sharded_mutex.lock("scanner_request_state").clone();
//         // let scanner_request_state =
//         //     match scanner_request_state_shard.get_value("scanner_request_state") {
//         //         SPValue::String(value) => value,
//         //         _ => "unknown".to_string(),
//         //     };
//         if scanner_request_trigger {
//             if scanner_request_state == "initial".to_string() {
//                 let request = TriggerScan::Request {
//                     point_cloud_path: "/path/to/file".to_string(),
//                     parameters: "parameters".to_string(),
//                 };

//                 let response = scanner_client
//                     .request(&request)
//                     .expect("Could not send scan request.")
//                     .await
//                     .expect("Cancelled.");

//                 // *data.entry(key).or_insert(false) = true;

//                 match response.success {
//                     true => {
//                         *sharded_mutex.lock("scanner_request_state") = scanner_request_state_shard
//                             .update("scanner_request_state", "succeeded".to_spvalue());
//                         // *scanner_request_state_shard = scanner_request_state_shard
//                         //     .update("scanner_request_state", "succeeded".to_spvalue())
//                     }
//                     false => {
//                         let fail_counter_scanner_shard = sharded_mutex.lock("fail_counter_scanner");
//                         println!("TOUPDATEFAILSCANNER: {:?}", fail_counter_scanner_shard);
//                         let fail_counter_scanner =
//                             match fail_counter_scanner_shard.get_value("fail_counter_scanner") {
//                                 SPValue::Int32(value) => value,
//                                 _ => 0,
//                             };
//                         *sharded_mutex.lock("scanner_request_state") = scanner_request_state_shard
//                             .update("scanner_request_state", "failed".to_spvalue());
//                         *sharded_mutex.lock("fail_counter_scanner") = fail_counter_scanner_shard
//                             .update(
//                                 "fail_counter_scanner",
//                                 (fail_counter_scanner + 1).to_spvalue(),
//                             );
//                     }
//                 }
//             }
//         }
//         timer.tick().await?;
//     }
// }


use futures::Future;
use micro_sp::SPValue;
use micro_sp::*;
use r2r::{micro_sp_emulation_msgs::srv::TriggerScan, Error};
use std::sync::{Arc, Mutex};

pub async fn scanner_client_ticker(
    scanner_client: &r2r::Client<TriggerScan::Service>,
    wait_for_server: impl Future<Output = Result<(), Error>>,
    shared_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_warn!(node_id, "Waiting for the scanner server...");
    wait_for_server.await?;
    r2r::log_warn!(node_id, "Scanner server available.");

    loop {
        let shsl = shared_state.lock().unwrap().clone();
        let scanner_request_trigger = match shsl.get_value("scanner_request_trigger") {
            SPValue::Bool(value) => value,
            _ => false,
        };
        let scanner_request_state = match shsl.get_value("scanner_request_state") {
            SPValue::String(value) => value,
            _ => "unknown".to_string(),
        };
        if scanner_request_trigger {
            if scanner_request_state == "initial".to_string() {
                let request = TriggerScan::Request {
                    point_cloud_path: "/path/to/file".to_string(),
                    parameters: "parameters".to_string(),
                };

                let response = scanner_client
                    .request(&request)
                    .expect("Could not send scan request.")
                    .await
                    .expect("Cancelled.");

                *shared_state.lock().unwrap() = match response.success {
                    true => shsl.update("scanner_request_state", "succeeded".to_spvalue()),
                    false => {
                        let fail_counter_scanner = match shsl.get_value("fail_counter_scanner") {
                            SPValue::Int32(value) => value,
                            _ => 0,
                        };
                        shsl.update("scanner_request_state", "failed".to_spvalue())
                            .update(
                                "fail_counter_scanner",
                                (fail_counter_scanner + 1).to_spvalue(),
                            )
                    }
                }
            }
        }
        timer.tick().await?;
    }
}