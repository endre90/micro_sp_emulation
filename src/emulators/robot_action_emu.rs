use futures::{Stream, StreamExt};
use micro_sp::SPValue;
use r2r::micro_sp_emulation_msgs::action::URCommand;

use micro_sp::*;
use r2r::QosProfile;
use std::error::Error;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "robot_action_emu";
// pub static PUBLISHER_RATE: u64 = 1000;
// pub static STATE_UPDATE_RATE: u64 = 1000;

// pub fn make_initial_state() -> State {
//     let state = State::new();
//     let state = state.add(SPAssignment::new(
//         v_command!("gripper_ref", vec!("opened", "closed")),
//         SPValue::Unknown,
//     ));
//     let state = state.add(SPAssignment::new(
//         v_measured!("gripper_act", vec!("opened", "closed", "gripping")),
//         SPValue::Unknown,
//     ));
//     state
// }

// copy over the model here (now should only look at the command vars in the guards, so filter out the rest)
// pub fn the_model() -> Model {
//     let state = make_initial_state();

//     let mut operations = vec![];
//     // 1. Robot move operations
//     for pose in vec![
//         "home",
//         "rack_scanner",
//         "rack_gripper",
//         "rack_suction",
//         "box_a",
//         "box_b",
//         "atr",
//         "item_a",
//         "item_b",
//     ] {
//         operations.push(
//             Operation::new(
//                 &format!("op_robot_move_to_{pose}"), 
//                 // precondition
//                 t!(
//                     // name
//                     &format!("start_robot_move_to_{pose}").as_str(),
//                     // planner guard
//                     &format!("var:ur_action_trigger == false && var:ur_action_state == initial && ur_pose != {pose}").as_str(),
//                     // runner guard
//                     "true",
//                     // planner actions
//                     vec!("var:ur_command <- movej", "var:ur_action_trigger <- true", &format!("var:ur_goal_frame <- {pose}").as_str()),
//                     //runner actions
//                     Vec::<&str>::new(),
//                     &state
//                 ),
//                 // postcondition
//                 t!(
//                     // name
//                     &format!("complete_robot_move_to_{pose}").as_str(),
//                     // planner guard
//                     &format!("var:ur_action_state == done").as_str(),
//                     // runner guard
//                     "true", // the action emulator could update this...
//                     // planner actions
//                     vec!("var:ur_action_trigger <- false", &format!("var:ur_pose <- {pose}").as_str()),
//                     //runner actions
//                     Vec::<&str>::new(),
//                     &state
//                 ),
//             )
//         )
//     }

//     Model::new("asdf", state.clone(), vec![], operations, vec![])
// }

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    // let emulator_state = Arc::new(Mutex::new(make_initial_state()));

    let action_server =
        node.create_action_server::<URCommand::Action>("robot_action")?;

    tokio::task::spawn(async move {
        let result = robot_action_server(
            action_server,
        )
        .await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Robot action service call succeeded."),
            Err(e) => r2r::log_error!(
                NODE_ID,
                "Robot action service call failed with: '{}'.",
                e
            ),
        };
    });

    // let emulator_state_clone = emulator_state.clone();
    // let update_state_timer =
    //     node.create_wall_timer(std::time::Duration::from_millis(STATE_UPDATE_RATE))?;
    // tokio::task::spawn(async move {
    //     let result = update_state(&emulator_state_clone, update_state_timer, NODE_ID).await;
    //     match result {
    //         Ok(()) => r2r::log_info!(NODE_ID, "Publisher succeeded."),
    //         Err(e) => r2r::log_error!(NODE_ID, "Publisher failed with: {}.", e),
    //     };
    // });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(20));
    });

    r2r::log_warn!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}
pub async fn robot_action_server(
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<URCommand::Action>> + Unpin,
    // shared_state: &Arc<Mutex<State>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        // tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
        // filter the actions of operations and the command from the message to figure out which one is to be taken
        match requests.next().await {
            Some(request) => {
                println!("Got goal request: {:?}", request.goal);
                
                let (mut g, mut _cancel) =
                    request.accept().expect("Could not accept goal request.");
                    // emulate task execution time
               
                println!("just_succeeding...");
                g.succeed(URCommand::Result { 
                    success: true,
                    info: "just_succeeding...".to_string()
                })
                    .expect("Could not send result.");
                continue;
                // let g_clone = g.clone();
                // match execute_urscript(g_clone, &urc_client, &tf_lookup_client, &templates).await {
                //     Ok(ok) => {
                //         g.succeed(URControl::Result { success: ok })
                //             .expect("Could not send result.");
                //         continue;
                //     }
                //     Err(e) => {
                //         let _ = g.abort(URControl::Result { success: false });
                //         continue;
                //     }
                // }
            }
            None => (),
        }
    }
}

// pub async fn update_state(
//     emulator_state: &Arc<Mutex<State>>,
//     mut timer: r2r::Timer,
//     node_id: &str,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     // ) -> HashMap<String, SPValue> {
//     let model = the_model();
//     loop {
//         let mut emulator_state_local = emulator_state.lock().unwrap().clone();
//         model.operations.iter().for_each(|op| {
//             emulator_state_local = match op
//                 .postcondition
//                 .clone()
//                 .eval_planning(&emulator_state_local)
//             {
//                 false => emulator_state_local.clone(),
//                 true => take_emulated(&op.postcondition.clone(), &emulator_state_local),
//             };
//         });
//         println!("{}", emulator_state_local);
//         *emulator_state.lock().unwrap() = emulator_state_local;

//         timer.tick().await?;
//     }
// }

// pub fn take_emulated(t: &Transition, state: &State) -> State {
//     let mut new_state = state.clone();
//     for a in &t.actions {
//         new_state = a.clone().assign(&new_state)
//     }
//     for a in &t.runner_actions {
//         new_state = a.clone().assign(&new_state)
//     }
//     new_state
// }
