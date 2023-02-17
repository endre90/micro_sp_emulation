use futures::{Stream, StreamExt};
use micro_sp::SPValue;
use r2r::micro_sp_emulation_msgs::msg::GantryIncoming;
use r2r::micro_sp_emulation_msgs::msg::GantryOutgoing;

use micro_sp::*;
use r2r::QosProfile;
use std::error::Error;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "gantry_pub_sub_emu";
pub static PUBLISHER_RATE: u64 = 100;
pub static STATE_UPDATE_RATE: u64 = 100;

pub fn make_initial_state() -> State {
    let state = State::new();
    let state = state.add(SPAssignment::new(
        v_command!("gantry_ref", vec!("a", "b", "atr")),
        SPValue::Unknown,
    ));
    let state = state.add(SPAssignment::new(
        v_measured!("gantry_act", vec!("a", "b", "atr")),
        SPValue::Unknown,
    ));
    state
}

// copy over the model here (now should only look at the command vars in the guards, so filter out the rest)
pub fn the_model() -> Model {
    let state = make_initial_state();

    let mut operations = vec!();
    for pose in vec!["a", "b", "atr"] {
        operations.push(Operation::new(
            &format!("op_gantry_move_to_{pose}"),
            // precondition
            t!(
                // name
                &format!("start_gantry_move_to_{pose}").as_str(),
                // planner guard
                &format!("var:gantry_ref != {pose}").as_str(),
                // runner guard
                "true",
                // planner actions
                vec!(&format!("var:gantry_ref <- {pose}").as_str()),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                &format!("complete_gantry_move_to_{pose}").as_str(),
                // planner guard
                &format!("var:gantry_ref == {pose}").as_str(),
                // runner guard
                "true",
                // planner actions
                vec!(&format!("var:gantry_act <- {pose}").as_str()),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        ))
    }

    Model::new(
        "asdf",
        state.clone(),
        vec![],
        operations,
        vec![],
    )
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let emulator_state = Arc::new(Mutex::new(make_initial_state()));

    // spawn a tokio task to listen to messages from micro_sp
    let subscriber = node.subscribe::<GantryOutgoing>(
        "gantry_outgoing",
        QosProfile::default().reliable()
    )?;
    let emulator_state_clone = emulator_state.clone();
    tokio::task::spawn(async move {
        match subscriber_callback(&emulator_state_clone, subscriber, NODE_ID).await {
            Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
        };
    });

    let publisher_timer =
        node.create_wall_timer(std::time::Duration::from_millis(PUBLISHER_RATE))?;
    let publisher =
        node.create_publisher::<GantryIncoming>("gantry_incoming", QosProfile::default().reliable())?;
    let emulator_state_clone = emulator_state.clone();
    tokio::task::spawn(async move {
        let result =
            publisher_callback(&emulator_state_clone, publisher, publisher_timer, NODE_ID).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Publisher succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Publisher failed with: {}.", e),
        };
    });

    let emulator_state_clone = emulator_state.clone();
    let update_state_timer =
        node.create_wall_timer(std::time::Duration::from_millis(STATE_UPDATE_RATE))?;
    tokio::task::spawn(async move {
        let result = update_state(&emulator_state_clone, update_state_timer, NODE_ID).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Publisher succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Publisher failed with: {}.", e),
        };
    });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(20));
    });

    r2r::log_warn!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

pub async fn subscriber_callback(
    emulator_state: &Arc<Mutex<State>>,
    mut subscriber: impl Stream<Item = GantryOutgoing> + Unpin,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let emulator_state_local = emulator_state.lock().unwrap().clone();
                // will have to hapen for each field, but can be generated
                let emulator_state_local =
                    emulator_state_local.update("gantry_ref", message.gantry_ref.to_spvalue());
                *emulator_state.lock().unwrap() = emulator_state_local;
            }
            None => r2r::log_error!(node_id, "Subscriber did not get the message?"),
        }
    }
}

pub async fn publisher_callback(
    emulator_state: &Arc<Mutex<State>>,
    publisher: r2r::Publisher<GantryIncoming>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let emulator_state_local = emulator_state.lock().unwrap().clone();
        let msg = GantryIncoming {
            gantry_act: match emulator_state_local.get_value("gantry_act") {
                SPValue::String(value) => value,
                _ => "unknown".to_string(),
            },
        };
        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => (),
        }
        timer.tick().await?;
    }
}

pub async fn update_state(
    emulator_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    // ) -> HashMap<String, SPValue> {
    let model = the_model();
    loop {
        let mut emulator_state_local = emulator_state.lock().unwrap().clone();
        model.operations.iter().for_each(|op| {
            emulator_state_local = match op
                .postcondition
                .clone()
                .eval_planning(&emulator_state_local)
            {
                false => emulator_state_local.clone(),
                true => take_emulated(&op.postcondition.clone(), &emulator_state_local),
            };
        });
        println!("{}", emulator_state_local);
        *emulator_state.lock().unwrap() = emulator_state_local;

        timer.tick().await?;
    }
}

pub fn take_emulated(t: &Transition, state: &State) -> State {
    let mut new_state = state.clone();
    for a in &t.actions {
        new_state = a.clone().assign(&new_state)
    }
    for a in &t.runner_actions {
        new_state = a.clone().assign(&new_state)
    }
    new_state
}
