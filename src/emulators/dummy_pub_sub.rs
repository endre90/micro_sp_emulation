use futures::{Stream, StreamExt};
use micro_sp::SPValue;
use r2r::micro_sp_emulation_msgs::msg::DummyIncoming;
use r2r::micro_sp_emulation_msgs::msg::DummyOutgoing;

use micro_sp::*;
use r2r::QosProfile;
use std::error::Error;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "dummy_pub_sub";
// pub static BUFFER_MAINTAIN_RATE: u64 = 100;
pub static PUBLISHER_RATE: u64 = 100;
pub static STATE_UPDATE_RATE: u64 = 1000;
// pub static FRAME_LIFETIME: i32 = 3; //seconds

// copy over the model here
pub fn make_initial_state() -> State {
    let state = State::new();
    let state = state.add(SPAssignment::new(
        v_command!("ref_pos", vec!("a", "b", "c")),
        SPValue::Unknown,
    ));
    let state = state.add(SPAssignment::new(
        v_measured!("act_pos", vec!("a", "b", "c")),
        SPValue::Unknown,
    ));
    state
}

// this file should be completely autogenerated, as well as the message package
// copy over the model here (now the stateless should only look at the command vars)
pub fn the_model() -> Model {
    let state = make_initial_state();
    // let op_move_to_a = Operation::new(
    //     "op_move_to_a",
    //     t!(
    //         "start_move_to_a",
    //         "var:ref_pos != a",
    //         "true",
    //         vec!("var:ref_pos <- a"),
    //         Vec::<&str>::new(),
    //         &state
    //     ),
    //     t!(
    //         "complete_move_to_a",
    //         "var:ref_pos == a",
    //         "true",
    //         vec!("var:act_pos <- a"),
    //         Vec::<&str>::new(),
    //         &state
    //     ),
    // );
    // let op_move_to_b = Operation::new(
    //     "op_move_to_b",
    //     t!(
    //         "start_move_to_b",
    //         "var:ref_pos == a",
    //         "true",
    //         vec!("var:ref_pos <- b"),
    //         Vec::<&str>::new(),
    //         &state
    //     ),
    //     t!(
    //         "complete_move_to_b",
    //         "var:ref_pos == b",
    //         "true",
    //         vec!("var:act_pos <- b"),
    //         Vec::<&str>::new(),
    //         &state
    //     ),
    // );
    // let op_move_to_c = Operation::new(
    //     "op_move_to_c",
    //     t!(
    //         "start_move_to_c",
    //         "var:ref_pos == b",
    //         "true",
    //         vec!("var:ref_pos <- c"),
    //         Vec::<&str>::new(),
    //         &state
    //     ),
    //     t!(
    //         "complete_move_to_c",
    //         "var:ref_pos == c",
    //         "true",
    //         vec!("var:act_pos <- c"),
    //         Vec::<&str>::new(),
    //         &state
    //     ),
    // );

        // Define the operations
        let op_move_to_a = Operation::new(
            "op_move_to_a",
            t!(
                "start_move_to_a",
                "var:ref_pos != a && var:act_pos != a",
                "true",
                vec!("var:ref_pos <- a"),
                Vec::<&str>::new(),
                &state
            ),
            t!(
                "complete_move_to_a",
                "var:ref_pos == a", // && var:act_pos != a",
                "var:act_pos == a",
                vec!("var:act_pos <- a"),
                Vec::<&str>::new(),
                &state
            ),
        );
        
        let op_move_to_b = Operation::new(
            "op_move_to_b",
            t!(
                "start_move_to_b",
                "var:ref_pos == a && var:act_pos == a",
                "true",
                vec!("var:ref_pos <- b"),
                Vec::<&str>::new(),
                &state
            ),
            t!(
                "complete_move_to_b",
                "var:ref_pos == b", // && var:act_pos == a",
                "var:act_pos == b",
                vec!("var:act_pos <- b"),
                Vec::<&str>::new(),
                &state
            ),
        );
    
        let op_move_to_c = Operation::new(
            "op_move_to_c",
            t!(
                "start_move_to_c",
                "var:ref_pos == b && var:act_pos == b",
                "true",
                vec!("var:ref_pos <- c"),
                Vec::<&str>::new(),
                &state
            ),
            t!(
                "complete_move_to_c",
                "var:ref_pos == c", // && var:act_pos == b",
                "var:act_pos == c",
                vec!("var:act_pos <- c"),
                Vec::<&str>::new(),
                &state
            ),
        );

    Model::new(
        "asdf",
        state.clone(),
        vec![],
        vec![
            op_move_to_a.clone(),
            op_move_to_b.clone(),
            op_move_to_c.clone(),
        ],
        vec!()
    )
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let emulator_state = Arc::new(Mutex::new(make_initial_state()));

    // spawn a tokio task to listen to messages from micro_sp
    let subscriber = node.subscribe::<DummyOutgoing>(
        "dummy_outgoing",
        QosProfile::best_effort(QosProfile::default()),
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
        node.create_publisher::<DummyIncoming>("dummy_incoming", QosProfile::default())?;
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
        let result =
            update_state(&emulator_state_clone, update_state_timer, NODE_ID).await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "Publisher succeeded."),
            Err(e) => r2r::log_error!(NODE_ID, "Publisher failed with: {}.", e),
        };
    });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));
    });

    r2r::log_warn!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

pub async fn subscriber_callback(
    emulator_state: &Arc<Mutex<State>>,
    mut subscriber: impl Stream<Item = DummyOutgoing> + Unpin,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let emulator_state_local = emulator_state.lock().unwrap().clone();
                // will have to hapen for each field, but can be generated
                let emulator_state_local =
                    emulator_state_local.update("ref_pos", message.ref_pos.to_spvalue());
                *emulator_state.lock().unwrap() = emulator_state_local;
            }
            None => r2r::log_error!(node_id, "Subscriber did not get the message?"),
        }
    }
}

pub async fn publisher_callback(
    emulator_state: &Arc<Mutex<State>>,
    publisher: r2r::Publisher<DummyIncoming>,
    mut timer: r2r::Timer,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let emulator_state_local = emulator_state.lock().unwrap().clone();
        let msg = DummyIncoming {
            act_pos: match emulator_state_local.get_value("act_pos") {
                SPValue::String(value) => value,
                _ => "unknown".to_string()
            }
        };
        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => ()
        }
        timer.tick().await?;
    }
}

pub async fn update_state(
    emulator_state: &Arc<Mutex<State>>,
    mut timer: r2r::Timer,
    node_id: &str
) -> Result<(), Box<dyn std::error::Error>> {
// ) -> HashMap<String, SPValue> {
    let model = the_model();
    loop {
        let mut emulator_state_local = emulator_state.lock().unwrap().clone();
        model.operations.iter().for_each(|op| {
            emulator_state_local = match op.postcondition.clone().eval_planning(&emulator_state_local) {
                false => emulator_state_local.clone(),
                true => take_emulated(&op.postcondition.clone(), &emulator_state_local)
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