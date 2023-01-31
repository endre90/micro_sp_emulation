use micro_sp::*;
use serde::{Deserialize, Serialize};
use serde_json::json;

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

pub fn the_model() -> Model {

    // Define the variables
    let act_pos = v_command!("act_pos", vec!("a", "b", "c"));
    let ref_pos = v_measured!("ref_pos", vec!("a", "b", "c"));

    // Define a state (usually, unknown is a safe value for variables)
    let state = State::new();
    let state = state.add(SPAssignment::new(
        act_pos.clone(),
        SPValue::Unknown,
    ));
    let state = state.add(SPAssignment::new(
        ref_pos.clone(),
        SPValue::Unknown,
    ));

    // Define automatic transitions (these transitions will immediatelly 
    // be executed if evaluated to be true)
    let autos: Vec<Transition> = vec!();

    // Define the message types
    #[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
    struct DummyOutgoing {
        ref_pos: String
    }

    // Define the reources
    let dummy_resource = Resource {
        name: "dummy_pub_sub".to_string(),
        msgs: vec!(
            Message {
                name: "DummyOutgoing".to_string(),
                topic: "dummy_outgoing".to_string(),
                category: MessageCategory::OutGoing,
                message_type: json!({
                    "ref_pos": "String"
                }).to_string(),
                variables: vec!(
                    ref_pos
                ),
                variables_feedback: vec!(),
                variables_response: vec!()
            },
            Message {
                name: "DummyIncoming".to_string(),
                topic: "dummy_incoming".to_string(),
                category: MessageCategory::Incoming,
                message_type: json!({
                    "act_pos": "String"
                }).to_string(),
                variables: vec!(
                    act_pos
                ),
                variables_feedback: vec!(),
                variables_response: vec!()
            }
        )
    };


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
            "var:ref_pos == a && var:act_pos != a",
            "true",
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
            "var:ref_pos == b && var:act_pos == a",
            "true",
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
            "var:ref_pos == c && var:act_pos == b",
            "true",
            vec!("var:act_pos <- c"),
            Vec::<&str>::new(),
            &state
        ),
    );

    Model::new(
        "asdf",
        state.clone(),
        autos,
        vec![
            op_move_to_a.clone(),
            op_move_to_b.clone(),
            op_move_to_c.clone(),
        ],
        vec!(dummy_resource)
    )
}