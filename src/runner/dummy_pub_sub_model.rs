use micro_sp::*;
use serde_json::json;

pub fn the_model() -> Model {

    // Define the variables
    let act_pos = v_command!("act_pos", vec!("a", "b", "c"));
    let ref_pos = v_measured!("ref_pos", vec!("a", "b", "c"));

    // Make a state and assign some values to variables
    // (usually, unknown is a safe initial value for variables)
    let state = State::new();
    let state = state.add(SPAssignment::new(
        act_pos.clone(),
        SPValue::Unknown,
    ));
    let state = state.add(SPAssignment::new(
        ref_pos.clone(),
        SPValue::Unknown,
    ));

    // And some mandatory variables (actually, these should be 
    // included with the Model::new function...)
        let state = state.add(SPAssignment::new(
        v_runner!("runner_goal"),
        "var:act_pos == c".to_spvalue(),
    ));
    let state = state.add(SPAssignment::new(
        av_runner!("runner_plan"),
        SPValue::Unknown,
    ));
    let state = state.add(SPAssignment::new(
        v_runner!("runner_plan_status"),
        SPValue::Unknown,
    ));
    let state = state.add(SPAssignment::new(
        iv_runner!("runner_plan_current_step"),
        SPValue::Unknown,
    ));
    let state = state.add(SPAssignment::new(
        bv_runner!("runner_replan"),
        true.to_spvalue(),
    ));
    let state = state.add(SPAssignment::new(
        bv_runner!("runner_replanned"),
        false.to_spvalue(),
    ));  

    // Define automatic transitions (these transitions will immediatelly 
    // be executed if evaluated to be true)
    let autos: Vec<Transition> = vec!();

    // Define the message types
    let dummy_ougoing_msg = json!({
        "ref_pos": "String"
    });

    let dummy_incoming_msg = json!({
        "act_pos": "String"
    });

    // Define the reources
    let dummy_resource = Resource {
        name: "dummy_pub_sub".to_string(),
        msgs: vec!(
            Message {
                name: "DummyOutgoing".to_string(),
                topic: "dummy_outgoing".to_string(),
                category: MessageCategory::OutGoing,
                message_type: dummy_ougoing_msg.to_string(),
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
                message_type: dummy_incoming_msg.to_string(),
                variables: vec!(
                    act_pos
                ),
                variables_feedback: vec!(),
                variables_response: vec!()
            }
        )
    };

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