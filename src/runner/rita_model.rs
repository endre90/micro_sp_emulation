use micro_sp::*;
use serde_json::json;

pub fn rita_model() -> Model {

    // Define the variables
    // -------------------------------------------------------
    // 1. Robot variables
    let ur_action_trigger = bv_runner!("ur_action_trigger");
    let ur_action_state = v_runner!("ur_action_state"); //, vec!("initial", "executing", "failed", "done"));
    let ur_command = v_command!("ur_command", vec!("movej", "movel", "mount", "unmount", "pick_with_gripper", "place_with_gripper", "pick_with_suction", "place_with_suction"));
    let ur_goal_frame = v_measured!("ur_goal_frame", vec!("home", "rack_gripper", "rack_suction", "rack_scanner", "box_a", "box_b", "item_a", "item_b", "atr"));
    let ur_pose = v_estimated!("ur_pose", vec!("home", "rack_gripper", "rack_suction", "rack_scanner", "box_a", "box_b", "item_a", "item_b", "atr"));
    let ur_tcp = v_estimated!("ur_tcp", vec!("gripper", "suction", "scanner"));
    let ur_mounted = v_estimated!("ur_mounted", vec!("none", "scanner", "gripper", "suction"));

    // 2. Gantry variables
    let gantry_act = v_measured!("gantry_act", vec!("a", "b", "left", "right"));
    let gantry_ref = v_command!("gantry_ref", vec!("a", "b", "left", "right"));

    // 3. Scanner variables
    let scanner_trigger = bv_runner!("scanner_trigger");
    let scanner_state = v_runner!("scanner_state"); //, vec!("initial", "executing", "failed", "done"));
    let scanned_a = bv_estimated!("scanned_a");
    let scanned_b = bv_estimated!("scanned_b");
    let scanner_item_a_path = v_runner!("scanner_item_a_path");
    let scanner_item_b_path = v_runner!("scanner_item_b_path");

    // 4. Gripper variables
    let gripper_act = v_measured!("gripper_act", vec!("opened", "closed", "gripping"));
    let gripper_ref = v_command!("gripper_ref", vec!("opened", "closed"));

    // 5. Suction tool variables
    let suction_trigger = bv_runner!("suction_trigger");
    let suction_state = v_estimated!("suction_state", vec!("on", "off"));

    // 6. Localization variables
    let localization_trigger = bv_runner!("localization_trigger");
    let localization_state = v_runner!("localization_state"); //, vec!("initial", "executing", "failed", "done"));

    // 7. Estimated positions of products
    let item_a_pose = v_estimated!("item_a_pose", vec!("box_a", "gripper", "atr")); 
    let item_b_pose = v_estimated!("item_b_pose", vec!("box_b", "suction", "atr"));

    // Make a state and assign some values to variables
    // (usually, unknown is a safe initial value for measured and command variables)
    // ----------------------------------------------------
    let state = State::new();

    // 1. Robot variables
    let state = state.add(assign!(ur_action_trigger, false.to_spvalue()));
    let state = state.add(assign!(ur_action_state, "initial".to_spvalue()));
    let state = state.add(assign!(ur_command, SPValue::Unknown));
    let state = state.add(assign!(ur_goal_frame, SPValue::Unknown));
    let state = state.add(assign!(ur_tcp, SPValue::Unknown));
    let state = state.add(assign!(ur_pose, SPValue::Unknown));
    let state = state.add(assign!(ur_mounted, "none".to_spvalue()));

    // 2. Gantry variables
    let state = state.add(assign!(gantry_act, SPValue::Unknown));
    let state = state.add(assign!(gantry_ref, SPValue::Unknown));

    // 3. Scanner variables
    let state = state.add(assign!(scanner_trigger, false.to_spvalue()));
    let state = state.add(assign!(scanner_state, "initial".to_spvalue()));
    let state = state.add(assign!(scanned_a, false.to_spvalue()));
    let state = state.add(assign!(scanned_b, false.to_spvalue()));
    let state = state.add(assign!(scanner_item_a_path, SPValue::Unknown));
    let state = state.add(assign!(scanner_item_b_path, SPValue::Unknown));

    // 4. Gripper variables
    let state = state.add(assign!(gripper_act, SPValue::Unknown));
    let state = state.add(assign!(gripper_ref, SPValue::Unknown));

    // 5. Suction variables
    let state = state.add(assign!(suction_trigger, false.to_spvalue()));
    let state = state.add(assign!(suction_state, SPValue::Unknown));

    // 6. Localization variables
    let state = state.add(assign!(localization_trigger, false.to_spvalue()));
    let state = state.add(assign!(localization_state, SPValue::Unknown));

    // 7. Estimated positions of products
    let state = state.add(assign!(item_a_pose, "box_a".to_spvalue()));
    let state = state.add(assign!(item_b_pose, "box_b".to_spvalue()));
    
    // And some mandatory variables (actually, these should be 
    // included with the Model::new function...)
        let state = state.add(SPAssignment::new(
        v_runner!("runner_goal"),
        "var:item_a_pose == atr".to_spvalue(),
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

    // // Define the message types
    // let dummy_ougoing_msg = json!({
    //     "ref_pos": "String"
    // });

    // let dummy_incoming_msg = json!({
    //     "act_pos": "String"
    // });

    // // Define the reources
    // let dummy_resource = Resource {
    //     name: "dummy_pub_sub".to_string(),
    //     msgs: vec!(
    //         Message {
    //             name: "DummyOutgoing".to_string(),
    //             topic: "dummy_outgoing".to_string(),
    //             category: MessageCategory::OutGoing,
    //             message_type: dummy_ougoing_msg.to_string(),
    //             variables: vec!(
    //                 ref_pos
    //             ),
    //             variables_feedback: vec!(),
    //             variables_response: vec!()
    //         },
    //         Message {
    //             name: "DummyIncoming".to_string(),
    //             topic: "dummy_incoming".to_string(),
    //             category: MessageCategory::Incoming,
    //             message_type: dummy_incoming_msg.to_string(),
    //             variables: vec!(
    //                 act_pos
    //             ),
    //             variables_feedback: vec!(),
    //             variables_response: vec!()
    //         }
    //     )
    // };

    // Define the operations
    // ------------------------------------------------
    let mut operations = vec!();
    // 1. Robot move operations
    for pose in vec!("home", "rack_scanner", "rack_gripper", "rack_suction", "box_a", "box_b", "item_a", "item_b", "atr") {
        operations.push(
            Operation::new(
                &format!("op_robot_move_to_{pose}"), 
                // precondition
                t!(
                    // name
                    &format!("start_robot_move_to_{pose}").as_str(),
                    // planner guard
                    &format!("var:ur_action_trigger == false && var:ur_action_state == initial && ur_pose != {pose}").as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!("var:ur_command <- movej", "var:ur_action_trigger <- true", &format!("var:ur_goal_frame <- {pose}").as_str()),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
                // postcondition
                t!(
                    // name
                    &format!("complete_robot_move_to_{pose}").as_str(),
                    // planner guard
                    &format!("var:ur_action_state == done").as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!("var:ur_action_trigger <- false", &format!("var:ur_pose <- {pose}").as_str()),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
            )
        )
    }

    // 2. Gantry operations
    for pose in vec!("a", "b", "left", "right") {
        operations.push(
            Operation::new(
                &format!("op_gantry_move_to_{pose}"), 
                // precondition
                t!(
                    // name
                    &format!("start_gantry_move_to_{pose}").as_str(),
                    // planner guard
                    &format!("var:gantry_ref != {pose} && var:gantry_act != {pose} && var:ur_pose == home").as_str(),
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
                    &format!("var:gantry_act == {pose}").as_str(),
                    // planner actions
                    vec!(&format!("var:gantry_act <- {pose}").as_str()),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
            )
        )
    }
    
    // 3. Scanner operations
    for pose in vec!("a", "b") {
        operations.push(
            Operation::new(
                &format!("op_scan_box_{pose}"), 
                // precondition
                t!(
                    // name
                    &format!("start_scan_box_{pose}").as_str(),
                    // planner guard
                    &format!("\
                        var:ur_mounted == scanner && \
                        var:ur_pose == box_{pose} && \
                        var:gantry_act == {pose} && \
                        var:scanner_state == initial && \
                        var:scanner_trigger == false && \
                        var:scanned_{pose} == false"
                    ).as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!(&format!("var:scanner_trigger <- true").as_str()),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
                // postcondition
                t!(
                    // name
                    &format!("complete_scan_box_{pose}").as_str(),
                    // planner guard
                    &format!("var:scanner_state == done").as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!(&format!("var:scanned_{pose} <- true").as_str()),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
            )
        )
    }

    // 4. Gripper operation: pick item_a
    operations.push(
        Operation::new(
            &format!("op_pick_a"), 
            // precondition
            t!(
                // name
                &format!("start_pick_a").as_str(),
                // planner guard
                &format!("\
                    var:gripper_ref != closed && (var:gripper_act != closed || var:gripper_act != gripping) && \
                    var:scanned_a == true && \
                    var:ur_action_trigger == false && \
                    var:ur_action_state == initial && \
                    var:ur_pose == box_a && \
                    var:item_a_pose == box_a"
                ).as_str(),
                // runner guard
                "true",
                // planner actions
                vec!("var:ur_command <- pick_with_gripper", "var:ur_action_trigger <- true"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                &format!("complete_pick_a").as_str(),
                // planner guard
                &format!("var:ur_action_state == done").as_str(),
                // runner guard
                &format!("var:gripper_act == gripping").as_str(),
                // planner actions
                vec!("var:ur_action_trigger <- false", "var:gripper_act <- gripping", "var:item_a_pose <- gripper"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        )
    );

    // 5. Gripper operation: place item_a
    operations.push(
        Operation::new(
            &format!("op_place_a"), 
            // precondition
            t!(
                // name
                &format!("start_place_a").as_str(),
                // planner guard
                &format!("\
                    var:item_a_pose == gripper && \
                    var:ur_action_trigger == false && \
                    var:ur_action_state == initial && \
                    var:ur_pose == atr"
                ).as_str(),
                // runner guard
                "true",
                // planner actions
                vec!("var:ur_command <- place_with_gripper", "var:ur_action_trigger <- true"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                &format!("complete_place_a").as_str(),
                // planner guard
                &format!("var:ur_action_state == done").as_str(),
                // runner guard
                &format!("var:gripper_act == opened").as_str(),
                // planner actions
                vec!("var:ur_action_trigger <- false", "var:gripper_act <- opened", "var:item_a_pose <- atr"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        )
    );

    // 5. Robot mount tool operations
    for tool in vec!("scanner", "gripper", "suction") {
        operations.push(
            Operation::new(
                &format!("op_robot_mount_{tool}"), 
                // precondition
                t!(
                    // name
                    &format!("start_robot_mount_{tool}").as_str(),
                    // planner guard
                    &format!("var:ur_action_trigger == false && var:ur_action_state == initial && var:ur_pose == rack_{tool} && var:ur_mounted == none").as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!("var:ur_command <- mount", "var:ur_action_trigger <- true"),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
                // postcondition
                t!(
                    // name
                    &format!("complete_robot_mount_{tool}").as_str(),
                    // planner guard
                    &format!("var:ur_action_state == done").as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!("var:ur_action_trigger <- false", &format!("var:ur_mounted <- {tool}").as_str()),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
            )
        )
    }

    Model::new(
        "asdf",
        state.clone(),
        autos,
        operations,
        vec!()
    )
}