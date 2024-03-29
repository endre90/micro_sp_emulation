use micro_sp::*;
use serde_json::json;
use proptest::{prelude::*, test_runner::TestRunner};

use crate::runner::ticker::extract_goal_from_state;


// will use this in the case 2023 papers

// Resources:
// 1. Robot
// 2. Gantry
// 3. Scanner
// 4. Gripper
// 5. Suction tool
// 6. Localization algorithm -> Localization can just be started as soon as the items are scanned, 
// which means that going there and picking the items depends on a runner guards that items are in the scene
pub fn rita_model() -> (String, State, Vec<Transition>, Vec<Operation>, Vec<Operation>, Vec<Resource>) {

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
    let gantry_act = v_measured!("gantry_act", vec!("a", "b", "atr"));
    let gantry_ref = v_command!("gantry_ref", vec!("a", "b", "atr"));

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

    // // 5. Suction tool variables
    // let suction_trigger = bv_runner!("suction_trigger");
    // let suction_state = v_estimated!("suction_state", vec!("on", "off"));

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

    // // 5. Suction variables
    // let state = state.add(assign!(suction_trigger, false.to_spvalue()));
    // let state = state.add(assign!(suction_state, SPValue::Unknown));

    // 6. Localization variables
    let state = state.add(assign!(localization_trigger, false.to_spvalue()));
    let state = state.add(assign!(localization_state, SPValue::Unknown));

    // 7. Estimated positions of products
    let state = state.add(assign!(item_a_pose, "box_a".to_spvalue()));
    let state = state.add(assign!(item_b_pose, "box_b".to_spvalue()));
    
    // And some mandatory variables (actually, these should be automatically
    // included when calling Model::new()...)
        let state = state.add(SPAssignment::new(
        v_runner!("runner_goal"),
        // "var:item_a_pose == atr && var:item_b_pose == atr".to_spvalue(),
        "var:scanned_a == true".to_spvalue()
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

    // Define automatic transitions and operations (these will immediatelly 
    // be executed if evaluated to be true)
    let auto_transitions: Vec<Transition> = vec!();
    let auto_operations: Vec<Operation> = vec!();

    // might need this later for autogeneration but now we hardcode everything...
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
    for pose in vec!("home", "rack_scanner", "rack_gripper", "rack_suction", "box_a", "box_b", "atr", "item_a") {
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
                    &format!("var:ur_action_state == succeeded").as_str(),
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
    for pose in vec!("a", "b", "atr") {
        operations.push(
            Operation::new(
                &format!("op_gantry_move_to_{pose}"), 
                // precondition
                t!(
                    // name
                    &format!("start_gantry_move_to_{pose}").as_str(),
                    // planner guard
                    &format!("var:gantry_ref != {pose}").as_str(), // && var:ur_pose == home").as_str(),
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
                    &format!("var:scanner_state == succeeded").as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!("var:scanner_trigger <- false", &format!("var:scanned_{pose} <- true").as_str()),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
            )
        )
    }

    // 4. Gripper close operation
    operations.push(
        Operation::new(
            &format!("op_gripper_close"), 
            // precondition
            t!(
                // name
                "start_gripper_close",
                // planner guard
                "var:gripper_ref != closed",
                // runner guard
                "true",
                // planner actions
                vec!("var:gripper_ref <- closed"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                "complete_gripper_close",
                // planner guard
                "var:gripper_ref == closed",
                // runner guard
                "var:gripper_act == closed",
                // planner actions
                vec!("var:gripper_act <- closed"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        )
    );

    // 4. Gripper open operation
    operations.push(
        Operation::new(
            &format!("op_gripper_open"), 
            // precondition
            t!(
                // name
                "start_gripper_open",
                // planner guard
                "var:gripper_ref != opened",
                // runner guard
                "true",
                // planner actions
                vec!("var:gripper_ref <- opened"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                "complete_gripper_open",
                // planner guard
                "var:gripper_ref == opened",
                // runner guard
                "var:gripper_act == opened",
                // planner actions
                vec!("var:gripper_act <- opened"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        )
    );

    // 4. Gripper operation: pick item_a
    operations.push(
        Operation::new(
            &format!("op_pick_a"), 
            // precondition
            t!(
                // name
                &format!("start_pick_a").as_str(),
                // planner guard
                // maybe for gripper we don't need this: \ && (var:gripper_act != closed || var:gripper_act != gripping) && \
                &format!("\
                    var:ur_mounted == gripper && \
                    var:gripper_ref != closed && \
                    (var:gripper_act != closed && var:gripper_act != gripping) && \
                    var:scanned_a == true && \
                    var:ur_pose == item_a && \
                    var:item_a_pose == box_a"
                ).as_str(),
                // runner guard
                "true",
                // planner actions
                vec!("var:gripper_ref <- closed"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                &format!("complete_pick_a").as_str(),
                // planner guard
                &format!("var:gripper_ref == closed").as_str(),
                // runner guard
                &format!("var:gripper_act == gripping").as_str(),
                // planner actions
                vec!("var:gripper_act <- gripping", "var:item_a_pose <- gripper"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        )
    );

    // 4. Suction operation: pick item_b
    operations.push(
        Operation::new(
            &format!("op_pick_b"), 
            // precondition
            t!(
                // name
                &format!("start_pick_b").as_str(),
                // planner guard
                &format!("\
                    var:ur_mounted == suction && \
                    var:scanned_b == true && \
                    var:ur_action_trigger == false && \
                    var:ur_action_state == initial && \
                    var:ur_pose == box_b && \
                    var:item_b_pose == box_b"
                ).as_str(),
                // runner guard
                "true",
                // planner actions
                vec!("var:ur_command <- pick_with_suction", "var:ur_action_trigger <- true"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                &format!("complete_pick_b").as_str(),
                // planner guard
                &format!("var:ur_action_state == done").as_str(), // this can fail
                // runner guard
                "true",
                // planner actions
                vec!("var:ur_action_trigger <- false", "var:item_b_pose <- suction"),
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
                    var:ur_mounted == gripper && \
                    var:item_a_pose == gripper && \
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
                &format!("var:gripper_ref == opened").as_str(),
                // runner guard
                &format!("var:gripper_act == opened").as_str(),
                // planner actions
                vec!("var:gripper_act <- opened", "var:item_a_pose <- atr"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        )
    );

    // 5. Suction operation: place item_b
    operations.push(
        Operation::new(
            &format!("op_place_b"), 
            // precondition
            t!(
                // name
                &format!("start_place_b").as_str(),
                // planner guard
                &format!("\
                    var:ur_mounted == suction && \
                    var:item_b_pose == suction && \
                    var:ur_action_trigger == false && \
                    var:ur_action_state == initial && \
                    var:ur_pose == atr"
                ).as_str(),
                // runner guard
                "true",
                // planner actions
                vec!("var:ur_command <- place_with_suction", "var:ur_action_trigger <- true"),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                &format!("complete_place_b").as_str(),
                // planner guard
                &format!("var:ur_action_state == done").as_str(),
                // runner guard
                "true",
                // planner actions
                vec!("var:ur_action_trigger <- false", "var:item_b_pose <- atr"),
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
                    &format!("var:ur_action_state == succeeded").as_str(),
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

    // 5. Robot unmount tool operations
    for tool in vec!("scanner", "gripper", "suction") {
        operations.push(
            Operation::new(
                &format!("op_robot_unmount_{tool}"), 
                // precondition
                t!(
                    // name
                    &format!("start_robot_unmount_{tool}").as_str(),
                    // planner guard
                    &format!("var:ur_action_trigger == false && var:ur_action_state == initial && var:ur_pose == rack_{tool} && var:ur_mounted == {tool}").as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!("var:ur_command <- unmount", "var:ur_action_trigger <- true"),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
                // postcondition
                t!(
                    // name
                    &format!("complete_robot_unmount_{tool}").as_str(),
                    // planner guard
                    &format!("var:ur_action_state == succeeded").as_str(),
                    // runner guard
                    "true",
                    // planner actions
                    vec!("var:ur_action_trigger <- false", &format!("var:ur_mounted <- none").as_str()),
                    //runner actions
                    Vec::<&str>::new(),
                    &state
                ),
            )
        )
    }

    (
        "asdf".to_string(),
        state.clone(),
        auto_transitions,
        auto_operations,
        operations,
        vec!()
    )
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(10))]
    #[test]
    fn my_behavior_model_works(gantry_act_val in prop_oneof!("a", "b")) {

        let m = rita_model();
        // let model = Model::new(&m.0, m.1, m.2, m.3, m.4);
        // let gantry_act = v_measured!("gantry_act", vec!("a", "b", "atr"));
        let new_state = m.1.update("gantry_act", gantry_act_val.to_spvalue());

        let model = Model::new(
            "asdf",
            new_state.clone(),
            m.2,
            m.3,
            m.4,
            vec!()
        );

        let plan = bfs_operation_planner(model.state.clone(), extract_goal_from_state(&model.state.clone()), model.operations.clone(), 50);
        for p in plan.plan {
            println!("{}", p);
        }

        // let mut runner = TestRunner::default();
        // let config = ProptestConfig::with_cases(10); // Set the number of test cases to 10
        // runner.set_config(config);

        prop_assert!(plan.found);
        // prop_assert!(!model.is_empty());
        // prop_assert!(model.last_value().is_some());
    }
}

// adversary testing - look into this
// the adversasy can have different goals
// MCDC coverability in standard (coverability criteria)
// johans thesis -> use falsification to maximize mcdc
// Volvo: 80% of mcdc coverability goal
// as soon as I can translate the code to if-else statements (textual), I can talk about mcdc
// modified decision ... coverage - mcdc
// coverability criteria <- caught with mcdc
// this is the research question of the paper:
// how do I construct the adversary so that the requirement coverability criteria is fullfilled
// next bottleneck: come up with the correct specifications
// in implementation: monitor if the antecedent have beed true, and then if the whole implication has been true
// antecedent is the specification
// a nice conclusion (outcome of the paper) would be: compared to random testing, if we use a smart way of defining the adversary,
// we can have fewer simulations steps in order to have bigger coverabilty
// with the real resources (later), I would have to manually trigger the fails, i.e. fail the gripping
// maybe goal: from 10000 autogenerated test cases, I can have 100% coverability by choosing these 127 test cases