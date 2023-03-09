use micro_sp::*;

pub fn scan_grip_rob_model() -> (
    String,
    State,
    Vec<Transition>,
    Vec<Operation>,
    Vec<Resource>,
) {
    // Define the variables
    // -------------------------------------------------------

    // Scanner variables
    let scanner_request_trigger = bv_runner!("scanner_request_trigger");
    let fail_counter_scanner = iv_runner!("fail_counter_scanner");
    let scanner_request_state = v_runner!("scanner_request_state");
    
    // Gripper variables
    let gripper_request_trigger = bv_runner!("gripper_request_trigger");
    let gripper_command = v_runner!("gripper_command");
    let fail_counter_gripper = iv_runner!("fail_counter_gripper");
    let gripper_request_state = v_runner!("gripper_request_state");
    let gripper_actual_state = v_measured!("gripper_actual_state", vec!("opened", "closed", "gripping", "unknown"));

    // Estimated(memory) variables too keep track of the state
    let scanned_a = bv_estimated!("scanned_a");

    // Make a state and assign some values to variables
    // (usually, unknown is a safe initial value for measured and command variables)
    // ----------------------------------------------------
    let state = State::new();

    // Scanner variables
    let state = state.add(assign!(scanner_request_trigger, false.to_spvalue()));
    let state = state.add(assign!(scanner_request_state, "initial".to_spvalue()));
    let state = state.add(assign!(fail_counter_scanner, 0.to_spvalue()));
    let state = state.add(assign!(scanned_a, false.to_spvalue()));

    // Gripper variables
    let state = state.add(assign!(gripper_request_trigger, false.to_spvalue()));
    let state = state.add(assign!(gripper_request_state, "initial".to_spvalue()));
    let state = state.add(assign!(gripper_actual_state, "unknown".to_spvalue()));
    let state = state.add(assign!(fail_counter_gripper, 0.to_spvalue()));
    let state = state.add(assign!(gripper_command, "none".to_spvalue()));

    // And some mandatory variables (actually, these should be automatically
    // included when calling Model::new()...)
    let state = state.add(SPAssignment::new(
        v_runner!("runner_goal"),
        // "var:item_a_pose == atr && var:item_b_pose == atr".to_spvalue(),
        "var:scanned_a == true && var:gripper_actual_state == closed".to_spvalue(),
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
        iv_runner!("runner_replan_counter"),
        0.to_spvalue(),
    ));
    let state = state.add(SPAssignment::new(
        bv_runner!("runner_replan_trigger"),
        false.to_spvalue(),
    ));
    let state = state.add(SPAssignment::new(
        bv_runner!("runner_replanned"),
        false.to_spvalue(),
    ));

    let mut operations = vec![];

    // Scanner operation
    let op_scan_box_a = v_runner!("op_scan_box_a");
    let timestamp_op_scan_box_a = fv_runner!("timestamp_op_scan_box_a");
    let deadline_op_scan_box_a = fv_runner!("deadline_op_scan_box_a");
    let timeouts_op_scan_box_a = iv_runner!("timeouts_op_scan_box_a");
    let state = state.add(assign!(op_scan_box_a, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_scan_box_a, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_scan_box_a, 5.0.to_spvalue()));
    let state = state.add(assign!(timeouts_op_scan_box_a, 0.to_spvalue()));
    operations.push(Operation::new(
        &format!("op_scan_box_a"),
        // precondition
        t!(
            // name
            &format!("start_scan_box_a").as_str(),
            // planner guard
            "var:scanner_request_state == initial && var:scanner_request_trigger == false && var:scanned_a == false",
            // runner guard
            "true",
            // planner actions
            vec!("var:scanner_request_trigger <- true"),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
        // postcondition
        t!(
            // name
            &format!("complete_scan_box_a").as_str(),
            // planner guard
            "true",
            // runner guard
            &format!("var:scanner_request_state == succeeded").as_str(),
            // "true",
            // planner actions
            vec!(
                "var:scanner_request_trigger <- false",
                "var:scanner_request_state <- initial",
                &format!("var:scanned_a <- true").as_str()
            ),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
    ));

    // Open gripper operation - should fail if it can't open
    let op_open_gripper = v_runner!("op_open_gripper");
    let timestamp_op_open_gripper = fv_runner!("timestamp_op_open_gripper");
    let deadline_op_open_gripper = fv_runner!("deadline_op_open_gripper");
    let timeouts_op_open_gripper = iv_runner!("timeouts_op_open_gripper");
    let state = state.add(assign!(op_open_gripper, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_open_gripper, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_open_gripper, 5.0.to_spvalue()));
    let state = state.add(assign!(timeouts_op_open_gripper, 0.to_spvalue()));
    operations.push(Operation::new(
        &format!("op_open_gripper"),
        // precondition
        t!(
            // name
            &format!("start_open_gripper").as_str(),
            // planner guard
            "var:gripper_request_state == initial && var:gripper_request_trigger == false",
            // runner guard
            "true",
            // planner actions
            vec!("var: gripper_command <- open", "var:gripper_request_trigger <- true"),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
        // postcondition
        t!(
            // name
            &format!("complete_open_gripper").as_str(),
            // planner guard
            "true",
            // runner guard
            &format!("var:gripper_request_state == succeeded && var:gripper_actual_state == opened").as_str(),
            // "true",
            // planner actions
            vec!(
                "var:gripper_request_trigger <- false",
                "var:gripper_request_state <- initial",
                "var:gripper_actual_state <- opened"
            ),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
    ));

    // Close gripper operation - should fail if it can't completely close, which also means gripping
    let op_close_gripper = v_runner!("op_close_gripper");
    let timestamp_op_close_gripper = fv_runner!("timestamp_op_close_gripper");
    let deadline_op_close_gripper = fv_runner!("deadline_op_close_gripper");
    let timeouts_op_close_gripper = iv_runner!("timeouts_op_close_gripper");
    let state = state.add(assign!(op_close_gripper, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_close_gripper, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_close_gripper, 5.0.to_spvalue()));
    let state = state.add(assign!(timeouts_op_close_gripper, 0.to_spvalue()));
    operations.push(Operation::new(
        &format!("op_close_gripper"),
        // precondition
        t!(
            // name
            &format!("start_close_gripper").as_str(),
            // planner guard
            "var:gripper_request_state == initial && var:gripper_request_trigger == false && var:gripper_actual_state == opened",
            // runner guard
            "true",
            // planner actions
            vec!("var: gripper_command <- close", "var:gripper_request_trigger <- true"),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
        // postcondition
        t!(
            // name
            &format!("complete_close_gripper").as_str(),
            // planner guard
            "true",
            // runner guard
            &format!("var:gripper_request_state == succeeded && var:gripper_actual_state == closed").as_str(),
            // "true",
            // planner actions
            vec!(
                "var:gripper_request_trigger <- false",
                "var:gripper_request_state <- initial",
                "var:gripper_actual_state <- closed"
            ),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
    ));    

    // Define automatic transitions (these transitions will immediatelly
    // be executed if evaluated to be true)
    let mut auto_transitions: Vec<Transition> = vec![];

    // TODO: add replan or abort if timeout?

    auto_transitions.push(t!(
        // name
        "replan_if_scanner_failed",
        // planner guard
        "var:scanner_request_state == failed",
        // ruuner guard = none
        "true",
        // planner actions
        Vec::<&str>::new(),
        // runner actions - none
        vec!(
            "var:scanner_request_state <- initial",
            "var:scanner_request_trigger <- false",
            "var:runner_plan <- [unknown]",
            "var:runner_plan_current_step <- [unknown]",
            "var:runner_plan_status <- Waiting_for_the_re_plan",
            "var:runner_replan <- true"
            // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    auto_transitions.push(t!(
        // name
        "replan_if_gripper_failed",
        // planner guard
        "var:gripper_request_state == failed",
        // ruuner guard = none
        "true",
        // planner actions
        Vec::<&str>::new(),
        // runner actions - none
        vec!(
            "var:gripper_request_state <- initial",
            "var:gripper_request_trigger <- false",
            "var:runner_plan <- [unknown]",
            "var:runner_plan_current_step <- [unknown]",
            "var:runner_plan_status <- Waiting_for_the_re_plan",
            "var:runner_replan <- true"
            // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    auto_transitions.push(t!(
        // name
        "replan_if_gripper_cant_completely_close",
        // planner guard
        "var:op_close_gripper == executing && var:gripper_actual_state == gripping",
        // ruuner guard = none
        "true",
        // planner actions
        Vec::<&str>::new(),
        // runner actions - none
        vec!(
            "var:gripper_request_state <- initial",
            "var:gripper_request_trigger <- false",
            "var:runner_plan <- [unknown]",
            "var:runner_plan_current_step <- [unknown]",
            "var:runner_plan_status <- Waiting_for_the_re_plan",
            "var:runner_replan <- true"
            // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    (
        "scanner_model".to_string(),
        state.clone(),
        auto_transitions,
        operations,
        vec![],
    )
}

#[test]
fn test_operations() {
    let m = scan_grip_rob_model();

    let model = Model::new(&m.0, m.1, m.2, m.3, vec![]);

    let plan = bfs_operation_planner(
        model.state.clone(),
        crate::runner::ticker::extract_goal_from_state(&model.state.clone()),
        model.operations.clone(),
        10,
    );
    for p in plan.plan {
        println!("{}", p);
    }

    assert!(plan.found);
}