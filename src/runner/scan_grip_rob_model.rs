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
    let scanner_fail_counter = iv_runner!("scanner_fail_counter");
    let scanner_state = v_runner!("scanner_state");
    
    // Gripper variables
    let gripper_request_trigger = bv_runner!("gripper_request_trigger");
    let gripper_command = v_runner!("gripper_command");
    let gripper_fail_counter = iv_runner!("gripper_fail_counter");
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
    let state = state.add(assign!(scanner_state, "initial".to_spvalue()));
    let state = state.add(assign!(scanner_fail_counter, 0.to_spvalue()));
    let state = state.add(assign!(scanned_a, false.to_spvalue()));

    // Gripper variables
    let state = state.add(assign!(gripper_request_trigger, false.to_spvalue()));
    let state = state.add(assign!(gripper_request_state, "initial".to_spvalue()));
    let state = state.add(assign!(gripper_actual_state, "unknown".to_spvalue()));
    let state = state.add(assign!(gripper_fail_counter, 0.to_spvalue()));
    let state = state.add(assign!(gripper_command, "none".to_spvalue()));

    // hack to keep tracl of operations - but really fix this later, remove the automatic adding
    let op_close_gripper_state = v_runner!("op_close_gripper_state");
    let state = state.add(assign!(op_close_gripper_state, "initial".to_spvalue()));

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
        bv_runner!("runner_replan_trigger"),
        false.to_spvalue(),
    ));
    let state = state.add(SPAssignment::new(
        bv_runner!("runner_replanned"),
        false.to_spvalue(),
    ));

    // Define automatic transitions (these transitions will immediatelly
    // be executed if evaluated to be true)
    let mut auto_transitions: Vec<Transition> = vec![];

    auto_transitions.push(t!(
        // name
        "replan_if_scanner_failed",
        // planner guard
        "var:scanner_state == failed",
        // ruuner guard = none
        "true",
        // planner actions
        Vec::<&str>::new(),
        // runner actions - none
        vec!(
            "var:scanner_state <- initial",
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
        "var:op_close_gripper_state == executing && var:gripper_actual_state == gripping",
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

    let mut operations = vec![];

    // Scanner operation
    let op_scan_box_a = v_runner!("op_scan_box_a");
    let state = state.add(assign!(op_scan_box_a, "initial".to_spvalue()));
    operations.push(Operation::new(
        &format!("op_scan_box_a"),
        // precondition
        t!(
            // name
            &format!("start_scan_box_a").as_str(),
            // planner guard
            "var:scanner_state == initial && var:scanner_request_trigger == false && var:scanned_a == false",
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
            &format!("var:scanner_state == succeeded").as_str(),
            // "true",
            // planner actions
            vec!(
                "var:scanner_request_trigger <- false",
                "var:scanner_state <- initial",
                &format!("var:scanned_a <- true").as_str()
            ),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
    ));

    // Open gripper operation - should fail if it can't open
    let op_open_gripper = v_runner!("op_open_gripper");
    let state = state.add(assign!(op_open_gripper, "initial".to_spvalue()));
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
    let state = state.add(assign!(op_close_gripper, "initial".to_spvalue()));
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
            vec!("var: gripper_command <- close", "var:gripper_request_trigger <- true", "var:op_close_gripper_state <- executing"),
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
                "var:gripper_actual_state <- closed",
                "var:op_close_gripper_state <- initial"
            ),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
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