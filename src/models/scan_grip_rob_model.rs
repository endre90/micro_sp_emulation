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
    let scanner_request_trigger = bv_estimated!("scanner_request_trigger");
    let fail_counter_scanner = iv_runner!("fail_counter_scanner");
    let scanner_request_state = v_runner!("scanner_request_state");
    
    // Gripper variables
    let gripper_request_trigger = bv_estimated!("gripper_request_trigger");
    let gripper_command = v_runner!("gripper_command");
    let fail_counter_gripper = iv_runner!("fail_counter_gripper");
    let gripper_request_state = v_runner!("gripper_request_state");
    let gripper_actual_state = v_estimated!("gripper_actual_state", vec!("opened", "closed", "gripping", "unknown"));

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
        v_runner!("runner_plan_info"),
        SPValue::Unknown,
    ));
    let state = state.add(SPAssignment::new(
        v_runner!("runner_plan_state"),
        "empty".to_spvalue(),
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
    let timedout_op_scan_box_a = iv_runner!("timedout_op_scan_box_a");
    let started_op_scan_box_a = iv_runner!("started_op_scan_box_a");
    let completed_op_scan_box_a = iv_runner!("completed_op_scan_box_a");
    let waiting_to_start_op_scan_box_a = iv_runner!("waiting_to_start_op_scan_box_a");
    let waiting_to_complete_op_scan_box_a = iv_runner!("waiting_to_complete_op_scan_box_a");
    let state = state.add(assign!(op_scan_box_a, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_scan_box_a, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_scan_box_a, 1.0.to_spvalue()));
    let state = state.add(assign!(timedout_op_scan_box_a, 0.to_spvalue()));
    let state = state.add(assign!(started_op_scan_box_a, 0.to_spvalue()));
    let state = state.add(assign!(completed_op_scan_box_a, 0.to_spvalue()));
    let state = state.add(assign!(waiting_to_start_op_scan_box_a, 0.to_spvalue()));
    let state = state.add(assign!(waiting_to_complete_op_scan_box_a, 0.to_spvalue()));
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
    let timedout_op_open_gripper = iv_runner!("timedout_op_open_gripper");
    let started_op_open_gripper = iv_runner!("started_op_open_gripper");
    let completed_op_open_gripper = iv_runner!("completed_op_open_gripper");
    let waiting_to_start_op_open_gripper = iv_runner!("waiting_to_start_op_open_gripper");
    let waiting_to_complete_op_open_gripper = iv_runner!("waiting_to_complete_op_open_gripper");
    let state = state.add(assign!(op_open_gripper, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_open_gripper, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_open_gripper, 1.0.to_spvalue()));
    let state = state.add(assign!(timedout_op_open_gripper, 0.to_spvalue()));
    let state = state.add(assign!(started_op_open_gripper, 0.to_spvalue()));
    let state = state.add(assign!(completed_op_open_gripper, 0.to_spvalue()));
    let state = state.add(assign!(waiting_to_start_op_open_gripper, 0.to_spvalue()));
    let state = state.add(assign!(waiting_to_complete_op_open_gripper, 0.to_spvalue()));
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
    let timedout_op_close_gripper = iv_runner!("timedout_op_close_gripper");
    let started_op_close_gripper = iv_runner!("started_op_close_gripper");
    let completed_op_close_gripper = iv_runner!("completed_op_close_gripper");
    let waiting_to_start_op_close_gripper = iv_runner!("waiting_to_start_op_close_gripper");
    let waiting_to_complete_op_close_gripper = iv_runner!("waiting_to_complete_op_close_gripper");
    let state = state.add(assign!(op_close_gripper, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_close_gripper, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_close_gripper, 1.0.to_spvalue()));
    let state = state.add(assign!(timedout_op_close_gripper, 0.to_spvalue()));
    let state = state.add(assign!(started_op_close_gripper, 0.to_spvalue()));
    let state = state.add(assign!(completed_op_close_gripper, 0.to_spvalue()));
    let state = state.add(assign!(waiting_to_start_op_close_gripper, 0.to_spvalue()));
    let state = state.add(assign!(waiting_to_complete_op_close_gripper, 0.to_spvalue()));
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

    // let mut state = state.clone(); 
    // for op in vec!("op_scan_box_a", "op_open_gripper", "op_close_gripper") {
    //     // let op_close_gripper = v_runner!("op_close_gripper");
    //     // let timestamp_op_close_gripper = fv_runner!("timestamp_op_close_gripper");
    //     // let deadline_op_close_gripper = fv_runner!("deadline_op_close_gripper");
    //     // let timedout_op_close_gripper = iv_runner!("timedout_op_close_gripper");
    //     // let started_op_close_gripper = iv_runner!("started_op_close_gripper");
    //     // let completed_op_close_gripper = iv_runner!("completed_op_close_gripper");
    //     let state = state.add(assign!(v_runner!(op), "initial".to_spvalue()));
    //     let state = state.add(assign!(timestamp_op_close_gripper, 0.0.to_spvalue()));
    //     let state = state.add(assign!(deadline_op_close_gripper, 1.0.to_spvalue()));
    //     let state = state.add(assign!(timedout_op_close_gripper, 0.to_spvalue()));
    //     let state = state.add(assign!(started_op_close_gripper, 0.to_spvalue()));
    //     state = state.add(assign!(completed_op_close_gripper, 0.to_spvalue()));
    // }

    // Define automatic transitions (these transitions will immediatelly
    // be executed if evaluated to be true)
    let mut auto_transitions: Vec<Transition> = vec![];

    // TODO: add replan or abort if timeout?

    let taken_auto_replan_if_scanner_failed = iv_runner!("taken_auto_replan_if_scanner_failed");
    let state = state.add(assign!(taken_auto_replan_if_scanner_failed, 0.to_spvalue()));
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
            "var:runner_plan_info <- Waiting_for_the_re_plan",
            "var:runner_replan <- true"
            // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    let taken_auto_replan_if_gripper_failed = iv_runner!("taken_auto_replan_if_gripper_failed");
    let state = state.add(assign!(taken_auto_replan_if_gripper_failed, 0.to_spvalue()));
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
            "var:runner_plan_info <- Waiting_for_the_re_plan",
            "var:runner_replan <- true"
            // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    let taken_auto_replan_if_gripper_cant_completely_close = iv_runner!("taken_auto_replan_if_gripper_cant_completely_close");
    let state = state.add(assign!(taken_auto_replan_if_gripper_cant_completely_close, 0.to_spvalue()));
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
            "var:runner_plan_info <- Waiting_for_the_re_plan",
            "var:runner_replan <- true"
            // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    let taken_auto_abort_planning_if_scanning_timedout_5_times = iv_runner!("taken_auto_abort_if_scanning_timedout_5_times");
    let state = state.add(assign!(taken_auto_abort_planning_if_scanning_timedout_5_times, 0.to_spvalue()));
    auto_transitions.push(t!(
        // name
        "abort_if_scanning_timedout_5_times",
        // planner guard
        "var:timedout_op_scan_box_a == 5 && var:runner_plan_state != aborted",
        // ruuner guard = none
        "true",
        // planner actions
        Vec::<&str>::new(),
        // runner actions - none
        vec!(
            "var:runner_plan <- [unknown]",
            "var:runner_plan_current_step <- [unknown]",
            "var:runner_plan_info <- Aborted_due_to_timeout",
            "var:runner_plan_state <- aborted",
            "var:runner_replan <- false",
            "var:runner_replanned <- false"
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