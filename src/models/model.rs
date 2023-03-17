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
    let gripper_actual_state = v_estimated!(
        "gripper_actual_state",
        vec!("opened", "closed", "gripping", "unknown")
    );

    // Gantry variables
    let gantry_request_trigger = bv_estimated!("gantry_request_trigger");
    let gantry_command = v_runner!("gantry_command");
    let fail_counter_gantry = iv_runner!("fail_counter_gantry");
    let gantry_request_state = v_runner!("gantry_request_state");
    let gantry_actual_state = v_estimated!(
        "gantry_actual_state",
        vec!("box_a", "box_b", "agv", "unknown")
    );

    // Robot variables
    let robot_request_trigger = bv_estimated!("robot_request_trigger");
    let robot_command = v_runner!("robot_command");
    let robot_position = v_runner!("robot_position");
    let fail_counter_robot = iv_runner!("fail_counter_robot");
    let robot_request_state = v_runner!("robot_request_state");
    let robot_actual_state = v_estimated!(
        "robot_actual_state",
        vec!(
            "home",
            "toolbox_gripper",
            "toolbox_scanner",
            "box_a",
            "box_b",
            "item_a",
            "item_b",
            "agv",
            "unknown"
        )
    );

    // Estimated(memory) variables too keep track of the state
    let scanned_a = bv_estimated!("scanned_a");
    let scanned_b = bv_estimated!("scanned_b");
    let mounted = v_estimated!("mounted", vec!("gripper", "scanner", "none"));

    // Make a state and assign some values to variables
    // (usually, unknown is a safe initial value for measured and command variables)
    // ----------------------------------------------------
    let state = State::new();

    // Scanner variables
    let state = state.add(assign!(scanner_request_trigger, false.to_spvalue()));
    let state = state.add(assign!(scanner_request_state, "initial".to_spvalue()));
    let state = state.add(assign!(fail_counter_scanner, 0.to_spvalue()));
    let state = state.add(assign!(scanned_a, false.to_spvalue()));
    let state = state.add(assign!(scanned_b, false.to_spvalue()));
    let state = state.add(assign!(mounted, "none".to_spvalue()));

    // Gripper variables
    let state = state.add(assign!(gripper_request_trigger, false.to_spvalue()));
    let state = state.add(assign!(gripper_request_state, "initial".to_spvalue()));
    let state = state.add(assign!(gripper_actual_state, "unknown".to_spvalue()));
    let state = state.add(assign!(fail_counter_gripper, 0.to_spvalue()));
    let state = state.add(assign!(gripper_command, "none".to_spvalue()));

    // Gantry variables
    let state = state.add(assign!(gantry_request_trigger, false.to_spvalue()));
    let state = state.add(assign!(gantry_request_state, "initial".to_spvalue()));
    let state = state.add(assign!(gantry_actual_state, "unknown".to_spvalue()));
    let state = state.add(assign!(fail_counter_gantry, 0.to_spvalue()));
    let state = state.add(assign!(gantry_command, "none".to_spvalue()));

    // Robot variables
    let state = state.add(assign!(robot_request_trigger, false.to_spvalue()));
    let state = state.add(assign!(robot_request_state, "initial".to_spvalue()));
    let state = state.add(assign!(robot_actual_state, "unknown".to_spvalue()));
    let state = state.add(assign!(fail_counter_robot, 0.to_spvalue()));
    let state = state.add(assign!(robot_command, "none".to_spvalue()));
    let state = state.add(assign!(robot_position, "unknown".to_spvalue()));

    // And some mandatory variables (actually, these should be automatically
    // included when calling Model::new()...)
    let state = state.add(SPAssignment::new(
        v_runner!("runner_goal"),
        SPValue::Unknown, // now we can inject from the tester
                          // "var:scanned_a == true && var:gripper_actual_state == closed".to_spvalue(),
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
    let disabled_op_scan_box_a = iv_runner!("disabled_op_scan_box_a");
    let executing_op_scan_box_a = iv_runner!("executing_op_scan_box_a");
    let state = state.add(assign!(op_scan_box_a, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_scan_box_a, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_scan_box_a, 1.0.to_spvalue()));
    let state = state.add(assign!(timedout_op_scan_box_a, 0.to_spvalue()));
    let state = state.add(assign!(started_op_scan_box_a, 0.to_spvalue()));
    let state = state.add(assign!(completed_op_scan_box_a, 0.to_spvalue()));
    let state = state.add(assign!(disabled_op_scan_box_a, 0.to_spvalue()));
    let state = state.add(assign!(executing_op_scan_box_a, 0.to_spvalue()));
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
    let disabled_op_open_gripper = iv_runner!("disabled_op_open_gripper");
    let executing_op_open_gripper = iv_runner!("executing_op_open_gripper");
    let state = state.add(assign!(op_open_gripper, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_open_gripper, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_open_gripper, 1.0.to_spvalue()));
    let state = state.add(assign!(timedout_op_open_gripper, 0.to_spvalue()));
    let state = state.add(assign!(started_op_open_gripper, 0.to_spvalue()));
    let state = state.add(assign!(completed_op_open_gripper, 0.to_spvalue()));
    let state = state.add(assign!(disabled_op_open_gripper, 0.to_spvalue()));
    let state = state.add(assign!(executing_op_open_gripper, 0.to_spvalue()));
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
            vec!(
                "var: gripper_command <- open",
                "var:gripper_request_trigger <- true"
            ),
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
            &format!(
                "var:gripper_request_state == succeeded && var:gripper_actual_state == opened"
            )
            .as_str(),
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
    let disabled_op_close_gripper = iv_runner!("disabled_op_close_gripper");
    let executing_op_close_gripper = iv_runner!("executing_op_close_gripper");
    let state = state.add(assign!(op_close_gripper, "initial".to_spvalue()));
    let state = state.add(assign!(timestamp_op_close_gripper, 0.0.to_spvalue()));
    let state = state.add(assign!(deadline_op_close_gripper, 1.0.to_spvalue()));
    let state = state.add(assign!(timedout_op_close_gripper, 0.to_spvalue()));
    let state = state.add(assign!(started_op_close_gripper, 0.to_spvalue()));
    let state = state.add(assign!(completed_op_close_gripper, 0.to_spvalue()));
    let state = state.add(assign!(disabled_op_close_gripper, 0.to_spvalue()));
    let state = state.add(assign!(executing_op_close_gripper, 0.to_spvalue()));
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

    // uuuugh hackady hack
    let pos = "box_a";
    let state = state.add(assign!(
        v_runner!(&format!("op_gantry_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_gantry_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_gantry_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "box_b";
    let state = state.add(assign!(
        v_runner!(&format!("op_gantry_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_gantry_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_gantry_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "agv";
    let state = state.add(assign!(
        v_runner!(&format!("op_gantry_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_gantry_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_gantry_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_gantry_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    // Gantry operations
    for pos in vec!["box_a", "box_b", "agv"] {
        operations.push(Operation::new(
            &format!("op_gantry_move_to_{}", pos),
            // precondition
            t!(
                // name
                &format!("start_gantry_move_to_{}", pos).as_str(),
                // planner guard
                "var:gantry_request_state == initial && var:gantry_request_trigger == false",
                // runner guard
                "true",
                // planner actions
                vec!(
                    format!("var: gantry_command <- {pos}").as_str(),
                    "var:gantry_request_trigger <- true"
                ),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                &format!("complete_gantry_move_to_{}", pos).as_str(),
                // planner guard
                "true",
                // runner guard
                &format!(
                    "var:gantry_request_state == succeeded && var:gantry_actual_state == {pos}"
                )
                .as_str(),
                // "true",
                // planner actions
                vec!(
                    "var:gantry_request_trigger <- false",
                    "var:gantry_request_state <- initial",
                    &format!("var:gantry_actual_state <- {pos}")
                ),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        ));
    }

    // horrible I should be ashamed
    let pos = "home";
    let state = state.add(assign!(
        v_runner!(&format!("op_robot_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_robot_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_robot_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "toolbox_gripper";
    let state = state.add(assign!(
        v_runner!(&format!("op_robot_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_robot_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_robot_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "toolbox_scanner";
    let state = state.add(assign!(
        v_runner!(&format!("op_robot_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_robot_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_robot_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "box_a";
    let state = state.add(assign!(
        v_runner!(&format!("op_robot_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_robot_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_robot_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "box_b";
    let state = state.add(assign!(
        v_runner!(&format!("op_robot_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_robot_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_robot_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "item_a";
    let state = state.add(assign!(
        v_runner!(&format!("op_robot_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_robot_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_robot_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "item_b";
    let state = state.add(assign!(
        v_runner!(&format!("op_robot_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_robot_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_robot_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    let pos = "agv";
    let state = state.add(assign!(
        v_runner!(&format!("op_robot_move_to_{pos}").as_str()),
        "initial".to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("timestamp_op_robot_move_to_{pos}").as_str()),
        0.0.to_spvalue()
    ));
    let state = state.add(assign!(
        fv_runner!(&format!("deadline_op_robot_move_to_{pos}").as_str()),
        1.0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("timedout_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("started_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("completed_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("disabled_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));
    let state = state.add(assign!(
        iv_runner!(&format!("executing_op_robot_move_to_{pos}").as_str()),
        0.to_spvalue()
    ));

    // Robot move operations
    for pos in vec!(
        "home",
        "toolbox_gripper",
        "toolbox_scanner",
        "box_a",
        "box_b",
        "item_a",
        "item_b",
        "agv"
    ) {
        operations.push(Operation::new(
            &format!("op_robot_move_to_{}", pos),
            // precondition
            t!(
                // name
                &format!("start_robot_move_to_{}", pos).as_str(),
                // planner guard
                "var:robot_request_state == initial && var:robot_request_trigger == false",
                // runner guard
                "true",
                // planner actions
                vec!(
                    format!("var: robot_position <- {pos}").as_str(),
                    "var:robot_command <- move",
                    "var:robot_request_trigger <- true"
                ),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
            // postcondition
            t!(
                // name
                &format!("complete_robot_move_to_{}", pos).as_str(),
                // planner guard
                "true",
                // runner guard
                &format!(
                    "var:robot_request_state == succeeded && var:robot_actual_state == {pos}"
                )
                .as_str(),
                // "true",
                // planner actions
                vec!(
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_actual_state <- {pos}")
                ),
                //runner actions
                Vec::<&str>::new(),
                &state
            ),
        ));
    }

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
            "var:runner_replan <- true" // "var:runner_replan_trigger <- true"
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
            "var:runner_replan <- true" // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    let taken_auto_replan_if_gripper_cant_completely_close =
        iv_runner!("taken_auto_replan_if_gripper_cant_completely_close");
    let state = state.add(assign!(
        taken_auto_replan_if_gripper_cant_completely_close,
        0.to_spvalue()
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
            "var:runner_plan_info <- Waiting_for_the_re_plan",
            "var:runner_replan <- true" // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    let taken_auto_abort_planning_if_scanning_timedout_5_times =
        iv_runner!("taken_auto_abort_if_scanning_timedout_5_times");
    let state = state.add(assign!(
        taken_auto_abort_planning_if_scanning_timedout_5_times,
        0.to_spvalue()
    ));
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
            "var:runner_replanned <- false",
            "var:timedout_op_scan_box_a <- 1" // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    let taken_auto_replan_if_gantry_failed = iv_runner!("taken_auto_replan_if_gantry_failed");
    let state = state.add(assign!(taken_auto_replan_if_gantry_failed, 0.to_spvalue()));
    auto_transitions.push(t!(
        // name
        "replan_if_gantry_failed",
        // planner guard
        "var:gantry_request_state == failed",
        // ruuner guard = none
        "true",
        // planner actions
        Vec::<&str>::new(),
        // runner actions - none
        vec!(
            "var:gantry_request_state <- initial",
            "var:gantry_request_trigger <- false",
            "var:runner_plan <- [unknown]",
            "var:runner_plan_current_step <- [unknown]",
            "var:runner_plan_info <- Waiting_for_the_re_plan",
            "var:runner_replan <- true" // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    let taken_auto_replan_if_robot_failed = iv_runner!("taken_auto_replan_if_robot_failed");
    let state = state.add(assign!(taken_auto_replan_if_robot_failed, 0.to_spvalue()));
    auto_transitions.push(t!(
        // name
        "replan_if_robot_failed",
        // planner guard
        "var:robot_request_state == failed",
        // ruuner guard = none
        "true",
        // planner actions
        Vec::<&str>::new(),
        // runner actions - none
        vec!(
            "var:robot_request_state <- initial",
            "var:robot_request_trigger <- false",
            "var:runner_plan <- [unknown]",
            "var:runner_plan_current_step <- [unknown]",
            "var:runner_plan_info <- Waiting_for_the_re_plan",
            "var:runner_replan <- true" // "var:runner_replan_trigger <- true"
        ),
        &state
    ));

    let state = state.add(assign!(
        iv_runner!(&format!("nr_autos").as_str()),
        (auto_transitions.len() as i32).to_spvalue()
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
