use micro_sp::*;
// use nanoid::nanoid;
// use crate::*;

// Operations that we need:
// Gantry: move, calibrate, lock, unlock
// Scanner: scan item //TODO
// Camera System: update blue boxes // TODO
// Robot: move, mount, unmount, pick, place
// SOP tryout: Before unmounting the tool, blink the gantry indicator 3 times for 1 seconds each

pub fn minimal_model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let auto_transitions = vec![];
    // let sops = vec![];
    let mut operations = vec![];

    let blink_on = Operation::new(
        "blink_on_for_sop",
        None,
        Some(2),
        None,
        false,
        Vec::from([Transition::parse(
            "start_blink",
            "true",
            "true",
            vec![&format!("var:gantry_light_indicator <- true")],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "start_blink",
            "true",
            "true",
            vec![],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([])
    );

    let blink_red = Operation::new(
        "blink_red_for_sop",
        None,
        Some(2),
        false,
        Vec::from([Transition::parse(
            "start_blink_red",
            "true",
            "true",
            vec![&format!("var:gantry_light_indicator <- true")],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "start_blink_red",
            "true",
            "true",
            vec![],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    );

    let blink_blue = Operation::new(
        "blink_blue_for_sop",
        None,
        Some(2),
        false,
        Vec::from([Transition::parse(
            "start_blink_blue",
            "true",
            "true",
            vec![&format!("var:gantry_light_indicator <- true")],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "start_blink_blue",
            "true",
            "true",
             
            vec![],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    );

    let blink_green = Operation::new(
        "blink_green_for_sop",
        None,
        Some(2),
        false,
        Vec::from([Transition::parse(
            "start_blink_green",
            "true",
            "true",
             
            vec![&format!("var:gantry_light_indicator <- true")],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "start_blink_green",
            "true",
            "true",
             
            vec![],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    );

    let blink_off = Operation::new(
        "blink_off_for_sop",
        None,
        Some(2),
        false,
        Vec::from([Transition::parse(
            "start_blink",
            "true",
            "true",
             
            Vec::<&str>::new(),
            vec![&format!("var:gantry_light_indicator <- false")],
            &state,
        )]),
        Vec::from([Transition::parse(
            "start_blink",
            "true",
            "true",
             
            vec![],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    );

    // This is the high level SOP
    let sop_blinking = Operation::new(
        "sop_blinking",
        None,
        Some(2),
        false,
        Vec::from([Transition::parse(
            "start_blinking",
            "var:robot_mounted_estimated == suction_tool",
            "true",
             
            vec![
                // should be the way as we control the robot... maybe
                &format!("var:{sp_id}_sop_enabled <- true"),
                &format!("var:{sp_id}_sop_state <- initial"),
                &format!("var:{sp_id}_sop_id <- blinker")
                ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "complete_blinking",
            "true",
            &format!("var:{sp_id}_sop_state == completed"),
             
            vec!(&format!("var:blinked <- true")),
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    );

    operations.push(sop_blinking);

    let gantry_unlock = Operation::new(
        "gantry_unlock",
        None,
        Some(3),
        false,
        Vec::from([Transition::parse(
            "start_gantry_unlock",
            "var:gantry_request_state == initial \
                && var:gantry_request_trigger == false",
            "true",
             
            vec![
                &format!("var:gantry_command_command <- unlock"),
                "var:gantry_request_trigger <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "complete_gantry_unlock",
            "true",
            "var:gantry_request_state == succeeded",
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_locked_estimated <- false",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "fail_gantry_unlock",
            "true",
            "var:gantry_request_state == failed",
             
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_locked_estimated <- UNKNOWN_bool",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    );

    let robot_move_to_x = Operation::new(
        &format!("robot_move_to_x"),
        None,
        Some(3),
        false,
        Vec::from([Transition::parse(
            &format!("start_robot_move_to_{}", "x"),
            "var:robot_request_state == initial \
            && var:robot_request_trigger == false \
            && var:gantry_locked_estimated == true \
            && var:gantry_calibrated_estimated == true",
            "true",
            vec![
                &format!("var:robot_command_command <- move"),
                &format!("var:robot_position_command <- x"),
                &format!("var:robot_speed_command <- 0.5"),
                "var:robot_request_trigger <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            &format!("complete_robot_move_to_{}", "x"),
            "true",
            &format!("var:robot_request_state == succeeded"),
             
            vec![
                "var:robot_request_trigger <- false",
                "var:robot_request_state <- initial",
                &format!("var:robot_position_estimated <- x"),
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            &format!("fail_robot_move_to_{}", "x"),
            "true",
            &format!("var:robot_request_state == failed"),
             
            vec![
                "var:robot_request_trigger <- false",
                "var:robot_request_state <- initial",
                &format!("var:robot_position_estimated <- UNKNOWN_string"),
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    );

    let sleep = Operation::new(
        &format!("sleep"),
        None,
        None,
        false,
        Vec::from([Transition::parse(
            &format!("start_sleep"),
            "var:micro_sp_time_request_state == initial \
            && var:micro_sp_time_request_trigger == false",
            "true",
            vec![
                &format!("var:micro_sp_time_request_trigger <- true"),
                &format!("var:micro_sp_time_duration_ms <- 3000"),
                &format!("var:micro_sp_time_command <- sleep"),
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            &format!("complete_sleep"),
            "true",
            &format!("var:micro_sp_time_request_state == succeeded"),
             
            vec![
                "var:micro_sp_time_request_trigger <- false",
                "var:micro_sp_time_request_state <- initial",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    );
    

    operations.push(gantry_unlock.clone());

    //new sop runner
    let sop = SOPStruct {
        id: "blinker".to_string(),
        sop: SOP::Sequence(vec!(
            SOP::Operation(Box::new(blink_on.clone())),
            SOP::Operation(Box::new(blink_off.clone())),
            SOP::Operation(Box::new(sleep.clone())),
            SOP::Operation(Box::new(blink_on.clone())),
            SOP::Operation(Box::new(blink_off.clone())),
            SOP::Operation(Box::new(blink_on.clone())),
            SOP::Operation(Box::new(blink_off.clone())),
            SOP::Parallel(vec![
                SOP::Operation(Box::new(blink_blue.clone())),
                SOP::Operation(Box::new(blink_red.clone())),
                SOP::Operation(Box::new(sleep.clone())),
                SOP::Operation(Box::new(blink_green.clone())),
                SOP::Operation(Box::new(gantry_unlock.clone())),
                SOP::Operation(Box::new(robot_move_to_x.clone())),
            ]),
            SOP::Operation(Box::new(blink_on.clone())),
            SOP::Operation(Box::new(blink_off.clone())),
        )),
    };

    let sops: Vec<SOPStruct> = vec![sop];

    operations.push(Operation::new(
        "op_gantry_lock",
        None,
        Some(3),
        false,
        Vec::from([Transition::parse(
            "start_gantry_lock",
            "var:gantry_request_state == initial \
                && var:gantry_request_trigger == false",
            "true",
             
            vec![
                &format!("var:gantry_command_command <- lock"),
                "var:gantry_request_trigger <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "complete_gantry_lock",
            "true",
            "var:gantry_request_state == succeeded",
             
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_locked_estimated <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "fail_gantry_lock",
            "true",
            "var:gantry_request_state == failed",
             
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_locked_estimated <- UNKNOWN_bool",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    ));

    
    

    operations.push(Operation::new(
        "op_gantry_calibrate",
        None,
        Some(3),
        false,
        Vec::from([Transition::parse(
            "start_gantry_calibrate",
            "var:gantry_locked_estimated == false \
                && var:gantry_request_state == initial \
                && var:gantry_request_trigger == false",
            "true",
             
            vec![
                &format!("var:gantry_command_command <- calibrate"),
                "var:gantry_request_trigger <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "complete_gantry_calibrate",
            "true",
            "var:gantry_request_state == succeeded",
             
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_calibrated_estimated <- true",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            "fail_gantry_calibrate",
            "true",
            "var:gantry_request_state == failed",
             
            vec![
                "var:gantry_request_trigger <- false",
                "var:gantry_request_state <- initial",
                "var:gantry_calibrated_estimated <- UNKNOWN_bool",
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    ));

    for pos in vec!["home", "pipe_blue_box", "plate_pipe_box"] {
        operations.push(Operation::new(
            &format!("gantry_move_to_{}", pos),
            None,
            Some(3),
            false,
            Vec::from([Transition::parse(
                &format!("start_gantry_move_to_{}", pos),
                "var:gantry_request_state == initial \
                    && var:gantry_request_trigger == false \
                    && var:gantry_locked_estimated == false \
                    && var:gantry_calibrated_estimated == true",
                "true",
                vec![
                    &format!("var:gantry_command_command <- move"),
                    &format!("var:gantry_position_command <- {pos}"),
                    &format!("var:gantry_speed_command <- 0.5"),
                    "var:gantry_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_gantry_move_to_{}", pos),
                "true",
                &format!("var:gantry_request_state == succeeded"),
                 
                vec![
                    "var:gantry_request_trigger <- false",
                    "var:gantry_request_state <- initial",
                    &format!("var:gantry_position_estimated <- {pos}"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("fail_gantry_move_to_{}", pos),
                "true",
                "var:gantry_request_state == failed",
                 
                vec![
                    "var:gantry_request_trigger <- false",
                    "var:gantry_request_state <- initial",
                    "var:gantry_position_estimated <- UNKNOWN_string",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    // for blue_box in vec!["pipe_blue_box", "plate_blue_box"] {
    //     operations.push(Operation::new(
    //         &format!("update_position_for_{}", blue_box),
    //         None,
    //         Some(3),
    //         Vec::from([Transition::parse(
    //             &format!("start_update_position_for_{}", blue_box),
    //             "var:camera_system_request_state == initial \
    //                 && var:camera_system_request_trigger == false",
    //             "true",
    //             vec![
    //                 &format!("var:camera_system_update_command <- {blue_box}"),
    //                 "var:camera_system_request_trigger <- true",
    //             ],
    //             Vec::<&str>::new(),
    //             &state,
    //         )),
    //         Vec::from([Transition::parse(
    //             &format!("complete_update_position_for_{}", blue_box),
    //             "true",
    //             &format!("var:camera_system_request_state == succeeded"),
    //             vec![
    //                 "var:camera_system_request_trigger <- false",
    //                 "var:camera_system_request_state <- initial",
    //                 &format!("var:{blue_box}_position_updated_estimated <- true"),
    //             ],
    //             Vec::<&str>::new(),
    //             &state,
    //         )),
    //         Vec::from([Transition::parse(
    //             &format!("fail_update_position_for_{}", blue_box),
    //             "true",
    //             &format!("var:camera_system_request_state == failed"),
    //             vec![
    //                 "var:camera_system_request_trigger <- false",
    //                 "var:camera_system_request_state <- initial",
    //                 &format!("var:{blue_box}_position_updated_estimated <- UNKNOWN"),
    //             ],
    //             Vec::<&str>::new(),
    //             &state,
    //         )),
    //         Transition::empty(),
    //     ));
    // }

    // for item in vec!["pipe", "plate"] {
    //     operations.push(Operation::new(
    //         &format!("scan_{}_blue_box", item),
    //         None,
    //         Some(3),
    //         Vec::from([Transition::parse(
    //             &format!("start_scan_{}_blue_box", item),
    //             &&format!(
    //                 "var:scanner_request_state == initial \
    //                 && var:scanner_request_trigger == false \
    //                 && var:gantry_position_estimated == {item}_blue_box \
    //                 && var:robot_position_estimated == {item}_blue_box"
    //             ),
    //             "true",
    //             vec![
    //                 &format!("var:scanner_item_command <- {item}"),
    //                 "var:scanner_request_trigger <- true",
    //             ],
    //             Vec::<&str>::new(),
    //             &state,
    //         ),
    //         Vec::from([Transition::parse(
    //             &format!("complete_scan_{}_blue_box", item),
    //             "true",
    //             &format!("var:scanner_request_state == succeeded"),
    //             vec![
    //                 "var:scanner_request_trigger <- false",
    //                 "var:scanner_request_state <- initial",
    //                 &format!("var:{item}_blue_box_scanned_estimated <- true"),
    //                 &format!("var:{item}_position_estimated <- {item}_blue_box"),
    //             ],
    //             Vec::<&str>::new(),
    //             &state,
    //         ),
    //         Vec::from([Transition::parse(
    //             &format!("fail_scan_{}_blue_box", item),
    //             "true",
    //             &format!("var:scanner_request_state == failed"),
    //             vec![
    //                 "var:scanner_request_trigger <- false",
    //                 "var:scanner_request_state <- initial",
    //                 &format!("var:{item}_blue_box_scanned_estimated <- UNKNOWN"),
    //                 &format!("var:{item}_position_estimated <- UNKNOWN"),
    //             ],
    //             Vec::<&str>::new(),
    //             &state,
    //         ),
    //         Transition::empty(),
    //     ));
    // }

    //     && var:gantry_locked_estimated == true \
    // && var:gantry_calibrated_estimated == true",

    for pos in vec![
        "a",
        "b",
        "c",
        "d",
        "pipe_blue_box",
        "plate_pipe_box",
        "gripper_tool_rack",
        "suction_tool_rack",
    ] {
        operations.push(Operation::new(
            &format!("robot_move_to_{}", pos),
            None,
            Some(3),
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_move_to_{}", pos),
                "var:robot_request_state == initial \
                && var:robot_request_trigger == false \
                && var:gantry_locked_estimated == true \
                && var:gantry_calibrated_estimated == true",
                "true",
                vec![
                    &format!("var:robot_command_command <- move"),
                    &format!("var:robot_position_command <- {pos}"),
                    &format!("var:robot_speed_command <- 0.5"),
                    "var:robot_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_robot_move_to_{}", pos),
                "true",
                &format!("var:robot_request_state == succeeded"),
                 
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_position_estimated <- {pos}"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("fail_robot_move_to_{}", pos),
                "true",
                &format!("var:robot_request_state == failed"),
                 
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_position_estimated <- UNKNOWN_string"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    for tool in vec!["gripper_tool", "suction_tool", "none", "unknown"] {
        operations.push(Operation::new(
            &format!("robot_check_for_{tool}_mounted"),
            None,
            Some(3),
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_check_for_{tool}_mounted"),
                &format!(
                    "(var:robot_mounted_checked == false || var:robot_mounted_checked == UNKNOWN_bool) \
                    && var:robot_request_state == initial \
                    && var:robot_request_trigger == false \
                    && var:robot_mounted_estimated == UNKNOWN_string"
                ),
                "true",
                vec![
                    &format!("var:robot_command_command <- check_mounted_tool"),
                    "var:robot_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([
                Transition::parse(
                    &format!("complete_robot_check_for_{tool}_mounted"),
                    "true",
                    &format!("var:robot_request_state == succeeded && var:robot_mounted_one_time_measured == {tool}"),
                    vec![
                        "var:robot_request_trigger <- false",
                        "var:robot_request_state <- initial",
                        "var:robot_mounted_checked <- true",
                        &format!("var:robot_mounted_estimated <- {tool}")
                    ],
                    Vec::<&str>::new(),
                    &state,
                ),
                Transition::parse(
                    &format!("complete_robot_check_for_{tool}_mounted_2"),
                    "true",
                    &format!("var:robot_request_state == succeeded && var:robot_mounted_one_time_measured != {tool}"),
                     
                    vec![
                        "var:robot_request_trigger <- false",
                        "var:robot_request_state <- initial",
                        "var:robot_mounted_checked <- true",
                        &format!("var:robot_mounted_estimated <- var:robot_mounted_one_time_measured"),
                        &format!("var:{sp_id}_replan_trigger <- true"),
                        &format!("var:{sp_id}_replanned <- false"),
                    ],
                    Vec::<&str>::new(),
                    &state,
                )
            ]),
            Vec::from([Transition::parse(
                &format!("fail_robot_check_mounted"),
                "true",
                &format!("var:robot_request_state == failed"),
                 
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_mounted_estimated <- UNKNOWN_string"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    for tool in vec!["gripper_tool", "suction_tool"] {
        operations.push(Operation::new(
            &format!("robot_mount_{}", tool),
            None,
            Some(3),
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_mount_{}", tool),
                &format!(
                    "var:robot_request_state == initial \
                    && var:robot_request_trigger == false \
                    && var:robot_position_estimated == {tool}_rack \
                    && var:robot_mounted_estimated == none \
                    && var:gantry_locked_estimated == true",
                ),
                "true",
                vec![
                    &format!("var:robot_command_command <- mount"),
                    "var:robot_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_robot_mount_{}", tool),
                "true",
                &format!("var:robot_request_state == succeeded"),
                 
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_mounted_estimated <- {tool}"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("fail_robot_mount_{}", tool),
                "true",
                &format!("var:robot_request_state == failed"),
                 
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_mounted_estimated <- UNKNOWN_string"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    for tool in vec!["gripper_tool", "suction_tool"] {
        operations.push(Operation::new(
            &format!("robot_unmount_{tool}"),
            None,
            Some(3),
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_unmount_{tool}"),
                &format!(
                    "var:robot_request_state == initial \
                    && var:robot_request_trigger == false \
                    && var:robot_position_estimated == {tool}_rack \
                    && var:robot_mounted_estimated == {tool} \
                    && var:gantry_locked_estimated == true"
                ),
                "true",
                 
                vec![
                    &format!("var:robot_command_command <- unmount"),
                    "var:robot_request_trigger <- true",
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("complete_robot_unmount_{tool}"),
                "true",
                &format!("var:robot_request_state == succeeded"),
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_mounted_estimated <- none"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([Transition::parse(
                &format!("fail_robot_unmount_{tool}"),
                "true",
                &format!("var:robot_request_state == failed"),
                vec![
                    "var:robot_request_trigger <- false",
                    "var:robot_request_state <- initial",
                    &format!("var:robot_mounted_estimated <- UNKNOWN_string"),
                ],
                Vec::<&str>::new(),
                &state,
            )]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    // auto_operations.push(Operation::new(
    //     &format!("robot_check_mounted"),
    //     None,
    //     Some(3),
    //     Vec::from([Transition::parse(
    //         &format!("start_robot_check_mounted"),
    //         &format!(
    //             "var:robot_request_state == initial \
    //             && var:robot_request_trigger == false \
    //             && var:robot_mounted_estimated == UNKNOWN"
    //         ),
    //         "true",
    //         vec![
    //             &format!("var:robot_command_command <- check_mounted_tool"),
    //             "var:robot_request_trigger <- true",
    //         ],
    //         Vec::<&str>::new(),
    //         &state,
    //     ),
    //     Vec::from([Transition::parse(
    //         &format!("complete_robot_check_mounted"),
    //         "true",
    //         &format!("var:robot_request_state == succeeded"),
    //         vec![
    //             "var:robot_request_trigger <- false",
    //             "var:robot_request_state <- initial",
    //             &format!("var:robot_mounted_estimated <- var:robot_mounted_one_time_measured"),
    //         ],
    //         Vec::<&str>::new(),
    //         &state,
    //     ),
    //     Vec::from([Transition::parse(
    //         &format!("fail_robot_check_mounted"),
    //         "true",
    //         &format!("var:robot_request_state == failed"),
    //         vec![
    //             "var:robot_request_trigger <- false",
    //             "var:robot_request_state <- initial",
    //             &format!("var:robot_mounted_estimated <- UNKNOWN"),
    //         ],
    //         Vec::<&str>::new(),
    //         &state,
    //     ),
    //     Transition::empty(),
    // ));

    // // reason enough to have automatic operations?
    // auto_transitions.push(Vec::from([Transition::parse(
    //     "start_robot_check_mounted_tool",
    //     "true",
    //     "var:robot_mounted_estimated == UNKNOWN \
    //         && var:robot_request_state == initial \
    //         && var:robot_request_trigger == false",
    //     Vec::<&str>::new(),
    //     Vec::from([
    //         "var:robot_request_trigger <- true",
    //         "var:robot_command_command <- check_mounted_tool", // non-deterministic outcome
    //     ),
    //     &state
    // ));

    // auto_transitions.push(Vec::from([Transition::parse(
    //     "complete_robot_check_mounted_tool",
    //     "true",
    //     "var:robot_mounted_estimated == UNKNOWN \
    //         && var:robot_check_mounted_tool == true \
    //         && var:robot_request_state == initial \
    //         && var:robot_request_trigger == false",
    //     Vec::<&str>::new(),
    //     Vec::from([
    //         "var:robot_request_state <- initial",
    //         "var:robot_request_trigger <- true",
    //         "var:robot_command_command <- check_mounted_tool", // non-deterministic outcome
    //     ),
    //     &state
    // ));

    // TODO: An automatic transition or operation that automatically updates
    // the positions of boxes every minute or so or when updated_boxes is false

    let model = Model::new(sp_id, auto_transitions, vec!(), sops, operations);

    (model, state)
}

// #[test]
// fn test_model() {
//     let state = crate::models::minimal::state::state();

//     // Add the variables that keep track of the runner state
//     let runner_vars = generate_runner_state_variables("minimal");
//     let state = state.extend(runner_vars, true);

//     let (model, state) = crate::models::minimal::model::minimal_model("minimal", &state);
//     // let name = model.clone().name;

//     let vars = generate_operation_state_variables(&model, false);
//     let state = state.extend(vars, true);

//     for s in &state.state {
//         println!("{:?}", s.1);
//     }

//     let goal = state.get_value(&format!("{}_goal", model.name));
//     let val = state.get_value("gantry_position_estimated");
//     println!("Current goal: {:?}", goal);
//     println!("Current value: {:?}", val);

//     let state = state.update(
//         &format!("{}_goal", model.name),
//         "var:robot_position_estimated == b".to_spvalue(),
//     );
//     let goal = state.get_value(&format!("{}_goal", model.name));
//     let val = state.get_value("gantry_position_estimated");
//     println!("Current goal: {:?}", goal);
//     println!("Current value: {:?}", val);

//     // let extracted_goal = extract_goal_from_state(name, state)

//     let plan = bfs_operation_planner(
//         state.clone(),
//         state.extract_goal(&model.name),
//         model.operations.clone(),
//         3
//     );

//     let val = state.get_value("gantry_position_estimated");
//     println!("Current goal: {:?}", goal);
//     println!("Current value: {:?}", val);

//     println!("{:?}", plan);

//     assert!(plan.found);
// }
