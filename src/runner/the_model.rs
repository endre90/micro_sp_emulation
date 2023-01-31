// // use micro_sp::{
// //     av_run, bv_run, fv, pred_parser, t, v, v_run, Action, Predicate, SPAssignment, SPValueType,
// //     SPVariable, SPVariableType, State, ToSPValue, Transition, Operation, OperationModel, bv,
// // };
// use micro_sp::*;

// // pub fn make_initial_state() -> State {
// //     let state = State::new();
// //     let state = state.add(SPAssignment::new(
// //         v_run!("runner_goal"),
// //         "var:ur_current_pose == c".to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         av_run!("runner_plan"),
// //         Vec::<String>::new().to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         v_run!("runner_plan_status"),
// //         "unknown".to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         iv_run!("runner_plan_current_step"),
// //         (-1).to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         bv_run!("runner_replan"),
// //         true.to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         bv_run!("runner_replanned"),
// //         false.to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         bv!("ur_action_trigger"),
// //         false.to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         v!("ur_action_state", vec!("initial", "executing", "done")),
// //         "initial".to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         v!("ur_current_pose", vec!("a", "b", "c", "d")),
// //         "a".to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         v!("ur_command", vec!("movej", "movel")),
// //         "movej".to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         fv_run!("ur_velocity"),
// //         0.2.to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         fv_run!("ur_acceleration"),
// //         0.4.to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         v!("ur_goal_feature_id", vec!("a", "b", "c", "d")),
// //         "a".to_spvalue(),
// //     ));
// //     let state = state.add(SPAssignment::new(
// //         v!("ur_tcp_id", vec!("svt_tcp")),
// //         "svt_tcp".to_spvalue(),
// //     ));
// //     state
// // }

// // pub fn the_model() -> Model {
// //     let state = make_initial_state();
// //     let op_move_to_b = Operation::new(
// //         "op_move_to_b",
// //         t!(
// //             "start_moving_to_b",
// //             "var:ur_action_trigger == false && var:ur_action_state == initial && var:ur_current_pose != b",
// //             "true",
// //             vec!(
// //                 "var:ur_command <- movej",
// //                 "var:ur_action_trigger <- true",
// //                 "var:ur_goal_feature_id <- b",
// //                 "var:ur_tcp_id <- svt_tcp"
// //             ),
// //             Vec::<&str>::new(),
// //             &state
// //         ),
// //         t!(
// //             "complete_moving_to_b",
// //             "var:ur_action_state == done",
// //             "true",
// //             vec!(
// //                 "var:ur_action_trigger <- false",
// //                 "var:ur_current_pose <- b"
// //             ),
// //             Vec::<&str>::new(),
// //             &state
// //         )
// //     );

// //     let op_move_to_c = Operation::new(
// //         "op_move_to_c",
// //         t!(
// //             "start_moving_to_c",
// //             "var:ur_action_trigger == false && var:ur_action_state == initial && var:ur_current_pose == b",
// //             "true",
// //             vec!(
// //                 "var:ur_command <- movej",
// //                 "var:ur_action_trigger <- true",
// //                 "var:ur_goal_feature_id <- c",
// //                 "var:ur_tcp_id <- svt_tcp"
// //             ),
// //             Vec::<&str>::new(),
// //             &state
// //         ),
// //         t!(
// //             "complete_moving_to_c",
// //             "var:ur_action_state == done",
// //             "true",
// //             vec!(
// //                 "var:ur_action_trigger <- false",
// //                 "var:ur_current_pose <- c"
// //             ),
// //             Vec::<&str>::new(),
// //             &state
// //         )
// //     );

// //     let op_move_to_d = Operation::new(
// //         "op_move_to_d",
// //         t!(
// //             "start_moving_to_d",
// //             "var:ur_action_trigger == false && var:ur_action_state == initial && var:ur_current_pose == c",
// //             "true",
// //             vec!(
// //                 "var:ur_command <- movej",
// //                 "var:ur_action_trigger <- true",
// //                 "var:ur_goal_feature_id <- d",
// //                 "var:ur_tcp_id <- svt_tcp"
// //             ),
// //             Vec::<&str>::new(),
// //             &state
// //         ),
// //         t!(
// //             "complete_moving_to_d",
// //             "var:ur_action_state == done",
// //             "true",
// //             vec!(
// //                 "var:ur_action_trigger <- false",
// //                 "var:ur_current_pose <- d"
// //             ),
// //             Vec::<&str>::new(),
// //             &state
// //         )
// //     );

// //     // Adding the opeation states in the model
// //     Model::new(
// //         "asdf",
// //         state.clone(),
// //         vec![],
// //         vec![
// //             op_move_to_b.clone(),
// //             op_move_to_c.clone(),
// //             op_move_to_d.clone(),
// //         ],
// //     )

// //     // let goal = pred_parser::pred("var:ur_current_pose == d", &m.initial_state).unwrap();
// //     // let result = bfs_operation_planner(m.initial_state, goal, m.operations, 30);
// //     // assert_eq!(vec!("op_move_to_b", "op_move_to_c", "op_move_to_d"), result.plan);
// // }


// pub fn make_initial_state() -> State {
//     let state = State::new();
//     let state = state.add(SPAssignment::new(
//         v_command!("ref_pos", vec!("a", "b", "c")),
//         SPValue::Unknown,
//     ));
//     let state = state.add(SPAssignment::new(
//         v_measured!("act_pos", vec!("a", "b", "c")),
//         SPValue::Unknown,
//     ));
//     state
// }

// pub fn the_model() -> Model {
//     let state = make_initial_state();
//     let op_move_to_a = Operation::new(
//         "op_move_to_a",
//         t!(
//             "start_move_to_a",
//             "var:ref_pos != a && var:act_pos != a",
//             "true",
//             vec!("var:ref_pos <- a"),
//             Vec::<&str>::new(),
//             &state
//         ),
//         t!(
//             "complete_move_to_a",
//             "var:ref_pos == a && var:act_pos != a",
//             "true",
//             vec!("var:act_pos <- a"),
//             Vec::<&str>::new(),
//             &state
//         ),
//     );
    
//     let op_move_to_b = Operation::new(
//         "op_move_to_b",
//         t!(
//             "start_move_to_b",
//             "var:ref_pos == a && var:act_pos == a",
//             "true",
//             vec!("var:ref_pos <- b"),
//             Vec::<&str>::new(),
//             &state
//         ),
//         t!(
//             "complete_move_to_b",
//             "var:ref_pos == b && var:act_pos == a",
//             "true",
//             vec!("var:act_pos <- b"),
//             Vec::<&str>::new(),
//             &state
//         ),
//     );

//     let op_move_to_c = Operation::new(
//         "op_move_to_c",
//         t!(
//             "start_move_to_c",
//             "var:ref_pos == b && var:act_pos == b",
//             "true",
//             vec!("var:ref_pos <- c"),
//             Vec::<&str>::new(),
//             &state
//         ),
//         t!(
//             "complete_move_to_c",
//             "var:ref_pos == c && var:act_pos == b",
//             "true",
//             vec!("var:act_pos <- c"),
//             Vec::<&str>::new(),
//             &state
//         ),
//     );

//     Model::new(
//         "asdf",
//         state.clone(),
//         vec![],
//         vec![
//             op_move_to_a.clone(),
//             op_move_to_b.clone(),
//             op_move_to_c.clone(),
//         ],
//     )
// }