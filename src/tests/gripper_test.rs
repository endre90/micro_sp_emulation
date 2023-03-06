use crate::runner;
use micro_sp::{
    a, assign, av_command, av_estimated, av_measured, av_runner, bv_command, bv_estimated,
    bv_measured, bv_runner, eq, fv_command, fv_estimated, fv_measured, fv_runner, iv_command,
    iv_estimated, iv_measured, iv_runner, t, t_plan, v_command, v_estimated, v_measured, v_runner,
    Operation, Model,
};
use micro_sp::{
    pred_parser, Action, Predicate, SPAssignment, SPValue, SPValueType, SPVariable, SPVariableType,
    State, ToSPValue, ToSPWrapped, ToSPWrappedVar, Transition,
};
use proptest::{bool, prelude::*};
use r2r::micro_sp_emulation_msgs::msg::GripperIncoming;
use r2r::micro_sp_emulation_msgs::msg::GripperOutgoing;
use r2r::QosProfile;
use std::sync::{Arc, Mutex};

use runner::gripper_pub_sub_ticker::*;
use runner::ticker::*;

pub static NODE_ID: &'static str = "gripper_test";
pub static TICKER_RATE: u64 = 100;
pub static PUBLISHER_RATE: u64 = 100;

proptest! {
    #![proptest_config(ProptestConfig::with_cases(10))]
    #[test]
    fn test_the_gripper_mcdc(gripper_ref_val in prop_oneof!("opened", "closed"), gripper_act_val in prop_oneof!("opened", "closed", "gripping")) {
        
        tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            assert!(true);
        });

        let gripper_act = v_measured!("gripper_act", vec!("opened", "closed", "gripping"));
        let gripper_ref = v_command!("gripper_ref", vec!("opened", "closed"));

        let state = State::new();
        let state = state.add(assign!(gripper_act, gripper_act_val.to_spvalue()));
        let state = state.add(assign!(gripper_ref, gripper_ref_val.to_spvalue()));

        // And some mandatory variables (actually, these should be automatically
        // included when calling Model::new()...)
        let state = state.add(SPAssignment::new(
            v_runner!("runner_goal"),
            // "var:item_a_pose == atr && var:item_b_pose == atr".to_spvalue(),
            "var:gripper_act == closed".to_spvalue()
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

        let mut operations = vec!();
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

        let model = Model::new(
            "asdf",
            state.clone(),
            autos,
            operations,
            vec!()
        );

        let ctx = r2r::Context::create()?;
        let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

        let ticker_timer =
            node.create_wall_timer(std::time::Duration::from_millis(TICKER_RATE))?;

        let gripper_subscriber = node.subscribe::<GripperIncoming>(
            "gripper_incoming",
            QosProfile::default().reliable()
        )?;

        let gripper_publisher_timer =
            node.create_wall_timer(std::time::Duration::from_millis(PUBLISHER_RATE))?;
        let gripper_publisher =
            node.create_publisher::<GripperOutgoing>("gripper_outgoing", QosProfile::default().reliable())?;

        let shared_state = Arc::new(Mutex::new(state.clone()));

        let shared_state_clone = shared_state.clone();
        tokio::task::spawn(async move {
            match gripper_pub_sub_subscriber_callback(&shared_state_clone, gripper_subscriber, NODE_ID).await {
                Ok(()) => r2r::log_info!(NODE_ID, "Subscriber succeeded."),
                Err(e) => r2r::log_error!(NODE_ID, "Subscriber failed with: '{}'.", e),
            };
        });

        let shared_state_clone = shared_state.clone();
        tokio::task::spawn(async move {
        // wait for the measured values to update the state
        // tokio::time::sleep(std::time::Duration::from_millis(5000)).await;
            let result = ticker(
                NODE_ID,
                // &ur_action_client,
                &model,
                &shared_state_clone.clone(),
                ticker_timer,
            )
            .await;
            match result {
                Ok(()) => r2r::log_info!(NODE_ID, "Simple Controller Service call succeeded."),
                Err(e) => r2r::log_error!(
                    NODE_ID,
                    "Simple Controller Service call failed with: {}.",
                    e
                ),
            };
        });

        // prop_assert!(plan.found);
        // prop_assert!(!model.is_empty());
        // prop_assert!(model.last_value().is_some());
    }
}
