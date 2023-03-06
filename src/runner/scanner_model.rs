use micro_sp::*;
use proptest::{prelude::*, test_runner::TestRunner};
use serde_json::json;

use crate::runner::ticker::extract_goal_from_state;

pub fn scanner_model() -> (
    String,
    State,
    Vec<Transition>,
    Vec<Operation>,
    Vec<Resource>,
) {
    // Define the variables
    // -------------------------------------------------------

    // Scanner variables
    let scanner_trigger = bv_runner!("scanner_trigger");
    let scanner_state = v_runner!("scanner_state"); //, vec!("initial", "executing", "failed", "done"));
    let scanned_a = bv_estimated!("scanned_a");

    // Make a state and assign some values to variables
    // (usually, unknown is a safe initial value for measured and command variables)
    // ----------------------------------------------------
    let state = State::new();

    // 3. Scanner variables
    let state = state.add(assign!(scanner_trigger, false.to_spvalue()));
    let state = state.add(assign!(scanner_state, "initial".to_spvalue()));
    let state = state.add(assign!(scanned_a, false.to_spvalue()));

    // And some mandatory variables (actually, these should be automatically
    // included when calling Model::new()...)
    let state = state.add(SPAssignment::new(
        v_runner!("runner_goal"),
        // "var:item_a_pose == atr && var:item_b_pose == atr".to_spvalue(),
        "var:scanned_a == true".to_spvalue(),
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
    let autos: Vec<Transition> = vec![];

    let mut operations = vec!();

    // Scanner operation
    operations.push(Operation::new(
        &format!("op_scan_box_a"),
        // precondition
        t!(
            // name
            &format!("start_scan_box_a").as_str(),
            // planner guard
            &format!(
                "\
                    var:scanner_state == initial && \
                    var:scanner_trigger == false && \
                    var:scanned_a == false"
            )
            .as_str(),
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
            &format!("complete_scan_box_a").as_str(),
            // planner guard
            "true",
            // runner guard
            &format!("var:scanner_state == succeeded").as_str(),
            // "true",
            // planner actions
            vec!(
                "var:scanner_trigger <- false",
                &format!("var:scanned_a <- true").as_str()
            ),
            //runner actions
            Vec::<&str>::new(),
            &state
        ),
    ));

    (
        "scanner_model".to_string(),
        state.clone(),
        autos,
        operations,
        vec![],
    )
}

#[test]
    fn test_scanning_operation() {

        let m = scanner_model();

        let model = Model::new(
            &m.0,
            m.1,
            m.2,
            m.3,
            vec!()
        );

        let plan = bfs_operation_planner(model.state.clone(), extract_goal_from_state(&model.state.clone()), model.operations.clone(), 10);
        for p in plan.plan {
            println!("{}", p);
        }

        assert!(plan.found);
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
