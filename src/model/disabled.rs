use micro_sp::*;
use redis::aio::MultiplexedConnection;
use std::error::Error;

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let auto_transitions = vec![];
    let sops = vec![];
    let mut operations = vec![];

    let enabled = bv!(&&format!("enabled"));
    let state = state.add(assign!(enabled, SPValue::Bool(BoolOrUnknown::Bool(false))));

    let done = bv!(&&format!("done"));
    let state = state.add(assign!(done, SPValue::Bool(BoolOrUnknown::Bool(false))));

    operations.push(Operation::new(
        &format!("emulate_disabled"),
        None,
        None,
        None, 
        None,
        false,
        Vec::from([Transition::parse(
            &format!("start_disabled"),
            "true",
            "var:enabled == true",
            vec![],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([Transition::parse(
            &format!("complete_disabled"),
            "true",
            "true",
            vec![
                "var:done <- true"
            ],
            Vec::<&str>::new(),
            &state,
        )]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
        Vec::from([]),
    ));

    let model = Model::new(sp_id, auto_transitions, vec![], sops, operations);

    (model, state)
}

pub async fn run_emultaion(
    sp_id: &str,
    mut con: MultiplexedConnection,
) -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    let goal = "var:done == true";

    if let Some(state) = StateManager::get_full_state(&mut con).await {
        let plan_state = state.get_string_or_default_to_unknown(
            &format!("{}_plan_state", sp_id),
            &format!("{}_disabled_test", sp_id),
        );

        if PlanState::from_str(&plan_state) == PlanState::Failed
            || PlanState::from_str(&plan_state) == PlanState::Completed
            || PlanState::from_str(&plan_state) == PlanState::Initial
            || PlanState::from_str(&plan_state) == PlanState::UNKNOWN
        {
            let new_state = state
                .update(
                    &format!("{sp_id}_current_goal_state"),
                    CurrentGoalState::Initial.to_spvalue(),
                )
                .update(
                    &format!("{sp_id}_current_goal_predicate"),
                    goal.to_spvalue(),
                )
                .update(&format!("{sp_id}_replan_trigger"), true.to_spvalue())
                .update(&format!("{sp_id}_replanned"), false.to_spvalue());

            let modified_state = state.get_diff_partial_state(&new_state);
            StateManager::set_state(&mut con, &modified_state).await;
        }
    }

    Ok(())
}
