use micro_sp::*;
use redis::aio::MultiplexedConnection;
use std::error::Error;

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let mut auto_transitions = vec![];
    let sops = vec![];
    let operations = vec![];

    let lights_on = bv!(&&format!("lights_on"));
    let state = state.add(assign!(
        lights_on,
        SPValue::Bool(BoolOrUnknown::Bool(false))
    ));

    auto_transitions.push(Transition::parse(
        "turn_lights_on",
        "true",
        "var:lights_on == false",
        Vec::from([]),
        vec!["var:lights_on <- true"],
        &state,
    ));

    auto_transitions.push(Transition::parse(
        "turn_lights_off",
        "true",
        "var:lights_on == true",
        Vec::from([]),
        vec!["var:lights_on <- false"],
        &state,
    ));

    let model = Model::new(sp_id, auto_transitions, vec![], sops, operations);

    (model, state)
}


pub async fn run_emultaion(
    _sp_id: &str,
    mut _con: MultiplexedConnection,
) -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    
    Ok(())
}