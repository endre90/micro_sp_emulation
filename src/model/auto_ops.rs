use micro_sp::*;
use redis::aio::MultiplexedConnection;
use std::error::Error;

pub fn model(sp_id: &str, state: &State) -> (Model, State) {
    let state = state.clone();
    let auto_transitions = vec![];
    let sops = vec![];
    let operations = vec![];
    let mut auto_operations = vec![];

    for pos in vec!["a", "b", "c", "d"] {
        auto_operations.push(Operation::new(
            &format!("robot_move_to_{}", pos),
            None,
            None,
            None,
            false,
            Vec::from([Transition::parse(
                &format!("start_robot_move_to_{pos}"),
                &format!("var:robot_request_state == initial \
                && var:robot_request_trigger == false \
                && var:robot_position_estimated != {pos}"),
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
                &format!("complete_robot_move_to_{pos}"),
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
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
            Vec::from([]),
        ));
    }

    let model = Model::new(sp_id, auto_transitions, auto_operations, sops, operations);

    (model, state)
}

pub async fn run_emultaion(
    _sp_id: &str,
    mut _con: MultiplexedConnection,
) -> Result<(), Box<dyn Error>> {
    initialize_env_logger();

    Ok(())
}
