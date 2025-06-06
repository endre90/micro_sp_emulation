use std::time::SystemTime;

use micro_sp::*;
use ordered_float::OrderedFloat;

fn generate_basic_variables(name: &str, state: &State) -> State {
    let resource_online = bv!(&&format!("{}_resource_online", name));
    let request_trigger = bv!(&&format!("{}_request_trigger", name));
    let request_state = v!(&&format!("{}_request_state", name));
    let total_fail_counter = iv!(&&format!("{}_total_fail_counter", name));
    let subsequent_fail_counter = iv!(&&format!("{}_subsequent_fail_counter", name));
    let ref_counter = iv!(&&format!("{}_ref_counter", name));

    let state = state.add(assign!(resource_online, false.to_spvalue()));
    let state = state.add(assign!(request_trigger, false.to_spvalue()));
    let state = state.add(assign!(request_state, "initial".to_spvalue()));
    let state = state.add(assign!(total_fail_counter, 0.to_spvalue()));
    let state = state.add(assign!(subsequent_fail_counter, 0.to_spvalue()));
    let state = state.add(assign!(ref_counter, 1.to_spvalue()));

    state
}

fn generate_emulation_variables(name: &str, state: &State) -> State {
    // -----------------------------------------------------------------------
    // # DONT_EMULATE_EXECUTION_TIME: The action will be executed immediatelly
    // # EMULATE_EXACT_EXECUTION_TIME: The action will always take "emulate_execution_time" amount of time
    // # EMULATE_RANDOM_EXECUTION_TIME: The action will randomly take between 0 and "emulated_execution_time" amount of time
    // uint8 DONT_EMULATE_EXECUTION_TIME = 0
    // uint8 EMULATE_EXACT_EXECUTION_TIME = 1
    // uint8 EMULATE_RANDOM_EXECUTION_TIME = 2
    // uint8 emulate_execution_time
    // int32 emulated_execution_time # milliseconds

    // # DONT_EMULATE_FAILURE: The action will be execute succesfully every time
    // # EMULATE_FAILURE_ALWAYS: The action will always fail
    // # EMULATE_RANDOM_FAILURE_RATE: The action will randomly fail with a "emulated_failure_rate" rate
    // uint8 DONT_EMULATE_FAILURE = 0
    // uint8 EMULATE_FAILURE_ALWAYS = 1
    // uint8 EMULATE_RANDOM_FAILURE_RATE = 2
    // uint8 emulate_failure_rate
    // int32 emulated_failure_rate # percentage 0..100

    // # DONT_EMULATE_FAILURE_CAUSE: If the action fails, it wil fail with a generic "fail" cause
    // # EMULATE_EXACT_FAILURE_CAUSE: Specify why the exact reason why the action fails (takes the first from the "emulated_failure_cause" list)
    // # EMULATE_RANDOM_FAILURE_CAUSE: The action will fail and randomly choose a cause from the "emulated_failure_cause" list
    // uint8 DONT_EMULATE_FAILURE_CAUSE = 0
    // uint8 EMULATE_EXACT_FAILURE_CAUSE = 1
    // uint8 EMULATE_RANDOM_FAILURE_CAUSE = 2
    // uint8 emulate_failure_cause
    // string[] emulated_failure_cause # For example: ["violation", "timeout", "collision", etc.]
    // -----------------------------------------------------------------------

    let emulate_execution_time = iv!(&&format!("{}_emulate_execution_time", name));
    let emulate_failure_rate = iv!(&&format!("{}_emulate_failure_rate", name));
    let emulate_failure_cause = iv!(&&format!("{}_emulate_failure_cause", name));

    let state = state.add(assign!(emulate_execution_time, 0.to_spvalue()));
    let state = state.add(assign!(emulate_failure_rate, 0.to_spvalue()));
    let state = state.add(assign!(emulate_failure_cause, 0.to_spvalue()));

    let emulated_execution_time = iv!(&&format!("{}_emulated_execution_time", name));
    let emulated_failure_rate = iv!(&&format!("{}_emulated_failure_rate", name));
    let emulated_failure_cause = av!(&&format!("{}_emulated_failure_cause", name));

    let state = state.add(assign!(emulated_execution_time, 0.to_spvalue()));
    let state = state.add(assign!(emulated_failure_rate, 0.to_spvalue()));
    let state = state.add(assign!(
        emulated_failure_cause,
        SPValue::Array(ArrayOrUnknown::Array(vec![]))
    ));

    state
}

pub fn state() -> State {
    let state = State::new();

    // -----------------------------------------------------------------------
    // Gantry:
    // string command # move, calibrate, lock, unlock
    // float32 speed
    // string position
    // -----------------------------------------------------------------------

    let state = generate_basic_variables("gantry", &state);

    let gantry_command_command = v!("gantry_command_command");
    let gantry_speed_command = fv!("gantry_speed_command");
    let gantry_position_command = v!("gantry_position_command");
    let gantry_light_indicator = bv!("gantry_light_indicator");
    let blinked = bv!("blinked");

    let state = state.add(assign!(
        gantry_command_command,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(gantry_speed_command, 0.0.to_spvalue()));
    let state = state.add(assign!(
        gantry_position_command,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(
        gantry_light_indicator,
        SPValue::Bool(BoolOrUnknown::UNKNOWN)
    ));

    let state = state.add(assign!(
        blinked,
        SPValue::Bool(BoolOrUnknown::Bool(false))
    ));

    // We estimate (memory variables) the following, since we cannot directly measure
    let gantry_speed_measured = fv!("gantry_speed_estimated");
    let gantry_position_estimated = v!("gantry_position_estimated");
    let gantry_calibrated_estimated = bv!("gantry_calibrated_estimated");
    let gantry_locked_estimated = bv!("gantry_locked_estimated");

    let state = state.add(assign!(
        gantry_calibrated_estimated,
        SPValue::Bool(BoolOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(
        gantry_locked_estimated,
        SPValue::Bool(BoolOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(
        gantry_speed_measured,
        SPValue::Float64(FloatOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(
        gantry_position_estimated,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));

    // Optional: emulate gantry failure and execution time
    let state = generate_emulation_variables("gantry", &state);

    // -----------------------------------------------------------------------
    // Robot:
    // string command # move, pick, place
    // float32 speed
    // string position
    // -----------------------------------------------------------------------

    let state = generate_basic_variables("robot", &state);

    let robot_command_command = v!("robot_command_command");
    let robot_speed_command = fv!("robot_speed_command");
    let robot_position_command = v!("robot_position_command");

    let state = state.add(assign!(
        robot_command_command,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(robot_speed_command, 0.0.to_spvalue()));
    let state = state.add(assign!(
        robot_position_command,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));

    // We estimate (memory variables) the following, since we cannot directly measure
    let robot_speed_measured = fv!("robot_speed_estimated");
    let robot_position_estimated = v!("robot_position_estimated");
    let robot_mounted_estimated = v!("robot_mounted_estimated"); // gripper, vacuum
    let robot_mounted_checked = bv!("robot_mounted_checked");
    let robot_mounted_one_time_measured = v!("robot_mounted_one_time_measured");

    let test_frame_1 = tfv!("test_frame_1");
    let state = state.add(assign!(
        test_frame_1,
        SPValue::Transform(TransformOrUnknown::Transform(SPTransformStamped {
            active_transform: true,
            enable_transform: true,
            time_stamp: SystemTime::now(),
            parent_frame_id: "a".to_string(),
            child_frame_id: "b".to_string(),
            transform: SPTransform {
                translation: SPTranslation {
                    x: OrderedFloat::from(1.23),
                    y: OrderedFloat::from(1.23),
                    z: OrderedFloat::from(1.23),
                },
                rotation: SPRotation {
                    x: OrderedFloat::from(1.23),
                    y: OrderedFloat::from(1.23),
                    z: OrderedFloat::from(1.23),
                    w: OrderedFloat::from(1.23),
                }
            },
            metadata: MapOrUnknown::UNKNOWN
        }))
    ));

    let state = state.add(assign!(
        robot_speed_measured,
        SPValue::Float64(FloatOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(
        robot_position_estimated,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(
        robot_mounted_estimated,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(
        robot_mounted_checked,
        SPValue::Bool(BoolOrUnknown::Bool(false))
    ));
    let state = state.add(assign!(
        robot_mounted_one_time_measured,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));
    // let state = state.add(assign!(asdf, SPValue::Bool(BoolOrUnknown::Bool(false))));

    // let robot_mode_measured = v!("robot_mode_measured"); // safety_stop, emergency_stop, operational

    // Optional: emulate gantry failure and execution time
    let state = generate_emulation_variables("robot", &state);

    state
}
