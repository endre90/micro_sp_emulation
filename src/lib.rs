pub static MAIN_TICK_INTERVAL: u64 = 1000; // milliseconds
pub static EMULATOR_TICK_INTERVAL: u64 = 200; // milliseconds
pub static NUMBER_OF_TEST_CASES: u64 = 20;

pub static DONT_EMULATE_EXECUTION_TIME: i64 = 0; // The action will be executed immediatelly
pub static EMULATE_EXACT_EXECUTION_TIME: i64 = 1; // The action will always take "emulate_execution_time" amount of time
pub static EMULATE_RANDOM_EXECUTION_TIME: i64 = 2; //The action will randomly take between 0 and "emulated_execution_time" amount of time

pub static DONT_EMULATE_FAILURE: i64 = 0; // The action will be execute succesfully every time
pub static EMULATE_FAILURE_ALWAYS: i64 = 1; // The action will always fail
pub static EMULATE_FAILURE_RANDOM_RATE: i64 = 2; // The action will randomly fail with a "emulated_failure_rate" rate

pub static DONT_EMULATE_FAILURE_CAUSE: i64 = 0; // If the action fails, it wil fail with a generic "fail" cause
pub static EMULATE_EXACT_FAILURE_CAUSE: i64 = 1; // Specify why the exact reason why the action fails (takes the first from the "emulated_failure_cause" list)
pub static EMULATE_RANDOM_FAILURE_CAUSE: i64 = 2; // The action will fail and randomly choose a cause from the "emulated_failure_cause" list

pub mod emulators;
pub use crate::emulators::gantry::*;
pub use crate::emulators::robot::*;

pub mod model;
pub use crate::model::*;

// pub mod utils;
// pub use crate::utils::state_publisher::*;
// pub use crate::utils::env_logger::*;
