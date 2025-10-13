pub static MAIN_TICK_INTERVAL: u64 = 1000; // milliseconds
pub static EMULATOR_TICK_INTERVAL: u64 = 200; // milliseconds
pub static NUMBER_OF_TEST_CASES: u64 = 20;

pub mod emulators;
pub use crate::emulators::gantry::*;
pub use crate::emulators::robot::*;

pub mod model;
pub use crate::model::*;

// pub mod utils;
// pub use crate::utils::state_publisher::*;
// pub use crate::utils::env_logger::*;
