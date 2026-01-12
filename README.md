## An emulation example using micro_sp

## Stress testing:
```
cargo nextest run --stress-count 5 --no-fail-fast --status-level fail --test-threads 1
```

## Test coverage (todo, single thread, no fail fast):
```
cargo tarpaulin --out Html
```

### Unrelated: getting ROS2 jazzy on ubuntu25.10:
### 1. Install core build tools and pipx
sudo apt update
sudo apt install \
  cmake \
  git \
  python3 \
  python3-setuptools \
  python3-pip \
  pipx \
  wget

### 2. Install ROS 2 build tools using pipx
pipx install bloom
pipx install colcon-common-extensions
pipx install rosdep
pipx install vcstool

### 3. Update your shell PATH to find the new commands
pipx ensurepath
source ~/.bashrc

### 4. Initialize rosdep
sudo env "PATH=$PATH" rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool python3-catkin-pkg-modules python3-rosdistro-modules"


## Quickstart
1. Install rust and rust_analyzer for inlay hints (makes modeling easier)
2. Build with `colcon build` in a workspace to build the custom msgs
3. Run the example with `cargo run`

## Architecture:
![](figures/architecture.png)

## Features:
We emulate two resources, a robot and a gantry. These resources can perform some dummy actions, move, calibrate, lock, unlock for the gantry, and move, pick, place, mount, unmount, check_mounted_tool for the robot. In reality, problems arise during execution so these actions can fail and timeout. To emulate such failures and timeouts, we can send a nested Emulation message in the command request to the nodes, forcing them to fail or timeout. This helps us develop the initial behavior model much easier, without the need of connecting to real equipment or simulations.  

## How is this useful:
Exchange the emulation with the real resource driver or simulation, and update the model and interfaces. Enables quicker iterations of the behavior model.

## Failure Emulation:
To disable all failure emulation and test only for nominal behavior, set all emulation values to 0.

To emulate failure, inject emulation state like this (gantry example):

TODO: set_state ROS2 service so that we can do this from other nodes.
```
let new_state = state
    .update("gantry_emulate_execution_time", 2.to_spvalue())
    .update("gantry_emulated_execution_time", 3000.to_spvalue())
    .update("gantry_emulate_failure_rate", 2.to_spvalue())
    .update("gantry_emulated_failure_rate", 30.to_spvalue())
    .update("gantry_emulate_failure_cause", 2.to_spvalue())
    .update(
        "gantry_emulated_failure_cause",
        vec!["violation", "collision", "detected_drift"].to_spvalue(),
    );

    let modified_state = state.get_diff_partial_state(&new_state);
    command_sender
        .send(Command::SetPartialState(modified_state))
        .await?;
```
where:
```
# DONT_EMULATE_EXECUTION_TIME: The action will be executed immediatelly
# EMULATE_EXACT_EXECUTION_TIME: The action will always take "emulate_execution_time" amount of time
# EMULATE_RANDOM_EXECUTION_TIME: The action will randomly take between 0 and "emulated_execution_time" amount of time
uint8 DONT_EMULATE_EXECUTION_TIME = 0
uint8 EMULATE_EXACT_EXECUTION_TIME = 1
uint8 EMULATE_RANDOM_EXECUTION_TIME = 2
uint8 emulate_execution_time
int32 emulated_execution_time # milliseconds

# DONT_EMULATE_FAILURE: The action will be execute succesfully every time
# EMULATE_FAILURE_ALWAYS: The action will always fail
# EMULATE_RANDOM_FAILURE_RATE: The action will randomly fail with a "emulated_failure_rate" rate
uint8 DONT_EMULATE_FAILURE = 0
uint8 EMULATE_FAILURE_ALWAYS = 1
uint8 EMULATE_RANDOM_FAILURE_RATE = 2
uint8 emulate_failure_rate
int32 emulated_failure_rate # percentage 0..100

# DONT_EMULATE_FAILURE_CAUSE: If the action fails, it wil fail with a generic "fail" cause
# EMULATE_EXACT_FAILURE_CAUSE: Specify why the exact reason why the action fails (takes the first from the "emulated_failure_cause" list)
# EMULATE_RANDOM_FAILURE_CAUSE: The action will fail and randomly choose a cause from the "emulated_failure_cause" list
uint8 DONT_EMULATE_FAILURE_CAUSE = 0
uint8 EMULATE_EXACT_FAILURE_CAUSE = 1
uint8 EMULATE_RANDOM_FAILURE_CAUSE = 2
uint8 emulate_failure_cause
string[] emulated_failure_cause # For example: ["violation", "timeout", "collision", etc.]
```

## Example run:
```
[INFO] [1742390440.685306102] [micro_sp_emulator]: Spawning emulators...
[INFO] [1742390441.089377046] [micro_sp_emulator]: Spawning interfaces...
[WARN] [1742390441.091808710] [gantry_interface]: Waiting for the server...
[WARN] [1742390441.293560961] [robot_interface]: Waiting for the server...
[INFO] [1742390441.493436451] [micro_sp_emulator]: Spawning operation planner...
[INFO] [minimal_model_planner_ticker] Planner info: UNKNOWN
[INFO] [minimal_model_planner_ticker] Planner info: Planner is not triggered
[INFO] [1742390441.695609438] [micro_sp_emulator]: Spawning auto transition runner...
[INFO] [1742390441.908251250] [micro_sp_emulator]: Spawning auto operation runner...
[INFO] [1742390442.110411121] [micro_sp_emulator]: Spawning operation runner...
[INFO] [minimal_model_operation_runner] Plan current state: UNKNOWN.
[INFO] [1742390442.311677038] [micro_sp_emulator]: Spawning test generator...
[WARN] [1742390443.321323705] [micro_sp_emulator]: Starting tests...
[INFO] [1742390443.321720673] [micro_sp_emulator]: Node started.
[INFO] [1742390443.322351650] [gantry_interface]: Server available.
[INFO] [1742390443.322441893] [gantry_interface]: Spawned.
[INFO] [1742390443.322452204] [robot_interface]: Server available.
[INFO] [1742390443.322473639] [robot_interface]: Spawned.
[WARN] [1742390444.321925738] [micro_sp_emulator]: Tests started.
[WARN] [1742390444.322102168] [micro_sp_emulator]: Starting test 1.
[INFO] [minimal_model_planner_ticker] Planner info: Planner triggered (try 0/3): A new plan was found.
[INFO] [minimal_model_planner_ticker] Got a plan:
       1 -> op_robot_check_for_suction_tool_mounted
[INFO] [minimal_model_operation_runner] Plan current state: initial.
[INFO] [minimal_model_planner_ticker] Planner info: Planner triggered and (re)planned.
[INFO] [minimal_model_operation_runner] Plan current state: executing.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_check_for_suction_tool_mounted: initial.
[INFO] [minimal_model_operation_runner] Current operation 'op_robot_check_for_suction_tool_mounted' info: UNKNOWN.
[INFO] [1742390444.695218780] [robot_interface]: Requesting to check_mounted_tool.
[INFO] [minimal_model_planner_ticker] Planner info: Planner is not triggered
[INFO] [1742390444.697557517] [robot_emulator]: Got request to check_mounted_tool.
[INFO] [1742390444.697614201] [robot_emulator]: Succeeded to check_mounted_tool.
[INFO] [1742390444.698093881] [robot_interface]: Requested check_mounted_tool succeeded.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_check_for_suction_tool_mounted: executing.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_check_for_suction_tool_mounted: completed.
[INFO] [minimal_model_operation_runner] Current operation 'op_robot_check_for_suction_tool_mounted' info: Completing operation..
[INFO] [minimal_model_operation_runner] Plan current state: completed.
[INFO] [minimal_model_planner_ticker] Plan current step: 1.
[INFO] [minimal_model_planner_ticker] Planner info: Planner triggered (try 0/3): A new plan was found.
[INFO] [minimal_model_planner_ticker] Got a plan:
       1 -> op_gantry_unlock
       2 -> op_gantry_calibrate
       3 -> op_gantry_lock
       4 -> op_robot_move_to_suction_tool_rack
       5 -> op_robot_mount_suction_tool
[INFO] [minimal_model_planner_ticker] Planner info: Planner triggered and (re)planned.
[INFO] [minimal_model_planner_ticker] Planner info: Planner is not triggered
[INFO] [minimal_model_operation_runner] Plan current state: initial.
[INFO] [minimal_model_planner_ticker] Plan current step: 0.
[INFO] [minimal_model_operation_runner] Plan current state: executing.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_unlock: initial.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_unlock' info: UNKNOWN.
[INFO] [1742390445.295791637] [gantry_interface]: Requesting to unlock.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_unlock: executing.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_unlock' info: Waiting to be completed..
[INFO] [1742390447.258873400] [gantry_emulator]: Got request to unlock.
[INFO] [1742390447.258999295] [gantry_emulator]: Succeeded to unlock.
[INFO] [1742390447.260063229] [gantry_interface]: Requested unlock succeeded.
[WARN] [1742390447.337345884] [micro_sp_emulator]: All tests are finished. Generating report...
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_unlock: completed.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_unlock' info: Completing operation..
[INFO] [minimal_model_planner_ticker] Plan current step: 1.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_calibrate: initial.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_calibrate' info: UNKNOWN.
[INFO] [1742390447.594877074] [gantry_interface]: Requesting to calibrate.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_calibrate: executing.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_calibrate' info: Waiting to be completed..
[INFO] [1742390447.744288799] [gantry_emulator]: Got request to calibrate.
[INFO] [1742390447.744416168] [gantry_emulator]: Succeeded to calibrate.
[INFO] [1742390447.745168887] [gantry_interface]: Requested calibration succeeded.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_calibrate: completed.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_calibrate' info: Completing operation..
[INFO] [minimal_model_planner_ticker] Plan current step: 2.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_lock: initial.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_lock' info: UNKNOWN.
[INFO] [1742390448.095038105] [gantry_interface]: Requesting to lock.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_lock: executing.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_lock' info: Waiting to be completed..
[INFO] [1742390449.708422215] [gantry_emulator]: Got request to lock.
[ERROR] [1742390449.708478068] [gantry_emulator]: Failed to lock due to violation.
[ERROR] [1742390449.708885976] [gantry_interface]: Requested lock failed.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_lock: failed.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_lock' info: Failing operation..
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_lock: initial.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_lock' info: Retrying. Retry nr. 1 out of 3..
[INFO] [1742390450.095741708] [gantry_interface]: Requesting to lock.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_lock: executing.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_lock' info: Waiting to be completed..
[INFO] [1742390451.210370769] [gantry_emulator]: Got request to lock.
[INFO] [1742390451.210437654] [gantry_emulator]: Succeeded to lock.
[INFO] [1742390451.210854221] [gantry_interface]: Requested lock succeeded.
[INFO] [minimal_model_operation_runner] Current state of operation op_gantry_lock: completed.
[INFO] [minimal_model_operation_runner] Current operation 'op_gantry_lock' info: Completing operation..
[INFO] [minimal_model_planner_ticker] Plan current step: 3.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_move_to_suction_tool_rack: initial.
[INFO] [minimal_model_operation_runner] Current operation 'op_robot_move_to_suction_tool_rack' info: UNKNOWN.
[INFO] [1742390451.596503706] [robot_interface]: Requesting to move.
[INFO] [1742390451.598601089] [robot_emulator]: Got request to move to suction_tool_rack.
[INFO] [1742390451.598675932] [robot_emulator]: Succeeded to move to suction_tool_rack.
[INFO] [1742390451.599139458] [robot_interface]: Requested move to 'suction_tool_rack' succeeded.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_move_to_suction_tool_rack: executing.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_move_to_suction_tool_rack: completed.
[INFO] [minimal_model_operation_runner] Current operation 'op_robot_move_to_suction_tool_rack' info: Completing operation..
[INFO] [minimal_model_planner_ticker] Plan current step: 4.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_mount_suction_tool: initial.
[INFO] [minimal_model_operation_runner] Current operation 'op_robot_mount_suction_tool' info: UNKNOWN.
[INFO] [1742390451.896164760] [robot_interface]: Requesting to mount.
[INFO] [1742390451.898489581] [robot_emulator]: Got request to mount.
[INFO] [1742390451.898537368] [robot_emulator]: Succeeded to mount.
[INFO] [1742390451.899002128] [robot_interface]: Requested mount succeeded.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_mount_suction_tool: executing.
[INFO] [minimal_model_operation_runner] Current state of operation op_robot_mount_suction_tool: completed.
[INFO] [minimal_model_operation_runner] Current operation 'op_robot_mount_suction_tool' info: Completing operation..
[INFO] [minimal_model_planner_ticker] Plan current step: 5.
[INFO] [minimal_model_operation_runner] Plan current state: completed.
```