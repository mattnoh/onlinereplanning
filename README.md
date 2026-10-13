# CoManDO planner

One ROS-independent C++ library, complete OCP files, and separate ROS1/ROS2 transports.

- `include/ocp`: each file contains a complete optimal-control formulation.
- `include/parameters`: parameter defaults and environment overrides.
- `include/target`: prediction and synthetic target models, without ROS.
- `include/planner_core`, `src`: solver, RH replay, direct MPC, and shared controller API.
- `transport/ros1`, `transport/ros2`: messages, subscriptions, timers, parameter loading, and platform commands.
- `tools/analysis`: the existing plotters and bag converters, retaining their designs.
- `logs`: generated flight results only; starts empty.

The `comando_planner` package name, planner node name, OCP keys, and supported command topics are retained. Explicitly legacy formulations are removed.

## Execution

RH replanning uses `stc_landing_noaug`: planned states go to PX4 position/velocity control or Crazyflie's full-state tracker. Direct MPC uses `stc_landing_mpc`: asynchronous optimization supplies a policy evaluated at 50 Hz, and thrust acceleration plus body rates go to PX4 raw attitude commands or Crazyflie's patched `/cmd_bodyrate` input. Direct MPC is not supported in open-loop mode.

Both transports call `planner_core::Controller` with ROS-free input snapshots and receive typed commands and diagnostics. Its lifecycle lock permits solve/replay concurrency and serializes configuration and resets. Absolute timestamps are scalar seconds; scheduling uses `steady_clock`.

## Build

Keep ALIPDDP as a sibling, or set `COMANDO_SOLVER_DIR` explicitly. Source the appropriate ROS environment before building. Never build both workspaces concurrently.

ROS1: `catkin build comando_planner -j1 -p1 --make-args comando_planner`; build `target_publisher` and each test in separate invocations.

ROS2: build the `comando_planner` target with one worker, then `target_publisher` separately. The CMake cache option `COMANDO_ROS_VERSION` selects 1 or 2; it defaults from `ROS_VERSION`.

Standalone core:

```sh
cmake -S . -B build-core -DCOMANDO_ROS_VERSION=0 -DCOMANDO_SOLVER_DIR=/path/to/ALIPDDP-main
cmake --build build-core --target test_controller -j1
./build-core/test_controller
```

Research benchmark executables are opt-in via `COMANDO_BUILD_BENCHMARKS`; the core is compiled once and linked by all executables.

## Profiles and validation

ROS1 profiles are in `config`; ROS2 parameter-file equivalents are in `config/ros2`. OCP parameter defaults are initialized from the process environment, preserving the validated startup tuning. Restart the process to change environment-based mathematical parameters; runtime target/profile commands retain their existing ROS interfaces.

PX4 and Crazyflie require separate vehicle calibration and tuning. The archived planner's results remain the regression reference. See `docs/CLEAN_PLANNER_VALIDATION.md` for the actual checks, limitations, and cutover status.
