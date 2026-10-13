# CoManDO planner

The active MAVROS/ROS1 planner uses a ROS-independent RH/MPC core. The previous planner and historical logs are in `../comando_planner_old`.

- `include/ocp`: five complete formulations: startup hover, CTCS `stc_landing_noaug`/`stc_landing_mpc`, and ordinary constrained `landing_rh`/`landing_mpc`. Equations, costs, constraints and derivatives stay in each OCP file.
- `include/parameters.hpp`: shared typed vehicle, horizon, cost-weight, constraint, MPC and ALIPDDP solver defaults. The same parameter types serve every OCP. Startup environment loading preserves saved tuning.
- `include/target`: one `target_generator.hpp` for synthetic movements and seeded MC1 shapes; one `predictor.hpp` for constant-acceleration prediction. Neither depends on ROS.
- `include/planner_core`: three public headers: `controller.hpp` (RH/MPC API), `types.hpp` (shared contracts and OCP registry queries), and `adapters.hpp` (frame/command conversion).
- `src`: compiled controller and OCP registry. One private `controller_internal.hpp` contains the shared `OcpSolver`, RH/direct-MPC runtime and async logging implementation; ROS adapters do not include it.
- `support/transport/ros1`, `support/transport/ros2`: ROS adapters, nodes and launch files. The ROS2 Crazyflie adapter remains pending flight validation.
- `config/ros1`, `config/ros2`: matching hover/RH/MPC profiles in each ROS parameter format.
- `support/rviz`: the retained ROS1 viewer settings.
- `support/scripts`: profile application and RViz markers, plus the sequential SITL campaign, its bearing probe, metrics and MPC audit.
- `support/tools/analysis`: the existing bag converter, flight analysis, 3D/state plots, animation and horizon overlay. Flight analysis uses the paper plot/animation scripts; their designs are retained.
- `support/test`: six focused regressions, disabled by default. Retired OCP tests, offline research tools, session patches and duplicate campaign plotters are removed.
- `logs`: runtime output only; saved campaigns contain bags, CSVs, profiles, plots and audits.

Frame conventions and estimator inputs are described in [FRAMES.md](docs/FRAMES.md). The cone-start experiment without CTCS/STC is described in [ORDINARY_LANDING.md](docs/ORDINARY_LANDING.md).

## Run

Source `/opt/ros/noetic/setup.bash` and `/home/lab/mavros_ws/devel/setup.bash`. Launch `planner_launch.launch` for hover/RH or `direct_mpc_sitl.launch` for calibrated x500 MPC; apply a profile from `config/ros1` with `rosrun comando_planner apply_profile.sh`. Direct MPC accepts feedback mode only and publishes at 50 Hz; RH publishes planned state references. See `docs/MAVROS_SITL.md` and `docs/DIRECT_MPC.md` for settings.

## Build and verify

Keep ALIPDDP as a sibling or set `COMANDO_SOLVER_DIR`. Build each executable separately with one worker:

```sh
catkin build comando_planner --no-deps -j1 -p1 --make-args comando_planner
catkin build comando_planner --no-deps -j1 -p1 --make-args target_publisher
```

Set `COMANDO_BUILD_TESTS=ON` explicitly to build the six test targets individually. They cover controller modes, RH acceptance/handoff, trajectory replay, warm-started noaug landing, CTCS values/Jacobians, and direct MPC feedback/derivatives/expiry. Standalone builds select `COMANDO_ROS_VERSION=0`; ROS1 and ROS2 select 1 and 2.

SITL runs always use one target at a time. The already completed RH/MPC circle, figure-eight and straight flights, their source revisions and remaining flown STC violations are recorded in `docs/CLEAN_PLANNER_VALIDATION.md`. Cleanup verification does not repeat those campaigns. The later estimator-normalization regressions and cone-start ordinary-constraint experiments are recorded in [FRAME_CONE_VALIDATION.md](docs/FRAME_CONE_VALIDATION.md).
