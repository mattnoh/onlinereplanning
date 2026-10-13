# CoManDO planner

Current documentation for the active `mpc` branch, updated 2026-10-14. The MAVROS/PX4 ROS1 planner uses a ROS-independent core for RH replanning and direct MPC. The previous planner is archived in `../comando_planner_old` and excluded from catkin discovery. The shared ROS2 adapter builds; Crazyflie MPC flight validation and deployment remain pending.

## Layout

| Location | Purpose |
|---|---|
| `include/ocp/` | Five complete OCPs. Each file owns its dynamics, costs, constraints, derivatives and descriptor. |
| `include/parameters.hpp` | Shared typed vehicle, horizon, cost, constraint, MPC and solver defaults; startup environment overrides preserve saved tuning. |
| `include/target/` | One target generator and one constant-acceleration predictor, independent of ROS and the OCP files. |
| `include/planner_core/` | Three public headers: controller API, shared types/registry and coordinate adapters. |
| `src/` | Compiled core and registry; solver, RH/MPC runtime and asynchronous logging internals. |
| `support/transport/ros1/`, `support/transport/ros2/` | ROS nodes, platform adapters and launch files. |
| `config/ros1/`, `config/ros2/` | Hover, CTCS RH/MPC and ordinary RH/MPC profiles. |
| `support/scripts/`, `support/tools/analysis/` | Profile application, sequential campaigns, metrics, physical/control audits and existing plotters. |
| `support/rviz/`, `support/test/` | Retained viewer settings and six opt-in regressions. |
| `logs/` | Bags, CSVs, applied profiles, source/environment manifests, results, plots and GIFs. |

## OCPs and execution

| OCP | Coordinates | Execution and constraints |
|---|---|---|
| `hover` | World | Startup/staging state references. |
| `stc_landing_noaug` | Target-relative | RH with variable time steps and the shared CTCS density interface. |
| `stc_landing_mpc` | Target-relative | Fixed-grid direct MPC with the shared CTCS density interface. |
| `landing_rh` | Target-relative | RH with ordinary constraints active throughout the horizon and terminal node. |
| `landing_mpc` | Target-relative | Direct MPC with ordinary constraints active throughout the horizon and terminal node. |

RH sends planned position, velocity and yaw to PX4's tracking controller. Direct MPC sends thrust/body-rate commands on MAVROS `setpoint_raw/attitude`; PX4's rate controller remains active. An optimization worker supplies DDP controls for 100 Hz transport publication. PX4 performs rate/thrust tracking. The command path applies each optimized control with the OCP's zero-order hold; it contains no separate LQR/QP correction or post-solve slew shaping. The fixed-grid MPC horizon has 40 intervals of 0.1 seconds. The next measured-state DDP solve starts when the worker is free and its scheduled policy has activated; there is no extra 10 Hz request cap. Rejected solves retain a valid policy; stale state or policy expiry pauses into hold. Touchdown hands the landing action to PX4.

Both CTCS OCPs use `StcStageConstraint` in density form with analytic trigger/consequence derivatives. Noaug preserves its 14-state layout, original eight implications and scaled interval budget. The migration parity checks compared the shared row with the independent density/integral, including free-time derivatives, settle/capture/disabled-STC modes and nonzero trigger floors.

The ordinary OCPs contain no CTCS interval rows, violation integrators, state triggers, staging conditions or trigger-dependent costs. They share defaults and prediction with the CTCS versions but use an unconditional landing objective. Their always-active limits are 0.75 m/s relative speed, 0.95 degrees tilt, 0.019 rad/s body angular speed, 15 degrees glideslope with a 0.1 m apex offset, 30 degrees LOS and a zero relative-altitude floor. Speed, angular rate, tilt, glideslope and LOS use native SOC rows; the floor and actuator bounds use ordinary inequalities. Quaternion-dependent constraints normalize attitude.

## Estimator and frame contract

The core receives world position and linear velocity, a body-to-world quaternion in wxyz order, and body angular velocity in rad/s. World axes are ENU; body axes are FLU. Odometry describes the body origin; target odometry describes the landing point.

OCP selection never changes the estimator subscription. `World` uses canonical state directly. `TargetRelative` subtracts target position and velocity with world-aligned axes; target yaw does not rotate the state. Attitude remains body-to-world and angular velocity remains in the body frame. RH reconstructs commands using the live target, while planned paths use constant-acceleration prediction.

ROS1 defaults:

| Parameter | Default |
|---|---|
| `drone_odom_topic` | `/mavros/local_position/odom` |
| `state_velocity_frame` | `body` |
| `world_frame` | `map` |
| `target_odom_topic` | `/target/odom` |
| `target_velocity_frame` | `world` |
| `command_frame` | `map` |

Velocity conventions accept `body` or `world`. Target acceleration uses `/target/accel`. Configure topics and conventions at node startup. Inputs are transformed into the configured world frame using TF at message timestamps. Missing frames, timestamps or TF reject the input; timestamps must be no more than 0.2 seconds old or 1 ms in the future. MAVROS state-reference commands transform into its local ENU command frame.

For another estimator, supply calibrated static TF and full SI odometry. For example:

```sh
roslaunch comando_planner planner_launch.launch platform:=mavros \
  drone_odom_topic:=/estimator/odom state_velocity_frame:=world \
  world_frame:=map command_frame:=map
```

Mocap can feed PX4 through MAVROS vision/mocap for fused local odometry, or feed an external estimator. Raw pose alone lacks velocity and body rates. The producer handles marker-to-body calibration, and vehicle/target origins and axes must agree. Moving coordinate frames need additional velocity transport terms and are outside this contract. Old shifted/rotating relative-input modes and relative vehicle-topic switches are removed.

The ROS2 Crazyflie native adapter uses `world`, rotates native body velocity into world axes, and converts native gyro units into rad/s. A configured external odometry source supplies full SI state. Build compatibility does not establish Crazyflie flight performance.

## Build and launch ROS1

Run these commands in bash, or source the corresponding `.zsh` setup files in zsh. Keep `ALIPDDP-main` as a sibling, or set `COMANDO_SOLVER_DIR`.

```sh
source /opt/ros/noetic/setup.bash
source /home/lab/mavros_ws/devel/setup.bash
catkin build comando_planner --no-deps -j1 -p1 --make-args comando_planner
catkin build comando_planner --no-deps -j1 -p1 --make-args target_publisher
```

Build one executable at a time with one worker. Use the installed PX4 simulator; campaigns do not rebuild PX4. Run one target at a time and do not build alongside SITL.

With PX4/MAVROS running, launch the target and planner in separate terminals:

```sh
roslaunch comando_planner target_launch.launch target_trajectory:=circle
roslaunch comando_planner planner_launch.launch platform:=mavros
rosrun comando_planner apply_profile.sh /comando_planner \
  /home/lab/mavros_ws/src/CoManDO_planner/config/ros1/profile_hover_sitl.yaml
```

After hover/staging, apply `profile_stc_landing_noaug_sitl.yaml` for CTCS RH. For direct MPC, use `roslaunch comando_planner direct_mpc_sitl.launch` from startup, then stage and apply `profile_stc_landing_mpc_sitl.yaml`. This launcher aligns the adapter/model thrust calibration. The profile script increments `command_seq` to activate a command. Mathematical environment settings load at process startup; restart to change them.

Saved MC1 RH settings include `SZMUK_RH_N=40`, `SZMUK_MAX_ITER=120`, `SZMUK_THH=0.16`, `SZMUK_W_LAND_GS=20000`, replay prefix 1, a 1.55 m staging floor and 1.2 m LOS trigger. Terminal freeze velocity threshold is 0.30 m/s and the staging hover command is 2.05 m.

Validated x500 direct-MPC settings:

| Setting | Value |
|---|---:|
| Normalized hover thrust | 0.7282821834 |
| Effective rate-response time | 0.08 s |
| Body-rate command bound | 0.35 rad/s |
| Maximum optimizer iterations | 120 |
| Maximum policy age | 2 s |
| CTCS staging floor / LOS trigger | 1.3 m / 1.2 m |
| Raw CTCS interval epsilon / scaling | 0.0016 / 10 |
| Thrust-acceleration / body-rate slew | 20 m/s³ / 5 rad/s² |

These are simulator calibration estimates. The first-order rate response and linear thrust mapping approximate the plant. The next solve runs when the worker and policy handoff are ready; command publication runs at 100 Hz. Publication frequency is distinct from the DDP solve frequency.

## Sequential cone-start experiment

Use `profile_landing_rh_sitl.yaml` or `profile_landing_mpc_sitl.yaml` with the campaign's `--start cone`. Preparation follows the target 1.85 m above its landing point, with acceleration/jerk limiting for high-curvature motion. Every tight measured landing limit must hold for 0.5 seconds. Preparation continues until the first planner command, with a 30-second start timeout and a first-plan check after 15 seconds. Cases stop on failure. A successful case requires the planner's touchdown handoff, PX4 disarm and clean teardown.

Example for one ordinary RH target:

```sh
python3 support/scripts/run_scenario_campaign.py \
  --motions circle --levels zero --trials 1 --start cone \
  --hover-thrust 0.7282821834 --stc-z-stage 1.55 --stc-los-alt-trig 1.2 \
  --profile config/ros1/profile_landing_rh_sitl.yaml \
  --out logs/ordinary_rh_check
```

For MPC, use `profile_landing_mpc_sitl.yaml`, staging value 1.3, and the saved startup environment: `MPC_RATE_TAU=0.08`, `MPC_HOVER_THRUST=0.7282821834`, `MPC_MAX_ITER=120`, `MPC_MAX_POLICY_AGE=2`, `MPC_RATE_LIMIT=0.35`. Refer to each saved cell's environment/profile for exact reproduction settings.

## Logs, analysis and plots

Keep outputs together under `logs/<campaign>/<motion>__zero__t0/`. Cells save the bag, solver CSVs, `planner_environment.json`, `landing_profile.yaml`, `source.json`, `cell.json` and metrics. Direct MPC additionally records nominal/applied controls, feedback, limits, policy age and tracking errors in `direct_mpc_controls.csv`. Inspect `cell.json` and teardown status before treating a run as complete.

`scenario_metrics.py` produces campaign metrics. For physical constraint scoring, enable `COMANDO_BUILD_ANALYSIS` and build `constraint_audit` separately, then run:

```sh
python3 support/scripts/analyze_direct_mpc.py logs/ordinary_landing_mpc \
  --physical --audit-bin /path/to/constraint_audit
python3 support/scripts/analyze_direct_mpc.py logs/ordinary_landing_mpc
```

The physical audit checks accepted future nodes, 1 ms interval propagation, full-horizon continuous rollouts and measured state. Ordinary measured scoring begins at first controller output and ends at touchdown handoff or policy expiry. Preparation, later hover and post-handoff disarm are excluded. No accepted plan means no measured controller result. Horizon durations overlap and are summed, not elapsed flight time.

`support/tools/analysis/analyze_flight.py` converts bags and invokes the existing plotters. `bag_to_stc_csv.py` supplies executed-state CSVs; its `--t0`/`--t1` trim is relative to first vehicle odometry. Use saved environment settings and the controlled interval for correct bounds and scoring. Existing `plot_stc_paper.py` and `animate_stc_paper.py` generate state plots, world/relative 3D views and GIFs. Horizon tools show planned states and overlays. Ordinary plots use always-active limits and the altitude floor, with no staging or STC trigger annotations.

Historical campaigns remain physically under `logs/ros1_current_rh` and `logs/ros1_current_mpc`; former `.Codex` paths are compatibility symlinks. Earlier initialization/preparation diagnostics remain separate from final ordinary cases. Deleting old documentation does not delete these data or their source manifests.

## Latest validation

Six existing core tests passed: normalized coordinates, lifecycle/modes, replay and acceptance/handoff, CTCS parity, noaug warm starts, direct dynamics/feedback/expiry, and ordinary cone/terminal derivatives and start feasibility. Stationary and measured moving-start cold solves passed. The active ROS1 and shared ROS2 planner builds passed sequentially. Tests are opt-in (`COMANDO_BUILD_TESTS=ON`); standalone builds select `COMANDO_ROS_VERSION=0`, ROS1/ROS2 use 1/2.

Latest flight campaigns used one zero-noise trial per circle, figure-eight and straight case, sequentially. All final cases shut down cleanly. Successful flights were not repeated during final analysis. Data are in:

- [CTCS RH regression](logs/frame_regression_rh)
- [CTCS MPC regression](logs/frame_regression_mpc)
- [Ordinary RH cone starts](logs/ordinary_landing_rh)
- [Ordinary MPC cone starts](logs/ordinary_landing_mpc)

## Flight outcomes

| Formulation | Execution | Circle | Figure-eight | Straight |
|---|---|---|---|---|
| CTCS/STC | RH | LANDED (37/0) | LANDED (32/0) | LANDED (32/0) |
| CTCS/STC | MPC | LANDED (31/1) | LANDED (24/0) | LANDED (30/5) |
| Ordinary | RH | LANDED (33/0) | INITIALIZATION_FAILED (0/48) | LANDED (23/16) |
| Ordinary | MPC | TIMEOUT (2/11) | INITIALIZATION_FAILED (0/3) | LANDED (14/8) |

Parentheses show accepted/rejected solves. Initialization failure means that no trajectory was accepted with the saved settings; it does not prove mathematical infeasibility. Figure-eight satisfied the measured cone-start gate in both ordinary cases. MPC circle accepted two plans, rejected later solves, expired its policy, returned to hover, and timed out. MPC straight retained OFFBOARD throughout its direct-control interval. Both ordinary MPC cases that controlled used approximately 20 ms command periods and had no interleaved position commands.

The RH figure-eight harness originally labeled its result `NO_FEASIBLE_START` because it inspected buffered console output. Its bag contains a verified feasible start and no accepted plan. The retained metadata records both the original harness result and corrected `INITIALIZATION_FAILED` diagnosis. Subsequent cases use the explicit ROS verification flag.

## Measured ordinary constraint excess

Values below are excess above the limit, during first planner output through touchdown handoff or policy expiry. Preparation, post-expiry hover and post-handoff disarm are excluded. No-control cases have no measured constraint result.

| Execution / motion | Duration s | Speed m/s | Tilt deg | Body rate rad/s | Glideslope m | LOS deg |
|---|---:|---:|---:|---:|---:|---:|
| RH / circle | 7.000 | 0.000000 | 0.000000 | 0.023606 | 0.010763 | 0.000000 |
| RH / figure8 | No control | — | — | — | — | — |
| RH / straight | 5.500 | 0.337658 | 7.317067 | 0.622878 | 0.269139 | 8.746057 |
| MPC / circle | 2.100 | 0.000000 | 0.537883 | 0.191311 | 0.000000 | 0.000000 |
| MPC / figure8 | No control | — | — | — | — | — |
| MPC / straight | 4.098 | 0.000000 | 0.083959 | 0.190951 | 0.000000 | 0.000000 |

Measured altitude floor remained satisfied in all controlled ordinary cases. Staging is absent from these OCPs. Landing is an outcome, not a certificate that all constraints were maintained.

## Predicted ordinary constraint excess

Accepted future nodes exclude each measured initial state. The 1 ms audit propagates each interval from its node; a separate continuous rollout propagates the complete horizon without resetting. All accepted ordinary initial states passed the physical audit. The small mixed-unit propagation defect is not a physical safety margin.

| Execution / motion | Future-node floor m | Future-node LOS cone m | Dense LOS cone m | Full-rollout LOS cone m | Max dynamics defect norm |
|---|---:|---:|---:|---:|---:|
| RH / circle | 0.0038080 | 0.0075656 | 0.0075657 | 0.0075231 | 4.74e-06 |
| RH / figure8 | No accepted horizon | — | — | — | — |
| RH / straight | 0.0035850 | 0.0057197 | 0.0067208 | 0.0067262 | 3.98e-06 |
| MPC / circle | 0.0000000 | 0.0000000 | 0.0000103 | 0.0001020 | 2.47e-05 |
| MPC / figure8 | No accepted horizon | — | — | — | — |
| MPC / straight | 0.0000000 | 0.0000000 | 0.0003553 | 0.0003816 | 3.47e-05 |

RH future-node speed, tilt, angular rate and glideslope excesses stayed below audit tolerance, but floor/LOS errors of several millimetres remained. MPC future nodes passed the physical tolerances; dense/continuous propagation still produced LOS cone excess, especially near the terminal point. LOS angles become ill-conditioned near zero range: large reported angles there correspond to small spatial cone errors. The spatial metric is `norm(lateral) - tan(30 deg) * axial`, using the normalized body vertical axis. It complements, rather than replaces, the angle audit.

Removing CTCS/STC therefore did not eliminate measured violations. In RH the PX4 tracking controller follows state references; in MPC feedback, command slew and actuator response change execution from the nominal horizon. The existing solver acceptance threshold can also accept physically imperfect trajectories. These data support inspecting physical acceptance checks, execution/model consistency and tracking margins next; this experiment does not change those policies or relax the limits.

The CTCS regression flights also retain predicted and measured violations. All six landed, but this was not a constraint-feasibility pass. For example, CTCS RH figure-eight measured 0.13015 m glideslope excess and 2.04682 deg tilt excess; CTCS MPC straight measured 1.77361 deg tilt and 0.15870 rad/s rate excess. Each campaign contains full per-constraint audits.

The comparison changes both start geometry and formulation/objective. CTCS retained its original engagement starts; ordinary cases started inside the cone with all tight limits checked. As agreed, CTCS was not rerun from matched starts. With one trial per case, these results cannot assign every difference to CTCS or establish statistical equivalence.

## Existing plots from controlled ordinary runs

| Ordinary controlled run | States | 3D | GIF |
|---|---|---|---|
| RH circle | [States](logs/ordinary_landing_rh/circle__zero__t0/flight_states.png) | [3D](logs/ordinary_landing_rh/circle__zero__t0/flight_3d.png) | [GIF](logs/ordinary_landing_rh/circle__zero__t0/flight.gif) |
| RH straight | [States](logs/ordinary_landing_rh/straight__zero__t0/flight_states.png) | [3D](logs/ordinary_landing_rh/straight__zero__t0/flight_3d.png) | [GIF](logs/ordinary_landing_rh/straight__zero__t0/flight.gif) |
| MPC circle | [States](logs/ordinary_landing_mpc/circle__zero__t0/flight_states.png) | [3D](logs/ordinary_landing_mpc/circle__zero__t0/flight_3d.png) | [GIF](logs/ordinary_landing_mpc/circle__zero__t0/flight.gif) |
| MPC straight | [States](logs/ordinary_landing_mpc/straight__zero__t0/flight_states.png) | [3D](logs/ordinary_landing_mpc/straight__zero__t0/flight_3d.png) | [GIF](logs/ordinary_landing_mpc/straight__zero__t0/flight.gif) |


## DDP-to-PX4 execution correction (in progress)

The unflown separate tracking-QP experiment was removed. The older custom LQR backward pass, feedback correction and post-solve command shaping were also removed. Both ordinary and CTCS direct MPC now publish the DDP-optimized inputs unchanged, and latency projection uses the same ZOH controls. RH continues to publish DDP state references for PX4 tracking.

The non-ROS solver runs are the reference for checking the ROS/PX4 mismatch. This correction does not establish physical constraint satisfaction: matched simulator checks, actuator/tracking model validation, and faster warm-started DDP updates remain required. No physical limit or solve-acceptance threshold was changed.

Verification for this correction: all six core tests passed, including exact DDP control delivery and scheduled-policy handoff. ROS1 and shared ROS2 planner builds passed sequentially. The historical ROS horizon audits are not matched comparisons with the non-ROS driver. New SITL constraint validation remains pending.

The first DDP-only ordinary MPC straight run (`logs/ddp_only_landing_mpc`) timed out after policy expiry. Its accepted horizon had zero future-node physical excess, but first controlled measured angular speed was 0.05753 rad/s. During the first solve, the startup reference was frozen despite a nonzero moving-target velocity; that reference update is now maintained until DDP handoff.

RH MAVROS commands now include world acceleration feedforward computed by the selected OCP as `a = R(q) * [0,0,T/m] + g`. The full equations stay in each OCP. Target acceleration is not added twice. Hold/preparation commands have no fabricated thrust feedforward. MAVROS rotates acceleration into the command frame without translating it. Six core tests and the ROS1 build pass; simulator verification remains required.

The x500 SITL launch uses the actuator inverse `signal = (hover + offset) * (a/9.81)^(1/exponent) - offset`, with exponent 2 and offset `150/850`. The measured PX4 ESC range is 150–1000 rad/s and `THR_MDL_FAC=0`; Gazebo rotor thrust is quadratic in rotor speed. This converts the physical DDP thrust input to the simulator signal without adding feedback. Generic parameter defaults retain exponent 1 and offset 0; override `MPC_MOTOR_THRUST_EXPONENT` and `MPC_MOTOR_SPEED_OFFSET` for the identified actuator model. Both MPC OCPs use the same mapping to derive their achievable thrust ceiling. This mapping still needs SITL validation; it is not a robust constraint guarantee.

Follow-up evidence: `logs/ddp_startup_landing_mpc/straight__zero__t0` ended TIMEOUT/CLEAN after two accepted plans. Initial measured angular speed improved to 0.02489 rad/s and controlled glideslope excess was zero, but maximum rate excess was 0.02895 rad/s and tilt excess 0.12268 degrees. `logs/ddp_motor_mapping_landing_mpc/straight__zero__t0` ended TIMEOUT/CLEAN after one accepted plan; it remains a failed constraint test. That run included an abrupt 0.90718 m/s target-velocity reversal at 0.849 seconds after handoff, unlike the preceding two runs. Thus the cases do not isolate the thrust mapping's effect. A drone closely matching pre-turn velocity cannot instantaneously match the reversal; anticipation or an explicitly bounded target-motion model is required. `target_velocity_jump.json` records the measured event. No change to the target generator or predictor has yet been made.

The sequential campaign now reads `control_stop_reason` and records policy expiry/stale-state failure promptly. This avoids waiting five minutes for a stopped controller to land and does not alter the controller, constraints or solver acceptance.

The shared ROS2 planner build passed after the acceleration-feedforward and motor-mapping changes. ROS2 flight validation remains pending.

RH reference consistency: accepted trajectories now use the OCP model to sample states between optimizer nodes under the unchanged optimized input. Each plan retains its own target forecast and time origin; live target measurements enter the next solve instead of deforming the active world reference. Handoff seeds convert through world coordinates when the target forecast changes. Both execution modes publish at 100 Hz, independently of solve frequency. All six standalone tests and the ROS1 build passed for these changes.

The earlier feedforward-only RH straight run (`logs/ddp_feedforward_landing_rh/straight__zero__t0`) landed but failed constraints: measured speed excess 0.33991 m/s, tilt excess 7.56934 degrees, angular-speed excess 0.60325 rad/s and glideslope excess 0.21684 m. It predates the reference-consistency fixes and is not evidence that they work.

PX4 inner-loop extracts are saved as `px4_inner_loop.csv` with source metadata alongside the three DDP-only MPC cases. The motor-mapping case shows approximately 148 ms between the first body-rate mode request and disabling PX4 attitude control; the first optimized control interval was not delivered as modeled. The current first-order MPC rate model also omits PX4 rate integrator and motor dynamics. These are unresolved execution-model differences; adding an independent correction controller would not resolve the requested DDP formulation.
