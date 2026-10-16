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
| `landing_mpc` | Target-relative | Direct MPC with ordinary constraints on every future state and the terminal node; the fixed measured initial state is recorded separately. |

RH sends planned position, velocity, yaw and acceleration feedforward to PX4's tracking controller. Direct MPC starts each DDP solve from measured state and sends only its first optimized thrust/body-rate input on MAVROS `setpoint_raw/attitude`; PX4's rate controller remains active. The publication loop repeats that first input until the next measured-state solve completes. It never advances through the remaining controls of a solution. The command path contains no separate LQR/QP correction or post-solve slew shaping. The fixed-grid prediction horizon has 40 intervals of 0.1 seconds. Node zero is fixed measured data: input bounds remain there, while pointwise state constraints apply to predicted nodes 1 through N. CTCS also constrains the first predicted interval. This permits a recovery solve after a measured limit excursion without changing the measured initial state or relaxing future bounds. Warm MPC updates retain the solver barrier/penalty continuation and use the previous state/control trajectory as an FDDP seed with a measured initial state. `MPC_MAX_ITER` controls the cold solve; `MPC_WARM_ITER` controls warm updates (default 8). Solve runtime and publication rate are distinct: 100 Hz publication does not establish a 100 Hz MPC update rate. Stale state or command expiry pauses into hold. Touchdown hands the landing action to PX4.

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

The unflown separate tracking-QP experiment was removed. The older custom LQR backward pass, feedback correction and post-solve command shaping were also removed. Both ordinary and CTCS direct MPC publish the first DDP-optimized input unchanged; subsequent solves start from measured state. RH continues to publish DDP state references for PX4 tracking.

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

The reference-consistency RH circle check (`logs/ddp_consistent_reference_landing_rh/circle__zero__t0`, commit `a36f2b6`) landed with clean teardown. Over 20.99 controlled seconds, measured speed, glideslope, LOS and floor excess were zero; tilt excess was 0.00752 degrees, below the audit tolerance. Angular-rate excess remained 0.02414 rad/s over 1.03593 seconds, so the constraint check failed. Publication median was 9.991 ms, with a maximum 82.493 ms gap. There were 128 accepted horizons: predicted speed/tilt/rate/glideslope excess was zero, but small terminal floor/LOS spatial excess remained. This is a circle case and does not isolate improvement against the earlier straight case. The shared ROS2 build passed, without a ROS2 flight.

ROS1 parameter polling now tries the exclusive session lock and retries at the next poll when a solve is active. Previously a waiting parameter writer could block replay until the solve finished. This removes that scheduling stall without changing DDP inputs or physical limits; the subsequent sequential case verified a smaller maximum command gap (see below). RH still seeds attitude/rates from the predicted handoff rather than a validated PX4 tracking-model projection of measured state. Consequently a feasible nominal angular-rate prediction does not certify measured rate feasibility.

The nonblocking-poll RH circle check (`logs/ddp_nonblocking_reference_landing_rh/circle__zero__t0`, commit `86bd3f7`) landed with clean teardown. Across 95.18 controlled seconds and 551 accepted horizons, measured speed, tilt, glideslope, LOS and floor excess were zero. Angular-rate excess was still 0.00607 rad/s for 2.40060 seconds. Command median period was 9.990 ms and maximum gap 16.045 ms, versus 82.493 ms in the preceding case. This confirms the scheduling stall was removed; differing flight durations and engagement phases prevent attributing the entire angular-rate difference to that change. The physical constraint goal remains incomplete.

PX4 RH acceleration conversion (in progress): the x500 FCU logs show `MPC_ACC_DECOUPLE=1` and horizontal velocity derivative gain 0.2. Native PX4 forms its attitude axis from `[acc_internal.xy, g]` in decoupled mode and adds `-D_vel * acc_measured` in the velocity loop. Sending physical DDP acceleration directly therefore does not reproduce the DDP attitude, especially at thrust steps. Each full RH landing OCP now supplies its own MAVROS mapping: `a_ff.xy = g * b.xy / b.z + D_xy * a_ddp.xy` in decoupled mode, `a_ff = a_ddp + D * a_ddp` otherwise, and the corresponding vertical derivative term. Here `b = R(q) e_z` and `a_ddp = (T/m)b + gravity`. The core selects this mapping only for MAVROS; Crazyflie retains physical acceleration. These are nominal native-controller input equations, with no measured-error correction or second optimizer.

The shared parameter library owns the native FCU coefficients (`Px4TrackingParameters`; startup overrides `PX4_ACC_DECOUPLE`, `PX4_VEL_D_XY`, `PX4_VEL_D_Z`). They must match the FCU. The inverse reproduces the desired thrust axis at zero tracking error and zero velocity integrator; it does not yet model feedback transients, learned hover thrust, nonlinear collective tracking or initial integrator state. Physical validation is required. The saved PX4 demand audit aligns ROS/PX4 clocks by matching position/velocity and shows desired-axis rates above 0.09 rad/s over 0.1-second windows during RH control, despite nominal DDP-axis rates below 0.019 rad/s over those windows. Exploratory torque/gyro fits failed held-out prediction and were not used as OCP coefficients.

For this native-conversion change, all six core tests passed (6.12 seconds). The independent reconstruction test also passed with coupled acceleration and zero derivative gain. ROS1 and shared ROS2 planner builds passed sequentially. One ordinary RH circle SITL verification is next; no physical success is claimed from algebraic tests or builds.

The inverse-conversion circle case (`logs/ddp_px4_inverse_landing_rh/circle__zero__t0`, commit `008ce8a`) landed cleanly but failed physical constraints. Across 12.66 controlled seconds and 72 accepted horizons, measured tilt excess was 0.21303 degrees for 0.46784 seconds and angular-rate excess 0.02168 rad/s for 1.12815 seconds. Speed, floor, glideslope and LOS excess remained zero. Peak angular rate occurred 0.169 seconds after handoff, from an initial rate of 0.00129 rad/s; peak tilt occurred at 0.401 seconds. The nominal zero-error input conversion does not model PX4's pre-existing velocity integrator/tracking state. It is not a physical constraint fix by itself. Matching native controller state and closed-loop response inside the DDP prediction is the next required step.

Native-model data capture (`logs/ddp_native_model_identification_landing_mpc/circle__zero__t0`, commit `e6ef4c1`) used the saved MPC tuning and `PX4_PARAM_SDLOG_PROFILE=147` (131 plus native high-rate bit 16). It landed cleanly, but was a failed constraint test: angular-rate excess 0.01656 rad/s for 3.00380 seconds, tilt excess 0.03151 degrees, and glideslope excess 0.00824 m across 3.83 controlled seconds. Eight horizons were accepted. Direct commands were published unchanged every 10.000 ms median, 10.069 ms maximum; no position commands were interleaved during direct control. First controlled measured angular rate was already 0.03154 rad/s, despite the prior feasible-start proof.

The native ULog is copied into that case as `px4_inner_loop.ulg`; `px4_inner_loop.csv` contains 4 ms gyro, motor, torque and rate-request records plus 20 ms rate-integrator records. Source metadata, native allocator/motor parameters and diagnostic model validation are saved alongside it. The source-derived PX4 allocation matrix reproduces nonsaturated motor commands with RMS error 4.25e-5 normalized signal. A source-derived x500 rotor/inertia model explains the excited response substantially better than the sparse fits, but still needs gyro-filter/timing and held-out validation before use as a complete constraint prediction. The logged body-rate request to attitude-disable interval was 188 ms, versus 148 ms in an earlier case. A fixed instantaneous switch is therefore an incorrect assumption.

The former first-plan latency projection assumed a hover body-rate input while PX4 still ran its position/attitude cascade. That projection has been removed from the measured-state MPC path. A detailed motor/PID model has not been established as necessary and the experimental native model remains disabled by default. The measured-state first-input path is validated in core ordinary/CTCS moving-target tests and the ordinary ROS1 circle SITL case recorded below. Full physical constraint satisfaction and broader flight coverage remain separate outstanding work.

Measured-state MPC implementation check (2026-10-14): planner `411044e`, ALIPDDP `e9130217`. The core CTCS moving-target test completed 121 accepted updates in five seconds (maximum warm solve 27.5 ms); the ordinary moving-target test completed 162 updates (maximum warm solve 21.1 ms). Both verify exact first-input publication, measured initial state, and command hold without advancing through a solution. Controller lifecycle, RH checks and the single-job ROS1 build pass.

The sequential ROS1 circle check is saved in `logs/measured_state_ddp_mpc/circle__zero__t0/`. It ended `POLICY_EXPIRED / CLEAN`, with its bag finalized. Nine measured-state solves were accepted before subsequent constrained updates failed (first rejected residual 1.531). Warm solves took about 12–13 ms. All 116 published controls match the corresponding solution's node-zero input within the CSV's six-decimal precision (maximum difference 4.87e-7); publication median was 10.000 ms, maximum 11.430 ms, with no position commands interleaved and OFFBOARD maintained. Physical angular-rate and speed limits were exceeded; this is a failed flight validation, not a completed landing or constraint-satisfaction result. `control_audit.json` and `constraint_audit.json` contain the existing audit outputs. Full successful flight validation remains outstanding.

Measured-root recovery check (2026-10-14): planner `f99d23c`. Fixed measured point constraints at node zero previously made any angular-rate excursion an infeasible OCP; state constraints now apply to predicted nodes 1 through N, with input bounds at all control nodes. CTCS retains its first-interval density constraint. Regression tests preserve an out-of-bound measured rate exactly, verify feasible future states, and keep the feedback loop running after a 0.03 rad/s injected excursion. Both ordinary/CTCS moving-target core tests, controller tests and the sequential ROS1 build pass.

The same single ROS1 circle case now completes `LANDED / CLEAN` in `logs/measured_state_ddp_mpc_recovery/circle__zero__t0/`. Its landing handoff follows 396 accepted DDP updates and zero rejected updates. Median update interval is 20.001 ms (about 50 Hz); warm solve median is 12.491 ms and maximum 25.086 ms. All 800 published inputs equal their solution's first input within six-decimal CSV precision (maximum error 4.99e-7). Publication median is 10.000 ms and maximum 10.685 ms, OFFBOARD is maintained, and no position commands interleave the direct-control interval. The finalized bag, CSVs, `mpc_execution_check.json` and existing audit outputs are saved together. The physical audit finds no speed, tilt, floor, glideslope or LOS excess, but a peak angular-rate excess of 0.002694 rad/s lasting 0.632 s. This establishes the measured-state MPC execution path and fast feedback cadence; it does not establish complete physical constraint satisfaction.

STC MPC verification (2026-10-16, unchanged controller): `logs/measured_state_stc_mpc/circle__zero__t0/` uses the saved circle/original staging profile with `stc_landing_mpc` and eight warm iterations. The case completed `LANDED / CLEAN`, with 303 accepted solves and zero rejected solves. Cold solve: 332.707 ms; warm median/max: 19.456/39.259 ms. New first inputs average 37.845 updates/s, held for 29.990 ms median and 49.999 ms maximum. The main publisher runs at 100 Hz; the adapter's 10 Hz heartbeat adds duplicate publications, giving 110.013 Hz on the ROS command topic. PX4 gyro/rate PID/torque/motor execution is nominally 250 Hz, verified by live uORB status and 4 ms median ULog intervals (about 248 Hz mean in the selected direct-mode window). `IMU_GYRO_RATEMAX=800` is a cap, not this simulation's actual update rate. High-rate logger profile 147 was used only to measure the native loop.

Measured staging was active initially and last active at 2.570 s after activation. LOS activated below 1.2 m at 2.902 s, and the tight landing constraints below 0.8 m at 4.928 s. Activation and landing handoff work, but constraint satisfaction does not pass: measured maxima above limits were LOS 7.432 degrees, glideslope 0.04661 m, staging height 0.04220 m and angular rate 0.003933 rad/s. The current dense model audit also finds nominal future pointwise excesses (including tilt 3.007 degrees and LOS 11.377 degrees), with maximum local dynamics defect 0.003253. The issue therefore is not solely physical tracking. Reports and the copied native ULog are saved alongside the finalized bag and CSVs; no controller parameters or source behavior were changed during this verification.

Warm-start semantics: X/U are trajectory seeds and x0 is replaced by measured state. `updateProblem` retains scalar mu/muT and floors them at 1% of their initial values (0.01 with these defaults); inequality slacks S/ST and duals Y/YT are reconstructed from the new constraints, not copied from the previous solve. Scalar rho/rhoT and generic equality-multiplier storage are retained, but these two MPC OCPs have no explicit equality-constraint rows; FDDP handles dynamics gaps. A numerical initialization issue remains to investigate: the scalar slack floor is 0.01 while the CTCS row epsilon is 0.00016, so even a zero-density interval starts with primal residual 0.00984 before Newton updates. Eight iterations and the existing aggregate residual check have not established full CTCS feasibility. This observation identifies a candidate numerical cause, not a verified causal fix.

Saved STC MPC / historical RH plots (2026-10-16): the existing paper plotters generate controlled-interval state plots, world/relative 3D views and twin-view GIFs directly in each run folder. This is a visual comparison of saved circle runs: MPC staging height is 1.30 m and historical RH staging height is 1.55 m, with different source revisions, so the pair is not a matched experiment.

| Saved run | States | 3D | GIF |
|---|---|---|---|
| Measured-state STC MPC | [States](logs/measured_state_stc_mpc/circle__zero__t0/stc_flight_states.png) | [3D](logs/measured_state_stc_mpc/circle__zero__t0/stc_flight_3d.png) | [GIF](logs/measured_state_stc_mpc/circle__zero__t0/stc_flight.gif) |
| Historical STC RH replanning | [States](logs/frame_regression_rh/circle__zero__t0/stc_flight_states.png) | [3D](logs/frame_regression_rh/circle__zero__t0/stc_flight_3d.png) | [GIF](logs/frame_regression_rh/circle__zero__t0/stc_flight.gif) |

The measured-state MPC box-flow diagram is saved with the MPC run: [PNG](logs/measured_state_stc_mpc/circle__zero__t0/mpc_loop.png), [editable SVG](logs/measured_state_stc_mpc/circle__zero__t0/mpc_loop.svg), [PDF](logs/measured_state_stc_mpc/circle__zero__t0/mpc_loop.pdf). Plotting code and planner behavior are unchanged.

General control architecture and saved-run comparison (2026-10-16): [comparison report](logs/measured_state_stc_mpc/circle__zero__t0/rh_mpc_comparison.md), [numerical metrics](logs/measured_state_stc_mpc/circle__zero__t0/rh_mpc_comparison.json). MPC's observed control interval was 8.33 s versus historical RH's 9.63 s, and warm solve medians were 19.46 ms versus 89.11 ms, but RH had smaller measured constraint violations and lower angular motion. These are unmatched saved runs with different staging heights, initial positions, iteration budgets, tracking interfaces and source revisions; the RH snapshot predates later transport timing fixes. They do not establish that either controller is generally superior.

The revised general architecture uses grouped problem/optimizer/controller blocks inspired by Figure 2 of the user-supplied Wang et al. paper, without case-specific frequencies or settings: [PNG](logs/measured_state_stc_mpc/circle__zero__t0/mpc_architecture_general.png), [SVG](logs/measured_state_stc_mpc/circle__zero__t0/mpc_architecture_general.svg), [PDF](logs/measured_state_stc_mpc/circle__zero__t0/mpc_architecture_general.pdf). It depicts one DDP optimization problem, first-input application, target prediction outside the OCP, PX4 rate tracking/allocation, and measured-state feedback.
