# Clean planner validation

Candidate source: `/tmp/comando-clean`. Active planners remain unchanged until all required SITL gates pass. No directory cutover has been performed.

Reference commits: PX4 `37fc32f`; Crazyflie `fb424a6` (checkpoint of existing log deletions). The existing Crazyflie solver changes were checkpointed as `0a3a9db`; the clean core currently links the shared CTCS solver in the MAVROS workspace.

Implemented: compiled shared library, common controller API, ROS1/ROS2 wrappers, independent complete direct MPC OCP, external prediction, typed landing parameter defaults, legacy OCP removal, structural validation, profile/reset locking, and separate Crazyflie direct-control publication.

Passed standalone executable checks: planner core, controller lifecycle/mode/invalid input, replay, CTCS parity (160 randomized samples), stateswitch prediction, target-frame warm start, polynomial OCP, five warm-started RH landing solves, and direct MPC derivatives/feedback/expiry. Direct dynamics Jacobian error was 7.53e-10 and CTCS Jacobian relative error 3.43e-10.

SITL and transport checks are recorded below as they complete. A successful build or model test is not a SITL landing result.

## ROS1 first

The ROS1 planner and target-publisher executables rebuilt successfully after checkpoint `84fc44c`. Validation priority is MAVROS/PX4: RH circle, figure-eight, straight; then direct MPC for the same motions. Crazyflie runtime checks resume only after this gate. Each executable builds separately with one worker; each target flies alone.

The first clean RH circle run (`a2d5561`, seed 8570001, saved MC1 tuning) landed and disarmed with clean teardown: 36 accepted / 0 rejected solves, median solve 89.1795 ms, zero logged deadline misses, 0.0970 m touchdown lateral error. Artifacts are in `/tmp/comando-validation/px4_rh/circle__zero__t0`. Historical post-CTCS circle error was 0.1070 m; this single trial does not establish an accuracy improvement.

New campaign cells save the planner revision, launch setup, landing profile, and inherited SZMUK/MPC environment. The bag converter now reads this saved environment and uses the actual noaug raw-altitude trigger and 0.95-degree tilt default. Earlier historical metric files remain unchanged; constraint metrics with corrected semantics should not be compared directly to their old values.

### Clean ROS1 RH results

| Motion | Result | Lateral error (m) | Median solve (ms) | Accepted / rejected | Deadline misses | Teardown |
|---|---|---:|---:|---:|---:|---|
| circle | LANDED | 0.0970 | 89.1795 | 36 / 0 | 0 | CLEAN |
| figure8 | LANDED | 0.2349 | 89.0105 | 31 / 8 | 0 | CLEAN |
| straight | LANDED | 0.0396 | 93.2906 | 34 / 0 | 0 | CLEAN |

Figure-eight constraint rejections occurred during one measured-seed transition below the landing altitude trigger, around solve 24. The valid active trajectory remained available and the planner recovered without a logged deadline miss. The old reference had 37 accepted / 0 rejected solves and 0.2910 m lateral error. This regression remains under review; phase-dependent single trials do not isolate its cause. Full new data: `/tmp/comando-validation/px4_rh`.

### Clean ROS1 direct MPC results

| Motion | Result | Lateral error (m) | Median solve (ms) | Accepted / rejected | Maximum feedback gap (ms) |
|---|---|---:|---:|---:|---:|
| circle | LANDED | 0.0421 | 299.4026 | 25 / 1 | 20.11 |
| figure8 | LANDED | 0.0458 | 306.4716 | 12 / 13 | 21.41 |
| straight | LANDED | 0.0393 | 162.3043 | 29 / 0 | 20.10 |

All three had clean teardown, remained OFFBOARD during direct control, emitted zero interleaved position commands, and respected the 20 m/s³ thrust and 5 rad/s² rate slew bounds. Figure-eight's maximum retained policy age was 1.8918 s against the saved 2 s limit. Executed STC violations remain nonzero; landing success does not demonstrate exact constraint satisfaction on the simulated plant. Existing audit tool results and complete data are under `/tmp/comando-validation/px4_mpc`; binary/source hashes are in `/tmp/comando-validation/ros1_manifest.json`.

The direct MPC test rebuilt against the current shared library and passed derivatives (7.53015e-10 dynamics, 3.43396e-10 CTCS), moving-target feedback (13 accepted solves, 228 ticks), and expiry. A subsequent ROS1-only cleanup removed the unused Crazyflie state stub and unreachable platform fallbacks; the planner rebuilt successfully. No OCP/controller math changed in this cleanup.

### Fresh unchanged ROS1 comparison

A fresh run of the unchanged installed planner (source checkout `37fc32f`) with the same figure-eight seed and saved MC1 tuning also landed with clean teardown: 32 accepted / 4 rejected solves, 85.4841 ms median solve, 0.2712 m lateral error, zero logged deadline misses. Artifacts: `/tmp/comando-validation/px4_old_review`. The historical zero-rejection figure-eight result is not invariant across fresh runs. The candidate's eight rejections remain recorded; these unmatched wall-clock activation phases cannot establish a regression cause.

Final sequential executable checks all passed: controller, planner core, replay, CTCS (160 samples), and five warm-started noaug solves (distance 3.70945 → 0.114301 m). The candidate normalizes MAVROS body-frame velocity at its sensor boundary, including RH; the old node performed this conversion for direct MPC only. This is an explicit frame-consistency change outside the unchanged noaug OCP equations.

Durable candidate copy: `/home/lab/mavros_ws/src/.Codex/comando-planner-clean-candidate`; validation data copy: `/home/lab/mavros_ws/src/.Codex/comando-planner-validation`. The workspace-level `.Codex/CATKIN_IGNORE` prevents candidate package duplication during active workspace discovery. Active planner directories still have not been renamed or replaced. Crazyflie validation and cutover remain pending.

### Final ROS1 wrapper smoke

After removal of the unused ROS1 Crazyflie stub, a fresh direct MPC circle run also landed and tore down cleanly: 0.0129 m lateral error, 25 accepted / 2 rejected solves, 307.8696 ms median accepted solve, maximum feedback gap 20.2843 ms, OFFBOARD throughout, and no position commands during direct control. Data: `/tmp/comando-validation/px4_final_smoke`.

The moved plotters retain their existing designs; only their default log-directory lookup was adjusted for the new `tools/analysis` location. No new plot outputs were generated. Remaining polynomial prediction implementations in the Crazyflie stateswitch OCPs were replaced with the external predictor alias before ROS2 validation.

## Current ROS1 revalidation before further Crazyflie work

At the user's request, Crazyflie MPC investigation is paused. The clean candidate at `d58e631` was rebuilt for MAVROS/ROS1, with one executable target and one build worker at a time. Solver revision: `d0c05cfb`. Ten standalone executables passed: controller, planner core, replay, target-frame warm start, stateswitch prediction, polynomial OCP, augmented STC landing, noaug STC landing, noaug CTCS, and direct MPC. CTCS parity also passed default, settle, capture, disabled-STC, and nonzero-floor configurations (160 samples each). Direct MPC passed analytic derivatives, asynchronous submission, quaternion-sign feedback, policy expiry, and moving-target model feedback (14 accepted solves, 230 feedback ticks).

The augmented STC test initially failed when incorrectly run under the direct-MPC stage/LOS tuning; it passed with its own default benchmark tuning. No formulation or assertion was changed to make it pass.

All six fresh flights below ran strictly sequentially, using the saved MC1 target seeds and RH/MPC tuning. All disarmed after terminal freeze and had clean teardown and zero logged deadline misses. Open-loop hover/staging and the runtime transition into RH or direct MPC were exercised in each cell.

| Mode | Motion | Result | Lateral offset at touchdown (m) | Median solve (ms) | Accepted / rejected |
|---|---|---|---:|---:|---:|
| RH | circle | LANDED | 0.0978 | 85.676 | 31 / 0 |
| RH | figure8 | LANDED | 0.2256 | 88.0465 | 32 / 0 |
| RH | straight | LANDED | 0.0527 | 89.726 | 37 / 0 |
| MPC | circle | LANDED | 0.0285 | 306.4575 | 24 / 2 |
| MPC | figure8 | LANDED | 0.0303 | 316.5348 | 18 / 0 |
| MPC | straight | LANDED | 0.0395 | 181.3877 | 28 / 0 |

Direct control audits passed for all three motions: median feedback period 20 ms, maximum gap 20.37 ms, OFFBOARD throughout, no interleaved position commands, and bounded thrust/rate slew (20 m/s³ and 5 rad/s², within floating-point tolerance). Maximum retained policy age was 0.9004 s, below the saved 2 s expiry. Recorded MAVROS attitude packets were finite, had the correct IGNORE_ATTITUDE mask, normalized thrust within [0,1], and body rates within ±0.35 rad/s. Core policy expiry is covered by the standalone test; transport sensor-loss behavior was reviewed in source but not separately fault-injected in these six flights.

Landing/transport checks passed; exact flown STC satisfaction did not. Nonzero executed glideslope/tilt/rate violations remain in the metrics, particularly figure-eight. These are single trials with wall-clock target phase differences, not a statistical MC1 repeat or proof of identical historical trajectories.

A queued MPC command initially had the wrong hover-thrust argument; it was cancelled before MPC launched. Cancelling that queue interrupted its RH figure-eight child before campaign finalization, despite a logged terminal freeze. Its logs are retained separately under `ros1_queue_interrupted`, with an explicit interruption record; it is not counted as a validated landing. The repeated figure-eight cell above completed normally. All three MPC cells used hover thrust `0.7282821834`, matching `MPC_HOVER_THRUST`.

Artifacts: `ros1_current_rh`, `ros1_current_mpc`, `ros1_current_checks`, and `ros1_queue_interrupted` beneath `/tmp/comando-validation` and the durable `.Codex/comando-planner-validation` copy. The checks directory includes tested source/solver revisions, executable SHA-256 hashes, build/test logs, and transport-packet audit. Candidate logs remain empty. Active planner directories have not been replaced or renamed; Crazyflie MPC still has an actual failed flight and remains unvalidated.

## Crazyflie runtime validation

The ROS2 candidate planner and target executables rebuilt sequentially against the same shared CTCS solver. CrazySim uses prebuilt firmware, one server and one target, and a streamed CSV recorder instead of rosbag. The temporary validation runner is now saved as `scripts/run_crazysim_validation.py` for reproducibility.

| RH target | Accepted / rejected solves | Measured lateral offset at shutdown (m) | Relative height at shutdown (m) |
|---|---:|---:|---:|
| circle | 19 / 7 | 0.2825 | 0.2442 |
| figure8 | 19 / 0 | 0.0582 | 0.1935 |
| straight | 19 / 0 | 0.2567 | 0.2016 |

These runs dispatched `notify_setpoints_stop -> arm(false)` at terminal freeze and settled to the simulator floor. The RH freeze follows the planned state, so actual tracking error must also be reviewed; reaching shutdown alone is not an accuracy gate. Historical Crazyflie reference CSVs contain planned solver trajectories, without a matched executed-trajectory record. The saved reference horizon was 40 with timestep ceiling 0.16, and circle activation was phase-matched to 4.294 rad where possible.

The existing body-rate backend mapped SI acceleration through an offset PWM range (10001–60000). With CrazySim's current mass and motor model, the corresponding acceleration bounds are 2.994533 and 17.965402 m/s². An uncorrected 8 m/s² step climbed from about 1.82 to 2.71 m. The calibrated 9.81 m/s² roll/pitch steps retained approximately 1.8 m altitude. Rate steps showed coupled ringing; disabling rate integral gains worsened this, and that experimental tuning is not used for the first MPC flight check.

The backend now supports an optional `bodyrate_T_min_ms2`, default 0, retaining the existing mapping unless explicitly calibrated. Backend transport checkpoint: `af31b0b`; required CFlib manual packet checkpoint: `ffd667a`. Unrelated backend working-tree changes remain unstaged and unchanged. A focused calibration patch is saved under `transport/ros2/integration`.

Crazyflie direct MPC validation and both directory cutovers remain pending.

### Crazyflie MPC first attempt — failed

The calibrated circle flight (`crazyflie_mpc/circle`) accepted policies 0–3, then its measured state diverged and subsequent policies were rejected. The retained policy reached its 2 s expiry and the wrapper switched to hold. Final height was zero after the vehicle fell; this is a failed flight, not a landing. Rate-loop calibration and the physical-model mismatch must be resolved before any Crazyflie MPC acceptance or directory cutover. Disabling rate integral gains worsened the standalone rate step; a reduced-bandwidth calibration is under investigation. All failed attempts remain recorded.

The lower-bandwidth rate-gain experiment failed: applying 0.1× roll/pitch gains before takeoff prevented staging; applying them after normal staging destabilized the neutral body-rate command and the vehicle fell before the requested step. These are calibration failures, and the settings are not adopted. The runner now records exceptions as harness failures and records requested rate tuning in each completed calibration cell.

A concrete Crazyflie state-frame discrepancy was found during review: the Python backend publishes `kalman.statePX/PY/PZ`, which are body-frame velocity components (confirmed by firmware `kalmanCoreExternalizeState`, which rotates them to form world velocity). The native ROS2 Crazyflie adapter now rotates those components using the separately received ENU pose before passing state to the core. Explicit already-relative odometry overrides remain in their declared frame. Pose/velocity callbacks reject nonfinite updates without refreshing freshness timestamps. This ROS2-only correction requires renewed Crazyflie RH/MPC checks; ROS1 numerical behavior is unchanged.
