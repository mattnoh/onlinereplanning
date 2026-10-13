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
