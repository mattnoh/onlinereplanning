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
