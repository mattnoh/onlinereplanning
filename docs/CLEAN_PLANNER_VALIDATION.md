# Clean planner validation

Candidate source: `/tmp/comando-clean`. Active planners remain unchanged until all required SITL gates pass. No directory cutover has been performed.

Reference commits: PX4 `37fc32f`; Crazyflie `fb424a6` (checkpoint of existing log deletions). The existing Crazyflie solver changes were checkpointed as `0a3a9db`; the clean core currently links the shared CTCS solver in the MAVROS workspace.

Implemented: compiled shared library, common controller API, ROS1/ROS2 wrappers, independent complete direct MPC OCP, external prediction, typed landing parameter defaults, legacy OCP removal, structural validation, profile/reset locking, and separate Crazyflie direct-control publication.

Passed standalone executable checks: planner core, controller lifecycle/mode/invalid input, replay, CTCS parity (160 randomized samples), stateswitch prediction, target-frame warm start, polynomial OCP, five warm-started RH landing solves, and direct MPC derivatives/feedback/expiry. Direct dynamics Jacobian error was 7.53e-10 and CTCS Jacobian relative error 3.43e-10.

SITL and transport checks are recorded below as they complete. A successful build or model test is not a SITL landing result.
