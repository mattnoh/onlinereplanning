# Direct thrust/body-rate MPC

Branch `mpc` adds `stc_landing_mpc`, leaving `stc_landing_noaug` available for
comparison. The new controller sends thrust acceleration [m/s²] and FLU body
rates [rad/s] via MAVROS `setpoint_raw/attitude`; MAVROS converts them to PX4
FRD. It bypasses the PX4 position/attitude controllers and retains its rate
controller. MAVROS body-frame odometry velocity is rotated to world ENU only
for the new controller.

The fixed horizon is 40 × 0.1 s. The solver state is physical position,
velocity, quaternion, angular rate (13), prediction time (1), and previous
control (4). A first-order rate response models the remaining PX4 controller.
Four RK4 substeps integrate each interval. The eight original noaug CTCS
implications and scaled aggregate interval budget use the shared interface.
Thrust and body-rate bounds replace thrust/moment bounds. The upper thrust
bound also respects normalized throttle saturation.

The optimization cost includes changes from the previous applied control.
A separate worker accepts solve requests at most every 0.1 s. The ROS command
timer applies live-state local quadratic feedback every 0.02 s, independently of the solver.
The first policy projects the staging hold through a 0.35 s initial solve
lead; subsequent leads use measured solve latency. State prediction uses the currently applied
feedback policy (including command limits); policy age compensates for the remaining latency. Controls are shifted
by elapsed time and the factory re-rolls them from the measured projected state.
Policies are swapped without the RH node-handoff scheduler. Quaternion signs
are aligned for feedback and the prediction-time state has zero feedback error.
The optimizer’s previous-control state is held on its 0.1 s grid. The fast
tracking gains are recomputed through a quadratic DDP/Riccati backward pass
at 0.02 s, with physical-state tracking weights and positive control weights.
The optimizer’s IPM/barrier gains are retained with the solve result but are
not reused as a 50 Hz controller. Fast gains exclude the prediction-time and
control-memory states. This avoids repeated application of 10 Hz memory gains
and stiff barrier curvature at the wrong sample rate.

DDP corrections are bounded to ±3 m/s² thrust acceleration and ±0.35 rad/s
body rate. Commands are bounded and slew-limited (20 m/s³ thrust acceleration and 5 rad/s²
body-rate changes). Invalid/rejected solves retain the prior valid policy;
a policy older than 1 s or stale odometry/target state switches to PX4 position hold and
pauses. Normal touchdown uses the existing PX4 landing handoff.
`direct_mpc_controls.csv` logs nominal controls, feedback, output, saturation,
policy age, and command jumps. The command queue/heartbeat serialize publication.

## Calibration and launch

The 2026-10-13 x500 SITL hover and ±0.1 rad/s roll/pitch steps measured:

- Normalized hover thrust: 0.7282821834.
- Effective first-order rate time constant: 0.08 s (grid-fit minimum).
- Rate-step altitude: 1.9918–2.0205 m; OFFBOARD throughout.

These are simulator-specific estimates, not hardware parameters. Calibration
metadata is in `docs/benchmarks/direct_mpc/calibration.json`; full traces are in
`logs/direct_mpc_20261013/calibration/`. Calibrate other airframes before using
this profile. The plant model uses a linear thrust-acceleration mapping around
hover, and the lag fit absorbs telemetry delay as well as rate-loop response.

```sh
export MPC_RATE_TAU=0.08 MPC_HOVER_THRUST=0.7282821834
roslaunch comando_planner planner_launch.launch platform:=mavros \
  drone_name:=px4_drone hover_thrust:=0.7282821834 \
  stc_z_stage:=1.55 stc_los_alt_trig:=1.2
# After the saved staging hover:
rosrun comando_planner apply_profile.sh /comando_planner \
  /home/lab/mavros_ws/src/CoManDO_planner/config/profile_stc_landing_mpc_sitl.yaml
```

`MPC_MAX_ITER` defaults to 60, `MPC_RATE_LIMIT` to 1 rad/s, and
`MPC_SLEW_THR`/`MPC_SLEW_RATE` set the optimizer's control-change cost weights.
The calibrated defaults are intended for x500 SITL. Explicit `hover_thrust`
is checked at activation and must match `MPC_HOVER_THRUST` for actuator bounds and command scaling to agree.

## Tests

Build one target at a time with `catkin build comando_planner --no-deps -j1 -p1
--make-args <target>`. `test_direct_mpc` checks discrete dynamics and CTCS
Jacobians, world/body velocity conversion, thrust normalization, nonblocking
solve submission, live feedback, quaternion-sign equivalence, and policy expiry.

The initial three-motion check passed circle/figure8 but failed straight after
startup skipped the first policy’s solve latency, producing an excessive
feedback correction. The failed trace is preserved in
`logs/direct_mpc_20261013/mc1/`; it motivated first-policy latency compensation,
correction bounds, and the 1 s policy-age cutoff before the corrected checks.

A second failed straight retry exposed this gain/cadence mismatch in both SITL
and the independent moving-target model. After recomputing command-rate gains,
the 5 s moving-target model test passed with 13 accepted policies and 232
feedback ticks, reducing distance to 0.70 m without policy expiry. The full
model regression is part of `test_direct_mpc`.
