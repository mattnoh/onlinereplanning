# CoManDO DDP-to-PX4 constraint satisfaction

The user requires DDP to compute the states and controls, with PX4 performing tracking. No separate LQR, QP, safety filter or invented tracking controller is authorized. The unflown QP experiment was checkpointed and removed.

1. Restore exact DDP control delivery, preserving the optimizer's hold convention and node timing. Remove the custom LQR correction, its latency-prediction replica and post-solve command shaping. RH continues to send the DDP state references to PX4.
2. Compare non-ROS DDP rollouts with ROS command delivery using matching saved configurations. Treat the user's constraint-satisfying non-ROS runs as the reference; identify any configuration/model mismatch rather than infer failure of DDP from hardware tracking.
3. Identify PX4 actuator/tracking response, frame conventions, target/state age and planning/command latency. Place the necessary dynamics and mathematical constraint margins in the full DDP OCP, not a separate optimizer.
4. Improve warm-started DDP iteration frequency. Measure solver frequency separately from command-publication frequency; fast publication alone is not fast MPC.
5. Validate ordinary and CTCS, RH and MPC, using circle/figure-eight/straight sequentially. Retain physical limits, saved settings, bags and CSVs. Score predicted and measured constraints independently and retain failed cases.

Completion requires constraint satisfaction on simulated hardware in every requested formulation/mode at a verified fast MPC update rate. Removing external corrections is necessary but does not prove the remaining model/timing mismatch is fixed.

## Verified progress and next work

- DDP-only control delivery and consistent pending-policy handoff committed as f863864; six tests and ROS1/ROS2 builds passed.
- Moving startup position/velocity references are updated while the first DDP solve runs. RH transmits the DDP-derived acceleration feedforward instead of dropping it (d2183a7); six tests and ROS1 build passed. RH SITL validation is pending.
- Configurable x500 motor-thrust inverse and consistent MPC actuator bounds implemented (5a5f2f9). Generic defaults remain linear; x500 launch selects squared motor speed with minimum-speed offset. Six tests and ROS1 build passed. Physical validation is still a failure, not a demonstrated fix.
- Three new ordinary MPC straight cases are finalized under logs/ddp_only_landing_mpc, ddp_startup_landing_mpc, and ddp_motor_mapping_landing_mpc. All timed out cleanly after policy expiry. Physical audits and saved settings exist. Do not repeat them unnecessarily.
- The final case's target reverses velocity instantaneously during control; earlier cases do not. This invalidates a simple causal comparison of the actuator map. Inspect forecast consistency, target-motion continuity and rate-controller dynamics before further tuning. Preserve the user's non-ROS successful trajectories as the reference.
- Next: validate RH acceleration feedforward with one sequential SITL case; identify rate-response/handoff dynamics and investigate target forecast versus abrupt reversals. Improve the mathematics inside DDP/prediction, keeping PX4 as the tracking controller. Full ordinary/CTCS RH/MPC validation and verified fast solve rate remain outstanding.
