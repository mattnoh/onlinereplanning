# CoManDO constraint satisfaction through predictive constrained control

Goal: maintain physical landing constraints with both ordinary and CTCS OCPs at a measured fast command rate. Preserve tight published limits, complete OCP files, the ROS-free core, and sequential builds/flights. Do not substitute per-solve approval gates for constrained control.

1. Identify applied versus nominal commands and measured response from existing bags. Separate delay, actuator error, feedback excursions, target prediction, feasibility and intersample effects. Fit and validate on separate cases; tracking error alone is not a disturbance bound.
2. Derive constrained fast tracking for plant-facing inputs. Include actuator evolution, slew, viability and sample/hold margins. Ordinary constraints stay always active. Anticipate CTCS/STC activation so entry into the tighter set is feasible. Keep residuals and Jacobians in each complete OCP.
3. Implement consistent constrained feedback and prediction, including timing. Preserve RH and MPC. RH's PX4 state-tracking loop needs its own execution model or equivalent constrained reference controller.
4. Verify derivatives, active-boundary feasibility, sampled closed-loop invariance, held-out model mismatch and computation latency. Report infeasible states honestly; recovery is not an invariance guarantee.
5. Build sequentially. Validate ordinary/CTCS, RH/MPC, circle/figure-eight/straight in SITL one target at a time. Score dense predicted and measured constraints with fixed physical tolerances. Failed initialization or policy expiry is a failure. Save evidence under logs and reuse existing plots.

Completion requires the original limits to hold in all formulations/modes/motions at a verified fast update rate. Landing and nominal-only tests are insufficient. If target motion is outside the limits' viability region, derive and demonstrate the incompatibility rather than silently relax a limit.
