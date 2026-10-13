# Frame normalization and cone-start validation

The active ROS1 planner on branch `mpc` now uses canonical world-state input and OCP-selected coordinates. The shared core remains independent of ROS. Each complete OCP is kept in its own file; shared defaults and target prediction stay outside it. `landing_rh` and `landing_mpc` provide the requested ordinary-constraint comparison without CTCS/STC. See [frame conventions](FRAMES.md) and [ordinary experiment settings](ORDINARY_LANDING.md).

All flights ran sequentially, one target at a time. There was one zero-noise trial per motion/mode. Every final case shut down cleanly. Successful existing flights were not repeated during final analysis. The three motions retain saved MC1 configuration and tuning.

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

## Verification and saved evidence

- Six existing core tests passed, including normalized coordinate conversion, native cone/terminal Jacobians, feasible/infeasible starts, controller modes and direct-model consistency. Stationary and measured moving-start cold solves passed.
- The active ROS1 planner and shared ROS2 planner build passed sequentially. ROS2 Crazyflie flight validation/deployment remains pending.
- Final analysis executable rebuilt against the final OCP models; Python syntax and Git whitespace checks passed. Analysis changes do not alter flight controllers.
- Each cell saves `flight_*.bag`, solver CSVs, `planner_environment.json`, `landing_profile.yaml`, `source.json`, `cell.json` and `constraint_audit.json`. Controlled ordinary cases also save trimmed `moving_executed.csv`, `constraints.csv`, state/3D PNGs and a twin-frame GIF. Failed initialization cases have no fabricated trajectory plots.
- Early initialization/preparation diagnostics remain under `logs/ordinary_landing_rh_initialization` and `logs/ordinary_landing_rh_preparation`, separate from the final cases. Historical `ros1_current_rh` and `ros1_current_mpc` logs are physically under active `logs/`; former `.Codex` paths are compatibility symlinks.

Existing plot layouts are reused. Ordinary plots show always-active limits and the altitude floor, with no staging or trigger annotations.

| Saved campaign | Data and audits |
|---|---|
| frame_regression_rh | [Campaign](../logs/frame_regression_rh) |
| frame_regression_mpc | [Campaign](../logs/frame_regression_mpc) |
| ordinary_landing_rh | [Campaign](../logs/ordinary_landing_rh) |
| ordinary_landing_mpc | [Campaign](../logs/ordinary_landing_mpc) |

| Ordinary controlled run | States | 3D | GIF |
|---|---|---|---|
| RH circle | [States](../logs/ordinary_landing_rh/circle__zero__t0/flight_states.png) | [3D](../logs/ordinary_landing_rh/circle__zero__t0/flight_3d.png) | [GIF](../logs/ordinary_landing_rh/circle__zero__t0/flight.gif) |
| RH straight | [States](../logs/ordinary_landing_rh/straight__zero__t0/flight_states.png) | [3D](../logs/ordinary_landing_rh/straight__zero__t0/flight_3d.png) | [GIF](../logs/ordinary_landing_rh/straight__zero__t0/flight.gif) |
| MPC circle | [States](../logs/ordinary_landing_mpc/circle__zero__t0/flight_states.png) | [3D](../logs/ordinary_landing_mpc/circle__zero__t0/flight_3d.png) | [GIF](../logs/ordinary_landing_mpc/circle__zero__t0/flight.gif) |
| MPC straight | [States](../logs/ordinary_landing_mpc/straight__zero__t0/flight_states.png) | [3D](../logs/ordinary_landing_mpc/straight__zero__t0/flight_3d.png) | [GIF](../logs/ordinary_landing_mpc/straight__zero__t0/flight.gif) |
