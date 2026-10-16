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
- RH feedforward-only straight case landed but violated constraints. Dense OCP reference sampling, per-plan target forecasts and world-frame handoff conversion are now committed (d420def); six tests and ROS1/shared ROS2 builds passed. Publication is 100 Hz, separate from solver frequency.
- RH circle case with consistent references landed cleanly. Measured speed/glideslope/LOS/floor excess was zero and tilt was within audit tolerance, but angular-rate excess remained 0.02414 rad/s for 1.03593 seconds. The 128 accepted horizons had zero predicted speed/tilt/rate/glideslope excess; small terminal floor/LOS spatial excess remained. It is not an isolated comparison with the earlier straight case.
- A ROS1 parameter writer waited on each solve and could block replay; the recorded maximum command gap was 82.493 ms. The poll now uses a nonblocking session lock (86bd3f7). The sequential circle verification in logs/ddp_nonblocking_reference_landing_rh landed cleanly: maximum command gap 16.045 ms, zero measured speed/tilt/glideslope/LOS/floor excess, but angular-rate excess 0.00607 rad/s for 2.40060 seconds across 95.18 controlled seconds. No active flights/builds remain.
- PX4 inner-loop evidence is saved alongside the three MPC cases. Body-rate mode switching took approximately 148 ms in the motor-mapping case. The first-order rate model omits actual rate PID integrator/motor dynamics. RH seeds nominal attitude/rates instead of a measured-state PX4 tracking-model prediction, so its nominal feasibility is not physical feedback feasibility.
- Next: trace reference handoff continuity, align measurement epochs, and model the relevant PX4 dynamics inside the full DDP OCP. No separate optimizer/controller, changed physical limits, or solve-approval shortcut. Full ordinary/CTCS RH/MPC validation and verified fast solve rate remain outstanding.

## Native PX4 acceleration conversion

The follow-up attitude-demand audit uses a ROS/PX4 clock offset estimated from matching ENU/NED position/velocity (near handoff, residual timing spread about +/-4 ms). During the actual control interval, PX4 requested axis rates above 0.09 rad/s over 0.1-second windows, whereas the physical DDP-reference axis rate remained below 0.019 rad/s over that window. This is not a yaw error. Accepted initial DDP attitudes match the prior scheduled handoff nodes exactly in the saved precision.

Native PositionControl decouples horizontal/vertical acceleration (`MPC_ACC_DECOUPLE=1`) and subtracts measured acceleration through D=0.2 horizontally. Raw DDP acceleration is therefore not the correct PX4 acceleration feedforward. The inverse conversion is now inside each complete ordinary/CTCS RH OCP, using DDP state/input only; shared typed parameters contain native FCU coefficients. Crazyflie retains physical acceleration. Independent tests reconstruct the PX4 attitude across thrust changes, including coupled/decoupled configurations. All six core tests and ROS1 build pass; ROS2 build also passed sequentially. The circle SITL validation landed but failed physical tilt/rate limits; startup rate peak at 0.169 seconds shows that the zero-error/zero-integrator conversion is insufficient. See logs/ddp_px4_inverse_landing_rh and README.

Exploratory physical torque fits failed held-out prediction and are not used as plant coefficients. They require better sampling/identification rather than hardcoding misleading fitted gains. Remaining work still includes actual tracking-state dynamics, MPC mode handoff, collective mapping, target forecasts, small terminal nominal excess, and all ordinary/CTCS RH/MPC motion cases at a measured fast solve cadence.

Next data action: one ordinary MPC circle diagnostic with native SDLOG_PROFILE=147 (existing default 131 plus high-rate bit 16), preserving control settings. Full-rate actuator/rate topics are needed to validate native PID/rotor dynamics; sparse fits were rejected. No PX4 controller/source changes and no parallel flights. Then implement/validate native controller-state prediction in the complete OCPs.

The high-rate MPC circle diagnostic is finalized LANDED/CLEAN (92 seconds) but failed rate/tilt/glideslope constraints; first controlled rate was already 0.03154 rad/s. Eight accepted horizons, 100 Hz exact DDP input publication, and full-rate native PID/actuator evidence are now saved with a copied ULog in logs/ddp_native_model_identification_landing_mpc/circle__zero__t0. The native mixing matrix is validated to RMS 4.25e-5 normalized motor signal. The logged mode-switch delay is 188 ms and varies from prior 148 ms; rate integral is nonzero.

Next required implementation: replace the first-order/instantaneous-handoff prediction with validated native PID + rotor/controller-state dynamics in the complete MPC OCPs. First-plan projection currently assumes hover body-rate tracking while the actual PX4 position/attitude cascade remains active. Account for that phase mathematically and initialize native controller/actuator states from an estimator, rather than inserting post-solve correction or altering acceptance. Validate native dynamics/derivatives and closed-loop rollout against the full-rate dataset before another flight. Then address the RH physical/reference tracking-state model, forecast epochs/reversals, terminal nominal excess and warm-start cadence. Keep all physical limits and required formulations/motions intact.


## Enforcement-first investigation, 2026-10-16

The current priority is actual offline feasibility before additional SITL or
plant-model work. The earlier native-model next actions above are superseded
until this passes. No SITL was launched during this investigation.

Confirmed defects and changes:

- A one-step scalar barrier problem returned u=0 with zero reported primal
  error although its barrier gradient was 1. The correct minimizer is
  (1-sqrt(5))/2. The stage control step omitted the current multiplier
  gradient. Corrected it, the condensed value recursion, terminal
  complementarity sign and terminal curvature in ALIPDDP (`237d29e3`).
- SOC L(s)^-1 L(y) omitted the y_tail/s0 term in its lower first column.
  The focused solver regression compares every entry with a dense Jordan
  matrix solve. Scalar NO/SOC stage/terminal tests, equality tests, gapped
  FDDP seeds and repeated measured-root LQR updates also pass. The LQR
  reference uses independent dense Newton equations; control discrepancy
  is below 2.4e-11.
- Feasible small slacks must not be replaced by a barrier-dependent floor.
  Preserve S=-C for interior scalar/SOC rows; retain positive interior
  initialization for violated rows. Check CTCS initialization at mu=1,
  0.01 and 1e-6.
- Optimal termination includes primal/gap residuals and active barriers;
  equality convergence no longer waits on inactive barriers. At final
  continuation limits, inner stationarity is checked at solver tolerance.
  Failed line-search logging no longer indexes beyond the attempted steps.
- Shared OcpSolver now independently audits every returned NO/SOC/EQ stage
  and terminal row, root and full discrete dynamics. This applies to RH
  and MPC, including descriptors that skip broader trajectory heuristics.
  It does not claim unrelaxed physical STC or robust tracking guarantees.

Validation and remaining failures:

- Focused generic solver regression passes. Offline MPC lifecycle/moving
  model tests pass with the new primal gate; existing core/controller
  checks also passed before the latest slack-initialization correction.
- The historical RH test forces 30 iterations. Its staging-hover cold
  solve now fails the stricter primal check, previously hidden by the
  aggregate gate. Do not loosen the audit or rewrite this as success.
- Replay of solve 3 from the retained repeat-circle flight, reconstructed
  from its saved states/controls and target acceleration, still fails:
  largest scalar CTCS residual about 6.98e-5 versus tolerance 1e-6, despite
  essentially closed dynamics (max component gap about 5.4e-15) and a
  1000-iteration cap. It stalls at the continuation floor, not convergence.
  Command-memory reconstruction is approximate, so this is a reproduction
  from the saved physical horizon, not an exact internal solver snapshot.
- Longer line searches, dimensionless per-implication experimental rows
  and removing discontinuous trigger floors did not resolve the replay.
  These experiments remain under /tmp; production OCP/tuning is unchanged.
- Next: resolve actual quadrotor/RH primal feasibility and physical CTCS
  allowance/scaling, then strengthen complete offline measured-state loops
  before any sequential SITL validation. No new controller corrections.

Intermediate tuning logs are archived with verified SHA-256 manifest;
208.29 MiB was reclaimed. Baseline, final circle runs and representative
straight/figure-eight failures remain accessible under logs/.


## MPC feedback scope correction and warm-start validation

The user reconfirmed that the validated standalone CTCS solver, including
between-node constraints, is the reference. The requested correction is the
added closed-loop MPC/RH integration. Do not introduce further SOC formulation
changes or treat new planner-probe failures as evidence against those original
standalone results. The uncommitted Nesterov–Todd experiment was checkpointed
and removed in solver commits 9cbaffac / 4d68a48b. No SOC scaling experiment
remains in the worktree.

Planner checkpoint b52a74d corrects two warm-seed paths: generic horizon shifting
propagates terminal extension instead of repeating the terminal state; direct
MPC measures seed age from the previous measured root and converts old relative
position/velocity into the new target forecast while preserving the nominal
world trajectory. The direct conversion is now one internal helper used by the
controller and independently exercised in the existing test_direct_mpc test.
It also tests a sub-grid age and terminal extension. These are seed corrections;
x0 remains measured and publication still holds exactly the first DDP input.

Validation: sequential standalone build of test_direct_mpc and execution with
MPC_MAX_ITER=120 MPC_WARM_ITER=24 pass, including the new forecast/epoch tests,
dynamics/CTCS derivatives, first-input hold and both moving-target lifecycle
cases. Output: /tmp/comando-warm-shift-check.log. This lifecycle test does not
prove complete physical constraint satisfaction during landing.

The temporary deterministic full landing probe still fails the physical audit.
Default planner CTCS budget is 1.6e-3 versus 1e-5 in the standalone source; the
saved SITL preset overrides it to 1e-5. Neither the default nor a temporary
original-budget run is a clean constraint baseline: even the first open-loop
trajectory from this added MPC OCP has physical excess. At the original budget,
the complete straight-target probe accepted 90 solves but still exceeded rate,
glideslope, LOS and staging constraints. Increasing the iteration cap alone did
not make the first horizon physically clean. These probes use the added MPC
OCP/model and are not the user's validated standalone example. Do not conflate
them. The aligned curved-target probe also still fails after its first solve,
with both reused and fresh solver instances.

No production CTCS budget, physical limit, model, extra controller or acceptance
criterion changed in this continuation. No SITL launched. Next: establish an
identical, physically clean open-loop reference for the actual deployed MPC
OCP, then isolate the first feedback transition that diverges from it. Complete
RH/MPC enforcement remains unverified; keep the goal active.


## Retained warm-start loop: actual asynchronous offline landing

The user corrected the scope again: preserve the already functioning warm MPC
loop. Do not substitute ROS RH as a clean reference, require free-time MPC, or
turn an isolated fixed-period diagnostic failure into a generic cold-retry
architecture. The untested seed-based cold-rebuild candidate was checkpointed
as c71c6bd and removed as 75c2155. The solver worktree remains unchanged. The
planner warm-start runtime is back to the tested e2c3d09 behavior.

The existing test_direct_mpc passes again with MPC_MAX_ITER=120 and
MPC_WARM_ITER=24; ordinary moving test: 139 accepted updates / five seconds,
max warm solve 23.91 ms. This is lifecycle/recovery evidence, not complete
landing constraint evidence.

A temporary /tmp/comando-actual-mpc-landing.cpp now exercises the actual
DirectMpcController worker, measured world state, held first DDP input and
independent physical residual audit through a complete model landing. The
plant is the descriptor's zero-target-acceleration world model, integrated at
at most 1 ms; the controller receives relative prediction via the existing
frame preparation. Targets come from the existing TargetGenerator, not new
handwritten motion equations. No ROS or SITL was launched. All cases ran
sequentially. No permanent new scripts, tests, plots or log artifacts were
added to the branch.

Authoritative ROS1-cadence constant-velocity cases (1 ms solve polling, 100 Hz
command publication, unchanged defaults, max120/warm24):

- Ordinary MPC: 381 accepted, zero rejected, landing condition reached,
  9.90621 controlled seconds; every measured physical excess and accepted
  predicted-node physical excess exactly zero. Max solve 36.727 ms.
  /tmp/comando-actual-ordinary-constant-100hz.log.
- STC MPC: 144 accepted, zero rejected, landing condition reached,
  9.23358 controlled seconds; physical limits still fail (rate excess
  0.247128 rad/s, tilt 1.72274 deg, GS .0478512 m, LOS 4.05228 deg,
  staging .128537 m). Accepted predictions also contain physical excess.
  /tmp/comando-actual-stc-constant-100hz.log.

The constant target reproduces the existing moving regression's velocity
(.134,.435,0), via TargetGenerator straight with span zero. These two cases
are not a complete three-motion MC1 validation or an independent hardware
plant validation. Future predicted trajectories above were audited at nodes;
the executed trajectory was sampled at approximately 1 ms. Do not claim a
full future-interior audit from this probe.

Before correcting the probe's command rate from 1 kHz to 100 Hz, the saved
circle generator seed 8570001 reached landing in ordinary MPC with 407
accepted / one rejected, and no measured violation above the explicit audit
tolerances (rate excess 5.04613e-5 rad/s, tolerance 1e-4). The saved figure-eight
seed 7060001 had zero accepted cold plans; it never entered warm MPC execution.
These are preliminary diagnostics, not ROS1-cadence validation gates, and zero
measured samples in the figure-eight failure must never be counted as a pass.

A constructive current-MPC counterexample isolates constraint semantics:
first STC input has body-rate norm .607747995 rad/s. In the existing first-order
rate model (tau .08 s, zero initial omega), its .1 s predicted rate is
.433625279 rad/s, exceeding the loose .35 rad/s physical limit. Independent
100000-midpoint integration of max(0,omega^2-.35^2)^2 gives raw integral
4.6995873e-5, below the current raw 1.6e-3 budget; scaled row is -1.5530041e-4.
At this first interval staging/landing/LOS are inactive or satisfied. Thus the
programmed CTCS allowance admits this physically violating MPC input; this is
not evidence of a failed CTCS optimizer, nor a claim about the user's validated
internal-RH runs. Reject/cold-retry behavior, SOC scaling changes and a free-time
conversion do not fix that mathematical allowance.

Next: correct the added MPC problem's physical enforcement while preserving
measured-root warm DDP, unchanged first-input publication, fixed grid and the
CTCS interface. Verify complete predicted and executed constraints on the saved
three motions, then RH as required. Do not report the current goal complete.
