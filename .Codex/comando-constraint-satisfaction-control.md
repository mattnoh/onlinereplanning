# Current authorized task — local point-mass open-loop four-arm comparison

2026-10-19 user requests review of the local point-mass open-loop comparison,
SCP/DDP x CTCS/stage comparison, preserving its local plotting. Found canonical
benchmark at /home/lab/projects/solver/benchmark/openscvx_point_mass_ctcs and
original fixed/free plotting under DDP/ALIPDDP-ctstc-al/results/point_mass_ctcs_compare.
Existing SCP sweeps are running in another local process; do not stop, alter or
claim those runs. Freeze completed inputs before selection. No new optimization
or landing work in this diagnostic; landing repair remains paused.

Audit hypothesis: saved fixed N=6 four-arm policies expose nodal intersample
penetration in DDP and SCP while CTCS policies avoid the sphere. Apply the
existing selector unchanged, replay all selected controls from the common root
with exact point-mass flow, independently audit extrema and CTCS integrals.
Preserve physical OCP, CTCS budgets/margins and declared acceptance rules.
Keep fixed/free original serif style, 3D geometry, clearance traces with node
markers, mesh heatmap and timing/quality table. Do not replace original figures.
Failed/missing accepted SCP free-time cells must remain explicit, with diagnostic
paths clearly labeled and excluded from accepted comparisons. Snapshot settings,
raw selected data, plotting source and SHA256 inventory under planner logs.
Verify sampled traces against exact extrema; no speed claims from loaded timing.
Older sections below are historical task records.

---

# Current authorized task — CTCS stopping efficiency and landing diagnosis

User asks whether CTCS DDP stopping can be faster without losing safety, and
why paper MPC success has not fixed landing. Landing-fix goal remains paused:
only read-only landing diagnosis is authorized by the question; no landing
changes or experiments. Test stopping tuning on the paper benchmark first.
Startplannerb103ef4, solver3a962c5a; bothtrackedclean. Prior speed profile is
M50/sharedcache, allphysical/integralchecks and all15Optimal verified.

P17 hypothesis: reduce unnecessary intermediate barrier centering by increasing
inner_tolerance10->100, leaving the final/globaltolerance1e-6, barrierfloor,
physicalbounds, delta.05, epsilon1e-6, horizon/grid/hold and fastM50/shared
profile unchanged. Single intended change: --inner-tol100. This parameter
controls when to advance barrier subproblems, not final plan acceptance.
Pass: complete30s/15 measuredroot MPC solves, allOptimal<=1e-6, modeledprimal
andindependentstateflow<=1e-6, executedphysicalviolation0within1e-6 andtrue
integral<=1e-6 withinexistingrelative1e-6audit, usefultracking. Recorditerations,
CPUtime andwalltime; do not infer performance from differing hostload. Reject
candidate if modelrows pass but independent budget/physical checks fail.
No shared solver algorithm change and no relaxation of finaltolerance.

---

# Current authorized task — faster DDP CTCS on the paper benchmark

User2026-10-19 requests RK4/tuning speed experiments. Landing repair remains
paused. Retain physical obstacle/input limits, delta=.05, epsilon1e-6,
horizon8, nodes17, ZOH, K4, duration30, root/reference and independent1ms audit.
Do not confuse RK4 integration substeps with changing the OCP node spacing.

P12 hypothesis: 100 RK4 substeps per .5s interval over-resolves the smooth
physical dynamics, and 20 may preserve physical/integral feasibility at lower
solve cost. Single intended change from P7: --substeps100->20. No tolerance,
physical budget, tightening, constraints or shared solver algorithm change.
Pass: full30s/15 replans, all solvesOptimal to1e-6, returnedprimal<=1e-6,
dynamics/independentphysicalflow<=1e-6, executed physicalexcess<=1e-6,
executed integral<=1e-6 within existingrelative1e-6 budgetaudit; usefultracking.
Report actual raw integral maxima, quadrature errors and timer median/p95/max
separately. Reject coarse integration if it passes modeled rows but fails the
independent budget/physical audit. Compare saved seed/solution accuracy.
All campaigns sequential. Failed candidates stay labeled; change defaults
only after the complete relevant verification. Prior source commit133f53d.

---

# Current authorized task — matched paper-example 2×2 comparison

2026-10-19: user paused the landing-fix goal. No further landing experiments
while paused. Earlier Claude landing diagnostic implementation was checkpointed
as cbdbcd7, unverified; it is not a retained landing fix.
Current explicit request: SCP/DDP × CTCS/node-only with performance comparison.
Previous explanatory turn yielded no implementation progress; this turn adds
the missing SCP nodal comparator and verifies it against saved runs.

P10 hypothesis: normal nonconvex nodal SCP rows can be satisfied while the
independently propagated continuous trajectory violates obstacles between
nodes. Outcome is measured, not forced. Single intended change from P5:
replace obstacle CTCS accumulator/increment rows by masked g_i+.05<=0
at all 17 nodes, including terminal. Omit the unused obstacle accumulator
entirely so it supplies no hidden proximal term or integral constraint.
Keep ZOH, dt=.5, horizon8, K4, duration30, root, reference, sensing, normu<=5,
drag, integrated objective, cost scale100, prox1, penalty1e4, 10 SCP iterations.
The library's identically-zero clock CTCS remains audited and unchanged.
Pass as a nodal method: complete30s, finite solutions, modeled node rows and
physical dynamics/root defects<=1e-6, useful tracking. Independently report
physical dense peak/duration and prediction/plant drift; dense failure must
not be attributed to intersampling alone unless nodal/defect evidence holds.
Integral ratios are diagnostic only for both nodal methods, not constraints.
Measure solve wall time, first/JIT separately from subsequent solves; timing
across Python/JAX and C++/Eigen is implementation evidence, not optimizer
superiority or real-time certification. Saved SCP10/DDP budgets differ.
All scenario runs sequential; Claude plotting worker edits only existing plot
code in parallel. No shared solver edits or physical-limit changes.

---

# Current priority — paper CTCS MPC reproduction, then identical DDP problem

User direction 2026-10-19: implement and execute the original Uzun et al.
2024 obstacle-avoidance MPC in SCP first, including its tightening, then
recreate the same physical OCP and synchronous MPC in DDP. This diagnostic
precedes further landing tuning or shared-solver modifications. The user
explicitly prioritizes physical enforcement in synchronous zero-deviation MPC;
perfect optimality is diagnostic, not a substitute for that result. Earlier
landing experiments below are historical evidence, not this reproduction.

1. Three actual Claude implementation workers run in parallel: native SCP
   driver, original-problem DDP driver, and independent shared model/audit.
   Parent runs all builds and optimization scenarios sequentially.
2. SCP paper configuration: 2-D drag double integrator, five ellipses,
   integrated reference-tracking cost, 8 s horizon / 17 nodes, K=4 intervals
   executed per update, 30 s campaign, three SCP iterations, delta_obs=.05,
   single horizon accumulator with per-interval increment bounds. FOH first.
3. Document unreported settings as assumptions: epsilon=1e-6, initial
   (r,v)=(-10,0,0,0), exact-penalty weight=1e4, proximal coefficient=1.
   No claim these are recovered author settings. Record source hashes.
4. Independently replay at <=1 ms, separately audit actual physical g,
   tightened integral, physical and accumulator dynamics defects, virtual
   controls, input limits and tracking progress. Preserve failed artifacts.
5. Matched ZOH SCP replay, then the SAME ZOH obstacle OCP in untouched
   ALIPDDP, same initial state / density / budgets / objective / MPC shift
   and tail extension. Establish open-loop behavior and successive solves.
   No asynchronous latency, tracking error, ROS or landing dynamics enter.
6. Diagnose first discrepancy using saved shifted seeds and returned plans;
   only then return to the landing gates recorded below. No solver changes
   or physical-limit relaxation before the matched comparison.

Implementation starts from planner 2148894 and solver 4d68a48b.
Existing catkin cache has an E13 temporary linker override pointing to a
missing /tmp object; do NOT use that build for the DDP paper baseline.
The paper executable will be built with clean standalone linkage.

Experiment P1: hypothesis — native SCP with the paper's CTCS structure and
explicit tightening supports the original complete obstacle MPC example.
Single intended change: new reproduction driver, no library modifications.
Pass: completed 30 simulated seconds with useful tracking, independently
verified zero physical obstacle penetration within 1e-6 and norm(u)<=5
within 1e-6; report nonlinear dynamics / integral defects and interval budget
feasibility separately. Three iterations is the paper-performance run;
additional iterations are labeled diagnostics, never silently substituted.
Current state: original paper-example SCP/FOH/ZOH and DDP fresh/retained
MPC implemented and executed. Physical results verified; SCP finite-iteration
integral residuals are reported separately. Landing gates remain incomplete.

---

# Authoritative gate record — reliable constrained DDP MPC

Updated 2026-10-19. This ordered sequence supersedes the historical experiments
below. Evidence of feasibility alone never passes the convergence gate.

1. **Progress/checkpoint:** inspect planner and solver before each prompt;
   commit task changes; run builds and scenarios sequentially. Planner starts
   at `cd3fda9`, solver at `4d68a48b`, both without tracked modifications.
   Unrelated solver untracked papers/results and CATKIN_IGNORE are left alone.
2. **Shared OCP:** match dynamics, costs, prediction, root and physical limits;
   independently verify derivatives/signs/scaling/quadrature. Preserve the
   feasible witnesses and committed integration (`d1b41d4`) and cold-seed
   (`cd3fda9`) fixes. Protect standalone/RH reference. Shared solver changes
   require isolated reproduction plus regression evidence before retention.
3. **Open loop (CURRENT):** both production cold starts must make useful
   progress, primal/dynamics <= 1e-6, optimality <= configured tolerance and
   Optimal Solution termination. Separately audit future nodes, local intervals
   and full continuous propagation at 1 ms. Physical limits/budget stay fixed.
4. **Offline MPC:** both modes, exact first-control hold for .1 s, 1-ms plant;
   inspect root, shift, appended interval, defects and IPM initialization. No
   failed solves/tolerance violations; allow 40 s, finish <3 mm and <.01 m/s.
   Then actual latency/held-input timing; report publication, update intervals,
   solve latency and policy expiry separately. Straight/circle/figure-eight
   sequentially for each mode. NOT STARTED for the demanding converged case.
5. **Triggered STC:** only after both base modes pass offline. Inactive,
   active, crossing, altitude/LOS/capture/staging triggers; within-interval
   activation and complete landings. No always-active surrogate. NOT STARTED.
6. **ROS1 SITL then portability:** base modes then STC, three saved scenarios
   sequentially; bags/CSVs/settings/revisions under logs; predicted and measured
   audits separate; clean teardown. Crazyflie/ROS2 afterward. NOT STARTED.

Fixed benchmark: relative root (.518,0,1.85), upright, zero relative velocity
and rates; constant target .455 m/s at heading .6; N40, dt .1; speed .25 m/s,
tilt .1 deg, rate .019 rad/s, GS 15 deg with .1-m apex, LOS 16 deg, floor 0,
raw CTCS interval budget 1e-8. Floor is geometrically redundant under these
cones. Physical audit tolerances remain 1e-4 m, m/s, rad/s and .01 deg,
including spatial LOS error. Activity and one-limit counterfactuals are required.
No separate optimizer/controller or post-solve command clipping is authorized.

## Verified checkpoint (2026-10-19)

Retained implementation commit: `5747e3acb1a96c23312cf45c8065c4bec2b0df47`.

- E7 PASS: test_direct_mpc passes with independent ordinary dynamics Jacobians,
  all six constraint Jacobians/geometric signs/scales, and stage/terminal cost
  gradients. Existing analytic rate response, folded RK quadrature, Jacobian,
  Hessian, measured-state root/first-input hold and moving-target checks pass.
  Configuration: MPC_MAX_ITER=120, MPC_WARM_ITER=24. Ordinary moving case accepts
  50 updates, max warm solve 95.7455 ms. This regression is not the demanding
  no-failed-solve/complete-landing acceptance gate.
- Both final --initial-guess stress diagnostics PASS: primal/dynamics/dense
  residual zero and distance 1.92115 -> 1.66210.
- Final --open-loop checks use the benchmark environment in README and the
  manifest. Both return exit 1 as required. Stage max-iteration optimality
  .0011274245137822753, primal/dynamics zero; spatial LOS .00022214422768535025 m
  for .074 s. CTCS Outer Max/Min, optimality 4.1469056741378148e13,
  normalized row violation 1.4069693824087672e9. Gate 3 remains FAIL.
- Existing trajectory_replayer, planner_core, stc_landing_noaug_ctcs and
  controller CTest cases PASS. stc_landing_noaug FAILS its 30-iteration
  staging-hover case. E8 reproduces the identical 1.65674 constraint residual,
  zero dynamics gap and Max Iteration on pristine cd3fda9 with solver 4d68a48b.
  No test limit or acceptance rule was changed to hide this inherited failure.
- Sequential focused builds include all six test targets and constraint_audit;
  temporary E6 linker override is removed. Solver tracked tree remains clean.
- Exact settings, raw logs, isolated rejected override source and revisions are
  archived under logs/constraint_gate_20261019/ (manifest.json and
  source_revisions.txt). These include failed experiments, not just successes.

Next unresolved work: convergence of the folded CTCS OCP from the production
cold seed, plus the stage formulation's intersample spatial LOS violation.
Objective scaling alone is insufficient and is not retained in production.
No gate 4–6 validation is claimed. Gate 2's independent derivative/reference
checks passed; full counterfactual activity assessment awaits a converged
physically acceptable benchmark. The standalone/RH source remains protected.

ROS1 comando_planner transport build also PASS after the diagnostic fields,
with normal solver linkage. No SITL was launched before the offline gates.

## Active-goal continuation (2026-10-19)

Previous goal turn: no progress (explanation only). Trees revalidated clean at
planner 31f7c74 / solver 4d68a48b. Goal remains complete offline CTCS MPC with
convergence and physical enforcement, preserving the matched benchmark.

E9 — Hypothesis: the stalled solver residual conceals either an incorrect
costate recursion or an inaccurate primal-dual central path. Independently
recompute reduced Lagrangian and barrier gradients by uncondensed adjoints
at the returned trajectory, plus complementarity and row slack/dual ranges.
Use the existing feasible E3 configuration (2000 inner, 30 steps, 600 total)
without modifying OCP, limits, budget, or solver. Pass for diagnosis: distinguish
a real stationarity defect from a misleading reported residual; save the
trajectory for isolated replays. This is not itself a convergence pass.

E9 result: true reduced Lagrangian gradient .000956927 versus reported
.000293629; exact-barrier gradient .0324710. Complementarity 4.96e-21,
minimum slack 1.23207e-7, max dual .512113, final mu 6.30957e-8. Row/slack
mismatch 1.03e-11 is amplified by the near-boundary barrier curvature.
Physical GS .000149241 m and LOS .000380824 m remain above tolerance.

E10 — Hypothesis: feasible nonlinear slack update eliminates amplified
row/slack cancellation without the earlier E6 dual reset. Isolate only
forwardPass: when already strictly feasible (no equalities), require candidate
nonlinear rows to remain interior, set S_new=-C_new, retain affine Newton dual
update and its fraction-to-boundary test. Other cases use original path.
Same E9 configuration/seed/objective/budget/limits. Pass for convergence:
Optimal Solution and independent stationarity <=1e-6; physical audit separate.
No shared source change until isolated and reference regressions pass.

E10 FAIL: feasible primal/physical audit, but dual stagnates (independent
stationarity 6984, complementarity about initial mu). Reject exact-slack-only
update. No shared source edits.

E11 — Hypothesis: the line-search merit omits stationarity/complementarity,
allowing grossly infeasible cost-reducing steps and rejecting useful dual
corrections when objective differences reach rounding precision. Isolate a
residual-norm line search using independent adjoint stationarity, primal slack
residuals, complementarity and dynamics defects. Original Newton directions,
slacks and dual updates unchanged. Scope initially scalar/no-equality MPC.
Same raw objective, fixed limits/budget and production cold seed; 600 total,
2000 inner, 30 steps. Pass: true and reported stationarity <=1e-6 with Optimal
Solution and primal/dynamics <=1e-6. Audit physical errors independently.

E11 FAIL: reduced stationarity decreases to 261.666 but scalar primal violation
65.5946 remains; approximate Newton directions do not reliably decrease full
KKT merit near this nonlinear boundary. No production change retained.

E12 — A specific bug was found in E6's attempted feasible-barrier experiment:
the original filter never checks objective decrease when both old and new
primal residuals are exactly zero. Consequently E6 accepted uphill barrier
steps. Repeat the same exact nonlinear slack/central-dual path with an explicit
barrier-objective descent test, first on the unscaled production objective.
One intended difference from the existing isolated feasible-barrier override:
reject dV_act<0 whenever the feasible path is used. Fixed limits/budget/seed,
600 total / 2000 inner / 30 line-search trials. Pass: independent and reported
stationarity <=1e-6, Optimal Solution, and primal/dynamics <=1e-6.

E12 FAIL but isolates a concrete numerical collapse: feasible descent accepts
S_min=2.4647e-14, Y_max=4.0573e8 at the initial barrier. Subsequent barrier
updates cannot recover a representable primal step; last duals remain centered
on 1e-5 while final mu is 1e-8. The existing tau=max(.99,1-mu) allows the
fraction remaining to approach mu, so it shrinks towards zero with continuation.

E13 — Relative to E12 change only the feasible nonlinear path's fraction to
boundary: require at least .1 of old slack to remain, independent of mu. Keep
central-dual update and explicit barrier descent. Hypothesis: stop the first
cost-reducing iterations from entering a numerically unrecoverable boundary.
Fixed limits/budget/objective/seed; 600 total / 2000 inner / 30 steps. Pass:
reported and independent stationarity <=1e-6 plus primal/dynamics <=1e-6.

## Current experiment register

E1 — Reproduce current production cold convergence with the committed feasible
seed. No formulation or solver change. Extend the existing direct-MPC diagnostic
with fail-closed convergence and separate physical audits. Pass: both modes
meet gate 3, including raw optimality and 1-ms continuous propagation.
FAIL at 120 iterations: stage optimality .0011274, primal/dynamics 0,
continuous spatial LOS .000222144 m for .074 s (limit .0001 m).
CTCS Outer Max/Min, optimality 4.1469e13, row violation 1.40697e9.
Logs: /tmp/comando-gate-e1-{stage,ctcs}.log. Both seeds have zero residual.

E2 — Test whether continuation advances before solving its inner subproblem.
Change only inner-iteration budget 50 -> 2000 (total budget also 2000 to
observe the inner solve), retain default 10 line-search trials. No retained
parameter change. Pass: gate 3; compare termination/stationarity with E1.


E2 FAIL: stage stalls at Outer Max/Min after 1211 iterations, optimality .025455,
physical LOS failure persists. CTCS terminates at Outer Max/Min with large
residuals. Logs /tmp/comando-gate-e2-{stage,ctcs}.log.

E3 — Compared with E2, change only line-search trials 10 -> 30, testing whether
short backtracking forces premature regularization/outer termination. Same
budgets/limits/objective/seed. Pass: gate 3. Diagnostic only; result below.

E3 stage FAIL: 336 iterations, Outer Max/Min, optimality .0159; line search
reaches regularization ceiling at final barrier. CTCS result is recorded below.

E4 — Hypothesis: raw objective and tight normalized CTCS row have mismatched
numerical scales. Compared with E3, scale the entire objective and all of its
derivatives by 1e-3 (same positive factor, both formulations). Preserve roots,
physical limits, integral budget and initialization. Diagnostic wrappers only;
production OCP unchanged. Pass: gate 3; report both scaled stationarity and
factor, do not claim raw stationarity is equivalent. No tolerance relaxation.

E3 CTCS FAIL: 2000-iteration cap, optimality about .0012, physical failure.
E4 stage: Optimal Solution in 45 iterations, scaled optimality 1.10046e-7,
primal/dynamics zero, distance 1.92115 -> 1.40365. Physical FAIL: continuous
LOS .000222114 m for .074 s. CTCS FAIL with large residuals after 48 iterations.
No production scaling retained. Logs /tmp/comando-gate-e4-{stage,ctcs}.log.

E5 — Relative to E4 CTCS only, change initial barrier 1e-5 -> 1 to match stage
continuation under the common normalized objective. Hypothesis: the low CTCS
initial barrier permits an uncentralized first step. Pass: gate 3; all physical
settings and integral budget fixed. Diagnostic only; result below.

E5 FAIL: CTCS Outer Max/Min at 90 iterations, large primal/optimality.
Evidence /tmp/comando-gate-e5-ctcs.log. Barrier increase alone is rejected.

E6 — Isolated feasible-interior line search, coupled with E4 objective scale.
Hypothesis: for a feasible root/trajectory the current filter accepts arbitrarily
large nonlinear primal violations for a cost decrease; preserving the nonlinear
interior and recentering slacks/duals restores a useful central path. Use the
existing temporary forward-pass override /tmp/comando-mpc-feasible-barrier.o,
MPC_FEASIBLE_BARRIER=1, with the current untouched production solver library.
No shared source edits. Match E4 otherwise. Pass: gate 3; only consider retention
following isolated reproduction AND standalone/RH regression. Prior unscaled
version failed warm MPC; this experiment only checks open-loop scaling.

E6 FAIL: isolated feasible-interior forward pass returns zero primal/dynamics
but Outer Max/Min, optimality 6.01885e8, spatial LOS .000461326 m for .109 s.
Log /tmp/comando-gate-e6-ctcs.log. Reject override; restore default linker flags.
No shared solver change or objective normalization is retained.

Retained implementation: extend test_direct_mpc --open-loop with exact settings,
seed audit, termination/optimality, primal/dynamics, independent 1-ms interval
and continuous audits, physical tolerances and nonzero failure exit. Share the
existing physical audit definitions with constraint_audit, reject invalid
physical states, and expose raw optimality/converged evidence through core
results. Acceptance/control delivery behavior is unchanged. Full gate 3 still
FAILS; gates 4–6 remain unstarted. Verification and evidence archival are recorded above.

E7 — Verify the shared ordinary dynamics, stage/terminal objective gradients
and all six constraint signs/scales/Jacobians against geometry and central
finite differences in the existing direct-MPC test. No production change;
require relative derivative error <1e-5 and geometric residual error <1e-10.
Existing analytic actuator and folded quadrature/Jacobian/Hessian references
remain in the same test. PASS in the final sequential build/test.

E8 — Existing test_stc_landing_noaug fails staging-hover cold solve (30-iteration
budget, constraint 1.65674) in a fresh sequential build. Four other core CTest
cases pass; direct-MPC regression separately passed before E7's added checks.
Hypothesis: this is pre-existing rather than caused by diagnostic fields.
Build pristine planner cd3fda9 in detached /tmp/comando-baseline-ws worktree
against unchanged solver 4d68a48b, run the identical unmodified test. Pass for
non-regression: same failure/residual; any difference requires investigation.
No change to test budgets or acceptance. Baseline reproduces the identical failure (see verified checkpoint).


## Historical evidence (superseded as an execution sequence)

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


## Failed-startup cache correction

A reproducible planner bookkeeping defect is corrected without changing the
solver algorithm, OCP or control publication. OcpSolver cached failed X/U/K
and reused the optimizer merely because its object existed. With an infeasible
45-degree initial tilt in ordinary MPC, the next valid upright measured root
therefore received failed state/control seeds and the short warm-update path.
Before: 41 failed states / 40 controls cached; next solve failed with constraint
residual 235.689 and dynamics gap 1.30372. After: failed cache stays empty; next
call initializes a fresh solve and succeeds in 20 iterations with both residuals
zero. Successful prior solutions still populate and reuse the warm cache. No
within-call retry/cold-rebuild mechanism is added. The existing test_direct_mpc
now checks this regression and the full executable passes with max120/warm24
(including both existing moving-target cases). Evidence:
/tmp/comando-startup-warm-state-before.log,
/tmp/comando-startup-warm-state-after.log,
/tmp/comando-startup-warm-state-fix-check.log.

Separate CTCS normalization experiments remain exclusively under /tmp. A
fractional-density budget derived from interval * solver_tolerance^2 produced
severely ill-conditioned cold iterates. An equivalent logarithmic row bounded
the row magnitudes but did not restore feasibility. The existing conservative
line-search option, tested with a tolerance floor for error comparisons, kept
accepted predictions/execution within audited physical limits but fell behind
the target and expired (11 accepted / 22 rejected); this is a failure, not a
constraint-enforcement pass. None of these OCP/solver experiments is in the
branch. Further work must solve both useful motion/landing progress and actual
physical enforcement; do not substitute feasible hover or zero measured samples
for completion.


Follow-up validation: rebuilt test_controller and its lifecycle, mode selection,
ordinary RH/MPC cold solves, measured moving activation, derivatives and rate
recovery checks pass with max120, RH N40 / THH .16. Output:
/tmp/comando-startup-controller-check.log. A temporary gapped-capture cold seed
for the normalized conservative prototype also failed (zero accepted plans);
it remains outside the branch and must not be reported as a successful run.

## CTCS-only MPC numerical improvements (2026-10-17)

The user requested isolating ordinary folded CTCS before returning to STC.
`landing_mpc_ctcs` retains the ordinary physical limits, costs and dynamics;
all six path limits use a single always-active interval row. No additional
physical inequalities, safety controller, shared-solver changes, or SITL.

- Added local generalized GN curvature for the speed/rate norm consequences.
  The original residual GN terms remain; their calculation and the positive
  tangential norm correction share one RK sensitivity pass. Independent FD
  of the linear rate state/control blocks: relative error 1.73437e-10 (the
  residual-only approximation differed by 0.858866 on this diagnostic).
- CTCS-only initial barrier is 1e-5. It permits a tight cold solve inside
  120 iterations instead of exhausting the budget at mu=1e-3. Lowering the
  barrier alone did not restore reliable successive solves; retain the
  curvature improvement as well. Shared ALIPDDP remains at 4d68a48b.
- Normalize the CTCS row as I/epsilon-1, with the same raw integral budget.
  This fixes numerical acceptance being dominated by an absolute residual
  tolerance after dividing tiny budgets by legacy y_scale=10. Y_SCALE is
  no longer needed for this comparison. Positive finite epsilon required.
- Existing test_direct_mpc passes with max120/warm24, including new analytic
  integral, interval-Jacobian, tangential-curvature and PSD checks.
- Wall-clock constant target: epsilon=1e-6, 665 accepted / 0 rejected,
  landed, all measured and 1-ms dense predicted excess zero. Evidence:
  /tmp/comando-ctcs-final-constant.log.
- Wall-clock circle: same epsilon, 3 accepted then failed warm solves and
  policy expiry. Do not report it as a safe flight. Evidence:
  /tmp/comando-ctcs-onepass-relativebudget-circle.log.
- Synchronous fixed-step circle: epsilon=1e-8, 100 accepted measured-root
  solves, no failures, landed; 1-ms world-plant integration, exact u0 held
  for each 0.1-second interval, time frozen during solve. Max measured rate
  excess 1.00267e-5 rad/s; dense future excess 1.0038e-5, other limits zero.
  Peak actual rate .01901 against .019 exercises the constraint. Evidence:
  /tmp/comando-ctcs-final-synchronous-circle.log.
- Matched synchronous ordinary MPC also lands in 100 solves, measured excess
  zero and future rate excess 4.78031e-7. These runs establish useful tight
  CTCS feasibility, not superiority of CTCS on this particular path.
- Application-age warm-seed retiming and control-only single-shooting seed
  experiments did not fix the strict wall-clock circle; neither is retained.

Next: reconcile the wall-clock measured-root solve / delayed application and
held input with the tight CTCS prediction before SITL or STC claims. Keep
explicitly distinguishing the successful synchronous offline MPC reference
from the still-failing asynchronous circle. The goal remains incomplete.

Final default-budget wall-clock circle verification: 484 accepted, zero
rejected, landed; measured rate excess .0042305 rad/s, other excesses zero.
This improves over the initial comparison (316/3 and .0060141 rad/s), but
is not the tight-constraint completion gate. Evidence:
/tmp/comando-ctcs-final-default-circle.log. Final direct-MPC test passes in
/tmp/comando-ctcs-final-test.log. No shared solver files were changed.

## Matched active-constraint stress and open-loop feasibility check

The next comparison must exercise the limits rather than reuse the earlier
mostly slack circle. Temporary case: target constant velocity .455 m/s at
heading .6 rad, matched initial velocity, relative position (.518,0,1.85),
upright attitude, zero rates. Both formulations use speed .25 m/s, tilt .1
degree, rate .019 rad/s, glideslope 15 degrees with .1 m apex offset, LOS 16
degrees, and floor zero. CTCS interval budget is 1e-8; N40, dt .1 seconds.
These experimental settings have NOT replaced production defaults.

The user explicitly required checking open-loop feasibility before further
MPC work. Both exact OCPs admit the same nontrivial feasible trajectory:
a temporary speed/cone-aware initial guess reduces relative distance from
1.92115 m to 1.66210 m over four seconds. Its actual OCP constraint and dynamics
residuals are zero in both formulations. Independent 1-ms propagation of
every interval reports zero physical excess for all six limits. A matched
hover trajectory also provides an analytic feasibility witness. These are
feasibility witnesses, not claims of successful optimizer convergence.

Touchdown within this horizon is impossible: initial distance / speed limit
is at least 7.6846 seconds, already longer than four seconds. There is no
hard terminal touchdown equality in either OCP, so this does not make them
infeasible. Do not introduce a terminal touchdown requirement in this case.
With a tightly bounded upright attitude and LOS half-angle summing to less
than 90 degrees, exact LOS membership implies nonnegative height. Floor is
therefore redundant here; do not claim it independently changes the optimum.

Unmodified production initialization starts infeasible (ordinary residual
1.31571, normalized CTCS residual 2.65787e7). Production ordinary DDP returns
a nodally feasible plan, distance 1.40358 m, but terminates Outer Max/Min;
CTCS fails cold. A feasible initialization alone does not fix the CTCS solve.

Temporary diagnostic changes only to DDP's existing parameters (30 line-search
trials, inner budget matching total budget, total 2000) yield an integral-row
feasible CTCS plan, distance 1.40544 m, but optimality residual stays 1.5508e-4
against tolerance 1e-6 and termination is Max Iteration. Ordinary also returns
a feasible plan without Optimal Solution termination. CTCS dense physical
excesses include GS .000132759 m and LOS .0144429 degrees / .000380028 m;
zero integral-row violation is NOT zero physical violation. The corresponding
repeated MPC experiment fails on the first warm update. This is not a working
matched benchmark, and more iteration budget alone is not the remedy.

All stress harnesses, seed/parameter variants, and rejected solver experiments
remain under /tmp. No planner implementation or shared solver change is
retained from these checks. Evidence: /tmp/comando-mpc-constraint-stress.cpp,
/tmp/comando-stress-openloop-production-stage.log,
/tmp/comando-stress-openloop-production-ctcs.log,
/tmp/comando-stress-openloop-stage-extended.log,
/tmp/comando-stress-openloop-ctcs-extended.log,
/tmp/comando-stress-ctcs-longlinesearch-los16.log.

## Actuator integration correction

Ordinary MPC and its CTCS transcription now resolve the first-order rate
response with RK substeps no longer than 0.1 * rate_tau (previously 0.65 *
rate_tau). At defaults, this changes two steps per interval to thirteen.
The independent analytic response at .019 rad/s shows endpoint error falling
from 1.46301e-5 to 5.25194e-9 rad/s. A test at the command bound and at an
off-grid warm-shift duration checks the continuous response, independently
of the discrete-map Jacobian FD test. Native PX4 integration is unchanged.

Existing test_direct_mpc passes with max120/warm24, including both moving
cases and measured-rate recovery. Ordinary moving case: 49 accepted updates,
max warm solve 96.8079 ms. Evidence: /tmp/comando-rate-integration-test.log.
This improves model propagation accuracy, not the complete stressed MPC loop:
the temporary tighter case still fails its first warm update. Its dynamics
gap drops to 2.01732e-9, but the extended terminal interval has CTCS residual
1.145e6. The warm horizon tail must be investigated separately; do not
describe this numerical accuracy fix as completed constraint enforcement.
Evidence: /tmp/comando-stress-ctcs-accurate-los16.log.

The cold descent initialization now respects configured speed and reserves
height for the GS/LOS cones. The earlier fixed .5 m/s seed could violate the
.25 m/s stress bound before DDP started. Existing test_direct_mpc now exposes
an optional --initial-guess landing_mpc|landing_mpc_ctcs diagnostic, using the
same near-cone state and a 1-ms audit. Both strict-case seeds return zero
primal/dynamics/dense residual and distance 1.92115 -> 1.6621. This changes
initialization only, not optimized controls, constraints, objective or budget.
Full direct-MPC checks pass after the change (ordinary moving: 50 accepted,
max warm 96.3276 ms): /tmp/comando-seed-constraints-test.log.

Rejected tail experiments remain temporary: extending with the last DDP gain
made the terminal violation worse; keeping old feasible nominal states left
a .158292 dynamics gap and did not restore convergence. Neither is in the
branch. The stressed CTCS terminal state is almost at its tilt cap with
outward angular velocity, so holding the last command is not a feasible
extension. Evidence: /tmp/comando-stress-ctcs-feedback-tail.log and
/tmp/comando-stress-ctcs-feasible-states.log. The main goal remains active.

P1 startup diagnostic: native problem initialized and solved, but adapter logging
failed on scalar-valued J_tr_history (TypeError len(float)). No physical result.
P1b intended change: accept scalar/array diagnostic fields; OCP/settings unchanged.
Pass criterion remains full physical MPC audit; source hashes saved per run.

P1b startup: native solve and first independent interval audit succeeded,
but driver failed at obsolete dense_fields call. No complete MPC result.
P1c change: use shared dense replay helpers; identical OCP and defaults.
Same physical pass criterion, useful tracking and all diagnostics reported.

P1c complete: 32 s (literal inclusive pseudocode), 16 solves, physical
penetration zero but mean tracking error 11.4324 m and final x=-24.9649 m.
This is FAIL for useful reproduction despite the early adapter physical-only
status=pass. Source snapshot retained; that inadequate pass definition has
been corrected to require tracking, input limits and executed interval budgets.
Execution now stops at T=30 rather than one update beyond T.
P2 hypothesis: assumed w_prox=1 over the integrated-cost state suppresses
useful changes in the three-iteration solver. Change ONLY w_prox to 1e-4;
all dynamics, physical bounds, tightening .05, epsilon1e-6 and iterations3 fixed.
Pass: useful tracking mean<=1 m, no physical excess>1e-6, input norm<=5+1e-6,
executed interval integrals<=epsilon; report future defects independently.

P2 FAIL: completed30s, mean tracking44.4437m, physicalexcess0 because
vehicle departed obstacle region; max nonlinear future defect~2m, negative
linearized l values, max executed prediction drift4.54. No SCP success claimed.
P3 hypothesis: identity scaling of integrated cost over-damps physical motion
for w_prox1; coststate scale100 and objective coefficient compensation100
preserve physical objective while changing numerical conditioning. Only
scaling is changed relative to P1c defaults; first2s diagnostic before campaign.
Pass diagnostic: root tracking error reduced substantially from5m and
physical/control limits respected, no numerical failure; then full30s audit.

P3 initial2s diagnostic: position (-8.7711,3.2539), velocity(.8143,.6746),
physical excess0, normu max3.9497, cost22.0883; useful approach toward
reference, unlike P1/P2. Whole-horizon nonlinear defect.0054 and executed
prediction drift.0109 remain reported (not a convergence claim).
P3 full30s: same exact settings, no parameter change; acceptance as P2.

P3 full30s: physical execution PASS (zero obstacle penetration/duration,
minimum shape clearance.0311668, geometric clearance.0388717m, maxu4.31184),
useful tracking mean.746276m over15solves. CTCS integral budget FAIL:
max executed tightened integral2.22613e-5 versus1e-6. Future predictions
also have nonlinear defects; do not claim complete constraint convergence.
P4 hypothesis: the paper's ten-iteration diagnostic reduces remaining nonlinear
accumulator defects while preserving physical execution. Single change:
iterations3->10, all P3 settings including delta.05/eps1e-6/scaling100 fixed.
Pass: same physical/useful gate plus independently verified executed budgets.

P4 full30s: physical execution PASS, mean tracking.400330m (paper reports
.44m under unspecified remaining settings), zero physical violation/duration,
maxu5.0, minimum geometric clearance.039435m. Tightened executed integral
max1.26985e-6 still exceeds eps1e-6; numerical-budget status remains FAIL.
This is distinct from successful physical avoidance, not hidden as convergence.
DDP build PASS via single catkin target, normal linkage restored (E13 missing
object override removed). Solver algorithm source unchanged. Active-density
FD derivative relative error2.21e-10. Independent SciPy flow/cost/integral
fixture passes (raw evidence ddp_derivatives.json/ddp_independent_parity.json).
P5: change only control hold FOH->ZOH from P4; exact same original OCP,
tightening/budget/scaling and teniterations. Run30s before DDP comparison.

P5 complete30s ZOH: physical execution PASS, zero penetration/duration,
mean tracking.403897m, maxu5.0, minimum geometric clearance.0374448m.
FOH .400330 vs ZOH .403897: no material physical enforcement difference
in this original example. Tightened executed integral max1.26407e-6
versus1e-6 (budget FAIL still reported); future nonlinear defects remain.
DDP near-budget independent parity: RK4 M100 raw integral error1.12e-10,
flow error1.78e-15, costerror3.98e-13. Gradients pass FD; M100 chosen
explicitly for matched comparison, no physical OCP change.
P6 hypothesis: unchanged DDP with same ZOH physical OCP and density can
execute the original30s MPC. Only solver representation/algorithm changes
from SCP P5; same root, dt.5/N16/K4/reference/mask/tightening.05/eps1e-6.
DDP uses M100 RK4, max300iterations, existing stage SOC inputnorm5,
folded normalized interval rows, shifted nodes+controls and integratedtail,
fresh dual initialization per solve (explicitly not production dual reuse).
Apply only primal-feasible returned controls; log optimality separately.
Pass physical/useful criteria as SCP; independent budgets separate.

P6 PASS physical/useful/executed budgets over30s: all15 DDP solves Optimal
Solution, zero model dynamics gaps, mean tracking.250053m, zero physical
penetration/duration, maxu4.99999965, max executed integral9.99960e-7,
max executed prediction/plant drift1.67e-9. This uses fresh dual initialization
and does not establish the production retained-solver lifecycle or landing.
P7 hypothesis: production updateProblem barrier/penalty retention changes
successive MPC behavior. Single intended change from P6: retain the same
ALIPDDP instance using updateProblem; same max300 iteration budget, same
OCP/root/seeds/plant/limits/quadrature. Reinitialize only on actual stage
structure changes (no CTCS rows when sensed mask is empty). Each horizon
logs used_update_problem. No shared solver source edits. Same pass criteria.

P7 PASS full30s: all15 DDP solves Optimal Solution, updateProblem used13times,
zero model gaps, zero physical violation/duration, mean tracking.250058m,
max executed integral9.99933e-7, max prediction/plant drift1.70e-9.
Same physical outcome as fresh P6. Generic MPC/updateProblem failure is not
reproduced on this original problem; do not attribute the landing failure to
DDP generally. Folded per-interval CTCS works here with the paper tightening.
All Claude implementation/review workers finished; retained solver process
tears down cleanly. Source is retained in support/test/uzun2024_ctcs_mpc
and the single solver example, alongside exact per-run snapshots under logs.
No shared solver algorithm source edits. Catkin cache override was cleared
but existing planner/test executables were not relinked by the paper target;
do not treat old binaries as a freshly verified landing baseline.
Original demanding landing MPC/STC/SITL remain incomplete.

Retained solver example/CMake commit: d07d8d46. Final canonical model/audit
tests:18/18 PASS (model_tests.log); canonical SCP/DDP CLI imports and Python
compilation PASS. No shared alipddp/ or problem_maker/ diffs. All scenario
processes and the retained solver server are stopped. Source+results summary
retained in support/test/uzun2024_ctcs_mpc/README.md and logs comparison.json.

## Requested node-vs-continuous comparison (2026-10-19)

Current results are to be written and plotted, with an ordinary DDP stage-only
comparator. Outcome is measured, never manufactured. Parallel Claude worker
owns plot_comparison.py; parent owns nodal OCP mode and report. All builds and
scenarios remain sequential. Existing paper CTCS evidence is preserved.

P8 hypothesis: replacing interval CTCS by ordinary inequalities at every node,
including terminal, may permit between-node violations even with delta=.05.
Single intended change from P7: enforcement location (stage-only), preserving
root, reference, horizon/grid, cost, bounds, sensing, tightening and solver.
Pass for a demonstrated intersample counterexample: all enforced selected
node rows/primal gaps<=1e-6, yet independent executed dense max g>1e-6.
A dense-safe result will be reported as such. Complete30s, retained solver,
M100, max300 iterations, same control delivery. Integral remains diagnostic
only in stage mode and is not misclassified as a stage constraint.

P9 hypothesis: paper usual nodal baseline g<=0 without CTCS tightening permits
physical intersample penetration. Relative to P8 change only delta .05->0;
label the margin difference explicitly rather than attributing everything to
optimizer choice. Same demonstration criteria; do not coarsen grid, move
obstacles or otherwise force a failure. Plot all cases and selected nodes
against the independent1ms trajectory, physical g separately from integral.

P8 complete30s: node-feasible / dense-unsafe demonstrated. All15 returned
plans have feasible enforced nodal rows and zero dynamics gaps;12/15 report
Optimal Solution and3 Max Iteration (optimality not hidden). Executed maximum
node physicalg=-.0500000010, dense peakg=.6070366852 at12.25236s,
physical violation2.19815585s, geometric penetration max.381378m.
Max prediction/plant deviation7.10e-12: zero-deviation intersample violation.
CTCS same-tightening P7 remains physicalsafe. No physical limit/grid change.
P9 now runs the explicitly labeled untightened usual nodal comparator.

Goal-resume audit: previous turn is PROGRESS: paper SCP/DDP complete30s
campaigns, retained state verified, committed sources922574d/d07d8d46.
Active landing objective remains incomplete and is not redefined as the paper
benchmark. Current user deliverable is node-vs-dense results and plots.
P8 strongest intersample witness is from an Optimal Solution at solve6,
t=[12,12.5]: endpoints g=-.05000048,-.05000041, peak .6070366852.
Thus the failure is not explained by the three nonconverged solves; retain
their labels and report the converged witness separately.

P9 complete30s: all15 Optimal Solution, node physicalmax=-3.9763e-7,
dense max.6072811035, violation2.42926547s, max geometrypenetration.381092m,
max prediction/plant discrepancy7.10e-12. Nodal primal passes; physical dense
audit FAIL from five between-node intervals. No optimizerfailure needed.
Plots generated from saved dense data and visually inspected: overall
trajectory/g/tracking/integral and nodal closeups, PNG/PDF plus exactseries
CSV/JSON. RESULTS.md records both contrasts and convergence caveats.
Previous explanation-only user question was NO PROGRESS; resumed by confirming
P9 terminal state and finishing requested artifacts, not restarting runs.
Active landing goal remains incomplete.

P10 early evidence: fixed10 SCP nodal mode executes intersample obstacle
penetration. Physical dynamics defects remain up to ~1e-3 over future horizon;
will report explicitly instead of claiming converged zero-deviation SCP.
P11 regression hypothesis: CTCS branch remains numerically unchanged after
adding stage mode. Single change from saved P5: diagnostic adapter code only,
execute first2s/one solve with original ZOH10 parameters. Pass: X/U agree with
P5 solve0 within1e-8, same residuals, correct y state and integral rows. Short
startup tracking status is not a full-campaign verdict. Run only after P10.

P10 INVALID as intended nodal comparator: independent audit found node rows
up to+.616, because native OpenSCvx default lam_vb=0 makes nodal slack free.
Actual node physicalg+.54957, dense+.70326. Retained as setup failure, not
intersample evidence. No claim it satisfies node constraints.
P10b hypothesis: matching nodal L1 slack weight to w_ep=1e4 enforces the intended
ordinary node constraints. Single change: lam_vb=penalty (lam_vc already1e4).
CTCS branch has no nonconvex obstacle node rows, so this setting has no effect
there. Same physicalOCP/settings/grid/teniterations. Require node<=1e-6 before
attributing dense violations to intersampling; report nonlinear gaps separately.
Run after P11 regression finishes. Plot case must reference P10b, not invalidP10.
P11 CTCS regression PASS: saved X/U exactly equal P5 first solve (both0 max
absolute difference). Short-run aggregate fail is startup tracking criterion,
not a CTCS regression. Its source predates adding explicit lam_vb; because
CTCS has no nonconvex nodal rows this has no objective contribution. Verify
final-source branch again after P10b before claiming final regression.
First provisional plots from invalid P10 are archived as comparison_2x2_invalid_P10;
they must not be delivered as the matched comparison.

P10b complete30s/15 measured-root replans. Actual executed node endpoints are
all physically safe (maxg=-.0499976043); dense refined peakg=.7896887086,
physical violation2.623946857s across7 intersample-only intervals, geometric
penetration.406581m. No obstacle y state exists in saved slices; library clock
accumulator<=1.16e-11. No integral budget is applied to stage acceptance.
All15 MPC solves run10 SCP iterations, final conic statusoptimal. Strict
nonlinear gate FAIL: max modeled tightened node residual2.395731e-6 exceeds
1e-6, future physical dynamics defect.0120815, cost-integrator defect1.18817.
Rootdefect4.44e-15; maximum executed prediction/plant discrepancy1.65794e-4.
Do not claim zero-deviation or fully converged SCP. Actual node-safe/dense-
unsafe execution is independently established despite these residuals.
Same physical benchmark with .05 margin: SCPCTCS physicalduration0,
DDPCTCS0, DDPnodal2.19815585s. Allfixed30s / sameZOH / grid / physicalsettings.
Warm median solve seconds: SCPCTCS .169795, SCPnodal .207791,
DDPCTCS .416259, DDPnodal .134705. DDPnodal warmmax3.62285s (threecapped
solves), so median alone is inadequate. Separate initialization, IPC and audit
costs; do not infer solver superiority from unmatched iteration/convergence.
P11b final-source CTCS regression PASS, X/U parityexact0 against savedP5solve0.
18 existing independent model/audit tests PASS. No sharedsolverlibrarychange.
2x2 and performance plots rebuilt from saved evidence, initiallegendoverlap
caught by visualinspection and repaired. Plot formatting retry did not rerun
optimization or alter scenario data. Allfailures remain labeled.
Landing objective remains PAUSED; no landing experiments executed this task.
P12 early result: M20 reduces early solves (solve1 .177s versusP7 .588s),
all earlyOptimal and physicalclearance, but C++ independentfine integral at
solve1 futureinterval1.0019153e-6 >1e-6 while modeled9.999287e-7. Do not
accept coarse quadrature merely because solver row is feasible.
P13 hypothesis: M50 halves quadrature work while resolving activation peaks
well enough for the independent budget. Single intended change fromP7:
--substeps100->50. Same fullrun pass criteria asP12, no tuning or marginchange.
Run sequentially afterP12 completes, preserve failed candidates honestly.
P14 hypothesis: initialmu1 forces unnecessary barrier-reduction iterations;
mu.1 also lowers updateProblem's recenter floor .01->.001 without changing
constraints or stopping tolerance. Single change relativeP7: --mu1->.1 at
validatedM100 (not combined with P13 yet). Same fullrun acceptance, independent
integral/physical and all15Optimal. Distinguish numerics from changing epsilon.
Choose this isolated tuning test afterP13, not a broad parameter sweep.
P12 finished15/15Optimal, physicalexcess0/duration0, tracking.250060.
Independent executedImax1.00692670e-6: fixedbudgetFAIL (.69267% excess).
Candidate rejected as recommended profile despite faster solves and clearance.

P15 hypothesis: redundant interval integration and cross-stage cache thrash
can be removed by one shared FlowCache per stage between dynamics and cost.
Single intended change: exact memoization, atM100 and alloriginalsolverparams.
Keep a --flow-cache legacy path in the same local OCP to isolate/cache parity;
shared solver/StcStageConstraint and quadrature/density unchanged. Pass:
existingderivativechecks plus full15solve X/U/iterations/termination parity
againstP7 to1e-8 (expect exact), physical and budgetchecks unchanged, measured
warmmedian/p95 improve. Newflag diagnostic-only, no ROS/control changes.
Implement whileP13 runs oldbinary; build only afterP13 is confirmed terminal.
ActualClaude review confirms four sensitivity integrations per acceptedstage
plus trialvalue repeats; this change removes only dynamics/cost duplication,
not CTCS interface integration or sharedsolver work.
P13 completePASS:15/15Optimal, physicalexcess0/duration0, independent
executedImax9.99905365e-7<=1e-6, tracking.250056094 (P7 .250057624),
maxexecutedpredictionplant1.75923e-8. Host backgroundload rose above18, so
historicalwallclock medians cannot establish clean speedup. Exact paired
replays on one newbinary will measure cache speed and CPUtime separately.
P15 validation extension: replay every saved retained-MPC request fromP7 and
P13 in two independentservers (legacy/shared), sequential solves with pair
orderalternated. Require all15fulltrajectories/iterations/termination equal
saved audited outputs<=1e-8. This verifies the entire measured-root/shift/tail
sequence, not a singlehorizon. The existing independentdenseplant audit is
transferred only when complete control/state parity holds; do not claim newly
integratedplant or async/real-time completion. Sameimmutable modeldata/cache
perstage, noCTCSquad/solverinternalschanged. CompareCPUtime duehostcontention.
P14 (mu.1 isolated tuning) remains planned, not run; prioritize verified exact
reuse first. No claim about that unexecuted candidate.
P15/P16 PASS: paired full15-request retained-MPC sequence replays atM100 and
M50 both match original audited trajectories/iterations/termination EXACTLY
(maxdifference0). Newbinarylegacy also matches both oldbinaries. Bothsource
campaigns passedphysical and integralchecks; samecontrols/states transfer the
independent30sdenseaudits. No freshplant integration is claimed for replays.
Warmmedian currentpaired measurements: M100legacy825.985ms, shared617.586ms;
M50legacy319.005ms, shared224.404ms. Combined3.68x median and2.50x totalCPU
improvement; p95 1852.783->842.254ms; latermax1962.532->1214.989ms. M50shared
max is worse than M50legacy1095.152ms, so avoid per-solve speedguarantees.
Hostload>18; allvalues scoped tocurrentpaired replays, not originalSCP comparison.
All100/50 derivative checksPASS. Physicalmodel/defaultbounds/epsilon/delta/
objective/tolerance remain unchanged. Sharedsolver and CTCSinterface untouched.
Retained recommendation is --substeps50 --flow-cache shared. Reference defaults
remain100/legacy for reproducibility. P14mu.1 deferred; notneeded for this
verifiedprofile and notclaimed as a successful experiment. Benchmarkdriver
adds pairedsavedrequest mode and CPUtime field, no publicROS interfacechange.
Speedreport and CSV/JSON saved, sourcehashes/snapshots verified exact forP15/P16.
Landingrepair remains PAUSED. This task changes only the originalpaperbenchmark.
Finalverification: single-target sequential buildPASS; RK100/RK50 existing
finite-differencechecksPASS;18 independent model/audit regression testsPASS;
Pythoncompile andgitdiffchecksPASS. RetainedlocalOCP cache commit3a962c5a.
No scenario/build/server processes from this task remain running.
P17 fullPASS:15/15Optimal, totaliterations593->530, zeroexecutedphysical
violation, independentlyexecutedImax9.99965105e-7<=1e-6, tracking.250055296m.
Final/globaltolerance1e-6 remainsunchanged. Initialbarrier/epsilon/tightening
unchanged; advance intermediatebarrierstages sooner, not infeasibleacceptance.
P18 hypothesis: identicalsaved roots/seeds and pairedsequentialreplays measure
whether reducedintermediatecentering improves timing ratherthan hostload.
Single intended change between profiles: inner_tolerance10 vs100, cache/grid/
OCP identical. Reuse existing replay diagnostic with --replay-inner100; candidate
must exactlymatch savedP17 trajectories/iterationcounts, baselineallowedto
have different convergedtrajectory; both require primal/dynamics/independent
finebudget/selectedphysical/inputchecks. No control delivery fromthis replay,
no claim that baselineandcandidate trajectories are identical. Fullindependent
plant audit belongs toP17. Run only afterP17isconfirmedterminal.
P18 PASS: pairedsame-root/same-seed baselineinner10 andcandidateinner100
bothconverge and passmodeled/independentfinephysical/integral/inputchecks.
Candidateall15trajectories/iterations exactlymatchP17. Baselineisexpectedto
have a different solution: no globaltrajectoryparity or audittransferclaim.
Medianwall164.888->139.751ms (15.24% lower,1.180x); p95441.165->417.142ms;
max577.923->573.579ms (littleworstcaseimprovement); totalCPU2.584386->2.247520s.
No comparison toold224ms acrosshostload is claimed. P17fullMPCiterations
593->530 (10.62% fewer), maxoptimality8.94937e-7 andreturnedprimal0.

P19 regression hypothesis: extending the existing replayhelper with an optional
innercomparison does not change its originalcache-parity mode. Single change
fromP15: diagnostichelpercode, sameP7savedrequests and currentbinary. Require
all15legacy/shared fulltrajectory/iteration/terminationparityexact0 andnew
independentfinechecks. No optimizationmath orplant changes. RunafterP18.

Landingread-only finding: archivedE1CTCSlog is present. Feasibleproductionseed
haszero primal/dynamics; returnedplanphysicalFAIL: speedexcess.29641648m/s,
tiltexcess1.18789182deg, LOSspatialexcess.06767562m, dynamicsgap0, interval
fineflowdefect1.96463e-8. E3rawlog firststepacceptsalpha1 andnormalizederror
1.750489829e10 fromzero-feasibilityseed. Thisisrealfeasibilityfailure, not
merelystationarityfailure orplanttrackingdeviation. Currentdensityhaszero
firstderivativewhileallconsequences<=0; terminalweights2500/900 andtight
angularnormalizations/budget1e-8 amplifythe firstinfeasiblestep. Mechanism
plausible; noisolatedcausalrepairhasbeenverified. Landingdensityhasnophysical
bufferaddedtopathrows, unlikepaperdelta.05 (nottransferableacrossunits).
Priorprogressrecord also reports warmtailresidual1.145e6 despite2e-9gap:
laststateattiltcap/outwardrate makesheldlastcommandaninfeasibletail.
The /tmprawlogs citedbythatolderentryarenotcurrentlyavailable, so that warm
mechanism remains historicalevidence pendingfreshpersistenttraces; do not
claim afreshreproduction orverifiedfix. No landingexperimentrunwhilepaused.
P19 oldcachepath regressionPASS: all15legacy/shared trajectories/iterations
exactlymatch savedP7 (maxdifference0); newfinechecksallpass.18existingmodel/
auditregressionsPASS, PythoncompilePASS, invalidreplayargumentguardPASS.
Currentpaperstoprecommendation optionalinner100 onM50/shared retainsfinal1e-6.
No solverlibrarychanges. Pairedhelper snapshots forP18/P19matchtheirrecorded
sourcehashes; laterCLIargumentguard onlyrejectsinvalidflagcombinations.
WrittenRESULTS/README now separate testedstoppinggain fromunverifiedlanding
repair andmarkmissinghistorical/tmptraces. LandingremainsPAUSED.


## Logging audit, actual CTCS path overlay and solver distinction

User asks whether logs are safe, to show actual SCP/DDP CTCS paths, and about
interior-point versus conic enforcement. No optimization/landing experiment.
Checked key runsP5/P7/P10b/P12/P13/P17/P18/P19: required configs/summary/solves,
completecampaign arrays/denseplant/executedCSV and everyrecordedsourcehash
match the saved snapshots. Rawlogs are Gitignored; localstorage is not an
offmachinebackup. Known missing olderlanding/tmp traces remain disclosed.
Extendexistingplotter with executedCTCS-only overlay, saveddataP5SCPZOH10
andlatestverifiedP17DDPM50/shared/inner100; independent1ms actualplantpaths,
allfive physicalobstacles and tightenedhalos, notpredictedhorizoncurves.
Source/dataexport andvisualinspectionPASS. Plotdoesnotrerunoptimization.
CreateaSHA256-inventoriedlocalarchive ofallavailablepaperlogs, archivedlanding
gatelogs, canonicalreproductionsource/record, currentbinary/buildflags/revisions.
Archiveandchecksum live underlogs/archives; verify everyarchivedfileagainst
inventory. This doesnotrecover missing/tmpdata or claimremote backup.
Clarabel0.11.1 installedprimaryMETADATA describesitsinterior-point conic
algorithm. BothcurrentDDPconstraintmethodandSCP'sconicbackend useIPM; they
solve different nonlinear/quadratic versus convexifiedsubproblems. Innerconic
feasibility doesnotremove outerSCP nonlinear/integraldefects. Fixedbudget,
tightening, integration and finalnonlinearfeasibility remain necessary.


## Point-mass open-loop four-arm review result — 2026-10-19

Extended existing local compare_fair.py, preserved original fixed/free figures
and reused their styling. Planner versioned mirror and POINT_MASS_OPEN_LOOP.md
record the implementation and limitations. Froze and exactly replayed 34 saved
policies, all saved obstacle grades reproduced within 1e-7 m, SCP replay arrays
within 1e-10; <=1 ms plotted spacing. Snapshot v3 completed, visually inspected
both figures, 324 file SHA256 inventory verified. Earlier plot-only failures
are retained and labeled in the report; no new solver runs.
Fixed N6 tf4: DDPstage +8.828351mm penetration/0.112693685s, SCPstage
+23.535304mm/0.169057703s with feasible nodes and negligible dynamics gaps;
DDPCTCS clearance3.172080mm, SCPCTCS0.736495mm, both physical and integral
checks pass. Both epsilon1e-11; individual speed-bound tightening differs.
Free N10 SCPCTCS diagnostic has physical safety/budget .9996561763eps,
gap1.41466533e-8 narrowly misses unchanged1e-8 gate. Saved free DDPCTCS N6
fails matched integral replay at36.0518eps despite physical clearance; label
failure rather than inherit old saved zero-integral grade. No causal claim.
Review corrected the old local note's matched-seed claim: DDP consistent
line velocity vs SCP zero velocity. Saved timing and seed differences preclude
a controlled speed ranking. Existing active SCP sweeps remain untouched;
missing accepted cells stay explicit. No landing or shared-solver changes.
