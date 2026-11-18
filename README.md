# CoManDO planner

Current documentation for the active `mpc` branch, updated 2026-11-05. The MAVROS/PX4 ROS1 planner uses a ROS-independent core for RH replanning and direct MPC. The previous planner is archived in `../comando_planner_old` and excluded from catkin discovery. The shared ROS2 adapter builds; Crazyflie MPC flight validation and deployment remain pending.

## Layout

| Location | Purpose |
|---|---|
| `include/ocp/` | Five complete OCPs. Each file owns its dynamics, costs, constraints, derivatives and descriptor. |
| `include/parameters.hpp` | Shared typed vehicle, horizon, cost, constraint, MPC and solver defaults; startup environment overrides preserve saved tuning. |
| `include/target/` | One target generator and one constant-acceleration predictor, independent of ROS and the OCP files. |
| `include/planner_core/` | Three public headers: controller API, shared types/registry and coordinate adapters. |
| `src/` | Compiled core and registry; solver, RH/MPC runtime and asynchronous logging internals. |
| `support/transport/ros1/`, `support/transport/ros2/` | ROS nodes, platform adapters and launch files. |
| `config/ros1/`, `config/ros2/` | Hover, CTCS RH/MPC and ordinary RH/MPC profiles. |
| `support/scripts/`, `support/tools/analysis/` | Profile application, sequential campaigns, metrics, physical/control audits and existing plotters. |
| `support/rviz/`, `support/test/` | Retained viewer settings and six opt-in regressions. |
| `logs/` | Bags, CSVs, applied profiles, source/environment manifests, results, plots and GIFs. |

## Frozen-time measured-state NODE and CTCS baseline

The active solver is `../ALIPDDP-main` at `002850df`. Its default single-shooting
iLQR path restores the lab-original numerical implementation from
`/home/lab/projects/STC/multiquad_carrier_landing/src/solver/ddp` (native `main`
at `1bc94647`). The previous `4d68a48b` baseline is not that original solver.
The existing opt-in FDDP/full-DDP extension is retained; pointmass uses its
existing FDDP setting. No new numerical selector was introduced. The planner
branch remains `mpc`; its RH loop, OCPs and pointmass adapter are unchanged.

The supplied `src/examples/quad_sh_ctcs/quad_sh_ctcs.cpp`, linked to our solver
through its public interface (`STC_ROW=header`), reproduces **100/100 landings**
using the supplied `run_100ic.py` profile. All 100 node CSVs match a direct
original-core run byte for byte; ICs 0–2 also match fresh builds of the supplied
reference. The additional existing physical graders report 89/100 compliant
and certified: 100/100 is a landing result, not a zero-violation claim.

Current frozen measured-root regressions pass: quad NODE lands after 19 replans
(9.58346 simulated seconds), pointmass NODE completes 15 replans, and pointmass
CTCS completes 15 replans. Native violation is zero in all three. Pointmass CTCS
has zero executed obstacle excess and executed budget ratio 0.9999601983.
NODE between-node checks remain diagnostics. Results, profiles and hashes:
`logs/frozen_node_baseline_20261104/clean_solver_restoration_20261105.json`.

Pointmass NODE, the current pointmass CTCS reference-adapter case, and quad NODE
have passing frozen-time results recorded in
[`support/test/frozen_node_baseline.json`](support/test/frozen_node_baseline.json).
Quad interval-only CTCS RH now has a **scoped passing frozen staging/moving-target
case** using preset `n80` (`STC_PRESET=n80`). It completes 20 replans / 7.73252 simulated
seconds, with zero executed original physical excess and maximum planned
physical excess 8.13919e-7 against the unchanged 1e-6 audit threshold. Staging
starts active and deactivates after capture. Its full-horizon control and first
RH request have identical prepared and returned trajectories. The solver and
RH loop remain unchanged; the planner now honors the supplied SH barrier and
positive thrust/duration-bound scaling settings. The profile uses existing
physical tightening and stronger terminal position accuracy weights.

This is not a general cold-start fix or a strict integral certificate. The
maximum native row residual is 1.76514e-11, below native tolerance 1e-6. A
30-case budget/conditioning sweep retained equivalent row scaling Y=0.5 instead
of 10; the physical budget remains 1e-5. Native budget ratio fell from
1.00324973 to 1.00000088257 (0.0000883% overspend, or 8.83e-12 physical integral
units). This is a 3682-fold reduction, not strict-zero budget satisfaction.
Independent refined ratios remain 1.85482 planned / 1.00443 executed; they are
diagnostics and do not constitute a passing finer-integral certificate. Both
the full single-horizon check and a fresh RH repeat pass with the selected
profile; first-request prepared/returned trajectories are identical. The
preceding Y10 profile failed some stationary-hover, inbound and near-center
cold starts; broader coverage with Y0.5 remains unestablished.
The literal physical checks and solver tolerance were not
weakened. Native cone-normalization, inner-loop, nominal-hover-tail and
shrinking-horizon trials did not supply a general fix; their changes were
removed or left unselected. Profiles, failures and passing regression evidence:
`logs/frozen_node_baseline_20261104/quad_ctcs_rh_from_sh_20261105.json`;
passed raw runs are retained alongside it as a `.tar.gz` archive. The later
30-case tuning sweep and selected single/RH runs are retained in
`quad_ctcs_budget_tuning_20261105.tar.gz` alongside that record.

```bash
export STC_PRESET=n80   # stc_landing_noaug numbers: Params presets in the OCP file
build/test_measured_root_rh quad matched 1 /tmp/quad-ctcs-single --ctcs --staging-initial --audit-loop --single-horizon
build/test_measured_root_rh quad matched 40 /tmp/quad-ctcs-rh --ctcs --staging-initial --audit-loop
```

Frozen-time checkpoint (2026-11-05): the four recorded loop regressions pass
under their existing respective checks: quad NODE (19 replans), quad CTCS N80
(20), pointmass NODE (15), and pointmass CTCS (15). This is scoped matched-plant
validation, not a general initial-condition or real-time claim. Quad CTCS is
not strictly integral-budget certified, as quantified above.

Preset `n60` preserves the N80 nominal
horizon time while using N60, initial duration .0933333 s, duration bounds
.0666667–.16 s, and **three** executed intervals per replan. This tuned case
passes its single-horizon and RH physical checks (20 replans / 7.70133 s,
zero executed physical excess, planned excess 8.51501e-7). Mean solve time in
this run was 483 ms versus 656 ms for N80. Native budget overspend remains
2.26%, so the N80 profile remains the baseline. The 59 tested coupled parameter
configurations / 78 runs, failures and selected profile are recorded under
`coarse_dt_tuning` in the existing evidence JSON. Selected raw runs were added
to the existing budget archive under `coarse/`; no new results directory.

The shared loop is `src/synchronous_mpc_internal.hpp`; quad NODE and CTCS
factories share `include/ocp/ocp_stc_landing_noaug.hpp`. Pointmass NODE and CTCS
in this harness use `support/test/pointmass_reference_adapter.hpp`, importing
`../ALIPDDP-main/problem_examples/uzun2024_ctcs_mpc.cpp`. The single regression
entry point is `support/test/test_measured_root_rh.cpp`. Full checkpoint status
and summaries are in `support/test/frozen_node_baseline.json`.

Run each command in a fresh shell without inherited SZMUK/STC/MPC overrides;
use a new output path per run:

```bash
# Same executable target for all four, built from this repository.
BIN=build/test_measured_root_rh
$BIN quad matched 40 /tmp/checkpoint-quad-node --audit-loop
$BIN pointmass matched 15 /tmp/checkpoint-pointmass-node --audit-loop
$BIN pointmass matched 15 /tmp/checkpoint-pointmass-ctcs --ctcs --audit-loop
# Quad CTCS N80:
STC_PRESET=n80 $BIN quad matched 40 /tmp/checkpoint-quad-ctcs --ctcs --staging-initial --audit-loop
# N60:
STC_PRESET=n60 $BIN quad matched 40 /tmp/checkpoint-quad-ctcs-coarse --ctcs --staging-initial --audit-loop --replay-intervals 3
```

Independent measured-plant follow-up (frozen clock): the shared loop also
accepts independently integrated physical state via the same public API. The
existing harness now accepts `--plant-acceleration-x MPS2` with `fine` plant
execution, adding unmodelled physical acceleration at 1 ms steps; default zero
preserves the historical tests. This is test-plant support only. The solver,
production shared loop, constraints and acceptance tolerances are unchanged.

With finer integration alone, quad NODE and pointmass NODE/CTCS pass, but N80
quad CTCS rejects solve 14 on future interval rows. With acceleration 0.01 m/s²
and baseline cadence, quad NODE still lands; quad CTCS rejects solve 15 on
future CTCS rows, while pointmass NODE and CTCS reject solve 7 at stage 0.
All saved prepared and returned roots, including rejected requests, equal the
measured root exactly. A one-interval cadence diagnostic lets pointmass CTCS
complete 60 replans / 30 s with zero executed obstacle excess, but its refined
executed integral still exceeds epsilon. It does not fix pointmass NODE.
The optional coarser quad profile also fails this independent-plant follow-up.
These are recorded failures, not a completed measured-plant or real-time fix.

Detailed results are under `independent_measured_plant_next_step` in the
existing checkpoint JSON, with console logs, cycles and first/last requests
inside the existing budget archive under `measured_plant/`. No new results
folder. Reproduce using the same profiles/commands above, replacing `matched`
with `fine`; add `--plant-acceleration-x 0.01` for the disturbed test. Numerical
barrier/slack/multiplier state is still freshly initialized on every request.

Measured-state PX4/Gazebo SITL progress (2026-11-17): the stationary-target
quad CTCS case landed with seven accepted replans and zero measured excess in
all eight original physical categories (floor, speed, tilt, body rate,
glideslope, LOS angle/cone and staging). Fifteen updates were rejected; this
is a scoped executed-physical pass, not universal recursive feasibility.
A later stationary repeat also went stale after two accepted plans; repeatable
stationary operation is not established. The moving circle (radius 1 m, rate 0.4 rad/s) still fails: seven accepted
plans, 94 rejected updates, stale trajectory and timeout. It must not be
presented as fixed. The native ALIPDDP solver remains `002850df` unchanged.

A further N40 timing/tightening trial (0.5 s lead + 0.15 s guard,
LOS margin 0.045 m, speed margin 0.08 m/s) landed with nine accepted replans,
one rejection and zero measured physical excess. Its identical repeat landed
with four accepted replans / 81 rejections but violated LOS by 0.016753 m,
glideslope by 0.002084 m and body angular rate by 0.0002906 rad/s. These runs
remain in the denominator. This trial is not promoted to a default profile.
There is currently no repeatably validated SITL CTCS baseline. Full measured
flight bags, physical audits, exact settings and failed single-horizon trials
reuse the existing compressed archive under `measured_sitl_repair_20261117/`;
the existing baseline JSON records them under the same key. All measured
physical audits cover OCP activation through touchdown handoff (or recorded
stop/end for failures) at MAVROS odometry resolution; they are not a
continuous-time certificate.

**Measured-state SITL profile (2026-11-18).** Start the planner with
`STC_PRESET=sitl` (staging 1.8 m and LOS trigger 1.5 m are in that preset) and
apply `config/ros1/profile_stc_landing_noaug_measured_sitl.yaml`. Hover target 2.4 m and settle window 2.02–2.6 m. Each replan is rooted at the
measured state, rolled forward under the PX4 tracking-error model to the plan
handoff, against the live target. These used to be opt-in and are now defaults:

- `measured_state_delay_compensation: true` also anchors the target and the
  terminal freeze on the live target state.
- Intervals before node 4 (`CTCS_TIGHTEN_FROM_NODE`) use the physical
  CTCS rows, so a fixed measured root inside the physical set but in the margin
  band is still feasible. Physical rows keep the tightened rows' trigger floors
  (`triggerFloors`), so they stay active at trigger boundaries.
- The native initial guess is one staged controller (`nativeGuessControl`).
- `reprime_after_rejections` defaults to 5 (`<= 0` disables).

Approach tilt, approach rate, approach speed and landing rate are sized from
what the planned trajectories used (a 1 ms dense audit of every accepted
horizon), not from PX4 tracking error. The remaining bounds are active in the
plans and keep the values moving targets need; see the env header. Physical rows integrate with 8 (`CTCS_PHYSICAL_SUBSTEPS`)
RK4 substeps, so a trigger crossed between quadrature points is still seen. In
the verification campaign (record `measured_sitl_20261118`) all 7 target
motions landed (circle, figure-eight, straight, stop-and-go, piecewise
acceleration, random acceleration, stationary). The part of every accepted plan
that was flown had no excess against the physical bounds, and the whole plans
stayed within 0.3 mm near the pad. Plant excess remained only in the PX4
tracking: body rate 0.34 rad/s on stop-and-go and 0.31 rad/s on straight, and
1.2 cm glideslope on stop-and-go. Frozen-time
(`test_measured_root_rh ... --native-initial-guess`) and wall-clock
(`WALL_RH_NATIVE_GUESS=1 WALL_RH_MAX_CONSECUTIVE_REJECTIONS=5`) loops with
this env land with zero executed physical excess. The earlier widened and
moving profiles were removed; their records remain in
`support/test/frozen_node_baseline.json`.

The retained runtime changes are optional native control initialization and
local PX4 tracking-error propagation to the future handoff. The latter
propagates the complete measured error under position/velocity and attitude/rate
feedback about the committed reference, rather than applying raw DDP moments
to a plant actually following full-state references. Live FCU position,
velocity and attitude gains match the recorded model defaults. The local model
omits nonlinear saturation, the velocity integrator and full motor dynamics;
it is not an exact PX4 model. Existing core analytic checks pass.

LOS has its own tightening, `margins.los`, separate from the geometric
margin. With a shared margin, values above 0.026795 m also
tightened the glideslope enough to exclude the landing point. The SITL
setpoints are trajectory references to PX4, not direct motor or body-rate
commands. Pointmass has no PX4 SITL plant here.

The circle recording identifies both forecast and tracking errors: at its first
LOS violation they were 0.1277 m and 0.1048 m, respectively. Its required steady
tracking tilt is about 0.9344 degrees against a 0.95 degree physical limit;
the inherited 0.2 degree tightening leaves a 0.75 degree optimizer limit,
which cannot support steady tracking at that acceleration. An experimental turning forecast derived from measured velocity and acceleration
passed kinematic and dynamics/CTCS derivative checks (maximum relative error
1.74e-11), but no tested full-horizon profile passed the unchanged 1e-6
physical check. That experiment was removed from active code; its patch and
failed gate results are preserved in the existing evidence archive.

ROS-free wall-clock RH checkpoint (2026-11-06): the standalone
`test_wall_clock_rh` harness now calls the shared production asynchronous
`Controller::configure/trySolve/sample` API while a separate 1 ms physical
plant keeps running. This is not the frozen `solveSynchronous` API or SITL.
Opt in with `PlannerCoreConfig.measured_state_delay_compensation=true`.
The full measured state is propagated under committed controls to the chosen
activation; target prediction, cost epoch and plan origin use that same time.
The state-predictor idea follows [Carlos et al., Section II-B](https://arxiv.org/html/2010.11264#S2.SS2);
the lab solver is retained, with no RTI/SQP or numerical-state continuation.

The standalone settings reserve an initial compute lead of 1.2 s plus a 0.3 s
guard, retaining that minimum when solve-time EMA decreases. Four intervals
are the minimum replay amount; real handoffs occur later when the computation
lead requires it. The plant executes the old control sequence during solves.
First activation also waits for its scheduled time. Late results are rejected;
there is no custom optimizer early stopping. Shift-only initial guesses now
clear previous feedback gains, matching their intended meaning. Model-specific
native constraints remain active; quad-only altitude/thrust sanity checks no
longer treat 4-state pointmass velocity/acceleration as quad altitude/thrust.

Runtime-completion results (not full constraint passes): quad NODE reaches the landing gate after 8 accepted solves / 10.379 wall-sim seconds;
quad CTCS after 7 / 9.209 s; both pointmass variants complete 16 solves / 30 s.
Quad sampled physical excess is zero. Pointmass CTCS obstacle excess is zero;
pointmass NODE executed shooting-node excess is zero (58 distinct nodes / 72
node samples), with expected inter-node excess retained as a diagnostic. The
quad repeats also complete. Raw quad CTCS budget ratios still exceed one
(maximum final/repeat ratio 1.00569552); no strict integral certificate is claimed.
All four frozen regressions still pass, with returned X/U identical numerically
to their previous saved runs on all 69 requests. Analytic delay-rollout,
shift-only-policy and existing core tests pass. Neither native solver equations
nor constraints/tolerances were changed.

**Constraint-review correction:** the complete saved-trajectory audit fails
quad CTCS. The fourth proposed horizon has nominal LOS excess 1.49830e-6 at
horizon time 4.71903 s, above the unchanged 1e-6 threshold. Executed physical
excess remains zero. Its native planned budget ratio is 1.00474441; the finer
planned integral ratio is 1.56890080, while executed integral ratio is
0.99529422. Pointmass CTCS physical excess is zero and its planned/executed
integral ratios remain below one. Quad NODE and pointmass NODE pass their
native and executed-node checks. Therefore all four runtime completions must
not be presented as four complete constraint passes. Quad CTCS still needs work.

Wall-clock quad CTCS tuning now has a separate optional profile:
preset `wall_clock` (N80 with the two changes below).
Only horizontal terminal weight (100000 to 200000) and CTCS row scale
(0.5 to 2) change. The physical interval budget, solver/audit tolerances,
triggers, timing, native solver and shared loop stay unchanged.
Across 52 distinct profiles, this selection passed six repeated measured
wall-clock runs (42 accepted horizons) and the full single-horizon checks.
Worst proposed nominal LOS excess is 8.54438e-7, below the unchanged 1e-6
threshold; executed physical excess and planned/executed hard excess are zero.
This supersedes the untuned physical-audit failure above for this scoped test.
It does **not** establish strict CTCS budget satisfaction: worst native,
finer planned and executed budget ratios are 1.00046470, 1.80057045 and
1.77024742. Positive sub-tolerance LOS excess remains. Neither zero violation
nor a continuous-time certificate is claimed. Experimental substep changes
were removed; the frozen defaults and other three cases are unchanged.
Trial profiles/outcomes and selected raw runs reuse the existing archive under
`wall_clock_tuning_20261106/`; the checkpoint record stores all six audits in
`wall_clock_non_sitl_20261106.parameter_tuning_20261106`.

`runtime_complete` deliberately describes runtime progress only. The separate
saved constraint audit reuses the native rows and the frozen harness' nominal
implication/Simpson checks; it returns exit code 2 for this quad CTCS run.
It reads saved plans and plant samples without re-solving or modifying them:

```bash
python3 support/tools/analysis/audit_wall_clock.py /tmp/wall-quad-ctcs --binary build/test_wall_clock_rh --output /tmp/wall-quad-ctcs-audit.json
```

New standalone runs save their parameter environment in `profile.json`; the
audit also accepts the earlier grouped run metadata. Detailed findings are
under `saved_constraint_review_20261106` in the existing checkpoint record.

The ROS transports retain their existing default execution policy; this flag
has not been deployed there. These tests cover a matched-parameter plant with
finer integration, without sensor noise, actuator lag or hard real-time claims.
Earlier invalid quad adapter trials and unsuccessful timing/tuning trials are
explicitly marked in the evidence, not counted as passes.

Build and run in fresh shells without inherited SZMUK/STC/MPC overrides:

```bash
cmake -S . -B build -DCOMANDO_ROS_VERSION=0 -DCOMANDO_BUILD_TESTS=ON
cmake --build build --target test_wall_clock_rh -j1
# Quad NODE:
STC_PRESET=wall_clock_node build/test_wall_clock_rh quad node 20 /tmp/wall-quad-node delay-compensated
# Quad CTCS:
STC_PRESET=wall_clock build/test_wall_clock_rh quad ctcs 20 /tmp/wall-quad-ctcs delay-compensated
# Pointmass (separate fresh shell, no quad profile):
build/test_wall_clock_rh pointmass node 30 /tmp/wall-pm-node delay-compensated
build/test_wall_clock_rh pointmass ctcs 30 /tmp/wall-pm-ctcs delay-compensated
```

Each run stores `plant.csv`, `solves.jsonl`, all accepted horizons in one
`plans.jsonl`, a summary, and the latest/rejected request for inspection. The
existing checkpoint JSON records results under `wall_clock_non_sitl_20261106`;
selected raw runs are in the existing budget archive under `wall_clock_20261106/`.
No new repository results/log directory or branch was created.

The historical `d588218` passing quad result
used extra conditional nodal guards and a guided cold initializer. That result
was rejected as insufficient for the requested formulation. Both additions have
now been removed from production code.

The caller freezes simulation time during each fresh native solve, executes four
whole optimized intervals and then supplies the propagated plant state. Only a
trajectory initial guess is carried; IPM/AL numerical state is not reused.
Original physical STC triggers and limits remain the grading reference, with no
excluded switching band. NODE acceptance uses native nodal constraints; dense
violations are diagnostics for NODE.

The historical interval-only investigations before the clean solver restoration include scalar-row scaling, the retained
single-horizon curvature contract, native parameter configurations, consistent
trigger-tightening trials, an exactly equivalent SOC representation and finer
CTCS quadrature. None completed the required quad RH case. **Those trial code
changes were removed.** In the finer-quadrature trial, two replans passed before
the third returned infeasible; fresh standalone repeats matched that failure.
The independent integral discrepancy decreased, but the RH failure remained.
Results, profiles and discarded trial source are under
`logs/frozen_node_baseline_20261104/quad_ctcs_interval_only_investigation`.

The pointmass adapter's `100` substeps describe the recent reference test only.
They do not establish a requirement for the user's thesis code, which the user
reports worked with 2–3 substeps. A pointmass-versus-quad substep comparison is
not established as the cause of the quad failure. Historical thesis files have
not been modified or overwritten.

```bash
cmake -S . -B build -DCOMANDO_ROS_VERSION=0 -DCOMANDO_BUILD_TESTS=ON
cmake --build build --target test_measured_root_rh -j1
# Quad NODE check:
build/test_measured_root_rh quad matched 40 /tmp/quad-node --audit-loop
```

Saved-request replay uses the recorded descriptor, horizon and environment:
`test_measured_root_rh --diagnose-request quad REQUEST_FILE`.
The diagnostic now handles an exhausted finite horizon without attempting to
compare an empty rejected trajectory with a stale previous request.
Real-time operation and 100IC/MC1/MC2 validation remain outside these results.

Historical fixed-duration comparison (before restoration, 2026-11-05): the test adapter removes duration from
optimization entirely (four input decisions per stage), rather than pinning two
inequality bounds to the same value. It reuses the original model, costs and
CTCS implications, including their shared-row lifetime. `--fixed-dt 0.1` gives
80 intervals / 8 s, with four intervals / 0.4 s executed per RH update.
`--single-horizon` with a one-solve bound executes the entire returned horizon
without replanning. Adapter value/Jacobian/curvature parity is exact at the
sampled points; independent CTCS input finite differences pass.

The matched comparison uses the native-default noaug cost/constraint profile,
N80, initial dt=0.1, free bounds [0.05,0.12], max_iter1200, CTCS epsilon0.0016,
Y-scale10 and **no tightening exports**. It is not a reproduction of the thesis
100IC configuration or a claim that its earlier single-horizon results regressed.
Both single solves land and satisfy native interval constraints; both fail the
original physical STC audit. Fixed-time RH accepts the first prefix, then its
second solve returns native CTCS residual 9.34636e-6 against tolerance 1e-6.
Free-time RH fails on its seventh solve (2.64991e-4). Rebuilding a uniform hover
input guess each fixed-time request also fails on the second solve. Fixed time
alone therefore did not resolve this tested case. The fixed NODE near-center
control completes40requests with zero native violations. Raw requests, profiles,
full-horizon trajectories, independent replays and checks are in
`logs/frozen_node_baseline_20261104/fixed_dt_comparison_20261105`.

Historical parameter sweep (before restoration, 2026-11-05): 480 fixed-duration quad runs cover interval budgets
`[1e-6,1e-5,1e-4,.0016,.016]`, row scales `[1,10]`, regularization minima
`[.01,.1,1]`, fresh initial barriers `[.01,1]`, barrier reductions `[.1,.5]`,
and a full-horizon solve plus RH execution prefixes `[1,4,16]`. Each RH case
allows 20 simulated seconds; six cases reach the separate 60-second wall-time
cap. Of 360 RH cases, 45 land with native feasibility; none passes the physical
STC audit. The three smaller budgets yield no complete native-feasible RH
landings. Budget `.0016` yields 21 and `.016` yields 24, all at prefix 16.

Longer validation removes the 20-second classification limit: prefix 1 lands
after 207 replans / 20.7 s, and prefix 4 after 51 replans / 20.4 s, with zero native
inequality violation at budget `.0016`, scale 1, reg2 `.01`, initial mu `.01` and
mu_mul `.1`. A controlled follow-up keeps the original scale 10 / reg2 `.1` profile
and changes **only the fresh initial mu/muT from 1 to .01**: prefix 4 likewise
lands after 51 replans / 20.4 s with zero native inequality violation. All three
still fail physical STC checks; this is not a completed CTCS fix. Production
defaults and the solver remain unchanged. The resumable runner is
`support/test/run_quad_ctcs_parameter_sweep.py --binary PATH --output DIRECTORY`;
all settings, per-case results and extended validations are recorded in
`logs/frozen_node_baseline_20261104/fixed_dt_comparison_20261105/parameter-sweep-results.json`.
The sweep uses the recent test adapter, not an identified historical clean
fixed-dt baseline.

Paper check: Uzun et al. (2024), Sec.2.4, Eq.(11)-(12), impose a separate
violation-integrator increment bound on each interval, and reset the auxiliary
initial values through xc=(physical state,0,0) at each new OCP. Rebasing a shifted
auxiliary trajectory by subtracting its old prefix value follows from those
equations; it is not an IPM continuation prescription. Our interval-only host has
no violation-integrator state: a fresh row evaluates the new rollout's integral
minus the same epsilon (both divided by Y-scale). It carries neither previous
row residuals nor consumed budgets into future intervals. Sec.4 additionally
uses physical constraint tightening; it does not enforce physical satisfaction
by zeroing residuals. The paper's SCP initialization does not specify how to
initialize ALIPDDP slacks or multipliers. This review is based on the local PDF
`/home/lab/Zotero/storage/NR6G7V8D/Uzun et al. - 2024 - Successive Convexification for Nonlinear Model Predictive Control with Continuous-Time Constraint Sa.pdf`.

## OCPs and execution

| OCP | Coordinates | Execution and constraints |
|---|---|---|
| `hover` | World | Startup/staging state references. |
| `stc_landing_noaug` | Target-relative | RH with variable time steps and the shared CTCS density interface. |
| `stc_landing_mpc` | Target-relative | Fixed-grid direct MPC with the shared CTCS density interface. |
| `landing_rh` | Target-relative | RH with ordinary constraints active throughout the horizon and terminal node. |
| `landing_mpc` | Target-relative | Direct MPC with ordinary constraints on every future state and the terminal node; the fixed measured initial state is recorded separately. |

RH sends planned position, velocity, yaw and acceleration feedforward to PX4's tracking controller. Direct MPC starts each DDP solve from measured state and sends only its first optimized thrust/body-rate input on MAVROS `setpoint_raw/attitude`; PX4's rate controller remains active. The publication loop repeats that first input until the next measured-state solve completes. It never advances through the remaining controls of a solution. The command path contains no separate LQR/QP correction or post-solve slew shaping. The fixed-grid prediction horizon has 40 intervals of 0.1 seconds. Node zero is fixed measured data: input bounds remain there, while pointwise state constraints apply to predicted nodes 1 through N. CTCS also constrains the first predicted interval. This permits a recovery solve after a measured limit excursion without changing the measured initial state or relaxing future bounds. Each request constructs and initializes a fresh native solver from its configured parameters. Only the previous state/control trajectory supplies an initial guess; barrier, penalty, slack, multiplier and iteration state are not carried between requests. The legacy `reuse_solver` flag does not enable numerical continuation. `MPC_MAX_ITER` controls the configured solve allowance. Solve runtime and publication rate are distinct: 100 Hz publication does not establish a 100 Hz MPC update rate. Stale state or command expiry pauses into hold. Touchdown hands the landing action to PX4.

Both CTCS OCPs use `StcStageConstraint` in density form with analytic trigger/consequence derivatives. Noaug preserves its 14-state layout, original eight implications and scaled interval budget. The migration parity checks compared the shared row with the independent density/integral, including free-time derivatives, settle/capture/disabled-STC modes and nonzero trigger floors.

The ordinary OCPs contain no CTCS interval rows, violation integrators, state triggers, staging conditions or trigger-dependent costs. They share defaults and prediction with the CTCS versions but use an unconditional landing objective. Their always-active limits are 0.75 m/s relative speed, 0.95 degrees tilt, 0.019 rad/s body angular speed, 15 degrees glideslope with a 0.1 m apex offset, 30 degrees LOS and a zero relative-altitude floor. Speed, angular rate, tilt, glideslope and LOS use native SOC rows; the floor and actuator bounds use ordinary inequalities. Quaternion-dependent constraints normalize attitude.

## Estimator and frame contract

The core receives world position and linear velocity, a body-to-world quaternion in wxyz order, and body angular velocity in rad/s. World axes are ENU; body axes are FLU. Odometry describes the body origin; target odometry describes the landing point.

OCP selection never changes the estimator subscription. `World` uses canonical state directly. `TargetRelative` subtracts target position and velocity with world-aligned axes; target yaw does not rotate the state. Attitude remains body-to-world and angular velocity remains in the body frame. RH reconstructs commands using the live target, while planned paths use constant-acceleration prediction.

ROS1 defaults:

| Parameter | Default |
|---|---|
| `drone_odom_topic` | `/mavros/local_position/odom` |
| `state_velocity_frame` | `body` |
| `world_frame` | `map` |
| `target_odom_topic` | `/target/odom` |
| `target_velocity_frame` | `world` |
| `command_frame` | `map` |

Velocity conventions accept `body` or `world`. Target acceleration uses `/target/accel`. Configure topics and conventions at node startup. Inputs are transformed into the configured world frame using TF at message timestamps. Missing frames, timestamps or TF reject the input; timestamps must be no more than 0.2 seconds old or 1 ms in the future. MAVROS state-reference commands transform into its local ENU command frame.

For another estimator, supply calibrated static TF and full SI odometry. For example:

```sh
roslaunch comando_planner planner_launch.launch platform:=mavros \
  drone_odom_topic:=/estimator/odom state_velocity_frame:=world \
  world_frame:=map command_frame:=map
```

Mocap can feed PX4 through MAVROS vision/mocap for fused local odometry, or feed an external estimator. Raw pose alone lacks velocity and body rates. The producer handles marker-to-body calibration, and vehicle/target origins and axes must agree. Moving coordinate frames need additional velocity transport terms and are outside this contract. Old shifted/rotating relative-input modes and relative vehicle-topic switches are removed.

The ROS2 Crazyflie native adapter uses `world`, rotates native body velocity into world axes, and converts native gyro units into rad/s. A configured external odometry source supplies full SI state. Build compatibility does not establish Crazyflie flight performance.

## Solver validation update (2026-10-20)

NT SOC scaling is active; strict barrier scheduling is opt-in and production
retains the legacy gate/default tolerance. The planner and all production test
binaries were rebuilt. Two tests still fail with both the old SOC baseline and
NT, and one SITL smoke run ended `NO_SETTLE` before landing. Historical landing
results below do not validate this solver revision. See the
[production checks and ablation report](support/test/uzun2024_ctcs_mpc/POINT_MASS_DDP_ABLATION.md).

## Build and launch ROS1

Run these commands in bash, or source the corresponding `.zsh` setup files in zsh. Keep `ALIPDDP-main` as a sibling, or set `COMANDO_SOLVER_DIR`.

```sh
source /opt/ros/noetic/setup.bash
source /home/lab/mavros_ws/devel/setup.bash
catkin build comando_planner --no-deps -j1 -p1 --make-args comando_planner
catkin build comando_planner --no-deps -j1 -p1 --make-args target_publisher
```

Build one executable at a time with one worker. Use the installed PX4 simulator; campaigns do not rebuild PX4. Run one target at a time and do not build alongside SITL.

With PX4/MAVROS running, launch the target and planner in separate terminals:

```sh
roslaunch comando_planner target_launch.launch target_trajectory:=circle
roslaunch comando_planner planner_launch.launch platform:=mavros
rosrun comando_planner apply_profile.sh /comando_planner \
  /home/lab/mavros_ws/src/CoManDO_planner/config/ros1/profile_hover_sitl.yaml
```

After hover/staging, apply `profile_stc_landing_noaug_sitl.yaml` for CTCS RH. For direct MPC, use `roslaunch comando_planner direct_mpc_sitl.launch` from startup, then stage and apply `profile_stc_landing_mpc_sitl.yaml`. This launcher aligns the adapter/model thrust calibration. The profile script increments `command_seq` to activate a command. Mathematical environment settings load at process startup; restart to change them.

Saved MC1 RH settings include `SZMUK_RH_N=40`, `SZMUK_MAX_ITER=120`, `SZMUK_THH=0.16`, `SZMUK_W_LAND_GS=20000`, replay prefix 1, a 1.55 m staging floor and 1.2 m LOS trigger. Terminal freeze velocity threshold is 0.30 m/s and the staging hover command is 2.05 m.

Validated x500 direct-MPC settings:

| Setting | Value |
|---|---:|
| Normalized hover thrust | 0.7282821834 |
| Effective rate-response time | 0.08 s |
| Body-rate command bound | 0.35 rad/s |
| Maximum optimizer iterations | 120 |
| Maximum policy age | 2 s |
| CTCS staging floor / LOS trigger | 1.3 m / 1.2 m |
| Raw CTCS interval epsilon / scaling | 0.0016 / 10 |
| Thrust-acceleration / body-rate slew | 20 m/s³ / 5 rad/s² |

These are simulator calibration estimates. The first-order rate response and linear thrust mapping approximate the plant. The next solve runs when the worker and policy handoff are ready; command publication runs at 100 Hz. Publication frequency is distinct from the DDP solve frequency.

## Sequential cone-start experiment

Use `profile_landing_rh_sitl.yaml` or `profile_landing_mpc_sitl.yaml` with the campaign's `--start cone`. Preparation follows the target 1.85 m above its landing point, with acceleration/jerk limiting for high-curvature motion. Every tight measured landing limit must hold for 0.5 seconds. Preparation continues until the first planner command, with a 30-second start timeout and a first-plan check after 15 seconds. Cases stop on failure. A successful case requires the planner's touchdown handoff, PX4 disarm and clean teardown.

Example for one ordinary RH target:

```sh
python3 support/scripts/run_scenario_campaign.py \
  --motions circle --levels zero --trials 1 --start cone \
  --hover-thrust 0.7282821834 \
  --profile config/ros1/profile_landing_rh_sitl.yaml \
  --out logs/landing/ros1/ordinary_rh_check
```

For MPC, use `profile_landing_mpc_sitl.yaml`, staging value 1.3, and the saved startup environment: `MPC_RATE_TAU=0.08`, `MPC_HOVER_THRUST=0.7282821834`, `MPC_MAX_POLICY_AGE=2`, `MPC_RATE_LIMIT=0.35`. The campaign runner selects the `campaign` presets of `landing_rh` and `landing_mpc` (horizon 40, 120 iterations, 0.16 s duration cap). Refer to each saved cell's environment/profile for exact reproduction settings.

## Logs, analysis and plots

Keep landing outputs together under `logs/landing/ros1/<campaign>/<motion>__zero__t0/`. Cells save the bag, solver CSVs, `planner_environment.json`, `landing_profile.yaml`, `source.json`, `cell.json` and metrics. Direct MPC additionally records nominal/applied controls, feedback, limits, policy age and tracking errors in `direct_mpc_controls.csv`. Inspect `cell.json` and teardown status before treating a run as complete.

`scenario_metrics.py` produces campaign metrics. For physical constraint scoring, enable `COMANDO_BUILD_ANALYSIS` and build `constraint_audit` separately, then run:

```sh
python3 support/scripts/analyze_direct_mpc.py logs/landing/ros1/ordinary_landing_mpc \
  --physical --audit-bin /path/to/constraint_audit
python3 support/scripts/analyze_direct_mpc.py logs/landing/ros1/ordinary_landing_mpc
```

The physical audit checks accepted future nodes, 1 ms interval propagation, full-horizon continuous rollouts and measured state. Ordinary measured scoring begins at first controller output and ends at touchdown handoff or policy expiry. Preparation, later hover and post-handoff disarm are excluded. No accepted plan means no measured controller result. Horizon durations overlap and are summed, not elapsed flight time.

`support/tools/analysis/analyze_flight.py` converts bags and invokes the existing plotters. `bag_to_stc_csv.py` supplies executed-state CSVs; its `--t0`/`--t1` trim is relative to first vehicle odometry. Use saved environment settings and the controlled interval for correct bounds and scoring. Existing `plot_stc_paper.py` and `animate_stc_paper.py` generate state plots, world/relative 3D views and GIFs. Horizon tools show planned states and overlays. Ordinary plots use always-active limits and the altitude floor, with no staging or STC trigger annotations.

Older ROS1 campaigns and initialization diagnostics are preserved in `logs/archive/landing_history_20261019.tar.gz`. Current retained landing evidence is under `logs/landing/`; see [the log index](logs/README.md) for locations and restore instructions. Historical `archive::member` paths below identify recoverable original locations.

## Latest validation

Six existing core tests passed: normalized coordinates, lifecycle/modes, replay and acceptance/handoff, CTCS parity, noaug warm starts, direct dynamics/feedback/expiry, and ordinary cone/terminal derivatives and start feasibility. Stationary and measured moving-start cold solves passed. The active ROS1 and shared ROS2 planner builds passed sequentially. Tests are opt-in (`COMANDO_BUILD_TESTS=ON`); standalone builds select `COMANDO_ROS_VERSION=0`, ROS1/ROS2 use 1/2.

Latest flight campaigns used one zero-noise trial per circle, figure-eight and straight case, sequentially. All final cases shut down cleanly. Successful flights were not repeated during final analysis. Data are in:

- [CTCS RH regression](logs/landing/ros1/frame_regression_rh)
- [CTCS MPC regression](logs/landing/ros1/frame_regression_mpc)
- [Ordinary RH cone starts](logs/landing/ros1/ordinary_landing_rh)
- [Ordinary MPC cone starts](logs/landing/ros1/ordinary_landing_mpc)

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

## Existing plots from controlled ordinary runs

| Ordinary controlled run | States | 3D | GIF |
|---|---|---|---|
| RH circle | [States](logs/landing/ros1/ordinary_landing_rh/circle__zero__t0/flight_states.png) | [3D](logs/landing/ros1/ordinary_landing_rh/circle__zero__t0/flight_3d.png) | [GIF](logs/landing/ros1/ordinary_landing_rh/circle__zero__t0/flight.gif) |
| RH straight | [States](logs/landing/ros1/ordinary_landing_rh/straight__zero__t0/flight_states.png) | [3D](logs/landing/ros1/ordinary_landing_rh/straight__zero__t0/flight_3d.png) | [GIF](logs/landing/ros1/ordinary_landing_rh/straight__zero__t0/flight.gif) |
| MPC circle | [States](logs/landing/ros1/ordinary_landing_mpc/circle__zero__t0/flight_states.png) | [3D](logs/landing/ros1/ordinary_landing_mpc/circle__zero__t0/flight_3d.png) | [GIF](logs/landing/ros1/ordinary_landing_mpc/circle__zero__t0/flight.gif) |
| MPC straight | [States](logs/landing/ros1/ordinary_landing_mpc/straight__zero__t0/flight_states.png) | [3D](logs/landing/ros1/ordinary_landing_mpc/straight__zero__t0/flight_3d.png) | [GIF](logs/landing/ros1/ordinary_landing_mpc/straight__zero__t0/flight.gif) |


## DDP-to-PX4 execution correction (in progress)

The unflown separate tracking-QP experiment was removed. The older custom LQR backward pass, feedback correction and post-solve command shaping were also removed. Both ordinary and CTCS direct MPC publish the first DDP-optimized input unchanged; subsequent solves start from measured state. RH continues to publish DDP state references for PX4 tracking.

The non-ROS solver runs are the reference for checking the ROS/PX4 mismatch. This correction does not establish physical constraint satisfaction: matched simulator checks, actuator/tracking model validation, and faster warm-started DDP updates remain required. No physical limit or solve-acceptance threshold was changed.

Verification for this correction: all six core tests passed, including exact DDP control delivery and scheduled-policy handoff. ROS1 and shared ROS2 planner builds passed sequentially. The historical ROS horizon audits are not matched comparisons with the non-ROS driver. New SITL constraint validation remains pending.

The first DDP-only ordinary MPC straight run (`logs/archive/landing_history_20261019.tar.gz::ddp_only_landing_mpc`) timed out after policy expiry. Its accepted horizon had zero future-node physical excess, but first controlled measured angular speed was 0.05753 rad/s. During the first solve, the startup reference was frozen despite a nonzero moving-target velocity; that reference update is now maintained until DDP handoff.

RH MAVROS commands now include world acceleration feedforward computed by the selected OCP as `a = R(q) * [0,0,T/m] + g`. The full equations stay in each OCP. Target acceleration is not added twice. Hold/preparation commands have no fabricated thrust feedforward. MAVROS rotates acceleration into the command frame without translating it. Six core tests and the ROS1 build pass; simulator verification remains required.

The x500 SITL launch uses the actuator inverse `signal = (hover + offset) * (a/9.81)^(1/exponent) - offset`, with exponent 2 and offset `150/850`. The measured PX4 ESC range is 150–1000 rad/s and `THR_MDL_FAC=0`; Gazebo rotor thrust is quadratic in rotor speed. This converts the physical DDP thrust input to the simulator signal without adding feedback. Generic parameter defaults retain exponent 1 and offset 0; override `MPC_MOTOR_THRUST_EXPONENT` and `MPC_MOTOR_SPEED_OFFSET` for the identified actuator model. Both MPC OCPs use the same mapping to derive their achievable thrust ceiling. This mapping still needs SITL validation; it is not a robust constraint guarantee.

Follow-up evidence: `logs/archive/landing_history_20261019.tar.gz::ddp_startup_landing_mpc/straight__zero__t0` ended TIMEOUT/CLEAN after two accepted plans. Initial measured angular speed improved to 0.02489 rad/s and controlled glideslope excess was zero, but maximum rate excess was 0.02895 rad/s and tilt excess 0.12268 degrees. `logs/archive/landing_history_20261019.tar.gz::ddp_motor_mapping_landing_mpc/straight__zero__t0` ended TIMEOUT/CLEAN after one accepted plan; it remains a failed constraint test. That run included an abrupt 0.90718 m/s target-velocity reversal at 0.849 seconds after handoff, unlike the preceding two runs. Thus the cases do not isolate the thrust mapping's effect. A drone closely matching pre-turn velocity cannot instantaneously match the reversal; anticipation or an explicitly bounded target-motion model is required. `target_velocity_jump.json` records the measured event. No change to the target generator or predictor has yet been made.

The sequential campaign now reads `control_stop_reason` and records policy expiry/stale-state failure promptly. This avoids waiting five minutes for a stopped controller to land and does not alter the controller, constraints or solver acceptance.

The shared ROS2 planner build passed after the acceleration-feedforward and motor-mapping changes. ROS2 flight validation remains pending.

RH reference consistency: accepted trajectories now use the OCP model to sample states between optimizer nodes under the unchanged optimized input. Each plan retains its own target forecast and time origin; live target measurements enter the next solve instead of deforming the active world reference. Handoff seeds convert through world coordinates when the target forecast changes. Both execution modes publish at 100 Hz, independently of solve frequency. All six standalone tests and the ROS1 build passed for these changes.

The earlier feedforward-only RH straight run (`logs/archive/landing_history_20261019.tar.gz::ddp_feedforward_landing_rh/straight__zero__t0`) landed but failed constraints: measured speed excess 0.33991 m/s, tilt excess 7.56934 degrees, angular-speed excess 0.60325 rad/s and glideslope excess 0.21684 m. It predates the reference-consistency fixes and is not evidence that they work.

PX4 inner-loop extracts are saved as `px4_inner_loop.csv` with source metadata alongside the three DDP-only MPC cases. The motor-mapping case shows approximately 148 ms between the first body-rate mode request and disabling PX4 attitude control; the first optimized control interval was not delivered as modeled. The current first-order MPC rate model also omits PX4 rate integrator and motor dynamics. These are unresolved execution-model differences; adding an independent correction controller would not resolve the requested DDP formulation.

The reference-consistency RH circle check (`logs/archive/landing_history_20261019.tar.gz::ddp_consistent_reference_landing_rh/circle__zero__t0`, commit `a36f2b6`) landed with clean teardown. Over 20.99 controlled seconds, measured speed, glideslope, LOS and floor excess were zero; tilt excess was 0.00752 degrees, below the audit tolerance. Angular-rate excess remained 0.02414 rad/s over 1.03593 seconds, so the constraint check failed. Publication median was 9.991 ms, with a maximum 82.493 ms gap. There were 128 accepted horizons: predicted speed/tilt/rate/glideslope excess was zero, but small terminal floor/LOS spatial excess remained. This is a circle case and does not isolate improvement against the earlier straight case. The shared ROS2 build passed, without a ROS2 flight.

ROS1 parameter polling now tries the exclusive session lock and retries at the next poll when a solve is active. Previously a waiting parameter writer could block replay until the solve finished. This removes that scheduling stall without changing DDP inputs or physical limits; the subsequent sequential case verified a smaller maximum command gap (see below). RH still seeds attitude/rates from the predicted handoff rather than a validated PX4 tracking-model projection of measured state. Consequently a feasible nominal angular-rate prediction does not certify measured rate feasibility.

The nonblocking-poll RH circle check (`logs/archive/landing_history_20261019.tar.gz::ddp_nonblocking_reference_landing_rh/circle__zero__t0`, commit `86bd3f7`) landed with clean teardown. Across 95.18 controlled seconds and 551 accepted horizons, measured speed, tilt, glideslope, LOS and floor excess were zero. Angular-rate excess was still 0.00607 rad/s for 2.40060 seconds. Command median period was 9.990 ms and maximum gap 16.045 ms, versus 82.493 ms in the preceding case. This confirms the scheduling stall was removed; differing flight durations and engagement phases prevent attributing the entire angular-rate difference to that change. The physical constraint goal remains incomplete.

PX4 RH acceleration conversion (in progress): the x500 FCU logs show `MPC_ACC_DECOUPLE=1` and horizontal velocity derivative gain 0.2. Native PX4 forms its attitude axis from `[acc_internal.xy, g]` in decoupled mode and adds `-D_vel * acc_measured` in the velocity loop. Sending physical DDP acceleration directly therefore does not reproduce the DDP attitude, especially at thrust steps. Each full RH landing OCP now supplies its own MAVROS mapping: `a_ff.xy = g * b.xy / b.z + D_xy * a_ddp.xy` in decoupled mode, `a_ff = a_ddp + D * a_ddp` otherwise, and the corresponding vertical derivative term. Here `b = R(q) e_z` and `a_ddp = (T/m)b + gravity`. The core selects this mapping only for MAVROS; Crazyflie retains physical acceleration. These are nominal native-controller input equations, with no measured-error correction or second optimizer.

The shared parameter library owns the native FCU coefficients (`Px4TrackingParameters`; startup overrides `PX4_ACC_DECOUPLE`, `PX4_VEL_D_XY`, `PX4_VEL_D_Z`). They must match the FCU. The inverse reproduces the desired thrust axis at zero tracking error and zero velocity integrator; it does not yet model feedback transients, learned hover thrust, nonlinear collective tracking or initial integrator state. Physical validation is required. The saved PX4 demand audit aligns ROS/PX4 clocks by matching position/velocity and shows desired-axis rates above 0.09 rad/s over 0.1-second windows during RH control, despite nominal DDP-axis rates below 0.019 rad/s over those windows. Exploratory torque/gyro fits failed held-out prediction and were not used as OCP coefficients.

For this native-conversion change, all six core tests passed (6.12 seconds). The independent reconstruction test also passed with coupled acceleration and zero derivative gain. ROS1 and shared ROS2 planner builds passed sequentially. One ordinary RH circle SITL verification is next; no physical success is claimed from algebraic tests or builds.

The inverse-conversion circle case (`logs/archive/landing_history_20261019.tar.gz::ddp_px4_inverse_landing_rh/circle__zero__t0`, commit `008ce8a`) landed cleanly but failed physical constraints. Across 12.66 controlled seconds and 72 accepted horizons, measured tilt excess was 0.21303 degrees for 0.46784 seconds and angular-rate excess 0.02168 rad/s for 1.12815 seconds. Speed, floor, glideslope and LOS excess remained zero. Peak angular rate occurred 0.169 seconds after handoff, from an initial rate of 0.00129 rad/s; peak tilt occurred at 0.401 seconds. The nominal zero-error input conversion does not model PX4's pre-existing velocity integrator/tracking state. It is not a physical constraint fix by itself. Matching native controller state and closed-loop response inside the DDP prediction is the next required step.

Native-model data capture (`logs/archive/landing_history_20261019.tar.gz::ddp_native_model_identification_landing_mpc/circle__zero__t0`, commit `e6ef4c1`) used the saved MPC tuning and `PX4_PARAM_SDLOG_PROFILE=147` (131 plus native high-rate bit 16). It landed cleanly, but was a failed constraint test: angular-rate excess 0.01656 rad/s for 3.00380 seconds, tilt excess 0.03151 degrees, and glideslope excess 0.00824 m across 3.83 controlled seconds. Eight horizons were accepted. Direct commands were published unchanged every 10.000 ms median, 10.069 ms maximum; no position commands were interleaved during direct control. First controlled measured angular rate was already 0.03154 rad/s, despite the prior feasible-start proof.

The native ULog is copied into that case as `px4_inner_loop.ulg`; `px4_inner_loop.csv` contains 4 ms gyro, motor, torque and rate-request records plus 20 ms rate-integrator records. Source metadata, native allocator/motor parameters and diagnostic model validation are saved alongside it. The source-derived PX4 allocation matrix reproduces nonsaturated motor commands with RMS error 4.25e-5 normalized signal. A source-derived x500 rotor/inertia model explains the excited response substantially better than the sparse fits, but still needs gyro-filter/timing and held-out validation before use as a complete constraint prediction. The logged body-rate request to attitude-disable interval was 188 ms, versus 148 ms in an earlier case. A fixed instantaneous switch is therefore an incorrect assumption.

The former first-plan latency projection assumed a hover body-rate input while PX4 still ran its position/attitude cascade. That projection has been removed from the measured-state MPC path. A detailed motor/PID model has not been established as necessary and the experimental native model remains disabled by default. The measured-state first-input path is validated in core ordinary/CTCS moving-target tests and the ordinary ROS1 circle SITL case recorded below. Full physical constraint satisfaction and broader flight coverage remain separate outstanding work.

Measured-state MPC implementation check (2026-10-14): planner `411044e`, ALIPDDP `e9130217`. The core CTCS moving-target test completed 121 accepted updates in five seconds (maximum warm solve 27.5 ms); the ordinary moving-target test completed 162 updates (maximum warm solve 21.1 ms). Both verify exact first-input publication, measured initial state, and command hold without advancing through a solution. Controller lifecycle, RH checks and the single-job ROS1 build pass.

The sequential ROS1 circle check is saved in `logs/archive/landing_history_20261019.tar.gz::measured_state_ddp_mpc/circle__zero__t0/`. It ended `POLICY_EXPIRED / CLEAN`, with its bag finalized. Nine measured-state solves were accepted before subsequent constrained updates failed (first rejected residual 1.531). Warm solves took about 12–13 ms. All 116 published controls match the corresponding solution's node-zero input within the CSV's six-decimal precision (maximum difference 4.87e-7); publication median was 10.000 ms, maximum 11.430 ms, with no position commands interleaved and OFFBOARD maintained. Physical angular-rate and speed limits were exceeded; this is a failed flight validation, not a completed landing or constraint-satisfaction result. `control_audit.json` and `constraint_audit.json` contain the existing audit outputs. Full successful flight validation remains outstanding.

Measured-root recovery check (2026-10-14): planner `f99d23c`. Fixed measured point constraints at node zero previously made any angular-rate excursion an infeasible OCP; state constraints now apply to predicted nodes 1 through N, with input bounds at all control nodes. CTCS retains its first-interval density constraint. Regression tests preserve an out-of-bound measured rate exactly, verify feasible future states, and keep the feedback loop running after a 0.03 rad/s injected excursion. Both ordinary/CTCS moving-target core tests, controller tests and the sequential ROS1 build pass.

The same single ROS1 circle case now completes `LANDED / CLEAN` in `logs/landing/ros1/measured_state_ddp_mpc_recovery/circle__zero__t0/`. Its landing handoff follows 396 accepted DDP updates and zero rejected updates. Median update interval is 20.001 ms (about 50 Hz); warm solve median is 12.491 ms and maximum 25.086 ms. All 800 published inputs equal their solution's first input within six-decimal CSV precision (maximum error 4.99e-7). Publication median is 10.000 ms and maximum 10.685 ms, OFFBOARD is maintained, and no position commands interleave the direct-control interval. The finalized bag, CSVs, `mpc_execution_check.json` and existing audit outputs are saved together. The physical audit finds no speed, tilt, floor, glideslope or LOS excess, but a peak angular-rate excess of 0.002694 rad/s lasting 0.632 s. This establishes the measured-state MPC execution path and fast feedback cadence; it does not establish complete physical constraint satisfaction.

STC MPC verification (2026-10-16, unchanged controller): `logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/` uses the saved circle/original staging profile with `stc_landing_mpc` and eight warm iterations. The case completed `LANDED / CLEAN`, with 303 accepted solves and zero rejected solves. Cold solve: 332.707 ms; warm median/max: 19.456/39.259 ms. New first inputs average 37.845 updates/s, held for 29.990 ms median and 49.999 ms maximum. The main publisher runs at 100 Hz; the adapter's 10 Hz heartbeat adds duplicate publications, giving 110.013 Hz on the ROS command topic. PX4 gyro/rate PID/torque/motor execution is nominally 250 Hz, verified by live uORB status and 4 ms median ULog intervals (about 248 Hz mean in the selected direct-mode window). `IMU_GYRO_RATEMAX=800` is a cap, not this simulation's actual update rate. High-rate logger profile 147 was used only to measure the native loop.

Measured staging was active initially and last active at 2.570 s after activation. LOS activated below 1.2 m at 2.902 s, and the tight landing constraints below 0.8 m at 4.928 s. Activation and landing handoff work, but constraint satisfaction does not pass: measured maxima above limits were LOS 7.432 degrees, glideslope 0.04661 m, staging height 0.04220 m and angular rate 0.003933 rad/s. The current dense model audit also finds nominal future pointwise excesses (including tilt 3.007 degrees and LOS 11.377 degrees), with maximum local dynamics defect 0.003253. The issue therefore is not solely physical tracking. Reports and the copied native ULog are saved alongside the finalized bag and CSVs; no controller parameters or source behavior were changed during this verification.

Warm-start semantics: X/U are trajectory seeds and x0 is replaced by measured state. `updateProblem` retains scalar mu/muT and floors them at 1% of their initial values (0.01 with these defaults); inequality slacks S/ST and duals Y/YT are reconstructed from the new constraints, not copied from the previous solve. Scalar rho/rhoT and generic equality-multiplier storage are retained, but these two MPC OCPs have no explicit equality-constraint rows; FDDP handles dynamics gaps. A numerical initialization issue remains to investigate: the scalar slack floor is 0.01 while the CTCS row epsilon is 0.00016, so even a zero-density interval starts with primal residual 0.00984 before Newton updates. Eight iterations and the existing aggregate residual check have not established full CTCS feasibility. This observation identifies a candidate numerical cause, not a verified causal fix.

Saved STC MPC / historical RH plots (2026-10-16): the existing paper plotters generate controlled-interval state plots, world/relative 3D views and twin-view GIFs directly in each run folder. This is a visual comparison of saved circle runs: MPC staging height is 1.30 m and historical RH staging height is 1.55 m, with different source revisions, so the pair is not a matched experiment.

| Saved run | States | 3D | GIF |
|---|---|---|---|
| Measured-state STC MPC | [States](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/stc_flight_states.png) | [3D](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/stc_flight_3d.png) | [GIF](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/stc_flight.gif) |
| Historical STC RH replanning | [States](logs/landing/ros1/frame_regression_rh/circle__zero__t0/stc_flight_states.png) | [3D](logs/landing/ros1/frame_regression_rh/circle__zero__t0/stc_flight_3d.png) | [GIF](logs/landing/ros1/frame_regression_rh/circle__zero__t0/stc_flight.gif) |

The measured-state MPC box-flow diagram is saved with the MPC run: [PNG](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/mpc_loop.png), [editable SVG](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/mpc_loop.svg), [PDF](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/mpc_loop.pdf). Plotting code and planner behavior are unchanged.

General control architecture and saved-run comparison (2026-10-16): [comparison report](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/rh_mpc_comparison.md), [numerical metrics](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/rh_mpc_comparison.json). MPC's observed control interval was 8.33 s versus historical RH's 9.63 s, and warm solve medians were 19.46 ms versus 89.11 ms, but RH had smaller measured constraint violations and lower angular motion. These are unmatched saved runs with different staging heights, initial positions, iteration budgets, tracking interfaces and source revisions; the RH snapshot predates later transport timing fixes. They do not establish that either controller is generally superior.

The revised general architecture uses grouped problem/optimizer/controller blocks inspired by Figure 2 of the user-supplied Wang et al. paper, without case-specific frequencies or settings: [PNG](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/mpc_architecture_general.png), [SVG](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/mpc_architecture_general.svg), [PDF](logs/landing/ros1/measured_state_stc_mpc/circle__zero__t0/mpc_architecture_general.pdf). It depicts one DDP optimization problem, first-input application, target prediction outside the OCP, PX4 rate tracking/allocation, and measured-state feedback.

## STC MPC tuning: circle verification, 2026-10-16

The startup preset [tuning_stc_mpc_sitl.env](config/ros1/tuning_stc_mpc_sitl.env) produced two sequential circle flights with **zero measured violations against the original physical limits**, both `LANDED / CLEAN`. Controlled durations were 10.00 and 12.76 s; the repeated run had 222 accepted solves, zero rejected solves, 53.38 ms median warm solve and 17.58 Hz new first inputs. DDP still supplies the unchanged first input and PX4 tracks it. Physical scoring retains the original 0.8 m landing trigger, 0.019 rad/s rate limit, 0.75 m/s speed limit, 0.95 degree tilt, 15/30 degree cones and 1.3 m staging floor. Optimizer activation/margins are stricter and are saved separately from the scoring environment.

Source the preset **before starting the planner process**; it selects `STC_MPC_PRESET=mc` (OCP numbers, including staging 1.5 m and LOS trigger 1.2 m, in `ocp_stc_landing_mpc.hpp`) and the PX4 platform values. The preset is circle-verified; it is not a replacement for the ordinary or RH defaults.

The broader fix remains incomplete: straight and figure-eight validation failed to complete cleanly, and accepted predicted horizons still have tilt/rate excesses and dynamics gaps. Do not interpret the circle result as universal physical or predicted feasibility. The existing plotters were reused with the original physical bounds; no plotting design changed. See the [full tuning report and reproduction command](logs/landing/ros1/mpc_stc_speed_margin_repeat/circle__zero__t0/tuning_report.md), [state plot](logs/landing/ros1/mpc_stc_speed_margin_repeat/circle__zero__t0/stc_flight_states.png), [3D plot](logs/landing/ros1/mpc_stc_speed_margin_repeat/circle__zero__t0/stc_flight_3d.png), and [measured/predicted audit](logs/landing/ros1/mpc_stc_speed_margin_repeat/circle__zero__t0/constraint_audit_reference.json). All flights and builds were run sequentially; recordings and failed experiments remain under `logs/`.

Enforcement review: the historical circle runs accepted returned trajectories through a scaled aggregate residual gate (default maximum 1.0), without independently certifying OCP feasibility or dynamics consistency. The shared solver wrapper now checks the returned primal against every scalar, SOC and equality row, terminal rows and full dynamics, using the OCP solver tolerance. A feasible, suboptimal real-time result can pass; a returned trajectory or termination label alone cannot. This check certifies the transcribed OCP, including CTCS's finite interval allowance, rather than unrelaxed physical STCs. The solver's condensed primal-dual recursion, terminal residual sign, SOC matrix product and feasible-slack initialization have also been corrected and checked independently. The saved quadrotor reproduction still fails, and the historical 30-iteration RH smoke test now exposes an infeasible cold solve. Offline validation remains incomplete; installed ROS binaries and SITL have not been refreshed for these changes. Simply rejecting more updates while holding stale input is insufficient.

Log cleanup: [logs/README.md](logs/README.md) groups current evidence into `point_mass/` and `landing/`, with historical runs and recovery bundles under `archive/`. The 2026-10-19 cleanup archived 705 superseded files after full SHA-256 verification and reclaimed about 325 MiB net. Important point-mass results, landing baselines, circle results and representative straight/figure-eight failures remain directly accessible. The earlier verified intermediate-tuning archive is preserved under `archive/prior_tuning/`; restore instructions and the complete relocation manifest are in the index.


Offline warm-MPC recheck: the untested cold-rebuild experiment was removed
(`75c2155`); no such fallback is active. At 100 Hz command publication, the
ordinary constant-velocity model landing completed 381 accepted solves with
zero rejected solves and zero measured/predicted-node physical excess. The
STC version also landed with zero rejected solves, but retained physical
violations in both predictions and execution. This is a constraint-enforcement
failure, not a generally broken warm-start loop. In the current STC MPC, a
first input predicting 0.4336 rad/s against the 0.35 rad/s physical bound can
still satisfy the programmed CTCS integral allowance (about 4.7e-5 against
1.6e-3). These results concern the added MPC formulation, not the validated
standalone/internal RH reference. Complete three-motion physical enforcement
and future-interior validation remain outstanding; no new SITL was run.

`landing_mpc_ctcs` is the controlled, non-STC comparison for `landing_mpc`.
It uses the same complete OCP file, dynamics, costs, input bounds, initial
guess, measured-state warm-start loop and physical limits. The six path
constraints are replaced by one folded CTCS interval-density row through
the existing interface, with constant activation; there are no STC triggers
or additional nodal/terminal path constraints. Speed, rate and tilt use
the ordinary normalized cone residuals; geometric distances use a one-metre
reference. `SZMUK_CTCS_STEP_EPS` sets the raw integral budget. This variant
now divides the entire row by that budget, so solver feasibility tolerance
is relative to the allowance; `SZMUK_CTCS_Y_SCALE` remains a setting for the
STC formulations and is not needed here. A positive budget is a relaxation,
not a guarantee of zero pointwise excess.

The CTCS-only OCP now includes tangential curvature of the speed/rate norms
in addition to residual Gauss-Newton curvature, evaluated in one RK pass.
Its initial barrier is 1e-5; the objective, path limits and first-input
control delivery are unchanged. Finite differences validate the linear-rate
Hessian blocks to 1.7e-10 relative error. Existing direct-MPC checks pass.
At the inherited default budget 1.6e-3, the updated wall-clock circle landed
with 484 accepted solves and zero rejections; maximum measured rate excess
was 0.004231 rad/s, with other physical excesses zero. The initial comparison
before these corrections had 316 accepted/3 rejected and 0.006014 rad/s
rate excess. The larger allowance still permits physical excursions.

With a raw budget of 1e-6, the wall-clock constant-velocity model landing
completed 665 accepted solves, zero rejected solves, and zero measured or
dense predicted excess. The wall-clock circle still failed after three
accepted plans; its remaining execution/warm-update issue is unresolved.
Separately, synchronous fixed-step offline circle MPC at raw budget 1e-8
completed 100 successive measured-root solves, zero failures, and landed.
The plant was integrated in 1 ms steps under the first optimized input for
each 0.1 s MPC interval; simulation time did not advance during optimization.
Maximum measured rate excess was 1.003e-5 rad/s and dense predicted excess
was 1.004e-5 rad/s, below the 1e-4 audit tolerance; other physical excesses
were zero. The rate bound was exercised (peak 0.01901 against 0.019 rad/s).
Ordinary synchronous MPC also landed with zero measured excess, so this
case demonstrates tight CTCS feasibility, not superiority over nodal MPC.
The inherited default raw budget remains 1.6e-3; the tighter results require
`SZMUK_CTCS_STEP_EPS=1e-8` and used cold/warm iteration limits 120/24. Neither
the original shared solver nor the active STC formulation was changed.

The demanding matched near-cone benchmark is tracked in
[the authoritative gate record](.Codex/comando-constraint-satisfaction-control.md).
Run its production cold-start diagnostic after building `test_direct_mpc`:

```sh
env MPC_MAX_ITER=120 SZMUK_RH_N=40 SZMUK_SPD_STC_MAX=.25 \
  SZMUK_TILT_STC_DEG=.1 SZMUK_OMEGA_STC_MAX=.019 SZMUK_GS_CONE_DEG=15 \
  SZMUK_LOS_CONE_DEG=16 SZMUK_CTCS_STEP_EPS=1e-8 \
  ./test_direct_mpc --open-loop landing_mpc
# Then run the same command with landing_mpc_ctcs, sequentially.
```

The relative root is (.518,0,1.85), upright with zero relative velocity/rates.
For the constant target (.455 m/s at heading .6), this is the measured world
state with target position/velocity subtracted; target acceleration is zero.
The diagnostic reports the actual production seed, solver termination and
optimality, primal/dynamics residuals, and separate root/future-node/local
interval/full-propagation physical audits at 1 ms. It exits nonzero unless all
convergence, useful-progress and physical checks pass. Spatial LOS error is
included alongside its angular error. Both production formulations currently
fail this benchmark; later offline MPC, STC and SITL gates remain outstanding.
`AUDIT_INNER`, `AUDIT_STEPS`, `AUDIT_MU`, `AUDIT_INNER_TOL` and `AUDIT_COST_SCALE`
are isolated diagnostic overrides printed with each result; they do not alter
production settings. The common cost scale defaults to one. Core solve results
now expose `optimality_error` and `solver_converged` separately from primal
feasibility, without changing first-input delivery or solve acceptance.
