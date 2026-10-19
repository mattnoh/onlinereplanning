# Continuous-time CTCS versus nodal-only MPC results

All runs use the original Uzun2024 two-dimensional obstacle problem for
30 simulated seconds, with 15 measured-root replans, horizon8s, 17 nodes,
ZOH/FOH as labeled and K=4 intervals per update. Plant time is frozen during
optimization. Independent replay uses DOP853 at <=1ms, event-resolved
violations and refined peaks. No tracking controller or control clipping.

## What the comparison establishes

Both SCP CTCS and DDP CTCS maintain **zero physical obstacle penetration**
on the executed trajectory. Ordinary DDP node constraints can be satisfied
at every node while the same executed trajectory crosses an obstacle between
nodes. This happens with the **same .05 tightening** as CTCS; removing the
margin also fails, as a separately labeled comparison.

The successful original-problem DDP MPC does not fix the stressed landing
OCP. It rules out a blanket claim that per-interval folded CTCS or the
retained DDP MPC interface cannot work. Landing-specific densities/scales,
initialization, tail feasibility and terminal behavior still require isolation.

## Actual executed CTCS paths and retained logs

[Executed path overlay PNG](../../../logs/uzun2024_ctcs_mpc/ctcs_actual_paths_current/fig_ctcs_actual_paths.png)
([PDF](../../../logs/uzun2024_ctcs_mpc/ctcs_actual_paths_current/fig_ctcs_actual_paths.pdf)).
Blue: SCP CTCS ZOH10 (P5). Green: latest verified DDP CTCS M50/shared/inner100
(P17). Dashed black: reference. Grey ellipses: physical obstacles; dashed
outlines: .05 tightening. Hollow markers are .5s executed nodes. Curves are
actual simulated plant paths from saved independent1ms replay, not horizon
predictions or physical-flight measurements. Both have zero physical violation;
mean tracking is.403897m for SCP and.250055m for DDP. The lower panel shows
physical maxg over allfive obstacles, clipped below-.15; exactseries remain
in the exportedCSV/NPZ. SCP integral-budget residual remains separately reported.

Key recent run files/sourcehashes were checked. Canonical source and this report
are committed; raw `logs/` is Gitignored. A checksummed local archive under
`logs/archives/` captures available paper logs (including failed candidates),
archived landing gate logs, canonical source/progress record and current solver
binary/build flags. Archive index/checksum lists exact contents. This is local
retention, not off-machine backup; older landing `/tmp` traces noted below are
still missing. The archive preserves evidence, not a declaration that every
included experiment passed.

## Interior point and conic solvers

Clarabel0.11.1, used by this OpenSCvx reproduction, is itself an interior-point
conic solver (see its installed package METADATA). Conic describes the problem
class; interior point describes a solution algorithm. The paper used ECOS or
MOSEK; our configured numerical backend is CLARABEL.

DDP applies its structured primal-dual inequality method to the nonlinear OCP,
using local quadratic models in its backward pass. Its CTCS row directly
evaluates the folded interval integral. SCP instead constructs a convex
subproblem from linearized augmented dynamics; its `y[k+1]-y[k]<=epsilon`
rows are affine constraints solved by the conic backend. The current SCP
subproblem can satisfy those rows while the original nonlinear accumulator
dynamics retain defects. This accounts for why a conic `optimal` status alone
does not prove feasibility of the original CTCS problem after finitely many
SCP iterations. The entire SCP problem remains nonlinear despite convex inner
subproblems. DDP also needs accurate integration and actual nonlinear primal
checks; the rejected20-step experiment demonstrates that solver feasibility
alone can miss an integral-budget excess.

At exact dynamics/integration and convergence, the two CTCS representations
have the same per-interval meaning. Their numerical conditioning, curvature,
step acceptance and stopping differ; neither algorithm label alone establishes
continuous-time physical safety or general performance superiority.

## Matched SCP/DDP × CTCS/nodal comparison

The missing SCP nodal case is implemented and executed (P10b). All four
rows below use **ZOH, delta=.05, 17 nodes, .5s intervals, horizon8s, K4,
30s duration, identical dynamics/reference/root/sensing/input limits**.
SCP uses 10 subproblems per solve in both modes; DDP has a 300-iteration cap.
No control clipping or obstacle integral budget is used in either nodal mode.
SCP nodal mode removes its obstacle accumulator and uses ordinary nonconvex
rows at every node, including terminal. Its nodal slack L1 penalty is1e4,
matching its dynamics-defect exact-penalty weight. The OpenSCvx library's
identically inactive clock accumulator is unchanged.

| Solver | Constraint enforcement | Mean tracking [m] | Max physical g | Physical violation [s] | Max actual node g | Median solve after first [ms] |
|---|---|---:|---:|---:|---:|---:|
| SCP | CTCS | 0.403897 | -0.044138 | 0 | -0.044177 | 169.795 |
| SCP | Nodal | 0.634589 | +0.789689 | 2.623947 | -0.049998 | 207.791 |
| DDP | CTCS | 0.250058 | -0.045259 | 0 | -0.045901 | 416.259 |
| DDP | Nodal | 0.215346 | +0.607037 | 2.198156 | -0.050000 | 134.705 |

The executed physical endpoints remain safe in **both nodal runs**, while
independent dense replay finds7 violating intervals for SCP and5 for DDP.
Both CTCS runs have zero observed physical penetration in this benchmark.
This is the expected enforcement-location distinction on these trajectories,
not a universal safety guarantee from finite sampling or a positive budget.

[2×2 figure PNG](../../../logs/uzun2024_ctcs_mpc/comparison_2x2/fig_2x2.png)
([PDF](../../../logs/uzun2024_ctcs_mpc/comparison_2x2/fig_2x2.pdf)),
[performance PNG](../../../logs/uzun2024_ctcs_mpc/comparison_2x2/fig_performance.png)
([PDF](../../../logs/uzun2024_ctcs_mpc/comparison_2x2/fig_performance.pdf)),
[node/dense close-ups](../../../logs/uzun2024_ctcs_mpc/comparison_2x2/fig_intersample.png),
[full statistics](../../../logs/uzun2024_ctcs_mpc/comparison_2x2/comparison_stats.json).

The 2×2 and performance figure contain the matched four cases. The overall
comparison and close-up also preserve the separately labeled untightened DDP
supplement. FOH cases remain supplementary statistics, not matched ZOH panels.

### Feasibility and convergence limits

This is a matched **physical problem and execution** comparison, not a claim
that all four nonlinear programs converged to equal tolerance. Fixed10 SCP
retains nonlinear residuals. In P10b the largest returned tightened node row is
+2.396e-6 (strict1e-6 row check fails), all actual physical node endpoints have
g<=-.0499976, root defect<=4.45e-15, and max executed prediction/plant error
1.658e-4. The worst future-interval physical dynamics defect is .01208 and
cost-integrator defect1.188. The strict full-horizon convergence/defect gate
therefore remains failed. These residuals are not hidden by conic subproblems
reporting `optimal`; all15 MPC solves completed10 SCP subproblems each. Nevertheless,
the executed endpoints are safe and the executed dense intervals collide,
which is measured directly without relying on nonlinear convergence.

SCP CTCS remains physically safe while exceeding the assumed integral budget
(max executed I/epsilon1.26407). DDP CTCS has15/15 `Optimal Solution` and meets
executed budgets. Matched DDP nodal has12/15 `Optimal Solution`, three capped
solves, with nodal primal/dynamics feasibility throughout. Its strongest
intersample witness comes from a converged solve, as documented below.

The first SCP nodal attempt P10 had free node slack because OpenSCvx defaults
`lam_vb` to zero. It failed nodal enforcement and is **excluded**. P10b adds the
required exact penalty; invalid figures are archived separately. The final
CTCS code regression P11b reproduces saved P5 solve0 X/U exactly (zero maximum
difference). Its short2s startup tracking summary is not a full-run verdict.

### Timing interpretation

| Case | First solve [ms] | Later median [ms] | Later p95 [ms] | Later max [ms] | Iterations |
|---|---:|---:|---:|---:|---|
| SCP CTCS | 156.911 | 169.795 | 259.900 | 292.335 | fixed10 |
| SCP nodal | 162.230 | 207.791 | 249.520 | 262.549 | fixed10 |
| DDP CTCS | 119.138 | 416.259 | 906.113 | 946.691 | 8–79; median36 |
| DDP nodal | 160.399 | 134.705 | 2786.825 | 3622.850 | 8–301; median22 |

These are saved implementation measurements, **not an algorithm speed ranking**.
SCP uses Python/JAX/OpenSCvx/CLARABEL and fixed iterations; its timer covers
reset+solve, excluding initialization/JIT (P5 2.85s) and independent audits.
DDP uses the C++ solver timer and a different stopping rule; process/IPC/I/O
wall timing is saved separately. Subsequent statistics exclude solve0, with
only14 samples: p95 is descriptive rather than a tail guarantee. No actual
latency is imposed on the plant in these synchronous experiments.

DDP CTCS has lower observed mean tracking error than SCP CTCS in these runs;
the iteration counts and residuals differ, so that is not evidence of intrinsic
optimizer superiority. Unsafe nodal tracking scores do not count as a safety
performance improvement. These original-obstacle results do not validate the
stressed landing MPC, whose repair remains paused at the user's request.

## Verified intermediate stopping adjustment

Optional addition to the fast profile:
`--solver-arg=--inner-tol --solver-arg=100` (reference inner tolerance10).
This advances intermediate barrier subproblems sooner. It **does not change**
the final/global1e-6 tolerance, physical/input bounds, delta.05 or epsilon1e-6.
The existing solver still requires final stationarity/barrier/primal/dynamics
checks to pass. No shared solver source change was needed.

P17 completed30s with15/15 `Optimal Solution`, returned modeled primal0,
max optimality8.94937e-7, zero independently audited physical violation,
max executed integral9.99965105e-7 and mean tracking.250055296m. Compared
with the prior M50/shared campaign, total iterations fell593→530 (10.62%).

P18 compares inner10/100 on **identical saved roots and seeds**, with separate
retained solvers and alternating sequential solve order. Both profiles pass
convergence and independent fine predicted constraints. Candidate trajectories
and iteration counts exactly reproduce P17. Baseline differs (maximum saved
trajectory-column difference.000708), so their plant audits are not transferred
by equivalence; the actual candidate plant audit comes from full campaignP17.
Paired iteration totals are602→530; these differ from separate-campaign totals
because the paired roots/seeds are fixed to P17.

| Paired stopping setting | Warm median [ms] | Warm p95 [ms] | Warm max [ms] | Total solver CPU [s] |
|---|---:|---:|---:|---:|
| Inner tolerance10 | 164.888 | 441.165 | 577.923 | 2.584386 |
| Inner tolerance100 | 139.751 | 417.142 | 573.579 | 2.247520 |

Observed median latency is15.24% lower (1.18× speedup); total solver CPU is
13.03% lower. Worst-case latency barely improves. First solve/audits/IPC are
excluded from warm statistics. Host load differs from previous timing tables;
compare the two rows within this experiment, not139ms directly against earlier
224ms or the historical SCP170ms.

[Stopping comparison data](../../../logs/uzun2024_ctcs_mpc/ddp_stopping_comparison.json),
[full MPC audit](../../../logs/uzun2024_ctcs_mpc/P17_ddp_ctcs_inner100/summary.json),
[paired diagnostics](../../../logs/uzun2024_ctcs_mpc/P18_inner_stopping_paired/summary.json).

```bash
python ddp.py --reuse-solver --substeps 50 --flow-cache shared --solver-arg=--inner-tol --solver-arg=100 --output-dir NEW_FAST_INNER100
python ddp.py --replay-saved ../../../logs/uzun2024_ctcs_mpc/P17_ddp_ctcs_inner100 --replay-inner 100 --output-dir NEW_STOPPING_PAIR
```

## What still fails in the stressed landing OCP

The successful obstacle example establishes that folded interval CTCS and
measured-root retained DDP MPC can work. It does **not** establish convergence
for the landing OCP or rule out solver robustness problems at landing scales.
No matched SCP landing reproduction has yet been validated.

The retained [stressed E1 log](../../../logs/constraint_gate_20261019/comando-gate-e1-ctcs.log)
starts with a feasible production seed but returns a physically unsafe plan:
speed excess.29641648m/s, tilt excess1.18789182deg, spatial LOS excess.06767562m.
Model dynamics gaps are zero and dense interval propagation discrepancy is
1.96e-8. Thus these are substantial path-constraint failures, not an optimality
label or plant tracking error. The [E3 iteration log](../../../logs/constraint_gate_20261019/comando-gate-e3-ctcs.log)
shows the first full step taking normalized residual from zero to1.75e10.

Relevant mechanisms requiring isolated repair tests:

- The squared positive-part CTCS density has zero value/gradient while every
  consequence is inside its limit. This also occurs in the paper model; it is
  not intrinsically unique to landing. In landing, a strong terminal position/
  velocity objective (weights2500/900), .1deg tilt bound and budget1e-8 create a
  much steeper transition once a trial step crosses the physical boundary.
- The earlier progress record reports a shifted warm seed whose new tail had
  CTCS residual1.145e6 despite2e-9 dynamics gap: the old trajectory ended near
  the tilt cap with outward rate, so holding the final command did not yield a
  feasible extension. A safe finite horizon alone does not guarantee a safe
  appended interval even with exact execution of the old prefix. That record's
  original `/tmp` raw logs are unavailable here; a fresh persistent warm trace
  is still needed before claiming a current causal reproduction/fix.
- Paper density includes explicit delta.05 tightening. Current landing density
  uses physical consequences directly, without corresponding per-limit CTCS
  tightening. Its conservative cold seed does not constrain later iterates.
  Appropriate margins must respect the different physical units and initial
  clearance; copying the obstacle's dimensionless.05 would not be justified.

These explain where to isolate failures, but no verified landing repair is
claimed. Earlier stopping can accept a less stationary **feasible** iterate;
it cannot make an infeasible returned plan acceptable. Landing repair remains
paused, and no landing optimization experiment was run during this task.

## Verified faster DDP CTCS profile

Use **`--substeps 50 --flow-cache shared`**. The horizon remains8s with17
nodes, dt=.5s, ZOH and K4. Only RK4 integration inside each interval is reduced
from100 to50 substeps (5ms to10ms); the MPC/node grid is unchanged. Physical
limits, obstacle tightening.05, epsilon1e-6, objective and solver tolerance1e-6
remain fixed. No shared solver algorithm or CTCS-row implementation changed.
The reference defaults remain100 substeps and legacy cache for reproducibility.

The 50-substep campaign P13 completed all15 MPC solves with `Optimal Solution`,
zero physical violation in the independent1ms replay, max executed integral
9.999053652e-7, max future-interval independent integral9.999053746e-7,
tracking.250056094m (100-step reference.250057624m), and max executed
prediction/plant error1.75923e-8. The max independently audited interval state
error is1.22306e-8; model/root defects are zero. Integral quadrature discrepancy
can reach4.50e-10; exact quadrature equality is not claimed, but the independently
evaluated integral remains within the unchanged budget.

The local OCP now shares one `FlowCache` per stage between dynamics and cost.
Previously one dynamics cache thrashed across all stages, and cost repeated its
flow/sensitivity calculation. Shared cache uses exactly the same RK4 formulas;
the existing folded CTCS interface/integration remains intact. Its ownership
is restricted to one stage of an immutable horizon problem.

To isolate this from host load, P15/P16 replay each saved 15-request measured-
root/shift/tail sequence in two independent retained solvers, alternating
legacy/shared order and solving sequentially. Every returned trajectory column
and iteration/termination matches the original audited run **exactly (max
absolute difference0)** at both100 and50 substeps. Original independent plant
and integral audits therefore transfer by full trajectory parity. These are
saved-MPC request replays, not a second plant integration or a latency-aware MPC
validation. Process CPU time is recorded separately from wall time.

| RK4 substeps | Cache | Later median [ms] | Later p95 [ms] | Later max [ms] | Total solver CPU [s] |
|---:|---|---:|---:|---:|---:|
| 100 | legacy | 825.985 | 1852.783 | 1962.532 | 11.449 |
| 100 | shared | 617.586 | 1418.526 | 1427.539 | 8.263 |
| 50 | legacy | 319.005 | 952.831 | 1095.152 | 5.423 |
| 50 | shared | **224.404** | **842.254** | 1214.989 | **4.586** |

The combined profile is3.68× faster by measured warm median and2.50× by total
solver CPU time in these runs. Cache alone improves median1.34× at100 and1.42×
at50. The 50-step shared maximum is worse than its legacy maximum, so no
per-solve speed guarantee is implied. Background host load rose above18 during
this task. These paired replay times must not be directly mixed with earlier
SCP/DDP timing rows collected under another load. First solve is excluded from
median/p95/max; audits, initialization and IPC are outside the solver timer.
This improvement applies to the paper obstacle benchmark; landing remains paused.

**Rejected candidate:**20 RK4 substeps completed15/15 Optimal and zero physical
violation, but independent executed integral reached1.006926698e-6 (.6927%
over the fixed budget). It is not recommended. No budget/tightening was relaxed
to call it a pass. Initial-mu tuning was considered but not run; it is not part
of the retained profile.

[Speed statistics JSON](../../../logs/uzun2024_ctcs_mpc/ddp_speed_comparison.json)
([CSV](../../../logs/uzun2024_ctcs_mpc/ddp_speed_comparison.csv));
[100-step parity/timing](../../../logs/uzun2024_ctcs_mpc/P15_cache_replay_rk100/summary.json),
[50-step parity/timing](../../../logs/uzun2024_ctcs_mpc/P16_cache_replay_rk50/summary.json).
Source snapshots/hashes, per-solve requests, returned plans, diagnostics and
both complete dense-audited campaigns are saved with the runs.

```bash
python ddp.py --constraint-mode ctcs --reuse-solver --substeps 50 --flow-cache shared --output-dir NEW_FAST_CTCS
python ddp.py --replay-saved ../../../logs/uzun2024_ctcs_mpc/P7_ddp_zoh_reuse --output-dir NEW_CACHE100_PAIR
python ddp.py --replay-saved ../../../logs/uzun2024_ctcs_mpc/P13_ddp_ctcs_rk50 --output-dir NEW_CACHE50_PAIR
```

## Executed results

Physical constraint is `g_i=1-norm(S*(r-c_i))<=0`, over **all five obstacles**.
Negative maxima indicate clearance. These dimensionless residuals are not
metres; geometric distance is saved separately in the raw statistics.

| Case | Mean tracking [m] | Max physical g | Violation [s] | Max executed node g | Max I/epsilon |
|---|---:|---:|---:|---:|---:|
| SCP CTCS, FOH, 3 it. | 0.746276 | -0.031167 | 0.000000 | -0.031712217 | 22.2613 |
| SCP CTCS, FOH, 10 it. | 0.400330 | -0.045364 | 0.000000 | -0.046035774 | 1.26985 |
| SCP CTCS, ZOH, 10 it. | 0.403897 | -0.044138 | 0.000000 | -0.044176870 | 1.26407 |
| DDP CTCS, ZOH | 0.250058 | -0.045259 | 0.000000 | -0.045901126 | 0.999933 |
| DDP nodal only, $g+0.05\leq 0$ | 0.215346 | +0.607037 | 2.198156 | -0.050000001 | 98729.5 |
| DDP nodal only, $g\leq 0$ (margin 0, not matched) | 0.209086 | +0.607281 | 2.429265 | -0.000000398 | 84994 |

CTCS uses tightened density `sum_sensed max(0,g_i+.05)^2` and interval budget
1e-6. The stage-only rows enforce `g_i+delta<=0` at stages **and terminal**;
no interval budget is enforced or used for their primal acceptance. `I/epsilon`
is a post-run diagnostic only for stage cases. The zero-margin row changes
both the margin and its diagnostic integral and is not a matched CTCS row.

SCP three-iteration and ten-iteration cases maintain physical clearance but
exceed the strict assumed integral budget. Their raw summary status remains
`fail`; no exact integral feasibility or perfect convergence is claimed.
DDP CTCS meets executed integral budgets. Iteration budgets differ between
solvers, so tracking numbers are not an optimizer superiority claim.

## A converged, node-feasible intersample counterexample

In the same-tightening stage run P8, solve6 at t=12s reports `Optimal Solution`.
Its executed interval [12,12.5]s has:

- Start g=-0.0500004834; end g=-0.0500004075 (both satisfy tightened nodes).
- Refined peak g=+0.6070366852 at t=12.2523606767s, inside obstacle2.
- Violation occupies .4722s of this half-second interval.
- Whole run: 2.1981558522s of physical violation; maximum geometric penetration
  .381378m, executed prediction/plant discrepancy <=7.11e-12.

All15 P8 solves have feasible modeled node rows and zero model dynamics gaps;
12 report `Optimal Solution`, three reach `Max Iteration`. That caveat is
preserved. The strongest physical counterexample above is from a converged
solve, so the conclusion does not depend on the unfinished solves.

The usual untightened nodal comparator P9 converges on **all15 solves**,
keeps all executed nodes physically feasible, yet has a dense physical peak
+0.6072811035 and 2.4292654686s of violation. Both runs fail the physical audit.

## Figures and raw evidence

[Overall comparison PNG](../../../logs/uzun2024_ctcs_mpc/comparison_plots/fig_comparison.png)
([PDF](../../../logs/uzun2024_ctcs_mpc/comparison_plots/fig_comparison.pdf)):
executed trajectories, physical max g versus time with node markers, tracking
error and audited interval integral. Values below -.3 in the physical-g panel
are clipped and labeled; exact values remain in the exported CSV.

[Intersample close-up PNG](../../../logs/uzun2024_ctcs_mpc/comparison_plots/fig_intersample.png)
([PDF](../../../logs/uzun2024_ctcs_mpc/comparison_plots/fig_intersample.pdf)):
worst intervals, node endpoints, dense physical violations and planar geometry.
The close-up makes the node-feasible/dense-infeasible distinction explicit.

[Statistics JSON](../../../logs/uzun2024_ctcs_mpc/comparison_plots/comparison_stats.json),
[statistics CSV](../../../logs/uzun2024_ctcs_mpc/comparison_plots/comparison_stats.csv),
[interval-by-interval evidence](../../../logs/uzun2024_ctcs_mpc/comparison_plots/executed_intervals_all.csv).
Exact plotted dense/node/close-up series, raw solves, seeds, controls, configs,
source snapshots and hashes are under the same logs directory. The plotting
code reads saved evidence; it does not resolve or alter trajectories.

Physical/data checks: saved residuals match independently recomputed g exactly;
duplicate dense junctions are removed after verifying state continuity. Input
norm<=5 in every campaign. New nodal row derivative error is1.93e-10. A CTCS
regression after adding stage mode reproduces the saved initial X/U exactly;
its two-second tracking-only summary is not used as a complete-run pass.

## Exact commands

From `support/test/uzun2024_ctcs_mpc`, using the openscvx Python environment:

```bash
bash build_ddp.sh
python scp.py --constraint-mode stage --hold ZOH --iterations 10 --cost-scale 100 --delta .05 --output-dir NEW_P10b
python ddp.py --constraint-mode stage --delta .05 --reuse-solver --output-dir NEW_P8
python ddp.py --constraint-mode stage --delta 0 --reuse-solver --output-dir NEW_P9
python plot_comparison.py --output-dir ../../../logs/uzun2024_ctcs_mpc/comparison_2x2
```

The stage cases change the enforcement location, then the margin, explicitly.
No obstacles or grid were changed to manufacture the failure. See README.md
for the paper parameters, original CTCS commands and unreported numerical
settings that are explicitly assumptions. No shared solver algorithm changed.
