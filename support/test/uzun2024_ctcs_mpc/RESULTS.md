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
python ddp.py --constraint-mode stage --delta .05 --reuse-solver --output-dir NEW_P8
python ddp.py --constraint-mode stage --delta 0 --reuse-solver --output-dir NEW_P9
python plot_comparison.py --output-dir ../../../logs/uzun2024_ctcs_mpc/comparison_plots
```

The stage cases change the enforcement location, then the margin, explicitly.
No obstacles or grid were changed to manufacture the failure. See README.md
for the paper parameters, original CTCS commands and unreported numerical
settings that are explicitly assumptions. No shared solver algorithm changed.
