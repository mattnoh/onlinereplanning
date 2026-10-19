# Local point-mass open-loop 2 × 2 comparison

## Updated plots at the required 1e-6 dynamics threshold

[Free-time PNG](../../../logs/point_mass_open_loop_2x2_gap1e6_20261019_v2/point_mass_2x2_free.png)
([PDF](../../../logs/point_mass_open_loop_2x2_gap1e6_20261019_v2/point_mass_2x2_free.pdf));
[fixed-time PNG](../../../logs/point_mass_open_loop_2x2_gap1e6_20261019_v2/point_mass_2x2_fixed.png)
([PDF](../../../logs/point_mass_open_loop_2x2_gap1e6_20261019_v2/point_mass_2x2_fixed.pdf)).

Regenerated with `--ox-gap-tol 1e-6 --plot-snapshot NEW_DIRECTORY`, keeping the
original local figure style. Both figures now print the selection threshold.
The mesh panel explicitly reports the worse of obstacle and glide clearance;
the highlighted policy table reports obstacle clearance. Consequently stage
heatmap numbers can differ from an obstacle-only summary table.

The completed-results snapshot replays 40 saved policies. All five selected
free-time SCP CTCS meshes N=100,40,20,10,6 pass the independent physical,
integral-budget and base checks. A newly completed N=100 CTCS run is included
(clearance about 4.94 mm, zero tightened violation integral, dynamics gap
6.76e-9). The separate live N=100 sweep remains ongoing; the stage arm has
no accepted saved run in this snapshot. No optimizer was launched here.
Earlier plots remain intact. Both final figures were visually inspected;
all 394 inventoried files were SHA256-verified. The versioned mirror and local
compare_fair.py are identical. A separate local four_arm_review_gap1e6_20261019
symlink points to the corrected snapshot.


## Current interpretation: reconcile Claude's table (2026-10-19)

**Free-time SCP CTCS works for the tested N=40,20,10,6 cases under the original
1e-6 dynamics-residual requirement.** Claude's quoted table exactly matches
`fair_gap1e-06/fair_tables.md`; the earlier figure in this report uses the
stricter `fair/` selector's 1e-8 physical dynamics-gap threshold. Its grey
"no accepted setting" cells are strict-accuracy classifications, not evidence
that no physically safe, integral-feasible free-time policy exists. Presenting
them as a general failure of free-time SCP was misleading and is corrected here.

Independently regraded the four selected returned control policies and verified
their saved replay arrays, physical constraints, terminal checks, native and
independent budgets, and existing 0.1-epsilon integral-consistency check:

| N | Maximum physical dynamics gap | Obstacle clearance | Independent I/epsilon | Physical and integral checks |
|---|---:|---:|---:|---|
| 40 | 2.08898e-7 | 4.61723 mm | 0.0791923 | pass |
| 20 | 2.81490e-7 | 0.96973 mm | 0.911619 | pass |
| 10 | 1.40805e-7 | 2.00181 mm | 0.939702 | pass |
| 6 | 7.86512e-7 | 1.99575 mm | 0.990561 | pass |

All four gaps are below 1e-6 and above 1e-8. Their physical limits, tightening
and epsilon are the saved values; no safety budget was enlarged in this review.
The N=20,10,6 selected runs already existed in the earlier frozen pool. N=40
completed later. Changing the selection gate changes which run is selected,
which also explains the different stage-only penetration numbers between the
two tables. The quoted table labels DDP selections at 1e-8 and SCP at 1e-6;
this must remain explicit for precision/performance comparisons.

Raw tables, selected settings/native arrays/replay arrays, grading source,
reconciliation.json and a verified 28-file SHA256 inventory are saved under
`logs/point_mass_scp_threshold_reconciliation_20261019/`. No optimizer was run
and no solver settings were changed. At inspection, the separate live free
N=100 campaign was still running. The older snapshot and its stricter audit
below remain historical evidence, with their original source and hashes intact.


Reviewed the existing benchmark at
`/home/lab/projects/solver/benchmark/openscvx_point_mass_ctcs` and its DDP
reference under `/home/lab/projects/solver/DDP/ALIPDDP-ctstc-al`.
Extended the existing local `compare_fair.py` with `--plot-snapshot`; the
versioned `point_mass_compare_fair.py` here is its byte-identical mirror.
Original fixed/free figures and optimization libraries are unchanged.
No new optimizer runs were launched; other local SCP sweeps were already
running and were left alone. The comparison freezes completed saved policies.

## Saved evidence

Evidence directory: `logs/point_mass_open_loop_2x2_20261019_v3/`.
It contains fixed/free PNG and PDF figures, `comparison.csv`, `audit.json`,
per-policy 1 ms dense CSVs, exact interval extrema and obstacle violation
durations, raw selected inputs/settings/arrays, selection logs, source copies
and `sha256_inventory.json`. All 324 inventoried files were hash-verified.
The first two plotting attempts failed on a missing optional pandas import and
NumPy bool JSON serialization, respectively. Their console logs/partial
outputs are retained under the corresponding unsuffixed and `_v2` directories;
`_v3` is the completed result. These were diagnostic/plotting failures, not
optimization failures.

The original plot definitions supply serif fonts, palette and heatmap colors.
The extension retains 3D sphere/cone geometry, clearance traces with shooting
node markers and a contact inset, the mesh heatmap, and the quality/time table.
Rows are DDP stage, DDP CTCS, SCP stage and SCP CTCS; original figures are not
replaced. A local `four_arm_review_20261019` symlink points to this snapshot.

## Fixed time: same N=6 grid, tf=4 s

| Method | Worst obstacle clearance | Time inside obstacle | Independent interval budget |
|---|---:|---:|---|
| DDP stage | -8.82835 mm | 0.112694 s | not part of this formulation |
| DDP CTCS | +3.17208 mm | 0 s | met, I/eps=0 |
| SCP stage | -23.53530 mm | 0.169058 s | not part of this formulation |
| SCP CTCS | +0.73650 mm | 0 s | met, I/eps=0.96286247 |

Both stage policies satisfy their obstacle node rows. Their largest physical
shooting gaps are 2.24e-11 (DDP, rounded CSV) and 4.65e-10 (SCP), so the observed
penetration is between nodes, rather than an appreciable root/replay mismatch.
Both CTCS policies clear the sphere and the glide slope, meet terminal/input
checks and the independently evaluated interval budgets. Both use epsilon
1e-11, with saved per-setting tightening: DDP obstacle/glide
0.000857250077/0.000962230677 m; SCP
0.000810960266/0.000910272121 m. These are different speed-bound refinements,
not identical numerical margins. Native stopping flags are preserved; the
DDP rows end at `Outer Max/Min`, not a new claim of proven optimality.

## Free time and the full saved mesh sweep

The analogous N=10 picture is saved, including a clearly labeled diagnostic
SCP CTCS policy. That policy is physically safe, has I/eps=0.9996561763 and a
shooting gap 1.41466533e-8. It narrowly misses the existing 1e-8 gap gate, so it
is not promoted into an accepted result. The saved completed crossnode SCP
CTCS pool has no fully accepted free-time cell under the unchanged selector.
That is a result about this frozen pool and strict gate, not a statement that
SCP cannot solve free-time CTCS. The older interval-gated native OpenSCvx
successful solution remains separately documented in the local RESULTS.md.

The new common replay also finds the saved DDP CTCS free-time N=6 policy has
I=3.60518082e-10 against epsilon=1e-11 (36.05 times the budget). It physically
clears the sphere and glide slope, but fails the matched integral audit. The
heatmap labels this budget failure rather than inheriting the old selected
row's zero-integral report. No solver defect or cause is inferred without a
separate reproduction of that historical run.

## What this comparison establishes

All 34 displayed/mesh policies were replayed from the common physical initial
state using exact constant-acceleration flow. Obstacle extrema and violation
durations use polynomial roots; glide extrema use their analytic stationary
equations. Integral quadrature splits at extrema and zero crossings. Dense
plots use spacing <=1 ms and are bounded by the exact extrema. All saved
penetration grades were reproduced within 1e-7 m, and SCP replay states matched
their saved independently propagated arrays within 1e-10.

The physical OCP, objective, controls and grid match across the four arms at a
given mesh. Saved tuning, CTCS margins and budgets can vary between cells.
Review found an incorrect same-initial-guess claim in the old local comparison
notes: saved DDP `consistent_guess=1` uses line velocities away from the root,
while SCP uses zero velocities away from the root. The local note is corrected.
Consequently these are saved-policy enforcement comparisons, not controlled
optimization-performance trials. DDP times are historical serial reruns; SCP
times are from loaded sweeps. Figures explicitly exclude a runtime ranking and
separate initialization. The existing selector is not an optimality proof.
Landing optimization remains paused and unchanged.

## Reproduce from the available local results

Use a new output directory: snapshots refuse to overwrite existing evidence.

```bash
/home/lab/miniforge3/envs/openscvx/bin/python \
  support/test/uzun2024_ctcs_mpc/point_mass_compare_fair.py \
  --ox-gap-tol 1e-6 --plot-snapshot /absolute/path/to/NEW_DIRECTORY
```

This command only selects, independently audits and plots saved policies.
`COMANDO_POINT_MASS_BENCHMARK` can specify another local benchmark root. The
snapshot includes the exact executed plotting source; its final portable
lookup extension is also saved separately. Source of the original historical
DDP campaign is not reconstructed from a current source revision.


## Follow-up: why free-time SCP was labeled unaccepted

This label does not mean the displayed CTCS path penetrates the obstacle.
N=10 is physically safe (4.21446 mm obstacle clearance), independently budget
feasible (I/eps=0.9996561763) and natively converged. Two strict comparisons
reject it: the 1.41466533e-8 physical gap exceeds 1e-8, and its native interval
budget is above epsilon by 1.31724e-18. The previous gap-only explanation was
incomplete: the selector also requires native I-eps<=0 with no numerical
allowance. That second residual is 1.31724e-9 of epsilon, while the independent
physical replay remains below the actual budget. No criterion was changed.

Other free-time CTCS diagnostics fail different gates. N=20 has a small physical
gap (3.13689e-9), a safe path and I/eps=0.4275082, but the native-vs-independent
maximum-integral difference is 0.5724911 epsilon, exceeding the existing 0.1
epsilon consistency gate. N=6 is safe and budget feasible but has a 5.20176e-8
physical gap. Thus the whole pool is not explained solely by a tiny strict-gap
miss; some candidates have substantive augmented-integral inconsistency, and
others fail convergence or actual replay budgets.

Free duration makes the physical discrete map nonlinear in the decision
variables (v_next=v+Theta*(a-g), p_next=p+Theta*v+0.5*Theta^2*(a-g)); fixed
Theta makes that physical map affine in x,a. CTCS integration also depends on
Theta. These couplings explain why the free-time transcription requires more
nonlinear refinement, but do not establish the cause of every failed candidate.
The pinned OpenSCvx stopping test uses scaled step/linearized virtual-control/
buffer metrics, rather than independently replayed physical and CTCS defects.
So native convergence and the stricter independent audit can disagree.

A read-only rescore of the frozen pool finds 25 settings would pass the other
unchanged gates with a 1e-6 physical-gap threshold instead of 1e-8 (5/11/9 at
N=6/10/20). This is diagnostic sensitivity only; no production threshold,
budget or tightening was relaxed. Detailed counts are saved separately as
logs/point_mass_free_scp_review_20261019.json. The correct next diagnostic is
nonlinear-residual-aware stopping and accumulator conditioning, with physical
limits, tightening and epsilon held fixed. No new optimization was authorized
or run by this explanatory review.
