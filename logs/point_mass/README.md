# Point-mass evidence

## Frozen-profile matched calibration

[Completed twelve-solve calibration](../../support/test/uzun2024_ctcs_mpc/POINT_MASS_MATCHED_CALIBRATION.md) · [raw evidence](audits/frozen_profile_calibration_20261020/REPORT.md).
User-selected DDP profile: NT, legacy gate, CTCS GN off. Both primary methods pass 2/3 fixed-time and 0/3 free-time cases. No test bearings solved. Solver/globalization work is stopped.

## Production validation and factor separation

[Production regressions, SOC/gate 2×2 and fixed-barrier diagnostics](../../support/test/uzun2024_ctcs_mpc/POINT_MASS_DDP_ABLATION.md) ·
[raw evidence](audits/production_and_ablation_20261020/).
Strict gating is opt-in, production keeps NT plus the legacy gate. Two production
tests fail with both SOC variants; SITL ends before landing. Normalized/GN-off
still succeeds with NT/legacy; all four GN-on cells fail. Slack/dual traces
identify a large CTCS violation accepted by the current globalization rule.

## DDP prerequisite repairs

[NT scaling, strict barrier gate and pilot results](../../support/test/uzun2024_ctcs_mpc/POINT_MASS_DDP_REPAIR.md) ·
[raw repaired N=6 traces](audits/nt_barrier_repair_n6_20261020/).
Both normalized/GN-on reruns fail independent path and integral checks.
No backward failures occur in these reruns, but SOC line-search failures remain.
The solver fixes do not establish a CTCS formulation difference or paper ranking.

## Matched paper campaign preparation

[Protocol and release gates](../../support/test/uzun2024_ctcs_mpc/POINT_MASS_PAPER_PROTOCOL.md) ·
[candidate, shared seeds and manifest](audits/matched_paper_protocol_20261020/).
The candidate contains 32 test bearings and three disjoint calibration bearings.
It is not frozen and no paper campaign has been launched. Historical selected
cost/tf/runtime tables cannot support quantitative solver rankings.
[Actual DDP factorization diagnosis](../../support/test/uzun2024_ctcs_mpc/POINT_MASS_DDP_FACTORIZATION.md):
logging-only replay reproduces all 42 failures with identical optimizer records;
the first misleading positive spectrum is traced to the asymmetric SOC IPM block.

## Direct paper non-augmented comparison

[Primary optimization report](../../support/test/uzun2024_ctcs_mpc/CTCS_OPTIMIZATION_DIFFERENCES.md) ·
[progress plot](audits/direct_nonaug_n6_20261020_v3/direct_nonaug_progress_first200.png) ·
[traces and ablations](audits/direct_nonaug_n6_20261020_v3/).
The matched N=6 run compares an L1/prox-linear reference without y against
our explicit primal-dual folded DDP, using the same physical problem, seed
and C++ integral map. This supersedes using an augmented-y distinction to
answer the question about the paper’s direct formulation.


## Optimization research

[Direct answer and source evidence](../../support/test/uzun2024_ctcs_mpc/CTCS_OPTIMIZATION_DIFFERENCES.md) ·
[local experiments](audits/optimization_structure_20261020_v2/REPORT.md) ·
[raw C++/convex-subproblem results](audits/optimization_structure_20261020_v2/results.json).
This separates exact accumulator equivalence from curvature, proximal metrics,
finite virtual penalties and stopping differences. It does not claim an
end-to-end speed or robustness ranking.


## Current open-loop 2 × 2 comparison

[Free-time PNG](open_loop/point_mass_2x2_free.png) · [PDF](open_loop/point_mass_2x2_free.pdf)

[Fixed-time PNG](open_loop/point_mass_2x2_fixed.png) · [PDF](open_loop/point_mass_2x2_fixed.pdf)

[Independent audits](open_loop/audit.json) · [comparison CSV](open_loop/comparison.csv) ·
[per-file hashes](open_loop/sha256_inventory.json) · [raw selected policies](open_loop/inputs/)

SCP selection uses a 1e-6 dynamics gap; saved DDP selections use 1e-8. All five
saved free-time SCP CTCS meshes pass physical and integral checks. The DDP
free-time N=6 budget failure is retained. The heatmap reports the worse of
obstacle and glide clearance; the policy table reports obstacle clearance.
Settings, seeds and historical host load differ, so timings are not a matched
performance trial. [Full maintained report](../../support/test/uzun2024_ctcs_mpc/POINT_MASS_OPEN_LOOP.md).

## Paper MPC

**Current:** [rerun with Theorem-14 acceptance gate](paper_mpc_v2/) · [2×2 figure](paper_mpc_v2/comparison_2x2/fig_2x2.png) · [overview](paper_mpc_v2/comparison_2x2/fig_comparison.png). All four matched cells verified; both CTCS arms pass, both nodal arms violate between nodes. Original campaign below.

[Complete campaign](paper_mpc/) ·
[actual SCP/DDP CTCS paths](paper_mpc/ctcs_actual_paths_current/fig_ctcs_actual_paths.png) ·
[matched four-arm figure](paper_mpc/comparison_2x2/fig_2x2.png) ·
[latest verified fast DDP run](paper_mpc/P17_ddp_ctcs_inner100/summary.json)

All raw paper campaigns, including failed numerical candidates, remain together
for reproduction. [Maintained paper report](../../support/test/uzun2024_ctcs_mpc/RESULTS.md).

## Folded versus lifted nominal certificate

[Certified report](audits/folded_lifted_certificate_20261020_v7/REPORT.md) ·
[per-policy classifications](audits/folded_lifted_certificate_20261020_v7/certification.json) ·
[matched derivatives](audits/folded_lifted_certificate_20261020_v7/matched_cases.json).
All 40 frozen policies are re-audited; physical path, strict inputs, integral
budgets and sufficient tightening remain separate. The actual historical C++
row reproduces the N=6 4/8-substep miss. Original plots and selectors are unchanged.

## Audit history

[Threshold reconciliation](audits/threshold_reconciliation_20261019/reconciliation.json) ·
[earlier strict-gate diagnosis](audits/free_scp_strict_review.json).
The old strict-threshold plot is kept separately under
[archive/point_mass_history/](../archive/point_mass_history/); superseded attempts
and duplicate snapshots are in the [verified archive](../archive/README.md).
The running local benchmark and original plotting scripts remain under
`/home/lab/projects/solver/benchmark/openscvx_point_mass_ctcs`.
