# Point-mass evidence

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
