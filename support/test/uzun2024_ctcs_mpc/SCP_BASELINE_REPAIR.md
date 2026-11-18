# SCP performance baseline repair (point mass, 2026-10-21)

Response to `SCP_PERFORMANCE_BASELINE_HANDOFF.md`. DDP was not rerun: the thesis DDP columns
use the original open-loop bundle (logs/point_mass/open_loop, 1-4 RK4 substeps, per-mesh
budgets 1e-12..1e-10 s, archived serial times), drawn via paper_2x2/figures_v5.

## 1. Measured bottlenecks (before)

| Evidence | Finding |
|---|---|
| paper_2x2 fixed CTCS K=40 (thesis setting), `openscvx_point_mass_ctcs/scp_repair/profile_before/paper2x2_ignore_dpp_fixed_ctcs_N40.json` | 1.95 s/iteration: CVXPY canonicalization 42.3 s of 46.8 s (ignore_dpp), integration 4.2 s, conic solve 0.6 s |
| original archive fixed CTCS K=40, `scp_repair/screen_n40/clarabel_driver.json` | 97 ms/iteration: Clarabel 1e-10 1.52 s, Dopri8 (dtmax 1/(10N)) 1.33 s of 3.0 s |
| paper_2x2 free CTCS (selection v3) | 765-857 iterations with constant proximal weights (lam_prox 3-10) |
| OpenSCvx `examples/abstract/hypersensitive.py`, same machine | 17 ms/iteration, 132 iterations, 2.2 s without printing (4.5 s with printing) |

ignore_dpp existed because the accumulator y = I/B (1e10 scale) made Clarabel fail on the
compiled DPP problem (status `unknown`, reproduced in `paper_2x2/scp_backend_diag/clarabel_dpp`).
The original 320-iteration fixed K=6 pick is a crawl: dynamically feasible by iteration 4,
then ~315 iterations moving clearance from 1.76 m to the 5 mm shell with cost change 1e-3
(constant weights, cost gradient ~1e-5 effort term; Elango Remark 25).

## 2. Corrections and equivalence checks

Final setting (`REPAIR` in `point_mass_2x2_scp_grid_repaired.py`): y = 1e5 * I (change of variables,
same feasible set I_k <= B), compiled DPP with Clarabel at the unchanged 1e-10 tolerance,
OpenSCvx acceptance-ratio weight update (`AugmentedLagrangian`, Elango Fig. 3 "Update
Hyperparameters"), printing off. Problem, seed, integrator, convergence thresholds and
selection rule unchanged. Defaults of `point_mass_2x2_paper.py` reproduce the historical runs.

Rejected, with evidence kept (diagnostic scripts removed; outputs remain):
- QOCO (paper_2x2/scp_repaired, scp_repaired_v2): node-only arms gave identical policies
  (objective, clearance, iterations) 6-15x faster, but 4/8 CTCS cells ended with a final
  unchecked step into the 5 mm shell (certified I/B lower bound 182 at fixed K=6). On the
  original picks QOCO left 3 CTCS cells at the 400-iteration cap (`scp_repair/retimed_repaired.json`).
- Uncapped adaptive Dopri8 (scp_repaired v1): missed density pockets at K=6.
- Fixed-step Dopri8 (`scp_repair/integration_accuracy.log`, recorded SCP iterates): near-budget
  error 60/26/8.6/2.1/0.43 eps at S=1/2/4/8/16 (fixed K=40), up to 155 eps at K=10; the
  original adaptive integrator is kept.
- Held-out-bearing step calibration (`scp_repair/calibration_results.json`): the obstacle is
  inactive at bearings pi/3 and pi, so it does not discriminate step counts; not used.
- Constant weights with QOCO grid (percell_grid_v3_qoco, 35 runs): 500-1000 iterations.

## 3. Horizon comparison

Per-cell SCP grid rerun serially (paper_2x2/percell_grid_v4, 328 runs, all exit 0),
selected with the unchanged rule (`point_mass_2x2_select_repaired.py`,
paper_2x2/figures_v4/selection.json; figures_v5 swaps in the original DDP rows). All 16 SCP cells are tier 1 (accepted, converged).

| Time | K | SCP node c [mm] / t [s] (was t) | SCP CTCS c [mm] / t [s] (was t) |
|---|---|---|---|
| Fixed | 6 | -17.33 / 1.15 (6.61) | 20.19 / 0.537 (1.27) |
| Fixed | 10 | -7.88 / 1.95 (12.2) | 4.87 / 1.18 (43.8) |
| Fixed | 20 | -13.24 / 3.94 (23.7) | 4.90 / 1.46 (19.2) |
| Fixed | 40 | -8.36 / 7.18 (60.5) | 5.67 / 1.24 (46.2) |
| Free | 6 | -29.85 / 1.59 (2.39) | 4.76 / 0.866 (137) |
| Free | 10 | -11.90 / 1.45 (10.7) | 5.27 / 0.775 (227) |
| Free | 20 | -25.07 / 5.23 (17.4) | 4.76 / 24.0 (503) |
| Free | 40 | -8.41 / 21.4 (594) | 4.82 / 4.07 (1503) |

Times are the single serial (nice 19) run of the selected setting in the grid; iterations
14-501. Every CTCS policy passes the certified physical check and the Theorem-14 budget gate.

## 4. Thesis

KO/EN/Markdown: merged clearance/time table (tab:eval-budget-summary, best feasible per row
in bold), SCP method sentence, timing paragraph, K=6 clearance sentence, appendix timing note;
Figs 4.1/4.2 redrawn from figures_v4 (`main/figures/evaluation/replot_pointmass.py`). Both PDFs
build without errors.

## 5. Remaining limitations

- DDP (original bundle) and SCP CTCS use different budgets: per-mesh 1e-12..1e-10 s with
  0.5-2.2 mm tightening for DDP, common 1e-10 s with 5 mm for SCP; stated in the thesis.
  At 1 substep on the common budget, 7/8 DDP CTCS cells failed (paper_2x2/ddp_substeps).
- Free K=20 SCP CTCS needs 501 iterations (24 s) under the selected setting.
- SCP times include Python/JAX/CVXPY; Elango's 59 ms is generated C and not comparable.
