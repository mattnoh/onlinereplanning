# What changes in CTCS optimization: folded DDP versus lifted SCP

**Answer:** exact folding does not change the CTCS feasible set. In these implementations, the optimization differences come from the local curvature model, the accumulator’s scaled proximal metric and finite virtual-control penalty, globalization, and stopping. We have now isolated several of these effects with matched local experiments; a full-solver convergence ranking remains unestablished.

## Progress against the research question

| Question | Evidence obtained | Conclusion |
|---|---|---|
| Does accumulator elimination itself change a local convex step? | Nine paired lifted/condensed subproblems | No, when every objective and constraint term is retained; maximum step difference 7.3e-12 |
| Does eliminating y permit dropping its proximal cost? | Same physical objective and constraints, with accumulator proximal term varied | No. That cost becomes an extra metric on physical steps and can change them |
| Can a hard increment row coexist with an over-budget physical integral model? | Low versus high virtual-control penalty on the same local subproblem | Yes: virtual control can compensate the integral-model row at finite penalty |
| What CTCS curvature does our DDP use? | Actual historical/current C++ evaluators on 12 identical fixed/free inputs | Same values and Jacobians; historical CTCS-specific Hessian absent, current GN contribution has rank 3 in the smooth active example |
| Does lifting remove small-gradient CTCS behavior? | True sphere integral and shape derivative near grazing | No. Both inherit the same physical integral gradient after exact elimination |
| Are stopping metrics comparable? | Pinned SCP source expressions and a controlled scaled-slack solve | No. We reproduced an additional scaling of already scaled CVXPY virtual control in J_vc |

The previous certificate work established evaluation accuracy and independently graded feasibility. It did not measure optimizer convergence. Those checks were necessary to avoid mistaking integration failures for optimization failures, but the work reported here addresses the local optimization mechanisms directly.

## 1. Same constraint does not imply the same local objective

Write the physical integral vector as I(z), its local Jacobian as G, and let C be the lower-triangular cumulative-sum matrix. At a dynamically consistent accumulator reference with pinned y0, hard linearized accumulator dynamics give delta y = D C G delta z, where D includes density scaling and the accumulator coordinate scale. Therefore eliminating y also transforms its proximal cost:

    lambda_y ||delta y||² = lambda_y D² delta zᵀ Gᵀ Cᵀ C G delta z.

This adds the Hessian 2 lambda_y D² Gᵀ Cᵀ C G. Dropping it yields a different optimization problem. If the reference has accumulator defects, condensation also produces affine terms. This is an algorithmic metric effect, not a relaxation of the exact CTCS feasible set.

The pinned OpenSCvx solver includes all scaled states and controls in its proximal objective, including the user accumulator. Density scaling D therefore changes this metric quadratically unless the corresponding weights or accumulator scaling are adjusted. Its virtual-control penalty changes linearly with D. A consistent coordinate change rescales both weights; simply multiplying dynamics and budgets is insufficient to preserve optimization steps.

Controlled paired-subproblem results (dimensionless toy model, not a trajectory benchmark):

| Accumulator metric lambda_y | D | VC penalty | Lifted physical step | Worst true linearized row |
|---|---|---|---|---|
| 0.0 | 1.0 | None | (-0.044444, -0.027778) | -6.76958e-14 |
| 0.5 | 1.0 | None | (-0.044444, -0.027778) | -6.55677e-13 |
| 0.5 | 10.0 | None | (-0.068233, 0.091165) | -2.79707e-14 |
| 0.005 | 10.0 | None | (-0.044444, -0.027778) | -4.65045e-14 |
| 0.0 | 1.0 | 0.01 | (0.483000, -0.149000) | 0.5032 |
| 0.0 | 1.0 | 10.0 | (-0.044444, -0.027778) | -3.26579e-14 |
| 0.5 | 1.0 | 0.01 | (0.486000, -0.144000) | 0.5072 |
| 0.5 | 10.0 | 0.01 | (0.360000, -0.090000) | 0.392 |
| 0.005 | 10.0 | 0.001 | (0.486000, -0.144000) | 0.5072 |

Each lifted result is paired with a condensed problem retaining every term. The strongest metric changes the hard-constrained step from approximately (-0.04444, -0.02778) to (-0.06823, 0.09117). Compensating the metric by 1/D² and VC weight by 1/D restores the original step. At the weak metric the hard solution happens to remain at the same active-set corner; a metric change does not necessarily change every step.

## 2. Virtual control creates a real optimization tradeoff

With a virtual control, the linearized accumulator equation is delta y = C(D G delta z + nu). A hard increment bound then controls the physical integral model plus nu/D. Negative nu can make the increment feasible while the physical integral model remains over budget. The L1 virtual penalty discourages this; finite penalty is not identical to a hard dynamics equation.

In the controlled example, initial penalty 0.01 permits a worst physical linearized row residual 0.5032 while all increment rows pass. Penalty 10 recovers the hard solution. These numbers prove the tradeoff mechanism; they do not attribute any historical run to a particular penalty, since its adapted weights were not saved.

At the saved SCP policies, the following geometry uses the measured JAX interval Jacobians and actual state/control scales, with the configured **initial uniform** proximal weights. It is not a replay of the initial iterate or a reconstruction of final adapted weights. The ratio is the largest added accumulator-metric eigenvalue relative to the physical proximal metric before imposing physical dynamics; it bounds potential influence and does not measure convergence speed.

| Saved policy | Maximum added/physical proximal metric ratio | Initial VC cost for one interval budget |
|---|---|---|
| fixed_SCP_ctcs_N10 | 0 | 0.01 |
| fixed_SCP_ctcs_N100 | 0 | 1 |
| fixed_SCP_ctcs_N20 | 0 | 0.1 |
| fixed_SCP_ctcs_N40 | 67.1711 | 1 |
| fixed_SCP_ctcs_N6 | 0.00651091 | 0.0001 |
| free_SCP_ctcs_N10 | 0.0509135 | 0.001 |
| free_SCP_ctcs_N100 | 0 | 10 |
| free_SCP_ctcs_N20 | 1.12356 | 0.01 |
| free_SCP_ctcs_N40 | 1902.27 | 10 |
| free_SCP_ctcs_N6 | 0.031589 | 0.001 |

Thus the accumulator metric is negligible for some frozen profiles and potentially large for others. It cannot be assumed uniformly negligible or uniformly dominant.

## 3. Our historical and current DDP do not use the same CTCS curvature

The historical scalar folded row supplies an IPM slack-elimination contribution proportional to (dual/slack) grad(I) grad(I)ᵀ. That individual scalar-row contribution has rank at most one. This statement concerns the isolated CTCS contribution, not the rank of the entire DDP backward-pass matrix.

The current DDP additionally supplies a CTCS Gauss–Newton approximation proportional to 2 sum_j weight_j grad(r_j) grad(r_j)ᵀ for active integrated violation residuals. In the matched smooth active example, its rank is 3, and its projection orthogonal to grad(I) has rank 2. These two directions receive CTCS curvature that the historical scalar-row slack term alone cannot supply. The GN approximation is not the full exact integral Hessian.

Across all 12 actual C++ fixed/free cases, historical and current row values and Jacobians agree exactly. Their curvature models differ. Consequently, comparing historical point-mass DDP results with current MPC DDP results mixes optimizer revisions even when the physical integral definition agrees. Unit-multiplier curvature probes do not reconstruct historical dual values or prove a performance benefit.

## 4. Small-gradient behavior is shared by both formulations

For a smooth quadratic grazing profile of positive residual height h, the active time window shrinks like sqrt(h). The squared-violation integral behaves like h^(5/2), and its derivative with respect to h behaves like h^(3/2). Independent root-aware integration of the true sphere model verifies these orders:

| Residual height [m] | True integral | dI/dheight |
|---|---|---|
| 1.0e-02 | 2.6614233e-06 | 0.00066483096 |
| 1.0e-03 | 8.4310838e-09 | 2.1076053e-05 |
| 1.0e-04 | 2.6666143e-11 | 6.6664833e-07 |
| 1.0e-05 | 8.4327239e-14 | 2.1081793e-08 |
| 1.0e-06 | 2.6666661e-16 | 6.6666648e-10 |

The final decade has observed exponents 2.499999 and 1.499999. Lifted dynamics contain the accumulator identity derivative, but after elimination the physical decision variables still see grad(I). Lifting does not create a missing physical avoidance direction. If numerical integration misses the active window, both value and physical derivatives can be zero; the earlier N=6 reproduction isolated that separate evaluation error.

## 5. Reproduced SCP stopping-metric scaling discrepancy

The pinned CVXPY dynamics impose inv_S_x times the physical dynamics defect = nu, so nu is already in scaled state coordinates. The solver returns that nu unchanged through its iteration callback. The SCP iteration then computes:

    VC = abs(inv_S_x @ solution.nu.T).T
    J_vc = sum(VC)

This applies the state scale again. With a diagonal scale S, the metric is proportional to physical defect/S² rather than physical defect/S. The reproduction executes these assignments extracted verbatim from the pinned source and uses its actual convergence predicate.

| State scale S | Physical defect | Solver nu | Once-scaled norm | Actual J_vc | Actual stop at ep_vc=0.05 |
|---|---|---|---|---|---|
| 1.0 | 1.0 | 1 | 1 | 1 | False |
| 10.0 | 1.0 | 0.1 | 0.1 | 0.01 | True |
| 100.0 | 1.0 | 0.01 | 0.01 | 0.0001 | True |

At S=10 the source predicate accepts J_vc=0.01, whereas the once-scaled norm is 0.1. The threshold 0.05 is chosen to isolate the contract discrepancy, not taken from a historical benchmark. For the user accumulator S_y=1 this particular discrepancy has no extra scaling effect; it affects physical states whose scales differ from one. It does not by itself explain an accumulator defect or any particular historical stopping event. No upstream solver code was changed.

DDP stopping also differs by revision: historical inner tolerance is multiplied by barrier and AL penalty schedules, and schedule exhaustion can return Outer Max/Min. Current DDP additionally gates optimal termination on primal feasibility and shooting gaps and tightens the final inner limit. SCP checks scaled step, virtual-control and buffer metrics rather than a DDP-style stationarity residual. A termination flag is therefore not a matched optimality or independent feasibility claim.

## Source evidence and reproducibility

- Current CTCS GN: [stc_stage_constraint.h](/home/lab/mavros_ws/src/ALIPDDP-main/problem_maker/constraint/stc_stage_constraint.h:424); connected to the backward pass at [alipddp.tpp](/home/lab/mavros_ws/src/ALIPDDP-main/alipddp/alipddp/alipddp.tpp:1217).
- Historical slack-elimination curvature: [alipddp.tpp](/home/lab/projects/solver/DDP/ALIPDDP-ctstc-al/alipddp/alipddp/alipddp.tpp:969).
- SCP proximal and virtual-control costs: [cvxpy_ptr_solver.py](/home/lab/projects/solver/SCP/OpenSCvx/openscvx/solvers/cvxpy_ptr_solver.py:482); scaled dynamics at line 634; unchanged nu returned at line 1095 and through callback at line 1234.
- SCP stopping metrics: [iteration.py](/home/lab/projects/solver/SCP/OpenSCvx/openscvx/algorithms/scvx/iteration.py:314); predicate in [base.py](/home/lab/projects/solver/SCP/OpenSCvx/openscvx/algorithms/base.py:213).

The adjacent results.json includes actual C++ matrices, paired-subproblem solutions, saved-profile metric estimates and the source-extracted metric probe. Binaries, sequential build logs, source snapshots, test output and hashes are preserved alongside it. The prior certificate snapshot remains unchanged.

## Remaining question

We have identified and reproduced optimization mechanisms, not established a global speed or robustness ranking. A full-solver comparison still needs the same integral discretization and derivatives, initial guess, physical scaling, tolerances and revision, then traces of objective, true CTCS feasibility, steps, multipliers/penalties and stopping. Adapted historical penalties and duals are not available. Fixing the stopping metric or modifying an optimizer is a separate implementation gate; no trajectory optimization campaigns or production changes were made here.
