# DDP prerequisite repairs and unchanged-seed N=6 reruns

SOC appears here because the existing point mass has input norm and tilt-cone
constraints in addition to CTCS obstacle/glide rows. DDP solves their slack/dual
systems in the same backward pass. Correcting those equations is a prerequisite
for attributing a convergence failure to CTCS. It is not a new distinction
between folded and lifted CTCS and does not change the paper's integral theory.

The local Elango paper already has direct integral inequalities in Section 5.
Keep the three questions separate: lifted versus folded representation; direct
prox-linear versus direct primal-dual DDP; and revision-specific DDP defects.
The SOC finding and barrier strategy belong to the third question.

## Implemented changes

The solver core now uses a symmetric Nesterov–Todd metric H with Hs=y,
together with the consistent centered scaled right-hand side
b=H*rp+mu*s^{-1}-y. Stage and terminal condensed Hessians, cross blocks,
slack/dual recovery and the parameter-coupling applications use that same
metric. This replaces the nonsymmetric L(s)^{-1}L(y) product; it does not
average the old matrix. The existing raw complementarity residual remains
part of the convergence measurement.

The inner barrier threshold is now

    max(tolerance, min(1,inner_tolerance)*minimum_active_mu).

It has no rho multiplier. Inactive terminal/stage barriers are excluded. The
default kappa is 1; old caller values above 1 are capped at 1. The numerical
tolerance floor is explicit. Reaching an inner cap or exhausting backward/
forward regularization no longer advances the barrier/AL schedule. A stationary
point may advance even when a roundoff-sized trial step is rejected, because
stationarity is measured at the current point before that trial.

The previous report checked only whether mu decreased numerically. That missed
the user's intended Fiacco–McCormick subproblem-solve requirement. The corrected
protocol now distinguishes these two properties.

The solver implementation/derivation and primary-source pointer are in
[SOC_NT_AND_BARRIER.md](/home/lab/mavros_ws/src/ALIPDDP-main/tests/SOC_NT_AND_BARRIER.md).
Core implementation commit: `b994e4d4`; subsequent regression additions test full
three-dimensional stage and terminal cones against closed-form central points.

## Pilot results: both fail

Use exactly the old N=6 warm seed, original epsilon/margins, fixed tf=4 s,
512 substeps, normalized rows, CTCS GN on and 1200 total iterations. The first
rerun retains the 60 accepted-step inner cap. A second diagnostic changes only
that cap to 600. These are neither candidate-protocol calibration starts nor
paper campaign runs. No failed policy is replaced by a successful incumbent.

| Measurement | Original normalized/GN pilot | NT + strict gate, inner 60 | NT + strict gate, inner 600 |
|---|---:|---:|---:|
| Termination | Outer Max/Min | Inner Iteration Limit | Forward Pass Failed |
| Native iterations | 299 | 84 | 234 |
| Final mu | 1e-10 | 1e-3 | 1e-3 |
| Final true integral/budget | 1.50356e8 | 9.20204e8 | 5.25625e8 |
| Maximum physical excess, m | 0.11805 | 0.28516 | 0.21506 |
| Terminal infinity error | 1.815e-5 | 0.03520 | 0.03587 |
| Returned policy feasible | no | no | no |

Both repaired runs have zero logged backward failures and zero QUU_DIAG
failure entries. This shows removal of the observed factorization failure in
these reruns, not a universal guarantee. The inner-60 run has 60 accepted
iterations, 19 ERROR_INCREASE and 5 BOUNDARY_SOC flags; inner-600 has 191
accepted, 19 ERROR_INCREASE and 24 BOUNDARY_SOC flags. Thus SOC boundary
rejections are not eliminated by NT. Their equivalence to the earlier
factorization root cause is not established.

Both stop in the initial barrier subproblem with mu=1e-3 and rhoT=100. The
fixed-barrier optimality/primal errors are still enormous. The strict exit
logic now exposes that unresolved failure instead of advancing schedules.
Increasing the cap is insufficient. Rational full-policy auditing certifies
physical and integral failures in intervals 2,3,4 (zero-based) for both policies.
All intervals retain their separate pass/fail results.

These failures leave the normalized/GN-on profile unready. Further work must
investigate fixed-barrier globalization and normalized-row/slack-dual
initialization, with CTCS GN isolated as an ablation. This report does not claim
those hypotheses are established causes, and it does not disable GN or change
the controls, physical margins, budgets or production quadrature.

## Verification and raw evidence

The solver KKT target passes 90 noncollinear NT cases with independent scaled
Jordan equations and dense KKT directions, full 3D stage/terminal cone optima,
barrier gate/cap checks, and existing scalar, LQR, equality and CTCS GN tests.
The existing CTCS self-check passes all 10 checks. The paper-model derivative
check passes (relative derivative error 5.94e-11). The 42 focused Python tests
also pass. Executable targets were built sequentially.

[Archived configurations, binaries, source hashes, independent audits,
certificates and traces](../../../logs/point_mass/audits/nt_barrier_repair_n6_20261020/).
An initial longer-cap invocation used the wrong positional argument and aborted
before solving; its exact invocation and stderr are retained under `inner600`.
The corrected invocation is the one reported above.

Calibration, protocol freezing and the 960 test runs remain unstarted. Commit
the final protocol hash before any test outcome is observed, after the solver
readiness and adapter gates are met. No quantitative paper table follows from
these repairs.
