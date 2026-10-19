# Paper non-augmented CTCS versus our folded DDP

**Paper comparison status:** historical mesh figures are qualitative evidence;
their selected costs, final times and timings cannot rank the methods. The
N=6 experiment below is a controlled warm-start pilot, not a robustness study.
The [matched paper protocol](POINT_MASS_PAPER_PROTOCOL.md) records the new
shared-budget, shared-start campaign and its unresolved release gates. Its
candidate settings and generated seeds have not been used for paper runs.

**The formulation is shared; the optimization treatment differs.** The local Elango paper already defines direct interval-integral inequalities without y in Section 5, equations (47)–(49). The comparison here is its L1/prox-linear treatment in Section 5.1 versus our explicit primal-dual DDP treatment of the same CTCS rows. No accumulator metric or accumulator virtual control appears in this comparison.

Local primary source: [Elango et al. PDF](</home/lab/Zotero/storage/FGP9CC2S/Elango et al. - 2024 - Successive Convexification for Trajectory Optimization with Continuous-Time Constraint Satisfaction.pdf>). Section 5 starts on printed page 14; equations (49)–(50) and Theorems 29–30 are on printed page 15. A source copy and hash are saved with the experiment.

## 1. The actual optimization difference

Let c_k(z)=xi_k(z)-B_k, with no y variable. Both methods target c_k(z)<=0. The paper constructs, in equation (50),

    Phi_gamma(z) = f(z) + indicator_Z(z) + gamma sum_k [c_k(z)]_+.

Its prox-linear step (equation (42), applied to this composite problem) uses first-order integral models:

    min_d  convex cost model + gamma sum_k [c_k(z)+grad(c_k(z)) d]_+
           + ||d||²/(2 rho),    z+d in Z.

The positive-part epigraph is a convex subproblem variable, not an accumulator. In our reference, known convex quadratic physical effort cost is retained exactly in the convex part J, while xi is linearized. This is a valid prox-linear splitting; it is not a claim to reproduce every choice in the authors’ code.

Our DDP instead keeps c+s=0, s>0, lambda>0, and drives primal residual and complementarity s*lambda-mu toward zero. Its backward pass contains the scalar-row slack-elimination term (lambda/s) grad(c) grad(c)^T. The current revision also supplies lambda times a CTCS Gauss–Newton Hessian. It obtains a control update from the Riccati backward pass, then searches along that policy update while checking merit and slack/dual positivity, including the control SOCs.

| Mechanism | Paper non-augmented prox-linear | Our folded DDP |
|---|---|---|
| Integral inequality | L1 positive-part penalty on the budget residual | Explicit nonlinear inequality with slack and dual |
| Integral model in a step | First-order affine model inside the hinge | First derivative plus current CTCS GN curvature and IPM terms |
| Constraint multiplier | Local hinge multiplier is between 0 and gamma | Positive dual determined by primal-dual updates; no equivalent gamma cap |
| Globalization | Proximal weight and line search on the penalized objective | Policy line search, barrier/AL merit and slack/dual cone guards |
| Other convex constraints in this fixed-time example | Affine dynamics/endpoints and input SOCs enforced directly in the convex subproblem | Input SOCs handled by the DDP primal-dual machinery; terminal equality handled by AL machinery |
| Stationarity/stopping | Prox-gradient mapping G_rho; feasibility must also be checked | Native optimality/primal/barrier/gap tests and possible schedule/iteration exits |

For the hinge epigraph t>=c+grad(c)d, t>=0, minimization of gamma*t bounds its local row multiplier by gamma. This is an immediate difference in optimization forces: a small gamma can trade objective reduction for integral-budget violation. Under the paper’s convex/Slater assumptions a sufficiently large gamma gives exact penalization; arbitrary gamma is not automatically sufficient. DDP also permits infeasible intermediate iterates because c+s residual need not yet be zero. Explicit inequality handling does not mean every returned iterate is feasible.

The scalar test min 0.5(z-1)^2+gamma[z]_+ has solution z=max(0,1-gamma). At gamma=.25 it deliberately retains violation .75; at gamma=1.5 it recovers the hard constraint solution. A finite log-barrier central-path point instead has z=(1-sqrt(1+4mu))/2<0, with lambda=mu/(-z). These identities and the multiplier bound are tested.

## 2. Matched run on our actual point-mass problem

This is the existing **fixed-time 3D N=6 point mass**, not a substitute convex toy: tf=4, original initial/terminal states, gravity, input norm/cone, obstacle and glide CTCS, margins and epsilon=1e-11. The obstacle makes the problem nonconvex, so Section 5’s global convex-minimizer theorem is not claimed. We apply the same direct-integral prox-linear template as an empirical nonconvex extension.

This is a new current-revision research run, not a reconstruction of a historical optimizer run or its exact tuning. The two methods use exactly the same X/U seed, objective and C++ CTCS map with 512 RK4 substeps. The warm seed is an exact rollout of 90% frozen stage-policy controls and 10% frozen CTCS-policy controls. This convex control blend avoids the almost zero input-SOC slack of the original stage seed; it is shared by both methods and is not a cold-start robustness test. The first recorded X/U arrays agree exactly.

For fixed time the DDP point-mass row is mean-square: I_row=I_physical/dt. Thus B_physical=epsilon*dt. The reference uses c=I_row/epsilon-1 and gamma=1e-6, equivalent to L1 weight 1e5 on I_row-epsilon. DDP’s original raw row and its normalized counterpart are both tested below; all derivatives/curvature are transformed consistently. Equality of feasible sets does not make absolute stopping tolerances or slack initialization invariant to this change of coordinates.

The reference uses an adaptive proximal step with sufficient decrease of the true numerical penalized merit. It stops on prox-gradient mapping below 1e-7 plus budget satisfaction. DDP uses the unmodified current solver with a 1200-iteration cap, tolerance 1e-8, mu=1e-3, mu_min=1e-10, terminal AL rho=100 with cap 1e10, and its existing line-search/cone guards. Exact settings and raw logs are archived.

Independent replay evaluates the full propagated policy, not only the interval-local states used by either optimizer. The common gate is integral/budget<=1.0001, physical excess<=1e-7, terminal error<=1e-6, shooting gap<=1e-8, input/cone excess<=1e-7. Strict budget satisfaction is recorded separately and both successful final policies below also receive strict rational nominal-model certificates.

| Method/settings | First common-feasible native iteration | Native termination | Final objective | Final true integral / budget | Returned policy passes? |
|---|---|---|---|---|---|
| Raw DDP, GN on | 4 | Max Iteration (1201) | 4.004225939253 | 188.75829 | False |
| Normalized DDP, GN on | 149 | Outer Max/Min (299) | 4.004697721396 | 1.5035598e+08 | False |
| Raw DDP, GN off | 6 | Max Iteration (1201) | 4.004203440619 | 1327.5076 | False |
| Normalized DDP, GN off | 158 | Optimal Solution (541) | 4.004202515041 | 0.96781635 | True |
| Direct non-augmented prox-linear | 41 | Prox-gradient + budget (71) | 4.004202520112 | 0.77212889 | True |

**Observed:** raw-row/current-GN DDP becomes feasible at iteration 4, then loses the integral budget. Its best independently feasible objective is 4.004416845202; its cheaper final iterate is not budget feasible. The direct prox-linear reference becomes feasible at iteration 41 and finishes at 71 with mapping 8.45e-8, objective 4.004202520112 and integral/budget 0.77213. Normalized DDP with CTCS GN disabled also succeeds, with objective 4.004202515041 and integral/budget 0.96782. Those successful objectives differ by only about 5.1e-9.

GN-off here is a controlled research constraint-model ablation in the wrapper; it is not a production solver change or a reconstruction of every historical revision difference. The same nonlinear row and Jacobian remain in place.

For a common final-point stationarity probe, the same prox-linear mapping was also evaluated at fixed rho=1000 and gamma=1e-6 for all three endpoints. This is a stationarity surrogate for the same penalized objective, not a global-optimality proof:

| Endpoint | Common prox-gradient mapping norm |
|---|---|
| DDP raw/GN | 2.334004e-05 |
| prox-linear | 1.6686977e-07 |
| DDP normalized/no-GN | 6.9366146e-08 |

The reference’s own adaptive-rho mapping (8.45e-8) and the common fixed-rho mapping are different numerical stopping measurements. They must not be silently equated.

## 3. What the failure traces actually show

For raw-row/current-GN DDP, 1155 of 1200 logged iterations report ForwardFail::BOUNDARY_SOC; only 45 have no failure flag. The check is applied to the **control SOC slack or dual**, not directly to the physical CTCS integral. Thus it would be wrong to explain this run only by integral-gradient flatness or claim folding itself caused failure. The reference instead handles those convex control SOCs directly in its convex subproblems.

The normalized+GN DDP run also records backward QUU_NOT_PD failures. A subsequent
[logging-only replay](POINT_MASS_DDP_FACTORIZATION.md) now captures the actual
matrices: all 42 failures reproduce independently, with all 301 optimizer
records unchanged. Six matrices have a positive averaged spectrum but an
indefinite upper-triangle matrix used by LLT. The first has averaged minimum
eigenvalue +79.29 versus actual factored minimum −1005.98; its asymmetry comes
from the SOC IPM contribution. This locates a concrete SOC elimination and
factorization issue without claiming the CTCS GN Hessian is itself defective.

The raw final integral ratio 188.76 corresponds to a raw mean-square row residual of approximately 1.88e-9, below the solver’s absolute tolerance 1e-8. This shows why that absolute numerical scale cannot stand in for a relative 1e-11 budget check; it is not proof that tolerance alone caused the whole stall. Row normalization and GN ablations expose a substantial interaction with the rest of the primal-dual solver.

Numerical-map versus independent full-policy replay at the two main endpoints:

| Endpoint | Same C++ map integral/budget | Independent integral/budget |
|---|---|---|
| Raw DDP + GN | 188.7555183 | 188.7582911 |
| Non-augmented prox-linear | 0.7717140 | 0.7721289 |

These are not the earlier 4/8-substep missed-pocket failure. The DDP budget failure persists with the shared fine map and independent integration. Rational nominal-model auditing separately proves the returned raw/GN DDP policy fails the budget while clearing the physical obstacle/glide constraints, and proves the two successful final policies pass both path and integral checks.

## 4. How convergence is now visible

The new evidence contains each method’s X/U iterate, objective and independently replayed physical/integral/terminal/shooting checks. Reference trials include proximal parameter, penalty, merit, rejected trials and prox-gradient mapping; DDP preserves its native backward/forward failure codes, schedule values and every logged trajectory. Recording uses a stream observer outside the unmodified solver; its measured overhead is recorded.

![Direct objective, budget and best-feasible progress](../../../logs/point_mass/audits/direct_nonaug_n6_20261020_v3/direct_nonaug_progress_first200.png)

The three views distinguish cheaper iterates from feasible improvements. The complete 1200-iteration plot and the first-200 view are both archived. Native iteration types have different work. Python/CVXPY/oracle-IPC versus compiled C++ timings also have different overhead, so this pilot is not an intrinsic runtime ranking.

This gives a concrete result for **one warm-started N=6 case and recorded settings**: the direct prox-linear reference returns a feasible high-quality policy where the original raw/current-GN DDP profile does not; a normalized/GN-off DDP profile reaches essentially the same quality. It does not establish that SCP always converges better or that GN should generally be removed. More meshes, seeds and documented parameter calibration would be required for that broader claim.

## Local sources and artifacts

- Paper: Sections 5–5.1, equations (49)–(50), prox-linear template (42), mapping G_rho, and Theorems 29–30 in the local PDF. The convex theorem assumptions are not transplanted to our obstacle case.
- DDP primal-dual residuals and slack elimination: current alipddp.tpp, calcCres; CTCS curvature: constraint/stc_stage_constraint.h, lagrangianHessian; control SOC forward guard: alipddp.tpp around line 1597.
- Research tools: [point_mass_nonaug_compare.py](point_mass_nonaug_compare.py), [point_mass_nonaug_trace.cpp](point_mass_nonaug_trace.cpp), [tests](test_point_mass_nonaug_compare.py).
- [Raw comparison](../../../logs/point_mass/audits/direct_nonaug_n6_20261020_v3/comparison.json), [DDP ablations](../../../logs/point_mass/audits/direct_nonaug_n6_20261020_v3/ablation_summary.json), [nominal certificates](../../../logs/point_mass/audits/direct_nonaug_n6_20261020_v3/nominal_certificates.json), and adjacent traces, sources, build/test logs and hashes.

All 36 tests pass. Existing benchmark pools, original plots, controllers and production optimization algorithms are unchanged. The read-only research wrapper can disable its CTCS Hessian contribution for the explicit ablation. The earlier lifted-SCP implementation study is preserved as a [supplement](CTCS_LIFTED_COMPARISON_SUPPLEMENT.md), not used as the answer to this direct comparison.
