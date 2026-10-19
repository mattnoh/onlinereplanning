# Normalized DDP/GN: actual factorization failure

An isolated logging-only replay of the existing fixed-time N=6 normalized/GN
pilot reproduces all 42 reported QUU_NOT_PD failures. All 301 optimizer records
(iteration, X, U, mu, rho and optimality error) match the original exactly;
timing is excluded. This is a diagnosis of the old pilot, not a new protocol
calibration run and not a production solver repair.

The original diagnostic diagonalizes (H+H.T)/2, but the solver performs LLT on
the symmetric matrix constructed from H's upper triangle. Those are different
when H is asymmetric. Six of the 42 failures have a positive minimum eigenvalue
in the original diagnostic and a negative one in the upper-triangle matrix.
Every failed *actual factored* matrix, including the last local jitter retry,
also fails NumPy Cholesky independently.

The first failure at stage 0 has:

| Measurement | Value |
|---|---:|
| Original averaged minimum eigenvalue | +79.29327375 |
| Upper-triangle minimum eigenvalue before local retry | −1105.97754844 |
| Actual factored minimum eigenvalue after local retry | −1005.97754844 |
| max absolute H−H.T entry | 2168.43977581 |
| Asymmetry of uncondensed Quu | 0 |
| Asymmetry of IPM contribution | 2168.43977581 |

Across the 42 captures, uncondensed Quu asymmetry is at most 2.91e-11.
The IPM contribution exceeds 1e-8 asymmetry in 16 captures, with maximum
2168.44. No stage equality-AL block contributes in these captured stages.
The remaining failures must not all be characterized as positive-definite
matrices with a misleading diagnostic: 36 averaged matrices are themselves
non-positive-definite.

The source explains why the SOC contribution need not be symmetric.
`calcCres` forms

    D = L(s)^(-1) L(y)
    H_ipm = C_u.T D C_u

using `soc_helper::LinvTimesArrow`. Individual Jordan multiplication matrices
L(s) and L(y) are symmetric, but their product in that order is generally not.
Scalar positive inequalities instead contribute C_u.T diag(y/s) C_u, which is
symmetric positive semidefinite. The LLT call then selects the upper triangle
of the complete condensed matrix. The logged uncondensed/IPM split locates the
first observed asymmetry in this SOC elimination contribution. It is not
evidence that the CTCS GN Hessian itself is asymmetric.

This is an optimization difference relevant to the research question: our DDP
eliminates SOC slack/dual Newton equations into a Riccati factorization, whereas
the prox-linear reference enforces input SOCs inside its convex conic subproblem.
The folded integral constraint does not create those SOC equations. The current
evidence does not establish that every SOC boundary stall shares this cause or
that averaging H alone gives a consistent primal-dual Newton method. A repair
must derive a consistent SOC scaling/elimination, including cross blocks,
directions and residual equations, and verify it before paper runs.

No production code was changed. The snapshot differs only by matrix logging
inside the existing SW_QUU_DIAG failure branch. Its source manifest, exact
command, binary, traces and results are under
[matched protocol evidence](../../../logs/point_mass/audits/matched_paper_protocol_20261020/).

Reproduce the snapshot with [the preparation script](point_mass_factorization_snapshot.py),
build the single `point_mass_factorization_trace` target with
`COMANDO_POINT_MASS_FACTORIZATION_SOURCE` pointing to that snapshot, and replay
the archived `factorization_command.json`. Analyze it with
[the audit script](point_mass_factorization_audit.py). Synthetic regression
tests explicitly cover a positive averaged spectrum with an indefinite
upper-triangle matrix and ensure logging replay checks detect changed policies.
