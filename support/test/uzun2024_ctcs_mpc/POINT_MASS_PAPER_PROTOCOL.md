# Matched point-mass paper protocol

The historical mesh selections support qualitative path and enforcement examples.
They do not support quantitative cost, final-time, convergence-rate or runtime
rankings. Keep the raw figures and evidence; identify panels (b)/(c) as qualitative
only and retain the N=6 budget failure in their audit. A physically clear path
does not excuse a tightened CTCS budget failure.

The executable [protocol generator](point_mass_paper_protocol.py) records a
**candidate**, shared seed arrays and a manifest. The DDP profile is now frozen by user selection; the complete campaign and other profiles remain candidates. A candidate hash is not a preregistration or a
completed experiment. Current historical and N=6 pilot results cannot fill its
cells: their budgets, margins, initialization and/or settings differ.

## Physical problem and sampling

Use folded primal-dual DDP and lifted OpenSCvx CTCS as the primary pair. Include
the paper-style direct non-augmented prox-linear reference as an ablation.
Elango Sections 5–5.1 already contain the direct integral formulation; folding
is not the novelty claim. The proposed contribution is the DDP stage constraint
and primal-dual optimization treatment. The obstacle case is nonconvex; the
paper's convex global guarantee does not apply to it.

Candidate settings are B=1e-11 seconds of squared dimensionless residual per
interval, 5 mm obstacle tightening and 5 mm glide-residual tightening, with
the same 2 m residual length scale and density in every arm. The glide residual
is horizontal radius minus altitude (tan=1); its margin is a residual in metres,
not the perpendicular distance to the cone. These candidate values require
calibration and certification before freezing. They are not claimed sufficient.

Use N=6,10,20,50,100, fixed tf=4 s and free tf with a common initial tf=4 s.
The candidate free-time bounds are [1,30] s and require adapter verification.
Budgets apply to each physical interval. Thus total allowed budget is N*B;
mesh comparisons measure transcription sensitivity at this declared convention,
not an identical total-horizon budget. Do not imply otherwise.

For legacy fixed-time mean-square rows, epsilon=B/dt and normalized row
(I/dt-epsilon)/epsilon=I/B-1. For free-time integral rows, epsilon=B and
(I-epsilon)/epsilon=I/B-1. Normalize derivatives and curvature consistently.
For SCP s=N*dt, dI/ddt=N*dI/ds. Density scaling must also scale the lifted
accumulator dynamics and budgets. Record the physical increment before scaling.

The 32 test cases rotate initial position and horizontal velocity around the
obstacle at z=10 m, keeping distance to the obstacle and velocity norm fixed.
This varies approach bearing across a full circle. Three distinct calibration
bearings include the original start; no test bearing overlaps calibration.
No starts may be removed after seeing solver outcomes. Unknown physical
infeasibility must remain distinct from a proved infeasible problem.

Each arm receives identical binary-equivalent X/U/durations for each case and
mesh: hover controls and exact rollout from that initial state. This is a
deliberately cold seed with an endpoint defect, not a policy drawn from a saved
successful run. Adapters must demonstrate equality of received arrays and
objective values before the freeze. A later warm-start study needs its own
predeclared common seed rule.

## Calibration and release gates

Give every method the same declared calibration work budget on the three held
out calibration starts. Log every attempted setting, penalty, rejection and
failure, then select one profile per method across meshes and time modes.
Freeze those profiles before any test solve. Adaptation within a solve is
allowed only when its deterministic rule is part of that frozen profile. In
particular, record the initial and adapted virtual-buffer/virtual-control
weights, not just their initial values. The calibration work budget, profiles,
native iteration caps and stationarity thresholds must be recorded. The first matched calibration uses one profile trial per primary method; see the [completed calibration](POINT_MASS_MATCHED_CALIBRATION.md).

DDP's historical scalar update satisfies monotonicity for positive mu_min,
mu>=mu_min and 0<mu_mul<1:

    mu_next=max(mu_min,min(mu_mul*mu,mu**mu_exp)) <= mu.

This arithmetic property is distinct from the monotone barrier strategy:
each fixed-barrier subproblem must be solved before decreasing mu. The old
rho-weighted inner threshold failed that requirement. The
[implemented repair and pilot reruns](POINT_MASS_DDP_REPAIR.md) remove the rho
factor, cap effective kappa at 1, exclude inactive barriers, and prevent
unsolved inner/pass exits from advancing mu or AL schedules. These fixes do
not yet make the normalized/GN-on pilot converge.
Strict gating is now opt-in (`use_strict_barrier_gate=true`); production restores
the legacy gate and `inner_tolerance=10` until validation. The
[subsequent production tests and 2×2](POINT_MASS_DDP_ABLATION.md) retain two
production failures and a SITL `NO_SETTLE`. The user subsequently selected NT + legacy gate + CTCS GN off for the point-mass experiment and stopped solver/globalization work. The [frozen profile and twelve matched calibration solves](POINT_MASS_MATCHED_CALIBRATION.md) supersede further solver debugging as the next-step requirement. This selection is not a robustness claim. Production failures remain recorded separately. The complete campaign is still not frozen.

Choose fixed optimizer integration settings from calibration accuracy tests,
including inactive, grazing and narrow-positive-pocket cases. Sixteen substeps
is not an accuracy certificate. Keep an independent rational/exact-flow auditor
for all arms; convergence of two grids is insufficient. Require certified
numerical-map error <=1e-4*B per interval and certified physical budget bounds.
If this cannot be proved within the existing subdivision limit, mark the case
uncertified and retain it in the denominator. Never repair a test run by tuning
quadrature after looking at its result.

Before test runs, pin each repository's clean tracked commit, relevant source
hashes, compiler/build settings, binary hashes, environments, profile hash,
protocol hash and seed hashes. Unrelated untracked artifacts are not sources.
Archive required untracked source dependencies explicitly. A dirty historical
source tree plus a base commit is not a pinned paper implementation.

## Common assessment and reporting

Use independently propagated trajectories and common infinity-norm shooting
and terminal error limits of 1e-8, physical excess <=1e-8 m, and input/cone
residual excess <=1e-8 in their declared physical residual units. Record strict
physical certificates separately from those tolerance checks. Budget and
tightening certificates must pass; a fail or uncertified result is not success.
The candidate gate rejects missing and nonfinite evidence. It never interprets
an optimizer's own row value as a certificate.

Keep native status, common feasibility and stationarity separate. Outer Max/Min
is schedule exhaustion, not optimality. Record DDP normalized KKT components,
complementarity, physical shooting gap and step norms; record SCP step,
virtual-control, virtual-buffer and penalty metrics. Report a common fixed
penalized prox-gradient probe as an additional diagnostic where applicable,
with its gamma, rho and variable scales fixed in advance. Native iterations
are not equal units of computational work.

For lifted SCP export, per interval: optimized accumulator increment, numerical
integration increment, independently certified increment and accumulator defect.
Do not substitute the first for the second or compare only maxima. For DDP
export numerical and certified increments, row residuals and shooting gaps.
Record configured margins, sufficient certified margins and remaining reserve.

Report success rate with all 32 starts in each cell's denominator, median final
objective/tf on the paired-success subset with its size, worst-case physical
clearance (including failures), and native iteration distributions among
successes. Report uncertified cases separately. Show objective, normalized
integral ratio and best feasible incumbent against iteration, keeping final
returned policy and incumbent distinct. Different local minima remain part of
the shared-start results; they are not proof of per-iteration solver efficiency.

One returned policy per cell; no test retries, tuning pool, best start, or
post-hoc replacement by an earlier feasible iterate. Report incumbent quality
separately if desired. The candidate creates 960 test runs across three arms,
five meshes, two time modes and 32 starts. It does not launch them.

Omit runtime from primary claims. A separate timing study must fix hardware,
thread counts and host load, repeat runs, exclude JIT/compilation, distinguish
observer/IPC overhead, and state the C++ versus Python/JAX/CVXPY implementations.

## Reproduce preparation

Run from this directory, using a new output directory:

```sh
python3 point_mass_paper_protocol.py --prepare /tmp/point-mass-paper-candidate
python3 -m unittest test_point_mass_paper_protocol
```

The manifest reports `not_ready_for_paper_runs` until the release gates above
are resolved and the final protocol is frozen. Preparing this manifest is not
evidence that the adapters or the 960 solver runs have been completed.
