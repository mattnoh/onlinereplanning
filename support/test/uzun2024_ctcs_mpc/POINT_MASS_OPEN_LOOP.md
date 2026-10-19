# Local point-mass open-loop 2 × 2 comparison

**Paper correction:** Elango et al. Section 5, equations (47)–(49), already
provides non-augmented CTCS integral inequalities with variables X,U and no y.
Our direct interval-integral structure is not a distinction unique to our DDP.
That section's convex, fixed-time, linear-dynamics setting differs from our
nonconvex obstacle problem and free-time cases. See the correction in
[the optimization report](CTCS_OPTIMIZATION_DIFFERENCES.md).


## Optimization differences: direct research update (2026-10-20)

The original question concerns **optimization differences**, and the earlier
certificate work addressed the numerical-enforcement prerequisite. The new
[optimization report](CTCS_OPTIMIZATION_DIFFERENCES.md) now isolates optimization
mechanisms using actual C++ row/curvature evaluators from both DDP revisions,
source-extracted SCP stopping metrics and controlled paired convex subproblems.
[Raw results](../../../logs/point_mass/audits/optimization_structure_20261020_v2/results.json),
[research evidence](../../../logs/point_mass/audits/optimization_structure_20261020_v2/REPORT.md)
and source/binary/build/test hashes are kept together.

The concrete progress is:

- Exact accumulator condensation preserves the local physical step when every
  objective and constraint term is retained (nine paired problems; largest
  step difference 7.3e-12).
- SCP's accumulator proximal cost becomes an additional metric on physical
  steps after condensation. Its virtual-control penalty permits a hard
  increment row to coexist with an over-budget physical integral model.
  Both effects were isolated with matched local problems, including consistent
  density/weight scaling. They are not effects of exact folding itself.
- Twelve actual C++ comparisons have identical historical/current values and
  Jacobians. The current CTCS GN contribution has rank 3 in the active test,
  including two directions orthogonal to the integral gradient; the historical
  scalar-row slack contribution alone has rank at most one. This isolates a
  revision-dependent optimization difference, not a change in integral meaning.
- Both formulations inherit the physical integral's weak gradient near grazing.
  Measured integral/gradient exponents are approximately 2.5/1.5. Adding an
  accumulator identity derivative does not remove this reduced physical flatness.
- A pinned SCP implementation discrepancy was reproduced: CVXPY nu is already
  scaled, but J_vc applies the state scale again. A controlled S=10 case yields
  J_vc=0.01 instead of the once-scaled 0.1. This affects physical states with
  nonunit scales; the user accumulator's scale is one. It has not been attributed
  as the cause of any particular historical run, and no upstream fix was made.

The report distinguishes these measured mechanisms from hypotheses about
convergence speed. A controlled full-solver performance ranking is still open.
All 32 tests passed. Only the read-only evaluator and research tooling were
extended; no trajectory optimizer campaigns, controller changes, production
quadrature changes or benchmark-pool edits were made.


## Certified folded/lifted review (2026-10-20)

The saved-policy extension is implemented and run on all 40 frozen selections.
[Certificate report](../../../logs/point_mass/audits/folded_lifted_certificate_20261020_v7/REPORT.md),
[summary](../../../logs/point_mass/audits/folded_lifted_certificate_20261020_v7/certification.json),
[matched derivative cases](../../../logs/point_mass/audits/folded_lifted_certificate_20261020_v7/matched_cases.json)
and the adjacent per-policy JSON files preserve interval-local and propagated
rational enclosures, independent reference integrals, native SCP reintegration,
accumulator defects, actual historical C++ row values, derivatives and four SCP
finite-difference step sizes. The versioned comparison now extends its original mirror with certificate flags;
the external live benchmark remains unchanged. No optimizer, production quadrature, margins,
budgets, controls, selector thresholds or plots were changed.

The real C++ evaluator reproduces zero at 4 and 8 substeps for free-time DDP N=6
interval 3. Exact-flow sampling at those same stage times also returns zero;
both grids miss the positive pocket. A rational enclosure proves its integral
exceeds 1e-11, and independently split quadrature returns about 3.60518e-10.
At 16 substeps, production RK4 gives about 2.93489e-10 while exact flow at the
same stage times gives about 9.66261e-13: intermediate-stage approximation is
a separate error channel. Exact RK4 endpoints do not make stage states exact.
The N=6 physical-clearance certificate passes separately.

All 20 CTCS policies receive strict obstacle/glide path-clearance certificates;
19 meet their physical integral budgets and N=6 fails. Thirteen satisfy the
reported conservative sufficient-tightening test and seven fail that test.
Failure of that sufficient test does not imply penetration. Strict input
residuals and the original tolerance-based base gate are reported separately:
11 serialized policies have positive input residuals, all below the existing
1e-6 audit tolerance (largest upper bound 2.32548e-7 m/s²).
The fixed-time stage DDP N=100 path remains uncertified at exact zero rather
than being promoted from its tolerance-based grade.

The certificate uses exact rational constant-acceleration flow, quadratic and
quartic Bernstein bounds, and integer-square-root brackets. It reports
configured/sufficient margins and reserve in metres, the theorem's small-budget
condition, certified speed bounds and propagated residual error. Binary time
dilation retains exact duration s/N. Historical CSV bounds distinguish the
serialized doubles from conditional original-policy bounds using the recorded
12-significant-digit export format, decimal conversion and accumulated error.
Limits are 100,000 subdivisions and depth 60; ambiguous cases remain uncertified.

SCP's optimized accumulator increment, actual native Dopri8 increment and
certified true increment are distinct per-interval fields. JAX and finite
differences are both retained, including sensitivity near narrow regions;
no derivative agreement is presumed. Stage virtual-buffer penalties start at
the pinned default zero. Their adapted values were not recorded in the saved
arrays and cannot be recovered from J_vb alone. The report records that limit
and distinguishes augmented-Lagrangian from constant-weight settings.

Source archives, working-tree status, base commits, binaries, build caches and
hash inventories preserve historical point-mass DDP versus current paper-MPC
DDP attribution. Dirty historical sources are explicitly recorded; their base
commit is not presented as an exact reconstruction of an unknown old build.
The MPC revision's CTCS Gauss–Newton curvature affects optimization rather than
the definition of the folded integral. Exact accumulator elimination introduces
no mathematical relaxation when density, flow, time measure and budgets match.

The new sequential C++ target builds through catkin without changing either
solver checkout:

```bash
source /opt/ros/noetic/setup.bash
catkin build comando_planner --no-deps -j1 -p1 \
  --cmake-args -DCOMANDO_BUILD_POINT_MASS_DIAGNOSTIC=ON \
  -DCOMANDO_POINT_MASS_DDP_SOURCE=/path/to/historical/point-mass-ddp \
  --make-args point_mass_saved_policy
```

Run the existing comparison entrypoint against a frozen snapshot and a new
output directory (paths below are relative to the planner repository):

```bash
/home/lab/miniforge3/envs/openscvx/bin/python \
  support/test/uzun2024_ctcs_mpc/point_mass_compare_fair.py \
  --certify-snapshot logs/point_mass/open_loop \
  --certificate-output /absolute/path/to/NEW_CERTIFICATE_DIRECTORY \
  --ddp-evaluator /home/lab/mavros_ws/devel/.private/comando_planner/lib/comando_planner/point_mass_saved_policy \
  --native-scp
python3 -m unittest discover -s support/test/uzun2024_ctcs_mpc -p 'test_*.py'
```

All 26 Python tests pass, including the saved N=6 regression. The historical
CTCS self-check reports exact value/cache agreement and completes its derivative
step sweep. The paper-model C++ derivative check passes. All 394 original
snapshot hashes were verified unchanged. Earlier diagnostic attempts remain
alongside the final evidence with ATTEMPT.txt explanations. This nominal-model
gate excludes flight uncertainty, MPC reset/shift/tail guarantees and nonlinear
landing guarantees.


## Updated plots at the required 1e-6 dynamics threshold

[Free-time PNG](../../../logs/point_mass/open_loop/point_mass_2x2_free.png)
([PDF](../../../logs/point_mass/open_loop/point_mass_2x2_free.pdf));
[fixed-time PNG](../../../logs/point_mass/open_loop/point_mass_2x2_fixed.png)
([PDF](../../../logs/point_mass/open_loop/point_mass_2x2_fixed.pdf)).

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
`logs/point_mass/audits/threshold_reconciliation_20261019/`. No optimizer was run
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

Evidence directory: `logs/archive/point_mass_history/point_mass_open_loop_2x2_20261019_v3/`.
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
logs/point_mass/audits/free_scp_strict_review.json. The correct next diagnostic is
nonlinear-residual-aware stopping and accumulator conditioning, with physical
limits, tightening and epsilon held fixed. No new optimization was authorized
or run by this explanatory review.


## Folded versus lifted CTCS: evidence and numerical guarantee

2026-10-19 follow-up is a read-only mathematical/interface review, not another
optimization campaign or an authorization to resume landing experiments.
Primary reference re-read locally: Elango et al. (2024), Section 3.3,
Theorem 14, Remark 15 and Appendix A, from
`/home/lab/Zotero/storage/FGP9CC2S/Elango et al. - 2024 - Successive Convexification for Trajectory Optimization with Continuous-Time Constraint Satisfaction.pdf`.

### Exact equivalence and the scope of the evidence

With identical physical flow/control parameterization, density, time measure
and positive interval budgets, the lifted formulation has

    y[k+1] - y[k] = I_k,  y[k+1] - y[k] <= B_k.

Eliminating y gives I_k <= B_k, exactly the folded constraint. This is the
construction in the paper and our Uzun SCP/DDP drivers. One horizon accumulator
with per-interval increments has the same feasible set as those folded rows
when its dynamics hold. A single total-horizon budget is a different feasible
set and must not be substituted silently. Shared versus per-consequence budgets
also change the feasible set unless their allocation is matched.

The successful full paper MPC runs and matched point-mass policies support the
correctness/usefulness of this construction. They do not certify all numerical
configurations. The historical free-time DDP N=6 policy still has independently
replayed I=3.60518082e-10 against B=1e-11 (36.05 times B), despite physical
clearance. Its source/grade history needs an isolated reproduction before
assigning a cause; it is not proof that algebraic elimination is invalid.

### Numerical error channels differ

Let d_y = (y[k+1]-y[k]) - I_k be a lifted accumulator dynamics residual, using
unscaled physical integral units. A lifted increment bound can permit
I_k <= B_k + positive increment-row residual + |d_y|. Folding removes this
extra accumulator variable/defect channel, but still relies on numerical
quadrature and the approximate physical flow. Both also need full-rollout
consistency across physical multiple-shooting gaps. Algorithm stopping flags
alone do not bound all of these quantities.

For a folded normalized row Qhat_k/B_k-1 <= rho_k, the computed integral obeys
Qhat_k <= B_k*(1+rho_k). Thus a conservative true local-integral upper budget is

    B_eff,k = B_k*(1+max(rho_k,0)) + eta_eval,k,

where eta_eval bounds quadrature, nominal numerical-flow and roundoff error in
integral units. Lifted SCP needs its increment-row tolerance and accumulator
defect included after undoing the y/density scaling. This is error accounting,
not increasing the configured optimization budget B_k. Report B_k and B_eff,k
separately. Actual residuals should be used; an aggregate solver tolerance is
not automatically an integral-error bound.

For physical residual c_i measured in its own units, write normalized residual
h_i=c_i/s_i, with s_i>0. Suppose the true local density is
sum_i [h_i+delta_i]_+^2. Let L_i bound |d h_i/dt| on that continuous interval.
The paper gives the sufficient pointwise bound

    max(h_i + delta_i) <= (4*L_i*B_eff,k)^(1/3)

provided B_eff,k <= L_i^2*Delta_t_min^3/4. Check this condition using a valid
positive interval-duration lower bound, including free time. If the density has
weights w_i, the component budget in this expression is B_eff,k/w_i. Equality
constraints need a violation bound; inequality tightening does not establish
exact equality satisfaction.

If the globally propagated/implemented residual differs from the exact local
model residual by at most E_i,k, a sufficient inequality tightening is

    delta_i >= (4*L_i*B_eff,k)^(1/3) + E_i,k.

The physical tightening is s_i*delta_i. E_i,k must be obtained by bounding
propagated root/shooting errors; a flight guarantee additionally needs bounded
model/tracking/timing errors. A local gap tolerance alone is not that bound.
Avoid double counting errors already included in eta_eval. Verify error bounds
for the actual chosen margin. For L_i=0 or when the theorem's small-budget
condition is unavailable, do not use the cubic expression unconditionally:
a universally sufficient Lipschitz bound is sqrt(B_eff,k/Delta_t_k)+L_i*Delta_t_k,
which is conservative; exact interval extrema can provide a tighter certificate
for this point-mass model.

### What must be kept straight in our interface

For the point-mass normalized obstacle residual, L_obs <= V/R; for its glide
residual, L_glide <= sqrt(2)*V/R. These recover the existing physical margin
rule R*(4*B_k*L_i)^(1/3) in exact arithmetic. Under ZOH point-mass dynamics,
velocity is linear on each interval, so its norm maximum occurs at an endpoint;
exact replay node velocities can validate V a posteriori for the returned
policy. The initial guessed V is not a certificate until this check passes.
The current speed-refinement loop checks V after solving, but does not supply
certified quadrature/flow/roundoff bounds or a complete numerical certificate.

Fixed-time point-mass rows currently constrain an interval MEAN SQUARE,
(1/Delta_t_k)*integral sum_i [h_i+delta_i]_+^2 <= epsilon.
Consequently their physical integral budget is B_k=epsilon*Delta_t_k.
Free-time rows use the plain integral, B_k=epsilon. Use physical time: in the
SCP normalized time coordinate, dy/dtau = s(tau)*density; DDP's duration action
must carry the identical Jacobian/time factor. Numerical density scaling is
reversed before interpreting budgets or residuals. Changing duration, units,
normalization or density weights without these conversions changes the
problem or invalidates the intended margin calculation.

The Uzun obstacle benchmark uses its paper margin delta=.05; that successful
profile does not by itself establish a deterministic margin for landing speed,
tilt, angular-rate, glideslope or LOS residuals. Those need their own scales,
derivative/domain bounds and numerical-error allowances. Near touchdown,
spatial cone/LOS residuals with bounded derivatives need explicit treatment;
do not transfer an obstacle's dimensionless margin to an angular/length limit.

### Recommended next investigation, retaining the folded formulation

1. At identical states, controls and durations, compare lifted and folded true
   integrals and first derivatives, especially duration derivatives and fixed-
   versus free-time normalization. Treat this as an interface equivalence test.
2. Isolate the saved N=6 DDP discrepancy; separate transcription/configuration,
   grading-version, quadrature and physical-flow errors before changing a solver.
3. Add per-interval B_eff and per-consequence physical-margin diagnostics to the
   existing auditors. First use this point-mass problem's exact flow/extrema.
   Dense sampling and M-versus-2M agreement are useful evidence, not certified
   numerical error bounds. A strict deterministic certificate needs validated
   error enclosures or analytic bounds, including rounding, over each interval.
4. Extend that numerical contract to the nonlinear paper model, then to landing
   when its work is resumed. Hold physical limits and configured budgets fixed.

No optimizer, controller or tightening configuration was changed by this review.
