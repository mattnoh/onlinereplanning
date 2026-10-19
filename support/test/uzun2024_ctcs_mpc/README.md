# Original Uzun 2024 CTCS MPC: SCP reference and DDP reproduction

Implemented and executed the paper's original **2D obstacle problem**, before
returning to landing. All reported campaigns use synchronous execution: plant
time is frozen during optimization, then the first K optimized intervals are
applied with their specified hold. There is no ROS or tracking controller.

The matched **SCP/DDP × CTCS/nodal 2×2** is now implemented and executed.
See [RESULTS.md](RESULTS.md) for the four-case plot, tracking/clearance/timing
table, remaining SCP nonlinear residuals and exact reproduction commands.

## Verified results (2026-10-19)

Every row below completed 30 simulated seconds / 15 replans. Physical violations
are independently audited against all five untightened obstacles, using DOP853
with steps <=1 ms, event crossings and refined intersample peaks.

| Case | Mean tracking [m] | Physical excess / duration | Max executed integral |
|---|---:|---:|---:|
| SCP FOH, 3 iterations | 0.746276 | 0 / 0 s | 2.22612611e-05 |
| SCP FOH, 10 iterations | 0.400330 | 0 / 0 s | 1.26984781e-06 |
| SCP ZOH, 10 iterations | 0.403897 | 0 / 0 s | 1.26406637e-06 |
| DDP ZOH, fresh solver | 0.250053 | 0 / 0 s | 9.99960093e-07 |
| DDP ZOH, retained solver | 0.250058 | 0 / 0 s | 9.99933028e-07 |

The assumed per-interval integral budget is **1e-6**. The SCP three-iteration
and ten-iteration runs achieve physical avoidance but do **not** pass that
strict integral-budget check; their saved summaries correctly remain `fail`.
Do not present small nonlinear/accumulator defects as exact feasibility.
Both DDP runs pass the executed physical, input and integral-budget checks;
all 15 solves in each report `Optimal Solution` with zero model dynamics gaps.
The retained run actually calls `updateProblem` 13 times, with initialization
at startup and when the empty sensed set first changes the stage structure.
Its maximum executed prediction/plant discrepancy is 1.70e-9.

This is not a solver-performance ranking: DDP has a 300-iteration cap, whereas
the SCP runs deliberately use 3 or 10 subproblems. It shows that **folded
per-interval CTCS and retained-state DDP MPC work on this original example**.
It does not fix or validate the demanding landing OCP. FOH and ZOH give nearly
the same physical result in the matched SCP ten-iteration comparison.

Evidence: `../../../logs/point_mass/paper_mpc/` contains `comparison.json`, complete
configs, source snapshots/hashes, per-solve trajectories and seeds, convergence
and defect diagnostics, applied controls, and independent dense replays. P1/P2
failed conditioning experiments are retained and identified in the authoritative
`.Codex/comando-constraint-satisfaction-control.md` record. Their lack of obstacle
penetration was not a useful tracking success.

## Paper formulation and explicit assumptions

Local source: Uzun et al., IFAC PapersOnLine 58-18 (2024), pp.421-429,
*Successive Convexification for Nonlinear Model Predictive Control with
Continuous-Time Constraint Satisfaction*, Zotero attachment `NR6G7V8D`.

Paper Table 1 / Algorithm 1: horizon8 s, 17 nodes (16 intervals of .5 s), K=4,
30 s simulation, 3 SCP iterations; norm(u)<=5; drag coefficient .05;
five ellipses with S=diag(2.4,.4), centers (-6.5,-3),(-1.8,3.5),(.5,-3.5),
(2.7,1.5),(6.2,-3); shape-scaled sensing radius6. Reference:
`[-10+2t/3, 5*exp(-.05t)*sin(pi/2+t/5+t*t/36)]`.
Running cost is the time integral of squared position tracking error.

**Tightening is included:** density is
`sum_sensed max(0, 1 - norm(S*(r-center)) + .05)^2`.
SCP carries one horizon y state with hard convex cross-node increment bounds
`y[k+1]-y[k]<=epsilon`; no global y<=epsilon box is introduced. DDP eliminates
y using the existing folded stage interface `I_k/epsilon-1<=0`.
Sensing is fixed at each measured root for that horizon. Cost and integral
states are reset/recomputed at every solve, shifted tails are integrated with
the old last input, and the loop stops at T rather than executing beyond T.
The input norm constraint is continuous under either matching FOH or ZOH.

The paper does not report epsilon, numerical weights/scaling, the initial
state/first seed, or the integration settings. Our explicit assumptions are:

- epsilon1e-6; initial [r,v]=[-10,0,0,0]; first controls zero, with states/integrals
  rolled out consistently; zero terminal cost and no terminal constraints.
- SCP fixed proximal weight1 and L1 penalty1e4; physical states/controls/y have
  identity scaling, l scale100 with objective coefficient compensated100.
  This preserves the physical objective and changes the numerical weights on
  l. Identity l scaling stalled useful early controls; reducing every proximal
  weight instead caused large nonlinear defects. These are unreported settings,
  not recovered author values. Native OpenSCvx/CLARABEL float64, Dopri8 1e-10.
- DDP RK4 with 100 substeps/interval for matched runs, integrated tracking cost,
  existing SOC input and folded CTCS interfaces, maximum300 iterations. All
  options and barrier/penalty settings are saved per solve. No solver algorithm
  source or control clipping was added.
- Physical audit tolerance1e-6 for this original obstacle benchmark and useful
  mean tracking<=1 m are our explicit acceptance checks. Integral budgets,
  future-node/full-propagation audits and convergence are reported separately.

The main implementation is `scp.py` (full native SCP OCP + MPC), `ddp.py` (same
MPC protocol around the C++ OCP), and solver-independent `common.py` (physical
model/replay/audit). The complete DDP OCP is one solver-repository file:
`problem_examples/uzun2024_ctcs_mpc.cpp`.

## Faster DDP CTCS profile

Verified profile: `--substeps 50 --flow-cache shared`. Complete30s MPC and
full retained-request parity checks preserve all15 converged solves, zero
observed physical violation and the unchanged integral budget. Current paired
replay median improves826ms→224ms; total solver CPU improves2.50×. See
[RESULTS.md](RESULTS.md) for timing scope, residuals and rejected20-step case.
Reference defaults remain100/legacy. No shared solver algorithm changed.
Optional verified intermediate stopping adjustment:
`--solver-arg=--inner-tol --solver-arg=100`. Final tolerance remains1e-6;
all15 solves and independent physical/budget audits pass. Separate paired
measurements show median165→140ms (15% lower), with little worst-case gain.
See RESULTS.md for the distinct timing conditions and landing-failure evidence.

```bash
python ddp.py --reuse-solver --substeps 50 --flow-cache shared --output-dir NEW_FAST_CTCS
```

## Reproduction

Use `/home/lab/miniforge3/envs/openscvx/bin/python` and run campaigns sequentially.
Each output directory must be new. `OPENSCVX_ROOT` can override the checkout
(default `~/projects/solver/SCP/OpenSCvx`).

```bash
python scp.py --hold FOH --iterations 3 --cost-scale 100 --output-dir NEW_SCP_3
python scp.py --hold FOH --iterations 10 --cost-scale 100 --output-dir NEW_SCP_10
python scp.py --hold ZOH --iterations 10 --cost-scale 100 --output-dir NEW_SCP_ZOH
bash build_ddp.sh
python ddp.py --hold ZOH --substeps 100 --output-dir NEW_DDP_FRESH
python ddp.py --hold ZOH --substeps 100 --reuse-solver --output-dir NEW_DDP_RETAINED
```

The build script uses one catkin target with `-j1 -p1`, enables only
`BUILD_UZUN_PAPER_MPC`, and clears the obsolete temporary E13 linker override.
It does not modify the shared solver. Previously compiled planner binaries
were not relinked by this target and are not claimed as new baseline evidence.

`--reuse-solver` uses the same `ALIPDDP::updateProblem` path as production.
A finite but primal-infeasible DDP result is saved and stops execution; a small
stationarity residual is not required as an alternative to physical checks.
The C++ streaming protocol is diagnostic-only, not a ROS interface change.

## Verification

18 shared model/audit tests pass, including analytic drag response, derivatives,
sensing, tightening, constant-control FOH/ZOH equivalence, quadrature, geometry,
and persistence of failed results. The C++ `--check-derivatives --substeps 100`
checks the map, cost and active folded constraint against central differences.
Its near-budget integral agrees with independent SciPy to 1.12e-10, flow to
1.78e-15 and cost to 3.98e-13. Saved initial active-density checks also pass.

```bash
python -m unittest discover -s . -p test_common.py -v
/home/lab/mavros_ws/build/comando_planner/alipddp/uzun2024_ctcs_mpc --check-derivatives --substeps 100
```

## Nodal comparator and figures

`scp.py --constraint-mode stage` and `ddp.py --constraint-mode stage` replace the CTCS interval constraints with
ordinary `g_i + delta <= 0` constraints at every selected stage and terminal
node. It preserves the same dynamics, integrated objective, ZOH grid, sensing,
initial state, limits, seeds and retained-solver lifecycle. The independent
interval integrals remain diagnostics; they are not enforced in stage mode.
SCP uses native nonconvex `.at(all_nodes)` constraints, linearized by OpenSCvx,
and omits its obstacle `y` accumulator completely. The library clock CTCS is
unchanged and identically inactive; it does not enforce obstacle clearance.

The same-tightening comparator (`--delta .05`) is P8; an explicitly separate
usual nodal baseline (`--delta 0`) is P9. The P8 run has feasible node rows
throughout yet violates the physical obstacle constraints between nodes. Its
worst interval [12,12.5] s is from a solve reporting `Optimal Solution`, so the
intersample failure is not inferred from a failed optimization. Full results,
convergence caveats, plots and commands are in [RESULTS.md](RESULTS.md).

```bash
python scp.py --constraint-mode stage --hold ZOH --iterations 10 --delta .05 --output-dir NEW_SCP_STAGE_005
python ddp.py --constraint-mode stage --delta .05 --reuse-solver --output-dir NEW_STAGE_005
python ddp.py --constraint-mode stage --delta 0 --reuse-solver --output-dir NEW_STAGE_0
python plot_comparison.py --output-dir ../../../logs/point_mass/paper_mpc/comparison_plots
```

Figures are standalone PNG and PDF; their exact dense and node series plus
statistics are exported as CSV/JSON. Physical `max_i g_i(t)`, nodal feasibility,
and tightened integral feasibility are separate quantities. The DDP CTCS
result continues to work on this benchmark; the demanding landing case remains
unfixed and cannot be declared solved from these plots.


## Separate local 3-D point-mass open-loop comparison

[POINT_MASS_OPEN_LOOP.md](POINT_MASS_OPEN_LOOP.md) records the local point-mass
DDP/SCP × stage/CTCS comparison and preserves its original plotting style.
This is a separate six-state sphere-avoidance OCP; its saved-policy comparisons
are not the two-dimensional Uzun MPC campaign above. The versioned local
comparison mirror supports `--plot-snapshot NEW_DIRECTORY` without solving.
