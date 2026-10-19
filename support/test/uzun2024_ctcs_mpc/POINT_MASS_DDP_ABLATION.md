# Production regression, SOC/gate ablation and CTCS globalization

The strict barrier strategy is now **opt-in** through
`Param::use_strict_barrier_gate=true`. Production defaults restore
`inner_tolerance=10`, the legacy rho-weighted gate and schedule-on-cap behavior.
NT scaling remains active. Solver commit: `82b2da66`.
The legacy strategy is retained for compatibility, not certified as a suitable
paper profile. Neither production nor the matched paper campaign is declared ready.

## 1. Production was rebuilt and tested

Built sequentially: `comando_planner`, all six production test executables,
and the KKT regression. The final CTest suite uses these refreshed binaries.
The six test executables run from `/home/lab/mavros_ws/build/comando_planner/test_*`,
as registered in CTest, not older duplicate files under the devel package's
`lib` directory. The planner itself runs from the devel package directory.
Exact executable paths and hashes are archived.

| Check | NT + default legacy gate |
|---|---|
| trajectory_replayer | pass |
| planner_core | pass |
| stc_landing_noaug | fail: staging-hover cold solve rejected |
| stc_landing_noaug_ctcs | pass |
| direct_mpc | fail: moving-target policy expires |
| controller | pass |
| alipddp_kkt_check | pass |

Rebuilt the **same two failing tests** against an isolated old-SOC library,
with the same current OCP/controller sources and legacy gate. Both also fail
there. Staging-hover constraint excess is 1.65674 with old SOC and 0.0181517
with NT; both exceed the unchanged acceptance gate. Both direct-MPC variants
expire their moving-target policy. These are not observed pass-to-fail
regressions from NT in this comparison, but they remain real production failures.
This does not establish production robustness or rule out other NT regressions.

A single headless PX4 x500 smoke run used the rebuilt planner, an isolated ROS
master, the existing direct CTCS landing profile and documented thrust/rate
calibration. It ended **NO_SETTLE**: the staging hover was rejected by the
primal audit (`constraints=inf`, `dynamics=inf`), despite native termination
`Optimal Solution`. The landing profile was never activated. This is not a
landing pass or evidence about a completed landing's tracking. Cleanup was
restricted to owned process groups and completed cleanly. No production build
ran alongside SITL. Strict-gate production/SITL operation remains unvalidated.

## 2. Matched 2×2, normalized rows and CTCS GN on

All cells use the exact original warm seed, N=6, tf=4 s, the original fixed
epsilon/margins, 512 substeps, 1200 total iterations and 60 accepted inner
steps. The only factors are SOC elimination and the gate strategy. The old
SOC equations are restricted to an explicitly prepared research snapshot;
production has no switch back to the old SOC step.

| SOC | Gate | Native termination / iteration | True integral / budget | Physical excess (m) | Terminal infinity error |
|---|---|---|---:|---:|---:|
| old | legacy | Outer Max/Min / 299 | 1.50356e8 | 0.118050 | 1.81537e-5 |
| old | strict | Inner Iteration Limit / 93 | 8.86678e5 | 0.013090 | 5.26034e-5 |
| NT | legacy | Outer Max/Min / 255 | 7.73051e8 | 0.261142 | 0.034540 |
| NT | strict | Inner Iteration Limit / 84 | 9.20204e8 | 0.285159 | 0.035201 |

All four returned policies fail independent physical and budget certification.
The old-SOC/legacy-gate run exactly reproduces all original 301 optimizer
records, excluding timing: X/U, iteration, mu, rho and native optimality error.
Thus the new observer and isolated baseline do not perturb that reference run.

The factors interact: strict gating improves the old-SOC run's integral and
clearance results but does not make it feasible; NT removes the logged
factorization failures without improving this GN-on pilot's final feasibility.
Neither change alone solves this pilot. Native iteration counts have different
termination conditions; they are not a robustness or speed ranking.

## 3. The successful GN-off configuration was rerun

| SOC / gate / CTCS GN | Termination / iteration | True integral / budget | Objective | Returned common gate |
|---|---|---:|---:|---|
| old / legacy / off, frozen reference | Optimal / 541 | 0.9678163481 | 4.004202515041 | pass |
| NT / legacy / off, new run | Optimal / 640 | 0.9678163430 | 4.004202515041 | pass |
| NT / strict / off, new run | Inner Iteration Limit / 96 | 2.9582167849 | 4.004211132254 | fail |

The NT/legacy GN-off run passes rational full-policy path and integral
certificates on all six intervals; propagated terminal error is 2.64e-9.
The strict GN-off run remains physically clear, but its integral and terminal
checks fail. It ends at mu=1e-3 with the original 60-step inner cap. This is
not evidence that it would converge with a larger cap.

The previously successful GN-off configuration survives NT with the legacy
schedule. This is a useful regression result for this one warm start, not a
recommendation to disable GN generally or reuse a pilot-tuned paper profile.

## 4. Per-iteration slack/dual evidence identifies an accepted violation jump

The wrapper records row, slack, dual, c+s, s*y-mu and y/s for intervals 2–4,
plus mu/muT/rho/rhoT, X/U, native failures and actual gate settings. It uses
existing read-only getters; no production debug logging was added.

The seed's interval-2 normalized row is about 288526, although its physical
excess is only 9.7 mm. Initialization gives s=288526, y=3.47e-9 and c+s=577052,
with zero complementarity residual. At iteration 2 the density is zero but
s is still about 155000. Thus the seed is not close to feasible in normalized
CTCS primal coordinates, even though its physical penetration is small.
This is measured initialization evidence, not proof that initialization alone
causes the later failure.

With NT, strict gate and GN on, the following step is accepted at fixed mu=1e-3:

| Quantity | Iteration 49 | Iteration 50 |
|---|---:|---:|
| Recomputed barrier/AL merit | 3.987379507949 | 3.986640593843 |
| Primal residual sum | 9.86892529 | 1.768891815e9 |
| Independently integrated max integral/budget | 2.52395438 | 1.670219305e9 |
| Independent physical excess (m) | −0.000866752 | +0.384104623 |

The source's `forward_filter=1` rejects a trial only if BOTH

    error_new > (1-1e-5)*error
    merit_old-merit_new < 1e-5*error.

Here merit decreases by 0.0007389141065, exceeding the required
0.0000986892529. The rule therefore permits the huge residual increase. This
is an observed acceptance mechanism, not a hypothesis about numerical
quadrature or NT scaling. Independent propagation confirms the new path's
violation. Numerical-map and physical shooting defects are separate checks.

The candidate is also dominated by iteration 44, which has error 13.20645 and
merit 3.986145632038: both are smaller than at iteration 50, at the same fixed
barrier/AL parameters. The current test compares only against the immediately
preceding point; it does not implement a historical nondominance filter.

A second read-only observer recomputes the complete merit from physical cost,
slacks, AL variables and existing helpers at full precision. Its reference
replay matches every optimizer record. Native optimality is evaluated before
the trial; X/U, row and primal error in these records refer to the accepted
post-step point. Do not attribute the pre-step optimality error to that point.

![CTCS slack and dual evolution](../../../logs/point_mass/audits/production_and_ablation_20261020/ctcs_slack_dual_trace.png)

## 5. Existing stricter acceptance setting: useful diagnostic, not a repair

A paired run changes only `forward_filter` from 1 to 0. The existing mode 0
requires both nonincreasing error and nonincreasing merit. It avoids the large
violation jump and returns a physically clear path with zero independently
integrated density. However, it ends `Forward Pass Failed` at iteration 23,
with terminal error 6.92e-6, so it fails the common acceptance gate. Both modes
and failures are retained. Production's acceptance rule remains unchanged.

Consequently, simply selecting mode 0 is not a demonstrated fix. The next
solver investigation concerns globalization with a consistent feasibility
restoration/merit or historical filter, and its interaction with the measured
normalized-row initialization and GN model. No new strategy is silently enabled
in production or selected from these pilot outcomes for paper runs.

This reconnects to the original optimization question. Our direct prox-linear
pilot tests a merit containing gamma*sum[c]+ on the actual CTCS rows, with
gamma=1e-6 in normalized coordinates. For the iteration-50 candidate, interval
2 alone contributes over 1670 to that hinge penalty, while the physical
objective improves by only 9.61e-6. It cannot be accepted by that pilot's
sufficient-decrease test from iteration 49. DDP's recorded barrier/AL merit
instead uses independent slacks and the current-point acceptance rule above.
That is a concrete implementation difference in CTCS optimization; it is not
novelty of folding or a universal SCP/DDP convergence theorem.

## Evidence and remaining gates

[All configurations, source snapshot, binaries, production build/test logs,
SITL logs, 2×2 traces, certificates and acceptance diagnostics](../../../logs/point_mass/audits/production_and_ablation_20261020/).
The 42 focused Python tests and refreshed KKT checks pass. Production CTest
does not pass; it retains the two failures above. Research old-SOC targets are
disabled again in the build cache, and production remains linked to NT.

The paper protocol remains unfrozen. Calibration and all 960 paper runs remain
unstarted. Commit the final protocol hash before test outcomes, after the
production/readiness, globalization, integration and adapter gates are met.
