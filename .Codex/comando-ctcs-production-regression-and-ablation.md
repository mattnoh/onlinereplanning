# CTCS production regression and factor separation

User requested production rebuild/tests, strict gate behind a default-off flag,
the old/NT SOC x legacy/strict gate 2x2, the formerly successful GN-off case,
and only then fixed-mu diagnostics for CTCS intervals2–4.

Completed:

- ALIPDDP commit82b2da66 restores inner_tolerance10/default legacy strategy;
  use_strict_barrier_gate opts into the stricter strategy. NT remains active.
- Rebuilt comando_planner and all six production tests sequentially, plus KKT.
  Final fresh CTest:5/7 pass; stc_landing_noaug and direct_mpc fail. Isolated
  old-SOC/legacy baseline rebuilds fail those same tests. No thresholds changed.
- One isolated-ROS headless SITL run: NO_SETTLE before landing; staging hover
  fails primal audit despite native Optimal. Owned-process cleanup completed.
- 2x2 GN-on normalizedN6: all fail. Old-SOC/legacy exactly reproduces301
  original optimizer records. NT/legacy GN-off still succeeds (Optimal640,
  trueI/B .967816343, objective4.004202515041, strict certificates pass).
  NT/strict GN-off ends inner cap96, fails budget/terminal. No best-of selection.
- Instrumentation uses existing read-only getters in the research wrapper.
  It records CTCS c,s,y,c+s,s*y-mu,y/s for intervals2–4, gate/settings/X/U.
- Fixed-mu jump49->50: error9.8689 ->1.76889e9 while barrier/AL merit decreases
  .000738914 >required .0000986893. Existing acceptance rule permits it.
  Earlier44 dominates candidate50 on both error and merit. Independent replay
  shows physical excess-.867mm ->+.384m. The native optimality log is pre-step.
- Existing forward_filter0 avoids the jump but stalls at23; terminalerror6.92e-6
  means no common acceptance. Production globalization remains unchanged.
- 42 focused Python tests and refreshed KKT pass. Research old-SOC build flag
  restored OFF. Paper protocol remains unfrozen and 960 runs unstarted.

Evidence: logs/point_mass/audits/production_and_ablation_20261020.
Maintained report: support/test/uzun2024_ctcs_mpc/POINT_MASS_DDP_ABLATION.md.

Remaining work: principled fixed-barrier globalization/restoration and initialization
investigation, production failures and staging/SITL readiness, then independent
calibration/adapter checks before protocol freeze. Do not equate the SOC issue
with CTCS folding, claim all production tests passed, or enable strict gating
by default from the one successful GN-off pilot.
