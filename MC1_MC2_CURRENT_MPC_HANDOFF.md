# Handoff: rerun MC1/MC2 with current folded cSTC-DDP MPC

Prepared 2026-10-21. This document authorizes the next working session to perform the campaign, analysis and thesis revisions described below. Its creation did not launch experiments or change the controller.

## Objective and author intent

Rerun the original MC1 and MC2 scenario campaigns with the current improved folded cSTC-DDP MPC. The motivating problem was physical constraint violations during execution. Measure whether the improved STC/CTCS handling and measured-state MPC reduce those violations while maintaining landing completion and accuracy.

Rewrite the thesis SITL results around **MC1 and MC2 only**. Remove the separate `원형 목표 비행` subsection. The current MPC is the method used throughout the new campaigns. Preserve the hardware section and the five-chapter structure plus Appendix A.

Complete configuration verification, necessary sequential builds, SITL execution, physical scoring, figures, manuscript revisions and PDF verification. Do not stop after proposing a plan. Do not assume the new results will improve; report the actual outcomes.

## Read first and preserve ongoing work

- Planner: `/home/lab/mavros_ws/src/CoManDO_planner/AGENTS.md` and `README.md`.
- Solver sibling: `/home/lab/mavros_ws/src/ALIPDDP-main/`; read its applicable instructions before changing anything there.
- Thesis: `/home/lab/paper/2026_학위논문/AGENTS.md`.
- Shared thesis context: `main/shared/CTCS_MATH_BASE.md` and `main/shared/SESSION_SYNC.md`.
- Landing evidence indexes: planner `logs/README.md` and `logs/landing/README.md`.

Inspect git status in all relevant repositories. Another session has uncommitted thesis edits and point-mass comparison scripts; preserve these. Follow the repositories' checkpoint instructions while respecting ownership of concurrent work. Do not blindly stage all modified files or commit another session's active work without coordinating ownership.

This handoff was created in a side conversation. Only this Markdown file was added by that handoff task; the previously modified files belong to existing work.

## Commit regularly — explicit user instruction

Use focused commits at meaningful checkpoints:

1. Before launching the campaign: commit the tested runner/configuration changes, frozen experiment definition and reproducibility notes. Record the exact solver/planner revisions and effective configuration hash.
2. During execution: checkpoint resumable trial accounting and compact manifests after each substantial completed batch, approximately every 10–20 trials or completed scenario group. Keep partial results explicitly marked partial.
3. After MC1 and MC2: commit each campaign's completed accounting, analysis code and compact result summaries.
4. After figures and manuscript edits: commit the synchronized Korean/English/Markdown changes and final verification notes.

Commit only task-owned changes using explicit paths. Keep large bags and raw binary outputs in the existing experiment storage; commit references/manifests according to repository practice. Preserve failures. Record cross-repository revision references so results can be reproduced. Avoid long stretches of uncommitted work, including while campaigns are running.

## Verified locations

### Current planner, execution and scenario code

All paths below are relative to `/home/lab/mavros_ws/src/CoManDO_planner/`:

| Path | Role |
|---|---|
| `support/scripts/run_scenario_campaign.py` | Sequential, resumable PX4/Gazebo campaign runner; motion/noise dictionaries and trial metadata |
| `support/scripts/scenario_metrics.py` | Campaign completion, landing and physical metrics |
| `support/scripts/analyze_direct_mpc.py` | MPC execution and physical/predicted constraint analysis |
| `support/tools/analysis/constraint_audit.cpp` and `.hpp` | Physical constraint evaluator and audit support |
| `support/transport/ros1/launch/direct_mpc_sitl.launch` | Direct-MPC startup and plant calibration |
| `config/ros1/profile_stc_landing_mpc_sitl.yaml` | STC MPC profile |
| `config/ros1/tuning_stc_mpc_sitl.env` | Saved improved startup settings that produced zero measured excess in two circle flights |
| `src/controller_internal.hpp` | Measured-root initialization, warm starting, solve acceptance, input holding and replanning |
| `include/ocp/ocp_stc_landing_mpc.hpp` | STC landing MPC problem |
| `include/ocp/ocp_landing_mpc.hpp` | Ordinary landing MPC context |
| `support/transport/ros1/src/target_publisher.cpp` | Target motion, randomization and planner-facing target estimates/noise |

**Critical runner pitfall:** `run_scenario_campaign.py` defaults to the stc_landing_noaug measured-state profile. Pass the intended MPC profile explicitly with `--profile`.

Startup environment parameters are read when the planner starts. Applying a ROS profile afterward does not reload them. Launch staging/LOS arguments can override environment values.

### Original MC1/MC2 data and definitions

Original data still exist at:

- MC1: `/home/lab/mavros_ws/src/comando_planner_old/logs/mc1_motion_20260825_2313/`
- MC2: `/home/lab/mavros_ws/src/comando_planner_old/logs/mc2_uncertainty_20260820/`
- Each has `samples.csv`; inspect cell metadata, seeds, source/configuration snapshots and bags for the exact trial definitions.

Historical scripts:

- `/home/lab/mavros_ws/src/comando_planner_old/scripts/run_scenario_campaign.py`
- `/home/lab/mavros_ws/src/comando_planner_old/scripts/scenario_metrics.py`
- Plotters alongside them: `plot_campaign_mc.py`, `plot_scenario_campaign.py`, `plot_campaign_box.py`.

Treat historical scripts/data as references. Run the current planner and current controller for the new experiments. Do not overwrite historical results.

### Improved MPC evidence and reproduction details

Current retained evidence, relative to the planner repository:

- `logs/landing/ros1/mpc_stc_speed_margin_validation/circle__zero__t0/`
- `logs/landing/ros1/mpc_stc_speed_margin_repeat/circle__zero__t0/`
- Detailed explanation and known limitations: the repeated run's `tuning_report.md`.
- Physical scoring environment: `logs/landing/ros1/mpc_stc_speed_margin_repeat/physical_reference_environment.json` (confirm actual location from the report).

Important per-run files include `planner_environment.json`, `landing_profile.yaml`, `source.json`, `cell.json`, `constraint_audit_reference.json`, `mpc_execution_check.json`, `direct_mpc_controls.csv`, `all_solves.csv`, plots and the finalized bag.

Recorded results:

- Both circle flights completed `LANDED / CLEAN`.
- Zero measured executed-trajectory excess in all eight audited categories.
- Accepted/rejected solves: 167/0 and 222/0.
- Warm median solve times: 53.613 and 53.380 ms.
- Repeated flight: new first-input updates approximately 17.58 Hz; hold median/max approximately 60.0/90.1 ms; command publication 100 Hz.
- These two runs use seed 8570001. Randomized target geometry was approximately radius 0.514 m and angular frequency 0.407 rad/s.

These successful runs establish a working circle MPC configuration. The same evidence retains predicted-horizon defects and unsuccessful other-motion tests. MC1/MC2 are intended to measure performance across the broader scenarios and noise levels.

Historical source identities for those saved circle flights are planner `e2f3fdd` and solver `2789d974`. They identify saved evidence, not necessarily today's source or binary. The current README also describes later solver/check changes and stale-binary concerns. Reconcile current source, build and effective settings before freezing the new campaign. Do not silently run an old binary while attributing it to newer source, or rebuild a different solver and assume it reproduces the saved configuration.

## What is new since the old campaigns

The saved improved configuration includes tighter planning limits and earlier activation, increased warm iteration allowance, revised weights and measured-state/sub-grid warm-start behavior. Consult the preset and tuning report for exact values; do not infer them from the thesis alone.

Examples from the saved preset:

- 40 intervals, cold iteration limit 120 and warm iteration allowance 24.
- Raw CTCS interval allowance `1e-5`, with the documented scaling convention.
- Planning activation height 1.0 m; landing speed 0.5 m/s; angular rate 0.012 rad/s.
- Planning glideslope/LOS half-angles 12/25 degrees; staging floor 1.5 m.
- Source the preset before startup; explicitly pass staging 1.5 m and LOS trigger 1.2 m as launch arguments where required.

These are planning settings. Score physical behavior with the appropriate ORIGINAL campaign limits and activation definitions. In particular, the circle tuning report retains physical landing activation 0.8 m, speed 0.75 m/s, rate 0.019 rad/s, tilt 0.95 degrees, cones 15/30 degrees and staging height 1.3 m. Verify original MC1/MC2 scoring definitions in their own metadata rather than assigning one audit setup to every study.

Do not import the point-mass paper's 512-substep setting into this landing campaign.

## Campaign protocol

### Freeze before running

- Recover MC1/MC2 trial definitions and create explicit trial manifests.
- Reuse original seeds, target randomization, initial conditions, noise realizations and timeout rules wherever recoverable.
- Keep the new controller configuration fixed across both campaigns. Scenario/noise changes must not restore historical optimizer settings.
- Record actual startup environment, ROS profile, launch overrides, source/build identity and scoring settings.
- Reproduce the working circle startup once with the intended binary before committing to hundreds of trials. Keep this commissioning run outside the MC1/MC2 denominators.
- Check runner resume logic: a partial solver CSV alone is not evidence of a completed trial. Resume only when completion/status metadata and configuration identity match. Preserve incomplete attempts and identify infrastructure retries.

### MC1

Run six motions with 50 trials per motion, 300 total:

`straight`, `circle`, `figure8`, `stop_and_go`, `piecewise_accel`, `random_accel`.

Recover the original uncertainty level and scenario sampling from the original campaign. The recorded MC1 is the motion-generalization study; do not accidentally run the full motion × three-noise cross-product.

### MC2

Run the original circular-motion uncertainty study: 20 trials at each of `zero`, `nominal`, `high`, 60 total.

Recorded nominal noise standard deviations are `(0.02 m, 0.05 m/s, 0.10 m/s²)`; high values are `(0.10 m, 0.25 m/s, 0.50 m/s²)`. The campaign injects horizontal target-state noise plus a per-trial fixed bias with standard deviation `0.5 × sigma`; ground-truth scoring remains uncorrupted. Verify these details in the original target publisher and metadata, including seeds and timing.

### Execution discipline

- Build one executable at a time with the repository's `-j1 -p1` rules.
- Run SITL sequentially and use the installed PX4 binary; do not rebuild PX4 for every cell.
- Write new outputs under `logs/landing/ros1/` in clearly named new MC1/MC2 directories.
- Keep a status ledger covering every planned trial, actual attempt, infrastructure retry, valid failure, timeout and completion.
- Freeze controller settings before campaign trials. Preserve outcomes rather than replacing failed trials with successful substitutes.
- If the runner/configuration fails repeatedly, repair the experiment infrastructure and document it before consuming the full campaign. Do not turn this task into unrestricted solver/globalization work.

## Primary metrics and old/new comparison

The primary result is PHYSICAL CONSTRAINT VIOLATION ON THE EXECUTED TRAJECTORY, including active STC consequences. For each relevant requirement, compute:

1. Maximum positive physical excess in its physical units.
2. Violation duration at the available measurement resolution.
3. Number/fraction of trials with violations.
4. Distribution across trials and motion/noise groups.

Also report landing completion/timeouts, terminal position/velocity errors, completion time, tracking error, replanning times and rejected updates.

Keep predicted-horizon results labelled as predictions. Use the same physical limits, activation logic, tolerances and analysis window for comparable old/new results. If original scoring evaluated only the landing phase, preserve that comparable metric and label any broader controller-active-window analysis separately; do not substitute one for the other.

MC1 previously completed 290/300 landings, with all ten non-landings in circular motion. MC2 previously completed 60/60, so its improvement must primarily be assessed through violations, accuracy and tracking. Recompute comparable old metrics from available raw data where possible; explain missing or unmatched data without inventing pairings.

Do not assert a causal explanation for a violation without evidence. Do not change physical criteria to obtain a pass.

## Thesis locations and requested rewrite

Thesis root: `/home/lab/paper/2026_학위논문/`.

| Path relative to thesis root | Role |
|---|---|
| `main/v0.0/4_evaluation.tex` | Active Korean evaluation chapter |
| `main/draft_en/chapter4_evaluation.tex` | English counterpart |
| `main/thesis.md` | Synchronized Markdown |
| `main/v0.0/abstract_ko.tex`, `main/draft_en/abstract_en.tex`, `main/abstract.md` | Abstracts to update when results change |
| `main/v0.0/5_conclusion.tex`, `main/draft_en/chapter5_conclusion.tex` | Dependent result summaries |
| `main/figures/evaluation/` | Evaluation figures |
| `main/figures/evaluation/plot_sitl_focused.py` | Recent MC1/MC2 plotting work; currently points to historical samples.csv files |
| `main/shared/reviews/chapter4-evidence-index-2026-10-21.json` | Evidence/source index, including current MPC paths |
| `main/shared/reviews/full-manuscript-prose-review-2026-10-21.md` | Recent prose corrections to preserve |

The existing SITL subsection contains `목표 운동별 착륙 결과`, `목표 추정 잡음의 영향` and a separate `원형 목표 비행`. Rewrite it around the TWO campaign result groups:

- MC1: `목표 운동별 착륙 결과`.
- MC2: `목표 추정 잡음의 영향`.

Delete the standalone `원형 목표 비행` heading and its separate two-trial result block. Move necessary common setup/execution/constraint definitions into the shared setup. Select identified representative runs from the NEW MC1/MC2 data for any trajectory or state-history illustrations. Do not count the old two circle demonstrations as new campaign samples.

Keep the historical raw circle evidence intact. Treat the landing method consistently as folded cSTC-DDP; do not introduce additional method branches or experiment chapters. Preserve the existing hardware results and their denominators.

Lead with the measured physical findings, then explain how the plots support them. Report actual computation times. State mixed outcomes plainly. Keep provenance, hashes and code identifiers in reproducibility notes rather than printed thesis prose.

Retain the recent prose/typography fixes: no bold run-in paragraph labels, no repeated “not A, but B” framing, no lab-notebook narration, and no repeated per-sentence disclaimers. Figure text must be readable at print size. Keep method colors consistent and explain long legends in captions. Reuse current plot design where suitable.

Update every dependent campaign total and result claim in tables, captions, abstracts and conclusions, and synchronize Korean/English/Markdown. Do not overwrite unrelated incoming manuscript edits.

## Completion and reporting

- Finish all valid planned trials or report a concrete blocking condition and exact completed/remaining counts.
- Preserve raw evidence and produce machine-readable compact summaries.
- Produce old/new physical-violation comparisons where supported.
- Regenerate figures from the new data and update evidence/source manifests.
- Build both thesis PDFs. Check citations/references, totals, overfull boxes and result-page layout visually.
- Record what improved, what remained problematic and the limits of the comparison.
- Make the regular focused commits requested above, including a final verification checkpoint, and report commit identities and the result locations.

Do not claim that the campaigns have already been rerun or that improvement is established. This document is a handoff; the new results remain to be produced.
