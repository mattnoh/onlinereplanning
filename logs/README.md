# Planner logs

Start with [point-mass results](point_mass/README.md) or [landing evidence](landing/README.md).
Historical experiments and recovery bundles are indexed under [archive/](archive/README.md).

| Location | What to inspect |
|---|---|
| [point_mass/open_loop/](point_mass/open_loop/) | Latest fixed/free DDP/SCP × stage/CTCS figures, exact policy replays, settings, source and audits; SCP dynamics gate 1e-6 |
| [point_mass/paper_mpc/](point_mass/paper_mpc/) | Complete Uzun MPC reproduction, matched comparisons, tuning/replay evidence and current path overlay |
| [landing/offline/](landing/offline/) | Stressed landing gate logs and configurations; landing convergence/constraint work remains incomplete |
| [landing/ros1/](landing/ros1/) | Retained baselines, circle results and representative straight/figure-eight failures, including bags and audits |
| [landing/identification/](landing/identification/) | Plant/rate/thrust identification and target-motion diagnostics |
| [archive/](archive/README.md) | Verified older runs, superseded snapshots and recovery bundles |

The current free-time SCP CTCS snapshot passes physical and integral checks at
all five saved meshes. Its N=100 sweep was still ongoing when the snapshot was
made. The DDP free-time N=6 integral-budget failure remains marked in the figure.
Saved timing/seed differences prevent a controlled solver-speed conclusion.

The circle flights with zero measured violations do not establish predicted
horizon feasibility or successful straight/figure-eight landings. Preserve the
saved failures and distinguish measured, predicted and integral-row checks.

## Cleanup and recovery

2026-10-19: 41 flat top-level entries became three categories plus this index
and two compatibility shortcuts. All 3,687 original files are either retained
byte-for-byte or recoverable from hash-verified archives. 705 files were
compressed, reclaiming about 326 MiB before small index/manifest overhead.

[Complete relocation manifest](archive/cleanup_20261019/cleanup_manifest.json),
[original SHA256 inventory](archive/cleanup_20261019/inventory_before.json),
[original index](archive/cleanup_20261019/README.before.md).
See [restore instructions](archive/README.md) before extracting historical data.
Recorded paths inside immutable raw reports are original locations; the
relocation manifest maps them to their current file or archive member.

`uzun2024_ctcs_mpc` and `point_mass_open_loop_2x2_gap1e6_20261019_v2` are only
compatibility symlinks for recent tool/chat links. New work should use the
category paths above. Separate live solver benchmark pools were not moved.
These are local files and archives, not off-machine backups.
