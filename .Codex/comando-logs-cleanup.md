# CoManDO log cleanup — 2026-10-19

User requests less clutter, retaining important point-mass and landing evidence.
Scope: CoManDO_planner/logs only. Separate local point-mass sweeps are active;
do not move their benchmark result pools or modify their optimizers.

Create point_mass/, landing/ and archive/ categories. Keep current 1e-6
point-mass figures with their complete raw inputs/audits and the whole paper
MPC campaign. Retain offline landing gates/identification, measured-state STC
baseline, circle margin validation/repeat, representative straight/figure-eight
failures, measured-state recovery, ordinary RH/MPC and frame references.
Compress superseded point-mass plotting attempts and older landing experiments
into two archives with per-file SHA256 manifests. Retain the old strict point-
mass review unpacked under archive/ for its existing external review shortcut.
Consolidate archive/ and archives/. Preserve current point-mass/paper-MPC
legacy shortcuts, update external review shortcuts and current documentation.
No raw policy, bag, CSV, source snapshot or numeric audit is discarded without
an independently read-back, hash-verified recoverable archive.

Before mutation: inventory file SHA256, sizes and paths, reject unknown root
entries and active writers. For every archived file verify archived bytes and
unchanged original immediately before removing unpacked copies. For moved
files verify destination SHA256; retain a complete relocation/restore manifest.
Update README indexes and check current figure/report links, original snapshot
inventories and review shortcuts. No controller/model/solver changes or runs.

## Completed and verified

41 root entries reduced to six: point_mass/, landing/, archive/, README.md and
two compatibility symlinks for recent point-mass links/tools. Latest point-mass
figures, full raw evidence, paper MPC runs and nine important ROS1 landing
campaigns remain unpacked, alongside offline gates and identification.
Fourteen older landing campaigns (291 files) and superseded point-mass
plotting attempts/snapshots (414 files) were archived. All 705 archived members
were read back and hash-verified before unpacked copies were removed. All
2,982 remaining original files also hash-match their recorded originals; the
old index is preserved separately. All 3,687 original files remain recoverable.
Net savings about325 MiB after new manifest/index overhead. The source script,
before inventory, full relocation map and verification are under
logs/archive/cleanup_20261019/. Existing recovery bundles and the earlier
intermediate-tuning archive remain preserved.

Both current and strict historical point-mass snapshot inventories PASS after
relocation;114 current index/evidence links resolve. External local review
shortcuts resolve, current figures and key landing reports remain accessible.
Original immutable raw logs/settings/source snapshots were not rewritten.
Maintained documentation links and paper plotter default root were updated.
Python compile/help and git diff checks PASS. No optimizations, flights,
controller changes, or live benchmark pool moves.
