# Historical evidence and recovery

| Location | Contents |
|---|---|
| [landing_history_20261019.tar.gz](landing_history_20261019.tar.gz) | Fourteen superseded landing initialization/model/mapping/reference campaigns; 291 verified files |
| [superseded_point_mass_20261019.tar.gz](superseded_point_mass_20261019.tar.gz) | Failed plot attempts and superseded duplicate snapshots; 414 verified files |
| [point_mass_history/](point_mass_history/) | Older strict-1e-8 point-mass snapshot kept unpacked for historical review |
| [backups/](backups/) | Earlier complete paper/point-mass recovery bundles and original checksums |
| [prior_tuning/](prior_tuning/) | Earlier verified intermediate STC tuning archive and its manifest |
| [cleanup_20261019/](cleanup_20261019/) | Original file hashes, per-file relocation mapping and cleanup verification |

Each new archive has an adjacent `.sha256` and `.manifest.json`. Every regular
member was read back and SHA256 matched against its original before unpacked
copies were removed. Remaining originals were checked again before removal.
No unique policy, bag, CSV, source snapshot or audit was discarded.

## Restore a historical campaign

List members first:

```sh
tar -tzf logs/archive/landing_history_20261019.tar.gz
```

Extract the chosen campaign into a **new empty recovery directory**, for example:

```sh
mkdir -p /tmp/comando-landing-recovery
tar -xzf logs/archive/landing_history_20261019.tar.gz \
  -C /tmp/comando-landing-recovery ddp_only_landing_mpc
```

Archive member names retain original campaign names. The
[complete relocation manifest](cleanup_20261019/cleanup_manifest.json) maps each
original path relative to `logs/` to either a current file or an archive/member.
Historical absolute paths inside raw JSON, logs or old bundle indexes describe
original locations and were deliberately not rewritten. For older bundles,
use the archive filename beside its index under `backups/`.

The original intermediate-tuning restore procedure now uses
`logs/archive/prior_tuning/stc_mpc_intermediate_tuning_2026-10-16.tar.gz`.
These verified archives are on the same local disk, not remote backups.
