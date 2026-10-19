# Matched point-mass calibration with frozen DDP

Solver and globalization work is stopped at the user's instruction. The DDP experiment profile is frozen as NT + legacy barrier gate + CTCS GN off. GN is disabled only in the point-mass constraint wrapper; production code is unchanged. The staging-hover audit investigation remains separate.

## Completed experiment

Twelve actual solves used three calibration bearings, N=6, fixed/free time, and one profile for each primary method. All arms received the same cold hover rollout, physical interval budget B=1e-11 and 5 mm tightening. No test bearing was solved, no optimizer was retried, and no returned policy was replaced by an incumbent.

| Time mode | Folded DDP full certificate gate | Lifted OpenSCvx full certificate gate |
|---|---:|---:|
| Fixed | 2/3 | 2/3 |
| Free | 0/3 | 0/3 |

These are calibration results, not a paper ranking. Both passing fixed DDP runs report Optimal Solution. One passing fixed SCP run reports converged, the other iteration_limit. Native stationarity and certified feasibility remain separate. Free-time SCP calibration bearings 1 and 2 have zero integral density but terminal/shooting errors around 2–4e-6, above the common 1e-8 gate. DDP free-time results fail terminal and/or physical/input checks. A small integral alone is insufficient.

The full gate requires independent physical, integral and tightening certificates, a certified numerical-map error at most 1e-4 B, and common terminal, shooting and input tolerances. Strict zero input certificates are exported separately from input residual tolerance checks. Unresolved error bounds remain uncertified.

## Reproduction and evidence

- [Executable adapter and orchestration](point_mass_matched.py), [C++ DDP adapter](point_mass_matched_ddp.cpp), and [frozen DDP profile](point_mass_ddp_frozen_profile.json).
- [Raw evidence and per-case table](../../../logs/point_mass/audits/frozen_profile_calibration_20261020/REPORT.md), alongside summary.json, original protocol/manifest, source snapshots, binary, build flags, native traces and SHA256 inventory.
- DDP solver commit: `82b2da6651e8e71d3c8ad3af9e21af4785e5218f`; tracked solver sources unchanged. The evidence records the clean OpenSCvx source pin.
- Preparation/adapter commit: `90d16b4`; profile-record correction: `9a96487`.

The original frozen record mistakenly represented forward_cost_threshold as 0.1 although the C++ field is int and always received 0. Runtime equality checking detected this after the first three DDP solves. Those workers exited with metadata errors; their native policies were recovered read-only without rerunning optimization. The original protocol and exit records are retained. Corrected profile v1_1 records 0; all six actual runtime profiles match it. This is a record correction, not a setting or trajectory change.

The SCP candidate records initial and adapted virtual-control/buffer penalties. Each interval exports optimized accumulator increment, native Dopri8 integration increment, certified increment and accumulator defect separately. DDP uses 512 RK4 substeps; that resolution is not itself an accuracy certificate.

Verification: the C++ diagnostic target built sequentially; 44 focused tests pass, including all three bearings in both modes through the actual OpenSCvx seed constructor. Initial costs and actual DDP settings are checked at runtime. No production or SITL validation is claimed by this experiment.

## Remaining protocol work

The full 960-run protocol is not frozen and no held-out test runs have started. The free-time direct non-augmented ablation adapter, remaining mesh checks, stationarity diagnostics and profile-release evidence remain gates. Continue experiment preparation with the selected DDP profile; do not make further solver/globalization repair a prerequisite. Keep failures and uncertified cases in reported denominators. The existing warm-start direct-formulation pilot remains the evidence about the paper's non-augmented prox-linear method; these new lifted-SCP runs address the primary implementation baseline.
