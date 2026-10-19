# Point-mass matched paper comparison

Preserve the local-paper distinction: Elango Section 5 already has non-augmented
integral constraints. DDP's contribution is its primal-dual stage treatment.
User requests lifted OpenSCvx as main baseline, direct prox-linear as ablation.

Implemented candidate protocol and seed generator, physical-budget coordinate
conversion, fail-closed common feasibility checker, and qualitative-only labels
on historical result reports. Generated 175 shared seed files for 35 initial
conditions times five meshes; fixed/free seeds initially coincide. The candidate
manifest has 960 test runs but is explicitly not ready for launching them.

All 42 focused tests pass. No optimizer changes, paper campaign or timing study
are claimed. Current source mu update is already monotone under valid settings;
normalization/GN and SOC failures remain unresolved. Do not blindly disable GN
based on one successful ablation or call 16 substeps an accuracy certificate.

Additional source diagnosis: isolated logging snapshot and sequential build of
point_mass_factorization_trace reproduce all 42 normalized/GN LLT failures,
with exact agreement on all 301 optimizer records. Six failures have positive
averaged spectra but negative upper-triangle spectra; first asymmetry comes
from SOC IPM block. See POINT_MASS_DDP_FACTORIZATION.md. No production repair
is claimed; naive symmetrization is not a derived consistent SOC Newton step.

Remaining: source-level DDP failure investigation, common adapters and objective
verification, declared equal calibration work budget, one calibrated profile per
method, accurate fixed quadrature, stationarity thresholds and pinned sources.
Only then freeze the protocol before any test outcomes. Preserve failed and
uncertified cases; no best-of-pool selection or post-hoc retries.

## Subsequent explicit solver-repair request

User explicitly authorized NT SOC scaling and the fixed-barrier inner gate.
Core commit b994e4d4 implements symmetric NT H and consistent RHS for stage,
terminal and coupling application, and removes rho from the barrier inner
threshold. Unsolved caps do not advance schedules. Follow-up 4b99cc55 verifies
full 3D cone central points. KKT, 10-check CTCS self-test, paper-model derivative
check and 42 focused Python tests pass. No production debug logging was added.

The original normalized/GN-on N6 warm pilot still fails: inner60 exits at84;
inner600 diagnostic exits Forward Pass Failed at234. Both remain at mu=.001,
rhoT=100, have zero backward failures, but fail physical/budget certificates
in intervals2,3,4 and retain SOC boundary rejections. See POINT_MASS_DDP_REPAIR.md
and raw logs/point_mass/audits/nt_barrier_repair_n6_20261020. Neither run is
candidate calibration or paper evidence. Do not claim NT fixed all SOC stalls.
Remaining readiness work: fixed-barrier globalization/row initialization and
GN interaction investigation; no cause beyond the recorded evidence is proved.
