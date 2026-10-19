# Authoritative research scope: paper non-augmented CTCS versus our DDP

The user repeatedly requested the difference between the paper's DIRECT,
NON-AUGMENTED CTCS optimization method and ours. Do not replace this with a
comparison against augmented-y SCP or claim the paper has no non-augmented form.

Read the LOCAL Elango PDF at Zotero storage FGP9CC2S. Section 5, equations
(47)-(49), has variables X,U and direct interval integral inequalities xi<=eps.
Section 5.1, equation (50), L1-penalizes those rows and uses the prox-linear
method. Equation (42) gives the proximal linearized composite step. The convex
specialization assumes fixed time, linear dynamics, convex constraints and
Slater; our sphere-exterior obstacle and free-time variants do not inherit its
global-minimizer theorem.

Our DDP keeps the same integral inequality explicit, with primal-dual slack /
complementarity machinery, CTCS GN curvature in the current revision, a Riccati
pass, and barrier/AL line search including control SOC slack/dual guards.
The relevant comparison is penalty/prox-linear versus these DDP updates.

Primary maintained report:
support/test/uzun2024_ctcs_mpc/CTCS_OPTIMIZATION_DIFFERENCES.md
Earlier augmented-y experiments are a supplement, not this task's answer.

Actual fixed-time 3D N=6 direct comparison:
logs/point_mass/audits/direct_nonaug_n6_20261020_v3/
Uses identical seed X/U and shared C++ 512-substep integral map. Reference
non-augmented prox-linear: feasible at native iteration 41, ends at 71,
objective 4.004202520112, independently propagated integral/budget 0.77213.
Original raw/current-GN DDP: feasible at iteration 4, later loses budget,
ends at limit 1201 with ratio 188.76; 1155 native SOC slack/dual boundary
rejections. Normalized/GN-off research ablation returns feasible, ratio .96782,
objective 4.004202515041. Both successful endpoints have strict nominal path /
integral certificates. One warm seed/settings case is not a universal ranking.
Prototype implementation/IPC timings are not intrinsic algorithm runtimes.

No core optimizer, controllers or live benchmark pools were edited. Research
driver and paper-style reference live in point_mass_nonaug_trace.cpp and
point_mass_nonaug_compare.py. All 36 tests passed. More meshes/seeds and
calibrated settings are needed before claiming broad convergence superiority.
