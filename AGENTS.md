# Planner working rules

- Before work on each new user prompt, commit any pending changes. Check both
  this repository and the sibling `ALIPDDP-main` repository.
- Build one executable target at a time, with `catkin build ... -j1 -p1
  --make-args <single-target>`. Never run uncapped or concurrent builds.
- Run SITL cases sequentially. Scenario campaigns launch the installed PX4
  binary directly, without rebuilding PX4.
- MC1 baseline settings and CTCS migration results are documented in
  `docs/CTCS_NOAUG_MIGRATION.md`.
