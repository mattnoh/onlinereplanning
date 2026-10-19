# Landing evidence

[Offline stressed gate](offline/) · [plant/control identification](identification/)

The offline gate is diagnostic evidence of unresolved stressed landing behavior.
No new landing experiment or solver change was made during this cleanup.

## Retained ROS1 campaigns

| Folder | Why retained |
|---|---|
| [measured_state_stc_mpc](ros1/measured_state_stc_mpc/) | Original measured-state STC baseline, bag, timing and predicted/physical audits |
| [mpc_stc_speed_margin_validation](ros1/mpc_stc_speed_margin_validation/) | First circle landing with zero measured violations and representative figure-eight failure |
| [mpc_stc_speed_margin_repeat](ros1/mpc_stc_speed_margin_repeat/) | Repeat circle result, current plots and full tuning report; predicted audit still fails |
| [mpc_stc_tuned_validation](ros1/mpc_stc_tuned_validation/) | Representative aligned straight-motion failure |
| [measured_state_ddp_mpc_recovery](ros1/measured_state_ddp_mpc_recovery/) | Measured-state DDP execution/recovery evidence; remaining rate violation documented |
| [ordinary_landing_mpc](ros1/ordinary_landing_mpc/) | Ordinary stage-constraint MPC straight/circle/figure-eight baseline results |
| [ordinary_landing_rh](ros1/ordinary_landing_rh/) | Ordinary RH baseline results |
| [frame_regression_mpc](ros1/frame_regression_mpc/) | Historical frame/MPC reference |
| [frame_regression_rh](ros1/frame_regression_rh/) | Historical frame/RH reference; not a matched current-version benchmark |

Bags, CSVs, applied settings, source manifests and audit outputs remain intact
inside every retained campaign. A completed landing or a measured-only circle
pass is not proof of predicted feasibility or robust constraint satisfaction.
Older initialization, startup, mapping, transport and model experiments are
recoverable from [the landing-history archive](../archive/README.md).
