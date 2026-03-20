# Developer Documentation

This directory contains focused developer-facing references and operational manuals.

## Manuals

- RL recommended workflow: [manuals/rl_recommended_workflow.md](manuals/rl_recommended_workflow.md)
- Multi-USV RL retraining plan: [manuals/multi_usv_rl_retraining_plan.md](manuals/multi_usv_rl_retraining_plan.md)
- PAI-DSW Git and remote workflow: [manuals/pai_dsw_git_remote_workflow.md](manuals/pai_dsw_git_remote_workflow.md)
- MAPPO model registry: [manuals/mappo_model_registry.md](manuals/mappo_model_registry.md)
- PAI-DSW MAPPO runbook: [manuals/pai_dsw_mappo_runbook.md](manuals/pai_dsw_mappo_runbook.md)
- PAI-DSW environment repair: [manuals/pai_dsw_environment_repair.md](manuals/pai_dsw_environment_repair.md)

## Notes

- The RL recommended workflow manual is the short operator-facing reference for the currently accepted online RL replacement path.
- The multi-USV retraining plan is the engineering proposal for upgrading RL from single-encounter validation to cluster-grade COLREGS-compliant avoidance.
- The PAI-DSW workflow manual is the operational reference for pushing the workspace to Git, cloning it on Alibaba Cloud PAI-DSW, and continuing development from a remote environment.
- The MAPPO model registry is the tracked metadata file that records which checkpoints are worth publishing or resuming, without committing model binaries into Git.
- The PAI-DSW MAPPO runbook is the operator reference for writing multi-agent training outputs to /mnt/data and running dense-scenario offline training and evaluation.
- The PAI-DSW environment repair manual is the recovery guide for Ubuntu 24.04 GPU images that have Python and CUDA but are missing ROS 2 Jazzy.
- The full technical background remains in [../../usv_rl/README.md](../../usv_rl/README.md).