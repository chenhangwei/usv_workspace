#!/bin/bash
# fresh645: geometry-PD teacher + causal-cut BC + short RL polish.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CKPT=/mnt/data/checkpoints/usv_rl
export WARMSTART_FROM="${WARMSTART_FROM:-$CKPT/fresh645_bc_init.pt}"
export OUTPUT="${OUTPUT:-$CKPT/fresh645_geometry_pd_5usv.pt}"
export CKPT_DIR="${CKPT_DIR:-$CKPT/fresh645_checkpoints}"
export EVAL_DIR="${EVAL_DIR:-$CKPT/fresh645_eval}"
export EXPERIMENT_NAME="${EXPERIMENT_NAME:-fresh645}"
export EXPERIMENT_SUMMARY="${EXPERIMENT_SUMMARY:-GEOMETRY-PD-FULL: teacher v5 uses geometry PD as the goal-tracking base in ALL branches (fresh644 kept raw_angular_z in the engaged branch while dropout removed that input, making 72.7% of targets unlearnable); explicit heading/CTE PD in open water, COLREGS gated avoidance in encounters, and turn-away escape below 1.1m; BC input dropout slots 8,10 retained; short 50k RL with constant full-domain anchor 0.6}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-50000}"
export CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-8192}"
export POLICY_ANCHOR_WEIGHT="${POLICY_ANCHOR_WEIGHT:-0.6}"
export POLICY_ANCHOR_WEIGHT_END="${POLICY_ANCHOR_WEIGHT_END:-0.6}"
export POLICY_ANCHOR_ONLY_CPA_CONFLICT="${POLICY_ANCHOR_ONLY_CPA_CONFLICT:-0}"
exec bash "$SCRIPT_DIR/train_fresh641_duck_lock_5usv.sh"
