#!/bin/bash
# fresh642 WSL2 断点续跑: 找最新 checkpoint, 以累计 total-timesteps 语义恢复.
# 用法: bash scripts/resume_fresh642.sh [额外新步数, 默认跑满 300000 累计]
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CKPT_DIR=/mnt/data/checkpoints/usv_rl/fresh642_checkpoints
TARGET_TOTAL="${1:-300000}"

LATEST=$(ls -t "$CKPT_DIR"/fresh642_duck_v3_5usv_step_*.pt 2>/dev/null | head -1)
[[ -n "$LATEST" ]] || { echo "no fresh642 checkpoint found in $CKPT_DIR"; exit 1; }
DONE=$(basename "$LATEST" | grep -oE '[0-9]+' | tail -1 | sed 's/^0*//')
echo "resuming from $LATEST (completed~$DONE), cumulative target $TARGET_TOTAL"
# 陷阱: --total-timesteps 是累计目标 (见 train_mappo_resume_total_timesteps_pitfall)
if (( TARGET_TOTAL <= DONE )); then
  echo "FATAL: target $TARGET_TOTAL <= completed $DONE, would exit immediately"; exit 1
fi

export RESUME_FROM="$LATEST"
export TOTAL_TIMESTEPS="$TARGET_TOTAL"
exec bash "$SCRIPT_DIR/train_fresh642_duck_v3_5usv.sh"
