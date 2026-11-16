#!/usr/bin/env bash
# fresh640 方案B管线: 等采集完成 -> BC 预训练 -> 启动 RL 精修
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CKPT=/mnt/data/checkpoints/usv_rl
DATASET="$CKPT/fresh640_teacher_5usv.npz"
BC_INIT="$CKPT/fresh640_bc_init.pt"
ARCH_BASE="$CKPT/fresh639_checkpoints/fresh639_graze_pass_5usv_step_0317333.pt"

echo "[pipeline] waiting for teacher dataset $DATASET ..."
while pgrep -f collect_multi_agent_teacher_dataset >/dev/null; do sleep 60; done
[[ -f "$DATASET" ]] || { echo "[pipeline] FATAL: dataset missing after collection"; exit 1; }
echo "[pipeline] dataset ready: $(du -h "$DATASET" | cut -f1)"

echo "[pipeline] BC pretraining attention actor ..."
python3 -m usv_rl.bc_pretrain_attention_actor \
  --warmstart "$ARCH_BASE" \
  --dataset "$DATASET" \
  --output "$BC_INIT" \
  --epochs 200 --lr 1e-3 --batch-size 512 --device cpu
[[ -f "$BC_INIT" ]] || { echo "[pipeline] FATAL: BC init missing"; exit 1; }

echo "[pipeline] launching fresh640 RL refinement ..."
exec bash "$SCRIPT_DIR/train_fresh640_duck_bc_5usv.sh"
