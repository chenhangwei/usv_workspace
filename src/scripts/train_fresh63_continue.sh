#!/bin/bash
# fresh63 接力脚本: 等待 Phase 1 完成后自动执行 Phase 2 → Phase 3
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PHASE1_OUTPUT="/mnt/data/checkpoints/usv_rl/fresh63_phase1.pt"

echo "========== fresh63 接力: 等待 Phase 1 → 自动 Phase 2 → Phase 3 =========="
echo "等待 Phase 1 输出: $PHASE1_OUTPUT"

# 等待 Phase 1 模型文件出现（说明 Phase 1 训练完成）
while [ ! -f "$PHASE1_OUTPUT" ]; do
    sleep 30
done
echo "[$(date)] >>> Phase 1 模型已检测到: $PHASE1_OUTPUT"
# 额外等待 10s 确保文件写入完成
sleep 10
echo ""

echo "[$(date)] >>> Phase 2 开始"
bash "$SCRIPT_DIR/train_fresh63_phase2.sh"
echo "[$(date)] >>> Phase 2 完成"
echo ""

echo "[$(date)] >>> Phase 3 开始"
bash "$SCRIPT_DIR/train_fresh63_phase3.sh"
echo "[$(date)] >>> Phase 3 完成"
echo ""

echo "========== 全部训练完成 [$(date)] =========="
