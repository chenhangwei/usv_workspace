#!/bin/bash
# fresh63 三阶段自动训练
# Phase 1 → Phase 2 → Phase 3 依次执行
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "========== fresh63 课程学习: 三阶段自动训练 =========="
echo "Phase 1: 纯路径跟踪 (solo, 150K)"
echo "Phase 2: 路径 + 物理避让 (solo + head_on, 200K)"
echo "Phase 3: 全场景避让 (all 4 scenarios, 200K)"
echo "======================================================"
echo ""

echo "[$(date)] >>> Phase 1 开始"
bash "$SCRIPT_DIR/train_fresh63_phase1.sh"
echo "[$(date)] >>> Phase 1 完成"
echo ""

echo "[$(date)] >>> Phase 2 开始"
bash "$SCRIPT_DIR/train_fresh63_phase2.sh"
echo "[$(date)] >>> Phase 2 完成"
echo ""

echo "[$(date)] >>> Phase 3 开始"
bash "$SCRIPT_DIR/train_fresh63_phase3.sh"
echo "[$(date)] >>> Phase 3 完成"
echo ""

echo "========== 全部三阶段训练完成 [$(date)] =========="
