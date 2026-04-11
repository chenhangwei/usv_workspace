#!/bin/bash
# SITL 部署门控验证 — 在部署到真实硬件前验证策略
#
# 使用 SITL 仿真器运行策略，检查:
#   1. 碰撞率 = 0%
#   2. 最小间距 > 1.5m
#   3. omega 翻转次数 < 5
#   4. 成功率 > 40%（至少有进度）
#
# 用法: bash scripts/fresh57_sitl_gate.sh <model_path>
# 示例: bash scripts/fresh57_sitl_gate.sh /mnt/data/checkpoints/usv_rl/fresh57.pt

set -eo pipefail

MODEL="${1:?Usage: $0 <model_path>}"

if [[ ! -f "$MODEL" ]]; then
  echo "ERROR: Model not found: $MODEL"
  exit 1
fi

cd "$(dirname "$0")/../.."
source install/setup.bash

export ROS_DOMAIN_ID=115
export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

BASENAME=$(basename "$MODEL" .pt)
RESULT_DIR="/tmp/sitl_gate_${BASENAME}"
mkdir -p "$RESULT_DIR"

EPISODES=10
STEPS=180
SCENARIOS=("two_usv_head_on" "three_usv_crossing" "three_usv_overtaking")

# 门控阈值
GATE_MAX_COLLISION=0.0
GATE_MIN_SEP=1.5
GATE_MAX_OMEGA_FLIP=5.0
GATE_MIN_PROGRESS=0.40

echo "════════════════════════════════════════════════════════"
echo "  SITL Deployment Gate Check"
echo "  Model: $MODEL"
echo "  Episodes: $EPISODES per scenario"
echo "  Gate Criteria:"
echo "    collision_rate   ≤ $GATE_MAX_COLLISION"
echo "    worst_min_sep    ≥ ${GATE_MIN_SEP}m"
echo "    omega_flip_count ≤ $GATE_MAX_OMEGA_FLIP"
echo "    progress_ratio   ≥ $GATE_MIN_PROGRESS"
echo "════════════════════════════════════════════════════════"
echo ""

PASS_COUNT=0
FAIL_COUNT=0

for SCENARIO in "${SCENARIOS[@]}"; do
  OUT_JSON="$RESULT_DIR/${SCENARIO}_eval.json"
  echo "──── Testing: $SCENARIO ($EPISODES episodes) ────"

  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$MODEL" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --scenario "$SCENARIO" \
    --output-json "$OUT_JSON" \
    2>&1 | grep -E "Saved|ERROR" || true

  SCENARIO_PASSED=0
  python3 << GATE_EOF && SCENARIO_PASSED=1 || SCENARIO_PASSED=0
import json, sys

with open("$OUT_JSON") as f:
    d = json.load(f)

coll = d.get("collision_rate", 1.0)
succ = d.get("success_rate", 0.0)
progress = d.get("mean_team_goal_progress_ratio", 0.0)
min_sep = d.get("worst_episode_min_separation", 0.0)
omega = d.get("mean_omega_flip_count", 999.0)

checks = [
    ("collision_rate", coll, "≤", $GATE_MAX_COLLISION, coll <= $GATE_MAX_COLLISION),
    ("worst_min_sep", min_sep, "≥", $GATE_MIN_SEP, min_sep >= $GATE_MIN_SEP),
    ("omega_flip", omega, "≤", $GATE_MAX_OMEGA_FLIP, omega <= $GATE_MAX_OMEGA_FLIP),
    ("progress", progress, "≥", $GATE_MIN_PROGRESS, progress >= $GATE_MIN_PROGRESS),
]

all_pass = True
for name, value, op, threshold, passed in checks:
    tag = "✅" if passed else "❌"
    print(f"  {tag} {name}: {value:.3f} {op} {threshold}")
    if not passed:
        all_pass = False

if all_pass:
    print(f"  ── SCENARIO PASSED ──")
    sys.exit(0)
else:
    print(f"  ── SCENARIO FAILED ──")
    sys.exit(1)
GATE_EOF

  if [[ $SCENARIO_PASSED -eq 1 ]]; then
    PASS_COUNT=$((PASS_COUNT + 1))
  else
    FAIL_COUNT=$((FAIL_COUNT + 1))
  fi
  echo ""
done

echo "════════════════════════════════════════════════════════"
echo "  Gate Results: $PASS_COUNT/${#SCENARIOS[@]} scenarios passed"

if [[ $FAIL_COUNT -eq 0 ]]; then
  echo "  ✅ ALL SCENARIOS PASSED — Ready for field deployment"
  echo "════════════════════════════════════════════════════════"
  exit 0
else
  echo "  ❌ $FAIL_COUNT SCENARIO(S) FAILED — NOT ready for deployment"
  echo "  Review results in: $RESULT_DIR/"
  echo "════════════════════════════════════════════════════════"
  exit 1
fi
