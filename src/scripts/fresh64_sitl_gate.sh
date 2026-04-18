#!/bin/bash
# fresh64 SITL 部署门控验证
# 在部署到真实硬件前验证策略的关键能力
#
# 与 fresh57_sitl_gate 的区别:
#   1. 新增 solo_navigation 门控 (fresh64核心: heading/CTE)
#   2. 新增 random_encounter 场景
#   3. heading_error 和 CTE 有专门的门控阈值
#   4. 更多 episodes (8/场景) 提升统计稳定性
#
# 门控条件:
#   [全场景] collision_rate   ≤ 0%
#   [全场景] progress_ratio   ≥ 0.40
#   [multi]  worst_min_sep    ≥ 1.5m
#   [multi]  omega_flip_count ≤ 5
#   [solo]   heading_error    ≤ 0.50 rad (~29°)
#   [solo]   cross_track_error ≤ 1.0m
#
# 用法: bash scripts/fresh64_sitl_gate.sh <model_path>
# 示例: bash scripts/fresh64_sitl_gate.sh /mnt/data/checkpoints/usv_rl/fresh64_phase3.pt

set -eo pipefail

MODEL="${1:?Usage: $0 <model_path>}"

if [[ ! -f "$MODEL" ]]; then
  echo "ERROR: Model not found: $MODEL"
  exit 1
fi

cd "$(dirname "$0")/../.."
source install/setup.bash

export ROS_DOMAIN_ID=116
export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

BASENAME=$(basename "$MODEL" .pt)
RESULT_DIR="/tmp/sitl_gate_${BASENAME}"
mkdir -p "$RESULT_DIR"

EPISODES=8
STEPS=275  # 55s

# ── 门控阈值 ──
GATE_MAX_COLLISION=0.0
GATE_MIN_SEP=1.5
GATE_MAX_OMEGA_FLIP=5.0
GATE_MIN_PROGRESS=0.40
GATE_MAX_HEADING_ERR=0.50      # 0.5 rad ≈ 29° (fresh63基线: 1.2-1.7)
GATE_MAX_CTE=1.0               # 1.0m (fresh63基线: 大幅偏离)

# 场景分组
SOLO_SCENARIOS=("solo_navigation")
MULTI_SCENARIOS=("two_usv_head_on" "three_usv_crossing" "three_usv_overtaking" "three_usv_random_encounter")

echo "════════════════════════════════════════════════════════"
echo "  fresh64 SITL Deployment Gate Check"
echo "  Model: $MODEL"
echo "  Episodes: $EPISODES per scenario"
echo "  Scenarios: ${#SOLO_SCENARIOS[@]} solo + ${#MULTI_SCENARIOS[@]} multi"
echo ""
echo "  Gate Criteria:"
echo "    [all]   collision_rate   ≤ ${GATE_MAX_COLLISION}"
echo "    [all]   progress_ratio   ≥ ${GATE_MIN_PROGRESS}"
echo "    [multi] worst_min_sep    ≥ ${GATE_MIN_SEP}m"
echo "    [multi] omega_flip_count ≤ ${GATE_MAX_OMEGA_FLIP}"
echo "    [solo]  heading_error    ≤ ${GATE_MAX_HEADING_ERR} rad"
echo "    [solo]  cross_track_err  ≤ ${GATE_MAX_CTE}m"
echo "════════════════════════════════════════════════════════"
echo ""

PASS_COUNT=0
FAIL_COUNT=0
TOTAL_SCENARIOS=$(( ${#SOLO_SCENARIOS[@]} + ${#MULTI_SCENARIOS[@]} ))

# ════════════════════════════════════════
# (A) Solo navigation 门控
# ════════════════════════════════════════
for SCENARIO in "${SOLO_SCENARIOS[@]}"; do
  OUT_JSON="$RESULT_DIR/${SCENARIO}_eval.json"
  echo "──── Testing: $SCENARIO ($EPISODES episodes) [SOLO] ────"

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
progress = d.get("mean_team_goal_progress_ratio", 0.0)
heading_err = d.get("mean_heading_error", 999.0)
cte = d.get("mean_cross_track_error", 999.0)
success = d.get("success_rate", 0.0)
omega_sat = d.get("mean_omega_saturation_ratio", 0.0)

checks = [
    ("collision_rate", coll, "<=", $GATE_MAX_COLLISION, coll <= $GATE_MAX_COLLISION),
    ("progress", progress, ">=", $GATE_MIN_PROGRESS, progress >= $GATE_MIN_PROGRESS),
    ("heading_error", heading_err, "<=", $GATE_MAX_HEADING_ERR, heading_err <= $GATE_MAX_HEADING_ERR),
    ("cross_track_err", cte, "<=", $GATE_MAX_CTE, cte <= $GATE_MAX_CTE),
]

all_pass = True
for name, value, op, threshold, passed in checks:
    tag = "✅" if passed else "❌"
    unit = "rad" if "heading" in name else ("m" if "track" in name else "")
    print(f"  {tag} {name}: {value:.3f}{unit} {op} {threshold}{unit}")
    if not passed:
        all_pass = False

# Info-only (not gated)
print(f"  ℹ️  success_rate: {success:.1%}  omega_sat: {omega_sat:.1%}")

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

# ════════════════════════════════════════
# (B) Multi-agent 避碰门控
# ════════════════════════════════════════
for SCENARIO in "${MULTI_SCENARIOS[@]}"; do
  OUT_JSON="$RESULT_DIR/${SCENARIO}_eval.json"
  echo "──── Testing: $SCENARIO ($EPISODES episodes) [MULTI] ────"

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
progress = d.get("mean_team_goal_progress_ratio", 0.0)
min_sep = d.get("worst_episode_min_separation", 0.0)
omega = d.get("mean_omega_flip_count", 999.0)
heading_err = d.get("mean_heading_error", 0.0)
success = d.get("success_rate", 0.0)
colregs = d.get("mean_colregs_compliance_ratio", 0.0)

checks = [
    ("collision_rate", coll, "<=", $GATE_MAX_COLLISION, coll <= $GATE_MAX_COLLISION),
    ("progress", progress, ">=", $GATE_MIN_PROGRESS, progress >= $GATE_MIN_PROGRESS),
    ("worst_min_sep", min_sep, ">=", $GATE_MIN_SEP, min_sep >= $GATE_MIN_SEP),
    ("omega_flip", omega, "<=", $GATE_MAX_OMEGA_FLIP, omega <= $GATE_MAX_OMEGA_FLIP),
]

all_pass = True
for name, value, op, threshold, passed in checks:
    tag = "✅" if passed else "❌"
    unit = "m" if "sep" in name else ""
    print(f"  {tag} {name}: {value:.3f}{unit} {op} {threshold}{unit}")
    if not passed:
        all_pass = False

# Info-only (not gated)
print(f"  ℹ️  success={success:.1%}  he={heading_err:.3f}rad  COLREGs={colregs:.1%}")

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

# ════════════════════════════════════════
# 汇总
# ════════════════════════════════════════
echo "════════════════════════════════════════════════════════"
echo "  Gate Results: $PASS_COUNT/$TOTAL_SCENARIOS scenarios passed"

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
