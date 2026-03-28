#!/bin/bash
# 批量评估headon27检查点
# 用法: bash scripts/eval_h27_checkpoints.sh [检查点文件...]
# 不指定参数时评估所有检查点
set -euo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash
export ROS_DOMAIN_ID=115  # 使用独立域ID，避免与训练冲突

CKPT_DIR="/mnt/data/checkpoints/usv_rl/headon27_checkpoints"
OUT_DIR="/tmp/h27_eval"
EPISODES=5
STEPS=180
SCENARIO="two_usv_head_on"

mkdir -p "$OUT_DIR"

if [ $# -gt 0 ]; then
  CKPTS=("$@")
else
  CKPTS=($(ls "$CKPT_DIR"/*.pt 2>/dev/null | sort))
fi

if [ ${#CKPTS[@]} -eq 0 ]; then
  echo "未找到检查点文件"
  exit 1
fi

echo "=== headon27 批量评估 ==="
echo "检查点数: ${#CKPTS[@]}"
echo "场景: $SCENARIO | 回合: $EPISODES | 步数: $STEPS"
echo ""

for ckpt in "${CKPTS[@]}"; do
  name=$(basename "$ckpt" .pt)
  outjson="$OUT_DIR/${name}.json"

  if [ -f "$outjson" ]; then
    echo "[$name] 跳过 (已有结果)"
    continue
  fi

  echo -n "[$name] 评估中... "
  if python3 -m usv_rl.evaluate_mappo_policy \
    --model "$ckpt" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --scenario "$SCENARIO" \
    --output-json "$outjson" \
    > /dev/null 2>&1; then
    # 解析结果
    python3 -c "
import json,sys
d=json.load(open('$outjson'))
col=d['collision_rate']
prog=d['mean_team_goal_progress_ratio']
sep=d['worst_pairwise_min_separation']
msep=d['mean_pairwise_min_separation']
print(f'碰撞={col:.0%} 进度={prog:.1%} 最差间距={sep:.2f}m 平均间距={msep:.2f}m')
"
  else
    echo "失败!"
  fi
done

echo ""
echo "=== 汇总 ==="
python3 << 'PYEOF'
import json, os, sys

out_dir = "/tmp/h27_eval"
results = []
for f in sorted(os.listdir(out_dir)):
    if not f.endswith('.json'): continue
    d = json.load(open(os.path.join(out_dir, f)))
    step = int(f.split('step_')[1].split('.')[0]) if 'step_' in f else 0
    results.append((step, d))

if not results:
    print("无评估结果")
    sys.exit(0)

print(f"{'检查点':>12} | {'碰撞率':>6} | {'进度':>6} | {'最差间距':>8} | {'平均间距':>8}")
print("-" * 55)
for step, d in sorted(results):
    col = d['collision_rate']
    prog = d['mean_team_goal_progress_ratio']
    sep = d['worst_pairwise_min_separation']
    msep = d['mean_pairwise_min_separation']
    marker = " ⚠️" if col > 0 else ""
    print(f"{step//1000:>10}K | {col:>5.0%} | {prog:>5.1%} | {sep:>7.2f}m | {msep:>7.2f}m{marker}")
PYEOF
