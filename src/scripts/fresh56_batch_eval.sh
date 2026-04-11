#!/bin/bash
# fresh56 Phase 2 检查点批量评估
# 在训练完成后单独运行，避免OOM（不与训练进程同时运行）
#
# 评估所有 Phase 2 检查点 (fresh56_step_*.pt) 在3个场景下

set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export ROS_DOMAIN_ID=115
export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh56_checkpoints"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh56_eval"
RANKING="/mnt/data/checkpoints/usv_rl/fresh56_ranking.json"
EPISODES=5
STEPS=180

mkdir -p "$EVAL_DIR"

# 获取所有 Phase 2 检查点
CKPTS=$(ls "$CKPT_DIR"/fresh56_step_*.pt 2>/dev/null | sort -t_ -k3 -n)
CKPT_COUNT=$(echo "$CKPTS" | grep -c "\.pt$" || true)

if [[ "$CKPT_COUNT" -eq 0 ]]; then
  echo "No Phase 2 checkpoints found in $CKPT_DIR"
  exit 1
fi

echo "═══════════════════════════════════════════════════"
echo "  fresh56 Phase 2 Batch Evaluation"
echo "  Checkpoints: $CKPT_COUNT"
echo "  Episodes: $EPISODES per scenario"
echo "  Scenarios: head_on, crossing, overtaking"
echo "═══════════════════════════════════════════════════"

SCENARIOS="two_usv_head_on three_usv_crossing three_usv_overtaking"
TOTAL_DONE=0

for CKPT in $CKPTS; do
  BASENAME=$(basename "$CKPT" .pt)
  OUT_JSON="$EVAL_DIR/${BASENAME}_eval.json"
  
  # 跳过已评估的
  if [[ -f "$OUT_JSON" ]]; then
    echo "SKIP (already evaluated): $BASENAME"
    TOTAL_DONE=$((TOTAL_DONE + 1))
    continue
  fi
  
  TOTAL_DONE=$((TOTAL_DONE + 1))
  echo ""
  echo "──── [$TOTAL_DONE/$CKPT_COUNT] Evaluating: $BASENAME ────"
  
  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$CKPT" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --output-json "$OUT_JSON" \
    2>&1 | tail -5
  
  # 打印摘要
  python3 -c "
import json
with open('$OUT_JSON') as f:
    d = json.load(f)
coll = d.get('collision_rate', -1)
timeout = d.get('timeout_rate', -1)
progress = d.get('mean_team_goal_distance_delta', 0)
sep = d.get('worst_episode_min_separation', d.get('mean_episode_min_separation', -1))
tag = '✅' if coll == 0 else ('⚠️' if coll < 0.2 else '❌')
print(f'  {tag} coll={coll:.2f} timeout={timeout:.2f} progress={progress:.2f} min_sep={sep:.2f}')
" 2>/dev/null || echo "  (parse error)"
  
  echo "  Saved: $OUT_JSON"
done

# ─────────────────────────────────────────────────────────────
# 生成排名
# ─────────────────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════"
echo "  Generating Ranking"
echo "═══════════════════════════════════════════════════"

python3 << 'RANK_EOF'
import json, os, glob

eval_dir = "/mnt/data/checkpoints/usv_rl/fresh56_eval"
ranking_path = "/mnt/data/checkpoints/usv_rl/fresh56_ranking.json"

results = []
for jf in sorted(glob.glob(os.path.join(eval_dir, "fresh56_step_*_eval.json"))):
    with open(jf) as f:
        d = json.load(f)
    name = os.path.basename(jf).replace("_eval.json", "")
    results.append({
        "checkpoint": name,
        "collision_rate": d.get("collision_rate", 1.0),
        "timeout_rate": d.get("timeout_rate", 1.0),
        "success_rate": d.get("success_rate", 0.0),
        "mean_team_goal_distance_delta": d.get("mean_team_goal_distance_delta", 0),
        "worst_episode_min_separation": d.get("worst_episode_min_separation", 0),
    })

# 排序: 先碰撞率升序, 再进度降序
results.sort(key=lambda r: (r["collision_rate"], -r["mean_team_goal_distance_delta"]))

# 选择最优
zero_coll = [r for r in results if r["collision_rate"] == 0]
best_overall = results[0]["checkpoint"] if results else None
best_safe = zero_coll[0]["checkpoint"] if zero_coll else None
best_progress = max(results, key=lambda r: r["mean_team_goal_distance_delta"])["checkpoint"] if results else None

ranking = {
    "evaluated_checkpoints": results,
    "best_overall": best_overall,
    "best_safe_checkpoint": best_safe,
    "best_progress": best_progress,
    "total_evaluated": len(results),
}

with open(ranking_path, "w") as f:
    json.dump(ranking, f, indent=2)

print(f"\nTotal evaluated: {len(results)}")
print(f"Zero-collision: {len(zero_coll)}")
print(f"Best overall (lowest collision): {best_overall}")
print(f"Best safe (zero collision, best progress): {best_safe}")
print(f"Best progress (any collision): {best_progress}")

if zero_coll:
    b = zero_coll[0]
    print(f"\n  Recommended: {b['checkpoint']}")
    print(f"    collision={b['collision_rate']:.2f} progress={b['mean_team_goal_distance_delta']:.2f} min_sep={b['worst_episode_min_separation']:.2f}")
else:
    print("\n  ⚠️ No zero-collision checkpoint found!")
    if results:
        b = results[0]
        print(f"  Least-bad: {b['checkpoint']}")
        print(f"    collision={b['collision_rate']:.2f} progress={b['mean_team_goal_distance_delta']:.2f}")

RANK_EOF

echo ""
echo "Ranking saved: $RANKING"
echo "═══════════════════════════════════════════════════"
echo "  Batch evaluation complete"
echo "═══════════════════════════════════════════════════"
