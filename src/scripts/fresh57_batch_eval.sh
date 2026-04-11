#!/bin/bash
# fresh57 Phase 2 检查点批量评估脚本
# 训练完成后单独运行，避免 OOM
set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export ROS_DOMAIN_ID=115
export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh57_checkpoints"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh57_eval"
RANKING="/mnt/data/checkpoints/usv_rl/fresh57_ranking.json"
EPISODES=5
STEPS=180

mkdir -p "$EVAL_DIR"

CKPTS=$(ls "$CKPT_DIR"/fresh57_step_*.pt 2>/dev/null | sort -t_ -k3 -n)
CKPT_COUNT=$(echo "$CKPTS" | grep -c "\.pt$" || true)

if [[ "$CKPT_COUNT" -eq 0 ]]; then
  echo "No Phase 2 checkpoints found. Evaluating Phase 1 checkpoints instead..."
  CKPTS=$(ls "$CKPT_DIR"/fresh57_phase1_step_*.pt 2>/dev/null | sort -t_ -k4 -n)
  CKPT_COUNT=$(echo "$CKPTS" | grep -c "\.pt$" || true)
fi

echo "═══════════════════════════════════════════════════"
echo "  fresh57 Batch Evaluation"
echo "  Checkpoints: $CKPT_COUNT"
echo "  Episodes: $EPISODES × 3 scenarios"
echo "═══════════════════════════════════════════════════"

TOTAL_DONE=0
for CKPT in $CKPTS; do
  BASENAME=$(basename "$CKPT" .pt)
  OUT_JSON="$EVAL_DIR/${BASENAME}_eval.json"

  if [[ -f "$OUT_JSON" ]]; then
    echo "SKIP: $BASENAME"
    TOTAL_DONE=$((TOTAL_DONE + 1))
    continue
  fi

  TOTAL_DONE=$((TOTAL_DONE + 1))
  echo ""
  echo "──── [$TOTAL_DONE/$CKPT_COUNT] $BASENAME ────"

  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$CKPT" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --output-json "$OUT_JSON" \
    2>&1 | tail -3

  python3 -c "
import json
with open('$OUT_JSON') as f:
    d = json.load(f)
coll = d.get('collision_rate', -1)
succ = d.get('success_rate', -1)
progress = d.get('mean_team_goal_progress_ratio', 0)
sep = d.get('worst_episode_min_separation', -1)
tag = '✅' if coll == 0 else ('⚠️' if coll < 0.2 else '❌')
print(f'  {tag} coll={coll:.0%} succ={succ:.0%} progress={progress:.3f} min_sep={sep:.3f}')
" 2>/dev/null || echo "  (parse error)"
done

echo ""
echo "Generating ranking..."
python3 << 'RANK_EOF'
import json, os, glob

eval_dir = os.environ.get('EVAL_DIR', '/mnt/data/checkpoints/usv_rl/fresh57_eval')
ranking_path = os.environ.get('RANKING', '/mnt/data/checkpoints/usv_rl/fresh57_ranking.json')

results = []
for jf in sorted(glob.glob(os.path.join(eval_dir, "*_eval.json"))):
    with open(jf) as f:
        d = json.load(f)
    name = os.path.basename(jf).replace("_eval.json", "")
    results.append({
        "checkpoint": name,
        "collision_rate": d.get("collision_rate", 1.0),
        "success_rate": d.get("success_rate", 0.0),
        "timeout_rate": d.get("timeout_rate", 1.0),
        "progress": d.get("mean_team_goal_progress_ratio", 0),
        "worst_min_sep": d.get("worst_episode_min_separation", 0),
    })

results.sort(key=lambda r: (r["collision_rate"], -r["progress"]))
zero_coll = [r for r in results if r["collision_rate"] == 0]

ranking = {
    "evaluated_checkpoints": results,
    "best_overall": results[0]["checkpoint"] if results else None,
    "best_safe": zero_coll[0]["checkpoint"] if zero_coll else None,
    "total_evaluated": len(results),
    "zero_collision_count": len(zero_coll),
}

with open(ranking_path, "w") as f:
    json.dump(ranking, f, indent=2)

print(f"Total: {len(results)}, Zero-collision: {len(zero_coll)}")
if zero_coll:
    b = zero_coll[0]
    print(f"Best safe: {b['checkpoint']} (progress={b['progress']:.3f} min_sep={b['worst_min_sep']:.3f})")
elif results:
    b = results[0]
    print(f"Least-bad: {b['checkpoint']} (coll={b['collision_rate']:.0%} progress={b['progress']:.3f})")

RANK_EOF

echo "Ranking: $RANKING"
echo "Done."
