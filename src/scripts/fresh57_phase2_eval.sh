#!/bin/bash
# fresh57 Phase 2 checkpoint 精简评估 — 只评估 100K+ 的 checkpoint
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

# 只选 Phase 2 的 checkpoint (100K+) + 最终模型, 每隔一个采样
CKPTS=()
for f in $(ls "$CKPT_DIR"/fresh57_step_*.pt 2>/dev/null | sort -t_ -k3 -n); do
  STEP=$(basename "$f" .pt | sed 's/fresh57_step_0*//')
  if (( STEP >= 100000 )); then
    CKPTS+=("$f")
  fi
done

# 加入最终模型
FINAL="/mnt/data/checkpoints/usv_rl/fresh57.pt"
if [[ -f "$FINAL" ]]; then
  CKPTS+=("$FINAL")
fi

# 每隔一个采样 (约 25 个 checkpoint)
SAMPLED=()
IDX=0
for f in "${CKPTS[@]}"; do
  BASENAME=$(basename "$f" .pt)
  # 保留最终模型和第一个/最后一个 step checkpoint
  if [[ "$BASENAME" == "fresh57" ]] || (( IDX % 2 == 0 )); then
    SAMPLED+=("$f")
  fi
  IDX=$((IDX + 1))
done

TOTAL=${#SAMPLED[@]}
echo "═══════════════════════════════════════════════════"
echo "  fresh57 Phase 2 Eval (100K+ checkpoints)"
echo "  Sampled: $TOTAL / ${#CKPTS[@]}"
echo "  Episodes: $EPISODES × 3 scenarios"
echo "═══════════════════════════════════════════════════"

DONE=0
for CKPT in "${SAMPLED[@]}"; do
  BASENAME=$(basename "$CKPT" .pt)
  OUT_JSON="$EVAL_DIR/${BASENAME}_eval.json"

  if [[ -f "$OUT_JSON" ]]; then
    echo "SKIP: $BASENAME"
    DONE=$((DONE + 1))
    continue
  fi

  DONE=$((DONE + 1))
  echo ""
  echo "──── [$DONE/$TOTAL] $BASENAME ────"

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
