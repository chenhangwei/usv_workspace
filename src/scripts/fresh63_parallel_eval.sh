#!/bin/bash
# fresh63 并行批量评估脚本 — 自动循环，6进程并行
set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export ROS_DOMAIN_ID=123
export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh63_phase3_checkpoints"
FINAL_CKPT="/mnt/data/checkpoints/usv_rl/fresh63_phase3.pt"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh63_eval"
RANKING="/mnt/data/checkpoints/usv_rl/fresh63_ranking.json"
EPISODES=12
STEPS=275
PARALLEL=6

mkdir -p "$EVAL_DIR"

# 收集所有 checkpoint
ALL_CKPTS=$(ls "$CKPT_DIR"/fresh63_phase3_step_*.pt 2>/dev/null | sort -t_ -k4 -n)
if [[ -f "$FINAL_CKPT" ]]; then
  ALL_CKPTS="$ALL_CKPTS $FINAL_CKPT"
fi

TOTAL=$(echo $ALL_CKPTS | wc -w)
echo "═══════════════════════════════════════════════════"
echo "  fresh63 Parallel Evaluation ($PARALLEL workers)"
echo "  Total checkpoints: $TOTAL"
echo "  Episodes: $EPISODES × 3 scenarios"
echo "  Steps per episode: $STEPS"
echo "═══════════════════════════════════════════════════"

eval_one() {
  local CKPT="$1"
  local BASENAME=$(basename "$CKPT" .pt)
  local OUT_JSON="$EVAL_DIR/${BASENAME}_eval.json"
  local LOG="$EVAL_DIR/${BASENAME}_eval.log"

  if [[ -f "$OUT_JSON" ]]; then
    return 0
  fi

  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$CKPT" \
    --episodes "$EPISODES" \
    --steps-per-episode "$STEPS" \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --output-json "$OUT_JSON" \
    > "$LOG" 2>&1

  if [[ -f "$OUT_JSON" ]]; then
    python3 -c "
import json
with open('$OUT_JSON') as f:
    d = json.load(f)
coll = d.get('collision_rate', -1)
progress = d.get('mean_team_goal_progress_ratio', 0)
tag = '✅' if coll == 0 else ('⚠️' if coll < 0.2 else '❌')
print(f'  {tag} {\"$BASENAME\"}: coll={coll:.0%} prog={progress:.3f}')
" 2>/dev/null || echo "  $BASENAME: done (parse error)"
  else
    echo "  ❌ $BASENAME: FAILED"
  fi
}

RUNNING=0
DONE_COUNT=0
for CKPT in $ALL_CKPTS; do
  BASENAME=$(basename "$CKPT" .pt)
  OUT_JSON="$EVAL_DIR/${BASENAME}_eval.json"
  if [[ -f "$OUT_JSON" ]]; then
    DONE_COUNT=$((DONE_COUNT + 1))
    continue
  fi

  eval_one "$CKPT" &
  RUNNING=$((RUNNING + 1))

  if (( RUNNING >= PARALLEL )); then
    wait -n 2>/dev/null || wait
    RUNNING=$((RUNNING - 1))
    DONE_COUNT=$((DONE_COUNT + 1))
    CURRENT_DONE=$(ls "$EVAL_DIR"/*_eval.json 2>/dev/null | wc -l)
    echo "── Progress: $CURRENT_DONE / $TOTAL ──"
  fi
done

# 等待所有剩余进程
wait
echo ""

CURRENT_DONE=$(ls "$EVAL_DIR"/*_eval.json 2>/dev/null | wc -l)
echo "All evaluations done: $CURRENT_DONE / $TOTAL"

# 生成排名
echo "Generating composite ranking..."
export EVAL_DIR RANKING
python3 << 'RANK_EOF'
import json, os, glob
import numpy as np

eval_dir = os.environ.get('EVAL_DIR', '/mnt/data/checkpoints/usv_rl/fresh63_eval')
ranking_path = os.environ.get('RANKING', '/mnt/data/checkpoints/usv_rl/fresh63_ranking.json')

results = []
for jf in sorted(glob.glob(os.path.join(eval_dir, "*_eval.json"))):
    with open(jf) as f:
        d = json.load(f)
    name = os.path.basename(jf).replace("_eval.json", "")

    collision_rate = d.get("collision_rate", 1.0)
    progress = d.get("mean_team_goal_progress_ratio", 0.0)
    proj_progress = d.get("mean_projected_progress", progress)
    prog_efficiency = d.get("mean_progress_efficiency", 0.0)
    prog_consistency = d.get("mean_progress_consistency", 0.0)
    worst_sep = d.get("worst_episode_min_separation", 0.0)
    omega_sat = d.get("mean_omega_saturation_ratio", 0.0)
    omega_flip = d.get("mean_omega_flip_count", 0.0)
    heading_err = d.get("mean_heading_error", 0.0)
    cte = d.get("mean_cross_track_error", 0.0)
    entanglement = d.get("mean_entanglement_ratio", 0.0)
    colregs_comp = d.get("mean_colregs_compliance_ratio", 0.0)
    colregs_viol = d.get("mean_colregs_violation_ratio", 0.0)
    cpa_sb = d.get("mean_cpa_starboard_pass_ratio", 0.0)

    safety = (1.0 - collision_rate) * 25.0 + min(worst_sep / 2.0, 1.0) * 10.0
    prog_score = min(proj_progress, 1.0) * 15.0 + min(prog_efficiency, 1.0) * 5.0 + min(prog_consistency, 1.0) * 5.0
    smooth = 15.0 * max(0.0, 1.0 - 0.5 * omega_sat - 0.02 * omega_flip - 0.3 * heading_err)
    tracking = 10.0 * max(0.0, 1.0 - 0.3 * cte - 0.5 * entanglement)
    colregs_sc = max(0.0, 15.0 * (0.5 * colregs_comp + 0.5 * cpa_sb) - 5.0 * colregs_viol)

    composite = round(safety + prog_score + smooth + tracking + colregs_sc, 2)

    scenario_summaries = d.get("scenario_summaries", {})
    per_scenario_progress = [
        float(v.get("mean_team_goal_progress_ratio", 0.0))
        for v in scenario_summaries.values()
    ]
    worst_scenario_progress = min(per_scenario_progress) if per_scenario_progress else 0.0

    results.append({
        "checkpoint": name,
        "composite_score": composite,
        "collision_rate": collision_rate,
        "progress": progress,
        "projected_progress": proj_progress,
        "progress_efficiency": prog_efficiency,
        "progress_consistency": prog_consistency,
        "worst_scenario_progress": worst_scenario_progress,
        "worst_min_sep": worst_sep,
        "omega_saturation": omega_sat,
        "omega_flip": omega_flip,
        "heading_error": heading_err,
        "cross_track_error": cte,
        "entanglement_ratio": entanglement,
        "colregs_compliance": colregs_comp,
        "colregs_violation": colregs_viol,
        "cpa_starboard_pass": cpa_sb,
        "breakdown": {
            "safety": round(safety, 1),
            "progress": round(prog_score, 1),
            "smoothness": round(smooth, 1),
            "tracking": round(tracking, 1),
            "colregs": round(colregs_sc, 1),
        },
    })

results.sort(key=lambda r: -r["composite_score"])
zero_coll = [r for r in results if r["collision_rate"] == 0]

ranking = {
    "evaluated_checkpoints": results,
    "best_composite": results[0]["checkpoint"] if results else None,
    "best_safe": zero_coll[0]["checkpoint"] if zero_coll else None,
    "total_evaluated": len(results),
    "zero_collision_count": len(zero_coll),
}

with open(ranking_path, "w") as f:
    json.dump(ranking, f, indent=2, ensure_ascii=False)

print(f"\nTotal: {len(results)}, Zero-collision: {len(zero_coll)}")
print(f"{'Rank':>4} {'Checkpoint':<40} {'Score':>6} {'Coll':>6} {'Prog':>6} {'COLRs':>6} {'CPA_sb':>6} {'Sep':>6}")
print("─" * 118)
for i, r in enumerate(results[:10]):
    tag = "✅" if r["collision_rate"] == 0 else ("⚠️" if r["collision_rate"] < 0.2 else "❌")
    print(f"{i+1:>4} {tag} {r['checkpoint']:<37} {r['composite_score']:>6.1f} {r['collision_rate']:>5.0%} {r['progress']:>6.3f} {r['colregs_compliance']:>5.1%} {r['cpa_starboard_pass']:>5.1%} {r['worst_min_sep']:>6.3f}")

if results:
    b = results[0]
    bd = b["breakdown"]
    print(f"\nBest: {b['checkpoint']} (score={b['composite_score']:.1f} = safety:{bd['safety']:.0f} + prog:{bd['progress']:.0f} + smooth:{bd['smoothness']:.0f} + track:{bd['tracking']:.0f} + colregs:{bd['colregs']:.0f})")

RANK_EOF

echo ""
echo "Ranking saved: $RANKING"
echo "═══════════════════════════════════════════════════"
echo "  fresh63 Parallel Evaluation Complete"
echo "═══════════════════════════════════════════════════"
