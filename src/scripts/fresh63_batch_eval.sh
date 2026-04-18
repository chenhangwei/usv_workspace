#!/bin/bash
# fresh63 检查点批量评估脚本
# 改编自 fresh61_batch_eval.sh
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
EPISODES=12        # 4 per scenario
STEPS=275          # 55s / 0.2s = 275 步，匹配训练超时

mkdir -p "$EVAL_DIR"


# 每隔2个取样 + final（适配 fresh63_phase3_step_*.pt 命名）
ALL_CKPTS=$(ls "$CKPT_DIR"/fresh63_phase3_step_*.pt 2>/dev/null | sort -t_ -k4 -n)
CKPTS=""
IDX=0
for C in $ALL_CKPTS; do
  if (( IDX % 2 == 0 )); then
    CKPTS="$CKPTS $C"
  fi
  IDX=$((IDX + 1))
done
if [[ -f "$FINAL_CKPT" ]]; then
  CKPTS="$CKPTS $FINAL_CKPT"
fi

CKPT_COUNT=$(echo $CKPTS | wc -w)

echo "═══════════════════════════════════════════════════"
echo "  fresh63 Batch Evaluation (Composite Scoring)"
echo "  Checkpoints: $CKPT_COUNT (sampled every-2 + final)"
echo "  Episodes: $EPISODES × 3 scenarios = $(( EPISODES * 3 )) total"
echo "  Steps per episode: $STEPS (55s equivalent)"
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
progress = d.get('mean_team_goal_progress_ratio', 0)
proj_prog = d.get('mean_projected_progress', 0)
prog_eff = d.get('mean_progress_efficiency', 0)
sep = d.get('worst_episode_min_separation', -1)
colregs = d.get('mean_colregs_compliance_ratio', -1)
cpa_sb = d.get('mean_cpa_starboard_pass_ratio', -1)
tag = '✅' if coll == 0 else ('⚠️' if coll < 0.2 else '❌')
print(f'  {tag} coll={coll:.0%} prog={progress:.3f} proj={proj_prog:.3f} eff={prog_eff:.1%} sep={sep:.3f} COLREGs={colregs:.1%} CPA_sb={cpa_sb:.1%}')
" 2>/dev/null || echo "  (parse error)"
done

echo ""
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

    # 综合评分 (100分制, 越高越好)
    # 安全性 (0-35): 碰撞率 + 最小间距
    safety = (1.0 - collision_rate) * 25.0 + min(worst_sep / 2.0, 1.0) * 10.0
    # 进度 (0-25): 投射进度(15) + 速度利用率(5) + 推进一致性(5)
    prog_score = min(proj_progress, 1.0) * 15.0 + min(prog_efficiency, 1.0) * 5.0 + min(prog_consistency, 1.0) * 5.0
    # 平滑性 (0-15): 角速度饱和、翻转、航向误差
    smooth = 15.0 * max(0.0, 1.0 - 0.5 * omega_sat - 0.02 * omega_flip - 0.3 * heading_err)
    # 路径跟踪 (0-10): CTE + 纠缠
    tracking = 10.0 * max(0.0, 1.0 - 0.3 * cte - 0.5 * entanglement)
    # COLREGs 合规 (0-15): 避让方向正确率
    colregs_sc = max(0.0, 15.0 * (0.5 * colregs_comp + 0.5 * cpa_sb) - 5.0 * colregs_viol)

    composite = round(safety + prog_score + smooth + tracking + colregs_sc, 2)

    # 场景均衡性: 各场景进度的最小值
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

# 按综合评分排序 (降序)
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

# 输出 Top-10 排名表
print(f"\nTotal: {len(results)}, Zero-collision: {len(zero_coll)}")
print(f"{'Rank':>4} {'Checkpoint':<30} {'Score':>6} {'Coll':>6} {'Prog':>6} {'COLRs':>6} {'CPA_sb':>6} {'ω_sat':>6} {'CTE':>6} {'Sep':>6}")
print("─" * 108)
for i, r in enumerate(results[:10]):
    tag = "✅" if r["collision_rate"] == 0 else ("⚠️" if r["collision_rate"] < 0.2 else "❌")
    print(f"{i+1:>4} {tag} {r['checkpoint']:<27} {r['composite_score']:>6.1f} {r['collision_rate']:>5.0%} {r['progress']:>6.3f} {r['colregs_compliance']:>5.1%} {r['cpa_starboard_pass']:>5.1%} {r['omega_saturation']:>5.1%} {r['cross_track_error']:>6.3f} {r['worst_min_sep']:>6.3f}")

if results:
    b = results[0]
    bd = b["breakdown"]
    print(f"\nBest: {b['checkpoint']} (score={b['composite_score']:.1f} = safety:{bd['safety']:.0f} + prog:{bd['progress']:.0f} + smooth:{bd['smoothness']:.0f} + track:{bd['tracking']:.0f} + colregs:{bd['colregs']:.0f})")

RANK_EOF

echo ""
echo "Ranking: $RANKING"
echo "═══════════════════════════════════════════════════"
echo "  fresh63 Batch Evaluation Complete"
