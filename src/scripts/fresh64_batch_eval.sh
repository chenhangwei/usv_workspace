#!/bin/bash
# fresh64 检查点批量评估脚本 v2
# 改进:
#   1. 新增 solo_navigation + random_encounter 场景 (共6场景全覆盖)
#   2. 每场景5个episode (共30), 统计更稳定
#   3. solo_navigation 单独评估 (导航专项), 避难场景另行评估
#   4. 评分公式重新加权: heading_error权重大幅提升 (fresh64核心优化目标)
#   5. 新增 per-scenario floor 惩罚 (任一场景崩溃则扣分)
#   6. 排名输出增加 heading_error / success_rate / per-scenario 明细
set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export ROS_DOMAIN_ID=123
export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh64_checkpoints"
PHASE3_FINAL="/mnt/data/checkpoints/usv_rl/fresh64_phase3.pt"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh64_eval"
RANKING="/mnt/data/checkpoints/usv_rl/fresh64_ranking.json"

# ── 评估参数 ──
SOLO_EPISODES=5              # solo_navigation 专项
MULTI_EPISODES=20            # 4场景×5=20 episodes (head_on, crossing, overtaking, random)
STEPS=275                    # 55s / 0.2s = 275 步

mkdir -p "$EVAL_DIR"

# 收集所有三阶段检查点 (每隔2个取样 + 各阶段 final)
ALL_CKPTS=$(ls "$CKPT_DIR"/fresh64_*_step_*.pt 2>/dev/null | sort -t_ -k4 -n)
CKPTS=""
IDX=0
for C in $ALL_CKPTS; do
  if (( IDX % 2 == 0 )); then
    CKPTS="$CKPTS $C"
  fi
  IDX=$((IDX + 1))
done
for FINAL in \
  "/mnt/data/checkpoints/usv_rl/fresh64_phase1.pt" \
  "/mnt/data/checkpoints/usv_rl/fresh64_phase2.pt" \
  "$PHASE3_FINAL"; do
  if [[ -f "$FINAL" ]]; then
    CKPTS="$CKPTS $FINAL"
  fi
done
# Deduplicate
CKPTS=$(echo $CKPTS | tr ' ' '\n' | sort -u | tr '\n' ' ')

CKPT_COUNT=$(echo $CKPTS | wc -w)

echo "═══════════════════════════════════════════════════"
echo "  fresh64 Batch Evaluation v2 (6-Scenario)"
echo "  Checkpoints: $CKPT_COUNT (sampled every-2 + finals)"
echo "  Solo eval:  $SOLO_EPISODES episodes × 1 scenario"
echo "  Multi eval: $MULTI_EPISODES episodes × 4 scenarios"
echo "  Steps per episode: $STEPS (55s equivalent)"
echo "═══════════════════════════════════════════════════"

TOTAL_DONE=0
for CKPT in $CKPTS; do
  BASENAME=$(basename "$CKPT" .pt)
  SOLO_JSON="$EVAL_DIR/${BASENAME}_solo_eval.json"
  MULTI_JSON="$EVAL_DIR/${BASENAME}_multi_eval.json"

  if [[ -f "$SOLO_JSON" ]] && [[ -f "$MULTI_JSON" ]]; then
    echo "SKIP: $BASENAME (both exist)"
    TOTAL_DONE=$((TOTAL_DONE + 1))
    continue
  fi

  TOTAL_DONE=$((TOTAL_DONE + 1))
  echo ""
  echo "──── [$TOTAL_DONE/$CKPT_COUNT] $BASENAME ────"

  # (A) solo_navigation 专项评估 — 纯导航能力 (heading/CTE/progress)
  if [[ ! -f "$SOLO_JSON" ]]; then
    echo "  [solo] running $SOLO_EPISODES episodes..."
    python3 -m usv_rl.evaluate_mappo_policy \
      --model "$CKPT" \
      --episodes "$SOLO_EPISODES" \
      --steps-per-episode "$STEPS" \
      --scenario solo_navigation \
      --output-json "$SOLO_JSON" \
      2>&1 | tail -2
  fi

  # (B) 多船避碰评估 — 4种场景
  if [[ ! -f "$MULTI_JSON" ]]; then
    echo "  [multi] running $MULTI_EPISODES episodes (4 scenarios)..."
    python3 -m usv_rl.evaluate_mappo_policy \
      --model "$CKPT" \
      --episodes "$MULTI_EPISODES" \
      --steps-per-episode "$STEPS" \
      --scenario two_usv_head_on \
      --scenario three_usv_crossing \
      --scenario three_usv_overtaking \
      --scenario three_usv_random_encounter \
      --output-json "$MULTI_JSON" \
      2>&1 | tail -2
  fi

  python3 -c "
import json
with open('$SOLO_JSON') as f:
    solo = json.load(f)
with open('$MULTI_JSON') as f:
    multi = json.load(f)
s_he = solo.get('mean_heading_error', -1)
s_cte = solo.get('mean_cross_track_error', -1)
s_prog = solo.get('mean_team_goal_progress_ratio', 0)
m_coll = multi.get('collision_rate', -1)
m_prog = multi.get('mean_team_goal_progress_ratio', 0)
m_sep = multi.get('worst_episode_min_separation', -1)
m_colr = multi.get('mean_colregs_compliance_ratio', -1)
tag = '✅' if m_coll == 0 else ('⚠️' if m_coll < 0.2 else '❌')
print(f'  {tag} solo: he={s_he:.3f}rad cte={s_cte:.3f}m prog={s_prog:.3f} | multi: coll={m_coll:.0%} prog={m_prog:.3f} sep={m_sep:.3f} COLREGs={m_colr:.1%}')
" 2>/dev/null || echo "  (parse error)"
done

echo ""
echo "Generating composite ranking..."
export EVAL_DIR RANKING
python3 << 'RANK_EOF'
import json, os, glob
import numpy as np

eval_dir = os.environ.get('EVAL_DIR', '/mnt/data/checkpoints/usv_rl/fresh64_eval')
ranking_path = os.environ.get('RANKING', '/mnt/data/checkpoints/usv_rl/fresh64_ranking.json')

# 收集配对的 solo + multi 评估结果
pairs = {}
for jf in sorted(glob.glob(os.path.join(eval_dir, "*_eval.json"))):
    fname = os.path.basename(jf)
    if fname.endswith("_solo_eval.json"):
        name = fname.replace("_solo_eval.json", "")
        pairs.setdefault(name, {})["solo"] = jf
    elif fname.endswith("_multi_eval.json"):
        name = fname.replace("_multi_eval.json", "")
        pairs.setdefault(name, {})["multi"] = jf

results = []
for name, files in sorted(pairs.items()):
    if "solo" not in files or "multi" not in files:
        continue
    with open(files["solo"]) as f:
        solo = json.load(f)
    with open(files["multi"]) as f:
        multi = json.load(f)

    # ── solo_navigation 指标 ──
    solo_heading_err = solo.get("mean_heading_error", 1.0)
    solo_cte = solo.get("mean_cross_track_error", 5.0)
    solo_progress = solo.get("mean_team_goal_progress_ratio", 0.0)
    solo_proj_progress = solo.get("mean_projected_progress", solo_progress)
    solo_omega_sat = solo.get("mean_omega_saturation_ratio", 0.0)
    solo_omega_flip = solo.get("mean_omega_flip_count", 0.0)
    solo_success = solo.get("success_rate", 0.0)

    # ── multi 避碰指标 ──
    collision_rate = multi.get("collision_rate", 1.0)
    multi_progress = multi.get("mean_team_goal_progress_ratio", 0.0)
    multi_proj_progress = multi.get("mean_projected_progress", multi_progress)
    multi_prog_efficiency = multi.get("mean_progress_efficiency", 0.0)
    multi_prog_consistency = multi.get("mean_progress_consistency", 0.0)
    worst_sep = multi.get("worst_episode_min_separation", 0.0)
    multi_omega_sat = multi.get("mean_omega_saturation_ratio", 0.0)
    multi_omega_flip = multi.get("mean_omega_flip_count", 0.0)
    multi_heading_err = multi.get("mean_heading_error", 1.0)
    multi_cte = multi.get("mean_cross_track_error", 5.0)
    multi_entanglement = multi.get("mean_entanglement_ratio", 0.0)
    colregs_comp = multi.get("mean_colregs_compliance_ratio", 0.0)
    colregs_viol = multi.get("mean_colregs_violation_ratio", 0.0)
    cpa_sb = multi.get("mean_cpa_starboard_pass_ratio", 0.0)
    multi_success = multi.get("success_rate", 0.0)

    # ════════════════════════════════════════════════
    # 综合评分 v2 (100分制)
    #   Navigation(30): solo 导航能力 (fresh64核心)
    #   Safety(20):     碰撞 + 最小间距
    #   Progress(15):   多船场景的前进效率
    #   Smoothness(10): 动作平滑度
    #   COLREGs(10):    避碰规则合规
    #   Floor(-15):     per-scenario 地板惩罚
    # ════════════════════════════════════════════════

    # Navigation (30分) — 基于solo_navigation, 但必须被真实到达证据约束
    #   heading_error: 0→0.3rad=满分, >1.2rad=0分 (fresh63水平70-100°=1.2-1.7rad)
    heading_quality = max(0.0, 1.0 - solo_heading_err / 1.2)
    #   CTE: 0→0.5m=满分, >3m=0分
    cte_quality = max(0.0, 1.0 - solo_cte / 3.0)
    #   progress: 使用真实team_goal_progress, 不再单独奖励projected_progress
    nav_progress = min(solo_progress, 1.0)
    navigation_raw = heading_quality * 14.0 + cte_quality * 8.0 + nav_progress * 8.0
    # success_rate 缺失时, 零动作策略会伪造出 he=0 / cte=0 / proj=1 的假满分。
    # 用 success 和真实 progress 共同 gate navigation, 避免未到达却拿满分。
    arrival_gate = min(1.0, 0.5 * solo_success + 0.5 * nav_progress)
    navigation_score = navigation_raw * arrival_gate

    # Safety (20分)
    safety_score = (1.0 - collision_rate) * 14.0 + min(worst_sep / 2.0, 1.0) * 6.0

    # Progress (15分) — 多船场景
    progress_score = (
        min(multi_proj_progress, 1.0) * 8.0
        + min(multi_prog_efficiency, 1.0) * 4.0
        + min(multi_prog_consistency, 1.0) * 3.0
    )

    # Smoothness (10分) — 综合solo和multi
    avg_omega_sat = 0.5 * solo_omega_sat + 0.5 * multi_omega_sat
    avg_omega_flip = 0.5 * solo_omega_flip + 0.5 * multi_omega_flip
    avg_heading_err = 0.5 * solo_heading_err + 0.5 * multi_heading_err
    smoothness_score = 10.0 * max(0.0, 1.0 - 0.4 * avg_omega_sat - 0.015 * avg_omega_flip - 0.5 * avg_heading_err)

    # COLREGs (10分)
    colregs_score = max(0.0, 10.0 * (0.5 * colregs_comp + 0.5 * cpa_sb) - 4.0 * colregs_viol)

    # ── Per-scenario floor 惩罚 (最多-15分) ──
    # 任何场景的progress过低 → 扣分, 防止"一好遮百丑"
    floor_penalty = 0.0
    multi_scenario_summaries = multi.get("scenario_summaries", {})
    all_scenario_progresses = {}

    # solo progress floor
    all_scenario_progresses["solo_navigation"] = nav_progress

    # multi per-scenario progress
    for sc_name, sc_data in multi_scenario_summaries.items():
        sc_prog = float(sc_data.get("mean_team_goal_progress_ratio", 0.0))
        all_scenario_progresses[sc_name] = sc_prog

    PROGRESS_FLOOR = 0.20  # 低于此值的场景触发惩罚
    for sc_name, sc_prog in all_scenario_progresses.items():
        if sc_prog < PROGRESS_FLOOR:
            shortfall = PROGRESS_FLOOR - sc_prog
            floor_penalty += shortfall * 30.0  # 每0.1 shortfall 扣3分
    floor_penalty = min(floor_penalty, 15.0)

    composite = round(
        navigation_score + safety_score + progress_score
        + smoothness_score + colregs_score - floor_penalty,
        2,
    )

    # 找出最差场景
    worst_scenario = min(all_scenario_progresses, key=all_scenario_progresses.get) if all_scenario_progresses else "N/A"
    worst_scenario_progress = min(all_scenario_progresses.values()) if all_scenario_progresses else 0.0

    results.append({
        "checkpoint": name,
        "composite_score": composite,
        # solo metrics
        "solo_heading_error": solo_heading_err,
        "solo_cte": solo_cte,
        "solo_progress": solo_progress,
        "solo_proj_progress": solo_proj_progress,
        "solo_success_rate": solo_success,
        "solo_omega_saturation": solo_omega_sat,
        # multi metrics
        "collision_rate": collision_rate,
        "multi_progress": multi_progress,
        "multi_proj_progress": multi_proj_progress,
        "multi_progress_efficiency": multi_prog_efficiency,
        "multi_progress_consistency": multi_prog_consistency,
        "multi_success_rate": multi_success,
        "worst_min_sep": worst_sep,
        "multi_omega_saturation": multi_omega_sat,
        "multi_omega_flip": multi_omega_flip,
        "multi_heading_error": multi_heading_err,
        "multi_cte": multi_cte,
        "multi_entanglement": multi_entanglement,
        "colregs_compliance": colregs_comp,
        "colregs_violation": colregs_viol,
        "cpa_starboard_pass": cpa_sb,
        # scenario analysis
        "worst_scenario": worst_scenario,
        "worst_scenario_progress": worst_scenario_progress,
        "per_scenario_progress": all_scenario_progresses,
        "floor_penalty": floor_penalty,
        "breakdown": {
            "navigation": round(navigation_score, 1),
            "safety": round(safety_score, 1),
            "progress": round(progress_score, 1),
            "smoothness": round(smoothness_score, 1),
            "colregs": round(colregs_score, 1),
            "floor_penalty": round(-floor_penalty, 1),
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
print()
print(f"{'Rank':>4} {'Checkpoint':<30} {'Score':>6} {'Nav':>5} {'Safe':>5} {'Prog':>5} {'Smth':>5} {'COLR':>5} {'Flr':>5} | {'soloHE':>7} {'soloCTE':>7} {'Coll':>5} {'mSep':>5}")
print("─" * 130)
for i, r in enumerate(results[:15]):
    tag = "✅" if r["collision_rate"] == 0 else ("⚠️" if r["collision_rate"] < 0.2 else "❌")
    bd = r["breakdown"]
    print(
        f"{i+1:>4} {tag} {r['checkpoint']:<27}"
        f" {r['composite_score']:>6.1f}"
        f" {bd['navigation']:>5.1f}"
        f" {bd['safety']:>5.1f}"
        f" {bd['progress']:>5.1f}"
        f" {bd['smoothness']:>5.1f}"
        f" {bd['colregs']:>5.1f}"
        f" {bd['floor_penalty']:>5.1f}"
        f" | {r['solo_heading_error']:>6.3f}r"
        f" {r['solo_cte']:>6.3f}m"
        f" {r['collision_rate']:>4.0%}"
        f" {r['worst_min_sep']:>5.3f}"
    )

if results:
    b = results[0]
    bd = b["breakdown"]
    print(f"\nBest: {b['checkpoint']} (score={b['composite_score']:.1f})")
    print(f"  Navigation: {bd['navigation']:.1f}/30  Safety: {bd['safety']:.1f}/20  Progress: {bd['progress']:.1f}/15  Smoothness: {bd['smoothness']:.1f}/10  COLREGs: {bd['colregs']:.1f}/10  Floor: {bd['floor_penalty']:.1f}")
    print(
        f"  Solo: heading_err={b['solo_heading_error']:.3f}rad ({np.degrees(b['solo_heading_error']):.1f}°) "
        f"CTE={b['solo_cte']:.3f}m actual_progress={b['solo_progress']:.3f} "
        f"proj_progress={b['solo_proj_progress']:.3f} success={b['solo_success_rate']:.1%}"
    )
    if b.get("per_scenario_progress"):
        print(f"  Per-scenario progress:")
        for sc, pr in sorted(b["per_scenario_progress"].items()):
            flag = "⚠️" if pr < 0.20 else "  "
            print(f"    {flag} {sc:<30s} {pr:.3f}")

RANK_EOF

echo ""
echo "Ranking: $RANKING"
echo "═══════════════════════════════════════════════════"
echo "  fresh64 Batch Evaluation v2 Complete"
