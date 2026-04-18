#!/bin/bash
# fresh64 并行批量评估脚本
# 同时运行 N 个评估进程, 充分利用 CPU 资源
# 已完成的检查点自动跳过 (幂等)
set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh64_checkpoints"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh64_eval"
RANKING="/mnt/data/checkpoints/usv_rl/fresh64_ranking.json"

PARALLEL=${1:-6}         # 并行数, 默认6
SOLO_EPISODES=5
MULTI_EPISODES=20
STEPS=275

mkdir -p "$EVAL_DIR"

# ── 收集检查点 (与 batch_eval 一致: 每隔2个取样 + 各阶段 final) ──
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
  "/mnt/data/checkpoints/usv_rl/fresh64_phase3.pt"; do
  if [[ -f "$FINAL" ]]; then
    CKPTS="$CKPTS $FINAL"
  fi
done
CKPTS=$(echo $CKPTS | tr ' ' '\n' | sort -u | tr '\n' ' ')
CKPT_COUNT=$(echo $CKPTS | wc -w)

# ── 过滤已完成的 ──
TODO_CKPTS=""
SKIP_COUNT=0
for CKPT in $CKPTS; do
  BASENAME=$(basename "$CKPT" .pt)
  SOLO_JSON="$EVAL_DIR/${BASENAME}_solo_eval.json"
  MULTI_JSON="$EVAL_DIR/${BASENAME}_multi_eval.json"
  if [[ -f "$SOLO_JSON" ]] && [[ -f "$MULTI_JSON" ]]; then
    SKIP_COUNT=$((SKIP_COUNT + 1))
  else
    TODO_CKPTS="$TODO_CKPTS $CKPT"
  fi
done
TODO_COUNT=$(echo $TODO_CKPTS | wc -w)

echo "═══════════════════════════════════════════════════"
echo "  fresh64 Parallel Evaluation (${PARALLEL} workers)"
echo "  Total checkpoints: $CKPT_COUNT"
echo "  Already done: $SKIP_COUNT"
echo "  Remaining: $TODO_COUNT"
echo "  Per checkpoint: solo=${SOLO_EPISODES}ep + multi=${MULTI_EPISODES}ep"
echo "═══════════════════════════════════════════════════"

if [[ $TODO_COUNT -eq 0 ]]; then
  echo "All checkpoints already evaluated. Generating ranking..."
  # 跳到排名阶段
else

# ── 单个检查点评估函数 ──
eval_one_checkpoint() {
  local CKPT="$1"
  local WORKER_ID="$2"
  local BASENAME
  BASENAME=$(basename "$CKPT" .pt)
  local SOLO_JSON="$EVAL_DIR/${BASENAME}_solo_eval.json"
  local MULTI_JSON="$EVAL_DIR/${BASENAME}_multi_eval.json"
  local ROS_DOMAIN_ID=$((200 + WORKER_ID))

  export ROS_DOMAIN_ID

  # solo evaluation
  if [[ ! -f "$SOLO_JSON" ]]; then
    python3 -m usv_rl.evaluate_mappo_policy \
      --model "$CKPT" \
      --episodes "$SOLO_EPISODES" \
      --steps-per-episode "$STEPS" \
      --scenario solo_navigation \
      --output-json "$SOLO_JSON" \
      2>/dev/null
  fi

  # multi evaluation
  if [[ ! -f "$MULTI_JSON" ]]; then
    python3 -m usv_rl.evaluate_mappo_policy \
      --model "$CKPT" \
      --episodes "$MULTI_EPISODES" \
      --steps-per-episode "$STEPS" \
      --scenario two_usv_head_on \
      --scenario three_usv_crossing \
      --scenario three_usv_overtaking \
      --scenario three_usv_random_encounter \
      --output-json "$MULTI_JSON" \
      2>/dev/null
  fi

  # 简短结果
  if [[ -f "$SOLO_JSON" ]] && [[ -f "$MULTI_JSON" ]]; then
    python3 -c "
import json
with open('$SOLO_JSON') as f: s=json.load(f)
with open('$MULTI_JSON') as f: m=json.load(f)
tag='✅' if m.get('collision_rate',1)==0 else ('⚠️' if m.get('collision_rate',1)<0.2 else '❌')
print(f'[W{$WORKER_ID}] {tag} $BASENAME he={s.get(\"mean_heading_error\",-1):.3f} cte={s.get(\"mean_cross_track_error\",-1):.3f} coll={m.get(\"collision_rate\",-1):.0%} prog={m.get(\"mean_team_goal_progress_ratio\",0):.3f}')
" 2>/dev/null || echo "[W${WORKER_ID}] done: $BASENAME"
  else
    echo "[W${WORKER_ID}] FAIL: $BASENAME"
  fi
}
export -f eval_one_checkpoint
export EVAL_DIR SOLO_EPISODES MULTI_EPISODES STEPS

# ── 并行执行 ──
echo ""
echo "Starting $PARALLEL parallel workers..."
echo ""

# 使用 GNU parallel 或 xargs 并行
WORKER_ID=0
PIDS=()
for CKPT in $TODO_CKPTS; do
  # 等待, 维持最多 PARALLEL 个并行进程
  while (( ${#PIDS[@]} >= PARALLEL )); do
    NEW_PIDS=()
    for PID in "${PIDS[@]}"; do
      if kill -0 "$PID" 2>/dev/null; then
        NEW_PIDS+=("$PID")
      fi
    done
    PIDS=("${NEW_PIDS[@]}")
    if (( ${#PIDS[@]} >= PARALLEL )); then
      sleep 2
    fi
  done

  WORKER_ID=$(( (WORKER_ID % PARALLEL) ))
  eval_one_checkpoint "$CKPT" "$WORKER_ID" &
  PIDS+=($!)
  WORKER_ID=$((WORKER_ID + 1))
done

# 等待所有完成
echo "All jobs dispatched. Waiting for completion..."
for PID in "${PIDS[@]}"; do
  wait "$PID" 2>/dev/null || true
done

echo ""
echo "All evaluations complete."
fi  # end of TODO_COUNT > 0 block

# ═══════════════════════════════════════════════════════
# 生成排名 (与 batch_eval v2 一致)
# ═══════════════════════════════════════════════════════
echo ""
echo "Generating composite ranking..."
export EVAL_DIR RANKING
python3 << 'RANK_EOF'
import json, os, glob
import numpy as np

eval_dir = os.environ.get('EVAL_DIR', '/mnt/data/checkpoints/usv_rl/fresh64_eval')
ranking_path = os.environ.get('RANKING', '/mnt/data/checkpoints/usv_rl/fresh64_ranking.json')

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

    solo_heading_err = solo.get("mean_heading_error", 1.0)
    solo_cte = solo.get("mean_cross_track_error", 5.0)
    solo_progress = solo.get("mean_team_goal_progress_ratio", 0.0)
    solo_proj_progress = solo.get("mean_projected_progress", solo_progress)
    solo_omega_sat = solo.get("mean_omega_saturation_ratio", 0.0)
    solo_omega_flip = solo.get("mean_omega_flip_count", 0.0)
    solo_success = solo.get("success_rate", 0.0)

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

    heading_quality = max(0.0, 1.0 - solo_heading_err / 1.2)
    cte_quality = max(0.0, 1.0 - solo_cte / 3.0)
    nav_progress = min(solo_progress, 1.0)
    navigation_raw = heading_quality * 14.0 + cte_quality * 8.0 + nav_progress * 8.0
    arrival_gate = min(1.0, 0.5 * solo_success + 0.5 * nav_progress)
    navigation_score = navigation_raw * arrival_gate

    safety_score = (1.0 - collision_rate) * 14.0 + min(worst_sep / 2.0, 1.0) * 6.0

    progress_score = (
        min(multi_proj_progress, 1.0) * 8.0
        + min(multi_prog_efficiency, 1.0) * 4.0
        + min(multi_prog_consistency, 1.0) * 3.0
    )

    avg_omega_sat = 0.5 * solo_omega_sat + 0.5 * multi_omega_sat
    avg_omega_flip = 0.5 * solo_omega_flip + 0.5 * multi_omega_flip
    avg_heading_err = 0.5 * solo_heading_err + 0.5 * multi_heading_err
    smoothness_score = 10.0 * max(0.0, 1.0 - 0.4 * avg_omega_sat - 0.015 * avg_omega_flip - 0.5 * avg_heading_err)

    colregs_score = max(0.0, 10.0 * (0.5 * colregs_comp + 0.5 * cpa_sb) - 4.0 * colregs_viol)

    floor_penalty = 0.0
    multi_scenario_summaries = multi.get("scenario_summaries", {})
    all_scenario_progresses = {}
    all_scenario_progresses["solo_navigation"] = nav_progress
    for sc_name, sc_data in multi_scenario_summaries.items():
        sc_prog = float(sc_data.get("mean_team_goal_progress_ratio", 0.0))
        all_scenario_progresses[sc_name] = sc_prog

    PROGRESS_FLOOR = 0.20
    for sc_name, sc_prog in all_scenario_progresses.items():
        if sc_prog < PROGRESS_FLOOR:
            shortfall = PROGRESS_FLOOR - sc_prog
            floor_penalty += shortfall * 30.0
    floor_penalty = min(floor_penalty, 15.0)

    composite = round(
        navigation_score + safety_score + progress_score
        + smoothness_score + colregs_score - floor_penalty,
        2,
    )

    worst_scenario = min(all_scenario_progresses, key=all_scenario_progresses.get) if all_scenario_progresses else "N/A"
    worst_scenario_progress = min(all_scenario_progresses.values()) if all_scenario_progresses else 0.0

    results.append({
        "checkpoint": name,
        "composite_score": composite,
        "solo_heading_error": solo_heading_err,
        "solo_cte": solo_cte,
        "solo_progress": solo_progress,
        "solo_proj_progress": solo_proj_progress,
        "solo_success_rate": solo_success,
        "solo_omega_saturation": solo_omega_sat,
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
echo "  fresh64 Parallel Evaluation Complete"
