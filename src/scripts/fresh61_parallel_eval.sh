#!/bin/bash
# fresh61 三阶段智能评估 — 并行版
# Phase 1: 粗扫描 (每5个取1, 6 episodes, 6路并行)
# Phase 2: 精筛选 (Top-15 邻域, 12 episodes, 6路并行)
# Phase 3: 精排名 (Top-5, 24 episodes, 5路并行)
#
# 预估耗时: ~2.5h（vs 旧串行版 ~18h）
set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh61_checkpoints"
FINAL_CKPT="/mnt/data/checkpoints/usv_rl/fresh61.pt"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh61_eval"
RANKING="/mnt/data/checkpoints/usv_rl/fresh61_ranking.json"

STEPS=275
MAX_PARALLEL=6        # 同时跑6个评估进程
BASE_DOMAIN_ID=120    # 每个进程用 120+slot 的 ROS_DOMAIN_ID

mkdir -p "$EVAL_DIR"

ALL_CKPTS=$(ls "$CKPT_DIR"/fresh61_step_*.pt 2>/dev/null | sort -t_ -k3 -n)
TOTAL_CKPTS=$(echo "$ALL_CKPTS" | wc -w)

# ─────────────────────────────────────────
# 并行评估核心函数
# ─────────────────────────────────────────
run_eval_parallel() {
  local ckpt_list="$1" episodes="$2" suffix="$3" phase_name="$4"

  local ckpt_array=()
  for c in $ckpt_list; do
    ckpt_array+=("$c")
  done
  local total=${#ckpt_array[@]}

  echo "  Checkpoints: $total, Episodes: $episodes, Parallel: $MAX_PARALLEL"
  echo ""

  local running_pids=()
  local running_slots=()
  local running_names=()
  local done=0
  local launched=0

  while (( done < total )); do
    # 启动新任务直到填满并行槽
    while (( ${#running_pids[@]} < MAX_PARALLEL && launched < total )); do
      local ckpt="${ckpt_array[$launched]}"
      local basename=$(basename "$ckpt" .pt)
      local out_json="$EVAL_DIR/${basename}${suffix}_eval.json"

      # 找空闲slot
      local slot=-1
      for s in $(seq 0 $((MAX_PARALLEL-1))); do
        local used=0
        for rs in "${running_slots[@]}"; do
          [[ "$rs" == "$s" ]] && used=1 && break
        done
        [[ $used == 0 ]] && slot=$s && break
      done

      local domain_id=$((BASE_DOMAIN_ID + slot))

      if [[ -f "$out_json" ]]; then
        echo "  [$((done+1))/$total] $basename — CACHE HIT"
        done=$((done + 1))
        launched=$((launched + 1))
        continue
      fi

      # 启动后台评估
      ROS_DOMAIN_ID=$domain_id python3 -m usv_rl.evaluate_mappo_policy \
        --model "$ckpt" \
        --episodes "$episodes" \
        --steps-per-episode "$STEPS" \
        --scenario two_usv_head_on \
        --scenario three_usv_crossing \
        --scenario three_usv_overtaking \
        --output-json "$out_json" \
        > /dev/null 2>&1 &

      local pid=$!
      running_pids+=($pid)
      running_slots+=($slot)
      running_names+=("$basename")
      launched=$((launched + 1))
    done

    # 等待任意一个完成
    if (( ${#running_pids[@]} > 0 )); then
      # 轮询检查哪些进程已退出
      while true; do
        local new_pids=()
        local new_slots=()
        local new_names=()
        local any_finished=0

        for i in "${!running_pids[@]}"; do
          if kill -0 "${running_pids[$i]}" 2>/dev/null; then
            new_pids+=("${running_pids[$i]}")
            new_slots+=("${running_slots[$i]}")
            new_names+=("${running_names[$i]}")
          else
            wait "${running_pids[$i]}" 2>/dev/null
            done=$((done + 1))
            local bn="${running_names[$i]}"
            local oj="$EVAL_DIR/${bn}${suffix}_eval.json"
            if [[ -f "$oj" ]]; then
              local summary=$(python3 -c "
import json
with open('$oj') as f:
    d = json.load(f)
c=d.get('collision_rate',-1)
p=d.get('mean_team_goal_progress_ratio',0)
s=d.get('worst_episode_min_separation',-1)
tag='✅' if c==0 else ('⚠️' if c<0.2 else '❌')
print(f'{tag} coll={c:.0%} prog={p:.3f} sep={s:.3f}')
" 2>/dev/null || echo "?")
              echo "  [$done/$total] $bn — $summary"
            else
              echo "  [$done/$total] $bn — FAILED"
            fi
            any_finished=1
          fi
        done

        running_pids=("${new_pids[@]}")
        running_slots=("${new_slots[@]}")
        running_names=("${new_names[@]}")

        [[ $any_finished == 1 ]] && break
        sleep 2
      done
    fi
  done
}

# ─────────────────────────────────────────
# 综合评分计算 (Python helper)
# ─────────────────────────────────────────
composite_rank_python() {
  local eval_dir="$1" top_n="$2" ranking_path="$3"
  python3 << PYEOF
import json, os, glob, re

eval_dir = "$eval_dir"
top_n = int("$top_n")
ranking_path = "$ranking_path" if "$ranking_path" else None

def composite_breakdown(d):
    collision_rate = d.get("collision_rate", 1.0)
    proj_progress = d.get("mean_projected_progress", d.get("mean_team_goal_progress_ratio", 0.0))
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
    return round(safety + prog_score + smooth + tracking + colregs_sc, 2), {
        "safety": round(safety, 1), "progress": round(prog_score, 1),
        "smoothness": round(smooth, 1), "tracking": round(tracking, 1),
        "colregs": round(colregs_sc, 1),
    }

# Collect all unique basenames, prefer highest-phase eval
seen_best = {}  # basename -> (phase_priority, path)
for f in sorted(os.listdir(eval_dir)):
    if not f.endswith('.json'):
        continue
    for suffix, priority in [("_p3_eval.json", 3), ("_p2_eval.json", 2), ("_eval.json", 1)]:
        if f.endswith(suffix):
            bn = f[:-len(suffix)]
            if bn not in seen_best or priority > seen_best[bn][0]:
                seen_best[bn] = (priority, os.path.join(eval_dir, f))
            break

results = []
for bn, (pri, path) in seen_best.items():
    with open(path) as f:
        d = json.load(f)
    score, bd = composite_breakdown(d)
    results.append({
        "checkpoint": bn, "phase": f"p{pri}",
        "composite_score": score, "breakdown": bd,
        "collision_rate": d.get("collision_rate", 1.0),
        "progress": d.get("mean_team_goal_progress_ratio", 0.0),
        "projected_progress": d.get("mean_projected_progress", 0.0),
        "worst_min_sep": d.get("worst_episode_min_separation", 0.0),
        "colregs_compliance": d.get("mean_colregs_compliance_ratio", 0.0),
        "cpa_starboard_pass": d.get("mean_cpa_starboard_pass_ratio", 0.0),
        "cross_track_error": d.get("mean_cross_track_error", 0.0),
        "entanglement_ratio": d.get("mean_entanglement_ratio", 0.0),
    })

results.sort(key=lambda r: -r["composite_score"])

# Print table
print(f"  Total: {len(results)}, Zero-coll: {sum(1 for r in results if r['collision_rate']==0)}")
print(f"  {'Rk':>3} {'Ph':>2} {'Checkpoint':<30} {'Score':>6} {'Coll':>5} {'Prog':>6} {'COLRs':>5} {'Sep':>6} {'CTE':>5}")
print("  " + "─" * 90)
for i, r in enumerate(results[:min(top_n, 20)]):
    tag = "✅" if r["collision_rate"] == 0 else ("⚠️" if r["collision_rate"] < 0.2 else "❌")
    print(f"  {i+1:>3} {r['phase']:>2} {tag}{r['checkpoint']:<28} {r['composite_score']:>6.1f} {r['collision_rate']:>4.0%} {r['progress']:>6.3f} {r['colregs_compliance']:>4.1%} {r['worst_min_sep']:>6.3f} {r['cross_track_error']:>5.3f}")

# Output top-N step numbers
top_steps = []
for r in results[:top_n]:
    m = re.search(r'step_(\d+)', r['checkpoint'])
    if m:
        top_steps.append(int(m.group(1)))
    elif r['checkpoint'] == 'fresh61':
        top_steps.append(500000)
print("TOP_STEPS=" + ",".join(str(s) for s in top_steps))

if r := results[0] if results else None:
    bd = r['breakdown']
    print(f"  🏆 Best: {r['checkpoint']} (score={r['composite_score']:.1f} = S:{bd['safety']:.0f}+P:{bd['progress']:.0f}+Sm:{bd['smoothness']:.0f}+T:{bd['tracking']:.0f}+C:{bd['colregs']:.0f})")

# Save ranking if path given
if ranking_path:
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
    print(f"  Ranking saved: {ranking_path}")
PYEOF
}

# ═══════════════════════════════════════════════════
# Phase 1: 粗扫描 — 每5个取1个 + final, 6 episodes, 6路并行
# ═══════════════════════════════════════════════════
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Phase 1: 粗扫描 (every 5th, 6 ep, ${MAX_PARALLEL}x parallel)  ║"
echo "╚═══════════════════════════════════════════════════════════╝"

PHASE1_CKPTS=""
IDX=0
for C in $ALL_CKPTS; do
  if (( IDX % 5 == 0 )); then
    PHASE1_CKPTS="$PHASE1_CKPTS $C"
  fi
  IDX=$((IDX + 1))
done
[[ -f "$FINAL_CKPT" ]] && PHASE1_CKPTS="$PHASE1_CKPTS $FINAL_CKPT"

P1_START=$(date +%s)
run_eval_parallel "$PHASE1_CKPTS" 6 "" "Phase1"
P1_END=$(date +%s)
echo ""
echo "  Phase 1 done in $(( (P1_END - P1_START) / 60 ))min $(( (P1_END - P1_START) % 60 ))s"
echo ""

# Phase 1 ranking → Top-15
echo "  Phase 1 排名 (selecting Top-15):"
P1_RESULT=$(composite_rank_python "$EVAL_DIR" 15 "")
echo "$P1_RESULT"
TOP_STEPS=$(echo "$P1_RESULT" | grep "^TOP_STEPS=" | cut -d= -f2)

# ═══════════════════════════════════════════════════
# Phase 2: 精筛选 — Top-15 ±2邻居, 12 episodes, 6路并行
# ═══════════════════════════════════════════════════
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Phase 2: 精筛选 (Top-15 neighbors, 12 ep, ${MAX_PARALLEL}x)   ║"
echo "╚═══════════════════════════════════════════════════════════╝"

PHASE2_CKPTS=$(python3 << P2SELECT
import os, re

ckpt_dir = "$CKPT_DIR"
final = "$FINAL_CKPT"

all_files = sorted(
    [f for f in os.listdir(ckpt_dir) if f.endswith('.pt')],
    key=lambda f: int(re.search(r'step_(\d+)', f).group(1)) if re.search(r'step_(\d+)', f) else 0
)
all_steps = []
for f in all_files:
    m = re.search(r'step_(\d+)', f)
    if m:
        all_steps.append((int(m.group(1)), os.path.join(ckpt_dir, f)))

top_steps_str = "$TOP_STEPS"
top_steps = [int(s) for s in top_steps_str.split(",") if s.strip()]

step_to_idx = {s: i for i, (s, _) in enumerate(all_steps)}
selected_indices = set()
for ts in top_steps:
    if ts in step_to_idx:
        idx = step_to_idx[ts]
        for offset in range(-2, 3):
            ni = idx + offset
            if 0 <= ni < len(all_steps):
                selected_indices.add(ni)

# 过滤掉 Phase 1 中碰撞率 > 50% 的 (已有 _eval.json)
import json
eval_dir = "$EVAL_DIR"
for i in sorted(selected_indices):
    step, path = all_steps[i]
    bn = os.path.basename(path).replace('.pt', '')
    p1_json = os.path.join(eval_dir, f"{bn}_eval.json")
    if os.path.isfile(p1_json):
        with open(p1_json) as f:
            d = json.load(f)
        if d.get("collision_rate", 1.0) > 0.5:
            continue  # skip high-collision checkpoints
    print(path)

if os.path.isfile(final):
    print(final)
P2SELECT
)

P2_START=$(date +%s)
run_eval_parallel "$PHASE2_CKPTS" 12 "_p2" "Phase2"
P2_END=$(date +%s)
echo ""
echo "  Phase 2 done in $(( (P2_END - P2_START) / 60 ))min $(( (P2_END - P2_START) % 60 ))s"
echo ""

echo "  Phase 2 排名 (selecting Top-5):"
P2_RESULT=$(composite_rank_python "$EVAL_DIR" 5 "")
echo "$P2_RESULT"
TOP_STEPS=$(echo "$P2_RESULT" | grep "^TOP_STEPS=" | cut -d= -f2)

# ═══════════════════════════════════════════════════
# Phase 3: 精确排名 — Top-5, 24 episodes, 5路并行
# ═══════════════════════════════════════════════════
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Phase 3: 精确排名 (Top-5, 24 ep, parallel)              ║"
echo "╚═══════════════════════════════════════════════════════════╝"

PHASE3_CKPTS=$(python3 << P3SELECT
import os, re

ckpt_dir = "$CKPT_DIR"
final = "$FINAL_CKPT"
top_steps_str = "$TOP_STEPS"
top_steps = [int(s) for s in top_steps_str.split(",") if s.strip()]

for ts in top_steps:
    if ts == 500000 and os.path.isfile(final):
        print(final)
    else:
        p = os.path.join(ckpt_dir, f"fresh61_step_{ts:07d}.pt")
        if os.path.isfile(p):
            print(p)
P3SELECT
)

P3_START=$(date +%s)
run_eval_parallel "$PHASE3_CKPTS" 24 "_p3" "Phase3"
P3_END=$(date +%s)
echo ""
echo "  Phase 3 done in $(( (P3_END - P3_START) / 60 ))min $(( (P3_END - P3_START) % 60 ))s"

# ═══════════════════════════════════════════════════
# Final Ranking
# ═══════════════════════════════════════════════════
TOTAL_TIME=$(( $(date +%s) - P1_START ))
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║  Final Ranking                                           ║"
echo "╚═══════════════════════════════════════════════════════════╝"
composite_rank_python "$EVAL_DIR" 999 "$RANKING"
echo ""
echo "═══════════════════════════════════════════════════"
echo "  Total time: $((TOTAL_TIME / 3600))h $((TOTAL_TIME % 3600 / 60))m"
echo "  fresh61 Smart Parallel Evaluation Complete"
echo "═══════════════════════════════════════════════════"
