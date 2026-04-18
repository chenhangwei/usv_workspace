#!/bin/bash
# fresh61 三阶段智能评估
# Phase 1: 粗扫描 (每5个取1, 6 episodes, 早停)
# Phase 2: 精筛选 (Top-15 邻域密集, 12 episodes)
# Phase 3: 精排名 (Top-5, 24 episodes)
set -eo pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export ROS_DOMAIN_ID=118
export PYTHONPATH="/home/chenhangwei/usv_workspace/build/usv_rl:$PYTHONPATH"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh61_checkpoints"
FINAL_CKPT="/mnt/data/checkpoints/usv_rl/fresh61.pt"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh61_eval"
RANKING="/mnt/data/checkpoints/usv_rl/fresh61_ranking.json"

SCENARIOS="two_usv_head_on three_usv_crossing three_usv_overtaking"
STEPS=275

mkdir -p "$EVAL_DIR"

ALL_CKPTS=$(ls "$CKPT_DIR"/fresh61_step_*.pt 2>/dev/null | sort -t_ -k3 -n)
TOTAL_CKPTS=$(echo "$ALL_CKPTS" | wc -w)

run_eval() {
  local ckpt="$1" episodes="$2" out_json="$3"
  if [[ -f "$out_json" ]]; then
    echo "  CACHE HIT"
    return 0
  fi
  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$ckpt" \
    --episodes "$episodes" \
    --steps-per-episode "$STEPS" \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --output-json "$out_json" \
    2>&1 | tail -2
}

print_summary() {
  local json_file="$1"
  python3 -c "
import json, sys
with open('$json_file') as f:
    d = json.load(f)
coll = d.get('collision_rate', -1)
prog = d.get('mean_team_goal_progress_ratio', 0)
proj = d.get('mean_projected_progress', 0)
sep = d.get('worst_episode_min_separation', -1)
colregs = d.get('mean_colregs_compliance_ratio', -1)
cpa = d.get('mean_cpa_starboard_pass_ratio', -1)
tag = '✅' if coll == 0 else ('⚠️' if coll < 0.2 else '❌')
print(f'  {tag} coll={coll:.0%} prog={prog:.3f} proj={proj:.3f} sep={sep:.3f} COLREGs={colregs:.1%} CPA_sb={cpa:.1%}')
" 2>/dev/null || echo "  (parse error)"
}

compute_composite() {
  # print composite score for a json file
  python3 -c "
import json, sys
with open('$1') as f:
    d = json.load(f)
collision_rate = d.get('collision_rate', 1.0)
progress = d.get('mean_team_goal_progress_ratio', 0.0)
proj_progress = d.get('mean_projected_progress', progress)
prog_efficiency = d.get('mean_progress_efficiency', 0.0)
prog_consistency = d.get('mean_progress_consistency', 0.0)
worst_sep = d.get('worst_episode_min_separation', 0.0)
omega_sat = d.get('mean_omega_saturation_ratio', 0.0)
omega_flip = d.get('mean_omega_flip_count', 0.0)
heading_err = d.get('mean_heading_error', 0.0)
cte = d.get('mean_cross_track_error', 0.0)
entanglement = d.get('mean_entanglement_ratio', 0.0)
colregs_comp = d.get('mean_colregs_compliance_ratio', 0.0)
colregs_viol = d.get('mean_colregs_violation_ratio', 0.0)
cpa_sb = d.get('mean_cpa_starboard_pass_ratio', 0.0)
safety = (1.0 - collision_rate) * 25.0 + min(worst_sep / 2.0, 1.0) * 10.0
prog_score = min(proj_progress, 1.0) * 15.0 + min(prog_efficiency, 1.0) * 5.0 + min(prog_consistency, 1.0) * 5.0
smooth = 15.0 * max(0.0, 1.0 - 0.5 * omega_sat - 0.02 * omega_flip - 0.3 * heading_err)
tracking = 10.0 * max(0.0, 1.0 - 0.3 * cte - 0.5 * entanglement)
colregs_sc = max(0.0, 15.0 * (0.5 * colregs_comp + 0.5 * cpa_sb) - 5.0 * colregs_viol)
composite = safety + prog_score + smooth + tracking + colregs_sc
print(f'{composite:.2f}')
" 2>/dev/null || echo "0"
}

# ═══════════════════════════════════════════════════
# Phase 1: 粗扫描 — 每5个取1个 + final, 6 episodes
# ═══════════════════════════════════════════════════
echo ""
echo "╔═══════════════════════════════════════════════════╗"
echo "║  Phase 1: 粗扫描 (every 5th, 6 episodes)         ║"
echo "╚═══════════════════════════════════════════════════╝"

PHASE1_CKPTS=""
IDX=0
for C in $ALL_CKPTS; do
  if (( IDX % 5 == 0 )); then
    PHASE1_CKPTS="$PHASE1_CKPTS $C"
  fi
  IDX=$((IDX + 1))
done
if [[ -f "$FINAL_CKPT" ]]; then
  PHASE1_CKPTS="$PHASE1_CKPTS $FINAL_CKPT"
fi
P1_COUNT=$(echo $PHASE1_CKPTS | wc -w)
echo "  Checkpoints: $P1_COUNT / $TOTAL_CKPTS"

P1_DONE=0
for CKPT in $PHASE1_CKPTS; do
  BASENAME=$(basename "$CKPT" .pt)
  OUT_JSON="$EVAL_DIR/${BASENAME}_eval.json"
  P1_DONE=$((P1_DONE + 1))
  echo "── P1 [$P1_DONE/$P1_COUNT] $BASENAME ──"
  run_eval "$CKPT" 6 "$OUT_JSON"
  print_summary "$OUT_JSON"
done

echo ""
echo "Phase 1 complete. Selecting Top-15..."

# Phase 1 排名 → 取 Top-15
PHASE1_RANKING=$(python3 << 'P1RANK'
import json, os, glob

eval_dir = "/mnt/data/checkpoints/usv_rl/fresh61_eval"
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
    composite = safety + prog_score + smooth + tracking + colregs_sc
    results.append((composite, name))

results.sort(key=lambda x: -x[0])
# Print Top-15 names and scores
print("Phase 1 Top-15:")
for i, (score, name) in enumerate(results[:15]):
    tag = "🥇" if i == 0 else ("🥈" if i == 1 else ("🥉" if i == 2 else "  "))
    print(f"  {i+1:>2}. {tag} {name:<30} score={score:.1f}")

# Extract step numbers of top-15
import re
top_steps = []
for _, name in results[:15]:
    m = re.search(r'step_(\d+)', name)
    if m:
        top_steps.append(int(m.group(1)))
    elif name == "fresh61":
        top_steps.append(500000)
# Output step numbers for Phase 2 neighbor expansion
print("TOP_STEPS=" + ",".join(str(s) for s in top_steps))
P1RANK
)

echo "$PHASE1_RANKING"
TOP_STEPS=$(echo "$PHASE1_RANKING" | grep "^TOP_STEPS=" | cut -d= -f2)

# ═══════════════════════════════════════════════════
# Phase 2: 精筛选 — Top-15 各±2邻居, 12 episodes
# ═══════════════════════════════════════════════════
echo ""
echo "╔═══════════════════════════════════════════════════╗"
echo "║  Phase 2: 精筛选 (neighbors of Top-15, 12 ep)     ║"
echo "╚═══════════════════════════════════════════════════╝"

# Build Phase 2 checkpoint list: for each top step, include ±1, ±2 neighbors
PHASE2_CKPTS=$(python3 << P2SELECT
import os, re

ckpt_dir = "/mnt/data/checkpoints/usv_rl/fresh61_checkpoints"
final = "/mnt/data/checkpoints/usv_rl/fresh61.pt"

# All available checkpoints sorted by step
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

# For each top step, find indices and expand ±2
step_to_idx = {s: i for i, (s, _) in enumerate(all_steps)}
selected_indices = set()
for ts in top_steps:
    if ts in step_to_idx:
        idx = step_to_idx[ts]
        for offset in range(-2, 3):
            ni = idx + offset
            if 0 <= ni < len(all_steps):
                selected_indices.add(ni)
    elif ts == 500000:  # final checkpoint
        pass  # handled separately

selected = [all_steps[i][1] for i in sorted(selected_indices)]
if os.path.isfile(final):
    selected.append(final)

for s in selected:
    print(s)
P2SELECT
)

P2_COUNT=$(echo "$PHASE2_CKPTS" | wc -l)
echo "  Phase 2 checkpoints: $P2_COUNT (Top-15 × ±2 neighbors)"

P2_DONE=0
while IFS= read -r CKPT; do
  [[ -z "$CKPT" ]] && continue
  BASENAME=$(basename "$CKPT" .pt)
  OUT_JSON="$EVAL_DIR/${BASENAME}_eval.json"

  # 如果已有 Phase 1 的 6-ep 结果，先用它筛选：碰撞率>60%的跳过
  if [[ -f "$OUT_JSON" ]]; then
    COLL=$(python3 -c "import json; d=json.load(open('$OUT_JSON')); print(d.get('collision_rate',1))" 2>/dev/null)
    if python3 -c "exit(0 if float('$COLL') > 0.6 else 1)" 2>/dev/null; then
      P2_DONE=$((P2_DONE + 1))
      echo "── P2 [$P2_DONE/$P2_COUNT] $BASENAME ── SKIP (coll=${COLL})"
      continue
    fi
  fi

  # Phase 2 uses 12 episodes — remove old 6-ep result to re-eval with more episodes
  P2_JSON="$EVAL_DIR/${BASENAME}_p2_eval.json"
  P2_DONE=$((P2_DONE + 1))
  echo "── P2 [$P2_DONE/$P2_COUNT] $BASENAME ──"
  run_eval "$CKPT" 12 "$P2_JSON"
  print_summary "$P2_JSON"
done <<< "$PHASE2_CKPTS"

echo ""
echo "Phase 2 complete. Selecting Top-5..."

# Phase 2 排名 → 取 Top-5 (优先用 p2 结果)
PHASE2_RANKING=$(python3 << 'P2RANK'
import json, os, glob, re

eval_dir = "/mnt/data/checkpoints/usv_rl/fresh61_eval"

def load_best_eval(basename):
    """Load p2 result if available, otherwise p1"""
    p2 = os.path.join(eval_dir, f"{basename}_p2_eval.json")
    p1 = os.path.join(eval_dir, f"{basename}_eval.json")
    path = p2 if os.path.isfile(p2) else p1
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return json.load(f)

def composite(d):
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
    return safety + prog_score + smooth + tracking + colregs_sc

# Collect all unique basenames
seen = set()
basenames = []
for f in os.listdir(eval_dir):
    if not f.endswith('.json'):
        continue
    bn = f.replace('_p2_eval.json', '').replace('_eval.json', '')
    if bn not in seen:
        seen.add(bn)
        basenames.append(bn)

results = []
for bn in basenames:
    d = load_best_eval(bn)
    if d is None:
        continue
    score = composite(d)
    results.append((score, bn, d.get('collision_rate', 1.0)))

results.sort(key=lambda x: -x[0])
print("Phase 2 Top-10:")
for i, (score, name, coll) in enumerate(results[:10]):
    tag = "✅" if coll == 0 else ("⚠️" if coll < 0.2 else "❌")
    print(f"  {i+1:>2}. {tag} {name:<30} score={score:.1f} coll={coll:.0%}")

# Output top-5 checkpoint paths for Phase 3
import os
ckpt_dir = "/mnt/data/checkpoints/usv_rl/fresh61_checkpoints"
final = "/mnt/data/checkpoints/usv_rl/fresh61.pt"
top5_paths = []
for _, name, _ in results[:5]:
    if name == "fresh61":
        top5_paths.append(final)
    else:
        p = os.path.join(ckpt_dir, name + ".pt")
        if os.path.isfile(p):
            top5_paths.append(p)
print("TOP5=" + "|".join(top5_paths))
P2RANK
)

echo "$PHASE2_RANKING"
TOP5=$(echo "$PHASE2_RANKING" | grep "^TOP5=" | cut -d= -f2)

# ═══════════════════════════════════════════════════
# Phase 3: 精确排名 — Top-5, 24 episodes
# ═══════════════════════════════════════════════════
echo ""
echo "╔═══════════════════════════════════════════════════╗"
echo "║  Phase 3: 精确排名 (Top-5, 24 episodes)           ║"
echo "╚═══════════════════════════════════════════════════╝"

IFS='|' read -ra P3_ARRAY <<< "$TOP5"
P3_COUNT=${#P3_ARRAY[@]}
echo "  Final candidates: $P3_COUNT"

P3_DONE=0
for CKPT in "${P3_ARRAY[@]}"; do
  [[ -z "$CKPT" ]] && continue
  BASENAME=$(basename "$CKPT" .pt)
  P3_JSON="$EVAL_DIR/${BASENAME}_p3_eval.json"
  P3_DONE=$((P3_DONE + 1))
  echo "── P3 [$P3_DONE/$P3_COUNT] $BASENAME (24 episodes) ──"
  run_eval "$CKPT" 24 "$P3_JSON"
  print_summary "$P3_JSON"
done

# ═══════════════════════════════════════════════════
# Final Ranking
# ═══════════════════════════════════════════════════
echo ""
echo "╔═══════════════════════════════════════════════════╗"
echo "║  Final Ranking (综合评分)                          ║"
echo "╚═══════════════════════════════════════════════════╝"

python3 << 'FINAL_RANK'
import json, os, glob, re

eval_dir = "/mnt/data/checkpoints/usv_rl/fresh61_eval"
ranking_path = "/mnt/data/checkpoints/usv_rl/fresh61_ranking.json"

def composite_breakdown(d):
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
    return {
        "composite": round(safety + prog_score + smooth + tracking + colregs_sc, 2),
        "safety": round(safety, 1),
        "progress": round(prog_score, 1),
        "smoothness": round(smooth, 1),
        "tracking": round(tracking, 1),
        "colregs": round(colregs_sc, 1),
    }

# Load best available eval for each checkpoint (p3 > p2 > p1)
seen = set()
basenames = []
for f in sorted(os.listdir(eval_dir)):
    if not f.endswith('.json'):
        continue
    bn = f.replace('_p3_eval.json', '').replace('_p2_eval.json', '').replace('_eval.json', '')
    if bn not in seen:
        seen.add(bn)
        basenames.append(bn)

results = []
for bn in basenames:
    # Priority: p3 > p2 > p1
    best_path = None
    best_phase = "p1"
    for suffix, phase in [("_p3_eval.json", "p3"), ("_p2_eval.json", "p2"), ("_eval.json", "p1")]:
        p = os.path.join(eval_dir, bn + suffix)
        if os.path.isfile(p):
            best_path = p
            best_phase = phase
            break
    if best_path is None:
        continue
    with open(best_path) as f:
        d = json.load(f)
    bd = composite_breakdown(d)

    scenario_summaries = d.get("scenario_summaries", {})
    per_scenario_progress = [
        float(v.get("mean_team_goal_progress_ratio", 0.0))
        for v in scenario_summaries.values()
    ]
    worst_scenario_progress = min(per_scenario_progress) if per_scenario_progress else 0.0

    results.append({
        "checkpoint": bn,
        "phase": best_phase,
        "composite_score": bd["composite"],
        "collision_rate": d.get("collision_rate", 1.0),
        "progress": d.get("mean_team_goal_progress_ratio", 0.0),
        "projected_progress": d.get("mean_projected_progress", 0.0),
        "progress_efficiency": d.get("mean_progress_efficiency", 0.0),
        "worst_scenario_progress": worst_scenario_progress,
        "worst_min_sep": d.get("worst_episode_min_separation", 0.0),
        "omega_saturation": d.get("mean_omega_saturation_ratio", 0.0),
        "heading_error": d.get("mean_heading_error", 0.0),
        "cross_track_error": d.get("mean_cross_track_error", 0.0),
        "entanglement_ratio": d.get("mean_entanglement_ratio", 0.0),
        "colregs_compliance": d.get("mean_colregs_compliance_ratio", 0.0),
        "cpa_starboard_pass": d.get("mean_cpa_starboard_pass_ratio", 0.0),
        "breakdown": bd,
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

# Print final ranking
print(f"\nTotal evaluated: {len(results)}, Zero-collision: {len(zero_coll)}")
print(f"{'Rank':>4} {'Phase':>5} {'Checkpoint':<30} {'Score':>6} {'Coll':>6} {'Prog':>6} {'COLRs':>6} {'CPA_sb':>6} {'Sep':>6} {'CTE':>6}")
print("─" * 110)
for i, r in enumerate(results[:20]):
    tag = "✅" if r["collision_rate"] == 0 else ("⚠️" if r["collision_rate"] < 0.2 else "❌")
    print(f"{i+1:>4} {r['phase']:>5} {tag} {r['checkpoint']:<27} {r['composite_score']:>6.1f} {r['collision_rate']:>5.0%} {r['progress']:>6.3f} {r['colregs_compliance']:>5.1%} {r['cpa_starboard_pass']:>5.1%} {r['worst_min_sep']:>6.3f} {r['cross_track_error']:>6.3f}")

if results:
    b = results[0]
    bd = b["breakdown"]
    print(f"\n🏆 Best: {b['checkpoint']} [{b['phase']}] (score={b['composite_score']:.1f} = safety:{bd['safety']:.0f} + prog:{bd['progress']:.0f} + smooth:{bd['smoothness']:.0f} + track:{bd['tracking']:.0f} + colregs:{bd['colregs']:.0f})")
if zero_coll:
    b = zero_coll[0]
    print(f"🛡️  Best safe: {b['checkpoint']} [{b['phase']}] (score={b['composite_score']:.1f}, zero collision)")
FINAL_RANK

echo ""
echo "Ranking saved: $RANKING"
echo "═══════════════════════════════════════════════════"
echo "  fresh61 Smart Evaluation Complete"
echo "═══════════════════════════════════════════════════"
