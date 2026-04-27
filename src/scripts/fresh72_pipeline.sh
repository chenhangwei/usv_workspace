#!/bin/bash
# fresh72 pipeline (Single-Lever Sprint):
#   1. 训练 30K (从 fresh70_stageB.pt 续训, 仅 2 个新 lever)
#   2. 用 candidate scoring 挑出 top-3 ckpt + 加上 final output + base 对照
#   3. 对每个候选跑完整 SITL gate, 选 (pass_count, score) 最高者
#   4. 复制为 fresh72_best.pt

set -eo pipefail

REPO="/home/chenhangwei/usv_workspace"
cd "$REPO"
source install/setup.bash

BASE_MODEL="/mnt/data/checkpoints/usv_rl/fresh70_stageB.pt"
SPRINT_OUT="/mnt/data/checkpoints/usv_rl/fresh72_sprint.pt"
BEST_MODEL="/mnt/data/checkpoints/usv_rl/fresh72_best.pt"

CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh72_checkpoints"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh72_eval"
RANKING_JSON="/mnt/data/checkpoints/usv_rl/fresh72_ranking.json"

LOG_FILE="/tmp/fresh72_pipeline.log"
exec > >(tee -a "$LOG_FILE") 2>&1
log() { echo "[$(date +%H:%M:%S)] $*"; }

quick_eval() {
  local model="$1"
  local output_json="$2"
  local episodes="$3"
  shift 3
  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$model" \
    --episodes "$episodes" \
    --steps-per-episode 275 \
    --device auto \
    --output-json "$output_json" \
    "$@"
}

log_candidate_table() {
  python3 src/scripts/fresh72_score_candidates.py \
    --eval-json-dir "$EVAL_DIR" \
    --top-k 8 \
    --format table \
    --existing-only || true
}

select_candidates() {
  local top_k="$1"
  python3 src/scripts/fresh72_score_candidates.py \
    --eval-json-dir "$EVAL_DIR" \
    --top-k "$top_k" \
    --format paths \
    --existing-only
}

score_gate_dir() {
  local result_dir="$1"
  local output_json="$2"
  python3 - "$result_dir" "$output_json" <<'PY'
import json
import math
import sys
from pathlib import Path

result_dir = Path(sys.argv[1])
output_json = Path(sys.argv[2])

scenarios = {
    "solo_navigation": result_dir / "solo_navigation_eval.json",
    "two_usv_head_on": result_dir / "two_usv_head_on_eval.json",
    "three_usv_crossing": result_dir / "three_usv_crossing_eval.json",
    "three_usv_overtaking": result_dir / "three_usv_overtaking_eval.json",
    "three_usv_random_encounter": result_dir / "three_usv_random_encounter_eval.json",
}

if not result_dir.is_dir():
    raise SystemExit(f"Gate result directory not found: {result_dir}")

def gap_min(value, threshold):
    return max(0.0, threshold - value)

def gap_max(value, threshold):
    return max(0.0, value - threshold)

def scenario_pass(name, payload):
    collision = float(payload.get("collision_rate", 1.0))
    progress = float(payload.get("mean_team_goal_progress_ratio", 0.0))
    if name == "solo_navigation":
        heading = float(payload.get("mean_heading_error", math.inf))
        cte = float(payload.get("mean_cross_track_error", math.inf))
        return collision <= 0.0 and progress >= 0.40 and heading <= 0.50 and cte <= 1.0
    separation = float(payload.get("worst_episode_min_separation", 0.0))
    omega = float(payload.get("mean_omega_flip_count", math.inf))
    return collision <= 0.0 and progress >= 0.40 and separation >= 1.5 and omega <= 5.0

def scenario_score(name, payload):
    collision = float(payload.get("collision_rate", 1.0))
    progress = float(payload.get("mean_team_goal_progress_ratio", 0.0))
    separation = float(payload.get("worst_episode_min_separation", 0.0))
    omega = float(payload.get("mean_omega_flip_count", 999.0))
    heading = float(payload.get("mean_heading_error", 0.0))
    cte = float(payload.get("mean_cross_track_error", 0.0))
    colregs = float(payload.get("mean_colregs_compliance_ratio", 0.0))
    entanglement = float(payload.get("mean_entanglement_ratio", 0.0))
    if name == "solo_navigation":
        score = 280.0 * progress
        score -= 1200.0 * collision
        score -= 220.0 * gap_max(heading, 0.50)
        score -= 160.0 * gap_max(cte, 1.0)
        if collision <= 0.0: score += 25.0
        if progress >= 0.40: score += 30.0
        if heading <= 0.50: score += 20.0
        if cte <= 1.0: score += 20.0
        return score
    score = 240.0 * progress
    score -= 1400.0 * collision
    score -= 180.0 * gap_min(separation, 1.5)
    score -= 10.0 * omega
    score += 40.0 * max(0.0, min(colregs, 1.0))
    score -= 70.0 * max(0.0, entanglement)
    if collision <= 0.0: score += 30.0
    if progress >= 0.40: score += 25.0
    if separation >= 1.5: score += 25.0
    if omega <= 5.0: score += 20.0
    if name == "two_usv_head_on":
        score -= 28.0 * max(0.0, separation - 4.0)
        score -= 140.0 * gap_max(heading, 0.70)
    return score

pass_count = 0
total_score = 0.0
details = {}
for name, path in scenarios.items():
    if not path.is_file():
        raise SystemExit(f"Missing gate JSON: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    passed = scenario_pass(name, payload)
    score = scenario_score(name, payload)
    details[name] = {
        "passed": passed,
        "score": score,
        "collision_rate": float(payload.get("collision_rate", 1.0)),
        "progress": float(payload.get("mean_team_goal_progress_ratio", 0.0)),
        "worst_min_sep": float(payload.get("worst_episode_min_separation", 0.0)),
        "omega_flip": float(payload.get("mean_omega_flip_count", 999.0)),
        "heading_error": float(payload.get("mean_heading_error", 0.0)),
        "cross_track_error": float(payload.get("mean_cross_track_error", 0.0)),
    }
    total_score += score
    if passed:
        pass_count += 1

summary = {
    "result_dir": str(result_dir),
    "pass_count": pass_count,
    "score": total_score,
    "details": details,
}
output_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
print(f"{pass_count} {total_score:.6f}")
PY
}

if [[ ! -f "$BASE_MODEL" ]]; then
  log "ERROR: base model not found: $BASE_MODEL"
  exit 1
fi

log "==== 清理 fresh72 旧产物 ===="
rm -f "$SPRINT_OUT" "$BEST_MODEL" "$RANKING_JSON"
rm -rf "$CKPT_DIR" "$EVAL_DIR"
mkdir -p "$EVAL_DIR"

log "==== 启动 fresh72 sprint training ===="
bash src/scripts/train_fresh72.sh > /tmp/fresh72_train.log 2>&1
log "训练完成 -> $SPRINT_OUT"

log "==== Final-output quick eval (5 scenarios) ===="
quick_eval "$SPRINT_OUT" "$EVAL_DIR/fresh72_sprint_final.json" 5 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter

log "==== Baseline (fresh70_stageB) reference eval ===="
quick_eval "$BASE_MODEL" "$EVAL_DIR/fresh70_stageB_baseline.json" 5 \
  --scenario solo_navigation \
  --scenario two_usv_head_on \
  --scenario three_usv_crossing \
  --scenario three_usv_overtaking \
  --scenario three_usv_random_encounter

log "==== Candidate ranking (top 8) ===="
log_candidate_table

mapfile -t RAW_CANDIDATES < <(select_candidates 3)
declare -A SEEN=()
CANDIDATES=()
for c in "${RAW_CANDIDATES[@]}" "$SPRINT_OUT"; do
  if [[ -z "$c" || ! -f "$c" || -n "${SEEN[$c]:-}" ]]; then continue; fi
  SEEN["$c"]=1
  CANDIDATES+=("$c")
done
if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  CANDIDATES=("$SPRINT_OUT")
fi

BEST_PASS=-1
BEST_SCORE=-1e18
BEST_SOURCE=""

for candidate in "${CANDIDATES[@]}"; do
  label="$(basename "$candidate" .pt)"
  gate_log="/tmp/${label}_fresh72_gate.log"
  gate_summary="/tmp/${label}_fresh72_gate_summary.json"
  gate_result_dir="/tmp/sitl_gate_${label}"
  log "==== Full SITL gate: $candidate ===="
  rm -rf "$gate_result_dir"
  bash src/scripts/fresh64_sitl_gate.sh "$candidate" > "$gate_log" 2>&1 || true
  if ! read -r PASS_COUNT SCORE < <(score_gate_dir "$gate_result_dir" "$gate_summary"); then
    log "WARNING: gate scoring failed for $candidate; skip"
    continue
  fi
  log "Gate result for $label: pass=${PASS_COUNT}/5  score=${SCORE}"
  grep -E "Gate Results|SCENARIO|progress:|heading_error:|cross_track_err:|collision_rate:|worst_min_sep:|omega_flip:" "$gate_log" | tail -50 || true
  if [[ -z "$BEST_SOURCE" ]] || [[ "$PASS_COUNT" -gt "$BEST_PASS" ]] || { [[ "$PASS_COUNT" -eq "$BEST_PASS" ]] && awk "BEGIN {exit !($SCORE > $BEST_SCORE)}"; }; then
    BEST_PASS="$PASS_COUNT"
    BEST_SCORE="$SCORE"
    BEST_SOURCE="$candidate"
  fi
done

if [[ -z "$BEST_SOURCE" ]]; then
  log "ERROR: no candidate survived gate evaluation"
  exit 1
fi

cp -f "$BEST_SOURCE" "$BEST_MODEL"

log "==== fresh72 pipeline complete ===="
log "Best candidate source: $BEST_SOURCE  (pass=${BEST_PASS}/5, score=${BEST_SCORE})"
log "Deployment model: $BEST_MODEL"
log "Train log: /tmp/fresh72_train.log"
log "Pipeline log: $LOG_FILE"
