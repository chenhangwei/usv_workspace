#!/bin/bash
# fresh72 post-train rescue:
#   背景: train_mappo_policy --auto-evaluate-checkpoints 在本机会 SIGSEGV (ROS2 multi-proc DDS 冲突),
#   导致 EVAL_DIR 为空, fresh72_pipeline.sh 无法选出 top-3 中段 ckpt。
#   本脚本在训练完成 (主进程 PID 退出) 后接管:
#     1. 对所有 fresh72_sprint_step_*.pt + fresh72_sprint.pt + fresh70_stageB.pt 各跑 1 次 quick_eval
#     2. 用 fresh72_score_candidates.py 排序, 取 top-3
#     3. 对 top-3 + sprint final + baseline 跑 full SITL gate
#     4. 选 (pass_count, score) 最高者复制为 fresh72_best.pt
#
# 用法:
#   bash scripts/fresh72_post_train.sh                  # 等当前训练完
#   FRESH72_TRAIN_PID=67352 bash scripts/fresh72_post_train.sh   # 显式 PID
#   FRESH72_SKIP_WAIT=1 bash scripts/fresh72_post_train.sh       # 不等, 立即开始 (训练已结束时)

set -eo pipefail
REPO="/home/chenhangwei/usv_workspace"
cd "$REPO"
source install/setup.bash

BASE_MODEL="/mnt/data/checkpoints/usv_rl/fresh70_stageB.pt"
SPRINT_OUT="/mnt/data/checkpoints/usv_rl/fresh72_sprint.pt"
BEST_MODEL="/mnt/data/checkpoints/usv_rl/fresh72_best.pt"
CKPT_DIR="/mnt/data/checkpoints/usv_rl/fresh72_checkpoints"
EVAL_DIR="/mnt/data/checkpoints/usv_rl/fresh72_eval"

LOG_FILE="/tmp/fresh72_post_train.log"
exec > >(tee -a "$LOG_FILE") 2>&1
log() { echo "[$(date +%H:%M:%S)] $*"; }

EVAL_SCENARIOS=(
  --scenario solo_navigation
  --scenario two_usv_head_on
  --scenario three_usv_crossing
  --scenario three_usv_overtaking
  --scenario three_usv_random_encounter
)

quick_eval_one() {
  local model="$1"
  local label
  label="$(basename "$model" .pt)"
  local out="$EVAL_DIR/${label}_quickeval.json"
  if [[ -f "$out" && "$out" -nt "$model" ]]; then
    log "skip (cached): $label"
    return 0
  fi
  log "quick_eval: $label"
  python3 -m usv_rl.evaluate_mappo_policy \
    --model "$model" \
    --episodes 5 \
    --steps-per-episode 275 \
    --device auto \
    --output-json "$out" \
    "${EVAL_SCENARIOS[@]}" >/dev/null 2>&1 \
    || { log "  WARNING: quick_eval failed for $label (exit=$?)"; rm -f "$out"; return 1; }
  return 0
}

score_gate_dir() {
  local result_dir="$1"
  local output_json="$2"
  python3 - "$result_dir" "$output_json" <<'PY'
import json, math, sys
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
    raise SystemExit(f"missing dir: {result_dir}")
def gmin(v,t): return max(0.0, t-v)
def gmax(v,t): return max(0.0, v-t)
def passed(name, p):
    c=float(p.get("collision_rate",1.0)); pr=float(p.get("mean_team_goal_progress_ratio",0.0))
    if name=="solo_navigation":
        h=float(p.get("mean_heading_error",math.inf)); ce=float(p.get("mean_cross_track_error",math.inf))
        return c<=0.0 and pr>=0.40 and h<=0.50 and ce<=1.0
    s=float(p.get("worst_episode_min_separation",0.0)); o=float(p.get("mean_omega_flip_count",math.inf))
    return c<=0.0 and pr>=0.40 and s>=1.5 and o<=5.0
def score(name,p):
    c=float(p.get("collision_rate",1.0)); pr=float(p.get("mean_team_goal_progress_ratio",0.0))
    s=float(p.get("worst_episode_min_separation",0.0)); o=float(p.get("mean_omega_flip_count",999.0))
    h=float(p.get("mean_heading_error",0.0)); ce=float(p.get("mean_cross_track_error",0.0))
    cr=float(p.get("mean_colregs_compliance_ratio",0.0)); en=float(p.get("mean_entanglement_ratio",0.0))
    if name=="solo_navigation":
        sc=280.0*pr-1200.0*c-220.0*gmax(h,0.50)-160.0*gmax(ce,1.0)
        if c<=0.0: sc+=25
        if pr>=0.40: sc+=30
        if h<=0.50: sc+=20
        if ce<=1.0: sc+=20
        return sc
    sc=240.0*pr-1400.0*c-180.0*gmin(s,1.5)-10.0*o+40.0*max(0.0,min(cr,1.0))-70.0*max(0.0,en)
    if c<=0.0: sc+=30
    if pr>=0.40: sc+=25
    if s>=1.5: sc+=25
    if o<=5.0: sc+=20
    if name=="two_usv_head_on":
        sc-=28.0*max(0.0,s-4.0); sc-=140.0*gmax(h,0.70)
    return sc
pc=0; tot=0.0; det={}
for n,pth in scenarios.items():
    if not pth.is_file(): raise SystemExit(f"missing {pth}")
    p=json.loads(pth.read_text(encoding="utf-8"))
    ok=passed(n,p); sc=score(n,p)
    det[n]={"passed":ok,"score":sc,
            "collision_rate":float(p.get("collision_rate",1.0)),
            "progress":float(p.get("mean_team_goal_progress_ratio",0.0)),
            "worst_min_sep":float(p.get("worst_episode_min_separation",0.0)),
            "omega_flip":float(p.get("mean_omega_flip_count",999.0)),
            "heading_error":float(p.get("mean_heading_error",0.0)),
            "cross_track_error":float(p.get("mean_cross_track_error",0.0))}
    tot+=sc
    if ok: pc+=1
output_json.write_text(json.dumps({"result_dir":str(result_dir),"pass_count":pc,"score":tot,"details":det},
                                  ensure_ascii=False,indent=2),encoding="utf-8")
print(f"{pc} {tot:.6f}")
PY
}

# 0. 等训练结束
TRAIN_PID="${FRESH72_TRAIN_PID:-}"
if [[ -z "$TRAIN_PID" ]]; then
  TRAIN_PID="$(pgrep -f "train_mappo_policy.*fresh72_sprint" | head -1 || true)"
fi
if [[ "${FRESH72_SKIP_WAIT:-0}" != "1" && -n "$TRAIN_PID" ]] && kill -0 "$TRAIN_PID" 2>/dev/null; then
  log "Waiting for training PID $TRAIN_PID to finish..."
  while kill -0 "$TRAIN_PID" 2>/dev/null; do sleep 30; done
  log "Training PID $TRAIN_PID exited."
else
  log "No live training PID found (or skipped wait). Proceeding."
fi

# 也杀掉旧的 fresh72_pipeline (它后续逻辑会重复跑 evaluate, 浪费时间)
PIPE_PID="$(pgrep -f "src/scripts/fresh72_pipeline.sh" | head -1 || true)"
if [[ -n "$PIPE_PID" ]] && kill -0 "$PIPE_PID" 2>/dev/null; then
  log "Killing old fresh72_pipeline PID $PIPE_PID to avoid double work."
  kill "$PIPE_PID" 2>/dev/null || true
  sleep 2
  pkill -P "$PIPE_PID" 2>/dev/null || true
fi

mkdir -p "$EVAL_DIR"

# 1. 对 baseline + sprint final + 所有中段 ckpt 跑 quick_eval
log "==== Phase 1: quick_eval all candidates ===="
quick_eval_one "$BASE_MODEL" || true
[[ -f "$SPRINT_OUT" ]] && quick_eval_one "$SPRINT_OUT" || true
for ckpt in "$CKPT_DIR"/fresh72_sprint_step_*.pt; do
  [[ -f "$ckpt" ]] || continue
  quick_eval_one "$ckpt" || true
done

log "==== Phase 1 done. Eval files: ===="
ls -1 "$EVAL_DIR"/*.json 2>/dev/null | wc -l
ls -lt "$EVAL_DIR"/*.json 2>/dev/null | head -20

# 2. Candidate ranking
log "==== Phase 2: rank candidates (top 10) ===="
python3 src/scripts/fresh72_score_candidates.py \
  --eval-json-dir "$EVAL_DIR" \
  --top-k 10 \
  --format table \
  --existing-only || true

mapfile -t TOP_CANDIDATES < <(python3 src/scripts/fresh72_score_candidates.py \
  --eval-json-dir "$EVAL_DIR" \
  --top-k 3 \
  --format paths \
  --existing-only)

declare -A SEEN=()
GATE_LIST=()
for c in "${TOP_CANDIDATES[@]}" "$SPRINT_OUT" "$BASE_MODEL"; do
  [[ -z "$c" || ! -f "$c" ]] && continue
  [[ -n "${SEEN[$c]:-}" ]] && continue
  SEEN["$c"]=1
  GATE_LIST+=("$c")
done

if [[ ${#GATE_LIST[@]} -eq 0 ]]; then
  log "ERROR: nothing to gate"
  exit 1
fi
log "Gate list (${#GATE_LIST[@]} candidates):"
for c in "${GATE_LIST[@]}"; do log "  $c"; done

# 3. Full SITL gate per candidate
log "==== Phase 3: full SITL gate ===="
BEST_PASS=-1
BEST_SCORE=-1e18
BEST_SOURCE=""
GATE_RESULTS=()

for candidate in "${GATE_LIST[@]}"; do
  label="$(basename "$candidate" .pt)"
  gate_log="/tmp/${label}_fresh72gate.log"
  gate_summary="/tmp/${label}_fresh72gate_summary.json"
  gate_result_dir="/tmp/sitl_gate_${label}"
  log "---- Gate: $label ----"
  rm -rf "$gate_result_dir"
  bash src/scripts/fresh64_sitl_gate.sh "$candidate" > "$gate_log" 2>&1 || true
  if ! read -r PASS_COUNT SCORE < <(score_gate_dir "$gate_result_dir" "$gate_summary"); then
    log "  WARNING: gate scoring failed; skip"
    continue
  fi
  log "  $label: pass=${PASS_COUNT}/5 score=${SCORE}"
  GATE_RESULTS+=("${PASS_COUNT}/5  score=${SCORE}  ${label}")
  grep -E "Testing|collision_rate|progress:|heading_error:|cross_track_err:|worst_min_sep:|omega_flip:|SCENARIO|Gate Results" "$gate_log" | tail -50 || true
  if [[ -z "$BEST_SOURCE" ]] || [[ "$PASS_COUNT" -gt "$BEST_PASS" ]] \
       || { [[ "$PASS_COUNT" -eq "$BEST_PASS" ]] && awk "BEGIN {exit !($SCORE > $BEST_SCORE)}"; }; then
    BEST_PASS="$PASS_COUNT"
    BEST_SCORE="$SCORE"
    BEST_SOURCE="$candidate"
  fi
done

log "==== Gate summary ===="
for line in "${GATE_RESULTS[@]}"; do log "  $line"; done

if [[ -z "$BEST_SOURCE" ]]; then
  log "ERROR: no candidate survived"
  exit 1
fi

cp -f "$BEST_SOURCE" "$BEST_MODEL"
log "==== Best ===="
log "Source : $BEST_SOURCE"
log "Pass   : ${BEST_PASS}/5"
log "Score  : ${BEST_SCORE}"
log "Output : $BEST_MODEL"
log "Log    : $LOG_FILE"
