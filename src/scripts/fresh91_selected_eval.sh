#!/bin/bash
# fresh91 selected evaluation: quick full-5 + targeted hard-scenario triage.
# Defaults to fresh86/fresh89/fresh90 baselines plus the latest fresh91 checkpoints.

set -o pipefail

cd "$(dirname "$0")/../.."
source install/setup.bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export MALLOC_ARENA_MAX=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh91_selected_eval}"
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh91_checkpoints}"
FULL5_EPISODES="${FULL5_EPISODES:-5}"
TARGETED_EPISODES="${TARGETED_EPISODES:-9}"
STEPS="${STEPS:-300}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-70}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-20}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-360}"
BASE_DOMAIN="${BASE_DOMAIN:-181}"
CLEAN_IPC="${CLEAN_IPC:-1}"
FRESH91_LATEST_COUNT="${FRESH91_LATEST_COUNT:-4}"

mkdir -p "$EVAL_DIR"
LOG="${LOG:-/tmp/fresh91_selected_eval.log}"
: > "$LOG"

declare -A SEEN=()
CANDIDATES=()

add_candidate() {
  local model="$1"
  if [[ -z "$model" || ! -f "$model" ]]; then
    return 0
  fi
  if [[ -n "${SEEN[$model]:-}" ]]; then
    return 0
  fi
  SEEN["$model"]=1
  CANDIDATES+=("$model")
}

if [[ $# -gt 0 ]]; then
  for model in "$@"; do
    add_candidate "$model"
  done
else
  add_candidate /mnt/data/checkpoints/usv_rl/fresh86_checkpoints/fresh86_overtaking_scenario_trunk_step_0001152.pt
  add_candidate /mnt/data/checkpoints/usv_rl/fresh89_checkpoints/fresh89_full_scenario_trunk_unfrozen_rewardfix_step_0013124.pt
  add_candidate /mnt/data/checkpoints/usv_rl/fresh90_checkpoints/fresh90_full_scenario_trunk_curriculum_step_0008198.pt
  add_candidate /mnt/data/checkpoints/usv_rl/fresh91_full_scenario_trunk_branch_repair.pt
  mapfile -t latest_fresh91 < <(ls -1t "$CKPT_DIR"/fresh91_full_scenario_trunk_branch_repair_step_*.pt 2>/dev/null | head -n "$FRESH91_LATEST_COUNT")
  for model in "${latest_fresh91[@]}"; do
    add_candidate "$model"
  done
fi

domain_for_index() {
  local idx="$1"
  local domain=$((BASE_DOMAIN + idx))
  if (( domain > 232 )); then
    domain=$((181 + (idx % 40)))
  fi
  echo "$domain"
}

run_eval() {
  local model="$1"
  local label="$2"
  local output_json="$3"
  local episodes="$4"
  local domain="$5"
  shift 5
  if [[ -f "$output_json" && "$output_json" -nt "$model" ]]; then
    echo "[$(date +%T)] skip cached: $label -> $(basename "$output_json")" | tee -a "$LOG"
    return 0
  fi
  echo "[$(date +%T)] eval $label domain=$domain episodes=$episodes scenarios=$*" | tee -a "$LOG"
  if [[ "$CLEAN_IPC" == "1" ]]; then
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  fi
  ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model "$model" \
    --episodes "$episodes" \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --output-json "$output_json" \
    "$@" \
    > "/tmp/fresh91_selected_${label}_$(basename "$output_json" .json).log" 2>&1
  local rc=$?
  if [[ $rc -ne 0 || ! -f "$output_json" ]]; then
    echo "[$(date +%T)]   FAIL $label rc=$rc log=/tmp/fresh91_selected_${label}_$(basename "$output_json" .json).log" | tee -a "$LOG"
    rm -f "$output_json"
    return 1
  fi
  python3 - "$output_json" <<'PY' | tee -a "$LOG"
import json
import sys
from pathlib import Path
payload = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
print(
    f"[{Path(sys.argv[1]).name}] "
    f"coll={payload.get('collision_rate', 1.0):.3f} "
    f"succ={payload.get('success_rate', 0.0):.3f} "
    f"timeout={payload.get('timeout_rate', 0.0):.3f} "
    f"progress={payload.get('mean_team_goal_progress_ratio', 0.0):.3f} "
    f"sep={payload.get('worst_episode_min_separation', 0.0):.3f} "
    f"ent={payload.get('mean_entanglement_ratio', 1.0):.3f} "
    f"omega={payload.get('mean_omega_flip_count', 999.0):.1f}"
)
PY
}

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No existing candidates found." | tee -a "$LOG"
  exit 1
fi

idx=0
for model in "${CANDIDATES[@]}"; do
  label="$(basename "$model" .pt)"
  domain="$(domain_for_index "$idx")"
  run_eval "$model" "$label" "$EVAL_DIR/${label}_full5_eval.json" "$FULL5_EPISODES" "$domain" \
    --scenario solo_navigation \
    --scenario two_usv_head_on \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --scenario three_usv_random_encounter || true
  idx=$((idx + 1))

  domain="$(domain_for_index "$idx")"
  run_eval "$model" "$label" "$EVAL_DIR/${label}_targeted3_eval.json" "$TARGETED_EPISODES" "$domain" \
    --scenario three_usv_crossing \
    --scenario three_usv_overtaking \
    --scenario three_usv_random_encounter || true
  idx=$((idx + 1))
done

python3 - "$EVAL_DIR" <<'PY' | tee -a "$LOG"
import glob
import json
import os
import sys
from pathlib import Path

eval_dir = Path(sys.argv[1])
records = {}

def load(path: Path):
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return None
    model = str(payload.get('model', path.stem))
    rec = records.setdefault(model, {'model': model, 'full5': None, 'targeted3': None})
    if path.name.endswith('_full5_eval.json'):
        rec['full5'] = payload
    elif path.name.endswith('_targeted3_eval.json'):
        rec['targeted3'] = payload

for raw_path in glob.glob(str(eval_dir / '*_eval.json')):
    load(Path(raw_path))

def metrics(payload):
    if not payload:
        return {
            'collision_rate': 1.0,
            'timeout_rate': 1.0,
            'success_rate': 0.0,
            'progress': 0.0,
            'worst_sep': 0.0,
            'entanglement': 1.0,
            'omega_flip': 999.0,
        }
    return {
        'collision_rate': float(payload.get('collision_rate', 1.0)),
        'timeout_rate': float(payload.get('timeout_rate', 1.0)),
        'success_rate': float(payload.get('success_rate', 0.0)),
        'progress': float(payload.get('mean_team_goal_progress_ratio', 0.0)),
        'worst_sep': float(payload.get('worst_episode_min_separation', 0.0)),
        'entanglement': float(payload.get('mean_entanglement_ratio', 1.0)),
        'omega_flip': float(payload.get('mean_omega_flip_count', 999.0)),
    }

rows = []
for rec in records.values():
    full5 = metrics(rec.get('full5'))
    targeted = metrics(rec.get('targeted3'))
    rows.append({
        'model': rec['model'],
        'full5': full5,
        'targeted3': targeted,
        'score_tuple': (
            targeted['collision_rate'],
            full5['collision_rate'],
            targeted['timeout_rate'],
            full5['timeout_rate'],
            targeted['entanglement'],
            targeted['omega_flip'],
            -targeted['success_rate'],
            -targeted['progress'],
            -full5['progress'],
            -targeted['worst_sep'],
        ),
    })
rows.sort(key=lambda item: item['score_tuple'])
summary = {
    'eval_dir': str(eval_dir),
    'best_model': rows[0]['model'] if rows else None,
    'candidates': rows,
}
(eval_dir / 'selected_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(f"Summary saved to {eval_dir / 'selected_summary.json'}")
for row in rows:
    t = row['targeted3']
    f = row['full5']
    print(
        f"targeted coll={t['collision_rate']:.3f} timeout={t['timeout_rate']:.3f} "
        f"succ={t['success_rate']:.3f} prog={t['progress']:.3f} | "
        f"full coll={f['collision_rate']:.3f} timeout={f['timeout_rate']:.3f} "
        f"prog={f['progress']:.3f} | {row['model']}"
    )
PY

echo "[$(date +%T)] fresh91 selected eval complete. Log: $LOG"