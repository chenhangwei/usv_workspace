#!/bin/bash
# Fixed-seed random-only screening for fresh165 checkpoints.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash

OUT="${OUT:-/tmp/fresh165_random_eval}"
SUMMARY="$OUT/summary.tsv"
SEED="${SEED:-1460}"
STEPS="${STEPS:-420}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108.0}"
BASE_DOMAIN="${BASE_DOMAIN:-228}"

mkdir -p "$OUT"
printf 'label\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tentanglement\tcolregs\tomega\tsteps\tmodel\n' > "$SUMMARY"

eval_one() {
  local label="$1"
  local model="$2"
  local domain="$3"
  local json="$OUT/${label}_random_seed${SEED}.json"
  local log="$OUT/${label}_random_seed${SEED}.log"

  echo "----- fresh165 random-only label=${label} domain=${domain} seed=${SEED} -----"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  ROS_DOMAIN_ID="$domain" timeout 1800 /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model "$model" \
    --episodes 1 \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --seed "$SEED" \
    --output-json "$json" \
    --scenario three_usv_random_encounter \
    > "$log" 2>&1

  /bin/python3 - "$json" "$label" "$model" "$SUMMARY" <<'PY'
import json
import sys

json_path, label, model, summary_path = sys.argv[1:5]
with open(json_path, 'r', encoding='utf-8') as handle:
    data = json.load(handle)
episode = data['episode_metrics'][0]
values = [
    float(data.get('collision_rate', 0.0)),
    float(data.get('success_rate', 0.0)),
    float(data.get('timeout_rate', 0.0)),
    float(episode.get('team_goal_progress_ratio', 0.0)),
    float(episode.get('goal_completion_ratio', 0.0)),
    float(episode.get('episode_min_separation', 0.0)),
    float(episode.get('mean_cross_track_error', 0.0)),
    float(episode.get('entanglement_ratio', 0.0)),
    float(episode.get('colregs_violation_ratio', 0.0)),
    float(episode.get('omega_flip_count', 0.0)),
    float(episode.get('steps', 0.0)),
]
row = '\t'.join([label] + [f'{value:.6f}' for value in values] + [model])
with open(summary_path, 'a', encoding='utf-8') as handle:
    handle.write(row + '\n')
print(row)
print(
    f"fresh165-random {label}: coll={values[0]:.3f} succ={values[1]:.3f} "
    f"timeout={values[2]:.3f} progress={values[3]:.3f} goal={values[4]:.3f} "
    f"sep={values[5]:.3f} cte={values[6]:.3f} ent={values[7]:.3f} "
    f"colregs={values[8]:.3f} omega={values[9]:.1f} steps={values[10]:.1f}"
)
PY
}

eval_one 0576 /mnt/data/checkpoints/usv_rl/fresh165_checkpoints/fresh165_priority_release_finish_from_fresh163_step1152_step_0000576.pt "$BASE_DOMAIN"
eval_one 1152 /mnt/data/checkpoints/usv_rl/fresh165_checkpoints/fresh165_priority_release_finish_from_fresh163_step1152_step_0001152.pt "$((BASE_DOMAIN + 1))"
eval_one final /mnt/data/checkpoints/usv_rl/fresh165_priority_release_finish_from_fresh163_step1152.pt "$((BASE_DOMAIN + 2))"

echo '========== fresh165 random summary =========='
column -t -s $'\t' "$SUMMARY"