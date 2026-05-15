#!/bin/bash
# Smoke evaluation for fresh309 rear-only overtaking continuation.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh309_single_overtake_finish_from_fresh308.pt}"
OUT="${OUT:-/tmp/fresh309_eval}"
LABEL="${LABEL:-fresh309}"
STEPS="${STEPS:-800}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108.0}"
BASE_DOMAIN="${BASE_DOMAIN:-210}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-1800}"

mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
printf 'scenario\tseed\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tomega\tsteps\tjson\n' > "$SUMMARY"

eval_one() {
  local scenario="$1"
  local seed="$2"
  local domain="$3"
  local label="${scenario}_seed${seed}"
  local json="$OUT/${label}.json"
  local log="$OUT/${label}.log"

  if [[ ! -f "$MODEL" ]]; then
    echo "missing model: $MODEL" >&2
    return 1
  fi

  echo "----- ${LABEL} eval scenario=${scenario} seed=${seed} domain=${domain} -----"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy mappo \
    --model "$MODEL" \
    --episodes 1 \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --seed "$seed" \
    --output-json "$json" \
    --scenario "$scenario" \
    > "$log" 2>&1

  /bin/python3 - "$json" "$scenario" "$seed" "$SUMMARY" <<'PY'
import json
import sys

json_path, scenario, seed, summary_path = sys.argv[1:5]
with open(json_path, 'r', encoding='utf-8') as handle:
    data = json.load(handle)
episode = data['episode_metrics'][0]
row = [
    scenario,
    seed,
    f"{float(data.get('collision_rate', 0.0)):.6f}",
    f"{float(data.get('success_rate', 0.0)):.6f}",
    f"{float(data.get('timeout_rate', 0.0)):.6f}",
    f"{float(episode.get('team_goal_progress_ratio', 0.0)):.6f}",
    f"{float(episode.get('goal_completion_ratio', 0.0)):.6f}",
    f"{float(episode.get('episode_min_separation', 0.0)):.6f}",
    f"{float(episode.get('mean_cross_track_error', 0.0)):.6f}",
    f"{float(episode.get('omega_flip_count', 0.0)):.6f}",
    f"{float(episode.get('steps', 0.0)):.6f}",
    json_path,
]
with open(summary_path, 'a', encoding='utf-8') as handle:
    handle.write('\t'.join(row) + '\n')
PY
}

index=0
for seed in 3071 3072 3073 3074; do
  eval_one single_usv_overtaking "$seed" "$((BASE_DOMAIN + index))"
  index=$((index + 1))
done

for seed in 3001 3002; do
  eval_one three_usv_clear_route "$seed" "$((BASE_DOMAIN + index))"
  index=$((index + 1))
done

for scenario in two_usv_head_on two_usv_crossing; do
  eval_one "$scenario" 3101 "$((BASE_DOMAIN + index))"
  index=$((index + 1))
done

for seed in 1458 1460 1461; do
  eval_one three_usv_random_encounter "$seed" "$((BASE_DOMAIN + index))"
  index=$((index + 1))
done

echo "summary: $SUMMARY"
cat "$SUMMARY"