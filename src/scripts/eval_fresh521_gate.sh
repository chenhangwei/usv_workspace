#!/bin/bash
# Gate evaluation for fresh521 5-agent pentagram expansion.
# Gates: per-scenario collision<=0.2, success>=0.5, min_sep>=0.5,
#        no regression on three_usv_* vs fresh520.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh521_pentagram5.pt}"
OUT="${OUT:-/tmp/fresh521_eval}"
LABEL="${LABEL:-fresh521}"
STEPS="${STEPS:-400}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-110.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-60.0}"
BASE_DOMAIN="${BASE_DOMAIN:-220}"
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
# New 5-USV scenarios (primary gate)
for seed in 5211 5212 5213; do
  eval_one pentagram_convergence "$seed" "$((BASE_DOMAIN + index))"
  index=$((index + 1))
done

for seed in 5221 5222; do
  eval_one five_usv_dense_crossing "$seed" "$((BASE_DOMAIN + index))"
  index=$((index + 1))
done

# 3-USV regression check vs fresh520
for scenario in three_usv_crossing three_usv_overtaking three_usv_random_encounter; do
  for seed in 5231 5232; do
    eval_one "$scenario" "$seed" "$((BASE_DOMAIN + index))"
    index=$((index + 1))
  done
done

echo
echo "===== ${LABEL} summary ====="
column -t "$SUMMARY"
