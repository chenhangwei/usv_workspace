#!/bin/bash
# Focused regression gate: the 6 deterministic-failure cases from fresh522
# plus 2 known-good sanity cases. Fast (~8 cases) mid-training checkpoint gate.
# Usage: MODEL=/path/to/ckpt.pt bash scripts/eval_fresh523_focus.sh

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

MODEL="${MODEL:?need MODEL=}"
OUT="${OUT:-/tmp/fresh523_focus}"
LABEL="${LABEL:-fresh523focus}"
STEPS="${STEPS:-400}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-110.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-60.0}"
BASE_DOMAIN="${BASE_DOMAIN:-150}"
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
# fresh522 deterministic failures
for seed in 5211 5213; do
  eval_one pentagram_convergence "$seed" "$((BASE_DOMAIN + index))"; index=$((index + 1))
done
for seed in 5221 5222; do
  eval_one five_usv_dense_crossing "$seed" "$((BASE_DOMAIN + index))"; index=$((index + 1))
done
eval_one three_usv_crossing 5231 "$((BASE_DOMAIN + index))"; index=$((index + 1))
eval_one three_usv_overtaking 5232 "$((BASE_DOMAIN + index))"; index=$((index + 1))
# sanity (should stay passing)
eval_one pentagram_convergence 5212 "$((BASE_DOMAIN + index))"; index=$((index + 1))
eval_one three_usv_random_encounter 5232 "$((BASE_DOMAIN + index))"; index=$((index + 1))

echo
echo "===== ${LABEL} summary ====="
column -t "$SUMMARY"

# quick verdict
/bin/python3 - "$SUMMARY" <<'PY'
import sys
rows = [l.split('\t') for l in open(sys.argv[1]).read().splitlines()[1:]]
fails = [(r[0], r[1]) for r in rows if float(r[2]) > 0.2 or float(r[3]) < 0.5 or float(r[7]) < 0.5]
print(f"\nVERDICT: {len(rows)-len(fails)}/{len(rows)} pass")
for s, seed in fails:
    print(f"  FAIL {s} seed={seed}")
PY
