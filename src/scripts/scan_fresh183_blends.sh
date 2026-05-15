#!/bin/bash
# Scan fresh183 blend strengths between fresh181 base and fresh183 deconf-strong.

set -eo pipefail

cleanup_children() {
  pkill -P $$ 2>/dev/null || true
}
trap cleanup_children EXIT INT TERM

cd "$(dirname "$0")/.."
source ../install/setup.bash

OUT="${OUT:-/tmp/fresh183_blend_scan}"
BASE="${BASE:-/mnt/data/checkpoints/usv_rl/fresh181_checkpoints/fresh181_pairwise_priority_delta_from_fresh172_step_0001260.pt}"
STRONG="${STRONG:-/mnt/data/checkpoints/usv_rl/fresh183_checkpoints/fresh183_deconf_strong_from_fresh181_step1260.pt}"
CHECKPOINT_DIR="${CHECKPOINT_DIR:-/mnt/data/checkpoints/usv_rl/fresh183_checkpoints}"
BLENDS="${BLENDS:-a025:0.25 a050:0.50 a075:0.75}"
SEEDS="${SEEDS:-1460 1462}"
BASE_DOMAIN="${BASE_DOMAIN:-190}"
STEPS="${STEPS:-420}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108.0}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-900}"

mkdir -p "$OUT" "$CHECKPOINT_DIR"
printf 'label\tseed\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tent\tcolregs\tsteps\tmodel\n' > "$OUT/summary.tsv"

eval_index=0
for spec in $BLENDS; do
  label="${spec%%:*}"
  alpha="${spec##*:}"
  model="$CHECKPOINT_DIR/fresh183_blend_${label}.pt"
  base_weight="$(/bin/python3 -c "alpha=float('$alpha'); print(f'{1.0 - alpha:.6f}')")"

  printf 'blend-start label=%s alpha=%s base_weight=%s model=%s\n' "$label" "$alpha" "$base_weight" "$model"

  /bin/python3 scripts/fresh79_blend_checkpoints.py \
    --checkpoint "$BASE:$base_weight" \
    --checkpoint "$STRONG:$alpha" \
    --template "$BASE" \
    --output "$model" \
    --label "fresh183_${label}" \
    > "$OUT/${label}_blend.log" 2>&1
  printf 'blend-done label=%s\n' "$label"

  for seed in $SEEDS; do
    domain=$((BASE_DOMAIN + eval_index))
    eval_index=$((eval_index + 1))
    json="$OUT/${label}_seed${seed}.json"
    log="$OUT/${label}_seed${seed}.log"
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

    printf 'eval-start label=%s seed=%s domain=%s\n' "$label" "$seed" "$domain"

    if ! ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
      --policy mappo \
      --model "$model" \
      --episodes 1 \
      --steps-per-episode "$STEPS" \
      --device cpu \
      --episode-timeout "$EPISODE_TIMEOUT" \
      --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
      --seed "$seed" \
      --output-json "$json" \
      --scenario three_usv_random_encounter \
      > "$log" 2>&1; then
      printf '%s\t%s\tERROR\tERROR\tERROR\t0\t0\t0\t0\t0\t0\t0\t%s\n' "$label" "$seed" "$model" >> "$OUT/summary.tsv"
      tail -40 "$log"
      continue
    fi
    printf 'eval-done label=%s seed=%s\n' "$label" "$seed"

    /bin/python3 - "$json" "$label" "$seed" "$model" "$OUT/summary.tsv" <<'PY'
import json
import sys

json_path, label, seed, model, summary_path = sys.argv[1:6]
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
    float(episode.get('steps', 0.0)),
]
row = '\t'.join([label, seed] + [f'{value:.6f}' for value in values] + [model])
with open(summary_path, 'a', encoding='utf-8') as handle:
    handle.write(row + '\n')
print(row, flush=True)
PY
  done
done

column -t -s $'\t' "$OUT/summary.tsv"