#!/bin/bash
# fresh182: active-slice trace fitting from fresh181 without changing reward gates.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh181_checkpoints/fresh181_pairwise_priority_delta_from_fresh172_step_0001260.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh182_checkpoints/fresh182_deconf_trace_anchor_from_fresh181_step1260.pt}"
TRACE_DIR="${TRACE_DIR:-/tmp/fresh182_active_trace}"
BASE_DOMAIN="${BASE_DOMAIN:-188}"
STEPS="${STEPS:-420}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108.0}"
SEEDS="${SEEDS:-1460 1462}"
TARGET_PRIORITY="${TARGET_PRIORITY:-deconf}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.04}"
EPOCHS="${EPOCHS:-140}"
LEARNING_RATE="${LEARNING_RATE:-1.6e-5}"
DECONF_WEIGHT="${DECONF_WEIGHT:-2.0}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-3.0}"

mkdir -p "$TRACE_DIR" "$(dirname "$OUTPUT")"

trace_args=()
index=0
for seed in $SEEDS; do
  json="$TRACE_DIR/fresh181_step1260_raw_trace_seed${seed}.json"
  trace_args+=(--trace-json "$json")
  if [[ "${SKIP_TRACE:-0}" != "1" ]]; then
    echo "----- fresh182 trace seed=${seed} domain=$((BASE_DOMAIN + index)) -----"
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    ROS_DOMAIN_ID="$((BASE_DOMAIN + index))" timeout 1800 /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
      --policy mappo \
      --model "$SOURCE_MODEL" \
      --episodes 1 \
      --steps-per-episode "$STEPS" \
      --device cpu \
      --episode-timeout "$EPISODE_TIMEOUT" \
      --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
      --seed "$seed" \
      --output-json "$json" \
      --scenario three_usv_random_encounter \
      --trace-stride 10 \
      --trace-raw-observation
  fi
  index=$((index + 1))
done

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 64 \
  --learning-rate "$LEARNING_RATE" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --target-priority "$TARGET_PRIORITY" \
  --deconf-weight "$DECONF_WEIGHT" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size 256
