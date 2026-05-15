#!/bin/bash
# fresh184: active trace deconf fitting with source-speed preservation.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh181_checkpoints/fresh181_pairwise_priority_delta_from_fresh172_step_0001260.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh184_checkpoints/fresh184_constrained_deconf_from_fresh181_step1260.pt}"
TRACE_DIR="${TRACE_DIR:-/tmp/fresh182_active_trace}"
BASE_DOMAIN="${BASE_DOMAIN:-192}"
STEPS="${STEPS:-420}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108.0}"
SEEDS="${SEEDS:-1460 1462}"
TARGET_PRIORITY="${TARGET_PRIORITY:-deconf}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.04}"
EPOCHS="${EPOCHS:-120}"
LEARNING_RATE="${LEARNING_RATE:-1.0e-5}"
DECONF_WEIGHT="${DECONF_WEIGHT:-1.6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-6.0}"
DECONF_MIN_LINEAR="${DECONF_MIN_LINEAR:-0.12}"
DECONF_MAX_LINEAR_DROP="${DECONF_MAX_LINEAR_DROP:-0.16}"
DECONF_LINEAR_BLEND="${DECONF_LINEAR_BLEND:-0.75}"
DECONF_OMEGA_BLEND="${DECONF_OMEGA_BLEND:-0.60}"

mkdir -p "$TRACE_DIR" "$(dirname "$OUTPUT")"

trace_args=()
index=0
for seed in $SEEDS; do
  json="$TRACE_DIR/fresh181_step1260_raw_trace_seed${seed}.json"
  trace_args+=(--trace-json "$json")
  if [[ "${SKIP_TRACE:-1}" != "1" || ! -f "$json" ]]; then
    echo "----- fresh184 trace seed=${seed} domain=$((BASE_DOMAIN + index)) -----"
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
  --anchor-batch-size 256 \
  --deconf-min-linear "$DECONF_MIN_LINEAR" \
  --deconf-max-linear-drop "$DECONF_MAX_LINEAR_DROP" \
  --deconf-linear-blend "$DECONF_LINEAR_BLEND" \
  --deconf-omega-blend "$DECONF_OMEGA_BLEND"