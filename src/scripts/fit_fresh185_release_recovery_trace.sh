#!/bin/bash
# Fit fresh185 from fresh181 raw traces with constrained release/recovery targets.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh181_checkpoints/fresh181_pairwise_priority_delta_from_fresh172_step_0001260.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh185_checkpoints/fresh185_release_recovery_from_fresh181_step1260.pt}"
TRACE_DIR="${TRACE_DIR:-/tmp/fresh182_active_trace}"
SEEDS="${SEEDS:-1460 1462}"
EPOCHS="${EPOCHS:-140}"
LEARNING_RATE="${LEARNING_RATE:-8.0e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-7.0}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.04}"
DECONF_WEIGHT="${DECONF_WEIGHT:-1.25}"
OFFROUTE_WEIGHT="${OFFROUTE_WEIGHT:-1.10}"
CTE_WEIGHT="${CTE_WEIGHT:-1.15}"
FINISH_WEIGHT="${FINISH_WEIGHT:-0.75}"
TARGET_MIN_LINEAR="${TARGET_MIN_LINEAR:-0.17}"
TARGET_MAX_LINEAR_DROP="${TARGET_MAX_LINEAR_DROP:-0.10}"
TARGET_LINEAR_BLEND="${TARGET_LINEAR_BLEND:-0.65}"
TARGET_OMEGA_BLEND="${TARGET_OMEGA_BLEND:-0.45}"
DECONF_MIN_LINEAR="${DECONF_MIN_LINEAR:-0.17}"
DECONF_MAX_LINEAR_DROP="${DECONF_MAX_LINEAR_DROP:-0.10}"
DECONF_LINEAR_BLEND="${DECONF_LINEAR_BLEND:-1.0}"
DECONF_OMEGA_BLEND="${DECONF_OMEGA_BLEND:-1.0}"

trace_args=()
anchor_args=()
for seed in $SEEDS; do
  trace_json="$TRACE_DIR/fresh181_step1260_raw_trace_seed${seed}.json"
  if [[ ! -s "$trace_json" ]]; then
    echo "missing trace json: $trace_json" >&2
    exit 1
  fi
  trace_args+=(--trace-json "$trace_json")
  anchor_args+=(--anchor-trace-json "$trace_json")
done

mkdir -p "$(dirname "$OUTPUT")"
/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  "${trace_args[@]}" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --target-priority deconf,offroute,cte,finish \
  --min-error-norm "$MIN_ERROR_NORM" \
  --epochs "$EPOCHS" \
  --learning-rate "$LEARNING_RATE" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --deconf-weight "$DECONF_WEIGHT" \
  --offroute-weight "$OFFROUTE_WEIGHT" \
  --cte-weight "$CTE_WEIGHT" \
  --finish-weight "$FINISH_WEIGHT" \
  --target-min-linear "$TARGET_MIN_LINEAR" \
  --target-max-linear-drop "$TARGET_MAX_LINEAR_DROP" \
  --target-linear-blend "$TARGET_LINEAR_BLEND" \
  --target-omega-blend "$TARGET_OMEGA_BLEND" \
  --deconf-min-linear "$DECONF_MIN_LINEAR" \
  --deconf-max-linear-drop "$DECONF_MAX_LINEAR_DROP" \
  --deconf-linear-blend "$DECONF_LINEAR_BLEND" \
  --deconf-omega-blend "$DECONF_OMEGA_BLEND" \
  --device cpu
