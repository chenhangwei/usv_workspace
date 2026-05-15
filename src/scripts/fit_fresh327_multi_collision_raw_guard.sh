#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source ../install/setup.bash
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh321_yield_guard_tracefit_from_fresh320b.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh327_multi_collision_raw_guard_from_fresh321.pt}"
TRACE_A="${TRACE_A:-/tmp/fresh321_collision_raw_d207_batch/attempt1/seed1461.json}"
TRACE_B="${TRACE_B:-/tmp/fresh321_collision_raw_d207_multi/attempt7/seed1461.json}"
TRACE_C="${TRACE_C:-/tmp/fresh321_collision_raw_d207_multi_b/attempt1/seed1461.json}"
ANCHOR_DIR="${ANCHOR_DIR:-/tmp/fresh321_yield_guard_trace}"
EPOCHS="${EPOCHS:-55}"
LEARNING_RATE="${LEARNING_RATE:-4.0e-7}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-160.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-0.80}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-1.20}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.010}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.05}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.060}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi
for trace in "$TRACE_A" "$TRACE_B" "$TRACE_C"; do
  if [[ ! -f "$trace" ]]; then
    echo "missing collision raw trace: $trace" >&2
    exit 1
  fi
done

anchor_args=()
for trace in \
  "$ANCHOR_DIR/fresh320b_seed1460_attempt1_raw_trace.json" \
  "$ANCHOR_DIR/fresh320b_seed1460_attempt2_raw_trace.json" \
  "$ANCHOR_DIR/fresh320b_seed1461_attempt1_raw_trace.json" \
  "$ANCHOR_DIR/fresh320b_seed1461_attempt2_raw_trace.json"; do
  if [[ -f "$trace" ]]; then
    anchor_args+=(--anchor-trace-json "$trace")
  fi
done

mkdir -p "$(dirname "$OUTPUT")"

/bin/python3 -u -m usv_rl.fit_mappo_trace_targets \
  --model "$SOURCE_MODEL" \
  --trace-json "$TRACE_A" \
  --trace-json "$TRACE_B" \
  --trace-json "$TRACE_C" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 32 \
  --learning-rate "$LEARNING_RATE" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size 256 \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --target-priority deconf \
  --target-shape-kinds deconf,guard \
  --deconf-weight "$DECONF_WEIGHT" \
  --deconf-omega-blend 1.0 \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-yield-only \
  --risk-guard-require-threat \
  --risk-guard-min-threat 0.50 \
  --risk-guard-max-separation 1.00 \
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR" \
  --risk-guard-min-starboard-omega 0.0 \
  --risk-guard-min-starboard-threat 1.0 \
  --risk-guard-min-distance 0.40

echo "$OUTPUT"