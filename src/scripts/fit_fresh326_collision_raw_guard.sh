#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source ../install/setup.bash
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh321_yield_guard_tracefit_from_fresh320b.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh326_collision_raw_guard_from_fresh321.pt}"
TRACE_COLLISION="${TRACE_COLLISION:-/tmp/fresh321_collision_raw_d207_batch/attempt1/seed1461.json}"
ANCHOR_DIR="${ANCHOR_DIR:-/tmp/fresh321_yield_guard_trace}"
EPOCHS="${EPOCHS:-45}"
LEARNING_RATE="${LEARNING_RATE:-5.0e-7}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-120.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-0.60}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-1.60}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.015}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.06}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.070}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi
if [[ ! -f "$TRACE_COLLISION" ]]; then
  echo "missing collision raw trace: $TRACE_COLLISION" >&2
  exit 1
fi

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
  --trace-json "$TRACE_COLLISION" \
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
  --risk-guard-min-distance 1.20

echo "$OUTPUT"