#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh321_yield_guard_tracefit_from_fresh320b.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh322_event_guard_tracefit_from_fresh321.pt}"
TRACE_D104="${TRACE_D104:-/tmp/fresh322_event_raw_d104/seed1461.json}"
TRACE_D105="${TRACE_D105:-/tmp/fresh322_event_raw_d105/seed1461.json}"
ANCHOR_DIR="${ANCHOR_DIR:-/tmp/fresh321_yield_guard_trace}"
EPOCHS="${EPOCHS:-90}"
LEARNING_RATE="${LEARNING_RATE:-2.0e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-28.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-0.35}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-3.20}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.020}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.12}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi
if [[ ! -f "$TRACE_D104" || ! -f "$TRACE_D105" ]]; then
  echo "missing event raw trace(s): $TRACE_D104 $TRACE_D105" >&2
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
  --trace-json "$TRACE_D104" \
  --trace-json "$TRACE_D105" \
  "${anchor_args[@]}" \
  --output "$OUTPUT" \
  --epochs "$EPOCHS" \
  --batch-size 128 \
  --learning-rate "$LEARNING_RATE" \
  --anchor-weight "$ANCHOR_WEIGHT" \
  --anchor-batch-size 256 \
  --max-grad-norm "$MAX_GRAD_NORM" \
  --min-error-norm "$MIN_ERROR_NORM" \
  --target-priority deconf \
  --target-shape-kinds deconf \
  --deconf-weight "$DECONF_WEIGHT" \
  --deconf-omega-blend 0.85 \
  --low-speed-source-threshold 0.13 \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-yield-only \
  --risk-guard-require-threat \
  --risk-guard-min-threat 0.20 \
  --risk-guard-max-separation 2.55 \
  --risk-guard-max-linear 0.055 \
  --risk-guard-min-starboard-omega 0.24 \
  --risk-guard-min-starboard-threat 0.18 \
  --risk-guard-min-distance 2.00

echo "$OUTPUT"