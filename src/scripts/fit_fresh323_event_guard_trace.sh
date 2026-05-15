#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source ../install/setup.bash
set -u
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh321_yield_guard_tracefit_from_fresh320b.pt}"
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh323_event_guard_tracefit_from_fresh321.pt}"
TRACE_D104="${TRACE_D104:-/tmp/fresh322_event_raw_d104/seed1461.json}"
TRACE_D105="${TRACE_D105:-/tmp/fresh322_event_raw_d105/seed1461.json}"
TRACE_D115="${TRACE_D115:-/tmp/fresh322_collision_event_d115/seed1461.json}"
ANCHOR_DIR="${ANCHOR_DIR:-/tmp/fresh321_yield_guard_trace}"
EPOCHS="${EPOCHS:-90}"
LEARNING_RATE="${LEARNING_RATE:-1.8e-6}"
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-32.0}"
DECONF_WEIGHT="${DECONF_WEIGHT:-0.75}"
RISK_GUARD_WEIGHT="${RISK_GUARD_WEIGHT:-3.00}"
MIN_ERROR_NORM="${MIN_ERROR_NORM:-0.020}"
MAX_GRAD_NORM="${MAX_GRAD_NORM:-0.12}"
RISK_GUARD_MAX_LINEAR="${RISK_GUARD_MAX_LINEAR:-0.075}"

if [[ ! -f "$SOURCE_MODEL" ]]; then
  echo "missing source model: $SOURCE_MODEL" >&2
  exit 1
fi
for trace in "$TRACE_D104" "$TRACE_D105" "$TRACE_D115"; do
  if [[ ! -f "$trace" ]]; then
    echo "missing event raw trace: $trace" >&2
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
  --trace-json "$TRACE_D104" \
  --trace-json "$TRACE_D105" \
  --trace-json "$TRACE_D115" \
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
  --deconf-omega-blend 0.95 \
  --low-speed-source-threshold 0.13 \
  --risk-guard-weight "$RISK_GUARD_WEIGHT" \
  --risk-guard-yield-only \
  --risk-guard-require-threat \
  --risk-guard-min-threat 0.20 \
  --risk-guard-max-separation 2.55 \
  --risk-guard-max-linear "$RISK_GUARD_MAX_LINEAR" \
  --risk-guard-min-starboard-omega 0.0 \
  --risk-guard-min-starboard-threat 1.0 \
  --risk-guard-min-distance 1.20

echo "$OUTPUT"