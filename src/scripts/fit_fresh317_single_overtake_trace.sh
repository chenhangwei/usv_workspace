#!/bin/bash
# fresh317: second trace-fit pass using fresh316 closed-loop traces.

set -eo pipefail

cd "$(dirname "$0")/.."

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh316_single_overtake_cte_tracefit_from_fresh313.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh317_single_overtake_iter_tracefit_from_fresh316.pt}" \
TRACE_DIR="${TRACE_DIR:-/tmp/fresh317_single_overtake_trace}" \
SKIP_TRACE="${SKIP_TRACE:-0}" \
EPOCHS="${EPOCHS:-260}" \
LEARNING_RATE="${LEARNING_RATE:-1.0e-5}" \
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-1.2}" \
SCRIPTED_WEIGHT="${SCRIPTED_WEIGHT:-9.0}" \
TRACE_STRIDE="${TRACE_STRIDE:-4}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/fit_fresh315_single_overtake_trace.sh \
  --train-all-actor