#!/bin/bash
# fresh316: refit fresh315 traces with CTE-forced single-overtake return targets.

set -eo pipefail

cd "$(dirname "$0")/.."

SOURCE_MODEL="${SOURCE_MODEL:-/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh316_single_overtake_cte_tracefit_from_fresh313.pt}" \
TRACE_DIR="${TRACE_DIR:-/tmp/fresh315_single_overtake_trace}" \
SKIP_TRACE="${SKIP_TRACE:-1}" \
EPOCHS="${EPOCHS:-220}" \
LEARNING_RATE="${LEARNING_RATE:-1.2e-5}" \
ANCHOR_WEIGHT="${ANCHOR_WEIGHT:-1.8}" \
SCRIPTED_WEIGHT="${SCRIPTED_WEIGHT:-7.0}" \
exec bash scripts/fit_fresh315_single_overtake_trace.sh "$@"