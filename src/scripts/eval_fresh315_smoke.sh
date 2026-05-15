#!/bin/bash
# Smoke evaluation for fresh315 trace-fit model.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh315_single_overtake_tracefit_from_fresh313.pt}" \
OUT="${OUT:-/tmp/fresh315_eval}" \
LABEL="${LABEL:-fresh315}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"