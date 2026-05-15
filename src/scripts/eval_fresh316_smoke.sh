#!/bin/bash
# Smoke evaluation for fresh316 trace-fit model.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh316_single_overtake_cte_tracefit_from_fresh313.pt}" \
OUT="${OUT:-/tmp/fresh316_eval}" \
LABEL="${LABEL:-fresh316}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"