#!/bin/bash
# Smoke evaluation for fresh317 iterative trace-fit model.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh317_single_overtake_iter_tracefit_from_fresh316.pt}" \
OUT="${OUT:-/tmp/fresh317_eval}" \
LABEL="${LABEL:-fresh317}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"