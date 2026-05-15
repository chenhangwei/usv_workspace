#!/bin/bash
# Smoke evaluation for fresh313.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt}" \
OUT="${OUT:-/tmp/fresh313_eval}" \
LABEL="${LABEL:-fresh313}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"