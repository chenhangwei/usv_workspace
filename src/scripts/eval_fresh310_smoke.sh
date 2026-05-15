#!/bin/bash
# Smoke evaluation for fresh310 single overtaking imitation run.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh310_single_overtake_bc_from_fresh309.pt}" \
OUT="${OUT:-/tmp/fresh310_eval}" \
LABEL="${LABEL:-fresh310}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh