#!/bin/bash
# Smoke evaluation for fresh312.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh312_single_overtake_return_loose_anchor_from_fresh310.pt}" \
OUT="${OUT:-/tmp/fresh312_eval}" \
LABEL="${LABEL:-fresh312}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"