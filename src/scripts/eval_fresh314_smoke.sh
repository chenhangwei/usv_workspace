#!/bin/bash
# Smoke evaluation for fresh314.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh314_single_overtake_heading_return_from_fresh310.pt}" \
OUT="${OUT:-/tmp/fresh314_eval}" \
LABEL="${LABEL:-fresh314}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"