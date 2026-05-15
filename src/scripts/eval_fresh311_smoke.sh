#!/bin/bash
# Smoke evaluation for fresh311.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh311_single_overtake_return_from_fresh310.pt}" \
OUT="${OUT:-/tmp/fresh311_eval}" \
LABEL="${LABEL:-fresh311}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"