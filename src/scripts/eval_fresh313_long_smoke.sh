#!/bin/bash
# Long-timeout smoke evaluation for fresh313 single-overtake completion.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt}" \
OUT="${OUT:-/tmp/fresh313_eval_long}" \
LABEL="${LABEL:-fresh313_long}" \
STEPS="${STEPS:-1200}" \
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}" \
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"