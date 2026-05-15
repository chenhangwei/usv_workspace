#!/bin/bash
# Long-timeout smoke evaluation for fresh320b.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt}" \
OUT="${OUT:-/tmp/fresh320b_eval_long}" \
LABEL="${LABEL:-fresh320b_long}" \
STEPS="${STEPS:-1200}" \
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}" \
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"