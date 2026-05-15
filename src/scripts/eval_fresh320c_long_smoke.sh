#!/bin/bash
# Long-timeout smoke evaluation for fresh320c.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh320c_pairwise_guard_progress_from_fresh320b.pt}" \
OUT="${OUT:-/tmp/fresh320c_eval_long}" \
LABEL="${LABEL:-fresh320c_long}" \
STEPS="${STEPS:-1200}" \
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}" \
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"