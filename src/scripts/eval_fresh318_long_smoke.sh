#!/bin/bash
# Long-timeout smoke evaluation for fresh318 random hard-seed replay.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh318_random_hard_replay_from_fresh313.pt}" \
OUT="${OUT:-/tmp/fresh318_eval_long}" \
LABEL="${LABEL:-fresh318_long}" \
STEPS="${STEPS:-1200}" \
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}" \
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"