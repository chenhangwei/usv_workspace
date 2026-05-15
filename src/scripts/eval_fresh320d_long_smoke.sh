#!/bin/bash
# Long-timeout smoke evaluation for fresh320d.

set -eo pipefail

cd "$(dirname "$0")/.."

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh320d_conservative_release_from_fresh320b.pt}" \
OUT="${OUT:-/tmp/fresh320d_eval_long}" \
LABEL="${LABEL:-fresh320d_long}" \
STEPS="${STEPS:-1200}" \
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}" \
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}" \
BASE_DOMAIN="${BASE_DOMAIN:-210}" \
exec bash scripts/eval_fresh309_smoke.sh "$@"