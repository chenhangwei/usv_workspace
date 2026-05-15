#!/bin/bash
# Fixed-seed random-only screening for fresh181.

set -eo pipefail

cd "$(dirname "$0")/.."

export OUT="${OUT:-/tmp/fresh181_random_eval}"
export FINAL_MODEL="${FINAL_MODEL:-/mnt/data/checkpoints/usv_rl/fresh181_pairwise_priority_delta_from_fresh172.pt}"
export CKPT_MODEL="${CKPT_MODEL:-/mnt/data/checkpoints/usv_rl/fresh181_checkpoints/fresh181_pairwise_priority_delta_from_fresh172_step_0001260.pt}"
export BASE_DOMAIN="${BASE_DOMAIN:-222}"

exec bash ./scripts/eval_fresh172_random.sh