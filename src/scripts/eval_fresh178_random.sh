#!/bin/bash
# Fixed-seed random-only screening for fresh178.

set -eo pipefail

cd "$(dirname "$0")/.."

OUT="${OUT:-/tmp/fresh178_random_eval}" \
FINAL_MODEL="${FINAL_MODEL:-/mnt/data/checkpoints/usv_rl/fresh178_cte_recovery_from_fresh172_step1260.pt}" \
CKPT_MODEL="${CKPT_MODEL:-/mnt/data/checkpoints/usv_rl/fresh178_checkpoints/fresh178_cte_recovery_from_fresh172_step1260_step_0001260.pt}" \
BASE_DOMAIN="${BASE_DOMAIN:-230}" \
scripts/eval_fresh172_random.sh