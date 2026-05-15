#!/bin/bash
# Fixed-seed random-only screening for fresh182 active-slice trace fits.

set -eo pipefail

cd "$(dirname "$0")/.."

export OUT="${OUT:-/tmp/fresh182_random_eval}"
export FINAL_MODEL="${FINAL_MODEL:-/mnt/data/checkpoints/usv_rl/fresh182_checkpoints/fresh182_deconf_trace_anchor_from_fresh181_step1260.pt}"
export CKPT_MODEL="${CKPT_MODEL:-/mnt/data/checkpoints/usv_rl/fresh182_checkpoints/fresh182_deconf_trace_anchor_from_fresh181_step1260.pt}"
export BASE_DOMAIN="${BASE_DOMAIN:-188}"

exec bash ./scripts/eval_fresh172_random.sh