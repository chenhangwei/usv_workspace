#!/bin/bash
# Fixed-seed random-only screening for fresh179.

set -eo pipefail

cd "$(dirname "$0")/.."

export OUT="${OUT:-/tmp/fresh179_random_eval}"
export FINAL_MODEL="${FINAL_MODEL:-/mnt/data/checkpoints/usv_rl/fresh179_action_target_gate_from_fresh172.pt}"
export CKPT_MODEL="${CKPT_MODEL:-/mnt/data/checkpoints/usv_rl/fresh179_checkpoints/fresh179_action_target_gate_from_fresh172_step_0001260.pt}"
export BASE_DOMAIN="${BASE_DOMAIN:-226}"

exec ./scripts/eval_fresh172_random.sh