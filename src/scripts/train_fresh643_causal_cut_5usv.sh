#!/bin/bash
# fresh643: CAUSAL-CUT -- fresh642 stack with the BC causal-confusion fix.
#
# ROOT CAUSE (2026-08-26 response-surface diagnosis): the fresh642 BC student
# learned omega ~= 0.8*raw_angular_z - 0.09 (a nav-command follower): heading
# response was sign-blind and the -0.09 offset showed up in SITL as a
# 21-28 deg heading-error limit cycle (snake) once the MPC's slower raw_cmd
# dynamics shifted slot 8 to the -0.84 sigma tail of the training distribution.
#
# FIX: BC re-trained with input dropout p=0.6 on slots 8,10 (raw/final omega),
# forcing the student to steer from heading/CTE geometry. Response surface
# after fix: nominal bias -0.224 -> -0.021, raw-omega gain 1.26 -> ~0,
# heading response sign-correct 8/8 with starboard preference intact.
#
# RL: 50k SHORT run (fresh642 lesson: RL only adds value in the first ~20k
# steps, then the 0.83 reward attractor destroys the duck behaviour).
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CKPT=/mnt/data/checkpoints/usv_rl

export WARMSTART_FROM="${WARMSTART_FROM:-$CKPT/fresh643_bc_init.pt}"
export OUTPUT="${OUTPUT:-$CKPT/fresh643_causal_cut_5usv.pt}"
export CKPT_DIR="${CKPT_DIR:-$CKPT/fresh643_checkpoints}"
export EVAL_DIR="${EVAL_DIR:-$CKPT/fresh643_eval}"
export EXPERIMENT_NAME="${EXPERIMENT_NAME:-fresh643}"
export EXPERIMENT_SUMMARY="${EXPERIMENT_SUMMARY:-CAUSAL-CUT: fresh642 duck-v3 stack, BC re-init with input dropout (slots 8,10 p=0.6) severing the nav-command copy shortcut that caused the SITL heading limit-cycle; short 50k RL polish (value only in first ~20k), anchor 0.6 constant full domain, checkpoint every 8192 for dense early sampling}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-50000}"
export CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-8192}"
export POLICY_ANCHOR_WEIGHT="${POLICY_ANCHOR_WEIGHT:-0.6}"
export POLICY_ANCHOR_WEIGHT_END="${POLICY_ANCHOR_WEIGHT_END:-0.6}"
export POLICY_ANCHOR_ONLY_CPA_CONFLICT="${POLICY_ANCHOR_ONLY_CPA_CONFLICT:-0}"

exec bash "$SCRIPT_DIR/train_fresh641_duck_lock_5usv.sh"
