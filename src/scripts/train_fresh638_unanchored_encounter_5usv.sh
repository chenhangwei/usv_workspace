#!/bin/bash
# fresh638: unanchored encounters — break the policy-anchor fossil chain.
# Root cause (repo memory fresh638_anchor_fossil_20260815): every generation was
# MSE-pinned to its warmstart parent (anchor end-weight 0.35 never reaching 0),
# so encounter behaviour stayed the fresh633 straight-line fossil across four
# reward reworks (bare-policy eval: zero turning, min_sep 0.64-0.74 constant;
# SITL "avoidance" was the deployment guard's 0.16/0.40 turn floors, not the net).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh637_shoulder_pass_5usv.pt}"
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh638_unanchored_encounter_5usv.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh638_checkpoints}"
export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh638_eval}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-500000}"
export EXPERIMENT_NAME="${EXPERIMENT_NAME:-fresh638}"
export EXPERIMENT_SUMMARY="${EXPERIMENT_SUMMARY:-UNANCHORED-ENCOUNTERS: anchor exempt on predicted-CPA conflict samples, anchor 1.0/0.35 -> 0.6/0.10, entropy 0.01/0.003 -> 0.014/0.006 (re-open exploration); keeps fresh637 symmetric-risk Rule-14 reward stack}"

# THE fix: drop the anchor exactly where avoidance must be learned.
export POLICY_ANCHOR_EXCLUDE_CPA_CONFLICT="${POLICY_ANCHOR_EXCLUDE_CPA_CONFLICT:-1}"
export POLICY_ANCHOR_WEIGHT="${POLICY_ANCHOR_WEIGHT:-0.6}"
export POLICY_ANCHOR_WEIGHT_END="${POLICY_ANCHOR_WEIGHT_END:-0.10}"

# Re-open exploration so the freed policy can discover turn arcs
# (log_std re-inflates through the entropy bonus).
export ENTROPY_COEF="${ENTROPY_COEF:-0.014}"
export ENTROPY_COEF_END="${ENTROPY_COEF_END:-0.006}"

# fresh637 reward stack unchanged: symmetric risk (code), Rule-14 both-arc
# head-on (code), keep-speed floor, geometry-first DCPA, early wake-up.
export CONFLICT_SPEED_RELIEF="${CONFLICT_SPEED_RELIEF:-0.30}"
export CONFLICT_SPEED_FLOOR="${CONFLICT_SPEED_FLOOR:-0.24}"
export ANTICIPATORY_CPA_TIME_HORIZON="${ANTICIPATORY_CPA_TIME_HORIZON:-12.0}"
export ANTICIPATORY_CPA_DISTANCE="${ANTICIPATORY_CPA_DISTANCE:-7.0}"
export ANTICIPATORY_DCPA_TARGET="${ANTICIPATORY_DCPA_TARGET:-1.40}"
export ANTICIPATORY_DCPA_DEFICIT_WEIGHT="${ANTICIPATORY_DCPA_DEFICIT_WEIGHT:-1.6}"
export ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT="${ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT:-1.3}"
export ANTICIPATORY_CLOSING_REDUCTION_WEIGHT="${ANTICIPATORY_CLOSING_REDUCTION_WEIGHT:-0.0}"
export ANTICIPATORY_CPA_GIVEWAY_ONLY="${ANTICIPATORY_CPA_GIVEWAY_ONLY:-1}"
export ANTICIPATORY_TURN_REWARD_WEIGHT="${ANTICIPATORY_TURN_REWARD_WEIGHT:-0.7}"
export ANTICIPATORY_TURN_PENALTY_WEIGHT="${ANTICIPATORY_TURN_PENALTY_WEIGHT:-0.25}"
export ANTICIPATORY_TURN_DISTANCE="${ANTICIPATORY_TURN_DISTANCE:-6.5}"
export ROLE_STAND_ON_COURSE_HOLD_WEIGHT="${ROLE_STAND_ON_COURSE_HOLD_WEIGHT:-1.2}"
export TWO_USV_CROSSING_CURRICULUM_REPEATS="${TWO_USV_CROSSING_CURRICULUM_REPEATS:-4}"
export TWO_USV_HEAD_ON_CURRICULUM_REPEATS="${TWO_USV_HEAD_ON_CURRICULUM_REPEATS:-4}"
export ACTION_SMOOTHNESS_WEIGHT="${ACTION_SMOOTHNESS_WEIGHT:-0.28}"
export ANGULAR_ACCEL_PENALTY_WEIGHT="${ANGULAR_ACCEL_PENALTY_WEIGHT:-0.30}"
export FORWARD_SPEED_CHANGE_PENALTY_WEIGHT="${FORWARD_SPEED_CHANGE_PENALTY_WEIGHT:-0.12}"
export EVAL_TWO_USV_CROSSING="${EVAL_TWO_USV_CROSSING:-1}"

exec "$SCRIPT_DIR/train_fresh634_decisive_encounter_5usv.sh"
