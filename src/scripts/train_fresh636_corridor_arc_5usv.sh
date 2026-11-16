#!/bin/bash
# fresh636: single-arc corridor avoidance — early turn, keep speed, return to route.
# Root cause fix for fresh635 SITL mutual-crawl (see repo memory fresh635_sitl_s2_diagnosis_20260812).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh635_graceful_encounter_5usv.pt}"
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh636_corridor_arc_5usv.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh636_checkpoints}"
export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh636_eval}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-400000}"
export EXPERIMENT_NAME="${EXPERIMENT_NAME:-fresh636}"
export EXPERIMENT_SUMMARY="${EXPERIMENT_SUMMARY:-CORRIDOR-ARC: conflict speed relief 0.55->0.30 floor 0.18->0.24 (kill crawl equilibrium), closing-reduction reward removed, DCPA target 1.15->1.40 deficit 1.6 improvement 1.3, anticipatory turn 3.5m->5.0m reward 0.7}"

# Keep speed during conflict: reward-optimal conflict speed ~0.24 (aligned with role_giveway_speed 0.26).
export CONFLICT_SPEED_RELIEF="${CONFLICT_SPEED_RELIEF:-0.30}"
export CONFLICT_SPEED_FLOOR="${CONFLICT_SPEED_FLOOR:-0.24}"

# Geometry-first CPA shaping: remove brake-hackable closing-reduction reward,
# push separation target past the near-miss band.
export ANTICIPATORY_CLOSING_REDUCTION_WEIGHT="${ANTICIPATORY_CLOSING_REDUCTION_WEIGHT:-0.0}"
export ANTICIPATORY_CPA_DISTANCE="${ANTICIPATORY_CPA_DISTANCE:-5.5}"
export ANTICIPATORY_DCPA_TARGET="${ANTICIPATORY_DCPA_TARGET:-1.40}"
export ANTICIPATORY_DCPA_DEFICIT_WEIGHT="${ANTICIPATORY_DCPA_DEFICIT_WEIGHT:-1.6}"
export ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT="${ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT:-1.3}"
export ANTICIPATORY_CPA_GIVEWAY_ONLY="${ANTICIPATORY_CPA_GIVEWAY_ONLY:-1}"

# Earlier, stronger single-arc turn window.
export ANTICIPATORY_TURN_REWARD_WEIGHT="${ANTICIPATORY_TURN_REWARD_WEIGHT:-0.7}"
export ANTICIPATORY_TURN_PENALTY_WEIGHT="${ANTICIPATORY_TURN_PENALTY_WEIGHT:-0.25}"
export ANTICIPATORY_TURN_DISTANCE="${ANTICIPATORY_TURN_DISTANCE:-5.0}"

# Inherit fresh635 role/smoothness stack.
export TWO_USV_CROSSING_CURRICULUM_REPEATS="${TWO_USV_CROSSING_CURRICULUM_REPEATS:-4}"
export ROLE_STAND_ON_COURSE_HOLD_WEIGHT="${ROLE_STAND_ON_COURSE_HOLD_WEIGHT:-1.2}"
export ACTION_SMOOTHNESS_WEIGHT="${ACTION_SMOOTHNESS_WEIGHT:-0.28}"
export ANGULAR_ACCEL_PENALTY_WEIGHT="${ANGULAR_ACCEL_PENALTY_WEIGHT:-0.30}"
export FORWARD_SPEED_CHANGE_PENALTY_WEIGHT="${FORWARD_SPEED_CHANGE_PENALTY_WEIGHT:-0.12}"
export EVAL_TWO_USV_CROSSING="${EVAL_TWO_USV_CROSSING:-1}"

exec "$SCRIPT_DIR/train_fresh634_decisive_encounter_5usv.sh"
