#!/bin/bash
# fresh637: shoulder-pass — both-arc head-on (Rule 14), symmetric risk, early wake-up.
# Supersedes fresh636 (never launched). Root causes: repo memory
# fresh635_sitl_s2_diagnosis_20260812 + P0 risk-blindness fix (giveway_only gate
# was inside _compute_conflict_risk, blinding the stand-on hull).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export WARMSTART_FROM="${WARMSTART_FROM:-/mnt/data/checkpoints/usv_rl/fresh635_graceful_encounter_5usv.pt}"
export OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh637_shoulder_pass_5usv.pt}"
export CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh637_checkpoints}"
export EVAL_DIR="${EVAL_DIR:-/mnt/data/checkpoints/usv_rl/fresh637_eval}"
export TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-500000}"
export EXPERIMENT_NAME="${EXPERIMENT_NAME:-fresh637}"
export EXPERIMENT_SUMMARY="${EXPERIMENT_SUMMARY:-SHOULDER-PASS: symmetric conflict risk (P0 fix), Rule-14 both-arc head-on, CPA horizon 8->12s distance 7m turn 6.5m, keep-speed floor 0.24, DCPA 1.40, pair-mastery curriculum head_on x4 crossing x4}"

# Keep speed during conflict (fresh636 stack, kills the crawl equilibrium).
export CONFLICT_SPEED_RELIEF="${CONFLICT_SPEED_RELIEF:-0.30}"
export CONFLICT_SPEED_FLOOR="${CONFLICT_SPEED_FLOOR:-0.24}"

# Geometry-first CPA shaping, waking early enough for pure-geometry avoidance
# (head-on closing 0.68 m/s: 12s horizon = ~8m; each hull needs ~7s for 0.6m
# lateral at a 15-degree offset).
export ANTICIPATORY_CPA_TIME_HORIZON="${ANTICIPATORY_CPA_TIME_HORIZON:-12.0}"
export ANTICIPATORY_CPA_DISTANCE="${ANTICIPATORY_CPA_DISTANCE:-7.0}"
export ANTICIPATORY_DCPA_TARGET="${ANTICIPATORY_DCPA_TARGET:-1.40}"
export ANTICIPATORY_DCPA_DEFICIT_WEIGHT="${ANTICIPATORY_DCPA_DEFICIT_WEIGHT:-1.6}"
export ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT="${ANTICIPATORY_DCPA_IMPROVEMENT_WEIGHT:-1.3}"
export ANTICIPATORY_CLOSING_REDUCTION_WEIGHT="${ANTICIPATORY_CLOSING_REDUCTION_WEIGHT:-0.0}"
# Gate now lives in CPA shaping only (risk stays symmetric); head-on exempted in code.
export ANTICIPATORY_CPA_GIVEWAY_ONLY="${ANTICIPATORY_CPA_GIVEWAY_ONLY:-1}"

# Earlier single-arc turn window.
export ANTICIPATORY_TURN_REWARD_WEIGHT="${ANTICIPATORY_TURN_REWARD_WEIGHT:-0.7}"
export ANTICIPATORY_TURN_PENALTY_WEIGHT="${ANTICIPATORY_TURN_PENALTY_WEIGHT:-0.25}"
export ANTICIPATORY_TURN_DISTANCE="${ANTICIPATORY_TURN_DISTANCE:-6.5}"

# Role stack: course hold now head-on-exempt; speed asymmetry keeps BOTH hulls
# at cruise in head-on (code), so only crossing/overtaking get the speed split.
export ROLE_STAND_ON_COURSE_HOLD_WEIGHT="${ROLE_STAND_ON_COURSE_HOLD_WEIGHT:-1.2}"

# Pair-mastery curriculum: master the two canonical pair encounters first.
export TWO_USV_CROSSING_CURRICULUM_REPEATS="${TWO_USV_CROSSING_CURRICULUM_REPEATS:-4}"
export TWO_USV_HEAD_ON_CURRICULUM_REPEATS="${TWO_USV_HEAD_ON_CURRICULUM_REPEATS:-4}"

# Smoothness inherited from fresh635.
export ACTION_SMOOTHNESS_WEIGHT="${ACTION_SMOOTHNESS_WEIGHT:-0.28}"
export ANGULAR_ACCEL_PENALTY_WEIGHT="${ANGULAR_ACCEL_PENALTY_WEIGHT:-0.30}"
export FORWARD_SPEED_CHANGE_PENALTY_WEIGHT="${FORWARD_SPEED_CHANGE_PENALTY_WEIGHT:-0.12}"
export EVAL_TWO_USV_CROSSING="${EVAL_TWO_USV_CROSSING:-1}"

exec "$SCRIPT_DIR/train_fresh634_decisive_encounter_5usv.sh"
