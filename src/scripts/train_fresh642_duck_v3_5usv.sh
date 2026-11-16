#!/bin/bash
# fresh642: DUCK-V3 -- clean-teacher BC re-init + gentle full-domain-anchored RL polish.
#
# WHY (2026-08-24 diagnosis, memories/repo/graceful_duck_roadmap_20260817.md):
#   The anchor dilemma (weak anchor => reward stack destroys the duck arcs,
#   fresh640 51k coll 0.50 -> 250k 0.83; strong/masked anchor => learning
#   freezes, fresh641 actor L1 drift 0.0029, best only 0.58) exists because the
#   v2 teacher's behaviour genuinely conflicts with the reward optimum: it
#   biases starboard for ANY ahead neighbour inside the 5.6m anticipation
#   window (clip min 0.05), which BC clones into a full-domain right drift
#   (SITL: open-water signed omega median -0.026..-0.077, CTE p90 0.8-1.3m).
#
#   fresh642 removes the conflict at the SOURCE: teacher v3 gates the entire
#   avoidance geometry on a genuine predicted conflict (0<TCPA<12s, DCPA<1.6m,
#   dist<5.6m) with a per-neighbour hysteresis latch. Open water = pure
#   goal-tracking (straight), encounters = committed duck arcs (no chatter).
#   With a clean teacher, anchoring the WHOLE domain protects straightness AND
#   the arcs simultaneously -- no masking needed.
#
# CHANGES vs fresh641 (everything else inherited verbatim via the 641 script):
#   - WARMSTART = fresh642_bc_init.pt (BC on the v3 TCPA-gated teacher dataset)
#   - anchor 0.6 CONSTANT full-domain (was 0.8 conflict-only-masked)
#   - --policy-anchor-only-cpa-conflict OFF
#
# ACCEPTANCE (on top of the duck P0-P4 set):
#   open-water |omega| p50 < 0.09 (deadband), CTE p90 < 0.5m,
#   collision rate <= 0.50 (51427 baseline), P0 role separation > 52%.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CKPT=/mnt/data/checkpoints/usv_rl

export WARMSTART_FROM="${WARMSTART_FROM:-$CKPT/fresh642_bc_init.pt}"
export OUTPUT="${OUTPUT:-$CKPT/fresh642_duck_v3_5usv.pt}"
export CKPT_DIR="${CKPT_DIR:-$CKPT/fresh642_checkpoints}"
export EVAL_DIR="${EVAL_DIR:-$CKPT/fresh642_eval}"
export EXPERIMENT_NAME="${EXPERIMENT_NAME:-fresh642}"
export EXPERIMENT_SUMMARY="${EXPERIMENT_SUMMARY:-DUCK-V3: BC re-init from the TCPA-gated teacher (v3: avoidance only on genuine predicted conflict + per-neighbour latch; open water drives straight), then gentle RL polish with anchor 0.6 CONSTANT over the FULL domain (clean teacher removes the anchor-vs-reward conflict that killed fresh640/641); dither stack (smoothness 0.45, ang-accel 0.50) and conflict-resolution 1.4 inherited from fresh641; 300k, min-update-samples 1024}"
export POLICY_ANCHOR_WEIGHT="${POLICY_ANCHOR_WEIGHT:-0.6}"
export POLICY_ANCHOR_WEIGHT_END="${POLICY_ANCHOR_WEIGHT_END:-0.6}"
export POLICY_ANCHOR_ONLY_CPA_CONFLICT="${POLICY_ANCHOR_ONLY_CPA_CONFLICT:-0}"

exec bash "$SCRIPT_DIR/train_fresh641_duck_lock_5usv.sh"
