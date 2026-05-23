#!/bin/bash
# fresh488: raw/overflow CTE observation branch from fresh486.
#
# fresh486 kept seed1461 useful but seed1460 could still sprint after clipped CTE
# saturation.  This branch keeps the signed-CTE recovery direction, exposes raw
# CTE/overflow to the actor, and switches the random route gates to raw CTE so
# clear-ahead speed pressure stops when the true route error is already large.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh486_signed_cte_brake_from_fresh484.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh488_raw_cte_obs_from_fresh486.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh488_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 3.2e-8 \
  --learning-rate-end 1.5e-8 \
  --clip-range 0.000022 \
  --update-epochs 4 \
  --random-cte-source raw \
  --random-cte-recovery-agent-index 1 \
  --random-cte-recovery-min-abs-cte 0.80 \
  --random-cte-recovery-full-abs-cte 5.50 \
  --random-cte-recovery-max-distance 18.0 \
  --random-cte-recovery-min-neighbor-separation 5.2 \
  --random-cte-recovery-omega-mode signed-cte-inverted \
  --random-cte-recovery-weight 1.45 \
  --random-cte-recovery-weight-end 2.15 \
  --random-cte-recovery-target-speed 0.105 \
  --random-cte-recovery-min-speed 0.020 \
  --random-cte-recovery-max-omega 0.44 \
  --random-cte-recovery-omega-reference 0.88 \
  --random-cte-recovery-linear-weight 0.75 \
  --random-cte-recovery-omega-weight 1.65 \
  --random-cte-recovery-speed-cte-slowdown 1.00 \
  --random-cte-recovery-speed-heading-gate 0.25 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.10e-6 \
  --random-recovery-pretrain-max-grad-norm 0.24 \
  --random-recovery-pretrain-cte-scale 1.35 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.82 \
  --random-clear-ahead-weight-end 1.20 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.70 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 2.40 \
  --random-clear-ahead-speed-cte-slowdown 1.00 \
  --random-clear-ahead-speed-cte-start 0.70 \
  --random-clear-ahead-speed-cte-full 2.10 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.180 \
  --random-clear-ahead-min-speed 0.035 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 3.20 \
  --policy-anchor-weight 5800.0 \
  --policy-anchor-weight-end 8700.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"