#!/bin/bash
# fresh489: raw-CTE signed-return repair from fresh486.
#
# fresh488 proved the new raw/overflow observation path works, but seed1460
# usv_02 still drove outward at high positive raw CTE.  This branch keeps the
# raw CTE source and switches the high-CTE teacher to signed-cte so positive CTE
# commands negative yaw, with a much lower high-CTE speed target.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh486_signed_cte_brake_from_fresh484.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh489_raw_cte_signed_return_from_fresh486.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh489_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 3.0e-8 \
  --learning-rate-end 1.4e-8 \
  --clip-range 0.000020 \
  --update-epochs 4 \
  --random-cte-source raw \
  --random-cte-recovery-agent-index 1 \
  --random-cte-recovery-min-abs-cte 0.60 \
  --random-cte-recovery-full-abs-cte 4.20 \
  --random-cte-recovery-max-distance 22.0 \
  --random-cte-recovery-min-neighbor-separation 4.5 \
  --random-cte-recovery-omega-mode signed-cte \
  --random-cte-recovery-weight 2.20 \
  --random-cte-recovery-weight-end 3.20 \
  --random-cte-recovery-target-speed 0.075 \
  --random-cte-recovery-min-speed 0.000 \
  --random-cte-recovery-max-omega 0.50 \
  --random-cte-recovery-omega-reference 0.70 \
  --random-cte-recovery-linear-weight 1.60 \
  --random-cte-recovery-omega-weight 2.25 \
  --random-cte-recovery-speed-cte-slowdown 1.00 \
  --random-cte-recovery-speed-heading-gate 0.20 \
  --random-recovery-pretrain-epochs 6 \
  --random-recovery-pretrain-learning-rate 2.00e-6 \
  --random-recovery-pretrain-max-grad-norm 0.30 \
  --random-recovery-pretrain-cte-scale 2.40 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.45 \
  --random-clear-ahead-weight-end 0.70 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.62 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 1.80 \
  --random-clear-ahead-speed-cte-slowdown 1.00 \
  --random-clear-ahead-speed-cte-start 0.55 \
  --random-clear-ahead-speed-cte-full 1.60 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.135 \
  --random-clear-ahead-min-speed 0.015 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 1.80 \
  --policy-anchor-weight 4500.0 \
  --policy-anchor-weight-end 6500.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"