#!/bin/bash
# fresh484: keep fresh483's route push, but add a stronger high-CTE brake.
#
# fresh483 proved the policy can move usv_02 forward, but it did so by letting
# |CTE| climb to about 4.0.  This continuation starts from fresh483, keeps a
# moderate clear-ahead teacher, and makes high-CTE samples crawl while the CTE
# recovery teacher gets more yaw authority than fresh483.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh483_balanced_soft_cte_from_fresh481.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh484_high_cte_brake_from_fresh483.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh484_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 4.6e-8 \
  --learning-rate-end 2.2e-8 \
  --clip-range 0.00003 \
  --update-epochs 4 \
  --random-cte-recovery-omega-mode goal-heading \
  --random-cte-recovery-weight 1.35 \
  --random-cte-recovery-weight-end 2.05 \
  --random-cte-recovery-target-speed 0.090 \
  --random-cte-recovery-min-speed 0.045 \
  --random-cte-recovery-max-omega 0.40 \
  --random-cte-recovery-omega-reference 0.82 \
  --random-cte-recovery-linear-weight 0.55 \
  --random-cte-recovery-omega-weight 1.45 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.45e-6 \
  --random-recovery-pretrain-max-grad-norm 0.28 \
  --random-recovery-pretrain-cte-scale 1.15 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.95 \
  --random-clear-ahead-weight-end 1.35 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.72 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 2.80 \
  --random-clear-ahead-speed-cte-slowdown 1.00 \
  --random-clear-ahead-speed-cte-start 0.95 \
  --random-clear-ahead-speed-cte-full 2.30 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.195 \
  --random-clear-ahead-min-speed 0.030 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 3.70 \
  --policy-anchor-weight 5200.0 \
  --policy-anchor-weight-end 7800.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
