#!/bin/bash
# fresh485: suppress high-CTE high-speed escape after fresh484.
#
# fresh484 kept seed1461 near |CTE| ~= 3, but focused r3 exposed one seed1460
# repeat where usv_02 overflowed to raw CTE > 11 while moving fast.  This keeps
# the fresh484 correction but enables the recovery-heading speed gate, so high
# CTE samples only keep speed when aligned with the recovery heading.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh484_high_cte_brake_from_fresh483.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh485_heading_gated_cte_brake_from_fresh484.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh485_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-1200}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-220}" \
exec bash scripts/train_fresh464_goal_heading_action_probe.sh \
  --learning-rate 3.8e-8 \
  --learning-rate-end 1.8e-8 \
  --clip-range 0.000025 \
  --update-epochs 4 \
  --random-cte-recovery-omega-mode goal-heading \
  --random-cte-recovery-weight 1.55 \
  --random-cte-recovery-weight-end 2.30 \
  --random-cte-recovery-target-speed 0.075 \
  --random-cte-recovery-min-speed 0.000 \
  --random-cte-recovery-max-omega 0.42 \
  --random-cte-recovery-omega-reference 0.78 \
  --random-cte-recovery-linear-weight 1.15 \
  --random-cte-recovery-omega-weight 1.55 \
  --random-cte-recovery-speed-cte-slowdown 1.00 \
  --random-cte-recovery-speed-heading-gate 1.00 \
  --random-recovery-pretrain-epochs 3 \
  --random-recovery-pretrain-learning-rate 1.25e-6 \
  --random-recovery-pretrain-max-grad-norm 0.25 \
  --random-recovery-pretrain-cte-scale 1.45 \
  --random-recovery-safety-gate-scale 0.25 \
  --random-recovery-safety-gate-mode sample \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-agent-index 1 \
  --random-clear-ahead-weight 0.85 \
  --random-clear-ahead-weight-end 1.25 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 30.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 18.0 \
  --random-clear-ahead-min-route-progress 0.035 \
  --random-clear-ahead-max-route-progress 0.72 \
  --random-clear-ahead-min-abs-cte 0.0 \
  --random-clear-ahead-max-abs-cte 2.65 \
  --random-clear-ahead-speed-cte-slowdown 1.00 \
  --random-clear-ahead-speed-cte-start 0.80 \
  --random-clear-ahead-speed-cte-full 2.15 \
  --random-clear-ahead-min-neighbor-separation 6.2 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source constant \
  --random-clear-ahead-target-speed 0.180 \
  --random-clear-ahead-min-speed 0.020 \
  --random-clear-ahead-max-omega 0.0 \
  --random-clear-ahead-omega-weight 0.0 \
  --random-recovery-pretrain-clear-scale 3.20 \
  --policy-anchor-weight 5600.0 \
  --policy-anchor-weight-end 8400.0 \
  --policy-anchor-exclude-random-cte-recovery \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"
