#!/bin/bash
# fresh320f: minimal-noise micro-tune from fresh320b; no pair guard and no release.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh320b_pairwise_guard_from_fresh318_step1260.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh320f_micro_no_release_from_fresh320b.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh320f_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-420}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-233}" \
exec bash scripts/train_fresh320a.sh \
  --curriculum-seed 1461 \
  --curriculum-seed 1460 \
  --curriculum-seed 1458 \
  --curriculum-seed 1463 \
  --learning-rate 1.6e-8 \
  --learning-rate-end 8.0e-9 \
  --clip-range 0.000018 \
  --update-epochs 1 \
  --random-deconflict-weight 5.00 \
  --random-deconflict-weight-end 5.15 \
  --random-deconflict-pretrain-epochs 0 \
  --random-role-balance-weight 3.65 \
  --random-role-balance-weight-end 3.85 \
  --random-role-balance-pretrain-epochs 0 \
  --random-pairwise-role-guard-weight 0.0 \
  --random-pairwise-role-guard-weight-end 0.0 \
  --random-safe-finish-weight 0.0 \
  --random-safe-finish-weight-end 0.0 \
  --random-offroute-finish-weight 0.65 \
  --random-offroute-finish-weight-end 0.75 \
  --random-offroute-finish-min-team-separation 3.00 \
  --random-offroute-finish-min-neighbor-separation 3.10 \
  --random-offroute-finish-target-speed 0.10 \
  --random-offroute-finish-min-speed 0.040 \
  --random-offroute-finish-max-omega 0.12 \
  --random-offroute-finish-omega-weight 0.36 \
  --random-cte-recovery-weight 0.10 \
  --random-cte-recovery-weight-end 0.12 \
  --random-cte-recovery-min-neighbor-separation 2.10 \
  --random-cte-recovery-target-speed 0.08 \
  --random-cte-recovery-min-speed 0.030 \
  --random-cte-recovery-max-omega 0.12 \
  --random-cte-recovery-omega-weight 0.12 \
  --policy-anchor-weight 2400.0 \
  --policy-anchor-weight-end 2800.0 \
  "$@"