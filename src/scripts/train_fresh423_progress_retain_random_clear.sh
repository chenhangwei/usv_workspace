#!/bin/bash
# fresh423: fresh422 with clear-ahead explicitly enabled on random encounters.
# The fresh318 base script restricts random-clear-ahead to non-random route
# scenarios, so fresh422 produced rand_clear_active=0. This variant appends the
# actual hard-seed scenario while keeping the light cruise/strong-anchor setup.

set -eo pipefail

cd "$(dirname "$0")/.."

BASE_INPUT="${BASE_INPUT:-/mnt/data/checkpoints/usv_rl/fresh415_usv03_narrow_signed_cte_late.pt}" \
OUTPUT="${OUTPUT:-/mnt/data/checkpoints/usv_rl/fresh423_progress_retain_random_clear.pt}" \
CKPT_DIR="${CKPT_DIR:-/mnt/data/checkpoints/usv_rl/fresh423_checkpoints}" \
TOTAL_TIMESTEPS="${TOTAL_TIMESTEPS:-2700}" \
BASE_ROS_DOMAIN_ID="${BASE_ROS_DOMAIN_ID:-221}" \
exec bash scripts/train_fresh403_low_strength_late_lagging.sh \
  --curriculum-seed 1461 \
  --curriculum-seed 1463 \
  --random-cte-recovery-weight 0.0 \
  --random-cte-recovery-weight-end 0.0 \
  --random-offroute-finish-weight 0.0 \
  --random-offroute-finish-weight-end 0.0 \
  --random-late-lagging-weight 0.0 \
  --random-late-lagging-weight-end 0.0 \
  --random-clear-ahead-scenario three_usv_random_encounter \
  --random-clear-ahead-weight 0.06 \
  --random-clear-ahead-weight-end 0.12 \
  --random-clear-ahead-distance 6.0 \
  --random-clear-ahead-bearing-deg 32.0 \
  --random-clear-ahead-cone-mode goal \
  --random-clear-ahead-max-distance 25.0 \
  --random-clear-ahead-min-neighbor-separation 2.0 \
  --random-clear-ahead-max-cpa-score 0.0 \
  --random-clear-ahead-max-local-score 0.0 \
  --random-clear-ahead-exclude-deconflict \
  --random-clear-ahead-target-source raw \
  --random-clear-ahead-target-speed 0.22 \
  --random-clear-ahead-min-speed 0.10 \
  --random-clear-ahead-max-omega 0.10 \
  --random-clear-ahead-omega-weight 0.20 \
  --policy-anchor-weight 3500.0 \
  --policy-anchor-weight-end 5000.0 \
  --policy-anchor-exclude-random-clear-ahead \
  "$@"