#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/chenhangwei/usv_workspace
cd "$ROOT"

RUN_STEM=/mnt/data/checkpoints/usv_rl/mappo_pure_dense_20260324_v055_headon22_a10base

export ROS_DOMAIN_ID=84
export PYTHONUNBUFFERED=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export MALLOC_ARENA_MAX=1

source /opt/ros/jazzy/setup.bash
source install/setup.bash

set -u

cd src

mkdir -p "${RUN_STEM}_ckpts" "${RUN_STEM}_eval"

# headon22: Fine-tune from a10@358400 — the verified gold-standard checkpoint
# a10@358400: collision=0.000, progress=+0.089 (11/11 zero-collision run)
# Architecture identical: (256,34)→(256,256)→(2,256), all 27 reward params match
# Strategy: ultra-conservative fine-tune to improve progress while preserving
#           zero-collision behavior. a10 already in the right weight-space basin.
# Key settings:
#   LR=5e-6 (ultra-low, 60x below default, minimize policy drift)
#   clip_range=0.05 (very tight trust region, protect zero-collision)
#   rollout_steps=128 (match a10 training config)
#   total_steps=256K (moderate budget, a10 is already converged)
/bin/python3 -m usv_rl.train_mappo_policy \
  --output "${RUN_STEM}.pt" \
  --resume-from /mnt/data/checkpoints/usv_rl/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_step_0307200_resume_20260320_171751_checkpoints/mappo_dense_a10_parallel_headon_stabilize_20260320_003331_step_0307200_resume_20260320_171751_step_0358400.pt \
  --reset-optimizer \
  --total-timesteps 614400 \
  --rollout-steps 128 \
  --update-epochs 3 \
  --minibatch-size 256 \
  --learning-rate 5.0e-6 \
  --clip-range 0.05 \
  --entropy-coef 0.005 \
  --value-coef 0.5 \
  --max-grad-norm 0.5 \
  --device cpu \
  --amp off \
  --num-sampler-workers 1 \
  --torch-num-threads 1 \
  --checkpoint-interval 1600 \
  --checkpoint-dir "${RUN_STEM}_ckpts" \
  --auto-evaluate-checkpoints \
  --checkpoint-eval-episodes 12 \
  --checkpoint-eval-steps 180 \
  --checkpoint-eval-scenario five_usv_dense_head_on \
  --checkpoint-eval-scenario five_usv_dense_crossing \
  --checkpoint-eval-scenario five_usv_dense_overtaking \
  --checkpoint-ranking-json "${RUN_STEM}_ranking.json" \
  --checkpoint-eval-json-dir "${RUN_STEM}_eval" \
  --log-interval-updates 1