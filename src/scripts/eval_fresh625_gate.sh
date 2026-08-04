#!/bin/bash
# fresh625 offline gate（辅判据）。重建自 /tmp/diag_624/run_eval.sh（断电丢失），放持久位置。
# 模型：fresh622（consolidation 备选祖先）/ fresh624（上一代）/ fresh625（本轮，最终 .pt 缺失时自动用最新 ckpt）。
# 场景：cluster_escape / two_usv_head_on / three_usv_crossing / three_usv_overtaking，seeds 5241-5243。
# Arbiter（2026-07-08 用户优先级）：碰撞只报告不硬卡；关注 cluster reach 9/9、crossing 成功、
# CTE / entanglement / omega 饱和（贴线直线指标）、无缠死堆叠。
# ROS domain 从 100 起自增，避开训练的 294。
set -o pipefail
cd /home/chenhangwei/usv_workspace/src
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

MODELS_fresh622=/mnt/data/checkpoints/usv_rl/fresh622_balanced_no_entangle_3usv.pt
MODELS_fresh624=/mnt/data/checkpoints/usv_rl/fresh624_dense_transit_route_hold_3usv.pt
MODELS_fresh625=/mnt/data/checkpoints/usv_rl/fresh625_route_first_straight_3usv.pt
if [[ ! -f "$MODELS_fresh625" ]]; then
  MODELS_fresh625="$(ls -1t /mnt/data/checkpoints/usv_rl/fresh625_checkpoints/*.pt 2>/dev/null | head -1)"
  echo "fresh625 final model missing; using latest ckpt: $MODELS_fresh625"
fi

SEEDS="${SEEDS:-5241 5242 5243}"
SCENARIOS="${SCENARIOS:-cluster_escape two_usv_head_on three_usv_crossing three_usv_overtaking}"
MODEL_NAMES="${MODEL_NAMES:-fresh622 fresh624 fresh625}"
OUT="${OUT:-/home/chenhangwei/usv_workspace/src/logs/eval_fresh625_gate}"
mkdir -p "$OUT"
dom="${BASE_DOMAIN:-100}"

for name in $MODEL_NAMES; do
  eval "model=\$MODELS_${name}"
  if [[ -z "$model" || ! -f "$model" ]]; then
    echo "SKIP $name (model missing: $model)"
    continue
  fi
  for scen in $SCENARIOS; do
    for s in $SEEDS; do
      tag="${name}_${scen}_s${s}"
      if [[ -s "$OUT/${tag}.json" ]]; then
        echo "===== $tag exists, skip ====="
        dom=$((dom+1))
        continue
      fi
      echo "===== $tag domain=$dom ====="
      rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
      ROS_DOMAIN_ID=$dom timeout 900 /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
        --policy mappo --model "$model" \
        --num-agents 3 --episodes 1 --steps-per-episode 600 \
        --device cpu --episode-timeout 110.0 --no-progress-timeout 60.0 \
        --seed "$s" --scenario "$scen" \
        --trace-stride 10 \
        --output-json "$OUT/${tag}.json" \
        > "$OUT/${tag}.log" 2>&1 || echo "  (nonzero/timeout)"
      dom=$((dom+1))
    done
  done
done
echo "ALL_DONE"
