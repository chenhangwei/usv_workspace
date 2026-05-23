#!/bin/bash
# Focused long-timeout evaluation for hard random encounter seeds.

set -eo pipefail

cd "$(dirname "$0")/.."
source ../install/setup.bash
export PYTHONPATH="$PWD/usv_rl:${PYTHONPATH:-}"

MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh313_single_overtake_early_return_from_fresh310.pt}"
POLICY="${POLICY:-auto}"
OUT="${OUT:-/tmp/random_hard_focus_eval}"
LABEL="${LABEL:-random_hard_focus}"
SEEDS="${SEEDS:-1458 1460 1461 1463}"
STEPS="${STEPS:-1200}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-240.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-220.0}"
BASE_DOMAIN="${BASE_DOMAIN:-210}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-1800}"
TRACE_STRIDE="${TRACE_STRIDE:-0}"
TRACE_EVENT_SEPARATION="${TRACE_EVENT_SEPARATION:-0}"
TRACE_EVENT_WINDOW="${TRACE_EVENT_WINDOW:-12}"
TRACE_EVENT_RAW_OBSERVATION="${TRACE_EVENT_RAW_OBSERVATION:-0}"
TRACE_COLLISION_RAW_OBSERVATION="${TRACE_COLLISION_RAW_OBSERVATION:-0}"

mkdir -p "$OUT"
SUMMARY="$OUT/summary.tsv"
DIAGNOSTICS="$OUT/diagnostics.tsv"
printf 'seed\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tomega\tsteps\tjson\n' > "$SUMMARY"
printf 'seed\tstep_kind\tstep\tpair_sep\tagent\tpriority\tdistance\troute_progress\tcte\tlinear\tomega\tnearest\tnearest_distance\trel_x\trel_y\tdeconf_active\tdeconf_yield\tdeconf_target_linear\tdeconf_target_omega\tdeconf_linear_error\tdeconf_omega_error\tjson\n' > "$DIAGNOSTICS"

if [[ ! -f "$MODEL" ]]; then
  echo "missing model: $MODEL" >&2
  exit 1
fi

index=0
for seed in $SEEDS; do
  domain=$((BASE_DOMAIN + index))
  json="$OUT/seed${seed}.json"
  log="$OUT/seed${seed}.log"
  echo "----- ${LABEL} random seed=${seed} domain=${domain} -----"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  trace_args=()
  if [[ "$TRACE_STRIDE" != "0" ]]; then
    trace_args+=(--trace-stride "$TRACE_STRIDE" --trace-raw-observation)
  fi
  if [[ "$TRACE_EVENT_SEPARATION" != "0" ]]; then
    trace_args+=(--trace-event-separation "$TRACE_EVENT_SEPARATION" --trace-event-window "$TRACE_EVENT_WINDOW")
    if [[ "$TRACE_EVENT_RAW_OBSERVATION" == "1" ]]; then
      trace_args+=(--trace-event-raw-observation)
    fi
  fi
  if [[ "$TRACE_COLLISION_RAW_OBSERVATION" == "1" ]]; then
    trace_args+=(--trace-collision-raw-observation)
  fi
  ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
    --policy "$POLICY" \
    --model "$MODEL" \
    --episodes 1 \
    --steps-per-episode "$STEPS" \
    --device cpu \
    --episode-timeout "$EPISODE_TIMEOUT" \
    --no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
    --seed "$seed" \
    --scenario three_usv_random_encounter \
    --output-json "$json" \
    "${trace_args[@]}" \
    > "$log" 2>&1

  /bin/python3 - "$json" "$seed" "$SUMMARY" "$DIAGNOSTICS" <<'PY'
import json
import sys

json_path, seed, summary_path, diagnostics_path = sys.argv[1:5]
with open(json_path, 'r', encoding='utf-8') as handle:
    data = json.load(handle)
episode = data['episode_metrics'][0]
row = [
    seed,
    f"{float(data.get('collision_rate', 0.0)):.6f}",
    f"{float(data.get('success_rate', 0.0)):.6f}",
    f"{float(data.get('timeout_rate', 0.0)):.6f}",
    f"{float(episode.get('team_goal_progress_ratio', 0.0)):.6f}",
    f"{float(episode.get('goal_completion_ratio', 0.0)):.6f}",
    f"{float(episode.get('episode_min_separation', 0.0)):.6f}",
    f"{float(episode.get('mean_cross_track_error', 0.0)):.6f}",
    f"{float(episode.get('omega_flip_count', 0.0)):.6f}",
    f"{float(episode.get('steps', 0.0)):.6f}",
    json_path,
]
with open(summary_path, 'a', encoding='utf-8') as handle:
    handle.write('\t'.join(row) + '\n')

samples = episode.get('trace_samples') or []
if samples:
    min_sample = min(samples, key=lambda item: float(item.get('pairwise_min_separation', 1e9)))
    selected = [('min', min_sample), ('last', samples[-1])]
    lines = []
    for kind, sample in selected:
        for agent_id, agent in sorted((sample.get('agents') or {}).items()):
            diagnostics = agent.get('mask_diagnostics') or {}
            deconf = diagnostics.get('deconf_target') or {}
            fields = [
                seed,
                kind,
                str(sample.get('step', '')),
                f"{float(sample.get('pairwise_min_separation', 0.0)):.6f}",
                agent_id,
                f"{float(agent.get('crossing_priority', 0.0)):.6f}",
                f"{float(agent.get('distance_to_goal', 0.0)):.6f}",
                f"{float(agent.get('route_progress', 0.0)):.6f}",
                f"{float(agent.get('cross_track_error', 0.0)):.6f}",
                f"{float(agent.get('final_linear_x', 0.0)):.6f}",
                f"{float(agent.get('final_angular_z', 0.0)):.6f}",
                str(agent.get('nearest_id', '')),
                f"{float(agent.get('nearest_distance', 0.0)):.6f}",
                f"{float(agent.get('nearest_rel_x', 0.0)):.6f}",
                f"{float(agent.get('nearest_rel_y', 0.0)):.6f}",
                str(bool(diagnostics.get('random_deconflict_weighted_active', False))),
                str(bool(diagnostics.get('deconf_is_yield', False))),
                f"{float(deconf.get('target_linear', 0.0)):.6f}",
                f"{float(deconf.get('target_omega', 0.0)):.6f}",
                f"{float(deconf.get('linear_error', 0.0)):.6f}",
                f"{float(deconf.get('omega_error', 0.0)):.6f}",
                json_path,
            ]
            lines.append('\t'.join(fields))
    if lines:
        with open(diagnostics_path, 'a', encoding='utf-8') as handle:
            handle.write('\n'.join(lines) + '\n')
PY
  index=$((index + 1))
done

echo "summary: $SUMMARY"
cat "$SUMMARY"
echo "diagnostics: $DIAGNOSTICS"