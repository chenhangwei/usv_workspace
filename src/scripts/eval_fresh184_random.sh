#!/bin/bash
# Two-seed random-only screening for fresh184 constrained active-slice fits.

set -eo pipefail

cleanup_children() {
  pkill -P $$ 2>/dev/null || true
}
trap cleanup_children EXIT INT TERM

cd "$(dirname "$0")/.."
source ../install/setup.bash

OUT="${OUT:-/tmp/fresh184_random_eval}"
SUMMARY="$OUT/summary.tsv"
MODEL="${MODEL:-/mnt/data/checkpoints/usv_rl/fresh184_checkpoints/fresh184_constrained_deconf_from_fresh181_step1260.pt}"
LABEL="${LABEL:-fresh184_constrained_deconf}"
SEEDS="${SEEDS:-1460 1462}"
STEPS="${STEPS:-420}"
EPISODE_TIMEOUT="${EPISODE_TIMEOUT:-145.0}"
NO_PROGRESS_TIMEOUT="${NO_PROGRESS_TIMEOUT:-108.0}"
EVAL_TIMEOUT="${EVAL_TIMEOUT:-900}"
BASE_DOMAIN="${BASE_DOMAIN:-192}"

mkdir -p "$OUT"
printf 'label\tseed\tcollision\tsuccess\ttimeout\tprogress\tgoal\tsep\tcte\tentanglement\tcolregs\tomega\tsteps\tmodel\n' > "$SUMMARY"

index=0
for seed in $SEEDS; do
  domain=$((BASE_DOMAIN + index))
  index=$((index + 1))
  json="$OUT/${LABEL}_seed${seed}.json"
  log="$OUT/${LABEL}_seed${seed}.log"
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  printf 'eval-start label=%s seed=%s domain=%s\n' "$LABEL" "$seed" "$domain"
  if ! ROS_DOMAIN_ID="$domain" timeout "$EVAL_TIMEOUT" /bin/python3 -u -m usv_rl.evaluate_mappo_policy \
	--policy mappo \
	--model "$MODEL" \
	--episodes 1 \
	--steps-per-episode "$STEPS" \
	--device cpu \
	--episode-timeout "$EPISODE_TIMEOUT" \
	--no-progress-timeout "$NO_PROGRESS_TIMEOUT" \
	--seed "$seed" \
	--output-json "$json" \
	--scenario three_usv_random_encounter \
	> "$log" 2>&1; then
	printf '%s\t%s\tERROR\tERROR\tERROR\t0\t0\t0\t0\t0\t0\t0\t0\t%s\n' "$LABEL" "$seed" "$MODEL" >> "$SUMMARY"
	tail -40 "$log"
	continue
  fi
  printf 'eval-done label=%s seed=%s\n' "$LABEL" "$seed"

  /bin/python3 - "$json" "$LABEL" "$seed" "$MODEL" "$SUMMARY" <<'PY'
import json
import sys

json_path, label, seed, model, summary_path = sys.argv[1:6]
with open(json_path, 'r', encoding='utf-8') as handle:
	data = json.load(handle)
episode = data['episode_metrics'][0]
values = [
	float(data.get('collision_rate', 0.0)),
	float(data.get('success_rate', 0.0)),
	float(data.get('timeout_rate', 0.0)),
	float(episode.get('team_goal_progress_ratio', 0.0)),
	float(episode.get('goal_completion_ratio', 0.0)),
	float(episode.get('episode_min_separation', 0.0)),
	float(episode.get('mean_cross_track_error', 0.0)),
	float(episode.get('entanglement_ratio', 0.0)),
	float(episode.get('colregs_violation_ratio', 0.0)),
	float(episode.get('omega_flip_count', 0.0)),
	float(episode.get('steps', 0.0)),
]
agent_bits = []
for agent_id, metrics in sorted(episode.get('final_agent_metrics', {}).items()):
	agent_bits.append(
		f"{agent_id}:d={float(metrics.get('distance_to_goal', 0.0)):.2f},"
		f"rp={float(metrics.get('route_progress', 0.0)):.2f},"
		f"cte={float(metrics.get('cross_track_error', 0.0)):.2f},"
		f"vx={float(metrics.get('final_linear_x', 0.0)):.2f}"
	)
row = '\t'.join([label, seed] + [f'{value:.6f}' for value in values] + [model])
with open(summary_path, 'a', encoding='utf-8') as handle:
	handle.write(row + '\n')
print(row, flush=True)
print(
	f"fresh184-random {label} seed={seed}: coll={values[0]:.3f} succ={values[1]:.3f} "
	f"timeout={values[2]:.3f} progress={values[3]:.3f} goal={values[4]:.3f} "
	f"sep={values[5]:.3f} cte={values[6]:.3f} ent={values[7]:.3f} "
	f"colregs={values[8]:.3f} omega={values[9]:.1f} steps={values[10]:.1f}",
	flush=True,
)
if agent_bits:
	print('final_agents ' + ' | '.join(agent_bits), flush=True)
PY
done

printf '========== fresh184 random summary =========='"\n"
column -t -s $'\t' "$SUMMARY"