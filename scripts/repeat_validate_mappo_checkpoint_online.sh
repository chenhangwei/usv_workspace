#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: repeat_validate_mappo_checkpoint_online.sh <checkpoint.pt> <baseline_benchmark.json> [--runs N] [--cooldown-between-runs seconds] [-- extra validate args...]" >&2
  exit 2
fi

WORKSPACE_DIR="/mnt/workspace/usv_workspace"
CHECKPOINT_PATH="$1"
BASELINE_BENCHMARK_PATH="$2"
shift 2 || true

RUNS="${ONLINE_REPEAT_RUNS:-3}"
COOLDOWN_BETWEEN_RUNS="${ONLINE_REPEAT_COOLDOWN_SECONDS:-5}"
EXTRA_VALIDATE_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runs)
      RUNS="$2"
      shift 2
      ;;
    --cooldown-between-runs)
      COOLDOWN_BETWEEN_RUNS="$2"
      shift 2
      ;;
    --)
      shift
      EXTRA_VALIDATE_ARGS=("$@")
      break
      ;;
    *)
      EXTRA_VALIDATE_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -f "${CHECKPOINT_PATH}" ]]; then
  echo "error: checkpoint not found: ${CHECKPOINT_PATH}" >&2
  exit 2
fi

if [[ ! -f "${BASELINE_BENCHMARK_PATH}" ]]; then
  echo "error: baseline benchmark JSON not found: ${BASELINE_BENCHMARK_PATH}" >&2
  exit 2
fi

if ! [[ "${RUNS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "error: --runs must be a positive integer, got: ${RUNS}" >&2
  exit 2
fi

CHECKPOINT_DIR="$(dirname "${CHECKPOINT_PATH}")"
CHECKPOINT_STEM="$(basename "${CHECKPOINT_PATH}" .pt)"
RUN_ROOT="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.online_repeats"
SUMMARY_JSON_PATH="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.online.repeat.summary.json"
SUMMARY_LOG_PATH="${CHECKPOINT_DIR}/${CHECKPOINT_STEM}.online.repeat.log"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

mkdir -p "${RUN_ROOT}"

echo "checkpoint_path=${CHECKPOINT_PATH}" | tee "${SUMMARY_LOG_PATH}"
echo "baseline_benchmark_path=${BASELINE_BENCHMARK_PATH}" | tee -a "${SUMMARY_LOG_PATH}"
echo "runs=${RUNS}" | tee -a "${SUMMARY_LOG_PATH}"
echo "cooldown_between_runs=${COOLDOWN_BETWEEN_RUNS}" | tee -a "${SUMMARY_LOG_PATH}"
echo "run_root=${RUN_ROOT}" | tee -a "${SUMMARY_LOG_PATH}"
echo "summary_json_path=${SUMMARY_JSON_PATH}" | tee -a "${SUMMARY_LOG_PATH}"

for (( run_index=1; run_index<=RUNS; run_index++ )); do
  run_name="$(printf 'run_%02d' "${run_index}")"
  benchmark_path="${RUN_ROOT}/${run_name}.benchmark.json"
  gate_path="${RUN_ROOT}/${run_name}.gate.json"
  log_path="${RUN_ROOT}/${run_name}.validate.log"
  run_log_dir="${RUN_ROOT}/${run_name}_logs"
  exit_code_path="${RUN_ROOT}/${run_name}.exit_code"

  echo "=== ${run_name} ===" | tee -a "${SUMMARY_LOG_PATH}"

  set +e
  stdbuf -oL -eL ros2 run usv_rl validate_online_candidate \
    --model "${CHECKPOINT_PATH}" \
    --policy mappo \
    --baseline-benchmark "${BASELINE_BENCHMARK_PATH}" \
    --benchmark-output "${benchmark_path}" \
    --gate-output "${gate_path}" \
    --log-dir "${run_log_dir}" \
    "${EXTRA_VALIDATE_ARGS[@]}" |& tee "${log_path}"
  status=${PIPESTATUS[0]}
  set -e

  printf '%s\n' "${status}" > "${exit_code_path}"
  echo "${run_name}_exit_code=${status}" | tee -a "${SUMMARY_LOG_PATH}"

  if (( run_index < RUNS )); then
    sleep "${COOLDOWN_BETWEEN_RUNS}"
  fi
done

python3 - "${RUN_ROOT}" "${CHECKPOINT_PATH}" "${BASELINE_BENCHMARK_PATH}" "${SUMMARY_JSON_PATH}" "${RUNS}" <<'PY'
import json
import sys
from collections import Counter
from pathlib import Path

run_root = Path(sys.argv[1])
checkpoint_path = sys.argv[2]
baseline_path = sys.argv[3]
summary_json_path = Path(sys.argv[4])
expected_runs = int(sys.argv[5])

required = {
    'head_on': 'retreat',
    'crossing_starboard': 'progress',
    'overtaking': 'progress',
}

run_entries = []
pass_count = 0
scenario_match_counts = {scenario: 0 for scenario in required}
scenario_trend_counts = {scenario: Counter() for scenario in required}

for run_index in range(1, expected_runs + 1):
    run_name = f'run_{run_index:02d}'
    benchmark_path = run_root / f'{run_name}.benchmark.json'
    gate_path = run_root / f'{run_name}.gate.json'
    exit_code_path = run_root / f'{run_name}.exit_code'

    benchmark = json.loads(benchmark_path.read_text(encoding='utf-8')) if benchmark_path.exists() else None
    gate = json.loads(gate_path.read_text(encoding='utf-8')) if gate_path.exists() else None
    exit_code = int(exit_code_path.read_text(encoding='utf-8').strip()) if exit_code_path.exists() else None

    trends = {}
    gate_checks = {}
    if benchmark is not None:
        for result in benchmark.get('results', []):
            scenario = result.get('scenario')
            if scenario in required:
                trend = result.get('trend')
                trends[scenario] = trend
                scenario_trend_counts[scenario][trend] += 1
                if trend == required[scenario]:
                    scenario_match_counts[scenario] += 1

    if gate is not None:
        for check in gate.get('checks', []):
            gate_checks[check.get('scenario')] = {
                'status': check.get('status'),
                'reason': check.get('reason'),
                'actual_trend': check.get('actual_trend'),
            }
        if gate.get('passed', False):
            pass_count += 1

    run_entries.append(
        {
            'run': run_name,
            'exit_code': exit_code,
            'benchmark_json': str(benchmark_path),
            'gate_json': str(gate_path),
            'passed': None if gate is None else gate.get('passed'),
            'trends': trends,
            'gate_checks': gate_checks,
        }
    )

summary = {
    'checkpoint': checkpoint_path,
    'baseline_benchmark': baseline_path,
    'run_root': str(run_root),
    'runs_requested': expected_runs,
    'runs_completed': len(run_entries),
    'pass_count': pass_count,
    'fail_count': len(run_entries) - pass_count,
    'pass_rate': 0.0 if not run_entries else pass_count / len(run_entries),
    'required_trends': required,
    'scenario_required_trend_match_counts': scenario_match_counts,
    'scenario_required_trend_match_rates': {
        scenario: 0.0 if not run_entries else scenario_match_counts[scenario] / len(run_entries)
        for scenario in required
    },
    'scenario_trend_counts': {
        scenario: dict(counter)
        for scenario, counter in scenario_trend_counts.items()
    },
    'runs': run_entries,
}

summary_json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(summary, ensure_ascii=False, indent=2))
print(f'Saved repeat validation summary to {summary_json_path}')
PY