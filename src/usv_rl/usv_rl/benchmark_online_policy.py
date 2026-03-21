import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

from .log_health import inspect_benchmark_log
from .online_launch_parsing import has_unexpected_traceback
from .policy_inference_node import _resolve_model_path
from .scenarios import ScenarioFactory


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Benchmark a policy in the online simple validation loop across synthetic encounter scenarios.')
    parser.add_argument('--model', required=True, help='Policy model path (.npz, .zip, or .pt).')
    parser.add_argument('--policy', choices=['auto', 'bc', 'ppo', 'mappo', 'zero'], default='auto', help='Policy backend.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Encounter scenario to benchmark. Repeatable.')
    parser.add_argument('--goal-x', type=float, default=8.0, help='Validation goal X in meters.')
    parser.add_argument('--goal-y', type=float, default=0.0, help='Validation goal Y in meters.')
    parser.add_argument('--timeout-seconds', type=float, default=20.0, help='Per-scenario launch timeout in seconds.')
    parser.add_argument('--cooldown-seconds', type=float, default=2.0, help='Idle delay inserted between scenarios to let ROS resources settle.')
    parser.add_argument('--startup-retries', type=int, default=0, help='How many times to retry a scenario when the failure looks like a startup readiness flake. Default 0 keeps benchmark behavior strict.')
    parser.add_argument('--output-json', help='Optional path to save the benchmark summary as JSON.')
    parser.add_argument('--output-log-dir', help='Optional directory to save raw launch output logs for each scenario.')
    return parser.parse_args(argv)


def _scenario_log_path(output_log_dir: str | None, scenario: str) -> Path | None:
    if not output_log_dir:
        return None
    return Path(output_log_dir) / f'{scenario}.log'


def _attempt_log_path(raw_log_output_path: Path | None, attempt: int) -> Path | None:
    if raw_log_output_path is None:
        return None
    return raw_log_output_path.with_name(f'{raw_log_output_path.stem}.attempt_{attempt}{raw_log_output_path.suffix}')

def _parse_launch_output(output: str) -> dict:
    first_action_match = re.search(
        r'Publishing first pure RL action: linear_x=([-0-9.]+), angular_z=([-0-9.]+)',
        output,
    )
    runtime_distance_matches = re.findall(
        r'(?:🚀 (?:纯RL)?导航中:.*?距离=|导航中 \[ID=.*?\]: 距离=)([-0-9.]+)m',
        output,
    )
    distances = [float(value) for value in runtime_distance_matches]
    velocities = [float(value) for value in re.findall(r'vx=([-0-9.]+) m/s', output)]
    observation_ready = 'Observation stream ready; pure RL policy inference is active.' in output
    tracebacks = has_unexpected_traceback(output)

    summary = {
        'observation_ready': observation_ready,
        'error_detected': tracebacks,
        'first_policy_action': None,
        'distance_start': distances[0] if distances else None,
        'distance_end': distances[-1] if distances else None,
        'distance_delta': None,
        'min_distance_observed': min(distances) if distances else None,
        'max_distance_observed': max(distances) if distances else None,
        'min_forward_speed': min(velocities) if velocities else None,
        'max_forward_speed': max(velocities) if velocities else None,
    }

    if first_action_match:
        summary['first_policy_action'] = {
            'linear_x': float(first_action_match.group(1)),
            'angular_z': float(first_action_match.group(2)),
        }

    if summary['distance_start'] is not None and summary['distance_end'] is not None:
        summary['distance_delta'] = summary['distance_end'] - summary['distance_start']

    if summary['distance_delta'] is None:
        summary['trend'] = 'unknown'
    elif summary['distance_delta'] < -0.3:
        summary['trend'] = 'progress'
    elif summary['distance_delta'] > 0.3:
        summary['trend'] = 'retreat'
    else:
        summary['trend'] = 'stalled'

    return summary


def _run_single_scenario(*, model_path: str, policy_kind: str, namespace: str, scenario: str, goal_x: float, goal_y: float, timeout_seconds: float, raw_log_output_path: Path | None = None) -> dict:
    command = [
        'ros2',
        'launch',
        'usv_rl',
        'online_policy_validation.launch.py',
        f'model:={model_path}',
        f'policy:={policy_kind}',
        f'namespace:={namespace}',
        f'goal_x:={goal_x}',
        f'goal_y:={goal_y}',
        'encounter_enabled:=true',
        f'encounter_scenario:={scenario}',
    ]

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    timed_out = False
    try:
        stdout, _ = process.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(process.pid, signal.SIGINT)
        try:
            stdout, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            stdout, _ = process.communicate()

    if raw_log_output_path is not None:
        raw_log_output_path.parent.mkdir(parents=True, exist_ok=True)
        raw_log_output_path.write_text(stdout, encoding='utf-8')

    summary = _parse_launch_output(stdout)
    if process.returncode not in (0, None):
        summary['error_detected'] = True
    summary.update(
        {
            'scenario': scenario,
            'timeout_seconds': timeout_seconds,
            'timed_out': timed_out,
            'return_code': process.returncode,
            'raw_log_path': None if raw_log_output_path is None else str(raw_log_output_path.resolve()),
        }
    )
    return summary


def should_retry_startup_flake(result: dict) -> bool:
    if result.get('error_detected'):
        return False
    if result.get('observation_ready'):
        return False
    if result.get('first_policy_action') is not None:
        return False
    if result.get('trend') not in (None, 'unknown'):
        return False

    raw_log_path = result.get('raw_log_path')
    if not raw_log_path:
        return True

    log_health = inspect_benchmark_log(Path(raw_log_path), result['scenario'])
    return (
        log_health.get('exists', False)
        and log_health.get('scenario_started', False)
        and not log_health.get('observation_ready', False)
        and not log_health.get('first_policy_action_logged', False)
        and not log_health.get('unexpected_traceback', False)
    )


def run_scenario_with_startup_retry(*, model_path: str, policy_kind: str, namespace: str, scenario: str, goal_x: float, goal_y: float, timeout_seconds: float, raw_log_output_path: Path | None, startup_retries: int, retry_backoff_seconds: float) -> dict:
    attempts = 0
    retried_for_startup_flake = False
    attempt_log_paths = []

    while True:
        attempts += 1
        attempt_log_path = _attempt_log_path(raw_log_output_path, attempts)
        result = _run_single_scenario(
            model_path=model_path,
            policy_kind=policy_kind,
            namespace=namespace,
            scenario=scenario,
            goal_x=goal_x,
            goal_y=goal_y,
            timeout_seconds=timeout_seconds,
            raw_log_output_path=attempt_log_path,
        )

        if attempt_log_path is not None:
            attempt_log_paths.append(str(attempt_log_path.resolve()))

        should_retry = attempts <= (startup_retries + 1) and should_retry_startup_flake(result)
        if not should_retry:
            if raw_log_output_path is not None and attempt_log_path is not None and attempt_log_path.exists():
                raw_log_output_path.parent.mkdir(parents=True, exist_ok=True)
                raw_log_output_path.write_text(attempt_log_path.read_text(encoding='utf-8', errors='replace'), encoding='utf-8')
            result['attempt_count'] = attempts
            result['retried_for_startup_flake'] = retried_for_startup_flake
            result['attempt_raw_log_paths'] = attempt_log_paths
            result['raw_log_path'] = None if raw_log_output_path is None else str(raw_log_output_path.resolve())
            return result

        retried_for_startup_flake = True
        time.sleep(max(0.0, retry_backoff_seconds))


def main(argv=None):
    args = parse_args(argv)
    model_path = _resolve_model_path(args.model)
    scenarios = tuple(args.scenarios) if args.scenarios else ScenarioFactory.available()

    results = []
    for scenario in scenarios:
        result = run_scenario_with_startup_retry(
            model_path=model_path,
            policy_kind=args.policy,
            namespace=args.namespace,
            scenario=scenario,
            goal_x=args.goal_x,
            goal_y=args.goal_y,
            timeout_seconds=args.timeout_seconds,
            raw_log_output_path=_scenario_log_path(args.output_log_dir, scenario),
            startup_retries=max(0, args.startup_retries),
            retry_backoff_seconds=args.cooldown_seconds,
        )
        results.append(result)
        if scenario != scenarios[-1] and args.cooldown_seconds > 0.0:
            time.sleep(args.cooldown_seconds)

    summary = {
        'model': model_path,
        'policy': args.policy,
        'namespace': args.namespace,
        'goal': {'x': args.goal_x, 'y': args.goal_y},
        'log_dir': None if not args.output_log_dir else str(Path(args.output_log_dir).resolve()),
        'results': results,
    }

    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved benchmark summary to {output_path}')


if __name__ == '__main__':
    main(sys.argv[1:])