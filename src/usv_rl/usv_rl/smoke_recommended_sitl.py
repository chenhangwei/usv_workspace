import argparse
import json
import os
import signal
import subprocess
import sys
from pathlib import Path

from .online_launch_parsing import has_unexpected_traceback
from .policy_inference_node import _resolve_model_path
from .recommended import RECOMMENDED_MODEL_RELATIVE_PATH, RECOMMENDED_SMOKE_LOG_RELATIVE_PATH


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Smoke test the recommended SITL deployment launch and summarize whether the policy node came up cleanly.'
    )
    parser.add_argument('--namespace', default='usv_03', help='USV namespace for the recommended launch.')
    parser.add_argument('--timeout-seconds', type=float, default=20.0, help='How long to let the launch run before sending SIGINT.')
    parser.add_argument('--model', default=RECOMMENDED_MODEL_RELATIVE_PATH, help='Recommended model path override.')
    parser.add_argument('--policy', default='bc', choices=['auto', 'bc', 'ppo', 'mappo', 'zero'], help='Policy type for the recommended launch.')
    parser.add_argument('--output-json', help='Optional path to save the smoke summary as JSON.')
    parser.add_argument('--output-log', default=RECOMMENDED_SMOKE_LOG_RELATIVE_PATH, help='Path to save the raw launch output for this smoke run.')
    return parser.parse_args(argv)


def _parse_launch_output(output: str) -> dict:
    policy_process_started = 'policy_inference_node' in output
    controller_param_enabled = 'Enabled rl_policy_enabled on velocity_controller_node.' in output
    observation_ready = 'Observation stream ready; pure RL policy inference is active.' in output
    stale_data_warning = '数据过期 - Pose' in output or 'Pose/State' in output
    model_missing = 'Model file not found' in output
    unexpected_error = has_unexpected_traceback(output) or model_missing

    return {
        'policy_process_started': policy_process_started,
        'controller_param_enabled': controller_param_enabled,
        'observation_ready': observation_ready,
        'stale_data_warning': stale_data_warning,
        'error_detected': unexpected_error,
    }


def main(argv=None):
    args = parse_args(argv)
    resolved_model = _resolve_model_path(args.model)
    command = [
        'ros2',
        'launch',
        'usv_rl',
        'recommended_sitl.launch.py',
        f'namespace:={args.namespace}',
        f'rl_policy_model:={resolved_model}',
        f'rl_policy_kind:={args.policy}',
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
        stdout, _ = process.communicate(timeout=args.timeout_seconds)
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(process.pid, signal.SIGINT)
        try:
            stdout, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            stdout, _ = process.communicate()

    summary = _parse_launch_output(stdout)
    summary.update(
        {
            'namespace': args.namespace,
            'model': resolved_model,
            'policy': args.policy,
            'timeout_seconds': args.timeout_seconds,
            'timed_out': timed_out,
            'return_code': process.returncode,
        }
    )
    summary['passed'] = (
        summary['policy_process_started']
        and summary['controller_param_enabled']
        and not summary['error_detected']
    )

    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if args.output_log:
        log_path = Path(args.output_log)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(stdout, encoding='utf-8')
        print(f'Saved smoke log to {log_path}')

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved smoke summary to {output_path}')

    raise SystemExit(0 if summary['passed'] else 1)


if __name__ == '__main__':
    main(sys.argv[1:])