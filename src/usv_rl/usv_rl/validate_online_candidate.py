import argparse
import json
import sys
import time
from pathlib import Path

from .benchmark_online_policy import run_scenario_with_startup_retry
from .compare_online_benchmark import _build_baseline_comparison, _evaluate_candidate
from .policy_inference_node import _resolve_model_path
from .recommended import RECOMMENDED_BENCHMARK_RELATIVE_PATH
from .scenarios import ScenarioFactory


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run the standard online benchmark for a candidate policy and immediately apply the current replacement gate against the v2 baseline.'
    )
    parser.add_argument('--model', required=True, help='Candidate residual policy model path (.npz or .zip).')
    parser.add_argument('--policy', choices=['auto', 'bc', 'ppo', 'zero'], default='auto', help='Policy backend.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--goal-x', type=float, default=8.0, help='Validation goal X in meters.')
    parser.add_argument('--goal-y', type=float, default=0.0, help='Validation goal Y in meters.')
    parser.add_argument('--timeout-seconds', type=float, default=20.0, help='Per-scenario timeout in seconds.')
    parser.add_argument('--cooldown-seconds', type=float, default=2.0, help='Idle delay inserted between scenarios.')
    parser.add_argument('--startup-retries', type=int, default=1, help='How many times to retry a scenario when the failure looks like a startup readiness flake rather than a behavior regression.')
    parser.add_argument('--baseline-benchmark', default=RECOMMENDED_BENCHMARK_RELATIVE_PATH, help='Baseline benchmark JSON used as the replacement gate reference.')
    parser.add_argument('--benchmark-output', help='Optional path to save the candidate benchmark JSON.')
    parser.add_argument('--gate-output', help='Optional path to save the gate report JSON.')
    parser.add_argument('--log-dir', help='Optional directory to save raw launch output logs for each validation scenario.')
    return parser.parse_args(argv)


def _default_benchmark_output(model_path: str) -> Path:
    model_name = Path(model_path).stem
    return Path('./tmp_rl') / f'{model_name}_benchmark.json'


def _default_gate_output(model_path: str) -> Path:
    model_name = Path(model_path).stem
    return Path('./tmp_rl') / f'{model_name}_gate.json'


def _default_log_dir(model_path: str) -> Path:
    model_name = Path(model_path).stem
    return Path('./tmp_rl') / f'{model_name}_validation_logs'


def main(argv=None):
    args = parse_args(argv)
    model_path = _resolve_model_path(args.model)
    baseline_path = Path(args.baseline_benchmark).expanduser().resolve()
    if not baseline_path.exists():
        raise FileNotFoundError(
            f'Baseline benchmark JSON not found: {baseline_path}. Generate it first with benchmark_online_policy on the current recommended model.'
        )

    scenarios = ScenarioFactory.available()
    log_dir = Path(args.log_dir) if args.log_dir else _default_log_dir(model_path)
    candidate_results = []
    for index, scenario in enumerate(scenarios):
        candidate_results.append(
            run_scenario_with_startup_retry(
                model_path=model_path,
                policy_kind=args.policy,
                namespace=args.namespace,
                scenario=scenario,
                goal_x=args.goal_x,
                goal_y=args.goal_y,
                timeout_seconds=args.timeout_seconds,
                raw_log_output_path=log_dir / f'{scenario}.log',
                startup_retries=max(0, args.startup_retries),
                retry_backoff_seconds=args.cooldown_seconds,
            )
        )
        if index != len(scenarios) - 1 and args.cooldown_seconds > 0.0:
            time.sleep(args.cooldown_seconds)

    candidate_summary = {
        'model': model_path,
        'policy': args.policy,
        'namespace': args.namespace,
        'goal': {'x': args.goal_x, 'y': args.goal_y},
        'log_dir': str(log_dir.resolve()),
        'results': candidate_results,
    }

    benchmark_output = Path(args.benchmark_output) if args.benchmark_output else _default_benchmark_output(model_path)
    benchmark_output.parent.mkdir(parents=True, exist_ok=True)
    benchmark_output.write_text(json.dumps(candidate_summary, ensure_ascii=False, indent=2), encoding='utf-8')

    baseline_summary = json.loads(baseline_path.read_text(encoding='utf-8'))
    checks, passed = _evaluate_candidate(candidate_summary)
    gate_report = {
        'candidate': str(benchmark_output.resolve()),
        'baseline': str(baseline_path),
        'passed': passed,
        'required_trends': {
            'head_on': 'retreat',
            'crossing_starboard': 'progress',
            'overtaking': 'progress',
        },
        'checks': checks,
        'comparison_to_baseline': _build_baseline_comparison(candidate_summary, baseline_summary),
    }

    gate_output = Path(args.gate_output) if args.gate_output else _default_gate_output(model_path)
    gate_output.parent.mkdir(parents=True, exist_ok=True)
    gate_output.write_text(json.dumps(gate_report, ensure_ascii=False, indent=2), encoding='utf-8')

    print(json.dumps(candidate_summary, ensure_ascii=False, indent=2))
    print(f'Saved benchmark summary to {benchmark_output}')
    print(json.dumps(gate_report, ensure_ascii=False, indent=2))
    print(f'Saved gate report to {gate_output}')

    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main(sys.argv[1:])