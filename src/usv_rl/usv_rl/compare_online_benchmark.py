import argparse
import json
import sys
from pathlib import Path

from .log_health import inspect_benchmark_log


REQUIRED_TRENDS = {
    'head_on': 'retreat',
    'crossing_starboard': 'progress',
    'overtaking': 'progress',
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Compare an online benchmark JSON against the current acceptance gate for residual avoidance policies.'
    )
    parser.add_argument('--candidate', required=True, help='Candidate benchmark JSON path.')
    parser.add_argument('--baseline', help='Optional baseline benchmark JSON path for side-by-side reporting.')
    parser.add_argument('--output-json', help='Optional path to save the comparison report.')
    return parser.parse_args(argv)


def _load_json(path_str: str) -> dict:
    return json.loads(Path(path_str).read_text(encoding='utf-8'))


def _results_by_scenario(summary: dict) -> dict:
    return {result['scenario']: result for result in summary.get('results', [])}


def _resolve_result_log_path(summary: dict, result: dict | None, scenario: str) -> Path | None:
    if result is not None and result.get('raw_log_path'):
        return Path(result['raw_log_path'])

    log_dir = summary.get('log_dir')
    if log_dir:
        return Path(log_dir) / f'{scenario}.log'

    return None


def _log_diagnostics(log_health: dict) -> list[str]:
    if not log_health.get('exists', False):
        return ['raw_log_missing']

    diagnostics = []
    if not log_health.get('scenario_started', False):
        diagnostics.append('raw_log_missing_scenario_started')
    if not log_health.get('observation_ready', False):
        diagnostics.append('raw_log_missing_observation_ready')
    if not log_health.get('first_residual_action_logged', False):
        diagnostics.append('raw_log_missing_first_residual_action')
    if log_health.get('unexpected_traceback', False):
        diagnostics.append('raw_log_unexpected_traceback')
    return diagnostics


def _build_log_health_summary(summary: dict) -> tuple[dict[str, dict], bool]:
    results = _results_by_scenario(summary)
    health_by_scenario = {}
    all_valid = True

    for scenario in REQUIRED_TRENDS:
        result = results.get(scenario)
        health = inspect_benchmark_log(_resolve_result_log_path(summary, result, scenario), scenario)
        health_by_scenario[scenario] = health
        all_valid = all_valid and health.get('content_valid', False)

    return health_by_scenario, all_valid


def _evaluate_candidate(candidate: dict) -> tuple[list[dict], bool]:
    candidate_results = _results_by_scenario(candidate)
    checks = []

    for scenario, required_trend in REQUIRED_TRENDS.items():
        result = candidate_results.get(scenario)
        log_health = inspect_benchmark_log(_resolve_result_log_path(candidate, result, scenario), scenario)
        log_diagnostics = _log_diagnostics(log_health)
        if result is None:
            checks.append(
                {
                    'scenario': scenario,
                    'status': 'fail',
                    'reason': 'missing_scenario',
                    'required_trend': required_trend,
                    'raw_log_path': log_health.get('path'),
                    'raw_log_health': log_health,
                    'raw_log_diagnostics': log_diagnostics,
                }
            )
            continue

        failures = []
        if not result.get('observation_ready', False):
            failures.append('observation_not_ready')
        if result.get('error_detected', True):
            failures.append('runtime_error_detected')
        if result.get('trend') != required_trend:
            failures.append('trend_mismatch')

        checks.append(
            {
                'scenario': scenario,
                'status': 'pass' if not failures else 'fail',
                'reason': 'ok' if not failures else ','.join(failures),
                'required_trend': required_trend,
                'actual_trend': result.get('trend'),
                'observation_ready': result.get('observation_ready'),
                'error_detected': result.get('error_detected'),
                'distance_delta': result.get('distance_delta'),
                'first_residual_action': result.get('first_residual_action'),
                'raw_log_path': result.get('raw_log_path'),
                'raw_log_health': log_health,
                'raw_log_diagnostics': log_diagnostics,
            }
        )

    passed = all(check['status'] == 'pass' for check in checks)
    return checks, passed


def _build_baseline_comparison(candidate: dict, baseline: dict | None) -> list[dict]:
    if baseline is None:
        return []

    candidate_results = _results_by_scenario(candidate)
    baseline_results = _results_by_scenario(baseline)
    comparisons = []

    for scenario in REQUIRED_TRENDS:
        candidate_result = candidate_results.get(scenario)
        baseline_result = baseline_results.get(scenario)
        candidate_log_health = inspect_benchmark_log(_resolve_result_log_path(candidate, candidate_result, scenario), scenario)
        baseline_log_health = inspect_benchmark_log(_resolve_result_log_path(baseline, baseline_result, scenario), scenario)
        comparisons.append(
            {
                'scenario': scenario,
                'candidate_trend': None if candidate_result is None else candidate_result.get('trend'),
                'baseline_trend': None if baseline_result is None else baseline_result.get('trend'),
                'candidate_distance_delta': None if candidate_result is None else candidate_result.get('distance_delta'),
                'baseline_distance_delta': None if baseline_result is None else baseline_result.get('distance_delta'),
                'candidate_first_residual_action': None if candidate_result is None else candidate_result.get('first_residual_action'),
                'baseline_first_residual_action': None if baseline_result is None else baseline_result.get('first_residual_action'),
                'candidate_raw_log_path': None if candidate_result is None else candidate_result.get('raw_log_path'),
                'baseline_raw_log_path': None if baseline_result is None else baseline_result.get('raw_log_path'),
                'candidate_raw_log_health': candidate_log_health,
                'baseline_raw_log_health': baseline_log_health,
                'candidate_raw_log_diagnostics': _log_diagnostics(candidate_log_health),
                'baseline_raw_log_diagnostics': _log_diagnostics(baseline_log_health),
            }
        )

    return comparisons


def main(argv=None):
    args = parse_args(argv)
    candidate = _load_json(args.candidate)
    baseline = _load_json(args.baseline) if args.baseline else None

    checks, passed = _evaluate_candidate(candidate)
    comparison = _build_baseline_comparison(candidate, baseline)
    candidate_log_health, candidate_log_health_complete = _build_log_health_summary(candidate)
    baseline_log_health, baseline_log_health_complete = (None, None) if baseline is None else _build_log_health_summary(baseline)

    report = {
        'candidate': str(Path(args.candidate).resolve()),
        'baseline': None if not args.baseline else str(Path(args.baseline).resolve()),
        'candidate_log_dir': candidate.get('log_dir'),
        'baseline_log_dir': None if baseline is None else baseline.get('log_dir'),
        'candidate_log_health': candidate_log_health,
        'candidate_log_health_complete': candidate_log_health_complete,
        'baseline_log_health': baseline_log_health,
        'baseline_log_health_complete': baseline_log_health_complete,
        'passed': passed,
        'required_trends': REQUIRED_TRENDS,
        'checks': checks,
        'comparison_to_baseline': comparison,
    }

    print(json.dumps(report, ensure_ascii=False, indent=2))

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved comparison report to {output_path}')

    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main(sys.argv[1:])