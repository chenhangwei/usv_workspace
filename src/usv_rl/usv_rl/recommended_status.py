import argparse
import json
from pathlib import Path

from .log_health import inspect_benchmark_log, inspect_smoke_log
from .policy_inference_node import _candidate_workspace_roots, _resolve_model_path
from .recommended import (
    RECOMMENDED_BENCHMARK_LOG_DIR_RELATIVE_PATH,
    RECOMMENDED_BENCHMARK_RELATIVE_PATH,
    RECOMMENDED_GATE_RELATIVE_PATH,
    RECOMMENDED_MODEL_RELATIVE_PATH,
    RECOMMENDED_SMOKE_LOG_RELATIVE_PATH,
    RECOMMENDED_SMOKE_RELATIVE_PATH,
    RECOMMENDED_VALIDATION_BATCH_RELATIVE_PATH,
)
from .scenarios import ScenarioFactory


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Show the current recommended candidate status by summarizing the default model, benchmark, gate, and smoke artifacts.'
    )
    parser.add_argument('--output-json', help='Optional path to save the recommended status summary as JSON.')
    return parser.parse_args(argv)


def _resolve_artifact_path(relative_path: str) -> Path:
    raw_path = Path(relative_path).expanduser()
    candidates: list[Path] = []

    if raw_path.is_absolute():
        candidates.append(raw_path)
    else:
        candidates.append((Path.cwd() / raw_path).resolve())
        for workspace_root in _candidate_workspace_roots():
            candidates.append((workspace_root / raw_path).resolve())

    seen: set[Path] = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        if candidate.exists():
            return candidate

    return candidates[0].resolve()


def _load_optional_json(path: Path):
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding='utf-8'))


def _build_scenario_log_status(log_dir: Path) -> tuple[dict[str, dict], bool]:
    scenario_logs = {}
    all_present = True

    for scenario in ScenarioFactory.available():
        log_path = log_dir / f'{scenario}.log'
        exists = log_path.exists()
        scenario_logs[scenario] = {
            'path': str(log_path),
            'exists': exists,
        }
        all_present = all_present and exists

    return scenario_logs, all_present


def _build_benchmark_log_content_status(log_dir: Path) -> tuple[dict[str, dict], bool]:
    scenario_logs = {}
    all_valid = True

    for scenario in ScenarioFactory.available():
        log_status = inspect_benchmark_log(log_dir / f'{scenario}.log', scenario)
        scenario_logs[scenario] = log_status
        all_valid = all_valid and log_status['content_valid']

    return scenario_logs, all_valid


def _summarize_validation_batch(batch_dir: Path) -> dict:
    if not batch_dir.exists():
        return {
            'path': str(batch_dir),
            'exists': False,
            'runs_total': 0,
            'runs_passed': 0,
            'runs_failed': 0,
            'complete': False,
            'all_passed': False,
            'scenario_counts': {},
            'scenario_retries': {},
        }

    run_dirs = sorted(path for path in batch_dir.glob('run_*') if path.is_dir())
    scenario_counts: dict[str, dict[str, int]] = {
        scenario: {'pass': 0, 'fail': 0} for scenario in ScenarioFactory.available()
    }
    scenario_retries: dict[str, int] = {scenario: 0 for scenario in ScenarioFactory.available()}
    runs_total = 0
    runs_passed = 0
    complete = True

    for run_dir in run_dirs:
        gate_path = run_dir / 'gate.json'
        benchmark_path = run_dir / 'benchmark.json'
        if not gate_path.exists() or not benchmark_path.exists():
            complete = False
            continue

        runs_total += 1
        gate = json.loads(gate_path.read_text(encoding='utf-8'))
        benchmark = json.loads(benchmark_path.read_text(encoding='utf-8'))
        if gate.get('passed'):
            runs_passed += 1

        for check in gate.get('checks', []):
            scenario = check.get('scenario')
            status = check.get('status')
            if scenario in scenario_counts and status in scenario_counts[scenario]:
                scenario_counts[scenario][status] += 1

        for result in benchmark.get('results', []):
            if result.get('retried_for_startup_flake'):
                scenario = result.get('scenario')
                if scenario in scenario_retries:
                    scenario_retries[scenario] += 1

    runs_failed = runs_total - runs_passed
    return {
        'path': str(batch_dir),
        'exists': True,
        'runs_total': runs_total,
        'runs_passed': runs_passed,
        'runs_failed': runs_failed,
        'complete': complete,
        'all_passed': runs_total > 0 and runs_failed == 0,
        'scenario_counts': scenario_counts,
        'scenario_retries': scenario_retries,
    }


def main(argv=None):
    args = parse_args(argv)

    model_path = Path(_resolve_model_path(RECOMMENDED_MODEL_RELATIVE_PATH))
    benchmark_path = _resolve_artifact_path(RECOMMENDED_BENCHMARK_RELATIVE_PATH)
    benchmark_log_dir = _resolve_artifact_path(RECOMMENDED_BENCHMARK_LOG_DIR_RELATIVE_PATH)
    gate_path = _resolve_artifact_path(RECOMMENDED_GATE_RELATIVE_PATH)
    smoke_path = _resolve_artifact_path(RECOMMENDED_SMOKE_RELATIVE_PATH)
    smoke_log_path = _resolve_artifact_path(RECOMMENDED_SMOKE_LOG_RELATIVE_PATH)
    validation_batch_path = _resolve_artifact_path(RECOMMENDED_VALIDATION_BATCH_RELATIVE_PATH)

    benchmark = _load_optional_json(benchmark_path)
    gate = _load_optional_json(gate_path)
    smoke = _load_optional_json(smoke_path)
    benchmark_scenario_logs, benchmark_scenario_logs_complete = _build_scenario_log_status(benchmark_log_dir)
    benchmark_scenario_log_health, benchmark_scenario_log_health_complete = _build_benchmark_log_content_status(benchmark_log_dir)
    smoke_log_health = inspect_smoke_log(smoke_log_path)
    validation_batch_summary = _summarize_validation_batch(validation_batch_path)

    summary = {
        'recommended_model': str(model_path),
        'recommended_model_exists': model_path.exists(),
        'recommended_benchmark': str(benchmark_path),
        'recommended_benchmark_exists': benchmark_path.exists(),
        'recommended_benchmark_log_dir': str(benchmark_log_dir),
        'recommended_benchmark_log_dir_exists': benchmark_log_dir.exists(),
        'recommended_benchmark_scenario_logs': benchmark_scenario_logs,
        'recommended_benchmark_scenario_logs_complete': benchmark_scenario_logs_complete,
        'recommended_benchmark_scenario_log_health': benchmark_scenario_log_health,
        'recommended_benchmark_scenario_log_health_complete': benchmark_scenario_log_health_complete,
        'recommended_gate': str(gate_path),
        'recommended_gate_exists': gate_path.exists(),
        'recommended_smoke': str(smoke_path),
        'recommended_smoke_exists': smoke_path.exists(),
        'recommended_smoke_log': str(smoke_log_path),
        'recommended_smoke_log_exists': smoke_log_path.exists(),
        'recommended_smoke_log_health': smoke_log_health,
        'recommended_validation_batch_dir': str(validation_batch_path),
        'recommended_validation_batch_exists': validation_batch_path.exists(),
        'recommended_validation_batch_summary': validation_batch_summary,
        'gate_passed': None if gate is None else gate.get('passed'),
        'smoke_passed': None if smoke is None else smoke.get('passed'),
        'head_on_trend': None,
        'crossing_starboard_trend': None,
        'overtaking_trend': None,
    }

    if benchmark is not None:
        results = {item['scenario']: item for item in benchmark.get('results', [])}
        summary['head_on_trend'] = None if 'head_on' not in results else results['head_on'].get('trend')
        summary['crossing_starboard_trend'] = None if 'crossing_starboard' not in results else results['crossing_starboard'].get('trend')
        summary['overtaking_trend'] = None if 'overtaking' not in results else results['overtaking'].get('trend')

    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved recommended status to {output_path}')


if __name__ == '__main__':
    main()