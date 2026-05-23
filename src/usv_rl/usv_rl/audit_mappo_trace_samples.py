import argparse
import json
from collections import Counter
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description='Audit MAPPO evaluation trace samples.')
    parser.add_argument('paths', nargs='+', help='JSON files or directories containing evaluation JSON files.')
    parser.add_argument('--risk-min-threat', type=float, default=0.65)
    parser.add_argument('--risk-max-separation', type=float, default=1.60)
    parser.add_argument('--risk-min-route-progress', type=float, default=0.20)
    parser.add_argument('--risk-max-route-progress', type=float, default=0.75)
    parser.add_argument('--risk-min-abs-cte', type=float, default=1.20)
    parser.add_argument('--json', action='store_true', help='Print machine-readable JSON summary.')
    return parser.parse_args()


def _iter_json_paths(paths: list[str]):
    seen = set()
    for value in paths:
        path = Path(value)
        if path.is_file() and path.suffix == '.json':
            resolved = path.resolve()
            if resolved not in seen:
                seen.add(resolved)
                yield path
        elif path.is_dir():
            for json_path in sorted(path.rglob('*.json')):
                resolved = json_path.resolve()
                if resolved not in seen:
                    seen.add(resolved)
                    yield json_path


def _progress_bucket(route_progress: float) -> str:
    if route_progress < 0.20:
        return 'p00_20'
    if route_progress < 0.50:
        return 'p20_50'
    if route_progress < 0.65:
        return 'p50_65'
    if route_progress <= 0.75:
        return 'p65_75'
    if route_progress < 0.90:
        return 'p75_90'
    return 'p90_110'


def _agent_record(sample: dict, agent_id: str, agent: dict, args) -> dict:
    diagnostics = agent.get('mask_diagnostics') or {}
    route_progress = float(agent.get('route_progress', 0.0))
    abs_cte = abs(float(agent.get('cross_track_error', 0.0)))
    threat_score = float(diagnostics.get('threat_score', 0.0))
    team_min_separation = float(diagnostics.get('team_min_separation', sample.get('pairwise_min_separation', 0.0)))
    is_yield = bool(diagnostics.get('deconf_is_yield', False))
    risk_active = team_min_separation <= float(args.risk_max_separation) or threat_score >= float(args.risk_min_threat)
    in_window = (
        route_progress >= float(args.risk_min_route_progress)
        and route_progress <= float(args.risk_max_route_progress)
        and abs_cte >= float(args.risk_min_abs_cte)
        and risk_active
    )
    return {
        'agent_id': str(agent_id),
        'role': 'yield' if is_yield else 'standon',
        'has_raw_observation': agent.get('raw_observation') is not None,
        'route_progress': route_progress,
        'abs_cte': abs_cte,
        'threat_score': threat_score,
        'team_min_separation': team_min_separation,
        'risk_candidate': bool(in_window),
        'progress_bucket': _progress_bucket(route_progress),
    }


def audit(paths: list[str], args) -> dict:
    summary = {
        'files': 0,
        'episodes': 0,
        'collisions': 0,
        'timeouts': 0,
        'trace_samples': 0,
        'agent_samples': 0,
        'raw_agent_samples': 0,
        'risk_candidates': 0,
        'collision_paths': [],
        'by_role': Counter(),
        'risk_by_role': Counter(),
        'risk_by_agent': Counter(),
        'risk_by_progress': Counter(),
    }
    for json_path in _iter_json_paths(paths):
        summary['files'] += 1
        payload = json.loads(json_path.read_text(encoding='utf-8'))
        file_collision = False
        for episode in payload.get('episode_metrics', []):
            summary['episodes'] += 1
            if bool(episode.get('collision', False)):
                summary['collisions'] += 1
                file_collision = True
            if bool(episode.get('timeout', False)):
                summary['timeouts'] += 1
            for sample in episode.get('trace_samples', []):
                summary['trace_samples'] += 1
                for agent_id, agent in (sample.get('agents') or {}).items():
                    record = _agent_record(sample, str(agent_id), agent, args)
                    summary['agent_samples'] += 1
                    summary['by_role'][record['role']] += 1
                    if record['has_raw_observation']:
                        summary['raw_agent_samples'] += 1
                    if record['risk_candidate']:
                        summary['risk_candidates'] += 1
                        summary['risk_by_role'][record['role']] += 1
                        summary['risk_by_agent'][record['agent_id']] += 1
                        summary['risk_by_progress'][record['progress_bucket']] += 1
        if file_collision:
            summary['collision_paths'].append(str(json_path))
    for key in ('by_role', 'risk_by_role', 'risk_by_agent', 'risk_by_progress'):
        summary[key] = dict(summary[key])
    return summary


def main():
    args = parse_args()
    summary = audit(args.paths, args)
    if bool(args.json):
        print(json.dumps(summary, indent=2, sort_keys=True))
        return
    print(
        'files={files} episodes={episodes} collisions={collisions} timeouts={timeouts} '
        'trace_samples={trace_samples} agent_samples={agent_samples} raw_agent_samples={raw_agent_samples} '
        'risk_candidates={risk_candidates}'.format(**summary)
    )
    print(f"by_role={summary['by_role']}")
    print(f"risk_by_role={summary['risk_by_role']}")
    print(f"risk_by_agent={summary['risk_by_agent']}")
    print(f"risk_by_progress={summary['risk_by_progress']}")
    if summary['collision_paths']:
        print('collision_paths:')
        for path in summary['collision_paths']:
            print(path)


if __name__ == '__main__':
    main()