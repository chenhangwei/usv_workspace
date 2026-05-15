#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description='Summarize deconflict target consistency from collision raw traces.')
    parser.add_argument('--root', action='append', required=True, help='Directory or JSON file to scan. Repeatable.')
    parser.add_argument('--output-dir', required=True, help='Directory for TSV reports.')
    parser.add_argument('--max-separation', type=float, default=1.55, help='Only summarize agents in samples at or below this separation.')
    return parser.parse_args()


def iter_json_paths(roots):
    seen = set()
    for root in roots:
        path = Path(root)
        paths = [path] if path.is_file() else sorted(path.rglob('seed*.json'))
        for item in paths:
            key = str(item.resolve())
            if key in seen:
                continue
            seen.add(key)
            yield item


def sign(value, eps=1e-6):
    if value > eps:
        return 'positive'
    if value < -eps:
        return 'negative'
    return 'zero'


def load_collision_rows(path, max_separation):
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except Exception as exc:
        return [], [{'json': str(path), 'error': str(exc)}]
    rows = []
    errors = []
    label = path.parent.parent.name if path.parent.name.startswith('repeat') else path.parent.name
    seed = ''.join(ch for ch in path.stem if ch.isdigit()) or path.stem
    for episode_index, episode in enumerate(data.get('episode_metrics') or []):
        collision = bool(episode.get('collision', False)) or float(data.get('collision_rate', 0.0)) > 0.0
        if not collision:
            continue
        samples = episode.get('trace_samples') or []
        if not samples:
            errors.append({'json': str(path), 'error': 'collision_without_trace_samples'})
            continue
        selected = [sample for sample in samples if float(sample.get('pairwise_min_separation', 1e9)) <= max_separation]
        if not selected:
            selected = [min(samples, key=lambda item: float(item.get('pairwise_min_separation', 1e9)))]
        for sample in selected:
            sample_sep = float(sample.get('pairwise_min_separation', 0.0))
            for agent_id, agent in sorted((sample.get('agents') or {}).items()):
                diagnostics = agent.get('mask_diagnostics') or {}
                deconf = diagnostics.get('deconf_target') or {}
                if not diagnostics.get('random_deconflict_weighted_active', False):
                    continue
                target_linear = float(deconf.get('target_linear', 0.0))
                target_omega = float(deconf.get('target_omega', 0.0))
                actual_linear = float(agent.get('final_linear_x', 0.0))
                actual_omega = float(agent.get('final_angular_z', 0.0))
                rows.append({
                    'label': label,
                    'seed': seed,
                    'json': str(path),
                    'episode': episode_index,
                    'step': sample.get('step', ''),
                    'sample_sep': f'{sample_sep:.6f}',
                    'agent': agent_id,
                    'priority': f"{float(agent.get('crossing_priority', 0.0)):.6f}",
                    'yield': str(bool(diagnostics.get('deconf_is_yield', False))),
                    'nearest': str(agent.get('nearest_id', '')),
                    'nearest_distance': f"{float(agent.get('nearest_distance', 0.0)):.6f}",
                    'distance_to_goal': f"{float(agent.get('distance_to_goal', 0.0)):.6f}",
                    'route_progress': f"{float(agent.get('route_progress', 0.0)):.6f}",
                    'cte': f"{float(agent.get('cross_track_error', 0.0)):.6f}",
                    'threat': f"{float(diagnostics.get('threat_score', 0.0)):.6f}",
                    'target_linear': f'{target_linear:.6f}',
                    'target_omega': f'{target_omega:.6f}',
                    'target_omega_sign': sign(target_omega),
                    'actual_linear': f'{actual_linear:.6f}',
                    'actual_omega': f'{actual_omega:.6f}',
                    'linear_error': f"{float(deconf.get('linear_error', target_linear - actual_linear)):.6f}",
                    'omega_error': f"{float(deconf.get('omega_error', target_omega - actual_omega)):.6f}",
                })
    return rows, errors


def write_tsv(path, rows, fields):
    with Path(path).open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, delimiter='\t')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    detail_rows = []
    error_rows = []
    for path in iter_json_paths(args.root):
        rows, errors = load_collision_rows(path, float(args.max_separation))
        detail_rows.extend(rows)
        error_rows.extend(errors)

    detail_fields = [
        'label', 'seed', 'json', 'episode', 'step', 'sample_sep', 'agent', 'priority', 'yield',
        'nearest', 'nearest_distance', 'distance_to_goal', 'route_progress', 'cte', 'threat',
        'target_linear', 'target_omega', 'target_omega_sign', 'actual_linear', 'actual_omega',
        'linear_error', 'omega_error',
    ]
    write_tsv(output_dir / 'collision_target_details.tsv', detail_rows, detail_fields)

    groups = {}
    for row in detail_rows:
        key = (row['label'], row['seed'], row['yield'])
        group = groups.setdefault(key, {'label': row['label'], 'seed': row['seed'], 'yield': row['yield'], 'samples': 0, 'positive': 0, 'negative': 0, 'zero': 0, 'target_omega_sum': 0.0, 'target_linear_sum': 0.0})
        group['samples'] += 1
        group[row['target_omega_sign']] += 1
        group['target_omega_sum'] += float(row['target_omega'])
        group['target_linear_sum'] += float(row['target_linear'])
    summary_rows = []
    for group in groups.values():
        samples = max(1, int(group['samples']))
        dominant = max(('positive', 'negative', 'zero'), key=lambda key: group[key])
        summary_rows.append({
            'label': group['label'],
            'seed': group['seed'],
            'yield': group['yield'],
            'samples': group['samples'],
            'positive': group['positive'],
            'negative': group['negative'],
            'zero': group['zero'],
            'dominant_omega_sign': dominant,
            'mean_target_omega': f"{group['target_omega_sum'] / samples:.6f}",
            'mean_target_linear': f"{group['target_linear_sum'] / samples:.6f}",
        })
    summary_rows.sort(key=lambda row: (row['label'], int(row['seed']) if str(row['seed']).isdigit() else row['seed'], row['yield']))
    summary_fields = ['label', 'seed', 'yield', 'samples', 'positive', 'negative', 'zero', 'dominant_omega_sign', 'mean_target_omega', 'mean_target_linear']
    write_tsv(output_dir / 'collision_target_summary.tsv', summary_rows, summary_fields)
    if error_rows:
        write_tsv(output_dir / 'collision_target_errors.tsv', error_rows, ['json', 'error'])
    print(f'details={output_dir / "collision_target_details.tsv"}')
    print(f'summary={output_dir / "collision_target_summary.tsv"}')
    print(f'collision_target_rows={len(detail_rows)} errors={len(error_rows)}')


if __name__ == '__main__':
    main()