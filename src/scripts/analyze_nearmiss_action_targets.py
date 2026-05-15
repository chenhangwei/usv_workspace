#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description='Analyze actor-vs-target errors in near-miss/collision trace samples.')
    parser.add_argument('--root', action='append', required=True, help='Directory or seed JSON to scan. Repeatable.')
    parser.add_argument('--output-dir', required=True, help='Output directory for TSV reports.')
    parser.add_argument('--max-separation', type=float, default=1.05, help='Keep samples at or below this pairwise separation.')
    parser.add_argument('--include-inactive', action='store_true', help='Also include samples without active random_deconflict targets.')
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


def sep_bucket(separation):
    if separation <= 0.75:
        return '<=0.75'
    if separation <= 0.90:
        return '0.75-0.90'
    if separation <= 1.05:
        return '0.90-1.05'
    if separation <= 1.55:
        return '1.05-1.55'
    return '>1.55'


def root_label(path):
    parts = path.parts
    for marker in ('fresh329', 'fresh330', 'fresh328', 'fresh321'):
        for part in reversed(parts):
            if marker in part:
                return part
    if path.parent.name.startswith('repeat'):
        return path.parent.parent.name
    return path.parent.name


def load_rows(path, max_separation, include_inactive):
    rows = []
    errors = []
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except Exception as exc:
        return rows, [{'json': str(path), 'error': str(exc)}]
    label = root_label(path)
    seed = ''.join(ch for ch in path.stem if ch.isdigit()) or path.stem
    for episode_index, episode in enumerate(data.get('episode_metrics') or []):
        collision = bool(episode.get('collision', False)) or float(data.get('collision_rate', 0.0)) > 0.0
        for sample in episode.get('trace_samples') or []:
            sample_sep = float(sample.get('pairwise_min_separation', 1e9))
            if sample_sep > max_separation:
                continue
            for agent_id, agent in sorted((sample.get('agents') or {}).items()):
                diagnostics = agent.get('mask_diagnostics') or {}
                deconf_active = bool(diagnostics.get('random_deconflict_weighted_active', False))
                if not include_inactive and not deconf_active:
                    continue
                target = diagnostics.get('deconf_target') or {}
                target_linear = float(target.get('target_linear', agent.get('final_linear_x', 0.0)))
                target_omega = float(target.get('target_omega', agent.get('final_angular_z', 0.0)))
                actual_linear = float(agent.get('final_linear_x', 0.0))
                actual_omega = float(agent.get('final_angular_z', 0.0))
                linear_error = target_linear - actual_linear
                omega_error = target_omega - actual_omega
                rows.append({
                    'label': label,
                    'seed': seed,
                    'json': str(path),
                    'episode': episode_index,
                    'collision': int(collision),
                    'step': sample.get('step', ''),
                    'sample_sep': f'{sample_sep:.6f}',
                    'sep_bucket': sep_bucket(sample_sep),
                    'agent': str(agent_id),
                    'priority': f"{float(agent.get('crossing_priority', 0.0)):.6f}",
                    'yield': str(bool(diagnostics.get('deconf_is_yield', False))),
                    'deconf_active': str(deconf_active),
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
                    'actual_omega_sign': sign(actual_omega),
                    'linear_error': f'{linear_error:.6f}',
                    'omega_error': f'{omega_error:.6f}',
                    'abs_linear_error': f'{abs(linear_error):.6f}',
                    'abs_omega_error': f'{abs(omega_error):.6f}',
                })
    return rows, errors


def write_tsv(path, rows, fields):
    with Path(path).open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, delimiter='\t')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def mean(values):
    return sum(values) / len(values) if values else 0.0


def summarize(rows):
    groups = {}
    for row in rows:
        key = (row['label'], row['seed'], row['sep_bucket'], row['yield'], row['target_omega_sign'])
        group = groups.setdefault(key, {
            'label': row['label'],
            'seed': row['seed'],
            'sep_bucket': row['sep_bucket'],
            'yield': row['yield'],
            'target_omega_sign': row['target_omega_sign'],
            'samples': 0,
            'collisions': 0,
            'target_linear': [],
            'actual_linear': [],
            'target_omega': [],
            'actual_omega': [],
            'linear_error': [],
            'omega_error': [],
            'abs_linear_error': [],
            'abs_omega_error': [],
        })
        group['samples'] += 1
        group['collisions'] += int(row['collision'])
        for field in ('target_linear', 'actual_linear', 'target_omega', 'actual_omega', 'linear_error', 'omega_error', 'abs_linear_error', 'abs_omega_error'):
            group[field].append(float(row[field]))
    out = []
    order = {'<=0.75': 0, '0.75-0.90': 1, '0.90-1.05': 2, '1.05-1.55': 3, '>1.55': 4}
    for group in groups.values():
        out.append({
            'label': group['label'],
            'seed': group['seed'],
            'sep_bucket': group['sep_bucket'],
            'yield': group['yield'],
            'target_omega_sign': group['target_omega_sign'],
            'samples': group['samples'],
            'collisions': group['collisions'],
            'mean_target_linear': f"{mean(group['target_linear']):.6f}",
            'mean_actual_linear': f"{mean(group['actual_linear']):.6f}",
            'mean_linear_error': f"{mean(group['linear_error']):.6f}",
            'mean_abs_linear_error': f"{mean(group['abs_linear_error']):.6f}",
            'mean_target_omega': f"{mean(group['target_omega']):.6f}",
            'mean_actual_omega': f"{mean(group['actual_omega']):.6f}",
            'mean_omega_error': f"{mean(group['omega_error']):.6f}",
            'mean_abs_omega_error': f"{mean(group['abs_omega_error']):.6f}",
        })
    out.sort(key=lambda row: (row['label'], int(row['seed']) if str(row['seed']).isdigit() else row['seed'], order.get(row['sep_bucket'], 99), row['yield'], row['target_omega_sign']))
    return out


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    errors = []
    for path in iter_json_paths(args.root):
        loaded, load_errors = load_rows(path, float(args.max_separation), bool(args.include_inactive))
        rows.extend(loaded)
        errors.extend(load_errors)

    detail_fields = [
        'label', 'seed', 'json', 'episode', 'collision', 'step', 'sample_sep', 'sep_bucket',
        'agent', 'priority', 'yield', 'deconf_active', 'nearest', 'nearest_distance',
        'distance_to_goal', 'route_progress', 'cte', 'threat', 'target_linear',
        'target_omega', 'target_omega_sign', 'actual_linear', 'actual_omega',
        'actual_omega_sign', 'linear_error', 'omega_error', 'abs_linear_error', 'abs_omega_error',
    ]
    summary_fields = [
        'label', 'seed', 'sep_bucket', 'yield', 'target_omega_sign', 'samples', 'collisions',
        'mean_target_linear', 'mean_actual_linear', 'mean_linear_error', 'mean_abs_linear_error',
        'mean_target_omega', 'mean_actual_omega', 'mean_omega_error', 'mean_abs_omega_error',
    ]
    write_tsv(output_dir / 'nearmiss_target_details.tsv', rows, detail_fields)
    write_tsv(output_dir / 'nearmiss_target_summary.tsv', summarize(rows), summary_fields)
    if errors:
        write_tsv(output_dir / 'nearmiss_target_errors.tsv', errors, ['json', 'error'])
    print(f'details={output_dir / "nearmiss_target_details.tsv"}')
    print(f'summary={output_dir / "nearmiss_target_summary.tsv"}')
    print(f'nearmiss_rows={len(rows)} errors={len(errors)}')


if __name__ == '__main__':
    main()