#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description='Compare per-agent MAPPO trace progress/action bins across evaluation JSON files.')
    parser.add_argument('--run', action='append', required=True, help='Run spec as label:path/to/seed.json. Repeatable.')
    parser.add_argument('--agent', action='append', default=None, help='Agent id to include. Repeatable; unset includes all agents.')
    parser.add_argument('--bin-size', type=int, default=100, help='Step bin size for summaries.')
    parser.add_argument('--output-tsv', help='Optional TSV path for bin rows.')
    return parser.parse_args()


def _float_value(mapping, key, default=np.nan):
    try:
        return float(mapping.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _bool_value(mapping, key):
    return 1.0 if bool(mapping.get(key, False)) else 0.0


def _parse_run(value: str):
    label, separator, path = value.partition(':')
    if not separator or not label or not path:
        raise argparse.ArgumentTypeError(f'Invalid --run value {value!r}; expected label:path')
    return label, Path(path)


def _episode(path: Path):
    payload = json.loads(path.read_text(encoding='utf-8'))
    episodes = payload.get('episode_metrics') or []
    if not episodes:
        raise RuntimeError(f'No episode_metrics in {path}')
    return episodes[0]


def _agent_rows(episode: dict, agent_filter: set[str] | None):
    rows = []
    for sample in episode.get('trace_samples') or []:
        step = int(sample.get('step', 0))
        pairwise_min_separation = _float_value(sample, 'pairwise_min_separation')
        for agent_id, agent in sorted((sample.get('agents') or {}).items()):
            if agent_filter and str(agent_id) not in agent_filter:
                continue
            diagnostics = agent.get('mask_diagnostics') or {}
            rows.append({
                'agent': str(agent_id),
                'step': step,
                'route_progress': _float_value(agent, 'route_progress'),
                'distance_to_goal': _float_value(agent, 'distance_to_goal'),
                'linear': _float_value(agent, 'final_linear_x'),
                'omega': _float_value(agent, 'final_angular_z'),
                'cte_abs': abs(_float_value(agent, 'cross_track_error')),
                'heading_abs': abs(_float_value(agent, 'heading_error')),
                'nearest_distance': _float_value(agent, 'nearest_distance'),
                'pairwise_min_separation': pairwise_min_separation,
                'team_min_separation': _float_value(diagnostics, 'team_min_separation'),
                'threat_score': _float_value(diagnostics, 'threat_score'),
                'deconf_active': _bool_value(diagnostics, 'random_deconflict_weighted_active'),
                'cte_active': _bool_value(diagnostics, 'random_cte_recovery_weighted_active'),
                'finish_team_clear': _bool_value(diagnostics, 'finish_team_clear'),
                'finish_neighbor_clear': _bool_value(diagnostics, 'finish_neighbor_clear'),
            })
    return rows


def _mean(rows: list[dict], key: str):
    values = [row[key] for row in rows if np.isfinite(row[key])]
    return float(np.mean(values)) if values else float('nan')


def _minimum(rows: list[dict], key: str):
    values = [row[key] for row in rows if np.isfinite(row[key])]
    return float(np.min(values)) if values else float('nan')


def _bin_rows(label: str, episode: dict, rows: list[dict], bin_size: int):
    output = []
    seed = episode.get('seed', episode.get('reset_seed', ''))
    by_agent_bin = {}
    for row in rows:
        bin_start = int(row['step'] // bin_size) * bin_size
        by_agent_bin.setdefault((row['agent'], bin_start), []).append(row)

    for (agent_id, bin_start), items in sorted(by_agent_bin.items()):
        ordered = sorted(items, key=lambda item: item['step'])
        first = ordered[0]
        last = ordered[-1]
        progress_delta = float(last['route_progress'] - first['route_progress'])
        distance_drop = float(first['distance_to_goal'] - last['distance_to_goal'])
        output.append({
            'label': label,
            'seed': seed,
            'agent': agent_id,
            'bin_start': bin_start,
            'bin_end': bin_start + bin_size,
            'samples': len(ordered),
            'progress_start': first['route_progress'],
            'progress_end': last['route_progress'],
            'progress_delta': progress_delta,
            'distance_start': first['distance_to_goal'],
            'distance_end': last['distance_to_goal'],
            'distance_drop': distance_drop,
            'mean_linear': _mean(ordered, 'linear'),
            'mean_omega': _mean(ordered, 'omega'),
            'mean_abs_omega': _mean([dict(item, omega=abs(item['omega'])) for item in ordered], 'omega'),
            'mean_abs_cte': _mean(ordered, 'cte_abs'),
            'mean_abs_heading': _mean(ordered, 'heading_abs'),
            'min_nearest_distance': _minimum(ordered, 'nearest_distance'),
            'min_pairwise_separation': _minimum(ordered, 'pairwise_min_separation'),
            'mean_threat': _mean(ordered, 'threat_score'),
            'mean_team_separation': _mean(ordered, 'team_min_separation'),
            'deconf_rate': _mean(ordered, 'deconf_active'),
            'cte_active_rate': _mean(ordered, 'cte_active'),
            'finish_team_clear_rate': _mean(ordered, 'finish_team_clear'),
            'finish_neighbor_clear_rate': _mean(ordered, 'finish_neighbor_clear'),
        })
    return output


def _print_final(label: str, episode: dict, agent_filter: set[str] | None):
    print(
        'RUN'
        f'\tlabel={label}'
        f'\tscenario={episode.get("scenario", "")}'
        f'\tsteps={episode.get("steps", "")}'
        f'\tteam_progress={_float_value(episode, "team_goal_progress_ratio"):.6f}'
        f'\tsuccess={episode.get("success", "")}'
        f'\ttimeout={episode.get("timeout", "")}'
        f'\tcollision={episode.get("collision", "")}'
        f'\tmin_sep={_float_value(episode, "pairwise_min_separation"):.6f}'
    )
    for agent_id, metrics in sorted((episode.get('final_agent_metrics') or {}).items()):
        if agent_filter and str(agent_id) not in agent_filter:
            continue
        print(
            'FINAL'
            f'\tlabel={label}'
            f'\tagent={agent_id}'
            f'\tprogress={_float_value(metrics, "route_progress"):.6f}'
            f'\tdgoal={_float_value(metrics, "distance_to_goal"):.3f}'
            f'\tcte={_float_value(metrics, "cross_track_error"):.3f}'
            f'\tlinear={_float_value(metrics, "final_linear_x"):.3f}'
            f'\tomega={_float_value(metrics, "final_angular_z"):.3f}'
        )


def _print_bin_highlights(bin_rows: list[dict], label: str, agent_id: str):
    agent_bins = [row for row in bin_rows if row['label'] == label and row['agent'] == agent_id]
    if not agent_bins:
        return
    stagnant = [row for row in agent_bins if row['progress_delta'] < 0.002 and row['mean_linear'] > 0.01]
    worst_progress = min(agent_bins, key=lambda row: row['progress_delta'])
    best_progress = max(agent_bins, key=lambda row: row['progress_delta'])
    print(
        'HIGHLIGHT'
        f'\tlabel={label}'
        f'\tagent={agent_id}'
        f'\tstagnant_bins={len(stagnant)}'
        f'\tworst_bin={worst_progress["bin_start"]}-{worst_progress["bin_end"]}'
        f'\tworst_dprog={worst_progress["progress_delta"]:.6f}'
        f'\tworst_linear={worst_progress["mean_linear"]:.4f}'
        f'\tworst_cte={worst_progress["mean_abs_cte"]:.4f}'
        f'\tbest_bin={best_progress["bin_start"]}-{best_progress["bin_end"]}'
        f'\tbest_dprog={best_progress["progress_delta"]:.6f}'
    )


def main():
    args = parse_args()
    agent_filter = set(str(agent) for agent in args.agent or []) or None
    all_bin_rows = []
    labels = []
    agents_seen = set()
    for run_value in args.run:
        label, path = _parse_run(run_value)
        labels.append(label)
        episode = _episode(path)
        _print_final(label, episode, agent_filter)
        rows = _agent_rows(episode, agent_filter)
        agents_seen.update(row['agent'] for row in rows)
        all_bin_rows.extend(_bin_rows(label, episode, rows, int(args.bin_size)))
    print()
    for label in labels:
        for agent_id in sorted(agents_seen):
            _print_bin_highlights(all_bin_rows, label, agent_id)

    if args.output_tsv:
        fields = [
            'label', 'seed', 'agent', 'bin_start', 'bin_end', 'samples',
            'progress_start', 'progress_end', 'progress_delta',
            'distance_start', 'distance_end', 'distance_drop',
            'mean_linear', 'mean_omega', 'mean_abs_omega', 'mean_abs_cte',
            'mean_abs_heading', 'min_nearest_distance', 'min_pairwise_separation',
            'mean_threat', 'mean_team_separation', 'deconf_rate', 'cte_active_rate',
            'finish_team_clear_rate', 'finish_neighbor_clear_rate',
        ]
        with Path(args.output_tsv).open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields, delimiter='\t')
            writer.writeheader()
            for row in all_bin_rows:
                writer.writerow({field: row.get(field, '') for field in fields})
        print(f'output_tsv={args.output_tsv}')


if __name__ == '__main__':
    main()