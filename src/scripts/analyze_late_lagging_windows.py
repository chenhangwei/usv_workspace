#!/usr/bin/env python3
import argparse
import glob
import json
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description='Summarize late-lagging gate windows from MAPPO evaluation JSON files.')
    parser.add_argument('paths', nargs='+', help='Evaluation directories or JSON files.')
    parser.add_argument('--seed', action='append', dest='seeds', default=None, help='Optional seed filter. Repeatable.')
    parser.add_argument('--team-progress-min', type=float, default=0.75)
    parser.add_argument('--max-self-progress', type=float, default=0.55)
    parser.add_argument('--min-progress-gap', type=float, default=0.25)
    parser.add_argument('--min-neighbor-separation', type=float, default=4.8)
    parser.add_argument('--min-distance', type=float, default=2.0)
    parser.add_argument('--max-distance', type=float, default=80.0)
    parser.add_argument('--merge-gap', type=int, default=25)
    return parser.parse_args()


def iter_json_paths(inputs, seeds):
    seed_filter = {str(seed) for seed in seeds or ()}
    seen = set()
    for value in inputs:
        path = Path(value)
        if path.is_dir():
            patterns = ['repeat*_seed*/seed*.json', '**/seed*.json']
            candidates = []
            for pattern in patterns:
                candidates.extend(glob.glob(str(path / pattern)))
        else:
            candidates = glob.glob(str(path))
        for candidate in sorted(candidates):
            candidate_path = Path(candidate)
            if candidate_path in seen:
                continue
            seed_text = candidate_path.stem.replace('seed', '')
            if seed_filter and seed_text not in seed_filter:
                continue
            seen.add(candidate_path)
            yield candidate_path


def merge_windows(steps, merge_gap):
    windows = []
    for step in steps:
        if windows and windows[-1][1] >= step - merge_gap:
            windows[-1] = (windows[-1][0], step)
        else:
            windows.append((step, step))
    return windows


def repeat_from_path(path):
    parent = path.parent.name
    if parent.startswith('repeat') and '_seed' in parent:
        return parent.split('_', 1)[0].replace('repeat', '')
    return ''


def float_field(mapping, key, default=0.0):
    try:
        return float(mapping.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def analyze_episode(path, episode, args):
    seed = str(episode.get('seed') or path.stem.replace('seed', ''))
    repeat = repeat_from_path(path)
    final_metrics = episode.get('final_agent_metrics') or {}
    if not final_metrics:
        return

    slowest_agent = min(final_metrics, key=lambda agent_id: float_field(final_metrics[agent_id], 'route_progress', 999.0))
    print(f'CASE\trepeat={repeat}\tseed={seed}\tpath={path}')
    for agent_id in sorted(final_metrics):
        metrics = final_metrics[agent_id]
        print(
            'FINAL'
            f'\tagent={agent_id}'
            f'\tprogress={float_field(metrics, "route_progress"):.6f}'
            f'\tdgoal={float_field(metrics, "distance_to_goal"):.3f}'
            f'\tcte={float_field(metrics, "cross_track_error"):.3f}'
            f'\tlinear={float_field(metrics, "final_linear_x"):.3f}'
            f'\tomega={float_field(metrics, "final_angular_z"):.3f}'
        )
    print(f'SLOWEST\tagent={slowest_agent}\tprogress={float_field(final_metrics[slowest_agent], "route_progress"):.6f}')

    samples = episode.get('trace_samples') or []
    for agent_id in sorted(final_metrics):
        active_steps = []
        blocks = {
            'team_progress': 0,
            'self_progress': 0,
            'progress_gap': 0,
            'neighbor_sep': 0,
            'distance': 0,
        }
        for sample in samples:
            agents = sample.get('agents') or {}
            if agent_id not in agents:
                continue
            team_progress = max(float_field(agent, 'route_progress') for agent in agents.values())
            agent = agents[agent_id]
            self_progress = float_field(agent, 'route_progress')
            progress_gap = team_progress - self_progress
            nearest_distance = float_field(agent, 'nearest_distance', 999.0)
            distance_to_goal = float_field(agent, 'distance_to_goal', 999.0)

            team_ok = team_progress >= args.team_progress_min
            self_ok = self_progress <= args.max_self_progress
            gap_ok = progress_gap >= args.min_progress_gap
            neighbor_ok = nearest_distance >= args.min_neighbor_separation
            distance_ok = args.min_distance < distance_to_goal <= args.max_distance

            if team_ok and self_ok and gap_ok and neighbor_ok and distance_ok:
                active_steps.append(int(sample.get('step', 0)))
                continue
            if not team_ok:
                blocks['team_progress'] += 1
            elif not self_ok:
                blocks['self_progress'] += 1
            elif not gap_ok:
                blocks['progress_gap'] += 1
            elif not neighbor_ok:
                blocks['neighbor_sep'] += 1
            elif not distance_ok:
                blocks['distance'] += 1

        if active_steps:
            windows = merge_windows(active_steps, args.merge_gap)
            window_text = ','.join(f'{start}-{end}' for start, end in windows[:8])
            print(
                'WINDOW'
                f'\tagent={agent_id}'
                f'\tcount={len(active_steps)}'
                f'\trange={active_steps[0]}-{active_steps[-1]}'
                f'\twindows={window_text}'
            )
        else:
            block_text = ','.join(f'{name}:{count}' for name, count in sorted(blocks.items(), key=lambda item: item[1], reverse=True))
            print(f'WINDOW\tagent={agent_id}\tcount=0\tblocks={block_text}')
    print()


def main():
    args = parse_args()
    for path in iter_json_paths(args.paths, args.seeds):
        payload = json.loads(path.read_text(encoding='utf-8'))
        for episode in payload.get('episode_metrics') or []:
            analyze_episode(path, episode, args)


if __name__ == '__main__':
    main()