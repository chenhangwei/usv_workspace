#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def _true_cross_track(agent_geometry: dict, raw_observation: list[float]) -> float | None:
    if not agent_geometry or raw_observation is None or len(raw_observation) < 2:
        return None
    spawn = agent_geometry.get('spawn') or {}
    goal = agent_geometry.get('goal') or {}
    try:
        spawn_x = float(spawn['x'])
        spawn_y = float(spawn['y'])
        goal_x = float(goal['x'])
        goal_y = float(goal['y'])
        pose_x = float(raw_observation[0])
        pose_y = float(raw_observation[1])
    except (KeyError, TypeError, ValueError):
        return None
    route_dx = goal_x - spawn_x
    route_dy = goal_y - spawn_y
    route_length = (route_dx * route_dx + route_dy * route_dy) ** 0.5
    if route_length <= 1e-9:
        return None
    rel_x = pose_x - spawn_x
    rel_y = pose_y - spawn_y
    return ((rel_x * route_dy) - (rel_y * route_dx)) / route_length


def _iter_rows(path: Path):
    payload = json.loads(path.read_text(encoding='utf-8'))
    for episode_index, episode in enumerate(payload.get('episode_metrics', [])):
        geometry = ((episode.get('scenario_geometry') or {}).get('agents') or {})
        scenario = episode.get('scenario', '')
        reset_seed = episode.get('reset_seed', '')
        for sample in episode.get('trace_samples', []):
            step = int(sample.get('step', 0))
            for agent_id, agent in (sample.get('agents') or {}).items():
                true_cte = agent.get('raw_cross_track_error')
                if true_cte is not None:
                    true_cte = float(true_cte)
                else:
                    true_cte = _true_cross_track(geometry.get(agent_id, {}), agent.get('raw_observation'))
                if true_cte is None:
                    continue
                clipped_cte = float(agent.get('cross_track_error', 0.0))
                overflow = max(0.0, abs(true_cte) - abs(clipped_cte))
                diagnostics = agent.get('mask_diagnostics') or {}
                yield {
                    'path': str(path),
                    'episode_index': episode_index,
                    'scenario': scenario,
                    'reset_seed': reset_seed,
                    'step': step,
                    'agent_id': agent_id,
                    'true_cte': true_cte,
                    'clipped_cte': clipped_cte,
                    'overflow': overflow,
                    'route_progress': float(agent.get('route_progress', 0.0)),
                    'distance_to_goal': float(agent.get('distance_to_goal', 0.0)),
                    'heading_error': float(agent.get('heading_error', 0.0)),
                    'final_linear_x': float(agent.get('final_linear_x', 0.0)),
                    'final_angular_z': float(agent.get('final_angular_z', 0.0)),
                    'threat_score': float(diagnostics.get('threat_score', 0.0)),
                    'team_min_separation': float(diagnostics.get('team_min_separation', 0.0)),
                }


def main() -> None:
    parser = argparse.ArgumentParser(description='Analyze clipped vs true cross-track error in trace JSON files.')
    parser.add_argument('trace_json', nargs='+', type=Path)
    parser.add_argument('--agent', action='append', dest='agents', default=None, help='Agent id to include. Repeatable; default includes all agents.')
    parser.add_argument('--clip-range', type=float, default=3.0, help='Expected symmetric CTE clip range.')
    parser.add_argument('--top-k', type=int, default=20)
    parser.add_argument('--min-abs-clipped', type=float, default=2.8)
    args = parser.parse_args()

    selected_agents = set(str(agent) for agent in (args.agents or []))
    rows = []
    for path in args.trace_json:
        for row in _iter_rows(path):
            if selected_agents and row['agent_id'] not in selected_agents:
                continue
            rows.append(row)

    if not rows:
        print('rows=0')
        return

    saturated = [row for row in rows if abs(row['clipped_cte']) >= float(args.min_abs_clipped)]
    overflowed = [row for row in rows if row['overflow'] > 1e-6]
    print(
        'rows={rows} saturated={saturated} overflowed={overflowed} max_abs_true_cte={max_true:.6f} max_overflow={max_overflow:.6f}'.format(
            rows=len(rows),
            saturated=len(saturated),
            overflowed=len(overflowed),
            max_true=max(abs(row['true_cte']) for row in rows),
            max_overflow=max(row['overflow'] for row in rows),
        )
    )
    print('top_overflow:')
    print('path\tstep\tagent\ttrue_cte\tclipped_cte\toverflow\tprogress\tdgoal\theading\tvx\tomega\tthreat\tsep')
    for row in sorted(rows, key=lambda item: (item['overflow'], abs(item['true_cte'])), reverse=True)[: max(0, int(args.top_k))]:
        print(
            '{path}\t{step}\t{agent_id}\t{true_cte:.6f}\t{clipped_cte:.6f}\t{overflow:.6f}\t{route_progress:.6f}\t{distance_to_goal:.3f}\t{heading_error:.3f}\t{final_linear_x:.4f}\t{final_angular_z:.4f}\t{threat_score:.3f}\t{team_min_separation:.3f}'.format(**row)
        )


if __name__ == '__main__':
    main()