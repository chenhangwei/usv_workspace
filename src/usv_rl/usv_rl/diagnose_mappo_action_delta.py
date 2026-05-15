import argparse
import csv
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

from .observation_normalizer import ObservationNormalizer


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Compare two MAPPO checkpoints on saved raw-observation traces.')
    parser.add_argument('--base-model', required=True, help='Reference MAPPO checkpoint.')
    parser.add_argument('--candidate-model', required=True, help='Candidate MAPPO checkpoint.')
    parser.add_argument('--trace-json', action='append', required=True, help='Evaluation JSON with trace_samples raw_observation. Repeatable.')
    parser.add_argument('--device', default='cpu', help='Torch device.')
    parser.add_argument('--danger-separation', type=float, default=1.20, help='Pairwise separation threshold for focused stats.')
    parser.add_argument('--top-k', type=int, default=12, help='Rows to print from the largest normalized deltas.')
    parser.add_argument('--output-tsv', help='Optional TSV with per-agent action deltas.')
    return parser.parse_args(argv)


def _load_actor(checkpoint: dict, device):
    if checkpoint.get('neighbor_attention', False):
        from usv_rl.neighbor_attention import build_attention_actor_from_checkpoint
        actor = build_attention_actor_from_checkpoint(checkpoint, nn, torch, device)
    else:
        hidden_sizes = tuple(int(value) for value in checkpoint.get('hidden_sizes', [128, 128]))
        action_dim = int(checkpoint['action_dim'])
        obs_dim = int(checkpoint['local_observation_size'])
        layers = []
        current_dim = obs_dim
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(current_dim, hidden_size))
            layers.append(nn.Tanh())
            current_dim = hidden_size
        layers.append(nn.Linear(current_dim, action_dim))
        actor = nn.Sequential(*layers).to(device)
        actor.load_state_dict(checkpoint['actor_state_dict'])
    actor.eval()
    return actor


def _load_bundle(model_path: str, device):
    checkpoint = torch.load(model_path, map_location=device, weights_only=False)
    actor = _load_actor(checkpoint, device)
    obs_dim = int(checkpoint['local_observation_size'])
    normalizer = None
    if checkpoint.get('normalize_observations') and checkpoint.get('obs_normalizer') is not None:
        normalizer = ObservationNormalizer(obs_dim)
        normalizer.load_state_dict(checkpoint['obs_normalizer'])
    action_low = np.asarray(checkpoint.get('action_low', [0.0, -0.4]), dtype=np.float32)
    action_high = np.asarray(checkpoint.get('action_high', [0.4, 0.4]), dtype=np.float32)
    return {
        'checkpoint': checkpoint,
        'actor': actor,
        'normalizer': normalizer,
        'squash': bool(checkpoint.get('squash_actions', False)),
        'action_low': action_low,
        'action_high': action_high,
        'scenarios': tuple(str(name) for name in checkpoint.get('scenarios', ())),
        'obs_dim': obs_dim,
    }


def _scenario_id(bundle: dict, scenario_name: str) -> int:
    try:
        return bundle['scenarios'].index(str(scenario_name))
    except ValueError:
        return 0


def _actor_forward(actor, obs_tensor, scenario_ids):
    try:
        return actor(obs_tensor, scenario_ids)
    except TypeError:
        return actor(obs_tensor)


def _predict(bundle: dict, raw_observations: np.ndarray, scenario_ids: np.ndarray, device) -> np.ndarray:
    observations = raw_observations.astype(np.float32, copy=False)
    if bundle['normalizer'] is not None:
        observations = bundle['normalizer'].normalize(observations).astype(np.float32, copy=False)
    obs_tensor = torch.as_tensor(observations, dtype=torch.float32, device=device)
    scenario_tensor = torch.as_tensor(scenario_ids, dtype=torch.long, device=device)
    with torch.no_grad():
        action = _actor_forward(bundle['actor'], obs_tensor, scenario_tensor)
        if bundle['squash']:
            low = torch.as_tensor(bundle['action_low'], dtype=torch.float32, device=device)
            high = torch.as_tensor(bundle['action_high'], dtype=torch.float32, device=device)
            action = torch.tanh(action.clamp(-3.0, 3.0)) * ((high - low) / 2.0) + ((high + low) / 2.0)
    return action.detach().cpu().numpy().astype(np.float32, copy=False)


def _collect_records(trace_paths: list[str], base_bundle: dict, candidate_bundle: dict) -> list[dict]:
    records = []
    for trace_path in trace_paths:
        payload = json.loads(Path(trace_path).read_text(encoding='utf-8'))
        for episode_index, episode in enumerate(payload.get('episode_metrics', [])):
            scenario = str(episode.get('scenario', ''))
            for sample in episode.get('trace_samples', []):
                step = int(sample.get('step', 0))
                pair_sep = float(sample.get('pairwise_min_separation', np.nan))
                for agent_id, agent in sorted((sample.get('agents') or {}).items()):
                    raw_observation = agent.get('raw_observation')
                    if raw_observation is None:
                        continue
                    raw = np.asarray(raw_observation, dtype=np.float32)
                    if raw.shape[0] != int(base_bundle['obs_dim']) or raw.shape[0] != int(candidate_bundle['obs_dim']):
                        continue
                    records.append({
                        'trace': str(trace_path),
                        'episode': int(episode_index),
                        'scenario': scenario,
                        'step': step,
                        'agent': str(agent_id),
                        'pair_sep': pair_sep,
                        'nearest_distance': float(agent.get('nearest_distance', np.nan)),
                        'distance_to_goal': float(agent.get('distance_to_goal', np.nan)),
                        'route_progress': float(agent.get('route_progress', np.nan)),
                        'cte': float(agent.get('cross_track_error', np.nan)),
                        'trace_linear': float(agent.get('final_linear_x', np.nan)),
                        'trace_omega': float(agent.get('final_angular_z', np.nan)),
                        'raw_observation': raw,
                    })
    return records


def _stats(name: str, rows: list[dict]) -> str:
    if not rows:
        return f'{name}: count=0'
    abs_linear = np.asarray([abs(row['delta_linear']) for row in rows], dtype=np.float64)
    abs_omega = np.asarray([abs(row['delta_omega']) for row in rows], dtype=np.float64)
    norm = np.asarray([row['delta_norm'] for row in rows], dtype=np.float64)
    signed_linear = np.asarray([row['delta_linear'] for row in rows], dtype=np.float64)
    signed_omega = np.asarray([row['delta_omega'] for row in rows], dtype=np.float64)
    return (
        f'{name}: count={len(rows)} '
        f'mean_abs_linear={abs_linear.mean():.6f} p95_abs_linear={np.percentile(abs_linear, 95):.6f} '
        f'mean_abs_omega={abs_omega.mean():.6f} p95_abs_omega={np.percentile(abs_omega, 95):.6f} '
        f'mean_norm={norm.mean():.6f} p95_norm={np.percentile(norm, 95):.6f} max_norm={norm.max():.6f} '
        f'mean_signed_linear={signed_linear.mean():.6f} mean_signed_omega={signed_omega.mean():.6f}'
    )


def _write_tsv(path: str, rows: list[dict]):
    fields = [
        'trace', 'episode', 'scenario', 'step', 'agent', 'pair_sep', 'nearest_distance',
        'distance_to_goal', 'route_progress', 'cte', 'trace_linear', 'trace_omega',
        'base_linear', 'base_omega', 'candidate_linear', 'candidate_omega',
        'delta_linear', 'delta_omega', 'delta_norm',
    ]
    with Path(path).open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, delimiter='\t')
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, '') for field in fields})


def main(argv=None):
    args = parse_args(argv)
    device = torch.device(args.device if args.device != 'auto' else ('cuda' if torch.cuda.is_available() else 'cpu'))
    base = _load_bundle(args.base_model, device)
    candidate = _load_bundle(args.candidate_model, device)
    records = _collect_records(args.trace_json, base, candidate)
    if not records:
        raise SystemExit('No comparable raw_observation samples found in trace JSON.')

    raw_observations = np.stack([row['raw_observation'] for row in records], axis=0)
    base_scenario_ids = np.asarray([_scenario_id(base, row['scenario']) for row in records], dtype=np.int64)
    candidate_scenario_ids = np.asarray([_scenario_id(candidate, row['scenario']) for row in records], dtype=np.int64)
    base_actions = _predict(base, raw_observations, base_scenario_ids, device)
    candidate_actions = _predict(candidate, raw_observations, candidate_scenario_ids, device)
    action_range = np.maximum(base['action_high'] - base['action_low'], 1e-6)

    rows = []
    for row, base_action, candidate_action in zip(records, base_actions, candidate_actions):
        delta = candidate_action - base_action
        normalized_delta = delta / action_range
        out = dict(row)
        out.pop('raw_observation', None)
        out.update({
            'base_linear': float(base_action[0]),
            'base_omega': float(base_action[1]),
            'candidate_linear': float(candidate_action[0]),
            'candidate_omega': float(candidate_action[1]),
            'delta_linear': float(delta[0]),
            'delta_omega': float(delta[1]),
            'delta_norm': float(np.sqrt(np.mean(normalized_delta ** 2))),
        })
        rows.append(out)

    danger_rows = [row for row in rows if np.isfinite(row['pair_sep']) and row['pair_sep'] <= float(args.danger_separation)]
    min_pair_sep = min((row['pair_sep'] for row in rows if np.isfinite(row['pair_sep'])), default=float('nan'))
    min_step = min((row['step'] for row in rows if row['pair_sep'] == min_pair_sep), default=None)
    if min_step is None:
        min_window_rows = []
    else:
        min_window_rows = [row for row in rows if abs(int(row['step']) - int(min_step)) <= 8]

    print(f'samples={len(rows)} traces={len(args.trace_json)} min_pair_sep={min_pair_sep:.6f} min_step={min_step}')
    print(_stats('all', rows))
    print(_stats(f'pair_sep<={float(args.danger_separation):.2f}', danger_rows))
    print(_stats('min_sep_window(+/-8 steps)', min_window_rows))

    top_rows = sorted(rows, key=lambda item: item['delta_norm'], reverse=True)[:max(0, int(args.top_k))]
    if top_rows:
        print('top_deltas:')
        for row in top_rows:
            print(
                f"  step={row['step']} agent={row['agent']} pair_sep={row['pair_sep']:.6f} "
                f"base=({row['base_linear']:.6f},{row['base_omega']:.6f}) "
                f"cand=({row['candidate_linear']:.6f},{row['candidate_omega']:.6f}) "
                f"delta=({row['delta_linear']:.6f},{row['delta_omega']:.6f}) norm={row['delta_norm']:.6f}"
            )

    if args.output_tsv:
        _write_tsv(args.output_tsv, rows)
        print(f'output_tsv={args.output_tsv}')


if __name__ == '__main__':
    main()