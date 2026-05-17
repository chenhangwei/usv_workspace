import argparse
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch
from torch import nn

from .multi_agent_types import AgentLocalObservation, ENCOUNTER_TYPE_COUNT
from .observation_normalizer import ObservationNormalizer
from .train_mappo_policy import (
    _actor_forward,
    _crossing_imitation_loss,
    _lagging_finish_active_mask,
    _lagging_teammate_finish_loss,
    _maybe_freeze_actor_base,
    _near_goal_finish_loss,
    _overtaking_imitation_active_mask,
    _overtaking_imitation_loss,
    _policy_anchor_loss,
    _random_clear_ahead_active_mask,
    _random_clear_ahead_loss,
    _random_cte_recovery_active_mask,
    _random_cte_recovery_loss,
    _random_deconflict_active_mask,
    _random_deconflict_loss,
    _random_goal_hold_active_mask,
    _random_goal_hold_loss,
    _random_offroute_finish_active_mask,
    _random_offroute_finish_loss,
    _random_pairwise_role_guard_active_mask,
    _random_pairwise_role_guard_loss,
    _random_role_balance_loss,
    _random_safe_finish_active_mask,
    _random_safe_finish_loss,
    _team_safety_brake_active_mask,
    _team_safety_brake_loss,
)


LOSS_SPECS = (
    ('crossing_imitation', 'crossing_imitation_weight'),
    ('overtaking_imitation', 'overtaking_imitation_weight'),
    ('near_goal_finish', 'near_goal_finish_weight'),
    ('lagging_finish', 'lagging_finish_weight'),
    ('team_safety_brake', 'team_safety_brake_weight'),
    ('random_deconflict', 'random_deconflict_weight'),
    ('random_role_balance', 'random_role_balance_weight'),
    ('random_pairwise_role_guard', 'random_pairwise_role_guard_weight'),
    ('random_safe_finish', 'random_safe_finish_weight'),
    ('random_goal_hold', 'random_goal_hold_weight'),
    ('random_offroute_finish', 'random_offroute_finish_weight'),
    ('random_cte_recovery', 'random_cte_recovery_weight'),
    ('random_clear_ahead', 'random_clear_ahead_weight'),
    ('policy_anchor', 'policy_anchor_weight'),
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            'Diagnose per-loss actor gradient norms and cosine alignment on '
            'MAPPO evaluation traces containing raw_observation samples.'
        )
    )
    parser.add_argument('--model', required=True, help='MAPPO checkpoint to diagnose.')
    parser.add_argument('--trace-json', action='append', required=True, help='Evaluation JSON with trace_samples raw_observation. Repeatable.')
    parser.add_argument('--anchor-model', help='Optional reference checkpoint used for policy_anchor loss. If omitted, policy_anchor is skipped.')
    parser.add_argument('--device', default='cpu', help='Torch device. Use auto to prefer CUDA.')
    parser.add_argument('--max-samples', type=int, default=4096, help='Maximum trace-agent samples to load after filtering.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Only include these scenarios. Repeatable.')
    parser.add_argument('--agent', action='append', dest='agents', default=None, help='Only include these agent ids. Repeatable.')
    parser.add_argument('--danger-separation', type=float, default=None, help='Only include samples with pairwise_min_separation <= this value.')
    parser.add_argument('--step-window', type=int, default=0, help='Keep +/- N steps around each episode minimum separation. 0 disables.')
    parser.add_argument('--use-end-weights', action='store_true', help='Use *_weight_end values from the checkpoint when available.')
    parser.add_argument('--set', action='append', dest='overrides', default=None, metavar='KEY=VALUE', help='Override a trainer arg used by loss masks/weights. Repeatable.')
    parser.add_argument('--all-actor-params', action='store_true', help='Ignore freeze_actor_base metadata and attribute gradients to all actor parameters.')
    parser.add_argument('--include-zero-weight', action='store_true', help='Also report losses whose configured weight is zero.')
    parser.add_argument('--output-json', help='Optional JSON report path.')
    parser.add_argument('--output-tsv', help='Optional TSV path for per-loss gradient summary.')
    parser.add_argument('--cosine-tsv', help='Optional TSV path for pairwise gradient cosines.')
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
    actor.train()
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


def _coerce_value(text: str):
    lowered = str(text).strip().lower()
    if lowered in ('true', 'yes', 'on'):
        return True
    if lowered in ('false', 'no', 'off'):
        return False
    if lowered in ('none', 'null'):
        return None
    try:
        if any(char in lowered for char in ('.', 'e')):
            return float(text)
        return int(text)
    except ValueError:
        return text


def _loss_args_from_checkpoint(checkpoint: dict, overrides: list[str] | None):
    values = dict(checkpoint)
    values.pop('actor_state_dict', None)
    values.pop('critic_state_dict', None)
    values.pop('optimizer_state_dict', None)
    values.pop('grad_scaler_state_dict', None)
    values.pop('obs_normalizer', None)
    for item in overrides or ():
        if '=' not in item:
            raise ValueError(f'--set override must be KEY=VALUE, got: {item}')
        key, value = item.split('=', 1)
        values[key.strip().replace('-', '_')] = _coerce_value(value)
    return SimpleNamespace(**values)


def _scenario_id(bundle: dict, scenario_name: str) -> int:
    try:
        return bundle['scenarios'].index(str(scenario_name))
    except ValueError:
        return 0


def _agent_index(agent_id: str) -> int:
    try:
        return max(0, int(str(agent_id).rsplit('_', 1)[-1]) - 1)
    except (TypeError, ValueError):
        return 0


def _collect_records(trace_paths: list[str], bundle: dict, args) -> list[dict]:
    scenario_filter = set(str(item) for item in args.scenarios or ())
    agent_filter = set(str(item) for item in args.agents or ())
    records = []
    for trace_path in trace_paths:
        payload = json.loads(Path(trace_path).read_text(encoding='utf-8'))
        for episode_index, episode in enumerate(payload.get('episode_metrics', [])):
            scenario = str(episode.get('scenario', ''))
            if scenario_filter and scenario not in scenario_filter:
                continue
            trace_samples = list(episode.get('trace_samples', []))
            if args.step_window and trace_samples:
                finite_pairs = [
                    (int(sample.get('step', 0)), float(sample.get('pairwise_min_separation', math.inf)))
                    for sample in trace_samples
                ]
                min_step, _ = min(finite_pairs, key=lambda item: item[1])
            else:
                min_step = None
            for sample in trace_samples:
                step = int(sample.get('step', 0))
                if min_step is not None and abs(step - min_step) > int(args.step_window):
                    continue
                pair_sep = float(sample.get('pairwise_min_separation', np.nan))
                if args.danger_separation is not None and (not np.isfinite(pair_sep) or pair_sep > float(args.danger_separation)):
                    continue
                team_mean_goal_distance = float(sample.get('team_mean_goal_distance', np.nan))
                goal_completion_ratio = float(sample.get('goal_completion_ratio', np.nan))
                for agent_id, agent in sorted((sample.get('agents') or {}).items()):
                    if agent_filter and str(agent_id) not in agent_filter:
                        continue
                    raw_observation = agent.get('raw_observation')
                    if raw_observation is None:
                        continue
                    raw = np.asarray(raw_observation, dtype=np.float32)
                    if raw.shape[0] != int(bundle['obs_dim']):
                        continue
                    records.append({
                        'trace': str(trace_path),
                        'episode': int(episode_index),
                        'scenario': scenario,
                        'scenario_id': int(_scenario_id(bundle, scenario)),
                        'step': step,
                        'agent': str(agent_id),
                        'agent_index': int(_agent_index(str(agent_id))),
                        'pair_sep': pair_sep,
                        'team_mean_goal_distance': team_mean_goal_distance,
                        'goal_completion_ratio': goal_completion_ratio,
                        'nearest_distance': float(agent.get('nearest_distance', np.nan)) if agent.get('nearest_distance') is not None else float('nan'),
                        'distance_to_goal': float(agent.get('distance_to_goal', np.nan)),
                        'route_progress': float(agent.get('route_progress', np.nan)),
                        'cte': float(agent.get('cross_track_error', np.nan)),
                        'trace_linear': float(agent.get('final_linear_x', np.nan)),
                        'trace_omega': float(agent.get('final_angular_z', np.nan)),
                        'raw_observation': raw,
                    })
                    if len(records) >= int(args.max_samples):
                        return records
    return records


def _normalize_observations(bundle: dict, raw_observations: np.ndarray) -> np.ndarray:
    if bundle['normalizer'] is None:
        return raw_observations.astype(np.float32, copy=False)
    return bundle['normalizer'].normalize(raw_observations).astype(np.float32, copy=False)


def _squash_action(action_mean, action_low_tensor, action_high_tensor):
    half = (action_high_tensor - action_low_tensor) / 2.0
    mid = (action_high_tensor + action_low_tensor) / 2.0
    return torch.tanh(action_mean.clamp(-3.0, 3.0)) * half + mid


def _forward_action(bundle: dict, obs_tensor, scenario_ids, action_low_tensor, action_high_tensor):
    action_mean = _actor_forward(bundle['actor'], obs_tensor, scenario_ids)
    if bundle['squash']:
        action_mean = _squash_action(action_mean, action_low_tensor, action_high_tensor)
    return action_mean


def _global_state_tensor(records: list[dict], checkpoint: dict, device):
    global_dim = max(5, int(checkpoint.get('global_state_size', 5)))
    state = torch.zeros((len(records), global_dim), dtype=torch.float32, device=device)
    pair_sep = np.asarray([
        row['pair_sep'] if np.isfinite(row['pair_sep']) else 0.0
        for row in records
    ], dtype=np.float32)
    team_goal = np.asarray([
        row['team_mean_goal_distance'] if np.isfinite(row['team_mean_goal_distance']) else 0.0
        for row in records
    ], dtype=np.float32)
    completion = np.asarray([
        row['goal_completion_ratio'] if np.isfinite(row['goal_completion_ratio']) else 0.0
        for row in records
    ], dtype=np.float32)
    state[:, -5] = torch.as_tensor(pair_sep, dtype=torch.float32, device=device)
    state[:, -3] = torch.as_tensor(team_goal, dtype=torch.float32, device=device)
    state[:, -1] = torch.as_tensor(completion, dtype=torch.float32, device=device)
    return state


def _active_ratio(mask) -> float:
    if mask is None or int(mask.numel()) == 0:
        return 0.0
    return float(mask.to(dtype=torch.float32).mean().detach().cpu().item())


def _zero_actor_grads(actor):
    for parameter in actor.parameters():
        parameter.grad = None


def _flat_grad(actor, parameters: list[torch.nn.Parameter]) -> torch.Tensor:
    chunks = []
    for parameter in parameters:
        if parameter.grad is None:
            chunks.append(torch.zeros(parameter.numel(), dtype=torch.float32, device='cpu'))
        else:
            chunks.append(parameter.grad.detach().float().reshape(-1).cpu())
    if not chunks:
        return torch.zeros(0, dtype=torch.float32)
    return torch.cat(chunks)


def _grad_stats(vector: torch.Tensor) -> dict:
    if vector.numel() == 0:
        return {'grad_norm': 0.0, 'grad_l2_mean': 0.0, 'grad_linf': 0.0}
    abs_vector = torch.abs(vector)
    return {
        'grad_norm': float(torch.linalg.vector_norm(vector).item()),
        'grad_l2_mean': float(torch.sqrt(torch.mean(vector * vector)).item()),
        'grad_linf': float(abs_vector.max().item()),
    }


def _cosine(left: torch.Tensor, right: torch.Tensor) -> float:
    left_norm = torch.linalg.vector_norm(left)
    right_norm = torch.linalg.vector_norm(right)
    denom = float((left_norm * right_norm).item())
    if denom <= 1e-12:
        return float('nan')
    return float(torch.dot(left, right).item() / denom)


def _weight_for(name: str, weight_attr: str, loss_args, use_end: bool) -> float:
    if use_end:
        end_value = getattr(loss_args, f'{weight_attr}_end', None)
        if end_value is not None:
            return max(0.0, float(end_value))
    return max(0.0, float(getattr(loss_args, weight_attr, 0.0)))


def _compute_mask(name, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, agent_indices):
    if name == 'overtaking_imitation':
        return _overtaking_imitation_active_mask(torch, raw_obs, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    if name in ('random_deconflict', 'random_role_balance'):
        return _random_deconflict_active_mask(torch, raw_obs, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    if name == 'random_pairwise_role_guard':
        return _random_pairwise_role_guard_active_mask(torch, raw_obs, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    if name == 'random_safe_finish':
        return _random_safe_finish_active_mask(torch, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    if name == 'random_goal_hold':
        return _random_goal_hold_active_mask(torch, raw_obs, scenario_ids, scenario_to_index, loss_args)
    if name == 'random_offroute_finish':
        return _random_offroute_finish_active_mask(torch, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    if name == 'random_cte_recovery':
        return _random_cte_recovery_active_mask(torch, raw_obs, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    if name == 'random_clear_ahead':
        return _random_clear_ahead_active_mask(torch, raw_obs, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    if name == 'team_safety_brake':
        return _team_safety_brake_active_mask(torch, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    if name == 'lagging_finish':
        return _lagging_finish_active_mask(torch, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, agent_indices=agent_indices)
    return None


def _compute_loss(
    name,
    action_mean,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index,
    loss_args,
    action_low_tensor,
    action_high_tensor,
    agent_indices,
    anchor_action_mean=None,
):
    if name == 'crossing_imitation':
        return _crossing_imitation_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor)
    if name == 'overtaking_imitation':
        return _overtaking_imitation_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'near_goal_finish':
        return _near_goal_finish_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor)
    if name == 'lagging_finish':
        return _lagging_teammate_finish_loss(torch, action_mean, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'team_safety_brake':
        return _team_safety_brake_loss(torch, action_mean, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'random_deconflict':
        return _random_deconflict_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'random_role_balance':
        return _random_role_balance_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'random_pairwise_role_guard':
        return _random_pairwise_role_guard_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'random_safe_finish':
        return _random_safe_finish_loss(torch, action_mean, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'random_goal_hold':
        return _random_goal_hold_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor)
    if name == 'random_offroute_finish':
        return _random_offroute_finish_loss(torch, action_mean, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'random_cte_recovery':
        return _random_cte_recovery_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'random_clear_ahead':
        return _random_clear_ahead_loss(torch, action_mean, raw_obs, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    if name == 'policy_anchor':
        if anchor_action_mean is None:
            return action_mean.new_zeros(())
        return _policy_anchor_loss(torch, action_mean, anchor_action_mean, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, action_low_tensor, action_high_tensor, agent_indices=agent_indices)
    raise KeyError(name)


def _mask_overlap_report(masks: dict[str, torch.Tensor | None]) -> list[dict]:
    rows = []
    names = [name for name, mask in masks.items() if mask is not None and int(mask.numel()) > 0]
    for left_index, left_name in enumerate(names):
        left = masks[left_name]
        left_count = int(left.sum().detach().cpu().item())
        for right_name in names[left_index + 1:]:
            right = masks[right_name]
            both = left & right
            overlap = int(both.sum().detach().cpu().item())
            rows.append({
                'left': left_name,
                'right': right_name,
                'overlap_count': overlap,
                'overlap_ratio_all': float(overlap / max(1, int(left.numel()))),
                'overlap_ratio_left': float(overlap / max(1, left_count)),
            })
    return rows


def _write_tsv(path: str, rows: list[dict], fields: list[str]):
    with Path(path).open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, delimiter='\t')
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, '') for field in fields})


def main(argv=None):
    args = parse_args(argv)
    device = torch.device(args.device if args.device != 'auto' else ('cuda' if torch.cuda.is_available() else 'cpu'))
    bundle = _load_bundle(args.model, device)
    checkpoint = bundle['checkpoint']
    loss_args = _loss_args_from_checkpoint(checkpoint, args.overrides)
    if not bool(args.all_actor_params) and bool(getattr(loss_args, 'freeze_actor_base', False)):
        try:
            _maybe_freeze_actor_base(bundle['actor'], loss_args)
        except Exception as exc:  # pragma: no cover - diagnostic should remain usable on old checkpoints.
            print(f'warning: could not mimic freeze_actor_base; using all actor params: {exc}')
            for parameter in bundle['actor'].parameters():
                parameter.requires_grad = True

    records = _collect_records(args.trace_json, bundle, args)
    if not records:
        raise SystemExit('No matching raw_observation samples found. Re-run eval with --trace-raw-observation or event/collision raw tracing.')

    raw_np = np.stack([row['raw_observation'] for row in records], axis=0).astype(np.float32, copy=False)
    obs_np = _normalize_observations(bundle, raw_np)
    scenario_ids_np = np.asarray([row['scenario_id'] for row in records], dtype=np.int64)
    agent_indices_np = np.asarray([row['agent_index'] for row in records], dtype=np.int64)
    raw_obs = torch.as_tensor(raw_np, dtype=torch.float32, device=device)
    obs = torch.as_tensor(obs_np, dtype=torch.float32, device=device)
    scenario_ids = torch.as_tensor(scenario_ids_np, dtype=torch.long, device=device)
    agent_indices = torch.as_tensor(agent_indices_np, dtype=torch.long, device=device)
    global_state = _global_state_tensor(records, checkpoint, device)
    action_low_tensor = torch.as_tensor(bundle['action_low'], dtype=torch.float32, device=device)
    action_high_tensor = torch.as_tensor(bundle['action_high'], dtype=torch.float32, device=device)
    scenario_to_index = {name: index for index, name in enumerate(bundle['scenarios'])}

    anchor_action_mean = None
    if args.anchor_model:
        anchor_bundle = _load_bundle(args.anchor_model, device)
        if int(anchor_bundle['obs_dim']) != int(bundle['obs_dim']):
            print('warning: anchor-model obs dim differs; policy_anchor loss skipped.')
        else:
            anchor_obs_np = _normalize_observations(anchor_bundle, raw_np)
            anchor_obs = torch.as_tensor(anchor_obs_np, dtype=torch.float32, device=device)
            anchor_scenario_ids_np = np.asarray([_scenario_id(anchor_bundle, row['scenario']) for row in records], dtype=np.int64)
            anchor_scenario_ids = torch.as_tensor(anchor_scenario_ids_np, dtype=torch.long, device=device)
            anchor_low = torch.as_tensor(anchor_bundle['action_low'], dtype=torch.float32, device=device)
            anchor_high = torch.as_tensor(anchor_bundle['action_high'], dtype=torch.float32, device=device)
            with torch.no_grad():
                anchor_action_mean = _forward_action(anchor_bundle, anchor_obs, anchor_scenario_ids, anchor_low, anchor_high)

    action_mean = _forward_action(bundle, obs, scenario_ids, action_low_tensor, action_high_tensor)
    parameters = [parameter for parameter in bundle['actor'].parameters() if parameter.requires_grad]
    parameter_count = int(sum(parameter.numel() for parameter in parameters))
    if parameter_count <= 0:
        raise SystemExit('No trainable actor parameters selected for gradient attribution.')

    masks = {}
    vectors = {}
    summary_rows = []
    for name, weight_attr in LOSS_SPECS:
        if name == 'policy_anchor' and anchor_action_mean is None:
            continue
        weight = _weight_for(name, weight_attr, loss_args, bool(args.use_end_weights))
        if weight <= 0.0 and not bool(args.include_zero_weight):
            continue
        mask = _compute_mask(name, raw_obs, global_state, scenario_ids, scenario_to_index, loss_args, agent_indices)
        masks[name] = mask
        loss = _compute_loss(
            name,
            action_mean,
            raw_obs,
            global_state,
            scenario_ids,
            scenario_to_index,
            loss_args,
            action_low_tensor,
            action_high_tensor,
            agent_indices,
            anchor_action_mean=anchor_action_mean,
        )
        raw_loss = float(loss.detach().cpu().item()) if loss.numel() == 1 else float('nan')
        weighted_loss = loss * float(weight)
        _zero_actor_grads(bundle['actor'])
        if weighted_loss.requires_grad and float(weight) != 0.0:
            weighted_loss.backward(retain_graph=True)
            vector = _flat_grad(bundle['actor'], parameters)
        else:
            vector = torch.zeros(parameter_count, dtype=torch.float32)
        vectors[name] = vector
        stats = _grad_stats(vector)
        summary_rows.append({
            'loss': name,
            'weight_attr': weight_attr,
            'weight': float(weight),
            'raw_loss': raw_loss,
            'weighted_loss': float(raw_loss * weight) if np.isfinite(raw_loss) else float('nan'),
            'active_ratio': _active_ratio(mask),
            **stats,
        })
    _zero_actor_grads(bundle['actor'])

    cosine_rows = []
    names = list(vectors.keys())
    for left_index, left_name in enumerate(names):
        for right_name in names[left_index + 1:]:
            cosine_rows.append({
                'left': left_name,
                'right': right_name,
                'cosine': _cosine(vectors[left_name], vectors[right_name]),
            })

    overlap_rows = _mask_overlap_report(masks)
    report = {
        'model': str(args.model),
        'anchor_model': str(args.anchor_model) if args.anchor_model else None,
        'trace_json': [str(path) for path in args.trace_json],
        'sample_count': len(records),
        'trainable_actor_parameters': parameter_count,
        'scenario_counts': {scenario: sum(1 for row in records if row['scenario'] == scenario) for scenario in sorted({row['scenario'] for row in records})},
        'agent_counts': {agent: sum(1 for row in records if row['agent'] == agent) for agent in sorted({row['agent'] for row in records})},
        'losses': summary_rows,
        'cosines': cosine_rows,
        'mask_overlaps': overlap_rows,
        'note': 'PPO policy/value gradients require rollout old_log_probs, actions, returns, and advantages; eval JSON traces only support trainer-side auxiliary loss attribution.',
    }

    print(f"samples={len(records)} trainable_actor_parameters={parameter_count} losses={len(summary_rows)}")
    print('loss_gradients:')
    for row in sorted(summary_rows, key=lambda item: item['grad_norm'], reverse=True):
        print(
            f"  {row['loss']}: weight={row['weight']:.6g} active={row['active_ratio']:.3f} "
            f"raw={row['raw_loss']:.6f} weighted={row['weighted_loss']:.6f} "
            f"grad_norm={row['grad_norm']:.6e} grad_linf={row['grad_linf']:.6e}"
        )
    if cosine_rows:
        print('strongest_negative_cosines:')
        for row in sorted([item for item in cosine_rows if np.isfinite(item['cosine'])], key=lambda item: item['cosine'])[:8]:
            print(f"  {row['left']} vs {row['right']}: cosine={row['cosine']:.6f}")
    if overlap_rows:
        print('largest_mask_overlaps:')
        for row in sorted(overlap_rows, key=lambda item: item['overlap_ratio_all'], reverse=True)[:8]:
            print(
                f"  {row['left']} & {row['right']}: count={row['overlap_count']} "
                f"all={row['overlap_ratio_all']:.3f} left={row['overlap_ratio_left']:.3f}"
            )

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'output_json={output_path}')
    if args.output_tsv:
        _write_tsv(
            args.output_tsv,
            summary_rows,
            ['loss', 'weight_attr', 'weight', 'raw_loss', 'weighted_loss', 'active_ratio', 'grad_norm', 'grad_l2_mean', 'grad_linf'],
        )
        print(f'output_tsv={args.output_tsv}')
    if args.cosine_tsv:
        _write_tsv(args.cosine_tsv, cosine_rows, ['left', 'right', 'cosine'])
        print(f'cosine_tsv={args.cosine_tsv}')


if __name__ == '__main__':
    main()