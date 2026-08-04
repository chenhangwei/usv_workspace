import argparse
import json
import math
import os
from pathlib import Path
import random

import numpy as np

from .config import ActionBounds, RewardConfig
from .multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig
from .multi_agent_scenarios import MultiAgentScenarioFactory
from .multi_agent_types import (
    AgentLocalObservation,
    ENCOUNTER_TYPE_COUNT,
    NEIGHBOR_FEATURE_COUNT,
    ENC_HEAD_ON,
    ENC_CROSSING,
    ENC_OVERTAKING,
    classify_encounter_role,
)
from .observation_normalizer import ObservationNormalizer
from .policies import load_policy


def _infer_obs_layout_for_migration(obs_dim: int) -> tuple[int, int, int] | None:
    new_ego_dim = AgentLocalObservation.ego_feature_size()
    for ego_dim in (new_ego_dim, 17, 12, 11, 10):
        for neighbor_dim in (NEIGHBOR_FEATURE_COUNT, 6):
            neighbor_width = int(obs_dim) - int(ego_dim) - ENCOUNTER_TYPE_COUNT
            if neighbor_width >= 0 and neighbor_width % int(neighbor_dim) == 0:
                return int(ego_dim), int(neighbor_dim), int(neighbor_width // int(neighbor_dim))
    return None


def _migrate_obs_stats_by_layout(mean: np.ndarray, var: np.ndarray, new_obs_dim: int) -> tuple[np.ndarray, np.ndarray] | None:
    old_layout = _infer_obs_layout_for_migration(int(mean.shape[0]))
    new_layout = _infer_obs_layout_for_migration(int(new_obs_dim))
    if old_layout is None or new_layout is None or old_layout[2] != new_layout[2]:
        return None
    old_ego_dim, old_neighbor_dim, neighbor_slots = old_layout
    new_ego_dim, new_neighbor_dim, _ = new_layout
    mean_parts = []
    var_parts = []
    ego_cols = min(old_ego_dim, new_ego_dim)
    mean_parts.append(mean[:ego_cols])
    var_parts.append(var[:ego_cols])
    if new_ego_dim > ego_cols:
        mean_parts.append(np.zeros(new_ego_dim - ego_cols, dtype=np.float64))
        var_parts.append(np.ones(new_ego_dim - ego_cols, dtype=np.float64))
    old_pos = old_ego_dim
    for _ in range(neighbor_slots):
        neighbor_cols = min(old_neighbor_dim, new_neighbor_dim)
        mean_parts.append(mean[old_pos:old_pos + neighbor_cols])
        var_parts.append(var[old_pos:old_pos + neighbor_cols])
        if new_neighbor_dim > neighbor_cols:
            mean_parts.append(np.zeros(new_neighbor_dim - neighbor_cols, dtype=np.float64))
            var_parts.append(np.ones(new_neighbor_dim - neighbor_cols, dtype=np.float64))
        old_pos += old_neighbor_dim
    mean_parts.append(mean[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
    var_parts.append(var[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
    return np.concatenate(mean_parts), np.concatenate(var_parts)


def _adapt_observation_vector_dim(observation: np.ndarray, target_dim: int) -> np.ndarray:
    obs = np.asarray(observation, dtype=np.float32)
    current_dim = int(obs.shape[-1]) if obs.ndim > 0 else 0
    target_dim = int(target_dim)
    if current_dim == target_dim:
        return obs
    current_layout = _infer_obs_layout_for_migration(current_dim)
    target_layout = _infer_obs_layout_for_migration(target_dim)
    if current_layout is not None and target_layout is not None and current_layout[2] == target_layout[2]:
        current_ego_dim, current_neighbor_dim, neighbor_slots = current_layout
        target_ego_dim, target_neighbor_dim, _ = target_layout
        parts = []
        ego_cols = min(current_ego_dim, target_ego_dim)
        parts.append(obs[..., :ego_cols])
        if target_ego_dim > ego_cols:
            pad_shape = obs.shape[:-1] + (target_ego_dim - ego_cols,)
            parts.append(np.zeros(pad_shape, dtype=np.float32))
        old_pos = current_ego_dim
        for _ in range(neighbor_slots):
            neighbor_cols = min(current_neighbor_dim, target_neighbor_dim)
            parts.append(obs[..., old_pos:old_pos + neighbor_cols])
            if target_neighbor_dim > neighbor_cols:
                pad_shape = obs.shape[:-1] + (target_neighbor_dim - neighbor_cols,)
                parts.append(np.zeros(pad_shape, dtype=np.float32))
            old_pos += current_neighbor_dim
        parts.append(obs[..., old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
        return np.concatenate(parts, axis=-1).astype(np.float32, copy=False)
    if target_dim > current_dim:
        pad_shape = obs.shape[:-1] + (target_dim - current_dim,)
        return np.concatenate([obs, np.zeros(pad_shape, dtype=np.float32)], axis=-1)
    return obs[..., :target_dim].astype(np.float32, copy=False)


def _migrate_obs_normalizer_state_for_obs_dim(state: dict, new_obs_dim: int) -> dict:
    mean = np.asarray(state.get('mean', ()), dtype=np.float64)
    var = np.asarray(state.get('var', ()), dtype=np.float64)
    if mean.ndim != 1 or var.ndim != 1 or mean.shape != var.shape:
        return state
    old_obs_dim = int(mean.shape[0])
    new_obs_dim = int(new_obs_dim)
    if old_obs_dim == new_obs_dim or old_obs_dim <= 0 or new_obs_dim <= old_obs_dim:
        return state

    delta = new_obs_dim - old_obs_dim
    layout_stats = _migrate_obs_stats_by_layout(mean, var, new_obs_dim)
    if layout_stats is not None:
        migrated = dict(state)
        migrated['mean'], migrated['var'] = layout_stats
        print(f'Migrated eval observation normalizer: {old_obs_dim} -> {new_obs_dim} dims.', flush=True)
        return migrated
    new_ego_dim = AgentLocalObservation.ego_feature_size()
    old_neighbor_dim = 6
    old_neighbor_width = old_obs_dim - new_ego_dim - ENCOUNTER_TYPE_COUNT
    new_neighbor_width = new_obs_dim - new_ego_dim - ENCOUNTER_TYPE_COUNT
    can_insert_in_neighbors = (
        old_neighbor_width >= 0
        and new_neighbor_width >= 0
        and old_neighbor_width % old_neighbor_dim == 0
    )
    old_ego_dim = new_ego_dim - delta
    can_insert_in_ego = (
        old_ego_dim > 0
        and old_obs_dim >= old_ego_dim + ENCOUNTER_TYPE_COUNT
        and (old_obs_dim - old_ego_dim - ENCOUNTER_TYPE_COUNT) % 6 == 0
    )
    migrated = dict(state)
    if can_insert_in_neighbors:
        max_neighbors = old_neighbor_width // old_neighbor_dim
        if new_neighbor_width == max_neighbors * NEIGHBOR_FEATURE_COUNT and NEIGHBOR_FEATURE_COUNT > old_neighbor_dim:
            mean_parts = [mean[:new_ego_dim]]
            var_parts = [var[:new_ego_dim]]
            old_pos = new_ego_dim
            for _ in range(max_neighbors):
                mean_parts.append(mean[old_pos:old_pos + old_neighbor_dim])
                var_parts.append(var[old_pos:old_pos + old_neighbor_dim])
                mean_parts.append(np.zeros(NEIGHBOR_FEATURE_COUNT - old_neighbor_dim, dtype=np.float64))
                var_parts.append(np.ones(NEIGHBOR_FEATURE_COUNT - old_neighbor_dim, dtype=np.float64))
                old_pos += old_neighbor_dim
            mean_parts.append(mean[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
            var_parts.append(var[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
            migrated['mean'] = np.concatenate(mean_parts)
            migrated['var'] = np.concatenate(var_parts)
        else:
            migrated['mean'] = np.concatenate([mean, np.zeros(delta, dtype=np.float64)])
            migrated['var'] = np.concatenate([var, np.ones(delta, dtype=np.float64)])
    elif can_insert_in_ego:
        migrated['mean'] = np.concatenate([
            mean[:old_ego_dim],
            np.zeros(delta, dtype=np.float64),
            mean[old_ego_dim:],
        ])
        migrated['var'] = np.concatenate([
            var[:old_ego_dim],
            np.ones(delta, dtype=np.float64),
            var[old_ego_dim:],
        ])
    else:
        migrated['mean'] = np.concatenate([mean, np.zeros(delta, dtype=np.float64)])
        migrated['var'] = np.concatenate([var, np.ones(delta, dtype=np.float64)])
    print(f'Migrated eval observation normalizer: {old_obs_dim} -> {new_obs_dim} dims.', flush=True)
    return migrated


def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate an RL policy in the multi-USV environment.')
    parser.add_argument('--policy', choices=['auto', 'mappo', 'mappo_router', 'bc', 'zero'], default='auto', help='Policy backend.')
    parser.add_argument('--model', help='Policy model path (.pt for MAPPO, .npz for BC). Not required for zero policy.')
    parser.add_argument('--episodes', type=int, default=6, help='Evaluation episodes.')
    parser.add_argument('--steps-per-episode', type=int, default=160, help='Maximum steps per episode.')
    parser.add_argument('--device', default='cpu', help='Torch device.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name. Repeatable.')
    parser.add_argument('--num-agents', type=int, default=5, help='Controlled agent count for BC/zero evaluation.')
    parser.add_argument('--rl-control-mode', choices=['auto', 'pure'], default='pure', help='Pure final-command control mode used during evaluation.')
    parser.add_argument('--action-mode', choices=['auto', 'full'], default='full', help='Action representation for BC/zero evaluation.')
    parser.add_argument('--max-neighbors', default='auto', help='Neighbor slots encoded during BC/zero evaluation. Use auto or an integer such as 4.')
    parser.add_argument('--max-agents', type=int, help='Maximum agents encoded in the global state for BC/zero evaluation.')
    parser.add_argument('--episode-timeout', type=float, help='Optional override for environment episode timeout.')
    parser.add_argument('--no-progress-timeout', type=float, help='Optional override for no-progress timeout.')
    parser.add_argument('--seed', type=int, help='Optional base seed for paired/reproducible evaluation episodes.')
    parser.add_argument('--output-json', help='Optional JSON output path.')
    parser.add_argument('--trace-stride', type=int, default=0, help='If >0, store per-agent trajectory diagnostics every N steps in episode_metrics.')
    parser.add_argument('--trace-raw-observation', action='store_true', help='When tracing, include each agent raw observation vector for offline active-slice fitting.')
    parser.add_argument('--trace-event-separation', type=float, default=0.0, help='If >0, record a short diagnostic trace when pairwise separation is at or below this value.')
    parser.add_argument('--trace-event-window', type=int, default=12, help='Number of post-trigger steps to keep for event-triggered traces.')
    parser.add_argument('--trace-event-raw-observation', action='store_true', help='When event tracing, include raw observation vectors only in event-triggered trace samples.')
    parser.add_argument('--trace-collision-raw-observation', action='store_true', help='On collision, store one raw-observation diagnostic sample at the collision step.')
    return parser.parse_args()


class ZeroPolicy:
    def __init__(self, action_dim: int = 2, obs_dim: int | None = None):
        self.action_dim = int(action_dim)
        self.obs_dim = obs_dim

    def predict(self, observation: np.ndarray) -> np.ndarray:
        return np.zeros(self.action_dim, dtype=np.float32)


class MappoActorPolicy:
    def __init__(self, checkpoint: dict, *, device: str = 'cpu'):
        try:
            import torch
            from torch import nn
        except ImportError as exc:
            raise RuntimeError('torch is required for MAPPO evaluation.') from exc

        hidden_sizes = tuple(int(value) for value in checkpoint.get('hidden_sizes', [128, 128]))
        action_dim = int(checkpoint['action_dim'])
        obs_dim = int(checkpoint['local_observation_size'])

        if checkpoint.get('neighbor_attention', False):
            from usv_rl.neighbor_attention import build_attention_actor_from_checkpoint
            actor = build_attention_actor_from_checkpoint(
                checkpoint, nn, torch, torch.device(device),
            )
            obs_dim = int(getattr(actor, 'obs_dim', obs_dim))
        else:
            layers = []
            current_dim = obs_dim
            for hidden_size in hidden_sizes:
                layers.append(nn.Linear(current_dim, hidden_size))
                layers.append(nn.Tanh())
                current_dim = hidden_size
            layers.append(nn.Linear(current_dim, action_dim))
            actor = nn.Sequential(*layers)
            actor.load_state_dict(checkpoint['actor_state_dict'])
            actor.eval()

        self._torch = torch
        self._device = torch.device(device)
        self._actor = actor.to(self._device)
        self.action_dim = action_dim
        self.obs_dim = obs_dim
        self._squash_actions = bool(checkpoint.get('squash_actions', False))
        if self._squash_actions:
            _low = checkpoint.get('action_low', [0.0, -0.4])
            _high = checkpoint.get('action_high', [0.4, 0.4])
            self._action_low = torch.as_tensor(_low, dtype=torch.float32, device=self._device)
            self._action_high = torch.as_tensor(_high, dtype=torch.float32, device=self._device)

        self._obs_normalizer = None
        if checkpoint.get('normalize_observations') and 'obs_normalizer' in checkpoint:
            self._obs_normalizer = ObservationNormalizer(obs_dim)
            norm_state = _migrate_obs_normalizer_state_for_obs_dim(checkpoint['obs_normalizer'], obs_dim)
            self._obs_normalizer.load_state_dict(norm_state)

    def set_active_scenario(self, scenario_name: str | None) -> None:
        if hasattr(self._actor, 'set_active_scenario_name'):
            self._actor.set_active_scenario_name(scenario_name)

    def predict(self, observation: np.ndarray) -> np.ndarray:
        observation = _adapt_observation_vector_dim(observation, self.obs_dim)
        if self._obs_normalizer is not None:
            observation = self._obs_normalizer.normalize(observation)
        obs_tensor = self._torch.as_tensor(observation, dtype=self._torch.float32, device=self._device).unsqueeze(0)
        with self._torch.no_grad():
            raw = self._actor(obs_tensor)
            if self._squash_actions:
                _half = (self._action_high - self._action_low) / 2.0
                _mid = (self._action_high + self._action_low) / 2.0
                action = (self._torch.tanh(raw) * _half + _mid).cpu().numpy()[0]
            else:
                action = raw.cpu().numpy()[0]
        return np.asarray(action, dtype=np.float32)


class WeightedBlendPolicy:
    def __init__(self, weighted_policies: list[tuple[MappoActorPolicy, float]]):
        if not weighted_policies:
            raise RuntimeError('WeightedBlendPolicy requires at least one expert.')

        reference_policy = weighted_policies[0][0]
        total_weight = 0.0
        self._weighted_policies: list[tuple[MappoActorPolicy, float]] = []
        for policy, weight in weighted_policies:
            if policy.action_dim != reference_policy.action_dim or policy.obs_dim != reference_policy.obs_dim:
                raise RuntimeError('All blended experts must share the same obs_dim/action_dim.')
            scalar_weight = float(weight)
            if scalar_weight <= 0.0:
                raise RuntimeError('Blend weights must be positive.')
            total_weight += scalar_weight
            self._weighted_policies.append((policy, scalar_weight))

        self._weighted_policies = [
            (policy, weight / total_weight)
            for policy, weight in self._weighted_policies
        ]
        self.action_dim = reference_policy.action_dim
        self.obs_dim = reference_policy.obs_dim

    def set_active_scenario(self, scenario_name: str | None) -> None:
        for policy, _ in self._weighted_policies:
            policy.set_active_scenario(scenario_name)

    def predict(self, observation: np.ndarray) -> np.ndarray:
        blended = np.zeros(self.action_dim, dtype=np.float32)
        for policy, weight in self._weighted_policies:
            blended += np.asarray(policy.predict(observation), dtype=np.float32) * np.float32(weight)
        return blended


class GatedBlendPolicy:
    def __init__(
        self,
        base_policy: MappoActorPolicy,
        safety_policy: MappoActorPolicy,
        gate_config: dict,
        critical_safety_policy: MappoActorPolicy | None = None,
    ):
        if base_policy.action_dim != safety_policy.action_dim or base_policy.obs_dim != safety_policy.obs_dim:
            raise RuntimeError('Gated blend experts must share the same obs_dim/action_dim.')
        if critical_safety_policy is not None and (
            base_policy.action_dim != critical_safety_policy.action_dim or base_policy.obs_dim != critical_safety_policy.obs_dim
        ):
            raise RuntimeError('Critical gated blend expert must share the same obs_dim/action_dim.')
        self._base_policy = base_policy
        self._safety_policy = safety_policy
        self._critical_safety_policy = critical_safety_policy
        self._blend = float(gate_config.get('blend', 0.35))
        self._yield_blend = float(gate_config.get('yield_blend', self._blend))
        self._standon_blend = float(gate_config.get('standon_blend', self._blend))
        self._yield_linear_blend = float(gate_config.get('yield_linear_blend', gate_config.get('linear_blend', self._yield_blend)))
        self._standon_linear_blend = float(gate_config.get('standon_linear_blend', gate_config.get('linear_blend', self._standon_blend)))
        self._yield_angular_blend = float(gate_config.get('yield_angular_blend', gate_config.get('angular_blend', self._yield_blend)))
        self._standon_angular_blend = float(gate_config.get('standon_angular_blend', gate_config.get('angular_blend', self._standon_blend)))
        self._max_separation = float(gate_config.get('max_separation', 1.35))
        self._critical_separation = float(gate_config.get('critical_separation', 0.85))
        self._min_route_progress = float(gate_config.get('min_route_progress', 0.0))
        self._max_route_progress = float(gate_config.get('max_route_progress', 1.1))
        self._min_abs_cte = float(gate_config.get('min_abs_cte', 0.0))
        self._early_min_abs_cte = float(gate_config.get('early_min_abs_cte', 0.0))
        self._yield_only = bool(gate_config.get('yield_only', False))
        self._standon_only = bool(gate_config.get('standon_only', False))
        self.action_dim = base_policy.action_dim
        self.obs_dim = base_policy.obs_dim

    def set_active_scenario(self, scenario_name: str | None) -> None:
        self._base_policy.set_active_scenario(scenario_name)
        self._safety_policy.set_active_scenario(scenario_name)
        if self._critical_safety_policy is not None:
            self._critical_safety_policy.set_active_scenario(scenario_name)

    def _nearest_neighbor(self, observation: np.ndarray) -> tuple[float, float]:
        ego_dim = AgentLocalObservation.ego_feature_size()
        available = int(observation.shape[0]) - ego_dim - ENCOUNTER_TYPE_COUNT
        slots = max(0, available // NEIGHBOR_FEATURE_COUNT)
        nearest_distance = float('inf')
        nearest_priority_delta = 0.0
        for slot in range(slots):
            start = ego_dim + slot * NEIGHBOR_FEATURE_COUNT
            block = observation[start:start + NEIGHBOR_FEATURE_COUNT]
            if block.shape[0] < NEIGHBOR_FEATURE_COUNT:
                continue
            distance = float(block[4])
            if distance <= 1e-6 or distance >= nearest_distance:
                continue
            nearest_distance = distance
            nearest_priority_delta = float(block[9])
        return nearest_distance, nearest_priority_delta

    def _gate_weights(self, observation: np.ndarray) -> tuple[float, float]:
        route_progress = float(observation[12]) if observation.shape[0] > 12 else 0.0
        abs_cte = abs(float(observation[11])) if observation.shape[0] > 11 else 0.0
        if route_progress < self._min_route_progress or route_progress > self._max_route_progress:
            return 0.0, 0.0
        if abs_cte < self._min_abs_cte:
            return 0.0, 0.0
        nearest_distance, priority_delta = self._nearest_neighbor(observation)
        if nearest_distance > self._max_separation:
            return 0.0, 0.0
        if nearest_distance > self._critical_separation and abs_cte < self._early_min_abs_cte:
            return 0.0, 0.0
        is_yield = priority_delta < 0.0
        if self._yield_only and not is_yield:
            return 0.0, 0.0
        if self._standon_only and is_yield:
            return 0.0, 0.0
        linear_blend = self._yield_linear_blend if is_yield else self._standon_linear_blend
        angular_blend = self._yield_angular_blend if is_yield else self._standon_angular_blend
        if nearest_distance <= self._critical_separation:
            scale = 1.0
        else:
            width = max(1e-6, self._max_separation - self._critical_separation)
            scale = 1.0 - ((nearest_distance - self._critical_separation) / width)
        return (
            float(np.clip(linear_blend * scale, 0.0, 1.0)),
            float(np.clip(angular_blend * scale, 0.0, 1.0)),
        )

    def _gate_weight(self, observation: np.ndarray) -> float:
        linear_weight, angular_weight = self._gate_weights(observation)
        return max(linear_weight, angular_weight)

    def predict(self, observation: np.ndarray) -> np.ndarray:
        base_action = np.asarray(self._base_policy.predict(observation), dtype=np.float32)
        linear_weight, angular_weight = self._gate_weights(np.asarray(observation, dtype=np.float32))
        if max(linear_weight, angular_weight) <= 0.0:
            return base_action
        nearest_distance, _ = self._nearest_neighbor(np.asarray(observation, dtype=np.float32))
        safety_policy = self._safety_policy
        if self._critical_safety_policy is not None and nearest_distance <= self._critical_separation:
            safety_policy = self._critical_safety_policy
        safety_action = np.asarray(safety_policy.predict(observation), dtype=np.float32)
        component_weights = np.full(self.action_dim, angular_weight, dtype=np.float32)
        if self.action_dim > 0:
            component_weights[0] = np.float32(linear_weight)
        return ((1.0 - component_weights) * base_action + component_weights * safety_action).astype(np.float32)


class ScenarioRouterPolicy:
    def __init__(self, router_config: dict, *, device: str = 'cpu'):
        raw_scenario_models = router_config.get('scenario_models') or {}
        if not isinstance(raw_scenario_models, dict):
            raise RuntimeError('MAPPO router config requires scenario_models to be a mapping when provided.')

        raw_scenario_blends = router_config.get('scenario_blends') or {}
        if not isinstance(raw_scenario_blends, dict):
            raise RuntimeError('MAPPO router config requires scenario_blends to be a mapping when provided.')

        raw_scenario_gated_blends = router_config.get('scenario_gated_blends') or {}
        if not isinstance(raw_scenario_gated_blends, dict):
            raise RuntimeError('MAPPO router config requires scenario_gated_blends to be a mapping when provided.')

        if not raw_scenario_models and not raw_scenario_blends and not raw_scenario_gated_blends:
            raise RuntimeError('MAPPO router config requires at least one scenario_models, scenario_blends, or scenario_gated_blends entry.')

        overlapping_scenarios = (
            (set(raw_scenario_models) & set(raw_scenario_blends))
            | (set(raw_scenario_models) & set(raw_scenario_gated_blends))
            | (set(raw_scenario_blends) & set(raw_scenario_gated_blends))
        )
        if overlapping_scenarios:
            joined = ', '.join(sorted(str(value) for value in overlapping_scenarios))
            raise RuntimeError(f'Scenarios cannot be defined in multiple router policy maps: {joined}')

        self._scenario_models = {
            str(scenario_name): str(model_path)
            for scenario_name, model_path in raw_scenario_models.items()
        }
        self._scenario_blends: dict[str, list[tuple[str, float]]] = {}
        for scenario_name, raw_specs in raw_scenario_blends.items():
            if not isinstance(raw_specs, list) or not raw_specs:
                raise RuntimeError(
                    'Each scenario_blends entry must be a non-empty list of objects '
                    f'with model/model_path and weight fields. Problem scenario: {scenario_name}'
                )
            specs: list[tuple[str, float]] = []
            for raw_spec in raw_specs:
                if not isinstance(raw_spec, dict):
                    raise RuntimeError(
                        'Blend entries must be objects with model/model_path and weight fields. '
                        f'Problem scenario: {scenario_name}'
                    )
                model_path = str(raw_spec.get('model') or raw_spec.get('model_path') or '').strip()
                if not model_path:
                    raise RuntimeError(
                        'Blend entries must define model or model_path. '
                        f'Problem scenario: {scenario_name}'
                    )
                weight = float(raw_spec.get('weight', 1.0))
                if weight <= 0.0:
                    raise RuntimeError(
                        f'Blend weights must be positive. Problem scenario: {scenario_name}, model: {model_path}'
                    )
                specs.append((model_path, weight))
            self._scenario_blends[str(scenario_name)] = specs
        self._scenario_gated_blends: dict[str, dict] = {}
        for scenario_name, raw_spec in raw_scenario_gated_blends.items():
            if not isinstance(raw_spec, dict):
                raise RuntimeError(f'Gated blend entry must be an object. Problem scenario: {scenario_name}')
            base_model = str(raw_spec.get('base_model') or raw_spec.get('base') or '').strip()
            safety_model = str(raw_spec.get('safety_model') or raw_spec.get('safety') or '').strip()
            if not base_model or not safety_model:
                raise RuntimeError(f'Gated blend entry must define base_model and safety_model. Problem scenario: {scenario_name}')
            spec = dict(raw_spec)
            spec['base_model'] = base_model
            spec['safety_model'] = safety_model
            critical_safety_model = str(raw_spec.get('critical_safety_model') or raw_spec.get('critical_safety') or '').strip()
            if critical_safety_model:
                spec['critical_safety_model'] = critical_safety_model
            self._scenario_gated_blends[str(scenario_name)] = spec

        self._default_model = str(
            router_config.get('default_model')
            or next(iter(self._scenario_models.values()), '')
            or next(iter((specs[0][0] for specs in self._scenario_blends.values() if specs)), '')
            or next(iter((spec['base_model'] for spec in self._scenario_gated_blends.values())), '')
        )
        if not self._default_model:
            raise RuntimeError('MAPPO router config must resolve a default model or gated base model.')
        self._active_scenario: str | None = None
        self._policies: dict[str, MappoActorPolicy] = {}
        self._scenario_policies: dict[str, MappoActorPolicy | WeightedBlendPolicy | GatedBlendPolicy] = {}

        reference_policy: MappoActorPolicy | None = None
        unique_paths = {self._default_model, *self._scenario_models.values()}
        for blend_specs in self._scenario_blends.values():
            unique_paths.update(model_path for model_path, _ in blend_specs)
        for gated_spec in self._scenario_gated_blends.values():
            unique_paths.add(str(gated_spec['base_model']))
            unique_paths.add(str(gated_spec['safety_model']))
            if gated_spec.get('critical_safety_model'):
                unique_paths.add(str(gated_spec['critical_safety_model']))
        for model_path in unique_paths:
            _, policy = _load_mappo_checkpoint(model_path, device)
            _require_supported_action_dim(policy.action_dim, model_path=model_path)
            if reference_policy is None:
                reference_policy = policy
            elif policy.action_dim != reference_policy.action_dim or policy.obs_dim != reference_policy.obs_dim:
                raise RuntimeError(
                    'All router experts must share the same obs_dim/action_dim. '
                    f'Got {model_path} -> (obs={policy.obs_dim}, action={policy.action_dim}), '
                    f'expected (obs={reference_policy.obs_dim}, action={reference_policy.action_dim}).'
                )
            self._policies[model_path] = policy

        if reference_policy is None:
            raise RuntimeError('MAPPO router config did not resolve any expert checkpoints.')

        self._default_policy = self._policies[self._default_model]
        for scenario_name, model_path in self._scenario_models.items():
            self._scenario_policies[scenario_name] = self._policies[model_path]
        for scenario_name, blend_specs in self._scenario_blends.items():
            self._scenario_policies[scenario_name] = WeightedBlendPolicy(
                [(self._policies[model_path], weight) for model_path, weight in blend_specs]
            )
        for scenario_name, gated_spec in self._scenario_gated_blends.items():
            self._scenario_policies[scenario_name] = GatedBlendPolicy(
                self._policies[str(gated_spec['base_model'])],
                self._policies[str(gated_spec['safety_model'])],
                gated_spec,
                self._policies[str(gated_spec['critical_safety_model'])]
                if gated_spec.get('critical_safety_model') else None,
            )

        self.action_dim = self._default_policy.action_dim
        self.obs_dim = self._default_policy.obs_dim

    def set_active_scenario(self, scenario_name: str | None) -> None:
        self._active_scenario = str(scenario_name) if scenario_name else None
        for policy in self._policies.values():
            policy.set_active_scenario(self._active_scenario)

    def _resolve_policy(self) -> MappoActorPolicy | WeightedBlendPolicy | GatedBlendPolicy:
        return self._scenario_policies.get(self._active_scenario or '', self._default_policy)

    def predict(self, observation: np.ndarray) -> np.ndarray:
        return self._resolve_policy().predict(observation)


def _load_json_payload(model_path: str) -> dict:
    path = Path(model_path)
    payload = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(payload, dict):
        raise RuntimeError(f'Router config must be a JSON object: {model_path}')
    return payload


def _require_supported_action_dim(action_dim: int | None, *, model_path: str | None):
    if action_dim is None:
        return
    if int(action_dim) not in (1, 2):
        source = model_path or 'the selected policy'
        raise RuntimeError(
            'Pure RL evaluation supports 1D (angular_only) or 2D (full/speed_scale) '
            f'action policies. Received action_dim={int(action_dim)} from {source}.'
        )


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value == '':
        return float(default)
    return float(value)


def _env_str(name: str, default: str) -> str:
    value = os.environ.get(name)
    if value is None or value == '':
        return str(default)
    return str(value)


def _detect_policy_kind(policy_kind: str, model_path: str | None) -> str:
    if policy_kind != 'auto':
        return policy_kind
    if not model_path:
        return 'zero'
    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        return 'mappo'
    if suffix == '.npz':
        return 'bc'
    if suffix == '.json':
        payload = _load_json_payload(model_path)
        policy_type = str(payload.get('policy_type', '')).strip().lower()
        if policy_type in {'mappo_router', 'router', 'scenario_router'}:
            return 'mappo_router'
    raise RuntimeError(f'Unable to infer policy type from model path: {model_path}')


def _resolve_action_mode(action_mode: str, policy) -> str:
    if action_mode != 'auto':
        return action_mode
    _require_supported_action_dim(getattr(policy, 'action_dim', None), model_path=None)
    return 'full'


def _resolve_max_neighbors(max_neighbors: str | int, policy) -> int:
    if str(max_neighbors).lower() != 'auto':
        return max(1, int(max_neighbors))
    obs_dim = getattr(policy, 'obs_dim', None)
    if obs_dim is None or int(obs_dim) < AgentLocalObservation.ego_feature_size():
        return 4
    inferred = (int(obs_dim) - AgentLocalObservation.ego_feature_size() - ENCOUNTER_TYPE_COUNT) // NEIGHBOR_FEATURE_COUNT
    return max(1, inferred)


def _build_agent_namespaces(num_agents: int) -> tuple[str, ...]:
    return tuple(f'usv_{index + 1:02d}' for index in range(max(2, num_agents)))


def _resolve_scenarios(scenarios: tuple[str, ...] | None, agent_count: int) -> tuple[str, ...]:
    resolved = tuple(scenarios) if scenarios else MultiAgentScenarioFactory.cluster_available()
    incompatible = [
        scenario
        for scenario in resolved
        if MultiAgentScenarioFactory.required_agent_count(scenario) > agent_count
    ]
    if incompatible:
        joined = ', '.join(incompatible)
        raise ValueError(f'Scenarios require more agents than configured (agent_count={agent_count}): {joined}')
    return resolved


def _resolve_device(device: str) -> str:
    if device == 'auto':
        import torch
        return 'cuda' if torch.cuda.is_available() else 'cpu'
    return device


def _load_mappo_checkpoint(model_path: str, device: str) -> tuple[dict, MappoActorPolicy]:
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError('torch is required for MAPPO evaluation.') from exc

    device = _resolve_device(device)
    checkpoint = torch.load(model_path, map_location=device, weights_only=False)
    return checkpoint, MappoActorPolicy(checkpoint, device=device)


def _build_env_kwargs_from_checkpoint(
    checkpoint: dict,
    policy,
    *,
    scenarios: tuple[str, ...] | None,
    episode_timeout: float | None,
    no_progress_timeout: float | None,
    model_path: str,
) -> tuple[dict, tuple[str, ...]]:
    _require_supported_action_dim(getattr(policy, 'action_dim', None), model_path=model_path)
    agent_namespaces = tuple(checkpoint['agent_namespaces'])
    resolved_scenarios = tuple(scenarios) if scenarios else tuple(checkpoint.get('scenarios', MultiAgentScenarioFactory.available()))
    resolved_action_mode = str(checkpoint.get('action_mode', 'full'))
    if resolved_action_mode not in ('full', 'speed_scale', 'angular_only'):
        raise RuntimeError(
            'Pure RL evaluation supports action_mode in {"full", "speed_scale", "angular_only"}. '
            f'Received action_mode="{resolved_action_mode}" from {model_path}.'
        )
    resolved_max_neighbors = int(checkpoint.get('max_neighbors', max(1, int((policy.obs_dim - AgentLocalObservation.ego_feature_size() - ENCOUNTER_TYPE_COUNT) // NEIGHBOR_FEATURE_COUNT))))
    resolved_max_agents = int(checkpoint.get('max_agents', len(agent_namespaces)))
    env_kwargs = {
        'agent_namespaces': agent_namespaces,
        'enable_rl_backend': True,
        'rl_control_mode': 'pure',
        'action_mode': resolved_action_mode,
        'action_speed_scale_min': float(checkpoint.get('action_speed_scale_min', 0.25)),
        'action_bounds': ActionBounds(**checkpoint.get('action_bounds', {'linear_delta': 0.7, 'angular_delta': 0.6})),
        'max_neighbors': resolved_max_neighbors,
        'max_agents': max(resolved_max_agents, len(agent_namespaces)),
        'cruise_speed': float(checkpoint.get('cruise_speed', 0.5)),
        'max_angular_velocity': float(checkpoint.get('max_angular_velocity', 0.5)),
        'heading_omega_deadband': float(checkpoint.get('heading_omega_deadband', 0.06)),
        'heading_omega_reference': float(checkpoint.get('heading_omega_reference', 0.85)),
        'angular_authority_power': float(checkpoint.get('angular_authority_power', 1.6)),
        'angular_accel_limit': float(checkpoint.get('angular_accel_limit', 1.8)),
        'angular_decel_limit': float(checkpoint.get('angular_decel_limit', 2.4)),
        'conflict_turn_relief': float(checkpoint.get('conflict_turn_relief', 0.55)),
        'episode_timeout': float(episode_timeout) if episode_timeout is not None else float(checkpoint.get('episode_timeout', 45.0)),
        'no_progress_timeout': float(no_progress_timeout) if no_progress_timeout is not None else float(checkpoint.get('no_progress_timeout', 10.0)),
        'min_progress_delta': float(checkpoint.get('min_progress_delta', 0.3)),
        'collision_distance': _env_float('COLLISION_DISTANCE', float(checkpoint.get('collision_distance', 0.5))),
        'near_miss_distance': _env_float('NEAR_MISS_DISTANCE', float(checkpoint.get('near_miss_distance', 1.5))),
        'scenario_neighbor_speed': float(checkpoint.get('scenario_neighbor_speed', 0.45)),
        'cte_clip_range': _env_float('CTE_CLIP_RANGE', float(checkpoint.get('cte_clip_range', 3.0))),
        'route_progress_cte_gate_start': _env_float('ROUTE_PROGRESS_CTE_GATE_START', float(checkpoint.get('route_progress_cte_gate_start', 0.0))),
        'route_progress_cte_gate_width': _env_float('ROUTE_PROGRESS_CTE_GATE_WIDTH', float(checkpoint.get('route_progress_cte_gate_width', 0.0))),
        'route_progress_cte_gate_floor': _env_float('ROUTE_PROGRESS_CTE_GATE_FLOOR', float(checkpoint.get('route_progress_cte_gate_floor', 0.25))),
        'low_speed_recovery_floor_enabled': _env_bool('LOW_SPEED_RECOVERY_FLOOR_ENABLED', bool(checkpoint.get('low_speed_recovery_floor_enabled', False))),
        'low_speed_recovery_floor_speed': _env_float('LOW_SPEED_RECOVERY_FLOOR_SPEED', float(checkpoint.get('low_speed_recovery_floor_speed', 0.12))),
        'low_speed_recovery_floor_cte_speed': _env_float('LOW_SPEED_RECOVERY_FLOOR_CTE_SPEED', float(checkpoint.get('low_speed_recovery_floor_cte_speed', 0.08))),
        'low_speed_recovery_floor_min_raw_linear': _env_float('LOW_SPEED_RECOVERY_FLOOR_MIN_RAW_LINEAR', float(checkpoint.get('low_speed_recovery_floor_min_raw_linear', 0.16))),
        'low_speed_recovery_floor_min_distance': _env_float('LOW_SPEED_RECOVERY_FLOOR_MIN_DISTANCE', float(checkpoint.get('low_speed_recovery_floor_min_distance', 2.5))),
        'low_speed_recovery_floor_min_neighbor_separation': _env_float('LOW_SPEED_RECOVERY_FLOOR_MIN_NEIGHBOR_SEPARATION', float(checkpoint.get('low_speed_recovery_floor_min_neighbor_separation', 1.35))),
        'low_speed_recovery_floor_max_conflict_level': _env_float('LOW_SPEED_RECOVERY_FLOOR_MAX_CONFLICT_LEVEL', float(checkpoint.get('low_speed_recovery_floor_max_conflict_level', 0.12))),
        'low_speed_recovery_floor_min_route_progress': _env_float('LOW_SPEED_RECOVERY_FLOOR_MIN_ROUTE_PROGRESS', float(checkpoint.get('low_speed_recovery_floor_min_route_progress', 0.0))),
        'low_speed_recovery_floor_max_route_progress': _env_float('LOW_SPEED_RECOVERY_FLOOR_MAX_ROUTE_PROGRESS', float(checkpoint.get('low_speed_recovery_floor_max_route_progress', 1.10))),
        'low_speed_recovery_floor_cte_slow_threshold': _env_float('LOW_SPEED_RECOVERY_FLOOR_CTE_SLOW_THRESHOLD', float(checkpoint.get('low_speed_recovery_floor_cte_slow_threshold', 1.80))),
        'low_speed_recovery_floor_heading_slow_threshold': _env_float('LOW_SPEED_RECOVERY_FLOOR_HEADING_SLOW_THRESHOLD', float(checkpoint.get('low_speed_recovery_floor_heading_slow_threshold', 2.45))),
        'low_speed_recovery_floor_omega_blend': _env_float('LOW_SPEED_RECOVERY_FLOOR_OMEGA_BLEND', float(checkpoint.get('low_speed_recovery_floor_omega_blend', 0.60))),
        'low_speed_recovery_floor_cte_omega_enabled': _env_bool('LOW_SPEED_RECOVERY_FLOOR_CTE_OMEGA_ENABLED', bool(checkpoint.get('low_speed_recovery_floor_cte_omega_enabled', True))),
        'low_speed_recovery_floor_cte_omega_max': _env_float('LOW_SPEED_RECOVERY_FLOOR_CTE_OMEGA_MAX', float(checkpoint.get('low_speed_recovery_floor_cte_omega_max', 0.35))),
        'low_speed_recovery_floor_cte_omega_full_abs_cte': _env_float('LOW_SPEED_RECOVERY_FLOOR_CTE_OMEGA_FULL_ABS_CTE', float(checkpoint.get('low_speed_recovery_floor_cte_omega_full_abs_cte', 3.0))),
        'random_encounter_route_priority': bool(checkpoint.get('random_encounter_route_priority', False)),
        'default_scenarios': resolved_scenarios,
        'reward': RewardConfig(**checkpoint['reward_config']) if 'reward_config' in checkpoint else RewardConfig(),
        'goal_proximity_reward_weight': float(checkpoint.get('goal_proximity_reward_weight', 0.0)),
        'goal_proximity_relief_distance': float(checkpoint.get('goal_proximity_relief_distance', 3.0)),
        'goal_proximity_heading_relief': float(checkpoint.get('goal_proximity_heading_relief', 0.0)),
        'goal_proximity_smoothness_relief': float(checkpoint.get('goal_proximity_smoothness_relief', 0.0)),
        'goal_proximity_conflict_relief': float(checkpoint.get('goal_proximity_conflict_relief', 0.0)),
        'goal_proximity_speed_relief': float(checkpoint.get('goal_proximity_speed_relief', 0.0)),
        'team_reward_weight': float(checkpoint.get('team_reward_weight', 0.30)),
        'team_progress_weight': float(checkpoint.get('team_progress_weight', 1.20)),
        'team_goal_proximity_weight': float(checkpoint.get('team_goal_proximity_weight', 0.0)),
        'team_regression_penalty_weight': float(checkpoint.get('team_regression_penalty_weight', 0.0)),
        'team_dispersion_penalty_weight': float(checkpoint.get('team_dispersion_penalty_weight', 0.0)),
        'team_dispersion_margin': float(checkpoint.get('team_dispersion_margin', 0.0)),
        'coordination_reward_weight': float(checkpoint.get('coordination_reward_weight', 0.20)),
        'team_completion_bonus': float(checkpoint.get('team_completion_bonus', 18.0)),
        'deadlock_penalty_weight': float(checkpoint.get('deadlock_penalty_weight', 4.0)),
        'pairwise_shield_enabled': _env_bool('PAIRWISE_SHIELD_ENABLED', bool(checkpoint.get('pairwise_shield_enabled', False))),
        'pairwise_shield_release_separation': _env_float('PAIRWISE_SHIELD_RELEASE_SEPARATION', float(checkpoint.get('pairwise_shield_release_separation', 1.05))),
        'pairwise_shield_critical_separation': _env_float('PAIRWISE_SHIELD_CRITICAL_SEPARATION', float(checkpoint.get('random_deconflict_critical_separation', 0.92))),
        'pairwise_shield_min_closing_speed': _env_float('PAIRWISE_SHIELD_MIN_CLOSING_SPEED', float(checkpoint.get('random_deconflict_closing_speed_min', 0.004))),
        'pairwise_shield_yield_speed': _env_float('PAIRWISE_SHIELD_YIELD_SPEED', float(checkpoint.get('random_deconflict_yield_speed', 0.02))),
        'pairwise_shield_standon_speed': _env_float('PAIRWISE_SHIELD_STANDON_SPEED', float(checkpoint.get('random_deconflict_standon_speed', 0.24))),
        'pairwise_shield_yield_omega': _env_float('PAIRWISE_SHIELD_YIELD_OMEGA', float(checkpoint.get('random_deconflict_yield_omega', 0.44))),
        'pairwise_shield_standon_omega': _env_float('PAIRWISE_SHIELD_STANDON_OMEGA', float(checkpoint.get('random_deconflict_standon_omega', 0.04))),
        'pairwise_shield_yield_danger_scale': _env_float('PAIRWISE_SHIELD_YIELD_DANGER_SCALE', float(checkpoint.get('random_deconflict_yield_danger_scale', 1.0))),
        'pairwise_shield_blend': _env_float('PAIRWISE_SHIELD_BLEND', float(checkpoint.get('pairwise_shield_blend', 1.0))),
        'pairwise_shield_yield_blend': _env_float('PAIRWISE_SHIELD_YIELD_BLEND', float(checkpoint.get('pairwise_shield_yield_blend', -1.0))),
        'pairwise_shield_standon_blend': _env_float('PAIRWISE_SHIELD_STANDON_BLEND', float(checkpoint.get('pairwise_shield_standon_blend', -1.0))),
        'pairwise_shield_yield_only': _env_bool('PAIRWISE_SHIELD_YIELD_ONLY', bool(checkpoint.get('pairwise_shield_yield_only', False))),
        'pairwise_shield_yield_agent_ids': _env_str('PAIRWISE_SHIELD_YIELD_AGENT_IDS', str(checkpoint.get('pairwise_shield_yield_agent_ids', ''))),
        'pairwise_shield_standon_agent_ids': _env_str('PAIRWISE_SHIELD_STANDON_AGENT_IDS', str(checkpoint.get('pairwise_shield_standon_agent_ids', ''))),
        'pairwise_shield_pair_ids': _env_str('PAIRWISE_SHIELD_PAIR_IDS', str(checkpoint.get('pairwise_shield_pair_ids', ''))),
        'pairwise_shield_critical_bypass_gates': _env_bool('PAIRWISE_SHIELD_CRITICAL_BYPASS_GATES', bool(checkpoint.get('pairwise_shield_critical_bypass_gates', False))),
        'pairwise_shield_turn_mode': _env_str('PAIRWISE_SHIELD_TURN_MODE', str(checkpoint.get('random_deconflict_turn_mode', 'away'))),
        'pairwise_shield_role_mode': _env_str('PAIRWISE_SHIELD_ROLE_MODE', str(checkpoint.get('random_deconflict_role_mode', 'priority-delta'))),
        'pairwise_shield_priority_delta_yield_threshold': _env_float('PAIRWISE_SHIELD_PRIORITY_DELTA_YIELD_THRESHOLD', float(checkpoint.get('random_deconflict_priority_delta_yield_threshold', -0.01))),
        'pairwise_shield_route_eta_yield_threshold': _env_float('PAIRWISE_SHIELD_ROUTE_ETA_YIELD_THRESHOLD', float(checkpoint.get('random_deconflict_route_eta_yield_threshold', 0.02))),
        'pairwise_shield_min_route_progress': _env_float('PAIRWISE_SHIELD_MIN_ROUTE_PROGRESS', float(checkpoint.get('pairwise_shield_min_route_progress', 0.0))),
        'pairwise_shield_max_route_progress': _env_float('PAIRWISE_SHIELD_MAX_ROUTE_PROGRESS', float(checkpoint.get('pairwise_shield_max_route_progress', 1.1))),
        'pairwise_shield_min_abs_cte': _env_float('PAIRWISE_SHIELD_MIN_ABS_CTE', float(checkpoint.get('pairwise_shield_min_abs_cte', 0.0))),
        'pairwise_shield_yield_min_route_progress': _env_float('PAIRWISE_SHIELD_YIELD_MIN_ROUTE_PROGRESS', float(checkpoint.get('pairwise_shield_yield_min_route_progress', -1.0))),
        'pairwise_shield_yield_max_route_progress': _env_float('PAIRWISE_SHIELD_YIELD_MAX_ROUTE_PROGRESS', float(checkpoint.get('pairwise_shield_yield_max_route_progress', -1.0))),
        'pairwise_shield_yield_min_abs_cte': _env_float('PAIRWISE_SHIELD_YIELD_MIN_ABS_CTE', float(checkpoint.get('pairwise_shield_yield_min_abs_cte', -1.0))),
        'pairwise_shield_pair_yield_agent_ids': _env_str('PAIRWISE_SHIELD_PAIR_YIELD_AGENT_IDS', str(checkpoint.get('pairwise_shield_pair_yield_agent_ids', ''))),
        'pairwise_shield_pair_standon_agent_ids': _env_str('PAIRWISE_SHIELD_PAIR_STANDON_AGENT_IDS', str(checkpoint.get('pairwise_shield_pair_standon_agent_ids', ''))),
        'pairwise_shield_pair_yield_speed': _env_str('PAIRWISE_SHIELD_PAIR_YIELD_SPEED', str(checkpoint.get('pairwise_shield_pair_yield_speed', ''))),
        'pairwise_shield_pair_standon_speed': _env_str('PAIRWISE_SHIELD_PAIR_STANDON_SPEED', str(checkpoint.get('pairwise_shield_pair_standon_speed', ''))),
        'pairwise_shield_pair_critical_separation': _env_str('PAIRWISE_SHIELD_PAIR_CRITICAL_SEPARATION', str(checkpoint.get('pairwise_shield_pair_critical_separation', ''))),
        'pairwise_shield_pair_yield_min_route_progress': _env_str('PAIRWISE_SHIELD_PAIR_YIELD_MIN_ROUTE_PROGRESS', str(checkpoint.get('pairwise_shield_pair_yield_min_route_progress', ''))),
        'pairwise_shield_pair_yield_max_route_progress': _env_str('PAIRWISE_SHIELD_PAIR_YIELD_MAX_ROUTE_PROGRESS', str(checkpoint.get('pairwise_shield_pair_yield_max_route_progress', ''))),
        'pairwise_shield_pair_yield_min_abs_cte': _env_str('PAIRWISE_SHIELD_PAIR_YIELD_MIN_ABS_CTE', str(checkpoint.get('pairwise_shield_pair_yield_min_abs_cte', ''))),
        'pairwise_shield_pair_standon_min_route_progress': _env_str('PAIRWISE_SHIELD_PAIR_STANDON_MIN_ROUTE_PROGRESS', str(checkpoint.get('pairwise_shield_pair_standon_min_route_progress', ''))),
        'pairwise_shield_pair_standon_max_route_progress': _env_str('PAIRWISE_SHIELD_PAIR_STANDON_MAX_ROUTE_PROGRESS', str(checkpoint.get('pairwise_shield_pair_standon_max_route_progress', ''))),
        'pairwise_shield_pair_standon_min_abs_cte': _env_str('PAIRWISE_SHIELD_PAIR_STANDON_MIN_ABS_CTE', str(checkpoint.get('pairwise_shield_pair_standon_min_abs_cte', ''))),
        'pairwise_shield_pair_max_dcpa': _env_str('PAIRWISE_SHIELD_PAIR_MAX_DCPA', str(checkpoint.get('pairwise_shield_pair_max_dcpa', ''))),
        'pairwise_shield_pair_post_cpa_release_separation': _env_str('PAIRWISE_SHIELD_PAIR_POST_CPA_RELEASE_SEPARATION', str(checkpoint.get('pairwise_shield_pair_post_cpa_release_separation', ''))),
        'pairwise_shield_pair_require_colregs': _env_str('PAIRWISE_SHIELD_PAIR_REQUIRE_COLREGS', str(checkpoint.get('pairwise_shield_pair_require_colregs', ''))),
        'pairwise_shield_standon_min_route_progress': _env_float('PAIRWISE_SHIELD_STANDON_MIN_ROUTE_PROGRESS', float(checkpoint.get('pairwise_shield_standon_min_route_progress', -1.0))),
        'pairwise_shield_standon_max_route_progress': _env_float('PAIRWISE_SHIELD_STANDON_MAX_ROUTE_PROGRESS', float(checkpoint.get('pairwise_shield_standon_max_route_progress', -1.0))),
        'pairwise_shield_standon_min_abs_cte': _env_float('PAIRWISE_SHIELD_STANDON_MIN_ABS_CTE', float(checkpoint.get('pairwise_shield_standon_min_abs_cte', -1.0))),
        'pairwise_shield_yield_late_min_route_progress': _env_float('PAIRWISE_SHIELD_YIELD_LATE_MIN_ROUTE_PROGRESS', float(checkpoint.get('pairwise_shield_yield_late_min_route_progress', -1.0))),
        'pairwise_shield_yield_late_min_abs_cte': _env_float('PAIRWISE_SHIELD_YIELD_LATE_MIN_ABS_CTE', float(checkpoint.get('pairwise_shield_yield_late_min_abs_cte', -1.0))),
        'pairwise_shield_standon_late_min_route_progress': _env_float('PAIRWISE_SHIELD_STANDON_LATE_MIN_ROUTE_PROGRESS', float(checkpoint.get('pairwise_shield_standon_late_min_route_progress', -1.0))),
        'pairwise_shield_standon_late_min_abs_cte': _env_float('PAIRWISE_SHIELD_STANDON_LATE_MIN_ABS_CTE', float(checkpoint.get('pairwise_shield_standon_late_min_abs_cte', -1.0))),
    }
    return env_kwargs, resolved_scenarios


def _load_mappo_router_policy(model_path: str, device: str) -> tuple[dict, ScenarioRouterPolicy, dict]:
    router_config = _load_json_payload(model_path)
    template_path = str(
        router_config.get('env_template')
        or router_config.get('default_model')
        or next(iter(router_config.get('scenario_models', {}).values()), '')
    )
    if not template_path:
        raise RuntimeError(f'MAPPO router config does not declare any expert checkpoint: {model_path}')
    checkpoint, _ = _load_mappo_checkpoint(template_path, device)
    return checkpoint, ScenarioRouterPolicy(router_config, device=device), router_config


def _load_policy_bundle(
    policy_kind: str,
    model_path: str | None,
    *,
    device: str,
    num_agents: int,
    scenarios: tuple[str, ...] | None,
    episode_timeout: float | None,
    no_progress_timeout: float | None,
    rl_control_mode: str,
    action_mode: str,
    max_neighbors: str | int,
    max_agents: int | None,
) -> tuple[object, dict, tuple[str, ...]]:
    if policy_kind == 'mappo':
        if model_path is None:
            raise RuntimeError('MAPPO evaluation requires --model.')
        checkpoint, policy = _load_mappo_checkpoint(model_path, device)
        setattr(policy, 'trace_mask_config', checkpoint)
        env_kwargs, resolved_scenarios = _build_env_kwargs_from_checkpoint(
            checkpoint,
            policy,
            scenarios=scenarios,
            episode_timeout=episode_timeout,
            no_progress_timeout=no_progress_timeout,
            model_path=model_path,
        )
        return policy, env_kwargs, resolved_scenarios

    if policy_kind == 'mappo_router':
        if model_path is None:
            raise RuntimeError('MAPPO router evaluation requires --model.')
        checkpoint, policy, router_config = _load_mappo_router_policy(model_path, device)
        setattr(policy, 'trace_mask_config', checkpoint)
        configured_scenarios = tuple(router_config.get('scenarios', ())) or None
        env_kwargs, resolved_scenarios = _build_env_kwargs_from_checkpoint(
            checkpoint,
            policy,
            scenarios=tuple(scenarios) if scenarios else configured_scenarios,
            episode_timeout=episode_timeout,
            no_progress_timeout=no_progress_timeout,
            model_path=model_path,
        )
        return policy, env_kwargs, resolved_scenarios

    if policy_kind == 'bc':
        if model_path is None:
            raise RuntimeError('BC evaluation requires --model.')
        policy = load_policy(model_path)
    elif policy_kind == 'zero':
        policy = ZeroPolicy(action_dim=2)
    else:
        raise RuntimeError(f'Unsupported policy type: {policy_kind}')

    resolved_rl_control_mode = 'pure'
    resolved_action_mode = _resolve_action_mode(action_mode, policy)
    resolved_max_neighbors = _resolve_max_neighbors(max_neighbors, policy)
    required_agents = max((MultiAgentScenarioFactory.required_agent_count(name) for name in (scenarios or MultiAgentScenarioFactory.cluster_available())), default=2)
    agent_namespaces = _build_agent_namespaces(max(required_agents, num_agents))
    resolved_scenarios = _resolve_scenarios(scenarios, len(agent_namespaces))
    env_kwargs = {
        'agent_namespaces': agent_namespaces,
        'enable_rl_backend': True,
        'rl_control_mode': resolved_rl_control_mode,
        'action_mode': resolved_action_mode,
        'max_neighbors': resolved_max_neighbors,
        'max_agents': max(len(agent_namespaces), int(max_agents or len(agent_namespaces))),
        'episode_timeout': float(episode_timeout) if episode_timeout is not None else 45.0,
        'no_progress_timeout': float(no_progress_timeout) if no_progress_timeout is not None else 10.0,
        'default_scenarios': resolved_scenarios,
    }
    return policy, env_kwargs, resolved_scenarios


def _scenario_summary(metrics: list[dict]) -> dict[str, dict]:
    grouped = {}
    for item in metrics:
        grouped.setdefault(item['scenario'], []).append(item)

    result = {}
    for scenario, items in grouped.items():
        count = max(1, len(items))
        result[scenario] = {
            'episodes': len(items),
            'active_agent_count': len(items[0].get('active_agent_ids', [])),
            'runtime_agent_count': len(items[0].get('runtime_agent_ids', [])),
            'inactive_agent_ids': list(items[0].get('inactive_agent_ids', [])),
            'collision_rate': sum(item['collision'] for item in items) / count,
            'success_rate': sum(item['success'] for item in items) / count,
            'timeout_rate': sum(item['timeout'] for item in items) / count,
            'mean_steps': float(np.mean([item['steps'] for item in items])),
            'mean_pairwise_min_separation': float(np.mean([item['pairwise_min_separation'] for item in items])),
            'worst_pairwise_min_separation': float(np.min([item['pairwise_min_separation'] for item in items])),
            'mean_episode_min_separation': float(np.mean([item['episode_min_separation'] for item in items])),
            'worst_episode_min_separation': float(np.min([item['episode_min_separation'] for item in items])),
            'mean_goal_completion_ratio': float(np.mean([item['goal_completion_ratio'] for item in items])),
            'mean_team_goal_distance_delta': float(np.mean([item['team_goal_distance_delta'] for item in items])),
            'mean_team_goal_progress_ratio': float(np.mean([item['team_goal_progress_ratio'] for item in items])),
            # Progress rate metrics
            'mean_goal_approach_velocity': float(np.mean([item['goal_approach_velocity'] for item in items])),
            'mean_progress_efficiency': float(np.mean([item['progress_efficiency'] for item in items])),
            'mean_projected_progress': float(np.mean([item['projected_progress'] for item in items])),
            'mean_progress_consistency': float(np.mean([item['progress_consistency'] for item in items])),
            # Smoothness metrics
            'mean_linear_accel': float(np.mean([item['mean_linear_accel'] for item in items])),
            'mean_angular_accel': float(np.mean([item['mean_angular_accel'] for item in items])),
            'mean_omega_flip_count': float(np.mean([item['omega_flip_count'] for item in items])),
            'mean_omega_saturation_ratio': float(np.mean([item['omega_saturation_ratio'] for item in items])),
            'mean_heading_sign_flip_count': float(np.mean([item['heading_sign_flip_count'] for item in items])),
            'mean_heading_error': float(np.mean([item['mean_heading_error'] for item in items])),
            # Path tracking metrics
            'mean_cross_track_error': float(np.mean([item['mean_cross_track_error'] for item in items])),
            # Entanglement metrics
            'mean_entanglement_ratio': float(np.mean([item['entanglement_ratio'] for item in items])),
            # COLREGs compliance metrics
            'mean_colregs_compliance_ratio': float(np.mean([item['colregs_compliance_ratio'] for item in items])),
            'mean_colregs_violation_ratio': float(np.mean([item['colregs_violation_ratio'] for item in items])),
            'mean_cpa_starboard_pass_ratio': float(np.mean([item['cpa_starboard_pass_ratio'] for item in items])),
            'total_colregs_head_on_steps': int(sum(item['colregs_head_on_steps'] for item in items)),
            'total_colregs_crossing_steps': int(sum(item['colregs_crossing_steps'] for item in items)),
            'total_colregs_overtaking_steps': int(sum(item['colregs_overtaking_steps'] for item in items)),
        }
    return result


def _scenario_initial_team_mean_goal_distance(env: MultiAgentEnv) -> float | None:
    scenario = getattr(env, '_scenario', None)
    if scenario is None:
        return None

    distances = []
    for agent_id in env.agent_ids:
        spawn = scenario.agent_spawns.get(agent_id)
        goal = scenario.agent_goals.get(agent_id)
        if spawn is None or goal is None:
            continue
        distances.append(float(np.hypot(goal.x - spawn.x, goal.y - spawn.y)))

    if not distances:
        return None
    return float(np.mean(distances))


def _scenario_geometry_snapshot(env: MultiAgentEnv) -> dict | None:
    scenario = getattr(env, '_scenario', None)
    if scenario is None:
        return None

    agents = {}
    for agent_id in sorted(getattr(env, 'runtime_agent_ids', ())):
        spawn = scenario.agent_spawns.get(agent_id)
        goal = scenario.agent_goals.get(agent_id)
        if spawn is None and goal is None:
            continue
        agents[agent_id] = {
            'spawn': None if spawn is None else {
                'x': float(spawn.x),
                'y': float(spawn.y),
                'yaw': float(spawn.yaw),
            },
            'goal': None if goal is None else {
                'x': float(goal.x),
                'y': float(goal.y),
            },
        }

    background_tracks = []
    for track in getattr(scenario, 'background_tracks', ()):
        background_tracks.append({
            'track_id': str(track.track_id),
            'start_x': float(track.start_x),
            'start_y': float(track.start_y),
            'vx': float(track.vx),
            'vy': float(track.vy),
            'yaw': float(track.yaw),
        })

    return {
        'scenario': str(scenario.name),
        'active_agent_ids': list(getattr(env, 'agent_ids', ())),
        'runtime_agent_ids': list(getattr(env, 'runtime_agent_ids', ())),
        'agents': agents,
        'background_tracks': background_tracks,
    }


def _create_env(env_kwargs: dict) -> MultiAgentEnv:
    return MultiAgentEnv(MultiAgentEnvConfig(**env_kwargs))


def _trace_episode_sample(
    env: MultiAgentEnv,
    observations: dict,
    *,
    scenario_name: str,
    step: int,
    pairwise_min_separation: float,
    team_mean_goal_distance: float,
    goal_completion_ratio: float,
    trace_mask_config: dict,
    include_raw_observation: bool = False,
    trace_kind: str | None = None,
) -> dict:
    trace_agents = {}
    for trace_agent_id in env.agent_ids:
        obs_obj = env._latest_observations.get(trace_agent_id)
        if obs_obj is None:
            continue
        nearest_neighbor = min(
            obs_obj.neighbors,
            key=lambda item: float(item.distance),
            default=None,
        )
        trace_agent = {
            'distance_to_goal': float(obs_obj.distance_to_goal),
            'route_progress': float(obs_obj.route_progress),
            'cross_track_error': float(obs_obj.cross_track_error),
            'raw_cross_track_error': float(getattr(obs_obj, 'raw_cross_track_error', obs_obj.cross_track_error)),
            'cross_track_overflow': float(getattr(obs_obj, 'cross_track_overflow', 0.0)),
            'heading_error': float(obs_obj.heading_error),
            'crossing_priority': float(obs_obj.crossing_priority),
            'final_linear_x': float(obs_obj.final_linear_x),
            'final_angular_z': float(obs_obj.final_angular_z),
            'nearest_id': str(nearest_neighbor.source_id) if nearest_neighbor is not None else None,
            'nearest_distance': float(nearest_neighbor.distance) if nearest_neighbor is not None else None,
            'nearest_rel_x': float(nearest_neighbor.rel_x) if nearest_neighbor is not None else None,
            'nearest_rel_y': float(nearest_neighbor.rel_y) if nearest_neighbor is not None else None,
            'low_speed_recovery_floor_diagnostics': env.low_speed_recovery_floor_diagnostics(trace_agent_id),
            'pairwise_shield_diagnostics': env.pairwise_shield_diagnostics(trace_agent_id, obs_obj),
            'mask_diagnostics': _trace_random_mask_diagnostics(
                obs_obj,
                scenario_name=scenario_name,
                team_min_separation=pairwise_min_separation,
                config=trace_mask_config,
            ),
        }
        if bool(include_raw_observation) and trace_agent_id in observations:
            trace_agent['raw_observation'] = np.asarray(observations[trace_agent_id], dtype=np.float32).tolist()
        trace_agents[trace_agent_id] = trace_agent

    sample = {
        'step': int(step),
        'pairwise_min_separation': float(pairwise_min_separation),
        'team_mean_goal_distance': float(team_mean_goal_distance),
        'goal_completion_ratio': float(goal_completion_ratio),
        'agents': trace_agents,
    }
    if trace_kind is not None:
        sample['trace_kind'] = str(trace_kind)
    return sample


def _trace_float(config: dict, key: str, default: float) -> float:
    try:
        return float(config.get(key, default)) if isinstance(config, dict) else float(default)
    except (TypeError, ValueError):
        return float(default)


def _trace_bool(config: dict, key: str, default: bool = False) -> bool:
    value = config.get(key, default) if isinstance(config, dict) else default
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'on'}
    return bool(value)


def _trace_goal_heading_omega_sign(config: dict) -> float:
    return 1.0 if _trace_float(config, 'goal_heading_omega_sign', -1.0) >= 0.0 else -1.0


def _trace_signed_cte(obs_obj: AgentLocalObservation, config: dict) -> float:
    source = str(config.get('random_cte_source', 'clipped')).strip().lower()
    if source == 'raw':
        return float(getattr(obs_obj, 'raw_cross_track_error', getattr(obs_obj, 'cross_track_error', 0.0)))
    return float(obs_obj.cross_track_error)


def _trace_str(config: dict, key: str, default: str) -> str:
    value = config.get(key, default) if isinstance(config, dict) else default
    return str(value)


def _trace_agent_index(agent_id: str) -> int:
    suffix = str(agent_id).rsplit('_', 1)[-1]
    digits = ''.join(ch for ch in suffix if ch.isdigit())
    if not digits:
        return 0
    return max(0, int(digits) - 1)


def _trace_nearest_neighbor(obs_obj: AgentLocalObservation) -> tuple[object | None, dict]:
    nearest = min(obs_obj.neighbors, key=lambda item: float(item.distance), default=None)
    if nearest is None or float(nearest.distance) <= 1e-6:
        return None, {
            'valid': False,
            'id': None,
            'distance': None,
            'bearing': 0.0,
            'closing_speed': 0.0,
            'rel_x': None,
            'rel_y': None,
            'priority_delta': 0.0,
            'route_eta_delta': 0.0,
        }
    distance = max(float(nearest.distance), 1e-3)
    rel_x = float(nearest.rel_x)
    rel_y = float(nearest.rel_y)
    rel_vx = float(nearest.rel_vx)
    rel_vy = float(nearest.rel_vy)
    closing_speed = -((rel_x * rel_vx) + (rel_y * rel_vy)) / distance
    return nearest, {
        'valid': True,
        'id': str(nearest.source_id),
        'distance': float(nearest.distance),
        'bearing': float(nearest.bearing),
        'closing_speed': float(closing_speed),
        'rel_x': rel_x,
        'rel_y': rel_y,
        'priority_delta': float(getattr(nearest, 'route_priority_delta', 0.0)),
        'route_eta_delta': float(getattr(nearest, 'route_eta_delta', 0.0)),
    }


def _trace_cpa_threat(obs_obj: AgentLocalObservation, config: dict) -> dict:
    lookahead = max(0.0, _trace_float(config, 'random_deconflict_lookahead_distance', 10.8))
    horizon = max(0.0, _trace_float(config, 'random_deconflict_time_horizon', 30.0))
    dcpa_target = max(0.0, _trace_float(config, 'random_deconflict_dcpa_target', 1.45))
    closing_min = max(0.0, _trace_float(config, 'random_deconflict_closing_speed_min', 0.004))
    best = {
        'valid_cpa': False,
        'cpa_id': None,
        'cpa_distance': None,
        'cpa_bearing': 0.0,
        'cpa_closing_speed': 0.0,
        'cpa_dcpa': 0.0,
        'cpa_tcpa': 0.0,
        'cpa_score': 0.0,
        'cpa_priority_delta': 0.0,
        'cpa_route_eta_delta': 0.0,
    }
    if lookahead <= 0.0 or horizon <= 0.0 or dcpa_target <= 0.0:
        return best

    for neighbor in obs_obj.neighbors:
        distance = max(0.0, float(neighbor.distance))
        if distance <= 1e-6:
            continue
        rel_x = float(neighbor.rel_x)
        rel_y = float(neighbor.rel_y)
        rel_vx = float(neighbor.rel_vx)
        rel_vy = float(neighbor.rel_vy)
        dot = (rel_x * rel_vx) + (rel_y * rel_vy)
        rel_speed_sq = (rel_vx * rel_vx) + (rel_vy * rel_vy)
        closing_speed = -dot / max(distance, 1e-3)
        tcpa = -dot / max(rel_speed_sq, 1e-6) if rel_speed_sq > 1e-6 else 0.0
        positive_tcpa = tcpa > 0.0
        cpa_x = rel_x + rel_vx * (tcpa if positive_tcpa else 0.0)
        cpa_y = rel_y + rel_vy * (tcpa if positive_tcpa else 0.0)
        dcpa = float(np.hypot(cpa_x, cpa_y)) if positive_tcpa else distance

        distance_gate = min(1.0, max(0.0, (lookahead - distance) / max(lookahead, 1e-3)))
        time_gate = min(1.0, max(0.0, (horizon - tcpa) / max(horizon, 1e-3)))
        dcpa_deficit = min(1.0, max(0.0, (dcpa_target - dcpa) / max(dcpa_target, 1e-3)))
        closing_gate = min(1.0, max(0.0, (closing_speed - closing_min) / max(0.35 - closing_min, 1e-3)))
        threat_mask = (
            distance <= lookahead
            and closing_speed >= closing_min
            and positive_tcpa
            and tcpa <= horizon
            and dcpa < dcpa_target
        )
        score = max(distance_gate, time_gate) * dcpa_deficit * (0.25 + 0.75 * closing_gate) if threat_mask else 0.0
        if score > float(best['cpa_score']):
            best = {
                'valid_cpa': score > 0.0,
                'cpa_id': str(neighbor.source_id),
                'cpa_distance': distance,
                'cpa_bearing': float(neighbor.bearing),
                'cpa_closing_speed': float(closing_speed),
                'cpa_dcpa': float(dcpa),
                'cpa_tcpa': float(tcpa),
                'cpa_score': float(score),
                'cpa_priority_delta': float(getattr(neighbor, 'route_priority_delta', 0.0)),
                'cpa_route_eta_delta': float(getattr(neighbor, 'route_eta_delta', 0.0)),
            }
    return best


def _trace_local_threat(config: dict, nearest: dict) -> dict:
    if not _trace_bool(config, 'random_deconflict_local_danger', False) or not bool(nearest['valid']):
        return {'valid_local': False, 'local_score': 0.0}
    safe_separation = max(0.0, _trace_float(config, 'random_deconflict_safe_separation', 1.30))
    release_separation = max(
        safe_separation + 0.05,
        _trace_float(config, 'random_deconflict_release_separation', 2.40),
    )
    nearest_distance = float(nearest['distance'])
    closing_speed = float(nearest['closing_speed'])
    score = min(1.0, max(0.0, (release_separation - nearest_distance) / max(release_separation - safe_separation, 1e-3)))
    closing_gate = min(1.0, max(0.0, (closing_speed + 0.04) / 0.34))
    score *= 0.35 + 0.65 * closing_gate
    critical_separation = max(0.0, _trace_float(config, 'random_deconflict_critical_separation', 0.0))
    if critical_separation > 0.0 and nearest_distance <= critical_separation:
        score = 1.0
    valid = nearest_distance <= release_separation and score > 0.0
    return {'valid_local': bool(valid), 'local_score': float(score if valid else 0.0)}


def _trace_is_yield(obs_obj: AgentLocalObservation, config: dict, nearest: dict | None = None, cpa: dict | None = None, local: dict | None = None) -> bool:
    role_mode = _trace_str(config, 'random_deconflict_role_mode', 'agent-index').strip().lower()
    if role_mode == 'priority-delta':
        nearest = nearest or {}
        cpa = cpa or {}
        local = local or {}
        valid_cpa = bool(cpa.get('valid_cpa', False))
        valid_local = bool(local.get('valid_local', False))
        cpa_score = float(cpa.get('cpa_score', 0.0))
        local_score = float(local.get('local_score', 0.0))
        if valid_cpa and (not valid_local or cpa_score >= local_score):
            selected_delta = float(cpa.get('cpa_priority_delta', 0.0))
            selected_valid = True
        else:
            selected_delta = float(nearest.get('priority_delta', 0.0))
            selected_valid = valid_local and bool(nearest.get('valid', False))
        threshold = _trace_float(config, 'random_deconflict_priority_delta_yield_threshold', -0.01)
        return bool(selected_valid and selected_delta <= threshold)
    if role_mode == 'route-eta-delta':
        nearest = nearest or {}
        cpa = cpa or {}
        local = local or {}
        valid_cpa = bool(cpa.get('valid_cpa', False))
        valid_local = bool(local.get('valid_local', False))
        cpa_score = float(cpa.get('cpa_score', 0.0))
        local_score = float(local.get('local_score', 0.0))
        if valid_cpa and (not valid_local or cpa_score >= local_score):
            selected_eta = float(cpa.get('cpa_route_eta_delta', 0.0))
            selected_valid = True
        else:
            selected_eta = float(nearest.get('route_eta_delta', 0.0))
            selected_valid = valid_local and bool(nearest.get('valid', False))
        threshold = _trace_float(config, 'random_deconflict_route_eta_yield_threshold', 0.02)
        return bool(selected_valid and selected_eta > threshold)
    if role_mode == 'priority':
        threshold = _trace_float(config, 'random_deconflict_priority_yield_threshold', -0.10)
        return float(obs_obj.crossing_priority) <= threshold
    return _trace_agent_index(obs_obj.agent_id) >= 1


def _trace_release_overlap(obs_obj: AgentLocalObservation, nearest: dict, config: dict, *, low_priority: bool) -> bool:
    release_min_separation = max(0.0, _trace_float(config, 'random_safe_finish_yield_release_min_separation', 0.0))
    if release_min_separation <= 0.0 or not bool(nearest['valid']):
        return False
    release_max_closing = _trace_float(config, 'random_safe_finish_yield_release_max_closing_speed', 0.02)
    release_min_route_progress = max(0.0, _trace_float(config, 'random_safe_finish_yield_release_min_route_progress', 0.0))
    return bool(
        low_priority
        and float(nearest['distance']) >= release_min_separation
        and float(nearest['closing_speed']) <= release_max_closing
        and float(obs_obj.route_progress) >= release_min_route_progress
    )


def _trace_weight(config: dict, key: str) -> float:
    weight = max(0.0, _trace_float(config, key, 0.0))
    end_key = f'{key}_end'
    if isinstance(config, dict) and config.get(end_key) is not None:
        weight = max(weight, max(0.0, _trace_float(config, end_key, 0.0)))
    return float(weight)


def _trace_action_bounds(config: dict) -> tuple[tuple[float, float], tuple[float, float]]:
    default_high = [
        max(0.05, _trace_float(config, 'cruise_speed', 0.4)),
        max(0.05, _trace_float(config, 'max_angular_velocity', 0.5)),
    ]
    default_low = [0.0, -default_high[1]]
    raw_low = config.get('action_low', default_low) if isinstance(config, dict) else default_low
    raw_high = config.get('action_high', default_high) if isinstance(config, dict) else default_high
    try:
        low = (float(raw_low[0]), float(raw_low[1]))
        high = (float(raw_high[0]), float(raw_high[1]))
    except (TypeError, ValueError, IndexError):
        low = (float(default_low[0]), float(default_low[1]))
        high = (float(default_high[0]), float(default_high[1]))
    if high[0] <= low[0]:
        high = (low[0] + 1e-3, high[1])
    if high[1] <= low[1]:
        high = (high[0], low[1] + 1e-3)
    return low, high


def _trace_clamp(value: float, low: float, high: float) -> float:
    return min(float(high), max(float(low), float(value)))


def _trace_action_target(
    obs_obj: AgentLocalObservation,
    target_linear: float,
    target_omega: float,
    config: dict,
    *,
    active: bool,
    weighted_active: bool,
    **extra,
) -> dict:
    low, high = _trace_action_bounds(config)
    target_linear = _trace_clamp(target_linear, low[0], high[0])
    target_omega = _trace_clamp(target_omega, low[1], high[1])
    actual_linear = float(obs_obj.final_linear_x)
    actual_omega = float(obs_obj.final_angular_z)
    linear_range = max(high[0] - low[0], 1e-3)
    omega_range = max(high[1] - low[1], 1e-3)
    linear_error_norm = (actual_linear - target_linear) / linear_range
    omega_error_norm = (actual_omega - target_omega) / omega_range
    result = {
        'active': bool(active),
        'weighted_active': bool(weighted_active),
        'target_linear': float(target_linear),
        'target_omega': float(target_omega),
        'actual_linear': actual_linear,
        'actual_omega': actual_omega,
        'linear_error': float(actual_linear - target_linear),
        'omega_error': float(actual_omega - target_omega),
        'linear_error_norm': float(linear_error_norm),
        'omega_error_norm': float(omega_error_norm),
        'l2_error_norm': float(np.hypot(linear_error_norm, omega_error_norm)),
    }
    result.update(extra)
    return result


def _trace_deconflict_target(
    obs_obj: AgentLocalObservation,
    config: dict,
    *,
    active: bool,
    weighted_active: bool,
    threat_score: float,
    is_yield: bool,
    nearest: dict | None = None,
) -> dict:
    danger = _trace_clamp(threat_score, 0.0, 1.0)
    danger_power = max(0.1, _trace_float(config, 'random_deconflict_power', 1.0))
    if danger_power != 1.0:
        danger = danger ** danger_power
    standon_speed = max(0.0, _trace_float(config, 'random_deconflict_standon_speed', 0.30))
    yield_speed = max(0.0, _trace_float(config, 'random_deconflict_yield_speed', 0.04))
    yield_danger_scale = min(1.0, max(0.0, _trace_float(config, 'random_deconflict_yield_danger_scale', 1.0)))
    standon_omega = max(0.0, _trace_float(config, 'random_deconflict_standon_omega', 0.12))
    yield_omega = max(0.0, _trace_float(config, 'random_deconflict_yield_omega', 0.50))
    target_linear = yield_speed * _trace_clamp(1.0 - yield_danger_scale * danger, 0.0, 1.0) if is_yield else standon_speed
    target_omega_mag = yield_omega if is_yield else standon_omega
    turn_mode = str(config.get('random_deconflict_turn_mode', 'starboard')).strip().lower()
    turn_sign = -1.0
    if turn_mode == 'away' and nearest is not None:
        rel_y = nearest.get('rel_y')
        if rel_y is not None:
            rel_y_value = float(rel_y)
            if rel_y_value > 0.0:
                turn_sign = -1.0
            elif rel_y_value < 0.0:
                turn_sign = 1.0
    target_omega = turn_sign * target_omega_mag * danger
    return _trace_action_target(
        obs_obj,
        target_linear,
        target_omega,
        config,
        active=active,
        weighted_active=weighted_active,
        danger=float(danger),
    )


def _trace_safe_finish_target(
    obs_obj: AgentLocalObservation,
    nearest: dict,
    config: dict,
    *,
    team_min_separation: float,
    active: bool,
    weighted_active: bool,
) -> dict:
    distance = max(0.0, float(obs_obj.distance_to_goal))
    goal_tolerance = max(0.05, _trace_float(config, 'random_safe_finish_goal_tolerance', 0.8))
    max_distance = max(goal_tolerance + 0.05, _trace_float(config, 'random_safe_finish_max_distance', 13.0))
    hold_distance = max(0.0, _trace_float(config, 'random_safe_finish_hold_distance', 0.0))
    min_team_separation = max(0.0, _trace_float(config, 'random_safe_finish_min_team_separation', 2.7))
    full_team_separation = max(
        min_team_separation + 0.05,
        _trace_float(config, 'random_safe_finish_full_team_separation', 4.0),
    )
    min_neighbor_separation = max(0.0, _trace_float(config, 'random_safe_finish_min_neighbor_separation', 2.7))
    target_speed = max(0.0, _trace_float(config, 'random_safe_finish_target_speed', 0.18))
    min_speed_scale = min(1.0, max(0.0, _trace_float(config, 'random_safe_finish_min_speed_scale', 0.35)))
    low_priority_threshold = _trace_float(config, 'random_safe_finish_low_priority_threshold', -0.5)
    low_priority_multiplier = max(0.0, _trace_float(config, 'random_safe_finish_low_priority_speed_multiplier', 1.0))
    max_omega = max(0.0, _trace_float(config, 'random_safe_finish_max_omega', 0.10))
    speed_scale = _trace_clamp(
        (distance - (0.50 * goal_tolerance)) / max(max_distance - (0.50 * goal_tolerance), 1e-3),
        min_speed_scale,
        1.0,
    )
    separation_source = float(team_min_separation)
    scale_min_separation = min_team_separation
    if _trace_bool(config, 'random_safe_finish_local_separation_scale', False):
        separation_source = float(nearest['distance']) if bool(nearest['valid']) else full_team_separation
        scale_min_separation = min_neighbor_separation
    full_scale_separation = max(scale_min_separation + 0.05, full_team_separation)
    separation_scale = _trace_clamp(
        (separation_source - scale_min_separation) / max(full_scale_separation - scale_min_separation, 1e-3),
        0.0,
        1.0,
    )
    target_linear = target_speed * speed_scale * separation_scale
    low_priority = float(obs_obj.crossing_priority) <= low_priority_threshold
    if low_priority:
        target_linear *= low_priority_multiplier
    hold = hold_distance > 0.0 and distance <= hold_distance
    heading_error = float(obs_obj.heading_error)
    target_omega = -_trace_clamp(heading_error / 0.85, -1.0, 1.0) * max_omega
    if hold:
        target_linear = 0.0
        target_omega = 0.0
    return _trace_action_target(
        obs_obj,
        target_linear,
        target_omega,
        config,
        active=active,
        weighted_active=weighted_active,
        speed_scale=float(speed_scale),
        separation_scale=float(separation_scale),
        separation_source=float(separation_source),
        low_priority=bool(low_priority),
        hold=bool(hold),
    )


def _trace_offroute_target(
    obs_obj: AgentLocalObservation,
    config: dict,
    *,
    active: bool,
    weighted_active: bool,
) -> dict:
    abs_cte = abs(float(obs_obj.cross_track_error))
    min_abs_cte = max(0.0, _trace_float(config, 'random_offroute_finish_min_abs_cte', 1.20))
    full_abs_cte = max(min_abs_cte + 0.05, _trace_float(config, 'random_offroute_finish_full_abs_cte', 3.00))
    cte_urgency = _trace_clamp((abs_cte - min_abs_cte) / max(full_abs_cte - min_abs_cte, 1e-3), 0.0, 1.0)
    target_speed = max(0.0, _trace_float(config, 'random_offroute_finish_target_speed', 0.14))
    min_speed = max(0.0, _trace_float(config, 'random_offroute_finish_min_speed', 0.06))
    max_omega = max(0.0, _trace_float(config, 'random_offroute_finish_max_omega', 0.18))
    omega_reference = max(0.05, _trace_float(config, 'random_offroute_finish_omega_reference', 0.65))
    target_linear = target_speed - (target_speed - min_speed) * cte_urgency
    target_omega = _trace_goal_heading_omega_sign(config) * _trace_clamp(float(obs_obj.heading_error) / omega_reference, -1.0, 1.0) * max_omega * (0.45 + 0.55 * cte_urgency)
    return _trace_action_target(
        obs_obj,
        target_linear,
        target_omega,
        config,
        active=active,
        weighted_active=weighted_active,
        cte_urgency=float(cte_urgency),
    )


def _trace_cte_recovery_target(
    obs_obj: AgentLocalObservation,
    config: dict,
    *,
    active: bool,
    weighted_active: bool,
) -> dict:
    signed_cte = _trace_signed_cte(obs_obj, config)
    abs_cte = abs(signed_cte)
    min_abs_cte = max(0.0, _trace_float(config, 'random_cte_recovery_min_abs_cte', 1.10))
    full_abs_cte = max(min_abs_cte + 0.05, _trace_float(config, 'random_cte_recovery_full_abs_cte', 3.00))
    cte_urgency = _trace_clamp((abs_cte - min_abs_cte) / max(full_abs_cte - min_abs_cte, 1e-3), 0.0, 1.0)
    target_speed = max(0.0, _trace_float(config, 'random_cte_recovery_target_speed', 0.18))
    min_speed = max(0.0, _trace_float(config, 'random_cte_recovery_min_speed', 0.07))
    max_omega = max(0.0, _trace_float(config, 'random_cte_recovery_max_omega', 0.30))
    omega_reference = max(0.05, _trace_float(config, 'random_cte_recovery_omega_reference', 0.55))
    speed_cte_slowdown = max(0.0, _trace_float(config, 'random_cte_recovery_speed_cte_slowdown', 1.0))
    target_linear = target_speed - (target_speed - min_speed) * _trace_clamp(cte_urgency * speed_cte_slowdown, 0.0, 1.0)
    recovery_heading_error = float(obs_obj.heading_error)
    target_omega = _trace_goal_heading_omega_sign(config) * _trace_clamp(recovery_heading_error / omega_reference, -1.0, 1.0) * max_omega
    omega_mode = str(config.get('random_cte_recovery_omega_mode', 'goal-heading')).strip().lower().replace('_', '-')
    if omega_mode == 'signed-cte':
        cte_sign = 0.0 if abs(signed_cte) <= 1e-6 else float(np.sign(signed_cte))
        target_omega = -cte_sign * max_omega * (0.45 + 0.55 * cte_urgency)
    elif omega_mode == 'signed-cte-inverted':
        cte_sign = 0.0 if abs(signed_cte) <= 1e-6 else float(np.sign(signed_cte))
        target_omega = cte_sign * max_omega * (0.45 + 0.55 * cte_urgency)
    elif omega_mode == 'goal-cte-lookahead':
        cte_lookahead = max(0.25, _trace_float(config, 'random_cte_recovery_cte_lookahead', 4.0))
        cte_heading_scale = max(0.0, _trace_float(config, 'random_cte_recovery_cte_heading_scale', 1.0))
        cte_heading_bias = math.atan2(signed_cte, cte_lookahead) * cte_heading_scale
        recovery_heading_error = math.atan2(
            math.sin(float(obs_obj.heading_error) + cte_heading_bias),
            math.cos(float(obs_obj.heading_error) + cte_heading_bias),
        )
        target_omega = _trace_goal_heading_omega_sign(config) * _trace_clamp(recovery_heading_error / omega_reference, -1.0, 1.0) * max_omega
    speed_heading_gate = min(1.0, max(0.0, _trace_float(config, 'random_cte_recovery_speed_heading_gate', 0.0)))
    if speed_heading_gate > 0.0:
        recovery_alignment = max(0.0, math.cos(min(abs(recovery_heading_error), math.pi / 2.0)))
        target_linear *= (1.0 - speed_heading_gate) + speed_heading_gate * recovery_alignment
        target_linear = max(target_linear, min_speed)
    return _trace_action_target(
        obs_obj,
        target_linear,
        target_omega,
        config,
        active=active,
        weighted_active=weighted_active,
        cte_urgency=float(cte_urgency),
        signed_cte=float(signed_cte),
        omega_mode=omega_mode,
    )


def _trace_random_mask_diagnostics(
    obs_obj: AgentLocalObservation,
    *,
    scenario_name: str,
    team_min_separation: float,
    config: dict,
) -> dict:
    _, nearest = _trace_nearest_neighbor(obs_obj)
    cpa = _trace_cpa_threat(obs_obj, config)
    local = _trace_local_threat(config, nearest)
    scenario_mask = str(scenario_name) in {'two_usv_random_encounter', 'three_usv_random_encounter'}
    distance = max(0.0, float(obs_obj.distance_to_goal))
    route_progress = float(obs_obj.route_progress)
    signed_cte = _trace_signed_cte(obs_obj, config)
    abs_cte = abs(signed_cte)
    phase = float(obs_obj.conflict_phase)
    team_min = float(team_min_separation) if np.isfinite(team_min_separation) else 1e3
    valid_cpa = bool(cpa['valid_cpa'])
    valid_local = bool(local['valid_local'])
    cpa_score = float(cpa['cpa_score'])
    local_score = float(local['local_score'])
    threat_score = max(cpa_score, local_score)
    valid_threat = valid_cpa or valid_local

    deconf_goal_tolerance = max(0.05, _trace_float(config, 'random_deconflict_goal_tolerance', 0.8))
    deconf_max_distance = max(deconf_goal_tolerance + 0.05, _trace_float(config, 'random_deconflict_max_distance', 13.0))
    deconf_unfinished = distance > deconf_goal_tolerance and distance <= deconf_max_distance
    is_yield = _trace_is_yield(obs_obj, config, nearest=nearest, cpa=cpa, local=local)
    role_weight = max(
        0.0,
        _trace_float(config, 'random_deconflict_yield_weight' if is_yield else 'random_deconflict_standon_weight', 1.0 if is_yield else 0.35),
    )
    deconf_release_overlap = _trace_release_overlap(obs_obj, nearest, config, low_priority=is_yield)
    deconf_weight = _trace_weight(config, 'random_deconflict_weight')
    deconf_active = bool(
        scenario_mask
        and deconf_unfinished
        and valid_threat
        and threat_score > 0.0
        and role_weight > 0.0
        and not deconf_release_overlap
    )
    deconf_weighted_active = deconf_active and deconf_weight > 0.0

    finish_goal_tolerance = max(0.05, _trace_float(config, 'random_safe_finish_goal_tolerance', 0.8))
    finish_max_distance = max(finish_goal_tolerance + 0.05, _trace_float(config, 'random_safe_finish_max_distance', 13.0))
    hold_distance = max(0.0, _trace_float(config, 'random_safe_finish_hold_distance', 0.0))
    finish_phase_min = _trace_float(config, 'random_safe_finish_phase_min', -1.0)
    finish_min_team_sep = max(0.0, _trace_float(config, 'random_safe_finish_min_team_separation', 2.7))
    finish_min_neighbor_sep = max(0.0, _trace_float(config, 'random_safe_finish_min_neighbor_separation', 2.7))
    finish_max_abs_cte = max(0.0, _trace_float(config, 'random_safe_finish_max_abs_cte', 0.0))
    finish_max_cpa = max(0.0, _trace_float(config, 'random_safe_finish_max_cpa_score', 0.0))
    finish_max_local = max(0.0, _trace_float(config, 'random_safe_finish_max_local_score', 0.0))
    low_priority_threshold = _trace_float(config, 'random_safe_finish_low_priority_threshold', -0.5)
    finish_unfinished = distance > finish_goal_tolerance and distance <= finish_max_distance
    finish_hold = hold_distance > 0.0 and distance <= hold_distance
    finish_phase = phase >= finish_phase_min
    finish_team_clear = team_min >= finish_min_team_sep
    finish_neighbor_clear = (not bool(nearest['valid'])) or float(nearest['distance']) >= finish_min_neighbor_sep
    finish_cte_clear = finish_max_abs_cte <= 0.0 or abs_cte <= finish_max_abs_cte
    finish_cpa_clear = (not valid_cpa) or cpa_score <= finish_max_cpa
    finish_local_clear = (not valid_local) or local_score <= finish_max_local
    low_priority = float(obs_obj.crossing_priority) <= low_priority_threshold
    finish_release_overlap = _trace_release_overlap(obs_obj, nearest, config, low_priority=low_priority)
    finish_weight = _trace_weight(config, 'random_safe_finish_weight')
    finish_base_candidate = finish_unfinished and finish_team_clear and finish_neighbor_clear and finish_cte_clear
    finish_mask = finish_base_candidate and ((finish_cpa_clear and finish_local_clear) or finish_release_overlap)
    finish_block = bool(
        deconf_active
        and finish_base_candidate
        and ((valid_cpa and cpa_score > finish_max_cpa) or (valid_local and local_score > finish_max_local))
        and not finish_release_overlap
    )
    safe_finish_active = bool(scenario_mask and finish_phase and ((finish_mask and not finish_block) or finish_hold))
    safe_finish_weighted_active = safe_finish_active and finish_weight > 0.0

    off_goal_tolerance = max(0.05, _trace_float(config, 'random_offroute_finish_goal_tolerance', 0.8))
    off_max_distance = max(off_goal_tolerance + 0.05, _trace_float(config, 'random_offroute_finish_max_distance', 13.0))
    off_min_distance = max(off_goal_tolerance + 0.05, _trace_float(config, 'random_offroute_finish_min_distance', 1.60))
    off_min_route_progress = min(1.0, max(0.0, _trace_float(config, 'random_offroute_finish_route_progress_min', 0.82)))
    off_min_abs_cte = max(0.0, _trace_float(config, 'random_offroute_finish_min_abs_cte', 1.20))
    off_phase_min = _trace_float(config, 'random_offroute_finish_phase_min', -1.0)
    off_min_team_sep = max(0.0, _trace_float(config, 'random_offroute_finish_min_team_separation', 2.75))
    off_min_neighbor_sep = max(0.0, _trace_float(config, 'random_offroute_finish_min_neighbor_separation', 2.90))
    off_max_cpa = max(0.0, _trace_float(config, 'random_offroute_finish_max_cpa_score', 0.0))
    off_max_local = max(0.0, _trace_float(config, 'random_offroute_finish_max_local_score', 0.0))
    off_unfinished = distance > off_goal_tolerance and distance <= off_max_distance and distance >= off_min_distance
    offroute_gate = route_progress >= off_min_route_progress and abs_cte >= off_min_abs_cte
    off_team_clear = team_min >= off_min_team_sep
    off_neighbor_clear = (not bool(nearest['valid'])) or float(nearest['distance']) >= off_min_neighbor_sep
    off_cpa_clear = (not valid_cpa) or cpa_score <= off_max_cpa
    off_local_clear = (not valid_local) or local_score <= off_max_local
    off_threat_clear = off_cpa_clear and off_local_clear and not deconf_active
    if _trace_bool(config, 'random_offroute_finish_allow_threat_overlap', False):
        off_threat_clear = True
    offroute_weight = _trace_weight(config, 'random_offroute_finish_weight')
    offroute_active = bool(
        scenario_mask
        and off_unfinished
        and offroute_gate
        and phase >= off_phase_min
        and off_team_clear
        and off_neighbor_clear
        and off_threat_clear
    )
    offroute_weighted_active = offroute_active and offroute_weight > 0.0

    cte_goal_tolerance = max(0.05, _trace_float(config, 'random_cte_recovery_goal_tolerance', 0.8))
    cte_max_distance = max(cte_goal_tolerance + 0.05, _trace_float(config, 'random_cte_recovery_max_distance', 13.0))
    cte_min_abs = max(0.0, _trace_float(config, 'random_cte_recovery_min_abs_cte', 1.10))
    cte_min_neighbor_sep = max(0.0, _trace_float(config, 'random_cte_recovery_min_neighbor_separation', 0.85))
    if _trace_bool(config, 'random_cte_recovery_all_agents', False):
        cte_agent_indices = ()
    else:
        cte_agent_indices = tuple(int(index) for index in (config.get('random_cte_recovery_agent_indices', ()) or ()))
    cte_unfinished = distance > cte_goal_tolerance and distance <= cte_max_distance
    cte_gate = abs_cte >= cte_min_abs
    cte_role = (not cte_agent_indices) or (_trace_agent_index(str(obs_obj.agent_id)) in cte_agent_indices)
    cte_neighbor_clear = (not bool(nearest['valid'])) or float(nearest['distance']) >= cte_min_neighbor_sep
    cte_threat_clear = True
    if not _trace_bool(config, 'random_cte_recovery_allow_threat_overlap', False):
        cte_threat_clear = ((not valid_cpa) or cpa_score <= 0.0) and ((not valid_local) or local_score <= 0.0) and not deconf_active
    cte_weight = _trace_weight(config, 'random_cte_recovery_weight')
    cte_active = bool(scenario_mask and cte_unfinished and cte_gate and cte_role and cte_neighbor_clear and cte_threat_clear)
    cte_weighted_active = cte_active and cte_weight > 0.0

    return {
        'random_scenario': bool(scenario_mask),
        'team_min_separation': team_min,
        'nearest_closing_speed': float(nearest['closing_speed']),
        'valid_cpa': valid_cpa,
        'cpa_id': cpa['cpa_id'],
        'cpa_distance': cpa['cpa_distance'],
        'cpa_closing_speed': float(cpa['cpa_closing_speed']),
        'cpa_dcpa': float(cpa['cpa_dcpa']),
        'cpa_tcpa': float(cpa['cpa_tcpa']),
        'cpa_priority_delta': float(cpa['cpa_priority_delta']),
        'cpa_route_eta_delta': float(cpa['cpa_route_eta_delta']),
        'cpa_score': cpa_score,
        'nearest_priority_delta': float(nearest['priority_delta']),
        'nearest_route_eta_delta': float(nearest['route_eta_delta']),
        'valid_local': valid_local,
        'local_score': local_score,
        'threat_score': threat_score,
        'deconf_is_yield': bool(is_yield),
        'deconf_role_weight': role_weight,
        'deconf_weight': deconf_weight,
        'deconf_unfinished': bool(deconf_unfinished),
        'deconf_release_overlap': bool(deconf_release_overlap),
        'random_deconflict_active': deconf_active,
        'random_deconflict_weighted_active': bool(deconf_weighted_active),
        'deconf_target': _trace_deconflict_target(
            obs_obj,
            config,
            active=deconf_active,
            weighted_active=deconf_weighted_active,
            threat_score=threat_score,
            is_yield=is_yield,
            nearest=nearest,
        ),
        'finish_unfinished': bool(finish_unfinished),
        'finish_phase': bool(finish_phase),
        'finish_team_clear': bool(finish_team_clear),
        'finish_neighbor_clear': bool(finish_neighbor_clear),
        'finish_cte_clear': bool(finish_cte_clear),
        'finish_cpa_clear': bool(finish_cpa_clear),
        'finish_local_clear': bool(finish_local_clear),
        'finish_base_candidate': bool(finish_base_candidate),
        'finish_release_overlap': bool(finish_release_overlap),
        'finish_weight': finish_weight,
        'random_safe_finish_candidate': bool(scenario_mask and finish_phase and finish_unfinished),
        'random_safe_finish_finish': bool(scenario_mask and finish_phase and finish_mask and not finish_block),
        'random_safe_finish_hold': bool(scenario_mask and finish_phase and finish_hold),
        'random_safe_finish_blocked': finish_block,
        'random_safe_finish_active': safe_finish_active,
        'random_safe_finish_weighted_active': bool(safe_finish_weighted_active),
        'safe_finish_target': _trace_safe_finish_target(
            obs_obj,
            nearest,
            config,
            team_min_separation=team_min,
            active=safe_finish_active,
            weighted_active=safe_finish_weighted_active,
        ),
        'offroute_unfinished': bool(off_unfinished),
        'offroute_gate': bool(offroute_gate),
        'offroute_team_clear': bool(off_team_clear),
        'offroute_neighbor_clear': bool(off_neighbor_clear),
        'offroute_threat_clear': bool(off_threat_clear),
        'offroute_weight': offroute_weight,
        'random_offroute_finish_active': offroute_active,
        'random_offroute_finish_weighted_active': bool(offroute_weighted_active),
        'offroute_target': _trace_offroute_target(
            obs_obj,
            config,
            active=offroute_active,
            weighted_active=offroute_weighted_active,
        ),
        'cte_recovery_unfinished': bool(cte_unfinished),
        'cte_recovery_gate': bool(cte_gate),
        'cte_recovery_role': bool(cte_role),
        'cte_recovery_neighbor_clear': bool(cte_neighbor_clear),
        'cte_recovery_threat_clear': bool(cte_threat_clear),
        'cte_recovery_weight': cte_weight,
        'random_cte_recovery_active': cte_active,
        'random_cte_recovery_weighted_active': bool(cte_weighted_active),
        'cte_recovery_target': _trace_cte_recovery_target(
            obs_obj,
            config,
            active=cte_active,
            weighted_active=cte_weighted_active,
        ),
    }


def _append_timeout_agent_sample(
    windows: dict[str, list[dict]],
    agent_id: str,
    obs_obj: AgentLocalObservation,
    *,
    pairwise_shield_active: bool,
    mask_diagnostics: dict,
    window_limit: int,
) -> None:
    mask_active = {
        'deconf': bool(mask_diagnostics.get('random_deconflict_weighted_active', False)),
        'finish': bool(mask_diagnostics.get('random_safe_finish_weighted_active', False)),
        'offroute': bool(mask_diagnostics.get('random_offroute_finish_weighted_active', False)),
        'cte': bool(mask_diagnostics.get('random_cte_recovery_weighted_active', False)),
    }
    window = windows.setdefault(str(agent_id), [])
    window.append({
        'distance_to_goal': float(obs_obj.distance_to_goal),
        'route_progress': float(obs_obj.route_progress),
        'cross_track_error': float(obs_obj.cross_track_error),
        'raw_cross_track_error': float(getattr(obs_obj, 'raw_cross_track_error', obs_obj.cross_track_error)),
        'heading_error': float(obs_obj.heading_error),
        'final_linear_x': float(obs_obj.final_linear_x),
        'final_angular_z': float(obs_obj.final_angular_z),
        'pairwise_shield_active': bool(pairwise_shield_active),
        'mask_active': mask_active,
    })
    overflow = len(window) - max(1, int(window_limit))
    if overflow > 0:
        del window[:overflow]


def _timeout_attribution_agent(
    agent_id: str,
    final_metrics: dict,
    samples: list[dict],
    *,
    goal_tolerance: float,
) -> dict:
    if not samples:
        return {
            'status': 'unknown',
            'final_distance_to_goal': float(final_metrics.get('distance_to_goal', float('inf'))),
            'final_route_progress': float(final_metrics.get('route_progress', 0.0)),
            'sample_count': 0,
        }

    first = samples[0]
    last = samples[-1]
    distances = [float(item['distance_to_goal']) for item in samples]
    route_progress_values = [float(item['route_progress']) for item in samples]
    abs_cte_values = [abs(float(item['cross_track_error'])) for item in samples]
    linear_values = [max(0.0, float(item['final_linear_x'])) for item in samples]
    omega_values = [abs(float(item['final_angular_z'])) for item in samples]
    heading_values = [abs(float(item['heading_error'])) for item in samples]
    sample_count = max(1, len(samples))
    mask_ratios = {
        key: float(sum(1 for item in samples if bool(item.get('mask_active', {}).get(key, False))) / sample_count)
        for key in ('deconf', 'finish', 'offroute', 'cte')
    }
    pairwise_shield_active_ratio = float(sum(1 for item in samples if bool(item.get('pairwise_shield_active', False))) / sample_count)

    distance_delta = float(last['distance_to_goal'] - first['distance_to_goal'])
    route_progress_delta = float(last['route_progress'] - first['route_progress'])
    final_distance = float(final_metrics.get('distance_to_goal', last['distance_to_goal']))
    final_route_progress = float(final_metrics.get('route_progress', last['route_progress']))
    final_abs_cte = abs(float(final_metrics.get('cross_track_error', last['cross_track_error'])))
    mean_linear = float(np.mean(linear_values))
    stopped_ratio = float(sum(1 for value in linear_values if value <= 0.025) / sample_count)
    near_goal_ratio = float(sum(1 for value in distances if value <= max(goal_tolerance * 1.5, goal_tolerance + 0.25)) / sample_count)
    reached_goal = bool(final_metrics.get('reached_goal', final_distance <= goal_tolerance))

    blockers = []
    if reached_goal:
        status = 'reached_goal'
    else:
        if near_goal_ratio > 0.05 and distance_delta > 0.25:
            blockers.append('near_goal_rebound')
        if final_abs_cte >= 2.0 or float(np.mean(abs_cte_values)) >= 2.0:
            blockers.append('cte_drift')
        if distance_delta > 0.50:
            blockers.append('moving_away')
        if final_distance > max(3.0, goal_tolerance * 3.0) and stopped_ratio >= 0.50:
            blockers.append('stalled_far')
        elif final_distance > max(3.0, goal_tolerance * 3.0) and mean_linear < 0.10:
            blockers.append('slow_far')
        if mask_ratios['cte'] >= 0.50:
            blockers.append('cte_recovery_dominant')
        if mask_ratios['finish'] >= 0.50:
            blockers.append('finish_target_dominant')
        if pairwise_shield_active_ratio >= 0.50:
            blockers.append('shield_dominant')
        status = blockers[0] if blockers else 'unfinished'

    return {
        'status': status,
        'blockers': blockers,
        'sample_count': int(sample_count),
        'first_distance_to_goal': float(first['distance_to_goal']),
        'final_distance_to_goal': final_distance,
        'distance_delta': distance_delta,
        'min_distance_to_goal': float(min(distances)),
        'max_distance_to_goal': float(max(distances)),
        'first_route_progress': float(first['route_progress']),
        'final_route_progress': final_route_progress,
        'route_progress_delta': route_progress_delta,
        'min_route_progress': float(min(route_progress_values)),
        'max_route_progress': float(max(route_progress_values)),
        'final_abs_cte': final_abs_cte,
        'mean_abs_cte': float(np.mean(abs_cte_values)),
        'max_abs_cte': float(max(abs_cte_values)),
        'mean_linear_x': mean_linear,
        'mean_abs_omega': float(np.mean(omega_values)),
        'mean_abs_heading_error': float(np.mean(heading_values)),
        'stopped_ratio': stopped_ratio,
        'near_goal_ratio': near_goal_ratio,
        'pairwise_shield_active_ratio': pairwise_shield_active_ratio,
        'mask_weighted_active_ratio': mask_ratios,
    }


def _timeout_attribution_summary(
    final_agent_metrics: dict,
    windows: dict[str, list[dict]],
    *,
    goal_tolerance: float,
) -> dict:
    agents = {}
    blocker_counts: dict[str, int] = {}
    unreached_agents = []
    for agent_id in sorted(final_agent_metrics):
        agent_summary = _timeout_attribution_agent(
            agent_id,
            final_agent_metrics[agent_id],
            windows.get(agent_id, []),
            goal_tolerance=goal_tolerance,
        )
        agents[agent_id] = agent_summary
        if agent_summary.get('status') != 'reached_goal':
            unreached_agents.append(agent_id)
            blockers = agent_summary.get('blockers') or [agent_summary.get('status', 'unknown')]
            for blocker in blockers:
                blocker_counts[str(blocker)] = blocker_counts.get(str(blocker), 0) + 1
    return {
        'window_steps': int(max((len(samples) for samples in windows.values()), default=0)),
        'goal_tolerance': float(goal_tolerance),
        'unreached_agents': unreached_agents,
        'blocker_counts': blocker_counts,
        'agents': agents,
    }


def evaluate_policy(
    model_path: str | None,
    *,
    policy: str = 'auto',
    episodes: int = 6,
    steps_per_episode: int = 160,
    device: str = 'cpu',
    scenarios: tuple[str, ...] | None = None,
    num_agents: int = 5,
    action_mode: str = 'auto',
    rl_control_mode: str = 'auto',
    max_neighbors: str | int = 'auto',
    max_agents: int | None = None,
    episode_timeout: float | None = None,
    no_progress_timeout: float | None = None,
    seed: int | None = None,
    trace_stride: int = 0,
    trace_raw_observation: bool = False,
    trace_event_separation: float = 0.0,
    trace_event_window: int = 12,
    trace_event_raw_observation: bool = False,
    trace_collision_raw_observation: bool = False,
) -> dict:
    max_episode_attempts = 3
    if seed is not None:
        random.seed(int(seed))
        np.random.seed(int(seed))
        try:
            import torch
            torch.manual_seed(int(seed))
        except ImportError:
            pass
    policy_kind = _detect_policy_kind(policy, model_path)
    policy_impl, env_kwargs, resolved_scenarios = _load_policy_bundle(
        policy_kind,
        model_path,
        device=device,
        num_agents=num_agents,
        scenarios=scenarios,
        episode_timeout=episode_timeout,
        no_progress_timeout=no_progress_timeout,
        rl_control_mode=rl_control_mode,
        action_mode=action_mode,
        max_neighbors=max_neighbors,
        max_agents=max_agents,
    )
    trace_mask_config = getattr(policy_impl, 'trace_mask_config', {})

    env = _create_env(env_kwargs)

    episode_metrics = []
    try:
        for episode in range(episodes):
            scenario_name = resolved_scenarios[episode % len(resolved_scenarios)]
            last_error = None
            for attempt in range(max_episode_attempts):
                try:
                    reset_seed = None if seed is None else int(seed) + episode * max_episode_attempts + attempt
                    observations, info = env.reset(seed=reset_seed, options={'scenario_kind': scenario_name})
                    last_info = info
                    initial_team_mean_goal_distance = _scenario_initial_team_mean_goal_distance(env)
                    scenario_geometry = _scenario_geometry_snapshot(env)
                    if initial_team_mean_goal_distance is None:
                        initial_team_mean_goal_distance = float(info['global_state'][-3])
                    exhausted_horizon = True
                    truncated = False
                    steps = 0
                    running_pairwise_min = float('inf')
                    # --- Smoothness tracking ---
                    prev_vx: dict[str, float] = {}
                    prev_vw: dict[str, float] = {}
                    total_linear_accel = 0.0
                    total_angular_accel = 0.0
                    omega_flip_count = 0
                    omega_saturation_steps = 0
                    total_heading_error = 0.0
                    heading_sign_flip_count = 0
                    prev_heading_sign: dict[str, int] = {}
                    smoothness_samples = 0
                    # --- CTE tracking ---
                    total_abs_cte = 0.0
                    cte_samples = 0
                    # --- Entanglement tracking ---
                    entanglement_steps = 0
                    entanglement_distance = float(env.config.entanglement_distance) if env.config.entanglement_distance > 0 else 4.0
                    # --- COLREGs compliance tracking ---
                    colregs_head_on_steps = 0
                    colregs_crossing_steps = 0
                    colregs_overtaking_steps = 0
                    colregs_compliant_steps = 0
                    colregs_violation_steps = 0
                    colregs_detect_distance = 5.0
                    # Per-pair CPA: {(ego, neighbor): {'min_dist', 'body_y', 'encounter'}}
                    pair_cpa: dict[tuple[str, str], dict] = {}
                    # --- Progress rate tracking ---
                    prev_team_goal_dist = initial_team_mean_goal_distance
                    positive_progress_steps = 0  # steps where distance decreased
                    stall_steps = 0              # steps where distance didn't change or increased
                    trace_samples = []
                    trace_stride_value = max(0, int(trace_stride))
                    trace_event_threshold = max(0.0, float(trace_event_separation))
                    trace_event_window_value = max(0, int(trace_event_window))
                    trace_event_remaining = 0
                    trace_collision_recorded = False
                    timeout_agent_windows: dict[str, list[dict]] = {agent_id: [] for agent_id in env.agent_ids}
                    timeout_window_limit = min(200, max(1, int(steps_per_episode)))

                    if hasattr(policy_impl, 'set_active_scenario'):
                        policy_impl.set_active_scenario(last_info.get('scenario', scenario_name))

                    for step in range(steps_per_episode):
                        if hasattr(policy_impl, 'set_active_scenario'):
                            policy_impl.set_active_scenario(last_info.get('scenario', scenario_name))
                        action_map = {
                            agent_id: policy_impl.predict(observations[agent_id])
                            for agent_id in env.agent_ids
                        }
                        observations, _, terminated_dict, truncated_dict, last_info = env.step(action_map)
                        steps = step + 1
                        step_pair_min = float(last_info['pairwise_min_separation'])
                        # Track per-step goal approach
                        step_team_goal_dist = float(last_info['team_mean_goal_distance'])
                        if step_team_goal_dist < prev_team_goal_dist - 1e-4:
                            positive_progress_steps += 1
                        else:
                            stall_steps += 1
                        prev_team_goal_dist = step_team_goal_dist
                        if np.isfinite(step_pair_min):
                            running_pairwise_min = min(running_pairwise_min, step_pair_min)
                            if step_pair_min < entanglement_distance:
                                entanglement_steps += 1

                        active_scenario_name = str(last_info.get('scenario', scenario_name))
                        stride_trace_step = trace_stride_value > 0 and (step == 0 or (step + 1) % trace_stride_value == 0)
                        event_triggered = trace_event_threshold > 0.0 and step_pair_min <= trace_event_threshold
                        if event_triggered:
                            trace_event_remaining = max(trace_event_remaining, trace_event_window_value + 1)
                        event_trace_step = trace_stride_value <= 0 and trace_event_remaining > 0
                        if stride_trace_step or event_trace_step:
                            trace_samples.append(
                                _trace_episode_sample(
                                    env,
                                    observations,
                                    scenario_name=active_scenario_name,
                                    step=steps,
                                    pairwise_min_separation=step_pair_min,
                                    team_mean_goal_distance=step_team_goal_dist,
                                    goal_completion_ratio=float(last_info.get('goal_completion_ratio', 0.0)),
                                    trace_mask_config=trace_mask_config,
                                    include_raw_observation=(
                                        (bool(trace_raw_observation) and stride_trace_step)
                                        or (bool(trace_event_raw_observation) and event_trace_step)
                                    ),
                                    trace_kind='stride' if stride_trace_step else ('event_trigger' if event_triggered else 'event_window'),
                                )
                            )
                        if event_trace_step:
                            trace_event_remaining = max(0, trace_event_remaining - 1)
                        collision_trace_step = (
                            bool(trace_collision_raw_observation)
                            and not trace_collision_recorded
                            and step_pair_min < float(env.config.collision_distance)
                        )
                        if collision_trace_step:
                            trace_samples.append(
                                _trace_episode_sample(
                                    env,
                                    observations,
                                    scenario_name=active_scenario_name,
                                    step=steps,
                                    pairwise_min_separation=step_pair_min,
                                    team_mean_goal_distance=step_team_goal_dist,
                                    goal_completion_ratio=float(last_info.get('goal_completion_ratio', 0.0)),
                                    trace_mask_config=trace_mask_config,
                                    include_raw_observation=True,
                                    trace_kind='collision',
                                )
                            )
                            trace_collision_recorded = True

                        # Collect smoothness data from actual vehicle state
                        for agent_id in env.agent_ids:
                            obs_obj = env._latest_observations.get(agent_id)
                            if obs_obj is None:
                                continue
                            vx = max(0.0, float(obs_obj.final_linear_x))
                            vw = float(obs_obj.final_angular_z)
                            he = float(obs_obj.heading_error)
                            total_heading_error += abs(he)
                            if abs(vw) > 0.35:
                                omega_saturation_steps += 1
                            cte_val = float(obs_obj.cross_track_error)
                            total_abs_cte += abs(cte_val)
                            cte_samples += 1
                            mask_diagnostics = _trace_random_mask_diagnostics(
                                obs_obj,
                                scenario_name=active_scenario_name,
                                team_min_separation=step_pair_min,
                                config=trace_mask_config,
                            )
                            pairwise_diagnostics = env.pairwise_shield_diagnostics(agent_id, obs_obj)
                            _append_timeout_agent_sample(
                                timeout_agent_windows,
                                agent_id,
                                obs_obj,
                                pairwise_shield_active=bool(pairwise_diagnostics.get('active', False)),
                                mask_diagnostics=mask_diagnostics,
                                window_limit=timeout_window_limit,
                            )
                            if agent_id in prev_vx:
                                total_linear_accel += abs(vx - prev_vx[agent_id])
                                total_angular_accel += abs(vw - prev_vw[agent_id])
                                if prev_vw[agent_id] * vw < 0.0 and abs(prev_vw[agent_id]) > 0.03 and abs(vw) > 0.03:
                                    omega_flip_count += 1
                            he_sign = 1 if he >= 0 else -1
                            if agent_id in prev_heading_sign and prev_heading_sign[agent_id] != he_sign and abs(he) > 0.03:
                                heading_sign_flip_count += 1
                            prev_heading_sign[agent_id] = he_sign
                            prev_vx[agent_id] = vx
                            prev_vw[agent_id] = vw
                            smoothness_samples += 1

                        # --- COLREGs encounter classification & compliance ---
                        # Detection logic is the single source of truth in
                        # `multi_agent_types.classify_encounter_role`.
                        for agent_id in env.agent_ids:
                            obs_obj = env._latest_observations.get(agent_id)
                            if obs_obj is None:
                                continue
                            own_speed = max(0.0, float(obs_obj.speed), float(obs_obj.final_linear_x))
                            angular_z = float(obs_obj.final_angular_z)
                            for neighbor in obs_obj.neighbors:
                                if neighbor.distance <= 1e-3 or neighbor.distance > colregs_detect_distance:
                                    continue
                                bx = float(neighbor.rel_x)
                                by = float(neighbor.rel_y)
                                bvx = float(neighbor.rel_vx)
                                bvy = float(neighbor.rel_vy)
                                encounter_type, _role = classify_encounter_role(
                                    body_x=bx,
                                    body_y=by,
                                    body_vx=bvx,
                                    body_vy=bvy,
                                    distance=float(neighbor.distance),
                                    own_speed=own_speed,
                                    ego_id=agent_id,
                                    neighbor_id=str(getattr(neighbor, 'source_id', '')),
                                    own_yaw=float(obs_obj.yaw),
                                )
                                if encounter_type == ENC_HEAD_ON:
                                    enc_type = 'head_on'
                                    colregs_head_on_steps += 1
                                elif encounter_type == ENC_CROSSING:
                                    enc_type = 'crossing'
                                    colregs_crossing_steps += 1
                                elif encounter_type == ENC_OVERTAKING:
                                    enc_type = 'overtaking'
                                    colregs_overtaking_steps += 1
                                else:
                                    continue
                                # Compliance check: starboard turn = correct
                                if angular_z < -0.03:
                                    colregs_compliant_steps += 1
                                elif angular_z > 0.03:
                                    colregs_violation_steps += 1
                                # Track CPA per directional pair
                                pk = (agent_id, neighbor.source_id)
                                if pk not in pair_cpa:
                                    pair_cpa[pk] = {'min_dist': float('inf'), 'body_y': 0.0, 'encounter': 'none'}
                                if neighbor.distance < pair_cpa[pk]['min_dist']:
                                    pair_cpa[pk]['min_dist'] = neighbor.distance
                                    pair_cpa[pk]['body_y'] = by
                                    pair_cpa[pk]['encounter'] = enc_type

                        terminated = bool(terminated_dict['__all__'])
                        truncated = bool(truncated_dict['__all__'])
                        if terminated or truncated:
                            exhausted_horizon = False
                            break

                    pairwise_min = float(last_info['pairwise_min_separation'])
                    goal_completion_ratio = float(last_info['goal_completion_ratio'])
                    final_team_mean_goal_distance = float(last_info['team_mean_goal_distance'])
                    team_goal_distance_delta = initial_team_mean_goal_distance - final_team_mean_goal_distance
                    team_goal_progress_ratio = 0.0
                    if initial_team_mean_goal_distance > 1e-6:
                        team_goal_progress_ratio = team_goal_distance_delta / initial_team_mean_goal_distance
                    success = goal_completion_ratio >= 0.999
                    collision = pairwise_min < env.config.collision_distance
                    timeout = (truncated or exhausted_horizon) and not success and not collision
                    episode_running_min = running_pairwise_min if np.isfinite(running_pairwise_min) else pairwise_min
                    n_smooth = max(1, smoothness_samples)
                    # Progress rate metrics (step-independent)
                    dt = float(getattr(env.config, 'control_dt', 0.2))
                    cruise = float(getattr(env.config, 'cruise_speed', 0.36))
                    elapsed_time = steps * dt
                    goal_approach_velocity = team_goal_distance_delta / max(elapsed_time, 1e-6)
                    max_possible_distance = cruise * elapsed_time
                    progress_efficiency = team_goal_distance_delta / max(max_possible_distance, 1e-6)
                    progress_efficiency = max(0.0, min(1.0, progress_efficiency))
                    # Projected: if maintained this rate for full episode
                    projected_progress = 0.0
                    if initial_team_mean_goal_distance > 1e-6 and elapsed_time > 1e-6:
                        full_time = steps_per_episode * dt
                        projected_distance = goal_approach_velocity * full_time
                        projected_progress = min(1.0, max(0.0, projected_distance / initial_team_mean_goal_distance))
                    progress_consistency = positive_progress_steps / max(1, steps)
                    # COLREGs aggregation
                    total_colregs = colregs_head_on_steps + colregs_crossing_steps + colregs_overtaking_steps
                    colregs_compliance_ratio = colregs_compliant_steps / max(1, total_colregs)
                    colregs_violation_ratio = colregs_violation_steps / max(1, total_colregs)
                    # CPA pass-side analysis: body_y > 0 at CPA → neighbor on port side → correct starboard pass
                    cpa_encounters = [v for v in pair_cpa.values() if v['encounter'] != 'none']
                    cpa_starboard_passes = sum(1 for v in cpa_encounters if v['body_y'] > 0)
                    cpa_starboard_ratio = cpa_starboard_passes / max(1, len(cpa_encounters))
                    final_agent_metrics = {}
                    for agent_id in env.agent_ids:
                        obs_obj = env._latest_observations.get(agent_id)
                        if obs_obj is None:
                            continue
                        final_agent_metrics[agent_id] = {
                            'distance_to_goal': float(obs_obj.distance_to_goal),
                            'route_progress': float(obs_obj.route_progress),
                            'cross_track_error': float(obs_obj.cross_track_error),
                            'raw_cross_track_error': float(getattr(obs_obj, 'raw_cross_track_error', obs_obj.cross_track_error)),
                            'cross_track_overflow': float(getattr(obs_obj, 'cross_track_overflow', 0.0)),
                            'heading_error': float(obs_obj.heading_error),
                            'crossing_priority': float(obs_obj.crossing_priority),
                            'final_linear_x': float(obs_obj.final_linear_x),
                            'final_angular_z': float(obs_obj.final_angular_z),
                            'reached_goal': bool(obs_obj.distance_to_goal <= env.config.goal_tolerance),
                        }
                    timeout_attribution = _timeout_attribution_summary(
                        final_agent_metrics,
                        timeout_agent_windows,
                        goal_tolerance=float(env.config.goal_tolerance),
                    )
                    episode_metric = {
                            'episode': episode,
                            'scenario': scenario_name,
                            'reset_seed': reset_seed,
                            'scenario_geometry': scenario_geometry,
                            'active_agent_ids': list(last_info.get('active_agent_ids', [])),
                            'inactive_agent_ids': list(last_info.get('inactive_agent_ids', [])),
                            'runtime_agent_ids': list(last_info.get('runtime_agent_ids', [])),
                            'steps': steps,
                            'success': success,
                            'collision': collision,
                            'timeout': timeout,
                            'pairwise_min_separation': pairwise_min,
                            'episode_min_separation': episode_running_min,
                            'pairwise_mean_separation': float(last_info['pairwise_mean_separation']),
                            'goal_completion_ratio': goal_completion_ratio,
                            'initial_team_mean_goal_distance': initial_team_mean_goal_distance,
                            'team_mean_goal_distance': final_team_mean_goal_distance,
                            'final_agent_metrics': final_agent_metrics,
                            'timeout_attribution': timeout_attribution,
                            'team_goal_distance_delta': team_goal_distance_delta,
                            'team_goal_progress_ratio': team_goal_progress_ratio,
                            # Progress rate metrics (step-independent)
                            'goal_approach_velocity': goal_approach_velocity,
                            'progress_efficiency': progress_efficiency,
                            'projected_progress': projected_progress,
                            'progress_consistency': progress_consistency,
                            # Smoothness metrics (per-step averages over all agents)
                            'mean_linear_accel': total_linear_accel / n_smooth,
                            'mean_angular_accel': total_angular_accel / n_smooth,
                            'omega_flip_count': omega_flip_count,
                            'omega_saturation_ratio': omega_saturation_steps / n_smooth,
                            'heading_sign_flip_count': heading_sign_flip_count,
                            'mean_heading_error': total_heading_error / n_smooth,
                            # Path tracking metrics
                            'mean_cross_track_error': total_abs_cte / max(1, cte_samples),
                            # Entanglement metrics
                            'entanglement_steps': entanglement_steps,
                            'entanglement_ratio': entanglement_steps / max(1, steps),
                            # COLREGs compliance metrics
                            'colregs_encounter_steps': total_colregs,
                            'colregs_head_on_steps': colregs_head_on_steps,
                            'colregs_crossing_steps': colregs_crossing_steps,
                            'colregs_overtaking_steps': colregs_overtaking_steps,
                            'colregs_compliance_ratio': colregs_compliance_ratio,
                            'colregs_violation_ratio': colregs_violation_ratio,
                            'cpa_starboard_pass_ratio': cpa_starboard_ratio,
                            'cpa_encounter_count': len(cpa_encounters),
                        }
                    if trace_stride_value > 0 or trace_event_threshold > 0.0 or bool(trace_collision_raw_observation):
                        episode_metric['trace_samples'] = trace_samples
                    episode_metrics.append(episode_metric)
                    break
                except RuntimeError as exc:
                    last_error = exc
                    print(
                        f'Warning: episode {episode} ({scenario_name}) attempt {attempt + 1}/{max_episode_attempts} '
                        f'failed with {exc}. Recreating environment and retrying.'
                    )
                    env.close()
                    env = _create_env(env_kwargs)
            else:
                raise RuntimeError(
                    f'Failed to evaluate episode {episode} for scenario {scenario_name} '
                    f'after {max_episode_attempts} attempts.'
                ) from last_error
    finally:
        env.close()

    return {
        'policy': policy_kind,
        'model': model_path,
        'seed': seed,
        'episodes': len(episode_metrics),
        'agent_namespaces': list(env.agent_ids),
        'action_mode': env.config.action_mode,
        'max_neighbors': env.config.max_neighbors,
        'scenarios': list(resolved_scenarios),
        'collision_rate': sum(item['collision'] for item in episode_metrics) / max(1, len(episode_metrics)),
        'success_rate': sum(item['success'] for item in episode_metrics) / max(1, len(episode_metrics)),
        'timeout_rate': sum(item['timeout'] for item in episode_metrics) / max(1, len(episode_metrics)),
        'mean_pairwise_min_separation': float(np.mean([item['pairwise_min_separation'] for item in episode_metrics])),
        'worst_pairwise_min_separation': float(np.min([item['pairwise_min_separation'] for item in episode_metrics])),
        'mean_episode_min_separation': float(np.mean([item['episode_min_separation'] for item in episode_metrics])),
        'worst_episode_min_separation': float(np.min([item['episode_min_separation'] for item in episode_metrics])),
        'mean_goal_completion_ratio': float(np.mean([item['goal_completion_ratio'] for item in episode_metrics])),
        'mean_team_goal_distance_delta': float(np.mean([item['team_goal_distance_delta'] for item in episode_metrics])),
        'mean_team_goal_progress_ratio': float(np.mean([item['team_goal_progress_ratio'] for item in episode_metrics])),
        # Progress rate metrics (step-independent)
        'mean_goal_approach_velocity': float(np.mean([item['goal_approach_velocity'] for item in episode_metrics])),
        'mean_progress_efficiency': float(np.mean([item['progress_efficiency'] for item in episode_metrics])),
        'mean_projected_progress': float(np.mean([item['projected_progress'] for item in episode_metrics])),
        'mean_progress_consistency': float(np.mean([item['progress_consistency'] for item in episode_metrics])),
        # Smoothness metrics (independent quality indicators)
        'mean_linear_accel': float(np.mean([item['mean_linear_accel'] for item in episode_metrics])),
        'mean_angular_accel': float(np.mean([item['mean_angular_accel'] for item in episode_metrics])),
        'mean_omega_flip_count': float(np.mean([item['omega_flip_count'] for item in episode_metrics])),
        'mean_omega_saturation_ratio': float(np.mean([item['omega_saturation_ratio'] for item in episode_metrics])),
        'mean_heading_sign_flip_count': float(np.mean([item['heading_sign_flip_count'] for item in episode_metrics])),
        'mean_heading_error': float(np.mean([item['mean_heading_error'] for item in episode_metrics])),
        # Path tracking metrics
        'mean_cross_track_error': float(np.mean([item['mean_cross_track_error'] for item in episode_metrics])),
        # Entanglement metrics
        'mean_entanglement_ratio': float(np.mean([item['entanglement_ratio'] for item in episode_metrics])),
        # COLREGs compliance metrics
        'mean_colregs_compliance_ratio': float(np.mean([item['colregs_compliance_ratio'] for item in episode_metrics])),
        'mean_colregs_violation_ratio': float(np.mean([item['colregs_violation_ratio'] for item in episode_metrics])),
        'mean_cpa_starboard_pass_ratio': float(np.mean([item['cpa_starboard_pass_ratio'] for item in episode_metrics])),
        'scenario_summaries': _scenario_summary(episode_metrics),
        'episode_metrics': episode_metrics,
    }


def evaluate_checkpoint(
    model_path: str,
    *,
    episodes: int = 6,
    steps_per_episode: int = 160,
    device: str = 'cpu',
    scenarios: tuple[str, ...] | None = None,
    episode_timeout: float | None = None,
    no_progress_timeout: float | None = None,
    seed: int | None = None,
) -> dict:
    return evaluate_policy(
        model_path,
        policy='mappo',
        episodes=episodes,
        steps_per_episode=steps_per_episode,
        device=device,
        scenarios=scenarios,
        episode_timeout=episode_timeout,
        no_progress_timeout=no_progress_timeout,
        seed=seed,
    )


def main():
    args = parse_args()
    policy_kind = _detect_policy_kind(args.policy, args.model)
    if policy_kind != 'zero' and not args.model:
        raise RuntimeError(f'Policy type {policy_kind} requires --model.')

    summary = evaluate_policy(
        args.model,
        policy=policy_kind,
        episodes=args.episodes,
        steps_per_episode=args.steps_per_episode,
        device=args.device,
        scenarios=tuple(args.scenarios) if args.scenarios else None,
        num_agents=args.num_agents,
        rl_control_mode=args.rl_control_mode,
        action_mode=args.action_mode,
        max_neighbors=args.max_neighbors,
        max_agents=args.max_agents,
        episode_timeout=args.episode_timeout,
        no_progress_timeout=args.no_progress_timeout,
        seed=args.seed,
        trace_stride=args.trace_stride,
        trace_raw_observation=args.trace_raw_observation,
        trace_event_separation=args.trace_event_separation,
        trace_event_window=args.trace_event_window,
        trace_event_raw_observation=args.trace_event_raw_observation,
        trace_collision_raw_observation=args.trace_collision_raw_observation,
    )

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved multi-agent evaluation summary to {output_path}')



if __name__ == '__main__':
    main()