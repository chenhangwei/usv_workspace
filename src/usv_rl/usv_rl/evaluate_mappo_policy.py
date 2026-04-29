import argparse
import json
from pathlib import Path
import random

import numpy as np

from .config import ActionBounds, RewardConfig
from .multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig
from .multi_agent_scenarios import MultiAgentScenarioFactory
from .multi_agent_types import AgentLocalObservation, ENCOUNTER_TYPE_COUNT, NEIGHBOR_FEATURE_COUNT
from .observation_normalizer import ObservationNormalizer
from .policies import load_policy


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

    def predict(self, observation: np.ndarray) -> np.ndarray:
        blended = np.zeros(self.action_dim, dtype=np.float32)
        for policy, weight in self._weighted_policies:
            blended += np.asarray(policy.predict(observation), dtype=np.float32) * np.float32(weight)
        return blended


class ScenarioRouterPolicy:
    def __init__(self, router_config: dict, *, device: str = 'cpu'):
        raw_scenario_models = router_config.get('scenario_models') or {}
        if not isinstance(raw_scenario_models, dict):
            raise RuntimeError('MAPPO router config requires scenario_models to be a mapping when provided.')

        raw_scenario_blends = router_config.get('scenario_blends') or {}
        if not isinstance(raw_scenario_blends, dict):
            raise RuntimeError('MAPPO router config requires scenario_blends to be a mapping when provided.')

        if not raw_scenario_models and not raw_scenario_blends:
            raise RuntimeError('MAPPO router config requires at least one scenario_models or scenario_blends entry.')

        overlapping_scenarios = set(raw_scenario_models) & set(raw_scenario_blends)
        if overlapping_scenarios:
            joined = ', '.join(sorted(str(value) for value in overlapping_scenarios))
            raise RuntimeError(f'Scenarios cannot be defined in both scenario_models and scenario_blends: {joined}')

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

        self._default_model = str(router_config.get('default_model') or next(iter(self._scenario_models.values())))
        self._active_scenario: str | None = None
        self._policies: dict[str, MappoActorPolicy] = {}
        self._scenario_policies: dict[str, MappoActorPolicy | WeightedBlendPolicy] = {}

        reference_policy: MappoActorPolicy | None = None
        unique_paths = {self._default_model, *self._scenario_models.values()}
        for blend_specs in self._scenario_blends.values():
            unique_paths.update(model_path for model_path, _ in blend_specs)
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

        self.action_dim = self._default_policy.action_dim
        self.obs_dim = self._default_policy.obs_dim

    def set_active_scenario(self, scenario_name: str | None) -> None:
        self._active_scenario = str(scenario_name) if scenario_name else None

    def _resolve_policy(self) -> MappoActorPolicy | WeightedBlendPolicy:
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
    if int(action_dim) != 2:
        source = model_path or 'the selected policy'
        raise RuntimeError(
            'Pure RL evaluation only supports 2D full-action policies. '
            f'Received action_dim={int(action_dim)} from {source}. '
            'Please evaluate a pure policy checkpoint trained with action_mode="full".'
        )


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
    if resolved_action_mode != 'full':
        raise RuntimeError(
            'Pure RL evaluation only supports checkpoints exported with action_mode="full". '
            f'Received action_mode="{resolved_action_mode}" from {model_path}.'
        )
    resolved_max_neighbors = int(checkpoint.get('max_neighbors', max(1, int((policy.obs_dim - AgentLocalObservation.ego_feature_size() - ENCOUNTER_TYPE_COUNT) // NEIGHBOR_FEATURE_COUNT))))
    resolved_max_agents = int(checkpoint.get('max_agents', len(agent_namespaces)))
    env_kwargs = {
        'agent_namespaces': agent_namespaces,
        'enable_rl_backend': True,
        'rl_control_mode': 'pure',
        'action_mode': resolved_action_mode,
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
        'collision_distance': float(checkpoint.get('collision_distance', 0.5)),
        'near_miss_distance': float(checkpoint.get('near_miss_distance', 1.5)),
        'scenario_neighbor_speed': float(checkpoint.get('scenario_neighbor_speed', 0.45)),
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


def _create_env(env_kwargs: dict) -> MultiAgentEnv:
    return MultiAgentEnv(MultiAgentEnvConfig(**env_kwargs))


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
                                if bx <= 0.0:
                                    continue
                                bvx = float(neighbor.rel_vx)
                                bvy = float(neighbor.rel_vy)
                                closing = -((bx * bvx) + (by * bvy)) / max(neighbor.distance, 1e-3)
                                nfwd = own_speed + bvx
                                same_lane = bx > 0.8 and abs(by) < 1.5
                                is_overtaking = (
                                    same_lane and own_speed > 0.18 and bvx < -0.03
                                    and nfwd > 0.05 and nfwd < own_speed - 0.02
                                )
                                opposing = nfwd < 0.05
                                lateral_tol = max(1.3, 0.28 * neighbor.distance)
                                is_head_on = (
                                    not is_overtaking and opposing and closing > 0
                                    and abs(by) < lateral_tol
                                )
                                is_crossing = (
                                    by < -0.35 and closing > -0.05
                                    and not is_overtaking and not is_head_on
                                )
                                enc_type = 'none'
                                if is_head_on:
                                    enc_type = 'head_on'
                                    colregs_head_on_steps += 1
                                elif is_crossing:
                                    enc_type = 'crossing'
                                    colregs_crossing_steps += 1
                                elif is_overtaking:
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
                    episode_metrics.append(
                        {
                            'episode': episode,
                            'scenario': scenario_name,
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
                    )
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
    )

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved multi-agent evaluation summary to {output_path}')



if __name__ == '__main__':
    main()