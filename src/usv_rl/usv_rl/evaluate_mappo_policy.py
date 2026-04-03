import argparse
import json
from pathlib import Path

import numpy as np

from .config import ActionBounds, RewardConfig
from .multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig
from .multi_agent_scenarios import MultiAgentScenarioFactory
from .observation_normalizer import ObservationNormalizer
from .policies import load_policy


def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate an RL policy in the multi-USV environment.')
    parser.add_argument('--policy', choices=['auto', 'mappo', 'bc', 'zero'], default='auto', help='Policy backend.')
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
            self._obs_normalizer.load_state_dict(checkpoint['obs_normalizer'])

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
    if obs_dim is None or int(obs_dim) < 10:
        return 4
    inferred = (int(obs_dim) - 10) // 6
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
        _require_supported_action_dim(policy.action_dim, model_path=model_path)
        agent_namespaces = tuple(checkpoint['agent_namespaces'])
        resolved_scenarios = tuple(scenarios) if scenarios else tuple(checkpoint.get('scenarios', MultiAgentScenarioFactory.available()))
        resolved_rl_control_mode = 'pure'
        resolved_action_mode = str(checkpoint.get('action_mode', 'full'))
        if resolved_action_mode != 'full':
            raise RuntimeError(
                'Pure RL evaluation only supports checkpoints exported with action_mode="full". '
                f'Received action_mode="{resolved_action_mode}" from {model_path}.'
            )
        resolved_max_neighbors = int(checkpoint.get('max_neighbors', max(1, int((policy.obs_dim - 10) // 6))))
        resolved_max_agents = int(checkpoint.get('max_agents', len(agent_namespaces)))
        env_kwargs = {
            'agent_namespaces': agent_namespaces,
            'enable_rl_backend': True,
            'rl_control_mode': resolved_rl_control_mode,
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
            # Smoothness metrics
            'mean_linear_accel': float(np.mean([item['mean_linear_accel'] for item in items])),
            'mean_angular_accel': float(np.mean([item['mean_angular_accel'] for item in items])),
            'mean_omega_flip_count': float(np.mean([item['omega_flip_count'] for item in items])),
            'mean_heading_sign_flip_count': float(np.mean([item['heading_sign_flip_count'] for item in items])),
            'mean_heading_error': float(np.mean([item['mean_heading_error'] for item in items])),
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
) -> dict:
    max_episode_attempts = 3
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
                    observations, info = env.reset(options={'scenario_kind': scenario_name})
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
                    total_heading_error = 0.0
                    heading_sign_flip_count = 0
                    prev_heading_sign: dict[str, int] = {}
                    smoothness_samples = 0

                    for step in range(steps_per_episode):
                        action_map = {
                            agent_id: policy_impl.predict(observations[agent_id])
                            for agent_id in env.agent_ids
                        }
                        observations, _, terminated_dict, truncated_dict, last_info = env.step(action_map)
                        steps = step + 1
                        step_pair_min = float(last_info['pairwise_min_separation'])
                        if np.isfinite(step_pair_min):
                            running_pairwise_min = min(running_pairwise_min, step_pair_min)

                        # Collect smoothness data from actual vehicle state
                        for agent_id in env.agent_ids:
                            obs_obj = env._latest_observations.get(agent_id)
                            if obs_obj is None:
                                continue
                            vx = max(0.0, float(obs_obj.final_linear_x))
                            vw = float(obs_obj.final_angular_z)
                            he = float(obs_obj.heading_error)
                            total_heading_error += abs(he)
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
                            # Smoothness metrics (per-step averages over all agents)
                            'mean_linear_accel': total_linear_accel / n_smooth,
                            'mean_angular_accel': total_angular_accel / n_smooth,
                            'omega_flip_count': omega_flip_count,
                            'heading_sign_flip_count': heading_sign_flip_count,
                            'mean_heading_error': total_heading_error / n_smooth,
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
        # Smoothness metrics (independent quality indicators)
        'mean_linear_accel': float(np.mean([item['mean_linear_accel'] for item in episode_metrics])),
        'mean_angular_accel': float(np.mean([item['mean_angular_accel'] for item in episode_metrics])),
        'mean_omega_flip_count': float(np.mean([item['omega_flip_count'] for item in episode_metrics])),
        'mean_heading_sign_flip_count': float(np.mean([item['heading_sign_flip_count'] for item in episode_metrics])),
        'mean_heading_error': float(np.mean([item['mean_heading_error'] for item in episode_metrics])),
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
    )

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved multi-agent evaluation summary to {output_path}')



if __name__ == '__main__':
    main()