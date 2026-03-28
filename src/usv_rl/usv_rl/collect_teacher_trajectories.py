"""Collect teacher trajectories for offline distillation.

Usage::

    python3 -m usv_rl.collect_teacher_trajectories \
        --model /path/to/a10_step_358400.pt \
        --episodes 200 \
        --steps-per-episode 180 \
        --output /mnt/data/distillation/teacher_trajectories.npz
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from .evaluate_mappo_policy import (
    _create_env,
    _load_mappo_checkpoint,
)
from .config import ActionBounds, RewardConfig
from .multi_agent_env import MultiAgentEnvConfig
from .multi_agent_scenarios import MultiAgentScenarioFactory


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Collect (obs, action) pairs from a frozen MAPPO teacher policy.')
    parser.add_argument('--model', required=True, help='Teacher model checkpoint (.pt).')
    parser.add_argument('--output', required=True, help='Output path (.npz).')
    parser.add_argument('--episodes', type=int, default=200, help='Total episodes to collect.')
    parser.add_argument('--steps-per-episode', type=int, default=180, help='Maximum steps per episode.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name. Repeatable.')
    parser.add_argument('--device', default='cpu', help='Torch device.')
    parser.add_argument('--summary-json', help='Optional JSON path for collection summary.')
    return parser.parse_args(argv)


def _build_env_kwargs_from_checkpoint(checkpoint: dict, scenarios: tuple[str, ...]) -> dict:
    agent_namespaces = tuple(checkpoint['agent_namespaces'])
    obs_dim = int(checkpoint['local_observation_size'])
    action_dim = int(checkpoint['action_dim'])
    resolved_max_neighbors = int(checkpoint.get('max_neighbors', max(1, (obs_dim - 10) // 6)))
    resolved_max_agents = int(checkpoint.get('max_agents', len(agent_namespaces)))
    return {
        'agent_namespaces': agent_namespaces,
        'enable_rl_backend': True,
        'rl_control_mode': 'pure',
        'action_mode': str(checkpoint.get('action_mode', 'full')),
        'action_bounds': ActionBounds(**checkpoint.get('action_bounds', {'linear_delta': 0.7, 'angular_delta': 0.6})),
        'max_neighbors': resolved_max_neighbors,
        'max_agents': max(resolved_max_agents, len(agent_namespaces)),
        'cruise_speed': float(checkpoint.get('cruise_speed', 0.5)),
        'max_angular_velocity': float(checkpoint.get('max_angular_velocity', 0.5)),
        'episode_timeout': float(checkpoint.get('episode_timeout', 45.0)),
        'no_progress_timeout': float(checkpoint.get('no_progress_timeout', 10.0)),
        'min_progress_delta': float(checkpoint.get('min_progress_delta', 0.3)),
        'collision_distance': float(checkpoint.get('collision_distance', 0.5)),
        'near_miss_distance': float(checkpoint.get('near_miss_distance', 1.5)),
        'scenario_neighbor_speed': float(checkpoint.get('scenario_neighbor_speed', 0.45)),
        'default_scenarios': scenarios,
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


def collect_trajectories(
    model_path: str,
    output_path: str,
    *,
    episodes: int = 200,
    steps_per_episode: int = 180,
    scenarios: tuple[str, ...] | None = None,
    device: str = 'cpu',
) -> dict:
    checkpoint, policy = _load_mappo_checkpoint(model_path, device)
    resolved_scenarios = (
        tuple(scenarios) if scenarios
        else tuple(checkpoint.get('scenarios', MultiAgentScenarioFactory.cluster_available()))
    )
    env_kwargs = _build_env_kwargs_from_checkpoint(checkpoint, resolved_scenarios)
    env = _create_env(env_kwargs)

    all_obs = []
    all_actions = []
    all_scenarios = []
    all_episode_ids = []
    all_agent_ids = []
    episode_stats = []

    max_episode_attempts = 3
    t0 = time.monotonic()

    try:
        for episode in range(episodes):
            scenario_name = resolved_scenarios[episode % len(resolved_scenarios)]
            last_error = None
            for attempt in range(max_episode_attempts):
                try:
                    observations, info = env.reset(options={'scenario_kind': scenario_name})
                    ep_obs = []
                    ep_actions = []
                    ep_agents = []
                    steps = 0

                    for step in range(steps_per_episode):
                        action_map = {}
                        for agent_id in env.agent_ids:
                            obs = observations[agent_id]
                            action = policy.predict(obs)
                            action_map[agent_id] = action
                            ep_obs.append(obs.copy())
                            ep_actions.append(action.copy())
                            ep_agents.append(agent_id)

                        observations, _, terminated_dict, truncated_dict, info = env.step(action_map)
                        steps = step + 1
                        if bool(terminated_dict['__all__']) or bool(truncated_dict['__all__']):
                            break

                    all_obs.extend(ep_obs)
                    all_actions.extend(ep_actions)
                    all_scenarios.extend([scenario_name] * len(ep_obs))
                    all_episode_ids.extend([episode] * len(ep_obs))
                    all_agent_ids.extend(ep_agents)

                    pairwise_min = float(info['pairwise_min_separation'])
                    collision = pairwise_min < env.config.collision_distance
                    episode_stats.append({
                        'episode': episode,
                        'scenario': scenario_name,
                        'steps': steps,
                        'samples': len(ep_obs),
                        'collision': collision,
                    })

                    if (episode + 1) % 20 == 0:
                        elapsed = time.monotonic() - t0
                        print(
                            f'[{episode + 1}/{episodes}] samples={len(all_obs)}, '
                            f'elapsed={elapsed:.0f}s',
                            flush=True,
                        )
                    break
                except RuntimeError as exc:
                    last_error = exc
                    print(
                        f'Warning: episode {episode} attempt {attempt + 1}/{max_episode_attempts} '
                        f'failed: {exc}. Recreating env.',
                        flush=True,
                    )
                    env.close()
                    env = _create_env(env_kwargs)
            else:
                raise RuntimeError(
                    f'Failed to collect episode {episode} after {max_episode_attempts} attempts.'
                ) from last_error
    finally:
        env.close()

    elapsed = time.monotonic() - t0

    obs_array = np.array(all_obs, dtype=np.float32)
    actions_array = np.array(all_actions, dtype=np.float32)

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        observations=obs_array,
        actions=actions_array,
        scenarios=np.array(all_scenarios),
        episode_ids=np.array(all_episode_ids, dtype=np.int32),
        agent_ids=np.array(all_agent_ids),
    )

    summary = {
        'model': model_path,
        'output': output_path,
        'episodes': episodes,
        'steps_per_episode': steps_per_episode,
        'total_samples': len(all_obs),
        'obs_dim': int(obs_array.shape[1]),
        'action_dim': int(actions_array.shape[1]),
        'scenarios': list(resolved_scenarios),
        'elapsed_seconds': round(elapsed, 1),
        'collision_episodes': sum(1 for s in episode_stats if s['collision']),
        'collision_rate': sum(1 for s in episode_stats if s['collision']) / max(1, len(episode_stats)),
    }

    print(
        f'Collected {summary["total_samples"]} samples from {episodes} episodes '
        f'in {elapsed:.0f}s → {output_path}',
        flush=True,
    )
    return summary


def main():
    args = parse_args()
    scenarios = tuple(args.scenarios) if args.scenarios else None
    summary = collect_trajectories(
        model_path=args.model,
        output_path=args.output,
        episodes=args.episodes,
        steps_per_episode=args.steps_per_episode,
        scenarios=scenarios,
        device=args.device,
    )
    if args.summary_json:
        Path(args.summary_json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.summary_json).write_text(
            json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8'
        )
        print(f'Summary saved to {args.summary_json}')


if __name__ == '__main__':
    main()
