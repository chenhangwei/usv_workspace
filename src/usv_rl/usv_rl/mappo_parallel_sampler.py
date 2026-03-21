import math
import multiprocessing as mp
import os
import time
import traceback
from types import SimpleNamespace

import numpy as np
import rclpy

from .config import ActionBounds, RewardConfig
from .multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig


def _build_mlp(nn, input_dim: int, hidden_sizes: tuple[int, ...], output_dim: int):
    layers = []
    current_dim = input_dim
    for hidden_size in hidden_sizes:
        layers.append(nn.Linear(current_dim, hidden_size))
        layers.append(nn.Tanh())
        current_dim = hidden_size
    layers.append(nn.Linear(current_dim, output_dim))
    return nn.Sequential(*layers)


def _build_reward_config(args) -> RewardConfig:
    return RewardConfig(
        progress_weight=float(args.progress_weight),
        goal_bonus=float(args.goal_bonus),
        collision_penalty=float(args.collision_penalty),
        near_miss_weight=float(args.near_miss_weight),
        conflict_distance=float(args.conflict_distance),
        anticipation_distance=float(args.anticipation_distance),
        conflict_risk_weight=float(args.conflict_risk_weight),
        conflict_brake_weight=float(args.conflict_brake_weight),
        desired_conflict_speed=float(args.desired_conflict_speed),
        stop_go_penalty_weight=float(args.stop_go_penalty_weight),
        head_on_guidance_distance=float(args.head_on_guidance_distance),
        head_on_target_starboard_offset=float(args.head_on_target_starboard_offset),
        head_on_corridor_reward_weight=float(args.head_on_corridor_reward_weight),
        head_on_centerline_penalty_weight=float(args.head_on_centerline_penalty_weight),
        head_on_turn_reward_weight=float(args.head_on_turn_reward_weight),
        head_on_forward_reward_weight=float(args.head_on_forward_reward_weight),
        head_on_speed_drop_penalty_weight=float(args.head_on_speed_drop_penalty_weight),
        crossing_starboard_turn_reward_weight=float(args.crossing_starboard_turn_reward_weight),
        crossing_forward_reward_weight=float(args.crossing_forward_reward_weight),
        overtaking_starboard_turn_reward_weight=float(args.overtaking_starboard_turn_reward_weight),
        overtaking_forward_reward_weight=float(args.overtaking_forward_reward_weight),
        colregs_port_turn_penalty_weight=float(args.colregs_port_turn_penalty_weight),
        heading_relief_factor=float(args.heading_relief_factor),
        heading_error_weight=float(args.heading_error_weight),
        action_smoothness_weight=float(args.action_smoothness_weight),
        pure_cruise_reward_weight=float(args.pure_cruise_reward_weight),
        pure_idle_penalty_weight=float(args.pure_idle_penalty_weight),
        pure_turn_penalty_weight=float(args.pure_turn_penalty_weight),
        time_penalty=float(args.time_penalty),
        stall_penalty=float(args.stall_penalty),
    )


def _create_env(args, agent_namespaces: tuple[str, ...], scenarios: tuple[str, ...], reward_config: RewardConfig) -> MultiAgentEnv:
    return MultiAgentEnv(
        MultiAgentEnvConfig(
            agent_namespaces=agent_namespaces,
            enable_rl_backend=True,
            rl_control_mode=args.rl_control_mode,
            action_mode=args.action_mode,
            action_bounds=ActionBounds(
                linear_delta=float(args.linear_delta_limit),
                angular_delta=float(args.angular_delta_limit),
            ),
            max_neighbors=max(1, args.max_neighbors),
            max_agents=max(len(agent_namespaces), args.max_agents),
            cruise_speed=float(args.cruise_speed),
            max_angular_velocity=float(args.max_angular_velocity),
            episode_timeout=float(args.episode_timeout),
            no_progress_timeout=float(args.no_progress_timeout),
            min_progress_delta=float(args.min_progress_delta),
            collision_distance=float(args.collision_distance),
            near_miss_distance=float(args.near_miss_distance),
            default_scenarios=scenarios,
            reward=reward_config,
            goal_proximity_reward_weight=float(args.goal_proximity_reward_weight),
            goal_proximity_relief_distance=float(args.goal_proximity_relief_distance),
            goal_proximity_heading_relief=float(args.goal_proximity_heading_relief),
            goal_proximity_smoothness_relief=float(args.goal_proximity_smoothness_relief),
            goal_proximity_conflict_relief=float(args.goal_proximity_conflict_relief),
            goal_proximity_speed_relief=float(args.goal_proximity_speed_relief),
            scenario_neighbor_speed=float(args.scenario_neighbor_speed),
            team_reward_weight=float(args.team_reward_weight),
            team_progress_weight=float(args.team_progress_weight),
            team_goal_proximity_weight=float(args.team_goal_proximity_weight),
            team_regression_penalty_weight=float(args.team_regression_penalty_weight),
            team_dispersion_penalty_weight=float(args.team_dispersion_penalty_weight),
            team_dispersion_margin=float(args.team_dispersion_margin),
            coordination_reward_weight=float(args.coordination_reward_weight),
            team_completion_bonus=float(args.team_completion_bonus),
            deadlock_penalty_weight=float(args.deadlock_penalty_weight),
        )
    )


def _recover_env(env_factory, *, context: str, max_attempts: int = 3, worker_rank: int = 0):
    last_error = None
    for attempt in range(1, max_attempts + 1):
        env = None
        try:
            env = env_factory()
            observations, info = env.reset()
            return env, observations, info
        except RuntimeError as exc:
            last_error = exc
            print(
                f'[SamplerWorker {worker_rank}] Warning: {context} failed with {exc} '
                f'(attempt {attempt}/{max_attempts}). Recreating multi-agent env.',
                flush=True,
            )
            if env is not None:
                env.close()
    raise RuntimeError(f'Failed to recover multi-agent env during {context} after {max_attempts} attempts.') from last_error


def _split_rollout_steps(rollout_steps: int, remaining_agent_steps: int, agent_count: int, num_workers: int) -> list[int]:
    if agent_count <= 0 or rollout_steps <= 0 or remaining_agent_steps <= 0:
        return [0 for _ in range(num_workers)]
    total_env_steps = min(rollout_steps, int(math.ceil(remaining_agent_steps / agent_count)))
    base_steps, remainder = divmod(total_env_steps, num_workers)
    return [base_steps + (1 if worker_rank < remainder else 0) for worker_rank in range(num_workers)]


def _cpu_state_dict(module) -> dict:
    return {name: tensor.detach().cpu() for name, tensor in module.state_dict().items()}


def _collect_worker_rollout(
    *,
    env,
    observations,
    global_state,
    actor,
    critic,
    actor_log_std,
    action_low_tensor,
    action_high_tensor,
    rollout_steps: int,
    max_env_recovery_attempts: int,
    max_consecutive_env_failures: int,
    consecutive_env_failures: int,
    env_factory,
    worker_rank: int,
):
    import torch
    from torch.distributions import Normal

    storage_obs = []
    storage_states = []
    storage_actions = []
    storage_log_probs = []
    storage_values = []
    storage_rewards = []
    storage_dones = []
    agent_steps = 0

    for _ in range(rollout_steps):
        agent_order = env.agent_ids
        obs_batch = np.stack([observations[agent_id] for agent_id in agent_order], axis=0).astype(np.float32)
        state_batch = np.repeat(np.asarray(global_state, dtype=np.float32)[None, :], len(agent_order), axis=0)

        obs_tensor = torch.as_tensor(obs_batch, dtype=torch.float32)
        state_tensor = torch.as_tensor(state_batch, dtype=torch.float32)
        critic_input = torch.cat([obs_tensor, state_tensor], dim=-1)

        with torch.no_grad():
            action_mean = actor(obs_tensor)
            distribution = Normal(action_mean, actor_log_std.exp().expand_as(action_mean))
            action_tensor = distribution.sample()
            log_prob_tensor = distribution.log_prob(action_tensor).sum(dim=-1)
            value_tensor = critic(critic_input).squeeze(-1)

        clipped_actions = torch.max(
            action_low_tensor,
            torch.min(action_high_tensor, action_tensor),
        )
        action_map = {
            agent_id: clipped_actions[index].detach().cpu().numpy()
            for index, agent_id in enumerate(agent_order)
        }

        try:
            next_observations, reward_dict, terminated_dict, truncated_dict, next_info = env.step(action_map)
        except RuntimeError as exc:
            consecutive_env_failures += 1
            print(
                f'[SamplerWorker {worker_rank}] Warning: rollout step failed with {exc}. '
                f'Recovering environment ({consecutive_env_failures}/{max_consecutive_env_failures}).',
                flush=True,
            )
            if consecutive_env_failures >= max_consecutive_env_failures:
                raise RuntimeError(
                    f'Sampler worker {worker_rank} aborted after {consecutive_env_failures} consecutive environment failures.'
                ) from exc
            if storage_dones:
                storage_dones[-1] = np.ones(len(agent_order), dtype=np.float32)
            env.close()
            env, observations, info = _recover_env(
                env_factory,
                context='parallel MAPPO rollout step recovery',
                max_attempts=max_env_recovery_attempts,
                worker_rank=worker_rank,
            )
            global_state = info['global_state']
            break

        consecutive_env_failures = 0
        done = bool(terminated_dict['__all__'] or truncated_dict['__all__'])
        reward_batch = np.asarray([reward_dict[agent_id] for agent_id in agent_order], dtype=np.float32)

        storage_obs.append(obs_batch)
        storage_states.append(state_batch)
        storage_actions.append(action_tensor.detach().cpu().numpy())
        storage_log_probs.append(log_prob_tensor.detach().cpu().numpy())
        storage_values.append(value_tensor.detach().cpu().numpy())
        storage_rewards.append(reward_batch)
        storage_dones.append(np.full(len(agent_order), float(done), dtype=np.float32))

        agent_steps += len(agent_order)
        observations = next_observations
        global_state = next_info['global_state']

        if done:
            try:
                observations, info = env.reset()
            except RuntimeError as exc:
                consecutive_env_failures += 1
                print(
                    f'[SamplerWorker {worker_rank}] Warning: episode reset failed with {exc}. '
                    f'Recovering environment ({consecutive_env_failures}/{max_consecutive_env_failures}).',
                    flush=True,
                )
                if consecutive_env_failures >= max_consecutive_env_failures:
                    raise RuntimeError(
                        f'Sampler worker {worker_rank} aborted after {consecutive_env_failures} consecutive environment failures.'
                    ) from exc
                env.close()
                env, observations, info = _recover_env(
                    env_factory,
                    context='parallel MAPPO episode reset recovery',
                    max_attempts=max_env_recovery_attempts,
                    worker_rank=worker_rank,
                )
            else:
                consecutive_env_failures = 0
            global_state = info['global_state']

    if not storage_obs:
        return {
            'status': 'empty',
            'worker_rank': worker_rank,
            'agent_steps': 0,
            'rollout_steps_collected': 0,
        }, env, observations, global_state, consecutive_env_failures

    with torch.no_grad():
        agent_order = env.agent_ids
        bootstrap_obs = np.stack([observations[agent_id] for agent_id in agent_order], axis=0).astype(np.float32)
        bootstrap_state = np.repeat(np.asarray(global_state, dtype=np.float32)[None, :], len(agent_order), axis=0)
        bootstrap_obs_tensor = torch.as_tensor(bootstrap_obs, dtype=torch.float32)
        bootstrap_state_tensor = torch.as_tensor(bootstrap_state, dtype=torch.float32)
        bootstrap_values = critic(torch.cat([bootstrap_obs_tensor, bootstrap_state_tensor], dim=-1)).squeeze(-1).cpu().numpy()

    result = {
        'status': 'ok',
        'worker_rank': worker_rank,
        'obs': np.asarray(storage_obs, dtype=np.float32),
        'states': np.asarray(storage_states, dtype=np.float32),
        'actions': np.asarray(storage_actions, dtype=np.float32),
        'log_probs': np.asarray(storage_log_probs, dtype=np.float32),
        'values': np.asarray(storage_values, dtype=np.float32),
        'rewards': np.asarray(storage_rewards, dtype=np.float32),
        'dones': np.asarray(storage_dones, dtype=np.float32),
        'bootstrap_values': bootstrap_values.astype(np.float32),
        'agent_steps': int(agent_steps),
        'rollout_steps_collected': int(len(storage_obs)),
    }
    return result, env, observations, global_state, consecutive_env_failures


def _rollout_worker_main(connection, worker_rank: int, domain_id: int, args_dict: dict, agent_namespaces: tuple[str, ...], scenarios: tuple[str, ...], hidden_sizes: tuple[int, ...]):
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)

    env = None
    try:
        import torch
        from torch import nn

        args = SimpleNamespace(**args_dict)
        reward_config = _build_reward_config(args)
        env_factory = lambda: _create_env(args, agent_namespaces, scenarios, reward_config)
        max_env_recovery_attempts = 3
        max_consecutive_env_failures = 5
        consecutive_env_failures = 0

        env, observations, info = _recover_env(
            env_factory,
            context='parallel MAPPO environment setup',
            max_attempts=max_env_recovery_attempts,
            worker_rank=worker_rank,
        )
        global_state = info['global_state']
        actor = _build_mlp(nn, env.local_observation_size, hidden_sizes, env.action_dim).to('cpu')
        critic = _build_mlp(nn, env.local_observation_size + env.global_state_size, hidden_sizes, 1).to('cpu')
        actor_log_std = torch.zeros(env.action_dim, dtype=torch.float32)
        action_low_tensor = torch.as_tensor(env.action_low, dtype=torch.float32)
        action_high_tensor = torch.as_tensor(env.action_high, dtype=torch.float32)

        while True:
            message = connection.recv()
            message_type = message.get('type')
            if message_type == 'close':
                return
            if message_type != 'collect':
                raise RuntimeError(f'Unknown sampler worker message type: {message_type}')

            actor.load_state_dict(message['actor_state_dict'])
            critic.load_state_dict(message['critic_state_dict'])
            actor_log_std = message['actor_log_std'].to(dtype=torch.float32, device='cpu')

            result, env, observations, global_state, consecutive_env_failures = _collect_worker_rollout(
                env=env,
                observations=observations,
                global_state=global_state,
                actor=actor,
                critic=critic,
                actor_log_std=actor_log_std,
                action_low_tensor=action_low_tensor,
                action_high_tensor=action_high_tensor,
                rollout_steps=int(message['rollout_steps']),
                max_env_recovery_attempts=max_env_recovery_attempts,
                max_consecutive_env_failures=max_consecutive_env_failures,
                consecutive_env_failures=consecutive_env_failures,
                env_factory=env_factory,
                worker_rank=worker_rank,
            )
            connection.send(result)
    except EOFError:
        pass
    except Exception:
        try:
            connection.send({
                'status': 'error',
                'worker_rank': worker_rank,
                'traceback': traceback.format_exc(),
            })
        except Exception:
            pass
    finally:
        if env is not None:
            env.close()
        connection.close()


class ParallelRolloutSampler:
    def __init__(
        self,
        *,
        args_dict: dict,
        agent_namespaces: tuple[str, ...],
        scenarios: tuple[str, ...],
        hidden_sizes: tuple[int, ...],
        num_workers: int,
        base_ros_domain_id: int,
    ):
        self._agent_count = len(agent_namespaces)
        self._num_workers = int(num_workers)
        self._context = mp.get_context('spawn')
        self._connections = []
        self._processes = []

        for worker_rank in range(self._num_workers):
            parent_conn, child_conn = self._context.Pipe()
            process = self._context.Process(
                target=_rollout_worker_main,
                args=(
                    child_conn,
                    worker_rank,
                    int(base_ros_domain_id) + worker_rank,
                    dict(args_dict),
                    tuple(agent_namespaces),
                    tuple(scenarios),
                    tuple(hidden_sizes),
                ),
                daemon=True,
            )
            process.start()
            child_conn.close()
            self._connections.append(parent_conn)
            self._processes.append(process)

    def collect(self, *, actor, critic, actor_log_std, rollout_steps: int, remaining_agent_steps: int):
        worker_steps = _split_rollout_steps(rollout_steps, remaining_agent_steps, self._agent_count, self._num_workers)
        actor_state_dict = _cpu_state_dict(actor)
        critic_state_dict = _cpu_state_dict(critic)
        log_std = actor_log_std.detach().cpu()

        active_connections = []
        for connection, per_worker_steps in zip(self._connections, worker_steps):
            if per_worker_steps <= 0:
                continue
            connection.send({
                'type': 'collect',
                'rollout_steps': int(per_worker_steps),
                'actor_state_dict': actor_state_dict,
                'critic_state_dict': critic_state_dict,
                'actor_log_std': log_std,
            })
            active_connections.append(connection)

        results = []
        for connection in active_connections:
            result = connection.recv()
            if result.get('status') == 'error':
                raise RuntimeError(
                    f"Sampler worker {result.get('worker_rank')} failed:\n{result.get('traceback', 'unknown error')}"
                )
            if result.get('status') == 'ok':
                results.append(result)
        return results

    def close(self):
        for connection in self._connections:
            try:
                connection.send({'type': 'close'})
            except Exception:
                pass

        for process in self._processes:
            process.join(timeout=5.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=2.0)

        for connection in self._connections:
            connection.close()

        self._connections.clear()
        self._processes.clear()