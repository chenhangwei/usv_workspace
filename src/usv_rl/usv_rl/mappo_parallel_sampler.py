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
from .observation_normalizer import ObservationNormalizer


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
        near_miss_exponent=float(args.near_miss_exponent),
        head_on_near_miss_distance=float(args.head_on_near_miss_distance),
        conflict_distance=float(args.conflict_distance),
        anticipation_distance=float(args.anticipation_distance),
        conflict_risk_weight=float(args.conflict_risk_weight),
        conflict_brake_weight=float(args.conflict_brake_weight),
        conflict_progress_scale=float(args.conflict_progress_scale),
        conflict_resolution_reward_weight=float(args.conflict_resolution_reward_weight),
        conflict_escalation_penalty_weight=float(args.conflict_escalation_penalty_weight),
        unsafe_close_speed_penalty_weight=float(args.unsafe_close_speed_penalty_weight),
        path_deviation_penalty_weight=float(args.path_deviation_penalty_weight),
        path_deviation_tolerance=float(args.path_deviation_tolerance),
        path_deviation_conflict_scale=float(args.path_deviation_conflict_scale),
        desired_conflict_speed=float(args.desired_conflict_speed),
        stop_go_penalty_weight=float(args.stop_go_penalty_weight),
        head_on_guidance_distance=float(args.head_on_guidance_distance),
        head_on_target_starboard_offset=float(args.head_on_target_starboard_offset),
        head_on_corridor_reward_weight=float(args.head_on_corridor_reward_weight),
        head_on_centerline_penalty_weight=float(args.head_on_centerline_penalty_weight),
        head_on_turn_reward_weight=float(args.head_on_turn_reward_weight),
        head_on_forward_reward_weight=float(args.head_on_forward_reward_weight),
        head_on_speed_drop_penalty_weight=float(args.head_on_speed_drop_penalty_weight),
        head_on_close_penalty_weight=float(args.head_on_close_penalty_weight),
        head_on_no_turn_penalty_weight=float(args.head_on_no_turn_penalty_weight),
        head_on_phase_gate_strength=float(args.head_on_phase_gate_strength),
        crossing_starboard_turn_reward_weight=float(args.crossing_starboard_turn_reward_weight),
        crossing_forward_reward_weight=float(args.crossing_forward_reward_weight),
        crossing_slowdown_reward_weight=float(args.crossing_slowdown_reward_weight),
        crossing_overspeed_penalty_weight=float(args.crossing_overspeed_penalty_weight),
        crossing_close_forward_penalty_weight=float(args.crossing_close_forward_penalty_weight),
        crossing_yield_speed=float(args.crossing_yield_speed),
        crossing_time_separation_reward_weight=float(args.crossing_time_separation_reward_weight),
        crossing_time_separation_penalty_weight=float(args.crossing_time_separation_penalty_weight),
        crossing_time_gap_target=float(args.crossing_time_gap_target),
        overtaking_starboard_turn_reward_weight=float(args.overtaking_starboard_turn_reward_weight),
        overtaking_forward_reward_weight=float(args.overtaking_forward_reward_weight),
        overtaking_corridor_reward_weight=float(args.overtaking_corridor_reward_weight),
        overtaking_centerline_penalty_weight=float(args.overtaking_centerline_penalty_weight),
        overtaking_close_penalty_weight=float(args.overtaking_close_penalty_weight),
        colregs_port_turn_penalty_weight=float(args.colregs_port_turn_penalty_weight),
        heading_relief_factor=float(args.heading_relief_factor),
        heading_error_weight=float(args.heading_error_weight),
        action_smoothness_weight=float(args.action_smoothness_weight),
        pure_cruise_reward_weight=float(args.pure_cruise_reward_weight),
        pure_idle_penalty_weight=float(args.pure_idle_penalty_weight),
        pure_turn_penalty_weight=float(args.pure_turn_penalty_weight),
        pure_spin_penalty_weight=float(args.pure_spin_penalty_weight),
        time_penalty=float(args.time_penalty),
        stall_penalty=float(args.stall_penalty),
        angular_accel_penalty_weight=float(args.angular_accel_penalty_weight),
        straight_line_omega_penalty_weight=float(args.straight_line_omega_penalty_weight),
        saturated_omega_flip_penalty_weight=float(args.saturated_omega_flip_penalty_weight),
        forward_speed_change_penalty_weight=float(args.forward_speed_change_penalty_weight),
        omega_flip_saturation_threshold=float(args.omega_flip_saturation_threshold),
        straight_line_omega_conflict_floor=float(args.straight_line_omega_conflict_floor),
        conflict_overspeed_penalty_weight=float(args.conflict_overspeed_penalty_weight),
        proximity_gradient_penalty_weight=float(args.proximity_gradient_penalty_weight),
        proximity_gradient_distance=float(args.proximity_gradient_distance),
        speed_distance_coupling_penalty_weight=float(args.speed_distance_coupling_penalty_weight),
        speed_distance_coupling_threshold=float(args.speed_distance_coupling_threshold),
        heading_convergence_reward_weight=float(args.heading_convergence_reward_weight),
        heading_convergence_threshold_deg=float(args.heading_convergence_threshold_deg),
        heading_correction_reward_weight=float(args.heading_correction_reward_weight),
        straight_line_omega_cte_gate=float(args.straight_line_omega_cte_gate),
        avoidance_turn_reward_weight=float(args.avoidance_turn_reward_weight),
        near_goal_idle_penalty_weight=float(args.near_goal_idle_penalty_weight),
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
            heading_omega_deadband=float(args.heading_omega_deadband),
            heading_omega_reference=float(args.heading_omega_reference),
            angular_authority_power=float(args.angular_authority_power),
            angular_accel_limit=float(args.angular_accel_limit),
            angular_decel_limit=float(args.angular_decel_limit),
            conflict_turn_relief=float(args.conflict_turn_relief),
            angular_authority_floor=float(getattr(args, 'angular_authority_floor', 0.35)),
            min_forward_speed_floor=float(getattr(args, 'min_forward_speed_floor', 0.08)),
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
            separation_recovery_weight=float(args.separation_recovery_weight),
            entanglement_penalty_weight=float(args.entanglement_penalty_weight),
            entanglement_distance=float(args.entanglement_distance),
            entanglement_grace_steps=int(args.entanglement_grace_steps),
            entanglement_low_speed_penalty_weight=float(args.entanglement_low_speed_penalty_weight),
            coordination_reward_weight=float(args.coordination_reward_weight),
            team_completion_bonus=float(args.team_completion_bonus),
            deadlock_penalty_weight=float(args.deadlock_penalty_weight),
            domain_randomization=bool(getattr(args, 'domain_randomization', False)),
            dr_position_noise_std=float(getattr(args, 'dr_position_noise_std', 0.10)),
            dr_heading_noise_std=float(getattr(args, 'dr_heading_noise_std', 0.02)),
            dr_velocity_noise_ratio=float(getattr(args, 'dr_velocity_noise_ratio', 0.03)),
            dr_current_speed_max=float(getattr(args, 'dr_current_speed_max', 0.04)),
            dr_current_theta=float(getattr(args, 'dr_current_theta', 0.15)),
            dr_current_sigma=float(getattr(args, 'dr_current_sigma', 0.02)),
            dr_velocity_exec_noise=float(getattr(args, 'dr_velocity_exec_noise', 0.05)),
            scenario_spawn_position_std=float(getattr(args, 'scenario_spawn_position_std', 0.0)),
            scenario_spawn_heading_std=float(getattr(args, 'scenario_spawn_heading_std', 0.0)),
            scenario_goal_position_std=float(getattr(args, 'scenario_goal_position_std', 0.0)),
            encounter_type_dropout=float(getattr(args, 'encounter_type_dropout', 0.0)),
            sim_tau_linear=float(getattr(args, 'sim_tau_linear', 0.45)),
            sim_tau_angular=float(getattr(args, 'sim_tau_angular', 0.25)),
            dr_tau_linear_low=float(getattr(args, 'dr_tau_linear_low', 0.0)),
            dr_tau_linear_high=float(getattr(args, 'dr_tau_linear_high', 0.0)),
            dr_tau_angular_low=float(getattr(args, 'dr_tau_angular_low', 0.0)),
            dr_tau_angular_high=float(getattr(args, 'dr_tau_angular_high', 0.0)),
            speed_scale_distance=float(getattr(args, 'speed_scale_distance', 0.0)),
            speed_scale_min=float(getattr(args, 'speed_scale_min', 0.35)),
            cte_clip_range=float(getattr(args, 'cte_clip_range', 3.0)),
            max_waypoints_per_episode=int(getattr(args, 'max_waypoints_per_episode', 1)),
            waypoint_bonus=float(getattr(args, 'waypoint_bonus', 10.0)),
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


def _compute_advantages_and_returns(rewards, values, dones, bootstrap_values, gamma: float, gae_lambda: float):
    advantages_seq = []
    returns_seq = []
    next_values = np.asarray(bootstrap_values, dtype=np.float32)
    gae = np.zeros_like(next_values, dtype=np.float32)

    for step in reversed(range(len(rewards))):
        rewards_step = np.asarray(rewards[step], dtype=np.float32)
        values_step = np.asarray(values[step], dtype=np.float32)
        dones_step = np.asarray(dones[step], dtype=np.float32)
        if next_values.shape != values_step.shape:
            next_values = np.zeros_like(values_step, dtype=np.float32)
            gae = np.zeros_like(values_step, dtype=np.float32)
        mask = 1.0 - dones_step
        delta = rewards_step + gamma * next_values * mask - values_step
        gae = delta + gamma * gae_lambda * mask * gae
        advantages_seq.append(gae.copy())
        returns_seq.append((gae + values_step).copy())
        next_values = values_step

    advantages_seq.reverse()
    returns_seq.reverse()
    return (
        np.concatenate(advantages_seq, axis=0).astype(np.float32),
        np.concatenate(returns_seq, axis=0).astype(np.float32),
    )


def _flatten_rollout_batches(storage_batches: list[np.ndarray], *, dtype=np.float32) -> np.ndarray:
    if not storage_batches:
        return np.asarray([], dtype=dtype)
    return np.concatenate([np.asarray(batch, dtype=dtype) for batch in storage_batches], axis=0)


def _scenario_index_map(scenarios: tuple[str, ...]) -> dict[str, int]:
    return {str(scenario_name): index for index, scenario_name in enumerate(scenarios)}


def _actor_forward(actor, obs_tensor, scenario_ids=None):
    if scenario_ids is not None and (
        bool(getattr(actor, 'scenario_residual_enabled', False))
        or bool(getattr(actor, 'scenario_head_enabled', False))
        or bool(getattr(actor, 'scenario_trunk_enabled', False))
    ):
        return actor(obs_tensor, scenario_ids=scenario_ids)
    return actor(obs_tensor)


def _collect_worker_rollout(
    *,
    env,
    observations,
    global_state,
    scenario_to_index: dict[str, int],
    actor,
    critic,
    actor_log_std,
    action_low_tensor,
    action_high_tensor,
    rollout_steps: int,
    gamma: float,
    gae_lambda: float,
    max_env_recovery_attempts: int,
    max_consecutive_env_failures: int,
    consecutive_env_failures: int,
    env_factory,
    worker_rank: int,
    squash_actions: bool = False,
    obs_normalizer: ObservationNormalizer = None,
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
    storage_scenario_ids = []
    agent_steps = 0
    episode_count = 0
    current_scenario = env.current_scenario_name
    current_scenario_id = int(scenario_to_index.get(current_scenario, -1))

    for _ in range(rollout_steps):
        agent_order = env.agent_ids
        obs_batch = np.stack([observations[agent_id] for agent_id in agent_order], axis=0).astype(np.float32)
        state_batch = np.repeat(np.asarray(global_state, dtype=np.float32)[None, :], len(agent_order), axis=0)

        obs_batch_for_net = obs_normalizer.normalize(obs_batch) if obs_normalizer is not None else obs_batch
        obs_tensor = torch.as_tensor(obs_batch_for_net, dtype=torch.float32)
        state_tensor = torch.as_tensor(state_batch, dtype=torch.float32)
        critic_input = torch.cat([obs_tensor, state_tensor], dim=-1)
        scenario_id_tensor = None
        if current_scenario_id >= 0:
            scenario_id_tensor = torch.full((len(agent_order),), current_scenario_id, dtype=torch.long)

        with torch.no_grad():
            action_mean = _actor_forward(actor, obs_tensor, scenario_id_tensor)
            if squash_actions:
                _half = (action_high_tensor - action_low_tensor) / 2.0
                _mid = (action_high_tensor + action_low_tensor) / 2.0
                action_mean = torch.tanh(action_mean) * _half + _mid
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
        storage_scenario_ids.append(np.full(len(agent_order), current_scenario_id, dtype=np.int32))

        agent_steps += len(agent_order)
        observations = next_observations
        global_state = next_info['global_state']

        if done:
            episode_count += 1
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
            current_scenario = env.current_scenario_name
            current_scenario_id = int(scenario_to_index.get(current_scenario, -1))

    if not storage_obs:
        return {
            'status': 'empty',
            'worker_rank': worker_rank,
            'agent_steps': 0,
            'rollout_steps_collected': 0,
            'episode_count': int(episode_count),
        }, env, observations, global_state, consecutive_env_failures

    with torch.no_grad():
        agent_order = env.agent_ids
        bootstrap_obs = np.stack([observations[agent_id] for agent_id in agent_order], axis=0).astype(np.float32)
        bootstrap_state = np.repeat(np.asarray(global_state, dtype=np.float32)[None, :], len(agent_order), axis=0)
        bootstrap_obs_for_net = obs_normalizer.normalize(bootstrap_obs) if obs_normalizer is not None else bootstrap_obs
        bootstrap_obs_tensor = torch.as_tensor(bootstrap_obs_for_net, dtype=torch.float32)
        bootstrap_state_tensor = torch.as_tensor(bootstrap_state, dtype=torch.float32)
        bootstrap_values = critic(torch.cat([bootstrap_obs_tensor, bootstrap_state_tensor], dim=-1)).squeeze(-1).cpu().numpy()

    advantages, returns = _compute_advantages_and_returns(
        storage_rewards,
        storage_values,
        storage_dones,
        bootstrap_values,
        gamma,
        gae_lambda,
    )

    result = {
        'status': 'ok',
        'worker_rank': worker_rank,
        'obs': _flatten_rollout_batches(storage_obs),
        'states': _flatten_rollout_batches(storage_states),
        'actions': _flatten_rollout_batches(storage_actions),
        'log_probs': _flatten_rollout_batches(storage_log_probs),
        'rewards': _flatten_rollout_batches(storage_rewards),
        'advantages': advantages,
        'returns': returns,
        'scenario_ids': _flatten_rollout_batches(storage_scenario_ids),
        'agent_steps': int(agent_steps),
        'rollout_steps_collected': int(len(storage_obs)),
        'episode_count': int(episode_count),
    }
    return result, env, observations, global_state, consecutive_env_failures


def _rollout_worker_main(connection, worker_rank: int, domain_id: int, args_dict: dict, agent_namespaces: tuple[str, ...], scenarios: tuple[str, ...], hidden_sizes: tuple[int, ...], model_scenarios: tuple[str, ...] | None = None):
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)

    env = None
    try:
        import torch
        from torch import nn

        args = SimpleNamespace(**args_dict)
        model_scenarios = tuple(model_scenarios or scenarios)
        reward_config = _build_reward_config(args)
        env_factory = lambda: _create_env(args, agent_namespaces, scenarios, reward_config)
        scenario_to_index = _scenario_index_map(model_scenarios)
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
        use_neighbor_attention = bool(getattr(args, 'neighbor_attention', False))
        if use_neighbor_attention:
            from usv_rl.neighbor_attention import AttentionActor, AttentionCritic
            from usv_rl.multi_agent_types import ENCOUNTER_TYPE_COUNT
            max_neighbors = max(1, int(getattr(args, 'max_neighbors', 4)))
            embed_dim = int(getattr(args, 'attention_embed_dim', 32))
            num_heads = int(getattr(args, 'attention_num_heads', 1))
            actor = AttentionActor(
                max_neighbors=max_neighbors,
                encounter_dim=ENCOUNTER_TYPE_COUNT,
                hidden_sizes=hidden_sizes,
                action_dim=env.action_dim,
                embed_dim=embed_dim,
                num_heads=num_heads,
                encounter_residual=bool(getattr(args, 'attention_encounter_residual', False)),
                scenario_names=tuple(model_scenarios),
                scenario_residual=bool(getattr(args, 'attention_scenario_residual', False)),
                scenario_head=bool(getattr(args, 'attention_scenario_head', False)),
                scenario_trunk=bool(getattr(args, 'attention_scenario_trunk', False)),
            ).to('cpu')
            critic = AttentionCritic(
                max_neighbors=max_neighbors,
                encounter_dim=ENCOUNTER_TYPE_COUNT,
                global_state_dim=env.global_state_size,
                hidden_sizes=hidden_sizes,
                embed_dim=embed_dim,
                num_heads=num_heads,
            ).to('cpu')
        else:
            actor = _build_mlp(nn, env.local_observation_size, hidden_sizes, env.action_dim).to('cpu')
            critic = _build_mlp(nn, env.local_observation_size + env.global_state_size, hidden_sizes, 1).to('cpu')
        actor_log_std = torch.zeros(env.action_dim, dtype=torch.float32)
        action_low_tensor = torch.as_tensor(env.action_low, dtype=torch.float32)
        action_high_tensor = torch.as_tensor(env.action_high, dtype=torch.float32)
        squash_actions = bool(getattr(args, 'squash_actions', False))
        use_obs_norm = bool(getattr(args, 'normalize_observations', False))
        min_fwd = max(0.0, float(getattr(args, 'min_forward_speed', 0.0)))
        if min_fwd > 0.0:
            action_low_tensor[0] = min_fwd

        worker_normalizer = None
        if use_obs_norm:
            worker_normalizer = ObservationNormalizer(env.local_observation_size)

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

            norm_state = message.get('obs_normalizer_state')
            if use_obs_norm and norm_state is not None and worker_normalizer is not None:
                worker_normalizer.load_state_dict(norm_state)

            result, env, observations, global_state, consecutive_env_failures = _collect_worker_rollout(
                env=env,
                observations=observations,
                global_state=global_state,
                scenario_to_index=scenario_to_index,
                actor=actor,
                critic=critic,
                actor_log_std=actor_log_std,
                action_low_tensor=action_low_tensor,
                action_high_tensor=action_high_tensor,
                rollout_steps=int(message['rollout_steps']),
                gamma=float(args.gamma),
                gae_lambda=float(args.gae_lambda),
                max_env_recovery_attempts=max_env_recovery_attempts,
                max_consecutive_env_failures=max_consecutive_env_failures,
                consecutive_env_failures=consecutive_env_failures,
                env_factory=env_factory,
                worker_rank=worker_rank,
                squash_actions=squash_actions,
                obs_normalizer=worker_normalizer,
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
        model_scenarios: tuple[str, ...] | None = None,
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
                    tuple(model_scenarios or scenarios),
                ),
                daemon=True,
            )
            process.start()
            child_conn.close()
            self._connections.append(parent_conn)
            self._processes.append(process)

    def collect(self, *, actor, critic, actor_log_std, rollout_steps: int, remaining_agent_steps: int,
                obs_normalizer=None):
        worker_steps = _split_rollout_steps(rollout_steps, remaining_agent_steps, self._agent_count, self._num_workers)
        actor_state_dict = _cpu_state_dict(actor)
        critic_state_dict = _cpu_state_dict(critic)
        log_std = actor_log_std.detach().cpu()
        norm_state = obs_normalizer.state_dict() if obs_normalizer is not None else None

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
                'obs_normalizer_state': norm_state,
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