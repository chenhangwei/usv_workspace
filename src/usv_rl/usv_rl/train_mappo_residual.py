import argparse
from pathlib import Path
import subprocess
import sys

import numpy as np

from .config import RewardConfig
from .multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig
from .multi_agent_scenarios import MultiAgentScenarioFactory


def parse_args():
    parser = argparse.ArgumentParser(description='Minimal MAPPO training entry for multi-USV residual control.')
    parser.add_argument('--output', required=True, help='Output model path (.pt).')
    parser.add_argument('--num-agents', type=int, default=3, help='Number of controlled agents.')
    parser.add_argument('--total-timesteps', type=int, default=4096, help='Total training timesteps across all agents.')
    parser.add_argument('--rollout-steps', type=int, default=128, help='Rollout horizon before each update.')
    parser.add_argument('--update-epochs', type=int, default=4, help='Number of PPO update epochs per rollout.')
    parser.add_argument('--minibatch-size', type=int, default=128, help='Mini-batch size for PPO updates.')
    parser.add_argument('--learning-rate', type=float, default=3e-4, help='Optimizer learning rate.')
    parser.add_argument('--gamma', type=float, default=0.99, help='Discount factor.')
    parser.add_argument('--gae-lambda', type=float, default=0.95, help='GAE lambda.')
    parser.add_argument('--clip-range', type=float, default=0.2, help='PPO clip range.')
    parser.add_argument('--entropy-coef', type=float, default=0.01, help='Entropy bonus coefficient.')
    parser.add_argument('--value-coef', type=float, default=0.5, help='Value loss coefficient.')
    parser.add_argument('--max-grad-norm', type=float, default=0.5, help='Gradient clipping norm.')
    parser.add_argument('--device', default='cpu', help='Torch device.')
    parser.add_argument('--hidden-size', action='append', dest='hidden_sizes', type=int, default=None, help='Hidden layer size. Repeatable.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name. Repeatable.')
    parser.add_argument('--scenario-set', choices=['auto', 'smoke', 'dense', 'all'], default='auto', help='Scenario curriculum preset used when --scenario is not provided.')
    parser.add_argument('--max-neighbors', type=int, default=4, help='Neighbor slots in local observation encoding.')
    parser.add_argument('--max-agents', type=int, default=3, help='Maximum agents encoded in global state.')
    parser.add_argument('--action-mode', choices=['full', 'angular_only'], default='full', help='Residual action representation.')
    parser.add_argument('--episode-timeout', type=float, default=45.0, help='Episode timeout in seconds.')
    parser.add_argument('--no-progress-timeout', type=float, default=10.0, help='No-progress timeout in seconds.')
    parser.add_argument('--min-progress-delta', type=float, default=0.3, help='Fleet progress threshold for refreshing the no-progress timer.')
    parser.add_argument('--progress-weight', type=float, default=RewardConfig.progress_weight, help='Per-agent progress reward weight.')
    parser.add_argument('--goal-proximity-reward-weight', type=float, default=0.0, help='Per-agent dense reward for moving inside the near-goal relief radius.')
    parser.add_argument('--goal-proximity-relief-distance', type=float, default=3.0, help='Distance to goal below which near-goal shaping and relief activate.')
    parser.add_argument('--goal-proximity-heading-relief', type=float, default=0.0, help='Fractional heading penalty relief applied near goal.')
    parser.add_argument('--goal-proximity-smoothness-relief', type=float, default=0.0, help='Fractional smoothness penalty relief applied near goal.')
    parser.add_argument('--goal-bonus', type=float, default=RewardConfig.goal_bonus, help='Per-agent goal completion bonus.')
    parser.add_argument('--near-miss-weight', type=float, default=RewardConfig.near_miss_weight, help='Near-miss penalty weight.')
    parser.add_argument('--conflict-risk-weight', type=float, default=RewardConfig.conflict_risk_weight, help='Conflict-risk penalty weight.')
    parser.add_argument('--conflict-brake-weight', type=float, default=RewardConfig.conflict_brake_weight, help='Conflict braking penalty weight.')
    parser.add_argument('--desired-conflict-speed', type=float, default=RewardConfig.desired_conflict_speed, help='Target forward speed maintained during conflict handling.')
    parser.add_argument('--stop-go-penalty-weight', type=float, default=RewardConfig.stop_go_penalty_weight, help='Penalty weight for rapid stop-go behavior under conflict.')
    parser.add_argument('--head-on-guidance-distance', type=float, default=RewardConfig.head_on_guidance_distance, help='Distance threshold for head-on starboard guidance shaping.')
    parser.add_argument('--head-on-target-starboard-offset', type=float, default=RewardConfig.head_on_target_starboard_offset, help='Desired starboard lateral offset in head-on encounters.')
    parser.add_argument('--head-on-corridor-reward-weight', type=float, default=RewardConfig.head_on_corridor_reward_weight, help='Reward weight for entering the head-on starboard corridor.')
    parser.add_argument('--head-on-centerline-penalty-weight', type=float, default=RewardConfig.head_on_centerline_penalty_weight, help='Penalty weight for staying on the head-on centerline.')
    parser.add_argument('--head-on-turn-reward-weight', type=float, default=RewardConfig.head_on_turn_reward_weight, help='Reward weight for committing to starboard turn in head-on encounters.')
    parser.add_argument('--heading-relief-factor', type=float, default=RewardConfig.heading_relief_factor, help='Fractional relief applied to heading penalty during conflict handling.')
    parser.add_argument('--heading-error-weight', type=float, default=RewardConfig.heading_error_weight, help='Heading error penalty weight.')
    parser.add_argument('--action-smoothness-weight', type=float, default=RewardConfig.action_smoothness_weight, help='Action smoothness penalty weight.')
    parser.add_argument('--time-penalty', type=float, default=RewardConfig.time_penalty, help='Per-step time penalty.')
    parser.add_argument('--stall-penalty', type=float, default=RewardConfig.stall_penalty, help='Penalty applied on episode truncation.')
    parser.add_argument('--team-reward-weight', type=float, default=0.30, help='Fleet near-miss team penalty weight.')
    parser.add_argument('--team-progress-weight', type=float, default=1.20, help='Fleet progress reward weight.')
    parser.add_argument('--team-goal-proximity-weight', type=float, default=0.0, help='Fleet dense reward weight for reducing mean distance to goals from the scenario start.')
    parser.add_argument('--team-regression-penalty-weight', type=float, default=0.0, help='Fleet penalty weight for negative team progress.')
    parser.add_argument('--team-dispersion-penalty-weight', type=float, default=0.0, help='Fleet penalty weight for expanding mean pairwise separation beyond the initial formation.')
    parser.add_argument('--team-dispersion-margin', type=float, default=0.0, help='Allowed increase in fleet mean separation before dispersion penalty applies.')
    parser.add_argument('--coordination-reward-weight', type=float, default=0.20, help='Fleet goal-completion coordination reward weight.')
    parser.add_argument('--team-completion-bonus', type=float, default=18.0, help='Fleet-wide bonus when all agents reach their goals.')
    parser.add_argument('--deadlock-penalty-weight', type=float, default=4.0, help='Penalty weight applied when fleet mean goal distance stalls.')
    parser.add_argument('--checkpoint-interval', type=int, default=0, help='Optional periodic checkpoint interval measured in total agent timesteps. 0 disables periodic checkpointing.')
    parser.add_argument('--checkpoint-dir', help='Optional directory for periodic checkpoints. Defaults to <output_stem>_checkpoints next to --output.')
    parser.add_argument('--auto-evaluate-checkpoints', action='store_true', help='Run batch checkpoint evaluation automatically after training completes.')
    parser.add_argument('--checkpoint-eval-episodes', type=int, default=15, help='Evaluation episodes per checkpoint when --auto-evaluate-checkpoints is enabled. Default to a larger sample because 3-episode rankings are too noisy for close dense checkpoints.')
    parser.add_argument('--checkpoint-eval-steps', type=int, default=90, help='Maximum steps per evaluation episode when --auto-evaluate-checkpoints is enabled.')
    parser.add_argument('--checkpoint-eval-scenario', action='append', dest='checkpoint_eval_scenarios', default=None, help='Scenario used for automatic checkpoint evaluation. Repeatable.')
    parser.add_argument('--checkpoint-eval-device', default=None, help='Optional device override for automatic checkpoint evaluation. Defaults to --device.')
    parser.add_argument('--checkpoint-ranking-json', help='Optional JSON output path for automatic checkpoint ranking.')
    parser.add_argument('--checkpoint-eval-json-dir', help='Optional directory for per-checkpoint evaluation JSON outputs.')
    return parser.parse_args()


def _build_agent_namespaces(num_agents: int) -> tuple[str, ...]:
    return tuple(f'usv_{index + 1:02d}' for index in range(num_agents))


def _build_mlp(nn, input_dim: int, hidden_sizes: tuple[int, ...], output_dim: int):
    layers = []
    current_dim = input_dim
    for hidden_size in hidden_sizes:
        layers.append(nn.Linear(current_dim, hidden_size))
        layers.append(nn.Tanh())
        current_dim = hidden_size
    layers.append(nn.Linear(current_dim, output_dim))
    return nn.Sequential(*layers)


def _resolve_scenarios(args) -> tuple[str, ...]:
    agent_count = max(2, int(args.num_agents))
    if args.scenarios:
        scenarios = tuple(args.scenarios)
        incompatible = [
            scenario
            for scenario in scenarios
            if MultiAgentScenarioFactory.required_agent_count(scenario) > agent_count
        ]
        if incompatible:
            joined = ', '.join(incompatible)
            raise ValueError(
                f'Scenarios require more agents than configured (num_agents={agent_count}): {joined}'
            )
        return scenarios
    return MultiAgentScenarioFactory.scenario_set(args.scenario_set, agent_count)


def _build_reward_config(args) -> RewardConfig:
    return RewardConfig(
        progress_weight=float(args.progress_weight),
        goal_bonus=float(args.goal_bonus),
        collision_penalty=RewardConfig.collision_penalty,
        near_miss_weight=float(args.near_miss_weight),
        conflict_distance=RewardConfig.conflict_distance,
        anticipation_distance=RewardConfig.anticipation_distance,
        conflict_risk_weight=float(args.conflict_risk_weight),
        conflict_brake_weight=float(args.conflict_brake_weight),
        desired_conflict_speed=float(args.desired_conflict_speed),
        stop_go_penalty_weight=float(args.stop_go_penalty_weight),
        head_on_guidance_distance=float(args.head_on_guidance_distance),
        head_on_target_starboard_offset=float(args.head_on_target_starboard_offset),
        head_on_corridor_reward_weight=float(args.head_on_corridor_reward_weight),
        head_on_centerline_penalty_weight=float(args.head_on_centerline_penalty_weight),
        head_on_turn_reward_weight=float(args.head_on_turn_reward_weight),
        heading_relief_factor=float(args.heading_relief_factor),
        heading_error_weight=float(args.heading_error_weight),
        action_smoothness_weight=float(args.action_smoothness_weight),
        time_penalty=float(args.time_penalty),
        stall_penalty=float(args.stall_penalty),
    )


def _checkpoint_payload(
    actor,
    critic,
    actor_log_std,
    env,
    args,
    agent_namespaces: tuple[str, ...],
    hidden_sizes: tuple[int, ...],
    scenarios: tuple[str, ...],
    reward_config: RewardConfig,
    total_steps: int,
):
    return {
        'actor_state_dict': actor.state_dict(),
        'critic_state_dict': critic.state_dict(),
        'actor_log_std': actor_log_std.detach().cpu(),
        'agent_namespaces': agent_namespaces,
        'local_observation_size': env.local_observation_size,
        'global_state_size': env.global_state_size,
        'action_dim': env.action_dim,
        'hidden_sizes': hidden_sizes,
        'action_low': env.action_low.tolist(),
        'action_high': env.action_high.tolist(),
        'scenarios': scenarios,
        'max_neighbors': args.max_neighbors,
        'max_agents': max(len(agent_namespaces), args.max_agents),
        'action_mode': args.action_mode,
        'episode_timeout': float(args.episode_timeout),
        'no_progress_timeout': float(args.no_progress_timeout),
        'min_progress_delta': float(args.min_progress_delta),
        'reward_config': reward_config.__dict__,
        'goal_proximity_reward_weight': float(args.goal_proximity_reward_weight),
        'goal_proximity_relief_distance': float(args.goal_proximity_relief_distance),
        'goal_proximity_heading_relief': float(args.goal_proximity_heading_relief),
        'goal_proximity_smoothness_relief': float(args.goal_proximity_smoothness_relief),
        'team_reward_weight': float(args.team_reward_weight),
        'team_progress_weight': float(args.team_progress_weight),
        'team_goal_proximity_weight': float(args.team_goal_proximity_weight),
        'team_regression_penalty_weight': float(args.team_regression_penalty_weight),
        'team_dispersion_penalty_weight': float(args.team_dispersion_penalty_weight),
        'team_dispersion_margin': float(args.team_dispersion_margin),
        'coordination_reward_weight': float(args.coordination_reward_weight),
        'team_completion_bonus': float(args.team_completion_bonus),
        'deadlock_penalty_weight': float(args.deadlock_penalty_weight),
        'total_timesteps': max(0, args.total_timesteps),
        'completed_timesteps': int(total_steps),
    }


def _save_checkpoint(checkpoint_path: Path, payload: dict):
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    torch = __import__('torch')
    torch.save(payload, checkpoint_path)
    print(f'Saved MAPPO checkpoint scaffold to {checkpoint_path}')


def _run_checkpoint_evaluation(args, checkpoint_dir: Path, scenarios: tuple[str, ...], output_path: Path):
    if not args.auto_evaluate_checkpoints:
        return
    if not checkpoint_dir.exists():
        raise FileNotFoundError(f'Checkpoint directory does not exist for automatic evaluation: {checkpoint_dir}')

    eval_scenarios = tuple(args.checkpoint_eval_scenarios) if args.checkpoint_eval_scenarios else tuple(dict.fromkeys(scenarios))
    if not eval_scenarios:
        raise RuntimeError('Automatic checkpoint evaluation requires at least one scenario.')

    ranking_json = Path(args.checkpoint_ranking_json) if args.checkpoint_ranking_json else output_path.with_name(f'{output_path.stem}_ranking.json')
    eval_json_dir = Path(args.checkpoint_eval_json_dir) if args.checkpoint_eval_json_dir else output_path.with_name(f'{output_path.stem}_eval')
    command = [
        sys.executable,
        '-m',
        'usv_rl.evaluate_mappo_checkpoints',
        '--checkpoint-dir',
        str(checkpoint_dir),
        '--episodes',
        str(args.checkpoint_eval_episodes),
        '--steps-per-episode',
        str(args.checkpoint_eval_steps),
        '--device',
        str(args.checkpoint_eval_device or args.device),
        '--summary-json',
        str(ranking_json),
        '--per-checkpoint-json-dir',
        str(eval_json_dir),
    ]
    for scenario in eval_scenarios:
        command.extend(['--scenario', scenario])
    subprocess.run(command, check=True)
    print(f'Automatic checkpoint ranking saved to {ranking_json}')


def main():
    args = parse_args()
    try:
        import torch
        from torch import nn
        from torch.distributions import Normal
    except ImportError as exc:
        raise RuntimeError('torch is required for MAPPO training.') from exc

    hidden_sizes = tuple(args.hidden_sizes or [128, 128])
    agent_namespaces = _build_agent_namespaces(max(2, args.num_agents))
    scenarios = _resolve_scenarios(args)
    reward_config = _build_reward_config(args)
    output_path = Path(args.output)
    checkpoint_dir = None
    checkpoint_interval = max(0, int(args.checkpoint_interval))
    if checkpoint_interval > 0:
        checkpoint_dir = Path(args.checkpoint_dir) if args.checkpoint_dir else output_path.with_name(f'{output_path.stem}_checkpoints')
    print(f'Using MAPPO scenario curriculum: {scenarios}')
    env = MultiAgentEnv(
        MultiAgentEnvConfig(
            agent_namespaces=agent_namespaces,
            enable_rl_backend=True,
            action_mode=args.action_mode,
            max_neighbors=max(1, args.max_neighbors),
            max_agents=max(len(agent_namespaces), args.max_agents),
            episode_timeout=float(args.episode_timeout),
            no_progress_timeout=float(args.no_progress_timeout),
            min_progress_delta=float(args.min_progress_delta),
            default_scenarios=scenarios,
            reward=reward_config,
            goal_proximity_reward_weight=float(args.goal_proximity_reward_weight),
            goal_proximity_relief_distance=float(args.goal_proximity_relief_distance),
            goal_proximity_heading_relief=float(args.goal_proximity_heading_relief),
            goal_proximity_smoothness_relief=float(args.goal_proximity_smoothness_relief),
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

    device = torch.device(args.device)

    try:
        actor = _build_mlp(nn, env.local_observation_size, hidden_sizes, env.action_dim).to(device)
        critic = _build_mlp(nn, env.local_observation_size + env.global_state_size, hidden_sizes, 1).to(device)
        actor_log_std = nn.Parameter(torch.zeros(env.action_dim, device=device))
        optimizer = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()) + [actor_log_std], lr=args.learning_rate)

        observations, info = env.reset()
        global_state = info['global_state']
        total_steps = 0
        rollout_steps = max(1, args.rollout_steps)
        next_checkpoint_step = checkpoint_interval if checkpoint_interval > 0 else None

        while total_steps < max(0, args.total_timesteps):
            storage_obs = []
            storage_states = []
            storage_actions = []
            storage_log_probs = []
            storage_values = []
            storage_rewards = []
            storage_dones = []

            for _ in range(rollout_steps):
                agent_order = env.agent_ids
                obs_batch = np.stack([observations[agent_id] for agent_id in agent_order], axis=0).astype(np.float32)
                state_batch = np.repeat(np.asarray(global_state, dtype=np.float32)[None, :], len(agent_order), axis=0)

                obs_tensor = torch.as_tensor(obs_batch, dtype=torch.float32, device=device)
                state_tensor = torch.as_tensor(state_batch, dtype=torch.float32, device=device)
                critic_input = torch.cat([obs_tensor, state_tensor], dim=-1)

                with torch.no_grad():
                    action_mean = actor(obs_tensor)
                    distribution = Normal(action_mean, actor_log_std.exp().expand_as(action_mean))
                    action_tensor = distribution.sample()
                    log_prob_tensor = distribution.log_prob(action_tensor).sum(dim=-1)
                    value_tensor = critic(critic_input).squeeze(-1)

                clipped_actions = torch.max(
                    torch.as_tensor(env.action_low, dtype=torch.float32, device=device),
                    torch.min(torch.as_tensor(env.action_high, dtype=torch.float32, device=device), action_tensor),
                )
                action_map = {
                    agent_id: clipped_actions[index].detach().cpu().numpy()
                    for index, agent_id in enumerate(agent_order)
                }

                next_observations, reward_dict, terminated_dict, truncated_dict, next_info = env.step(action_map)
                done = bool(terminated_dict['__all__'] or truncated_dict['__all__'])
                reward_batch = np.asarray([reward_dict[agent_id] for agent_id in agent_order], dtype=np.float32)

                storage_obs.append(obs_batch)
                storage_states.append(state_batch)
                storage_actions.append(action_tensor.detach().cpu().numpy())
                storage_log_probs.append(log_prob_tensor.detach().cpu().numpy())
                storage_values.append(value_tensor.detach().cpu().numpy())
                storage_rewards.append(reward_batch)
                storage_dones.append(np.full(len(agent_order), float(done), dtype=np.float32))

                total_steps += len(agent_order)
                observations = next_observations
                global_state = next_info['global_state']
                if done:
                    observations, info = env.reset()
                    global_state = info['global_state']
                if total_steps >= args.total_timesteps:
                    break

            if not storage_obs:
                break

            with torch.no_grad():
                agent_order = env.agent_ids
                bootstrap_obs = np.stack([observations[agent_id] for agent_id in agent_order], axis=0).astype(np.float32)
                bootstrap_state = np.repeat(np.asarray(global_state, dtype=np.float32)[None, :], len(agent_order), axis=0)
                bootstrap_obs_tensor = torch.as_tensor(bootstrap_obs, dtype=torch.float32, device=device)
                bootstrap_state_tensor = torch.as_tensor(bootstrap_state, dtype=torch.float32, device=device)
                bootstrap_values = critic(torch.cat([bootstrap_obs_tensor, bootstrap_state_tensor], dim=-1)).squeeze(-1).cpu().numpy()

            rewards = np.asarray(storage_rewards, dtype=np.float32)
            values = np.asarray(storage_values, dtype=np.float32)
            dones = np.asarray(storage_dones, dtype=np.float32)
            advantages = np.zeros_like(rewards, dtype=np.float32)
            returns = np.zeros_like(rewards, dtype=np.float32)
            gae = np.zeros(len(env.agent_ids), dtype=np.float32)
            next_values = bootstrap_values.astype(np.float32)

            for step in reversed(range(rewards.shape[0])):
                mask = 1.0 - dones[step]
                delta = rewards[step] + args.gamma * next_values * mask - values[step]
                gae = delta + args.gamma * args.gae_lambda * mask * gae
                advantages[step] = gae
                returns[step] = gae + values[step]
                next_values = values[step]

            flat_obs = torch.as_tensor(np.asarray(storage_obs, dtype=np.float32).reshape(-1, env.local_observation_size), dtype=torch.float32, device=device)
            flat_states = torch.as_tensor(np.asarray(storage_states, dtype=np.float32).reshape(-1, env.global_state_size), dtype=torch.float32, device=device)
            flat_actions = torch.as_tensor(np.asarray(storage_actions, dtype=np.float32).reshape(-1, env.action_dim), dtype=torch.float32, device=device)
            flat_old_log_probs = torch.as_tensor(np.asarray(storage_log_probs, dtype=np.float32).reshape(-1), dtype=torch.float32, device=device)
            flat_advantages = torch.as_tensor(advantages.reshape(-1), dtype=torch.float32, device=device)
            flat_returns = torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=device)

            flat_advantages = (flat_advantages - flat_advantages.mean()) / (flat_advantages.std(unbiased=False) + 1e-6)
            sample_count = flat_obs.shape[0]
            minibatch_size = min(max(1, args.minibatch_size), sample_count)

            for _ in range(max(1, args.update_epochs)):
                permutation = torch.randperm(sample_count, device=device)
                for start in range(0, sample_count, minibatch_size):
                    batch_indices = permutation[start:start + minibatch_size]
                    batch_obs = flat_obs[batch_indices]
                    batch_states = flat_states[batch_indices]
                    batch_actions = flat_actions[batch_indices]
                    batch_old_log_probs = flat_old_log_probs[batch_indices]
                    batch_advantages = flat_advantages[batch_indices]
                    batch_returns = flat_returns[batch_indices]

                    action_mean = actor(batch_obs)
                    distribution = Normal(action_mean, actor_log_std.exp().expand_as(action_mean))
                    new_log_probs = distribution.log_prob(batch_actions).sum(dim=-1)
                    entropy = distribution.entropy().sum(dim=-1).mean()

                    ratio = torch.exp(new_log_probs - batch_old_log_probs)
                    surrogate_one = ratio * batch_advantages
                    surrogate_two = torch.clamp(ratio, 1.0 - args.clip_range, 1.0 + args.clip_range) * batch_advantages
                    actor_loss = -torch.min(surrogate_one, surrogate_two).mean()

                    critic_values = critic(torch.cat([batch_obs, batch_states], dim=-1)).squeeze(-1)
                    critic_loss = torch.nn.functional.mse_loss(critic_values, batch_returns)

                    loss = actor_loss + args.value_coef * critic_loss - args.entropy_coef * entropy
                    optimizer.zero_grad()
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()) + [actor_log_std], args.max_grad_norm)
                    optimizer.step()

            if checkpoint_dir is not None and next_checkpoint_step is not None and total_steps >= next_checkpoint_step:
                checkpoint_path = checkpoint_dir / f'{output_path.stem}_step_{total_steps:07d}.pt'
                _save_checkpoint(
                    checkpoint_path,
                    _checkpoint_payload(
                        actor,
                        critic,
                        actor_log_std,
                        env,
                        args,
                        agent_namespaces,
                        hidden_sizes,
                        scenarios,
                        reward_config,
                        total_steps,
                    ),
                )
                while next_checkpoint_step is not None and total_steps >= next_checkpoint_step:
                    next_checkpoint_step += checkpoint_interval

        _save_checkpoint(
            output_path,
            _checkpoint_payload(
                actor,
                critic,
                actor_log_std,
                env,
                args,
                agent_namespaces,
                hidden_sizes,
                scenarios,
                reward_config,
                total_steps,
            ),
        )
        if checkpoint_dir is not None:
            _run_checkpoint_evaluation(args, checkpoint_dir, scenarios, output_path)
    finally:
        env.close()


if __name__ == '__main__':
    main()