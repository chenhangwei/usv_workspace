import argparse
from contextlib import nullcontext
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from .config import ActionBounds, RewardConfig
from .mappo_parallel_sampler import ParallelRolloutSampler
from .multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig
from .multi_agent_scenarios import MultiAgentScenarioFactory
from .multi_agent_types import AgentLocalObservation, FleetGlobalState


MAX_FASTDDS_SAFE_DOMAIN_ID = 232


def parse_args():
    default_env = MultiAgentEnvConfig()
    parser = argparse.ArgumentParser(description='Minimal MAPPO training entry for multi-USV RL control.')
    parser.add_argument('--output', required=True, help='Output model path (.pt).')
    parser.add_argument('--resume-from', help='Optional checkpoint scaffold (.pt) used to resume MAPPO weights and training metadata.')
    parser.add_argument('--reset-optimizer', action='store_true', help='When resuming, ignore optimizer/scaler state stored in the checkpoint and restart optimizer state from scratch.')
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
    parser.add_argument('--device', default='auto', help='Torch device. Use auto to prefer CUDA when available.')
    parser.add_argument('--amp', choices=['auto', 'on', 'off'], default='auto', help='Mixed precision mode. Auto enables AMP on CUDA and keeps CPU training in full precision.')
    parser.add_argument('--matmul-precision', choices=['highest', 'high', 'medium'], default='high', help='Preferred float32 matmul precision on supported PyTorch builds.')
    parser.add_argument('--disable-tf32', action='store_true', help='Disable TF32 acceleration on Ampere-class GPUs such as A10.')
    parser.add_argument('--torch-num-threads', type=int, default=None, help='Optional torch CPU thread cap to reduce contention with ROS sampling on small CPU instances.')
    parser.add_argument('--hidden-size', action='append', dest='hidden_sizes', type=int, default=None, help='Hidden layer size. Repeatable.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name. Repeatable.')
    parser.add_argument('--scenario-set', choices=['auto', 'smoke', 'dense', 'all'], default='auto', help='Scenario curriculum preset used when --scenario is not provided.')
    parser.add_argument('--max-neighbors', type=int, default=4, help='Neighbor slots in local observation encoding.')
    parser.add_argument('--max-agents', type=int, default=3, help='Maximum agents encoded in global state.')
    parser.add_argument('--rl-control-mode', choices=['pure'], default='pure', help='Pure final-command control mode used during training.')
    parser.add_argument('--action-mode', choices=['full'], default='full', help='Action representation used during training.')
    parser.add_argument('--linear-delta-limit', type=float, default=default_env.action_bounds.linear_delta, help='Maximum forward command magnitude (m/s).')
    parser.add_argument('--angular-delta-limit', type=float, default=default_env.action_bounds.angular_delta, help='Maximum yaw-rate command magnitude (rad/s).')
    parser.add_argument('--cruise-speed', type=float, default=default_env.cruise_speed, help='Training controller cruise speed limit (m/s).')
    parser.add_argument('--max-angular-velocity', type=float, default=default_env.max_angular_velocity, help='Training controller yaw-rate limit (rad/s).')
    parser.add_argument('--episode-timeout', type=float, default=45.0, help='Episode timeout in seconds.')
    parser.add_argument('--no-progress-timeout', type=float, default=10.0, help='No-progress timeout in seconds.')
    parser.add_argument('--min-progress-delta', type=float, default=0.3, help='Fleet progress threshold for refreshing the no-progress timer.')
    parser.add_argument('--progress-weight', type=float, default=RewardConfig.progress_weight, help='Per-agent progress reward weight.')
    parser.add_argument('--goal-proximity-reward-weight', type=float, default=default_env.goal_proximity_reward_weight, help='Per-agent dense reward for moving inside the near-goal relief radius.')
    parser.add_argument('--goal-proximity-relief-distance', type=float, default=default_env.goal_proximity_relief_distance, help='Distance to goal below which near-goal shaping and relief activate.')
    parser.add_argument('--goal-proximity-heading-relief', type=float, default=default_env.goal_proximity_heading_relief, help='Fractional heading penalty relief applied near goal.')
    parser.add_argument('--goal-proximity-smoothness-relief', type=float, default=default_env.goal_proximity_smoothness_relief, help='Fractional smoothness penalty relief applied near goal.')
    parser.add_argument('--goal-proximity-conflict-relief', type=float, default=default_env.goal_proximity_conflict_relief, help='Fractional relief applied to conflict braking and COLREGs shaping near goal.')
    parser.add_argument('--goal-proximity-speed-relief', type=float, default=default_env.goal_proximity_speed_relief, help='Fractional reduction of target forward speed near goal. Use 0 to avoid reward-driven slowdown around goal points.')
    parser.add_argument('--goal-bonus', type=float, default=RewardConfig.goal_bonus, help='Per-agent goal completion bonus.')
    parser.add_argument('--collision-distance', type=float, default=default_env.collision_distance, help='Collision threshold derived from hull footprint (m).')
    parser.add_argument('--near-miss-distance', type=float, default=default_env.near_miss_distance, help='Near-miss threshold (m).')
    parser.add_argument('--scenario-neighbor-speed', type=float, default=default_env.scenario_neighbor_speed, help='Nominal scripted vessel speed used in training scenarios (m/s).')
    parser.add_argument('--collision-penalty', type=float, default=RewardConfig.collision_penalty, help='Penalty applied when fleet minimum separation falls below the collision threshold.')
    parser.add_argument('--near-miss-weight', type=float, default=RewardConfig.near_miss_weight, help='Near-miss penalty weight.')
    parser.add_argument('--conflict-distance', type=float, default=RewardConfig.conflict_distance, help='Distance threshold used for conflict-specific shaping.')
    parser.add_argument('--anticipation-distance', type=float, default=RewardConfig.anticipation_distance, help='Lookahead distance used for anticipatory conflict shaping.')
    parser.add_argument('--conflict-risk-weight', type=float, default=RewardConfig.conflict_risk_weight, help='Conflict-risk penalty weight.')
    parser.add_argument('--conflict-brake-weight', type=float, default=RewardConfig.conflict_brake_weight, help='Conflict braking penalty weight.')
    parser.add_argument('--desired-conflict-speed', type=float, default=RewardConfig.desired_conflict_speed, help='Target forward speed maintained during conflict handling.')
    parser.add_argument('--stop-go-penalty-weight', type=float, default=RewardConfig.stop_go_penalty_weight, help='Penalty weight for rapid stop-go behavior under conflict.')
    parser.add_argument('--head-on-guidance-distance', type=float, default=RewardConfig.head_on_guidance_distance, help='Distance threshold for head-on starboard guidance shaping.')
    parser.add_argument('--head-on-target-starboard-offset', type=float, default=RewardConfig.head_on_target_starboard_offset, help='Desired starboard lateral offset in head-on encounters.')
    parser.add_argument('--head-on-corridor-reward-weight', type=float, default=RewardConfig.head_on_corridor_reward_weight, help='Reward weight for entering the head-on starboard corridor.')
    parser.add_argument('--head-on-centerline-penalty-weight', type=float, default=RewardConfig.head_on_centerline_penalty_weight, help='Penalty weight for staying on the head-on centerline.')
    parser.add_argument('--head-on-turn-reward-weight', type=float, default=RewardConfig.head_on_turn_reward_weight, help='Reward weight for committing to starboard turn in head-on encounters.')
    parser.add_argument('--head-on-forward-reward-weight', type=float, default=RewardConfig.head_on_forward_reward_weight, help='Reward weight for maintaining forward motion during head-on avoidance.')
    parser.add_argument('--head-on-speed-drop-penalty-weight', type=float, default=RewardConfig.head_on_speed_drop_penalty_weight, help='Penalty weight for abrupt forward-speed loss during head-on avoidance.')
    parser.add_argument('--crossing-starboard-turn-reward-weight', type=float, default=RewardConfig.crossing_starboard_turn_reward_weight, help='Reward weight for starboard give-way turns in crossing encounters.')
    parser.add_argument('--crossing-forward-reward-weight', type=float, default=RewardConfig.crossing_forward_reward_weight, help='Reward weight for maintaining forward motion while committing to starboard give-way turns in crossing encounters.')
    parser.add_argument('--overtaking-starboard-turn-reward-weight', type=float, default=RewardConfig.overtaking_starboard_turn_reward_weight, help='Reward weight for starboard bias during overtaking.')
    parser.add_argument('--overtaking-forward-reward-weight', type=float, default=RewardConfig.overtaking_forward_reward_weight, help='Reward weight for maintaining forward motion while committing to starboard overtaking bias.')
    parser.add_argument('--colregs-port-turn-penalty-weight', type=float, default=RewardConfig.colregs_port_turn_penalty_weight, help='Penalty weight for turning to port during starboard-biased COLREGs encounters.')
    parser.add_argument('--heading-relief-factor', type=float, default=RewardConfig.heading_relief_factor, help='Fractional relief applied to heading penalty during conflict handling.')
    parser.add_argument('--heading-error-weight', type=float, default=RewardConfig.heading_error_weight, help='Heading error penalty weight.')
    parser.add_argument('--action-smoothness-weight', type=float, default=RewardConfig.action_smoothness_weight, help='Action smoothness penalty weight.')
    parser.add_argument('--pure-cruise-reward-weight', type=float, default=RewardConfig.pure_cruise_reward_weight, help='Pure-RL reward weight for sustaining aligned forward cruise in open water.')
    parser.add_argument('--pure-idle-penalty-weight', type=float, default=RewardConfig.pure_idle_penalty_weight, help='Pure-RL penalty weight for failing to maintain target forward speed in open water.')
    parser.add_argument('--pure-turn-penalty-weight', type=float, default=RewardConfig.pure_turn_penalty_weight, help='Pure-RL penalty weight for excessive yaw rate away from conflict handling.')
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
    parser.add_argument('--log-interval-updates', type=int, default=1, help='Print rollout/update throughput every N policy updates.')
    parser.add_argument('--num-sampler-workers', type=int, default=1, help='Number of sampler worker processes. Values >1 use process-level ROS domain isolation for rollout collection.')
    parser.add_argument('--base-ros-domain-id', type=int, default=100, help='Base ROS domain ID used when --num-sampler-workers > 1. Worker rank is added to this base.')
    return parser.parse_args()


def _build_agent_namespaces(num_agents: int) -> tuple[str, ...]:
    return tuple(f'usv_{index + 1:02d}' for index in range(num_agents))


def _validate_parallel_sampler_args(args):
    worker_count = max(1, int(args.num_sampler_workers))
    if worker_count <= 1:
        return

    base_domain_id = int(args.base_ros_domain_id)
    highest_domain_id = base_domain_id + worker_count - 1
    if base_domain_id < 0:
        raise ValueError('--base-ros-domain-id must be >= 0.')
    if highest_domain_id > MAX_FASTDDS_SAFE_DOMAIN_ID:
        raise ValueError(
            'Parallel sampler ROS domain range is too high for current Fast DDS defaults: '
            f'base_ros_domain_id={base_domain_id}, num_sampler_workers={worker_count}, '
            f'highest_domain_id={highest_domain_id}, safe_max={MAX_FASTDDS_SAFE_DOMAIN_ID}. '
            'Choose a lower base ROS domain ID.'
        )


def _build_mlp(nn, input_dim: int, hidden_sizes: tuple[int, ...], output_dim: int):
    layers = []
    current_dim = input_dim
    for hidden_size in hidden_sizes:
        layers.append(nn.Linear(current_dim, hidden_size))
        layers.append(nn.Tanh())
        current_dim = hidden_size
    layers.append(nn.Linear(current_dim, output_dim))
    return nn.Sequential(*layers)


def _cuda_device_support_status(torch, device) -> tuple[bool, str | None]:
    if not torch.cuda.is_available():
        return False, 'CUDA is not available.'

    try:
        device_index = device.index if device.index is not None else torch.cuda.current_device()
        capability = torch.cuda.get_device_capability(device_index)
        device_name = torch.cuda.get_device_name(device_index)
        arch_token = f'sm_{capability[0]}{capability[1]}'
        arch_list = set(torch.cuda.get_arch_list()) if hasattr(torch.cuda, 'get_arch_list') else set()
        if arch_list and arch_token not in arch_list:
            supported = ' '.join(sorted(arch_list))
            return (
                False,
                f'CUDA device {device_name} exposes {arch_token}, but current PyTorch only supports: {supported}',
            )
    except Exception as exc:
        return False, f'Failed to validate CUDA runtime compatibility: {exc}'

    return True, None


def _resolve_device(torch, device_arg: str):
    if device_arg == 'auto':
        cuda_device = torch.device('cuda')
        cuda_ok, reason = _cuda_device_support_status(torch, cuda_device)
        if cuda_ok:
            return cuda_device
        print(f'Falling back to CPU for MAPPO training: {reason}', flush=True)
        return torch.device('cpu')

    device = torch.device(device_arg)
    if device.type == 'cuda':
        cuda_ok, reason = _cuda_device_support_status(torch, device)
        if not cuda_ok:
            raise RuntimeError(reason)
    return device


def _configure_torch_runtime(torch, args, device):
    if args.torch_num_threads is not None and args.torch_num_threads > 0:
        torch.set_num_threads(int(args.torch_num_threads))

    if hasattr(torch, 'set_float32_matmul_precision'):
        torch.set_float32_matmul_precision(str(args.matmul_precision))

    allow_tf32 = device.type == 'cuda' and not bool(args.disable_tf32)
    if hasattr(torch.backends, 'cuda') and hasattr(torch.backends.cuda, 'matmul'):
        torch.backends.cuda.matmul.allow_tf32 = allow_tf32
    if hasattr(torch.backends, 'cudnn'):
        torch.backends.cudnn.allow_tf32 = allow_tf32

    return allow_tf32


def _should_enable_amp(args, device) -> bool:
    if device.type != 'cuda':
        return False
    if args.amp == 'on':
        return True
    if args.amp == 'off':
        return False
    return True


def _build_grad_scaler(torch, amp_enabled: bool):
    if hasattr(torch, 'amp') and hasattr(torch.amp, 'GradScaler'):
        return torch.amp.GradScaler('cuda', enabled=amp_enabled)
    return torch.cuda.amp.GradScaler(enabled=amp_enabled)


def _gpu_runtime_stats(torch, device) -> str:
    if device.type != 'cuda':
        return 'gpu=cpu'

    allocated_mb = torch.cuda.memory_allocated(device) / (1024 ** 2)
    reserved_mb = torch.cuda.memory_reserved(device) / (1024 ** 2)
    max_allocated_mb = torch.cuda.max_memory_allocated(device) / (1024 ** 2)
    return (
        f'gpu_mem_alloc={allocated_mb:.0f}MiB, '
        f'gpu_mem_reserved={reserved_mb:.0f}MiB, '
        f'gpu_mem_peak={max_allocated_mb:.0f}MiB'
    )


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


def _build_model_metadata(args, agent_namespaces: tuple[str, ...]) -> dict:
    action_dim = 1 if args.action_mode == 'angular_only' else 2
    if args.rl_control_mode == 'pure':
        linear_limit = max(float(args.linear_delta_limit), float(args.cruise_speed))
        angular_limit = max(float(args.angular_delta_limit), float(args.max_angular_velocity))
        action_low = np.asarray(
            [0.0, -angular_limit],
            dtype=np.float32,
        )
        action_high = np.asarray(
            [linear_limit, angular_limit],
            dtype=np.float32,
        )
    else:
        action_low = np.asarray(
            [-float(args.angular_delta_limit)] if action_dim == 1 else [
                -float(args.linear_delta_limit),
                -float(args.angular_delta_limit),
            ],
            dtype=np.float32,
        )
        action_high = np.asarray(
            [float(args.angular_delta_limit)] if action_dim == 1 else [
                float(args.linear_delta_limit),
                float(args.angular_delta_limit),
            ],
            dtype=np.float32,
        )
    max_agents = max(len(agent_namespaces), int(args.max_agents))
    return {
        'local_observation_size': AgentLocalObservation.vector_size(int(args.max_neighbors)),
        'global_state_size': FleetGlobalState.vector_size(max_agents, int(args.max_neighbors)),
        'action_dim': action_dim,
        'action_low': action_low,
        'action_high': action_high,
    }


def _checkpoint_payload(
    actor,
    critic,
    actor_log_std,
    optimizer,
    scaler,
    model_metadata: dict,
    args,
    agent_namespaces: tuple[str, ...],
    hidden_sizes: tuple[int, ...],
    scenarios: tuple[str, ...],
    reward_config: RewardConfig,
    total_steps: int,
    update_index: int,
    next_checkpoint_step,
):
    payload = {
        'actor_state_dict': actor.state_dict(),
        'critic_state_dict': critic.state_dict(),
        'actor_log_std': actor_log_std.detach().cpu(),
        'agent_namespaces': agent_namespaces,
        'local_observation_size': model_metadata['local_observation_size'],
        'global_state_size': model_metadata['global_state_size'],
        'action_dim': model_metadata['action_dim'],
        'hidden_sizes': hidden_sizes,
        'action_low': model_metadata['action_low'].tolist(),
        'action_high': model_metadata['action_high'].tolist(),
        'scenarios': scenarios,
        'max_neighbors': args.max_neighbors,
        'max_agents': max(len(agent_namespaces), args.max_agents),
        'rl_control_mode': args.rl_control_mode,
        'action_mode': args.action_mode,
        'episode_timeout': float(args.episode_timeout),
        'no_progress_timeout': float(args.no_progress_timeout),
        'min_progress_delta': float(args.min_progress_delta),
        'reward_config': reward_config.__dict__,
        'goal_proximity_reward_weight': float(args.goal_proximity_reward_weight),
        'goal_proximity_relief_distance': float(args.goal_proximity_relief_distance),
        'goal_proximity_heading_relief': float(args.goal_proximity_heading_relief),
        'goal_proximity_smoothness_relief': float(args.goal_proximity_smoothness_relief),
        'goal_proximity_conflict_relief': float(args.goal_proximity_conflict_relief),
        'goal_proximity_speed_relief': float(args.goal_proximity_speed_relief),
        'collision_distance': float(args.collision_distance),
        'near_miss_distance': float(args.near_miss_distance),
        'scenario_neighbor_speed': float(args.scenario_neighbor_speed),
        'cruise_speed': float(args.cruise_speed),
        'max_angular_velocity': float(args.max_angular_velocity),
        'action_bounds': {
            'linear_delta': float(args.linear_delta_limit),
            'angular_delta': float(args.angular_delta_limit),
        },
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
        'update_index': int(update_index),
        'next_checkpoint_step': int(next_checkpoint_step) if next_checkpoint_step is not None else None,
    }
    if optimizer is not None:
        payload['optimizer_state_dict'] = optimizer.state_dict()
    if scaler is not None and hasattr(scaler, 'state_dict'):
        payload['grad_scaler_state_dict'] = scaler.state_dict()
    return payload


def _compute_advantages_and_returns(rewards, values, dones, bootstrap_values, gamma: float, gae_lambda: float):
    advantages = np.zeros_like(rewards, dtype=np.float32)
    returns = np.zeros_like(rewards, dtype=np.float32)
    gae = np.zeros(rewards.shape[1], dtype=np.float32)
    next_values = bootstrap_values.astype(np.float32)

    for step in reversed(range(rewards.shape[0])):
        mask = 1.0 - dones[step]
        delta = rewards[step] + gamma * next_values * mask - values[step]
        gae = delta + gamma * gae_lambda * mask * gae
        advantages[step] = gae
        returns[step] = gae + values[step]
        next_values = values[step]

    return advantages, returns


def _save_checkpoint(checkpoint_path: Path, payload: dict):
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    torch = __import__('torch')
    torch.save(payload, checkpoint_path)
    print(f'Saved MAPPO checkpoint scaffold to {checkpoint_path}', flush=True)


def _next_checkpoint_step_after(total_steps: int, checkpoint_interval: int):
    if checkpoint_interval <= 0:
        return None
    return ((max(0, int(total_steps)) // checkpoint_interval) + 1) * checkpoint_interval


def _load_resume_payload(torch, resume_path: Path) -> dict:
    if not resume_path.exists():
        raise FileNotFoundError(f'Resume checkpoint does not exist: {resume_path}')

    payload = torch.load(resume_path, map_location='cpu')
    if not isinstance(payload, dict):
        raise ValueError(f'Resume checkpoint is not a MAPPO scaffold dictionary: {resume_path}')

    required_keys = ('actor_state_dict', 'critic_state_dict', 'actor_log_std')
    missing = [key for key in required_keys if key not in payload]
    if missing:
        raise ValueError(
            f'Resume checkpoint is missing required MAPPO fields {missing}: {resume_path}'
        )
    return payload


def _apply_resume_configuration(args, payload: dict):
    agent_namespaces = payload.get('agent_namespaces')
    if agent_namespaces:
        args.num_agents = len(tuple(agent_namespaces))

    hidden_sizes = payload.get('hidden_sizes')
    if hidden_sizes:
        args.hidden_sizes = [int(size) for size in hidden_sizes]

    scenarios = payload.get('scenarios')
    if scenarios:
        args.scenarios = list(scenarios)
        args.scenario_set = 'auto'

    simple_fields = (
        'max_neighbors',
        'max_agents',
        'rl_control_mode',
        'action_mode',
        'episode_timeout',
        'no_progress_timeout',
        'min_progress_delta',
        'goal_proximity_reward_weight',
        'goal_proximity_relief_distance',
        'goal_proximity_heading_relief',
        'goal_proximity_smoothness_relief',
        'goal_proximity_conflict_relief',
        'goal_proximity_speed_relief',
        'collision_distance',
        'near_miss_distance',
        'scenario_neighbor_speed',
        'cruise_speed',
        'max_angular_velocity',
        'team_reward_weight',
        'team_progress_weight',
        'team_goal_proximity_weight',
        'team_regression_penalty_weight',
        'team_dispersion_penalty_weight',
        'team_dispersion_margin',
        'coordination_reward_weight',
        'team_completion_bonus',
        'deadlock_penalty_weight',
    )
    for field in simple_fields:
        if field in payload and payload[field] is not None:
            setattr(args, field, payload[field])

    action_bounds = payload.get('action_bounds') or {}
    if 'linear_delta' in action_bounds:
        args.linear_delta_limit = float(action_bounds['linear_delta'])
    if 'angular_delta' in action_bounds:
        args.angular_delta_limit = float(action_bounds['angular_delta'])

    reward_config = payload.get('reward_config') or {}
    reward_fields = (
        'progress_weight',
        'goal_bonus',
        'collision_penalty',
        'near_miss_weight',
        'conflict_distance',
        'anticipation_distance',
        'conflict_risk_weight',
        'conflict_brake_weight',
        'desired_conflict_speed',
        'stop_go_penalty_weight',
        'head_on_guidance_distance',
        'head_on_target_starboard_offset',
        'head_on_corridor_reward_weight',
        'head_on_centerline_penalty_weight',
        'head_on_turn_reward_weight',
        'head_on_forward_reward_weight',
        'head_on_speed_drop_penalty_weight',
        'crossing_starboard_turn_reward_weight',
        'crossing_forward_reward_weight',
        'overtaking_starboard_turn_reward_weight',
        'overtaking_forward_reward_weight',
        'colregs_port_turn_penalty_weight',
        'heading_relief_factor',
        'heading_error_weight',
        'action_smoothness_weight',
        'pure_cruise_reward_weight',
        'pure_idle_penalty_weight',
        'pure_turn_penalty_weight',
        'time_penalty',
        'stall_penalty',
    )
    for field in reward_fields:
        if field in reward_config and reward_config[field] is not None:
            setattr(args, field, reward_config[field])


def _move_optimizer_state_to_device(torch, optimizer, device):
    for state in optimizer.state.values():
        for key, value in list(state.items()):
            if torch.is_tensor(value):
                state[key] = value.to(device=device)


def _restore_resume_state(torch, device, actor, critic, actor_log_std, optimizer, scaler, args, payload: dict):
    actor.load_state_dict(payload['actor_state_dict'])
    critic.load_state_dict(payload['critic_state_dict'])
    actor_log_std.data.copy_(
        torch.as_tensor(payload['actor_log_std'], dtype=actor_log_std.dtype, device=device)
    )

    optimizer_restored = False
    scaler_restored = False
    if not args.reset_optimizer and payload.get('optimizer_state_dict') is not None:
        optimizer.load_state_dict(payload['optimizer_state_dict'])
        _move_optimizer_state_to_device(torch, optimizer, device)
        optimizer_restored = True

        scaler_state = payload.get('grad_scaler_state_dict')
        if scaler_state is not None and scaler is not None and hasattr(scaler, 'load_state_dict'):
            scaler.load_state_dict(scaler_state)
            scaler_restored = True

    completed_timesteps = max(0, int(payload.get('completed_timesteps', 0)))
    checkpoint_interval = max(0, int(args.checkpoint_interval))
    saved_next_checkpoint_step = payload.get('next_checkpoint_step')
    if saved_next_checkpoint_step is not None:
        saved_next_checkpoint_step = int(saved_next_checkpoint_step)
    next_checkpoint_step = saved_next_checkpoint_step if saved_next_checkpoint_step and saved_next_checkpoint_step > completed_timesteps else _next_checkpoint_step_after(completed_timesteps, checkpoint_interval)

    approximate_update_index = completed_timesteps // max(1, int(args.rollout_steps) * max(1, int(args.num_agents)))
    update_index = int(payload.get('update_index', approximate_update_index))

    return {
        'completed_timesteps': completed_timesteps,
        'update_index': update_index,
        'next_checkpoint_step': next_checkpoint_step,
        'optimizer_restored': optimizer_restored,
        'scaler_restored': scaler_restored,
    }


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
    print(f'Automatic checkpoint ranking saved to {ranking_json}', flush=True)


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


def _recover_env(env_factory, *, context: str, max_attempts: int = 3):
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
                f'Warning: {context} failed with {exc} '
                f'(attempt {attempt}/{max_attempts}). Recreating multi-agent env.'
                ,
                flush=True,
            )
            if env is not None:
                env.close()
    raise RuntimeError(f'Failed to recover multi-agent env during {context} after {max_attempts} attempts.') from last_error


def main():
    args = parse_args()
    try:
        import torch
        from torch import nn
        from torch.distributions import Normal
    except ImportError as exc:
        raise RuntimeError('torch is required for MAPPO training.') from exc

    resume_payload = None
    resume_path = Path(args.resume_from) if args.resume_from else None
    if resume_path is not None:
        resume_payload = _load_resume_payload(torch, resume_path)
        _apply_resume_configuration(args, resume_payload)
        print(f'Resuming MAPPO from checkpoint scaffold: {resume_path}', flush=True)

    _validate_parallel_sampler_args(args)

    hidden_sizes = tuple(args.hidden_sizes or [128, 128])
    agent_namespaces = _build_agent_namespaces(max(2, args.num_agents))
    scenarios = _resolve_scenarios(args)
    reward_config = _build_reward_config(args)
    model_metadata = _build_model_metadata(args, agent_namespaces)
    output_path = Path(args.output)
    if resume_path is not None and output_path.resolve() == resume_path.resolve():
        print(
            f'Warning: --output points to the same file as --resume-from ({output_path}). '
            'Training will overwrite the resume scaffold at the end of the run.'
            ,
            flush=True,
        )
    checkpoint_dir = None
    checkpoint_interval = max(0, int(args.checkpoint_interval))
    if checkpoint_interval > 0:
        checkpoint_dir = Path(args.checkpoint_dir) if args.checkpoint_dir else output_path.with_name(f'{output_path.stem}_checkpoints')
    print(f'Using MAPPO scenario curriculum: {scenarios}', flush=True)
    env_factory = lambda: _create_env(args, agent_namespaces, scenarios, reward_config)
    max_env_recovery_attempts = 3
    max_consecutive_env_failures = 5
    consecutive_env_failures = 0
    env = None
    sampler = None

    device = _resolve_device(torch, args.device)
    tf32_enabled = _configure_torch_runtime(torch, args, device)
    amp_enabled = _should_enable_amp(args, device)
    scaler = _build_grad_scaler(torch, amp_enabled)
    autocast_context = (
        (lambda: torch.autocast(device_type='cuda', dtype=torch.float16))
        if amp_enabled else nullcontext
    )
    print(
        'Using torch runtime: '
        f'device={device}, amp={amp_enabled}, tf32={tf32_enabled}, '
        f'torch_threads={torch.get_num_threads()}'
        ,
        flush=True,
    )

    if int(args.num_sampler_workers) > 1:
        print(
            f'Using parallel MAPPO samplers: workers={int(args.num_sampler_workers)}, '
            f'base_ros_domain_id={int(args.base_ros_domain_id)}',
            flush=True,
        )

    try:
        if int(args.num_sampler_workers) > 1:
            sampler = ParallelRolloutSampler(
                args_dict=vars(args).copy(),
                agent_namespaces=agent_namespaces,
                scenarios=scenarios,
                hidden_sizes=hidden_sizes,
                num_workers=int(args.num_sampler_workers),
                base_ros_domain_id=int(args.base_ros_domain_id),
            )
            observations = None
            info = None
            global_state = None
        else:
            env, observations, info = _recover_env(
                env_factory,
                context='initial MAPPO environment setup',
                max_attempts=max_env_recovery_attempts,
            )
            global_state = info['global_state']

        actor = _build_mlp(nn, model_metadata['local_observation_size'], hidden_sizes, model_metadata['action_dim']).to(device)
        critic = _build_mlp(nn, model_metadata['local_observation_size'] + model_metadata['global_state_size'], hidden_sizes, 1).to(device)
        actor_log_std = nn.Parameter(torch.zeros(model_metadata['action_dim'], device=device))
        optimizer = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()) + [actor_log_std], lr=args.learning_rate)
        action_low_tensor = torch.as_tensor(model_metadata['action_low'], dtype=torch.float32, device=device)
        action_high_tensor = torch.as_tensor(model_metadata['action_high'], dtype=torch.float32, device=device)

        total_steps = 0
        rollout_steps = max(1, args.rollout_steps)
        next_checkpoint_step = _next_checkpoint_step_after(total_steps, checkpoint_interval)
        update_index = 0

        if resume_payload is not None:
            resume_state = _restore_resume_state(
                torch,
                device,
                actor,
                critic,
                actor_log_std,
                optimizer,
                scaler,
                args,
                resume_payload,
            )
            total_steps = resume_state['completed_timesteps']
            update_index = resume_state['update_index']
            next_checkpoint_step = resume_state['next_checkpoint_step']
            print(
                'Resume state restored: '
                f'completed_timesteps={total_steps}, '
                f'optimizer_restored={resume_state["optimizer_restored"]}, '
                f'scaler_restored={resume_state["scaler_restored"]}, '
                f'next_checkpoint_step={next_checkpoint_step}'
                ,
                flush=True,
            )
            if int(args.total_timesteps) <= total_steps:
                print(
                    f'Warning: requested total_timesteps={int(args.total_timesteps)} is not greater than '
                    f'the resumed completed_timesteps={total_steps}. The run will only emit the final output scaffold.'
                    ,
                    flush=True,
                )

        while total_steps < max(0, args.total_timesteps):
            rollout_wall_start = time.perf_counter()
            remaining_agent_steps = max(0, int(args.total_timesteps) - total_steps)
            current_rollout_steps = min(rollout_steps, max(1, int(math.ceil(remaining_agent_steps / len(agent_namespaces)))))

            rollout_agent_steps = 0
            rollout_steps_collected = 0
            if sampler is not None:
                worker_results = sampler.collect(
                    actor=actor,
                    critic=critic,
                    actor_log_std=actor_log_std,
                    rollout_steps=current_rollout_steps,
                    remaining_agent_steps=remaining_agent_steps,
                )
                rollout_wall_time = max(1e-6, time.perf_counter() - rollout_wall_start)
                if not worker_results:
                    if total_steps >= max(0, args.total_timesteps):
                        break
                    continue

                flat_obs_parts = []
                flat_states_parts = []
                flat_actions_parts = []
                flat_log_probs_parts = []
                flat_advantages_parts = []
                flat_returns_parts = []

                for result in worker_results:
                    rewards = np.asarray(result['rewards'], dtype=np.float32)
                    values = np.asarray(result['values'], dtype=np.float32)
                    dones = np.asarray(result['dones'], dtype=np.float32)
                    advantages, returns = _compute_advantages_and_returns(
                        rewards,
                        values,
                        dones,
                        np.asarray(result['bootstrap_values'], dtype=np.float32),
                        args.gamma,
                        args.gae_lambda,
                    )
                    flat_obs_parts.append(np.asarray(result['obs'], dtype=np.float32).reshape(-1, model_metadata['local_observation_size']))
                    flat_states_parts.append(np.asarray(result['states'], dtype=np.float32).reshape(-1, model_metadata['global_state_size']))
                    flat_actions_parts.append(np.asarray(result['actions'], dtype=np.float32).reshape(-1, model_metadata['action_dim']))
                    flat_log_probs_parts.append(np.asarray(result['log_probs'], dtype=np.float32).reshape(-1))
                    flat_advantages_parts.append(advantages.reshape(-1))
                    flat_returns_parts.append(returns.reshape(-1))
                    rollout_agent_steps += int(result['agent_steps'])
                    rollout_steps_collected += int(result['rollout_steps_collected'])

                total_steps += rollout_agent_steps
                flat_obs = torch.as_tensor(np.concatenate(flat_obs_parts, axis=0), dtype=torch.float32, device=device)
                flat_states = torch.as_tensor(np.concatenate(flat_states_parts, axis=0), dtype=torch.float32, device=device)
                flat_actions = torch.as_tensor(np.concatenate(flat_actions_parts, axis=0), dtype=torch.float32, device=device)
                flat_old_log_probs = torch.as_tensor(np.concatenate(flat_log_probs_parts, axis=0), dtype=torch.float32, device=device)
                flat_advantages = torch.as_tensor(np.concatenate(flat_advantages_parts, axis=0), dtype=torch.float32, device=device)
                flat_returns = torch.as_tensor(np.concatenate(flat_returns_parts, axis=0), dtype=torch.float32, device=device)
            else:
                storage_obs = []
                storage_states = []
                storage_actions = []
                storage_log_probs = []
                storage_values = []
                storage_rewards = []
                storage_dones = []

                for _ in range(current_rollout_steps):
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
                            f'Warning: MAPPO rollout step failed after {total_steps} timesteps with {exc}. '
                            f'Recovering environment ({consecutive_env_failures}/{max_consecutive_env_failures}).'
                            ,
                            flush=True,
                        )
                        if consecutive_env_failures >= max_consecutive_env_failures:
                            raise RuntimeError(
                                f'MAPPO training aborted after {consecutive_env_failures} consecutive environment failures.'
                            ) from exc
                        if storage_dones:
                            storage_dones[-1] = np.ones(len(agent_order), dtype=np.float32)
                        env.close()
                        env, observations, info = _recover_env(
                            env_factory,
                            context='MAPPO rollout step recovery',
                            max_attempts=max_env_recovery_attempts,
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

                    total_steps += len(agent_order)
                    rollout_agent_steps += len(agent_order)
                    rollout_steps_collected += 1
                    observations = next_observations
                    global_state = next_info['global_state']
                    if done:
                        try:
                            observations, info = env.reset()
                        except RuntimeError as exc:
                            consecutive_env_failures += 1
                            print(
                                f'Warning: MAPPO episode reset failed after {total_steps} timesteps with {exc}. '
                                f'Recovering environment ({consecutive_env_failures}/{max_consecutive_env_failures}).'
                                ,
                                flush=True,
                            )
                            if consecutive_env_failures >= max_consecutive_env_failures:
                                raise RuntimeError(
                                    f'MAPPO training aborted after {consecutive_env_failures} consecutive environment failures.'
                                ) from exc
                            env.close()
                            env, observations, info = _recover_env(
                                env_factory,
                                context='MAPPO episode reset recovery',
                                max_attempts=max_env_recovery_attempts,
                            )
                        else:
                            consecutive_env_failures = 0
                        global_state = info['global_state']
                    if total_steps >= args.total_timesteps:
                        break

                if not storage_obs:
                    if total_steps >= max(0, args.total_timesteps):
                        break
                    continue

                rollout_wall_time = max(1e-6, time.perf_counter() - rollout_wall_start)

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
                advantages, returns = _compute_advantages_and_returns(
                    rewards,
                    values,
                    dones,
                    bootstrap_values,
                    args.gamma,
                    args.gae_lambda,
                )

                flat_obs = torch.as_tensor(np.asarray(storage_obs, dtype=np.float32).reshape(-1, model_metadata['local_observation_size']), dtype=torch.float32, device=device)
                flat_states = torch.as_tensor(np.asarray(storage_states, dtype=np.float32).reshape(-1, model_metadata['global_state_size']), dtype=torch.float32, device=device)
                flat_actions = torch.as_tensor(np.asarray(storage_actions, dtype=np.float32).reshape(-1, model_metadata['action_dim']), dtype=torch.float32, device=device)
                flat_old_log_probs = torch.as_tensor(np.asarray(storage_log_probs, dtype=np.float32).reshape(-1), dtype=torch.float32, device=device)
                flat_advantages = torch.as_tensor(advantages.reshape(-1), dtype=torch.float32, device=device)
                flat_returns = torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=device)

            flat_advantages = (flat_advantages - flat_advantages.mean()) / (flat_advantages.std(unbiased=False) + 1e-6)
            sample_count = flat_obs.shape[0]
            minibatch_size = min(max(1, args.minibatch_size), sample_count)
            update_wall_start = time.perf_counter()

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

                    with autocast_context():
                        action_mean = actor(batch_obs)
                        action_std = actor_log_std.exp().expand_as(action_mean).to(dtype=action_mean.dtype)
                        distribution = Normal(action_mean, action_std)
                        new_log_probs = distribution.log_prob(batch_actions.to(dtype=action_mean.dtype)).sum(dim=-1)
                        entropy = distribution.entropy().sum(dim=-1).mean()

                        ratio = torch.exp(new_log_probs - batch_old_log_probs.to(dtype=new_log_probs.dtype))
                        batch_advantages_cast = batch_advantages.to(dtype=ratio.dtype)
                        surrogate_one = ratio * batch_advantages_cast
                        surrogate_two = torch.clamp(ratio, 1.0 - args.clip_range, 1.0 + args.clip_range) * batch_advantages_cast
                        actor_loss = -torch.min(surrogate_one, surrogate_two).mean()

                        critic_values = critic(torch.cat([batch_obs, batch_states], dim=-1)).squeeze(-1)
                        critic_loss = torch.nn.functional.mse_loss(
                            critic_values,
                            batch_returns.to(dtype=critic_values.dtype),
                        )

                        loss = actor_loss + args.value_coef * critic_loss - args.entropy_coef * entropy

                    optimizer.zero_grad(set_to_none=True)
                    if amp_enabled:
                        scaler.scale(loss).backward()
                        scaler.unscale_(optimizer)
                        torch.nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()) + [actor_log_std], args.max_grad_norm)
                        scaler.step(optimizer)
                        scaler.update()
                    else:
                        loss.backward()
                        torch.nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()) + [actor_log_std], args.max_grad_norm)
                        optimizer.step()

            update_wall_time = max(1e-6, time.perf_counter() - update_wall_start)
            update_index += 1
            if update_index % max(1, int(args.log_interval_updates)) == 0:
                optimizer_samples = int(sample_count * max(1, args.update_epochs))
                rollout_sps = rollout_agent_steps / rollout_wall_time
                optimizer_sps = optimizer_samples / update_wall_time
                loop_time = rollout_wall_time + update_wall_time
                update_share = update_wall_time / max(loop_time, 1e-6)
                print(
                    f'[MAPPO][update={update_index}] total_steps={total_steps} '
                    f'rollout_steps={rollout_steps_collected} sample_count={sample_count} '
                    f'rollout_time={rollout_wall_time:.2f}s rollout_sps={rollout_sps:.1f} '
                    f'update_time={update_wall_time:.2f}s update_sps={optimizer_sps:.1f} '
                    f'update_share={update_share:.2%} loss={float(loss.detach().cpu().item()):.4f} '
                    f'{_gpu_runtime_stats(torch, device)}'
                    ,
                    flush=True,
                )

            if checkpoint_dir is not None and next_checkpoint_step is not None and total_steps >= next_checkpoint_step:
                checkpoint_path = checkpoint_dir / f'{output_path.stem}_step_{total_steps:07d}.pt'
                _save_checkpoint(
                    checkpoint_path,
                    _checkpoint_payload(
                        actor,
                        critic,
                        actor_log_std,
                        optimizer,
                        scaler,
                        model_metadata,
                        args,
                        agent_namespaces,
                        hidden_sizes,
                        scenarios,
                        reward_config,
                        total_steps,
                        update_index,
                        _next_checkpoint_step_after(total_steps, checkpoint_interval),
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
                optimizer,
                scaler,
                model_metadata,
                args,
                agent_namespaces,
                hidden_sizes,
                scenarios,
                reward_config,
                total_steps,
                update_index,
                _next_checkpoint_step_after(total_steps, checkpoint_interval),
            ),
        )
        if checkpoint_dir is not None:
            _run_checkpoint_evaluation(args, checkpoint_dir, scenarios, output_path)
    finally:
        if sampler is not None:
            sampler.close()
        if env is not None:
            env.close()


if __name__ == '__main__':
    main()