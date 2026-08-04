"""Collect a COLREGS rule-teacher dataset on MultiAgentEnv for BC bootstrap (M2-2).

Drives every agent in MultiAgentEnv with the MultiAgentColregsTeacher and records
(observation_vector, teacher_action) pairs. The observation vector is exactly the
flat actor input the student network consumes (so the BC dataset transfers
directly to the speed_scale policy), and the teacher action is computed from the
structured observation the env exposes via ``_latest_observations``.

Env config mirrors the fresh601/602 speed_scale line so the state distribution
and obs layout match the policy that will be warm-started from the BC weights.

Example:
  python3 -m usv_rl.collect_multi_agent_teacher_dataset \
    --output /mnt/data/checkpoints/usv_rl/teacher_3usv.npz \
    --episodes 60 --steps-per-episode 240 \
    --scenario three_usv_crossing --scenario three_usv_overtaking \
    --scenario three_usv_clear_route --scenario three_usv_random_encounter
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig
from .config import ActionBounds
from .multi_agent_teacher import MultiAgentColregsTeacher


_DEFAULT_SCENARIOS = (
    'three_usv_crossing',
    'three_usv_overtaking',
    'three_usv_clear_route',
    'three_usv_random_encounter',
)


def parse_args():
    p = argparse.ArgumentParser(description='Collect COLREGS teacher dataset on MultiAgentEnv.')
    p.add_argument('--output', required=True, help='Output .npz path.')
    p.add_argument('--episodes', type=int, default=60)
    p.add_argument('--steps-per-episode', type=int, default=240)
    p.add_argument('--num-agents', type=int, default=3)
    p.add_argument('--max-agents', type=int, default=5)
    p.add_argument('--max-neighbors', type=int, default=4)
    p.add_argument('--scenario', action='append', dest='scenarios', default=None,
                   help='Scenario to include (repeatable). Defaults to the 3-USV set.')
    p.add_argument('--action-mode', choices=['speed_scale', 'full', 'angular_only'], default='speed_scale')
    p.add_argument('--action-speed-scale-min', type=float, default=0.30)
    p.add_argument('--cruise-speed', type=float, default=0.34)
    p.add_argument('--max-angular-velocity', type=float, default=0.50)
    p.add_argument('--linear-delta-limit', type=float, default=0.30)
    p.add_argument('--angular-delta-limit', type=float, default=0.50)
    p.add_argument('--sim-tau-linear', type=float, default=0.60)
    p.add_argument('--sim-tau-angular', type=float, default=0.35)
    p.add_argument('--collision-distance', type=float, default=0.75)
    p.add_argument('--near-miss-distance', type=float, default=1.20)
    p.add_argument('--scenario-neighbor-speed', type=float, default=0.34)
    p.add_argument('--episode-timeout', type=float, default=110.0)
    p.add_argument('--no-progress-timeout', type=float, default=60.0)
    p.add_argument('--min-progress-delta', type=float, default=0.006)
    p.add_argument('--scenario-spawn-position-std', type=float, default=0.10)
    p.add_argument('--scenario-spawn-heading-std', type=float, default=0.05)
    p.add_argument('--scenario-goal-position-std', type=float, default=0.08)
    p.add_argument('--base-ros-domain-id', type=int, default=290)
    p.add_argument('--seed', type=int, default=0)
    return p.parse_args()


def _build_env(args, scenarios) -> MultiAgentEnv:
    namespaces = tuple(f'usv_{i + 1:02d}' for i in range(max(1, args.num_agents)))
    config = MultiAgentEnvConfig(
        agent_namespaces=namespaces,
        launch_sitl=False,
        enable_rl_backend=True,
        rl_control_mode='pure',
        action_mode=args.action_mode,
        action_speed_scale_min=float(args.action_speed_scale_min),
        action_bounds=ActionBounds(
            linear_delta=float(args.linear_delta_limit),
            angular_delta=float(args.angular_delta_limit),
        ),
        max_neighbors=max(1, args.max_neighbors),
        max_agents=max(len(namespaces), args.max_agents),
        cruise_speed=float(args.cruise_speed),
        max_angular_velocity=float(args.max_angular_velocity),
        sim_tau_linear=float(args.sim_tau_linear),
        sim_tau_angular=float(args.sim_tau_angular),
        collision_distance=float(args.collision_distance),
        near_miss_distance=float(args.near_miss_distance),
        scenario_neighbor_speed=float(args.scenario_neighbor_speed),
        episode_timeout=float(args.episode_timeout),
        no_progress_timeout=float(args.no_progress_timeout),
        min_progress_delta=float(args.min_progress_delta),
        scenario_spawn_position_std=float(args.scenario_spawn_position_std),
        scenario_spawn_heading_std=float(args.scenario_spawn_heading_std),
        scenario_goal_position_std=float(args.scenario_goal_position_std),
        default_scenarios=tuple(scenarios),
    )
    return MultiAgentEnv(config)


def main():
    args = parse_args()
    scenarios = tuple(args.scenarios) if args.scenarios else _DEFAULT_SCENARIOS

    env = _build_env(args, scenarios)
    teacher = MultiAgentColregsTeacher(
        action_mode=args.action_mode,
        cruise_speed=float(args.cruise_speed),
        max_angular_velocity=float(args.max_angular_velocity),
        action_speed_scale_min=float(args.action_speed_scale_min),
        linear_speed_limit=float(args.cruise_speed),
    )

    observations_out: list[np.ndarray] = []
    actions_out: list[np.ndarray] = []
    scenario_out: list[str] = []

    try:
        for episode in range(args.episodes):
            scenario_name = scenarios[episode % len(scenarios)]
            obs_map, info = env.reset(
                seed=args.seed + episode,
                options={'scenario_kind': scenario_name},
            )
            active_scenario = info.get('scenario', scenario_name)
            for _ in range(args.steps_per_episode):
                action_map = {}
                for agent_id in env.agent_ids:
                    struct = env._latest_observations.get(agent_id)
                    if struct is None:
                        # Fallback: no structured obs -> straight, full speed.
                        action_map[agent_id] = teacher.act_default() if hasattr(teacher, 'act_default') else _idle_action(args)
                        continue
                    action = teacher.act(struct)
                    action_map[agent_id] = action
                    observations_out.append(np.asarray(obs_map[agent_id], dtype=np.float32))
                    actions_out.append(np.asarray(action, dtype=np.float32))
                    scenario_out.append(active_scenario)
                obs_map, _, terminated_dict, truncated_dict, info = env.step(action_map)
                active_scenario = info.get('scenario', active_scenario)
                done = all(terminated_dict.get(a, False) or truncated_dict.get(a, False)
                           for a in env.agent_ids)
                if done:
                    break
            print(f'[collect] episode {episode + 1}/{args.episodes} scenario={scenario_name} '
                  f'cumulative_samples={len(observations_out)}', flush=True)
    finally:
        env.close()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        observations=np.asarray(observations_out, dtype=np.float32),
        actions=np.asarray(actions_out, dtype=np.float32),
        scenario_names=np.asarray(scenario_out),
        action_mode=np.asarray(args.action_mode),
        action_speed_scale_min=np.asarray(args.action_speed_scale_min, dtype=np.float32),
        max_neighbors=np.asarray(args.max_neighbors, dtype=np.int64),
    )
    print(f'Saved {len(observations_out)} teacher samples to {output_path}', flush=True)


def _idle_action(args):
    if args.action_mode == 'angular_only':
        return np.asarray([0.0], dtype=np.float32)
    if args.action_mode == 'speed_scale':
        return np.asarray([1.0, 0.0], dtype=np.float32)
    return np.asarray([float(args.cruise_speed), 0.0], dtype=np.float32)


if __name__ == '__main__':
    main()
