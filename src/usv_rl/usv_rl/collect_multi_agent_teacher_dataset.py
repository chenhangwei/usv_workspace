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

import os
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from .multi_agent_env import MultiAgentEnv, MultiAgentEnvConfig
from .config import ActionBounds
from .multi_agent_teacher import MultiAgentColregsTeacher
from .runtime_collection_profile import collection_config_from_metadata, load_runtime_metadata


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
    p.add_argument('--teacher-turn-governor', action='store_true',
                   help='Teacher throttle follows the deployment turn-governor law for large heading errors.')
    p.add_argument('--teacher-cte-gain', type=float, default=0.45,
                   help='Teacher open-water CTE gain (rad/s per m).')
    p.add_argument('--teacher-cte-term-limit', type=float, default=0.0,
                   help='Clip of the teacher CTE term in rad/s; zero disables.')
    p.add_argument('--teacher-heading-gain', type=float, default=1.25)
    p.add_argument('--teacher-governor-floor', type=float, default=0.45)
    p.add_argument('--teacher-governor-power', type=float, default=1.0)
    p.add_argument('--teacher-brake-distance', type=float, default=1.3)
    p.add_argument('--teacher-brake-slowdown', type=float, default=0.22)
    p.add_argument('--teacher-giveway-distance', type=float, default=0.0)
    p.add_argument('--teacher-giveway-tcpa', type=float, default=15.0)
    p.add_argument('--teacher-giveway-dcpa', type=float, default=2.0)
    p.add_argument('--teacher-giveway-smooth', action='store_true')
    p.add_argument('--teacher-duck-offset', type=float, default=0.0)
    p.add_argument('--teacher-duck-slow', type=float, default=0.25)
    p.add_argument('--teacher-duck-smooth', action='store_true')
    p.add_argument('--teacher-duck-release', type=float, default=8.0)
    p.add_argument('--teacher-convoy-distance', type=float, default=0.0)
    p.add_argument('--teacher-turn-omega-limit', type=float, default=0.0)
    p.add_argument('--teacher-avoid-heading', type=float, default=0.0)
    p.add_argument('--teacher-avoid-clearance', type=float, default=0.0)
    p.add_argument('--teacher-tandem-clean', action='store_true')
    p.add_argument('--teacher-static-hull', action='store_true')
    p.add_argument('--teacher-park-slow', type=float, default=0.0)
    p.add_argument('--teacher-omega-deadband-comp', type=float, default=0.0)
    p.add_argument('--teacher-close-range-distance', type=float, default=3.2,
                   help='Teacher close-range avoidance ramp start (m).')
    p.add_argument('--teacher-escape-distance', type=float, default=1.35,
                   help='Teacher hard escape-turn distance (m).')
    p.add_argument('--teacher-dense-closing-only', type=int, default=0,
                   help='1 = dense-traffic fallback counts only closing neighbours.')
    p.add_argument('--teacher-engaged-cte-gain', type=float, default=0.0,
                   help='Route pull-back gain while a threat is engaged.')
    p.add_argument('--teacher-engaged-cte-fade', type=float, default=0.0,
                   help='When > 0, engaged route pull-back is zero below this nearest distance (m) and full 1.5 m beyond.')
    p.add_argument('--teacher-threat-latch', type=int, default=1,
                   help='0 = memoryless threat gate (labels depend only on the current observation).')
    p.add_argument('--teacher-conflict-gate', type=int, default=1,
                   help='1 = v3 TCPA/DCPA-gated teacher (open water drives straight); 0 = legacy v2 full-domain bias.')
    p.add_argument('--save-every-episodes', type=int, default=10,
                   help='Write the partial dataset to --output every N episodes (guards against VM restarts).')
    p.add_argument('--student', default=None,
                   help='DAgger: MAPPO/BC actor checkpoint (.pt) that drives the rollout; labels stay teacher actions.')
    p.add_argument('--student-mix', type=float, default=1.0,
                   help='DAgger: per-step probability that the executed action comes from the student (1.0 = pure student rollout).')
    p.add_argument('--exec-omega-noise-std', type=float, default=0.0,
                   help='DART: gaussian noise added to the EXECUTED omega (labels stay clean) so the teacher visits off-route states.')
    p.add_argument('--exec-omega-bias-std', type=float, default=0.0,
                   help='DART: per-agent omega bias resampled every --exec-bias-hold steps (builds up CTE/heading offsets).')
    p.add_argument('--exec-bias-hold', type=int, default=40)
    p.add_argument('--exec-speed-scale-min', type=float, default=1.0,
                   help='DART: minimum sampled execution-only throttle cap; teacher labels stay unchanged.')
    p.add_argument('--exec-speed-scale-max', type=float, default=1.0,
                   help='DART: maximum sampled execution-only throttle cap (speed_scale mode only).')
    p.add_argument('--exec-speed-hold', type=int, default=40,
                   help='Number of steps to hold each execution throttle cap.')
    p.add_argument('--solo-recovery-cte', type=float, default=0.0,
                   help='Paired true-solo initial lateral offset magnitude in meters; zero disables.')
    p.add_argument('--solo-recovery-heading-deg', type=float, default=15.0,
                   help='Paired initial goal heading error magnitude, with the same sign as CTE.')
    p.add_argument('--solo-recovery-progress', type=float, default=0.0,
                   help='Initial along-route fraction in [0, 1), preserving the original reference route.')
    p.add_argument('--validation-every-blocks', type=int, default=0,
                   help='Hold out every Nth full scenario cycle of recovery pairs; zero disables.')
    p.add_argument('--runtime-policy-trace',
                   help='Use effective projection and speed-expansion parameters from a policy trace header (solo only).')
    p.add_argument('--observation-frame', choices=['legacy', 'route_canonical_v1'], default='legacy',
                   help='Policy-facing observation frame; route_canonical_v1 requires --num-agents 1.')
    p.add_argument('--goal-distance', type=float, default=12.0, help='Route leg length for generated scenarios.')
    p.add_argument('--waypoints-per-episode', type=int, default=1,
                   help='Chain this many waypoints per episode (in-motion waypoint switches when > 1).')
    p.add_argument('--waypoint-turn-max-deg', type=float, default=math.degrees(2.9),
                   help='Maximum start turn of the waypoint_turn scenario (deg, <= 180).')
    p.add_argument('--waypoint-switch-max-deg', type=float, default=150.0,
                   help='Maximum heading change at in-episode waypoint switches (deg, <= 180).')
    p.add_argument('--sitl-dynamics', action='store_true',
                   help='SITL-fitted vehicle response (2026-10-06): yaw gain .6->.3 with speed, 0.22 rad/s cap, '
                        'tau_angular .75 s, tau_linear 1.5 s; overrides --sim-tau-*.')
    p.add_argument('--sitl-dynamics-v2', action='store_true',
                   help='SITL response after ATC_STR_RAT_FF .8 tuning (2026-10-08): 0.15 rad/s deadband, '
                        'unit gain, 0.5 rad/s cap, tau_angular .5 s, tau_linear 1.5 s.')
    p.add_argument('--angular-authority-floor', type=float, default=0.35,
                   help='Projection yaw-rate authority floor; 1.0 keeps full omega at small heading errors.')
    p.add_argument('--hold-arrived-agents', action='store_true',
                   help='Hold an arrived agent at zero command, as deployment stops on navigation completion.')
    args = p.parse_args()
    if not (0.0 <= args.action_speed_scale_min <= args.exec_speed_scale_min
            <= args.exec_speed_scale_max <= 1.0):
        p.error('execution speed caps must satisfy action-speed-scale-min <= min <= max <= 1')
    if args.exec_speed_hold < 1:
        p.error('--exec-speed-hold must be positive')
    if args.exec_speed_scale_min < 1.0 and args.action_mode != 'speed_scale':
        p.error('execution speed caps require --action-mode speed_scale')
    if not math.isfinite(args.solo_recovery_cte) or args.solo_recovery_cte < 0:
        p.error('--solo-recovery-cte must be finite and nonnegative')
    if not 0 <= args.solo_recovery_heading_deg < 90:
        p.error('--solo-recovery-heading-deg must be in [0, 90)')
    if not 0 <= args.solo_recovery_progress < 1:
        p.error('--solo-recovery-progress must be in [0, 1)')
    if args.solo_recovery_progress > 0 and args.solo_recovery_cte <= 0:
        p.error('--solo-recovery-progress requires paired solo recovery')
    if args.solo_recovery_cte > 0 and (args.num_agents != 1 or args.episodes < 2 or args.episodes % 2):
        p.error('paired solo recovery requires --num-agents 1 and a positive even episode count')
    if args.validation_every_blocks != 0 and args.validation_every_blocks < 2:
        p.error('--validation-every-blocks must be zero or at least two')
    if args.validation_every_blocks and args.solo_recovery_cte <= 0:
        p.error('blocked validation requires paired solo recovery')
    if args.observation_frame != 'legacy' and args.num_agents != 1:
        p.error('--observation-frame route_canonical_v1 requires --num-agents 1')
    if not math.isfinite(args.goal_distance) or args.goal_distance <= 0:
        p.error('--goal-distance must be finite and positive')
    if args.waypoints_per_episode < 1:
        p.error('--waypoints-per-episode must be positive')
    if not (math.isfinite(args.teacher_cte_gain) and args.teacher_cte_gain >= 0
            and math.isfinite(args.teacher_cte_term_limit) and args.teacher_cte_term_limit >= 0):
        p.error('--teacher-cte-gain and --teacher-cte-term-limit must be finite and nonnegative')
    for name in ('waypoint_turn_max_deg', 'waypoint_switch_max_deg'):
        if not 46.0 <= getattr(args, name) <= 180.0:
            p.error(f'--{name.replace("_", "-")} must be in [46, 180]')
    return args


def _episode_spec(args, scenarios, episode):
    paired = args.solo_recovery_cte > 0
    group = episode // 2 if paired else episode
    side = (-1 if episode % 2 == 0 else 1) if paired else 0
    scenario = scenarios[group % len(scenarios)]
    options = {'scenario_kind': scenario}
    if paired:
        options.update(solo_recovery_cte=side * args.solo_recovery_cte,
                       solo_recovery_heading_error=side * math.radians(args.solo_recovery_heading_deg),
                       solo_recovery_progress=args.solo_recovery_progress)
    block = group // len(scenarios)
    validation = (args.validation_every_blocks > 0
                  and (block + 1) % args.validation_every_blocks == 0)
    return args.seed + group, scenario, options, side, group, 'validation' if validation else 'train'


def _cap_execution_speed(action, scale_cap):
    executed = np.array(action, dtype=np.float32, copy=True)
    executed[0] = min(float(executed[0]), float(scale_cap))
    return executed


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
        sim_tau_linear=1.5 if (args.sitl_dynamics or args.sitl_dynamics_v2) else float(args.sim_tau_linear),
        sim_tau_angular=float(os.environ.get('SIM_TAU_ANG', '0.5')) if args.sitl_dynamics_v2 else (0.75 if args.sitl_dynamics else float(args.sim_tau_angular)),
        sim_yaw_gain_low=1.0 if args.sitl_dynamics_v2 else (0.55 if args.sitl_dynamics else 1.0),
        sim_yaw_gain_high=1.0 if args.sitl_dynamics_v2 else (0.25 if args.sitl_dynamics else 1.0),
        sim_yaw_rate_limit=0.5 if args.sitl_dynamics_v2 else (0.35 if args.sitl_dynamics else 0.0),
        sim_yaw_rate_limit_high=0.5 if args.sitl_dynamics_v2 else (0.09 if args.sitl_dynamics else 0.0),
        sim_yaw_deadband=0.15 if args.sitl_dynamics_v2 else 0.0,
        angular_authority_floor=float(args.angular_authority_floor),
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
        observation_frame=args.observation_frame,
        goal_distance=float(args.goal_distance),
        max_waypoints_per_episode=int(args.waypoints_per_episode),
        waypoint_turn_max_angle=math.radians(args.waypoint_turn_max_deg),
        waypoint_switch_max_angle=math.radians(args.waypoint_switch_max_deg),
        hold_arrived_agents=bool(args.hold_arrived_agents),
    )
    if args.runtime_policy_trace:
        metadata = load_runtime_metadata(args.runtime_policy_trace)
        config = collection_config_from_metadata(config, metadata)
        args.runtime_collection_profile_json = json.dumps({
            'scope': 'projection_and_speed_expansion_only',
            'source_trace': str(args.runtime_policy_trace), 'model_name': metadata.get('model_name'),
            'projection': metadata['projection'], 'action_contract': metadata['action_contract'],
        }, allow_nan=False)
    return MultiAgentEnv(config)


def main():
    args = parse_args()
    # The in-process ROS executor otherwise starves torch inference of the GIL (~1 s per 5-agent step).
    sys.setswitchinterval(1e-4)
    scenarios = tuple(args.scenarios) if args.scenarios else _DEFAULT_SCENARIOS

    env = _build_env(args, scenarios)
    teacher = MultiAgentColregsTeacher(
        action_mode=args.action_mode,
        cruise_speed=float(args.cruise_speed),
        max_angular_velocity=float(args.max_angular_velocity),
        action_speed_scale_min=float(args.action_speed_scale_min),
        linear_speed_limit=float(args.cruise_speed),
        conflict_gate=bool(args.teacher_conflict_gate),
        turn_speed_governor=bool(args.teacher_turn_governor),
        cte_p_gain=float(args.teacher_cte_gain),
        cte_term_limit=float(args.teacher_cte_term_limit) or None,
        threat_latch=bool(args.teacher_threat_latch),
        close_range_distance=float(args.teacher_close_range_distance),
        escape_distance=float(args.teacher_escape_distance),
        dense_closing_only=bool(args.teacher_dense_closing_only),
        engaged_cte_p_gain=float(args.teacher_engaged_cte_gain),
        engaged_cte_fade=float(args.teacher_engaged_cte_fade),
        heading_p_gain=float(args.teacher_heading_gain),
        governor_floor=float(args.teacher_governor_floor),
        governor_power=float(args.teacher_governor_power),
        brake_distance=float(args.teacher_brake_distance),
        brake_slowdown=float(args.teacher_brake_slowdown),
        giveway_distance=float(args.teacher_giveway_distance),
        giveway_tcpa=float(args.teacher_giveway_tcpa),
        giveway_dcpa=float(args.teacher_giveway_dcpa),
        giveway_smooth=bool(args.teacher_giveway_smooth),
        duck_offset=float(args.teacher_duck_offset),
        duck_slow=float(args.teacher_duck_slow),
        duck_smooth=bool(args.teacher_duck_smooth),
        duck_release=float(args.teacher_duck_release),
        convoy_distance=float(args.teacher_convoy_distance),
        turn_omega_limit=float(args.teacher_turn_omega_limit),
        avoid_heading=float(args.teacher_avoid_heading),
        avoid_clearance=float(args.teacher_avoid_clearance),
        tandem_clean=bool(args.teacher_tandem_clean),
        static_hull=bool(args.teacher_static_hull),
        park_slow=float(args.teacher_park_slow),
        omega_deadband_comp=float(args.teacher_omega_deadband_comp),
    )

    observations_out: list[np.ndarray] = []
    actions_out: list[np.ndarray] = []
    scenario_out: list[str] = []
    engaged_out: list[bool] = []
    sample_metadata = {key: [] for key in (
        'episode_ids', 'episode_seeds', 'pair_ids', 'step_ids', 'agent_ids',
        'recovery_sides', 'split', 'executed_actions',
        'monotonic_timestamps',
    )}

    student = None
    if args.student:
        from .evaluate_mappo_policy import _load_mappo_checkpoint
        _, student = _load_mappo_checkpoint(args.student, 'cpu')
        print(f'[collect] DAgger rollout: student={args.student} mix={args.student_mix}', flush=True)
    rng = np.random.default_rng(args.seed)

    try:
        for episode in range(args.episodes):
            episode_seed, scenario_name, reset_options, side, group, split = _episode_spec(args, scenarios, episode)
            if args.solo_recovery_cte > 0:
                rng = np.random.default_rng(episode_seed)
            obs_map, info = env.reset(
                seed=episode_seed,
                options=reset_options,
            )
            teacher.reset()
            exec_bias: dict[str, float] = {}
            exec_speed_caps: dict[str, float] = {}
            active_scenario = info.get('scenario', scenario_name)
            episode_min_sep = float('inf')
            for step in range(args.steps_per_episode):
                action_map = {}
                for agent_id in env.agent_ids:
                    struct = env._latest_observations.get(agent_id)
                    if struct is None:
                        # Fallback: no structured obs -> straight, full speed.
                        action_map[agent_id] = teacher.act_default() if hasattr(teacher, 'act_default') else _idle_action(args)
                        continue
                    action = teacher.act(struct)
                    if args.hold_arrived_agents and (agent_id in env._held_agents
                                                     or struct.distance_to_goal <= env.config.goal_tolerance):
                        # Deployment never queries the policy after arrival; the env holds this boat.
                        action_map[agent_id] = action
                        continue
                    executed = action
                    if student is not None and rng.random() < args.student_mix:
                        executed = np.asarray(student.predict(np.asarray(obs_map[agent_id], dtype=np.float32)), dtype=np.float32)
                    if args.exec_omega_noise_std > 0.0 or args.exec_omega_bias_std > 0.0:
                        if step % max(1, args.exec_bias_hold) == 0 or agent_id not in exec_bias:
                            exec_bias[agent_id] = float(rng.normal(0.0, args.exec_omega_bias_std)) if args.exec_omega_bias_std > 0.0 else 0.0
                        executed = np.array(executed, dtype=np.float32, copy=True)
                        executed[-1] = float(np.clip(executed[-1] + exec_bias[agent_id] + rng.normal(0.0, args.exec_omega_noise_std),
                                                     -args.max_angular_velocity, args.max_angular_velocity))
                    if args.exec_speed_scale_min < 1.0:
                        if step % args.exec_speed_hold == 0 or agent_id not in exec_speed_caps:
                            exec_speed_caps[agent_id] = float(rng.uniform(
                                args.exec_speed_scale_min, args.exec_speed_scale_max,
                            ))
                        executed = _cap_execution_speed(executed, exec_speed_caps[agent_id])
                    action_map[agent_id] = executed
                    observations_out.append(np.asarray(obs_map[agent_id], dtype=np.float32))
                    actions_out.append(np.asarray(action, dtype=np.float32))
                    scenario_out.append(active_scenario)
                    engaged_out.append(bool(teacher.last_engaged))
                    for key, value in (
                        ('episode_ids', episode), ('episode_seeds', episode_seed),
                        ('pair_ids', group), ('step_ids', step), ('agent_ids', agent_id),
                        ('recovery_sides', side), ('split', split),
                        ('executed_actions', np.asarray(executed, dtype=np.float32).copy()),
                        ('monotonic_timestamps', time.monotonic()),
                    ):
                        sample_metadata[key].append(value)
                obs_map, _, terminated_dict, truncated_dict, info = env.step(action_map)
                pair_min = info.get('pairwise_min_separation')
                if pair_min is not None and np.isfinite(pair_min):
                    episode_min_sep = min(episode_min_sep, float(pair_min))
                active_scenario = info.get('scenario', active_scenario)
                done = all(terminated_dict.get(a, False) or truncated_dict.get(a, False)
                           for a in env.agent_ids)
                if done:
                    break
            print(f'[collect] episode {episode + 1}/{args.episodes} scenario={scenario_name} '
                  f'min_sep={episode_min_sep if episode_min_sep != float("inf") else -1.0:.2f} '
                  f'cumulative_samples={len(observations_out)}', flush=True)
            if args.save_every_episodes > 0 and (episode + 1) % args.save_every_episodes == 0:
                _save_dataset(args, observations_out, actions_out, scenario_out, engaged_out, sample_metadata)
                print(f'[collect] checkpointed {len(observations_out)} samples after episode {episode + 1}', flush=True)
    finally:
        env.close()

    _save_dataset(args, observations_out, actions_out, scenario_out, engaged_out, sample_metadata)
    output_path = Path(args.output)
    actions_arr = np.asarray(actions_out, dtype=np.float32)
    engaged_arr = np.asarray(engaged_out, dtype=bool)
    if actions_arr.ndim == 2 and actions_arr.shape[1] >= 2 and len(engaged_arr):
        omega = actions_arr[:, -1]
        free = ~engaged_arr
        print(f'[collect][stats] engaged_fraction={engaged_arr.mean():.3f}', flush=True)
        if free.any():
            print(f'[collect][stats] open_water omega median={np.median(omega[free]):.4f} '
                  f'p90(|omega|)={np.percentile(np.abs(omega[free]), 90):.4f} '
                  f'scale_min={actions_arr[free, 0].min():.3f}', flush=True)
        if engaged_arr.any():
            print(f'[collect][stats] engaged omega median={np.median(omega[engaged_arr]):.4f} '
                  f'right_turn_fraction={(omega[engaged_arr] < -0.02).mean():.3f}', flush=True)
    print(f'Saved {len(observations_out)} teacher samples to {output_path}', flush=True)


def _save_dataset(args, observations_out, actions_out, scenario_out, engaged_out, sample_metadata=None):
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        observations=np.asarray(observations_out, dtype=np.float32),
        actions=np.asarray(actions_out, dtype=np.float32),
        scenario_names=np.asarray(scenario_out),
        engaged=np.asarray(engaged_out, dtype=bool),
        action_mode=np.asarray(args.action_mode),
        action_speed_scale_min=np.asarray(args.action_speed_scale_min, dtype=np.float32),
        max_neighbors=np.asarray(args.max_neighbors, dtype=np.int64),
        num_agents=np.asarray(args.num_agents, dtype=np.int64),
        exec_speed_scale_min=np.asarray(args.exec_speed_scale_min, dtype=np.float32),
        exec_speed_scale_max=np.asarray(args.exec_speed_scale_max, dtype=np.float32),
        exec_speed_hold=np.asarray(args.exec_speed_hold, dtype=np.int64),
        solo_recovery_cte=np.asarray(args.solo_recovery_cte, dtype=np.float32),
        solo_recovery_heading_deg=np.asarray(args.solo_recovery_heading_deg, dtype=np.float32),
        solo_recovery_progress=np.asarray(args.solo_recovery_progress, dtype=np.float32),
        observation_frame=np.asarray(args.observation_frame),
        goal_distance=np.asarray(args.goal_distance, dtype=np.float32),
        waypoints_per_episode=np.asarray(args.waypoints_per_episode, dtype=np.int64),
        waypoint_turn_max_deg=np.asarray(args.waypoint_turn_max_deg, dtype=np.float32),
        waypoint_switch_max_deg=np.asarray(args.waypoint_switch_max_deg, dtype=np.float32),
        validation_every_blocks=np.asarray(args.validation_every_blocks, dtype=np.int64),
        teacher_turn_governor=np.asarray(bool(args.teacher_turn_governor)),
        hold_arrived_agents=np.asarray(bool(args.hold_arrived_agents)),
        sitl_dynamics=np.asarray(bool(args.sitl_dynamics)),
        sitl_dynamics_v2=np.asarray(bool(args.sitl_dynamics_v2)),
        teacher_threat_latch=np.asarray(bool(args.teacher_threat_latch)),
        teacher_close_range_distance=np.asarray(args.teacher_close_range_distance, dtype=np.float32),
        teacher_escape_distance=np.asarray(args.teacher_escape_distance, dtype=np.float32),
        teacher_dense_closing_only=np.asarray(bool(args.teacher_dense_closing_only)),
        teacher_engaged_cte_gain=np.asarray(args.teacher_engaged_cte_gain, dtype=np.float32),
        teacher_engaged_cte_fade=np.asarray(args.teacher_engaged_cte_fade, dtype=np.float32),
        teacher_cte_gain=np.asarray(args.teacher_cte_gain, dtype=np.float32),
        teacher_cte_term_limit=np.asarray(args.teacher_cte_term_limit, dtype=np.float32),
        teacher_heading_gain=np.asarray(args.teacher_heading_gain, dtype=np.float32),
        teacher_governor_floor=np.asarray(args.teacher_governor_floor, dtype=np.float32),
        teacher_governor_power=np.asarray(args.teacher_governor_power, dtype=np.float32),
        teacher_giveway_distance=np.asarray(args.teacher_giveway_distance, dtype=np.float32),
        runtime_collection_profile_json=np.asarray(getattr(args, 'runtime_collection_profile_json', '')),
        **{key: np.asarray(value) for key, value in (sample_metadata or {}).items()},
    )


def _idle_action(args):
    if args.action_mode == 'angular_only':
        return np.asarray([0.0], dtype=np.float32)
    if args.action_mode == 'speed_scale':
        return np.asarray([1.0, 0.0], dtype=np.float32)
    return np.asarray([float(args.cruise_speed), 0.0], dtype=np.float32)


if __name__ == '__main__':
    main()
