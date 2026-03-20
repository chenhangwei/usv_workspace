import threading
import time
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from .action_projection import project_residual_action
from .config import ActionBounds, RewardConfig
from .multi_agent_bridge import MultiAgentTrainingBridge
from .multi_agent_scenarios import MultiAgentScenarioFactory
from .multi_agent_types import AgentLocalObservation, FleetGlobalState, MultiAgentRewardBreakdown
from .multi_usv_sim import MultiUsvSimNode
from usv_control.velocity_controller_node import VelocityControllerNode

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    gym = SimpleNamespace(Env=object)
    spaces = None


@dataclass
class MultiAgentEnvConfig:
    agent_namespaces: tuple[str, ...] = ('usv_01', 'usv_02', 'usv_03')
    launch_sitl: bool = True
    enable_rl_backend: bool = True
    action_mode: str = 'full'
    max_neighbors: int = 4
    max_agents: int = 3
    cruise_speed: float = 0.4
    max_angular_velocity: float = 0.4
    control_dt: float = 0.2
    state_timeout: float = 10.0
    ready_timeout: float = 45.0
    episode_timeout: float = 45.0
    no_progress_timeout: float = 10.0
    min_progress_delta: float = 0.3
    goal_distance: float = 12.0
    goal_tolerance: float = 0.8
    collision_distance: float = 0.75
    near_miss_distance: float = 1.2
    neighbor_publish_rate: float = 10.0
    scenario_neighbor_speed: float = 0.4
    default_scenarios: tuple[str, ...] = field(default_factory=MultiAgentScenarioFactory.available)
    action_bounds: ActionBounds = field(default_factory=ActionBounds)
    reward: RewardConfig = field(default_factory=RewardConfig)
    goal_proximity_reward_weight: float = 0.8
    goal_proximity_relief_distance: float = 2.0
    goal_proximity_heading_relief: float = 0.55
    goal_proximity_smoothness_relief: float = 0.70
    goal_proximity_conflict_relief: float = 0.45
    team_reward_weight: float = 0.30
    coordination_reward_weight: float = 0.20
    team_progress_weight: float = 1.20
    team_goal_proximity_weight: float = 0.0
    team_regression_penalty_weight: float = 0.0
    team_dispersion_penalty_weight: float = 0.0
    team_dispersion_margin: float = 0.0
    team_completion_bonus: float = 18.0
    deadlock_penalty_weight: float = 4.0


class MultiAgentEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, config: Optional[MultiAgentEnvConfig] = None):
        if not rclpy.ok():
            rclpy.init()

        self.config = config or MultiAgentEnvConfig()
        self._agent_ids = tuple(self.config.agent_namespaces)
        self._episode_index = 0
        self._rng = np.random.default_rng()
        self._executor = MultiThreadedExecutor()
        self._spin_thread: Optional[threading.Thread] = None
        self._bridge: Optional[MultiAgentTrainingBridge] = None
        self._sim_node: Optional[MultiUsvSimNode] = None
        self._controllers: Dict[str, VelocityControllerNode] = {}
        self._scenario = None
        self._episode_start = 0.0
        self._last_team_progress_time = 0.0
        self._previous_distances: Dict[str, float] = {}
        self._previous_actions: Dict[str, np.ndarray] = {}
        self._previous_forward_speeds: Dict[str, float] = {}
        self._latest_observations: Dict[str, AgentLocalObservation] = {}
        self._best_team_mean_distance = float('inf')
        self._initial_team_mean_separation = float('inf')
        self._initial_team_mean_goal_distance = float('inf')

        self.action_dim = 1 if self.config.action_mode == 'angular_only' else 2
        self.action_low = np.asarray(
            [-self.config.action_bounds.angular_delta] if self.action_dim == 1 else [
                -self.config.action_bounds.linear_delta,
                -self.config.action_bounds.angular_delta,
            ],
            dtype=np.float32,
        )
        self.action_high = np.asarray(
            [self.config.action_bounds.angular_delta] if self.action_dim == 1 else [
                self.config.action_bounds.linear_delta,
                self.config.action_bounds.angular_delta,
            ],
            dtype=np.float32,
        )

        if spaces is not None:
            self.single_agent_action_space = spaces.Box(low=self.action_low, high=self.action_high, dtype=np.float32)
            self.single_agent_observation_space = spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(self.local_observation_size,),
                dtype=np.float32,
            )

        self._ensure_runtime()

    @property
    def local_observation_size(self) -> int:
        return AgentLocalObservation.vector_size(self.config.max_neighbors)

    @property
    def global_state_size(self) -> int:
        return FleetGlobalState.vector_size(self.config.max_agents, self.config.max_neighbors)

    @property
    def agent_ids(self) -> tuple[str, ...]:
        return self._agent_ids

    def _ensure_runtime(self):
        if self._sim_node is None:
            self._sim_node = MultiUsvSimNode(self._agent_ids)
            self._executor.add_node(self._sim_node)

        if self._bridge is None:
            self._bridge = MultiAgentTrainingBridge(self._agent_ids, self.config.neighbor_publish_rate)
            self._executor.add_node(self._bridge)

        if not self._controllers:
            for namespace in self._agent_ids:
                controller = VelocityControllerNode(
                    namespace=f'/{namespace}',
                    parameter_overrides=[
                        Parameter('cruise_speed', value=self.config.cruise_speed),
                        Parameter('max_angular_velocity', value=self.config.max_angular_velocity),
                        Parameter('apf_enabled', value=True),
                        Parameter('apf_orca_enabled', value=False),
                        Parameter('require_guided_mode', value=True),
                        Parameter('require_armed', value=True),
                    ],
                )
                self._controllers[namespace] = controller
                self._executor.add_node(controller)

        if self._spin_thread is None:
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()

        if not self._bridge.wait_for_controllers(timeout=self.config.ready_timeout):
            raise RuntimeError('Timed out waiting for multi-agent velocity controller parameter services.')
        if not self._bridge.wait_for_agents(timeout=self.config.ready_timeout):
            raise RuntimeError('Timed out waiting for multi-agent pose/velocity topics.')

    def zero_policy_action(self) -> np.ndarray:
        return np.zeros(self.action_dim, dtype=np.float32)

    def project_policy_action(self, agent_id: str, action) -> np.ndarray:
        observation = self._latest_observations.get(agent_id)
        raw_linear_x = None if observation is None else observation.raw_linear_x
        return project_residual_action(
            action,
            action_mode=self.config.action_mode,
            linear_delta_limit=self.config.action_bounds.linear_delta,
            angular_delta_limit=self.config.action_bounds.angular_delta,
            raw_linear_x=raw_linear_x,
            forward_only=True,
        )

    def expand_policy_action(self, agent_id: str, action) -> tuple[float, float]:
        projected = self.project_policy_action(agent_id, action)
        if self.config.action_mode == 'angular_only':
            return 0.0, float(projected[0])
        return float(projected[0]), float(projected[1])

    def _wait_for_local_observations(self, timeout: Optional[float] = None) -> Dict[str, AgentLocalObservation]:
        deadline = time.monotonic() + (timeout or self.config.state_timeout)
        while time.monotonic() < deadline:
            observations = self._bridge.get_local_observations(self.config.max_neighbors)
            if observations is not None:
                return observations
            time.sleep(0.05)
        raise RuntimeError('Timed out waiting for multi-agent observations.')

    def _wait_for_global_state(self, timeout: Optional[float] = None) -> FleetGlobalState:
        deadline = time.monotonic() + (timeout or self.config.state_timeout)
        while time.monotonic() < deadline:
            state = self._bridge.get_global_state(self.config.max_agents, self.config.max_neighbors)
            if state is not None:
                return state
            time.sleep(0.05)
        raise RuntimeError('Timed out waiting for global fleet state.')

    def _scenario_initial_team_mean_goal_distance(self) -> float:
        if self._scenario is None:
            return float('inf')

        distances = []
        for agent_id in self._agent_ids:
            spawn = self._scenario.agent_spawns.get(agent_id)
            goal = self._scenario.agent_goals.get(agent_id)
            if spawn is None or goal is None:
                continue
            distances.append(float(np.hypot(goal.x - spawn.x, goal.y - spawn.y)))

        if not distances:
            return float('inf')
        return float(np.mean(distances))

    def _compute_conflict_risk(self, observation: AgentLocalObservation) -> float:
        max_risk = 0.0
        conflict_distance = max(
            self.config.reward.anticipation_distance,
            self.config.reward.conflict_distance,
            self.config.collision_distance + 1e-3,
        )
        for neighbor in observation.neighbors:
            if neighbor.distance <= 1e-3 or neighbor.distance >= conflict_distance:
                continue

            range_rate = -(
                (neighbor.rel_x * neighbor.rel_vx) +
                (neighbor.rel_y * neighbor.rel_vy)
            ) / max(neighbor.distance, 1e-3)
            if range_rate <= 0.0:
                continue

            forward_factor = max(0.0, float(np.cos(neighbor.bearing)))
            if forward_factor <= 0.0:
                continue

            proximity = (conflict_distance - neighbor.distance) / conflict_distance
            neighbor_risk = proximity * range_rate * forward_factor
            max_risk = max(max_risk, neighbor_risk)

        return max_risk

    def _compute_head_on_guidance_reward(self, observation: AgentLocalObservation, previous_forward_speed: float) -> float:
        scenario_name = getattr(self._scenario, 'name', '') if self._scenario is not None else ''
        if 'head_on' not in scenario_name:
            return 0.0

        guidance_distance = max(
            self.config.reward.head_on_guidance_distance,
            self.config.reward.anticipation_distance,
            self.config.collision_distance + 1e-3,
        )
        target_offset = max(0.2, self.config.reward.head_on_target_starboard_offset)
        own_speed = max(0.0, float(observation.speed), float(observation.final_linear_x), float(observation.raw_linear_x))

        best_neighbor = None
        best_score = -1.0
        for neighbor in observation.neighbors:
            if neighbor.distance <= 1e-3 or neighbor.distance > guidance_distance:
                continue
            body_x = float(neighbor.rel_x)
            body_y = float(neighbor.rel_y)
            body_vx = float(neighbor.rel_vx)
            body_vy = float(neighbor.rel_vy)
            if body_x <= 0.0:
                continue

            range_rate = -(
                (body_x * body_vx) + (body_y * body_vy)
            ) / max(neighbor.distance, 1e-3)
            if range_rate <= 0.0:
                continue

            lateral_tolerance = max(1.3, 0.28 * neighbor.distance)
            if abs(body_y) > lateral_tolerance:
                continue

            neighbor_forward_speed = own_speed + body_vx
            same_lane_ahead = body_x > 0.8 and abs(body_y) < 1.5
            overtaking_target = (
                same_lane_ahead
                and own_speed > 0.22
                and body_vx < -0.03
                and neighbor_forward_speed > 0.05
                and neighbor_forward_speed < own_speed - 0.03
            )
            opposing_flow = neighbor_forward_speed < 0.05
            if overtaking_target or not opposing_flow:
                continue

            proximity = (guidance_distance - neighbor.distance) / guidance_distance
            centerline_exposure = max(0.0, 1.0 - min(1.0, abs(body_y) / max(0.8, target_offset)))
            score = 0.65 * proximity + 0.35 * centerline_exposure
            if score > best_score:
                best_score = score
                best_neighbor = neighbor

        if best_neighbor is None:
            return 0.0

        proximity = max(0.0, min(1.0, (guidance_distance - best_neighbor.distance) / guidance_distance))
        starboard_offset = max(0.0, -best_neighbor.rel_y)
        corridor_progress = max(0.0, min(1.0, starboard_offset / target_offset))
        centerline_penalty = max(0.0, 1.0 - corridor_progress)
        desired_starboard_turn = 0.10 + 0.16 * proximity
        actual_starboard_turn = max(0.0, -observation.final_angular_z)
        turn_progress = max(0.0, min(1.0, actual_starboard_turn / max(desired_starboard_turn, 1e-3)))
        desired_forward_speed = min(
            max(self.config.reward.desired_conflict_speed, 0.0),
            max(observation.raw_linear_x, self.config.reward.desired_conflict_speed),
        )
        current_forward_speed = max(0.0, observation.final_linear_x)
        forward_progress = max(0.0, min(1.0, current_forward_speed / max(desired_forward_speed, 1e-3)))
        speed_drop = max(0.0, previous_forward_speed - current_forward_speed)
        speed_drop_ratio = max(0.0, min(1.0, speed_drop / max(desired_forward_speed, 1e-3)))

        return (
            self.config.reward.head_on_corridor_reward_weight * proximity * corridor_progress
            - self.config.reward.head_on_centerline_penalty_weight * proximity * centerline_penalty
            + self.config.reward.head_on_turn_reward_weight * proximity * turn_progress
            + self.config.reward.head_on_forward_reward_weight * proximity * forward_progress
            - self.config.reward.head_on_speed_drop_penalty_weight * proximity * speed_drop_ratio
        )

    def _compute_crossing_overtaking_guidance_reward(self, observation: AgentLocalObservation) -> float:
        lookahead_distance = max(
            self.config.reward.anticipation_distance,
            self.config.reward.conflict_distance,
            self.config.collision_distance + 1e-3,
        )
        own_speed = max(0.0, float(observation.speed), float(observation.final_linear_x), float(observation.raw_linear_x))
        actual_starboard_turn = max(0.0, -observation.final_angular_z)
        actual_port_turn = max(0.0, observation.final_angular_z)
        current_forward_speed = max(0.0, observation.final_linear_x)
        desired_forward_speed = min(
            max(self.config.reward.desired_conflict_speed, 0.0),
            max(observation.raw_linear_x, self.config.reward.desired_conflict_speed),
        )
        forward_progress = max(0.0, min(1.0, current_forward_speed / max(desired_forward_speed, 1e-3)))

        best_crossing_reward = 0.0
        best_crossing_forward_reward = 0.0
        best_overtaking_reward = 0.0
        best_overtaking_forward_reward = 0.0
        max_wrong_way_penalty = 0.0

        for neighbor in observation.neighbors:
            if neighbor.distance <= 1e-3 or neighbor.distance > lookahead_distance:
                continue

            body_x = float(neighbor.rel_x)
            body_y = float(neighbor.rel_y)
            if body_x <= 0.0:
                continue

            body_vx = float(neighbor.rel_vx)
            body_vy = float(neighbor.rel_vy)
            closing_speed = -((body_x * body_vx) + (body_y * body_vy)) / max(neighbor.distance, 1e-3)
            neighbor_forward_speed = own_speed + body_vx
            same_lane_ahead = body_x > 0.8 and abs(body_y) < 1.5
            overtaking_target = (
                same_lane_ahead
                and own_speed > 0.18
                and body_vx < -0.03
                and neighbor_forward_speed > 0.05
                and neighbor_forward_speed < own_speed - 0.02
            )
            starboard_crossing = body_y < -0.35 and closing_speed > -0.05

            proximity = max(0.0, min(1.0, (lookahead_distance - neighbor.distance) / lookahead_distance))
            closing_weight = max(0.0, min(1.0, (closing_speed + 0.15) / 0.9))
            desired_starboard_turn = 0.05 + 0.12 * proximity + 0.05 * closing_weight
            turn_progress = max(0.0, min(1.0, actual_starboard_turn / max(desired_starboard_turn, 1e-3)))
            wrong_way_progress = max(0.0, min(1.0, actual_port_turn / max(desired_starboard_turn, 1e-3)))

            if starboard_crossing:
                best_crossing_reward = max(best_crossing_reward, proximity * turn_progress)
                best_crossing_forward_reward = max(
                    best_crossing_forward_reward,
                    proximity * turn_progress * forward_progress,
                )
                max_wrong_way_penalty = max(max_wrong_way_penalty, proximity * wrong_way_progress)
                continue

            if overtaking_target:
                best_overtaking_reward = max(best_overtaking_reward, proximity * turn_progress)
                best_overtaking_forward_reward = max(
                    best_overtaking_forward_reward,
                    proximity * turn_progress * forward_progress,
                )
                max_wrong_way_penalty = max(max_wrong_way_penalty, proximity * wrong_way_progress)

        return (
            self.config.reward.crossing_starboard_turn_reward_weight * best_crossing_reward
            + self.config.reward.crossing_forward_reward_weight * best_crossing_forward_reward
            + self.config.reward.overtaking_starboard_turn_reward_weight * best_overtaking_reward
            + self.config.reward.overtaking_forward_reward_weight * best_overtaking_forward_reward
            - self.config.reward.colregs_port_turn_penalty_weight * max_wrong_way_penalty
        )

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        options = options or {}
        scenario_kind = options.get('scenario_kind')
        if not scenario_kind:
            scenario_kind = self.config.default_scenarios[self._episode_index % len(self.config.default_scenarios)]

        self._scenario = MultiAgentScenarioFactory.create(
            scenario_kind,
            self._agent_ids,
            goal_distance=self.config.goal_distance,
            neighbor_speed=self.config.scenario_neighbor_speed,
        )

        for controller in self._controllers.values():
            controller.reset_for_training_episode()

        self._bridge.reset_episode_state()
        self._sim_node.reset_agents(self._scenario)
        time.sleep(0.3)
        self._bridge.activate_scenario(self._scenario)
        self._bridge.set_rl_backend_enabled(self.config.enable_rl_backend)
        self._bridge.publish_goals(
            f'mappo_{self._episode_index:04d}',
            {namespace: (goal.x, goal.y) for namespace, goal in self._scenario.agent_goals.items()},
        )

        observations = self._wait_for_local_observations(timeout=self.config.state_timeout)
        global_state = self._wait_for_global_state(timeout=self.config.state_timeout)
        self._latest_observations = observations
        self._previous_distances = {
            namespace: observation.distance_to_goal for namespace, observation in observations.items()
        }
        self._previous_actions = {namespace: self.zero_policy_action() for namespace in self._agent_ids}
        self._previous_forward_speeds = {
            namespace: max(0.0, observation.final_linear_x) for namespace, observation in observations.items()
        }
        self._episode_start = time.monotonic()
        self._last_team_progress_time = self._episode_start
        self._best_team_mean_distance = global_state.team_mean_goal_distance
        self._initial_team_mean_separation = global_state.team_mean_separation
        self._initial_team_mean_goal_distance = self._scenario_initial_team_mean_goal_distance()
        self._episode_index += 1

        return (
            {namespace: observation.to_vector(self.config.max_neighbors) for namespace, observation in observations.items()},
            {
                'scenario': self._scenario.name,
                'global_state': global_state.to_vector(self.config.max_agents, self.config.max_neighbors),
            },
        )

    def _compute_reward(
        self,
        agent_id: str,
        observation: AgentLocalObservation,
        action: np.ndarray,
        global_state: FleetGlobalState,
        team_progress_delta: float,
        terminated: bool,
        truncated: bool,
        all_reached: bool,
    ) -> MultiAgentRewardBreakdown:
        progress_delta = self._previous_distances[agent_id] - observation.distance_to_goal
        progress = self.config.reward.progress_weight * progress_delta

        goal_proximity = 0.0
        relief_distance = max(self.config.goal_tolerance + 1e-3, self.config.goal_proximity_relief_distance)
        if observation.distance_to_goal < relief_distance:
            goal_proximity = max(
                0.0,
                (relief_distance - observation.distance_to_goal) / max(relief_distance - self.config.goal_tolerance, 1e-3),
            )
            goal_proximity = min(goal_proximity, 1.0)
        progress += self.config.goal_proximity_reward_weight * goal_proximity

        pair_min = global_state.team_min_separation
        if not np.isfinite(pair_min):
            pair_min = observation.min_neighbor_distance()

        safety = 0.0
        conflict_risk = self._compute_conflict_risk(observation)
        if pair_min < self.config.collision_distance:
            safety += self.config.reward.collision_penalty
        elif pair_min < self.config.near_miss_distance:
            safety -= self.config.reward.near_miss_weight * (self.config.near_miss_distance - pair_min)
        conflict_relief_scale = max(0.35, 1.0 - (self.config.goal_proximity_conflict_relief * goal_proximity))
        safety -= self.config.reward.conflict_risk_weight * conflict_risk * conflict_relief_scale

        conflict_level = min(conflict_risk, 1.0) * conflict_relief_scale
        current_forward_speed = max(0.0, observation.final_linear_x)
        desired_conflict_speed = min(
            max(self.config.reward.desired_conflict_speed, 0.0),
            max(observation.raw_linear_x, self.config.reward.desired_conflict_speed),
        )
        speed_deficit = max(0.0, desired_conflict_speed - current_forward_speed)
        speed_drop = max(0.0, self._previous_forward_speeds.get(agent_id, 0.0) - current_forward_speed)
        braking = -self.config.reward.conflict_brake_weight * conflict_level * speed_deficit
        braking -= self.config.reward.stop_go_penalty_weight * conflict_level * speed_drop
        braking += conflict_relief_scale * (
            self._compute_head_on_guidance_reward(
                observation,
                self._previous_forward_speeds.get(agent_id, current_forward_speed),
            )
            + self._compute_crossing_overtaking_guidance_reward(observation)
        )

        smoothness_scale = max(0.1, 1.0 - (self.config.goal_proximity_smoothness_relief * goal_proximity))
        smoothness = -(self.config.reward.action_smoothness_weight * smoothness_scale) * float(
            np.linalg.norm(action - self._previous_actions[agent_id])
        )
        heading_scale = max(
            0.1,
            1.0 - (self.config.reward.heading_relief_factor * min(conflict_risk, 1.0)) - (self.config.goal_proximity_heading_relief * goal_proximity),
        )
        heading = -((self.config.reward.heading_error_weight * heading_scale) * abs(observation.heading_error))
        team = -self.config.team_reward_weight * max(0.0, self.config.near_miss_distance - pair_min)
        if team_progress_delta >= 0.0:
            team += self.config.team_progress_weight * team_progress_delta
        else:
            team -= self.config.team_regression_penalty_weight * abs(team_progress_delta)

        separation_excess = 0.0
        if np.isfinite(global_state.team_mean_separation) and np.isfinite(self._initial_team_mean_separation):
            separation_excess = max(
                0.0,
                global_state.team_mean_separation - self._initial_team_mean_separation - self.config.team_dispersion_margin,
            )
        team -= self.config.team_dispersion_penalty_weight * separation_excess

        coordination = self.config.coordination_reward_weight * global_state.goal_completion_ratio
        if np.isfinite(self._initial_team_mean_goal_distance) and self._initial_team_mean_goal_distance > 1e-6:
            team_goal_proximity = max(
                0.0,
                1.0 - (global_state.team_mean_goal_distance / self._initial_team_mean_goal_distance),
            )
            coordination += self.config.team_goal_proximity_weight * team_goal_proximity
        if global_state.team_mean_goal_distance >= self._best_team_mean_distance - 1e-3:
            coordination -= self.config.deadlock_penalty_weight * 0.02

        time_cost = -self.config.reward.time_penalty

        terminal = 0.0
        if terminated and all_reached:
            terminal += self.config.reward.goal_bonus
            terminal += self.config.team_completion_bonus
        if truncated:
            terminal += self.config.reward.stall_penalty

        return MultiAgentRewardBreakdown(
            progress=progress,
            safety=safety,
            braking=braking,
            smoothness=smoothness,
            heading=heading,
            team=team,
            coordination=coordination,
            time_cost=time_cost,
            terminal=terminal,
        )

    def step(self, actions: Dict[str, np.ndarray]):
        action_map = {}
        projected_actions = {}
        for namespace in self._agent_ids:
            raw_action = actions.get(namespace, self.zero_policy_action())
            projected = self.project_policy_action(namespace, raw_action)
            projected_actions[namespace] = projected
            action_map[namespace] = self.expand_policy_action(namespace, projected)

        if self.config.enable_rl_backend:
            self._bridge.publish_actions(action_map)

        time.sleep(self.config.control_dt)
        observations = self._wait_for_local_observations(timeout=self.config.state_timeout)
        global_state = self._wait_for_global_state(timeout=self.config.state_timeout)
        self._latest_observations = observations

        team_progress = 0.0
        for namespace, observation in observations.items():
            team_progress += self._previous_distances[namespace] - observation.distance_to_goal
        if team_progress >= self.config.min_progress_delta:
            self._last_team_progress_time = time.monotonic()
        self._best_team_mean_distance = min(self._best_team_mean_distance, global_state.team_mean_goal_distance)

        pair_min = global_state.team_min_separation
        collision = np.isfinite(pair_min) and pair_min < self.config.collision_distance
        all_reached = all(observation.distance_to_goal <= self.config.goal_tolerance for observation in observations.values())
        elapsed = time.monotonic() - self._episode_start
        terminated = collision or all_reached
        truncated = elapsed >= self.config.episode_timeout or (time.monotonic() - self._last_team_progress_time) >= self.config.no_progress_timeout

        rewards = {}
        for namespace, observation in observations.items():
            breakdown = self._compute_reward(
                namespace,
                observation,
                projected_actions[namespace],
                global_state,
                team_progress,
                terminated,
                truncated,
                all_reached,
            )
            rewards[namespace] = breakdown.total
            self._previous_distances[namespace] = observation.distance_to_goal
            self._previous_actions[namespace] = projected_actions[namespace]
            self._previous_forward_speeds[namespace] = max(0.0, observation.final_linear_x)

        terminated_dict = {namespace: terminated for namespace in self._agent_ids}
        terminated_dict['__all__'] = terminated
        truncated_dict = {namespace: truncated for namespace in self._agent_ids}
        truncated_dict['__all__'] = truncated

        info = {
            'scenario': self._scenario.name if self._scenario is not None else 'unknown',
            'pairwise_min_separation': pair_min,
            'pairwise_mean_separation': global_state.team_mean_separation,
            'goal_completion_ratio': global_state.goal_completion_ratio,
            'team_mean_goal_distance': global_state.team_mean_goal_distance,
            'global_state': global_state.to_vector(self.config.max_agents, self.config.max_neighbors),
        }

        return (
            {namespace: observation.to_vector(self.config.max_neighbors) for namespace, observation in observations.items()},
            rewards,
            terminated_dict,
            truncated_dict,
            info,
        )

    def close(self):
        bridge = self._bridge
        sim_node = self._sim_node
        controllers = list(self._controllers.values())

        if bridge is not None:
            try:
                bridge.set_rl_backend_enabled(False)
            except Exception:
                pass
            try:
                bridge.clear_scenario()
            except Exception:
                pass
            try:
                bridge.prepare_for_shutdown()
            except Exception:
                pass
        if sim_node is not None:
            try:
                sim_node.prepare_for_shutdown()
            except Exception:
                pass
        for controller in controllers:
            try:
                controller.prepare_for_shutdown()
            except Exception:
                pass

        if bridge is not None:
            try:
                self._executor.remove_node(bridge)
            except Exception:
                pass
        if sim_node is not None:
            try:
                self._executor.remove_node(sim_node)
            except Exception:
                pass
        for controller in controllers:
            try:
                self._executor.remove_node(controller)
            except Exception:
                pass

        try:
            self._executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._spin_thread = None

        if bridge is not None:
            try:
                bridge.destroy_node()
            except Exception:
                pass
            self._bridge = None
        if sim_node is not None:
            try:
                sim_node.destroy_node()
            except Exception:
                pass
            self._sim_node = None
        for controller in controllers:
            try:
                controller.destroy_node()
            except Exception:
                pass
        self._controllers.clear()