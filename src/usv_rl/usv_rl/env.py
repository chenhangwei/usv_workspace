import threading
import time
from types import SimpleNamespace
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from .action_projection import project_residual_action
from .config import EnvConfig
from .ros_bridge import TrainingBridge
from .scenarios import ScenarioFactory
from .simple_sim import SimpleUsvSimNode
from .types import RewardBreakdown, UsvObservation
from usv_control.velocity_controller_node import VelocityControllerNode

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    gym = SimpleNamespace(Env=object)
    spaces = None


class UsvRlEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, config: Optional[EnvConfig] = None):
        if not rclpy.ok():
            rclpy.init()

        self.config = config or EnvConfig()
        self._episode_index = 0
        self._rng = np.random.default_rng()
        self._executor = MultiThreadedExecutor()
        self._spin_thread: Optional[threading.Thread] = None
        self._bridge: Optional[TrainingBridge] = None
        self._controller: Optional[VelocityControllerNode] = None
        self._sim_node: Optional[SimpleUsvSimNode] = None
        self._scenario = None
        self._episode_start = 0.0
        self._last_progress_time = 0.0
        self._best_distance = float('inf')
        self._previous_distance = float('inf')
        self._previous_action = self.zero_policy_action()
        self._previous_forward_speed = 0.0
        self._latest_observation: Optional[UsvObservation] = None

        if spaces is not None:
            policy_action_dim = self._policy_action_dim()
            if policy_action_dim == 1:
                low = np.asarray([
                    -self.config.action_bounds.angular_delta,
                ], dtype=np.float32)
                high = np.asarray([
                    self.config.action_bounds.angular_delta,
                ], dtype=np.float32)
            else:
                low = np.asarray([
                    -self.config.action_bounds.linear_delta,
                    -self.config.action_bounds.angular_delta,
                ], dtype=np.float32)
                high = np.asarray([
                    self.config.action_bounds.linear_delta,
                    self.config.action_bounds.angular_delta,
                ], dtype=np.float32)
            self.action_space = spaces.Box(
                low=low,
                high=high,
            )
            self.observation_space = spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(UsvObservation.vector_size(self.config.max_neighbors),),
                dtype=np.float32,
            )

        self._ensure_runtime()

    def _compute_conflict_risk(self, observation: UsvObservation) -> float:
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

    def _compute_head_on_guidance_reward(self, observation: UsvObservation, previous_forward_speed: float) -> float:
        scenario_name = getattr(self._scenario, 'name', '') if self._scenario is not None else ''
        if 'head_on' not in scenario_name:
            return 0.0

        guidance_distance = max(
            self.config.reward.head_on_guidance_distance,
            self.config.reward.anticipation_distance,
            self.config.collision_distance + 1e-3,
        )
        target_offset = max(0.2, self.config.reward.head_on_target_starboard_offset)

        best_neighbor = None
        best_score = -1.0
        for neighbor in observation.neighbors:
            if 'head_on' not in neighbor.usv_id:
                continue
            if neighbor.distance <= 1e-3 or neighbor.distance > guidance_distance:
                continue
            if neighbor.rel_x <= 0.0:
                continue

            range_rate = -(
                (neighbor.rel_x * neighbor.rel_vx) + (neighbor.rel_y * neighbor.rel_vy)
            ) / max(neighbor.distance, 1e-3)
            if range_rate <= 0.0:
                continue

            proximity = (guidance_distance - neighbor.distance) / guidance_distance
            centerline_exposure = max(0.0, 1.0 - min(1.0, abs(neighbor.rel_y) / max(0.8, target_offset)))
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
        turn_progress = max(0.0, min(1.0, actual_starboard_turn / desired_starboard_turn))
        desired_forward_speed = min(
            max(self.config.reward.desired_conflict_speed, 0.0),
            max(observation.raw_linear_x, self.config.reward.desired_conflict_speed),
        )
        current_forward_speed = max(0.0, observation.final_linear_x)
        forward_progress = max(0.0, min(1.0, current_forward_speed / max(desired_forward_speed, 1e-3)))
        speed_drop = max(0.0, previous_forward_speed - current_forward_speed)
        speed_drop_ratio = max(0.0, min(1.0, speed_drop / max(desired_forward_speed, 1e-3)))

        guidance_reward = (
            self.config.reward.head_on_corridor_reward_weight * proximity * corridor_progress
            - self.config.reward.head_on_centerline_penalty_weight * proximity * centerline_penalty
            + self.config.reward.head_on_turn_reward_weight * proximity * turn_progress
            + self.config.reward.head_on_forward_reward_weight * proximity * forward_progress
            - self.config.reward.head_on_speed_drop_penalty_weight * proximity * speed_drop_ratio
        )
        return guidance_reward

    def _compute_crossing_overtaking_guidance_reward(self, observation: UsvObservation) -> float:
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

    def _policy_action_dim(self) -> int:
        if self.config.action_mode == 'angular_only':
            return 1
        return 2

    def zero_policy_action(self) -> np.ndarray:
        return np.zeros(self._policy_action_dim(), dtype=np.float32)

    def project_policy_action(self, action) -> np.ndarray:
        raw_linear_x = None
        if self._latest_observation is not None:
            raw_linear_x = self._latest_observation.raw_linear_x

        return project_residual_action(
            action,
            action_mode=self.config.action_mode,
            linear_delta_limit=self.config.action_bounds.linear_delta,
            angular_delta_limit=self.config.action_bounds.angular_delta,
            raw_linear_x=raw_linear_x,
            forward_only=True,
        )

    def expand_policy_action(self, action) -> tuple[float, float]:
        policy_action = self.project_policy_action(action)
        if self.config.action_mode == 'angular_only':
            return 0.0, float(policy_action[0])
        return float(policy_action[0]), float(policy_action[1])

    def _ensure_runtime(self):
        if self._sim_node is None:
            self._sim_node = SimpleUsvSimNode(namespace=self.config.namespace)
            self._sim_node.reset()
            self._executor.add_node(self._sim_node)

        if self.config.launch_sitl and self._controller is None:
            self._controller = VelocityControllerNode(
                namespace=f'/{self.config.namespace}',
                parameter_overrides=[
                    Parameter('cruise_speed', value=self.config.cruise_speed),
                    Parameter('max_angular_velocity', value=self.config.max_angular_velocity),
                    Parameter('apf_enabled', value=True),
                    Parameter('apf_orca_enabled', value=False),
                    Parameter('require_guided_mode', value=True),
                    Parameter('require_armed', value=True),
                ]
            )
            self._executor.add_node(self._controller)

        if self._bridge is None:
            self._bridge = TrainingBridge(self.config)
            self._executor.add_node(self._bridge)
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()

        if self.config.launch_sitl and not self._bridge.wait_for_controller(timeout=self.config.ready_timeout):
            raise RuntimeError('Timed out waiting for velocity_controller_node parameter service.')

        if not self._bridge.wait_for_pose(timeout=self.config.ready_timeout):
            raise RuntimeError('Timed out waiting for simulated pose/velocity topics.')

    def _wait_for_observation(self, timeout: Optional[float] = None) -> UsvObservation:
        deadline = time.monotonic() + (timeout or self.config.state_timeout)
        while time.monotonic() < deadline:
            observation = self._bridge.get_observation()
            if observation is not None:
                return observation
            time.sleep(0.05)
        raise RuntimeError('Timed out waiting for navigation feedback.')

    def _wait_for_pose_snapshot(self, timeout: Optional[float] = None) -> tuple[float, float, float, float]:
        deadline = time.monotonic() + (timeout or self.config.state_timeout)
        while time.monotonic() < deadline:
            snapshot = self._bridge.get_pose_snapshot()
            if snapshot is not None:
                return snapshot
            time.sleep(0.05)
        raise RuntimeError('Timed out waiting for simulated pose snapshot.')

    def _request_initial_observation(self, task_name: str) -> UsvObservation:
        last_error: Optional[RuntimeError] = None
        for attempt in range(3):
            self._bridge.reset_episode_state()
            self._bridge.publish_goal(self._scenario.goal_x, self._scenario.goal_y, task_name)
            time.sleep(max(self.config.control_dt * 2.0, 0.5) + (0.25 * attempt))
            try:
                return self._wait_for_observation(timeout=self.config.state_timeout + (2.0 * attempt))
            except RuntimeError as exc:
                last_error = exc
        assert last_error is not None
        if self._controller is not None:
            controller_log_reader = getattr(self._controller, 'read_log_tail', None)
            if callable(controller_log_reader):
                controller_log = controller_log_reader()
                if controller_log:
                    raise RuntimeError(f'{last_error}\nController log tail:\n{controller_log}') from last_error
        raise last_error

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super_reset = getattr(super(), 'reset', None)
        if callable(super_reset):
            try:
                super_reset(seed=seed)
            except TypeError:
                pass
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        self._ensure_runtime()
        options = options or {}
        scenario_kind = options.get('scenario_kind')
        if not scenario_kind:
            scenario_kind = self.config.default_scenarios[self._episode_index % len(self.config.default_scenarios)]

        if self._controller is not None:
            self._controller.reset_for_training_episode()

        if self._sim_node is not None:
            self._sim_node.reset()
            time.sleep(0.3)

        current_pose_x, current_pose_y, current_yaw, _ = self._wait_for_pose_snapshot()
        self._scenario = ScenarioFactory.create(
            kind=scenario_kind,
            origin_x=current_pose_x,
            origin_y=current_pose_y,
            yaw=current_yaw,
            goal_distance=self.config.goal_distance,
            neighbor_speed=self.config.scenario_neighbor_speed,
        )
        self._bridge.activate_scenario(self._scenario)
        self._bridge.set_rl_backend_enabled(self.config.enable_rl_backend)
        task_name = f'rl_{self._scenario.name}_{self._episode_index:04d}'
        observation = self._request_initial_observation(task_name)
        self._episode_index += 1
        self._episode_start = time.monotonic()
        self._last_progress_time = self._episode_start
        self._best_distance = observation.distance_to_goal
        self._previous_distance = observation.distance_to_goal
        self._previous_action = self.zero_policy_action()
        self._previous_forward_speed = max(0.0, observation.final_linear_x)
        self._latest_observation = observation

        return observation.to_vector(self.config.max_neighbors), {'scenario': self._scenario.name}

    def _compute_reward(self, observation: UsvObservation, action: np.ndarray, terminated: bool, truncated: bool) -> RewardBreakdown:
        progress_delta = self._previous_distance - observation.distance_to_goal
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

        min_distance = observation.min_neighbor_distance()
        safety = 0.0
        conflict_risk = self._compute_conflict_risk(observation)
        if min_distance < self.config.collision_distance:
            safety += self.config.reward.collision_penalty
        elif min_distance < self.config.near_miss_distance:
            safety -= self.config.reward.near_miss_weight * (self.config.near_miss_distance - min_distance)
        conflict_relief_scale = max(0.35, 1.0 - (self.config.goal_proximity_conflict_relief * goal_proximity))
        safety -= self.config.reward.conflict_risk_weight * conflict_risk * conflict_relief_scale

        conflict_level = min(conflict_risk, 1.0) * conflict_relief_scale
        current_forward_speed = max(0.0, observation.final_linear_x)
        desired_conflict_speed = min(
            max(self.config.reward.desired_conflict_speed, 0.0),
            max(observation.raw_linear_x, self.config.reward.desired_conflict_speed),
        )
        speed_deficit = max(0.0, desired_conflict_speed - current_forward_speed)
        speed_drop = max(0.0, self._previous_forward_speed - current_forward_speed)
        braking = -self.config.reward.conflict_brake_weight * conflict_level * speed_deficit
        braking -= self.config.reward.stop_go_penalty_weight * conflict_level * speed_drop
        braking += conflict_relief_scale * (
            self._compute_head_on_guidance_reward(
                observation,
                self._previous_forward_speed,
            )
            + self._compute_crossing_overtaking_guidance_reward(observation)
        )

        heading_scale = max(
            0.15,
            1.0 - (self.config.reward.heading_relief_factor * min(conflict_risk, 1.0)) - (self.config.goal_proximity_heading_relief * goal_proximity),
        )
        heading = -(self.config.reward.heading_error_weight * heading_scale) * abs(observation.heading_error)
        smoothness_scale = max(0.1, 1.0 - (self.config.goal_proximity_smoothness_relief * goal_proximity))
        smoothness = -(self.config.reward.action_smoothness_weight * smoothness_scale) * float(np.linalg.norm(action - self._previous_action))
        time_cost = -self.config.reward.time_penalty

        terminal = 0.0
        if terminated and observation.distance_to_goal <= self.config.goal_tolerance:
            terminal += self.config.reward.goal_bonus
        if truncated and (time.monotonic() - self._last_progress_time) >= self.config.no_progress_timeout:
            terminal += self.config.reward.stall_penalty

        return RewardBreakdown(
            progress=progress,
            safety=safety,
            braking=braking,
            smoothness=smoothness,
            heading=heading,
            time_cost=time_cost,
            terminal=terminal,
        )

    def step(self, action):
        policy_action = self.project_policy_action(action)
        linear_delta, angular_delta = self.expand_policy_action(policy_action)

        if self.config.enable_rl_backend:
            self._bridge.publish_rl_action(linear_delta, angular_delta)

        time.sleep(self.config.control_dt)
        observation = self._wait_for_observation()
        elapsed = time.monotonic() - self._episode_start

        if observation.distance_to_goal < (self._best_distance - self.config.min_progress_delta):
            self._best_distance = observation.distance_to_goal
            self._last_progress_time = time.monotonic()

        terminated = observation.distance_to_goal <= self.config.goal_tolerance
        terminated = terminated or observation.min_neighbor_distance() < self.config.collision_distance
        truncated = elapsed >= min(self.config.episode_timeout, self._scenario.duration)
        truncated = truncated or ((time.monotonic() - self._last_progress_time) >= self.config.no_progress_timeout)

        reward = self._compute_reward(observation, policy_action, terminated, truncated)
        info = {
            'scenario': self._scenario.name,
            'reward_breakdown': {
                'progress': reward.progress,
                'safety': reward.safety,
                'braking': reward.braking,
                'smoothness': reward.smoothness,
                'heading': reward.heading,
                'time_cost': reward.time_cost,
                'terminal': reward.terminal,
            },
            'teacher_action': self.get_teacher_action(),
            'min_neighbor_distance': observation.min_neighbor_distance(),
        }

        self._previous_distance = observation.distance_to_goal
        self._previous_action = policy_action
        self._previous_forward_speed = max(0.0, observation.final_linear_x)
        self._latest_observation = observation
        return observation.to_vector(self.config.max_neighbors), reward.total, terminated, truncated, info

    def get_teacher_action(self) -> np.ndarray:
        teacher = self._bridge.get_teacher_residual_action()
        if teacher is None:
            return self.zero_policy_action()
        return self.project_policy_action(np.asarray(teacher, dtype=np.float32))

    def close(self):
        bridge = self._bridge
        sim_node = self._sim_node
        controller = self._controller

        if bridge is not None:
            try:
                bridge.set_rl_backend_enabled(False)
            except Exception:
                pass
            bridge.clear_scenario()
            bridge.prepare_for_shutdown()
        if sim_node is not None:
            sim_node.prepare_for_shutdown()
        if controller is not None:
            controller.prepare_for_shutdown()
        if bridge is not None:
            self._executor.remove_node(bridge)
        if sim_node is not None:
            self._executor.remove_node(sim_node)
        if controller is not None:
            self._executor.remove_node(controller)
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        if bridge is not None:
            bridge.destroy_node()
            self._bridge = None
        if sim_node is not None:
            sim_node.destroy_node()
            self._sim_node = None
        if controller is not None:
            controller.destroy_node()
            self._controller = None