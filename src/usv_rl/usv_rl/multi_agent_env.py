import math
import threading
import time
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from .action_projection import project_policy_action as project_rl_policy_action
from .config import ActionBounds, RewardConfig
from .multi_agent_bridge import MultiAgentTrainingBridge
from .multi_agent_scenarios import AgentGoalConfig, AgentSpawnConfig, MultiAgentScenarioFactory
from .multi_agent_types import AgentLocalObservation, FleetGlobalState, MultiAgentRewardBreakdown, ENCOUNTER_TYPE_COUNT
from .multi_usv_sim import MultiUsvSimNode
from usv_control.velocity_controller_node import VelocityControllerNode

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    gym = SimpleNamespace(Env=object)
    spaces = None

# Maps scenario name → encounter type index for one-hot conditioning.
_SCENARIO_ENCOUNTER_MAP = {
    'single_usv_overtaking': 2,
    'two_usv_head_on': 0,
    'two_usv_crossing': 1,
    'two_usv_overtaking': 2,
    'three_usv_crossing': 1,
    'three_usv_overtaking': 2,
}


@dataclass
class MultiAgentEnvConfig:
    agent_namespaces: tuple[str, ...] = ('usv_01', 'usv_02', 'usv_03')
    launch_sitl: bool = True
    enable_rl_backend: bool = True
    rl_control_mode: str = 'pure'
    action_mode: str = 'full'
    max_neighbors: int = 4
    max_agents: int = 3
    cruise_speed: float = 0.4
    max_angular_velocity: float = 0.4
    control_dt: float = 0.2
    heading_omega_deadband: float = 0.06
    heading_omega_reference: float = 0.85
    angular_authority_power: float = 1.6
    angular_accel_limit: float = 1.8
    angular_decel_limit: float = 2.4
    conflict_turn_relief: float = 0.55
    # [L1-1] Structural fix for near-goal hover / omega-bang-bang deadlock.
    # authority_floor: minimum ω capacity preserved at all times (fraction of max_omega).
    # min_forward_speed_floor: minimum forward speed enforced when policy requests motion (m/s).
    angular_authority_floor: float = 0.35
    min_forward_speed_floor: float = 0.08
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
    goal_proximity_conflict_relief: float = 0.25
    goal_proximity_speed_relief: float = 0.0
    team_reward_weight: float = 0.30
    coordination_reward_weight: float = 0.20
    team_progress_weight: float = 1.20
    team_goal_proximity_weight: float = 0.0
    team_regression_penalty_weight: float = 0.0
    team_dispersion_penalty_weight: float = 0.0
    team_dispersion_margin: float = 0.0
    separation_recovery_weight: float = 0.0
    team_completion_bonus: float = 18.0
    deadlock_penalty_weight: float = 4.0
    # Entanglement duration penalty: penalises sustained close proximity
    # to break orbital-lock behaviour where USVs circle each other.
    entanglement_penalty_weight: float = 0.0
    entanglement_distance: float = 3.0
    entanglement_grace_steps: int = 30
    entanglement_low_speed_penalty_weight: float = 0.0
    # Signed CTE observation clip range (symmetric, metres).
    cte_clip_range: float = 3.0
    # Observation-only route_progress correction: when enabled, large CTE lowers
    # the progress signal so clipped along-track projection is not treated as completion.
    route_progress_cte_gate_start: float = 0.0
    route_progress_cte_gate_width: float = 0.0
    route_progress_cte_gate_floor: float = 0.25
    # When enabled, random encounters use the mean active route midpoint as a
    # shared conflict point so priority features follow predicted ETA order.
    random_encounter_route_priority: bool = False
    # Domain randomization
    domain_randomization: bool = False
    dr_position_noise_std: float = 0.10
    dr_heading_noise_std: float = 0.02
    dr_velocity_noise_ratio: float = 0.03
    dr_current_speed_max: float = 0.04
    dr_current_theta: float = 0.15
    dr_current_sigma: float = 0.02
    dr_velocity_exec_noise: float = 0.05
    # Scenario randomization
    scenario_spawn_position_std: float = 0.0
    scenario_spawn_heading_std: float = 0.0
    scenario_goal_position_std: float = 0.0
    # Sim dynamics time constants (first-order velocity filter)
    sim_tau_linear: float = 0.45
    sim_tau_angular: float = 0.25
    # Tau domain randomization: if ranges are set (low>0 and high>low), tau is
    # sampled uniformly each episode reset for sim-to-real robustness.
    dr_tau_linear_low: float = 0.0
    dr_tau_linear_high: float = 0.0
    dr_tau_angular_low: float = 0.0
    dr_tau_angular_high: float = 0.0
    # Distance-aware speed scaling (must match SITL deployment)
    speed_scale_distance: float = 0.0
    speed_scale_min: float = 0.35
    # Encounter type dropout: probability of zeroing out the encounter
    # one-hot during training, forcing the policy to avoid relying on
    # scenario labels for avoidance decisions.
    encounter_type_dropout: float = 0.0
    # Multi-waypoint episodes: when > 1, reaching the goal generates a
    # new random waypoint instead of terminating.  Trains the policy to
    # handle waypoint transitions (heading snap) within a single episode.
    max_waypoints_per_episode: int = 1
    # Intermediate bonus given each time the agent reaches a waypoint
    # (except the final one, which uses goal_bonus).
    waypoint_bonus: float = 10.0
    pairwise_shield_enabled: bool = False
    pairwise_shield_release_separation: float = 1.05
    pairwise_shield_critical_separation: float = 0.92
    pairwise_shield_min_closing_speed: float = 0.004
    pairwise_shield_yield_speed: float = 0.02
    pairwise_shield_standon_speed: float = 0.24
    pairwise_shield_yield_omega: float = 0.44
    pairwise_shield_standon_omega: float = 0.04
    pairwise_shield_yield_danger_scale: float = 1.0
    pairwise_shield_blend: float = 1.0
    pairwise_shield_turn_mode: str = 'away'
    pairwise_shield_role_mode: str = 'priority-delta'
    pairwise_shield_priority_delta_yield_threshold: float = -0.01
    pairwise_shield_route_eta_yield_threshold: float = 0.02


class MultiAgentEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, config: Optional[MultiAgentEnvConfig] = None):
        if not rclpy.ok():
            rclpy.init()

        self.config = config or MultiAgentEnvConfig()
        self._validate_action_config()
        self._runtime_agent_ids = tuple(self.config.agent_namespaces)
        self._active_agent_ids = self._runtime_agent_ids
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
        self._previous_conflict_risks: Dict[str, float] = {}
        self._previous_cpa_metrics: Dict[str, Dict[str, Dict[str, float]]] = {}
        self._latest_observations: Dict[str, AgentLocalObservation] = {}
        self._best_team_mean_distance = float('inf')
        self._initial_team_mean_separation = float('inf')
        self._initial_team_mean_goal_distance = float('inf')
        self._waypoints_completed: Dict[str, int] = {}

        self.action_dim = 1 if self.config.action_mode == 'angular_only' else 2
        self.action_low, self.action_high = self._policy_action_bounds()

        if spaces is not None:
            self.single_agent_action_space = spaces.Box(low=self.action_low, high=self.action_high, dtype=np.float32)
            self.single_agent_observation_space = spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(self.local_observation_size,),
                dtype=np.float32,
            )

        self._ensure_runtime()

    def _validate_action_config(self):
        if self.config.action_mode != 'full':
            raise ValueError('Pure RL control requires action_mode="full".')

    def _pure_linear_speed_limit(self) -> float:
        return max(float(self.config.action_bounds.linear_delta), float(self.config.cruise_speed))

    def _pure_angular_speed_limit(self) -> float:
        return max(float(self.config.action_bounds.angular_delta), float(self.config.max_angular_velocity))

    def _policy_action_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        low = np.asarray([
            0.0,
            -self._pure_angular_speed_limit(),
        ], dtype=np.float32)
        high = np.asarray([
            self._pure_linear_speed_limit(),
            self._pure_angular_speed_limit(),
        ], dtype=np.float32)
        return low, high

    @property
    def local_observation_size(self) -> int:
        return AgentLocalObservation.vector_size(self.config.max_neighbors)

    @property
    def global_state_size(self) -> int:
        return FleetGlobalState.vector_size(self.config.max_agents, self.config.max_neighbors)

    @property
    def agent_ids(self) -> tuple[str, ...]:
        return self._active_agent_ids

    @property
    def current_scenario_name(self) -> str:
        return self._scenario.name if self._scenario is not None else 'unknown'

    @property
    def runtime_agent_ids(self) -> tuple[str, ...]:
        return self._runtime_agent_ids

    @property
    def inactive_agent_ids(self) -> tuple[str, ...]:
        return tuple(agent_id for agent_id in self._runtime_agent_ids if agent_id not in self._active_agent_ids)

    def _ensure_runtime(self):
        if self._sim_node is None:
            self._sim_node = MultiUsvSimNode(
                self._runtime_agent_ids,
                tau_linear=self.config.sim_tau_linear,
                tau_angular=self.config.sim_tau_angular,
            )
            if self.config.domain_randomization:
                self._sim_node.set_domain_randomization(
                    enabled=True,
                    position_noise_std=self.config.dr_position_noise_std,
                    heading_noise_std=self.config.dr_heading_noise_std,
                    velocity_noise_ratio=self.config.dr_velocity_noise_ratio,
                    current_speed_max=self.config.dr_current_speed_max,
                    current_theta=self.config.dr_current_theta,
                    current_sigma=self.config.dr_current_sigma,
                    velocity_exec_noise=self.config.dr_velocity_exec_noise,
                )
            self._executor.add_node(self._sim_node)

        if self._bridge is None:
            self._bridge = MultiAgentTrainingBridge(self._runtime_agent_ids, self.config.neighbor_publish_rate)
            self._executor.add_node(self._bridge)

        if not self._controllers:
            for namespace in self._runtime_agent_ids:
                controller_parameters = [
                    Parameter('cruise_speed', Parameter.Type.DOUBLE, self.config.cruise_speed),
                    Parameter('max_angular_velocity', Parameter.Type.DOUBLE, self.config.max_angular_velocity),
                    Parameter('ampc_enabled', Parameter.Type.BOOL, False),
                    Parameter('adaptive_tau_enabled', Parameter.Type.BOOL, False),
                    Parameter('apf_enabled', Parameter.Type.BOOL, True),
                    Parameter('apf_orca_enabled', Parameter.Type.BOOL, False),
                    Parameter('require_guided_mode', Parameter.Type.BOOL, True),
                    Parameter('require_armed', Parameter.Type.BOOL, True),
                ]
                controller = VelocityControllerNode(
                    namespace=f'/{namespace}',
                    parameter_overrides=controller_parameters,
                )
                controller.set_parameters(controller_parameters)
                if bool(controller.get_parameter('ampc_enabled').value) or bool(controller.get_parameter('adaptive_tau_enabled').value):
                    raise RuntimeError(
                        f'Training controller {namespace} failed to disable AMPC/adaptive tau overrides.'
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

    def _projection_conflict_level(self, observation: Optional[AgentLocalObservation]) -> float:
        if observation is None or not observation.neighbors:
            return 0.0
        lookahead_distance = max(
            self.config.reward.anticipation_distance,
            self.config.reward.conflict_distance,
            self.config.collision_distance + 1e-3,
        )
        min_neighbor_distance = observation.min_neighbor_distance()
        raw_level = float(np.clip((lookahead_distance - min_neighbor_distance) / lookahead_distance, 0.0, 1.0))
        # Reduce conflict level when the closest neighbour is separating,
        # so the agent regains turning authority to break free.
        if observation.neighbors and raw_level > 0.0:
            closest = min(observation.neighbors, key=lambda n: n.distance)
            d = max(closest.distance, 1e-3)
            # range_rate > 0 means closing, < 0 means separating
            range_rate = -(closest.rel_x * closest.rel_vx + closest.rel_y * closest.rel_vy) / d
            if range_rate < -0.02:
                separation_relief = min(1.0, abs(range_rate) / 0.15)
                raw_level *= max(0.25, 1.0 - 0.6 * separation_relief)
        return raw_level

    def project_policy_action(self, agent_id: str, action) -> np.ndarray:
        observation = self._latest_observations.get(agent_id)
        raw_linear_x = None if observation is None else observation.raw_linear_x
        return project_rl_policy_action(
            action,
            rl_control_mode='pure',
            action_mode=self.config.action_mode,
            linear_delta_limit=self._pure_linear_speed_limit(),
            angular_delta_limit=self._pure_angular_speed_limit(),
            raw_linear_x=raw_linear_x,
            heading_error=None if observation is None else observation.heading_error,
            current_angular_z=0.0 if observation is None else observation.final_angular_z,
            control_dt=self.config.control_dt,
            conflict_level=self._projection_conflict_level(observation),
            heading_omega_deadband=self.config.heading_omega_deadband,
            heading_omega_reference=self.config.heading_omega_reference,
            angular_authority_power=self.config.angular_authority_power,
            angular_accel_limit=self.config.angular_accel_limit,
            angular_decel_limit=self.config.angular_decel_limit,
            conflict_turn_relief=self.config.conflict_turn_relief,
            forward_only=True,
            authority_floor=self.config.angular_authority_floor,
            min_forward_speed_floor=self.config.min_forward_speed_floor,
        )

    def _target_forward_speed(
        self,
        observation: AgentLocalObservation,
        *,
        conflict_level: float,
        goal_proximity: float = 0.0,
    ) -> float:
        cruise_speed = max(0.18, self._pure_linear_speed_limit())
        target_speed = cruise_speed * (1.0 - 0.55 * float(np.clip(conflict_level, 0.0, 1.0)))
        heading_gate = 0.30 + 0.70 * max(0.0, math.cos(min(abs(observation.heading_error), math.pi / 2.0)))
        target_speed *= heading_gate
        if goal_proximity > 0.0:
            target_speed *= max(
                0.25,
                1.0 - (float(np.clip(self.config.goal_proximity_speed_relief, 0.0, 1.0)) * goal_proximity),
            )

        minimum_speed = max(0.08, min(self.config.reward.desired_conflict_speed, 0.18))
        return float(np.clip(target_speed, minimum_speed, cruise_speed))

    def _pure_goal_tracking_reward(
        self,
        observation: AgentLocalObservation,
        *,
        conflict_level: float,
        goal_proximity: float,
    ) -> float:
        target_speed = self._target_forward_speed(
            observation,
            conflict_level=conflict_level,
            goal_proximity=goal_proximity,
        )
        current_forward_speed = max(0.0, observation.final_linear_x)
        aligned = max(0.0, math.cos(min(abs(observation.heading_error), math.pi / 2.0)))
        open_water = max(0.0, 1.0 - float(np.clip(conflict_level, 0.0, 1.0)))
        speed_ratio = min(1.0, current_forward_speed / max(target_speed, 1e-3))
        speed_deficit_ratio = max(0.0, (target_speed - current_forward_speed) / max(target_speed, 1e-3))
        turn_rate = abs(observation.final_angular_z)
        excess_turn = max(0.0, turn_rate - 0.15)
        low_speed_ratio = max(0.0, 1.0 - speed_ratio)
        spin_ratio = min(1.0, low_speed_ratio * (0.45 + 0.55 * (1.0 - aligned)))
        idle_penalty_alignment = 0.45 + 0.55 * aligned
        crawl_ratio = max(0.0, (0.85 - speed_ratio) / 0.85)
        far_from_goal_gate = max(0.35, 1.0 - 0.65 * goal_proximity)

        reward = self.config.reward.pure_cruise_reward_weight * open_water * aligned * speed_ratio
        reward -= self.config.reward.pure_idle_penalty_weight * idle_penalty_alignment * speed_deficit_ratio
        reward -= (
            self.config.reward.pure_idle_penalty_weight
            * open_water
            * far_from_goal_gate
            * (0.35 + 0.65 * aligned)
            * 1.6
            * (crawl_ratio ** 2)
        )
        reward -= self.config.reward.pure_turn_penalty_weight * (0.50 + 0.50 * low_speed_ratio) * excess_turn
        reward -= self.config.reward.pure_spin_penalty_weight * spin_ratio * max(0.0, turn_rate - 0.18)
        return reward

    def _current_route_length(self, agent_id: str) -> float:
        if self._scenario is None:
            return float(self.config.goal_distance)

        spawn = self._scenario.agent_spawns.get(agent_id)
        goal = self._scenario.agent_goals.get(agent_id)
        if spawn is None or goal is None:
            return float(self.config.goal_distance)

        return float(max(np.hypot(goal.x - spawn.x, goal.y - spawn.y), self.config.goal_tolerance + 1e-3))

    def expand_policy_action(self, agent_id: str, action) -> tuple[float, float]:
        projected = self.project_policy_action(agent_id, action)
        if self.config.action_mode == 'angular_only':
            return 0.0, float(projected[0])
        linear_x = float(projected[0])
        angular_z = float(projected[1])
        # Distance-aware speed scaling (mirrors SITL deployment behaviour)
        if self.config.speed_scale_distance > 0.0:
            observation = self._latest_observations.get(agent_id)
            if observation is not None:
                min_dist = observation.min_neighbor_distance()
                if min_dist < self.config.speed_scale_distance:
                    speed_scale = max(
                        self.config.speed_scale_min,
                        min_dist / self.config.speed_scale_distance,
                    )
                    linear_x *= speed_scale
        return linear_x, angular_z

    def _pairwise_shield_is_yield(self, agent_id: str, observation: AgentLocalObservation, nearest) -> bool:
        role_mode = str(self.config.pairwise_shield_role_mode).strip().lower()
        if role_mode == 'priority-delta':
            return float(getattr(nearest, 'route_priority_delta', 0.0)) <= float(self.config.pairwise_shield_priority_delta_yield_threshold)
        if role_mode == 'route-eta-delta':
            return float(getattr(nearest, 'route_eta_delta', 0.0)) > float(self.config.pairwise_shield_route_eta_yield_threshold)
        if role_mode == 'priority':
            return float(observation.crossing_priority) <= -0.10
        return int(agent_id.rsplit('_', 1)[-1]) >= 2

    def _apply_pairwise_shield(
        self,
        agent_id: str,
        command: tuple[float, float],
        observation: Optional[AgentLocalObservation],
    ) -> tuple[float, float]:
        if not bool(self.config.pairwise_shield_enabled) or observation is None or not observation.neighbors:
            return command
        if self.current_scenario_name not in {'two_usv_random_encounter', 'three_usv_random_encounter'}:
            return command

        nearest = min(observation.neighbors, key=lambda item: float(item.distance), default=None)
        if nearest is None:
            return command
        nearest_distance = max(0.0, float(nearest.distance))
        release_separation = max(0.0, float(self.config.pairwise_shield_release_separation))
        if release_separation <= 0.0 or nearest_distance > release_separation:
            return command

        rel_x = float(nearest.rel_x)
        rel_y = float(nearest.rel_y)
        rel_vx = float(nearest.rel_vx)
        rel_vy = float(nearest.rel_vy)
        closing_speed = -((rel_x * rel_vx) + (rel_y * rel_vy)) / max(nearest_distance, 1e-3)
        critical = max(0.0, float(self.config.pairwise_shield_critical_separation))
        if nearest_distance > critical and closing_speed < float(self.config.pairwise_shield_min_closing_speed):
            return command

        danger = (release_separation - nearest_distance) / max(release_separation - min(critical, release_separation - 1e-3), 1e-3)
        danger = float(np.clip(danger, 0.0, 1.0))
        if nearest_distance <= critical:
            danger = 1.0

        is_yield = self._pairwise_shield_is_yield(agent_id, observation, nearest)
        turn_sign = -1.0
        if str(self.config.pairwise_shield_turn_mode).strip().lower() == 'away':
            if rel_y > 0.0:
                turn_sign = -1.0
            elif rel_y < 0.0:
                turn_sign = 1.0

        if is_yield:
            danger_scale = float(np.clip(self.config.pairwise_shield_yield_danger_scale, 0.0, 2.0))
            target_linear = max(0.0, float(self.config.pairwise_shield_yield_speed)) * float(np.clip(1.0 - danger_scale * danger, 0.0, 1.0))
            target_omega = turn_sign * max(0.0, float(self.config.pairwise_shield_yield_omega)) * danger
        else:
            target_linear = max(0.0, float(self.config.pairwise_shield_standon_speed))
            target_omega = turn_sign * max(0.0, float(self.config.pairwise_shield_standon_omega)) * danger

        blend = float(np.clip(self.config.pairwise_shield_blend, 0.0, 1.0))
        linear_x = (1.0 - blend) * float(command[0]) + blend * target_linear
        angular_z = (1.0 - blend) * float(command[1]) + blend * target_omega
        linear_x = float(np.clip(linear_x, self.action_low[0], self.action_high[0]))
        angular_z = float(np.clip(angular_z, self.action_low[1], self.action_high[1]))
        return linear_x, angular_z

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
            state = self._bridge.get_global_state(
                self.config.max_agents,
                self.config.max_neighbors,
                active_agent_ids=self._active_agent_ids,
                goal_tolerance=self.config.goal_tolerance,
            )
            if state is not None:
                return state
            time.sleep(0.05)
        raise RuntimeError('Timed out waiting for global fleet state.')

    def _scenario_initial_team_mean_goal_distance(self) -> float:
        if self._scenario is None:
            return float('inf')

        distances = []
        for agent_id in self._active_agent_ids:
            spawn = self._scenario.agent_spawns.get(agent_id)
            goal = self._scenario.agent_goals.get(agent_id)
            if spawn is None or goal is None:
                continue
            distances.append(float(np.hypot(goal.x - spawn.x, goal.y - spawn.y)))

        if not distances:
            return float('inf')
        return float(np.mean(distances))

    def _refresh_active_agent_ids(self):
        if self._scenario is None:
            self._active_agent_ids = self._runtime_agent_ids
            return

        active_ids = tuple(
            agent_id
            for agent_id in self._scenario.active_agent_ids
            if agent_id in self._runtime_agent_ids
        )
        self._active_agent_ids = active_ids or self._runtime_agent_ids

    def _filter_active_observations(self, observations: Dict[str, AgentLocalObservation]) -> Dict[str, AgentLocalObservation]:
        return {
            agent_id: observations[agent_id]
            for agent_id in self._active_agent_ids
            if agent_id in observations
        }

    def _scenario_conflict_point(self) -> tuple[float, float] | None:
        """Return the shared route conflict point for structured crossing scenarios."""
        if self._scenario is None:
            return None
        if self._scenario.name != 'three_usv_crossing':
            if not (
                bool(getattr(self.config, 'random_encounter_route_priority', False))
                and 'random_encounter' in self._scenario.name
            ):
                return None

        midpoints: list[tuple[float, float]] = []
        for agent_id in self._active_agent_ids:
            spawn = self._scenario.agent_spawns.get(agent_id)
            goal = self._scenario.agent_goals.get(agent_id)
            if spawn is None or goal is None:
                continue
            midpoints.append(((float(spawn.x) + float(goal.x)) * 0.5, (float(spawn.y) + float(goal.y)) * 0.5))
        if not midpoints:
            return None
        return (
            float(np.mean([point[0] for point in midpoints])),
            float(np.mean([point[1] for point in midpoints])),
        )

    @staticmethod
    def _crossing_priority_tiebreak(agent_id: str) -> int:
        # Break the fixed three-way crossing symmetry without prescribing an action.
        # The learned policy still decides how to use this observation feature.
        fixed_priority = {'usv_03': 0, 'usv_02': 1, 'usv_01': 2}
        if agent_id in fixed_priority:
            return fixed_priority[agent_id]
        try:
            return 100 - int(str(agent_id).rsplit('_', 1)[-1])
        except (TypeError, ValueError):
            return 100

    def _annotate_random_priority_for_observations(self, observations: Dict[str, AgentLocalObservation]) -> None:
        if self._scenario is None or 'random_encounter' not in self._scenario.name:
            return
        ordered_ids = sorted(
            (agent_id for agent_id in observations if agent_id in self._active_agent_ids),
            key=self._agent_priority_key,
        )
        if len(ordered_ids) <= 1:
            return

        denominator = max(1, len(ordered_ids) - 1)
        priority_by_id = {
            agent_id: float(1.0 - (2.0 * rank / denominator))
            for rank, agent_id in enumerate(ordered_ids)
        }
        for agent_id, observation in observations.items():
            observation.crossing_priority = priority_by_id.get(agent_id, 0.0)
            observation.crossing_eta_gap = 1.0

        for agent_id, observation in observations.items():
            own_priority = float(np.clip(observation.crossing_priority, -1.0, 1.0))
            for neighbor in observation.neighbors:
                other_priority = float(np.clip(priority_by_id.get(neighbor.source_id, 0.0), -1.0, 1.0))
                neighbor.route_priority_delta = float(np.clip((own_priority - other_priority) * 0.5, -1.0, 1.0))

    def _route_timing_features(
        self,
        agent_id: str,
        observation: AgentLocalObservation,
        conflict_point: tuple[float, float] | None,
    ) -> dict[str, float]:
        if self._scenario is None:
            return {
                'cross_track_error': 0.0,
                'route_progress': 0.0,
                'conflict_phase': 0.0,
                'conflict_eta': 1.0,
                'eta_seconds': 1e3,
                'to_conflict': 1e3,
            }

        spawn = self._scenario.agent_spawns.get(agent_id)
        goal = self._scenario.agent_goals.get(agent_id)
        if spawn is None or goal is None:
            return {
                'cross_track_error': 0.0,
                'route_progress': 0.0,
                'conflict_phase': 0.0,
                'conflict_eta': 1.0,
                'eta_seconds': 1e3,
                'to_conflict': 1e3,
            }

        route_dx = float(goal.x - spawn.x)
        route_dy = float(goal.y - spawn.y)
        route_length = float(np.hypot(route_dx, route_dy))
        if route_length <= 1e-6:
            return {
                'cross_track_error': 0.0,
                'route_progress': 0.0,
                'conflict_phase': 0.0,
                'conflict_eta': 1.0,
                'eta_seconds': 1e3,
                'to_conflict': 1e3,
            }

        unit_x = route_dx / route_length
        unit_y = route_dy / route_length
        relative_x = float(observation.pose_x - spawn.x)
        relative_y = float(observation.pose_y - spawn.y)
        progress_s = (relative_x * unit_x) + (relative_y * unit_y)
        cross_track = ((relative_x * route_dy) - (relative_y * route_dx)) / route_length
        clip_range = max(1.0, self.config.cte_clip_range)
        route_progress = float(np.clip(progress_s / route_length, 0.0, 1.0))
        gate_width = max(0.0, float(getattr(self.config, 'route_progress_cte_gate_width', 0.0)))
        if gate_width > 0.0:
            gate_start = max(0.0, float(getattr(self.config, 'route_progress_cte_gate_start', 0.0)))
            gate_floor = float(np.clip(getattr(self.config, 'route_progress_cte_gate_floor', 0.25), 0.0, 1.0))
            abs_cross_track = abs(float(cross_track))
            cte_ratio = np.clip((abs_cross_track - gate_start) / max(gate_width - gate_start, 1e-3), 0.0, 1.0)
            cte_gate = 1.0 - ((1.0 - gate_floor) * cte_ratio)
            route_progress *= float(np.clip(cte_gate, gate_floor, 1.0))

        conflict_phase = 0.0
        conflict_eta = 1.0
        eta_seconds = 1e3
        to_conflict = 1e3
        if conflict_point is not None:
            conflict_x, conflict_y = conflict_point
            conflict_s = ((float(conflict_x) - float(spawn.x)) * unit_x) + ((float(conflict_y) - float(spawn.y)) * unit_y)
            conflict_s = float(np.clip(conflict_s, 0.0, route_length))
            to_conflict = conflict_s - progress_s
            conflict_phase = float(np.clip((progress_s - conflict_s) / max(0.5 * route_length, 1e-3), -1.0, 1.0))
            along_speed = max(
                0.0,
                float(observation.speed) * math.cos(float(observation.heading_error)),
                float(observation.final_linear_x),
                float(observation.raw_linear_x),
            )
            eta_seconds = max(0.0, to_conflict) / max(along_speed, 0.03)
            conflict_eta = float(np.clip(eta_seconds / 25.0, 0.0, 1.0))

        return {
            'cross_track_error': float(np.clip(cross_track, -clip_range, clip_range)),
            'route_progress': route_progress,
            'conflict_phase': conflict_phase,
            'conflict_eta': conflict_eta,
            'eta_seconds': float(eta_seconds),
            'to_conflict': float(to_conflict),
        }

    def _annotate_route_features_for_observations(self, observations: Dict[str, AgentLocalObservation]) -> None:
        """Inject route progress and crossing timing features into observations."""
        conflict_point = self._scenario_conflict_point()
        stats: dict[str, dict[str, float]] = {}
        for agent_id, observation in observations.items():
            stat = self._route_timing_features(agent_id, observation, conflict_point)
            stats[agent_id] = stat
            observation.cross_track_error = stat['cross_track_error']
            observation.route_progress = stat['route_progress']
            observation.conflict_phase = stat['conflict_phase']
            observation.conflict_eta = stat['conflict_eta']
            observation.crossing_priority = 0.0
            observation.crossing_eta_gap = 1.0

        if conflict_point is None or len(stats) <= 1:
            self._annotate_random_priority_for_observations(observations)
            return

        ordered_ids = sorted(
            stats,
            key=lambda agent_id: (
                -1.0 if stats[agent_id]['to_conflict'] < -self.config.collision_distance else stats[agent_id]['eta_seconds'],
                self._crossing_priority_tiebreak(agent_id),
            ),
        )
        rank_by_id = {agent_id: rank for rank, agent_id in enumerate(ordered_ids)}
        denominator = max(1, len(ordered_ids) - 1)
        target_gap = max(1.0, float(self.config.reward.crossing_time_gap_target))
        for agent_id, observation in observations.items():
            rank = rank_by_id.get(agent_id, denominator)
            observation.crossing_priority = float(1.0 - (2.0 * rank / denominator))
            own_eta = stats[agent_id]['eta_seconds']
            eta_gaps = [abs(own_eta - stats[other_id]['eta_seconds']) for other_id in stats if other_id != agent_id]
            min_gap = min(eta_gaps) if eta_gaps else target_gap
            observation.crossing_eta_gap = float(np.clip(min_gap / target_gap, 0.0, 1.0))

        for agent_id, observation in observations.items():
            own_eta = stats[agent_id]['eta_seconds']
            own_priority = float(np.clip(observation.crossing_priority, -1.0, 1.0))
            for neighbor in observation.neighbors:
                other_id = neighbor.source_id
                if other_id not in stats or other_id not in observations:
                    neighbor.route_eta_delta = 0.0
                    neighbor.route_priority_delta = 0.0
                    continue
                other_eta = stats[other_id]['eta_seconds']
                other_priority = float(np.clip(observations[other_id].crossing_priority, -1.0, 1.0))
                # Positive ETA delta means this agent is currently later than
                # the neighbour at the shared conflict point; negative means
                # this agent is earlier.  The policy still chooses the action.
                neighbor.route_eta_delta = float(np.clip((own_eta - other_eta) / target_gap, -1.0, 1.0))
                neighbor.route_priority_delta = float(np.clip((own_priority - other_priority) * 0.5, -1.0, 1.0))

    def _annotate_cross_track_errors(self, observations: Dict[str, AgentLocalObservation], global_state: FleetGlobalState | None = None) -> None:
        """Inject route features into local observations and critic global-state observations."""
        self._annotate_route_features_for_observations(observations)
        if global_state is not None:
            self._annotate_route_features_for_observations(global_state.local_observations)

    def _annotate_encounter_types(self, observations, global_state):
        scenario_name = self._scenario.name if self._scenario else ''
        encounter_idx = _SCENARIO_ENCOUNTER_MAP.get(scenario_name, -1)
        if encounter_idx < 0:
            return
        # Encounter-type dropout: with configured probability, skip annotation
        # so the one-hot stays all-zeros, matching deployment conditions.
        if self.config.encounter_type_dropout > 0.0 and self._rng.random() < self.config.encounter_type_dropout:
            return
        for obs in observations.values():
            obs.encounter_type_index = encounter_idx
        if global_state is not None:
            for obs in global_state.local_observations.values():
                obs.encounter_type_index = encounter_idx

    def _build_episode_info(self, global_state: FleetGlobalState) -> dict:
        return {
            'scenario': self._scenario.name if self._scenario is not None else 'unknown',
            'active_agent_ids': list(self._active_agent_ids),
            'inactive_agent_ids': list(self.inactive_agent_ids),
            'runtime_agent_ids': list(self._runtime_agent_ids),
            'pairwise_min_separation': global_state.team_min_separation,
            'pairwise_mean_separation': global_state.team_mean_separation,
            'goal_completion_ratio': global_state.goal_completion_ratio,
            'team_mean_goal_distance': global_state.team_mean_goal_distance,
            'global_state': global_state.to_vector(self.config.max_agents, self.config.max_neighbors),
        }

    def _effective_pair_min_separation(
        self,
        observations: Dict[str, AgentLocalObservation],
        global_state: FleetGlobalState,
    ) -> float:
        """Minimum active-agent separation, including scripted/background neighbours.

        FleetGlobalState only measures pairwise separation among selected active
        observations.  Background tracks are encoded as neighbours, not agents,
        so rear-only overtaking needs this additional check to make the slow
        lead vessel count for collision and near-miss logic.
        """
        pair_min = float(global_state.team_min_separation)
        for agent_id in self._active_agent_ids:
            observation = observations.get(agent_id)
            if observation is None:
                continue
            pair_min = min(pair_min, float(observation.min_neighbor_distance()))
        return pair_min

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

            bearing_floor = max(0.0, min(1.0, float(self.config.reward.conflict_bearing_floor)))
            forward_factor = max(0.0, bearing_floor + (1.0 - bearing_floor) * float(np.cos(neighbor.bearing)))
            if forward_factor <= 0.0:
                continue

            proximity = (conflict_distance - neighbor.distance) / conflict_distance
            neighbor_risk = proximity * range_rate * forward_factor
            max_risk = max(max_risk, neighbor_risk)

        return max_risk

    def _compute_clear_ahead_gate(self, observation: AgentLocalObservation) -> float:
        clear_distance = max(0.0, float(self.config.reward.clear_ahead_distance))
        if clear_distance <= 0.0:
            return 0.0

        half_angle = math.radians(max(1.0, min(179.0, float(self.config.reward.clear_ahead_bearing_deg))))
        closest_front_blocker = float('inf')
        for neighbor in observation.neighbors:
            if neighbor.distance <= 1e-3:
                continue
            if abs(float(neighbor.bearing)) <= half_angle:
                closest_front_blocker = min(closest_front_blocker, float(neighbor.distance))

        if closest_front_blocker < clear_distance:
            return 0.0

        front_margin = clear_distance if not np.isfinite(closest_front_blocker) else closest_front_blocker - clear_distance
        return min(1.0, max(0.0, front_margin / max(0.5 * clear_distance, 1e-3)))

    @staticmethod
    def _neighbor_cpa_metrics(neighbor) -> Dict[str, float]:
        rel_x = float(neighbor.rel_x)
        rel_y = float(neighbor.rel_y)
        rel_vx = float(neighbor.rel_vx)
        rel_vy = float(neighbor.rel_vy)
        distance = max(float(neighbor.distance), math.hypot(rel_x, rel_y), 1e-3)
        dot = (rel_x * rel_vx) + (rel_y * rel_vy)
        range_rate = -dot / distance
        rel_speed_sq = (rel_vx * rel_vx) + (rel_vy * rel_vy)
        if rel_speed_sq <= 1e-6:
            return {
                'distance': distance,
                'range_rate': range_rate,
                'tcpa': float('inf'),
                'dcpa': distance,
            }

        tcpa = -dot / rel_speed_sq
        if tcpa <= 0.0:
            dcpa = distance
        else:
            cpa_x = rel_x + rel_vx * tcpa
            cpa_y = rel_y + rel_vy * tcpa
            dcpa = math.hypot(cpa_x, cpa_y)
        return {
            'distance': distance,
            'range_rate': range_rate,
            'tcpa': tcpa,
            'dcpa': dcpa,
        }

    def _collect_cpa_metrics(self, observation: AgentLocalObservation) -> Dict[str, Dict[str, float]]:
        return {
            neighbor.source_id: self._neighbor_cpa_metrics(neighbor)
            for neighbor in observation.neighbors
        }

    @staticmethod
    def _agent_priority_key(agent_id: str) -> tuple[int, str]:
        text = str(agent_id)
        try:
            return int(text.rsplit('_', 1)[-1]), text
        except (TypeError, ValueError):
            return 1000, text

    def _agent_has_random_yield_role(self, agent_id: str, neighbor_id: str) -> bool:
        if self._scenario is None or 'random_encounter' not in self._scenario.name:
            return False
        return self._agent_priority_key(agent_id) > self._agent_priority_key(neighbor_id)

    def _compute_anticipatory_cpa_reward(
        self,
        agent_id: str,
        observation: AgentLocalObservation,
        current_forward_speed: float,
    ) -> float:
        reward_cfg = self.config.reward
        active_weights = (
            reward_cfg.anticipatory_dcpa_deficit_penalty_weight,
            reward_cfg.anticipatory_dcpa_improvement_reward_weight,
            reward_cfg.anticipatory_closing_reduction_reward_weight,
            reward_cfg.anticipatory_yield_speed_penalty_weight,
        )
        if not observation.neighbors or max(active_weights) <= 0.0:
            return 0.0

        lookahead_distance = max(
            float(reward_cfg.anticipatory_cpa_distance),
            self.config.near_miss_distance,
            self.config.collision_distance + 1e-3,
        )
        if lookahead_distance <= self.config.collision_distance + 1e-3:
            return 0.0

        horizon = max(1.0, float(reward_cfg.anticipatory_cpa_time_horizon))
        dcpa_target = max(self.config.collision_distance + 0.05, float(reward_cfg.anticipatory_dcpa_target))
        previous_metrics_by_neighbor = self._previous_cpa_metrics.get(agent_id, {})
        closing_reference = max(0.35 * self.config.cruise_speed, 0.05)
        dcpa_reference = max(0.20 * dcpa_target, 0.05)
        cruise_reference = max(0.18, self._pure_linear_speed_limit())
        yield_speed = max(0.02, min(float(reward_cfg.anticipatory_yield_speed), cruise_reference))
        cpa_reward = 0.0

        for neighbor in observation.neighbors:
            metrics = self._neighbor_cpa_metrics(neighbor)
            distance = metrics['distance']
            if distance <= self.config.collision_distance or distance >= lookahead_distance:
                continue
            range_rate = metrics['range_rate']
            tcpa = metrics['tcpa']
            dcpa = metrics['dcpa']
            if range_rate <= 0.02 or not np.isfinite(tcpa) or tcpa <= 0.0 or tcpa > horizon:
                continue

            dcpa_deficit = max(0.0, dcpa_target - dcpa) / max(dcpa_target, 1e-3)
            if dcpa_deficit <= 0.0:
                continue

            distance_gate = (lookahead_distance - distance) / max(lookahead_distance - self.config.collision_distance, 1e-3)
            time_gate = (horizon - tcpa) / horizon
            closing_gate = min(1.0, range_rate / closing_reference)
            threat_gate = max(distance_gate, time_gate) * closing_gate * (0.35 + 0.65 * dcpa_deficit)

            cpa_reward -= reward_cfg.anticipatory_dcpa_deficit_penalty_weight * threat_gate * (dcpa_deficit ** 2)

            previous_metrics = previous_metrics_by_neighbor.get(neighbor.source_id)
            if previous_metrics is not None:
                dcpa_delta = dcpa - previous_metrics.get('dcpa', dcpa)
                if dcpa_delta > 0.0:
                    dcpa_improvement = min(1.0, dcpa_delta / dcpa_reference)
                    cpa_reward += reward_cfg.anticipatory_dcpa_improvement_reward_weight * threat_gate * dcpa_improvement

                closing_delta = previous_metrics.get('range_rate', range_rate) - range_rate
                if closing_delta > 0.0:
                    closing_reduction = min(1.0, closing_delta / closing_reference)
                    cpa_reward += reward_cfg.anticipatory_closing_reduction_reward_weight * threat_gate * closing_reduction

            if reward_cfg.anticipatory_yield_speed_penalty_weight > 0.0 and self._agent_has_random_yield_role(agent_id, neighbor.source_id):
                speed_excess_ratio = max(
                    0.0,
                    min(1.0, (current_forward_speed - yield_speed) / max(cruise_reference - yield_speed, 1e-3)),
                )
                cpa_reward -= reward_cfg.anticipatory_yield_speed_penalty_weight * threat_gate * speed_excess_ratio

        return cpa_reward

    def _compute_route_cross_track_error(
        self,
        agent_id: str,
        observation: AgentLocalObservation,
    ) -> float:
        if self._scenario is None:
            return 0.0

        spawn = self._scenario.agent_spawns.get(agent_id)
        goal = self._scenario.agent_goals.get(agent_id)
        if spawn is None or goal is None:
            return 0.0

        route_dx = float(goal.x - spawn.x)
        route_dy = float(goal.y - spawn.y)
        route_length = float(np.hypot(route_dx, route_dy))
        if route_length <= 1e-6:
            return 0.0

        relative_x = float(observation.pose_x - spawn.x)
        relative_y = float(observation.pose_y - spawn.y)
        cross_track = ((relative_x * route_dy) - (relative_y * route_dx)) / route_length
        clip_range = max(1.0, self.config.cte_clip_range)
        return float(np.clip(cross_track, -clip_range, clip_range))

    def _effective_near_miss_distance(self) -> float:
        near_miss_distance = float(self.config.near_miss_distance)
        if self._scenario is None:
            return near_miss_distance
        if self._scenario.name != 'two_usv_head_on':
            return near_miss_distance
        head_on_override = float(getattr(self.config.reward, 'head_on_near_miss_distance', 0.0))
        if head_on_override <= 0.0:
            return near_miss_distance
        # Always keep a non-zero safety band above collision threshold.
        return max(head_on_override, self.config.collision_distance + 1e-3)

    def _compute_head_on_guidance_reward(self, observation: AgentLocalObservation, previous_forward_speed: float) -> float:
        guidance_distance = max(
            self.config.reward.head_on_guidance_distance,
            self.config.reward.anticipation_distance,
            self.config.collision_distance + 1e-3,
        )
        target_offset = max(0.2, self.config.reward.head_on_target_starboard_offset)
        own_speed = max(0.0, float(observation.speed), float(observation.final_linear_x), float(observation.raw_linear_x))

        best_neighbor = None
        best_score = -1.0
        best_range_rate = 0.0
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
                best_range_rate = float(range_rate)

        if best_neighbor is None:
            return 0.0

        proximity = max(0.0, min(1.0, (guidance_distance - best_neighbor.distance) / guidance_distance))
        starboard_offset = max(0.0, -best_neighbor.rel_y)
        corridor_progress = max(0.0, min(1.0, starboard_offset / target_offset))
        centerline_penalty = max(0.0, 1.0 - corridor_progress)
        desired_starboard_turn = 0.10 + 0.16 * proximity
        actual_starboard_turn = max(0.0, -observation.final_angular_z)
        turn_progress = max(0.0, min(1.0, actual_starboard_turn / max(desired_starboard_turn, 1e-3)))
        desired_forward_speed = self._target_forward_speed(observation, conflict_level=proximity)
        current_forward_speed = max(0.0, observation.final_linear_x)
        forward_progress = max(0.0, min(1.0, current_forward_speed / max(desired_forward_speed, 1e-3)))
        speed_drop = max(0.0, previous_forward_speed - current_forward_speed)
        speed_drop_ratio = max(0.0, min(1.0, speed_drop / max(desired_forward_speed, 1e-3)))
        closing_weight = max(0.0, min(1.0, (best_range_rate - 0.05) / 0.75))
        urgency = proximity * (0.45 + 0.55 * closing_weight)
        close_centerline_penalty = urgency * centerline_penalty * max(0.35, forward_progress)
        no_turn_penalty = urgency * centerline_penalty * max(0.0, 1.0 - turn_progress)

        phase_gate = self.config.reward.head_on_phase_gate_strength
        forward_gate = max(0.0, min(1.0, 1.0 - phase_gate * (1.0 - corridor_progress)))

        return (
            self.config.reward.head_on_corridor_reward_weight * proximity * corridor_progress
            - self.config.reward.head_on_centerline_penalty_weight * proximity * centerline_penalty
            + self.config.reward.head_on_turn_reward_weight * proximity * turn_progress
            + self.config.reward.head_on_forward_reward_weight * proximity * forward_gate * forward_progress
            - self.config.reward.head_on_speed_drop_penalty_weight * proximity * forward_gate * speed_drop_ratio
            - self.config.reward.head_on_close_penalty_weight * close_centerline_penalty
            - self.config.reward.head_on_no_turn_penalty_weight * no_turn_penalty
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
        desired_forward_speed = self._target_forward_speed(observation, conflict_level=0.45)
        forward_progress = max(0.0, min(1.0, current_forward_speed / max(desired_forward_speed, 1e-3)))
        cruise_reference = max(0.18, self._pure_linear_speed_limit())

        best_crossing_reward = 0.0
        best_crossing_forward_reward = 0.0
        best_crossing_slowdown_reward = 0.0
        max_crossing_overspeed_penalty = 0.0
        max_crossing_close_forward_penalty = 0.0
        best_overtaking_reward = 0.0
        best_overtaking_forward_reward = 0.0
        best_overtaking_corridor_reward = 0.0
        max_overtaking_centerline_penalty = 0.0
        max_overtaking_close_penalty = 0.0
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
            co_directional_close = (
                same_lane_ahead
                and own_speed > 0.10
                and neighbor_forward_speed > 0.05
                and abs(body_vx) < 0.15
                and neighbor.distance < lookahead_distance * 0.7
            )
            if co_directional_close and not overtaking_target:
                overtaking_target = True
            starboard_crossing = body_y < -0.35 and closing_speed > -0.05
            scenario_crossing_conflict = (
                observation.encounter_type_index == 1
                and closing_speed > -0.10
                and (body_x > 0.2 or neighbor.distance < lookahead_distance * 0.65)
            )
            crossing_conflict = starboard_crossing or scenario_crossing_conflict

            proximity = max(0.0, min(1.0, (lookahead_distance - neighbor.distance) / lookahead_distance))
            closing_weight = max(0.0, min(1.0, (closing_speed + 0.15) / 0.9))
            desired_starboard_turn = 0.05 + 0.12 * proximity + 0.05 * closing_weight
            turn_progress = max(0.0, min(1.0, actual_starboard_turn / max(desired_starboard_turn, 1e-3)))
            wrong_way_progress = max(0.0, min(1.0, actual_port_turn / max(desired_starboard_turn, 1e-3)))

            if crossing_conflict:
                crossing_yield_speed = max(0.02, min(float(self.config.reward.crossing_yield_speed), desired_forward_speed))
                speed_excess = max(0.0, current_forward_speed - crossing_yield_speed)
                speed_excess_ratio = max(
                    0.0,
                    min(1.0, speed_excess / max(cruise_reference - crossing_yield_speed, 1e-3)),
                )
                yield_compliance = max(0.0, 1.0 - speed_excess_ratio)
                best_crossing_reward = max(best_crossing_reward, proximity * turn_progress)
                best_crossing_forward_reward = max(
                    best_crossing_forward_reward,
                    proximity * turn_progress * forward_progress,
                )
                best_crossing_slowdown_reward = max(
                    best_crossing_slowdown_reward,
                    proximity * closing_weight * yield_compliance * (0.35 + 0.65 * turn_progress),
                )
                max_crossing_overspeed_penalty = max(
                    max_crossing_overspeed_penalty,
                    proximity * closing_weight * speed_excess_ratio,
                )
                max_crossing_close_forward_penalty = max(
                    max_crossing_close_forward_penalty,
                    proximity * closing_weight * forward_progress * (0.35 + 0.65 * max(0.0, 1.0 - turn_progress)),
                )
                max_wrong_way_penalty = max(max_wrong_way_penalty, proximity * wrong_way_progress)
                continue

            if overtaking_target:
                target_offset = 0.55 + 0.35 * proximity
                starboard_offset = max(0.0, -body_y)
                corridor_progress = max(0.0, min(1.0, starboard_offset / max(target_offset, 1e-3)))
                centerline_penalty = max(0.0, 1.0 - corridor_progress)
                close_penalty = proximity * closing_weight * centerline_penalty * forward_progress
                best_overtaking_reward = max(best_overtaking_reward, proximity * turn_progress)
                best_overtaking_forward_reward = max(
                    best_overtaking_forward_reward,
                    proximity * turn_progress * forward_progress,
                )
                best_overtaking_corridor_reward = max(
                    best_overtaking_corridor_reward,
                    proximity * corridor_progress * max(turn_progress, 0.35),
                )
                max_overtaking_centerline_penalty = max(
                    max_overtaking_centerline_penalty,
                    proximity * centerline_penalty,
                )
                max_overtaking_close_penalty = max(
                    max_overtaking_close_penalty,
                    close_penalty,
                )
                max_wrong_way_penalty = max(max_wrong_way_penalty, proximity * wrong_way_progress)

        return (
            self.config.reward.crossing_starboard_turn_reward_weight * best_crossing_reward
            + self.config.reward.crossing_forward_reward_weight * best_crossing_forward_reward
            + self.config.reward.crossing_slowdown_reward_weight * best_crossing_slowdown_reward
            - self.config.reward.crossing_overspeed_penalty_weight * max_crossing_overspeed_penalty
            - self.config.reward.crossing_close_forward_penalty_weight * max_crossing_close_forward_penalty
            + self.config.reward.overtaking_starboard_turn_reward_weight * best_overtaking_reward
            + self.config.reward.overtaking_forward_reward_weight * best_overtaking_forward_reward
            + self.config.reward.overtaking_corridor_reward_weight * best_overtaking_corridor_reward
            - self.config.reward.overtaking_centerline_penalty_weight * max_overtaking_centerline_penalty
            - self.config.reward.overtaking_close_penalty_weight * max_overtaking_close_penalty
            - self.config.reward.colregs_port_turn_penalty_weight * max_wrong_way_penalty
        )

    def _compute_crossing_time_coordination_reward(self, observation: AgentLocalObservation) -> float:
        reward_weight = float(self.config.reward.crossing_time_separation_reward_weight)
        penalty_weight = float(self.config.reward.crossing_time_separation_penalty_weight)
        if reward_weight <= 0.0 and penalty_weight <= 0.0:
            return 0.0
        if self._scenario is None or self._scenario.name != 'three_usv_crossing':
            return 0.0
        if observation.conflict_phase > 0.20:
            return 0.0

        approach_gate = max(0.0, 1.0 - float(np.clip(observation.conflict_eta, 0.0, 1.0)))
        gap_deficit = max(0.0, 1.0 - float(np.clip(observation.crossing_eta_gap, 0.0, 1.0)))
        if approach_gate <= 0.0 or gap_deficit <= 0.0:
            return 0.0

        role = float(np.clip(observation.crossing_priority, -1.0, 1.0))
        # role≈+1: clear first; role≈-1: yield/hold gap. Middle roles blend both.
        clear_role = max(0.0, min(1.0, (role + 0.20) / 1.20))
        yield_role = max(0.0, min(1.0, (0.50 - role) / 1.50))
        current_forward_speed = max(0.0, observation.final_linear_x)
        cruise_reference = max(0.18, self._pure_linear_speed_limit())
        speed_ratio = max(0.0, min(1.0, current_forward_speed / cruise_reference))
        crossing_yield_speed = max(0.02, min(float(self.config.reward.crossing_yield_speed), cruise_reference))
        speed_excess_ratio = max(
            0.0,
            min(1.0, (current_forward_speed - crossing_yield_speed) / max(cruise_reference - crossing_yield_speed, 1e-3)),
        )
        yield_compliance = max(0.0, 1.0 - speed_excess_ratio)
        clear_stall_ratio = max(0.0, (crossing_yield_speed - current_forward_speed) / max(crossing_yield_speed, 1e-3))

        shaping_gate = approach_gate * gap_deficit
        coordination_reward = reward_weight * shaping_gate * (
            clear_role * speed_ratio
            + yield_role * yield_compliance
        )
        coordination_penalty = penalty_weight * shaping_gate * (
            yield_role * speed_excess_ratio
            + 0.35 * clear_role * clear_stall_ratio
        )
        return coordination_reward - coordination_penalty

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        options = options or {}
        scenario_kind = options.get('scenario_kind')
        if not scenario_kind:
            scenario_kind = self.config.default_scenarios[self._episode_index % len(self.config.default_scenarios)]

        self._scenario = MultiAgentScenarioFactory.create(
            scenario_kind,
            self._runtime_agent_ids,
            goal_distance=self.config.goal_distance,
            neighbor_speed=self.config.scenario_neighbor_speed,
            rng=self._rng,
            spawn_position_std=self.config.scenario_spawn_position_std,
            spawn_heading_std=self.config.scenario_spawn_heading_std,
            goal_position_std=self.config.scenario_goal_position_std,
        )
        self._refresh_active_agent_ids()

        for controller in self._controllers.values():
            controller.reset_for_training_episode()

        # Tau domain randomization: sample new time constants each episode
        if (self.config.dr_tau_linear_low > 0 and self.config.dr_tau_linear_high > self.config.dr_tau_linear_low):
            self._sim_node.randomize_tau(
                self.config.dr_tau_linear_low,
                self.config.dr_tau_linear_high,
                self.config.dr_tau_angular_low if self.config.dr_tau_angular_high > self.config.dr_tau_angular_low else self.config.sim_tau_angular,
                self.config.dr_tau_angular_high if self.config.dr_tau_angular_high > self.config.dr_tau_angular_low else self.config.sim_tau_angular,
            )

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
        self._annotate_encounter_types(observations, global_state)
        self._annotate_cross_track_errors(observations, global_state)
        self._latest_observations = observations
        self._previous_distances = {
            namespace: observations[namespace].distance_to_goal
            for namespace in self._active_agent_ids
        }
        self._previous_actions = {namespace: self.zero_policy_action() for namespace in self._active_agent_ids}
        self._previous_forward_speeds = {
            namespace: max(0.0, observations[namespace].final_linear_x)
            for namespace in self._active_agent_ids
        }
        self._previous_conflict_risks = {
            namespace: self._compute_conflict_risk(observations[namespace])
            for namespace in self._active_agent_ids
        }
        self._previous_cpa_metrics = {
            namespace: self._collect_cpa_metrics(observations[namespace])
            for namespace in self._active_agent_ids
        }
        self._previous_pair_min: float = float('inf')
        self._entanglement_steps: int = 0
        self._episode_start = time.monotonic()
        self._last_team_progress_time = self._episode_start
        self._best_team_mean_distance = global_state.team_mean_goal_distance
        self._initial_team_mean_separation = global_state.team_mean_separation
        self._initial_team_mean_goal_distance = self._scenario_initial_team_mean_goal_distance()
        self._waypoints_completed = {namespace: 0 for namespace in self._active_agent_ids}
        self._episode_index += 1

        active_observations = self._filter_active_observations(observations)
        return (
            {namespace: observation.to_vector(self.config.max_neighbors) for namespace, observation in active_observations.items()},
            self._build_episode_info(global_state),
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

        pair_min = min(float(global_state.team_min_separation), float(observation.min_neighbor_distance()))

        safety = 0.0
        conflict_risk = self._compute_conflict_risk(observation)
        previous_conflict_risk = self._previous_conflict_risks.get(agent_id, conflict_risk)
        near_miss_distance = self._effective_near_miss_distance()

        if pair_min < self.config.collision_distance:
            safety += self.config.reward.collision_penalty
        elif pair_min < near_miss_distance:
            near_miss_band = max(near_miss_distance - self.config.collision_distance, 1e-3)
            normalized_proximity = (near_miss_distance - pair_min) / near_miss_band
            safety -= self.config.reward.near_miss_weight * (normalized_proximity ** self.config.reward.near_miss_exponent) * near_miss_band
        conflict_relief_scale = max(0.35, 1.0 - (self.config.goal_proximity_conflict_relief * goal_proximity))
        safety -= self.config.reward.conflict_risk_weight * conflict_risk * conflict_relief_scale

        conflict_level = min(conflict_risk, 1.0) * conflict_relief_scale
        conflict_active = min(1.0, max(previous_conflict_risk, conflict_risk)) * conflict_relief_scale
        current_forward_speed = max(0.0, observation.final_linear_x)
        desired_conflict_speed = self._target_forward_speed(
            observation,
            conflict_level=conflict_level,
            goal_proximity=goal_proximity,
        )
        if progress > 0.0:
            progress_gate = 1.0 - (self.config.reward.conflict_progress_scale * max(0.0, conflict_active - 0.25))
            progress *= max(0.2, progress_gate)
        speed_deficit = max(0.0, desired_conflict_speed - current_forward_speed)
        speed_drop = max(0.0, self._previous_forward_speeds.get(agent_id, 0.0) - current_forward_speed)
        risk_drop = max(0.0, previous_conflict_risk - conflict_risk)
        risk_rise = max(0.0, conflict_risk - previous_conflict_risk)
        near_miss_band = max(near_miss_distance - self.config.collision_distance, 1e-3)
        near_miss_ratio = max(
            0.0,
            min(1.0, (near_miss_distance - pair_min) / near_miss_band),
        )
        unsafe_speed_excess = max(0.0, current_forward_speed - (0.85 * desired_conflict_speed))

        # Avoidance turn reward: positive reward for turning *away* from the
        # nearest neighbour when inside the near-miss zone.  This provides a
        # physics-based escape signal that complements (and can override)
        # COLREGs-specific starboard turn guidance when the neighbour is on
        # the starboard side.
        avoidance_turn = 0.0
        if self.config.reward.avoidance_turn_reward_weight > 0.0 and observation.neighbors:
            nearest = min(observation.neighbors, key=lambda n: n.distance)
            if nearest.distance < near_miss_distance and nearest.distance > self.config.collision_distance:
                proximity = (near_miss_distance - nearest.distance) / max(near_miss_band, 1e-3)
                proximity = min(1.0, proximity)
                current_omega = float(action[1])
                # bearing > 0 means neighbour is to the left  → turn right (ω < 0) to escape
                # bearing < 0 means neighbour is to the right → turn left  (ω > 0) to escape
                # Reward = proximity × alignment between ω and -bearing
                bearing_sign = -1.0 if nearest.bearing > 0.0 else (1.0 if nearest.bearing < 0.0 else 0.0)
                turn_alignment = bearing_sign * current_omega  # positive when turning away
                # Saturate at ω=0.25 (not 0.15) to encourage stronger evasive turns.
                avoidance_turn = self.config.reward.avoidance_turn_reward_weight * proximity * max(0.0, min(1.0, turn_alignment / 0.25))

        anticipatory_reward_weight = self.config.reward.anticipatory_avoidance_turn_reward_weight
        anticipatory_penalty_weight = self.config.reward.anticipatory_avoidance_turn_penalty_weight
        if (anticipatory_reward_weight > 0.0 or anticipatory_penalty_weight > 0.0) and observation.neighbors:
            turn_distance = max(near_miss_distance, self.config.reward.anticipatory_avoidance_turn_distance)
            best_gate = 0.0
            best_turn_sign = 0.0
            for neighbor in observation.neighbors:
                if neighbor.distance <= self.config.collision_distance or neighbor.distance >= turn_distance:
                    continue
                range_rate = -(
                    (neighbor.rel_x * neighbor.rel_vx) + (neighbor.rel_y * neighbor.rel_vy)
                ) / max(neighbor.distance, 1e-3)
                if range_rate <= 0.0:
                    continue
                distance_gate = (turn_distance - neighbor.distance) / max(turn_distance - self.config.collision_distance, 1e-3)
                closing_gate = min(1.0, range_rate / max(0.35 * self.config.cruise_speed, 0.05))
                time_to_collision = (neighbor.distance - self.config.collision_distance) / max(range_rate, 1e-3)
                time_gate = max(0.0, min(1.0, (7.0 - time_to_collision) / 7.0))
                urgency_gate = max(distance_gate * closing_gate, time_gate * closing_gate)
                if urgency_gate > best_gate:
                    best_gate = urgency_gate
                    best_turn_sign = -1.0 if neighbor.bearing >= 0.0 else 1.0
            if best_gate > 0.0 and best_turn_sign != 0.0:
                current_omega = float(action[1])
                turn_alignment = best_turn_sign * current_omega
                normalized_alignment = max(-1.0, min(1.0, turn_alignment / 0.25))
                avoidance_turn += anticipatory_reward_weight * best_gate * max(0.0, normalized_alignment)
                turn_shortfall = max(0.0, 0.14 - turn_alignment) / 0.14
                avoidance_turn -= anticipatory_penalty_weight * best_gate * min(1.5, turn_shortfall)

        braking = -self.config.reward.conflict_brake_weight * conflict_level * speed_deficit
        braking -= self.config.reward.stop_go_penalty_weight * conflict_level * speed_drop
        braking += self.config.reward.conflict_resolution_reward_weight * conflict_active * risk_drop
        braking -= self.config.reward.conflict_escalation_penalty_weight * conflict_active * risk_rise
        braking -= self.config.reward.unsafe_close_speed_penalty_weight * near_miss_ratio * unsafe_speed_excess

        # Conflict overspeed penalty: penalise exceeding desired_conflict_speed
        # during active conflict.  Counteracts the asymmetry where only speed
        # deficit (too slow) was penalised, encouraging the policy to slow down.
        if self.config.reward.conflict_overspeed_penalty_weight > 0.0 and conflict_level > 0.25:
            overspeed = max(0.0, current_forward_speed - desired_conflict_speed)
            braking -= self.config.reward.conflict_overspeed_penalty_weight * conflict_level * overspeed

        braking += conflict_relief_scale * (
            self._compute_head_on_guidance_reward(
                observation,
                self._previous_forward_speeds.get(agent_id, current_forward_speed),
            )
            + self._compute_crossing_overtaking_guidance_reward(observation)
        )
        braking += self._compute_crossing_time_coordination_reward(observation)
        progress += self._pure_goal_tracking_reward(
            observation,
            conflict_level=conflict_level,
            goal_proximity=goal_proximity,
        )
        # Dense near-goal idle penalty: penalise near-zero speed when close to
        # (but not at) the goal to prevent the "hover near goal" exploit.
        if self.config.reward.near_goal_idle_penalty_weight > 0.0 and goal_proximity > 0.0:
            min_approach_speed = 0.10
            near_goal_speed_deficit = max(0.0, min_approach_speed - current_forward_speed) / min_approach_speed
            progress -= self.config.reward.near_goal_idle_penalty_weight * goal_proximity * (near_goal_speed_deficit ** 2)
        cross_track_error = self._compute_route_cross_track_error(agent_id, observation)
        path_tolerance = (
            self.config.reward.path_deviation_tolerance
            + self.config.reward.path_deviation_conflict_scale * conflict_level
        )
        path_deviation_excess = max(0.0, abs(cross_track_error) - path_tolerance)
        if path_deviation_excess > 0.0:
            progress -= self.config.reward.path_deviation_penalty_weight * path_deviation_excess

        front_clear_gate = self._compute_clear_ahead_gate(observation)
        low_conflict_gate = max(0.0, 1.0 - (min(conflict_risk, 1.0) / 0.45))
        clear_ahead_gate = front_clear_gate * low_conflict_gate
        clear_ahead_route_gate = clear_ahead_gate * max(0.0, 1.0 - 0.65 * goal_proximity)
        clear_ahead_heading_penalty = 0.0
        if clear_ahead_route_gate > 0.0:
            abs_cte = abs(cross_track_error)
            abs_heading_error = abs(observation.heading_error)
            if self.config.reward.clear_ahead_cte_weight > 0.0:
                progress -= self.config.reward.clear_ahead_cte_weight * clear_ahead_route_gate * abs_cte
            if self.config.reward.clear_ahead_heading_weight > 0.0:
                heading_scale = max(
                    0.1,
                    1.0 - (self.config.reward.heading_relief_factor * min(conflict_risk, 1.0)) - (self.config.goal_proximity_heading_relief * goal_proximity),
                )
                clear_ahead_heading_penalty = self.config.reward.clear_ahead_heading_weight * clear_ahead_route_gate * heading_scale * abs_heading_error

        smoothness_scale = max(0.1, 1.0 - (self.config.goal_proximity_smoothness_relief * goal_proximity))
        smoothness = -(self.config.reward.action_smoothness_weight * smoothness_scale) * float(
            np.linalg.norm(action - self._previous_actions[agent_id])
        )
        if self.config.reward.angular_accel_penalty_weight > 0.0:
            angular_change = abs(float(action[1]) - float(self._previous_actions[agent_id][1]))
            smoothness -= self.config.reward.angular_accel_penalty_weight * smoothness_scale * angular_change

        # Forward-speed oscillation penalty: penalise |vx_t - vx_{t-1}| globally
        # (not gated by conflict) to prevent linear bang-bang behaviour.
        if self.config.reward.forward_speed_change_penalty_weight > 0.0:
            speed_change = abs(current_forward_speed - self._previous_forward_speeds.get(agent_id, current_forward_speed))
            smoothness -= self.config.reward.forward_speed_change_penalty_weight * smoothness_scale * speed_change

        # Proportional angular velocity penalty: penalise large |ω| when heading
        # error is small.  The alignment factor (cos of heading error) is high when
        # the USV is roughly pointing at the goal – exactly the situation where a
        # large angular velocity is undesirable and produces S-curves.
        if self.config.reward.straight_line_omega_penalty_weight > 0.0:
            alignment = max(0.0, math.cos(min(abs(observation.heading_error), math.pi / 2.0)))
            abs_omega = abs(float(action[1]))
            # Scale penalty down during conflicts, but keep a configurable floor
            # (default 0.1 is too low and nearly disables the penalty during avoidance).
            conflict_floor = max(0.1, self.config.reward.straight_line_omega_conflict_floor)
            conflict_gate = max(conflict_floor, 1.0 - min(conflict_risk, 1.0))
            # CTE gate: reduce penalty when cross-track error is large so the
            # agent is free to turn for path recovery.
            cte_gate = 1.0
            if self.config.reward.straight_line_omega_cte_gate > 0.0:
                abs_cte = abs(cross_track_error)
                cte_gate = max(0.0, 1.0 - abs_cte / (self.config.reward.straight_line_omega_cte_gate * 3.0))
            smoothness -= (
                self.config.reward.straight_line_omega_penalty_weight
                * alignment
                * conflict_gate
                * cte_gate
                * abs_omega
            )

        if self.config.reward.clear_ahead_omega_weight > 0.0 and clear_ahead_route_gate > 0.0:
            abs_cte = abs(cross_track_error)
            abs_omega = abs(float(action[1]))
            alignment = max(0.0, math.cos(min(abs(observation.heading_error), math.pi / 2.0)))
            cte_recovery_gate = 1.0
            if self.config.reward.straight_line_omega_cte_gate > 0.0:
                cte_recovery_gate = max(
                    0.0,
                    1.0 - abs_cte / (self.config.reward.straight_line_omega_cte_gate * 3.0),
                )
            smoothness -= (
                self.config.reward.clear_ahead_omega_weight
                * smoothness_scale
                * clear_ahead_route_gate
                * (0.35 + 0.65 * alignment)
                * cte_recovery_gate
                * abs_omega
            )

        if self.config.reward.saturated_omega_flip_penalty_weight > 0.0:
            previous_omega = float(self._previous_actions[agent_id][1])
            current_omega = float(action[1])
            if previous_omega * current_omega < 0.0:
                angular_limit = max(self._pure_angular_speed_limit(), 1e-3)
                min_saturation_ratio = min(abs(previous_omega), abs(current_omega)) / angular_limit
                sat_threshold = max(0.1, self.config.reward.omega_flip_saturation_threshold)
                flip_progress = max(0.0, min(1.0, (min_saturation_ratio - sat_threshold) / max(1.0 - sat_threshold, 0.1)))
                if flip_progress > 0.0:
                    alignment = max(0.0, math.cos(min(abs(observation.heading_error), math.pi / 2.0)))
                    conflict_gate = max(0.2, 1.0 - 0.7 * min(conflict_risk, 1.0))
                    smoothness -= (
                        self.config.reward.saturated_omega_flip_penalty_weight
                        * smoothness_scale
                        * conflict_gate
                        * (0.35 + 0.65 * alignment)
                        * flip_progress
                    )

        heading_scale = max(
            0.1,
            1.0 - (self.config.reward.heading_relief_factor * min(conflict_risk, 1.0)) - (self.config.goal_proximity_heading_relief * goal_proximity),
        )
        heading = -((self.config.reward.heading_error_weight * heading_scale) * abs(observation.heading_error))
        heading -= clear_ahead_heading_penalty
        team = -self.config.team_reward_weight * max(0.0, near_miss_distance - pair_min)

        # Separation recovery: positive reward when pair_min increases while
        # still in the near-miss zone, encouraging active disengagement.
        if self.config.separation_recovery_weight > 0.0:
            prev_pair_min = self._previous_pair_min
            if np.isfinite(prev_pair_min) and pair_min < near_miss_distance * 1.5:
                separation_delta = max(0.0, pair_min - prev_pair_min)
                team += self.config.separation_recovery_weight * separation_delta

        # Entanglement duration penalty: ramps up when any pair stays within
        # entanglement_distance for longer than the grace period.  Discourages
        # stable orbital locks where USVs circle each other indefinitely.
        if self.config.entanglement_penalty_weight > 0.0:
            exceeded = max(0, self._entanglement_steps - self.config.entanglement_grace_steps)
            if exceeded > 0:
                penalty_strength = min(10.0, exceeded / 5.0)
                team -= self.config.entanglement_penalty_weight * penalty_strength
                # Penalise low forward speed during entanglement to prevent orbital stalling.
                if self.config.entanglement_low_speed_penalty_weight > 0.0:
                    ent_speed_deficit = max(0.0, self.config.reward.desired_conflict_speed - current_forward_speed)
                    team -= self.config.entanglement_low_speed_penalty_weight * ent_speed_deficit

        # Continuous proximity gradient penalty: a 1/d^2 repulsive field that
        # creates a strong gradient pushing the agent away from neighbours well
        # before the collision threshold is reached.
        if self.config.reward.proximity_gradient_penalty_weight > 0.0:
            pg_dist = max(1e-3, self.config.reward.proximity_gradient_distance)
            if pair_min < pg_dist:
                clamped = max(self.config.collision_distance * 0.5, pair_min)
                normalised_inv_sq = (pg_dist / clamped) ** 2 - 1.0
                safety -= self.config.reward.proximity_gradient_penalty_weight * normalised_inv_sq

        # Speed-distance coupling penalty: penalise maintaining high forward
        # speed when a neighbour is dangerously close.  This teaches the policy
        # to slow down proactively, countering the vx-saturation problem.
        if self.config.reward.speed_distance_coupling_penalty_weight > 0.0:
            sd_thresh = self.config.reward.speed_distance_coupling_threshold
            if pair_min < sd_thresh:
                speed_excess = max(0.0, current_forward_speed - self.config.reward.desired_conflict_speed)
                proximity_factor = (sd_thresh - pair_min) / max(sd_thresh, 1e-3)
                safety -= self.config.reward.speed_distance_coupling_penalty_weight * speed_excess * proximity_factor

        safety += self._compute_anticipatory_cpa_reward(
            agent_id,
            observation,
            current_forward_speed,
        )

        # Heading convergence reward: positive reward for aligning closely with
        # the goal direction, encouraging straight-line tracking.
        # Scale down during conflicts so it does not oppose avoidance turns.
        if self.config.reward.heading_convergence_reward_weight > 0.0:
            heading_threshold_rad = math.radians(max(1.0, self.config.reward.heading_convergence_threshold_deg))
            if abs(observation.heading_error) < heading_threshold_rad:
                convergence_ratio = 1.0 - abs(observation.heading_error) / heading_threshold_rad
                heading += self.config.reward.heading_convergence_reward_weight * heading_scale * convergence_ratio

        # Heading correction direction reward: rewards turning in the
        # direction that reduces heading error (ω sign opposes error sign).
        # This provides explicit directional guidance that the scalar
        # heading_error penalty cannot convey.  Scaled by |error| so
        # the signal is strongest when misaligned and vanishes near zero.
        if self.config.reward.heading_correction_reward_weight > 0.0:
            he = observation.heading_error
            current_omega = float(action[1])
            correction_sign = -1.0 if he > 0.0 else (1.0 if he < 0.0 else 0.0)
            direction_alignment = correction_sign * current_omega  # positive when turning correct way
            error_magnitude = min(1.0, abs(he) / 0.5)  # saturates at ~29°
            heading += (
                self.config.reward.heading_correction_reward_weight
                * heading_scale
                * max(0.0, min(1.0, direction_alignment / 0.20))
                * error_magnitude
            )

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
            remaining_goal_ratio = min(
                1.0,
                observation.distance_to_goal / max(self._current_route_length(agent_id), self.config.goal_tolerance + 1e-3),
            )
            terminal += self.config.reward.stall_penalty * (0.35 + 0.65 * remaining_goal_ratio)

        return MultiAgentRewardBreakdown(
            progress=progress,
            safety=safety + avoidance_turn,
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
        for namespace in self._runtime_agent_ids:
            raw_action = actions.get(namespace, self.zero_policy_action())
            projected = self.project_policy_action(namespace, raw_action)
            projected_actions[namespace] = projected
            command = self.expand_policy_action(namespace, projected)
            action_map[namespace] = self._apply_pairwise_shield(
                namespace,
                command,
                self._latest_observations.get(namespace),
            )

        if self.config.enable_rl_backend:
            self._bridge.publish_actions(action_map)

        time.sleep(self.config.control_dt)
        observations = self._wait_for_local_observations(timeout=self.config.state_timeout)
        global_state = self._wait_for_global_state(timeout=self.config.state_timeout)
        self._annotate_encounter_types(observations, global_state)
        self._annotate_cross_track_errors(observations, global_state)
        self._latest_observations = observations

        team_progress = 0.0
        for namespace in self._active_agent_ids:
            observation = observations[namespace]
            team_progress += self._previous_distances[namespace] - observation.distance_to_goal
        if team_progress >= self.config.min_progress_delta:
            self._last_team_progress_time = time.monotonic()
        self._best_team_mean_distance = min(self._best_team_mean_distance, global_state.team_mean_goal_distance)

        pair_min = self._effective_pair_min_separation(observations, global_state)
        collision = np.isfinite(pair_min) and pair_min < self.config.collision_distance
        all_reached = all(
            observations[namespace].distance_to_goal <= self.config.goal_tolerance
            for namespace in self._active_agent_ids
        )

        # Multi-waypoint transition: if all agents reached goals but more
        # waypoints remain, mark as non-terminal so the episode continues.
        waypoint_transition = False
        if all_reached and not collision and self.config.max_waypoints_per_episode > 1:
            for ns in self._active_agent_ids:
                self._waypoints_completed[ns] = self._waypoints_completed.get(ns, 0) + 1
            max_completed = max(self._waypoints_completed.get(ns, 0) for ns in self._active_agent_ids)
            if max_completed < self.config.max_waypoints_per_episode:
                waypoint_transition = True
                all_reached = False

        elapsed = time.monotonic() - self._episode_start
        terminated = collision or all_reached
        truncated = elapsed >= self.config.episode_timeout or (time.monotonic() - self._last_team_progress_time) >= self.config.no_progress_timeout

        # Update entanglement counter (once per step, shared across agents)
        if self.config.entanglement_penalty_weight > 0.0:
            if np.isfinite(pair_min) and pair_min < self.config.entanglement_distance:
                self._entanglement_steps += 1
            else:
                # Decay slowly when separated to remember recent entanglement history
                self._entanglement_steps = max(0, self._entanglement_steps - 2)

        rewards = {}
        for namespace in self._active_agent_ids:
            observation = observations[namespace]
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
            self._previous_conflict_risks[namespace] = self._compute_conflict_risk(observation)
            self._previous_cpa_metrics[namespace] = self._collect_cpa_metrics(observation)

        self._previous_pair_min = pair_min

        # Multi-waypoint: assign new random goals and add intermediate bonus.
        if waypoint_transition:
            for namespace in self._active_agent_ids:
                rewards[namespace] += self.config.waypoint_bonus
                obs = observations[namespace]
                current_x, current_y = float(obs.pose_x), float(obs.pose_y)
                current_yaw = float(obs.yaw)
                # Random heading offset ±90° from current heading for the new leg.
                angle_offset = self._rng.uniform(-math.pi / 2, math.pi / 2)
                new_heading = current_yaw + angle_offset
                new_goal_x = current_x + self.config.goal_distance * math.cos(new_heading)
                new_goal_y = current_y + self.config.goal_distance * math.sin(new_heading)
                self._scenario.agent_spawns[namespace] = AgentSpawnConfig(
                    x=current_x, y=current_y, yaw=current_yaw,
                )
                self._scenario.agent_goals[namespace] = AgentGoalConfig(
                    x=new_goal_x, y=new_goal_y,
                )
                self._previous_distances[namespace] = self.config.goal_distance
            self._bridge.publish_goals(
                f'mappo_wp{max(self._waypoints_completed.get(ns, 0) for ns in self._active_agent_ids)}_{self._episode_index:04d}',
                {ns: (g.x, g.y) for ns, g in self._scenario.agent_goals.items()},
            )
            self._best_team_mean_distance = self.config.goal_distance
            self._last_team_progress_time = time.monotonic()

        terminated_dict = {namespace: terminated for namespace in self._active_agent_ids}
        terminated_dict['__all__'] = terminated
        truncated_dict = {namespace: truncated for namespace in self._active_agent_ids}
        truncated_dict['__all__'] = truncated

        info = self._build_episode_info(global_state)
        info['pairwise_min_separation'] = pair_min
        active_observations = self._filter_active_observations(observations)

        return (
            {namespace: observation.to_vector(self.config.max_neighbors) for namespace, observation in active_observations.items()},
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
                bridge.set_rl_backend_enabled(False)
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