from dataclasses import dataclass, field


@dataclass
class ActionBounds:
    linear_delta: float = 0.7
    angular_delta: float = 0.6


@dataclass
class RewardConfig:
    progress_weight: float = 4.0
    goal_bonus: float = 25.0
    collision_penalty: float = -80.0
    near_miss_weight: float = 12.0
    conflict_distance: float = 4.0
    anticipation_distance: float = 5.0
    conflict_risk_weight: float = 3.0
    conflict_brake_weight: float = 1.2
    desired_conflict_speed: float = 0.28
    stop_go_penalty_weight: float = 1.6
    head_on_guidance_distance: float = 6.0
    head_on_target_starboard_offset: float = 1.1
    head_on_corridor_reward_weight: float = 2.4
    head_on_centerline_penalty_weight: float = 2.8
    head_on_turn_reward_weight: float = 0.8
    heading_relief_factor: float = 0.45
    heading_error_weight: float = 0.2
    action_smoothness_weight: float = 0.25
    time_penalty: float = 0.02
    stall_penalty: float = -12.0


@dataclass
class EnvConfig:
    namespace: str = 'usv_03'
    launch_sitl: bool = True
    enable_rl_backend: bool = False
    action_mode: str = 'full'
    max_neighbors: int = 3
    control_dt: float = 0.2
    state_timeout: float = 10.0
    ready_timeout: float = 45.0
    episode_timeout: float = 45.0
    no_progress_timeout: float = 10.0
    min_progress_delta: float = 0.3
    goal_distance: float = 12.0
    goal_tolerance: float = 0.8
    collision_distance: float = 0.5
    near_miss_distance: float = 1.5
    neighbor_publish_rate: float = 10.0
    scenario_neighbor_speed: float = 0.45
    default_scenarios: tuple[str, ...] = ('head_on', 'crossing_starboard', 'overtaking')
    action_bounds: ActionBounds = field(default_factory=ActionBounds)
    reward: RewardConfig = field(default_factory=RewardConfig)