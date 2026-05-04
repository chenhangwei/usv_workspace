from dataclasses import dataclass, field


@dataclass
class ActionBounds:
    linear_delta: float = 0.3
    angular_delta: float = 0.4


@dataclass
class RewardConfig:
    progress_weight: float = 4.0
    goal_bonus: float = 25.0
    collision_penalty: float = -35.0
    near_miss_weight: float = 6.0
    near_miss_exponent: float = 1.0
    # Optional head-on-specific near-miss threshold. <= 0 means disabled.
    head_on_near_miss_distance: float = 0.0
    conflict_distance: float = 5.0
    anticipation_distance: float = 5.0
    conflict_risk_weight: float = 1.5
    conflict_bearing_floor: float = 0.3
    conflict_brake_weight: float = 1.2
    conflict_progress_scale: float = 0.6
    conflict_resolution_reward_weight: float = 0.9
    conflict_escalation_penalty_weight: float = 1.2
    unsafe_close_speed_penalty_weight: float = 1.0
    path_deviation_penalty_weight: float = 0.55
    path_deviation_tolerance: float = 0.8
    path_deviation_conflict_scale: float = 0.9
    desired_conflict_speed: float = 0.26
    stop_go_penalty_weight: float = 1.8
    head_on_guidance_distance: float = 5.0
    head_on_target_starboard_offset: float = 1.0
    head_on_corridor_reward_weight: float = 2.4
    head_on_centerline_penalty_weight: float = 2.8
    head_on_turn_reward_weight: float = 1.0
    head_on_forward_reward_weight: float = 1.4
    head_on_speed_drop_penalty_weight: float = 1.4
    head_on_close_penalty_weight: float = 2.2
    head_on_no_turn_penalty_weight: float = 1.4
    head_on_phase_gate_strength: float = 0.0
    crossing_starboard_turn_reward_weight: float = 0.7
    crossing_forward_reward_weight: float = 0.45
    crossing_slowdown_reward_weight: float = 0.0
    crossing_overspeed_penalty_weight: float = 0.0
    crossing_close_forward_penalty_weight: float = 0.0
    crossing_yield_speed: float = 0.12
    crossing_time_separation_reward_weight: float = 0.0
    crossing_time_separation_penalty_weight: float = 0.0
    crossing_time_gap_target: float = 6.0
    overtaking_starboard_turn_reward_weight: float = 0.55
    overtaking_forward_reward_weight: float = 0.35
    overtaking_corridor_reward_weight: float = 0.9
    overtaking_centerline_penalty_weight: float = 1.1
    overtaking_close_penalty_weight: float = 1.2
    colregs_port_turn_penalty_weight: float = 0.45
    heading_relief_factor: float = 0.45
    heading_error_weight: float = 0.2
    action_smoothness_weight: float = 0.35
    pure_cruise_reward_weight: float = 0.9
    pure_idle_penalty_weight: float = 0.8
    pure_turn_penalty_weight: float = 0.12
    pure_spin_penalty_weight: float = 0.3
    time_penalty: float = 0.02
    stall_penalty: float = -12.0
    angular_accel_penalty_weight: float = 0.0
    straight_line_omega_penalty_weight: float = 2.0
    saturated_omega_flip_penalty_weight: float = 0.0
    # Penalise forward-speed oscillation (|vx_t - vx_{t-1}|) independent of conflict state.
    forward_speed_change_penalty_weight: float = 0.0
    # Lower saturation threshold for omega-flip detection (default 0.65 is too high).
    omega_flip_saturation_threshold: float = 0.65
    # Floor for straight-line omega penalty during conflict (default 0.1 nearly disables it).
    straight_line_omega_conflict_floor: float = 0.1
    # Positive reward for increasing pairwise separation while inside near-miss band.
    # Encourages active disengagement after a close encounter.
    separation_recovery_weight: float = 0.0
    # Penalty for exceeding desired_conflict_speed during active conflict.
    # Addresses the asymmetry where only speed deficit (too slow) is penalised.
    conflict_overspeed_penalty_weight: float = 0.0
    # Continuous proximity gradient penalty: 1/d^2 scaled, activates below
    # ``proximity_gradient_distance`` to create a strong repulsive field.
    proximity_gradient_penalty_weight: float = 0.0
    proximity_gradient_distance: float = 3.0
    # Speed-distance coupling penalty: penalises high forward speed when a
    # neighbour is within ``speed_distance_coupling_threshold``.
    speed_distance_coupling_penalty_weight: float = 0.0
    speed_distance_coupling_threshold: float = 2.0
    # Heading convergence reward: positive reward when |heading_error| is
    # below ``heading_convergence_threshold_deg`` degrees.
    heading_convergence_reward_weight: float = 0.0
    heading_convergence_threshold_deg: float = 10.0
    # CTE gate for straight-line omega penalty: when |CTE| exceeds this
    # threshold (m), the omega penalty is progressively reduced so the agent
    # can turn to recover the path.  0 = disabled (no CTE gating).
    straight_line_omega_cte_gate: float = 0.0
    # Heading correction direction reward: positive reward for turning
    # in the direction that reduces heading error (ω sign matches
    # -sign(heading_error)).  Provides explicit directional guidance
    # that the scalar heading_error penalty alone cannot convey.
    heading_correction_reward_weight: float = 0.0
    # Clear-ahead route discipline: when no neighbour is close in the forward
    # cone, strongly prefer low CTE, low heading error, and low yaw rate.
    clear_ahead_distance: float = 0.0
    clear_ahead_bearing_deg: float = 35.0
    clear_ahead_cte_weight: float = 0.0
    clear_ahead_heading_weight: float = 0.0
    clear_ahead_omega_weight: float = 0.0
    # Avoidance turn reward: positive reward for turning *away* from the
    # nearest neighbour when inside the near-miss zone.  Helps break
    # COLREGs-vs-physics conflict where the agent turns into a neighbour.
    avoidance_turn_reward_weight: float = 0.0
    # Anticipatory version of the avoidance-turn signal.  It activates before
    # the near-miss band for closing neighbours, giving random encounters a
    # directional escape gradient before collision range.
    anticipatory_avoidance_turn_reward_weight: float = 0.0
    anticipatory_avoidance_turn_penalty_weight: float = 0.0
    anticipatory_avoidance_turn_distance: float = 0.0
    # Anticipatory CPA margin shaping: rewards measurable improvement in
    # predicted closest approach / closing rate before the near-miss band.
    anticipatory_cpa_distance: float = 0.0
    anticipatory_cpa_time_horizon: float = 8.0
    anticipatory_dcpa_target: float = 1.15
    anticipatory_dcpa_deficit_penalty_weight: float = 0.0
    anticipatory_dcpa_improvement_reward_weight: float = 0.0
    anticipatory_closing_reduction_reward_weight: float = 0.0
    anticipatory_yield_speed: float = 0.10
    anticipatory_yield_speed_penalty_weight: float = 0.0
    # Dense per-step penalty for near-zero speed when close to (but not
    # at) the goal.  Prevents the "hover near goal" exploit where agents
    # collect goal_proximity reward without actually arriving.
    near_goal_idle_penalty_weight: float = 0.0


@dataclass
class EnvConfig:
    namespace: str = 'usv_03'
    launch_sitl: bool = True
    enable_rl_backend: bool = False
    rl_control_mode: str = 'pure'
    action_mode: str = 'full'
    max_neighbors: int = 3
    cruise_speed: float = 0.4
    max_angular_velocity: float = 0.4
    control_dt: float = 0.2
    heading_omega_deadband: float = 0.06
    heading_omega_reference: float = 0.85
    angular_authority_power: float = 1.6
    angular_accel_limit: float = 1.8
    angular_decel_limit: float = 2.4
    conflict_turn_relief: float = 0.55
    # L1-1: Floor for angular authority. Prevents authority from collapsing
    # to 0 near the goal (heading aligned + no conflict) which caused the
    # "hover near goal" stall.  0.35 preserves ~35% of max omega capacity.
    angular_authority_floor: float = 0.35
    # L1-1: Linear-speed floor. When the policy asks for any non-trivial
    # forward motion AND forward_only is True, enforce a minimum forward
    # speed so the agent cannot freeze at vx=0.  Works at the actuator
    # layer instead of reward shaping.
    min_forward_speed_floor: float = 0.08
    state_timeout: float = 10.0
    ready_timeout: float = 45.0
    episode_timeout: float = 45.0
    no_progress_timeout: float = 10.0
    min_progress_delta: float = 0.3
    goal_distance: float = 12.0
    goal_tolerance: float = 0.5
    collision_distance: float = 0.75
    near_miss_distance: float = 1.2
    neighbor_publish_rate: float = 10.0
    scenario_neighbor_speed: float = 0.4
    default_scenarios: tuple[str, ...] = ('head_on', 'crossing_starboard', 'overtaking')
    action_bounds: ActionBounds = field(default_factory=ActionBounds)
    reward: RewardConfig = field(default_factory=RewardConfig)
    goal_proximity_reward_weight: float = 0.8
    goal_proximity_relief_distance: float = 2.0
    goal_proximity_heading_relief: float = 0.55
    goal_proximity_smoothness_relief: float = 0.70
    goal_proximity_conflict_relief: float = 0.25
    goal_proximity_speed_relief: float = 0.0
