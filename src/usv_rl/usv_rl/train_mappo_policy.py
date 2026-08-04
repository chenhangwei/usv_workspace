import argparse
import copy
import json
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
from .multi_agent_types import AgentLocalObservation, FleetGlobalState, ENCOUNTER_TYPE_COUNT, NEIGHBOR_FEATURE_COUNT
from .observation_normalizer import ObservationNormalizer


MAX_FASTDDS_SAFE_DOMAIN_ID = 232


def parse_args():
    default_env = MultiAgentEnvConfig()
    parser = argparse.ArgumentParser(description='Minimal MAPPO training entry for multi-USV RL control.')
    parser.add_argument('--output', required=True, help='Output model path (.pt).')
    parser.add_argument('--resume-from', help='Optional checkpoint scaffold (.pt) used to resume MAPPO weights and training metadata.')
    parser.add_argument('--load-weights-from', help='Load actor/critic/normalizer weights from a checkpoint without restoring training metadata or config. Useful for curriculum transfer.')
    parser.add_argument('--reset-optimizer', action='store_true', help='When resuming, ignore optimizer/scaler state stored in the checkpoint and restart optimizer state from scratch.')
    parser.add_argument('--num-agents', type=int, default=3, help='Number of controlled agents.')
    parser.add_argument('--total-timesteps', type=int, default=4096, help='Total training timesteps across all agents.')
    parser.add_argument('--rollout-steps', type=int, default=128, help='Rollout horizon before each update.')
    parser.add_argument('--reset-sampler-each-rollout', action='store_true', help='For parallel MAPPO sampling, reset each worker before every rollout collection after the first one so short repair runs advance the scenario curriculum even when episodes do not finish naturally.')
    parser.add_argument('--update-epochs', type=int, default=4, help='Number of PPO update epochs per rollout.')
    parser.add_argument('--minibatch-size', type=int, default=128, help='Mini-batch size for PPO updates.')
    parser.add_argument('--learning-rate', type=float, default=3e-4, help='Optimizer learning rate (initial value when --learning-rate-end is set).')
    parser.add_argument('--learning-rate-end', type=float, default=None, help='Final learning rate for linear annealing. If unset, learning-rate stays constant.')
    parser.add_argument('--gamma', type=float, default=0.99, help='Discount factor.')
    parser.add_argument('--gae-lambda', type=float, default=0.95, help='GAE lambda.')
    parser.add_argument('--clip-range', type=float, default=0.2, help='PPO clip range.')
    parser.add_argument('--entropy-coef', type=float, default=0.01, help='Entropy bonus coefficient (initial value when --entropy-coef-end is set).')
    parser.add_argument('--entropy-coef-end', type=float, default=None, help='Final entropy coefficient for linear annealing. If unset, entropy-coef stays constant.')
    parser.add_argument('--value-coef', type=float, default=0.5, help='Value loss coefficient.')
    parser.add_argument('--ppo-policy-loss-scale', type=float, default=1.0, help='Scale for PPO policy loss. Set to 0 for auxiliary-only repair updates.')
    parser.add_argument('--ppo-value-loss-scale', type=float, default=1.0, help='Scale for PPO value loss. Set to 0 for auxiliary-only repair updates.')
    parser.add_argument('--max-grad-norm', type=float, default=0.5, help='Gradient clipping norm.')
    parser.add_argument('--device', default='auto', help='Torch device. Use auto to prefer CUDA when available.')
    parser.add_argument('--amp', choices=['auto', 'on', 'off'], default='auto', help='Mixed precision mode. Auto enables AMP on CUDA and keeps CPU training in full precision.')
    parser.add_argument('--matmul-precision', choices=['highest', 'high', 'medium'], default='high', help='Preferred float32 matmul precision on supported PyTorch builds.')
    parser.add_argument('--disable-tf32', action='store_true', help='Disable TF32 acceleration on Ampere-class GPUs such as A10.')
    parser.add_argument('--torch-num-threads', type=int, default=None, help='Optional torch CPU thread cap to reduce contention with ROS sampling on small CPU instances.')
    parser.add_argument('--hidden-size', action='append', dest='hidden_sizes', type=int, default=None, help='Hidden layer size. Repeatable.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name. Repeatable.')
    parser.add_argument('--curriculum-scenario', action='append', dest='curriculum_scenarios', default=None, help='Rollout scenario name. Repeatable and may include duplicates to oversample hard scenarios while --scenario remains the unique deployable branch set.')
    parser.add_argument('--curriculum-seed', action='append', dest='curriculum_seeds', type=int, default=None, help='Fixed environment reset seed used by rollout training. Repeatable; seeds are cycled across episode resets to replay hard random-encounter geometries.')
    parser.add_argument('--random-encounter-route-priority', action='store_true', help='Use active route midpoint ETA ordering for random encounter priority features instead of fixed agent-id ordering.')
    parser.add_argument('--scenario-set', choices=['auto', 'smoke', 'dense', 'all'], default='auto', help='Scenario curriculum preset used when --scenario is not provided.')
    parser.add_argument('--max-neighbors', type=int, default=4, help='Neighbor slots in local observation encoding.')
    parser.add_argument('--max-agents', type=int, default=3, help='Maximum agents encoded in global state.')
    parser.add_argument('--rl-control-mode', choices=['pure'], default='pure', help='Pure final-command control mode used during training.')
    parser.add_argument('--action-mode', choices=['full', 'speed_scale', 'angular_only'], default='full', help='Action representation: "full"=[linear,omega]; "speed_scale"=[throttle_scale,omega] (no stop/reverse); "angular_only"=[omega] (nav owns speed).')
    parser.add_argument('--action-speed-scale-min', type=float, default=0.25, help='Lower bound of the throttle scale for action_mode="speed_scale".')
    parser.add_argument('--linear-delta-limit', type=float, default=default_env.action_bounds.linear_delta, help='Maximum forward command magnitude (m/s).')
    parser.add_argument('--angular-delta-limit', type=float, default=default_env.action_bounds.angular_delta, help='Maximum yaw-rate command magnitude (rad/s).')
    parser.add_argument('--cruise-speed', type=float, default=default_env.cruise_speed, help='Training controller cruise speed limit (m/s).')
    parser.add_argument('--max-angular-velocity', type=float, default=default_env.max_angular_velocity, help='Training controller yaw-rate limit (rad/s).')
    parser.add_argument('--heading-omega-deadband', type=float, default=default_env.heading_omega_deadband, help='Heading-error deadband (rad) below which yaw-rate authority collapses toward straight-line tracking.')
    parser.add_argument('--heading-omega-reference', type=float, default=default_env.heading_omega_reference, help='Heading error (rad) that restores full yaw-rate authority.')
    parser.add_argument('--angular-authority-power', type=float, default=default_env.angular_authority_power, help='Nonlinear power used to shrink yaw-rate authority at small heading errors.')
    parser.add_argument('--angular-accel-limit', type=float, default=default_env.angular_accel_limit, help='Angular acceleration limit (rad/s^2) used while building turn rate.')
    parser.add_argument('--angular-decel-limit', type=float, default=default_env.angular_decel_limit, help='Angular deceleration limit (rad/s^2) used while unwinding turn rate near alignment.')
    parser.add_argument('--conflict-turn-relief', type=float, default=default_env.conflict_turn_relief, help='Minimum yaw-rate authority retained at full conflict even when goal heading error is small.')
    parser.add_argument('--angular-authority-floor', type=float, default=default_env.angular_authority_floor, help='[L1-1] Minimum yaw-rate authority preserved at all times (fraction of max omega). Prevents hover-at-goal when heading aligned.')
    parser.add_argument('--min-forward-speed-floor', type=float, default=default_env.min_forward_speed_floor, help='[L1-1] Actuator-level linear-speed floor applied when policy asks for any forward motion. Eliminates vx=0 stall near goal.')
    parser.add_argument('--episode-timeout', type=float, default=45.0, help='Episode timeout in seconds.')
    parser.add_argument('--no-progress-timeout', type=float, default=10.0, help='No-progress timeout in seconds.')
    parser.add_argument('--max-waypoints-per-episode', type=int, default=1, help='Number of waypoints per episode (>1 enables multi-waypoint training).')
    parser.add_argument('--waypoint-bonus', type=float, default=10.0, help='Intermediate reward when reaching a non-final waypoint.')
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
    parser.add_argument('--near-miss-exponent', type=float, default=RewardConfig.near_miss_exponent, help='Near-miss penalty exponent. 1.0=linear (default), 2.0=quadratic (steeper near collision).')
    parser.add_argument('--head-on-near-miss-distance', type=float, default=RewardConfig.head_on_near_miss_distance, help='Optional near-miss threshold (m) used only in two_usv_head_on. <=0 disables override.')
    parser.add_argument('--conflict-distance', type=float, default=RewardConfig.conflict_distance, help='Distance threshold used for conflict-specific shaping.')
    parser.add_argument('--anticipation-distance', type=float, default=RewardConfig.anticipation_distance, help='Lookahead distance used for anticipatory conflict shaping.')
    parser.add_argument('--conflict-risk-weight', type=float, default=RewardConfig.conflict_risk_weight, help='Conflict-risk penalty weight.')
    parser.add_argument('--conflict-bearing-floor', type=float, default=RewardConfig.conflict_bearing_floor, help='Minimum bearing factor for closing neighbours in conflict-risk shaping.')
    parser.add_argument('--conflict-risk-time-horizon', type=float, default=RewardConfig.conflict_risk_time_horizon, help='TCPA look-ahead (s) for the predictive DCPA/TCPA conflict-risk index (0 -> fall back to anticipatory_cpa_time_horizon). Larger = look further ahead / avoid earlier.')
    parser.add_argument('--conflict-risk-safe-dcpa', type=float, default=RewardConfig.conflict_risk_safe_dcpa, help='Predicted closest-approach distance (m) beyond which a pass is treated as comfortable (0 risk). 0 -> max(near_miss, anticipatory_dcpa_target).')
    parser.add_argument('--conflict-brake-weight', type=float, default=RewardConfig.conflict_brake_weight, help='Conflict braking penalty weight.')
    parser.add_argument('--conflict-progress-scale', type=float, default=RewardConfig.conflict_progress_scale, help='Reduction applied to positive progress reward during unresolved conflict.')
    parser.add_argument('--conflict-resolution-reward-weight', type=float, default=RewardConfig.conflict_resolution_reward_weight, help='Reward weight for reducing conflict risk across consecutive steps.')
    parser.add_argument('--conflict-escalation-penalty-weight', type=float, default=RewardConfig.conflict_escalation_penalty_weight, help='Penalty weight when conflict risk grows across consecutive steps.')
    parser.add_argument('--unsafe-close-speed-penalty-weight', type=float, default=RewardConfig.unsafe_close_speed_penalty_weight, help='Penalty weight for maintaining excessive forward speed inside the near-miss band.')
    parser.add_argument('--path-deviation-penalty-weight', type=float, default=RewardConfig.path_deviation_penalty_weight, help='Penalty weight for drifting too far away from the spawn-to-goal reference route.')
    parser.add_argument('--path-deviation-tolerance', type=float, default=RewardConfig.path_deviation_tolerance, help='Cross-track deviation (m) allowed before route-tracking penalty activates.')
    parser.add_argument('--path-deviation-conflict-scale', type=float, default=RewardConfig.path_deviation_conflict_scale, help='Additional cross-track tolerance (m) allowed at maximum conflict level.')
    parser.add_argument('--path-deviation-use-unclipped-cte', action='store_true', default=RewardConfig.path_deviation_use_unclipped_cte, help='Compute route-deviation reward with the true CTE instead of the observation-clipped CTE.')
    parser.add_argument('--path-deviation-exclude-overtaking', action='store_true', default=RewardConfig.path_deviation_exclude_overtaking, help='Disable the route-deviation penalty on single_usv_overtaking / two_usv_overtaking scenarios, where passing a slow lead requires leaving the lane.')
    parser.add_argument('--path-inefficiency-penalty-weight', type=float, default=RewardConfig.path_inefficiency_penalty_weight, help='fresh627: per-step penalty on max(0, travel - progress). Waiting in place is free; lateral swings, detour arcs and loops pay per wasted metre.')
    parser.add_argument('--waypoint-pass-quality-bonus', type=float, default=RewardConfig.waypoint_pass_quality_bonus, help='fresh627: extra bonus at each waypoint pass / final arrival scaled by (1 - pass_distance/goal_tolerance); full for center-threading, zero for edge-grazing.')
    parser.add_argument('--role-speed-asymmetry-weight', type=float, default=RewardConfig.role_speed_asymmetry_weight, help='fresh628: strict master/slave speed shaping. Stand-on rewarded for holding cruise; give-way rewarded for slowing to role-giveway-speed while conflict risk is high.')
    parser.add_argument('--role-giveway-speed', type=float, default=RewardConfig.role_giveway_speed, help='fresh628: target creep speed (m/s) for the give-way vessel during an active encounter.')
    parser.add_argument('--wrong-heading-speed-penalty-weight', type=float, default=RewardConfig.wrong_heading_speed_penalty_weight, help='fresh628: tax forward speed while |heading error| exceeds the threshold, scaled by (1-cos(he)). Forces slow-spin-then-go at waypoint switches.')
    parser.add_argument('--wrong-heading-speed-threshold-deg', type=float, default=RewardConfig.wrong_heading_speed_threshold_deg, help='fresh628: heading-error threshold (deg) where the wrong-heading speed tax starts.')
    parser.add_argument('--wrong-heading-conflict-relief', type=float, default=RewardConfig.wrong_heading_conflict_relief, help='fresh633: waive the wrong-heading speed tax proportionally to conflict risk (0=always taxed, 1=fully waived at max conflict). Unbinds "turn hard" from "slow down" during encounters to prevent bow-to-bow standoffs.')
    parser.add_argument('--far-side-turn-penalty-weight', type=float, default=RewardConfig.far_side_turn_penalty_weight, help='fresh629: penalise the far-side omega component while |heading error| exceeds the threshold, internalising minimum-heading-error turn direction.')
    parser.add_argument('--far-side-turn-threshold-deg', type=float, default=RewardConfig.far_side_turn_threshold_deg, help='fresh629: heading-error threshold (deg) for the far-side turn penalty.')
    parser.add_argument('--goal-queue-weight', type=float, default=RewardConfig.goal_queue_weight, help='fresh629: same-goal queue shaping. Followers of a clustered shared waypoint hold low speed + separation while the leader clears it.')
    parser.add_argument('--goal-queue-cluster-radius', type=float, default=RewardConfig.goal_queue_cluster_radius, help='fresh629: two goals within this distance (m) count as a shared waypoint cluster.')
    parser.add_argument('--goal-queue-engage-distance', type=float, default=RewardConfig.goal_queue_engage_distance, help='fresh629: queue shaping engages when boats/goals are within this range (m).')
    parser.add_argument('--goal-queue-follower-speed', type=float, default=RewardConfig.goal_queue_follower_speed, help='fresh629: follower hold speed (m/s) while queueing.')
    parser.add_argument('--goal-queue-min-separation', type=float, default=RewardConfig.goal_queue_min_separation, help='fresh629: minimum separation (m) the queue follower must keep from the leader.')
    parser.add_argument('--desired-conflict-speed', type=float, default=RewardConfig.desired_conflict_speed, help='Target forward speed maintained during conflict handling.')
    parser.add_argument('--stop-go-penalty-weight', type=float, default=RewardConfig.stop_go_penalty_weight, help='Penalty weight for rapid stop-go behavior under conflict.')
    parser.add_argument('--head-on-guidance-distance', type=float, default=RewardConfig.head_on_guidance_distance, help='Distance threshold for head-on starboard guidance shaping.')
    parser.add_argument('--crossing-guidance-distance', type=float, default=RewardConfig.crossing_guidance_distance, help='Optional dedicated lookahead (m) for crossing/overtaking starboard guidance (0 -> legacy max(anticipation_distance, conflict_distance)).')
    parser.add_argument('--head-on-target-starboard-offset', type=float, default=RewardConfig.head_on_target_starboard_offset, help='Desired starboard lateral offset in head-on encounters.')
    parser.add_argument('--head-on-corridor-reward-weight', type=float, default=RewardConfig.head_on_corridor_reward_weight, help='Reward weight for entering the head-on starboard corridor.')
    parser.add_argument('--head-on-centerline-penalty-weight', type=float, default=RewardConfig.head_on_centerline_penalty_weight, help='Penalty weight for staying on the head-on centerline.')
    parser.add_argument('--head-on-turn-reward-weight', type=float, default=RewardConfig.head_on_turn_reward_weight, help='Reward weight for committing to starboard turn in head-on encounters.')
    parser.add_argument('--head-on-forward-reward-weight', type=float, default=RewardConfig.head_on_forward_reward_weight, help='Reward weight for maintaining forward motion during head-on avoidance.')
    parser.add_argument('--head-on-speed-drop-penalty-weight', type=float, default=RewardConfig.head_on_speed_drop_penalty_weight, help='Penalty weight for abrupt forward-speed loss during head-on avoidance.')
    parser.add_argument('--head-on-close-penalty-weight', type=float, default=RewardConfig.head_on_close_penalty_weight, help='Penalty weight for charging straight into a close head-on corridor without establishing starboard clearance.')
    parser.add_argument('--head-on-no-turn-penalty-weight', type=float, default=RewardConfig.head_on_no_turn_penalty_weight, help='Penalty weight for failing to commit to starboard turn during urgent head-on encounters.')
    parser.add_argument('--head-on-phase-gate-strength', type=float, default=RewardConfig.head_on_phase_gate_strength, help='Strength of corridor-progress gating on forward reward in head-on encounters (0=disabled, 1=full gate).')
    parser.add_argument('--crossing-starboard-turn-reward-weight', type=float, default=RewardConfig.crossing_starboard_turn_reward_weight, help='Reward weight for starboard give-way turns in crossing encounters.')
    parser.add_argument('--crossing-forward-reward-weight', type=float, default=RewardConfig.crossing_forward_reward_weight, help='Reward weight for maintaining forward motion while committing to starboard give-way turns in crossing encounters.')
    parser.add_argument('--crossing-slowdown-reward-weight', type=float, default=RewardConfig.crossing_slowdown_reward_weight, help='Reward weight for slowing to a yield speed while committing to a starboard crossing turn.')
    parser.add_argument('--crossing-overspeed-penalty-weight', type=float, default=RewardConfig.crossing_overspeed_penalty_weight, help='Penalty weight for exceeding crossing-yield-speed during give-way crossing encounters.')
    parser.add_argument('--crossing-close-forward-penalty-weight', type=float, default=RewardConfig.crossing_close_forward_penalty_weight, help='Penalty weight for continuing forward into a close crossing encounter before establishing a turn.')
    parser.add_argument('--crossing-yield-speed', type=float, default=RewardConfig.crossing_yield_speed, help='Target forward speed (m/s) for crossing give-way slowdown shaping.')
    parser.add_argument('--crossing-time-separation-reward-weight', type=float, default=RewardConfig.crossing_time_separation_reward_weight, help='Reward weight for respecting dynamic crossing ETA priority / yield roles.')
    parser.add_argument('--crossing-time-separation-penalty-weight', type=float, default=RewardConfig.crossing_time_separation_penalty_weight, help='Penalty weight for violating crossing ETA-gap yield roles.')
    parser.add_argument('--crossing-time-gap-target', type=float, default=RewardConfig.crossing_time_gap_target, help='Target ETA gap (s) between vessels at the shared crossing point.')
    parser.add_argument('--overtaking-starboard-turn-reward-weight', type=float, default=RewardConfig.overtaking_starboard_turn_reward_weight, help='Reward weight for starboard bias during overtaking.')
    parser.add_argument('--overtaking-forward-reward-weight', type=float, default=RewardConfig.overtaking_forward_reward_weight, help='Reward weight for maintaining forward motion while committing to starboard overtaking bias.')
    parser.add_argument('--overtaking-corridor-reward-weight', type=float, default=RewardConfig.overtaking_corridor_reward_weight, help='Reward weight for opening starboard lateral clearance before passing a slower vessel.')
    parser.add_argument('--overtaking-centerline-penalty-weight', type=float, default=RewardConfig.overtaking_centerline_penalty_weight, help='Penalty weight for staying directly behind a slower vessel while attempting to overtake.')
    parser.add_argument('--overtaking-close-penalty-weight', type=float, default=RewardConfig.overtaking_close_penalty_weight, help='Penalty weight for closing too aggressively on a slower vessel without enough starboard offset.')
    parser.add_argument('--colregs-port-turn-penalty-weight', type=float, default=RewardConfig.colregs_port_turn_penalty_weight, help='Penalty weight for turning to port during starboard-biased COLREGs encounters.')
    parser.add_argument('--heading-relief-factor', type=float, default=RewardConfig.heading_relief_factor, help='Fractional relief applied to heading penalty during conflict handling.')
    parser.add_argument('--heading-error-weight', type=float, default=RewardConfig.heading_error_weight, help='Heading error penalty weight.')
    parser.add_argument('--action-smoothness-weight', type=float, default=RewardConfig.action_smoothness_weight, help='Action smoothness penalty weight.')
    parser.add_argument('--pure-cruise-reward-weight', type=float, default=RewardConfig.pure_cruise_reward_weight, help='Pure-RL reward weight for sustaining aligned forward cruise in open water.')
    parser.add_argument('--pure-idle-penalty-weight', type=float, default=RewardConfig.pure_idle_penalty_weight, help='Pure-RL penalty weight for failing to maintain target forward speed in open water.')
    parser.add_argument('--pure-turn-penalty-weight', type=float, default=RewardConfig.pure_turn_penalty_weight, help='Pure-RL penalty weight for excessive yaw rate away from conflict handling.')
    parser.add_argument('--pure-spin-penalty-weight', type=float, default=RewardConfig.pure_spin_penalty_weight, help='Pure-RL penalty weight for low-speed large-yaw exploration in open water.')
    parser.add_argument('--time-penalty', type=float, default=RewardConfig.time_penalty, help='Per-step time penalty.')
    parser.add_argument('--stall-penalty', type=float, default=RewardConfig.stall_penalty, help='Penalty applied on episode truncation.')
    parser.add_argument('--angular-accel-penalty-weight', type=float, default=RewardConfig.angular_accel_penalty_weight, help='Angular acceleration penalty weight for smooth turning.')
    parser.add_argument('--straight-line-omega-penalty-weight', type=float, default=RewardConfig.straight_line_omega_penalty_weight, help='Penalty weight for large angular velocity when heading error is small (prevents S-curve oscillation).')
    parser.add_argument('--saturated-omega-flip-penalty-weight', type=float, default=RewardConfig.saturated_omega_flip_penalty_weight, help='Penalty weight for bang-bang reversals where consecutive angular commands flip sign near saturation.')
    parser.add_argument('--turn-speed-coupling-penalty-weight', type=float, default=RewardConfig.turn_speed_coupling_penalty_weight, help='Penalty for forward speed above a turn-appropriate cap: obtuse turn keeps speed, acute turn must slow for a tight radius.')
    parser.add_argument('--turn-speed-coupling-floor', type=float, default=RewardConfig.turn_speed_coupling_floor, help='Min speed fraction of cruise allowed at a 90deg turn (turn-speed coupling).')
    parser.add_argument('--turn-speed-coupling-deadband-deg', type=float, default=RewardConfig.turn_speed_coupling_deadband_deg, help='Heading-error deadband (deg) below which no turn-speed penalty applies (gentle/obtuse legs keep full speed).')
    parser.add_argument('--forward-speed-change-penalty-weight', type=float, default=RewardConfig.forward_speed_change_penalty_weight, help='Penalty weight for |vx_t - vx_{t-1}| (prevents linear speed oscillation / bang-bang).')
    parser.add_argument('--omega-flip-saturation-threshold', type=float, default=RewardConfig.omega_flip_saturation_threshold, help='Min saturation ratio to trigger omega-flip penalty (lower = catches milder bang-bang). Default 0.65.')
    parser.add_argument('--straight-line-omega-conflict-floor', type=float, default=RewardConfig.straight_line_omega_conflict_floor, help='Minimum conflict gate for straight-line omega penalty (higher = less suppression during conflict). Default 0.1.')
    parser.add_argument('--conflict-overspeed-penalty-weight', type=float, default=RewardConfig.conflict_overspeed_penalty_weight, help='Penalty weight for exceeding desired conflict speed during active conflict. Fixes the asymmetry where only speed deficit was penalised.')
    parser.add_argument('--proximity-gradient-penalty-weight', type=float, default=RewardConfig.proximity_gradient_penalty_weight, help='Weight for 1/d^2 repulsive gradient penalty when pair distance < proximity-gradient-distance.')
    parser.add_argument('--proximity-gradient-distance', type=float, default=RewardConfig.proximity_gradient_distance, help='Activation distance (m) for proximity gradient penalty.')
    parser.add_argument('--speed-distance-coupling-penalty-weight', type=float, default=RewardConfig.speed_distance_coupling_penalty_weight, help='Penalty weight for high speed when close to neighbors.')
    parser.add_argument('--speed-distance-coupling-threshold', type=float, default=RewardConfig.speed_distance_coupling_threshold, help='Activation distance (m) for speed-distance coupling penalty.')
    parser.add_argument('--heading-convergence-reward-weight', type=float, default=RewardConfig.heading_convergence_reward_weight, help='Positive reward for heading error within convergence threshold.')
    parser.add_argument('--heading-convergence-threshold-deg', type=float, default=RewardConfig.heading_convergence_threshold_deg, help='Heading error threshold (degrees) for convergence bonus.')
    parser.add_argument('--heading-correction-reward-weight', type=float, default=RewardConfig.heading_correction_reward_weight, help='Positive reward for turning in the direction that reduces heading error.')
    parser.add_argument('--straight-line-omega-cte-gate', type=float, default=RewardConfig.straight_line_omega_cte_gate, help='CTE threshold (m) above which straight-line omega penalty is progressively reduced for path recovery. 0=disabled.')
    parser.add_argument('--clear-ahead-distance', type=float, default=RewardConfig.clear_ahead_distance, help='Forward-cone distance (m) for clear-ahead route discipline. 0 disables.')
    parser.add_argument('--clear-ahead-bearing-deg', type=float, default=RewardConfig.clear_ahead_bearing_deg, help='Half-angle of the forward cone used by clear-ahead route discipline.')
    parser.add_argument('--clear-ahead-cte-weight', type=float, default=RewardConfig.clear_ahead_cte_weight, help='Extra CTE penalty weight applied only when the forward cone is clear.')
    parser.add_argument('--clear-ahead-heading-weight', type=float, default=RewardConfig.clear_ahead_heading_weight, help='Extra heading-error penalty weight applied only when the forward cone is clear.')
    parser.add_argument('--clear-ahead-omega-weight', type=float, default=RewardConfig.clear_ahead_omega_weight, help='Extra yaw-rate penalty weight applied only when the forward cone is clear.')
    parser.add_argument('--avoidance-turn-reward-weight', type=float, default=RewardConfig.avoidance_turn_reward_weight, help='Positive reward weight for turning away from nearest neighbour inside near-miss zone.')
    parser.add_argument('--anticipatory-avoidance-turn-reward-weight', type=float, default=RewardConfig.anticipatory_avoidance_turn_reward_weight, help='Positive reward weight for early away-turning against closing neighbours.')
    parser.add_argument('--anticipatory-avoidance-turn-penalty-weight', type=float, default=RewardConfig.anticipatory_avoidance_turn_penalty_weight, help='Penalty weight for weak or wrong-way turning against closing neighbours before near miss.')
    parser.add_argument('--anticipatory-avoidance-turn-distance', type=float, default=RewardConfig.anticipatory_avoidance_turn_distance, help='Distance (m) where anticipatory avoidance-turn shaping activates. 0 disables unless reward/penalty weights stay zero.')
    parser.add_argument('--anticipatory-cpa-distance', type=float, default=RewardConfig.anticipatory_cpa_distance, help='Distance (m) where anticipatory CPA margin shaping activates. 0 disables while all CPA weights are zero.')
    parser.add_argument('--anticipatory-cpa-time-horizon', type=float, default=RewardConfig.anticipatory_cpa_time_horizon, help='TCPA horizon (s) for anticipatory CPA margin shaping.')
    parser.add_argument('--anticipatory-dcpa-target', type=float, default=RewardConfig.anticipatory_dcpa_target, help='Predicted DCPA target (m) used by anticipatory CPA margin shaping.')
    parser.add_argument('--anticipatory-dcpa-deficit-penalty-weight', type=float, default=RewardConfig.anticipatory_dcpa_deficit_penalty_weight, help='Penalty weight for low predicted DCPA before near miss.')
    parser.add_argument('--anticipatory-dcpa-improvement-reward-weight', type=float, default=RewardConfig.anticipatory_dcpa_improvement_reward_weight, help='Reward weight for increasing predicted DCPA across consecutive steps.')
    parser.add_argument('--anticipatory-closing-reduction-reward-weight', type=float, default=RewardConfig.anticipatory_closing_reduction_reward_weight, help='Reward weight for reducing range-closing speed across consecutive steps.')
    parser.add_argument('--anticipatory-yield-speed', type=float, default=RewardConfig.anticipatory_yield_speed, help='Target give-way speed (m/s) for deterministic random-encounter CPA yield roles.')
    parser.add_argument('--anticipatory-yield-speed-penalty-weight', type=float, default=RewardConfig.anticipatory_yield_speed_penalty_weight, help='Penalty weight for give-way agents exceeding anticipatory-yield-speed in low-DCPA random encounters.')
    parser.add_argument('--near-goal-idle-penalty-weight', type=float, default=RewardConfig.near_goal_idle_penalty_weight, help='Dense per-step penalty for near-zero speed when close to (but not at) the goal. Prevents hover-near-goal exploit.')
    parser.add_argument('--cte-clip-range', type=float, default=3.0, help='Symmetric clip range (m) for signed cross-track error observation. Default 3.0.')
    parser.add_argument('--route-progress-cte-gate-start', type=float, default=0.0, help='CTE magnitude where observation route_progress gating starts. 0 disables unless width > 0.')
    parser.add_argument('--route-progress-cte-gate-width', type=float, default=0.0, help='CTE magnitude where observation route_progress reaches its floor. 0 disables gating.')
    parser.add_argument('--route-progress-cte-gate-floor', type=float, default=0.25, help='Minimum multiplier for CTE-gated observation route_progress.')
    parser.add_argument('--neighbor-attention', action='store_true', help='Replace fixed neighbor padding with attention-based neighbor aggregation. Learns to focus on the most relevant neighbor (nearest, highest TCPA, head-on, etc.).')
    parser.add_argument('--attention-embed-dim', type=int, default=32, help='Embedding dimension for neighbor attention encoder.')
    parser.add_argument('--attention-num-heads', type=int, default=1, help='Number of attention heads for neighbor attention encoder.')
    parser.add_argument('--attention-encounter-residual', action='store_true', help='Add an encounter-conditioned residual action head on top of the shared attention actor. The residual head is zero-initialized so loading an old checkpoint preserves its initial policy.')
    parser.add_argument('--attention-scenario-residual', action='store_true', help='Add scenario-specific residual action heads on top of the shared attention actor. Each scenario gets a separate zero-initialized branch selected by scenario id.')
    parser.add_argument('--attention-scenario-head', action='store_true', help='Add scenario-specific action heads that replace the final actor slice for the active scenario while leaving the shared trunk untouched.')
    parser.add_argument('--attention-scenario-trunk', action='store_true', help='Add scenario-specific post-attention actor trunks that replace the shared actor MLP for the active scenario while keeping the attention encoder shared.')
    parser.add_argument('--freeze-actor-base', action='store_true', help='Freeze the shared actor weights and train only scenario-specific branches, heads, or trunks. Requires --attention-encounter-residual, --attention-scenario-residual, --attention-scenario-head, or --attention-scenario-trunk.')
    parser.add_argument('--squash-actions', action='store_true', help='Apply tanh squashing to actor output for smooth bounded actions.')
    parser.add_argument('--min-forward-speed', type=float, default=0.0, help='Minimum forward speed enforced via action bounds when squash-actions is enabled.')
    parser.add_argument('--normalize-observations', action='store_true', help='Enable running observation normalization for stable training across mixed feature scales.')
    parser.add_argument('--freeze-observation-normalizer', action='store_true', help='Keep loaded observation-normalizer statistics fixed during training. Useful for branch-only scenario repair without shifting all deployed scenario inputs.')
    parser.add_argument('--domain-randomization', action='store_true', help='Enable domain randomization (sensor noise, current drift, actuator noise) for sim-to-real robustness.')
    parser.add_argument('--dr-position-noise-std', type=float, default=0.10, help='GPS observation noise standard deviation (m).')
    parser.add_argument('--dr-heading-noise-std', type=float, default=0.02, help='Compass observation noise standard deviation (rad).')
    parser.add_argument('--dr-velocity-noise-ratio', type=float, default=0.03, help='Velocity sensor noise ratio.')
    parser.add_argument('--dr-current-speed-max', type=float, default=0.04, help='Maximum water current speed (m/s).')
    parser.add_argument('--dr-velocity-exec-noise', type=float, default=0.05, help='Actuator velocity execution noise ratio.')
    parser.add_argument('--scenario-spawn-position-std', type=float, default=0.0, help='Spawn position Gaussian jitter std (m). 0 disables.')
    parser.add_argument('--scenario-spawn-heading-std', type=float, default=0.0, help='Spawn heading Gaussian jitter std (rad). 0 disables.')
    parser.add_argument('--scenario-goal-position-std', type=float, default=0.0, help='Goal position Gaussian jitter std (m). 0 disables.')
    parser.add_argument('--encounter-type-dropout', type=float, default=0.0, help='Probability of zeroing encounter one-hot per step (0-1). Improves deployment generalization.')
    parser.add_argument('--sim-tau-linear', type=float, default=0.45, help='Sim first-order linear velocity time constant (s). Higher = more inertia.')
    parser.add_argument('--sim-tau-angular', type=float, default=0.25, help='Sim first-order angular velocity time constant (s). Higher = more inertia.')
    parser.add_argument('--dr-tau-linear-low', type=float, default=0.0, help='Tau linear DR lower bound (s). 0 disables tau randomization.')
    parser.add_argument('--dr-tau-linear-high', type=float, default=0.0, help='Tau linear DR upper bound (s).')
    parser.add_argument('--dr-tau-angular-low', type=float, default=0.0, help='Tau angular DR lower bound (s).')
    parser.add_argument('--dr-tau-angular-high', type=float, default=0.0, help='Tau angular DR upper bound (s).')
    parser.add_argument('--speed-scale-distance', type=float, default=0.0, help='Distance threshold for neighbour-proximity speed scaling in training sim (m). 0 disables.')
    parser.add_argument('--speed-scale-min', type=float, default=0.35, help='Minimum speed scale factor at zero neighbour distance.')
    parser.add_argument('--actor-log-std-init', type=float, default=0.0, help='Initial value for actor_log_std parameter. Negative values reduce initial exploration noise.')
    parser.add_argument('--force-actor-log-std', type=float, default=None, help='If set, override actor_log_std to this value after loading checkpoint weights.')
    parser.add_argument('--team-reward-weight', type=float, default=0.30, help='Fleet near-miss team penalty weight.')
    parser.add_argument('--team-progress-weight', type=float, default=1.20, help='Fleet progress reward weight.')
    parser.add_argument('--team-goal-proximity-weight', type=float, default=0.0, help='Fleet dense reward weight for reducing mean distance to goals from the scenario start.')
    parser.add_argument('--team-regression-penalty-weight', type=float, default=0.0, help='Fleet penalty weight for negative team progress.')
    parser.add_argument('--team-dispersion-penalty-weight', type=float, default=0.0, help='Fleet penalty weight for expanding mean pairwise separation beyond the initial formation.')
    parser.add_argument('--team-dispersion-margin', type=float, default=0.0, help='Allowed increase in fleet mean separation before dispersion penalty applies.')
    parser.add_argument('--separation-recovery-weight', type=float, default=0.0, help='Positive reward weight for increasing pairwise separation inside near-miss band.')
    parser.add_argument('--entanglement-penalty-weight', type=float, default=0.0, help='Penalty weight for sustained close proximity (anti-orbital-lock). Ramps up after grace period.')
    parser.add_argument('--entanglement-distance', type=float, default=3.0, help='Distance threshold (m) below which entanglement counter increments.')
    parser.add_argument('--entanglement-grace-steps', type=int, default=30, help='Number of close-proximity steps allowed before entanglement penalty activates.')
    parser.add_argument('--entanglement-low-speed-penalty-weight', type=float, default=0.0, help='Penalty weight for low forward speed during entanglement (anti-orbital stalling).')
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
    parser.add_argument('--per-scenario-advantage-norm', action='store_true', help='Normalize advantages per-scenario instead of globally. Prevents reward-scale imbalance from causing gradient dominance by easier scenarios (anti-forgetting).')
    parser.add_argument('--value-loss-huber-delta', type=float, default=0.0, help='If >0, use a Huber-style critic value loss: squared error when |value_error|<=delta, linear beyond. Prevents large collision returns (e.g. -400) from making the critic loss explode. 0 (default) keeps pure squared error.')
    parser.add_argument('--scenario-balanced-loss', action='store_true', help='Weight PPO loss samples inversely to their scenario sample count. Equalizes per-scenario gradient contribution regardless of episode length (anti-forgetting).')
    parser.add_argument('--crossing-imitation-weight', type=float, default=0.0, help='Auxiliary trainer-side action imitation weight for the three_usv_crossing scenario. The final checkpoint remains a single neural MAPPO policy.')
    parser.add_argument('--crossing-imitation-weight-end', type=float, default=None, help='Final crossing imitation weight for linear annealing. If unset, crossing-imitation-weight stays constant.')
    parser.add_argument('--crossing-imitation-pretrain-epochs', type=int, default=0, help='Extra actor-only imitation epochs after each rollout update. Trainer-side only; final artifact remains one neural MAPPO checkpoint.')
    parser.add_argument('--crossing-imitation-clear-speed', type=float, default=0.32, help='Target forward speed for the highest-priority crossing vessel in the auxiliary imitation loss.')
    parser.add_argument('--crossing-imitation-middle-speed', type=float, default=0.10, help='Target forward speed for the middle-priority crossing vessel in the auxiliary imitation loss.')
    parser.add_argument('--crossing-imitation-yield-speed', type=float, default=0.0, help='Target forward speed for the yielding crossing vessel in the auxiliary imitation loss.')
    parser.add_argument('--crossing-imitation-clear-omega', type=float, default=-0.10, help='Target yaw rate for the highest-priority crossing vessel in the auxiliary imitation loss. Negative is starboard.')
    parser.add_argument('--crossing-imitation-middle-omega', type=float, default=-0.14, help='Target yaw rate for the middle-priority crossing vessel in the auxiliary imitation loss. Negative is starboard.')
    parser.add_argument('--crossing-imitation-yield-omega', type=float, default=-0.18, help='Target yaw rate for the yielding crossing vessel in the auxiliary imitation loss. Negative is starboard.')
    parser.add_argument('--crossing-imitation-eta-gate', type=float, default=0.95, help='Only apply crossing imitation while normalized conflict ETA is below this gate.')
    parser.add_argument('--crossing-imitation-phase-min', type=float, default=-0.90, help='Minimum conflict_phase for crossing imitation activation.')
    parser.add_argument('--crossing-imitation-phase-max', type=float, default=0.22, help='Maximum conflict_phase for crossing imitation activation.')
    parser.add_argument('--overtaking-imitation-weight', type=float, default=0.0, help='Trainer-side action target for overtaking scenarios: front keeps route, rear opens starboard corridor.')
    parser.add_argument('--overtaking-imitation-weight-end', type=float, default=None, help='Final overtaking-imitation auxiliary weight for linear annealing.')
    parser.add_argument('--overtaking-imitation-max-distance', type=float, default=13.0, help='Maximum distance-to-goal for overtaking-imitation samples.')
    parser.add_argument('--overtaking-imitation-front-min-speed', type=float, default=0.24, help='Minimum forward target for front/stand-on vessel.')
    parser.add_argument('--overtaking-imitation-front-max-omega', type=float, default=0.10, help='Raw yaw cap for front/stand-on vessel.')
    parser.add_argument('--overtaking-imitation-rear-approach-speed', type=float, default=0.12, help='Rear vessel target speed before corridor is opened.')
    parser.add_argument('--overtaking-imitation-rear-close-speed', type=float, default=0.055, help='Rear vessel target speed when too close and not offset.')
    parser.add_argument('--overtaking-imitation-rear-pass-speed', type=float, default=0.22, help='Rear vessel target speed once starboard corridor is opened.')
    parser.add_argument('--overtaking-imitation-close-separation', type=float, default=2.30, help='Separation below which rear vessel slows strongly unless offset.')
    parser.add_argument('--overtaking-imitation-release-separation', type=float, default=5.80, help='Nearest-neighbor distance where overtaking-imitation fades out.')
    parser.add_argument('--overtaking-imitation-target-starboard-offset', type=float, default=1.05, help='Starboard lateral offset target for rear vessel.')
    parser.add_argument('--overtaking-imitation-rear-omega', type=float, default=0.26, help='Starboard yaw target magnitude for rear vessel; negative omega is starboard.')
    parser.add_argument('--overtaking-imitation-omega-weight', type=float, default=1.0, help='Relative yaw loss weight for overtaking-imitation.')
    parser.add_argument('--overtaking-imitation-single-return-progress', type=float, default=0.70, help='Route-progress gate where single_usv_overtaking imitation starts preferring return-to-route targets after opening the pass corridor.')
    parser.add_argument('--overtaking-imitation-single-return-rel-x', type=float, default=-0.20, help='Nearest-lead relative x threshold for considering the single overtaker past the lead vessel.')
    parser.add_argument('--overtaking-imitation-single-return-min-separation', type=float, default=2.70, help='Minimum nearest-neighbor separation before single_usv_overtaking imitation may switch from pass corridor to return-to-route.')
    parser.add_argument('--overtaking-imitation-single-return-cte-start', type=float, default=0.70, help='Absolute CTE where single_usv_overtaking return-to-route yaw target begins.')
    parser.add_argument('--overtaking-imitation-single-return-cte-full', type=float, default=3.00, help='Absolute CTE where single_usv_overtaking return-to-route yaw target reaches full magnitude.')
    parser.add_argument('--overtaking-imitation-single-return-speed', type=float, default=0.32, help='Forward target while returning to route after a single_usv_overtaking pass.')
    parser.add_argument('--overtaking-imitation-single-return-omega', type=float, default=0.30, help='Maximum yaw target magnitude while returning to route after a single_usv_overtaking pass.')
    parser.add_argument('--overtaking-imitation-single-finish-distance', type=float, default=3.50, help='Distance-to-goal gate where single_usv_overtaking imitation switches to a straight finish target when CTE is controlled.')
    parser.add_argument('--overtaking-imitation-single-finish-speed', type=float, default=0.34, help='Forward target for the final single_usv_overtaking finish phase.')
    parser.add_argument('--overtaking-imitation-single-finish-max-omega', type=float, default=0.10, help='Yaw cap for the final single_usv_overtaking finish phase when CTE is controlled.')
    parser.add_argument('--near-goal-finish-weight', type=float, default=0.0, help='Trainer-side auxiliary action loss weight for finishing the final near-goal meters. Final artifact remains one neural MAPPO policy.')
    parser.add_argument('--near-goal-finish-weight-end', type=float, default=None, help='Final near-goal finish auxiliary weight for linear annealing. If unset, near-goal-finish-weight stays constant.')
    parser.add_argument('--near-goal-finish-distance', type=float, default=2.2, help='Distance to goal below which the trainer-side finish auxiliary can activate.')
    parser.add_argument('--near-goal-finish-goal-tolerance', type=float, default=0.8, help='Goal tolerance used by the trainer-side finish auxiliary to avoid pushing already-complete samples.')
    parser.add_argument('--near-goal-finish-phase-min', type=float, default=0.0, help='Minimum conflict_phase for the finish auxiliary. Use >=0 to target post-conflict completion.')
    parser.add_argument('--near-goal-finish-target-speed', type=float, default=0.18, help='Target forward speed for the trainer-side finish auxiliary near goal.')
    parser.add_argument('--near-goal-finish-max-omega', type=float, default=0.18, help='Maximum heading-correction yaw target used by the finish auxiliary.')
    parser.add_argument('--near-goal-finish-omega-weight', type=float, default=0.35, help='Relative loss weight for finish auxiliary yaw-rate target.')
    parser.add_argument('--near-goal-finish-crossing-only', action='store_true', help='Apply the finish auxiliary only to three_usv_crossing samples.')
    parser.add_argument('--near-goal-finish-exclude-overtaking', action='store_true', help='Disable the finish auxiliary on single_usv_overtaking / two_usv_overtaking samples, where the goal lies ahead in-lane and stopping/aligning conflicts with passing the lead vessel.')
    parser.add_argument('--lagging-finish-weight', type=float, default=0.0, help='Trainer-side auxiliary loss weight that only activates after part of the team has reached the goal and pushes remaining near-goal crossing agents to finish.')
    parser.add_argument('--lagging-finish-weight-end', type=float, default=None, help='Final lagging-finish auxiliary weight for linear annealing. If unset, lagging-finish-weight stays constant.')
    parser.add_argument('--lagging-finish-distance', type=float, default=4.0, help='Distance-to-goal band for lagging agents once teammate completion is already high.')
    parser.add_argument('--lagging-finish-goal-tolerance', type=float, default=0.8, help='Goal tolerance used by the lagging-finish auxiliary.')
    parser.add_argument('--lagging-finish-phase-min', type=float, default=0.0, help='Minimum conflict_phase for lagging-finish activation.')
    parser.add_argument('--lagging-finish-target-speed', type=float, default=0.14, help='Target forward speed for lagging agents after teammates have reached goals.')
    parser.add_argument('--lagging-finish-min-speed-scale', type=float, default=0.25, help='Minimum fraction of lagging-finish-target-speed used before an agent enters the goal tolerance.')
    parser.add_argument('--lagging-finish-raw-target', action='store_true', help='Use the stored raw navigation command as the lagging-finish trainer target when active, clipped by target-speed and max-omega.')
    parser.add_argument('--lagging-finish-max-omega', type=float, default=0.10, help='Maximum yaw-rate correction target for lagging-finish auxiliary.')
    parser.add_argument('--lagging-finish-omega-weight', type=float, default=0.25, help='Relative yaw-rate loss weight for lagging-finish auxiliary.')
    parser.add_argument('--lagging-finish-hold-omega-only', action='store_true', help='Apply lagging-finish yaw-rate loss only to already-reached hold samples, while lagging samples receive linear-speed guidance only.')
    parser.add_argument('--lagging-finish-hold-weight', type=float, default=1.0, help='Extra multiplier for already-reached hold samples inside the lagging-finish auxiliary.')
    parser.add_argument('--lagging-finish-min-team-completion', type=float, default=0.60, help='Minimum global goal_completion_ratio before lagging-finish activates. For three agents, 0.60 targets the 2/3-complete state.')
    parser.add_argument('--lagging-finish-max-team-completion', type=float, default=0.999, help='Maximum global goal_completion_ratio before lagging-finish deactivates.')
    parser.add_argument('--lagging-finish-near-team-tolerance', type=float, default=0.0, help='Optional distance-to-goal tolerance used to treat teammates as near-complete for lagging-finish activation. 0 disables this trainer-only gate.')
    parser.add_argument('--lagging-finish-min-team-separation', type=float, default=0.0, help='Optional minimum fleet separation required before lagging-finish activates. 0 disables this gate.')
    parser.add_argument('--lagging-finish-safe-team-separation', type=float, default=0.0, help='Optional separation where lagging-finish reaches full target speed; below this, the trainer target speed is reduced toward zero. 0 disables this trainer-only scaling.')
    parser.add_argument('--lagging-finish-safe-team-separation-power', type=float, default=1.0, help='Power applied to lagging-finish safe-team-separation speed scaling. Values above 1 reduce target speed more aggressively near unsafe separation.')
    parser.add_argument('--lagging-finish-crossing-only', action='store_true', help='Apply the lagging-finish auxiliary only to three_usv_crossing samples.')
    parser.add_argument('--lagging-finish-hold-reached', action='store_true', help='Also train already-reached agents to hold still while lagging teammates finish.')
    parser.add_argument('--team-safety-brake-weight', type=float, default=0.0, help='Trainer-side auxiliary loss weight that slows unfinished agents when part of the team has reached goals but fleet separation is still unsafe.')
    parser.add_argument('--team-safety-brake-weight-end', type=float, default=None, help='Final team-safety-brake auxiliary weight for linear annealing. If unset, team-safety-brake-weight stays constant.')
    parser.add_argument('--team-safety-brake-goal-tolerance', type=float, default=0.8, help='Goal tolerance used to exclude already-reached agents from team-safety-brake forward-speed targets.')
    parser.add_argument('--team-safety-brake-max-distance', type=float, default=12.0, help='Maximum distance-to-goal for unfinished agents affected by team-safety-brake.')
    parser.add_argument('--team-safety-brake-phase-min', type=float, default=-1.0, help='Minimum conflict_phase for team-safety-brake activation.')
    parser.add_argument('--team-safety-brake-min-team-completion', type=float, default=0.30, help='Minimum global or near-team completion ratio before team-safety-brake activates.')
    parser.add_argument('--team-safety-brake-max-team-completion', type=float, default=0.999, help='Maximum global team completion ratio before team-safety-brake deactivates.')
    parser.add_argument('--team-safety-brake-near-team-tolerance', type=float, default=0.0, help='Optional distance-to-goal tolerance used to treat teammates as near-complete for team-safety-brake activation. 0 disables this gate.')
    parser.add_argument('--team-safety-brake-safe-separation', type=float, default=1.2, help='Fleet separation where team-safety-brake applies full low-speed target.')
    parser.add_argument('--team-safety-brake-release-separation', type=float, default=2.4, help='Fleet separation where team-safety-brake fades out completely.')
    parser.add_argument('--team-safety-brake-target-speed', type=float, default=0.0, help='Maximum forward-speed target at the edge of team-safety-brake activation; target fades toward zero at unsafe separation.')
    parser.add_argument('--team-safety-brake-omega-weight', type=float, default=0.0, help='Relative yaw-rate loss weight for nearest-neighbor escape turning inside team-safety-brake.')
    parser.add_argument('--team-safety-brake-target-omega', type=float, default=0.0, help='Maximum yaw-rate target for nearest-neighbor escape turning inside team-safety-brake.')
    parser.add_argument('--team-safety-brake-turn-mode', choices=('away', 'starboard'), default='away', help='Yaw-rate target direction inside team-safety-brake: nearest-neighbor escape or fixed starboard commitment.')
    parser.add_argument('--team-safety-brake-head-on-starboard-threshold', type=float, default=0.0, help='When turn mode is away, use a starboard yaw target if the nearest-neighbor bearing is within this absolute radian threshold.')
    parser.add_argument('--team-safety-brake-require-neighbor', action='store_true', help='Require at least one encoded neighbor inside release separation before team-safety-brake activates.')
    parser.add_argument('--team-safety-brake-local-danger', action='store_true', help='Use the active agent nearest-neighbor distance, rather than global team-min separation, to scale team-safety-brake linear-speed danger.')
    parser.add_argument('--team-safety-brake-power', type=float, default=1.0, help='Power applied to team-safety-brake danger weighting. Values above 1 focus the loss closer to unsafe separation.')
    parser.add_argument('--team-safety-brake-cpa-danger', action='store_true', help='Also activate team-safety-brake from predicted low-DCPA closing encounters before nearest-neighbor distance is small.')
    parser.add_argument('--team-safety-brake-cpa-lookahead-distance', type=float, default=0.0, help='Maximum neighbor distance for CPA-based team-safety-brake activation. 0 disables the CPA lookahead distance gate.')
    parser.add_argument('--team-safety-brake-cpa-time-horizon', type=float, default=0.0, help='Maximum TCPA seconds for CPA-based team-safety-brake activation. 0 disables the CPA lookahead time gate.')
    parser.add_argument('--team-safety-brake-cpa-dcpa-target', type=float, default=0.0, help='Predicted DCPA target in meters for CPA-based team-safety-brake activation. 0 disables the CPA lookahead DCPA gate.')
    parser.add_argument('--team-safety-brake-cpa-closing-speed-min', type=float, default=0.02, help='Minimum positive range-closing speed for CPA-based team-safety-brake activation.')
    parser.add_argument('--team-safety-brake-crossing-only', action='store_true', help='Apply the team-safety-brake auxiliary only to three_usv_crossing samples.')
    parser.add_argument('--team-safety-brake-random-only', action='store_true', help='Apply the team-safety-brake auxiliary only to random encounter samples.')
    parser.add_argument('--team-safety-brake-random-yield-agents-only', action='store_true', help='When team-safety-brake is random-only, train only higher-index give-way agents so lower-index stand-on agents keep route progress.')
    parser.add_argument('--team-safety-brake-random-yield-agent-weight', type=float, default=1.0, help='Role multiplier for higher-index give-way agents when random-yield-agents-only is enabled.')
    parser.add_argument('--team-safety-brake-random-standon-agent-weight', type=float, default=0.0, help='Role multiplier for lower-index stand-on agents when random-yield-agents-only is enabled. 0 preserves hard yield-only gating.')
    parser.add_argument('--random-deconflict-weight', type=float, default=0.0, help='Trainer-side auxiliary action loss that role-staggers random encounters under CPA threat.')
    parser.add_argument('--random-deconflict-weight-end', type=float, default=None, help='Final random-deconflict auxiliary weight for linear annealing. If unset, random-deconflict-weight stays constant.')
    parser.add_argument('--random-deconflict-pretrain-epochs', type=int, default=0, help='Extra actor-only epochs per rollout for fitting random-deconflict action targets after PPO updates.')
    parser.add_argument('--random-deconflict-pretrain-learning-rate', type=float, default=0.0, help='Optional separate actor-only learning rate for random-deconflict pretrain; <=0 reuses the main optimizer.')
    parser.add_argument('--random-deconflict-pretrain-max-grad-norm', type=float, default=0.0, help='Optional separate grad norm for random-deconflict pretrain; <=0 uses --max-grad-norm.')
    parser.add_argument('--random-deconflict-role-mode', choices=['agent-index', 'priority', 'priority-delta', 'route-eta-delta'], default='agent-index', help='Role signal used for random-deconflict targets. priority uses ego crossing_priority; priority-delta uses route_priority_delta on the active neighbor; route-eta-delta yields when ego arrives later at the shared conflict point (positive neighbor route_eta_delta), typically with --random-encounter-route-priority.')
    parser.add_argument('--random-deconflict-priority-yield-threshold', type=float, default=-0.10, help='In priority role mode, samples with crossing_priority at or below this value are treated as give-way.')
    parser.add_argument('--random-deconflict-priority-delta-yield-threshold', type=float, default=-0.01, help='In priority-delta role mode, yield when own-minus-neighbor priority delta is at or below this value.')
    parser.add_argument('--random-deconflict-route-eta-yield-threshold', type=float, default=0.02, help='In route-eta-delta role mode, yield when normalized (own_eta-neighbor_eta) exceeds this positive threshold.')
    parser.add_argument('--random-deconflict-goal-tolerance', type=float, default=0.8, help='Goal tolerance used to ignore completed random-deconflict samples.')
    parser.add_argument('--random-deconflict-max-distance', type=float, default=13.0, help='Maximum distance-to-goal for random-deconflict samples.')
    parser.add_argument('--random-deconflict-lookahead-distance', type=float, default=10.8, help='Maximum neighbor distance for random-deconflict CPA activation.')
    parser.add_argument('--random-deconflict-time-horizon', type=float, default=30.0, help='Maximum TCPA seconds for random-deconflict CPA activation.')
    parser.add_argument('--random-deconflict-dcpa-target', type=float, default=1.45, help='Predicted DCPA target in meters for random-deconflict activation.')
    parser.add_argument('--random-deconflict-closing-speed-min', type=float, default=0.004, help='Minimum positive range-closing speed for random-deconflict activation.')
    parser.add_argument('--random-deconflict-local-danger', action='store_true', help='Also activate random-deconflict from nearest-neighbor local proximity, even if CPA score is weak.')
    parser.add_argument('--random-deconflict-safe-separation', type=float, default=1.30, help='Nearest-neighbor separation where random-deconflict local danger is full strength.')
    parser.add_argument('--random-deconflict-release-separation', type=float, default=2.40, help='Nearest-neighbor separation where random-deconflict local danger fades to zero.')
    parser.add_argument('--random-deconflict-critical-separation', type=float, default=0.0, help='If >0, force local random-deconflict danger to 1.0 below this nearest-neighbor separation instead of fading it by closing speed.')
    parser.add_argument('--random-deconflict-power', type=float, default=1.0, help='Power applied to random-deconflict CPA danger weighting.')
    parser.add_argument('--random-deconflict-standon-speed', type=float, default=0.30, help='Forward speed target for the lower-index stand-on agent in random encounters.')
    parser.add_argument('--random-deconflict-standon-close-separation', type=float, default=0.0, help='If >0, cap stand-on random-deconflict speed when nearest-neighbor distance is below this separation.')
    parser.add_argument('--random-deconflict-standon-close-speed', type=float, default=0.08, help='Stand-on random-deconflict speed cap used inside random-deconflict-standon-close-separation.')
    parser.add_argument('--random-deconflict-yield-speed', type=float, default=0.04, help='Maximum forward speed target for higher-index give-way agents under random-deconflict danger.')
    parser.add_argument('--random-deconflict-yield-danger-scale', type=float, default=1.0, help='Fraction of random-deconflict danger used to fade give-way speed. 1.0 preserves a full stop at danger=1; lower values keep residual progress.')
    parser.add_argument('--random-deconflict-standon-omega', type=float, default=0.12, help='Starboard yaw-rate target magnitude for the stand-on agent in random encounters.')
    parser.add_argument('--random-deconflict-yield-omega', type=float, default=0.50, help='Starboard yaw-rate target magnitude for give-way agents in random encounters.')
    parser.add_argument('--random-deconflict-turn-mode', choices=['starboard', 'away'], default='starboard', help='Yaw target direction for random-deconflict. starboard preserves legacy fixed-right targets; away turns away from the active neighbour bearing.')
    parser.add_argument('--random-deconflict-omega-weight', type=float, default=1.0, help='Relative yaw-rate loss weight for random-deconflict starboard commitment.')
    parser.add_argument('--random-deconflict-standon-weight', type=float, default=0.35, help='Loss multiplier for lower-index stand-on random-deconflict samples.')
    parser.add_argument('--random-deconflict-yield-weight', type=float, default=1.0, help='Loss multiplier for higher-index give-way random-deconflict samples.')
    parser.add_argument('--random-role-balance-weight', type=float, default=0.0, help='Hinge-style random encounter role-balance loss. Stand-on is only pushed above a minimum speed while yield agents are only capped below a maximum speed.')
    parser.add_argument('--random-role-balance-weight-end', type=float, default=None, help='Final role-balance loss weight for linear annealing. If unset, random-role-balance-weight stays constant.')
    parser.add_argument('--random-role-balance-pretrain-epochs', type=int, default=0, help='Extra actor-only epochs per rollout for role-balance hinge constraints after PPO updates.')
    parser.add_argument('--random-role-balance-pretrain-learning-rate', type=float, default=0.0, help='Optional separate actor-only learning rate for role-balance pretrain; <=0 reuses the main optimizer.')
    parser.add_argument('--random-role-balance-pretrain-max-grad-norm', type=float, default=0.0, help='Optional separate grad norm for role-balance pretrain; <=0 uses --max-grad-norm.')
    parser.add_argument('--random-role-balance-min-danger', type=float, default=0.05, help='Minimum random-deconflict danger score for role-balance hinge constraints.')
    parser.add_argument('--random-role-balance-standon-min-speed', type=float, default=0.28, help='Minimum action linear speed for stand-on agents under random role-balance constraints.')
    parser.add_argument('--random-role-balance-standon-close-separation', type=float, default=0.0, help='If >0, disables the stand-on speed floor when nearest-neighbor distance is below this separation.')
    parser.add_argument('--random-role-balance-yield-max-speed', type=float, default=0.10, help='Maximum action linear speed for yield agents under random role-balance constraints.')
    parser.add_argument('--random-role-balance-yield-min-starboard-omega', type=float, default=0.22, help='Minimum starboard yaw-rate magnitude for yield agents under random role-balance constraints.')
    parser.add_argument('--random-role-balance-standon-weight', type=float, default=1.0, help='Loss multiplier for stand-on speed-floor violations in role-balance constraints.')
    parser.add_argument('--random-role-balance-yield-weight', type=float, default=1.0, help='Loss multiplier for yield speed-cap violations in role-balance constraints.')
    parser.add_argument('--random-role-balance-omega-weight', type=float, default=0.6, help='Loss multiplier for yield starboard-omega hinge violations in role-balance constraints.')
    parser.add_argument('--random-pairwise-role-guard-weight', type=float, default=0.0, help='Local nearest-pair guard loss for random encounters. Enforces pair-relative yield/stand-on behavior near collision distance.')
    parser.add_argument('--random-pairwise-role-guard-weight-end', type=float, default=None, help='Final pairwise role guard weight for linear annealing. If unset, random-pairwise-role-guard-weight stays constant.')
    parser.add_argument('--random-pairwise-role-guard-pretrain-epochs', type=int, default=0, help='Extra actor-only epochs per rollout for pairwise role guard constraints after PPO updates.')
    parser.add_argument('--random-pairwise-role-guard-pretrain-learning-rate', type=float, default=0.0, help='Optional separate actor-only learning rate for pairwise role guard pretrain; <=0 reuses the main optimizer.')
    parser.add_argument('--random-pairwise-role-guard-pretrain-max-grad-norm', type=float, default=0.0, help='Optional separate grad norm for pairwise role guard pretrain; <=0 uses --max-grad-norm.')
    parser.add_argument('--random-pairwise-role-guard-safe-separation', type=float, default=0.82, help='Nearest-neighbor separation where pairwise guard danger reaches full strength.')
    parser.add_argument('--random-pairwise-role-guard-release-separation', type=float, default=1.45, help='Nearest-neighbor separation where pairwise guard fades out.')
    parser.add_argument('--random-pairwise-role-guard-min-danger', type=float, default=0.10, help='Minimum local pair danger required for pairwise role guard samples.')
    parser.add_argument('--random-pairwise-role-guard-yield-speed', type=float, default=0.015, help='Maximum forward speed target for yield agents under pairwise guard.')
    parser.add_argument('--random-pairwise-role-guard-standon-close-speed', type=float, default=0.12, help='Forward speed cap for stand-on agents inside pairwise guard release distance.')
    parser.add_argument('--random-pairwise-role-guard-yield-omega', type=float, default=0.42, help='Pair-relative away-turn yaw magnitude for yield agents under pairwise guard.')
    parser.add_argument('--random-pairwise-role-guard-standon-omega', type=float, default=0.14, help='Pair-relative away-turn yaw magnitude for stand-on agents under pairwise guard.')
    parser.add_argument('--random-pairwise-role-guard-linear-weight', type=float, default=1.0, help='Relative speed-cap loss weight for pairwise role guard.')
    parser.add_argument('--random-pairwise-role-guard-omega-weight', type=float, default=2.0, help='Relative away-turn yaw loss weight for pairwise role guard.')
    parser.add_argument('--random-pairwise-role-guard-yield-weight', type=float, default=2.0, help='Sample multiplier for yield agents under pairwise role guard.')
    parser.add_argument('--random-pairwise-role-guard-standon-weight', type=float, default=0.55, help='Sample multiplier for stand-on agents under pairwise role guard.')
    parser.add_argument('--random-safe-finish-weight', type=float, default=0.0, help='Trainer-side auxiliary loss that restores forward progress in random encounters only after CPA/local deconflict danger has cleared.')
    parser.add_argument('--random-safe-finish-weight-end', type=float, default=None, help='Final random-safe-finish auxiliary weight for linear annealing. If unset, random-safe-finish-weight stays constant.')
    parser.add_argument('--random-safe-finish-goal-tolerance', type=float, default=0.8, help='Goal tolerance used to ignore completed random-safe-finish samples.')
    parser.add_argument('--random-safe-finish-max-distance', type=float, default=13.0, help='Maximum distance-to-goal for unfinished random-safe-finish samples.')
    parser.add_argument('--random-safe-finish-phase-min', type=float, default=-1.0, help='Minimum conflict_phase for random-safe-finish activation.')
    parser.add_argument('--random-safe-finish-min-team-separation', type=float, default=2.7, help='Minimum fleet separation required before random-safe-finish can activate.')
    parser.add_argument('--random-safe-finish-full-team-separation', type=float, default=4.0, help='Fleet separation where random-safe-finish reaches full target speed.')
    parser.add_argument('--random-safe-finish-min-neighbor-separation', type=float, default=2.7, help='Minimum active-agent nearest-neighbor distance required before random-safe-finish can activate.')
    parser.add_argument('--random-safe-finish-local-separation-scale', action='store_true', help='Use each agent nearest-neighbor distance, rather than fleet minimum separation, to scale random-safe-finish target speed.')
    parser.add_argument('--random-safe-finish-max-abs-cte', type=float, default=0.0, help='If >0, block random-safe-finish acceleration when absolute CTE exceeds this value so off-route recovery can take over.')
    parser.add_argument('--random-safe-finish-yield-release-min-separation', type=float, default=0.0, help='If >0, low-priority random agents at or above this nearest-neighbor distance can enter safe-finish even when CPA threat remains.')
    parser.add_argument('--random-safe-finish-yield-release-max-closing-speed', type=float, default=0.02, help='Maximum nearest-neighbor closing speed for low-priority safe-finish release overlap.')
    parser.add_argument('--random-safe-finish-yield-release-min-route-progress', type=float, default=0.0, help='Minimum route progress for low-priority safe-finish release overlap.')
    parser.add_argument('--random-safe-finish-max-cpa-score', type=float, default=0.0, help='Maximum CPA threat score allowed for random-safe-finish activation.')
    parser.add_argument('--random-safe-finish-max-local-score', type=float, default=0.0, help='Maximum local proximity threat score allowed for random-safe-finish activation.')
    parser.add_argument('--random-safe-finish-target-speed', type=float, default=0.18, help='Target forward speed for random-safe-finish samples after deconflict danger has cleared.')
    parser.add_argument('--random-safe-finish-min-speed-scale', type=float, default=0.35, help='Minimum fraction of random-safe-finish-target-speed used for unfinished samples.')
    parser.add_argument('--random-safe-finish-hold-distance', type=float, default=0.0, help='Distance-to-goal band where random-safe-finish holds reached agents at zero speed. 0 disables hold samples.')
    parser.add_argument('--random-safe-finish-hold-weight', type=float, default=1.0, help='Per-sample loss multiplier for random-safe-finish hold samples.')
    parser.add_argument('--random-safe-finish-low-priority-threshold', type=float, default=-0.5, help='crossing_priority value at or below which random-safe-finish applies the low-priority speed multiplier.')
    parser.add_argument('--random-safe-finish-low-priority-speed-multiplier', type=float, default=1.0, help='Forward target multiplier for low-priority random-safe-finish samples after danger has cleared.')
    parser.add_argument('--random-safe-finish-max-omega', type=float, default=0.10, help='Maximum heading-correction yaw target used by random-safe-finish.')
    parser.add_argument('--random-safe-finish-omega-weight', type=float, default=0.12, help='Relative yaw-rate loss weight for random-safe-finish heading correction.')
    parser.add_argument('--random-goal-hold-weight', type=float, default=0.0, help='Trainer-side auxiliary loss that holds random-encounter agents still once they are inside the goal band.')
    parser.add_argument('--random-goal-hold-weight-end', type=float, default=None, help='Final random-goal-hold auxiliary weight for linear annealing. If unset, random-goal-hold-weight stays constant.')
    parser.add_argument('--random-goal-hold-distance', type=float, default=0.0, help='Distance-to-goal band where random-goal-hold trains zero forward speed. 0 disables the auxiliary.')
    parser.add_argument('--random-goal-hold-phase-min', type=float, default=-1.0, help='Minimum conflict_phase for random-goal-hold activation.')
    parser.add_argument('--random-goal-hold-target-speed', type=float, default=0.0, help='Forward speed target for random-goal-hold samples.')
    parser.add_argument('--random-goal-hold-target-omega', type=float, default=0.0, help='Yaw-rate target for random-goal-hold samples.')
    parser.add_argument('--random-goal-hold-omega-weight', type=float, default=1.0, help='Relative yaw-rate loss weight for random-goal-hold samples.')
    parser.add_argument('--random-offroute-finish-weight', type=float, default=0.0, help='Trainer-side auxiliary loss that recovers random-encounter agents that have high route progress but remain far from goal with large CTE.')
    parser.add_argument('--random-offroute-finish-weight-end', type=float, default=None, help='Final random-offroute-finish auxiliary weight for linear annealing. If unset, random-offroute-finish-weight stays constant.')
    parser.add_argument('--random-offroute-finish-goal-tolerance', type=float, default=0.8, help='Goal tolerance used to ignore completed random-offroute-finish samples.')
    parser.add_argument('--random-offroute-finish-max-distance', type=float, default=13.0, help='Maximum distance-to-goal for random-offroute-finish samples.')
    parser.add_argument('--random-offroute-finish-min-distance', type=float, default=1.60, help='Minimum distance-to-goal for random-offroute-finish activation.')
    parser.add_argument('--random-offroute-finish-route-progress-min', type=float, default=0.82, help='Minimum route_progress required for random-offroute-finish activation.')
    parser.add_argument('--random-offroute-finish-min-abs-cte', type=float, default=1.20, help='Minimum absolute cross-track error required for random-offroute-finish activation.')
    parser.add_argument('--random-offroute-finish-full-abs-cte', type=float, default=3.00, help='Absolute CTE where random-offroute-finish reaches full correction urgency.')
    parser.add_argument('--random-offroute-finish-phase-min', type=float, default=-1.0, help='Minimum conflict_phase for random-offroute-finish activation.')
    parser.add_argument('--random-offroute-finish-min-team-separation', type=float, default=2.75, help='Minimum fleet separation required before random-offroute-finish can activate.')
    parser.add_argument('--random-offroute-finish-min-neighbor-separation', type=float, default=2.90, help='Minimum active-agent nearest-neighbor distance required before random-offroute-finish can activate.')
    parser.add_argument('--random-offroute-finish-max-cpa-score', type=float, default=0.0, help='Maximum CPA threat score allowed for random-offroute-finish activation.')
    parser.add_argument('--random-offroute-finish-max-local-score', type=float, default=0.0, help='Maximum local proximity threat score allowed for random-offroute-finish activation.')
    parser.add_argument('--random-offroute-finish-allow-threat-overlap', action='store_true', help='Allow random-offroute-finish to overlap with CPA/local deconflict threats once its route-progress and CTE gates are met.')
    parser.add_argument('--random-offroute-finish-target-speed', type=float, default=0.14, help='Forward speed target for random-offroute-finish samples before CTE slowdown.')
    parser.add_argument('--random-offroute-finish-min-speed', type=float, default=0.06, help='Minimum forward speed target retained at full random-offroute-finish CTE urgency.')
    parser.add_argument('--random-offroute-finish-max-omega', type=float, default=0.18, help='Maximum heading-correction yaw target used by random-offroute-finish.')
    parser.add_argument('--random-offroute-finish-omega-reference', type=float, default=0.65, help='Heading-error magnitude where random-offroute-finish reaches full yaw target.')
    parser.add_argument('--random-offroute-finish-omega-weight', type=float, default=0.55, help='Relative yaw-rate loss weight for random-offroute-finish heading correction.')
    parser.add_argument('--random-cte-recovery-weight', type=float, default=0.0, help='Trainer-side auxiliary that recovers high-CTE random encounter agents without relying on route_progress.')
    parser.add_argument('--random-cte-recovery-weight-end', type=float, default=None, help='Final random-cte-recovery auxiliary weight for linear annealing.')
    parser.add_argument('--random-cte-recovery-goal-tolerance', type=float, default=0.8, help='Goal tolerance used to ignore completed random-cte-recovery samples.')
    parser.add_argument('--random-cte-recovery-max-distance', type=float, default=13.0, help='Maximum distance-to-goal for random-cte-recovery samples.')
    parser.add_argument('--random-cte-recovery-min-abs-cte', type=float, default=1.10, help='Minimum absolute CTE for random-cte-recovery activation.')
    parser.add_argument('--random-cte-recovery-full-abs-cte', type=float, default=3.00, help='Absolute CTE where random-cte-recovery reaches full slowdown urgency.')
    parser.add_argument('--random-cte-source', choices=('clipped', 'raw'), default='clipped', help='CTE feature used by random offroute/recovery/clear-ahead auxiliary gates. raw uses policy-visible raw CTE when available.')
    parser.add_argument('--random-cte-recovery-min-neighbor-separation', type=float, default=0.85, help='Minimum nearest-neighbor distance required for random-cte-recovery activation.')
    parser.add_argument('--random-cte-recovery-agent-index', action='append', dest='random_cte_recovery_agent_indices', type=int, default=None, help='Optional 0-based agent index allowed to receive random-cte-recovery. Repeatable. Defaults to all agents.')
    parser.add_argument('--random-cte-recovery-all-agents', action='store_true', help='Ignore any random-cte-recovery agent-index filters inherited from wrapper scripts.')
    parser.add_argument('--random-cte-recovery-allow-threat-overlap', action='store_true', help='Allow random-cte-recovery to overlap with CPA/local deconflict threats when the nearest-neighbor gate is satisfied.')
    parser.add_argument('--random-cte-recovery-target-speed', type=float, default=0.18, help='Forward target speed at the random-cte-recovery activation threshold.')
    parser.add_argument('--random-cte-recovery-min-speed', type=float, default=0.07, help='Forward target speed at full random-cte-recovery urgency.')
    parser.add_argument('--random-cte-recovery-speed-cte-slowdown', type=float, default=1.0, help='How strongly CTE urgency slows the random-cte-recovery speed target. 1 preserves legacy slowdown; 0 keeps target speed.')
    parser.add_argument('--random-cte-recovery-speed-heading-gate', type=float, default=0.0, help='How strongly recovery speed is gated by alignment to the selected recovery heading. 0 preserves legacy speed target; 1 fully applies cosine alignment.')
    parser.add_argument('--random-cte-recovery-max-omega', type=float, default=0.30, help='Maximum goal-heading yaw-rate target for random-cte-recovery.')
    parser.add_argument('--random-cte-recovery-omega-reference', type=float, default=0.55, help='Heading error magnitude mapped to max omega for random-cte-recovery.')
    parser.add_argument('--random-cte-recovery-linear-weight', type=float, default=1.00, help='Relative forward-speed loss weight for random-cte-recovery. 1 preserves legacy weighting.')
    parser.add_argument('--random-cte-recovery-omega-weight', type=float, default=1.00, help='Relative yaw-rate loss weight for random-cte-recovery.')
    parser.add_argument('--random-cte-recovery-omega-mode', choices=('goal-heading', 'signed-cte', 'signed-cte-inverted', 'goal-cte-lookahead'), default='goal-heading', help='Yaw target source for random-cte-recovery. goal-cte-lookahead blends goal heading with a CTE lookahead bias.')
    parser.add_argument('--random-cte-recovery-cte-lookahead', type=float, default=4.0, help='Lookahead distance used by goal-cte-lookahead omega mode.')
    parser.add_argument('--random-cte-recovery-cte-heading-scale', type=float, default=1.0, help='Scale applied to the CTE lookahead heading bias.')
    parser.add_argument('--random-recovery-pretrain-epochs', type=int, default=0, help='Extra actor-only epochs per rollout for random CTE/offroute/clear recovery constraints after PPO updates.')
    parser.add_argument('--random-recovery-pretrain-learning-rate', type=float, default=0.0, help='Optional separate actor-only learning rate for random recovery pretrain; <=0 reuses the main optimizer.')
    parser.add_argument('--random-recovery-pretrain-max-grad-norm', type=float, default=0.0, help='Optional separate grad norm for random recovery pretrain; <=0 uses --max-grad-norm.')
    parser.add_argument('--random-recovery-pretrain-offroute-scale', type=float, default=1.0, help='Multiplier for offroute-finish loss inside random recovery pretrain only.')
    parser.add_argument('--random-recovery-pretrain-cte-scale', type=float, default=1.0, help='Multiplier for CTE-recovery loss inside random recovery pretrain only.')
    parser.add_argument('--random-recovery-pretrain-clear-scale', type=float, default=1.0, help='Multiplier for clear-ahead loss inside random recovery pretrain only.')
    parser.add_argument('--random-recovery-pretrain-late-lagging-scale', type=float, default=1.0, help='Multiplier for random-late-lagging loss inside random recovery pretrain only.')
    parser.add_argument('--random-recovery-safety-gate-scale', type=float, default=1.0, help='Multiplier applied to random offroute/CTE recovery samples while random deconflict or pairwise role guard is active. 1 keeps legacy behavior; 0 fully suppresses recovery during active safety conflicts.')
    parser.add_argument('--random-recovery-safety-gate-mode', choices=['batch', 'sample'], default='batch', help='How random recovery safety gating is applied when scale < 1. batch scales the whole minibatch if any safety conflict is active; sample only scales overlapping samples.')
    parser.add_argument('--goal-heading-omega-sign', type=float, default=-1.0, help='Sign multiplier for goal-heading omega targets. -1 preserves legacy targets; +1 matches yaw += omega * dt with heading_error=target_yaw-yaw.')
    parser.add_argument('--random-clear-ahead-weight', type=float, default=0.0, help='Trainer-side action loss for random encounters when the forward route cone is clear.')
    parser.add_argument('--random-clear-ahead-weight-end', type=float, default=None, help='Final random-clear-ahead auxiliary weight for linear annealing.')
    parser.add_argument('--random-clear-ahead-scenario', action='append', dest='random_clear_ahead_scenarios', default=None, help='Scenario name where random-clear-ahead may activate. Repeatable. Defaults to random encounter scenarios.')
    parser.add_argument('--random-clear-ahead-distance', type=float, default=5.0, help='Forward-cone blocker distance for random-clear-ahead activation.')
    parser.add_argument('--random-clear-ahead-bearing-deg', type=float, default=35.0, help='Half-angle of the forward cone for random-clear-ahead activation.')
    parser.add_argument('--random-clear-ahead-cone-mode', choices=['heading', 'goal'], default='goal', help='Forward cone reference. heading uses ego yaw; goal uses the current target/route heading from heading_error.')
    parser.add_argument('--random-clear-ahead-goal-tolerance', type=float, default=1.0, help='Goal tolerance used to ignore completed random-clear-ahead samples.')
    parser.add_argument('--random-clear-ahead-max-distance', type=float, default=13.0, help='Maximum distance-to-goal for random-clear-ahead samples.')
    parser.add_argument('--random-clear-ahead-min-route-progress', type=float, default=0.0, help='Minimum route_progress for random-clear-ahead activation.')
    parser.add_argument('--random-clear-ahead-max-route-progress', type=float, default=1.0, help='Maximum route_progress for random-clear-ahead activation.')
    parser.add_argument('--random-clear-ahead-min-abs-cte', type=float, default=0.0, help='Minimum absolute CTE for random-clear-ahead activation. 0 disables the lower CTE gate.')
    parser.add_argument('--random-clear-ahead-max-abs-cte', type=float, default=0.0, help='Maximum absolute CTE for random-clear-ahead activation. 0 disables the upper CTE gate.')
    parser.add_argument('--random-clear-ahead-min-neighbor-separation', type=float, default=0.0, help='If >0, require all-around nearest-neighbor separation at least this large before random-clear-ahead activates.')
    parser.add_argument('--random-clear-ahead-agent-index', action='append', dest='random_clear_ahead_agent_indices', type=int, default=None, help='Optional 0-based agent index allowed to receive random-clear-ahead. Repeatable. Defaults to all agents.')
    parser.add_argument('--random-clear-ahead-max-cpa-score', type=float, default=0.0, help='Maximum CPA score allowed for random-clear-ahead activation.')
    parser.add_argument('--random-clear-ahead-max-local-score', type=float, default=0.0, help='Maximum local random-deconflict danger score allowed for random-clear-ahead activation.')
    parser.add_argument('--random-clear-ahead-exclude-deconflict', action='store_true', help='Disable random-clear-ahead on samples where random-deconflict is active.')
    parser.add_argument('--random-clear-ahead-target-source', choices=['raw', 'constant'], default='raw', help='Linear/yaw target source for random-clear-ahead. raw imitates base navigation; constant uses configured speed and heading-error yaw.')
    parser.add_argument('--random-clear-ahead-target-speed', type=float, default=0.30, help='Forward speed target when random-clear-ahead-target-source=constant or raw speed is below min speed.')
    parser.add_argument('--random-clear-ahead-min-speed', type=float, default=0.18, help='Minimum target forward speed retained for clear-ahead route discipline.')
    parser.add_argument('--random-clear-ahead-speed-cte-slowdown', type=float, default=0.0, help='How strongly clear-ahead target speed decays as absolute CTE approaches the configured full slowdown band. 0 preserves legacy speed target.')
    parser.add_argument('--random-clear-ahead-speed-cte-start', type=float, default=0.0, help='Absolute CTE where clear-ahead speed slowdown begins.')
    parser.add_argument('--random-clear-ahead-speed-cte-full', type=float, default=0.0, help='Absolute CTE where clear-ahead speed slowdown reaches full strength. 0 uses max-abs-cte when enabled.')
    parser.add_argument('--random-clear-ahead-max-omega', type=float, default=0.18, help='Absolute yaw-rate cap for random-clear-ahead target omega.')
    parser.add_argument('--random-clear-ahead-omega-reference', type=float, default=0.55, help='Heading error magnitude mapped to max omega when target-source=constant.')
    parser.add_argument('--random-clear-ahead-omega-weight', type=float, default=1.0, help='Relative yaw-rate loss weight for random-clear-ahead.')
    parser.add_argument('--random-late-lagging-weight', type=float, default=0.0, help='Trainer-side auxiliary that gently pushes low-progress random-encounter agents after teammates are clearly ahead.')
    parser.add_argument('--random-late-lagging-weight-end', type=float, default=None, help='Final random-late-lagging auxiliary weight for linear annealing. If unset, the start weight stays constant.')
    parser.add_argument('--random-late-lagging-goal-tolerance', type=float, default=1.0, help='Goal tolerance used by random-late-lagging.')
    parser.add_argument('--random-late-lagging-max-distance', type=float, default=80.0, help='Maximum distance-to-goal where random-late-lagging may activate.')
    parser.add_argument('--random-late-lagging-min-distance', type=float, default=2.0, help='Minimum distance-to-goal where random-late-lagging may activate.')
    parser.add_argument('--random-late-lagging-max-self-progress', type=float, default=0.25, help='Maximum own route_progress for a sample to count as late-lagging.')
    parser.add_argument('--random-late-lagging-min-team-progress', type=float, default=0.75, help='Minimum best team route_progress before random-late-lagging activates.')
    parser.add_argument('--random-late-lagging-min-progress-gap', type=float, default=0.50, help='Minimum gap between best team route_progress and own route_progress.')
    parser.add_argument('--random-late-lagging-agent-index', action='append', dest='random_late_lagging_agent_indices', type=int, default=None, help='Optional 0-based agent index allowed to receive random-late-lagging. Repeatable. Defaults to all agents.')
    parser.add_argument('--random-late-lagging-min-finished-teammates', type=int, default=0, help='If >0, require at least this many teammates to have route_progress above random-late-lagging-finished-progress.')
    parser.add_argument('--random-late-lagging-finished-progress', type=float, default=0.98, help='Route progress threshold used by random-late-lagging-min-finished-teammates.')
    parser.add_argument('--random-late-lagging-phase-min', type=float, default=-1.0, help='Minimum conflict_phase for random-late-lagging activation.')
    parser.add_argument('--random-late-lagging-min-team-separation', type=float, default=4.8, help='Minimum fleet separation required for random-late-lagging.')
    parser.add_argument('--random-late-lagging-min-neighbor-separation', type=float, default=4.8, help='Minimum nearest-neighbor separation required for random-late-lagging.')
    parser.add_argument('--random-late-lagging-max-cpa-score', type=float, default=0.0, help='Maximum CPA threat score allowed for random-late-lagging.')
    parser.add_argument('--random-late-lagging-max-local-score', type=float, default=0.0, help='Maximum local threat score allowed for random-late-lagging.')
    parser.add_argument('--random-late-lagging-allow-threat-overlap', action='store_true', help='Allow random-late-lagging to overlap active CPA/local/deconflict threats.')
    parser.add_argument('--random-late-lagging-target-speed', type=float, default=0.16, help='Target forward speed for random-late-lagging.')
    parser.add_argument('--random-late-lagging-min-speed', type=float, default=0.06, help='Minimum target forward speed for random-late-lagging.')
    parser.add_argument('--random-late-lagging-max-omega', type=float, default=0.34, help='Maximum yaw-rate target magnitude for random-late-lagging.')
    parser.add_argument('--random-late-lagging-omega-reference', type=float, default=0.80, help='Heading error magnitude mapped to max omega for random-late-lagging.')
    parser.add_argument('--random-late-lagging-omega-weight', type=float, default=2.60, help='Relative yaw-rate loss weight for random-late-lagging.')
    parser.add_argument('--random-late-lagging-target-source', choices=['goal', 'raw'], default='goal', help='Target source for random-late-lagging: goal-heading correction or raw navigation command.')
    parser.add_argument('--random-close-formation-escape-weight', type=float, default=0.0, help='Trainer-side auxiliary that nudges decisive forward + lateral motion when stuck in close-formation low-speed equilibria.')
    parser.add_argument('--random-close-formation-escape-weight-end', type=float, default=None, help='Final random-close-formation-escape auxiliary weight for linear annealing.')
    parser.add_argument('--random-close-formation-escape-scenario', action='append', dest='random_close_formation_escape_scenarios', default=None, help='Scenario name where random-close-formation-escape may activate. Repeatable. Defaults to random encounter scenarios.')
    parser.add_argument('--random-close-formation-escape-min-sep', type=float, default=1.40, help='Minimum nearest-neighbor separation for close-formation-escape activation (lower than pairwise shield gating).')
    parser.add_argument('--random-close-formation-escape-max-sep', type=float, default=2.40, help='Maximum nearest-neighbor separation for close-formation-escape activation.')
    parser.add_argument('--random-close-formation-escape-max-self-speed', type=float, default=0.22, help='Maximum own forward speed (raw_linear_x) for close-formation-escape activation.')
    parser.add_argument('--random-close-formation-escape-goal-tolerance', type=float, default=1.0, help='Goal tolerance used to ignore completed close-formation-escape samples.')
    parser.add_argument('--random-close-formation-escape-max-distance', type=float, default=20.0, help='Maximum distance-to-goal for close-formation-escape samples.')
    parser.add_argument('--random-close-formation-escape-target-speed', type=float, default=0.40, help='Target forward speed for close-formation-escape escape action.')
    parser.add_argument('--random-close-formation-escape-min-speed', type=float, default=0.20, help='Minimum target forward speed for close-formation-escape (lower bound for any sample).')
    parser.add_argument('--random-close-formation-escape-max-omega', type=float, default=0.45, help='Absolute yaw-rate cap for close-formation-escape escape omega target.')
    parser.add_argument('--random-close-formation-escape-omega-weight', type=float, default=1.6, help='Relative yaw-rate loss weight for close-formation-escape.')
    parser.add_argument('--random-close-formation-escape-agent-index', action='append', dest='random_close_formation_escape_agent_indices', type=int, default=None, help='Optional 0-based agent index allowed to receive close-formation-escape. Repeatable. Defaults to all agents.')
    parser.add_argument('--random-close-formation-escape-require-closing', action='store_true', help='If set, require nearest neighbor to be approaching (closing_speed > 0) for activation.')
    parser.add_argument('--policy-anchor-weight', type=float, default=0.0, help='Trainer-side auxiliary loss that keeps the current actor close to the loaded policy on non-target samples.')
    parser.add_argument('--policy-anchor-weight-end', type=float, default=None, help='Final policy-anchor auxiliary weight for linear annealing. If unset, policy-anchor-weight stays constant.')
    parser.add_argument('--policy-anchor-crossing-only', action='store_true', help='Apply policy-anchor loss only to three_usv_crossing samples.')
    parser.add_argument('--policy-anchor-exclude-lagging-finish', action='store_true', help='Exclude samples currently targeted by lagging-finish from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-team-safety-brake', action='store_true', help='Exclude samples currently targeted by team-safety-brake from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-random-deconflict', action='store_true', help='Exclude samples currently targeted by random-deconflict from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-random-safe-finish', action='store_true', help='Exclude samples currently targeted by random-safe-finish from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-random-goal-hold', action='store_true', help='Exclude samples currently targeted by random-goal-hold from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-random-offroute-finish', action='store_true', help='Exclude samples currently targeted by random-offroute-finish from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-random-cte-recovery', action='store_true', help='Exclude samples currently targeted by random-cte-recovery from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-random-clear-ahead', action='store_true', help='Exclude samples currently targeted by random-clear-ahead from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-random-late-lagging', action='store_true', help='Exclude samples currently targeted by random-late-lagging from policy-anchor loss.')
    parser.add_argument('--policy-anchor-exclude-random-pairwise-role-guard', action='store_true', help='Exclude samples currently targeted by random-pairwise-role-guard from policy-anchor loss.')
    parser.add_argument('--separate-actor-critic-grad-clip', action='store_true', help='Clip actor/log-std and critic gradients separately so large value losses do not suppress trainer-side actor auxiliaries.')
    return parser.parse_args()


def _build_agent_namespaces(num_agents: int) -> tuple[str, ...]:
    return tuple(f'usv_{index + 1:02d}' for index in range(num_agents))


def _goal_heading_omega_sign(args) -> float:
    try:
        sign_value = float(getattr(args, 'goal_heading_omega_sign', -1.0))
    except (TypeError, ValueError):
        sign_value = -1.0
    return 1.0 if sign_value >= 0.0 else -1.0


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


def _scenario_branch_indices_from_missing_keys(missing_keys, prefix: str) -> list[int]:
    indices: set[int] = set()
    for key in missing_keys:
        if not key.startswith(prefix):
            continue
        remainder = key[len(prefix):]
        index_text = remainder.split('.', 1)[0]
        if index_text.isdigit():
            indices.add(int(index_text))
    return sorted(indices)


def _copy_sequential_modules_from_base(target_modules, source_modules) -> None:
    if len(target_modules) != len(source_modules):
        raise RuntimeError(
            'Scenario branch/base MLP module count mismatch: '
            f'{len(target_modules)} != {len(source_modules)}'
        )
    for target_module, source_module in zip(target_modules, source_modules):
        target_module.load_state_dict(source_module.state_dict())


def _initialize_missing_scenario_branches_from_base(actor, missing_keys) -> list[str]:
    """Warm-start newly added scenario action heads/trunks from the loaded base MLP.

    Scenario heads/trunks are constructed before checkpoint loading. When a base checkpoint
    predates those modules, strict=False leaves them at random initialization unless we copy
    the now-loaded shared actor MLP into the missing per-scenario branches.
    """
    initialized: list[str] = []
    missing_keys = tuple(missing_keys or ())

    head_indices = _scenario_branch_indices_from_missing_keys(missing_keys, 'scenario_action_heads.')
    scenario_action_heads = getattr(actor, 'scenario_action_heads', None)
    if head_indices and scenario_action_heads is not None:
        tail_start = int(getattr(actor, '_scenario_head_tail_start', max(0, len(actor.mlp) - 3)))
        base_tail_modules = list(actor.mlp[tail_start:])
        for scenario_index in head_indices:
            if scenario_index < len(scenario_action_heads):
                _copy_sequential_modules_from_base(list(scenario_action_heads[scenario_index]), base_tail_modules)
        initialized.append(f'scenario_action_heads[{head_indices}]')

    trunk_indices = _scenario_branch_indices_from_missing_keys(missing_keys, 'scenario_actor_trunks.')
    scenario_actor_trunks = getattr(actor, 'scenario_actor_trunks', None)
    if trunk_indices and scenario_actor_trunks is not None:
        base_trunk_modules = list(actor.mlp)
        for scenario_index in trunk_indices:
            if scenario_index < len(scenario_actor_trunks):
                _copy_sequential_modules_from_base(list(scenario_actor_trunks[scenario_index]), base_trunk_modules)
        initialized.append(f'scenario_actor_trunks[{trunk_indices}]')

    return initialized


def _maybe_freeze_actor_base(actor, args):
    if not bool(getattr(args, 'freeze_actor_base', False)):
        return
    has_encounter_residual = bool(getattr(args, 'attention_encounter_residual', False))
    has_scenario_residual = bool(getattr(args, 'attention_scenario_residual', False))
    has_scenario_head = bool(getattr(args, 'attention_scenario_head', False))
    has_scenario_trunk = bool(getattr(args, 'attention_scenario_trunk', False))
    if not (has_encounter_residual or has_scenario_residual or has_scenario_head or has_scenario_trunk):
        raise ValueError('--freeze-actor-base requires --attention-encounter-residual, --attention-scenario-residual, --attention-scenario-head, or --attention-scenario-trunk.')
    if not (
        getattr(actor, 'encounter_residual_enabled', False)
        or getattr(actor, 'scenario_residual_enabled', False)
        or getattr(actor, 'scenario_head_enabled', False)
        or getattr(actor, 'scenario_trunk_enabled', False)
    ):
        raise ValueError('--freeze-actor-base is only supported for attention actors with scenario-specific branches or heads enabled.')

    trainable = []
    frozen_count = 0
    for name, parameter in actor.named_parameters():
        if (
            name.startswith('encounter_residual.')
            or name.startswith('scenario_residual_heads.')
            or name.startswith('scenario_action_heads.')
            or name.startswith('scenario_actor_trunks.')
        ):
            parameter.requires_grad = True
            trainable.append(name)
        else:
            parameter.requires_grad = False
            frozen_count += 1

    if not trainable:
        raise RuntimeError('freeze_actor_base left no trainable actor parameters.')

    print(
        'Freezing shared actor base and training only scenario-specific actor branches: '
        f'trainable={trainable}, frozen_count={frozen_count}',
        flush=True,
    )


def _actor_forward(actor, obs_tensor, scenario_ids=None):
    if scenario_ids is not None and (
        bool(getattr(actor, 'scenario_residual_enabled', False))
        or bool(getattr(actor, 'scenario_head_enabled', False))
        or bool(getattr(actor, 'scenario_trunk_enabled', False))
    ):
        return actor(obs_tensor, scenario_ids=scenario_ids)
    return actor(obs_tensor)


def _scenario_mix_label(flat_scenario_ids_np, scenarios: tuple[str, ...]) -> str:
    if flat_scenario_ids_np is None:
        return 'none'
    scenario_ids = np.asarray(flat_scenario_ids_np, dtype=np.int32).reshape(-1)
    if scenario_ids.size == 0:
        return 'empty'
    unique_ids, counts = np.unique(scenario_ids, return_counts=True)
    parts = []
    for scenario_id, count in zip(unique_ids.tolist(), counts.tolist()):
        if 0 <= int(scenario_id) < len(scenarios):
            name = str(scenarios[int(scenario_id)])
        else:
            name = f'id{int(scenario_id)}'
        parts.append(f'{name}:{int(count)}')
    return ','.join(parts)


def _clip_actor_critic_gradients(torch, actor, critic, actor_log_std, max_grad_norm: float, *, separate: bool):
    actor_params = [parameter for parameter in actor.parameters() if parameter.grad is not None]
    if actor_log_std.grad is not None:
        actor_params.append(actor_log_std)
    critic_params = [parameter for parameter in critic.parameters() if parameter.grad is not None]
    if separate:
        if actor_params:
            torch.nn.utils.clip_grad_norm_(actor_params, max_grad_norm)
        if critic_params:
            torch.nn.utils.clip_grad_norm_(critic_params, max_grad_norm)
    else:
        clip_params = actor_params + critic_params
        if clip_params:
            torch.nn.utils.clip_grad_norm_(clip_params, max_grad_norm)


def _crossing_imitation_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
):
    """Auxiliary trainer-side role schedule for the symmetric 3-USV crossing.

    This does not run during evaluation or deployment. It only biases the actor
    mean during PPO updates so the final single checkpoint can learn an
    asymmetric clear / yield timing pattern instead of all agents charging the
    shared conflict point.
    """
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    if raw_obs.numel() == 0 or raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return action_mean.new_zeros(())

    crossing_scenario_id = scenario_to_index.get('three_usv_crossing')
    if crossing_scenario_id is not None and scenario_ids is not None:
        scenario_mask = scenario_ids == int(crossing_scenario_id)
    else:
        # Last three entries are encounter one-hot: [head_on, crossing, overtaking].
        scenario_mask = raw_obs[:, -2] > 0.5

    phase = raw_obs[:, 13]
    eta = raw_obs[:, 14]
    priority = torch.clamp(raw_obs[:, 15], -1.0, 1.0)
    eta_gap = torch.clamp(raw_obs[:, 16], 0.0, 1.0)

    eta_gate = max(1e-3, float(getattr(args, 'crossing_imitation_eta_gate', 0.95)))
    phase_min = float(getattr(args, 'crossing_imitation_phase_min', -0.90))
    phase_max = float(getattr(args, 'crossing_imitation_phase_max', 0.22))
    active_mask = (
        scenario_mask
        & (phase >= phase_min)
        & (phase <= phase_max)
        & (eta <= eta_gate)
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    clear_role = torch.clamp((priority - 0.20) / 0.80, 0.0, 1.0)
    yield_role = torch.clamp((-priority - 0.20) / 0.80, 0.0, 1.0)
    middle_role = torch.clamp(1.0 - clear_role - yield_role, 0.0, 1.0)

    clear_speed = float(getattr(args, 'crossing_imitation_clear_speed', 0.32))
    middle_speed = float(getattr(args, 'crossing_imitation_middle_speed', 0.10))
    yield_speed = float(getattr(args, 'crossing_imitation_yield_speed', 0.0))
    target_linear = (
        clear_role * clear_speed
        + middle_role * middle_speed
        + yield_role * yield_speed
    )

    clear_omega = float(getattr(args, 'crossing_imitation_clear_omega', -0.10))
    middle_omega = float(getattr(args, 'crossing_imitation_middle_omega', -0.14))
    yield_omega = float(getattr(args, 'crossing_imitation_yield_omega', -0.18))
    target_omega = (
        clear_role * clear_omega
        + middle_role * middle_omega
        + yield_role * yield_omega
    )

    target = torch.stack([target_linear, target_omega], dim=-1).to(dtype=action_mean.dtype)
    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    target = torch.max(low, torch.min(high, target))

    approach_gate = torch.clamp(1.0 - (eta / eta_gate), 0.0, 1.0)
    gap_deficit = torch.clamp(1.0 - eta_gap, 0.0, 1.0)
    urgency = (0.30 + 0.70 * approach_gate) * (0.45 + 0.55 * gap_deficit)
    range_scale = torch.clamp(high - low, min=1e-3)
    per_sample_loss = (((action_mean - target) / range_scale) ** 2).mean(dim=-1) * urgency.to(dtype=action_mean.dtype)

    mask_f = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = mask_f * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    else:
        weights = mask_f
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _overtaking_imitation_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2 or scenario_ids is None or agent_indices is None:
        return action_mean.new_zeros(())
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return action_mean.new_zeros(())

    two_scenario_id = scenario_to_index.get('two_usv_overtaking')
    single_scenario_id = scenario_to_index.get('single_usv_overtaking')
    if two_scenario_id is None and single_scenario_id is None:
        return action_mean.new_zeros(())
    scenario_mask = torch.zeros_like(scenario_ids, dtype=torch.bool, device=scenario_ids.device)
    two_scenario_mask = torch.zeros_like(scenario_mask)
    single_scenario_mask = torch.zeros_like(scenario_mask)
    if two_scenario_id is not None:
        two_scenario_mask = scenario_ids == int(two_scenario_id)
        scenario_mask = scenario_mask | two_scenario_mask
    if single_scenario_id is not None:
        single_scenario_mask = scenario_ids == int(single_scenario_id)
        scenario_mask = scenario_mask | single_scenario_mask
    if not bool(scenario_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    max_distance = max(1.0, float(getattr(args, 'overtaking_imitation_max_distance', 13.0)))
    distance_to_goal = torch.clamp(raw_obs[:, 4], min=0.0)
    unfinished = (distance_to_goal > 0.8) & (distance_to_goal <= max_distance)
    nearest_distance, nearest_bearing, nearest_closing, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)

    neighbor_start = AgentLocalObservation.ego_feature_size()
    available = int(raw_obs.shape[-1]) - neighbor_start - ENCOUNTER_TYPE_COUNT
    neighbor_slots = max(0, available // NEIGHBOR_FEATURE_COUNT)
    if neighbor_slots <= 0:
        return action_mean.new_zeros(())
    neighbor_end = neighbor_start + neighbor_slots * NEIGHBOR_FEATURE_COUNT
    neighbors = raw_obs[:, neighbor_start:neighbor_end].reshape(sample_count, neighbor_slots, NEIGHBOR_FEATURE_COUNT)
    distances = torch.clamp(neighbors[:, :, 4], min=0.0)
    valid_slots = distances > 1e-6
    masked_distances = torch.where(valid_slots, distances, torch.full_like(distances, float('inf')))
    nearest_index = masked_distances.argmin(dim=1)
    row_index = torch.arange(sample_count, device=raw_obs.device)
    nearest = neighbors[row_index, nearest_index]
    rel_x = nearest[:, 0].to(dtype=action_mean.dtype)
    rel_y = nearest[:, 1].to(dtype=action_mean.dtype)

    agent_index_tensor = agent_indices.to(device=action_mean.device)
    front_role = agent_index_tensor == 0
    rear_role = agent_index_tensor == 1
    visible_ahead = nearest_distance.to(device=action_mean.device) <= max_distance
    same_lane_ahead = valid_neighbor.to(device=action_mean.device) & visible_ahead & (rel_x > 0.3) & (torch.abs(rel_y) <= 2.5)
    rear_active = (two_scenario_mask & rear_role & same_lane_ahead) | (single_scenario_mask & front_role)
    front_active = two_scenario_mask & front_role & valid_neighbor.to(device=action_mean.device) & (rel_x < 0.8)
    active_mask = unfinished & (front_active | rear_active)
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)

    raw_linear = raw_obs[:, 7].to(dtype=action_mean.dtype)
    raw_omega = raw_obs[:, 8].to(dtype=action_mean.dtype)
    cross_track_error = raw_obs[:, 11].to(dtype=action_mean.dtype) if raw_obs.shape[-1] > 11 else torch.zeros_like(raw_linear)
    route_progress = raw_obs[:, 12].to(dtype=action_mean.dtype) if raw_obs.shape[-1] > 12 else torch.zeros_like(raw_linear)
    front_min_speed = max(0.0, float(getattr(args, 'overtaking_imitation_front_min_speed', 0.24)))
    front_max_omega = max(0.0, float(getattr(args, 'overtaking_imitation_front_max_omega', 0.10)))
    rear_approach_speed = max(0.0, float(getattr(args, 'overtaking_imitation_rear_approach_speed', 0.12)))
    rear_close_speed = max(0.0, float(getattr(args, 'overtaking_imitation_rear_close_speed', 0.055)))
    rear_pass_speed = max(rear_approach_speed, float(getattr(args, 'overtaking_imitation_rear_pass_speed', 0.22)))
    close_sep = max(0.1, float(getattr(args, 'overtaking_imitation_close_separation', 2.30)))
    release_sep = max(close_sep + 0.1, float(getattr(args, 'overtaking_imitation_release_separation', 5.80)))
    target_offset = max(0.1, float(getattr(args, 'overtaking_imitation_target_starboard_offset', 1.05)))
    rear_omega_mag = max(0.0, float(getattr(args, 'overtaking_imitation_rear_omega', 0.26)))
    omega_weight = max(0.0, float(getattr(args, 'overtaking_imitation_omega_weight', 1.0)))
    single_return_progress = min(1.0, max(0.0, float(getattr(args, 'overtaking_imitation_single_return_progress', 0.70))))
    single_return_rel_x = float(getattr(args, 'overtaking_imitation_single_return_rel_x', -0.20))
    single_return_min_sep = max(0.1, float(getattr(args, 'overtaking_imitation_single_return_min_separation', 2.70)))
    single_return_cte_start = max(0.0, float(getattr(args, 'overtaking_imitation_single_return_cte_start', 0.70)))
    single_return_cte_full = max(single_return_cte_start + 1e-3, float(getattr(args, 'overtaking_imitation_single_return_cte_full', 3.00)))
    single_return_speed = max(0.0, float(getattr(args, 'overtaking_imitation_single_return_speed', 0.32)))
    single_return_omega_mag = max(0.0, float(getattr(args, 'overtaking_imitation_single_return_omega', 0.30)))
    single_finish_distance = max(0.8, float(getattr(args, 'overtaking_imitation_single_finish_distance', 3.50)))
    single_finish_speed = max(0.0, float(getattr(args, 'overtaking_imitation_single_finish_speed', 0.34)))
    single_finish_max_omega = max(0.0, float(getattr(args, 'overtaking_imitation_single_finish_max_omega', 0.10)))

    starboard_offset = torch.clamp(-rel_y, min=0.0)
    corridor = torch.clamp(starboard_offset / target_offset, 0.0, 1.0).to(dtype=action_mean.dtype)
    close = nearest_distance.to(device=action_mean.device) < close_sep
    rear_linear = rear_approach_speed + (rear_pass_speed - rear_approach_speed) * corridor
    rear_linear = torch.where(close & (corridor < 0.8), torch.minimum(rear_linear, torch.full_like(rear_linear, rear_close_speed)), rear_linear)
    rear_omega = -rear_omega_mag * torch.clamp(1.0 - corridor, 0.0, 1.0)

    abs_cte = torch.abs(cross_track_error)
    valid_neighbor_on_device = valid_neighbor.to(device=action_mean.device)
    single_safe_to_return = (~valid_neighbor_on_device) | (nearest_distance.to(device=action_mean.device) >= single_return_min_sep)
    single_past_lead = valid_neighbor_on_device & (rel_x <= single_return_rel_x)
    single_late_route = route_progress >= single_return_progress
    single_near_finish = distance_to_goal.to(device=action_mean.device) <= single_finish_distance
    single_return_active = single_scenario_mask & front_role & single_safe_to_return & (single_past_lead | single_late_route | single_near_finish)
    return_strength = torch.clamp((abs_cte - single_return_cte_start) / (single_return_cte_full - single_return_cte_start), 0.0, 1.0)
    return_omega = -torch.sign(cross_track_error) * single_return_omega_mag * return_strength
    finish_straight = single_near_finish & (abs_cte <= single_return_cte_start)
    return_linear = torch.where(single_near_finish, torch.full_like(raw_linear, single_finish_speed), torch.full_like(raw_linear, single_return_speed))
    return_omega = torch.where(finish_straight, torch.clamp(raw_omega, min=-single_finish_max_omega, max=single_finish_max_omega), return_omega)

    front_linear = torch.clamp(raw_linear, min=front_min_speed, max=max(front_min_speed, float(high[0].detach().cpu().item())))
    front_omega = torch.clamp(raw_omega, min=-front_max_omega, max=front_max_omega)
    target_linear = torch.where(single_return_active, return_linear, torch.where(rear_active, rear_linear, front_linear))
    target_omega = torch.where(single_return_active, return_omega, torch.where(rear_active, rear_omega, front_omega))
    target_linear = torch.clamp(target_linear, min=low[0], max=high[0])
    target_omega = torch.clamp(target_omega, min=low[1], max=high[1])

    danger = torch.clamp((release_sep - nearest_distance.to(device=action_mean.device)) / max(release_sep - close_sep, 1e-3), 0.0, 1.0).to(dtype=action_mean.dtype)
    urgency = torch.where(single_return_active, torch.full_like(danger, 0.85), torch.where(rear_active, 0.45 + 0.55 * danger, torch.full_like(danger, 0.65)))
    linear_loss = ((action_mean[:, 0] - target_linear) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target_omega) / range_scale[1]) ** 2
    per_sample_loss = (linear_loss + omega_weight * omega_loss) * urgency

    weights = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _overtaking_imitation_active_mask(torch, raw_obs, scenario_ids, scenario_to_index: dict[str, int], args, agent_indices=None):
    if scenario_ids is None or agent_indices is None or raw_obs.ndim == 0:
        return torch.zeros(0, dtype=torch.bool, device=raw_obs.device)
    two_scenario_id = scenario_to_index.get('two_usv_overtaking')
    single_scenario_id = scenario_to_index.get('single_usv_overtaking')
    sample_count = int(raw_obs.shape[0])
    if (two_scenario_id is None and single_scenario_id is None) or sample_count <= 0:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    scenario_mask = torch.zeros_like(scenario_ids, dtype=torch.bool, device=scenario_ids.device)
    if two_scenario_id is not None:
        scenario_mask = scenario_mask | (scenario_ids == int(two_scenario_id))
    if single_scenario_id is not None:
        scenario_mask = scenario_mask | (scenario_ids == int(single_scenario_id))
    distance_to_goal = torch.clamp(raw_obs[:, 4], min=0.0)
    nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    max_distance = max(
        float(getattr(args, 'overtaking_imitation_release_separation', 5.80)),
        float(getattr(args, 'overtaking_imitation_max_distance', 13.0)),
    )
    eligible_agent = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)
    if agent_indices is not None:
        agent_index_tensor = agent_indices.to(device=raw_obs.device)
        eligible_agent = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
        if two_scenario_id is not None:
            two_scenario_mask = scenario_ids.to(device=raw_obs.device) == int(two_scenario_id)
            eligible_agent = eligible_agent | (two_scenario_mask & ((agent_index_tensor == 0) | (agent_index_tensor == 1)))
        if single_scenario_id is not None:
            single_scenario_mask = scenario_ids.to(device=raw_obs.device) == int(single_scenario_id)
            eligible_agent = eligible_agent | (single_scenario_mask & (agent_index_tensor == 0))
    two_active = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if two_scenario_id is not None:
        two_scenario_mask = scenario_ids.to(device=raw_obs.device) == int(two_scenario_id)
        two_active = two_scenario_mask & valid_neighbor & (nearest_distance <= max_distance)
    single_active = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if single_scenario_id is not None:
        single_active = scenario_ids.to(device=raw_obs.device) == int(single_scenario_id)
    return eligible_agent & (distance_to_goal > 0.8) & (two_active | single_active)


def _near_goal_finish_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    if raw_obs.numel() == 0 or raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return action_mean.new_zeros(())

    if bool(getattr(args, 'near_goal_finish_crossing_only', False)):
        crossing_scenario_id = scenario_to_index.get('three_usv_crossing')
        if crossing_scenario_id is not None and scenario_ids is not None:
            scenario_mask = scenario_ids == int(crossing_scenario_id)
        else:
            scenario_mask = raw_obs[:, -2] > 0.5
    else:
        scenario_mask = torch.ones(raw_obs.shape[0], dtype=torch.bool, device=raw_obs.device)

    if bool(getattr(args, 'near_goal_finish_exclude_overtaking', False)) and scenario_ids is not None:
        for _ot_name in ('single_usv_overtaking', 'two_usv_overtaking'):
            _ot_id = scenario_to_index.get(_ot_name)
            if _ot_id is not None:
                scenario_mask = scenario_mask & (scenario_ids != int(_ot_id))

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    heading_error = torch.atan2(raw_obs[:, 5], raw_obs[:, 6])
    phase = raw_obs[:, 13]

    finish_distance = max(0.1, float(getattr(args, 'near_goal_finish_distance', 2.2)))
    goal_tolerance = max(0.05, float(getattr(args, 'near_goal_finish_goal_tolerance', 0.8)))
    phase_min = float(getattr(args, 'near_goal_finish_phase_min', 0.0))
    upper_distance = max(goal_tolerance + 0.05, finish_distance)
    active_mask = (
        scenario_mask
        & (distance <= upper_distance)
        & (phase >= phase_min)
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    target_speed = max(0.0, float(getattr(args, 'near_goal_finish_target_speed', 0.18)))
    speed_scale = torch.clamp(
        (distance - (0.50 * goal_tolerance)) / max(upper_distance - (0.50 * goal_tolerance), 1e-3),
        0.25,
        1.0,
    )
    hold_mask = distance <= goal_tolerance
    target_linear = torch.where(
        hold_mask,
        torch.zeros_like(distance),
        target_speed * speed_scale,
    )

    max_omega = max(0.0, float(getattr(args, 'near_goal_finish_max_omega', 0.18)))
    target_omega = torch.where(
        hold_mask,
        torch.zeros_like(heading_error),
        -torch.clamp(heading_error / 0.70, -1.0, 1.0) * max_omega,
    )
    target = torch.stack([target_linear, target_omega], dim=-1).to(dtype=action_mean.dtype)

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    target = torch.max(low, torch.min(high, target))
    range_scale = torch.clamp(high - low, min=1e-3)

    omega_weight = max(0.0, float(getattr(args, 'near_goal_finish_omega_weight', 0.35)))
    linear_loss = ((action_mean[:, 0] - target[:, 0]) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target[:, 1]) / range_scale[1]) ** 2
    closeness = torch.clamp((upper_distance - distance) / max(upper_distance, 1e-3), 0.0, 1.0)
    hold_boost = torch.where(hold_mask, torch.full_like(closeness, 0.65), torch.zeros_like(closeness))
    urgency = (0.45 + 0.55 * closeness + hold_boost).to(dtype=action_mean.dtype)
    per_sample_loss = (linear_loss + omega_weight * omega_loss) * urgency

    mask_f = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = mask_f * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    else:
        weights = mask_f
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _lagging_finish_active_mask(
    torch,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0:
        return torch.zeros(0, dtype=torch.bool, device=raw_obs.device)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if global_state.numel() == 0 or global_state.shape[0] != sample_count or global_state.shape[-1] < 5:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    if bool(getattr(args, 'lagging_finish_crossing_only', False)):
        crossing_scenario_id = scenario_to_index.get('three_usv_crossing')
        if crossing_scenario_id is not None and scenario_ids is not None:
            scenario_mask = scenario_ids == int(crossing_scenario_id)
        else:
            scenario_mask = raw_obs[:, -2] > 0.5
    else:
        scenario_mask = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    phase = raw_obs[:, 13]
    team_min_separation = global_state[:, -5]
    team_completion = torch.clamp(global_state[:, -1], 0.0, 1.0)
    completion_for_gate = team_completion

    finish_distance = max(0.1, float(getattr(args, 'lagging_finish_distance', 4.0)))
    goal_tolerance = max(0.05, float(getattr(args, 'lagging_finish_goal_tolerance', 0.8)))
    phase_min = float(getattr(args, 'lagging_finish_phase_min', 0.0))
    min_team_completion = max(0.0, float(getattr(args, 'lagging_finish_min_team_completion', 0.60)))
    max_team_completion = min(1.0, float(getattr(args, 'lagging_finish_max_team_completion', 0.999)))
    near_team_tolerance = max(0.0, float(getattr(args, 'lagging_finish_near_team_tolerance', 0.0)))
    min_team_separation = max(0.0, float(getattr(args, 'lagging_finish_min_team_separation', 0.0)))
    safe_team_separation = max(0.0, float(getattr(args, 'lagging_finish_safe_team_separation', 0.0)))
    safe_team_separation_power = max(0.1, float(getattr(args, 'lagging_finish_safe_team_separation_power', 1.0)))
    upper_distance = max(goal_tolerance + 0.05, finish_distance)

    if near_team_tolerance > 0.0:
        local_size = int(raw_obs.shape[-1])
        packed_size = int(global_state.shape[-1]) - 5
        if local_size > 0 and packed_size >= local_size and packed_size % local_size == 0:
            team_blocks = global_state[:, :packed_size].reshape(global_state.shape[0], packed_size // local_size, local_size)
            valid_team_mask = team_blocks.abs().sum(dim=-1) > 1e-6
            team_distances = torch.clamp(team_blocks[:, :, 4], min=0.0)
            near_team_mask = valid_team_mask & (team_distances <= near_team_tolerance)
            valid_count = torch.clamp(valid_team_mask.sum(dim=1).to(dtype=team_completion.dtype), min=1.0)
            near_completion = near_team_mask.sum(dim=1).to(dtype=team_completion.dtype) / valid_count
            completion_for_gate = torch.maximum(team_completion, torch.clamp(near_completion, 0.0, 1.0))

    team_gate = (completion_for_gate >= min_team_completion) & (team_completion < max_team_completion)
    if min_team_separation > 0.0:
        team_gate = team_gate & (team_min_separation >= min_team_separation)

    hold_mask = distance <= goal_tolerance
    lagging_mask = (distance > goal_tolerance) & (distance <= upper_distance)
    include_hold = bool(getattr(args, 'lagging_finish_hold_reached', False))
    role_mask = lagging_mask | (hold_mask if include_hold else torch.zeros_like(hold_mask))
    return scenario_mask & team_gate & (phase >= phase_min) & role_mask


def _lagging_teammate_finish_loss(
    torch,
    action_mean,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    if raw_obs.numel() == 0 or raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return action_mean.new_zeros(())
    if global_state.numel() == 0 or global_state.shape[0] != raw_obs.shape[0] or global_state.shape[-1] < 5:
        return action_mean.new_zeros(())

    if bool(getattr(args, 'lagging_finish_crossing_only', False)):
        crossing_scenario_id = scenario_to_index.get('three_usv_crossing')
        if crossing_scenario_id is not None and scenario_ids is not None:
            scenario_mask = scenario_ids == int(crossing_scenario_id)
        else:
            scenario_mask = raw_obs[:, -2] > 0.5
    else:
        scenario_mask = torch.ones(raw_obs.shape[0], dtype=torch.bool, device=raw_obs.device)

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    heading_error = torch.atan2(raw_obs[:, 5], raw_obs[:, 6])
    phase = raw_obs[:, 13]

    team_min_separation = global_state[:, -5]
    team_completion = torch.clamp(global_state[:, -1], 0.0, 1.0)
    completion_for_gate = team_completion

    finish_distance = max(0.1, float(getattr(args, 'lagging_finish_distance', 4.0)))
    goal_tolerance = max(0.05, float(getattr(args, 'lagging_finish_goal_tolerance', 0.8)))
    phase_min = float(getattr(args, 'lagging_finish_phase_min', 0.0))
    min_team_completion = max(0.0, float(getattr(args, 'lagging_finish_min_team_completion', 0.60)))
    max_team_completion = min(1.0, float(getattr(args, 'lagging_finish_max_team_completion', 0.999)))
    near_team_tolerance = max(0.0, float(getattr(args, 'lagging_finish_near_team_tolerance', 0.0)))
    min_team_separation = max(0.0, float(getattr(args, 'lagging_finish_min_team_separation', 0.0)))
    safe_team_separation = max(0.0, float(getattr(args, 'lagging_finish_safe_team_separation', 0.0)))
    safe_team_separation_power = max(0.1, float(getattr(args, 'lagging_finish_safe_team_separation_power', 1.0)))
    upper_distance = max(goal_tolerance + 0.05, finish_distance)

    if near_team_tolerance > 0.0:
        local_size = int(raw_obs.shape[-1])
        packed_size = int(global_state.shape[-1]) - 5
        if local_size > 0 and packed_size >= local_size and packed_size % local_size == 0:
            team_blocks = global_state[:, :packed_size].reshape(global_state.shape[0], packed_size // local_size, local_size)
            valid_team_mask = team_blocks.abs().sum(dim=-1) > 1e-6
            team_distances = torch.clamp(team_blocks[:, :, 4], min=0.0)
            near_team_mask = valid_team_mask & (team_distances <= near_team_tolerance)
            valid_count = torch.clamp(valid_team_mask.sum(dim=1).to(dtype=team_completion.dtype), min=1.0)
            near_completion = near_team_mask.sum(dim=1).to(dtype=team_completion.dtype) / valid_count
            completion_for_gate = torch.maximum(team_completion, torch.clamp(near_completion, 0.0, 1.0))

    team_gate = (completion_for_gate >= min_team_completion) & (team_completion < max_team_completion)
    if min_team_separation > 0.0:
        team_gate = team_gate & (team_min_separation >= min_team_separation)

    hold_mask = distance <= goal_tolerance
    lagging_mask = (distance > goal_tolerance) & (distance <= upper_distance)
    include_hold = bool(getattr(args, 'lagging_finish_hold_reached', False))
    role_mask = lagging_mask | (hold_mask if include_hold else torch.zeros_like(hold_mask))
    active_mask = scenario_mask & team_gate & (phase >= phase_min) & role_mask
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    target_speed = max(0.0, float(getattr(args, 'lagging_finish_target_speed', 0.14)))
    min_speed_scale = min(1.0, max(0.0, float(getattr(args, 'lagging_finish_min_speed_scale', 0.25))))
    speed_scale = torch.clamp(
        (distance - (0.50 * goal_tolerance)) / max(upper_distance - (0.50 * goal_tolerance), 1e-3),
        min_speed_scale,
        1.0,
    )

    max_omega = max(0.0, float(getattr(args, 'lagging_finish_max_omega', 0.10)))
    if bool(getattr(args, 'lagging_finish_raw_target', False)) and raw_obs.shape[-1] > 8:
        raw_linear = torch.clamp(raw_obs[:, 7], min=0.0, max=target_speed)
        min_linear = target_speed * min_speed_scale
        target_linear = torch.where(
            hold_mask,
            torch.zeros_like(distance),
            torch.maximum(raw_linear, torch.full_like(raw_linear, min_linear)),
        )
        raw_omega = torch.clamp(raw_obs[:, 8], -max_omega, max_omega)
        target_omega = torch.where(hold_mask, torch.zeros_like(raw_omega), raw_omega)
    else:
        target_linear = torch.where(
            hold_mask,
            torch.zeros_like(distance),
            target_speed * speed_scale,
        )
        target_omega = torch.where(
            hold_mask,
            torch.zeros_like(heading_error),
            -torch.clamp(heading_error / 0.70, -1.0, 1.0) * max_omega,
        )
    if safe_team_separation > min_team_separation and safe_team_separation > 0.0:
        separation_scale = torch.clamp(
            (team_min_separation - min_team_separation) / max(safe_team_separation - min_team_separation, 1e-3),
            0.0,
            1.0,
        )
        if safe_team_separation_power != 1.0:
            separation_scale = separation_scale.pow(safe_team_separation_power)
        target_linear = torch.where(hold_mask, target_linear, target_linear * separation_scale)
    target = torch.stack([target_linear, target_omega], dim=-1).to(dtype=action_mean.dtype)

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    target = torch.max(low, torch.min(high, target))
    range_scale = torch.clamp(high - low, min=1e-3)

    omega_weight = max(0.0, float(getattr(args, 'lagging_finish_omega_weight', 0.25)))
    hold_omega_only = bool(getattr(args, 'lagging_finish_hold_omega_only', False))
    hold_weight = max(0.0, float(getattr(args, 'lagging_finish_hold_weight', 1.0)))
    linear_loss = ((action_mean[:, 0] - target[:, 0]) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target[:, 1]) / range_scale[1]) ** 2
    lagging_closeness = torch.clamp((upper_distance - distance) / max(upper_distance - goal_tolerance, 1e-3), 0.0, 1.0)
    team_urgency = torch.clamp((completion_for_gate - min_team_completion) / max(max_team_completion - min_team_completion, 1e-3), 0.0, 1.0)
    if safe_team_separation > min_team_separation and safe_team_separation > 0.0:
        team_urgency = torch.maximum(
            team_urgency,
            torch.clamp((safe_team_separation - team_min_separation) / max(safe_team_separation - min_team_separation, 1e-3), 0.0, 1.0),
        )
    hold_boost = torch.where(hold_mask, torch.full_like(lagging_closeness, 0.50), torch.zeros_like(lagging_closeness))
    urgency = (0.35 + 0.40 * lagging_closeness + 0.25 * team_urgency + hold_boost).to(dtype=action_mean.dtype)
    if hold_omega_only:
        omega_weight_tensor = torch.where(
            hold_mask,
            torch.full_like(omega_loss, omega_weight),
            torch.zeros_like(omega_loss),
        )
        per_sample_loss = (linear_loss + omega_weight_tensor * omega_loss) * urgency
    else:
        per_sample_loss = (linear_loss + omega_weight * omega_loss) * urgency
    if hold_weight != 1.0:
        hold_scale = torch.where(
            hold_mask,
            torch.full_like(per_sample_loss, hold_weight),
            torch.ones_like(per_sample_loss),
        )
        per_sample_loss = per_sample_loss * hold_scale

    mask_f = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = mask_f * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    else:
        weights = mask_f
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _nearest_neighbor_features(torch, raw_obs):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0:
        empty = torch.zeros(0, dtype=raw_obs.dtype, device=raw_obs.device)
        return empty, empty, empty, empty

    neighbor_start = AgentLocalObservation.ego_feature_size()
    available = int(raw_obs.shape[-1]) - neighbor_start - ENCOUNTER_TYPE_COUNT
    neighbor_slots = max(0, available // NEIGHBOR_FEATURE_COUNT)
    if neighbor_slots <= 0:
        distance = torch.full((sample_count,), float('inf'), dtype=raw_obs.dtype, device=raw_obs.device)
        zeros = torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
        valid = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
        return distance, zeros, zeros, valid

    neighbor_end = neighbor_start + neighbor_slots * NEIGHBOR_FEATURE_COUNT
    neighbors = raw_obs[:, neighbor_start:neighbor_end].reshape(sample_count, neighbor_slots, NEIGHBOR_FEATURE_COUNT)
    distances = torch.clamp(neighbors[:, :, 4], min=0.0)
    valid_slots = distances > 1e-6
    masked_distances = torch.where(valid_slots, distances, torch.full_like(distances, float('inf')))
    nearest_distance, nearest_index = masked_distances.min(dim=1)
    row_index = torch.arange(sample_count, device=raw_obs.device)
    nearest = neighbors[row_index, nearest_index]
    valid = torch.isfinite(nearest_distance)

    rel_x = nearest[:, 0]
    rel_y = nearest[:, 1]
    rel_vx = nearest[:, 2]
    rel_vy = nearest[:, 3]
    bearing = torch.where(valid, nearest[:, 5], torch.zeros_like(nearest[:, 5]))
    closing_speed = -((rel_x * rel_vx) + (rel_y * rel_vy)) / torch.clamp(nearest_distance, min=1e-3)
    closing_speed = torch.where(valid, closing_speed, torch.zeros_like(closing_speed))
    return nearest_distance, bearing, closing_speed, valid


def _neighbor_cpa_threat_features(
    torch,
    raw_obs,
    lookahead_distance: float,
    tcpa_horizon: float,
    dcpa_target: float,
    closing_speed_min: float,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0:
        empty = torch.zeros(0, dtype=raw_obs.dtype, device=raw_obs.device)
        valid_empty = torch.zeros(0, dtype=torch.bool, device=raw_obs.device)
        return empty, empty, empty, empty, empty, valid_empty

    zeros = torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    invalid_distance = torch.full((sample_count,), float('inf'), dtype=raw_obs.dtype, device=raw_obs.device)
    if lookahead_distance <= 0.0 or tcpa_horizon <= 0.0 or dcpa_target <= 0.0:
        return invalid_distance, zeros, zeros, zeros, zeros, torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    neighbor_start = AgentLocalObservation.ego_feature_size()
    available = int(raw_obs.shape[-1]) - neighbor_start - ENCOUNTER_TYPE_COUNT
    neighbor_slots = max(0, available // NEIGHBOR_FEATURE_COUNT)
    if neighbor_slots <= 0:
        return invalid_distance, zeros, zeros, zeros, zeros, torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    neighbor_end = neighbor_start + neighbor_slots * NEIGHBOR_FEATURE_COUNT
    neighbors = raw_obs[:, neighbor_start:neighbor_end].reshape(sample_count, neighbor_slots, NEIGHBOR_FEATURE_COUNT)
    distances = torch.clamp(neighbors[:, :, 4], min=0.0)
    valid_slots = distances > 1e-6

    rel_x = neighbors[:, :, 0]
    rel_y = neighbors[:, :, 1]
    rel_vx = neighbors[:, :, 2]
    rel_vy = neighbors[:, :, 3]
    dot = (rel_x * rel_vx) + (rel_y * rel_vy)
    rel_speed_sq = (rel_vx * rel_vx) + (rel_vy * rel_vy)
    closing_speed = -dot / torch.clamp(distances, min=1e-3)
    tcpa = torch.where(rel_speed_sq > 1e-6, -dot / torch.clamp(rel_speed_sq, min=1e-6), torch.zeros_like(dot))
    positive_tcpa = tcpa > 0.0
    cpa_x = rel_x + rel_vx * torch.where(positive_tcpa, tcpa, torch.zeros_like(tcpa))
    cpa_y = rel_y + rel_vy * torch.where(positive_tcpa, tcpa, torch.zeros_like(tcpa))
    dcpa = torch.where(
        positive_tcpa,
        torch.sqrt(torch.clamp(cpa_x * cpa_x + cpa_y * cpa_y, min=0.0)),
        distances,
    )

    distance_gate = torch.clamp((lookahead_distance - distances) / max(lookahead_distance, 1e-3), 0.0, 1.0)
    time_gate = torch.clamp((tcpa_horizon - tcpa) / max(tcpa_horizon, 1e-3), 0.0, 1.0)
    dcpa_deficit = torch.clamp((dcpa_target - dcpa) / max(dcpa_target, 1e-3), 0.0, 1.0)
    closing_gate = torch.clamp((closing_speed - closing_speed_min) / max(0.35 - closing_speed_min, 1e-3), 0.0, 1.0)
    threat_mask = (
        valid_slots
        & (distances <= lookahead_distance)
        & (closing_speed >= closing_speed_min)
        & positive_tcpa
        & (tcpa <= tcpa_horizon)
        & (dcpa < dcpa_target)
    )
    threat_score = torch.where(
        threat_mask,
        torch.maximum(distance_gate, time_gate) * dcpa_deficit * (0.25 + 0.75 * closing_gate),
        torch.zeros_like(distances),
    )
    best_score, best_index = threat_score.max(dim=1)
    row_index = torch.arange(sample_count, device=raw_obs.device)
    best_distance = distances[row_index, best_index]
    best_bearing = neighbors[row_index, best_index, 5]
    best_closing = closing_speed[row_index, best_index]
    best_dcpa = dcpa[row_index, best_index]
    valid_threat = best_score > 0.0
    best_distance = torch.where(valid_threat, best_distance, invalid_distance)
    best_bearing = torch.where(valid_threat, best_bearing, zeros)
    best_closing = torch.where(valid_threat, best_closing, zeros)
    best_dcpa = torch.where(valid_threat, best_dcpa, zeros)
    return best_distance, best_bearing, best_closing, best_dcpa, best_score, valid_threat


def _random_deconflict_pairwise_priority_delta(torch, raw_obs, args):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    zeros = torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    valid_empty = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if sample_count <= 0:
        return zeros, valid_empty

    neighbor_start = AgentLocalObservation.ego_feature_size()
    available = int(raw_obs.shape[-1]) - neighbor_start - ENCOUNTER_TYPE_COUNT
    neighbor_slots = max(0, available // NEIGHBOR_FEATURE_COUNT)
    if neighbor_slots <= 0 or NEIGHBOR_FEATURE_COUNT <= 9:
        return zeros, valid_empty

    neighbor_end = neighbor_start + neighbor_slots * NEIGHBOR_FEATURE_COUNT
    neighbors = raw_obs[:, neighbor_start:neighbor_end].reshape(sample_count, neighbor_slots, NEIGHBOR_FEATURE_COUNT)
    distances = torch.clamp(neighbors[:, :, 4], min=0.0)
    valid_slots = distances > 1e-6
    priority_delta = neighbors[:, :, 9]

    masked_distances = torch.where(valid_slots, distances, torch.full_like(distances, float('inf')))
    nearest_distance, nearest_index = masked_distances.min(dim=1)
    row_index = torch.arange(sample_count, device=raw_obs.device)
    nearest_delta = priority_delta[row_index, nearest_index]
    valid_nearest = torch.isfinite(nearest_distance)

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    cpa_score = torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    cpa_delta = zeros
    valid_cpa = valid_empty
    if lookahead > 0.0 and horizon > 0.0 and dcpa_target > 0.0:
        rel_x = neighbors[:, :, 0]
        rel_y = neighbors[:, :, 1]
        rel_vx = neighbors[:, :, 2]
        rel_vy = neighbors[:, :, 3]
        dot = (rel_x * rel_vx) + (rel_y * rel_vy)
        rel_speed_sq = (rel_vx * rel_vx) + (rel_vy * rel_vy)
        closing_speed = -dot / torch.clamp(distances, min=1e-3)
        tcpa = torch.where(rel_speed_sq > 1e-6, -dot / torch.clamp(rel_speed_sq, min=1e-6), torch.zeros_like(dot))
        positive_tcpa = tcpa > 0.0
        cpa_x = rel_x + rel_vx * torch.where(positive_tcpa, tcpa, torch.zeros_like(tcpa))
        cpa_y = rel_y + rel_vy * torch.where(positive_tcpa, tcpa, torch.zeros_like(tcpa))
        dcpa = torch.where(
            positive_tcpa,
            torch.sqrt(torch.clamp(cpa_x * cpa_x + cpa_y * cpa_y, min=0.0)),
            distances,
        )
        distance_gate = torch.clamp((lookahead - distances) / max(lookahead, 1e-3), 0.0, 1.0)
        time_gate = torch.clamp((horizon - tcpa) / max(horizon, 1e-3), 0.0, 1.0)
        dcpa_deficit = torch.clamp((dcpa_target - dcpa) / max(dcpa_target, 1e-3), 0.0, 1.0)
        closing_gate = torch.clamp((closing_speed - closing_min) / max(0.35 - closing_min, 1e-3), 0.0, 1.0)
        threat_mask = (
            valid_slots
            & (distances <= lookahead)
            & (closing_speed >= closing_min)
            & positive_tcpa
            & (tcpa <= horizon)
            & (dcpa < dcpa_target)
        )
        threat_score = torch.where(
            threat_mask,
            torch.maximum(distance_gate, time_gate) * dcpa_deficit * (0.25 + 0.75 * closing_gate),
            torch.zeros_like(distances),
        )
        cpa_score, cpa_index = threat_score.max(dim=1)
        cpa_delta = priority_delta[row_index, cpa_index]
        valid_cpa = cpa_score > 0.0

    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    use_cpa = valid_cpa & ((~valid_local) | (cpa_score >= local_score))
    selected_delta = torch.where(use_cpa, cpa_delta, nearest_delta)
    valid = use_cpa | (valid_local & valid_nearest)
    return torch.where(valid, selected_delta, zeros), valid


def _random_deconflict_pairwise_route_eta_delta(torch, raw_obs, args):
    """Pairwise route ETA gap for the same neighbor selection as priority-delta (CPA vs local/nearest)."""
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    zeros = torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    valid_empty = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if sample_count <= 0:
        return zeros, valid_empty

    neighbor_start = AgentLocalObservation.ego_feature_size()
    available = int(raw_obs.shape[-1]) - neighbor_start - ENCOUNTER_TYPE_COUNT
    neighbor_slots = max(0, available // NEIGHBOR_FEATURE_COUNT)
    if neighbor_slots <= 0 or NEIGHBOR_FEATURE_COUNT < 9:
        return zeros, valid_empty

    neighbor_end = neighbor_start + neighbor_slots * NEIGHBOR_FEATURE_COUNT
    neighbors = raw_obs[:, neighbor_start:neighbor_end].reshape(sample_count, neighbor_slots, NEIGHBOR_FEATURE_COUNT)
    distances = torch.clamp(neighbors[:, :, 4], min=0.0)
    valid_slots = distances > 1e-6
    route_eta_delta = neighbors[:, :, 8]

    masked_distances = torch.where(valid_slots, distances, torch.full_like(distances, float('inf')))
    nearest_distance, nearest_index = masked_distances.min(dim=1)
    row_index = torch.arange(sample_count, device=raw_obs.device)
    nearest_eta = route_eta_delta[row_index, nearest_index]
    valid_nearest = torch.isfinite(nearest_distance)

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    cpa_score = torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    cpa_eta = zeros
    valid_cpa = valid_empty
    if lookahead > 0.0 and horizon > 0.0 and dcpa_target > 0.0:
        rel_x = neighbors[:, :, 0]
        rel_y = neighbors[:, :, 1]
        rel_vx = neighbors[:, :, 2]
        rel_vy = neighbors[:, :, 3]
        dot = (rel_x * rel_vx) + (rel_y * rel_vy)
        rel_speed_sq = (rel_vx * rel_vx) + (rel_vy * rel_vy)
        closing_speed = -dot / torch.clamp(distances, min=1e-3)
        tcpa = torch.where(rel_speed_sq > 1e-6, -dot / torch.clamp(rel_speed_sq, min=1e-6), torch.zeros_like(dot))
        positive_tcpa = tcpa > 0.0
        cpa_x = rel_x + rel_vx * torch.where(positive_tcpa, tcpa, torch.zeros_like(tcpa))
        cpa_y = rel_y + rel_vy * torch.where(positive_tcpa, tcpa, torch.zeros_like(tcpa))
        dcpa = torch.where(
            positive_tcpa,
            torch.sqrt(torch.clamp(cpa_x * cpa_x + cpa_y * cpa_y, min=0.0)),
            distances,
        )
        distance_gate = torch.clamp((lookahead - distances) / max(lookahead, 1e-3), 0.0, 1.0)
        time_gate = torch.clamp((horizon - tcpa) / max(horizon, 1e-3), 0.0, 1.0)
        dcpa_deficit = torch.clamp((dcpa_target - dcpa) / max(dcpa_target, 1e-3), 0.0, 1.0)
        closing_gate = torch.clamp((closing_speed - closing_min) / max(0.35 - closing_min, 1e-3), 0.0, 1.0)
        threat_mask = (
            valid_slots
            & (distances <= lookahead)
            & (closing_speed >= closing_min)
            & positive_tcpa
            & (tcpa <= horizon)
            & (dcpa < dcpa_target)
        )
        threat_score = torch.where(
            threat_mask,
            torch.maximum(distance_gate, time_gate) * dcpa_deficit * (0.25 + 0.75 * closing_gate),
            torch.zeros_like(distances),
        )
        cpa_score, cpa_index = threat_score.max(dim=1)
        cpa_eta = route_eta_delta[row_index, cpa_index]
        valid_cpa = cpa_score > 0.0

    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    use_cpa = valid_cpa & ((~valid_local) | (cpa_score >= local_score))
    selected_eta = torch.where(use_cpa, cpa_eta, nearest_eta)
    valid = use_cpa | (valid_local & valid_nearest)
    return torch.where(valid, selected_eta, zeros), valid


def _team_safety_brake_role_weights(torch, raw_obs, args, agent_indices=None):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    weights = torch.ones(sample_count, dtype=torch.float32, device=raw_obs.device)
    if sample_count <= 0:
        return weights
    if not (
        bool(getattr(args, 'team_safety_brake_random_only', False))
        and bool(getattr(args, 'team_safety_brake_random_yield_agents_only', False))
    ):
        return weights
    if agent_indices is None:
        return torch.zeros(sample_count, dtype=torch.float32, device=raw_obs.device)
    agent_index_tensor = agent_indices.to(device=raw_obs.device)
    yield_weight = max(0.0, float(getattr(args, 'team_safety_brake_random_yield_agent_weight', 1.0)))
    standon_weight = max(0.0, float(getattr(args, 'team_safety_brake_random_standon_agent_weight', 0.0)))
    return torch.where(
        agent_index_tensor >= 1,
        torch.full_like(weights, yield_weight),
        torch.full_like(weights, standon_weight),
    )


def _team_safety_brake_active_mask(
    torch,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0:
        return torch.zeros(0, dtype=torch.bool, device=raw_obs.device)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if global_state.numel() == 0 or global_state.shape[0] != sample_count or global_state.shape[-1] < 5:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    crossing_only = bool(getattr(args, 'team_safety_brake_crossing_only', False))
    random_only = bool(getattr(args, 'team_safety_brake_random_only', False))
    if crossing_only or random_only:
        scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
        if crossing_only:
            crossing_scenario_id = scenario_to_index.get('three_usv_crossing')
            if crossing_scenario_id is not None and scenario_ids is not None:
                scenario_mask = scenario_mask | (scenario_ids == int(crossing_scenario_id))
            elif not random_only:
                scenario_mask = raw_obs[:, -2] > 0.5
        if random_only and scenario_ids is not None:
            for scenario_name in ('two_usv_random_encounter', 'three_usv_random_encounter'):
                scenario_id = scenario_to_index.get(scenario_name)
                if scenario_id is not None:
                    scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))
    else:
        scenario_mask = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)

    role_weights = _team_safety_brake_role_weights(torch, raw_obs, args, agent_indices)
    if random_only and bool(getattr(args, 'team_safety_brake_random_yield_agents_only', False)):
        scenario_mask = scenario_mask & (role_weights > 0.0)

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    phase = raw_obs[:, 13]
    team_min_separation = global_state[:, -5]
    team_completion = torch.clamp(global_state[:, -1], 0.0, 1.0)
    completion_for_gate = team_completion

    goal_tolerance = max(0.05, float(getattr(args, 'team_safety_brake_goal_tolerance', 0.8)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'team_safety_brake_max_distance', 12.0)))
    phase_min = float(getattr(args, 'team_safety_brake_phase_min', -1.0))
    min_team_completion = max(0.0, float(getattr(args, 'team_safety_brake_min_team_completion', 0.30)))
    max_team_completion = min(1.0, float(getattr(args, 'team_safety_brake_max_team_completion', 0.999)))
    near_team_tolerance = max(0.0, float(getattr(args, 'team_safety_brake_near_team_tolerance', 0.0)))
    release_separation = max(0.0, float(getattr(args, 'team_safety_brake_release_separation', 2.4)))

    cpa_separation_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if bool(getattr(args, 'team_safety_brake_cpa_danger', False)):
        cpa_lookahead = max(release_separation, float(getattr(args, 'team_safety_brake_cpa_lookahead_distance', 0.0)))
        cpa_horizon = max(0.0, float(getattr(args, 'team_safety_brake_cpa_time_horizon', 0.0)))
        cpa_target = max(0.0, float(getattr(args, 'team_safety_brake_cpa_dcpa_target', 0.0)))
        cpa_closing_min = max(0.0, float(getattr(args, 'team_safety_brake_cpa_closing_speed_min', 0.02)))
        _, _, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
            torch,
            raw_obs,
            cpa_lookahead,
            cpa_horizon,
            cpa_target,
            cpa_closing_min,
        )
        cpa_separation_mask = valid_cpa & (cpa_score > 0.0)

    if near_team_tolerance > 0.0:
        local_size = int(raw_obs.shape[-1])
        packed_size = int(global_state.shape[-1]) - 5
        if local_size > 0 and packed_size >= local_size and packed_size % local_size == 0:
            team_blocks = global_state[:, :packed_size].reshape(global_state.shape[0], packed_size // local_size, local_size)
            valid_team_mask = team_blocks.abs().sum(dim=-1) > 1e-6
            team_distances = torch.clamp(team_blocks[:, :, 4], min=0.0)
            near_team_mask = valid_team_mask & (team_distances <= near_team_tolerance)
            valid_count = torch.clamp(valid_team_mask.sum(dim=1).to(dtype=team_completion.dtype), min=1.0)
            near_completion = near_team_mask.sum(dim=1).to(dtype=team_completion.dtype) / valid_count
            completion_for_gate = torch.maximum(team_completion, torch.clamp(near_completion, 0.0, 1.0))

    team_gate = (completion_for_gate >= min_team_completion) & (team_completion < max_team_completion)
    unfinished_mask = (distance > goal_tolerance) & (distance <= max_distance)
    separation_mask = team_min_separation < release_separation
    if bool(getattr(args, 'team_safety_brake_require_neighbor', False)):
        nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
        separation_mask = separation_mask & valid_neighbor & (nearest_distance < release_separation)
    if bool(cpa_separation_mask.any().detach().cpu()):
        separation_mask = separation_mask | cpa_separation_mask
    return scenario_mask & team_gate & separation_mask & (phase >= phase_min) & unfinished_mask


def _team_safety_brake_loss(
    torch,
    action_mean,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 1:
        return action_mean.new_zeros(())
    active_mask = _team_safety_brake_active_mask(
        torch,
        raw_obs,
        global_state,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    team_min_separation = global_state[:, -5]
    safe_separation = max(0.0, float(getattr(args, 'team_safety_brake_safe_separation', 1.2)))
    release_separation = max(safe_separation + 0.05, float(getattr(args, 'team_safety_brake_release_separation', 2.4)))
    brake_power = max(0.1, float(getattr(args, 'team_safety_brake_power', 1.0)))
    target_speed = max(0.0, float(getattr(args, 'team_safety_brake_target_speed', 0.0)))
    omega_weight = max(0.0, float(getattr(args, 'team_safety_brake_omega_weight', 0.0)))
    target_omega_limit = max(0.0, float(getattr(args, 'team_safety_brake_target_omega', 0.0)))
    turn_mode = str(getattr(args, 'team_safety_brake_turn_mode', 'away'))
    head_on_starboard_threshold = max(
        0.0,
        float(getattr(args, 'team_safety_brake_head_on_starboard_threshold', 0.0)),
    )
    nearest_distance = bearing = closing_speed = valid_neighbor = None
    cpa_distance = cpa_bearing = cpa_closing = cpa_score = valid_cpa = None
    if bool(getattr(args, 'team_safety_brake_cpa_danger', False)):
        cpa_lookahead = max(release_separation, float(getattr(args, 'team_safety_brake_cpa_lookahead_distance', 0.0)))
        cpa_horizon = max(0.0, float(getattr(args, 'team_safety_brake_cpa_time_horizon', 0.0)))
        cpa_target = max(0.0, float(getattr(args, 'team_safety_brake_cpa_dcpa_target', 0.0)))
        cpa_closing_min = max(0.0, float(getattr(args, 'team_safety_brake_cpa_closing_speed_min', 0.02)))
        cpa_distance, cpa_bearing, cpa_closing, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
            torch,
            raw_obs,
            cpa_lookahead,
            cpa_horizon,
            cpa_target,
            cpa_closing_min,
        )
    danger_distance = team_min_separation
    if bool(getattr(args, 'team_safety_brake_local_danger', False)):
        nearest_distance, bearing, closing_speed, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
        danger_distance = torch.where(valid_neighbor, nearest_distance, team_min_separation)

    danger = torch.clamp(
        (release_separation - danger_distance) / max(release_separation - safe_separation, 1e-3),
        0.0,
        1.0,
    ).to(dtype=action_mean.dtype)
    if cpa_score is not None:
        danger = torch.maximum(danger, cpa_score.to(dtype=action_mean.dtype))
    if brake_power != 1.0:
        danger = danger.pow(brake_power)
    target_linear = (target_speed * (1.0 - danger)).to(dtype=action_mean.dtype)

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    target_linear = torch.clamp(target_linear, min=low[0], max=high[0])
    range_scale = torch.clamp(high - low, min=1e-3)
    linear_range = range_scale[0]
    linear_loss = ((action_mean[:, 0] - target_linear) / linear_range) ** 2
    urgency = (0.25 + 0.75 * danger).to(dtype=linear_loss.dtype)
    per_sample_loss = linear_loss * urgency
    if action_mean.shape[-1] >= 2 and omega_weight > 0.0 and target_omega_limit > 0.0:
        if nearest_distance is None:
            nearest_distance, bearing, closing_speed, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
        neighbor_danger = torch.clamp(
            (release_separation - nearest_distance) / max(release_separation - safe_separation, 1e-3),
            0.0,
            1.0,
        ).to(dtype=action_mean.dtype)
        if cpa_score is not None:
            cpa_score_cast = cpa_score.to(dtype=action_mean.dtype)
            use_cpa = (valid_cpa & (cpa_score_cast > neighbor_danger)).to(device=action_mean.device)
            nearest_distance = torch.where(use_cpa, cpa_distance, nearest_distance)
            bearing = torch.where(use_cpa, cpa_bearing, bearing)
            closing_speed = torch.where(use_cpa, cpa_closing, closing_speed)
            valid_neighbor = valid_neighbor | valid_cpa
            neighbor_danger = torch.maximum(neighbor_danger, cpa_score_cast)
        if brake_power != 1.0:
            neighbor_danger = neighbor_danger.pow(brake_power)
        closing_gate = torch.clamp((closing_speed.to(dtype=action_mean.dtype) + 0.05) / 0.35, 0.0, 1.0)
        if turn_mode == 'starboard':
            turn_sign = torch.full_like(bearing, -1.0, dtype=action_mean.dtype)
        else:
            away_turn_sign = torch.where(
                bearing > 0.0,
                torch.full_like(bearing, -1.0),
                torch.where(bearing < 0.0, torch.ones_like(bearing), torch.zeros_like(bearing)),
            )
            if head_on_starboard_threshold > 0.0:
                starboard_turn_sign = torch.full_like(bearing, -1.0)
                away_turn_sign = torch.where(
                    torch.abs(bearing) <= head_on_starboard_threshold,
                    starboard_turn_sign,
                    away_turn_sign,
                )
            turn_sign = away_turn_sign.to(dtype=action_mean.dtype)
        turn_scale = torch.where(valid_neighbor, neighbor_danger * (0.35 + 0.65 * closing_gate), torch.zeros_like(neighbor_danger))
        target_omega = torch.clamp(turn_sign * target_omega_limit * turn_scale, min=low[1], max=high[1])
        omega_loss = ((action_mean[:, 1] - target_omega) / range_scale[1]) ** 2
        per_sample_loss = per_sample_loss + omega_weight * omega_loss * turn_scale

    mask_f = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = mask_f * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    else:
        weights = mask_f
    weights = weights * _team_safety_brake_role_weights(torch, raw_obs, args, agent_indices).to(
        dtype=per_sample_loss.dtype,
        device=per_sample_loss.device,
    )
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_deconflict_active_mask(
    torch,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or scenario_ids is None or agent_indices is None:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in ('two_usv_random_encounter', 'three_usv_random_encounter'):
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    goal_tolerance = max(0.05, float(getattr(args, 'random_deconflict_goal_tolerance', 0.8)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_deconflict_max_distance', 13.0)))
    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    unfinished_mask = (distance > goal_tolerance) & (distance <= max_distance)

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, cpa_bearing, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    threat_score = torch.maximum(cpa_score.to(dtype=raw_obs.dtype), local_score.to(dtype=raw_obs.dtype))
    valid_threat = valid_cpa | valid_local
    is_yield = _random_deconflict_yield_mask(torch, raw_obs, args, agent_indices)
    role_weight = torch.where(
        is_yield,
        torch.full((sample_count,), max(0.0, float(getattr(args, 'random_deconflict_yield_weight', 1.0))), device=raw_obs.device),
        torch.full((sample_count,), max(0.0, float(getattr(args, 'random_deconflict_standon_weight', 0.35))), device=raw_obs.device),
    )
    release_overlap = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    release_min_separation = max(0.0, float(getattr(args, 'random_safe_finish_yield_release_min_separation', 0.0)))
    if release_min_separation > 0.0:
        nearest_distance, _, nearest_closing_speed, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
        release_max_closing = float(getattr(args, 'random_safe_finish_yield_release_max_closing_speed', 0.02))
        release_min_route_progress = max(0.0, float(getattr(args, 'random_safe_finish_yield_release_min_route_progress', 0.0)))
        release_route_mask = raw_obs[:, 12] >= release_min_route_progress if raw_obs.shape[-1] > 12 else torch.ones_like(unfinished_mask)
        release_overlap = (
            is_yield
            & valid_neighbor
            & (nearest_distance >= release_min_separation)
            & (nearest_closing_speed <= release_max_closing)
            & release_route_mask
        )
    return scenario_mask & unfinished_mask & valid_threat & (threat_score > 0.0) & (role_weight > 0.0) & (~release_overlap)


def _random_deconflict_yield_mask(torch, raw_obs, args, agent_indices=None):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    role_mode = str(getattr(args, 'random_deconflict_role_mode', 'agent-index')).strip().lower()
    if role_mode == 'priority-delta':
        delta, valid_delta = _random_deconflict_pairwise_priority_delta(torch, raw_obs, args)
        threshold = float(getattr(args, 'random_deconflict_priority_delta_yield_threshold', -0.01))
        return valid_delta & (delta <= threshold)
    if role_mode == 'route-eta-delta':
        eta_delta, valid_eta = _random_deconflict_pairwise_route_eta_delta(torch, raw_obs, args)
        threshold = float(getattr(args, 'random_deconflict_route_eta_yield_threshold', 0.02))
        return valid_eta & (eta_delta > threshold)
    if role_mode == 'priority' and raw_obs.shape[-1] > 15:
        priority = raw_obs[:, 15]
        threshold = float(getattr(args, 'random_deconflict_priority_yield_threshold', -0.10))
        return priority <= threshold
    if agent_indices is None:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    return agent_indices.to(device=raw_obs.device) >= 1


def _random_deconflict_local_threat_score(torch, raw_obs, args):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    score = torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    valid = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if sample_count <= 0 or not bool(getattr(args, 'random_deconflict_local_danger', False)):
        return score, valid

    nearest_distance, _, closing_speed, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    safe_separation = max(0.0, float(getattr(args, 'random_deconflict_safe_separation', 1.30)))
    release_separation = max(
        safe_separation + 0.05,
        float(getattr(args, 'random_deconflict_release_separation', 2.40)),
    )
    local_score = torch.clamp(
        (release_separation - nearest_distance) / max(release_separation - safe_separation, 1e-3),
        0.0,
        1.0,
    ).to(dtype=raw_obs.dtype)
    closing_gate = torch.clamp((closing_speed.to(dtype=raw_obs.dtype) + 0.04) / 0.34, 0.0, 1.0)
    local_score = local_score * (0.35 + 0.65 * closing_gate)
    critical_separation = max(0.0, float(getattr(args, 'random_deconflict_critical_separation', 0.0)))
    if critical_separation > 0.0:
        critical_mask = valid_neighbor & (nearest_distance <= critical_separation)
        local_score = torch.where(critical_mask, torch.ones_like(local_score), local_score)
    valid = valid_neighbor & (nearest_distance <= release_separation) & (local_score > 0.0)
    return torch.where(valid, local_score, score), valid


def _random_deconflict_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 1:
        return action_mean.new_zeros(())
    active_mask = _random_deconflict_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, cpa_bearing, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    threat_score = torch.maximum(cpa_score.to(dtype=action_mean.dtype), local_score.to(dtype=action_mean.dtype))
    valid_threat = valid_cpa | valid_local

    danger_power = max(0.1, float(getattr(args, 'random_deconflict_power', 1.0)))
    danger = torch.clamp(threat_score, 0.0, 1.0)
    if danger_power != 1.0:
        danger = danger.pow(danger_power)

    is_yield = _random_deconflict_yield_mask(torch, raw_obs, args, agent_indices).to(device=action_mean.device)
    standon_speed = max(0.0, float(getattr(args, 'random_deconflict_standon_speed', 0.30)))
    yield_speed = max(0.0, float(getattr(args, 'random_deconflict_yield_speed', 0.04)))
    yield_danger_scale = min(1.0, max(0.0, float(getattr(args, 'random_deconflict_yield_danger_scale', 1.0))))
    standon_omega = max(0.0, float(getattr(args, 'random_deconflict_standon_omega', 0.12)))
    yield_omega = max(0.0, float(getattr(args, 'random_deconflict_yield_omega', 0.50)))
    omega_weight = max(0.0, float(getattr(args, 'random_deconflict_omega_weight', 1.0)))
    standon_weight = max(0.0, float(getattr(args, 'random_deconflict_standon_weight', 0.35)))
    yield_weight = max(0.0, float(getattr(args, 'random_deconflict_yield_weight', 1.0)))

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)

    standon_speed_tensor = torch.full_like(danger, standon_speed)
    standon_close_separation = max(0.0, float(getattr(args, 'random_deconflict_standon_close_separation', 0.0)))
    if standon_close_separation > 0.0:
        standon_close_speed = max(0.0, float(getattr(args, 'random_deconflict_standon_close_speed', 0.08)))
        nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
        close_standon = valid_neighbor.to(device=action_mean.device) & (nearest_distance.to(device=action_mean.device) < standon_close_separation)
        standon_speed_tensor = torch.where(
            close_standon,
            torch.minimum(standon_speed_tensor, torch.full_like(standon_speed_tensor, standon_close_speed)),
            standon_speed_tensor,
        )
    yield_speed_tensor = torch.full_like(danger, yield_speed) * torch.clamp(1.0 - yield_danger_scale * danger, min=0.0, max=1.0)
    target_linear = torch.where(is_yield, yield_speed_tensor, standon_speed_tensor)
    target_linear = torch.clamp(target_linear, min=low[0], max=high[0])
    linear_loss = ((action_mean[:, 0] - target_linear) / range_scale[0]) ** 2
    urgency = (0.25 + 0.75 * danger).to(dtype=linear_loss.dtype)
    per_sample_loss = linear_loss * urgency

    if action_mean.shape[-1] >= 2 and omega_weight > 0.0:
        target_omega_mag = torch.where(
            is_yield,
            torch.full_like(danger, yield_omega),
            torch.full_like(danger, standon_omega),
        )
        turn_mode = str(getattr(args, 'random_deconflict_turn_mode', 'starboard')).strip().lower()
        if turn_mode == 'away':
            _, nearest_bearing, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
            use_cpa_bearing = valid_cpa.to(device=action_mean.device) & (
                cpa_score.to(dtype=action_mean.dtype, device=action_mean.device)
                >= local_score.to(dtype=action_mean.dtype, device=action_mean.device)
            )
            active_bearing = torch.where(use_cpa_bearing, cpa_bearing.to(device=action_mean.device), nearest_bearing.to(device=action_mean.device))
            valid_bearing = use_cpa_bearing | valid_neighbor.to(device=action_mean.device)
            turn_sign = torch.where(
                active_bearing > 0.0,
                torch.full_like(active_bearing, -1.0),
                torch.where(active_bearing < 0.0, torch.ones_like(active_bearing), torch.full_like(active_bearing, -1.0)),
            )
            turn_sign = torch.where(valid_bearing, turn_sign, torch.full_like(turn_sign, -1.0))
        else:
            turn_sign = torch.full_like(danger, -1.0)
        target_omega = torch.clamp(turn_sign.to(dtype=action_mean.dtype) * target_omega_mag * danger, min=low[1], max=high[1])
        omega_loss = ((action_mean[:, 1] - target_omega) / range_scale[1]) ** 2
        per_sample_loss = per_sample_loss + omega_weight * omega_loss * danger

    role_weight = torch.where(
        is_yield,
        torch.full_like(danger, yield_weight),
        torch.full_like(danger, standon_weight),
    )
    weights = active_mask.to(dtype=per_sample_loss.dtype) * role_weight.to(dtype=per_sample_loss.dtype)
    weights = weights * valid_threat.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_role_balance_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 1:
        return action_mean.new_zeros(())
    active_mask = _random_deconflict_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, _, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    threat_score = torch.maximum(cpa_score.to(dtype=action_mean.dtype), local_score.to(dtype=action_mean.dtype))
    valid_threat = valid_cpa | valid_local

    danger_power = max(0.1, float(getattr(args, 'random_deconflict_power', 1.0)))
    danger = torch.clamp(threat_score, 0.0, 1.0)
    if danger_power != 1.0:
        danger = danger.pow(danger_power)
    min_danger = max(0.0, float(getattr(args, 'random_role_balance_min_danger', 0.05)))

    is_yield = _random_deconflict_yield_mask(torch, raw_obs, args, agent_indices).to(device=action_mean.device)
    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)
    linear_range = range_scale[0]

    standon_min_speed = max(0.0, float(getattr(args, 'random_role_balance_standon_min_speed', 0.28)))
    yield_max_speed = max(0.0, float(getattr(args, 'random_role_balance_yield_max_speed', 0.10)))
    standon_min_speed = min(standon_min_speed, float(high[0].detach().cpu().item()))
    yield_max_speed = min(yield_max_speed, float(high[0].detach().cpu().item()))

    standon_violation = torch.relu((standon_min_speed - action_mean[:, 0]) / linear_range) ** 2
    standon_close_separation = max(0.0, float(getattr(args, 'random_role_balance_standon_close_separation', 0.0)))
    if standon_close_separation > 0.0:
        nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
        close_standon = valid_neighbor.to(device=action_mean.device) & (nearest_distance.to(device=action_mean.device) < standon_close_separation)
        standon_violation = torch.where(close_standon, torch.zeros_like(standon_violation), standon_violation)
    yield_violation = torch.relu((action_mean[:, 0] - yield_max_speed) / linear_range) ** 2
    standon_weight = max(0.0, float(getattr(args, 'random_role_balance_standon_weight', 1.0)))
    yield_weight = max(0.0, float(getattr(args, 'random_role_balance_yield_weight', 1.0)))
    per_sample_loss = torch.where(
        is_yield,
        torch.full_like(yield_violation, yield_weight) * yield_violation,
        torch.full_like(standon_violation, standon_weight) * standon_violation,
    )

    omega_weight = max(0.0, float(getattr(args, 'random_role_balance_omega_weight', 0.6)))
    yield_min_starboard_omega = max(0.0, float(getattr(args, 'random_role_balance_yield_min_starboard_omega', 0.22)))
    if action_mean.shape[-1] >= 2 and omega_weight > 0.0 and yield_min_starboard_omega > 0.0:
        turn_mode = str(getattr(args, 'random_deconflict_turn_mode', 'starboard')).strip().lower()
        if turn_mode == 'away':
            _, cpa_bearing, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
                torch,
                raw_obs,
                lookahead,
                horizon,
                dcpa_target,
                closing_min,
            )
            local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
            _, nearest_bearing, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
            use_cpa_bearing = valid_cpa.to(device=action_mean.device) & (
                cpa_score.to(dtype=action_mean.dtype, device=action_mean.device)
                >= local_score.to(dtype=action_mean.dtype, device=action_mean.device)
            )
            active_bearing = torch.where(use_cpa_bearing, cpa_bearing.to(device=action_mean.device), nearest_bearing.to(device=action_mean.device))
            valid_bearing = use_cpa_bearing | valid_neighbor.to(device=action_mean.device)
            turn_sign = torch.where(
                active_bearing > 0.0,
                torch.full_like(active_bearing, -1.0),
                torch.where(active_bearing < 0.0, torch.ones_like(active_bearing), torch.full_like(active_bearing, -1.0)),
            )
            turn_sign = torch.where(valid_bearing, turn_sign, torch.full_like(turn_sign, -1.0))
        else:
            turn_sign = torch.full_like(danger, -1.0)
        target_omega = torch.clamp(
            turn_sign.to(dtype=action_mean.dtype) * yield_min_starboard_omega * torch.clamp(danger, 0.0, 1.0),
            min=low[1],
            max=high[1],
        )
        omega_diff = (action_mean[:, 1] - target_omega) / range_scale[1]
        omega_violation = torch.where(
            target_omega >= 0.0,
            torch.relu(-omega_diff),
            torch.relu(omega_diff),
        )
        per_sample_loss = per_sample_loss + is_yield.to(dtype=per_sample_loss.dtype) * omega_weight * (omega_violation ** 2)

    weights = active_mask.to(dtype=per_sample_loss.dtype)
    weights = weights * valid_threat.to(dtype=per_sample_loss.dtype)
    weights = weights * (danger >= min_danger).to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_pairwise_role_guard_active_mask(
    torch,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or scenario_ids is None or agent_indices is None:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in ('two_usv_random_encounter', 'three_usv_random_encounter'):
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    goal_tolerance = max(0.05, float(getattr(args, 'random_deconflict_goal_tolerance', 0.8)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_deconflict_max_distance', 13.0)))
    distance_to_goal = torch.clamp(raw_obs[:, 4], min=0.0)
    unfinished_mask = (distance_to_goal > goal_tolerance) & (distance_to_goal <= max_distance)

    nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, _, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    safe_separation = max(0.0, float(getattr(args, 'random_pairwise_role_guard_safe_separation', 0.82)))
    release_separation = max(
        safe_separation + 0.05,
        float(getattr(args, 'random_pairwise_role_guard_release_separation', 1.45)),
    )
    min_danger = max(0.0, float(getattr(args, 'random_pairwise_role_guard_min_danger', 0.10)))
    near_danger = torch.clamp(
        (release_separation - nearest_distance) / max(release_separation - safe_separation, 1e-3),
        0.0,
        1.0,
    )
    danger = torch.maximum(torch.maximum(cpa_score.to(dtype=raw_obs.dtype), local_score.to(dtype=raw_obs.dtype)), near_danger.to(dtype=raw_obs.dtype))
    valid_threat = valid_cpa | valid_local | valid_neighbor
    return scenario_mask & unfinished_mask & valid_threat & (danger >= min_danger)


def _random_pairwise_role_guard_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    active_mask = _random_pairwise_role_guard_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    nearest_distance, nearest_bearing, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, cpa_bearing, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    safe_separation = max(0.0, float(getattr(args, 'random_pairwise_role_guard_safe_separation', 0.82)))
    release_separation = max(
        safe_separation + 0.05,
        float(getattr(args, 'random_pairwise_role_guard_release_separation', 1.45)),
    )
    near_danger = torch.clamp(
        (release_separation - nearest_distance) / max(release_separation - safe_separation, 1e-3),
        0.0,
        1.0,
    ).to(dtype=raw_obs.dtype)
    threat_score = torch.maximum(torch.maximum(cpa_score.to(dtype=raw_obs.dtype), local_score.to(dtype=raw_obs.dtype)), near_danger)
    danger = torch.clamp(threat_score, 0.0, 1.0).to(dtype=action_mean.dtype, device=action_mean.device)

    is_yield = _random_deconflict_yield_mask(torch, raw_obs, args, agent_indices).to(device=action_mean.device)
    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)

    yield_speed = max(0.0, float(getattr(args, 'random_pairwise_role_guard_yield_speed', 0.015)))
    standon_close_speed = max(0.0, float(getattr(args, 'random_pairwise_role_guard_standon_close_speed', 0.12)))
    target_linear = torch.where(
        is_yield,
        torch.full_like(danger, yield_speed),
        torch.full_like(danger, standon_close_speed),
    )
    target_linear = torch.clamp(target_linear, min=low[0], max=high[0])
    linear_excess = torch.relu((action_mean[:, 0] - target_linear) / range_scale[0])
    linear_weight = max(0.0, float(getattr(args, 'random_pairwise_role_guard_linear_weight', 1.0)))
    per_sample_loss = linear_weight * (linear_excess ** 2) * (0.25 + 0.75 * danger)

    yield_omega = max(0.0, float(getattr(args, 'random_pairwise_role_guard_yield_omega', 0.42)))
    standon_omega = max(0.0, float(getattr(args, 'random_pairwise_role_guard_standon_omega', 0.14)))
    omega_weight = max(0.0, float(getattr(args, 'random_pairwise_role_guard_omega_weight', 2.0)))
    if omega_weight > 0.0:
        use_cpa_bearing = valid_cpa.to(device=action_mean.device) & (
            cpa_score.to(dtype=action_mean.dtype, device=action_mean.device)
            >= torch.maximum(local_score.to(dtype=action_mean.dtype, device=action_mean.device), near_danger.to(dtype=action_mean.dtype, device=action_mean.device))
        )
        active_bearing = torch.where(use_cpa_bearing, cpa_bearing.to(device=action_mean.device), nearest_bearing.to(device=action_mean.device))
        valid_bearing = use_cpa_bearing | valid_neighbor.to(device=action_mean.device) | valid_local.to(device=action_mean.device)
        turn_sign = torch.where(
            active_bearing > 0.0,
            torch.full_like(danger, -1.0),
            torch.where(active_bearing < 0.0, torch.ones_like(danger), torch.full_like(danger, -1.0)),
        )
        turn_sign = torch.where(valid_bearing, turn_sign, torch.full_like(turn_sign, -1.0))
        omega_mag = torch.where(is_yield, torch.full_like(danger, yield_omega), torch.full_like(danger, standon_omega))
        target_omega = torch.clamp(turn_sign * omega_mag * danger, min=low[1], max=high[1])
        omega_loss = ((action_mean[:, 1] - target_omega) / range_scale[1]) ** 2
        per_sample_loss = per_sample_loss + omega_weight * omega_loss * danger

    yield_weight = max(0.0, float(getattr(args, 'random_pairwise_role_guard_yield_weight', 2.0)))
    standon_weight = max(0.0, float(getattr(args, 'random_pairwise_role_guard_standon_weight', 0.55)))
    role_weight = torch.where(is_yield, torch.full_like(danger, yield_weight), torch.full_like(danger, standon_weight))
    weights = active_mask.to(dtype=per_sample_loss.dtype) * role_weight
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_safe_finish_empty_components(torch, raw_obs, sample_count: int):
    zeros = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    return {
        'active': zeros,
        'finish': zeros,
        'hold': zeros,
        'low_priority_active': zeros,
        'deconflict_block': zeros,
        'release_overlap': zeros,
        'candidate': zeros,
    }


def _random_aux_cte_source(args) -> str:
    source = str(getattr(args, 'random_cte_source', 'clipped')).strip().lower()
    return 'raw' if source == 'raw' else 'clipped'


def _random_aux_signed_cte(torch, raw_obs, args, *, like=None):
    if like is None:
        sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
        like = torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    clipped = raw_obs[:, 11] if raw_obs.shape[-1] > 11 else torch.zeros_like(like)
    if _random_aux_cte_source(args) != 'raw':
        return clipped.to(dtype=like.dtype, device=like.device)
    ego_dim = AgentLocalObservation.ego_feature_size()
    raw_cte_index = ego_dim - 2
    neighbor_width = int(raw_obs.shape[-1]) - ego_dim - ENCOUNTER_TYPE_COUNT
    has_current_layout = neighbor_width >= 0 and neighbor_width % NEIGHBOR_FEATURE_COUNT == 0
    if has_current_layout and raw_obs.shape[-1] > raw_cte_index:
        return raw_obs[:, raw_cte_index].to(dtype=like.dtype, device=like.device)
    return clipped.to(dtype=like.dtype, device=like.device)


def _random_safe_finish_components(
    torch,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or scenario_ids is None:
        return _random_safe_finish_empty_components(torch, raw_obs, sample_count)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return _random_safe_finish_empty_components(torch, raw_obs, sample_count)
    if global_state.numel() == 0 or global_state.shape[0] != sample_count or global_state.shape[-1] < 5:
        return _random_safe_finish_empty_components(torch, raw_obs, sample_count)

    scenario_names = tuple(getattr(args, 'random_clear_ahead_scenarios', None) or ('two_usv_random_encounter', 'three_usv_random_encounter'))
    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in scenario_names:
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    goal_tolerance = max(0.05, float(getattr(args, 'random_safe_finish_goal_tolerance', 0.8)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_safe_finish_max_distance', 13.0)))
    hold_distance = max(0.0, float(getattr(args, 'random_safe_finish_hold_distance', 0.0)))
    phase_min = float(getattr(args, 'random_safe_finish_phase_min', -1.0))
    min_team_separation = max(0.0, float(getattr(args, 'random_safe_finish_min_team_separation', 2.7)))
    min_neighbor_separation = max(0.0, float(getattr(args, 'random_safe_finish_min_neighbor_separation', 2.7)))
    max_abs_cte = max(0.0, float(getattr(args, 'random_safe_finish_max_abs_cte', 0.0)))
    max_cpa_score = max(0.0, float(getattr(args, 'random_safe_finish_max_cpa_score', 0.0)))
    max_local_score = max(0.0, float(getattr(args, 'random_safe_finish_max_local_score', 0.0)))
    low_priority_threshold = float(getattr(args, 'random_safe_finish_low_priority_threshold', -0.5))

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    unfinished_mask = (distance > goal_tolerance) & (distance <= max_distance)
    hold_mask = (distance <= hold_distance) if hold_distance > 0.0 else torch.zeros_like(unfinished_mask)
    phase_mask = raw_obs[:, 13] >= phase_min
    cte_clear = torch.ones_like(unfinished_mask)
    if max_abs_cte > 0.0 and raw_obs.shape[-1] > 11:
        cte_clear = torch.abs(_random_aux_signed_cte(torch, raw_obs, args, like=distance)) <= max_abs_cte
    team_min_separation = global_state[:, -5]
    team_clear = team_min_separation >= min_team_separation

    nearest_distance, _, nearest_closing_speed, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    neighbor_clear = (~valid_neighbor) | (nearest_distance >= min_neighbor_separation)

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, _, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    cpa_clear = (~valid_cpa) | (cpa_score <= max_cpa_score)
    local_clear = (~valid_local) | (local_score <= max_local_score)

    active_deconflict = _random_deconflict_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    low_priority_mask = raw_obs[:, 15] <= low_priority_threshold if raw_obs.shape[-1] > 15 else torch.zeros_like(unfinished_mask)
    release_min_separation = max(0.0, float(getattr(args, 'random_safe_finish_yield_release_min_separation', 0.0)))
    release_max_closing = float(getattr(args, 'random_safe_finish_yield_release_max_closing_speed', 0.02))
    release_min_route_progress = max(0.0, float(getattr(args, 'random_safe_finish_yield_release_min_route_progress', 0.0)))
    release_route_mask = raw_obs[:, 12] >= release_min_route_progress if raw_obs.shape[-1] > 12 else torch.ones_like(unfinished_mask)
    release_overlap = torch.zeros_like(unfinished_mask)
    if release_min_separation > 0.0:
        release_overlap = (
            low_priority_mask
            & valid_neighbor
            & (nearest_distance >= release_min_separation)
            & (nearest_closing_speed <= release_max_closing)
            & release_route_mask
        )

    base_finish_candidate = unfinished_mask & team_clear & neighbor_clear & cte_clear
    finish_mask = base_finish_candidate & ((cpa_clear & local_clear) | release_overlap)
    deconflict_block = active_deconflict & base_finish_candidate & (
        (valid_cpa & (cpa_score > max_cpa_score))
        | (valid_local & (local_score > max_local_score))
    ) & (~release_overlap)
    active_mask = scenario_mask & phase_mask & ((finish_mask & (~deconflict_block)) | hold_mask)
    return {
        'active': active_mask,
        'finish': scenario_mask & phase_mask & finish_mask & (~deconflict_block),
        'hold': scenario_mask & phase_mask & hold_mask,
        'low_priority_active': active_mask & low_priority_mask & (~hold_mask),
        'deconflict_block': scenario_mask & phase_mask & deconflict_block,
        'release_overlap': scenario_mask & phase_mask & base_finish_candidate & release_overlap,
        'candidate': scenario_mask & phase_mask & unfinished_mask,
    }


def _random_safe_finish_active_mask(
    torch,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    return _random_safe_finish_components(
        torch,
        raw_obs,
        global_state,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )['active']


def _random_safe_finish_loss(
    torch,
    action_mean,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    active_mask = _random_safe_finish_active_mask(
        torch,
        raw_obs,
        global_state,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    heading_error = torch.atan2(raw_obs[:, 5], raw_obs[:, 6])
    team_min_separation = global_state[:, -5]

    goal_tolerance = max(0.05, float(getattr(args, 'random_safe_finish_goal_tolerance', 0.8)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_safe_finish_max_distance', 13.0)))
    hold_distance = max(0.0, float(getattr(args, 'random_safe_finish_hold_distance', 0.0)))
    min_team_separation = max(0.0, float(getattr(args, 'random_safe_finish_min_team_separation', 2.7)))
    full_team_separation = max(
        min_team_separation + 0.05,
        float(getattr(args, 'random_safe_finish_full_team_separation', 4.0)),
    )
    min_neighbor_separation = max(0.0, float(getattr(args, 'random_safe_finish_min_neighbor_separation', 2.7)))
    target_speed = max(0.0, float(getattr(args, 'random_safe_finish_target_speed', 0.18)))
    min_speed_scale = min(1.0, max(0.0, float(getattr(args, 'random_safe_finish_min_speed_scale', 0.35))))
    hold_weight = max(0.0, float(getattr(args, 'random_safe_finish_hold_weight', 1.0)))
    low_priority_threshold = float(getattr(args, 'random_safe_finish_low_priority_threshold', -0.5))
    low_priority_multiplier = max(0.0, float(getattr(args, 'random_safe_finish_low_priority_speed_multiplier', 1.0)))
    max_omega = max(0.0, float(getattr(args, 'random_safe_finish_max_omega', 0.10)))

    speed_scale = torch.clamp(
        (distance - (0.50 * goal_tolerance)) / max(max_distance - (0.50 * goal_tolerance), 1e-3),
        min_speed_scale,
        1.0,
    )
    separation_source = team_min_separation
    scale_min_separation = min_team_separation
    if bool(getattr(args, 'random_safe_finish_local_separation_scale', False)):
        nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
        separation_source = torch.where(
            valid_neighbor,
            nearest_distance,
            torch.full_like(nearest_distance, full_team_separation),
        )
        scale_min_separation = min_neighbor_separation
    full_scale_separation = max(scale_min_separation + 0.05, full_team_separation)
    separation_scale = torch.clamp(
        (separation_source - scale_min_separation) / max(full_scale_separation - scale_min_separation, 1e-3),
        0.0,
        1.0,
    )
    target_linear = target_speed * speed_scale * separation_scale
    if raw_obs.shape[-1] > 15 and low_priority_multiplier != 1.0:
        priority = raw_obs[:, 15]
        low_priority_mask = priority <= low_priority_threshold
        multiplier = torch.where(
            low_priority_mask,
            torch.full_like(target_linear, low_priority_multiplier),
            torch.ones_like(target_linear),
        )
        target_linear = target_linear * multiplier

    hold_mask = (distance <= hold_distance) if hold_distance > 0.0 else torch.zeros_like(active_mask)
    target_linear = torch.where(hold_mask, torch.zeros_like(target_linear), target_linear)
    target_omega = torch.where(
        hold_mask,
        torch.zeros_like(heading_error),
        -torch.clamp(heading_error / 0.85, -1.0, 1.0) * max_omega,
    )
    target = torch.stack([target_linear, target_omega], dim=-1).to(dtype=action_mean.dtype)

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    target = torch.max(low, torch.min(high, target))
    range_scale = torch.clamp(high - low, min=1e-3)

    omega_weight = max(0.0, float(getattr(args, 'random_safe_finish_omega_weight', 0.12)))
    linear_loss = ((action_mean[:, 0] - target[:, 0]) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target[:, 1]) / range_scale[1]) ** 2
    progress_urgency = torch.clamp(distance / max(max_distance, 1e-3), 0.0, 1.0)
    clearance_urgency = torch.clamp(1.0 - separation_scale, 0.0, 1.0)
    hold_boost = torch.where(hold_mask, torch.full_like(progress_urgency, 0.45), torch.zeros_like(progress_urgency))
    urgency = (0.45 + 0.35 * progress_urgency + 0.20 * clearance_urgency).to(dtype=action_mean.dtype)
    urgency = urgency + hold_boost.to(dtype=action_mean.dtype)
    per_sample_loss = (linear_loss + omega_weight * omega_loss) * urgency
    if hold_weight != 1.0:
        per_sample_loss = per_sample_loss * torch.where(
            hold_mask,
            torch.full_like(per_sample_loss, hold_weight),
            torch.ones_like(per_sample_loss),
        )

    mask_f = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = mask_f * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    else:
        weights = mask_f
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_goal_hold_active_mask(
    torch,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or scenario_ids is None:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    hold_distance = max(0.0, float(getattr(args, 'random_goal_hold_distance', 0.0)))
    if hold_distance <= 0.0:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in ('two_usv_random_encounter', 'three_usv_random_encounter'):
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    phase_min = float(getattr(args, 'random_goal_hold_phase_min', -1.0))
    phase_mask = raw_obs[:, 13] >= phase_min
    return scenario_mask & phase_mask & (distance <= hold_distance)


def _random_goal_hold_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
):
    if action_mean.shape[-1] < 1:
        return action_mean.new_zeros(())
    active_mask = _random_goal_hold_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)
    target_speed = max(0.0, float(getattr(args, 'random_goal_hold_target_speed', 0.0)))
    target_linear = torch.full_like(action_mean[:, 0], target_speed).clamp(min=low[0], max=high[0])
    per_sample_loss = ((action_mean[:, 0] - target_linear) / range_scale[0]) ** 2

    if action_mean.shape[-1] >= 2:
        target_omega = float(getattr(args, 'random_goal_hold_target_omega', 0.0))
        target_omega_tensor = torch.full_like(action_mean[:, 1], target_omega).clamp(min=low[1], max=high[1])
        omega_weight = max(0.0, float(getattr(args, 'random_goal_hold_omega_weight', 1.0)))
        omega_loss = ((action_mean[:, 1] - target_omega_tensor) / range_scale[1]) ** 2
        per_sample_loss = per_sample_loss + omega_weight * omega_loss

    weights = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_offroute_finish_active_mask(
    torch,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or scenario_ids is None:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if global_state.numel() == 0 or global_state.shape[0] != sample_count or global_state.shape[-1] < 5:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in ('two_usv_random_encounter', 'three_usv_random_encounter'):
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    goal_tolerance = max(0.05, float(getattr(args, 'random_offroute_finish_goal_tolerance', 0.8)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_offroute_finish_max_distance', 13.0)))
    min_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_offroute_finish_min_distance', 1.60)))
    min_route_progress = min(1.0, max(0.0, float(getattr(args, 'random_offroute_finish_route_progress_min', 0.82))))
    min_abs_cte = max(0.0, float(getattr(args, 'random_offroute_finish_min_abs_cte', 1.20)))
    phase_min = float(getattr(args, 'random_offroute_finish_phase_min', -1.0))
    min_team_separation = max(0.0, float(getattr(args, 'random_offroute_finish_min_team_separation', 2.75)))
    min_neighbor_separation = max(0.0, float(getattr(args, 'random_offroute_finish_min_neighbor_separation', 2.90)))
    max_cpa_score = max(0.0, float(getattr(args, 'random_offroute_finish_max_cpa_score', 0.0)))
    max_local_score = max(0.0, float(getattr(args, 'random_offroute_finish_max_local_score', 0.0)))

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    abs_cte = torch.abs(_random_aux_signed_cte(torch, raw_obs, args, like=distance))
    route_progress = raw_obs[:, 12] if raw_obs.shape[-1] > 12 else torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    phase = raw_obs[:, 13] if raw_obs.shape[-1] > 13 else torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    offroute_mask = (route_progress >= min_route_progress) & (abs_cte >= min_abs_cte)
    unfinished_mask = (distance > goal_tolerance) & (distance <= max_distance) & (distance >= min_distance)
    phase_mask = phase >= phase_min

    team_min_separation = global_state[:, -5]
    team_clear = team_min_separation >= min_team_separation
    nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    neighbor_clear = (~valid_neighbor) | (nearest_distance >= min_neighbor_separation)

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, _, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    cpa_clear = (~valid_cpa) | (cpa_score <= max_cpa_score)
    local_clear = (~valid_local) | (local_score <= max_local_score)
    active_deconflict = _random_deconflict_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    threat_clear = cpa_clear & local_clear & (~active_deconflict)
    if bool(getattr(args, 'random_offroute_finish_allow_threat_overlap', False)):
        threat_clear = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)

    return (
        scenario_mask
        & unfinished_mask
        & offroute_mask
        & phase_mask
        & team_clear
        & neighbor_clear
        & threat_clear
    )


def _random_offroute_finish_loss(
    torch,
    action_mean,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    active_mask = _random_offroute_finish_active_mask(
        torch,
        raw_obs,
        global_state,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    abs_cte = torch.abs(_random_aux_signed_cte(torch, raw_obs, args, like=action_mean[:, 0]))
    heading_error = torch.atan2(raw_obs[:, 5], raw_obs[:, 6])
    min_abs_cte = max(0.0, float(getattr(args, 'random_offroute_finish_min_abs_cte', 1.20)))
    full_abs_cte = max(min_abs_cte + 0.05, float(getattr(args, 'random_offroute_finish_full_abs_cte', 3.00)))
    cte_urgency = torch.clamp((abs_cte - min_abs_cte) / max(full_abs_cte - min_abs_cte, 1e-3), 0.0, 1.0).to(dtype=action_mean.dtype)

    target_speed = max(0.0, float(getattr(args, 'random_offroute_finish_target_speed', 0.14)))
    min_speed = max(0.0, float(getattr(args, 'random_offroute_finish_min_speed', 0.06)))
    max_omega = max(0.0, float(getattr(args, 'random_offroute_finish_max_omega', 0.18)))
    omega_reference = max(0.05, float(getattr(args, 'random_offroute_finish_omega_reference', 0.65)))
    omega_weight = max(0.0, float(getattr(args, 'random_offroute_finish_omega_weight', 0.55)))

    target_linear = target_speed - (target_speed - min_speed) * cte_urgency
    target_omega = _goal_heading_omega_sign(args) * torch.clamp(heading_error / omega_reference, -1.0, 1.0) * max_omega * (0.45 + 0.55 * cte_urgency)
    target = torch.stack([target_linear, target_omega], dim=-1).to(dtype=action_mean.dtype)

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    target = torch.max(low, torch.min(high, target))
    range_scale = torch.clamp(high - low, min=1e-3)

    linear_loss = ((action_mean[:, 0] - target[:, 0]) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target[:, 1]) / range_scale[1]) ** 2
    urgency = (0.45 + 0.55 * cte_urgency).to(dtype=action_mean.dtype)
    per_sample_loss = (linear_loss + omega_weight * omega_loss * urgency) * urgency

    weights = active_mask.to(dtype=per_sample_loss.dtype)
    weights = weights * _random_recovery_safety_gate_weights(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    ).to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_cte_recovery_active_mask(
    torch,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or scenario_ids is None:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in ('two_usv_random_encounter', 'three_usv_random_encounter'):
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    goal_tolerance = max(0.05, float(getattr(args, 'random_cte_recovery_goal_tolerance', 0.8)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_cte_recovery_max_distance', 13.0)))
    min_abs_cte = max(0.0, float(getattr(args, 'random_cte_recovery_min_abs_cte', 1.10)))
    min_neighbor_separation = max(0.0, float(getattr(args, 'random_cte_recovery_min_neighbor_separation', 0.85)))
    if bool(getattr(args, 'random_cte_recovery_all_agents', False)):
        allowed_agent_indices = ()
    else:
        allowed_agent_indices = tuple(int(index) for index in (getattr(args, 'random_cte_recovery_agent_indices', None) or ()))

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    cte = _random_aux_signed_cte(torch, raw_obs, args, like=distance)
    unfinished_mask = (distance > goal_tolerance) & (distance <= max_distance)
    cte_mask = torch.abs(cte) >= min_abs_cte
    if allowed_agent_indices:
        if agent_indices is None:
            role_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
        else:
            role_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
            agent_index_tensor = agent_indices.to(device=raw_obs.device)
            for allowed_index in allowed_agent_indices:
                role_mask = role_mask | (agent_index_tensor == allowed_index)
    else:
        role_mask = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)
    nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    neighbor_clear = (~valid_neighbor) | (nearest_distance >= min_neighbor_separation)

    threat_clear = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)
    if not bool(getattr(args, 'random_cte_recovery_allow_threat_overlap', False)):
        lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
        horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
        dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
        closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
        _, _, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
            torch,
            raw_obs,
            lookahead,
            horizon,
            dcpa_target,
            closing_min,
        )
        local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
        active_deconflict = _random_deconflict_active_mask(
            torch,
            raw_obs,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        threat_clear = ((~valid_cpa) | (cpa_score <= 0.0)) & ((~valid_local) | (local_score <= 0.0)) & (~active_deconflict)

    return scenario_mask & unfinished_mask & cte_mask & role_mask & neighbor_clear & threat_clear


def _random_recovery_safety_gate_weights(
    torch,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    gate_scale = min(1.0, max(0.0, float(getattr(args, 'random_recovery_safety_gate_scale', 1.0))))
    weights = torch.ones(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    if sample_count <= 0 or gate_scale >= 1.0:
        return weights
    deconflict_mask = _random_deconflict_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    pairwise_guard_mask = _random_pairwise_role_guard_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    safety_mask = deconflict_mask | pairwise_guard_mask
    gate_mode = str(getattr(args, 'random_recovery_safety_gate_mode', 'batch')).strip().lower()
    if gate_mode == 'batch':
        if bool(safety_mask.any().detach().cpu()):
            return weights * gate_scale
        return weights
    return torch.where(
        safety_mask,
        torch.full((sample_count,), gate_scale, dtype=raw_obs.dtype, device=raw_obs.device),
        weights,
    )


def _random_cte_recovery_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    active_mask = _random_cte_recovery_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    signed_cte = _random_aux_signed_cte(torch, raw_obs, args, like=action_mean[:, 0])
    cte = torch.abs(signed_cte)
    heading_error = torch.atan2(raw_obs[:, 5], raw_obs[:, 6])
    min_abs_cte = max(0.0, float(getattr(args, 'random_cte_recovery_min_abs_cte', 1.10)))
    full_abs_cte = max(min_abs_cte + 0.05, float(getattr(args, 'random_cte_recovery_full_abs_cte', 3.00)))
    cte_urgency = torch.clamp((cte - min_abs_cte) / max(full_abs_cte - min_abs_cte, 1e-3), 0.0, 1.0).to(dtype=action_mean.dtype)

    target_speed = max(0.0, float(getattr(args, 'random_cte_recovery_target_speed', 0.18)))
    min_speed = max(0.0, float(getattr(args, 'random_cte_recovery_min_speed', 0.07)))
    max_omega = max(0.0, float(getattr(args, 'random_cte_recovery_max_omega', 0.30)))
    omega_reference = max(0.05, float(getattr(args, 'random_cte_recovery_omega_reference', 0.55)))
    linear_weight = max(0.0, float(getattr(args, 'random_cte_recovery_linear_weight', 1.0)))
    omega_weight = max(0.0, float(getattr(args, 'random_cte_recovery_omega_weight', 1.0)))

    speed_cte_slowdown = max(0.0, float(getattr(args, 'random_cte_recovery_speed_cte_slowdown', 1.0)))
    target_linear = target_speed - (target_speed - min_speed) * torch.clamp(cte_urgency * speed_cte_slowdown, 0.0, 1.0)
    recovery_heading_error = heading_error.to(dtype=action_mean.dtype)
    target_omega = _goal_heading_omega_sign(args) * torch.clamp(heading_error / omega_reference, -1.0, 1.0) * max_omega
    omega_mode = str(getattr(args, 'random_cte_recovery_omega_mode', 'goal-heading')).strip().lower().replace('_', '-')
    if omega_mode == 'signed-cte':
        target_omega = -torch.sign(signed_cte).to(dtype=action_mean.dtype) * max_omega * (0.45 + 0.55 * cte_urgency)
    elif omega_mode == 'signed-cte-inverted':
        target_omega = torch.sign(signed_cte).to(dtype=action_mean.dtype) * max_omega * (0.45 + 0.55 * cte_urgency)
    elif omega_mode == 'goal-cte-lookahead':
        cte_lookahead = max(0.25, float(getattr(args, 'random_cte_recovery_cte_lookahead', 4.0)))
        cte_heading_scale = max(0.0, float(getattr(args, 'random_cte_recovery_cte_heading_scale', 1.0)))
        lookahead = torch.full_like(signed_cte.to(dtype=action_mean.dtype), cte_lookahead)
        cte_heading_bias = torch.atan2(signed_cte.to(dtype=action_mean.dtype), lookahead) * cte_heading_scale
        blended_heading_error = heading_error.to(dtype=action_mean.dtype) + cte_heading_bias
        blended_heading_error = torch.atan2(torch.sin(blended_heading_error), torch.cos(blended_heading_error))
        recovery_heading_error = blended_heading_error
        target_omega = _goal_heading_omega_sign(args) * torch.clamp(blended_heading_error / omega_reference, -1.0, 1.0) * max_omega
    speed_heading_gate = min(1.0, max(0.0, float(getattr(args, 'random_cte_recovery_speed_heading_gate', 0.0))))
    if speed_heading_gate > 0.0:
        recovery_alignment = torch.clamp(torch.cos(torch.clamp(torch.abs(recovery_heading_error), max=math.pi / 2.0)), 0.0, 1.0)
        target_linear = target_linear * ((1.0 - speed_heading_gate) + speed_heading_gate * recovery_alignment)
        target_linear = torch.clamp(target_linear, min=min_speed)
    target = torch.stack([target_linear, target_omega], dim=-1).to(dtype=action_mean.dtype)

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    target = torch.max(low, torch.min(high, target))
    range_scale = torch.clamp(high - low, min=1e-3)

    linear_loss = ((action_mean[:, 0] - target[:, 0]) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target[:, 1]) / range_scale[1]) ** 2
    urgency = (0.55 + 0.45 * cte_urgency).to(dtype=action_mean.dtype)
    per_sample_loss = (linear_weight * linear_loss + omega_weight * omega_loss) * urgency

    weights = active_mask.to(dtype=per_sample_loss.dtype)
    weights = weights * _random_recovery_safety_gate_weights(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    ).to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_clear_ahead_front_clear_mask(torch, raw_obs, args):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0:
        return torch.zeros(0, dtype=torch.bool, device=raw_obs.device)

    neighbor_start = AgentLocalObservation.ego_feature_size()
    available = int(raw_obs.shape[-1]) - neighbor_start - ENCOUNTER_TYPE_COUNT
    neighbor_slots = max(0, available // NEIGHBOR_FEATURE_COUNT)
    if neighbor_slots <= 0:
        return torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)

    neighbor_end = neighbor_start + neighbor_slots * NEIGHBOR_FEATURE_COUNT
    neighbors = raw_obs[:, neighbor_start:neighbor_end].reshape(sample_count, neighbor_slots, NEIGHBOR_FEATURE_COUNT)
    distances = torch.clamp(neighbors[:, :, 4], min=0.0)
    valid_slots = distances > 1e-6
    bearings = neighbors[:, :, 5]

    cone_mode = str(getattr(args, 'random_clear_ahead_cone_mode', 'goal')).strip().lower()
    if cone_mode == 'goal':
        heading_error = torch.atan2(raw_obs[:, 5], raw_obs[:, 6]).unsqueeze(1)
        bearings = torch.atan2(torch.sin(bearings - heading_error), torch.cos(bearings - heading_error))

    half_angle = math.radians(max(1.0, min(179.0, float(getattr(args, 'random_clear_ahead_bearing_deg', 35.0)))))
    clear_distance = max(0.0, float(getattr(args, 'random_clear_ahead_distance', 5.0)))
    front_blocker = valid_slots & (torch.abs(bearings) <= half_angle) & (distances < clear_distance)
    return ~front_blocker.any(dim=1)


def _random_clear_ahead_component_masks(
    torch,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    empty = {
        'scenario': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'unfinished': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'front': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'cpa': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'local': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'neighbor': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'progress': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'cte': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'role': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'deconflict': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'active': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
    }
    if sample_count <= 0 or scenario_ids is None:
        return empty
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return empty

    scenario_names = tuple(getattr(args, 'random_clear_ahead_scenarios', None) or ('two_usv_random_encounter', 'three_usv_random_encounter'))
    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in scenario_names:
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    goal_tolerance = max(0.05, float(getattr(args, 'random_clear_ahead_goal_tolerance', 1.0)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_clear_ahead_max_distance', 13.0)))
    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    unfinished_mask = (distance > goal_tolerance) & (distance <= max_distance)
    route_progress = torch.clamp(raw_obs[:, 12], 0.0, 1.0) if raw_obs.shape[-1] > 12 else torch.zeros_like(distance)
    min_route_progress = min(1.0, max(0.0, float(getattr(args, 'random_clear_ahead_min_route_progress', 0.0))))
    max_route_progress = min(1.0, max(min_route_progress, float(getattr(args, 'random_clear_ahead_max_route_progress', 1.0))))
    progress_mask = (route_progress >= min_route_progress) & (route_progress <= max_route_progress)
    abs_cte = torch.abs(_random_aux_signed_cte(torch, raw_obs, args, like=distance))
    min_abs_cte = max(0.0, float(getattr(args, 'random_clear_ahead_min_abs_cte', 0.0)))
    max_abs_cte = max(0.0, float(getattr(args, 'random_clear_ahead_max_abs_cte', 0.0)))
    cte_mask = abs_cte >= min_abs_cte
    if max_abs_cte > 0.0:
        cte_mask = cte_mask & (abs_cte <= max_abs_cte)
    front_clear = _random_clear_ahead_front_clear_mask(torch, raw_obs, args)

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, _, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    max_cpa_score = max(0.0, float(getattr(args, 'random_clear_ahead_max_cpa_score', 0.0)))
    cpa_clear = (~valid_cpa) | (cpa_score <= max_cpa_score)

    max_local_score = max(0.0, float(getattr(args, 'random_clear_ahead_max_local_score', 0.0)))
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    local_clear = (~valid_local) | (local_score <= max_local_score)

    min_neighbor_separation = max(0.0, float(getattr(args, 'random_clear_ahead_min_neighbor_separation', 0.0)))
    neighbor_clear = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)
    if min_neighbor_separation > 0.0:
        nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
        neighbor_clear = (~valid_neighbor) | (nearest_distance >= min_neighbor_separation)

    allowed_agent_indices = tuple(int(index) for index in (getattr(args, 'random_clear_ahead_agent_indices', None) or ()))
    if allowed_agent_indices:
        if agent_indices is None:
            role_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
        else:
            role_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
            agent_index_tensor = agent_indices.to(device=raw_obs.device)
            for allowed_index in allowed_agent_indices:
                role_mask = role_mask | (agent_index_tensor == allowed_index)
    else:
        role_mask = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)

    deconflict_clear = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)
    if bool(getattr(args, 'random_clear_ahead_exclude_deconflict', False)):
        deconflict_clear = ~_random_deconflict_active_mask(
            torch,
            raw_obs,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )

    active = scenario_mask & unfinished_mask & progress_mask & cte_mask & front_clear & cpa_clear & local_clear & neighbor_clear & role_mask & deconflict_clear
    return {
        'scenario': scenario_mask,
        'unfinished': unfinished_mask,
        'front': front_clear,
        'cpa': cpa_clear,
        'local': local_clear,
        'neighbor': neighbor_clear,
        'progress': progress_mask,
        'cte': cte_mask,
        'role': role_mask,
        'deconflict': deconflict_clear,
        'active': active,
    }


def _random_clear_ahead_active_mask(
    torch,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    return _random_clear_ahead_component_masks(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )['active']


def _random_clear_ahead_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    active_mask = _random_clear_ahead_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)

    target_source = str(getattr(args, 'random_clear_ahead_target_source', 'raw')).strip().lower()
    target_speed = max(0.0, float(getattr(args, 'random_clear_ahead_target_speed', 0.30)))
    min_speed = max(0.0, float(getattr(args, 'random_clear_ahead_min_speed', 0.18)))
    max_omega = max(0.0, float(getattr(args, 'random_clear_ahead_max_omega', 0.18)))
    omega_reference = max(0.05, float(getattr(args, 'random_clear_ahead_omega_reference', 0.55)))
    omega_weight = max(0.0, float(getattr(args, 'random_clear_ahead_omega_weight', 1.0)))

    heading_error = torch.atan2(raw_obs[:, 5], raw_obs[:, 6])
    abs_cte = torch.abs(_random_aux_signed_cte(torch, raw_obs, args, like=action_mean[:, 0])).to(dtype=action_mean.dtype)
    if target_source == 'raw':
        raw_linear = raw_obs[:, 7].to(dtype=action_mean.dtype)
        raw_omega = raw_obs[:, 8].to(dtype=action_mean.dtype)
        target_linear = torch.clamp(raw_linear, min=min_speed, max=max(target_speed, min_speed))
        target_omega = torch.clamp(raw_omega, min=-max_omega, max=max_omega)
    else:
        target_linear = torch.full_like(action_mean[:, 0], target_speed)
        target_omega = _goal_heading_omega_sign(args) * torch.clamp(heading_error.to(dtype=action_mean.dtype) / omega_reference, -1.0, 1.0) * max_omega

    speed_cte_slowdown = max(0.0, float(getattr(args, 'random_clear_ahead_speed_cte_slowdown', 0.0)))
    if speed_cte_slowdown > 0.0:
        cte_start = max(0.0, float(getattr(args, 'random_clear_ahead_speed_cte_start', 0.0)))
        cte_full = float(getattr(args, 'random_clear_ahead_speed_cte_full', 0.0))
        if cte_full <= cte_start:
            cte_full = max(cte_start + 0.05, float(getattr(args, 'random_clear_ahead_max_abs_cte', 0.0)))
        if cte_full <= cte_start:
            cte_full = cte_start + 1.5
        cte_slowdown = torch.clamp((abs_cte - cte_start) / max(cte_full - cte_start, 1e-3), 0.0, 1.0)
        cte_slowdown = torch.clamp(cte_slowdown * speed_cte_slowdown, 0.0, 1.0).to(dtype=target_linear.dtype)
        target_linear = target_linear - (target_linear - min_speed) * cte_slowdown
        target_linear = torch.clamp(target_linear, min=min_speed)

    target_linear = torch.clamp(target_linear, min=low[0], max=high[0])
    target_omega = torch.clamp(target_omega, min=low[1], max=high[1])
    linear_loss = ((action_mean[:, 0] - target_linear) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target_omega) / range_scale[1]) ** 2

    heading_urgency = torch.clamp(torch.abs(heading_error).to(dtype=action_mean.dtype) / omega_reference, 0.0, 1.0)
    cte_urgency = torch.clamp(abs_cte / 1.5, 0.0, 1.0)
    urgency = 0.45 + 0.55 * torch.maximum(heading_urgency, cte_urgency)
    per_sample_loss = (linear_loss + omega_weight * omega_loss) * urgency

    weights = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _random_close_formation_escape_active_mask(
    torch,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or scenario_ids is None:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)

    scenario_names = tuple(
        getattr(args, 'random_close_formation_escape_scenarios', None)
        or ('two_usv_random_encounter', 'three_usv_random_encounter')
    )
    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in scenario_names:
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    goal_tolerance = max(0.05, float(getattr(args, 'random_close_formation_escape_goal_tolerance', 1.0)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_close_formation_escape_max_distance', 20.0)))
    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    unfinished_mask = (distance > goal_tolerance) & (distance <= max_distance)

    max_self_speed = max(0.0, float(getattr(args, 'random_close_formation_escape_max_self_speed', 0.22)))
    raw_linear = raw_obs[:, 7] if raw_obs.shape[-1] > 7 else torch.zeros_like(distance)
    self_slow_mask = raw_linear <= max_self_speed

    nearest_distance, _, closing_speed, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    min_sep = max(0.0, float(getattr(args, 'random_close_formation_escape_min_sep', 1.40)))
    max_sep = max(min_sep + 0.05, float(getattr(args, 'random_close_formation_escape_max_sep', 2.40)))
    sep_mask = valid_neighbor & (nearest_distance >= min_sep) & (nearest_distance <= max_sep)

    require_closing = bool(getattr(args, 'random_close_formation_escape_require_closing', False))
    if require_closing:
        sep_mask = sep_mask & (closing_speed > 0.0)

    allowed_agent_indices = tuple(
        int(index) for index in (getattr(args, 'random_close_formation_escape_agent_indices', None) or ())
    )
    if allowed_agent_indices:
        if agent_indices is None:
            role_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
        else:
            role_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
            agent_index_tensor = agent_indices.to(device=raw_obs.device)
            for allowed_index in allowed_agent_indices:
                role_mask = role_mask | (agent_index_tensor == allowed_index)
    else:
        role_mask = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)

    return scenario_mask & unfinished_mask & self_slow_mask & sep_mask & role_mask


def _random_close_formation_escape_loss(
    torch,
    action_mean,
    raw_obs,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    active_mask = _random_close_formation_escape_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)

    target_speed = max(0.0, float(getattr(args, 'random_close_formation_escape_target_speed', 0.40)))
    min_speed = max(0.0, float(getattr(args, 'random_close_formation_escape_min_speed', 0.20)))
    max_omega = max(0.0, float(getattr(args, 'random_close_formation_escape_max_omega', 0.45)))
    omega_weight = max(0.0, float(getattr(args, 'random_close_formation_escape_omega_weight', 1.6)))
    min_sep = max(0.05, float(getattr(args, 'random_close_formation_escape_min_sep', 1.40)))
    max_sep = max(min_sep + 0.05, float(getattr(args, 'random_close_formation_escape_max_sep', 2.40)))

    nearest_distance, nearest_bearing, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    # Urgency rises smoothly as nearest neighbor closes from max_sep toward min_sep.
    urgency = torch.clamp((max_sep - nearest_distance) / max(max_sep - min_sep, 1e-3), 0.0, 1.0).to(dtype=action_mean.dtype)

    # Lateral escape: turn AWAY from the nearest neighbor bearing (body-frame).
    bearing = nearest_bearing.to(dtype=action_mean.dtype)
    # bearing > 0 means neighbor on left -> turn right (negative omega) to escape; sign opposite.
    target_omega = -torch.sign(bearing) * max_omega * (0.50 + 0.50 * urgency)
    # If bearing magnitude is tiny (head-on), bias to a fixed starboard escape (right turn, COLREGS-flavored).
    head_on_mask = torch.abs(bearing) < 0.10
    target_omega = torch.where(head_on_mask, -max_omega * (0.50 + 0.50 * urgency), target_omega)

    # Decisive forward: keep target_speed at the upper bound; the floor avoids hard brake.
    target_linear = torch.full_like(action_mean[:, 0], max(target_speed, min_speed))

    target_linear = torch.clamp(target_linear, min=low[0], max=high[0])
    target_omega = torch.clamp(target_omega, min=low[1], max=high[1])

    linear_loss = ((action_mean[:, 0] - target_linear) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target_omega) / range_scale[1]) ** 2
    per_sample_loss = (linear_loss + omega_weight * omega_loss) * (0.55 + 0.45 * urgency)

    weights = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _team_route_progress_blocks_from_state(torch, raw_obs, global_state):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    empty_progress = torch.zeros((sample_count, 0), dtype=raw_obs.dtype, device=raw_obs.device)
    empty_mask = torch.zeros((sample_count, 0), dtype=torch.bool, device=raw_obs.device)
    if sample_count <= 0 or raw_obs.shape[-1] <= 12:
        return empty_progress, empty_mask
    if global_state.numel() == 0 or global_state.shape[0] != sample_count or global_state.shape[-1] < raw_obs.shape[-1] + 5:
        return empty_progress, empty_mask

    local_size = int(raw_obs.shape[-1])
    packed_size = int(global_state.shape[-1]) - 5
    if local_size <= 0 or packed_size < local_size or packed_size % local_size != 0:
        return empty_progress, empty_mask

    team_blocks = global_state[:, :packed_size].reshape(sample_count, packed_size // local_size, local_size)
    valid_team_mask = team_blocks.abs().sum(dim=-1) > 1e-6
    team_progress = torch.clamp(team_blocks[:, :, 12], 0.0, 1.0)
    return team_progress, valid_team_mask


def _team_max_route_progress_from_state(torch, raw_obs, global_state):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    if sample_count <= 0 or raw_obs.shape[-1] <= 12:
        return torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    team_progress, valid_team_mask = _team_route_progress_blocks_from_state(torch, raw_obs, global_state)
    if team_progress.numel() == 0:
        return torch.clamp(raw_obs[:, 12], 0.0, 1.0)
    masked_progress = torch.where(valid_team_mask, team_progress, torch.full_like(team_progress, -1.0))
    max_progress = masked_progress.max(dim=1).values
    own_progress = torch.clamp(raw_obs[:, 12], 0.0, 1.0)
    return torch.maximum(max_progress, own_progress)


def _random_late_lagging_component_masks(
    torch,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    sample_count = int(raw_obs.shape[0]) if raw_obs.ndim > 0 else 0
    empty = {
        'scenario': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'unfinished': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'lagging': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'phase': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'role': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'finished': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'team': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'neighbor': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'threat': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
        'active': torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device),
    }
    if sample_count <= 0 or scenario_ids is None:
        return empty
    if raw_obs.shape[-1] < AgentLocalObservation.ego_feature_size() + ENCOUNTER_TYPE_COUNT:
        return empty
    if global_state.numel() == 0 or global_state.shape[0] != sample_count or global_state.shape[-1] < 5:
        return empty

    scenario_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
    for scenario_name in ('two_usv_random_encounter', 'three_usv_random_encounter'):
        scenario_id = scenario_to_index.get(scenario_name)
        if scenario_id is not None:
            scenario_mask = scenario_mask | (scenario_ids == int(scenario_id))

    goal_tolerance = max(0.05, float(getattr(args, 'random_late_lagging_goal_tolerance', 1.0)))
    max_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_late_lagging_max_distance', 80.0)))
    min_distance = max(goal_tolerance + 0.05, float(getattr(args, 'random_late_lagging_min_distance', 2.0)))
    max_self_progress = min(1.0, max(0.0, float(getattr(args, 'random_late_lagging_max_self_progress', 0.25))))
    min_team_progress = min(1.0, max(0.0, float(getattr(args, 'random_late_lagging_min_team_progress', 0.75))))
    min_progress_gap = min(1.0, max(0.0, float(getattr(args, 'random_late_lagging_min_progress_gap', 0.50))))
    allowed_agent_indices = tuple(int(index) for index in (getattr(args, 'random_late_lagging_agent_indices', None) or ()))
    min_finished_teammates = max(0, int(getattr(args, 'random_late_lagging_min_finished_teammates', 0)))
    finished_progress = min(1.0, max(0.0, float(getattr(args, 'random_late_lagging_finished_progress', 0.98))))
    phase_min = float(getattr(args, 'random_late_lagging_phase_min', -1.0))
    min_team_separation = max(0.0, float(getattr(args, 'random_late_lagging_min_team_separation', 4.8)))
    min_neighbor_separation = max(0.0, float(getattr(args, 'random_late_lagging_min_neighbor_separation', 4.8)))
    max_cpa_score = max(0.0, float(getattr(args, 'random_late_lagging_max_cpa_score', 0.0)))
    max_local_score = max(0.0, float(getattr(args, 'random_late_lagging_max_local_score', 0.0)))

    distance = torch.clamp(raw_obs[:, 4], min=0.0)
    route_progress = torch.clamp(raw_obs[:, 12], 0.0, 1.0) if raw_obs.shape[-1] > 12 else torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    phase = raw_obs[:, 13] if raw_obs.shape[-1] > 13 else torch.zeros(sample_count, dtype=raw_obs.dtype, device=raw_obs.device)
    team_max_progress = _team_max_route_progress_from_state(torch, raw_obs, global_state)
    lagging_mask = (
        (route_progress <= max_self_progress)
        & (team_max_progress >= min_team_progress)
        & ((team_max_progress - route_progress) >= min_progress_gap)
    )
    if allowed_agent_indices:
        if agent_indices is None:
            role_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
        else:
            role_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
            agent_index_tensor = agent_indices.to(device=raw_obs.device)
            for allowed_index in allowed_agent_indices:
                role_mask = role_mask | (agent_index_tensor == allowed_index)
    else:
        role_mask = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)

    if min_finished_teammates > 0:
        team_progress, valid_team_mask = _team_route_progress_blocks_from_state(torch, raw_obs, global_state)
        if team_progress.numel() == 0 or agent_indices is None:
            finished_mask = torch.zeros(sample_count, dtype=torch.bool, device=raw_obs.device)
        else:
            finished_team = valid_team_mask & (team_progress >= finished_progress)
            block_indices = torch.arange(team_progress.shape[1], device=raw_obs.device).view(1, -1)
            self_indices = agent_indices.to(device=raw_obs.device).view(-1, 1)
            finished_team = finished_team & (block_indices != self_indices)
            finished_mask = finished_team.sum(dim=1) >= min_finished_teammates
    else:
        finished_mask = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)
    unfinished_mask = (distance > goal_tolerance) & (distance <= max_distance) & (distance >= min_distance)
    phase_mask = phase >= phase_min

    team_min_separation = global_state[:, -5]
    team_clear = team_min_separation >= min_team_separation
    nearest_distance, _, _, valid_neighbor = _nearest_neighbor_features(torch, raw_obs)
    neighbor_clear = (~valid_neighbor) | (nearest_distance >= min_neighbor_separation)

    lookahead = max(0.0, float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)))
    horizon = max(0.0, float(getattr(args, 'random_deconflict_time_horizon', 30.0)))
    dcpa_target = max(0.0, float(getattr(args, 'random_deconflict_dcpa_target', 1.45)))
    closing_min = max(0.0, float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)))
    _, _, _, _, cpa_score, valid_cpa = _neighbor_cpa_threat_features(
        torch,
        raw_obs,
        lookahead,
        horizon,
        dcpa_target,
        closing_min,
    )
    local_score, valid_local = _random_deconflict_local_threat_score(torch, raw_obs, args)
    active_deconflict = _random_deconflict_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    active_pairwise = _random_pairwise_role_guard_active_mask(
        torch,
        raw_obs,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    cpa_clear = (~valid_cpa) | (cpa_score <= max_cpa_score)
    local_clear = (~valid_local) | (local_score <= max_local_score)
    threat_clear = cpa_clear & local_clear & (~active_deconflict) & (~active_pairwise)
    if bool(getattr(args, 'random_late_lagging_allow_threat_overlap', False)):
        threat_clear = torch.ones(sample_count, dtype=torch.bool, device=raw_obs.device)

    active = scenario_mask & unfinished_mask & lagging_mask & phase_mask & role_mask & finished_mask & team_clear & neighbor_clear & threat_clear
    return {
        'scenario': scenario_mask,
        'unfinished': unfinished_mask,
        'lagging': lagging_mask,
        'phase': phase_mask,
        'role': role_mask,
        'finished': finished_mask,
        'team': team_clear,
        'neighbor': neighbor_clear,
        'threat': threat_clear,
        'active': active,
    }


def _random_late_lagging_active_mask(
    torch,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    agent_indices=None,
):
    return _random_late_lagging_component_masks(
        torch,
        raw_obs,
        global_state,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )['active']


def _random_late_lagging_loss(
    torch,
    action_mean,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape[-1] < 2:
        return action_mean.new_zeros(())
    active_mask = _random_late_lagging_active_mask(
        torch,
        raw_obs,
        global_state,
        scenario_ids,
        scenario_to_index,
        args,
        agent_indices=agent_indices,
    )
    if not bool(active_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)
    heading_error = torch.atan2(raw_obs[:, 5], raw_obs[:, 6])

    target_speed = max(0.0, float(getattr(args, 'random_late_lagging_target_speed', 0.16)))
    min_speed = max(0.0, float(getattr(args, 'random_late_lagging_min_speed', 0.06)))
    max_omega = max(0.0, float(getattr(args, 'random_late_lagging_max_omega', 0.34)))
    omega_reference = max(0.05, float(getattr(args, 'random_late_lagging_omega_reference', 0.80)))
    omega_weight = max(0.0, float(getattr(args, 'random_late_lagging_omega_weight', 2.60)))
    target_source = str(getattr(args, 'random_late_lagging_target_source', 'goal')).strip().lower()

    if target_source == 'raw' and raw_obs.shape[-1] > 8:
        raw_linear = torch.clamp(raw_obs[:, 7].to(dtype=action_mean.dtype), min=0.0, max=max(target_speed, min_speed))
        target_linear = torch.maximum(raw_linear, torch.full_like(raw_linear, min_speed))
        target_omega = torch.clamp(raw_obs[:, 8].to(dtype=action_mean.dtype), min=-max_omega, max=max_omega)
    else:
        target_linear = torch.full_like(action_mean[:, 0], target_speed)
        target_omega = _goal_heading_omega_sign(args) * torch.clamp(heading_error.to(dtype=action_mean.dtype) / omega_reference, -1.0, 1.0) * max_omega

    target_linear = torch.clamp(target_linear, min=low[0], max=high[0])
    target_omega = torch.clamp(target_omega, min=low[1], max=high[1])
    linear_loss = ((action_mean[:, 0] - target_linear) / range_scale[0]) ** 2
    omega_loss = ((action_mean[:, 1] - target_omega) / range_scale[1]) ** 2

    route_progress = torch.clamp(raw_obs[:, 12], 0.0, 1.0) if raw_obs.shape[-1] > 12 else torch.zeros_like(action_mean[:, 0])
    team_max_progress = _team_max_route_progress_from_state(torch, raw_obs, global_state).to(dtype=action_mean.dtype)
    progress_gap = torch.clamp(team_max_progress - route_progress.to(dtype=action_mean.dtype), 0.0, 1.0)
    urgency = 0.45 + 0.55 * progress_gap
    per_sample_loss = (linear_loss + omega_weight * omega_loss) * urgency

    weights = active_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = weights * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _policy_anchor_loss(
    torch,
    action_mean,
    anchor_action_mean,
    raw_obs,
    global_state,
    scenario_ids,
    scenario_to_index: dict[str, int],
    args,
    action_low_tensor,
    action_high_tensor,
    sample_weights=None,
    agent_indices=None,
):
    if action_mean.shape != anchor_action_mean.shape:
        return action_mean.new_zeros(())
    sample_count = int(action_mean.shape[0])
    if sample_count <= 0:
        return action_mean.new_zeros(())

    if bool(getattr(args, 'policy_anchor_crossing_only', False)):
        crossing_scenario_id = scenario_to_index.get('three_usv_crossing')
        if crossing_scenario_id is not None and scenario_ids is not None:
            anchor_mask = scenario_ids == int(crossing_scenario_id)
        elif raw_obs.numel() > 0 and raw_obs.shape[-1] >= 2:
            anchor_mask = raw_obs[:, -2] > 0.5
        else:
            anchor_mask = torch.zeros(sample_count, dtype=torch.bool, device=action_mean.device)
    else:
        anchor_mask = torch.ones(sample_count, dtype=torch.bool, device=action_mean.device)

    if bool(getattr(args, 'policy_anchor_exclude_lagging_finish', False)):
        lagging_mask = _lagging_finish_active_mask(
            torch,
            raw_obs,
            global_state,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if lagging_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~lagging_mask)

    if bool(getattr(args, 'policy_anchor_exclude_team_safety_brake', False)):
        brake_mask = _team_safety_brake_active_mask(
            torch,
            raw_obs,
            global_state,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if brake_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~brake_mask)

    if bool(getattr(args, 'policy_anchor_exclude_random_deconflict', False)):
        random_deconflict_mask = _random_deconflict_active_mask(
            torch,
            raw_obs,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if random_deconflict_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~random_deconflict_mask)

    if bool(getattr(args, 'policy_anchor_exclude_random_safe_finish', False)):
        random_safe_finish_mask = _random_safe_finish_active_mask(
            torch,
            raw_obs,
            global_state,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if random_safe_finish_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~random_safe_finish_mask)

    if bool(getattr(args, 'policy_anchor_exclude_random_goal_hold', False)):
        random_goal_hold_mask = _random_goal_hold_active_mask(
            torch,
            raw_obs,
            scenario_ids,
            scenario_to_index,
            args,
        )
        if random_goal_hold_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~random_goal_hold_mask)

    if bool(getattr(args, 'policy_anchor_exclude_random_offroute_finish', False)):
        random_offroute_finish_mask = _random_offroute_finish_active_mask(
            torch,
            raw_obs,
            global_state,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if random_offroute_finish_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~random_offroute_finish_mask)

    if bool(getattr(args, 'policy_anchor_exclude_random_cte_recovery', False)):
        random_cte_recovery_mask = _random_cte_recovery_active_mask(
            torch,
            raw_obs,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if random_cte_recovery_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~random_cte_recovery_mask)

    if bool(getattr(args, 'policy_anchor_exclude_random_clear_ahead', False)):
        random_clear_ahead_mask = _random_clear_ahead_active_mask(
            torch,
            raw_obs,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if random_clear_ahead_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~random_clear_ahead_mask)

    if bool(getattr(args, 'policy_anchor_exclude_random_late_lagging', False)):
        random_late_lagging_mask = _random_late_lagging_active_mask(
            torch,
            raw_obs,
            global_state,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if random_late_lagging_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~random_late_lagging_mask)

    if bool(getattr(args, 'policy_anchor_exclude_random_pairwise_role_guard', False)):
        pairwise_guard_mask = _random_pairwise_role_guard_active_mask(
            torch,
            raw_obs,
            scenario_ids,
            scenario_to_index,
            args,
            agent_indices=agent_indices,
        )
        if pairwise_guard_mask.shape[0] == sample_count:
            anchor_mask = anchor_mask & (~pairwise_guard_mask)

    if not bool(anchor_mask.any().detach().cpu()):
        return action_mean.new_zeros(())

    low = action_low_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    high = action_high_tensor.to(dtype=action_mean.dtype, device=action_mean.device)
    range_scale = torch.clamp(high - low, min=1e-3)
    per_sample_loss = (((action_mean - anchor_action_mean.detach().to(dtype=action_mean.dtype)) / range_scale) ** 2).mean(dim=-1)

    mask_f = anchor_mask.to(dtype=per_sample_loss.dtype)
    if sample_weights is not None:
        weights = mask_f * sample_weights.to(dtype=per_sample_loss.dtype, device=per_sample_loss.device)
    else:
        weights = mask_f
    return (per_sample_loss * weights).sum() / torch.clamp(weights.sum(), min=1.0)


def _run_crossing_imitation_pretrain(
    torch,
    actor,
    optimizer,
    flat_obs,
    flat_raw_obs,
    flat_scenario_ids_np,
    flat_scenario_weights,
    scenario_to_index,
    args,
    action_low_tensor,
    action_high_tensor,
    *,
    sample_count: int,
    minibatch_size: int,
    device,
    imitation_weight: float,
) -> float:
    """Run trainer-side actor-only crossing imitation updates.

    These updates are deliberately applied after a rollout has already been
    collected, so PPO ratios for that rollout are not computed against a policy
    that was moved before the PPO step. The next rollout then sees the stronger
    role-conditioned crossing behavior while the exported artifact remains a
    single neural MAPPO policy.
    """
    pretrain_epochs = max(0, int(getattr(args, 'crossing_imitation_pretrain_epochs', 0)))
    if pretrain_epochs <= 0 or imitation_weight <= 0.0 or sample_count <= 0:
        return 0.0

    actor_parameters = [parameter for parameter in actor.parameters() if parameter.requires_grad]
    if not actor_parameters:
        return 0.0

    last_loss = 0.0
    for _ in range(pretrain_epochs):
        permutation = torch.randperm(sample_count, device=device)
        for start in range(0, sample_count, minibatch_size):
            batch_indices = permutation[start:start + minibatch_size]
            batch_obs = flat_obs[batch_indices]
            batch_raw_obs = flat_raw_obs[batch_indices]
            batch_scenario_ids = None
            if flat_scenario_ids_np is not None:
                batch_scenario_ids = torch.as_tensor(
                    flat_scenario_ids_np[batch_indices.detach().cpu().numpy()],
                    dtype=torch.long,
                    device=device,
                )
            batch_weights = flat_scenario_weights[batch_indices] if flat_scenario_weights is not None else None

            action_mean = _actor_forward(actor, batch_obs, batch_scenario_ids)
            if getattr(args, 'squash_actions', False):
                action_mean = action_mean.clamp(-3.0, 3.0)
                _half = (action_high_tensor - action_low_tensor) / 2.0
                _mid = (action_high_tensor + action_low_tensor) / 2.0
                action_mean = torch.tanh(action_mean) * _half + _mid

            imitation_loss = _crossing_imitation_loss(
                torch,
                action_mean,
                batch_raw_obs,
                batch_scenario_ids,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_weights=batch_weights,
            )
            if not imitation_loss.requires_grad:
                continue

            optimizer.zero_grad(set_to_none=True)
            (float(imitation_weight) * imitation_loss).backward()
            torch.nn.utils.clip_grad_norm_(actor_parameters, args.max_grad_norm)
            optimizer.step()
            last_loss = float(imitation_loss.detach().cpu().item())

    return last_loss


def _run_random_deconflict_pretrain(
    torch,
    actor,
    optimizer,
    flat_obs,
    flat_raw_obs,
    flat_scenario_ids_np,
    flat_scenario_weights,
    flat_agent_indices_np,
    scenario_to_index,
    args,
    action_low_tensor,
    action_high_tensor,
    *,
    sample_count: int,
    minibatch_size: int,
    device,
    deconflict_weight: float,
) -> float:
    pretrain_epochs = max(0, int(getattr(args, 'random_deconflict_pretrain_epochs', 0)))
    if pretrain_epochs <= 0 or deconflict_weight <= 0.0 or sample_count <= 0 or flat_agent_indices_np is None:
        return 0.0

    actor_parameters = [parameter for parameter in actor.parameters() if parameter.requires_grad]
    if not actor_parameters:
        return 0.0
    pretrain_lr = float(getattr(args, 'random_deconflict_pretrain_learning_rate', 0.0))
    pretrain_optimizer = optimizer
    if pretrain_lr > 0.0:
        pretrain_optimizer = torch.optim.Adam(actor_parameters, lr=pretrain_lr)
    pretrain_max_grad_norm = float(getattr(args, 'random_deconflict_pretrain_max_grad_norm', 0.0))
    if pretrain_max_grad_norm <= 0.0:
        pretrain_max_grad_norm = float(getattr(args, 'max_grad_norm', 0.5))

    last_loss = 0.0
    for _ in range(pretrain_epochs):
        permutation = torch.randperm(sample_count, device=device)
        for start in range(0, sample_count, minibatch_size):
            batch_indices = permutation[start:start + minibatch_size]
            batch_np_indices = batch_indices.detach().cpu().numpy()
            batch_obs = flat_obs[batch_indices]
            batch_raw_obs = flat_raw_obs[batch_indices]
            batch_scenario_ids = None
            if flat_scenario_ids_np is not None:
                batch_scenario_ids = torch.as_tensor(
                    flat_scenario_ids_np[batch_np_indices],
                    dtype=torch.long,
                    device=device,
                )
            batch_agent_indices = torch.as_tensor(
                flat_agent_indices_np[batch_np_indices],
                dtype=torch.long,
                device=device,
            )
            batch_weights = flat_scenario_weights[batch_indices] if flat_scenario_weights is not None else None

            action_mean = _actor_forward(actor, batch_obs, batch_scenario_ids)
            if getattr(args, 'squash_actions', False):
                action_mean = action_mean.clamp(-3.0, 3.0)
                _half = (action_high_tensor - action_low_tensor) / 2.0
                _mid = (action_high_tensor + action_low_tensor) / 2.0
                action_mean = torch.tanh(action_mean) * _half + _mid

            deconflict_loss = _random_deconflict_loss(
                torch,
                action_mean,
                batch_raw_obs,
                batch_scenario_ids,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_weights=batch_weights,
                agent_indices=batch_agent_indices,
            )
            if not deconflict_loss.requires_grad:
                continue

            pretrain_optimizer.zero_grad(set_to_none=True)
            (float(deconflict_weight) * deconflict_loss).backward()
            torch.nn.utils.clip_grad_norm_(actor_parameters, pretrain_max_grad_norm)
            pretrain_optimizer.step()
            last_loss = float(deconflict_loss.detach().cpu().item())

    return last_loss


def _run_random_role_balance_pretrain(
    torch,
    actor,
    optimizer,
    flat_obs,
    flat_raw_obs,
    flat_scenario_ids_np,
    flat_scenario_weights,
    flat_agent_indices_np,
    scenario_to_index,
    args,
    action_low_tensor,
    action_high_tensor,
    *,
    sample_count: int,
    minibatch_size: int,
    device,
    role_balance_weight: float,
) -> float:
    pretrain_epochs = max(0, int(getattr(args, 'random_role_balance_pretrain_epochs', 0)))
    if pretrain_epochs <= 0 or role_balance_weight <= 0.0 or sample_count <= 0 or flat_agent_indices_np is None:
        return 0.0

    actor_parameters = [parameter for parameter in actor.parameters() if parameter.requires_grad]
    if not actor_parameters:
        return 0.0
    pretrain_lr = float(getattr(args, 'random_role_balance_pretrain_learning_rate', 0.0))
    pretrain_optimizer = optimizer
    if pretrain_lr > 0.0:
        pretrain_optimizer = torch.optim.Adam(actor_parameters, lr=pretrain_lr)
    pretrain_max_grad_norm = float(getattr(args, 'random_role_balance_pretrain_max_grad_norm', 0.0))
    if pretrain_max_grad_norm <= 0.0:
        pretrain_max_grad_norm = float(getattr(args, 'max_grad_norm', 0.5))

    last_loss = 0.0
    for _ in range(pretrain_epochs):
        permutation = torch.randperm(sample_count, device=device)
        for start in range(0, sample_count, minibatch_size):
            batch_indices = permutation[start:start + minibatch_size]
            batch_np_indices = batch_indices.detach().cpu().numpy()
            batch_obs = flat_obs[batch_indices]
            batch_raw_obs = flat_raw_obs[batch_indices]
            batch_scenario_ids = None
            if flat_scenario_ids_np is not None:
                batch_scenario_ids = torch.as_tensor(
                    flat_scenario_ids_np[batch_np_indices],
                    dtype=torch.long,
                    device=device,
                )
            batch_agent_indices = torch.as_tensor(
                flat_agent_indices_np[batch_np_indices],
                dtype=torch.long,
                device=device,
            )
            batch_weights = flat_scenario_weights[batch_indices] if flat_scenario_weights is not None else None

            action_mean = _actor_forward(actor, batch_obs, batch_scenario_ids)
            if getattr(args, 'squash_actions', False):
                action_mean = action_mean.clamp(-3.0, 3.0)
                _half = (action_high_tensor - action_low_tensor) / 2.0
                _mid = (action_high_tensor + action_low_tensor) / 2.0
                action_mean = torch.tanh(action_mean) * _half + _mid

            role_balance_loss = _random_role_balance_loss(
                torch,
                action_mean,
                batch_raw_obs,
                batch_scenario_ids,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_weights=batch_weights,
                agent_indices=batch_agent_indices,
            )
            if not role_balance_loss.requires_grad:
                continue

            pretrain_optimizer.zero_grad(set_to_none=True)
            (float(role_balance_weight) * role_balance_loss).backward()
            torch.nn.utils.clip_grad_norm_(actor_parameters, pretrain_max_grad_norm)
            pretrain_optimizer.step()
            last_loss = float(role_balance_loss.detach().cpu().item())

    return last_loss


def _run_random_pairwise_role_guard_pretrain(
    torch,
    actor,
    optimizer,
    flat_obs,
    flat_raw_obs,
    flat_scenario_ids_np,
    flat_scenario_weights,
    flat_agent_indices_np,
    scenario_to_index,
    args,
    action_low_tensor,
    action_high_tensor,
    *,
    sample_count: int,
    minibatch_size: int,
    device,
    pairwise_role_guard_weight: float,
) -> float:
    pretrain_epochs = max(0, int(getattr(args, 'random_pairwise_role_guard_pretrain_epochs', 0)))
    if pretrain_epochs <= 0 or pairwise_role_guard_weight <= 0.0 or sample_count <= 0 or flat_agent_indices_np is None:
        return 0.0

    actor_parameters = [parameter for parameter in actor.parameters() if parameter.requires_grad]
    if not actor_parameters:
        return 0.0
    pretrain_lr = float(getattr(args, 'random_pairwise_role_guard_pretrain_learning_rate', 0.0))
    pretrain_optimizer = optimizer
    if pretrain_lr > 0.0:
        pretrain_optimizer = torch.optim.Adam(actor_parameters, lr=pretrain_lr)
    pretrain_max_grad_norm = float(getattr(args, 'random_pairwise_role_guard_pretrain_max_grad_norm', 0.0))
    if pretrain_max_grad_norm <= 0.0:
        pretrain_max_grad_norm = float(getattr(args, 'max_grad_norm', 0.5))

    last_loss = 0.0
    for _ in range(pretrain_epochs):
        permutation = torch.randperm(sample_count, device=device)
        for start in range(0, sample_count, minibatch_size):
            batch_indices = permutation[start:start + minibatch_size]
            batch_np_indices = batch_indices.detach().cpu().numpy()
            batch_obs = flat_obs[batch_indices]
            batch_raw_obs = flat_raw_obs[batch_indices]
            batch_scenario_ids = None
            if flat_scenario_ids_np is not None:
                batch_scenario_ids = torch.as_tensor(
                    flat_scenario_ids_np[batch_np_indices],
                    dtype=torch.long,
                    device=device,
                )
            batch_agent_indices = torch.as_tensor(
                flat_agent_indices_np[batch_np_indices],
                dtype=torch.long,
                device=device,
            )
            batch_weights = flat_scenario_weights[batch_indices] if flat_scenario_weights is not None else None

            action_mean = _actor_forward(actor, batch_obs, batch_scenario_ids)
            if getattr(args, 'squash_actions', False):
                action_mean = action_mean.clamp(-3.0, 3.0)
                _half = (action_high_tensor - action_low_tensor) / 2.0
                _mid = (action_high_tensor + action_low_tensor) / 2.0
                action_mean = torch.tanh(action_mean) * _half + _mid

            pairwise_role_guard_loss = _random_pairwise_role_guard_loss(
                torch,
                action_mean,
                batch_raw_obs,
                batch_scenario_ids,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_weights=batch_weights,
                agent_indices=batch_agent_indices,
            )
            if not pairwise_role_guard_loss.requires_grad:
                continue

            pretrain_optimizer.zero_grad(set_to_none=True)
            (float(pairwise_role_guard_weight) * pairwise_role_guard_loss).backward()
            torch.nn.utils.clip_grad_norm_(actor_parameters, pretrain_max_grad_norm)
            pretrain_optimizer.step()
            last_loss = float(pairwise_role_guard_loss.detach().cpu().item())

    return last_loss


def _run_random_recovery_pretrain(
    torch,
    actor,
    optimizer,
    flat_obs,
    flat_raw_obs,
    flat_states,
    flat_scenario_ids_np,
    flat_scenario_weights,
    flat_agent_indices_np,
    scenario_to_index,
    args,
    action_low_tensor,
    action_high_tensor,
    *,
    sample_count: int,
    minibatch_size: int,
    device,
    offroute_weight: float,
    cte_weight: float,
    clear_ahead_weight: float,
    late_lagging_weight: float,
) -> float:
    pretrain_epochs = max(0, int(getattr(args, 'random_recovery_pretrain_epochs', 0)))
    if pretrain_epochs <= 0 or sample_count <= 0 or flat_agent_indices_np is None:
        return 0.0
    offroute_weight = float(offroute_weight) * max(0.0, float(getattr(args, 'random_recovery_pretrain_offroute_scale', 1.0)))
    cte_weight = float(cte_weight) * max(0.0, float(getattr(args, 'random_recovery_pretrain_cte_scale', 1.0)))
    clear_ahead_weight = float(clear_ahead_weight) * max(0.0, float(getattr(args, 'random_recovery_pretrain_clear_scale', 1.0)))
    late_lagging_weight = float(late_lagging_weight) * max(0.0, float(getattr(args, 'random_recovery_pretrain_late_lagging_scale', 1.0)))
    if offroute_weight <= 0.0 and cte_weight <= 0.0 and clear_ahead_weight <= 0.0 and late_lagging_weight <= 0.0:
        return 0.0

    actor_parameters = [parameter for parameter in actor.parameters() if parameter.requires_grad]
    if not actor_parameters:
        return 0.0
    pretrain_lr = float(getattr(args, 'random_recovery_pretrain_learning_rate', 0.0))
    pretrain_optimizer = optimizer
    if pretrain_lr > 0.0:
        pretrain_optimizer = torch.optim.Adam(actor_parameters, lr=pretrain_lr)
    pretrain_max_grad_norm = float(getattr(args, 'random_recovery_pretrain_max_grad_norm', 0.0))
    if pretrain_max_grad_norm <= 0.0:
        pretrain_max_grad_norm = float(getattr(args, 'max_grad_norm', 0.5))

    last_loss = 0.0
    for _ in range(pretrain_epochs):
        permutation = torch.randperm(sample_count, device=device)
        for start in range(0, sample_count, minibatch_size):
            batch_indices = permutation[start:start + minibatch_size]
            batch_np_indices = batch_indices.detach().cpu().numpy()
            batch_obs = flat_obs[batch_indices]
            batch_raw_obs = flat_raw_obs[batch_indices]
            batch_states = flat_states[batch_indices]
            batch_scenario_ids = None
            if flat_scenario_ids_np is not None:
                batch_scenario_ids = torch.as_tensor(
                    flat_scenario_ids_np[batch_np_indices],
                    dtype=torch.long,
                    device=device,
                )
            batch_agent_indices = torch.as_tensor(
                flat_agent_indices_np[batch_np_indices],
                dtype=torch.long,
                device=device,
            )
            batch_weights = flat_scenario_weights[batch_indices] if flat_scenario_weights is not None else None

            action_mean = _actor_forward(actor, batch_obs, batch_scenario_ids)
            if getattr(args, 'squash_actions', False):
                action_mean = action_mean.clamp(-3.0, 3.0)
                _half = (action_high_tensor - action_low_tensor) / 2.0
                _mid = (action_high_tensor + action_low_tensor) / 2.0
                action_mean = torch.tanh(action_mean) * _half + _mid

            recovery_loss = action_mean.new_zeros(())
            if offroute_weight > 0.0:
                recovery_loss = recovery_loss + float(offroute_weight) * _random_offroute_finish_loss(
                    torch,
                    action_mean,
                    batch_raw_obs,
                    batch_states,
                    batch_scenario_ids,
                    scenario_to_index,
                    args,
                    action_low_tensor,
                    action_high_tensor,
                    sample_weights=batch_weights,
                    agent_indices=batch_agent_indices,
                )
            if cte_weight > 0.0:
                recovery_loss = recovery_loss + float(cte_weight) * _random_cte_recovery_loss(
                    torch,
                    action_mean,
                    batch_raw_obs,
                    batch_scenario_ids,
                    scenario_to_index,
                    args,
                    action_low_tensor,
                    action_high_tensor,
                    sample_weights=batch_weights,
                    agent_indices=batch_agent_indices,
                )
            if clear_ahead_weight > 0.0:
                recovery_loss = recovery_loss + float(clear_ahead_weight) * _random_clear_ahead_loss(
                    torch,
                    action_mean,
                    batch_raw_obs,
                    batch_scenario_ids,
                    scenario_to_index,
                    args,
                    action_low_tensor,
                    action_high_tensor,
                    sample_weights=batch_weights,
                    agent_indices=batch_agent_indices,
                )
            if late_lagging_weight > 0.0:
                recovery_loss = recovery_loss + float(late_lagging_weight) * _random_late_lagging_loss(
                    torch,
                    action_mean,
                    batch_raw_obs,
                    batch_states,
                    batch_scenario_ids,
                    scenario_to_index,
                    args,
                    action_low_tensor,
                    action_high_tensor,
                    sample_weights=batch_weights,
                    agent_indices=batch_agent_indices,
                )
            if not recovery_loss.requires_grad:
                continue

            pretrain_optimizer.zero_grad(set_to_none=True)
            recovery_loss.backward()
            torch.nn.utils.clip_grad_norm_(actor_parameters, pretrain_max_grad_norm)
            pretrain_optimizer.step()
            last_loss = float(recovery_loss.detach().cpu().item())

    return last_loss


def _scenario_index_map(scenarios: tuple[str, ...]) -> dict[str, int]:
    return {str(scenario_name): index for index, scenario_name in enumerate(scenarios)}


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


def _resolve_curriculum_scenarios(args, model_scenarios: tuple[str, ...]) -> tuple[str, ...]:
    requested = tuple(str(scenario) for scenario in (getattr(args, 'curriculum_scenarios', None) or ()))
    if not requested:
        return model_scenarios

    agent_count = max(2, int(args.num_agents))
    model_set = set(model_scenarios)
    unknown = [scenario for scenario in requested if scenario not in model_set]
    if unknown:
        raise ValueError(
            '--curriculum-scenario entries must also be present in --scenario so the actor has '
            f'a stable scenario branch id. Missing from --scenario: {", ".join(unknown)}'
        )

    incompatible = [
        scenario
        for scenario in requested
        if MultiAgentScenarioFactory.required_agent_count(scenario) > agent_count
    ]
    if incompatible:
        raise ValueError(
            f'Curriculum scenarios require more agents than configured (num_agents={agent_count}): '
            f'{", ".join(incompatible)}'
        )
    return requested


class _CurriculumSeedScheduler:
    def __init__(self, seeds, *, start_index: int = 0):
        self._seeds = tuple(int(seed) for seed in (seeds or ()))
        self._index = int(start_index)

    @property
    def enabled(self) -> bool:
        return bool(self._seeds)

    @property
    def seeds(self) -> tuple[int, ...]:
        return self._seeds

    def next(self) -> int | None:
        if not self._seeds:
            return None
        seed = self._seeds[self._index % len(self._seeds)]
        self._index += 1
        return int(seed)


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
        conflict_bearing_floor=float(args.conflict_bearing_floor),
        conflict_risk_time_horizon=float(args.conflict_risk_time_horizon),
        conflict_risk_safe_dcpa=float(args.conflict_risk_safe_dcpa),
        conflict_brake_weight=float(args.conflict_brake_weight),
        conflict_progress_scale=float(args.conflict_progress_scale),
        conflict_resolution_reward_weight=float(args.conflict_resolution_reward_weight),
        conflict_escalation_penalty_weight=float(args.conflict_escalation_penalty_weight),
        unsafe_close_speed_penalty_weight=float(args.unsafe_close_speed_penalty_weight),
        path_deviation_penalty_weight=float(args.path_deviation_penalty_weight),
        path_deviation_tolerance=float(args.path_deviation_tolerance),
        path_deviation_conflict_scale=float(args.path_deviation_conflict_scale),
        path_deviation_use_unclipped_cte=bool(args.path_deviation_use_unclipped_cte),
        path_deviation_exclude_overtaking=bool(getattr(args, 'path_deviation_exclude_overtaking', False)),
        path_inefficiency_penalty_weight=float(getattr(args, 'path_inefficiency_penalty_weight', 0.0)),
        waypoint_pass_quality_bonus=float(getattr(args, 'waypoint_pass_quality_bonus', 0.0)),
        role_speed_asymmetry_weight=float(getattr(args, 'role_speed_asymmetry_weight', 0.0)),
        role_giveway_speed=float(getattr(args, 'role_giveway_speed', RewardConfig.role_giveway_speed)),
        wrong_heading_speed_penalty_weight=float(getattr(args, 'wrong_heading_speed_penalty_weight', 0.0)),
        wrong_heading_speed_threshold_deg=float(getattr(args, 'wrong_heading_speed_threshold_deg', RewardConfig.wrong_heading_speed_threshold_deg)),
        wrong_heading_conflict_relief=float(getattr(args, 'wrong_heading_conflict_relief', 0.0)),
        far_side_turn_penalty_weight=float(getattr(args, 'far_side_turn_penalty_weight', 0.0)),
        far_side_turn_threshold_deg=float(getattr(args, 'far_side_turn_threshold_deg', RewardConfig.far_side_turn_threshold_deg)),
        goal_queue_weight=float(getattr(args, 'goal_queue_weight', 0.0)),
        goal_queue_cluster_radius=float(getattr(args, 'goal_queue_cluster_radius', RewardConfig.goal_queue_cluster_radius)),
        goal_queue_engage_distance=float(getattr(args, 'goal_queue_engage_distance', RewardConfig.goal_queue_engage_distance)),
        goal_queue_follower_speed=float(getattr(args, 'goal_queue_follower_speed', RewardConfig.goal_queue_follower_speed)),
        goal_queue_min_separation=float(getattr(args, 'goal_queue_min_separation', RewardConfig.goal_queue_min_separation)),
        desired_conflict_speed=float(args.desired_conflict_speed),
        stop_go_penalty_weight=float(args.stop_go_penalty_weight),
        head_on_guidance_distance=float(args.head_on_guidance_distance),
        crossing_guidance_distance=float(args.crossing_guidance_distance),
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
        turn_speed_coupling_penalty_weight=float(args.turn_speed_coupling_penalty_weight),
        turn_speed_coupling_floor=float(args.turn_speed_coupling_floor),
        turn_speed_coupling_deadband_deg=float(args.turn_speed_coupling_deadband_deg),
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
        clear_ahead_distance=float(args.clear_ahead_distance),
        clear_ahead_bearing_deg=float(args.clear_ahead_bearing_deg),
        clear_ahead_cte_weight=float(args.clear_ahead_cte_weight),
        clear_ahead_heading_weight=float(args.clear_ahead_heading_weight),
        clear_ahead_omega_weight=float(args.clear_ahead_omega_weight),
        avoidance_turn_reward_weight=float(args.avoidance_turn_reward_weight),
        anticipatory_avoidance_turn_reward_weight=float(args.anticipatory_avoidance_turn_reward_weight),
        anticipatory_avoidance_turn_penalty_weight=float(args.anticipatory_avoidance_turn_penalty_weight),
        anticipatory_avoidance_turn_distance=float(args.anticipatory_avoidance_turn_distance),
        anticipatory_cpa_distance=float(args.anticipatory_cpa_distance),
        anticipatory_cpa_time_horizon=float(args.anticipatory_cpa_time_horizon),
        anticipatory_dcpa_target=float(args.anticipatory_dcpa_target),
        anticipatory_dcpa_deficit_penalty_weight=float(args.anticipatory_dcpa_deficit_penalty_weight),
        anticipatory_dcpa_improvement_reward_weight=float(args.anticipatory_dcpa_improvement_reward_weight),
        anticipatory_closing_reduction_reward_weight=float(args.anticipatory_closing_reduction_reward_weight),
        anticipatory_yield_speed=float(args.anticipatory_yield_speed),
        anticipatory_yield_speed_penalty_weight=float(args.anticipatory_yield_speed_penalty_weight),
        near_goal_idle_penalty_weight=float(args.near_goal_idle_penalty_weight),
    )


def _build_model_metadata(args, agent_namespaces: tuple[str, ...]) -> dict:
    action_dim = 1 if args.action_mode == 'angular_only' else 2
    if args.rl_control_mode == 'pure':
        linear_limit = max(float(args.linear_delta_limit), float(args.cruise_speed))
        angular_limit = max(float(args.angular_delta_limit), float(args.max_angular_velocity))
        min_fwd = max(0.0, float(getattr(args, 'min_forward_speed', 0.0)))
        if args.action_mode == 'angular_only':
            action_low = np.asarray([-angular_limit], dtype=np.float32)
            action_high = np.asarray([angular_limit], dtype=np.float32)
        elif args.action_mode == 'speed_scale':
            scale_min = float(np.clip(getattr(args, 'action_speed_scale_min', 0.25), 0.0, 1.0))
            action_low = np.asarray([scale_min, -angular_limit], dtype=np.float32)
            action_high = np.asarray([1.0, angular_limit], dtype=np.float32)
        else:
            action_low = np.asarray(
                [min_fwd, -angular_limit],
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
    obs_normalizer=None,
):
    payload = {
        'actor_state_dict': actor.state_dict(),
        'critic_state_dict': critic.state_dict(),
        'actor_log_std': actor_log_std.detach().cpu(),
        'agent_namespaces': agent_namespaces,
        'local_observation_size': model_metadata['local_observation_size'],
        'global_state_size': model_metadata['global_state_size'],
        'ego_dim': AgentLocalObservation.ego_feature_size(),
        'neighbor_feature_dim': NEIGHBOR_FEATURE_COUNT,
        'action_dim': model_metadata['action_dim'],
        'hidden_sizes': hidden_sizes,
        'ppo_policy_loss_scale': float(getattr(args, 'ppo_policy_loss_scale', 1.0)),
        'ppo_value_loss_scale': float(getattr(args, 'ppo_value_loss_scale', 1.0)),
        'action_low': model_metadata['action_low'].tolist(),
        'action_high': model_metadata['action_high'].tolist(),
        'scenarios': scenarios,
        'max_neighbors': args.max_neighbors,
        'max_agents': max(len(agent_namespaces), args.max_agents),
        'rl_control_mode': args.rl_control_mode,
        'action_mode': args.action_mode,
        'action_speed_scale_min': float(getattr(args, 'action_speed_scale_min', 0.25)),
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
        'cte_clip_range': float(getattr(args, 'cte_clip_range', 3.0)),
        'route_progress_cte_gate_start': float(getattr(args, 'route_progress_cte_gate_start', 0.0)),
        'route_progress_cte_gate_width': float(getattr(args, 'route_progress_cte_gate_width', 0.0)),
        'route_progress_cte_gate_floor': float(getattr(args, 'route_progress_cte_gate_floor', 0.25)),
        'random_encounter_route_priority': bool(getattr(args, 'random_encounter_route_priority', False)),
        'cruise_speed': float(args.cruise_speed),
        'max_angular_velocity': float(args.max_angular_velocity),
        'heading_omega_deadband': float(args.heading_omega_deadband),
        'heading_omega_reference': float(args.heading_omega_reference),
        'angular_authority_power': float(args.angular_authority_power),
        'angular_accel_limit': float(args.angular_accel_limit),
        'angular_decel_limit': float(args.angular_decel_limit),
        'conflict_turn_relief': float(args.conflict_turn_relief),
        'angular_authority_floor': float(args.angular_authority_floor),
        'min_forward_speed_floor': float(args.min_forward_speed_floor),
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
        'squash_actions': bool(getattr(args, 'squash_actions', False)),
        'min_forward_speed': float(getattr(args, 'min_forward_speed', 0.0)),
        'normalize_observations': bool(getattr(args, 'normalize_observations', False)),
        'freeze_observation_normalizer': bool(getattr(args, 'freeze_observation_normalizer', False)),
        'neighbor_attention': bool(getattr(args, 'neighbor_attention', False)),
        'attention_embed_dim': int(getattr(args, 'attention_embed_dim', 32)),
        'attention_num_heads': int(getattr(args, 'attention_num_heads', 1)),
        'attention_encounter_residual': bool(getattr(args, 'attention_encounter_residual', False)),
        'attention_scenario_residual': bool(getattr(args, 'attention_scenario_residual', False)),
        'attention_scenario_head': bool(getattr(args, 'attention_scenario_head', False)),
        'attention_scenario_trunk': bool(getattr(args, 'attention_scenario_trunk', False)),
        'crossing_imitation_weight': float(getattr(args, 'crossing_imitation_weight', 0.0)),
        'crossing_imitation_weight_end': (
            float(getattr(args, 'crossing_imitation_weight_end'))
            if getattr(args, 'crossing_imitation_weight_end', None) is not None
            else None
        ),
        'crossing_imitation_pretrain_epochs': int(getattr(args, 'crossing_imitation_pretrain_epochs', 0)),
        'crossing_imitation_clear_speed': float(getattr(args, 'crossing_imitation_clear_speed', 0.32)),
        'crossing_imitation_middle_speed': float(getattr(args, 'crossing_imitation_middle_speed', 0.10)),
        'crossing_imitation_yield_speed': float(getattr(args, 'crossing_imitation_yield_speed', 0.0)),
        'crossing_imitation_clear_omega': float(getattr(args, 'crossing_imitation_clear_omega', -0.10)),
        'crossing_imitation_middle_omega': float(getattr(args, 'crossing_imitation_middle_omega', -0.14)),
        'crossing_imitation_yield_omega': float(getattr(args, 'crossing_imitation_yield_omega', -0.18)),
        'overtaking_imitation_weight': float(getattr(args, 'overtaking_imitation_weight', 0.0)),
        'overtaking_imitation_weight_end': (
            float(getattr(args, 'overtaking_imitation_weight_end'))
            if getattr(args, 'overtaking_imitation_weight_end', None) is not None
            else None
        ),
        'overtaking_imitation_max_distance': float(getattr(args, 'overtaking_imitation_max_distance', 13.0)),
        'overtaking_imitation_front_min_speed': float(getattr(args, 'overtaking_imitation_front_min_speed', 0.24)),
        'overtaking_imitation_front_max_omega': float(getattr(args, 'overtaking_imitation_front_max_omega', 0.10)),
        'overtaking_imitation_rear_approach_speed': float(getattr(args, 'overtaking_imitation_rear_approach_speed', 0.12)),
        'overtaking_imitation_rear_close_speed': float(getattr(args, 'overtaking_imitation_rear_close_speed', 0.055)),
        'overtaking_imitation_rear_pass_speed': float(getattr(args, 'overtaking_imitation_rear_pass_speed', 0.22)),
        'overtaking_imitation_close_separation': float(getattr(args, 'overtaking_imitation_close_separation', 2.30)),
        'overtaking_imitation_release_separation': float(getattr(args, 'overtaking_imitation_release_separation', 5.80)),
        'overtaking_imitation_target_starboard_offset': float(getattr(args, 'overtaking_imitation_target_starboard_offset', 1.05)),
        'overtaking_imitation_rear_omega': float(getattr(args, 'overtaking_imitation_rear_omega', 0.26)),
        'overtaking_imitation_omega_weight': float(getattr(args, 'overtaking_imitation_omega_weight', 1.0)),
        'overtaking_imitation_single_return_progress': float(getattr(args, 'overtaking_imitation_single_return_progress', 0.70)),
        'overtaking_imitation_single_return_rel_x': float(getattr(args, 'overtaking_imitation_single_return_rel_x', -0.20)),
        'overtaking_imitation_single_return_min_separation': float(getattr(args, 'overtaking_imitation_single_return_min_separation', 2.70)),
        'overtaking_imitation_single_return_cte_start': float(getattr(args, 'overtaking_imitation_single_return_cte_start', 0.70)),
        'overtaking_imitation_single_return_cte_full': float(getattr(args, 'overtaking_imitation_single_return_cte_full', 3.00)),
        'overtaking_imitation_single_return_speed': float(getattr(args, 'overtaking_imitation_single_return_speed', 0.32)),
        'overtaking_imitation_single_return_omega': float(getattr(args, 'overtaking_imitation_single_return_omega', 0.30)),
        'overtaking_imitation_single_finish_distance': float(getattr(args, 'overtaking_imitation_single_finish_distance', 3.50)),
        'overtaking_imitation_single_finish_speed': float(getattr(args, 'overtaking_imitation_single_finish_speed', 0.34)),
        'overtaking_imitation_single_finish_max_omega': float(getattr(args, 'overtaking_imitation_single_finish_max_omega', 0.10)),
        'near_goal_finish_weight': float(getattr(args, 'near_goal_finish_weight', 0.0)),
        'near_goal_finish_weight_end': (
            float(getattr(args, 'near_goal_finish_weight_end'))
            if getattr(args, 'near_goal_finish_weight_end', None) is not None
            else None
        ),
        'near_goal_finish_distance': float(getattr(args, 'near_goal_finish_distance', 2.2)),
        'near_goal_finish_goal_tolerance': float(getattr(args, 'near_goal_finish_goal_tolerance', 0.8)),
        'near_goal_finish_phase_min': float(getattr(args, 'near_goal_finish_phase_min', 0.0)),
        'near_goal_finish_target_speed': float(getattr(args, 'near_goal_finish_target_speed', 0.18)),
        'near_goal_finish_max_omega': float(getattr(args, 'near_goal_finish_max_omega', 0.18)),
        'near_goal_finish_omega_weight': float(getattr(args, 'near_goal_finish_omega_weight', 0.35)),
        'near_goal_finish_crossing_only': bool(getattr(args, 'near_goal_finish_crossing_only', False)),
        'near_goal_finish_exclude_overtaking': bool(getattr(args, 'near_goal_finish_exclude_overtaking', False)),
        'lagging_finish_weight': float(getattr(args, 'lagging_finish_weight', 0.0)),
        'lagging_finish_weight_end': (
            float(getattr(args, 'lagging_finish_weight_end'))
            if getattr(args, 'lagging_finish_weight_end', None) is not None
            else None
        ),
        'lagging_finish_distance': float(getattr(args, 'lagging_finish_distance', 4.0)),
        'lagging_finish_goal_tolerance': float(getattr(args, 'lagging_finish_goal_tolerance', 0.8)),
        'lagging_finish_phase_min': float(getattr(args, 'lagging_finish_phase_min', 0.0)),
        'lagging_finish_target_speed': float(getattr(args, 'lagging_finish_target_speed', 0.14)),
        'lagging_finish_min_speed_scale': float(getattr(args, 'lagging_finish_min_speed_scale', 0.25)),
        'lagging_finish_raw_target': bool(getattr(args, 'lagging_finish_raw_target', False)),
        'lagging_finish_max_omega': float(getattr(args, 'lagging_finish_max_omega', 0.10)),
        'lagging_finish_omega_weight': float(getattr(args, 'lagging_finish_omega_weight', 0.25)),
        'lagging_finish_hold_omega_only': bool(getattr(args, 'lagging_finish_hold_omega_only', False)),
        'lagging_finish_hold_weight': float(getattr(args, 'lagging_finish_hold_weight', 1.0)),
        'lagging_finish_min_team_completion': float(getattr(args, 'lagging_finish_min_team_completion', 0.60)),
        'lagging_finish_max_team_completion': float(getattr(args, 'lagging_finish_max_team_completion', 0.999)),
        'lagging_finish_near_team_tolerance': float(getattr(args, 'lagging_finish_near_team_tolerance', 0.0)),
        'lagging_finish_min_team_separation': float(getattr(args, 'lagging_finish_min_team_separation', 0.0)),
        'lagging_finish_safe_team_separation': float(getattr(args, 'lagging_finish_safe_team_separation', 0.0)),
        'lagging_finish_safe_team_separation_power': float(getattr(args, 'lagging_finish_safe_team_separation_power', 1.0)),
        'lagging_finish_crossing_only': bool(getattr(args, 'lagging_finish_crossing_only', False)),
        'lagging_finish_hold_reached': bool(getattr(args, 'lagging_finish_hold_reached', False)),
        'team_safety_brake_weight': float(getattr(args, 'team_safety_brake_weight', 0.0)),
        'team_safety_brake_weight_end': (
            float(getattr(args, 'team_safety_brake_weight_end'))
            if getattr(args, 'team_safety_brake_weight_end', None) is not None
            else None
        ),
        'team_safety_brake_goal_tolerance': float(getattr(args, 'team_safety_brake_goal_tolerance', 0.8)),
        'team_safety_brake_max_distance': float(getattr(args, 'team_safety_brake_max_distance', 12.0)),
        'team_safety_brake_phase_min': float(getattr(args, 'team_safety_brake_phase_min', -1.0)),
        'team_safety_brake_min_team_completion': float(getattr(args, 'team_safety_brake_min_team_completion', 0.30)),
        'team_safety_brake_max_team_completion': float(getattr(args, 'team_safety_brake_max_team_completion', 0.999)),
        'team_safety_brake_near_team_tolerance': float(getattr(args, 'team_safety_brake_near_team_tolerance', 0.0)),
        'team_safety_brake_safe_separation': float(getattr(args, 'team_safety_brake_safe_separation', 1.2)),
        'team_safety_brake_release_separation': float(getattr(args, 'team_safety_brake_release_separation', 2.4)),
        'team_safety_brake_target_speed': float(getattr(args, 'team_safety_brake_target_speed', 0.0)),
        'team_safety_brake_omega_weight': float(getattr(args, 'team_safety_brake_omega_weight', 0.0)),
        'team_safety_brake_target_omega': float(getattr(args, 'team_safety_brake_target_omega', 0.0)),
        'team_safety_brake_turn_mode': str(getattr(args, 'team_safety_brake_turn_mode', 'away')),
        'team_safety_brake_head_on_starboard_threshold': float(getattr(args, 'team_safety_brake_head_on_starboard_threshold', 0.0)),
        'team_safety_brake_require_neighbor': bool(getattr(args, 'team_safety_brake_require_neighbor', False)),
        'team_safety_brake_local_danger': bool(getattr(args, 'team_safety_brake_local_danger', False)),
        'team_safety_brake_power': float(getattr(args, 'team_safety_brake_power', 1.0)),
        'team_safety_brake_cpa_danger': bool(getattr(args, 'team_safety_brake_cpa_danger', False)),
        'team_safety_brake_cpa_lookahead_distance': float(getattr(args, 'team_safety_brake_cpa_lookahead_distance', 0.0)),
        'team_safety_brake_cpa_time_horizon': float(getattr(args, 'team_safety_brake_cpa_time_horizon', 0.0)),
        'team_safety_brake_cpa_dcpa_target': float(getattr(args, 'team_safety_brake_cpa_dcpa_target', 0.0)),
        'team_safety_brake_cpa_closing_speed_min': float(getattr(args, 'team_safety_brake_cpa_closing_speed_min', 0.02)),
        'team_safety_brake_crossing_only': bool(getattr(args, 'team_safety_brake_crossing_only', False)),
        'team_safety_brake_random_only': bool(getattr(args, 'team_safety_brake_random_only', False)),
        'team_safety_brake_random_yield_agents_only': bool(getattr(args, 'team_safety_brake_random_yield_agents_only', False)),
        'team_safety_brake_random_yield_agent_weight': float(getattr(args, 'team_safety_brake_random_yield_agent_weight', 1.0)),
        'team_safety_brake_random_standon_agent_weight': float(getattr(args, 'team_safety_brake_random_standon_agent_weight', 0.0)),
        'random_deconflict_weight': float(getattr(args, 'random_deconflict_weight', 0.0)),
        'random_deconflict_weight_end': (
            float(getattr(args, 'random_deconflict_weight_end'))
            if getattr(args, 'random_deconflict_weight_end', None) is not None
            else None
        ),
        'random_deconflict_pretrain_epochs': int(getattr(args, 'random_deconflict_pretrain_epochs', 0)),
        'random_deconflict_pretrain_learning_rate': float(getattr(args, 'random_deconflict_pretrain_learning_rate', 0.0)),
        'random_deconflict_pretrain_max_grad_norm': float(getattr(args, 'random_deconflict_pretrain_max_grad_norm', 0.0)),
        'random_deconflict_role_mode': str(getattr(args, 'random_deconflict_role_mode', 'agent-index')),
        'random_deconflict_priority_yield_threshold': float(getattr(args, 'random_deconflict_priority_yield_threshold', -0.10)),
        'random_deconflict_priority_delta_yield_threshold': float(getattr(args, 'random_deconflict_priority_delta_yield_threshold', -0.01)),
        'random_deconflict_route_eta_yield_threshold': float(getattr(args, 'random_deconflict_route_eta_yield_threshold', 0.02)),
        'random_deconflict_goal_tolerance': float(getattr(args, 'random_deconflict_goal_tolerance', 0.8)),
        'random_deconflict_max_distance': float(getattr(args, 'random_deconflict_max_distance', 13.0)),
        'random_deconflict_lookahead_distance': float(getattr(args, 'random_deconflict_lookahead_distance', 10.8)),
        'random_deconflict_time_horizon': float(getattr(args, 'random_deconflict_time_horizon', 30.0)),
        'random_deconflict_dcpa_target': float(getattr(args, 'random_deconflict_dcpa_target', 1.45)),
        'random_deconflict_closing_speed_min': float(getattr(args, 'random_deconflict_closing_speed_min', 0.004)),
        'random_deconflict_local_danger': bool(getattr(args, 'random_deconflict_local_danger', False)),
        'random_deconflict_safe_separation': float(getattr(args, 'random_deconflict_safe_separation', 1.30)),
        'random_deconflict_release_separation': float(getattr(args, 'random_deconflict_release_separation', 2.40)),
        'random_deconflict_critical_separation': float(getattr(args, 'random_deconflict_critical_separation', 0.0)),
        'random_deconflict_power': float(getattr(args, 'random_deconflict_power', 1.0)),
        'random_deconflict_standon_speed': float(getattr(args, 'random_deconflict_standon_speed', 0.30)),
        'random_deconflict_standon_close_separation': float(getattr(args, 'random_deconflict_standon_close_separation', 0.0)),
        'random_deconflict_standon_close_speed': float(getattr(args, 'random_deconflict_standon_close_speed', 0.08)),
        'random_deconflict_yield_speed': float(getattr(args, 'random_deconflict_yield_speed', 0.04)),
        'random_deconflict_yield_danger_scale': float(getattr(args, 'random_deconflict_yield_danger_scale', 1.0)),
        'random_deconflict_standon_omega': float(getattr(args, 'random_deconflict_standon_omega', 0.12)),
        'random_deconflict_yield_omega': float(getattr(args, 'random_deconflict_yield_omega', 0.50)),
        'random_deconflict_turn_mode': str(getattr(args, 'random_deconflict_turn_mode', 'starboard')),
        'random_deconflict_omega_weight': float(getattr(args, 'random_deconflict_omega_weight', 1.0)),
        'random_deconflict_standon_weight': float(getattr(args, 'random_deconflict_standon_weight', 0.35)),
        'random_deconflict_yield_weight': float(getattr(args, 'random_deconflict_yield_weight', 1.0)),
        'random_role_balance_weight': float(getattr(args, 'random_role_balance_weight', 0.0)),
        'random_role_balance_weight_end': (
            float(getattr(args, 'random_role_balance_weight_end'))
            if getattr(args, 'random_role_balance_weight_end', None) is not None
            else None
        ),
        'random_role_balance_pretrain_epochs': int(getattr(args, 'random_role_balance_pretrain_epochs', 0)),
        'random_role_balance_pretrain_learning_rate': float(getattr(args, 'random_role_balance_pretrain_learning_rate', 0.0)),
        'random_role_balance_pretrain_max_grad_norm': float(getattr(args, 'random_role_balance_pretrain_max_grad_norm', 0.0)),
        'random_role_balance_min_danger': float(getattr(args, 'random_role_balance_min_danger', 0.05)),
        'random_role_balance_standon_min_speed': float(getattr(args, 'random_role_balance_standon_min_speed', 0.28)),
        'random_role_balance_standon_close_separation': float(getattr(args, 'random_role_balance_standon_close_separation', 0.0)),
        'random_role_balance_yield_max_speed': float(getattr(args, 'random_role_balance_yield_max_speed', 0.10)),
        'random_role_balance_yield_min_starboard_omega': float(getattr(args, 'random_role_balance_yield_min_starboard_omega', 0.22)),
        'random_role_balance_standon_weight': float(getattr(args, 'random_role_balance_standon_weight', 1.0)),
        'random_role_balance_yield_weight': float(getattr(args, 'random_role_balance_yield_weight', 1.0)),
        'random_role_balance_omega_weight': float(getattr(args, 'random_role_balance_omega_weight', 0.6)),
        'random_pairwise_role_guard_weight': float(getattr(args, 'random_pairwise_role_guard_weight', 0.0)),
        'random_pairwise_role_guard_weight_end': (
            float(getattr(args, 'random_pairwise_role_guard_weight_end'))
            if getattr(args, 'random_pairwise_role_guard_weight_end', None) is not None
            else None
        ),
        'random_pairwise_role_guard_pretrain_epochs': int(getattr(args, 'random_pairwise_role_guard_pretrain_epochs', 0)),
        'random_pairwise_role_guard_pretrain_learning_rate': float(getattr(args, 'random_pairwise_role_guard_pretrain_learning_rate', 0.0)),
        'random_pairwise_role_guard_pretrain_max_grad_norm': float(getattr(args, 'random_pairwise_role_guard_pretrain_max_grad_norm', 0.0)),
        'random_pairwise_role_guard_safe_separation': float(getattr(args, 'random_pairwise_role_guard_safe_separation', 0.82)),
        'random_pairwise_role_guard_release_separation': float(getattr(args, 'random_pairwise_role_guard_release_separation', 1.45)),
        'random_pairwise_role_guard_min_danger': float(getattr(args, 'random_pairwise_role_guard_min_danger', 0.10)),
        'random_pairwise_role_guard_yield_speed': float(getattr(args, 'random_pairwise_role_guard_yield_speed', 0.015)),
        'random_pairwise_role_guard_standon_close_speed': float(getattr(args, 'random_pairwise_role_guard_standon_close_speed', 0.12)),
        'random_pairwise_role_guard_yield_omega': float(getattr(args, 'random_pairwise_role_guard_yield_omega', 0.42)),
        'random_pairwise_role_guard_standon_omega': float(getattr(args, 'random_pairwise_role_guard_standon_omega', 0.14)),
        'random_pairwise_role_guard_linear_weight': float(getattr(args, 'random_pairwise_role_guard_linear_weight', 1.0)),
        'random_pairwise_role_guard_omega_weight': float(getattr(args, 'random_pairwise_role_guard_omega_weight', 2.0)),
        'random_pairwise_role_guard_yield_weight': float(getattr(args, 'random_pairwise_role_guard_yield_weight', 2.0)),
        'random_pairwise_role_guard_standon_weight': float(getattr(args, 'random_pairwise_role_guard_standon_weight', 0.55)),
        'random_safe_finish_weight': float(getattr(args, 'random_safe_finish_weight', 0.0)),
        'random_safe_finish_weight_end': (
            float(getattr(args, 'random_safe_finish_weight_end'))
            if getattr(args, 'random_safe_finish_weight_end', None) is not None
            else None
        ),
        'random_safe_finish_goal_tolerance': float(getattr(args, 'random_safe_finish_goal_tolerance', 0.8)),
        'random_safe_finish_max_distance': float(getattr(args, 'random_safe_finish_max_distance', 13.0)),
        'random_safe_finish_phase_min': float(getattr(args, 'random_safe_finish_phase_min', -1.0)),
        'random_safe_finish_min_team_separation': float(getattr(args, 'random_safe_finish_min_team_separation', 2.7)),
        'random_safe_finish_full_team_separation': float(getattr(args, 'random_safe_finish_full_team_separation', 4.0)),
        'random_safe_finish_min_neighbor_separation': float(getattr(args, 'random_safe_finish_min_neighbor_separation', 2.7)),
        'random_safe_finish_local_separation_scale': bool(getattr(args, 'random_safe_finish_local_separation_scale', False)),
        'random_safe_finish_max_abs_cte': float(getattr(args, 'random_safe_finish_max_abs_cte', 0.0)),
        'random_safe_finish_yield_release_min_separation': float(getattr(args, 'random_safe_finish_yield_release_min_separation', 0.0)),
        'random_safe_finish_yield_release_max_closing_speed': float(getattr(args, 'random_safe_finish_yield_release_max_closing_speed', 0.02)),
        'random_safe_finish_yield_release_min_route_progress': float(getattr(args, 'random_safe_finish_yield_release_min_route_progress', 0.0)),
        'random_safe_finish_max_cpa_score': float(getattr(args, 'random_safe_finish_max_cpa_score', 0.0)),
        'random_safe_finish_max_local_score': float(getattr(args, 'random_safe_finish_max_local_score', 0.0)),
        'random_safe_finish_target_speed': float(getattr(args, 'random_safe_finish_target_speed', 0.18)),
        'random_safe_finish_min_speed_scale': float(getattr(args, 'random_safe_finish_min_speed_scale', 0.35)),
        'random_safe_finish_hold_distance': float(getattr(args, 'random_safe_finish_hold_distance', 0.0)),
        'random_safe_finish_hold_weight': float(getattr(args, 'random_safe_finish_hold_weight', 1.0)),
        'random_safe_finish_low_priority_threshold': float(getattr(args, 'random_safe_finish_low_priority_threshold', -0.5)),
        'random_safe_finish_low_priority_speed_multiplier': float(getattr(args, 'random_safe_finish_low_priority_speed_multiplier', 1.0)),
        'random_safe_finish_max_omega': float(getattr(args, 'random_safe_finish_max_omega', 0.10)),
        'random_safe_finish_omega_weight': float(getattr(args, 'random_safe_finish_omega_weight', 0.12)),
        'random_goal_hold_weight': float(getattr(args, 'random_goal_hold_weight', 0.0)),
        'random_goal_hold_weight_end': (
            float(getattr(args, 'random_goal_hold_weight_end'))
            if getattr(args, 'random_goal_hold_weight_end', None) is not None
            else None
        ),
        'random_goal_hold_distance': float(getattr(args, 'random_goal_hold_distance', 0.0)),
        'random_goal_hold_phase_min': float(getattr(args, 'random_goal_hold_phase_min', -1.0)),
        'random_goal_hold_target_speed': float(getattr(args, 'random_goal_hold_target_speed', 0.0)),
        'random_goal_hold_target_omega': float(getattr(args, 'random_goal_hold_target_omega', 0.0)),
        'random_goal_hold_omega_weight': float(getattr(args, 'random_goal_hold_omega_weight', 1.0)),
        'random_offroute_finish_weight': float(getattr(args, 'random_offroute_finish_weight', 0.0)),
        'random_offroute_finish_weight_end': (
            float(getattr(args, 'random_offroute_finish_weight_end'))
            if getattr(args, 'random_offroute_finish_weight_end', None) is not None
            else None
        ),
        'random_offroute_finish_goal_tolerance': float(getattr(args, 'random_offroute_finish_goal_tolerance', 0.8)),
        'random_offroute_finish_max_distance': float(getattr(args, 'random_offroute_finish_max_distance', 13.0)),
        'random_offroute_finish_min_distance': float(getattr(args, 'random_offroute_finish_min_distance', 1.60)),
        'random_offroute_finish_route_progress_min': float(getattr(args, 'random_offroute_finish_route_progress_min', 0.82)),
        'random_offroute_finish_min_abs_cte': float(getattr(args, 'random_offroute_finish_min_abs_cte', 1.20)),
        'random_offroute_finish_full_abs_cte': float(getattr(args, 'random_offroute_finish_full_abs_cte', 3.00)),
        'random_offroute_finish_phase_min': float(getattr(args, 'random_offroute_finish_phase_min', -1.0)),
        'random_offroute_finish_min_team_separation': float(getattr(args, 'random_offroute_finish_min_team_separation', 2.75)),
        'random_offroute_finish_min_neighbor_separation': float(getattr(args, 'random_offroute_finish_min_neighbor_separation', 2.90)),
        'random_offroute_finish_max_cpa_score': float(getattr(args, 'random_offroute_finish_max_cpa_score', 0.0)),
        'random_offroute_finish_max_local_score': float(getattr(args, 'random_offroute_finish_max_local_score', 0.0)),
        'random_offroute_finish_allow_threat_overlap': bool(getattr(args, 'random_offroute_finish_allow_threat_overlap', False)),
        'random_offroute_finish_target_speed': float(getattr(args, 'random_offroute_finish_target_speed', 0.14)),
        'random_offroute_finish_min_speed': float(getattr(args, 'random_offroute_finish_min_speed', 0.06)),
        'random_offroute_finish_max_omega': float(getattr(args, 'random_offroute_finish_max_omega', 0.18)),
        'random_offroute_finish_omega_reference': float(getattr(args, 'random_offroute_finish_omega_reference', 0.65)),
        'random_offroute_finish_omega_weight': float(getattr(args, 'random_offroute_finish_omega_weight', 0.55)),
        'random_cte_recovery_weight': float(getattr(args, 'random_cte_recovery_weight', 0.0)),
        'random_cte_recovery_weight_end': (
            float(getattr(args, 'random_cte_recovery_weight_end'))
            if getattr(args, 'random_cte_recovery_weight_end', None) is not None
            else None
        ),
        'random_cte_recovery_goal_tolerance': float(getattr(args, 'random_cte_recovery_goal_tolerance', 0.8)),
        'random_cte_recovery_max_distance': float(getattr(args, 'random_cte_recovery_max_distance', 13.0)),
        'random_cte_recovery_min_abs_cte': float(getattr(args, 'random_cte_recovery_min_abs_cte', 1.10)),
        'random_cte_recovery_full_abs_cte': float(getattr(args, 'random_cte_recovery_full_abs_cte', 3.00)),
        'random_cte_source': str(getattr(args, 'random_cte_source', 'clipped')),
        'random_cte_recovery_min_neighbor_separation': float(getattr(args, 'random_cte_recovery_min_neighbor_separation', 0.85)),
        'random_cte_recovery_agent_indices': [int(index) for index in (getattr(args, 'random_cte_recovery_agent_indices', None) or [])],
        'random_cte_recovery_all_agents': bool(getattr(args, 'random_cte_recovery_all_agents', False)),
        'random_cte_recovery_allow_threat_overlap': bool(getattr(args, 'random_cte_recovery_allow_threat_overlap', False)),
        'random_cte_recovery_target_speed': float(getattr(args, 'random_cte_recovery_target_speed', 0.18)),
        'random_cte_recovery_min_speed': float(getattr(args, 'random_cte_recovery_min_speed', 0.07)),
        'random_cte_recovery_speed_cte_slowdown': float(getattr(args, 'random_cte_recovery_speed_cte_slowdown', 1.0)),
        'random_cte_recovery_speed_heading_gate': float(getattr(args, 'random_cte_recovery_speed_heading_gate', 0.0)),
        'random_cte_recovery_max_omega': float(getattr(args, 'random_cte_recovery_max_omega', 0.30)),
        'random_cte_recovery_omega_reference': float(getattr(args, 'random_cte_recovery_omega_reference', 0.55)),
        'random_cte_recovery_linear_weight': float(getattr(args, 'random_cte_recovery_linear_weight', 1.0)),
        'random_cte_recovery_omega_weight': float(getattr(args, 'random_cte_recovery_omega_weight', 1.0)),
        'random_cte_recovery_omega_mode': str(getattr(args, 'random_cte_recovery_omega_mode', 'goal-heading')),
        'random_cte_recovery_cte_lookahead': float(getattr(args, 'random_cte_recovery_cte_lookahead', 4.0)),
        'random_cte_recovery_cte_heading_scale': float(getattr(args, 'random_cte_recovery_cte_heading_scale', 1.0)),
        'random_recovery_pretrain_epochs': int(getattr(args, 'random_recovery_pretrain_epochs', 0)),
        'random_recovery_pretrain_learning_rate': float(getattr(args, 'random_recovery_pretrain_learning_rate', 0.0)),
        'random_recovery_pretrain_max_grad_norm': float(getattr(args, 'random_recovery_pretrain_max_grad_norm', 0.0)),
        'random_recovery_pretrain_offroute_scale': float(getattr(args, 'random_recovery_pretrain_offroute_scale', 1.0)),
        'random_recovery_pretrain_cte_scale': float(getattr(args, 'random_recovery_pretrain_cte_scale', 1.0)),
        'random_recovery_pretrain_clear_scale': float(getattr(args, 'random_recovery_pretrain_clear_scale', 1.0)),
        'random_recovery_pretrain_late_lagging_scale': float(getattr(args, 'random_recovery_pretrain_late_lagging_scale', 1.0)),
        'random_recovery_safety_gate_scale': float(getattr(args, 'random_recovery_safety_gate_scale', 1.0)),
        'random_recovery_safety_gate_mode': str(getattr(args, 'random_recovery_safety_gate_mode', 'batch')),
        'goal_heading_omega_sign': _goal_heading_omega_sign(args),
        'random_clear_ahead_weight': float(getattr(args, 'random_clear_ahead_weight', 0.0)),
        'random_clear_ahead_weight_end': (
            float(getattr(args, 'random_clear_ahead_weight_end'))
            if getattr(args, 'random_clear_ahead_weight_end', None) is not None
            else None
        ),
        'random_clear_ahead_scenarios': tuple(str(value) for value in (getattr(args, 'random_clear_ahead_scenarios', None) or ())),
        'random_clear_ahead_distance': float(getattr(args, 'random_clear_ahead_distance', 5.0)),
        'random_clear_ahead_bearing_deg': float(getattr(args, 'random_clear_ahead_bearing_deg', 35.0)),
        'random_clear_ahead_cone_mode': str(getattr(args, 'random_clear_ahead_cone_mode', 'goal')),
        'random_clear_ahead_goal_tolerance': float(getattr(args, 'random_clear_ahead_goal_tolerance', 1.0)),
        'random_clear_ahead_max_distance': float(getattr(args, 'random_clear_ahead_max_distance', 13.0)),
        'random_clear_ahead_min_route_progress': float(getattr(args, 'random_clear_ahead_min_route_progress', 0.0)),
        'random_clear_ahead_max_route_progress': float(getattr(args, 'random_clear_ahead_max_route_progress', 1.0)),
        'random_clear_ahead_min_abs_cte': float(getattr(args, 'random_clear_ahead_min_abs_cte', 0.0)),
        'random_clear_ahead_max_abs_cte': float(getattr(args, 'random_clear_ahead_max_abs_cte', 0.0)),
        'random_clear_ahead_min_neighbor_separation': float(getattr(args, 'random_clear_ahead_min_neighbor_separation', 0.0)),
        'random_clear_ahead_agent_indices': [int(index) for index in (getattr(args, 'random_clear_ahead_agent_indices', None) or [])],
        'random_clear_ahead_max_cpa_score': float(getattr(args, 'random_clear_ahead_max_cpa_score', 0.0)),
        'random_clear_ahead_max_local_score': float(getattr(args, 'random_clear_ahead_max_local_score', 0.0)),
        'random_clear_ahead_exclude_deconflict': bool(getattr(args, 'random_clear_ahead_exclude_deconflict', False)),
        'random_clear_ahead_target_source': str(getattr(args, 'random_clear_ahead_target_source', 'raw')),
        'random_clear_ahead_target_speed': float(getattr(args, 'random_clear_ahead_target_speed', 0.30)),
        'random_clear_ahead_min_speed': float(getattr(args, 'random_clear_ahead_min_speed', 0.18)),
        'random_clear_ahead_speed_cte_slowdown': float(getattr(args, 'random_clear_ahead_speed_cte_slowdown', 0.0)),
        'random_clear_ahead_speed_cte_start': float(getattr(args, 'random_clear_ahead_speed_cte_start', 0.0)),
        'random_clear_ahead_speed_cte_full': float(getattr(args, 'random_clear_ahead_speed_cte_full', 0.0)),
        'random_clear_ahead_max_omega': float(getattr(args, 'random_clear_ahead_max_omega', 0.18)),
        'random_clear_ahead_omega_reference': float(getattr(args, 'random_clear_ahead_omega_reference', 0.55)),
        'random_clear_ahead_omega_weight': float(getattr(args, 'random_clear_ahead_omega_weight', 1.0)),
        'random_close_formation_escape_weight': float(getattr(args, 'random_close_formation_escape_weight', 0.0)),
        'random_close_formation_escape_weight_end': (
            float(getattr(args, 'random_close_formation_escape_weight_end'))
            if getattr(args, 'random_close_formation_escape_weight_end', None) is not None
            else None
        ),
        'random_close_formation_escape_scenarios': tuple(str(value) for value in (getattr(args, 'random_close_formation_escape_scenarios', None) or ())),
        'random_close_formation_escape_min_sep': float(getattr(args, 'random_close_formation_escape_min_sep', 1.40)),
        'random_close_formation_escape_max_sep': float(getattr(args, 'random_close_formation_escape_max_sep', 2.40)),
        'random_close_formation_escape_max_self_speed': float(getattr(args, 'random_close_formation_escape_max_self_speed', 0.22)),
        'random_close_formation_escape_goal_tolerance': float(getattr(args, 'random_close_formation_escape_goal_tolerance', 1.0)),
        'random_close_formation_escape_max_distance': float(getattr(args, 'random_close_formation_escape_max_distance', 20.0)),
        'random_close_formation_escape_target_speed': float(getattr(args, 'random_close_formation_escape_target_speed', 0.40)),
        'random_close_formation_escape_min_speed': float(getattr(args, 'random_close_formation_escape_min_speed', 0.20)),
        'random_close_formation_escape_max_omega': float(getattr(args, 'random_close_formation_escape_max_omega', 0.45)),
        'random_close_formation_escape_omega_weight': float(getattr(args, 'random_close_formation_escape_omega_weight', 1.6)),
        'random_close_formation_escape_agent_indices': [int(index) for index in (getattr(args, 'random_close_formation_escape_agent_indices', None) or [])],
        'random_close_formation_escape_require_closing': bool(getattr(args, 'random_close_formation_escape_require_closing', False)),
        'random_late_lagging_weight': float(getattr(args, 'random_late_lagging_weight', 0.0)),
        'random_late_lagging_weight_end': (
            float(getattr(args, 'random_late_lagging_weight_end'))
            if getattr(args, 'random_late_lagging_weight_end', None) is not None
            else None
        ),
        'random_late_lagging_goal_tolerance': float(getattr(args, 'random_late_lagging_goal_tolerance', 1.0)),
        'random_late_lagging_max_distance': float(getattr(args, 'random_late_lagging_max_distance', 80.0)),
        'random_late_lagging_min_distance': float(getattr(args, 'random_late_lagging_min_distance', 2.0)),
        'random_late_lagging_max_self_progress': float(getattr(args, 'random_late_lagging_max_self_progress', 0.25)),
        'random_late_lagging_min_team_progress': float(getattr(args, 'random_late_lagging_min_team_progress', 0.75)),
        'random_late_lagging_min_progress_gap': float(getattr(args, 'random_late_lagging_min_progress_gap', 0.50)),
        'random_late_lagging_agent_indices': [int(index) for index in (getattr(args, 'random_late_lagging_agent_indices', None) or [])],
        'random_late_lagging_min_finished_teammates': int(getattr(args, 'random_late_lagging_min_finished_teammates', 0)),
        'random_late_lagging_finished_progress': float(getattr(args, 'random_late_lagging_finished_progress', 0.98)),
        'random_late_lagging_phase_min': float(getattr(args, 'random_late_lagging_phase_min', -1.0)),
        'random_late_lagging_min_team_separation': float(getattr(args, 'random_late_lagging_min_team_separation', 4.8)),
        'random_late_lagging_min_neighbor_separation': float(getattr(args, 'random_late_lagging_min_neighbor_separation', 4.8)),
        'random_late_lagging_max_cpa_score': float(getattr(args, 'random_late_lagging_max_cpa_score', 0.0)),
        'random_late_lagging_max_local_score': float(getattr(args, 'random_late_lagging_max_local_score', 0.0)),
        'random_late_lagging_allow_threat_overlap': bool(getattr(args, 'random_late_lagging_allow_threat_overlap', False)),
        'random_late_lagging_target_speed': float(getattr(args, 'random_late_lagging_target_speed', 0.16)),
        'random_late_lagging_min_speed': float(getattr(args, 'random_late_lagging_min_speed', 0.06)),
        'random_late_lagging_max_omega': float(getattr(args, 'random_late_lagging_max_omega', 0.34)),
        'random_late_lagging_omega_reference': float(getattr(args, 'random_late_lagging_omega_reference', 0.80)),
        'random_late_lagging_omega_weight': float(getattr(args, 'random_late_lagging_omega_weight', 2.60)),
        'random_late_lagging_target_source': str(getattr(args, 'random_late_lagging_target_source', 'goal')),
        'policy_anchor_weight': float(getattr(args, 'policy_anchor_weight', 0.0)),
        'policy_anchor_weight_end': (
            float(getattr(args, 'policy_anchor_weight_end'))
            if getattr(args, 'policy_anchor_weight_end', None) is not None
            else None
        ),
        'policy_anchor_crossing_only': bool(getattr(args, 'policy_anchor_crossing_only', False)),
        'policy_anchor_exclude_lagging_finish': bool(getattr(args, 'policy_anchor_exclude_lagging_finish', False)),
        'policy_anchor_exclude_team_safety_brake': bool(getattr(args, 'policy_anchor_exclude_team_safety_brake', False)),
        'policy_anchor_exclude_random_deconflict': bool(getattr(args, 'policy_anchor_exclude_random_deconflict', False)),
        'policy_anchor_exclude_random_safe_finish': bool(getattr(args, 'policy_anchor_exclude_random_safe_finish', False)),
        'policy_anchor_exclude_random_goal_hold': bool(getattr(args, 'policy_anchor_exclude_random_goal_hold', False)),
        'policy_anchor_exclude_random_offroute_finish': bool(getattr(args, 'policy_anchor_exclude_random_offroute_finish', False)),
        'policy_anchor_exclude_random_cte_recovery': bool(getattr(args, 'policy_anchor_exclude_random_cte_recovery', False)),
        'policy_anchor_exclude_random_clear_ahead': bool(getattr(args, 'policy_anchor_exclude_random_clear_ahead', False)),
        'policy_anchor_exclude_random_late_lagging': bool(getattr(args, 'policy_anchor_exclude_random_late_lagging', False)),
        'separate_actor_critic_grad_clip': bool(getattr(args, 'separate_actor_critic_grad_clip', False)),
        'ego_dim': AgentLocalObservation.ego_feature_size(),
        'neighbor_feature_dim': NEIGHBOR_FEATURE_COUNT,
        'total_timesteps': max(0, args.total_timesteps),
        'completed_timesteps': int(total_steps),
        'update_index': int(update_index),
        'next_checkpoint_step': int(next_checkpoint_step) if next_checkpoint_step is not None else None,
    }
    if optimizer is not None:
        payload['optimizer_state_dict'] = optimizer.state_dict()
    if scaler is not None and hasattr(scaler, 'state_dict'):
        payload['grad_scaler_state_dict'] = scaler.state_dict()
    if obs_normalizer is not None:
        payload['obs_normalizer'] = obs_normalizer.state_dict()
    return payload


def _compute_advantages_and_returns(rewards, values, dones, bootstrap_values, gamma: float, gae_lambda: float):
    if isinstance(rewards, list):
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


def _flatten_rollout_batches(storage_batches: list[np.ndarray], *, dtype=np.float32) -> np.ndarray:
    if not storage_batches:
        return np.asarray([], dtype=dtype)
    return np.concatenate([np.asarray(batch, dtype=dtype) for batch in storage_batches], axis=0)


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

    payload = torch.load(resume_path, map_location='cpu', weights_only=False)
    if not isinstance(payload, dict):
        raise ValueError(f'Resume checkpoint is not a MAPPO scaffold dictionary: {resume_path}')

    required_keys = ('actor_state_dict', 'critic_state_dict', 'actor_log_std')
    missing = [key for key in required_keys if key not in payload]
    if missing:
        raise ValueError(
            f'Resume checkpoint is missing required MAPPO fields {missing}: {resume_path}'
        )
    return payload


def _infer_obs_layout_for_migration(obs_dim: int) -> tuple[int, int, int] | None:
    new_ego_dim = AgentLocalObservation.ego_feature_size()
    # Stage A.2 bumped NEIGHBOR_FEATURE_COUNT 10 -> 15; legacy ckpts may use 10 or 6.
    for ego_dim in (new_ego_dim, 17, 12, 11, 10):
        for neighbor_dim in (NEIGHBOR_FEATURE_COUNT, 10, 6):
            neighbor_width = int(obs_dim) - int(ego_dim) - ENCOUNTER_TYPE_COUNT
            if neighbor_width >= 0 and neighbor_width % int(neighbor_dim) == 0:
                return int(ego_dim), int(neighbor_dim), int(neighbor_width // int(neighbor_dim))
    return None


def _copy_obs_block_weight(torch, old_block, new_block, old_layout, new_layout):
    old_ego_dim, old_neighbor_dim, old_neighbors = old_layout
    new_ego_dim, new_neighbor_dim, new_neighbors = new_layout
    if old_neighbors != new_neighbors:
        return None
    migrated = torch.zeros_like(new_block)
    ego_cols = min(old_ego_dim, new_ego_dim)
    migrated[:, :ego_cols] = old_block[:, :ego_cols]
    old_pos = old_ego_dim
    new_pos = new_ego_dim
    for _ in range(old_neighbors):
        neighbor_cols = min(old_neighbor_dim, new_neighbor_dim)
        migrated[:, new_pos:new_pos + neighbor_cols] = old_block[:, old_pos:old_pos + neighbor_cols]
        old_pos += old_neighbor_dim
        new_pos += new_neighbor_dim
    migrated[:, new_pos:new_pos + ENCOUNTER_TYPE_COUNT] = old_block[:, old_pos:old_pos + ENCOUNTER_TYPE_COUNT]
    return migrated


def _migrate_first_layer_weights(torch, old_state_dict, new_state_dict, old_obs_dim, new_obs_dim, num_obs_blocks, layer_key='0'):
    """Zero-pad the first MLP layer when observation dimension grows.

    Input structure: [obs_block_1, ..., obs_block_N, remaining_features].
    Each obs block grows from old_obs_dim to new_obs_dim.
    For actor: num_obs_blocks=1, no remaining.
    For critic: num_obs_blocks=1+max_agents (self obs + each agent in global state), remaining=fleet_stats.
    """
    weight_key = f'{layer_key}.weight'
    bias_key = f'{layer_key}.bias'
    old_w = old_state_dict[weight_key]  # [hidden, old_input]
    delta = new_obs_dim - old_obs_dim
    new_input_dim = new_state_dict[weight_key].shape[1]
    expected_new = old_w.shape[1] + num_obs_blocks * delta
    if expected_new != new_input_dim:
        raise ValueError(
            f'Weight migration dimension error: old_input={old_w.shape[1]}, '
            f'num_obs_blocks={num_obs_blocks}, delta={delta}, '
            f'expected_new={expected_new}, actual_new={new_input_dim}'
        )
    new_w = torch.zeros(old_w.shape[0], new_input_dim, dtype=old_w.dtype, device=old_w.device)
    old_layout = _infer_obs_layout_for_migration(old_obs_dim)
    new_layout = _infer_obs_layout_for_migration(new_obs_dim)
    old_pos = 0
    new_pos = 0
    for _ in range(num_obs_blocks):
        old_block = old_w[:, old_pos:old_pos + old_obs_dim]
        new_block = new_w[:, new_pos:new_pos + new_obs_dim]
        migrated_block = None
        if old_layout is not None and new_layout is not None:
            migrated_block = _copy_obs_block_weight(torch, old_block, new_block, old_layout, new_layout)
        if migrated_block is None:
            new_w[:, new_pos:new_pos + old_obs_dim] = old_block
        else:
            new_w[:, new_pos:new_pos + new_obs_dim] = migrated_block
        old_pos += old_obs_dim
        new_pos += new_obs_dim
    remaining = old_w.shape[1] - old_pos
    if remaining > 0:
        new_w[:, new_pos:new_pos + remaining] = old_w[:, old_pos:old_pos + remaining]
    migrated = dict(old_state_dict)
    migrated[weight_key] = new_w
    migrated[bias_key] = old_state_dict[bias_key]  # bias unchanged
    return migrated


def _migrate_normalizer_state(old_norm_state, old_obs_dim, new_obs_dim):
    """Extend observation normalizer running stats for new dimensions."""
    if old_norm_state is None:
        return None
    old_mean = np.asarray(old_norm_state.get('mean', []), dtype=np.float64)
    old_var = np.asarray(old_norm_state.get('var', []), dtype=np.float64)
    if old_mean.shape[0] == new_obs_dim:
        return old_norm_state
    if old_mean.shape[0] != old_obs_dim:
        return old_norm_state
    delta = new_obs_dim - old_obs_dim
    old_layout = _infer_obs_layout_for_migration(old_obs_dim)
    new_layout = _infer_obs_layout_for_migration(new_obs_dim)
    if old_layout is not None and new_layout is not None and old_layout[2] == new_layout[2]:
        old_ego_dim, old_neighbor_dim, neighbor_slots = old_layout
        new_ego_dim, new_neighbor_dim, _ = new_layout
        mean_parts = []
        var_parts = []
        ego_cols = min(old_ego_dim, new_ego_dim)
        mean_parts.append(old_mean[:ego_cols])
        var_parts.append(old_var[:ego_cols])
        if new_ego_dim > ego_cols:
            mean_parts.append(np.zeros(new_ego_dim - ego_cols, dtype=np.float64))
            var_parts.append(np.ones(new_ego_dim - ego_cols, dtype=np.float64))
        old_pos = old_ego_dim
        for _ in range(neighbor_slots):
            neighbor_cols = min(old_neighbor_dim, new_neighbor_dim)
            mean_parts.append(old_mean[old_pos:old_pos + neighbor_cols])
            var_parts.append(old_var[old_pos:old_pos + neighbor_cols])
            if new_neighbor_dim > neighbor_cols:
                mean_parts.append(np.zeros(new_neighbor_dim - neighbor_cols, dtype=np.float64))
                var_parts.append(np.ones(new_neighbor_dim - neighbor_cols, dtype=np.float64))
            old_pos += old_neighbor_dim
        mean_parts.append(old_mean[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
        var_parts.append(old_var[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
        migrated = dict(old_norm_state)
        migrated['mean'] = np.concatenate(mean_parts)
        migrated['var'] = np.concatenate(var_parts)
        return migrated
    new_ego_dim = AgentLocalObservation.ego_feature_size()
    old_neighbor_dim = 6
    old_neighbor_width = old_obs_dim - new_ego_dim - ENCOUNTER_TYPE_COUNT
    new_neighbor_width = new_obs_dim - new_ego_dim - ENCOUNTER_TYPE_COUNT
    can_insert_in_neighbors = (
        old_neighbor_width >= 0
        and new_neighbor_width >= 0
        and old_neighbor_width % old_neighbor_dim == 0
    )
    if can_insert_in_neighbors:
        max_neighbors = old_neighbor_width // old_neighbor_dim
        if new_neighbor_width == max_neighbors * NEIGHBOR_FEATURE_COUNT and NEIGHBOR_FEATURE_COUNT > old_neighbor_dim:
            mean_parts = [old_mean[:new_ego_dim]]
            var_parts = [old_var[:new_ego_dim]]
            old_pos = new_ego_dim
            for _ in range(max_neighbors):
                mean_parts.append(old_mean[old_pos:old_pos + old_neighbor_dim])
                var_parts.append(old_var[old_pos:old_pos + old_neighbor_dim])
                mean_parts.append(np.zeros(NEIGHBOR_FEATURE_COUNT - old_neighbor_dim, dtype=np.float64))
                var_parts.append(np.ones(NEIGHBOR_FEATURE_COUNT - old_neighbor_dim, dtype=np.float64))
                old_pos += old_neighbor_dim
            mean_parts.append(old_mean[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
            var_parts.append(old_var[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
            migrated = dict(old_norm_state)
            migrated['mean'] = np.concatenate(mean_parts)
            migrated['var'] = np.concatenate(var_parts)
            return migrated
    old_ego_dim = new_ego_dim - delta
    can_insert_in_ego = (
        delta > 0
        and old_ego_dim > 0
        and old_obs_dim >= old_ego_dim + ENCOUNTER_TYPE_COUNT
        and (old_obs_dim - old_ego_dim - ENCOUNTER_TYPE_COUNT) % 6 == 0
    )
    if can_insert_in_ego:
        new_mean = np.concatenate([
            old_mean[:old_ego_dim],
            np.zeros(delta, dtype=np.float64),
            old_mean[old_ego_dim:],
        ])
        new_var = np.concatenate([
            old_var[:old_ego_dim],
            np.ones(delta, dtype=np.float64),
            old_var[old_ego_dim:],
        ])
    else:
        new_mean = np.concatenate([old_mean, np.zeros(delta, dtype=np.float64)])
        new_var = np.concatenate([old_var, np.ones(delta, dtype=np.float64)])
    migrated = dict(old_norm_state)
    migrated['mean'] = new_mean
    migrated['var'] = new_var
    return migrated


def _remap_scenario_branch_state_dict(actor_sd: dict, checkpoint_scenarios, actor) -> dict:
    """Remap scenario-specific actor branch keys by scenario name, not by index.

    Older single-scenario branch checkpoints store keys such as
    ``scenario_actor_trunks.0.*`` where index 0 refers to that checkpoint's
    only scenario. Loading those keys directly into a full-scenario actor would
    incorrectly assign the branch to whatever scenario is index 0 in the new
    curriculum.  Use the checkpoint's saved scenario names to move matching
    branch tensors to the current actor's scenario index.
    """
    current_scenarios = tuple(str(name) for name in getattr(actor, 'scenario_names', ()) or ())
    saved_scenarios = tuple(str(name) for name in (checkpoint_scenarios or ()))
    if not current_scenarios or not saved_scenarios or current_scenarios == saved_scenarios:
        return actor_sd

    current_index_by_name = {scenario_name: index for index, scenario_name in enumerate(current_scenarios)}
    prefixes = (
        'scenario_residual_heads.',
        'scenario_action_heads.',
        'scenario_actor_trunks.',
    )
    remapped: dict = {}
    moved: list[str] = []
    dropped: list[str] = []

    for key, value in actor_sd.items():
        matched_prefix = None
        for prefix in prefixes:
            if key.startswith(prefix):
                matched_prefix = prefix
                break
        if matched_prefix is None:
            remapped[key] = value
            continue

        remainder = key[len(matched_prefix):]
        index_text, separator, tail = remainder.partition('.')
        if not separator or not index_text.isdigit():
            remapped[key] = value
            continue
        saved_index = int(index_text)
        if saved_index >= len(saved_scenarios):
            dropped.append(key)
            continue
        scenario_name = saved_scenarios[saved_index]
        current_index = current_index_by_name.get(scenario_name)
        if current_index is None:
            dropped.append(key)
            continue
        new_key = f'{matched_prefix}{current_index}.{tail}'
        remapped[new_key] = value
        if new_key != key:
            moved.append(f'{key}->{new_key}')

    if moved or dropped:
        print(
            'Remapped scenario-specific actor branch tensors by scenario name: '
            f'saved={saved_scenarios}, current={current_scenarios}, '
            f'moved={moved}, dropped={dropped}',
            flush=True,
        )
    return remapped


def _load_weights_only(torch, device, actor, critic, actor_log_std, weights_path: Path, obs_normalizer=None, *, max_agents: int = 5, neighbor_attention: bool = False):
    """Load actor/critic/normalizer weights without restoring any config or training state.

    Handles observation dimension mismatch by zero-padding first-layer weights
    when the current model has a larger observation than the checkpoint.
    Also handles flat MLP → attention architecture migration.
    """
    payload = _load_resume_payload(torch, weights_path)

    actor_sd = payload['actor_state_dict']
    critic_sd = payload['critic_state_dict']

    old_is_attention = 'neighbor_attention.query_proj.weight' in actor_sd
    new_is_attention = neighbor_attention
    if old_is_attention and new_is_attention:
        actor_sd = _remap_scenario_branch_state_dict(actor_sd, payload.get('scenarios'), actor)

    if not old_is_attention and new_is_attention:
        # Flat MLP checkpoint → attention architecture: smart migration.
        from usv_rl.neighbor_attention import migrate_flat_actor_to_attention, migrate_flat_critic_to_attention
        migrate_flat_actor_to_attention(actor_sd, actor, torch)
        migrate_flat_critic_to_attention(critic_sd, critic, torch)
        branch_missing = [
            key for key in actor.state_dict().keys()
            if key.startswith(('scenario_action_heads.', 'scenario_actor_trunks.'))
            and key not in actor_sd
        ]
        initialized_branches = _initialize_missing_scenario_branches_from_base(actor, branch_missing)
        print(
            f'Migrated flat MLP weights to attention architecture from {weights_path}',
            flush=True,
        )
        if initialized_branches:
            print(
                'Initialized missing scenario branches from loaded shared actor MLP: '
                f'{initialized_branches}',
                flush=True,
            )
    elif old_is_attention and new_is_attention:
        # Attention → attention: check for obs layout mismatch (ego_dim or neighbor_feature_dim change).
        old_q_shape = actor_sd['neighbor_attention.query_proj.weight'].shape  # [embed, old_ego]
        new_q_shape = actor.state_dict()['neighbor_attention.query_proj.weight'].shape
        old_key_shape = actor_sd.get('neighbor_attention.key_proj.weight', actor_sd['neighbor_attention.value_proj.weight']).shape
        new_key_shape = actor.state_dict()['neighbor_attention.key_proj.weight'].shape
        critic_shape_changed = critic_sd['mlp.0.weight'].shape != critic.state_dict()['mlp.0.weight'].shape
        if old_q_shape != new_q_shape or old_key_shape != new_key_shape or critic_shape_changed:
            from usv_rl.neighbor_attention import (
                _migrate_attention_actor_layout,
                _migrate_attention_critic_layout,
                _EGO_DIM,
                _NEIGHBOR_FEATURE_DIM,
            )
            old_ego = old_q_shape[1]
            new_ego = new_q_shape[1]
            old_neighbor_dim = old_key_shape[1]
            new_neighbor_dim = new_key_shape[1]
            old_max_agents = int(payload.get('max_agents') or max_agents)
            old_max_neighbors = int(payload.get('max_neighbors') or critic.max_neighbors)
            print(
                f'Attention observation layout changed: ego {old_ego} -> {new_ego}, '
                f'neighbor {old_neighbor_dim} -> {new_neighbor_dim}, '
                f'max_agents {old_max_agents} -> {max_agents}, '
                f'max_neighbors {old_max_neighbors} -> {critic.max_neighbors}. '
                'Migrating attention weights.',
                flush=True,
            )
            _migrate_attention_actor_layout(
                actor_sd,
                actor,
                torch,
                old_ego,
                _EGO_DIM,
                old_neighbor_dim,
                _NEIGHBOR_FEATURE_DIM,
            )
            _migrate_attention_critic_layout(
                critic_sd,
                critic,
                torch,
                old_ego,
                _EGO_DIM,
                old_neighbor_dim,
                _NEIGHBOR_FEATURE_DIM,
                max_agents,
                old_max_agents=old_max_agents,
                old_max_neighbors=old_max_neighbors,
            )
            branch_missing = [
                key for key in actor.state_dict().keys()
                if key.startswith(('scenario_action_heads.', 'scenario_actor_trunks.'))
                and key not in actor_sd
            ]
            initialized_branches = _initialize_missing_scenario_branches_from_base(actor, branch_missing)
            if initialized_branches:
                print(
                    'Initialized missing scenario branches from loaded shared actor MLP: '
                    f'{initialized_branches}',
                    flush=True,
                )
        else:
            actor_load_result = actor.load_state_dict(actor_sd, strict=False)
            critic.load_state_dict(critic_sd)
            residual_prefixes = ('encounter_residual.', 'scenario_residual_heads.', 'scenario_action_heads.', 'scenario_actor_trunks.')
            allowed_missing = {
                key for key in actor_load_result.missing_keys
                if key.startswith(residual_prefixes)
            }
            allowed_unexpected = {
                key for key in actor_load_result.unexpected_keys
                if key.startswith(residual_prefixes)
            }
            if len(allowed_missing) != len(actor_load_result.missing_keys) or len(allowed_unexpected) != len(actor_load_result.unexpected_keys):
                raise RuntimeError(
                    'Attention actor checkpoint compatibility failure. '
                    f'Missing keys={actor_load_result.missing_keys}, unexpected keys={actor_load_result.unexpected_keys}'
                )
            if allowed_missing or allowed_unexpected:
                print(
                    'Attention actor loaded with residual-head compatibility mode: '
                    f'missing={sorted(allowed_missing)}, unexpected={sorted(allowed_unexpected)}',
                    flush=True,
                )
            initialized_branches = _initialize_missing_scenario_branches_from_base(actor, allowed_missing)
            if initialized_branches:
                print(
                    'Initialized missing scenario branches from loaded shared actor MLP: '
                    f'{initialized_branches}',
                    flush=True,
                )
    else:
        # Flat → flat (original migration path).
        new_actor_sd = actor.state_dict()
        new_critic_sd = critic.state_dict()

        actor_needs_migration = actor_sd['0.weight'].shape != new_actor_sd['0.weight'].shape
        critic_needs_migration = critic_sd['0.weight'].shape != new_critic_sd['0.weight'].shape

        if actor_needs_migration or critic_needs_migration:
            old_obs_dim = int(actor_sd['0.weight'].shape[1])
            new_obs_dim = int(new_actor_sd['0.weight'].shape[1])
            print(
                f'Observation dimension changed: {old_obs_dim} -> {new_obs_dim}. '
                f'Migrating first-layer weights with zero-padding.',
                flush=True,
            )
            if actor_needs_migration:
                actor_sd = _migrate_first_layer_weights(
                    torch, actor_sd, new_actor_sd,
                    old_obs_dim, new_obs_dim, num_obs_blocks=1,
                )
            if critic_needs_migration:
                critic_sd = _migrate_first_layer_weights(
                    torch, critic_sd, new_critic_sd,
                    old_obs_dim, new_obs_dim, num_obs_blocks=1 + max_agents,
                )

        actor.load_state_dict(actor_sd)
        critic.load_state_dict(critic_sd)

    actor_log_std.data.copy_(
        torch.as_tensor(payload['actor_log_std'], dtype=actor_log_std.dtype, device=device)
    )
    if obs_normalizer is not None:
        saved_norm = payload.get('obs_normalizer')
        if saved_norm is not None:
            # Check if normalizer dimensions changed and need migration.
            old_norm_dim = int(np.asarray(saved_norm.get('mean', [])).shape[0]) if 'mean' in saved_norm else 0
            new_norm_dim = int(obs_normalizer.rms.mean.shape[0])
            if old_norm_dim > 0 and old_norm_dim != new_norm_dim:
                saved_norm = _migrate_normalizer_state(saved_norm, old_norm_dim, new_norm_dim)
                print(f'Migrated observation normalizer: {old_norm_dim} -> {new_norm_dim} dims.', flush=True)
            obs_normalizer.load_state_dict(saved_norm)
    print(f'Loaded weights from {weights_path} (no config/optimizer/training state restored).', flush=True)


def _cli_explicit_args() -> set:
    """Return the set of dest-style arg names explicitly provided on the command line."""
    explicit = set()
    for token in sys.argv[1:]:
        if token.startswith('--'):
            name = token.split('=', 1)[0].lstrip('-').replace('-', '_')
            explicit.add(name)
    return explicit


def _apply_resume_configuration(args, payload: dict, cli_overrides: set | None = None):
    if cli_overrides is None:
        cli_overrides = set()

    agent_namespaces = payload.get('agent_namespaces')
    if agent_namespaces:
        args.num_agents = len(tuple(agent_namespaces))

    hidden_sizes = payload.get('hidden_sizes')
    if hidden_sizes:
        args.hidden_sizes = [int(size) for size in hidden_sizes]

    scenarios = payload.get('scenarios')
    user_specified_scenarios = args.scenarios is not None
    user_specified_scenario_set = args.scenario_set != 'auto'
    if scenarios and not user_specified_scenarios and not user_specified_scenario_set:
        args.scenarios = list(scenarios)
        args.scenario_set = 'auto'

    simple_fields = (
        'max_neighbors',
        'max_agents',
        'rl_control_mode',
        'action_mode',
        'action_speed_scale_min',
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
        'cte_clip_range',
        'route_progress_cte_gate_start',
        'route_progress_cte_gate_width',
        'route_progress_cte_gate_floor',
        'cruise_speed',
        'max_angular_velocity',
        'heading_omega_deadband',
        'heading_omega_reference',
        'angular_authority_power',
        'angular_accel_limit',
        'angular_decel_limit',
        'conflict_turn_relief',
        'angular_authority_floor',
        'min_forward_speed_floor',
        'team_reward_weight',
        'team_progress_weight',
        'team_goal_proximity_weight',
        'team_regression_penalty_weight',
        'team_dispersion_penalty_weight',
        'team_dispersion_margin',
        'coordination_reward_weight',
        'team_completion_bonus',
        'deadlock_penalty_weight',
        'squash_actions',
        'min_forward_speed',
        'attention_encounter_residual',
        'attention_scenario_residual',
        'attention_scenario_head',
        'attention_scenario_trunk',
        'ppo_policy_loss_scale',
        'ppo_value_loss_scale',
        'near_goal_finish_weight',
        'near_goal_finish_weight_end',
        'near_goal_finish_distance',
        'near_goal_finish_goal_tolerance',
        'near_goal_finish_phase_min',
        'near_goal_finish_target_speed',
        'near_goal_finish_max_omega',
        'near_goal_finish_omega_weight',
        'near_goal_finish_crossing_only',
        'near_goal_finish_exclude_overtaking',
        'lagging_finish_weight',
        'lagging_finish_weight_end',
        'lagging_finish_distance',
        'lagging_finish_goal_tolerance',
        'lagging_finish_phase_min',
        'lagging_finish_target_speed',
        'lagging_finish_min_speed_scale',
        'lagging_finish_raw_target',
        'lagging_finish_max_omega',
        'lagging_finish_omega_weight',
        'lagging_finish_hold_omega_only',
        'lagging_finish_hold_weight',
        'lagging_finish_min_team_completion',
        'lagging_finish_max_team_completion',
        'lagging_finish_near_team_tolerance',
        'lagging_finish_min_team_separation',
        'lagging_finish_safe_team_separation',
        'lagging_finish_safe_team_separation_power',
        'lagging_finish_crossing_only',
        'lagging_finish_hold_reached',
        'team_safety_brake_weight',
        'team_safety_brake_weight_end',
        'team_safety_brake_goal_tolerance',
        'team_safety_brake_max_distance',
        'team_safety_brake_phase_min',
        'team_safety_brake_min_team_completion',
        'team_safety_brake_max_team_completion',
        'team_safety_brake_near_team_tolerance',
        'team_safety_brake_safe_separation',
        'team_safety_brake_release_separation',
        'team_safety_brake_target_speed',
        'team_safety_brake_omega_weight',
        'team_safety_brake_target_omega',
        'team_safety_brake_turn_mode',
        'team_safety_brake_head_on_starboard_threshold',
        'team_safety_brake_require_neighbor',
        'team_safety_brake_local_danger',
        'team_safety_brake_power',
        'team_safety_brake_cpa_danger',
        'team_safety_brake_cpa_lookahead_distance',
        'team_safety_brake_cpa_time_horizon',
        'team_safety_brake_cpa_dcpa_target',
        'team_safety_brake_cpa_closing_speed_min',
        'team_safety_brake_crossing_only',
        'team_safety_brake_random_only',
        'team_safety_brake_random_yield_agents_only',
        'team_safety_brake_random_yield_agent_weight',
        'team_safety_brake_random_standon_agent_weight',
        'random_deconflict_weight',
        'random_deconflict_weight_end',
        'random_deconflict_pretrain_epochs',
        'random_deconflict_pretrain_learning_rate',
        'random_deconflict_pretrain_max_grad_norm',
        'random_deconflict_role_mode',
        'random_deconflict_priority_yield_threshold',
        'random_deconflict_priority_delta_yield_threshold',
        'random_deconflict_route_eta_yield_threshold',
        'random_deconflict_goal_tolerance',
        'random_deconflict_max_distance',
        'random_deconflict_lookahead_distance',
        'random_deconflict_time_horizon',
        'random_deconflict_dcpa_target',
        'random_deconflict_closing_speed_min',
        'random_deconflict_local_danger',
        'random_deconflict_safe_separation',
        'random_deconflict_release_separation',
        'random_deconflict_critical_separation',
        'random_deconflict_power',
        'random_deconflict_standon_speed',
        'random_deconflict_standon_close_separation',
        'random_deconflict_standon_close_speed',
        'random_deconflict_yield_speed',
        'random_deconflict_yield_danger_scale',
        'random_deconflict_standon_omega',
        'random_deconflict_yield_omega',
        'random_deconflict_turn_mode',
        'random_deconflict_omega_weight',
        'random_deconflict_standon_weight',
        'random_deconflict_yield_weight',
        'random_role_balance_weight',
        'random_role_balance_weight_end',
        'random_role_balance_pretrain_epochs',
        'random_role_balance_pretrain_learning_rate',
        'random_role_balance_pretrain_max_grad_norm',
        'random_role_balance_min_danger',
        'random_role_balance_standon_min_speed',
        'random_role_balance_standon_close_separation',
        'random_role_balance_yield_max_speed',
        'random_role_balance_yield_min_starboard_omega',
        'random_role_balance_standon_weight',
        'random_role_balance_yield_weight',
        'random_role_balance_omega_weight',
        'random_pairwise_role_guard_weight',
        'random_pairwise_role_guard_weight_end',
        'random_pairwise_role_guard_pretrain_epochs',
        'random_pairwise_role_guard_pretrain_learning_rate',
        'random_pairwise_role_guard_pretrain_max_grad_norm',
        'random_pairwise_role_guard_safe_separation',
        'random_pairwise_role_guard_release_separation',
        'random_pairwise_role_guard_min_danger',
        'random_pairwise_role_guard_yield_speed',
        'random_pairwise_role_guard_standon_close_speed',
        'random_pairwise_role_guard_yield_omega',
        'random_pairwise_role_guard_standon_omega',
        'random_pairwise_role_guard_linear_weight',
        'random_pairwise_role_guard_omega_weight',
        'random_pairwise_role_guard_yield_weight',
        'random_pairwise_role_guard_standon_weight',
        'random_safe_finish_weight',
        'random_safe_finish_weight_end',
        'random_safe_finish_goal_tolerance',
        'random_safe_finish_max_distance',
        'random_safe_finish_phase_min',
        'random_safe_finish_min_team_separation',
        'random_safe_finish_full_team_separation',
        'random_safe_finish_min_neighbor_separation',
        'random_safe_finish_local_separation_scale',
        'random_safe_finish_max_abs_cte',
        'random_safe_finish_yield_release_min_separation',
        'random_safe_finish_yield_release_max_closing_speed',
        'random_safe_finish_yield_release_min_route_progress',
        'random_safe_finish_max_cpa_score',
        'random_safe_finish_max_local_score',
        'random_safe_finish_target_speed',
        'random_safe_finish_min_speed_scale',
        'random_safe_finish_hold_distance',
        'random_safe_finish_hold_weight',
        'random_safe_finish_low_priority_threshold',
        'random_safe_finish_low_priority_speed_multiplier',
        'random_safe_finish_max_omega',
        'random_safe_finish_omega_weight',
        'random_goal_hold_weight',
        'random_goal_hold_weight_end',
        'random_goal_hold_distance',
        'random_goal_hold_phase_min',
        'random_goal_hold_target_speed',
        'random_goal_hold_target_omega',
        'random_goal_hold_omega_weight',
        'random_offroute_finish_weight',
        'random_offroute_finish_weight_end',
        'random_offroute_finish_goal_tolerance',
        'random_offroute_finish_max_distance',
        'random_offroute_finish_min_distance',
        'random_offroute_finish_route_progress_min',
        'random_offroute_finish_min_abs_cte',
        'random_offroute_finish_full_abs_cte',
        'random_offroute_finish_phase_min',
        'random_offroute_finish_min_team_separation',
        'random_offroute_finish_min_neighbor_separation',
        'random_offroute_finish_max_cpa_score',
        'random_offroute_finish_max_local_score',
        'random_offroute_finish_allow_threat_overlap',
        'random_offroute_finish_target_speed',
        'random_offroute_finish_min_speed',
        'random_offroute_finish_max_omega',
        'random_offroute_finish_omega_reference',
        'random_offroute_finish_omega_weight',
        'random_cte_recovery_weight',
        'random_cte_recovery_weight_end',
        'random_cte_recovery_goal_tolerance',
        'random_cte_recovery_max_distance',
        'random_cte_recovery_min_abs_cte',
        'random_cte_recovery_full_abs_cte',
        'random_cte_source',
        'random_cte_recovery_min_neighbor_separation',
        'random_cte_recovery_agent_indices',
        'random_cte_recovery_all_agents',
        'random_cte_recovery_allow_threat_overlap',
        'random_cte_recovery_target_speed',
        'random_cte_recovery_min_speed',
        'random_cte_recovery_speed_cte_slowdown',
        'random_cte_recovery_speed_heading_gate',
        'random_cte_recovery_max_omega',
        'random_cte_recovery_omega_reference',
        'random_cte_recovery_linear_weight',
        'random_cte_recovery_omega_weight',
        'random_cte_recovery_omega_mode',
        'random_cte_recovery_cte_lookahead',
        'random_cte_recovery_cte_heading_scale',
        'random_recovery_pretrain_epochs',
        'random_recovery_pretrain_learning_rate',
        'random_recovery_pretrain_max_grad_norm',
        'random_recovery_pretrain_offroute_scale',
        'random_recovery_pretrain_cte_scale',
        'random_recovery_pretrain_clear_scale',
        'random_recovery_pretrain_late_lagging_scale',
        'random_recovery_safety_gate_scale',
        'random_recovery_safety_gate_mode',
        'goal_heading_omega_sign',
        'random_clear_ahead_weight',
        'random_clear_ahead_weight_end',
        'random_clear_ahead_scenarios',
        'random_clear_ahead_distance',
        'random_clear_ahead_bearing_deg',
        'random_clear_ahead_cone_mode',
        'random_clear_ahead_goal_tolerance',
        'random_clear_ahead_max_distance',
        'random_clear_ahead_min_route_progress',
        'random_clear_ahead_max_route_progress',
        'random_clear_ahead_min_abs_cte',
        'random_clear_ahead_max_abs_cte',
        'random_clear_ahead_min_neighbor_separation',
        'random_clear_ahead_agent_indices',
        'random_clear_ahead_max_cpa_score',
        'random_clear_ahead_max_local_score',
        'random_clear_ahead_exclude_deconflict',
        'random_clear_ahead_target_source',
        'random_clear_ahead_target_speed',
        'random_clear_ahead_min_speed',
        'random_clear_ahead_speed_cte_slowdown',
        'random_clear_ahead_speed_cte_start',
        'random_clear_ahead_speed_cte_full',
        'random_clear_ahead_max_omega',
        'random_clear_ahead_omega_reference',
        'random_clear_ahead_omega_weight',
        'random_close_formation_escape_weight',
        'random_close_formation_escape_weight_end',
        'random_close_formation_escape_scenarios',
        'random_close_formation_escape_min_sep',
        'random_close_formation_escape_max_sep',
        'random_close_formation_escape_max_self_speed',
        'random_close_formation_escape_goal_tolerance',
        'random_close_formation_escape_max_distance',
        'random_close_formation_escape_target_speed',
        'random_close_formation_escape_min_speed',
        'random_close_formation_escape_max_omega',
        'random_close_formation_escape_omega_weight',
        'random_close_formation_escape_agent_indices',
        'random_close_formation_escape_require_closing',
        'random_late_lagging_weight',
        'random_late_lagging_weight_end',
        'random_late_lagging_goal_tolerance',
        'random_late_lagging_max_distance',
        'random_late_lagging_min_distance',
        'random_late_lagging_max_self_progress',
        'random_late_lagging_min_team_progress',
        'random_late_lagging_min_progress_gap',
        'random_late_lagging_agent_indices',
        'random_late_lagging_min_finished_teammates',
        'random_late_lagging_finished_progress',
        'random_late_lagging_phase_min',
        'random_late_lagging_min_team_separation',
        'random_late_lagging_min_neighbor_separation',
        'random_late_lagging_max_cpa_score',
        'random_late_lagging_max_local_score',
        'random_late_lagging_allow_threat_overlap',
        'random_late_lagging_target_speed',
        'random_late_lagging_min_speed',
        'random_late_lagging_max_omega',
        'random_late_lagging_omega_reference',
        'random_late_lagging_omega_weight',
        'random_late_lagging_target_source',
        'policy_anchor_weight',
        'policy_anchor_weight_end',
        'policy_anchor_crossing_only',
        'policy_anchor_exclude_lagging_finish',
        'policy_anchor_exclude_team_safety_brake',
        'policy_anchor_exclude_random_deconflict',
        'policy_anchor_exclude_random_safe_finish',
        'policy_anchor_exclude_random_goal_hold',
        'policy_anchor_exclude_random_offroute_finish',
        'policy_anchor_exclude_random_cte_recovery',
        'policy_anchor_exclude_random_clear_ahead',
        'policy_anchor_exclude_random_late_lagging',
        'policy_anchor_exclude_random_pairwise_role_guard',
        'policy_anchor_exclude_random_pairwise_role_guard',
        'separate_actor_critic_grad_clip',
    )
    for field in simple_fields:
        if field in cli_overrides:
            continue
        if field in payload and payload[field] is not None:
            setattr(args, field, payload[field])

    action_bounds = payload.get('action_bounds') or {}
    if 'linear_delta' in action_bounds and 'linear_delta_limit' not in cli_overrides:
        args.linear_delta_limit = float(action_bounds['linear_delta'])
    if 'angular_delta' in action_bounds and 'angular_delta_limit' not in cli_overrides:
        args.angular_delta_limit = float(action_bounds['angular_delta'])

    reward_config = payload.get('reward_config') or {}
    reward_fields = (
        'progress_weight',
        'goal_bonus',
        'collision_penalty',
        'near_miss_weight',
        'near_miss_exponent',
        'head_on_near_miss_distance',
        'conflict_distance',
        'anticipation_distance',
        'conflict_risk_weight',
        'conflict_bearing_floor',
        'conflict_risk_time_horizon',
        'conflict_risk_safe_dcpa',
        'conflict_brake_weight',
        'conflict_progress_scale',
        'conflict_resolution_reward_weight',
        'conflict_escalation_penalty_weight',
        'unsafe_close_speed_penalty_weight',
        'path_deviation_penalty_weight',
        'path_deviation_tolerance',
        'path_deviation_conflict_scale',
        'path_inefficiency_penalty_weight',
        'waypoint_pass_quality_bonus',
        'role_speed_asymmetry_weight',
        'role_giveway_speed',
        'wrong_heading_speed_penalty_weight',
        'wrong_heading_speed_threshold_deg',
        'wrong_heading_conflict_relief',
        'far_side_turn_penalty_weight',
        'far_side_turn_threshold_deg',
        'goal_queue_weight',
        'goal_queue_cluster_radius',
        'goal_queue_engage_distance',
        'goal_queue_follower_speed',
        'goal_queue_min_separation',
        'desired_conflict_speed',
        'stop_go_penalty_weight',
        'head_on_guidance_distance',
        'crossing_guidance_distance',
        'head_on_target_starboard_offset',
        'head_on_corridor_reward_weight',
        'head_on_centerline_penalty_weight',
        'head_on_turn_reward_weight',
        'head_on_forward_reward_weight',
        'head_on_speed_drop_penalty_weight',
        'head_on_close_penalty_weight',
        'head_on_no_turn_penalty_weight',
        'head_on_phase_gate_strength',
        'crossing_starboard_turn_reward_weight',
        'crossing_forward_reward_weight',
        'crossing_slowdown_reward_weight',
        'crossing_overspeed_penalty_weight',
        'crossing_close_forward_penalty_weight',
        'crossing_yield_speed',
        'crossing_time_separation_reward_weight',
        'crossing_time_separation_penalty_weight',
        'crossing_time_gap_target',
        'overtaking_starboard_turn_reward_weight',
        'overtaking_forward_reward_weight',
        'overtaking_corridor_reward_weight',
        'overtaking_centerline_penalty_weight',
        'overtaking_close_penalty_weight',
        'colregs_port_turn_penalty_weight',
        'heading_relief_factor',
        'heading_error_weight',
        'action_smoothness_weight',
        'pure_cruise_reward_weight',
        'pure_idle_penalty_weight',
        'pure_turn_penalty_weight',
        'pure_spin_penalty_weight',
        'time_penalty',
        'stall_penalty',
        'angular_accel_penalty_weight',
        'straight_line_omega_penalty_weight',
        'saturated_omega_flip_penalty_weight',
        'forward_speed_change_penalty_weight',
        'omega_flip_saturation_threshold',
        'straight_line_omega_conflict_floor',
        'clear_ahead_distance',
        'clear_ahead_bearing_deg',
        'clear_ahead_cte_weight',
        'clear_ahead_heading_weight',
        'clear_ahead_omega_weight',
        'anticipatory_avoidance_turn_reward_weight',
        'anticipatory_avoidance_turn_penalty_weight',
        'anticipatory_avoidance_turn_distance',
        'anticipatory_cpa_distance',
        'anticipatory_cpa_time_horizon',
        'anticipatory_dcpa_target',
        'anticipatory_dcpa_deficit_penalty_weight',
        'anticipatory_dcpa_improvement_reward_weight',
        'anticipatory_closing_reduction_reward_weight',
        'anticipatory_yield_speed',
        'anticipatory_yield_speed_penalty_weight',
    )
    for field in reward_fields:
        if field in cli_overrides:
            continue
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


def _eval_ros_domain_id(args) -> int:
    base = int(getattr(args, 'base_ros_domain_id', 100))
    workers = max(1, int(getattr(args, 'num_sampler_workers', 1)))
    return base + workers


def _eval_subprocess_env(args) -> dict:
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(_eval_ros_domain_id(args))
    return env


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
    subprocess.run(command, check=True, env=_eval_subprocess_env(args))
    print(f'Automatic checkpoint ranking saved to {ranking_json}', flush=True)


def _checkpoint_eval_outputs(args, scenarios: tuple[str, ...], output_path: Path) -> tuple[tuple[str, ...], Path, Path]:
    eval_scenarios = tuple(args.checkpoint_eval_scenarios) if args.checkpoint_eval_scenarios else tuple(dict.fromkeys(scenarios))
    if not eval_scenarios:
        raise RuntimeError('Automatic checkpoint evaluation requires at least one scenario.')

    ranking_json = Path(args.checkpoint_ranking_json) if args.checkpoint_ranking_json else output_path.with_name(f'{output_path.stem}_ranking.json')
    eval_json_dir = Path(args.checkpoint_eval_json_dir) if args.checkpoint_eval_json_dir else output_path.with_name(f'{output_path.stem}_eval')
    return eval_scenarios, ranking_json, eval_json_dir


def _checkpoint_ranking_record(summary: dict) -> dict:
    scenario_summaries = summary.get('scenario_summaries', {})
    crossing = scenario_summaries.get('five_usv_dense_crossing', {})
    progress_values = [
        float(item.get('mean_team_goal_distance_delta', 0.0))
        for item in scenario_summaries.values()
    ]
    balanced_progress = min(progress_values) if progress_values else float('-inf')
    return {
        'model': summary['model'],
        'collision_rate': float(summary['collision_rate']),
        'timeout_rate': float(summary['timeout_rate']),
        'mean_team_goal_distance_delta': float(summary['mean_team_goal_distance_delta']),
        'mean_team_goal_progress_ratio': float(summary['mean_team_goal_progress_ratio']),
        'crossing_team_goal_distance_delta': float(crossing.get('mean_team_goal_distance_delta', float('-inf'))),
        'crossing_team_goal_progress_ratio': float(crossing.get('mean_team_goal_progress_ratio', float('-inf'))),
        'balanced_team_goal_distance_delta': balanced_progress,
    }


def _best_checkpoint_record(records: list[dict], key: str) -> dict | None:
    if not records:
        return None
    return max(records, key=lambda item: (item[key], -item['collision_rate'], -item['timeout_rate']))


def _write_checkpoint_ranking_summary(args, scenarios: tuple[str, ...], output_path: Path):
    eval_scenarios, ranking_json, eval_json_dir = _checkpoint_eval_outputs(args, scenarios, output_path)
    if not eval_json_dir.exists():
        return

    ranking = []
    for summary_path in sorted(eval_json_dir.glob('*.json')):
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
        ranking.append(_checkpoint_ranking_record(summary))

    result = {
        'checkpoint_dir': str(Path(args.checkpoint_dir) if args.checkpoint_dir else output_path.with_name(f'{output_path.stem}_checkpoints')),
        'scenarios': list(eval_scenarios),
        'episodes': int(args.checkpoint_eval_episodes),
        'steps_per_episode': int(args.checkpoint_eval_steps),
        'evaluated_checkpoints': ranking,
        'best_overall': _best_checkpoint_record(ranking, 'mean_team_goal_distance_delta'),
        'best_crossing': _best_checkpoint_record(ranking, 'crossing_team_goal_distance_delta'),
        'best_balanced': _best_checkpoint_record(ranking, 'balanced_team_goal_distance_delta'),
    }
    ranking_json.parent.mkdir(parents=True, exist_ok=True)
    ranking_json.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Automatic checkpoint ranking updated: {ranking_json}', flush=True)


def _evaluate_checkpoint_immediately(args, checkpoint_path: Path, scenarios: tuple[str, ...], output_path: Path):
    if not args.auto_evaluate_checkpoints:
        return

    eval_scenarios, _ranking_json, eval_json_dir = _checkpoint_eval_outputs(args, scenarios, output_path)
    eval_json_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_eval_json = eval_json_dir / f'{checkpoint_path.stem}.json'
    command = [
        sys.executable,
        '-m',
        'usv_rl.evaluate_mappo_policy',
        '--model',
        str(checkpoint_path),
        '--episodes',
        str(args.checkpoint_eval_episodes),
        '--steps-per-episode',
        str(args.checkpoint_eval_steps),
        '--device',
        str(args.checkpoint_eval_device or args.device),
        '--output-json',
        str(checkpoint_eval_json),
    ]
    for scenario in eval_scenarios:
        command.extend(['--scenario', scenario])

    try:
        subprocess.run(command, check=True, env=_eval_subprocess_env(args))
        print(f'Automatic checkpoint eval saved: {checkpoint_eval_json}', flush=True)
        _write_checkpoint_ranking_summary(args, scenarios, output_path)
    except subprocess.CalledProcessError as exc:
        print(f'WARNING: Checkpoint eval failed (exit={exc.returncode}), training continues.', flush=True)


def _create_env(args, agent_namespaces: tuple[str, ...], scenarios: tuple[str, ...], reward_config: RewardConfig) -> MultiAgentEnv:
    return MultiAgentEnv(
        MultiAgentEnvConfig(
            agent_namespaces=agent_namespaces,
            enable_rl_backend=True,
            rl_control_mode=args.rl_control_mode,
            action_mode=args.action_mode,
            action_speed_scale_min=float(getattr(args, 'action_speed_scale_min', 0.25)),
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
            angular_authority_floor=float(args.angular_authority_floor),
            min_forward_speed_floor=float(args.min_forward_speed_floor),
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
            route_progress_cte_gate_start=float(getattr(args, 'route_progress_cte_gate_start', 0.0)),
            route_progress_cte_gate_width=float(getattr(args, 'route_progress_cte_gate_width', 0.0)),
            route_progress_cte_gate_floor=float(getattr(args, 'route_progress_cte_gate_floor', 0.25)),
            random_encounter_route_priority=bool(getattr(args, 'random_encounter_route_priority', False)),
            max_waypoints_per_episode=int(getattr(args, 'max_waypoints_per_episode', 1)),
            waypoint_bonus=float(getattr(args, 'waypoint_bonus', 10.0)),
        )
    )


def _recover_env(env_factory, *, context: str, max_attempts: int = 3, reset_seed: int | None = None, reset_options: dict | None = None):
    last_error = None
    for attempt in range(1, max_attempts + 1):
        env = None
        try:
            env = env_factory()
            observations, info = env.reset(seed=reset_seed, options=reset_options)
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
    cli_overrides = _cli_explicit_args()
    if resume_path is not None:
        resume_payload = _load_resume_payload(torch, resume_path)
        _apply_resume_configuration(args, resume_payload, cli_overrides=cli_overrides)
        print(f'Resuming MAPPO from checkpoint scaffold: {resume_path}', flush=True)

    _validate_parallel_sampler_args(args)

    hidden_sizes = tuple(args.hidden_sizes or [128, 128])
    agent_namespaces = _build_agent_namespaces(max(2, args.num_agents))
    scenarios = _resolve_scenarios(args)
    curriculum_scenarios = _resolve_curriculum_scenarios(args, scenarios)
    seed_scheduler = _CurriculumSeedScheduler(getattr(args, 'curriculum_seeds', None))
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
    print(f'Using MAPPO scenario branches: {scenarios}', flush=True)
    print(f'Using MAPPO rollout curriculum: {curriculum_scenarios}', flush=True)
    if seed_scheduler.enabled:
        print(f'Using MAPPO rollout reset seeds: {seed_scheduler.seeds}', flush=True)
    env_factory = lambda: _create_env(args, agent_namespaces, curriculum_scenarios, reward_config)
    scenario_to_index = _scenario_index_map(scenarios)
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
                scenarios=curriculum_scenarios,
                model_scenarios=scenarios,
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
                reset_seed=seed_scheduler.next(),
            )
            global_state = info['global_state']

        use_neighbor_attention = bool(getattr(args, 'neighbor_attention', False))
        if use_neighbor_attention:
            from usv_rl.neighbor_attention import AttentionActor, AttentionCritic
            from usv_rl.multi_agent_types import ENCOUNTER_TYPE_COUNT
            actor = AttentionActor(
                max_neighbors=int(args.max_neighbors),
                encounter_dim=ENCOUNTER_TYPE_COUNT,
                hidden_sizes=hidden_sizes,
                action_dim=model_metadata['action_dim'],
                embed_dim=int(args.attention_embed_dim),
                num_heads=int(args.attention_num_heads),
                encounter_residual=bool(getattr(args, 'attention_encounter_residual', False)),
                scenario_names=tuple(scenarios),
                scenario_residual=bool(getattr(args, 'attention_scenario_residual', False)),
                scenario_head=bool(getattr(args, 'attention_scenario_head', False)),
                scenario_trunk=bool(getattr(args, 'attention_scenario_trunk', False)),
            ).to(device)
            critic = AttentionCritic(
                max_neighbors=int(args.max_neighbors),
                encounter_dim=ENCOUNTER_TYPE_COUNT,
                global_state_dim=model_metadata['global_state_size'],
                hidden_sizes=hidden_sizes,
                embed_dim=int(args.attention_embed_dim),
                num_heads=int(args.attention_num_heads),
            ).to(device)
        else:
            actor = _build_mlp(nn, model_metadata['local_observation_size'], hidden_sizes, model_metadata['action_dim']).to(device)
            critic = _build_mlp(nn, model_metadata['local_observation_size'] + model_metadata['global_state_size'], hidden_sizes, 1).to(device)
        _maybe_freeze_actor_base(actor, args)
        _log_std_init = float(getattr(args, 'actor_log_std_init', 0.0))
        actor_log_std = nn.Parameter(torch.full((model_metadata['action_dim'],), _log_std_init, device=device))
        actor_parameters = [parameter for parameter in actor.parameters() if parameter.requires_grad]
        optimizer = torch.optim.Adam(actor_parameters + list(critic.parameters()) + [actor_log_std], lr=args.learning_rate)
        action_low_tensor = torch.as_tensor(model_metadata['action_low'], dtype=torch.float32, device=device)
        action_high_tensor = torch.as_tensor(model_metadata['action_high'], dtype=torch.float32, device=device)

        obs_normalizer = None
        use_obs_norm = bool(getattr(args, 'normalize_observations', False))
        freeze_obs_norm = bool(getattr(args, 'freeze_observation_normalizer', False))
        if use_obs_norm:
            obs_normalizer = ObservationNormalizer(model_metadata['local_observation_size'])
            if freeze_obs_norm:
                print('Observation normalizer updates are frozen; loaded mean/variance will be reused.', flush=True)

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
            if use_obs_norm and obs_normalizer is not None:
                saved_norm = resume_payload.get('obs_normalizer')
                if saved_norm is not None:
                    obs_normalizer.load_state_dict(saved_norm)
                    print('Observation normalizer state restored from checkpoint.', flush=True)
        elif args.load_weights_from:
            _load_weights_only(
                torch, device, actor, critic, actor_log_std,
                Path(args.load_weights_from), obs_normalizer,
                max_agents=max(len(agent_namespaces), int(args.max_agents)),
                neighbor_attention=use_neighbor_attention,
            )

        # Override actor_log_std if --force-actor-log-std is set
        _force_log_std = getattr(args, 'force_actor_log_std', None)
        if _force_log_std is not None:
            actor_log_std.data.fill_(float(_force_log_std))
            print(f'Forced actor_log_std to {_force_log_std} (σ={math.exp(_force_log_std):.4f}).', flush=True)

        policy_anchor_start = float(getattr(args, 'policy_anchor_weight', 0.0))
        policy_anchor_end = (
            float(getattr(args, 'policy_anchor_weight_end'))
            if getattr(args, 'policy_anchor_weight_end', None) is not None
            else policy_anchor_start
        )
        policy_anchor_actor = None
        if max(abs(policy_anchor_start), abs(policy_anchor_end)) > 0.0:
            policy_anchor_actor = copy.deepcopy(actor).to(device)
            policy_anchor_actor.eval()
            for parameter in policy_anchor_actor.parameters():
                parameter.requires_grad_(False)
            print('Policy anchor actor captured from loaded weights.', flush=True)

        while total_steps < max(0, args.total_timesteps):
            rollout_wall_start = time.perf_counter()
            remaining_agent_steps = max(0, int(args.total_timesteps) - total_steps)
            current_rollout_steps = min(rollout_steps, max(1, int(math.ceil(remaining_agent_steps / len(agent_namespaces)))))

            rollout_agent_steps = 0
            rollout_steps_collected = 0
            rollout_mean_reward = 0.0
            rollout_episode_count = 0
            if sampler is not None:
                worker_results = sampler.collect(
                    actor=actor,
                    critic=critic,
                    actor_log_std=actor_log_std,
                    rollout_steps=current_rollout_steps,
                    remaining_agent_steps=remaining_agent_steps,
                    obs_normalizer=obs_normalizer,
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
                flat_scenario_ids_parts = []
                flat_agent_indices_parts = []
                rollout_reward_count = 0

                for result in worker_results:
                    rewards = np.asarray(result['rewards'], dtype=np.float32)
                    advantages = np.asarray(result['advantages'], dtype=np.float32)
                    returns = np.asarray(result['returns'], dtype=np.float32)
                    flat_obs_parts.append(np.asarray(result['obs'], dtype=np.float32).reshape(-1, model_metadata['local_observation_size']))
                    flat_states_parts.append(np.asarray(result['states'], dtype=np.float32).reshape(-1, model_metadata['global_state_size']))
                    flat_actions_parts.append(np.asarray(result['actions'], dtype=np.float32).reshape(-1, model_metadata['action_dim']))
                    flat_log_probs_parts.append(np.asarray(result['log_probs'], dtype=np.float32).reshape(-1))
                    flat_advantages_parts.append(advantages.reshape(-1))
                    flat_returns_parts.append(returns.reshape(-1))
                    if 'scenario_ids' in result:
                        flat_scenario_ids_parts.append(np.asarray(result['scenario_ids'], dtype=np.int32).reshape(-1))
                    if 'agent_indices' in result:
                        flat_agent_indices_parts.append(np.asarray(result['agent_indices'], dtype=np.int32).reshape(-1))
                    rollout_agent_steps += int(result['agent_steps'])
                    rollout_steps_collected += int(result['rollout_steps_collected'])
                    rollout_mean_reward += float(rewards.sum())
                    rollout_reward_count += int(rewards.size)
                    rollout_episode_count += int(result.get('episode_count', 0))

                if rollout_reward_count > 0:
                    rollout_mean_reward /= max(1, rollout_reward_count)
                total_steps += rollout_agent_steps
                raw_flat_obs_np = np.concatenate(flat_obs_parts, axis=0)
                if obs_normalizer is not None:
                    if not freeze_obs_norm:
                        obs_normalizer.update(raw_flat_obs_np)
                    flat_obs_np = obs_normalizer.normalize(raw_flat_obs_np)
                else:
                    flat_obs_np = raw_flat_obs_np
                flat_raw_obs = torch.as_tensor(raw_flat_obs_np, dtype=torch.float32, device=device)
                flat_obs = torch.as_tensor(flat_obs_np, dtype=torch.float32, device=device)
                flat_states = torch.as_tensor(np.concatenate(flat_states_parts, axis=0), dtype=torch.float32, device=device)
                flat_actions = torch.as_tensor(np.concatenate(flat_actions_parts, axis=0), dtype=torch.float32, device=device)
                flat_old_log_probs = torch.as_tensor(np.concatenate(flat_log_probs_parts, axis=0), dtype=torch.float32, device=device)
                flat_advantages = torch.as_tensor(np.concatenate(flat_advantages_parts, axis=0), dtype=torch.float32, device=device)
                flat_returns = torch.as_tensor(np.concatenate(flat_returns_parts, axis=0), dtype=torch.float32, device=device)
                flat_scenario_ids_np = np.concatenate(flat_scenario_ids_parts, axis=0) if flat_scenario_ids_parts else None
                flat_agent_indices_np = np.concatenate(flat_agent_indices_parts, axis=0) if flat_agent_indices_parts else None
            else:
                storage_obs = []
                storage_states = []
                storage_actions = []
                storage_log_probs = []
                storage_values = []
                storage_rewards = []
                storage_dones = []
                storage_scenario_ids = []
                storage_agent_indices = []
                rollout_episode_count = 0
                current_scenario = env.current_scenario_name
                current_scenario_id = int(scenario_to_index.get(current_scenario, -1))

                for _ in range(current_rollout_steps):
                    agent_order = env.agent_ids
                    obs_batch = np.stack([observations[agent_id] for agent_id in agent_order], axis=0).astype(np.float32)
                    state_batch = np.repeat(np.asarray(global_state, dtype=np.float32)[None, :], len(agent_order), axis=0)

                    if obs_normalizer is not None:
                        if not freeze_obs_norm:
                            obs_normalizer.update(obs_batch)
                        obs_batch_for_net = obs_normalizer.normalize(obs_batch)
                    else:
                        obs_batch_for_net = obs_batch

                    obs_tensor = torch.as_tensor(obs_batch_for_net, dtype=torch.float32, device=device)
                    state_tensor = torch.as_tensor(state_batch, dtype=torch.float32, device=device)
                    critic_input = torch.cat([obs_tensor, state_tensor], dim=-1)
                    scenario_id_tensor = None
                    if current_scenario_id >= 0:
                        scenario_id_tensor = torch.full((len(agent_order),), current_scenario_id, dtype=torch.long, device=device)

                    with torch.no_grad():
                        action_mean = _actor_forward(actor, obs_tensor, scenario_id_tensor)
                        if getattr(args, 'squash_actions', False):
                            action_mean = action_mean.clamp(-3.0, 3.0)
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
                            reset_seed=seed_scheduler.next(),
                        )
                        global_state = info['global_state']
                        current_scenario = env.current_scenario_name
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
                    storage_agent_indices.append(np.arange(len(agent_order), dtype=np.int32))

                    total_steps += len(agent_order)
                    rollout_agent_steps += len(agent_order)
                    rollout_steps_collected += 1
                    if done:
                        rollout_episode_count += 1
                    observations = next_observations
                    global_state = next_info['global_state']
                    if done:
                        reset_seed = seed_scheduler.next()
                        try:
                            observations, info = env.reset(seed=reset_seed)
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
                                reset_seed=reset_seed,
                            )
                        else:
                            consecutive_env_failures = 0
                        global_state = info['global_state']
                        current_scenario = env.current_scenario_name
                        current_scenario_id = int(scenario_to_index.get(current_scenario, -1))
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
                    bootstrap_obs_for_net = obs_normalizer.normalize(bootstrap_obs) if obs_normalizer is not None else bootstrap_obs
                    bootstrap_obs_tensor = torch.as_tensor(bootstrap_obs_for_net, dtype=torch.float32, device=device)
                    bootstrap_state_tensor = torch.as_tensor(bootstrap_state, dtype=torch.float32, device=device)
                    bootstrap_values = critic(torch.cat([bootstrap_obs_tensor, bootstrap_state_tensor], dim=-1)).squeeze(-1).cpu().numpy()

                rewards = _flatten_rollout_batches(storage_rewards)
                rollout_mean_reward = float(rewards.mean()) if rewards.size > 0 else 0.0
                advantages, returns = _compute_advantages_and_returns(
                    storage_rewards,
                    storage_values,
                    storage_dones,
                    bootstrap_values,
                    args.gamma,
                    args.gae_lambda,
                )

                raw_flat_obs_sw = _flatten_rollout_batches(storage_obs).reshape(-1, model_metadata['local_observation_size'])
                if obs_normalizer is not None:
                    flat_obs_sw = obs_normalizer.normalize(raw_flat_obs_sw)
                else:
                    flat_obs_sw = raw_flat_obs_sw
                flat_raw_obs = torch.as_tensor(raw_flat_obs_sw, dtype=torch.float32, device=device)
                flat_obs = torch.as_tensor(flat_obs_sw, dtype=torch.float32, device=device)
                flat_states = torch.as_tensor(_flatten_rollout_batches(storage_states).reshape(-1, model_metadata['global_state_size']), dtype=torch.float32, device=device)
                flat_actions = torch.as_tensor(_flatten_rollout_batches(storage_actions).reshape(-1, model_metadata['action_dim']), dtype=torch.float32, device=device)
                flat_old_log_probs = torch.as_tensor(_flatten_rollout_batches(storage_log_probs).reshape(-1), dtype=torch.float32, device=device)
                flat_advantages = torch.as_tensor(advantages.reshape(-1), dtype=torch.float32, device=device)
                flat_returns = torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=device)
                flat_scenario_ids_np = _flatten_rollout_batches(storage_scenario_ids).reshape(-1) if storage_scenario_ids else None
                flat_agent_indices_np = _flatten_rollout_batches(storage_agent_indices).reshape(-1) if storage_agent_indices else None

            # ---------- Advantage Normalization ----------
            if getattr(args, 'per_scenario_advantage_norm', False) and flat_scenario_ids_np is not None:
                unique_ids = np.unique(flat_scenario_ids_np)
                for sid in unique_ids:
                    mask = torch.as_tensor(flat_scenario_ids_np == sid, dtype=torch.bool, device=device)
                    if mask.sum() > 1:
                        subset = flat_advantages[mask]
                        flat_advantages[mask] = (subset - subset.mean()) / (subset.std(unbiased=False) + 1e-6)
            else:
                flat_advantages = (flat_advantages - flat_advantages.mean()) / (flat_advantages.std(unbiased=False) + 1e-6)

            # ---------- Scenario-Balanced Loss Weights ----------
            flat_scenario_weights = None
            if getattr(args, 'scenario_balanced_loss', False) and flat_scenario_ids_np is not None:
                unique_sids, sid_counts = np.unique(flat_scenario_ids_np, return_counts=True)
                K = len(unique_sids)
                N = len(flat_scenario_ids_np)
                sw = np.ones(N, dtype=np.float32)
                for sid, count in zip(unique_sids, sid_counts):
                    sw[flat_scenario_ids_np == sid] = float(N) / float(K * count)
                flat_scenario_weights = torch.as_tensor(sw, dtype=torch.float32, device=device)

            sample_count = flat_obs.shape[0]
            minibatch_size = min(max(1, args.minibatch_size), sample_count)
            update_wall_start = time.perf_counter()

            progress_fraction = min(1.0, total_steps / max(1, args.total_timesteps))
            entropy_coef_start = float(args.entropy_coef)
            entropy_coef_end = float(args.entropy_coef_end) if args.entropy_coef_end is not None else entropy_coef_start
            current_entropy_coef = entropy_coef_start + (entropy_coef_end - entropy_coef_start) * progress_fraction

            lr_start = float(args.learning_rate)
            lr_end = float(args.learning_rate_end) if args.learning_rate_end is not None else lr_start
            current_lr = lr_start + (lr_end - lr_start) * progress_fraction
            for param_group in optimizer.param_groups:
                param_group['lr'] = current_lr

            crossing_imitation_start = float(getattr(args, 'crossing_imitation_weight', 0.0))
            crossing_imitation_end = (
                float(getattr(args, 'crossing_imitation_weight_end'))
                if getattr(args, 'crossing_imitation_weight_end', None) is not None
                else crossing_imitation_start
            )
            current_crossing_imitation_weight = crossing_imitation_start + (crossing_imitation_end - crossing_imitation_start) * progress_fraction
            last_crossing_imitation_loss = 0.0
            last_crossing_pretrain_loss = 0.0
            overtaking_imitation_start = float(getattr(args, 'overtaking_imitation_weight', 0.0))
            overtaking_imitation_end = (
                float(getattr(args, 'overtaking_imitation_weight_end'))
                if getattr(args, 'overtaking_imitation_weight_end', None) is not None
                else overtaking_imitation_start
            )
            current_overtaking_imitation_weight = overtaking_imitation_start + (overtaking_imitation_end - overtaking_imitation_start) * progress_fraction
            last_overtaking_imitation_loss = 0.0
            overtaking_imitation_active_sum = 0
            overtaking_imitation_active_seen = 0
            near_goal_finish_start = float(getattr(args, 'near_goal_finish_weight', 0.0))
            near_goal_finish_end = (
                float(getattr(args, 'near_goal_finish_weight_end'))
                if getattr(args, 'near_goal_finish_weight_end', None) is not None
                else near_goal_finish_start
            )
            current_near_goal_finish_weight = near_goal_finish_start + (near_goal_finish_end - near_goal_finish_start) * progress_fraction
            last_near_goal_finish_loss = 0.0
            ppo_policy_loss_scale = max(0.0, float(getattr(args, 'ppo_policy_loss_scale', 1.0)))
            ppo_value_loss_scale = max(0.0, float(getattr(args, 'ppo_value_loss_scale', 1.0)))
            value_loss_huber_delta = max(0.0, float(getattr(args, 'value_loss_huber_delta', 0.0)))
            lagging_finish_start = float(getattr(args, 'lagging_finish_weight', 0.0))
            lagging_finish_end = (
                float(getattr(args, 'lagging_finish_weight_end'))
                if getattr(args, 'lagging_finish_weight_end', None) is not None
                else lagging_finish_start
            )
            current_lagging_finish_weight = lagging_finish_start + (lagging_finish_end - lagging_finish_start) * progress_fraction
            last_lagging_finish_loss = 0.0
            lagging_finish_active_sum = 0
            lagging_finish_active_seen = 0
            team_safety_brake_start = float(getattr(args, 'team_safety_brake_weight', 0.0))
            team_safety_brake_end = (
                float(getattr(args, 'team_safety_brake_weight_end'))
                if getattr(args, 'team_safety_brake_weight_end', None) is not None
                else team_safety_brake_start
            )
            current_team_safety_brake_weight = team_safety_brake_start + (team_safety_brake_end - team_safety_brake_start) * progress_fraction
            last_team_safety_brake_loss = 0.0
            team_safety_brake_active_sum = 0
            team_safety_brake_active_seen = 0
            random_deconflict_start = float(getattr(args, 'random_deconflict_weight', 0.0))
            random_deconflict_end = (
                float(getattr(args, 'random_deconflict_weight_end'))
                if getattr(args, 'random_deconflict_weight_end', None) is not None
                else random_deconflict_start
            )
            current_random_deconflict_weight = random_deconflict_start + (random_deconflict_end - random_deconflict_start) * progress_fraction
            last_random_deconflict_loss = 0.0
            last_random_deconflict_pretrain_loss = 0.0
            random_deconflict_active_sum = 0
            random_deconflict_active_seen = 0
            random_role_balance_start = float(getattr(args, 'random_role_balance_weight', 0.0))
            random_role_balance_end = (
                float(getattr(args, 'random_role_balance_weight_end'))
                if getattr(args, 'random_role_balance_weight_end', None) is not None
                else random_role_balance_start
            )
            current_random_role_balance_weight = random_role_balance_start + (random_role_balance_end - random_role_balance_start) * progress_fraction
            last_random_role_balance_loss = 0.0
            last_random_role_balance_pretrain_loss = 0.0
            random_role_balance_active_sum = 0
            random_role_balance_active_seen = 0
            random_pairwise_role_guard_start = float(getattr(args, 'random_pairwise_role_guard_weight', 0.0))
            random_pairwise_role_guard_end = (
                float(getattr(args, 'random_pairwise_role_guard_weight_end'))
                if getattr(args, 'random_pairwise_role_guard_weight_end', None) is not None
                else random_pairwise_role_guard_start
            )
            current_random_pairwise_role_guard_weight = random_pairwise_role_guard_start + (random_pairwise_role_guard_end - random_pairwise_role_guard_start) * progress_fraction
            last_random_pairwise_role_guard_loss = 0.0
            last_random_pairwise_role_guard_pretrain_loss = 0.0
            last_random_recovery_pretrain_loss = 0.0
            random_pairwise_role_guard_active_sum = 0
            random_pairwise_role_guard_active_seen = 0
            random_safe_finish_start = float(getattr(args, 'random_safe_finish_weight', 0.0))
            random_safe_finish_end = (
                float(getattr(args, 'random_safe_finish_weight_end'))
                if getattr(args, 'random_safe_finish_weight_end', None) is not None
                else random_safe_finish_start
            )
            current_random_safe_finish_weight = random_safe_finish_start + (random_safe_finish_end - random_safe_finish_start) * progress_fraction
            last_random_safe_finish_loss = 0.0
            random_safe_finish_active_sum = 0
            random_safe_finish_active_seen = 0
            random_safe_finish_hold_sum = 0
            random_safe_finish_low_priority_sum = 0
            random_safe_finish_blocked_sum = 0
            random_safe_finish_release_sum = 0
            random_goal_hold_start = float(getattr(args, 'random_goal_hold_weight', 0.0))
            random_goal_hold_end = (
                float(getattr(args, 'random_goal_hold_weight_end'))
                if getattr(args, 'random_goal_hold_weight_end', None) is not None
                else random_goal_hold_start
            )
            current_random_goal_hold_weight = random_goal_hold_start + (random_goal_hold_end - random_goal_hold_start) * progress_fraction
            last_random_goal_hold_loss = 0.0
            random_goal_hold_active_sum = 0
            random_goal_hold_active_seen = 0
            random_offroute_finish_start = float(getattr(args, 'random_offroute_finish_weight', 0.0))
            random_offroute_finish_end = (
                float(getattr(args, 'random_offroute_finish_weight_end'))
                if getattr(args, 'random_offroute_finish_weight_end', None) is not None
                else random_offroute_finish_start
            )
            current_random_offroute_finish_weight = random_offroute_finish_start + (random_offroute_finish_end - random_offroute_finish_start) * progress_fraction
            last_random_offroute_finish_loss = 0.0
            random_offroute_finish_active_sum = 0
            random_offroute_finish_active_seen = 0
            random_cte_recovery_start = float(getattr(args, 'random_cte_recovery_weight', 0.0))
            random_cte_recovery_end = (
                float(getattr(args, 'random_cte_recovery_weight_end'))
                if getattr(args, 'random_cte_recovery_weight_end', None) is not None
                else random_cte_recovery_start
            )
            current_random_cte_recovery_weight = random_cte_recovery_start + (random_cte_recovery_end - random_cte_recovery_start) * progress_fraction
            last_random_cte_recovery_loss = 0.0
            random_cte_recovery_active_sum = 0
            random_cte_recovery_active_seen = 0
            random_clear_ahead_start = float(getattr(args, 'random_clear_ahead_weight', 0.0))
            random_clear_ahead_end = (
                float(getattr(args, 'random_clear_ahead_weight_end'))
                if getattr(args, 'random_clear_ahead_weight_end', None) is not None
                else random_clear_ahead_start
            )
            current_random_clear_ahead_weight = random_clear_ahead_start + (random_clear_ahead_end - random_clear_ahead_start) * progress_fraction
            last_random_clear_ahead_loss = 0.0
            random_clear_ahead_active_sum = 0
            random_clear_ahead_active_seen = 0
            random_clear_ahead_scenario_sum = 0
            random_clear_ahead_unfinished_sum = 0
            random_clear_ahead_front_sum = 0
            random_clear_ahead_cpa_sum = 0
            random_clear_ahead_local_sum = 0
            random_clear_ahead_neighbor_sum = 0
            random_clear_ahead_progress_sum = 0
            random_clear_ahead_cte_sum = 0
            random_clear_ahead_role_sum = 0
            random_clear_ahead_deconflict_sum = 0
            random_late_lagging_start = float(getattr(args, 'random_late_lagging_weight', 0.0))
            random_late_lagging_end = (
                float(getattr(args, 'random_late_lagging_weight_end'))
                if getattr(args, 'random_late_lagging_weight_end', None) is not None
                else random_late_lagging_start
            )
            current_random_late_lagging_weight = random_late_lagging_start + (random_late_lagging_end - random_late_lagging_start) * progress_fraction
            last_random_late_lagging_loss = 0.0
            random_late_lagging_active_sum = 0
            random_late_lagging_active_seen = 0
            random_late_lagging_scenario_sum = 0
            random_late_lagging_unfinished_sum = 0
            random_late_lagging_lagging_sum = 0
            random_late_lagging_phase_sum = 0
            random_late_lagging_role_sum = 0
            random_late_lagging_finished_sum = 0
            random_late_lagging_team_sum = 0
            random_late_lagging_neighbor_sum = 0
            random_late_lagging_threat_sum = 0
            random_close_formation_escape_start = float(getattr(args, 'random_close_formation_escape_weight', 0.0))
            random_close_formation_escape_end = (
                float(getattr(args, 'random_close_formation_escape_weight_end'))
                if getattr(args, 'random_close_formation_escape_weight_end', None) is not None
                else random_close_formation_escape_start
            )
            current_random_close_formation_escape_weight = (
                random_close_formation_escape_start
                + (random_close_formation_escape_end - random_close_formation_escape_start) * progress_fraction
            )
            last_random_close_formation_escape_loss = 0.0
            random_close_formation_escape_active_sum = 0
            random_close_formation_escape_active_seen = 0
            current_policy_anchor_weight = policy_anchor_start + (policy_anchor_end - policy_anchor_start) * progress_fraction
            last_policy_anchor_loss = 0.0

            for _ in range(max(1, args.update_epochs)):
                permutation = torch.randperm(sample_count, device=device)
                for start in range(0, sample_count, minibatch_size):
                    batch_indices = permutation[start:start + minibatch_size]
                    batch_obs = flat_obs[batch_indices]
                    batch_raw_obs = flat_raw_obs[batch_indices]
                    batch_states = flat_states[batch_indices]
                    batch_actions = flat_actions[batch_indices]
                    batch_old_log_probs = flat_old_log_probs[batch_indices]
                    batch_advantages = flat_advantages[batch_indices]
                    batch_returns = flat_returns[batch_indices]
                    batch_scenario_ids = None
                    if flat_scenario_ids_np is not None:
                        batch_scenario_ids = torch.as_tensor(
                            flat_scenario_ids_np[batch_indices.detach().cpu().numpy()],
                            dtype=torch.long,
                            device=device,
                        )
                    batch_agent_indices = None
                    if flat_agent_indices_np is not None:
                        batch_agent_indices = torch.as_tensor(
                            flat_agent_indices_np[batch_indices.detach().cpu().numpy()],
                            dtype=torch.long,
                            device=device,
                        )
                    batch_weights = flat_scenario_weights[batch_indices] if flat_scenario_weights is not None else None

                    with autocast_context():
                        action_mean = _actor_forward(actor, batch_obs, batch_scenario_ids)
                        if getattr(args, 'squash_actions', False):
                            action_mean = action_mean.clamp(-3.0, 3.0)
                            _half = (action_high_tensor - action_low_tensor) / 2.0
                            _mid = (action_high_tensor + action_low_tensor) / 2.0
                            action_mean = torch.tanh(action_mean) * _half + _mid
                        action_std = actor_log_std.exp().expand_as(action_mean).to(dtype=action_mean.dtype)
                        distribution = Normal(action_mean, action_std)
                        new_log_probs = distribution.log_prob(batch_actions.to(dtype=action_mean.dtype)).sum(dim=-1)
                        entropy = distribution.entropy().sum(dim=-1).mean()

                        ratio = torch.exp(new_log_probs - batch_old_log_probs.to(dtype=new_log_probs.dtype))
                        batch_advantages_cast = batch_advantages.to(dtype=ratio.dtype)
                        surrogate_one = ratio * batch_advantages_cast
                        surrogate_two = torch.clamp(ratio, 1.0 - args.clip_range, 1.0 + args.clip_range) * batch_advantages_cast
                        clipped_surrogate = torch.min(surrogate_one, surrogate_two)

                        critic_values = critic(torch.cat([batch_obs, batch_states], dim=-1)).squeeze(-1)
                        value_delta = critic_values - batch_returns.to(dtype=critic_values.dtype)
                        if value_loss_huber_delta > 0.0:
                            abs_delta = value_delta.abs()
                            # Squared error for |err|<=delta, linear beyond. Value and
                            # slope are continuous at abs_delta=delta (quad x^2, linear
                            # delta*(2|x|-delta)), keeping small-error behavior identical
                            # to pure MSE so value_coef stays meaningful.
                            value_errors = torch.where(
                                abs_delta <= value_loss_huber_delta,
                                value_delta ** 2,
                                value_loss_huber_delta * (2.0 * abs_delta - value_loss_huber_delta),
                            )
                        else:
                            value_errors = value_delta ** 2

                        if batch_weights is not None:
                            actor_loss = -(clipped_surrogate * batch_weights).mean()
                            critic_loss = (value_errors * batch_weights).mean()
                        else:
                            actor_loss = -clipped_surrogate.mean()
                            critic_loss = value_errors.mean()

                        loss = (
                            ppo_policy_loss_scale * actor_loss
                            + ppo_value_loss_scale * args.value_coef * critic_loss
                            - current_entropy_coef * entropy
                        )
                        if current_crossing_imitation_weight > 0.0:
                            crossing_imitation_loss = _crossing_imitation_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                            )
                            loss = loss + current_crossing_imitation_weight * crossing_imitation_loss
                            last_crossing_imitation_loss = float(crossing_imitation_loss.detach().cpu().item())
                        if current_overtaking_imitation_weight > 0.0:
                            with torch.no_grad():
                                overtaking_imitation_mask = _overtaking_imitation_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if overtaking_imitation_mask.numel() > 0:
                                    overtaking_imitation_active_sum += int(overtaking_imitation_mask.sum().detach().cpu().item())
                                    overtaking_imitation_active_seen += int(overtaking_imitation_mask.numel())
                            overtaking_imitation_loss = _overtaking_imitation_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_overtaking_imitation_weight * overtaking_imitation_loss
                            last_overtaking_imitation_loss = float(overtaking_imitation_loss.detach().cpu().item())
                        if current_near_goal_finish_weight > 0.0:
                            near_goal_finish_loss = _near_goal_finish_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                            )
                            loss = loss + current_near_goal_finish_weight * near_goal_finish_loss
                            last_near_goal_finish_loss = float(near_goal_finish_loss.detach().cpu().item())
                        if current_lagging_finish_weight > 0.0:
                            with torch.no_grad():
                                lagging_active_mask = _lagging_finish_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_states,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if lagging_active_mask.numel() > 0:
                                    lagging_finish_active_sum += int(lagging_active_mask.sum().detach().cpu().item())
                                    lagging_finish_active_seen += int(lagging_active_mask.numel())
                            lagging_finish_loss = _lagging_teammate_finish_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_states,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_lagging_finish_weight * lagging_finish_loss
                            last_lagging_finish_loss = float(lagging_finish_loss.detach().cpu().item())
                        if current_team_safety_brake_weight > 0.0:
                            with torch.no_grad():
                                team_safety_brake_mask = _team_safety_brake_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_states,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if team_safety_brake_mask.numel() > 0:
                                    team_safety_brake_active_sum += int(team_safety_brake_mask.sum().detach().cpu().item())
                                    team_safety_brake_active_seen += int(team_safety_brake_mask.numel())
                            team_safety_brake_loss = _team_safety_brake_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_states,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_team_safety_brake_weight * team_safety_brake_loss
                            last_team_safety_brake_loss = float(team_safety_brake_loss.detach().cpu().item())
                        if current_random_deconflict_weight > 0.0:
                            with torch.no_grad():
                                random_deconflict_mask = _random_deconflict_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if random_deconflict_mask.numel() > 0:
                                    random_deconflict_active_sum += int(random_deconflict_mask.sum().detach().cpu().item())
                                    random_deconflict_active_seen += int(random_deconflict_mask.numel())
                            random_deconflict_loss = _random_deconflict_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_deconflict_weight * random_deconflict_loss
                            last_random_deconflict_loss = float(random_deconflict_loss.detach().cpu().item())
                        if current_random_role_balance_weight > 0.0:
                            with torch.no_grad():
                                random_role_balance_mask = _random_deconflict_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if random_role_balance_mask.numel() > 0:
                                    random_role_balance_active_sum += int(random_role_balance_mask.sum().detach().cpu().item())
                                    random_role_balance_active_seen += int(random_role_balance_mask.numel())
                            random_role_balance_loss = _random_role_balance_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_role_balance_weight * random_role_balance_loss
                            last_random_role_balance_loss = float(random_role_balance_loss.detach().cpu().item())
                        if current_random_pairwise_role_guard_weight > 0.0:
                            with torch.no_grad():
                                pairwise_role_guard_mask = _random_pairwise_role_guard_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if pairwise_role_guard_mask.numel() > 0:
                                    random_pairwise_role_guard_active_sum += int(pairwise_role_guard_mask.sum().detach().cpu().item())
                                    random_pairwise_role_guard_active_seen += int(pairwise_role_guard_mask.numel())
                            pairwise_role_guard_loss = _random_pairwise_role_guard_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_pairwise_role_guard_weight * pairwise_role_guard_loss
                            last_random_pairwise_role_guard_loss = float(pairwise_role_guard_loss.detach().cpu().item())
                        if current_random_safe_finish_weight > 0.0:
                            with torch.no_grad():
                                random_safe_finish_components = _random_safe_finish_components(
                                    torch,
                                    batch_raw_obs,
                                    batch_states,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                random_safe_finish_mask = random_safe_finish_components['active']
                                if random_safe_finish_mask.numel() > 0:
                                    random_safe_finish_active_sum += int(random_safe_finish_mask.sum().detach().cpu().item())
                                    random_safe_finish_active_seen += int(random_safe_finish_mask.numel())
                                    random_safe_finish_hold_sum += int(random_safe_finish_components['hold'].sum().detach().cpu().item())
                                    random_safe_finish_low_priority_sum += int(random_safe_finish_components['low_priority_active'].sum().detach().cpu().item())
                                    random_safe_finish_blocked_sum += int(random_safe_finish_components['deconflict_block'].sum().detach().cpu().item())
                                    random_safe_finish_release_sum += int(random_safe_finish_components['release_overlap'].sum().detach().cpu().item())
                            random_safe_finish_loss = _random_safe_finish_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_states,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_safe_finish_weight * random_safe_finish_loss
                            last_random_safe_finish_loss = float(random_safe_finish_loss.detach().cpu().item())
                        if current_random_goal_hold_weight > 0.0:
                            with torch.no_grad():
                                random_goal_hold_mask = _random_goal_hold_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                )
                                if random_goal_hold_mask.numel() > 0:
                                    random_goal_hold_active_sum += int(random_goal_hold_mask.sum().detach().cpu().item())
                                    random_goal_hold_active_seen += int(random_goal_hold_mask.numel())
                            random_goal_hold_loss = _random_goal_hold_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                            )
                            loss = loss + current_random_goal_hold_weight * random_goal_hold_loss
                            last_random_goal_hold_loss = float(random_goal_hold_loss.detach().cpu().item())
                        if current_random_offroute_finish_weight > 0.0:
                            with torch.no_grad():
                                random_offroute_finish_mask = _random_offroute_finish_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_states,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if random_offroute_finish_mask.numel() > 0:
                                    random_offroute_finish_active_sum += int(random_offroute_finish_mask.sum().detach().cpu().item())
                                    random_offroute_finish_active_seen += int(random_offroute_finish_mask.numel())
                            random_offroute_finish_loss = _random_offroute_finish_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_states,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_offroute_finish_weight * random_offroute_finish_loss
                            last_random_offroute_finish_loss = float(random_offroute_finish_loss.detach().cpu().item())
                        if current_random_cte_recovery_weight > 0.0:
                            with torch.no_grad():
                                random_cte_recovery_mask = _random_cte_recovery_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if random_cte_recovery_mask.numel() > 0:
                                    random_cte_recovery_active_sum += int(random_cte_recovery_mask.sum().detach().cpu().item())
                                    random_cte_recovery_active_seen += int(random_cte_recovery_mask.numel())
                            random_cte_recovery_loss = _random_cte_recovery_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_cte_recovery_weight * random_cte_recovery_loss
                            last_random_cte_recovery_loss = float(random_cte_recovery_loss.detach().cpu().item())
                        if current_random_late_lagging_weight > 0.0:
                            with torch.no_grad():
                                random_late_lagging_components = _random_late_lagging_component_masks(
                                    torch,
                                    batch_raw_obs,
                                    batch_states,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                random_late_lagging_mask = random_late_lagging_components['active']
                                if random_late_lagging_mask.numel() > 0:
                                    random_late_lagging_active_sum += int(random_late_lagging_mask.sum().detach().cpu().item())
                                    random_late_lagging_active_seen += int(random_late_lagging_mask.numel())
                                    random_late_lagging_scenario_sum += int(random_late_lagging_components['scenario'].sum().detach().cpu().item())
                                    random_late_lagging_unfinished_sum += int(random_late_lagging_components['unfinished'].sum().detach().cpu().item())
                                    random_late_lagging_lagging_sum += int(random_late_lagging_components['lagging'].sum().detach().cpu().item())
                                    random_late_lagging_phase_sum += int(random_late_lagging_components['phase'].sum().detach().cpu().item())
                                    random_late_lagging_role_sum += int(random_late_lagging_components['role'].sum().detach().cpu().item())
                                    random_late_lagging_finished_sum += int(random_late_lagging_components['finished'].sum().detach().cpu().item())
                                    random_late_lagging_team_sum += int(random_late_lagging_components['team'].sum().detach().cpu().item())
                                    random_late_lagging_neighbor_sum += int(random_late_lagging_components['neighbor'].sum().detach().cpu().item())
                                    random_late_lagging_threat_sum += int(random_late_lagging_components['threat'].sum().detach().cpu().item())
                            random_late_lagging_loss = _random_late_lagging_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_states,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_late_lagging_weight * random_late_lagging_loss
                            last_random_late_lagging_loss = float(random_late_lagging_loss.detach().cpu().item())
                        if current_random_clear_ahead_weight > 0.0:
                            with torch.no_grad():
                                random_clear_components = _random_clear_ahead_component_masks(
                                    torch,
                                    batch_raw_obs,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                random_clear_ahead_mask = random_clear_components['active']
                                if random_clear_ahead_mask.numel() > 0:
                                    random_clear_ahead_active_sum += int(random_clear_ahead_mask.sum().detach().cpu().item())
                                    random_clear_ahead_active_seen += int(random_clear_ahead_mask.numel())
                                    random_clear_ahead_scenario_sum += int(random_clear_components['scenario'].sum().detach().cpu().item())
                                    random_clear_ahead_unfinished_sum += int(random_clear_components['unfinished'].sum().detach().cpu().item())
                                    random_clear_ahead_front_sum += int(random_clear_components['front'].sum().detach().cpu().item())
                                    random_clear_ahead_cpa_sum += int(random_clear_components['cpa'].sum().detach().cpu().item())
                                    random_clear_ahead_local_sum += int(random_clear_components['local'].sum().detach().cpu().item())
                                    random_clear_ahead_neighbor_sum += int(random_clear_components['neighbor'].sum().detach().cpu().item())
                                    random_clear_ahead_progress_sum += int(random_clear_components['progress'].sum().detach().cpu().item())
                                    random_clear_ahead_cte_sum += int(random_clear_components['cte'].sum().detach().cpu().item())
                                    random_clear_ahead_role_sum += int(random_clear_components['role'].sum().detach().cpu().item())
                                    random_clear_ahead_deconflict_sum += int(random_clear_components['deconflict'].sum().detach().cpu().item())
                            random_clear_ahead_loss = _random_clear_ahead_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_clear_ahead_weight * random_clear_ahead_loss
                            last_random_clear_ahead_loss = float(random_clear_ahead_loss.detach().cpu().item())
                        if current_random_close_formation_escape_weight > 0.0:
                            with torch.no_grad():
                                random_close_formation_escape_mask = _random_close_formation_escape_active_mask(
                                    torch,
                                    batch_raw_obs,
                                    batch_scenario_ids,
                                    scenario_to_index,
                                    args,
                                    agent_indices=batch_agent_indices,
                                )
                                if random_close_formation_escape_mask.numel() > 0:
                                    random_close_formation_escape_active_sum += int(random_close_formation_escape_mask.sum().detach().cpu().item())
                                    random_close_formation_escape_active_seen += int(random_close_formation_escape_mask.numel())
                            random_close_formation_escape_loss = _random_close_formation_escape_loss(
                                torch,
                                action_mean,
                                batch_raw_obs,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_random_close_formation_escape_weight * random_close_formation_escape_loss
                            last_random_close_formation_escape_loss = float(random_close_formation_escape_loss.detach().cpu().item())
                        if policy_anchor_actor is not None and current_policy_anchor_weight > 0.0:
                            with torch.no_grad():
                                anchor_action_mean = _actor_forward(policy_anchor_actor, batch_obs, batch_scenario_ids)
                                if getattr(args, 'squash_actions', False):
                                    anchor_action_mean = anchor_action_mean.clamp(-3.0, 3.0)
                                    _half = (action_high_tensor - action_low_tensor) / 2.0
                                    _mid = (action_high_tensor + action_low_tensor) / 2.0
                                    anchor_action_mean = torch.tanh(anchor_action_mean) * _half + _mid
                            policy_anchor_loss = _policy_anchor_loss(
                                torch,
                                action_mean,
                                anchor_action_mean,
                                batch_raw_obs,
                                batch_states,
                                batch_scenario_ids,
                                scenario_to_index,
                                args,
                                action_low_tensor,
                                action_high_tensor,
                                sample_weights=batch_weights,
                                agent_indices=batch_agent_indices,
                            )
                            loss = loss + current_policy_anchor_weight * policy_anchor_loss
                            last_policy_anchor_loss = float(policy_anchor_loss.detach().cpu().item())

                    optimizer.zero_grad(set_to_none=True)
                    if amp_enabled:
                        scaler.scale(loss).backward()
                        scaler.unscale_(optimizer)
                        _clip_actor_critic_gradients(
                            torch,
                            actor,
                            critic,
                            actor_log_std,
                            args.max_grad_norm,
                            separate=bool(getattr(args, 'separate_actor_critic_grad_clip', False)),
                        )
                        scaler.step(optimizer)
                        scaler.update()
                    else:
                        loss.backward()
                        _clip_actor_critic_gradients(
                            torch,
                            actor,
                            critic,
                            actor_log_std,
                            args.max_grad_norm,
                            separate=bool(getattr(args, 'separate_actor_critic_grad_clip', False)),
                        )
                        optimizer.step()

            last_crossing_pretrain_loss = _run_crossing_imitation_pretrain(
                torch,
                actor,
                optimizer,
                flat_obs,
                flat_raw_obs,
                flat_scenario_ids_np,
                flat_scenario_weights,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_count=sample_count,
                minibatch_size=minibatch_size,
                device=device,
                imitation_weight=current_crossing_imitation_weight,
            )
            last_random_deconflict_pretrain_loss = _run_random_deconflict_pretrain(
                torch,
                actor,
                optimizer,
                flat_obs,
                flat_raw_obs,
                flat_scenario_ids_np,
                flat_scenario_weights,
                flat_agent_indices_np,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_count=sample_count,
                minibatch_size=minibatch_size,
                device=device,
                deconflict_weight=current_random_deconflict_weight,
            )
            last_random_role_balance_pretrain_loss = _run_random_role_balance_pretrain(
                torch,
                actor,
                optimizer,
                flat_obs,
                flat_raw_obs,
                flat_scenario_ids_np,
                flat_scenario_weights,
                flat_agent_indices_np,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_count=sample_count,
                minibatch_size=minibatch_size,
                device=device,
                role_balance_weight=current_random_role_balance_weight,
            )
            last_random_pairwise_role_guard_pretrain_loss = _run_random_pairwise_role_guard_pretrain(
                torch,
                actor,
                optimizer,
                flat_obs,
                flat_raw_obs,
                flat_scenario_ids_np,
                flat_scenario_weights,
                flat_agent_indices_np,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_count=sample_count,
                minibatch_size=minibatch_size,
                device=device,
                pairwise_role_guard_weight=current_random_pairwise_role_guard_weight,
            )
            last_random_recovery_pretrain_loss = _run_random_recovery_pretrain(
                torch,
                actor,
                optimizer,
                flat_obs,
                flat_raw_obs,
                flat_states,
                flat_scenario_ids_np,
                flat_scenario_weights,
                flat_agent_indices_np,
                scenario_to_index,
                args,
                action_low_tensor,
                action_high_tensor,
                sample_count=sample_count,
                minibatch_size=minibatch_size,
                device=device,
                offroute_weight=current_random_offroute_finish_weight,
                cte_weight=current_random_cte_recovery_weight,
                clear_ahead_weight=current_random_clear_ahead_weight,
                late_lagging_weight=current_random_late_lagging_weight,
            )

            update_wall_time = max(1e-6, time.perf_counter() - update_wall_start)
            update_index += 1
            if update_index % max(1, int(args.log_interval_updates)) == 0:
                optimizer_samples = int(sample_count * max(1, args.update_epochs))
                rollout_sps = rollout_agent_steps / rollout_wall_time
                optimizer_sps = optimizer_samples / update_wall_time
                loop_time = rollout_wall_time + update_wall_time
                update_share = update_wall_time / max(loop_time, 1e-6)
                scenario_mix = _scenario_mix_label(flat_scenario_ids_np, tuple(scenarios))
                print(
                    f'[MAPPO][update={update_index}] total_steps={total_steps} '
                    f'rollout_steps={rollout_steps_collected} sample_count={sample_count} '
                    f'scenarios={scenario_mix} '
                    f'rollout_time={rollout_wall_time:.2f}s rollout_sps={rollout_sps:.1f} '
                    f'update_time={update_wall_time:.2f}s update_sps={optimizer_sps:.1f} '
                    f'update_share={update_share:.2%} loss={float(loss.detach().cpu().item()):.4f} '
                    f'mean_reward={rollout_mean_reward:.4f} episodes={rollout_episode_count} '
                    f'entropy_coef={current_entropy_coef:.4f} lr={current_lr:.2e} '
                    f'ppo_pi={ppo_policy_loss_scale:.3f} ppo_v={ppo_value_loss_scale:.3f} '
                    f'crossing_bc_w={current_crossing_imitation_weight:.3f} crossing_bc={last_crossing_imitation_loss:.4f} '
                    f'overtake_bc_w={current_overtaking_imitation_weight:.3f} overtake_bc={last_overtaking_imitation_loss:.4f} '
                    f'overtake_active={(overtaking_imitation_active_sum / max(overtaking_imitation_active_seen, 1)):.3f} '
                    f'finish_bc_w={current_near_goal_finish_weight:.3f} finish_bc={last_near_goal_finish_loss:.4f} '
                    f'lag_finish_w={current_lagging_finish_weight:.3f} lag_finish={last_lagging_finish_loss:.4f} '
                    f'lag_active={(lagging_finish_active_sum / max(lagging_finish_active_seen, 1)):.3f} '
                    f'team_brake_w={current_team_safety_brake_weight:.3f} team_brake={last_team_safety_brake_loss:.4f} '
                    f'team_brake_active={(team_safety_brake_active_sum / max(team_safety_brake_active_seen, 1)):.3f} '
                    f'rand_deconf_w={current_random_deconflict_weight:.3f} rand_deconf={last_random_deconflict_loss:.4f} '
                    f'rand_deconf_active={(random_deconflict_active_sum / max(random_deconflict_active_seen, 1)):.3f} '
                    f'rand_role_w={current_random_role_balance_weight:.3f} rand_role={last_random_role_balance_loss:.4f} '
                    f'rand_role_active={(random_role_balance_active_sum / max(random_role_balance_active_seen, 1)):.3f} '
                    f'rand_pair_w={current_random_pairwise_role_guard_weight:.3f} rand_pair={last_random_pairwise_role_guard_loss:.4f} '
                    f'rand_pair_active={(random_pairwise_role_guard_active_sum / max(random_pairwise_role_guard_active_seen, 1)):.3f} '
                    f'rand_finish_w={current_random_safe_finish_weight:.3f} rand_finish={last_random_safe_finish_loss:.4f} '
                    f'rand_finish_active={(random_safe_finish_active_sum / max(random_safe_finish_active_seen, 1)):.3f} '
                    f'rand_finish_hold={(random_safe_finish_hold_sum / max(random_safe_finish_active_seen, 1)):.3f} '
                    f'rand_finish_lowprio={(random_safe_finish_low_priority_sum / max(random_safe_finish_active_seen, 1)):.3f} '
                    f'rand_finish_release={(random_safe_finish_release_sum / max(random_safe_finish_active_seen, 1)):.3f} '
                    f'rand_finish_block={(random_safe_finish_blocked_sum / max(random_safe_finish_active_seen, 1)):.3f} '
                    f'rand_goal_hold_w={current_random_goal_hold_weight:.3f} rand_goal_hold={last_random_goal_hold_loss:.4f} '
                    f'rand_goal_hold_active={(random_goal_hold_active_sum / max(random_goal_hold_active_seen, 1)):.3f} '
                    f'rand_offroute_w={current_random_offroute_finish_weight:.3f} rand_offroute={last_random_offroute_finish_loss:.4f} '
                    f'rand_offroute_active={(random_offroute_finish_active_sum / max(random_offroute_finish_active_seen, 1)):.3f} '
                    f'rand_cte_w={current_random_cte_recovery_weight:.3f} rand_cte={last_random_cte_recovery_loss:.4f} '
                    f'rand_cte_active={(random_cte_recovery_active_sum / max(random_cte_recovery_active_seen, 1)):.3f} '
                    f'rand_recovery_gate={float(getattr(args, "random_recovery_safety_gate_scale", 1.0)):.3f}/{str(getattr(args, "random_recovery_safety_gate_mode", "batch"))} '
                    f'rand_late_lag_w={current_random_late_lagging_weight:.3f} rand_late_lag={last_random_late_lagging_loss:.4f} '
                    f'rand_late_lag_active={(random_late_lagging_active_sum / max(random_late_lagging_active_seen, 1)):.3f} '
                    f'rand_late_lag_gates='
                    f'{(random_late_lagging_scenario_sum / max(random_late_lagging_active_seen, 1)):.3f}/'
                    f'{(random_late_lagging_unfinished_sum / max(random_late_lagging_active_seen, 1)):.3f}/'
                    f'{(random_late_lagging_lagging_sum / max(random_late_lagging_active_seen, 1)):.3f}/'
                    f'{(random_late_lagging_phase_sum / max(random_late_lagging_active_seen, 1)):.3f}/'
                    f'{(random_late_lagging_team_sum / max(random_late_lagging_active_seen, 1)):.3f}/'
                    f'{(random_late_lagging_neighbor_sum / max(random_late_lagging_active_seen, 1)):.3f}/'
                    f'{(random_late_lagging_threat_sum / max(random_late_lagging_active_seen, 1)):.3f} '
                    f'rand_late_lag_focus=' \
                    f'{(random_late_lagging_role_sum / max(random_late_lagging_active_seen, 1)):.3f}/' \
                    f'{(random_late_lagging_finished_sum / max(random_late_lagging_active_seen, 1)):.3f} '
                    f'rand_clear_w={current_random_clear_ahead_weight:.3f} rand_clear={last_random_clear_ahead_loss:.4f} '
                    f'rand_clear_active={(random_clear_ahead_active_sum / max(random_clear_ahead_active_seen, 1)):.3f} '
                    f'rand_clear_gates='
                    f'{(random_clear_ahead_scenario_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_unfinished_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_front_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_cpa_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_local_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_neighbor_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_progress_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_cte_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_role_sum / max(random_clear_ahead_active_seen, 1)):.3f}/'
                    f'{(random_clear_ahead_deconflict_sum / max(random_clear_ahead_active_seen, 1)):.3f} '
                    f'rand_cfe_w={current_random_close_formation_escape_weight:.3f} rand_cfe={last_random_close_formation_escape_loss:.4f} '
                    f'rand_cfe_active={(random_close_formation_escape_active_sum / max(random_close_formation_escape_active_seen, 1)):.3f} '
                    f'anchor_w={current_policy_anchor_weight:.3f} anchor={last_policy_anchor_loss:.4f} '
                    f'crossing_bc_pre={last_crossing_pretrain_loss:.4f} '
                    f'rand_deconf_pre={last_random_deconflict_pretrain_loss:.4f} '
                    f'rand_role_pre={last_random_role_balance_pretrain_loss:.4f} '
                    f'rand_pair_pre={last_random_pairwise_role_guard_pretrain_loss:.4f} '
                    f'rand_recovery_pre={last_random_recovery_pretrain_loss:.4f} '
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
                        obs_normalizer=obs_normalizer,
                    ),
                )
                _evaluate_checkpoint_immediately(args, checkpoint_path, scenarios, output_path)
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
                obs_normalizer=obs_normalizer,
            ),
        )
        if checkpoint_dir is not None:
            _write_checkpoint_ranking_summary(args, scenarios, output_path)
    finally:
        if sampler is not None:
            sampler.close()
        if env is not None:
            env.close()


if __name__ == '__main__':
    main()