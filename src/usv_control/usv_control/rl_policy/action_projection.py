from __future__ import annotations

import math

import numpy as np


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _resolve_turn_authority(
    *,
    heading_error: float,
    conflict_level: float,
    heading_omega_deadband: float,
    heading_omega_reference: float,
    angular_authority_power: float,
    conflict_turn_relief: float,
    authority_floor: float = 0.35,
) -> tuple[float, float]:
    """Resolve angular turn authority.

    L1-1 fix: Introduce ``authority_floor`` so angular authority never
    collapses to zero even when heading error is small and no conflict is
    active.  Previously ``authority = max(heading_ratio, relief*conflict)``
    would drop to ~0 near the goal (heading aligned, no neighbour), making
    fine yaw corrections impossible and causing the "hover near goal"
    exploit.  A floor of ~0.35 preserves ~35% of max omega capacity at all
    times, giving the policy stable fine control.
    """
    abs_heading_error = abs(float(heading_error))
    deadband = max(0.0, float(heading_omega_deadband))
    reference = max(deadband + 1e-3, float(heading_omega_reference))
    raw_heading_ratio = _clamp((abs_heading_error - deadband) / (reference - deadband), 0.0, 1.0)
    relief_authority = _clamp(float(conflict_turn_relief), 0.0, 1.0) * _clamp(float(conflict_level), 0.0, 1.0)
    floor = _clamp(float(authority_floor), 0.0, 1.0)
    base_authority = max(raw_heading_ratio, relief_authority, floor)
    authority = base_authority ** max(1e-3, float(angular_authority_power))
    return raw_heading_ratio, _clamp(authority, 0.0, 1.0)


def project_pure_action(
    action,
    *,
    max_linear_speed: float,
    max_angular_speed: float,
    heading_error: float | None = None,
    current_angular_z: float | None = None,
    control_dt: float | None = None,
    conflict_level: float = 0.0,
    heading_omega_deadband: float = 0.06,
    heading_omega_reference: float = 0.85,
    angular_authority_power: float = 1.6,
    angular_accel_limit: float = 1.8,
    angular_decel_limit: float = 2.4,
    conflict_turn_relief: float = 0.55,
    forward_only: bool = True,
    authority_floor: float = 0.35,
    min_forward_speed_floor: float = 0.0,
) -> np.ndarray:
    raw_action = np.asarray(action, dtype=np.float32).reshape(-1)
    if raw_action.size < 2:
        raise ValueError('Pure RL control requires a 2D action: [linear_x, angular_z].')

    min_linear = 0.0 if forward_only else -float(max_linear_speed)
    linear_x = float(np.clip(raw_action[0], min_linear, float(max_linear_speed)))

    # L1-1 fix: Linear-speed floor. When the policy asks for any non-trivial
    # forward motion (raw > 0.02 m/s) AND no strong braking is requested,
    # enforce a minimum forward speed so the agent cannot freeze at vx=0
    # near the goal.  This eliminates the reward-shaping fight around
    # "near_goal_idle_penalty" by handling the problem at the actuator.
    floor_min = max(0.0, float(min_forward_speed_floor))
    if forward_only and floor_min > 0.0 and raw_action[0] > 0.02:
        if linear_x < floor_min:
            linear_x = floor_min

    raw_angular_z = float(raw_action[1])
    if heading_error is None or current_angular_z is None or control_dt is None:
        angular_z = float(np.clip(raw_angular_z, -float(max_angular_speed), float(max_angular_speed)))
    else:
        heading_ratio, authority = _resolve_turn_authority(
            heading_error=float(heading_error),
            conflict_level=float(conflict_level),
            heading_omega_deadband=heading_omega_deadband,
            heading_omega_reference=heading_omega_reference,
            angular_authority_power=angular_authority_power,
            conflict_turn_relief=conflict_turn_relief,
            authority_floor=authority_floor,
        )
        angular_cap = float(max_angular_speed) * authority
        target_angular_z = 0.0 if angular_cap <= 1e-4 else float(np.clip(raw_angular_z, -angular_cap, angular_cap))

        current_angular_z = float(np.clip(current_angular_z, -float(max_angular_speed), float(max_angular_speed)))
        dt = max(1e-3, float(control_dt))
        drive_ratio = max(heading_ratio, _clamp(float(conflict_level), 0.0, 1.0), float(authority_floor))
        accel_step = max(1e-4, float(angular_accel_limit) * dt * (0.35 + 0.65 * math.sqrt(drive_ratio)))
        decel_step = max(1e-4, float(angular_decel_limit) * dt * (0.45 + 0.55 * math.sqrt(drive_ratio)))
        building_turn = current_angular_z * target_angular_z >= 0.0 and abs(target_angular_z) > abs(current_angular_z)
        step_limit = accel_step if building_turn else decel_step
        angular_delta = float(np.clip(target_angular_z - current_angular_z, -step_limit, step_limit))
        angular_z = float(np.clip(current_angular_z + angular_delta, -float(max_angular_speed), float(max_angular_speed)))

        if authority <= 1e-4 and abs(angular_z) <= decel_step:
            angular_z = 0.0

    return np.asarray([linear_x, angular_z], dtype=np.float32)


def project_policy_action(
    action,
    *,
    rl_control_mode: str = 'pure',
    action_mode: str,
    linear_delta_limit: float,
    angular_delta_limit: float,
    raw_linear_x=None,
    heading_error: float | None = None,
    current_angular_z: float | None = None,
    control_dt: float | None = None,
    conflict_level: float = 0.0,
    heading_omega_deadband: float = 0.06,
    heading_omega_reference: float = 0.85,
    angular_authority_power: float = 1.6,
    angular_accel_limit: float = 1.8,
    angular_decel_limit: float = 2.4,
    conflict_turn_relief: float = 0.55,
    forward_only: bool = True,
    authority_floor: float = 0.35,
    min_forward_speed_floor: float = 0.0,
) -> np.ndarray:
    if action_mode != 'full':
        raise ValueError('Pure RL control requires action_mode="full".')
    return project_pure_action(
        action,
        max_linear_speed=linear_delta_limit,
        max_angular_speed=angular_delta_limit,
        heading_error=heading_error,
        current_angular_z=current_angular_z,
        control_dt=control_dt,
        conflict_level=conflict_level,
        heading_omega_deadband=heading_omega_deadband,
        heading_omega_reference=heading_omega_reference,
        angular_authority_power=angular_authority_power,
        angular_accel_limit=angular_accel_limit,
        angular_decel_limit=angular_decel_limit,
        conflict_turn_relief=conflict_turn_relief,
        forward_only=forward_only,
        authority_floor=authority_floor,
        min_forward_speed_floor=min_forward_speed_floor,
    )