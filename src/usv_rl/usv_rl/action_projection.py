from __future__ import annotations

import numpy as np

def project_pure_action(
    action,
    *,
    max_linear_speed: float,
    max_angular_speed: float,
    forward_only: bool = True,
) -> np.ndarray:
    raw_action = np.asarray(action, dtype=np.float32).reshape(-1)
    if raw_action.size < 2:
        raise ValueError('Pure RL control requires a 2D action: [linear_x, angular_z].')

    min_linear = 0.0 if forward_only else -float(max_linear_speed)
    linear_x = float(np.clip(raw_action[0], min_linear, float(max_linear_speed)))
    angular_z = float(np.clip(raw_action[1], -float(max_angular_speed), float(max_angular_speed)))
    return np.asarray([linear_x, angular_z], dtype=np.float32)


def project_policy_action(
    action,
    *,
    rl_control_mode: str = 'pure',
    action_mode: str,
    linear_delta_limit: float,
    angular_delta_limit: float,
    raw_linear_x=None,
    forward_only: bool = True,
) -> np.ndarray:
    if action_mode != 'full':
        raise ValueError('Pure RL control requires action_mode="full".')
    return project_pure_action(
        action,
        max_linear_speed=linear_delta_limit,
        max_angular_speed=angular_delta_limit,
        forward_only=forward_only,
    )