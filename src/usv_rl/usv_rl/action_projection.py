from __future__ import annotations

from typing import Optional

import numpy as np


def project_residual_action(
    action,
    *,
    action_mode: str,
    linear_delta_limit: float,
    angular_delta_limit: float,
    raw_linear_x: Optional[float] = None,
    forward_only: bool = True,
) -> np.ndarray:
    raw_action = np.asarray(action, dtype=np.float32).reshape(-1)

    if action_mode == 'angular_only':
        if raw_action.size >= 2:
            angular_delta = float(raw_action[1])
        elif raw_action.size == 1:
            angular_delta = float(raw_action[0])
        else:
            angular_delta = 0.0
        angular_delta = float(np.clip(angular_delta, -angular_delta_limit, angular_delta_limit))
        return np.asarray([angular_delta], dtype=np.float32)

    linear_delta = float(raw_action[0]) if raw_action.size >= 1 else 0.0
    angular_delta = float(raw_action[1]) if raw_action.size >= 2 else 0.0

    min_linear_delta = -float(linear_delta_limit)
    if forward_only and raw_linear_x is not None:
        min_linear_delta = max(min_linear_delta, -max(0.0, float(raw_linear_x)))

    linear_delta = float(np.clip(linear_delta, min_linear_delta, linear_delta_limit))
    angular_delta = float(np.clip(angular_delta, -angular_delta_limit, angular_delta_limit))
    return np.asarray([linear_delta, angular_delta], dtype=np.float32)