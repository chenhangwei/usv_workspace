import numpy as np

from .config import RewardConfig


class RuntimeActionContract:
    def __init__(self, checkpoint, max_linear_speed):
        self.action_mode = str(checkpoint.get('action_mode', 'full'))
        if self.action_mode not in ('full', 'speed_scale'):
            raise ValueError(f'Unsupported runtime action_mode: {self.action_mode}')
        self.scale_min = float(np.clip(checkpoint.get('action_speed_scale_min', 0.25), 0.0, 1.0))
        self.cruise_speed = max(0.18, float(max_linear_speed))
        self.goal_tolerance = float(checkpoint.get('goal_tolerance', 0.8))
        self.relief_distance = float(checkpoint.get('goal_proximity_relief_distance', 3.0))
        self.speed_relief = float(checkpoint.get('goal_proximity_speed_relief', 0.0))
        reward = checkpoint.get('reward_config', {}) or {}
        defaults = RewardConfig()
        desired_speed = float(reward.get('desired_conflict_speed', defaults.desired_conflict_speed))
        speed_floor = float(reward.get('conflict_speed_floor', defaults.conflict_speed_floor))
        self.minimum_speed = max(0.08, min(desired_speed, speed_floor))

    def expand(self, action, projected_action, distance_to_goal):
        expanded = np.asarray(projected_action, dtype=np.float32).copy()
        if self.action_mode == 'full':
            return expanded
        scale = float(np.clip(action[0], self.scale_min, 1.0))
        relief_distance = max(self.goal_tolerance + 1e-3, self.relief_distance)
        proximity = float(np.clip(
            (relief_distance - float(distance_to_goal))
            / max(relief_distance - self.goal_tolerance, 1e-3), 0.0, 1.0,
        ))
        nav_speed = self.cruise_speed * max(
            0.25, 1.0 - float(np.clip(self.speed_relief, 0.0, 1.0)) * proximity,
        )
        nav_speed = float(np.clip(nav_speed, self.minimum_speed, self.cruise_speed))
        expanded[0] = scale * nav_speed
        return expanded