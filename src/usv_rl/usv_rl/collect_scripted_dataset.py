import argparse
import math
from pathlib import Path

import numpy as np

from .config import EnvConfig
from .env import UsvRlEnv
from .scenarios import ScenarioFactory


def parse_args():
    parser = argparse.ArgumentParser(description='Collect a scripted residual teacher dataset from the lightweight USV RL environment.')
    parser.add_argument('--output', required=True, help='Output .npz file path.')
    parser.add_argument('--episodes', type=int, default=18, help='Number of episodes to collect.')
    parser.add_argument('--steps-per-episode', type=int, default=240, help='Max control steps per episode.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name to include. Repeatable.')
    parser.add_argument(
        '--teacher-profile',
        choices=['safety_first_v2', 'safety_anchor_v3', 'headon_pass_v1', 'cluster_colregs_v1', 'cluster_colregs_v2'],
        default='cluster_colregs_v1',
        help='Scripted teacher heuristic profile.',
    )
    parser.add_argument('--external-stack', action='store_true', help='Use an already-running SITL stack.')
    parser.add_argument('--action-mode', choices=['full', 'angular_only'], default='angular_only', help='Residual action representation used for dataset collection.')
    parser.add_argument('--max-neighbors', type=int, default=4, help='Number of neighbors encoded for retraining. Use 4 for 5-USV standard scenarios.')
    return parser.parse_args()


_CLUSTER_CONFLICT_DISTANCE = float(EnvConfig().reward.conflict_distance)
_CLUSTER_ANTICIPATION_DISTANCE = max(5.0, float(EnvConfig().reward.anticipation_distance))
_CLUSTER_HEAD_ON_COMMIT_DISTANCE = max(5.8, _CLUSTER_ANTICIPATION_DISTANCE + 0.8)
_CLUSTER_LOOKAHEAD_DISTANCE = max(_CLUSTER_ANTICIPATION_DISTANCE + 1.0, _CLUSTER_CONFLICT_DISTANCE + 2.0)
_CLUSTER_MIN_SEPARATION = float(EnvConfig().collision_distance)
_CLUSTER_EMERGENCY_BUFFER = max(1.2, _CLUSTER_MIN_SEPARATION + 0.7)
_CLUSTER_HEAD_ON_TARGET_STARBOARD_OFFSET = 1.1
_CLUSTER_HEAD_ON_RELEASE_OFFSET = 1.5
_CLUSTER_V2_HEAD_ON_COMMIT_DISTANCE = max(6.4, _CLUSTER_HEAD_ON_COMMIT_DISTANCE + 0.6)
_CLUSTER_V2_HEAD_ON_TARGET_STARBOARD_OFFSET = 1.45
_CLUSTER_V2_HEAD_ON_RELEASE_OFFSET = 1.95


def _clip(value: float, lower: float, upper: float) -> float:
    return float(max(lower, min(upper, value)))


def _rotate_world_into_body(x_value: float, y_value: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x_value + sin_yaw * y_value,
        -sin_yaw * x_value + cos_yaw * y_value,
    )


def _iter_neighbor_features(observation: np.ndarray):
    yaw = float(observation[2]) if observation.shape[0] >= 3 else 0.0
    own_speed = float(observation[3]) if observation.shape[0] >= 4 else 0.0

    for offset in range(10, observation.shape[0], 6):
        if offset + 6 > observation.shape[0]:
            break

        rel_x, rel_y, rel_vx, rel_vy, distance, bearing = observation[offset:offset + 6]
        distance = float(distance)
        if distance <= 1e-6:
            continue

        rel_x = float(rel_x)
        rel_y = float(rel_y)
        rel_vx = float(rel_vx)
        rel_vy = float(rel_vy)
        bearing = float(bearing)

        body_x, body_y = _rotate_world_into_body(rel_x, rel_y, yaw)
        body_vx, body_vy = _rotate_world_into_body(rel_vx, rel_vy, yaw)
        closing_speed = -((body_x * body_vx) + (body_y * body_vy)) / max(distance, 1e-3)
        neighbor_forward_speed = own_speed + body_vx

        yield {
            'rel_x': rel_x,
            'rel_y': rel_y,
            'rel_vx': rel_vx,
            'rel_vy': rel_vy,
            'distance': distance,
            'bearing': bearing,
            'body_x': body_x,
            'body_y': body_y,
            'body_vx': body_vx,
            'body_vy': body_vy,
            'closing_speed': closing_speed,
            'neighbor_forward_speed': neighbor_forward_speed,
            'own_speed': own_speed,
        }


def _scripted_residual_action_cluster_colregs_v1(observation: np.ndarray) -> np.ndarray:
    action = np.zeros(2, dtype=np.float32)
    if observation.shape[0] < 16:
        return action

    raw_angular_z = float(observation[7]) if observation.shape[0] >= 8 else 0.0
    raw_linear_x = float(observation[6]) if observation.shape[0] >= 7 else 0.0
    total_starboard_turn = 0.0
    emergency_starboard_turn = 0.0
    proactive_starboard_turn = 0.0
    total_linear_slowdown = 0.0
    emergency_linear_slowdown = 0.0
    proactive_linear_slowdown = 0.0
    head_on_commit_active = False
    head_on_commit_floor = 0.0

    for neighbor in _iter_neighbor_features(observation):
        distance = neighbor['distance']
        if distance > _CLUSTER_LOOKAHEAD_DISTANCE:
            continue

        body_x = neighbor['body_x']
        body_y = neighbor['body_y']
        body_vx = neighbor['body_vx']
        closing_speed = neighbor['closing_speed']
        own_speed = neighbor['own_speed']
        neighbor_forward_speed = neighbor['neighbor_forward_speed']
        ahead = body_x > 0.0
        in_conflict_window = distance <= _CLUSTER_CONFLICT_DISTANCE
        in_anticipation_window = distance <= _CLUSTER_ANTICIPATION_DISTANCE
        anticipation_progress = _clip(
            (_CLUSTER_ANTICIPATION_DISTANCE - distance) / max(_CLUSTER_ANTICIPATION_DISTANCE - _CLUSTER_CONFLICT_DISTANCE, 1e-3),
            0.0,
            1.0,
        )
        head_on_commit_progress = _clip(
            (_CLUSTER_HEAD_ON_COMMIT_DISTANCE - distance) / max(_CLUSTER_HEAD_ON_COMMIT_DISTANCE - _CLUSTER_CONFLICT_DISTANCE, 1e-3),
            0.0,
            1.0,
        )
        proximity = _clip((_CLUSTER_LOOKAHEAD_DISTANCE - distance) / max(_CLUSTER_LOOKAHEAD_DISTANCE - _CLUSTER_CONFLICT_DISTANCE, 1e-3), 0.0, 1.0)
        close_quarters = _clip((_CLUSTER_CONFLICT_DISTANCE - distance) / max(_CLUSTER_CONFLICT_DISTANCE, 1e-3), 0.0, 1.0)
        separation_pressure = _clip(
            (_CLUSTER_EMERGENCY_BUFFER - distance) / max(_CLUSTER_EMERGENCY_BUFFER - _CLUSTER_MIN_SEPARATION, 1e-3),
            0.0,
            1.0,
        )
        closing_weight = _clip((closing_speed + 0.15) / 0.9, 0.0, 1.0)

        same_lane_ahead = ahead and abs(body_y) < 1.5 and body_x > 0.8
        overtaking_target = same_lane_ahead and own_speed > 0.22 and body_vx < -0.03 and neighbor_forward_speed < own_speed - 0.03
        head_on = ahead and abs(body_y) < max(1.3, 0.28 * distance) and closing_speed > 0.02
        starboard_crossing = ahead and body_y < -0.35 and closing_speed > -0.05
        port_crossing = ahead and body_y > 0.45 and closing_speed > -0.02
        head_on_corridor_deficit = _clip(
            (_CLUSTER_HEAD_ON_TARGET_STARBOARD_OFFSET + body_y) / _CLUSTER_HEAD_ON_TARGET_STARBOARD_OFFSET,
            0.0,
            1.0,
        )
        head_on_release_progress = _clip(
            (_CLUSTER_HEAD_ON_RELEASE_OFFSET + body_y) / (_CLUSTER_HEAD_ON_RELEASE_OFFSET - _CLUSTER_HEAD_ON_TARGET_STARBOARD_OFFSET),
            0.0,
            1.0,
        )

        if distance <= _CLUSTER_EMERGENCY_BUFFER and body_x > -0.5 and closing_speed > -0.05:
            emergency_starboard_turn = max(
                emergency_starboard_turn,
                0.10 + 0.22 * separation_pressure + 0.10 * closing_weight,
            )

        if in_anticipation_window and ahead and closing_speed > -0.03:
            proactive_starboard_turn = max(
                proactive_starboard_turn,
                0.03 + 0.10 * anticipation_progress,
            )

        if head_on:
            head_on_commit_active = True
            head_on_commit_floor = max(
                head_on_commit_floor,
                0.16 + 0.08 * head_on_corridor_deficit,
            )
            proactive_starboard_turn = max(
                proactive_starboard_turn,
                0.10 + 0.14 * head_on_commit_progress + 0.10 * closing_weight + 0.10 * head_on_corridor_deficit,
            )
            proactive_linear_slowdown = max(
                proactive_linear_slowdown,
                0.05 + 0.06 * head_on_commit_progress + 0.05 * head_on_corridor_deficit,
            )
            total_starboard_turn += 0.14 + 0.16 * proximity + 0.08 * closing_weight + 0.14 * head_on_corridor_deficit
            total_linear_slowdown += 0.05 + 0.05 * proximity + 0.03 * closing_weight + 0.08 * head_on_corridor_deficit
            if in_conflict_window:
                emergency_starboard_turn = max(
                    emergency_starboard_turn,
                    0.10 + 0.16 * close_quarters + 0.12 * head_on_corridor_deficit,
                )
                emergency_linear_slowdown = max(
                    emergency_linear_slowdown,
                    0.08 + 0.10 * close_quarters + 0.08 * head_on_corridor_deficit,
                )
            if body_y <= -_CLUSTER_HEAD_ON_TARGET_STARBOARD_OFFSET:
                proactive_starboard_turn = max(
                    proactive_starboard_turn,
                    0.08 + 0.08 * (1.0 - head_on_release_progress),
                )
            continue

        if starboard_crossing:
            proactive_starboard_turn = max(
                proactive_starboard_turn,
                0.04 + 0.10 * anticipation_progress + 0.04 * closing_weight,
            )
            total_starboard_turn += 0.08 + 0.16 * proximity + 0.07 * closing_weight
            if in_conflict_window:
                emergency_starboard_turn = max(
                    emergency_starboard_turn,
                    0.06 + 0.12 * close_quarters,
                )
            continue

        if overtaking_target:
            proactive_starboard_turn = max(
                proactive_starboard_turn,
                0.04 + 0.08 * anticipation_progress + 0.03 * closing_weight,
            )
            total_starboard_turn += 0.07 + 0.14 * proximity + 0.05 * closing_weight
            if in_conflict_window:
                emergency_starboard_turn = max(
                    emergency_starboard_turn,
                    0.04 + 0.10 * close_quarters,
                )
            continue

        if port_crossing and distance < (_CLUSTER_CONFLICT_DISTANCE - 0.3):
            total_starboard_turn += 0.03 + 0.06 * close_quarters
            continue

        if ahead and distance < (_CLUSTER_CONFLICT_DISTANCE - 0.6):
            total_starboard_turn += 0.04 + 0.08 * close_quarters

    if proactive_starboard_turn <= 1e-6 and total_starboard_turn <= 1e-6 and emergency_starboard_turn <= 1e-6:
        return action

    raw_starboard_rate = max(0.0, -raw_angular_z)
    sustain_floor = 0.80 * raw_starboard_rate if raw_starboard_rate > 0.03 else 0.0
    if head_on_commit_active:
        sustain_floor = max(sustain_floor, head_on_commit_floor)
    target_starboard_rate = max(proactive_starboard_turn, total_starboard_turn) + emergency_starboard_turn
    target_starboard_rate = max(sustain_floor, target_starboard_rate)
    target_starboard_rate = _clip(target_starboard_rate, 0.04, 0.52)
    if head_on_commit_active:
        target_linear_slowdown = max(proactive_linear_slowdown, total_linear_slowdown) + emergency_linear_slowdown
        target_linear_slowdown = _clip(target_linear_slowdown, 0.0, min(0.38, max(0.0, raw_linear_x - 0.10)))
        action[0] = -target_linear_slowdown
    action[1] = -target_starboard_rate
    return action


def _scripted_residual_action_cluster_colregs_v2(observation: np.ndarray) -> np.ndarray:
    action = np.zeros(2, dtype=np.float32)
    if observation.shape[0] < 16:
        return action

    raw_angular_z = float(observation[7]) if observation.shape[0] >= 8 else 0.0
    raw_linear_x = float(observation[6]) if observation.shape[0] >= 7 else 0.0
    total_starboard_turn = 0.0
    emergency_starboard_turn = 0.0
    proactive_starboard_turn = 0.0
    total_linear_slowdown = 0.0
    emergency_linear_slowdown = 0.0
    proactive_linear_slowdown = 0.0
    head_on_commit_active = False
    head_on_commit_floor = 0.0

    for neighbor in _iter_neighbor_features(observation):
        distance = neighbor['distance']
        if distance > _CLUSTER_LOOKAHEAD_DISTANCE + 0.8:
            continue

        body_x = neighbor['body_x']
        body_y = neighbor['body_y']
        body_vx = neighbor['body_vx']
        closing_speed = neighbor['closing_speed']
        own_speed = neighbor['own_speed']
        neighbor_forward_speed = neighbor['neighbor_forward_speed']
        ahead = body_x > -0.1
        in_conflict_window = distance <= _CLUSTER_CONFLICT_DISTANCE
        in_anticipation_window = distance <= (_CLUSTER_ANTICIPATION_DISTANCE + 0.6)
        anticipation_progress = _clip(
            ((_CLUSTER_ANTICIPATION_DISTANCE + 0.6) - distance) / max((_CLUSTER_ANTICIPATION_DISTANCE + 0.6) - _CLUSTER_CONFLICT_DISTANCE, 1e-3),
            0.0,
            1.0,
        )
        head_on_commit_progress = _clip(
            (_CLUSTER_V2_HEAD_ON_COMMIT_DISTANCE - distance) / max(_CLUSTER_V2_HEAD_ON_COMMIT_DISTANCE - _CLUSTER_CONFLICT_DISTANCE, 1e-3),
            0.0,
            1.0,
        )
        proximity = _clip((_CLUSTER_LOOKAHEAD_DISTANCE - distance) / max(_CLUSTER_LOOKAHEAD_DISTANCE - _CLUSTER_CONFLICT_DISTANCE, 1e-3), 0.0, 1.0)
        close_quarters = _clip((_CLUSTER_CONFLICT_DISTANCE - distance) / max(_CLUSTER_CONFLICT_DISTANCE, 1e-3), 0.0, 1.0)
        separation_pressure = _clip(
            (_CLUSTER_EMERGENCY_BUFFER - distance) / max(_CLUSTER_EMERGENCY_BUFFER - _CLUSTER_MIN_SEPARATION, 1e-3),
            0.0,
            1.0,
        )
        closing_weight = _clip((closing_speed + 0.18) / 0.95, 0.0, 1.0)

        same_lane_ahead = ahead and abs(body_y) < 1.5 and body_x > 0.8
        overtaking_target = same_lane_ahead and own_speed > 0.22 and body_vx < -0.03 and neighbor_forward_speed < own_speed - 0.03
        head_on = ahead and abs(body_y) < max(1.6, 0.34 * distance) and closing_speed > 0.02
        starboard_crossing = ahead and body_y < -0.35 and closing_speed > -0.05
        port_crossing = ahead and body_y > 0.45 and closing_speed > -0.02
        head_on_corridor_deficit = _clip(
            (_CLUSTER_V2_HEAD_ON_TARGET_STARBOARD_OFFSET + body_y) / _CLUSTER_V2_HEAD_ON_TARGET_STARBOARD_OFFSET,
            0.0,
            1.0,
        )
        head_on_release_progress = _clip(
            (_CLUSTER_V2_HEAD_ON_RELEASE_OFFSET + body_y) / (_CLUSTER_V2_HEAD_ON_RELEASE_OFFSET - _CLUSTER_V2_HEAD_ON_TARGET_STARBOARD_OFFSET),
            0.0,
            1.0,
        )

        if distance <= (_CLUSTER_EMERGENCY_BUFFER + 0.2) and body_x > -0.7 and closing_speed > -0.06:
            emergency_starboard_turn = max(
                emergency_starboard_turn,
                0.14 + 0.24 * separation_pressure + 0.12 * closing_weight,
            )
            emergency_linear_slowdown = max(
                emergency_linear_slowdown,
                0.05 + 0.12 * separation_pressure,
            )

        if in_anticipation_window and ahead and closing_speed > -0.04:
            proactive_starboard_turn = max(
                proactive_starboard_turn,
                0.05 + 0.12 * anticipation_progress,
            )

        if head_on:
            head_on_commit_active = True
            head_on_commit_floor = max(
                head_on_commit_floor,
                0.22 + 0.12 * head_on_corridor_deficit + 0.08 * head_on_commit_progress,
            )
            proactive_starboard_turn = max(
                proactive_starboard_turn,
                0.16 + 0.18 * head_on_commit_progress + 0.12 * closing_weight + 0.14 * head_on_corridor_deficit,
            )
            proactive_linear_slowdown = max(
                proactive_linear_slowdown,
                0.08 + 0.08 * head_on_commit_progress + 0.10 * head_on_corridor_deficit,
            )
            total_starboard_turn += 0.18 + 0.18 * proximity + 0.10 * closing_weight + 0.18 * head_on_corridor_deficit
            total_linear_slowdown += 0.08 + 0.08 * proximity + 0.05 * closing_weight + 0.12 * head_on_corridor_deficit
            if in_conflict_window:
                emergency_starboard_turn = max(
                    emergency_starboard_turn,
                    0.14 + 0.18 * close_quarters + 0.14 * head_on_corridor_deficit,
                )
                emergency_linear_slowdown = max(
                    emergency_linear_slowdown,
                    0.12 + 0.14 * close_quarters + 0.12 * head_on_corridor_deficit,
                )
            if body_y <= -_CLUSTER_V2_HEAD_ON_TARGET_STARBOARD_OFFSET:
                proactive_starboard_turn = max(
                    proactive_starboard_turn,
                    0.10 + 0.10 * (1.0 - head_on_release_progress),
                )
            continue

        if starboard_crossing:
            proactive_starboard_turn = max(
                proactive_starboard_turn,
                0.04 + 0.10 * anticipation_progress + 0.04 * closing_weight,
            )
            total_starboard_turn += 0.08 + 0.16 * proximity + 0.07 * closing_weight
            if in_conflict_window:
                emergency_starboard_turn = max(
                    emergency_starboard_turn,
                    0.06 + 0.12 * close_quarters,
                )
            continue

        if overtaking_target:
            proactive_starboard_turn = max(
                proactive_starboard_turn,
                0.04 + 0.08 * anticipation_progress + 0.03 * closing_weight,
            )
            total_starboard_turn += 0.07 + 0.14 * proximity + 0.05 * closing_weight
            if in_conflict_window:
                emergency_starboard_turn = max(
                    emergency_starboard_turn,
                    0.04 + 0.10 * close_quarters,
                )
            continue

        if port_crossing and distance < (_CLUSTER_CONFLICT_DISTANCE - 0.3):
            total_starboard_turn += 0.03 + 0.06 * close_quarters
            continue

        if ahead and distance < (_CLUSTER_CONFLICT_DISTANCE - 0.6):
            total_starboard_turn += 0.04 + 0.08 * close_quarters

    if proactive_starboard_turn <= 1e-6 and total_starboard_turn <= 1e-6 and emergency_starboard_turn <= 1e-6:
        return action

    raw_starboard_rate = max(0.0, -raw_angular_z)
    sustain_floor = 0.85 * raw_starboard_rate if raw_starboard_rate > 0.03 else 0.0
    if head_on_commit_active:
        sustain_floor = max(sustain_floor, head_on_commit_floor)
    target_starboard_rate = max(proactive_starboard_turn, total_starboard_turn) + emergency_starboard_turn
    target_starboard_rate = max(sustain_floor, target_starboard_rate)
    target_starboard_rate = _clip(target_starboard_rate, 0.05, 0.60)
    if head_on_commit_active:
        target_linear_slowdown = max(proactive_linear_slowdown, total_linear_slowdown) + emergency_linear_slowdown
        target_linear_slowdown = _clip(target_linear_slowdown, 0.0, max(0.0, raw_linear_x - 0.02))
        action[0] = -target_linear_slowdown
    action[1] = -target_starboard_rate
    return action


def _scripted_residual_action_safety_first_v2(observation: np.ndarray) -> np.ndarray:
    action = np.zeros(2, dtype=np.float32)
    if observation.shape[0] < 16:
        return action

    rel_x, rel_y, rel_vx, rel_vy, distance, bearing = observation[10:16]
    if distance <= 1e-6:
        return action

    range_rate = -((rel_x * rel_vx) + (rel_y * rel_vy)) / max(distance, 1e-3)
    ahead = rel_x > 0.0 or abs(bearing) < 0.7

    same_lane_ahead = (
        rel_x > 0.5
        and abs(rel_y) < 1.5
        and abs(rel_vx) < 0.35
        and abs(rel_vy) < 0.25
    )

    # Overtaking: a slower vessel sits ahead on a similar course.
    # Brake early and bias left to avoid closing into a low-clearance follow state.
    if distance < 5.5 and same_lane_ahead and range_rate > 0.02:
        if distance < 3.5:
            return np.asarray([-0.55, 0.45], dtype=np.float32)
        return np.asarray([-0.35, 0.20], dtype=np.float32)

    # Head-on or near head-on: brake hard and bias starboard to maximize separation.
    if distance < 6.0 and ahead and range_rate > 0.15:
        if abs(bearing) < 0.35:
            return np.asarray([-0.65, -0.55], dtype=np.float32)
        if bearing > 0.0:
            return np.asarray([-0.25, -0.35], dtype=np.float32)
        return np.asarray([-0.15, 0.15], dtype=np.float32)

    # Generic close-range closure fallback.
    if distance < 3.0 and range_rate > 0.05:
        turn = -0.25 if bearing >= 0.0 else 0.1
        return np.asarray([-0.2, turn], dtype=np.float32)

    return action


def _scripted_residual_action_headon_pass_v1(observation: np.ndarray) -> np.ndarray:
    action = np.zeros(2, dtype=np.float32)
    if observation.shape[0] < 16:
        return action

    rel_x, rel_y, rel_vx, rel_vy, distance, bearing = observation[10:16]
    if distance <= 1e-6:
        return action

    range_rate = -((rel_x * rel_vx) + (rel_y * rel_vy)) / max(distance, 1e-3)
    ahead = rel_x > 0.0 or abs(bearing) < 0.7
    same_lane_ahead = (
        rel_x > 0.5
        and abs(rel_y) < 1.5
        and abs(rel_vx) < 0.35
        and abs(rel_vy) < 0.25
    )

    # Overtaking stays conservative: keep mild brake and a slight left bias.
    if distance < 5.5 and same_lane_ahead and range_rate > 0.02:
        if distance < 3.5:
            return np.asarray([-0.40, 0.35], dtype=np.float32)
        return np.asarray([-0.20, 0.15], dtype=np.float32)

    # Head-on pass heuristic: reduce speed but keep some forward authority while committing starboard.
    if distance < 6.0 and ahead and range_rate > 0.15:
        if abs(bearing) < 0.25:
            if distance < 2.5:
                return np.asarray([-0.28, -0.55], dtype=np.float32)
            if distance < 4.0:
                return np.asarray([-0.15, -0.50], dtype=np.float32)
            return np.asarray([-0.05, -0.42], dtype=np.float32)
        if bearing > 0.0:
            return np.asarray([-0.08, -0.32], dtype=np.float32)
        return np.asarray([0.02, -0.10], dtype=np.float32)

    if distance < 3.0 and range_rate > 0.05:
        turn = -0.20 if bearing >= 0.0 else -0.05
        return np.asarray([-0.08, turn], dtype=np.float32)

    return action


def _scripted_residual_action_safety_anchor_v3(observation: np.ndarray) -> np.ndarray:
    action = np.zeros(2, dtype=np.float32)
    if observation.shape[0] < 16:
        return action

    rel_x, rel_y, rel_vx, rel_vy, distance, bearing = observation[10:16]
    if distance <= 1e-6:
        return action

    range_rate = -((rel_x * rel_vx) + (rel_y * rel_vy)) / max(distance, 1e-3)
    ahead = rel_x > 0.0 or abs(bearing) < 0.7
    same_lane_ahead = (
        rel_x > 0.5
        and abs(rel_y) < 1.5
        and abs(rel_vx) < 0.35
        and abs(rel_vy) < 0.25
    )
    crossing_from_starboard = rel_x > 1.0 and rel_y < -0.5 and rel_vy > 0.05

    # Keep the proven v2 head-on safety anchor unchanged.
    if distance < 6.0 and ahead and range_rate > 0.15:
        if abs(bearing) < 0.35:
            return np.asarray([-0.65, -0.55], dtype=np.float32)
        if bearing > 0.0:
            return np.asarray([-0.25, -0.35], dtype=np.float32)
        return np.asarray([-0.15, 0.15], dtype=np.float32)

    # For starboard crossing, yield with heading commitment first and lighter braking
    # so the vessel keeps making progress once the other track starts to clear.
    if crossing_from_starboard and distance < 6.0 and range_rate > -0.02:
        if distance < 3.2:
            return np.asarray([-0.18, -0.34], dtype=np.float32)
        if distance < 4.6:
            return np.asarray([-0.08, -0.22], dtype=np.float32)
        return np.asarray([-0.03, -0.12], dtype=np.float32)

    # Overtaking remains conservative but avoids the hard-brake/linger behavior from v2.
    if distance < 5.5 and same_lane_ahead and range_rate > 0.02:
        if distance < 3.2:
            return np.asarray([-0.28, 0.42], dtype=np.float32)
        if distance < 4.5:
            return np.asarray([-0.14, 0.28], dtype=np.float32)
        return np.asarray([-0.05, 0.14], dtype=np.float32)

    if distance < 3.0 and range_rate > 0.05:
        turn = -0.22 if bearing >= 0.0 else 0.08
        return np.asarray([-0.12, turn], dtype=np.float32)

    return action


def _scripted_residual_action(observation: np.ndarray, profile: str) -> np.ndarray:
    if profile == 'cluster_colregs_v2':
        return _scripted_residual_action_cluster_colregs_v2(observation)
    if profile == 'cluster_colregs_v1':
        return _scripted_residual_action_cluster_colregs_v1(observation)
    if profile == 'safety_anchor_v3':
        return _scripted_residual_action_safety_anchor_v3(observation)
    if profile == 'headon_pass_v1':
        return _scripted_residual_action_headon_pass_v1(observation)
    return _scripted_residual_action_safety_first_v2(observation)


def main():
    args = parse_args()
    scenarios = tuple(args.scenarios) if args.scenarios else ScenarioFactory.cluster_standard_available()
    config = EnvConfig(
        namespace=args.namespace,
        launch_sitl=not args.external_stack,
        enable_rl_backend=True,
        action_mode=args.action_mode,
        max_neighbors=max(1, args.max_neighbors),
        default_scenarios=scenarios,
    )
    env = UsvRlEnv(config)

    observations = []
    actions = []
    rewards = []
    dones = []
    scenario_names = []

    try:
        for episode in range(args.episodes):
            scenario_name = scenarios[episode % len(scenarios)]
            observation, info = env.reset(options={'scenario_kind': scenario_name})
            for _ in range(args.steps_per_episode):
                teacher_action = env.project_policy_action(
                    _scripted_residual_action(observation, args.teacher_profile)
                )
                next_observation, reward, terminated, truncated, _ = env.step(teacher_action)
                observations.append(observation)
                actions.append(teacher_action)
                rewards.append(reward)
                dones.append(bool(terminated or truncated))
                scenario_names.append(info['scenario'])
                observation = next_observation
                if terminated or truncated:
                    break
    finally:
        env.close()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        observations=np.asarray(observations, dtype=np.float32),
        actions=np.asarray(actions, dtype=np.float32),
        rewards=np.asarray(rewards, dtype=np.float32),
        dones=np.asarray(dones, dtype=np.bool_),
        scenario_names=np.asarray(scenario_names),
        teacher_profile=np.asarray(args.teacher_profile),
        action_mode=np.asarray(args.action_mode),
    )
    print(f'Saved {len(observations)} samples to {output_path}')


if __name__ == '__main__':
    main()