"""Canonical route frame shared by the training environment and online inference.

The solo expert observes its route through one fixed-length window instead of world
coordinates, so leg length, location and direction cannot push it out of distribution.
"""

import math

ROUTE_CANONICAL_FRAME = 'route_canonical_v1'
LEGACY_FRAME = 'legacy'
OBSERVATION_FRAMES = (LEGACY_FRAME, ROUTE_CANONICAL_FRAME)
# Training legs are 12 m (MultiAgentEnvConfig.goal_distance), so the window matches them exactly.
CANONICAL_LOOKAHEAD = 12.0


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def turn_governor_speed_cap(heading_error: float, cruise: float = 0.34, floor: float = 0.45,
                            min_speed: float = 0.14) -> float:
    """Deployment turn governor: forward speed cap (m/s) shrinking with heading error."""
    cos_gate = max(0.0, math.cos(min(abs(float(heading_error)), math.pi)))
    return max(cruise * (floor + (1.0 - floor) * cos_gate), min_speed)


def canonical_route_fields(
    *,
    pose_x: float,
    pose_y: float,
    yaw: float,
    feedback_distance: float,
    feedback_heading_error: float,
    route_start: tuple[float, float],
    route_goal: tuple[float, float],
    lookahead: float = CANONICAL_LOOKAHEAD,
) -> dict:
    """Return observation fields expressed in the canonical route window.

    The window ends at the point ``lookahead`` metres ahead of the hull's projection
    (capped at the goal) and starts ``lookahead`` metres before that (floored at the
    route start). Distance, heading error and progress all refer to that window; the
    pose is the hull position relative to the window start with the route along +x
    (left of route = +y) and yaw is measured from the route direction. Once the window
    end reaches the goal the controller feedback is used unchanged, exactly as in
    training on legs no longer than ``lookahead``.
    """
    values = (pose_x, pose_y, yaw, feedback_distance, feedback_heading_error, *route_start, *route_goal, lookahead)
    if not all(math.isfinite(float(value)) for value in values) or lookahead <= 0:
        raise ValueError('canonical route frame requires finite inputs and a positive lookahead')
    start_x, start_y = float(route_start[0]), float(route_start[1])
    route_dx, route_dy = float(route_goal[0]) - start_x, float(route_goal[1]) - start_y
    route_length = math.hypot(route_dx, route_dy)
    if route_length <= 1e-6:
        raise ValueError('canonical route frame requires a nonzero route')
    unit_x, unit_y = route_dx / route_length, route_dy / route_length
    relative_x, relative_y = float(pose_x) - start_x, float(pose_y) - start_y
    along = relative_x * unit_x + relative_y * unit_y
    left = unit_x * relative_y - unit_y * relative_x
    window_end = min(route_length, max(along, 0.0) + lookahead)
    window_start = max(0.0, window_end - lookahead)
    if window_end >= route_length - 1e-9:
        distance, heading_error = float(feedback_distance), float(feedback_heading_error)
    else:
        carrot_x, carrot_y = start_x + unit_x * window_end, start_y + unit_y * window_end
        distance = math.hypot(carrot_x - float(pose_x), carrot_y - float(pose_y))
        heading_error = _wrap(math.atan2(carrot_y - float(pose_y), carrot_x - float(pose_x)) - float(yaw))
    return {
        'pose_x': along - window_start,
        'pose_y': left,
        'yaw': _wrap(float(yaw) - math.atan2(route_dy, route_dx)),
        'distance_to_goal': distance,
        'heading_error': heading_error,
        'route_progress': min(1.0, max(0.0, (along - window_start) / (window_end - window_start))),
    }
