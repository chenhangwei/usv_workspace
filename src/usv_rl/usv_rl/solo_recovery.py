import math
from dataclasses import replace

from .multi_agent_scenarios import AgentSpawnConfig, FleetScenario


def recovery_reset_scenario(
    scenario: FleetScenario, cross_track_error: float, heading_error: float,
    route_progress: float = 0.0,
) -> FleetScenario:
    """Offset the simulated hull without moving the reference route."""
    if (len(scenario.agent_spawns) != 1 or len(scenario.active_agent_ids) != 1
            or scenario.background_tracks):
        raise ValueError('solo recovery requires exactly one hull and no background tracks')
    if not all(math.isfinite(value) for value in (cross_track_error, heading_error, route_progress)):
        raise ValueError('solo recovery offsets must be finite')
    if not 0.0 <= route_progress < 1.0:
        raise ValueError('solo recovery route progress must be in [0, 1)')
    agent_id = scenario.active_agent_ids[0]
    start = scenario.agent_spawns[agent_id]
    goal = scenario.agent_goals[agent_id]
    delta_x, delta_y = goal.x - start.x, goal.y - start.y
    length = math.hypot(delta_x, delta_y)
    if length <= 1e-6:
        raise ValueError('solo recovery requires a nonzero reference route')
    pose_x = start.x + route_progress * delta_x + cross_track_error * delta_y / length
    pose_y = start.y + route_progress * delta_y - cross_track_error * delta_x / length
    goal_bearing = math.atan2(goal.y - pose_y, goal.x - pose_x)
    yaw = math.atan2(math.sin(goal_bearing - heading_error), math.cos(goal_bearing - heading_error))
    return replace(scenario, agent_spawns={agent_id: AgentSpawnConfig(pose_x, pose_y, yaw)})