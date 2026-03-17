import math
from dataclasses import dataclass
from typing import List

from .types import NeighborState


@dataclass
class NeighborTrack:
    usv_id: str
    start_x: float
    start_y: float
    vx: float
    vy: float
    yaw: float

    def state_at(self, elapsed: float) -> NeighborState:
        return NeighborState(
            usv_id=self.usv_id,
            x=self.start_x + self.vx * elapsed,
            y=self.start_y + self.vy * elapsed,
            yaw=self.yaw,
            vx=self.vx,
            vy=self.vy,
        )


@dataclass
class EncounterScenario:
    name: str
    goal_x: float
    goal_y: float
    duration: float
    neighbors: List[NeighborTrack]

    def states_at(self, elapsed: float) -> List[NeighborState]:
        return [track.state_at(elapsed) for track in self.neighbors]


class ScenarioFactory:
    """生成相对于当前艇位的标准会遇场景。"""

    @staticmethod
    def _track_from_body(
        usv_id: str,
        body_x: float,
        body_y: float,
        body_vx: float,
        body_vy: float,
        *,
        origin_x: float,
        origin_y: float,
        forward_x: float,
        forward_y: float,
        left_x: float,
        left_y: float,
    ) -> NeighborTrack:
        start_x = origin_x + forward_x * body_x + left_x * body_y
        start_y = origin_y + forward_y * body_x + left_y * body_y
        vx = forward_x * body_vx + left_x * body_vy
        vy = forward_y * body_vx + left_y * body_vy
        return NeighborTrack(usv_id, start_x, start_y, vx, vy, math.atan2(vy, vx))

    @staticmethod
    def available() -> tuple[str, ...]:
        return ('head_on', 'crossing_starboard', 'overtaking')

    @staticmethod
    def cluster_standard_available() -> tuple[str, ...]:
        return (
            'five_usv_dense_head_on',
            'five_usv_dense_crossing',
            'five_usv_dense_overtaking',
        )

    @staticmethod
    def all_available() -> tuple[str, ...]:
        return ScenarioFactory.available() + ScenarioFactory.cluster_standard_available()

    @staticmethod
    def create(kind: str, origin_x: float, origin_y: float, yaw: float, goal_distance: float, neighbor_speed: float) -> EncounterScenario:
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)

        goal_x = origin_x + forward_x * goal_distance
        goal_y = origin_y + forward_y * goal_distance

        if kind == 'head_on':
            start_x = origin_x + forward_x * 6.0
            start_y = origin_y + forward_y * 6.0
            vx = -forward_x * neighbor_speed
            vy = -forward_y * neighbor_speed
            neighbor = NeighborTrack('teacher_head_on', start_x, start_y, vx, vy, math.atan2(vy, vx))
            return EncounterScenario(kind, goal_x, goal_y, 40.0, [neighbor])

        if kind == 'crossing_starboard':
            center_x = origin_x + forward_x * 5.0
            center_y = origin_y + forward_y * 5.0
            start_x = center_x - left_x * 2.5
            start_y = center_y - left_y * 2.5
            vx = left_x * neighbor_speed
            vy = left_y * neighbor_speed
            neighbor = NeighborTrack('teacher_crossing', start_x, start_y, vx, vy, math.atan2(vy, vx))
            return EncounterScenario(kind, goal_x, goal_y, 40.0, [neighbor])

        if kind == 'overtaking':
            start_x = origin_x + forward_x * 2.0 - left_x * 0.5
            start_y = origin_y + forward_y * 2.0 - left_y * 0.5
            vx = forward_x * (neighbor_speed * 0.55)
            vy = forward_y * (neighbor_speed * 0.55)
            neighbor = NeighborTrack('teacher_overtake', start_x, start_y, vx, vy, math.atan2(vy, vx))
            return EncounterScenario(kind, goal_x, goal_y, 35.0, [neighbor])

        if kind == 'five_usv_dense_head_on':
            neighbors = [
                ScenarioFactory._track_from_body(
                    'cluster_head_on', 3.8, 0.0, -neighbor_speed, 0.0,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_starboard_cross', 3.2, -3.4, 0.0, neighbor_speed,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_port_cross', 3.4, 3.3, 0.0, -neighbor_speed * 0.9,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_overtake', 1.8, -0.8, neighbor_speed * 0.55, 0.0,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
            ]
            return EncounterScenario(kind, goal_x, goal_y, 45.0, neighbors)

        if kind == 'five_usv_dense_crossing':
            neighbors = [
                ScenarioFactory._track_from_body(
                    'cluster_starboard_primary', 2.8, -3.6, 0.1 * neighbor_speed, neighbor_speed,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_starboard_secondary', 4.0, -2.2, -0.1 * neighbor_speed, neighbor_speed * 0.9,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_head_on_offset', 4.0, 1.4, -neighbor_speed * 0.9, 0.0,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_port_hold', 3.6, 3.2, 0.0, -neighbor_speed * 0.7,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
            ]
            return EncounterScenario(kind, goal_x, goal_y, 45.0, neighbors)

        if kind == 'five_usv_dense_overtaking':
            neighbors = [
                ScenarioFactory._track_from_body(
                    'cluster_lead_center', 2.2, 0.0, neighbor_speed * 0.42, 0.0,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_lead_starboard', 2.8, -1.2, neighbor_speed * 0.48, 0.0,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_starboard_cross', 3.4, -3.1, 0.0, neighbor_speed * 0.9,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
                ScenarioFactory._track_from_body(
                    'cluster_head_on_far', 4.0, 0.9, -neighbor_speed * 0.8, 0.0,
                    origin_x=origin_x, origin_y=origin_y,
                    forward_x=forward_x, forward_y=forward_y,
                    left_x=left_x, left_y=left_y,
                ),
            ]
            return EncounterScenario(kind, goal_x, goal_y, 45.0, neighbors)

        raise ValueError(f'Unsupported scenario kind: {kind}')