from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np


@dataclass
class AgentNeighborObservation:
    source_id: str
    rel_x: float
    rel_y: float
    rel_vx: float
    rel_vy: float
    distance: float
    bearing: float


@dataclass
class AgentLocalObservation:
    agent_id: str
    pose_x: float
    pose_y: float
    yaw: float
    speed: float
    distance_to_goal: float
    heading_error: float
    raw_linear_x: float
    raw_angular_z: float
    final_linear_x: float
    final_angular_z: float
    neighbors: List[AgentNeighborObservation] = field(default_factory=list)

    @staticmethod
    def vector_size(max_neighbors: int) -> int:
        return 10 + max_neighbors * 6

    def min_neighbor_distance(self) -> float:
        if not self.neighbors:
            return float('inf')
        return min(neighbor.distance for neighbor in self.neighbors)

    def to_vector(self, max_neighbors: int) -> np.ndarray:
        features = [
            self.pose_x,
            self.pose_y,
            self.yaw,
            self.speed,
            self.distance_to_goal,
            self.heading_error,
            self.raw_linear_x,
            self.raw_angular_z,
            self.final_linear_x,
            self.final_angular_z,
        ]

        sorted_neighbors = sorted(self.neighbors, key=lambda item: item.distance)[:max_neighbors]
        for neighbor in sorted_neighbors:
            features.extend([
                neighbor.rel_x,
                neighbor.rel_y,
                neighbor.rel_vx,
                neighbor.rel_vy,
                neighbor.distance,
                neighbor.bearing,
            ])

        for _ in range(max(0, max_neighbors - len(sorted_neighbors))):
            features.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        return np.asarray(features, dtype=np.float32)


@dataclass
class FleetGlobalState:
    local_observations: Dict[str, AgentLocalObservation]
    team_min_separation: float
    team_mean_separation: float
    team_mean_goal_distance: float
    team_max_heading_error: float
    goal_completion_ratio: float

    @staticmethod
    def vector_size(max_agents: int, max_neighbors: int) -> int:
        return max_agents * AgentLocalObservation.vector_size(max_neighbors) + 5

    @classmethod
    def from_local_observations(
        cls,
        local_observations: Dict[str, AgentLocalObservation],
    ) -> 'FleetGlobalState':
        distances = []
        heading_errors = []
        positions = []
        pairwise_separations = []
        for observation in local_observations.values():
            distances.append(observation.distance_to_goal)
            heading_errors.append(abs(observation.heading_error))
            positions.append((observation.pose_x, observation.pose_y))

        team_min_separation = float('inf')
        for idx, (x_a, y_a) in enumerate(positions):
            for x_b, y_b in positions[idx + 1:]:
                separation = float(np.hypot(x_b - x_a, y_b - y_a))
                team_min_separation = min(team_min_separation, separation)
                pairwise_separations.append(separation)

        goal_completion_ratio = 0.0
        if distances:
            goal_completion_ratio = float(np.mean([1.0 if distance <= 0.8 else 0.0 for distance in distances]))

        return cls(
            local_observations=dict(local_observations),
            team_min_separation=team_min_separation,
            team_mean_separation=float(np.mean(pairwise_separations)) if pairwise_separations else float('inf'),
            team_mean_goal_distance=float(np.mean(distances)) if distances else 0.0,
            team_max_heading_error=max(heading_errors) if heading_errors else 0.0,
            goal_completion_ratio=goal_completion_ratio,
        )

    def to_vector(self, max_agents: int, max_neighbors: int) -> np.ndarray:
        features = []
        ordered_ids = sorted(self.local_observations)
        for agent_id in ordered_ids[:max_agents]:
            features.extend(self.local_observations[agent_id].to_vector(max_neighbors).tolist())

        missing_agents = max(0, max_agents - len(ordered_ids))
        if missing_agents:
            features.extend([0.0] * (missing_agents * AgentLocalObservation.vector_size(max_neighbors)))

        team_min_separation = self.team_min_separation
        if not np.isfinite(team_min_separation):
            team_min_separation = 1e3
        team_mean_separation = self.team_mean_separation
        if not np.isfinite(team_mean_separation):
            team_mean_separation = 1e3
        features.extend([
            float(team_min_separation),
            float(team_mean_separation),
            float(self.team_mean_goal_distance),
            float(self.team_max_heading_error),
            float(self.goal_completion_ratio),
        ])
        return np.asarray(features, dtype=np.float32)


@dataclass
class MultiAgentRewardBreakdown:
    progress: float
    safety: float
    braking: float
    smoothness: float
    heading: float
    team: float
    coordination: float
    time_cost: float
    terminal: float

    @property
    def total(self) -> float:
        return self.progress + self.safety + self.braking + self.smoothness + self.heading + self.team + self.coordination + self.time_cost + self.terminal