import math
from dataclasses import dataclass, field
from typing import List

import numpy as np

from .multi_agent_types import ENCOUNTER_TYPE_COUNT, ROLE_TYPE_COUNT


USV_EGO_FEATURE_COUNT = 19
# SITL single-USV inference observation. Kept in EXACT parity with the training
# layout in usv_rl.multi_agent_types: 15 features per neighbor (10 base feats +
# 3-hot encounter + 2-hot role) followed by a trailing agent-level encounter
# one-hot. Must mirror NEIGHBOR_FEATURE_COUNT so the SITL observation vector
# matches the dimensions the policy was trained on.
USV_NEIGHBOR_FEATURE_COUNT = 15


@dataclass
class NeighborState:
    usv_id: str
    x: float
    y: float
    yaw: float
    vx: float
    vy: float


@dataclass
class NeighborObservation:
    usv_id: str
    rel_x: float
    rel_y: float
    rel_vx: float
    rel_vy: float
    distance: float
    bearing: float
    tcpa: float = 1.0
    dcpa: float = 1.0
    route_eta_delta: float = 0.0
    route_priority_delta: float = 0.0
    # Per-neighbor COLREGS tags. Default -1 (no encounter / unset).
    encounter_type_index: int = -1
    role_index: int = -1


@dataclass
class UsvObservation:
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
    cross_track_error: float = 0.0
    raw_cross_track_error: float = 0.0
    cross_track_overflow: float = 0.0
    route_progress: float = 0.0
    conflict_phase: float = 0.0
    conflict_eta: float = 1.0
    crossing_priority: float = 0.0
    crossing_eta_gap: float = 1.0
    neighbors: List[NeighborObservation] = field(default_factory=list)
    encounter_type_index: int = -1

    @staticmethod
    def ego_feature_size() -> int:
        return USV_EGO_FEATURE_COUNT

    @staticmethod
    def vector_size(max_neighbors: int) -> int:
        return USV_EGO_FEATURE_COUNT + max_neighbors * USV_NEIGHBOR_FEATURE_COUNT + ENCOUNTER_TYPE_COUNT

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
            math.sin(self.heading_error),
            math.cos(self.heading_error),
            self.raw_linear_x,
            self.raw_angular_z,
            self.final_linear_x,
            self.final_angular_z,
            self.cross_track_error,
            self.route_progress,
            self.conflict_phase,
            self.conflict_eta,
            self.crossing_priority,
            self.crossing_eta_gap,
            self.raw_cross_track_error,
            self.cross_track_overflow,
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
                neighbor.tcpa,
                neighbor.dcpa,
                neighbor.route_eta_delta,
                neighbor.route_priority_delta,
            ])
            # Per-neighbor COLREGS one-hots (Stage A.2): 3-hot encounter + 2-hot role.
            enc_one_hot = [0.0] * ENCOUNTER_TYPE_COUNT
            if 0 <= neighbor.encounter_type_index < ENCOUNTER_TYPE_COUNT:
                enc_one_hot[neighbor.encounter_type_index] = 1.0
            role_one_hot = [0.0] * ROLE_TYPE_COUNT
            if 0 <= neighbor.role_index < ROLE_TYPE_COUNT:
                role_one_hot[neighbor.role_index] = 1.0
            features.extend(enc_one_hot)
            features.extend(role_one_hot)

        for _ in range(max(0, max_neighbors - len(sorted_neighbors))):
            features.extend([0.0] * USV_NEIGHBOR_FEATURE_COUNT)

        # Trailing agent-level encounter one-hot: lets the network condition on
        # the scenario/encounter type (matches multi_agent_types.to_vector).
        encounter_one_hot = [0.0] * ENCOUNTER_TYPE_COUNT
        if 0 <= self.encounter_type_index < ENCOUNTER_TYPE_COUNT:
            encounter_one_hot[self.encounter_type_index] = 1.0
        features.extend(encounter_one_hot)

        return np.asarray(features, dtype=np.float32)


@dataclass
class RewardBreakdown:
    progress: float
    safety: float
    braking: float
    smoothness: float
    heading: float
    time_cost: float
    terminal: float

    @property
    def total(self) -> float:
        return self.progress + self.safety + self.braking + self.smoothness + self.heading + self.time_cost + self.terminal