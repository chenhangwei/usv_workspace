import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np

# Number of distinct encounter types encoded as a one-hot in the observation.
# 0=head_on, 1=crossing, 2=overtaking.  -1 means unset (zero vector).
ENCOUNTER_TYPE_COUNT = 3
# Role one-hot per neighbor: 0=give_way, 1=stand_on. -1 means unset (zero vector).
ROLE_TYPE_COUNT = 2
LOCAL_EGO_FEATURE_COUNT = 19
# Neighbor layout: relative pose/velocity plus pairwise conflict timing
# plus per-neighbor COLREGS encounter (3-hot) and role (2-hot) tags.
# 0..5:   rel_x, rel_y, rel_vx, rel_vy, distance, bearing
# 6..9:   tcpa_norm, dcpa_norm, route_eta_delta, route_priority_delta
# 10..12: encounter one-hot (head_on, crossing, overtaking)
# 13..14: role one-hot (give_way, stand_on)
NEIGHBOR_FEATURE_COUNT = 15
NEIGHBOR_BASE_FEATURE_COUNT = 10  # legacy base count, before encounter/role tags

# ---------------------------------------------------------------------------
# Canonical COLREGS encounter / role classifier.
#
# This is the SINGLE SOURCE OF TRUTH for whether a (ego, neighbor) pair is in
# a Rule 13/14/15 encounter and what role the ego plays. It is used by:
#   - evaluate_mappo_policy.py (compliance counting)
#   - any future code that needs to write per-neighbor encounter/role into
#     the observation vector (Stage A.2)
#
# The trainer's reward shaping in multi_agent_env.py uses a slightly broader
# detection (e.g. extra "co_directional_close" overtaking trigger) by design,
# so this classifier intentionally encodes the strict definitions only.
# ---------------------------------------------------------------------------
ENC_NONE = -1
ENC_HEAD_ON = 0
ENC_CROSSING = 1
ENC_OVERTAKING = 2

ROLE_NONE = -1
ROLE_GIVE_WAY = 0
ROLE_STAND_ON = 1


def classify_encounter_role(
    *,
    body_x: float,
    body_y: float,
    body_vx: float,
    body_vy: float,
    distance: float,
    own_speed: float,
    ego_id: str = '',
    neighbor_id: str = '',
    own_yaw: float = None,
) -> Tuple[int, int]:
    """Classify the COLREGS encounter type and ego role for a single pair.

    Inputs (fresh628 CONVENTION FIX): both production callers
    (multi_agent_bridge and policy_inference_node) pass WORLD-frame relative
    vectors (neighbor minus ego), NOT body-frame as the old docstring
    claimed — the old classifier silently mis-labelled every pair whose ego
    heading was not ~east. Pass ``own_yaw`` (ego heading, world frame) and
    the rotation into the ego body frame happens HERE; ``own_yaw=None``
    preserves the legacy no-rotation behaviour for old callers.
      body_x/body_y: relative offset to neighbor (world frame when own_yaw
        given; ego body frame otherwise)
      body_vx/body_vy: neighbor velocity minus ego velocity (same frame)
      distance: hypot(body_x, body_y)
      own_speed: ego linear speed (m/s)
      ego_id / neighbor_id: stable USV identifiers used ONLY as the
        deterministic tie-break for ambiguous geometry (fresh628).

    Returns (encounter_type, role) where:
      encounter_type ∈ {ENC_NONE, ENC_HEAD_ON, ENC_CROSSING, ENC_OVERTAKING}
      role ∈ {ROLE_NONE, ROLE_GIVE_WAY, ROLE_STAND_ON}

    fresh628 STRICT ROLE COMPLEMENT (user directive 2026-07-11): for every
    engaged pair exactly ONE vessel is give-way (从船) and ONE is stand-on
    (主船) -- never both. Guarantees, per encounter class:
      * OVERTAKING (Rule 13): role from LONGITUDINAL ORDER -- the vessel
        astern (the overtaker) gives way, the vessel ahead stands on. The
        ahead/astern relation is exactly complementary between the two hulls,
        so the pair can never disagree. This also fixes the old bug where
        body_x <= 0 returned NONE and the overtaken vessel never learned it
        was stand-on.
      * CROSSING (Rule 15): the vessel that sees the other on her STARBOARD
        side gives way; the other stands on. In converging crossing geometry
        the starboard relation is complementary.
      * HEAD-ON (Rule 14 says both give way, but the user's operational rule
        requires a strict master/slave split): DETERMINISTIC ID TIE-BREAK --
        lexicographically smaller id stands on (passes fast), larger id
        gives way. Independent of noisy geometry, so both hulls always agree.
        The near-reciprocal ambiguous band (|Δψ| in 135°..180°) where noisy
        starboard tests flip is folded into this tie-break too.
        fresh634 BAND FIX (SITL 2026-08-03): the band starts at 135° (was
        150°), matching _HEAD_ON_COURSE_DEG in policy_inference_node. At
        150° the two hulls' noisy |Δψ| estimates straddled the boundary in
        oblique head-ons (measured 135°-167° while maneuvering), one hull
        classified CROSSING/GIVE_WAY and the other HEAD_ON/GIVE_WAY ->
        mutual yield -> the bow-to-bow standoff loop.
    """
    if distance <= 1e-3:
        return ENC_NONE, ROLE_NONE
    if own_yaw is not None:
        cos_y = math.cos(own_yaw)
        sin_y = math.sin(own_yaw)
        bx = cos_y * body_x + sin_y * body_y
        by = -sin_y * body_x + cos_y * body_y
        bvx = cos_y * body_vx + sin_y * body_vy
        bvy = -sin_y * body_vx + cos_y * body_vy
        body_x, body_y, body_vx, body_vy = bx, by, bvx, bvy
    closing_speed = -((body_x * body_vx) + (body_y * body_vy)) / max(distance, 1e-3)
    # Reconstruct the neighbour's velocity in the ego body frame: ego travels
    # (own_speed, 0) in her own frame, rel_v = neighbour_v - ego_v.
    nb_vx = own_speed + body_vx
    nb_vy = body_vy
    nb_speed = math.hypot(nb_vx, nb_vy)
    both_slow = own_speed < 0.05 and nb_speed < 0.05
    # fresh634: both_slow only disengages beyond 3m -- a stalled close-quarters
    # pair must keep its roles so shaping can push the stand-on hull out.
    if (both_slow and distance > 3.0) or (closing_speed <= 0.0 and distance > 3.0):
        return ENC_NONE, ROLE_NONE
    # Relative course of the neighbour w.r.t. ego heading (ego heading = 0 in
    # her own body frame). |delta| near pi => reciprocal (head-on) courses.
    delta_psi = math.atan2(nb_vy, nb_vx) if nb_speed > 0.03 else 0.0
    abs_delta = abs(delta_psi)

    def _tie_break() -> int:
        # Deterministic, geometry-independent: smaller id stands on.
        if ego_id and neighbor_id:
            return ROLE_STAND_ON if str(ego_id) < str(neighbor_id) else ROLE_GIVE_WAY
        # No ids available (legacy caller): fall back to Rule 14 give-way.
        return ROLE_GIVE_WAY

    # --- OVERTAKING (Rule 13): near-parallel courses, one clearly faster,
    # same lane. Role = longitudinal order (strictly complementary).
    same_lane = abs(body_y) < 1.5
    near_parallel = abs_delta < (math.pi / 3.0) and nb_speed > 0.05
    if same_lane and near_parallel:
        speed_gap = own_speed - nb_speed
        if body_x > 0.8 and speed_gap > 0.02 and closing_speed > 0.0:
            # Neighbour ahead and slower; ego is the overtaker.
            return ENC_OVERTAKING, ROLE_GIVE_WAY
        if body_x < -0.8 and speed_gap < -0.02:
            # Neighbour astern and faster; ego is being overtaken.
            return ENC_OVERTAKING, ROLE_STAND_ON

    # From here on, only vessels ahead of the beam engage COLREGS shaping.
    if body_x <= 0.0:
        return ENC_NONE, ROLE_NONE

    # --- HEAD-ON band + ambiguous near-reciprocal band: ID tie-break.
    # 135° aligns with _HEAD_ON_COURSE_DEG (policy_inference_node) so the
    # type observation and the role assignment can never disagree.
    if abs_delta > math.radians(135.0) and closing_speed > 0.0:
        # fresh634: gate on DCPA instead of ego-frame |body_y| -- the cross
        # product |rel_pos x rel_vel| is identical from both hulls, while
        # body_y is measured against each hull's own (avoidance-deflected)
        # heading and made the two hulls disagree (SITL 2026-08-03: one hull
        # saw crossing, the other head-on -> mutual give-way standoff).
        rel_speed = math.hypot(body_vx, body_vy)
        dcpa = abs(body_x * body_vy - body_y * body_vx) / max(rel_speed, 1e-3)
        lateral_tol = max(1.3, 0.28 * distance)
        if dcpa < lateral_tol:
            return ENC_HEAD_ON, _tie_break()

    # --- CROSSING (Rule 15): converging, other on starboard => give way.
    if closing_speed > -0.05 and nb_speed > 0.05 and abs_delta > math.radians(20.0):
        # fresh634: in the near-reciprocal band the starboard rule is
        # unreliable -- an oblique head-on puts EACH hull on the other's
        # starboard bow, so Rule 15 hands BOTH hulls GIVE_WAY (SITL
        # 2026-08-03 mutual-yield standoff). Use the same ID tie-break as
        # the head-on branch so the roles stay complementary even when the
        # two hulls disagree about head_on vs crossing.
        if abs_delta > math.radians(120.0):
            return ENC_CROSSING, _tie_break()
        if body_y < -0.35:
            return ENC_CROSSING, ROLE_GIVE_WAY
        if body_y > 0.35:
            return ENC_CROSSING, ROLE_STAND_ON

    return ENC_NONE, ROLE_NONE


@dataclass
class AgentNeighborObservation:
    source_id: str
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
    cross_track_error: float = 0.0
    raw_cross_track_error: float = 0.0
    cross_track_overflow: float = 0.0
    route_progress: float = 0.0
    conflict_phase: float = 0.0
    conflict_eta: float = 1.0
    crossing_priority: float = 0.0
    crossing_eta_gap: float = 1.0
    neighbors: List[AgentNeighborObservation] = field(default_factory=list)
    encounter_type_index: int = -1

    @staticmethod
    def ego_feature_size() -> int:
        return LOCAL_EGO_FEATURE_COUNT

    @staticmethod
    def vector_size(max_neighbors: int) -> int:
        return LOCAL_EGO_FEATURE_COUNT + max_neighbors * NEIGHBOR_FEATURE_COUNT + ENCOUNTER_TYPE_COUNT

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
            features.extend([0.0] * NEIGHBOR_FEATURE_COUNT)

        # Encounter type one-hot: allows the network to condition on scenario type
        encounter_one_hot = [0.0] * ENCOUNTER_TYPE_COUNT
        if 0 <= self.encounter_type_index < ENCOUNTER_TYPE_COUNT:
            encounter_one_hot[self.encounter_type_index] = 1.0
        features.extend(encounter_one_hot)

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
        *,
        active_agent_ids: tuple[str, ...] | None = None,
        goal_tolerance: float = 0.8,
    ) -> 'FleetGlobalState':
        if active_agent_ids:
            active_set = set(active_agent_ids)
            selected_observations = {
                agent_id: observation
                for agent_id, observation in local_observations.items()
                if agent_id in active_set
            }
        else:
            selected_observations = dict(local_observations)

        distances = []
        heading_errors = []
        positions = []
        pairwise_separations = []
        for observation in selected_observations.values():
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
            goal_completion_ratio = float(np.mean([1.0 if distance <= goal_tolerance else 0.0 for distance in distances]))

        return cls(
            local_observations=selected_observations,
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