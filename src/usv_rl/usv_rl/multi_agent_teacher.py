"""Multi-USV COLREGS rule teacher for behavior-cloning bootstrap (M2).

Why this exists
---------------
fresh524/600/601 repeated gates were all 0/11. The decisive diagnostic
(fresh601 three_usv_crossing) showed the policy floors the throttle from
step 1 (vx 0.11 with no neighbour near, cruise 0.34) and only covers ~33% of
the route before timeout. Pure PPO -- across full / speed_scale / speed_scale-
no-gate action spaces -- never learned the "clear the conflict then drive to
the goal" temporal behaviour. goal completion was ~0 in every paradigm.

The fix is to inject that behaviour as a prior via a rule teacher -> behaviour
cloning -> PPO fine-tune pipeline (the project's own retraining plan, never
actually executed). This module is the teacher.

Design
------
The validated single-agent COLREGS avoidance heuristic
(``collect_scripted_dataset._scripted_policy_action_cluster_colregs_v2``) only
emits an *avoidance delta* and assumes a separate navigation layer drives the
craft toward the goal. In the multi-agent ``speed_scale`` action space the
policy output IS the full command, so the teacher must be a complete
controller:

    no conflict : omega = nav heading command (raw_angular_z), scale = 1.0
    avoidance   : omega = -starboard_rate (commit to a right turn),
                  scale  = 1 - slowdown / cruise   (>= scale_min, never stops)

The avoidance decision (which encounter -> how much starboard / slowdown) is
reused unchanged from the validated v2 heuristic via a thin observation
adapter, so we do not re-derive ~100 lines of COLREGS geometry. Crucially the
v2 heuristic only requests a slowdown in head-on commits; in crossing /
overtaking it keeps full speed and just turns -- which is exactly the
"veteran driver" behaviour the crawling policies lacked.

The teacher consumes the structured ``AgentLocalObservation`` (so it can use
the pre-computed world-frame relative geometry) and never commands reverse or
a full stop (scale is floored at ``scale_min``).
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from .collect_scripted_dataset import _scripted_policy_action_cluster_colregs_v2
from .multi_agent_types import AgentLocalObservation
from .route_frame import turn_governor_speed_cap


# Layout of the synthetic flat vector expected by the v2 heuristic
# (mirrors collect_scripted_dataset._iter_neighbor_features):
#   [2]=yaw, [3]=own_speed, [6]=raw_linear_x, [7]=raw_angular_z,
#   neighbours from offset 10, stride 6: rel_x, rel_y, rel_vx, rel_vy,
#   distance, bearing  (all world-frame; the heuristic rotates internally).
_TEACHER_VEC_HEADER = 10
_TEACHER_VEC_STRIDE = 6


def _build_teacher_layout_vector(obs: AgentLocalObservation) -> np.ndarray:
    """Pack a structured multi-agent observation into the flat layout the
    validated single-agent v2 COLREGS heuristic expects."""
    neighbors = sorted(obs.neighbors, key=lambda n: n.distance)
    vec = np.zeros(_TEACHER_VEC_HEADER + _TEACHER_VEC_STRIDE * max(1, len(neighbors)), dtype=np.float32)
    vec[2] = float(obs.yaw)
    vec[3] = float(obs.speed)
    vec[6] = float(obs.raw_linear_x)
    vec[7] = float(obs.raw_angular_z)
    for i, nb in enumerate(neighbors):
        base = _TEACHER_VEC_HEADER + _TEACHER_VEC_STRIDE * i
        vec[base + 0] = float(nb.rel_x)
        vec[base + 1] = float(nb.rel_y)
        vec[base + 2] = float(nb.rel_vx)
        vec[base + 3] = float(nb.rel_vy)
        vec[base + 4] = float(nb.distance)
        vec[base + 5] = float(nb.bearing)
    return vec


class MultiAgentColregsTeacher:
    """Complete COLREGS controller emitting actions for the chosen action mode.

    Parameters mirror the env so the teacher's commands land in exactly the
    action space the student will be trained on.
    """

    def __init__(
        self,
        *,
        action_mode: str = 'speed_scale',
        cruise_speed: float = 0.34,
        max_angular_velocity: float = 0.50,
        action_speed_scale_min: float = 0.30,
        linear_speed_limit: float | None = None,
        conflict_gate: bool = True,
        tcpa_gate_horizon: float = 18.0,
        dcpa_gate_distance: float = 2.2,
        gate_engage_distance: float = 7.8,
        gate_release_distance: float = 8.4,
        close_range_distance: float = 3.2,
        dense_neighbor_threshold: int = 2,
        heading_p_gain: float = 1.25,
        cte_p_gain: float = 0.45,
        cte_term_limit: float | None = None,
        engaged_cte_p_gain: float = 0.0,
        escape_distance: float = 1.35,
        escape_omega: float = 0.70,
        turn_speed_governor: bool = False,
        threat_latch: bool = True,
        dense_closing_only: bool = False,
        engaged_cte_fade: float = 0.0,
        governor_floor: float = 0.45,
        governor_power: float = 1.0,
        brake_distance: float = 1.3,
        brake_slowdown: float = 0.22,
        giveway_distance: float = 0.0,
        giveway_tcpa: float = 15.0,
        giveway_dcpa: float = 2.0,
        giveway_smooth: bool = False,
        duck_offset: float = 0.0,
        duck_slow: float = 0.25,
        omega_deadband_comp: float = 0.0,
    ) -> None:
        if action_mode not in ('speed_scale', 'full', 'angular_only'):
            raise ValueError(f'unsupported action_mode={action_mode!r}')
        self.action_mode = action_mode
        self.cruise_speed = float(cruise_speed)
        self.max_angular_velocity = float(max_angular_velocity)
        self.action_speed_scale_min = float(np.clip(action_speed_scale_min, 0.0, 1.0))
        # For 'full' mode the absolute forward speed ceiling.
        self.linear_speed_limit = float(linear_speed_limit if linear_speed_limit is not None else cruise_speed)
        # v3 conflict gate: the v2 heuristic biases starboard for ANY ahead
        # neighbour inside the 5.6m anticipation window (clip min 0.05), which
        # BC clones into a full-domain right drift (fresh640/641 lesson). Gate
        # the avoidance geometry on a genuine predicted conflict (TCPA/DCPA)
        # with a per-neighbour latch (hysteresis) so open water is pure
        # goal-tracking and engaged encounters stay committed (no rudder chatter).
        # Gate widths tuned by A/B smoke 2026-08-24: TCPA 12s/DCPA 1.6m engaged
        # too late in five_usv_dense_crossing (min_sep 0.75 = collision) while
        # 18s/2.2m keeps the slow-approach drift geometry (TCPA ~50s) excluded.
        # Widening alone did NOT recover dense safety: with 3+ converging hulls
        # everyone is turning, so pairwise straight-line CPA extrapolation is
        # unreliable. The strict TCPA gate is therefore only trusted in
        # near-two-body geometry; with >= dense_neighbor_threshold neighbours
        # inside the engage radius we fall back to v2 blanket caution locally.
        # Engage radius matches v2's true reaction horizon (lookahead+0.8=7.8m)
        # so the dense fallback sees exactly what v2 sees (5.6m truncated it
        # and dense min_sep stayed 0.73 vs v2's 1.23 on the same seed).
        self.conflict_gate = bool(conflict_gate)
        self.tcpa_gate_horizon = float(tcpa_gate_horizon)
        self.dcpa_gate_distance = float(dcpa_gate_distance)
        self.gate_engage_distance = float(gate_engage_distance)
        self.gate_release_distance = float(gate_release_distance)
        self.close_range_distance = float(close_range_distance)
        self.dense_neighbor_threshold = int(dense_neighbor_threshold)
        self.heading_p_gain = float(heading_p_gain)
        self.cte_p_gain = float(cte_p_gain)
        # Caps the route pull-in so a high CTE gain cannot demand a near-perpendicular intercept far off route.
        self.cte_term_limit = None if cte_term_limit is None else abs(float(cte_term_limit))
        # fresh646: with the CTE sign fixed, route pull-back cancelled the starboard
        # avoidance offset and dense five-USV min_sep fell to 0.70-0.75; while a
        # threat is latched only the heading term steers so the offset can build.
        self.engaged_cte_p_gain = float(engaged_cte_p_gain)
        self.escape_distance = float(escape_distance)
        self.escape_omega = float(abs(escape_omega))
        # Slow down for large heading errors with the same law as the deployment turn governor.
        self.turn_speed_governor = bool(turn_speed_governor)
        # SITL turns ~3x slower than commanded at cruise; power > 1 slows down already at moderate heading errors.
        self.governor_floor = float(governor_floor)
        self.governor_power = float(governor_power)
        # Sluggish SITL yaw: braking earlier is the only way to buy turn authority at close range.
        self.brake_distance = float(brake_distance)
        self.brake_slowdown = float(brake_slowdown)
        # Give-way mode (0 disables): a hull converging from the starboard bow/ahead makes this boat
        # slow to minimum and turn hard starboard; stand-on boats keep the route. Breaks the
        # symmetric all-yield crawl that sluggish SITL yaw produces in dense convergence.
        self.giveway_distance = float(giveway_distance)
        self.giveway_tcpa = float(giveway_tcpa)
        self.giveway_dcpa = float(giveway_dcpa)
        # Continuous urgency instead of turn/hold switching: a regression student cannot fit the step.
        self.giveway_smooth = bool(giveway_smooth)
        # Duck mode (>0 enables): give-way boats glide onto a starboard route offset of up to duck_offset m
        # at near-cruise speed and drift back after CPA, instead of braking and spinning.
        self.duck_offset = float(duck_offset)
        self.duck_slow = float(duck_slow)
        self.omega_deadband_comp = float(omega_deadband_comp)
        # False makes labels a function of the current observation (no hidden latch) for behaviour cloning.
        self.threat_latch = bool(threat_latch)
        # Parallel/diverging boats inside the engage radius otherwise keep the dense fallback (starboard bias) on all mission.
        self.dense_closing_only = bool(dense_closing_only)
        self.engaged_cte_fade = float(engaged_cte_fade)
        self._nearest_distance = float('inf')
        self._last_heading_error = 0.0
        self._latched: dict[str, set[str]] = {}
        self.last_engaged = False

    def reset(self) -> None:
        """Clear encounter latches (call at episode boundaries)."""
        self._latched.clear()
        self.last_engaged = False

    def _neighbor_is_threat(self, nb) -> bool:
        """Genuine predicted conflict: converging with CPA inside horizon/margin."""
        d = float(nb.distance)
        if d <= 1e-6 or d > self.gate_engage_distance:
            return False
        rr = (float(nb.rel_x) * float(nb.rel_vx) + float(nb.rel_y) * float(nb.rel_vy)) / max(d, 1e-3)
        if d < self.close_range_distance:
            return rr < -0.01
        rvx, rvy = float(nb.rel_vx), float(nb.rel_vy)
        v_sq = rvx * rvx + rvy * rvy
        if v_sq < 1e-6:
            return False
        tcpa = -(float(nb.rel_x) * rvx + float(nb.rel_y) * rvy) / v_sq
        if tcpa <= 0.0 or tcpa > self.tcpa_gate_horizon:
            return False
        dcpa = math.hypot(float(nb.rel_x) + rvx * tcpa, float(nb.rel_y) + rvy * tcpa)
        return dcpa < self.dcpa_gate_distance

    def _neighbor_released(self, nb) -> bool:
        """Latched neighbour can be dropped: past CPA and opening, or far away."""
        d = float(nb.distance)
        if d > self.gate_release_distance:
            return True
        rvx, rvy = float(nb.rel_vx), float(nb.rel_vy)
        rr = (float(nb.rel_x) * rvx + float(nb.rel_y) * rvy) / max(d, 1e-3)
        v_sq = rvx * rvx + rvy * rvy
        tcpa = -(float(nb.rel_x) * rvx + float(nb.rel_y) * rvy) / v_sq if v_sq >= 1e-6 else 0.0
        return tcpa <= 0.0 and rr > 0.02

    def _gated_neighbors(self, obs: AgentLocalObservation) -> list:
        latched = self._latched.setdefault(str(obs.agent_id), set())
        near = [nb for nb in obs.neighbors
                if 1e-6 < float(nb.distance) <= self.gate_engage_distance]
        if self.dense_closing_only:
            near = [nb for nb in near
                    if (float(nb.rel_x) * float(nb.rel_vx) + float(nb.rel_y) * float(nb.rel_vy)) < -0.01 * float(nb.distance)]
        if not self.threat_latch:
            if len(near) >= self.dense_neighbor_threshold:
                return near
            return [nb for nb in obs.neighbors if self._neighbor_is_threat(nb)]
        if len(near) >= self.dense_neighbor_threshold:
            # Dense convergence: CPA extrapolation untrustworthy, keep v2 caution.
            for nb in near:
                latched.add(str(nb.source_id))
            latched.intersection_update({str(nb.source_id) for nb in obs.neighbors})
            return near
        threats = []
        seen_ids = set()
        for nb in obs.neighbors:
            nb_id = str(nb.source_id)
            seen_ids.add(nb_id)
            if self._neighbor_is_threat(nb):
                latched.add(nb_id)
                threats.append(nb)
            elif nb_id in latched:
                if self._neighbor_released(nb):
                    latched.discard(nb_id)
                else:
                    threats.append(nb)
        latched.intersection_update(seen_ids)
        return threats

    def _avoidance_intent(self, obs: AgentLocalObservation) -> tuple[float, float]:
        """Return (slowdown >= 0, starboard_rate >= 0) from the v2 heuristic."""
        vec = _build_teacher_layout_vector(obs)
        raw = _scripted_policy_action_cluster_colregs_v2(vec)
        slowdown = max(0.0, -float(raw[0]))
        starboard_rate = max(0.0, -float(raw[1]))
        return slowdown, starboard_rate

    def _geometry_tracking_omega(self, obs: AgentLocalObservation) -> float:
        """Track the route directly when no genuine encounter is active."""
        # Env CTE > 0 means the hull is starboard of the route, so a port (+omega)
        # correction is needed; fresh645 used '-' and the two terms cancelled,
        # leaving a 3m starboard drift with ~60deg heading error (SITL 2026-09-02).
        cte_gain = self.engaged_cte_p_gain if self.last_engaged else self.cte_p_gain
        if self.last_engaged and self.engaged_cte_fade > 0.0:
            # Pull back to the route only once the nearest hull is no longer at close quarters.
            near, far = self.engaged_cte_fade, self.engaged_cte_fade + 1.5
            cte_gain *= float(np.clip((self._nearest_distance - near) / (far - near), 0.0, 1.0))
        cte_term = cte_gain * float(obs.cross_track_error)
        if self.cte_term_limit is not None:
            cte_term = float(np.clip(cte_term, -self.cte_term_limit, self.cte_term_limit))
        return self.heading_p_gain * float(obs.heading_error) + cte_term

    def _nearest_closing(self, obs: AgentLocalObservation) -> tuple[float, bool]:
        """Nearest-neighbour distance and whether it is closing (approaching)."""
        nearest = float('inf')
        closing = False
        for nb in obs.neighbors:
            d = float(nb.distance)
            if d <= 1e-6 or d >= nearest:
                continue
            nearest = d
            # range rate < 0 => closing (world-frame rel pos/vel)
            rr = (nb.rel_x * nb.rel_vx + nb.rel_y * nb.rel_vy) / max(d, 1e-3)
            closing = rr < -0.01
        return nearest, closing

    def act(self, obs: AgentLocalObservation) -> np.ndarray:
        """Produce the teacher action for a single agent's observation."""
        self._last_heading_error = float(obs.heading_error)
        if self.conflict_gate:
            threats = self._gated_neighbors(obs)
            self.last_engaged = bool(threats)
            gated_obs = replace(obs, neighbors=threats)
        else:
            self.last_engaged = bool(obs.neighbors)
            gated_obs = obs
        slowdown, starboard_rate = self._avoidance_intent(gated_obs)
        max_omega = self.max_angular_velocity
        if self.duck_offset > 0.0:
            urgency = self._duck_urgency(obs)
            nearest, closing = self._nearest_closing(obs)
            self._nearest_distance = nearest
            if closing and nearest < self.escape_distance:
                urgency = 1.0
            self.last_engaged = urgency > 0.0
            offset = self.duck_offset * urgency
            cte_term = self.cte_p_gain * (float(obs.cross_track_error) - offset)
            if self.cte_term_limit is not None:
                cte_term = float(np.clip(cte_term, -self.cte_term_limit, self.cte_term_limit))
            omega = float(np.clip(self.heading_p_gain * float(obs.heading_error) + cte_term, -max_omega, max_omega))
            return self._format_action(omega, self.cruise_speed * self.duck_slow * urgency)
        if self.giveway_distance > 0.0 and self.giveway_smooth:
            urgency = self._give_way_urgency(obs)
            if urgency is not None:
                self.last_engaged = True
                turning = float(obs.heading_error) < math.radians(60.0)
                omega = -min(self.escape_omega, max_omega) * urgency if turning else 0.0
                return self._format_action(omega, self.cruise_speed * (0.5 + 0.2 * urgency))
            slowdown, starboard_rate = 0.0, 0.0
        elif self.giveway_distance > 0.0:
            giveway = self._must_give_way(obs)
            if giveway == 2 and float(obs.heading_error) < math.radians(60.0):
                self.last_engaged = True
                return self._format_action(-min(self.escape_omega, max_omega), self.cruise_speed)
            if giveway:
                # Offset already achieved: hold the current course until the hull passes.
                self.last_engaged = True
                return self._format_action(0.0, 0.5 * self.cruise_speed)
            slowdown, starboard_rate = 0.0, 0.0

        # Close-range safety augmentation (fixes overtaking/random marginal
        # collisions at min_sep ~0.73 < 0.75). The base v2 heuristic turns too
        # mildly for a catching-up overtaker; here we (a) ramp up the starboard
        # turn early (from ~2.4m) so separation opens BEFORE close quarters, and
        # (b) brake only at very close range (<1.3m) as a last resort.
        nearest, closing = self._nearest_closing(obs)
        self._nearest_distance = nearest
        cr = self.close_range_distance
        if closing and nearest < cr:
            prox = float(np.clip((cr - nearest) / (cr - 0.85), 0.0, 1.0))
            # fresh646: reach full authority (0.70) by 0.85m; 0.52 left dense head-on at 0.73.
            starboard_rate = max(starboard_rate, 0.24 * prox + 0.46 * prox * prox)
            if nearest < self.brake_distance:
                brake = float(np.clip((self.brake_distance - nearest) / (self.brake_distance - 0.75), 0.0, 1.0))
                slowdown = max(slowdown, self.brake_slowdown * brake)

        # At very close range, lateral separation has priority over route
        # tracking: turn away from the closest closing hull to escape a lock.
        if closing and nearest < self.escape_distance:
            closest = min(
                (nb for nb in obs.neighbors if float(nb.distance) > 1e-6),
                key=lambda nb: float(nb.distance),
                default=None,
            )
            if closest is not None:
                body_y = -math.sin(float(obs.yaw)) * float(closest.rel_x) + math.cos(float(obs.yaw)) * float(closest.rel_y)
                # Starboard by default; only a hull clearly on the starboard bow
                # forces a port escape. Removes the +-0.70 sign chatter at
                # body_y~0 (dead-ahead) that a memoryless BC student cannot fit.
                escape_omega = -self.escape_omega if body_y > -0.35 else self.escape_omega
                return self._format_action(float(np.clip(escape_omega, -max_omega, max_omega)), slowdown)

        # BLEND goal-seeking with the starboard avoidance bias (do NOT override).
        # The closed-loop teacher eval showed a pure override makes the craft turn
        # away from every neighbour within lookahead and never reach the goal
        # (min_sep ~4.8m, goal~0). Subtracting the starboard rate from the goal
        # tracking command keeps the craft heading to goal while biasing right for
        # COLREGS give-way, and lets it resume the route once the conflict clears.
        # fresh645: goal_omega is geometry-PD in ALL branches. fresh644 kept
        # raw_angular_z in the engaged branch while BC dropout removed that input
        # slot, making 72.7% of targets unpredictable and corrupting the whole fit.
        goal_omega = self._geometry_tracking_omega(obs)
        omega = float(np.clip(goal_omega - starboard_rate, -max_omega, max_omega))

        return self._format_action(omega, slowdown)

    def _duck_urgency(self, obs: AgentLocalObservation) -> float:
        """0..1 ramp over the last giveway_tcpa seconds before CPA with a hull this boat must give way to."""
        cos_yaw, sin_yaw = math.cos(float(obs.yaw)), math.sin(float(obs.yaw))
        urgency = 0.0
        for nb in obs.neighbors:
            d = float(nb.distance)
            if d <= 1e-6 or d > self.giveway_distance:
                continue
            rx, ry, rvx, rvy = float(nb.rel_x), float(nb.rel_y), float(nb.rel_vx), float(nb.rel_vy)
            v_sq = rvx * rvx + rvy * rvy
            if v_sq < 0.12 ** 2 or (rx * rvx + ry * rvy) >= 0.0:
                continue
            bearing = math.degrees(math.atan2(-sin_yaw * rx + cos_yaw * ry, cos_yaw * rx + sin_yaw * ry))
            own_v = max(float(obs.speed), 0.0)
            nvx, nvy = rvx + own_v * cos_yaw, rvy + own_v * sin_yaw
            head_on = math.hypot(nvx, nvy) > 0.05 and (nvx * cos_yaw + nvy * sin_yaw) < -0.85 * math.hypot(nvx, nvy)
            if not -112.5 <= bearing <= (40.0 if head_on else 10.0):
                continue
            tcpa = -(rx * rvx + ry * rvy) / v_sq
            dcpa = math.hypot(rx + rvx * tcpa, ry + rvy * tcpa)
            if tcpa > self.giveway_tcpa or dcpa >= self.giveway_dcpa:
                continue
            time_ramp = float(np.clip((self.giveway_tcpa - tcpa) / (0.6 * self.giveway_tcpa), 0.0, 1.0))
            miss_fade = float(np.clip((self.giveway_dcpa - dcpa) / (0.4 * self.giveway_dcpa), 0.0, 1.0))
            urgency = max(urgency, time_ramp * miss_fade)
        return urgency

    def _give_way_urgency(self, obs: AgentLocalObservation):
        """None = stand-on; else 0 (hold course) .. 1 (hard starboard) from the worst give-way conflict."""
        cos_yaw, sin_yaw = math.cos(float(obs.yaw)), math.sin(float(obs.yaw))
        urgency = None
        for nb in obs.neighbors:
            d = float(nb.distance)
            if d <= 1e-6 or d > self.giveway_distance:
                continue
            rx, ry, rvx, rvy = float(nb.rel_x), float(nb.rel_y), float(nb.rel_vx), float(nb.rel_vy)
            if (rx * rvx + ry * rvy) / max(d, 1e-3) >= -0.01:
                continue
            bearing = math.degrees(math.atan2(-sin_yaw * rx + cos_yaw * ry, cos_yaw * rx + sin_yaw * ry))
            own_v = max(float(obs.speed), 0.0)
            nvx, nvy = rvx + own_v * cos_yaw, rvy + own_v * sin_yaw
            head_on = math.hypot(nvx, nvy) > 0.05 and (nvx * cos_yaw + nvy * sin_yaw) < -0.85 * math.hypot(nvx, nvy)
            if not -112.5 <= bearing <= (40.0 if head_on else 10.0):
                continue
            v_sq = rvx * rvx + rvy * rvy
            if d < self.escape_distance:
                return 1.0
            if v_sq < 0.12 ** 2:
                continue
            tcpa = -(rx * rvx + ry * rvy) / v_sq
            dcpa = math.hypot(rx + rvx * tcpa, ry + rvy * tcpa)
            if not 0.0 < tcpa <= self.giveway_tcpa or dcpa >= 1.6 * self.giveway_dcpa:
                continue
            value = float(np.clip((1.6 * self.giveway_dcpa - dcpa) / (0.6 * self.giveway_dcpa), 0.0, 1.0))
            urgency = value if urgency is None else max(urgency, value)
        return urgency

    def _must_give_way(self, obs: AgentLocalObservation) -> int:
        """2 = turn starboard, 1 = hold offset course, 0 = stand-on, for converging hulls on the starboard bow/ahead."""
        cos_yaw, sin_yaw = math.cos(float(obs.yaw)), math.sin(float(obs.yaw))
        state = 0
        for nb in obs.neighbors:
            d = float(nb.distance)
            if d <= 1e-6 or d > self.giveway_distance:
                continue
            rx, ry, rvx, rvy = float(nb.rel_x), float(nb.rel_y), float(nb.rel_vx), float(nb.rel_vy)
            if (rx * rvx + ry * rvy) / max(d, 1e-3) >= -0.01:
                continue
            bearing = math.degrees(math.atan2(-sin_yaw * rx + cos_yaw * ry, cos_yaw * rx + sin_yaw * ry))
            # Reciprocal course (head-on): keep giving way after our own starboard turn moves the hull to port.
            own_v = max(float(obs.speed), 0.0)
            nvx, nvy = rvx + own_v * cos_yaw, rvy + own_v * sin_yaw
            head_on = math.hypot(nvx, nvy) > 0.05 and (nvx * cos_yaw + nvy * sin_yaw) < -0.85 * math.hypot(nvx, nvy)
            if not -112.5 <= bearing <= (40.0 if head_on else 10.0):
                continue
            v_sq = rvx * rvx + rvy * rvy
            if v_sq < 0.12 ** 2 and d >= self.escape_distance:
                continue  # near-parallel convoy: no crossing conflict to yield in
            tcpa = -(rx * rvx + ry * rvy) / v_sq if v_sq >= 1e-6 else 0.0
            dcpa = math.hypot(rx + rvx * tcpa, ry + rvy * tcpa) if tcpa > 0.0 else d
            if d < self.escape_distance or (0.0 < tcpa <= self.giveway_tcpa and dcpa < self.giveway_dcpa):
                return 2
            if 0.0 < tcpa <= self.giveway_tcpa and dcpa < 1.6 * self.giveway_dcpa:
                state = 1
        return state

    def _format_action(self, omega: float, slowdown: float) -> np.ndarray:
        """Convert the teacher's physical command to the selected action mode."""
        # Throttle: full nav speed unless yielding; never stop / reverse.
        scale = 1.0 - slowdown / max(self.cruise_speed, 1e-3)
        if self.turn_speed_governor:
            cos_gate = max(0.0, math.cos(min(abs(self._last_heading_error), math.pi))) ** self.governor_power
            scale = min(scale, self.governor_floor + (1.0 - self.governor_floor) * cos_gate)
        scale = float(np.clip(scale, self.action_speed_scale_min, 1.0))
        if self.omega_deadband_comp > 0.0:
            # Vehicle ignores the first ~0.15 rad/s of yaw-rate demand; ramp the offset in so tiny corrections stay tiny.
            omega = omega + math.copysign(self.omega_deadband_comp * min(1.0, abs(omega) / 0.1), omega)
            omega = float(np.clip(omega, -self.max_angular_velocity, self.max_angular_velocity))

        if self.action_mode == 'angular_only':
            return np.asarray([omega], dtype=np.float32)
        if self.action_mode == 'speed_scale':
            return np.asarray([scale, omega], dtype=np.float32)
        # 'full': emit an absolute forward speed = scale * cruise.
        linear_x = float(np.clip(scale * self.cruise_speed, 0.0, self.linear_speed_limit))
        return np.asarray([linear_x, omega], dtype=np.float32)
