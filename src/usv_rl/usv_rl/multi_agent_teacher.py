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


def _smoothstep(x: float) -> float:
    x = min(max(float(x), 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


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
        heading_sqrt_gain: float = 0.0,
        duck_smooth: bool = False,
        duck_release: float = 8.0,
        convoy_distance: float = 0.0,
        turn_omega_limit: float = 0.0,
        avoid_heading: float = 0.0,
        avoid_clearance: float = 0.0,
        tandem_clean: bool = False,
        static_hull: bool = False,
        park_slow: float = 0.0,
        veteran_flow: bool = False,
        sleek_direct: bool = False,
        close_brush: bool = False,
    ) -> None:
        if action_mode not in ('speed_scale', 'full', 'angular_only'):
            raise ValueError(f'unsupported action_mode={action_mode!r}')
        self.action_mode = action_mode
        self.veteran_flow = bool(veteran_flow)
        self.sleek_direct = bool(sleek_direct)
        self.close_brush = bool(close_brush)
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
        self.heading_sqrt_gain = float(heading_sqrt_gain)
        self.duck_smooth = bool(duck_smooth)
        self.duck_release = max(float(duck_release), 1e-3)
        # >0: same-course hulls closer than this are pushed apart sideways (and the follower eases off) instead of running in tandem.
        self.convoy_distance = float(convoy_distance)
        # >0 caps |omega| so a corner is an arc of radius ~speed/limit, not a spot turn.
        self.turn_omega_limit = float(turn_omega_limit)
        # >0 (rad): while a conflict is urgent, aim this far off the route toward the pass side so the first yaw command clears the ~0.15 rad/s deadband.
        self.avoid_heading = float(abs(avoid_heading))
        # >0 (m): the avoidance bias runs until the predicted CPA puts the hull this far abeam (route frame), so a hull dead ahead on the route is cleared by a full margin.
        self.avoid_clearance = float(abs(avoid_clearance))
        # Same-direction hulls never trigger give-way: the follower only eases off, abreast hulls part by a fixed sign, the leader ignores the follower.
        self.tandem_clean = bool(tandem_clean)
        # A parked hull is a static obstacle: urgency and pass side come from route-frame geometry only (no relative-speed gate, no yaw-wobble flip).
        self.static_hull = bool(static_hull)
        # Extra slowdown fraction (of cruise) while closing on a parked hull, so the boat is slow before it has to turn.
        self.park_slow = float(abs(park_slow))
        self._park_near = 0.0
        self._duck_left = 0.0
        self._duck_has_cpa = False
        self._duck_dir = 1.0
        self._convoy_slow = 0.0
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

    def _act_veteran_flow(self, obs: AgentLocalObservation) -> np.ndarray:
        """Veteran-driver multi-neighbor smooth flow controller.

        Key properties:
        1. Superposes smooth lateral corridor offsets, heading anticipation, and
           pass-astern throttle modulation across ALL active neighbors (no single-
           neighbor argmax jumping in multi-USV convergence or parked-plus-moving traffic).
        2. Exact route-anchored static-hull geometry (target_cte = (cte - lat_r) + dir * clearance)
           so a boat already off-route never mistakes its own CTE for hull clearance.
        3. True crossing pass-astern: right-side crossers trigger smooth yield-behind
           (starboard bias + moderate throttle ease), NEVER convoy-push escorting.
        4. Nominal-speed lookahead floor prevents mutual low-speed urgency extinction
           and hesitation stalls, maintaining brisk steerage and ~0.8 m rounded arcs.
        """
        cos_yaw, sin_yaw = math.cos(float(obs.yaw)), math.sin(float(obs.yaw))
        own_v = max(float(obs.speed), 0.0)
        he_raw = float(obs.heading_error)
        phi = float(obs.yaw) + he_raw
        cos_phi, sin_phi = math.cos(phi), math.sin(phi)
        cte = float(obs.cross_track_error)

        engage_dist = max(self.giveway_distance, 8.5)
        clearance_target = max(self.avoid_clearance, 1.85)
        park_margin = max(1.55, 0.82 * clearance_target)
        base_offset = max(self.duck_offset, 1.35)
        head_gain = max(self.avoid_heading, 0.62)

        # Accumulate directional corridor shifts & throttle modulation smoothly
        pos_cte_target = 0.0   # starboard corridor demand (>0)
        neg_cte_target = 0.0   # port corridor demand (<0)
        pos_cte_weight = 0.0
        neg_cte_weight = 0.0
        head_bias = 0.0        # >0 biases starboard (-omega), <0 biases port (+omega)
        slow_frac = 0.0        # fraction of cruise speed to ease off
        repel_bias = 0.0       # close-quarters smooth hull cushion
        max_urg = 0.0
        nearest = float('inf')

        # Nominal forward speed floor so slowing down never blinds CPA lookahead
        v_nom = max(own_v, 0.32)

        for nb in obs.neighbors:
            d = float(nb.distance)
            if d <= 1e-6:
                continue
            nearest = min(nearest, d)
            if d > engage_dist:
                continue

            rx, ry = float(nb.rel_x), float(nb.rel_y)
            rvx, rvy = float(nb.rel_vx), float(nb.rel_vy)
            body_x = cos_yaw * rx + sin_yaw * ry
            body_y = -sin_yaw * rx + cos_yaw * ry       # >0: port, <0: starboard
            along_r = cos_phi * rx + sin_phi * ry      # >0: ahead along route
            lat_r = -sin_phi * rx + cos_phi * ry       # >0: left of us in route frame

            nvx = rvx + own_v * cos_yaw
            nvy = rvy + own_v * sin_yaw
            n_speed = math.hypot(nvx, nvy)

            # --- 1. Static / holding hull: route-anchored exact clearance (flip-free watershed) ---
            if n_speed < 0.09:
                if along_r < -2.2:
                    continue
                # Watershed pushed outside the collision corridor (lat_r ~ -0.95m):
                # any hull dead-ahead, on port, or up to 0.85m to starboard is passed cleanly on
                # starboard (+1), eliminating +/-omega sign flips inside the danger zone that
                # cause a deterministic BC student to average to omega=0.
                stbd_w = _smoothstep((lat_r + 1.05) / 0.25)  # 1.0 when lat_r >= -0.80, 0.0 when lat_r <= -1.05
                if along_r >= -0.25:
                    timing = _smoothstep((engage_dist - along_r) / (0.42 * engage_dist))
                else:
                    timing = _smoothstep(1.0 + (along_r + 0.25) / 2.0)
                nb_cte = cte - lat_r  # exact hull CTE relative to route

                if stbd_w > 1e-4:
                    clear_s = lat_r
                    miss_s = _smoothstep((park_margin + 0.45 - clear_s) / 0.85)
                    u_s = stbd_w * miss_s * timing
                    if u_s > 1e-4:
                        max_urg = max(max_urg, u_s)
                        req_cte_s = nb_cte + park_margin
                        pos_cte_target = max(pos_cte_target, max(0.0, req_cte_s) * u_s)
                        pos_cte_weight = max(pos_cte_weight, u_s)
                        need_s = float(np.clip((park_margin - clear_s) / park_margin, 0.0, 1.0))
                        near_boost = 1.0 + 0.35 * _smoothstep((4.5 - max(along_r, 0.0)) / 2.5)
                        head_bias += head_gain * near_boost * u_s * need_s
                        slow_frac = max(slow_frac, 0.16 * u_s * need_s * _smoothstep((4.2 - max(along_r, 0.0)) / 2.4))

                port_w = 1.0 - stbd_w
                if port_w > 1e-4:
                    clear_p = -lat_r  # already > 0.80m when port_w > 0
                    miss_p = _smoothstep((park_margin + 0.35 - clear_p) / 0.70)
                    u_p = port_w * miss_p * timing
                    if u_p > 1e-4:
                        max_urg = max(max_urg, u_p)
                        req_cte_p = nb_cte - park_margin
                        neg_cte_target = min(neg_cte_target, min(0.0, req_cte_p) * u_p)
                        neg_cte_weight = max(neg_cte_weight, u_p)
                        need_p = float(np.clip((park_margin - clear_p) / park_margin, 0.0, 1.0))
                        head_bias -= 0.48 * u_p * need_p

                if d < 2.15 and along_r > -0.65:
                    prox = _smoothstep((2.15 - d) / 1.25)
                    p_dir = 1.0 if lat_r >= -0.90 else -1.0
                    repel_bias += p_dir * 0.42 * prox
                continue

            # --- 2. Moving neighbor kinematics (actual + nominal-speed lookahead) ---
            course_body = (nvx * cos_yaw + nvy * sin_yaw) / n_speed
            course_route = (nvx * cos_phi + nvy * sin_phi) / n_speed
            lat_vel_body = (-nvx * sin_yaw + nvy * cos_yaw) / n_speed   # >0: moving R->L across bow
            lat_vel_route = (-nvx * sin_phi + nvy * cos_phi) / n_speed

            # True same-direction convoy ONLY when courses align within ~38 deg (never on 72 deg crossings!)
            same_w = _smoothstep((min(course_body, course_route) - 0.72) / 0.18)
            if same_w > 0.0 and d < 2.4:
                conv = same_w * _smoothstep((2.4 - d) / 1.1)
                if body_x > 0.15:
                    slow_frac = max(slow_frac, 0.32 * conv * _smoothstep(body_x / 0.8))
                if d < 1.65 and body_x > -0.5:
                    side_dir = 1.0 if body_y >= 0.0 else -1.0
                    repel_bias += side_dir * 0.25 * conv
                max_urg = max(max_urg, 0.5 * conv)

            # Lookahead relative velocity using nominal own speed along current heading/route blend
            rvx_nom = nvx - v_nom * (0.6 * cos_yaw + 0.4 * cos_phi)
            rvy_nom = nvy - v_nom * (0.6 * sin_yaw + 0.4 * sin_phi)
            v_sq_real = rvx * rvx + rvy * rvy
            v_sq_nom = rvx_nom * rvx_nom + rvy_nom * rvy_nom

            tcpa_r = -(rx * rvx + ry * rvy) / v_sq_real if v_sq_real > 1e-4 else -1.0
            dcpa_r = math.hypot(rx + rvx * max(tcpa_r, 0.0), ry + rvy * max(tcpa_r, 0.0))
            tcpa_n = -(rx * rvx_nom + ry * rvy_nom) / v_sq_nom if v_sq_nom > 1e-4 else -1.0
            dcpa_n = math.hypot(rx + rvx_nom * max(tcpa_n, 0.0), ry + rvy_nom * max(tcpa_n, 0.0))

            tcpa = tcpa_n if (tcpa_r <= 0.0 and tcpa_n > 0.0) else (min(tcpa_r, tcpa_n) if (tcpa_r > 0.0 and tcpa_n > 0.0) else tcpa_r)
            dcpa = min(dcpa_r, dcpa_n)

            # --- 2A. Head-on / oblique head-on (Rule 14: mutual early starboard arc, brisk flow) ---
            head_on_w = max(
                _smoothstep((-course_body - 0.42) / 0.30),
                _smoothstep((-course_route - 0.42) / 0.30),
            ) * (1.0 - same_w)
            if head_on_w > 1e-4 and along_r > -2.4:
                miss_ho = _smoothstep((self.giveway_dcpa - dcpa) / (0.42 * self.giveway_dcpa))
                if tcpa >= 0.0:
                    time_ho = _smoothstep((22.0 - tcpa) / 11.0)
                else:
                    time_ho = _smoothstep(1.0 + along_r / 2.4)
                u_ho = head_on_w * miss_ho * time_ho
                if u_ho > 1e-4:
                    max_urg = max(max_urg, u_ho)
                    t_cpa = max(tcpa_n, 0.0)
                    cpa_left = -sin_phi * (rx + rvx_nom * t_cpa) + cos_phi * (ry + rvy_nom * t_cpa)
                    need_ho = float(np.clip((clearance_target - cpa_left) / clearance_target, 0.0, 1.0))
                    # Anchor starboard offset so even off-center oblique lanes part by >= 1.6m
                    nb_cpa_cte = cte - cpa_left
                    req_cte = max(base_offset, nb_cpa_cte + 0.85 * clearance_target)
                    req_cte = min(req_cte, 2.25)
                    pos_cte_target = max(pos_cte_target, req_cte * u_ho * max(need_ho, 0.35))
                    pos_cte_weight = max(pos_cte_weight, u_ho)
                    head_bias += head_gain * u_ho * need_ho
                    # Veteran driver keeps flow speed (~84-90% cruise) in head-on arcs!
                    slow_frac = max(slow_frac, 0.14 * u_ho * need_ho)

            # --- 2B. Starboard crossing (Rule 15: pass-astern behind stand-on vessel, zero escort shove) ---
            cross_stbd_w = max(
                _smoothstep((lat_vel_body - 0.12) / 0.32),
                _smoothstep((lat_vel_route - 0.12) / 0.32),
            ) * (1.0 - same_w) * (1.0 - 0.75 * head_on_w)
            # Engage when hull is on starboard/bow and has not yet cleared well to our port (>1.35m)
            if cross_stbd_w > 1e-4 and along_r > -1.2 and lat_r < 1.45:
                miss_cr = _smoothstep((3.1 - dcpa) / 1.25)
                time_cr = _smoothstep((18.0 - tcpa) / 9.5) if tcpa >= 0.0 else _smoothstep(1.0 + tcpa / 2.2)
                port_clear_fade = _smoothstep((1.45 - lat_r) / 0.75)
                u_cr = cross_stbd_w * miss_cr * time_cr * port_clear_fade
                if u_cr > 1e-4:
                    max_urg = max(max_urg, u_cr)
                    t_cpa = max(tcpa_n, 0.0)
                    cpa_left = -sin_phi * (rx + rvx_nom * t_cpa) + cos_phi * (ry + rvy_nom * t_cpa)
                    need_cr = float(np.clip((1.65 - cpa_left) / 1.65, 0.0, 1.0))
                    # Aim starboard toward the crosser's stern & ease throttle so it clears ahead cleanly
                    pos_cte_target = max(pos_cte_target, 1.15 * u_cr * max(need_cr, 0.30))
                    pos_cte_weight = max(pos_cte_weight, u_cr)
                    head_bias += 0.56 * u_cr * max(need_cr, 0.25)
                    slow_frac = max(slow_frac, 0.34 * u_cr * _smoothstep((6.0 - d) / 3.2))

            # --- 2C. Defensive yield for a close port-side crosser already cutting across our bow ---
            cross_port_w = max(
                _smoothstep((-lat_vel_body - 0.18) / 0.30),
                _smoothstep((-lat_vel_route - 0.18) / 0.30),
            ) * (1.0 - same_w) * (1.0 - head_on_w)
            if cross_port_w > 1e-4 and d < 3.8 and body_x > 0.0 and lat_r > -1.15:
                miss_def = _smoothstep((1.65 - dcpa) / 0.70)
                prox_def = _smoothstep((3.8 - d) / 1.8)
                u_def = cross_port_w * miss_def * prox_def * _smoothstep((lat_r + 1.15) / 0.75)
                if u_def > 1e-4:
                    max_urg = max(max_urg, u_def)
                    # Ease throttle and tuck slightly toward its stern (port, -cte) instead of ramming its beam
                    neg_cte_target = min(neg_cte_target, -0.75 * u_def)
                    neg_cte_weight = max(neg_cte_weight, 0.7 * u_def)
                    head_bias -= 0.32 * u_def
                    slow_frac = max(slow_frac, 0.30 * u_def)

            # --- 2D. Close-quarters moving anti-shove cushion (d < 2.2m) ---
            if d < 2.20 and body_x > -0.50:
                prox = _smoothstep((2.20 - d) / 1.20)
                # Default starboard turn unless neighbor is clearly on starboard AND not a starboard-crossing stern pass
                if body_y >= -0.25 or cross_stbd_w > 0.35 or head_on_w > 0.35:
                    repel_bias += 0.38 * prox
                else:
                    repel_bias -= 0.34 * prox

        self._nearest_distance = nearest
        self.last_engaged = max_urg > 1e-3 or abs(repel_bias) > 1e-3

        # Combine starboard and port corridor offsets smoothly
        net_offset = pos_cte_target - abs(neg_cte_target) if (pos_cte_weight >= neg_cte_weight) else (pos_cte_target * 0.4 + neg_cte_target)
        # Fade back to route cleanly when unobstructed; soften CTE pull-back while actively avoiding
        eff_cte_gain = self.cte_p_gain * (1.0 - 0.35 * min(1.0, max_urg) * (1.0 - max(pos_cte_weight, neg_cte_weight)))
        cte_term = eff_cte_gain * (cte - net_offset)
        if self.cte_term_limit is not None:
            cte_term = float(np.clip(cte_term, -self.cte_term_limit, self.cte_term_limit))

        total_head_bias = float(np.clip(head_bias + repel_bias, -0.95, 0.95))
        he_eff = he_raw - total_head_bias
        heading_term = self.heading_p_gain * he_eff

        omega_cap = self.max_angular_velocity if self.turn_omega_limit <= 0.0 else min(self.max_angular_velocity, self.turn_omega_limit)
        omega = float(np.clip(heading_term + cte_term, -omega_cap, omega_cap))

        # Never stall below brisk steerage speed during avoidance (scale >= 0.56 unless tight waypoint cornering)
        slow_frac = float(np.clip(slow_frac, 0.0, 0.42))
        return self._format_action(omega, self.cruise_speed * slow_frac)

    def _act_sleek_direct(self, obs: AgentLocalObservation) -> np.ndarray:
        """Sleek-Direct (Point-and-Glide) Veteran Controller (a21).

        Eliminates all redundant S-bends and wide bows while maintaining >1.25 m
        close-pass margins and ~0.8 m rounded corner arcs:
        1. Zero backward chord-CTE pull inside a wide +/-2.2m corridor: once a corner
           turn or pass finishes and the bow points at the goal (he ~ 0), omega = 0
           (laser-straight glide to goal, zero post-corner S-bend).
        2. Minimal tangent-angle deflection in the goal line-of-sight frame:
           deflection = atan2(missing_clearance, forward_range), engaging at <=6.8m
           and releasing immediately past abeam (along_g < -0.45m).
        3. Corner-first suppression: while turning a sharp waypoint (|he| > 55 deg)
           with no immediate neighbor (<2.9m), complete the rounded ~0.8m corner
           cleanly without mid-turn avoidance overshoot.
        4. Port-bow crossers use smooth throttle feathering ONLY (zero port steering
           tail-hook).
        """
        cos_yaw, sin_yaw = math.cos(float(obs.yaw)), math.sin(float(obs.yaw))
        own_v = max(float(obs.speed), 0.0)
        he_raw = float(obs.heading_error)
        phi = float(obs.yaw) + he_raw  # exact bearing to current goal / virtual goal
        cos_phi, sin_phi = math.cos(phi), math.sin(phi)
        cte = float(obs.cross_track_error)

        engage_dist = max(self.giveway_distance, 7.2)
        ho_clearance = max(self.avoid_clearance, 1.62)
        park_margin = max(1.55, 0.96 * ho_clearance)
        v_nom = max(own_v, 0.32)

        head_bias = 0.0       # >0 turns starboard (-omega), <0 turns port (+omega)
        slow_frac = 0.0
        repel_bias = 0.0
        max_urg = 0.0
        nearest = float('inf')

        # Suppress distant avoidance heading bias during sharp >55 deg waypoint cornering
        corner_suppress = 1.0 - 0.80 * _smoothstep((abs(he_raw) - 0.96) / 0.55)

        for nb in obs.neighbors:
            d = float(nb.distance)
            if d <= 1e-6:
                continue
            nearest = min(nearest, d)
            if d > engage_dist:
                continue

            rx, ry = float(nb.rel_x), float(nb.rel_y)
            rvx, rvy = float(nb.rel_vx), float(nb.rel_vy)
            body_x = cos_yaw * rx + sin_yaw * ry
            body_y = -sin_yaw * rx + cos_yaw * ry
            along_g = cos_phi * rx + sin_phi * ry      # >0: ahead along goal LOS
            lat_g = -sin_phi * rx + cos_phi * ry       # >0: left of goal LOS

            nvx = rvx + own_v * cos_yaw
            nvy = rvy + own_v * sin_yaw
            n_speed = math.hypot(nvx, nvy)

            # --- 1. Static / holding hull: ALWAYS pass on starboard (+1) unless already >1.45m clear on starboard ---
            if n_speed < 0.09:
                if along_g < -1.1:
                    continue
                # Universal starboard pass eliminates the bimodal port/starboard watershed inside
                # the danger corridor (`lat_g > -1.45m`), so an unimodal BC network NEVER averages
                # opposing +/-omega branches to zero! If `lat_g <= -1.45m`, the hull is already >1.45m
                # clear on our right, so zero action (`omega = 0`) glides straight past safely.
                stbd_w = _smoothstep((lat_g + 1.55) / 0.20)  # 1.0 for lat_g >= -1.35m, 0.0 for lat_g <= -1.55m
                if stbd_w > 1e-4:
                    timing = (
                        _smoothstep((engage_dist - along_g) / (0.40 * engage_dist))
                        if along_g >= -0.15
                        else _smoothstep(1.0 + (along_g + 0.15) / 0.95)
                    )
                    need_s = float(np.clip((park_margin - lat_g) / park_margin, 0.0, 1.0))
                    u_s = stbd_w * _smoothstep((park_margin + 0.30 - lat_g) / 0.68) * timing
                    if u_s > 1e-4:
                        max_urg = max(max_urg, u_s)
                        req_angle = math.atan2(max(0.0, park_margin - lat_g), max(along_g * 0.72, 0.75))
                        head_bias += min(0.82, 1.32 * req_angle) * u_s
                        slow_frac = max(slow_frac, 0.18 * u_s * need_s * _smoothstep((4.2 - max(along_g, 0.0)) / 2.2))
                if d < 2.05 and along_g > -0.40 and lat_g > -1.45:
                    repel_bias += 0.42 * _smoothstep((2.05 - d) / 1.05)
                continue

            # --- 2. Moving neighbor kinematics in goal-LOS frame ---
            course_body = (nvx * cos_yaw + nvy * sin_yaw) / n_speed
            course_goal = (nvx * cos_phi + nvy * sin_phi) / n_speed
            lat_vel_body = (-nvx * sin_yaw + nvy * cos_yaw) / n_speed
            lat_vel_goal = (-nvx * sin_phi + nvy * cos_phi) / n_speed

            same_w = _smoothstep((min(course_body, course_goal) - 0.75) / 0.16)
            if same_w > 0.0 and d < 2.2:
                conv = same_w * _smoothstep((2.2 - d) / 1.0)
                if body_x > 0.15:
                    slow_frac = max(slow_frac, 0.28 * conv * _smoothstep(body_x / 0.8))
                if d < 1.55 and body_x > -0.35:
                    repel_bias += (1.0 if body_y >= 0.0 else -1.0) * 0.22 * conv
                max_urg = max(max_urg, 0.4 * conv)

            rvx_nom = nvx - v_nom * (0.5 * cos_yaw + 0.5 * cos_phi)
            rvy_nom = nvy - v_nom * (0.5 * sin_yaw + 0.5 * sin_phi)
            v_sq_r = rvx * rvx + rvy * rvy
            v_sq_n = rvx_nom * rvx_nom + rvy_nom * rvy_nom
            tcpa_r = -(rx * rvx + ry * rvy) / v_sq_r if v_sq_r > 1e-4 else -1.0
            dcpa_r = math.hypot(rx + rvx * max(tcpa_r, 0.0), ry + rvy * max(tcpa_r, 0.0))
            tcpa_n = -(rx * rvx_nom + ry * rvy_nom) / v_sq_n if v_sq_n > 1e-4 else -1.0
            dcpa_n = math.hypot(rx + rvx_nom * max(tcpa_n, 0.0), ry + rvy_nom * max(tcpa_n, 0.0))
            tcpa = tcpa_n if (tcpa_r <= 0.0 and tcpa_n > 0.0) else (min(tcpa_r, tcpa_n) if (tcpa_r > 0.0 and tcpa_n > 0.0) else tcpa_r)
            dcpa = min(dcpa_r, dcpa_n)

            eff_sup = 1.0 if d < 2.9 else corner_suppress

            # --- 2A. Head-on / oblique reciprocal: crisp starboard tangent glide ---
            head_on_w = max(
                _smoothstep((-course_body - 0.44) / 0.28),
                _smoothstep((-course_goal - 0.44) / 0.28),
            ) * (1.0 - same_w)
            if head_on_w > 1e-4 and along_g > -0.55:
                miss_ho = _smoothstep((2.45 - dcpa) / 0.95)
                time_ho = _smoothstep((16.5 - tcpa) / 8.5) if tcpa >= 0.0 else _smoothstep(1.0 + along_g / 0.55)
                u_ho = head_on_w * miss_ho * time_ho * eff_sup
                if u_ho > 1e-4:
                    max_urg = max(max_urg, u_ho)
                    t_cpa = max(tcpa_n, 0.0)
                    cpa_left = -sin_phi * (rx + rvx_nom * t_cpa) + cos_phi * (ry + rvy_nom * t_cpa)
                    deficit = max(0.0, ho_clearance - cpa_left)
                    req_ang = math.atan2(0.76 * deficit, max(along_g * 0.52, 0.95))
                    head_bias += min(0.60, 1.22 * req_ang) * u_ho
                    slow_frac = max(slow_frac, 0.10 * u_ho * float(np.clip(deficit / ho_clearance, 0.0, 1.0)))

            # --- 2B. Starboard crossing: decisive stern-aim + throttle feathering ---
            cross_stbd_w = max(
                _smoothstep((lat_vel_body - 0.12) / 0.30),
                _smoothstep((lat_vel_goal - 0.12) / 0.30),
            ) * (1.0 - same_w) * (1.0 - 0.75 * head_on_w)
            if cross_stbd_w > 1e-4 and along_g > -0.60 and lat_g < 1.35:
                miss_cr = _smoothstep((2.70 - dcpa) / 1.05)
                time_cr = _smoothstep((16.5 - tcpa) / 8.5) if tcpa >= 0.0 else _smoothstep(1.0 + tcpa / 1.5)
                u_cr = cross_stbd_w * miss_cr * time_cr * _smoothstep((1.35 - lat_g) / 0.70) * eff_sup
                if u_cr > 1e-4:
                    max_urg = max(max_urg, u_cr)
                    t_cpa = max(tcpa_n, 0.0)
                    cpa_left = -sin_phi * (rx + rvx_nom * t_cpa) + cos_phi * (ry + rvy_nom * t_cpa)
                    need_cr = float(np.clip((1.55 - cpa_left) / 1.55, 0.0, 1.0))
                    head_bias += 0.52 * u_cr * max(need_cr, 0.25)
                    slow_frac = max(slow_frac, 0.34 * u_cr * _smoothstep((5.8 - d) / 3.0))

            # --- 2C. Close port-bow crosser: throttle feathering ONLY (zero port tail-hook!) ---
            cross_port_w = max(
                _smoothstep((-lat_vel_body - 0.20) / 0.28),
                _smoothstep((-lat_vel_goal - 0.20) / 0.28),
            ) * (1.0 - same_w) * (1.0 - head_on_w)
            if cross_port_w > 1e-4 and d < 3.2 and body_x > 0.0 and lat_g > -0.95:
                u_def = (
                    cross_port_w
                    * _smoothstep((1.45 - dcpa) / 0.58)
                    * _smoothstep((3.2 - d) / 1.5)
                    * _smoothstep((lat_g + 0.95) / 0.58)
                )
                if u_def > 1e-4:
                    max_urg = max(max_urg, u_def)
                    slow_frac = max(slow_frac, 0.28 * u_def)

            # --- 2D. Close hull cushion (<1.90m) ---
            if d < 1.90 and body_x > -0.35:
                prox = _smoothstep((1.90 - d) / 0.95)
                if body_y >= -0.25 or cross_stbd_w > 0.35 or head_on_w > 0.35:
                    repel_bias += 0.38 * prox
                else:
                    repel_bias -= 0.32 * prox

        self._nearest_distance = nearest
        self.last_engaged = max_urg > 1e-3 or abs(repel_bias) > 1e-3

        # Zero chord-CTE pull inside +/-2.2m corridor; gentle fence only beyond 2.2m
        fence_cte = 0.0
        if abs(cte) > 2.2 and abs(he_raw) < 0.65:
            fence_cte = float(np.clip(0.06 * (cte - math.copysign(2.2, cte)), -0.06, 0.06))

        total_bias = float(np.clip(head_bias + repel_bias, -0.84, 0.84))
        he_eff = he_raw - total_bias
        omega_cap = self.max_angular_velocity if self.turn_omega_limit <= 0.0 else min(self.max_angular_velocity, self.turn_omega_limit)
        omega = float(np.clip(self.heading_p_gain * he_eff + fence_cte, -omega_cap, omega_cap))

        slow_frac = float(np.clip(slow_frac, 0.0, 0.38))
        return self._format_action(omega, self.cruise_speed * slow_frac)

    def _act_close_brush(self, obs: AgentLocalObservation) -> np.ndarray:
        """0.60m x 0.30m Monohull Smart Bilateral Pass-Side & Rounded Constant-Curvature
        Flow Controller (a28).

        Key physical & geometric guarantees for the 0.60m x 0.30m hull:
        1. Smart Bilateral Static-Hull Bypass ("该左绕就左绕，该右绕就右绕"):
           - Geometric watershed centered at `lat_r = -0.08m` (slight COLREGs starboard tie-break
             only for dead-ahead `lat_r ~ 0` hulls):
             * `lat_r >= -0.05m` (dead-ahead or port): smooth starboard arc (`pos_off = nb_cte + park_sep`).
             * `lat_r <= -0.11m` (already on starboard of us, e.g. Pentagram Step 1 `usv_03` at `-0.45m`
               and Run 3 `usv_02 -> usv_04/05/01` at `-0.23m..-0.87m`): natural shorter PORT glide
               (`neg_off = nb_cte - park_sep`), never crossing across a starboard hull's bow!
        2. Green-to-Green Preserving Head-On & Crossing:
           - Standard COLREGs Rule 14 starboard shift for dead-ahead / port-offset head-ons (`lat_r > -0.42m`).
           - When oncoming vessels are already safely starboard-separated (`lat_r < -0.52m`), preserve
             the natural green-to-green port pass instead of S-turning across each other's bows!
        3. Rounded Constant-Curvature Cornering & Raised-Cosine Avoidance Arcs ("弧度圆润"):
           - Zero blind pre-waypoint braking (`pre_corner_slow = 0`), keeping polygon/circle waypoints
             (e.g. 10-gon / Figure-8) flowing at ~95-100% cruise in one continuous circle.
           - Smooth constant-radius corner law (`R_turn ~ 0.85..1.15m`, `v >= 0.44*cruise`, `|omega| <= 0.48`)
             eliminates sharp pivot-in-place corners (`R = 0.16..0.25m`) while keeping overshoot low.
        """
        def _wrap(a: float) -> float:
            return (a + math.pi) % (2.0 * math.pi) - math.pi

        cos_yaw, sin_yaw = math.cos(float(obs.yaw)), math.sin(float(obs.yaw))
        own_v = max(float(obs.speed), 0.0)
        he_goal = float(obs.heading_error)
        d2g = max(float(obs.distance_to_goal), 1e-3)
        raw_cte = float(obs.raw_cross_track_error)

        sin_a = float(np.clip(raw_cte / max(d2g, 0.85), -0.85, 0.85))
        he_route = _wrap(he_goal - math.asin(sin_a))
        phi_r = float(obs.yaw) + he_route
        cos_r, sin_r = math.cos(phi_r), math.sin(phi_r)

        engage_dist = 6.2
        park_engage = 5.6
        ho_sep = 0.84
        park_sep = 0.80
        v_nom = max(own_v, 0.36)

        pos_off = 0.0
        neg_off = 0.0
        mov_pos_off = 0.0
        mov_urg = 0.0
        head_nudge = 0.0
        stat_port_nudge = 0.0
        slow_frac = 0.0
        repel_bias = 0.0
        stat_port_repel = 0.0
        max_urg = 0.0
        nearest = float('inf')
        omega_min_guard = -99.0
        omega_max_guard = +99.0
        stat_omega_min_guard = -99.0

        corner_sup = 1.0 - 0.30 * _smoothstep((abs(he_route) - 1.05) / 0.55)

        for nb in obs.neighbors:
            d = float(nb.distance)
            if d <= 1e-6:
                continue
            nearest = min(nearest, d)
            if d > park_engage:
                continue

            rx, ry = float(nb.rel_x), float(nb.rel_y)
            rvx, rvy = float(nb.rel_vx), float(nb.rel_vy)
            bx = cos_yaw * rx + sin_yaw * ry
            by = -sin_yaw * rx + cos_yaw * ry           # >0: port, <0: starboard
            along_r = cos_r * rx + sin_r * ry          # >0: ahead along route
            lat_r = -sin_r * rx + cos_r * ry           # >0: port of us, <0: starboard of us
            nb_cte = raw_cte - lat_r                   # >0: neighbor starboard of route, <0: port of route

            nvx = rvx + own_v * cos_yaw
            nvy = rvy + own_v * sin_yaw
            nsp = math.hypot(nvx, nvy)

            # --- 1. Static / holding hull: Route-Aware Cost-Optimal Bilateral Bypass (a30) ---
            # Evaluates BOTH lateral shift from current position AND deviation from original route (`nb_cte`):
            #   y_R = nb_cte + park_sep  (starboard bypass lane)
            #   y_L = nb_cte - park_sep  (port bypass lane)
            #   C_R = max(0, y_R - raw_cte) + 0.72 * abs(y_R)
            #   C_L = max(0, raw_cte - y_L) + 0.72 * abs(y_L) + 0.14  (tiny COLREGs starboard tie-break ONLY at dead-center)
            # Whenever left bypass is cheaper (`C_L < C_R`, e.g. hull sits on starboard of route `nb_cte > +0.08m`,
            # or ego is returning from a right-hand dodge past a starboard hull in Z-shape / Circle!),
            # the controller naturally & decisively glides on PORT (`neg_off = y_L`)!
            if nsp < 0.14:
                y_r = nb_cte + park_sep
                y_l = nb_cte - park_sep
                cost_r = max(0.0, y_r - raw_cte) + 0.72 * abs(y_r)
                cost_l = max(0.0, raw_cte - y_l) + 0.72 * abs(y_l) + 0.14
                delta_c = cost_l - cost_r  # >0 => starboard cheaper; <0 => port cheaper!

                if d < 1.90:
                    # Compact static cushion: protects bow approach (`bx > -0.22`) AND close corner proximity (`d < 1.20m`),
                    # while dropping to zero once hull is astern (`bx < -0.18`)!
                    cush_bow = _smoothstep((1.90 - d) / 0.85) * _smoothstep((bx + 0.22) / 0.36) * _smoothstep((0.82 - abs(by)) / 0.28)
                    cush_omni = _smoothstep((1.22 - d) / 0.45) * _smoothstep((bx + 0.30) / 0.30)
                    cush_b = max(cush_bow, cush_omni)
                    if cush_b > 1e-4:
                        max_urg = max(max_urg, cush_b)
                        if by > 0.18:
                            side_b = 1.0
                        elif by < -0.18:
                            side_b = -1.0
                        else:
                            side_b = 1.0 if delta_c >= 0.0 else -1.0
                        if side_b > 0.0:
                            repel_bias += 0.38 * cush_b
                            if bx > -0.05:
                                omega_max_guard = min(omega_max_guard, 0.04 - 0.32 * cush_b)
                        else:
                            stat_port_repel -= 0.38 * cush_b
                            if bx > -0.05:
                                stat_omega_min_guard = max(stat_omega_min_guard, -0.04 + 0.32 * cush_b)
                        if bx > 0.04 and abs(by) < 0.58:
                            slow_frac = max(slow_frac, 0.28 * cush_b)

                if along_r < -0.65:
                    continue

                tim = (
                    _smoothstep((park_engage - along_r) / (0.44 * park_engage))
                    if along_r >= -0.10
                    else _smoothstep(1.0 + (along_r + 0.10) / 0.55)
                )
                sw = _smoothstep((delta_c + 0.08) / 0.16)       # 1.0 when cost_r cheaper by >=0.08m, 0.0 when cost_l cheaper by >=0.08m
                pw = 1.0 - _smoothstep((delta_c + 0.08) / 0.16)
                lead_bell = _smoothstep((along_r - 0.15) / 1.15) * _smoothstep((park_engage - 0.2 - along_r) / 1.8)

                if sw > 1e-4:
                    us = sw * _smoothstep((park_sep + 0.26 - lat_r) / 0.44) * tim
                    if us > 1e-4:
                        max_urg = max(max_urg, us)
                        pos_off = max(pos_off, max(0.0, y_r) * us)
                        head_nudge += min(0.30, 0.78 * math.atan2(max(0.0, park_sep - lat_r), max(along_r * 0.80, 1.00))) * us * lead_bell
                        slow_frac = max(slow_frac, 0.08 * us * _smoothstep((2.2 - max(along_r, 0.0)) / 1.3))

                if pw > 1e-4:
                    clear_p = -lat_r  # positive when hull is already on our starboard
                    up = pw * _smoothstep((park_sep + 0.26 - clear_p) / 0.44) * tim
                    if up > 1e-4:
                        max_urg = max(max_urg, up)
                        neg_off = min(neg_off, min(0.0, y_l) * up)
                        stat_port_nudge -= min(0.30, 0.78 * math.atan2(max(0.0, park_sep - clear_p), max(along_r * 0.80, 1.00))) * up * lead_bell
                        slow_frac = max(slow_frac, 0.08 * up * _smoothstep((2.2 - max(along_r, 0.0)) / 1.3))
                continue

            if d > engage_dist:
                continue

            # --- 2. Moving 0.60x0.30m monohull: Green-to-Green Preserving Smooth Shoulder-Brush ---
            cb = (nvx * cos_yaw + nvy * sin_yaw) / nsp
            cr_ = (nvx * cos_r + nvy * sin_r) / nsp
            lvb = (-nvx * sin_yaw + nvy * cos_yaw) / nsp
            lvr = (-nvx * sin_r + nvy * cos_r) / nsp

            same_w = _smoothstep((min(cb, cr_) - 0.78) / 0.14)
            if same_w > 0.0 and d < 1.35:
                conv = same_w * _smoothstep((1.35 - d) / 0.60)
                if bx > 0.05:
                    slow_frac = max(slow_frac, 0.24 * conv)
                if d < 0.96 and bx > -0.25:
                    if by >= -0.06:
                        pos_off = max(pos_off, max(0.0, nb_cte + 0.82) * conv)
                        repel_bias += 0.15 * conv
                    else:
                        neg_off = min(neg_off, min(0.0, nb_cte - 0.82) * conv)
                        repel_bias -= 0.15 * conv
                max_urg = max(max_urg, 0.45 * conv)

            rvxn = nvx - v_nom * (0.5 * cos_yaw + 0.5 * cos_r)
            rvyn = nvy - v_nom * (0.5 * sin_yaw + 0.5 * sin_r)
            vsqr = rvx * rvx + rvy * rvy
            vsqn = rvxn * rvxn + rvyn * rvyn
            tcr = -(rx * rvx + ry * rvy) / vsqr if vsqr > 1e-4 else -1.0
            tcn = -(rx * rvxn + ry * rvyn) / vsqn if vsqn > 1e-4 else -1.0
            dcr = math.hypot(rx + rvx * max(tcr, 0.0), ry + rvy * max(tcr, 0.0))
            dcn = math.hypot(rx + rvxn * max(tcn, 0.0), ry + rvyn * max(tcn, 0.0))
            tcpa = tcn if (tcr <= 0.0 and tcn > 0.0) else (min(tcr, tcn) if (tcr > 0.0 and tcn > 0.0) else tcr)
            dcpa = min(dcr, dcn)
            es = 1.0 if d < 3.6 else corner_sup

            # --- 2A. Head-on: Compact Half-Width Shoulder-Brush (`0.66m` per ship -> `0.78..1.05m` pair separation!) ---
            how = _smoothstep((-cr_ - 0.50) / 0.22) * (1.0 - same_w)
            if how > 1e-4 and along_r > -0.85:
                along_env = _smoothstep((6.2 - along_r) / 1.85) * _smoothstep((along_r + 0.85) / 0.95)
                ho_bell = _smoothstep((along_r - 0.35) / 1.25) * _smoothstep((6.0 - along_r) / 1.65)
                wide_g2g = _smoothstep((nb_cte - 1.38) / 0.35) * _smoothstep((-0.82 - lat_r) / 0.28)
                lat_ho_gate = _smoothstep((2.55 - lat_r) / 0.60) * _smoothstep((lat_r + 1.22) / 0.28) * (1.0 - wide_g2g)
                uho_s = how * along_env * lat_ho_gate * es
                if uho_s > 1e-4:
                    max_urg = max(max_urg, uho_s)
                    mov_urg = max(mov_urg, uho_s)
                    lane_ho = min(1.95, max(0.68, (nb_cte + 0.94) if nb_cte > -0.18 else 0.68))
                    pos_off = max(pos_off, lane_ho * uho_s)
                    mov_pos_off = max(mov_pos_off, lane_ho * uho_s)
                    need_lat = float(np.clip((ho_sep + 0.20 - lat_r) / (ho_sep + 0.20), 0.0, 1.0))
                    head_nudge += 0.36 * uho_s * need_lat * ho_bell
                    slow_frac = max(slow_frac, 0.15 * uho_s * need_lat)

            # --- 2B. Starboard crossing (Give-Way): compact stern-brush (<1.02m DCPA, <4.0m range) ---
            csw = max(_smoothstep((lvb - 0.12) / 0.28), _smoothstep((lvr - 0.12) / 0.28)) * (1.0 - same_w) * (1.0 - 0.85 * how)
            if csw > 1e-4 and along_r > -0.45 and lat_r < 0.85:
                ucr_slow = (
                    csw
                    * _smoothstep((1.06 - dcpa) / 0.42)
                    * (_smoothstep((7.2 - tcpa) / 2.8) if tcpa >= 0.0 else _smoothstep(1.0 + tcpa / 0.8))
                    * _smoothstep((4.5 - d) / 1.4)
                    * _smoothstep((0.85 - lat_r) / 0.40)
                    * es
                )
                ucr_steer = (
                    csw
                    * _smoothstep((1.02 - dcpa) / 0.40)
                    * (_smoothstep((5.8 - tcpa) / 2.3) if tcpa >= 0.0 else _smoothstep(1.0 + tcpa / 0.75))
                    * _smoothstep((3.9 - d) / 1.3)
                    * _smoothstep((0.85 - lat_r) / 0.40)
                    * es
                )
                if ucr_slow > 1e-4:
                    max_urg = max(max_urg, ucr_slow)
                    slow_frac = max(slow_frac, 0.44 * ucr_slow * _smoothstep((4.2 - d) / 2.0))
                if ucr_steer > 1e-4:
                    stern_cte = max(0.46, raw_cte + 0.28 * _smoothstep((2.8 - d) / 1.3))
                    pos_off = max(pos_off, min(1.45, stern_cte) * ucr_steer)
                    mov_pos_off = max(mov_pos_off, min(1.45, stern_cte) * ucr_steer)
                    mov_urg = max(mov_urg, ucr_steer)
                    head_nudge += 0.24 * ucr_steer * _smoothstep((along_r + 0.10) / 1.05)

            # --- 2C. Port-side crosser (Stand-On): keep full speed to clear bow fast + compact cushion ---
            cpw = max(_smoothstep((-lvb - 0.16) / 0.28), _smoothstep((-lvr - 0.16) / 0.28)) * (1.0 - same_w) * (1.0 - how)
            if cpw > 1e-4 and d < 2.2 and bx > -0.22:
                udef = cpw * _smoothstep((0.92 - dcpa) / 0.38) * _smoothstep((2.2 - d) / 1.00)
                if udef > 1e-4:
                    max_urg = max(max_urg, udef)
                    if lat_r >= -0.02:
                        pos_off = max(pos_off, min(1.25, max(0.28, nb_cte + 0.74)) * udef)
                    elif lat_r <= -0.14:
                        neg_off = min(neg_off, max(-1.35, min(0.0, nb_cte - 0.74)) * udef)

            # --- 2D. Compact bilateral monohull envelope cushion (<1.28m) ---
            if d < 1.28 and bx > -0.30:
                ucush = _smoothstep((1.28 - d) / 0.56)
                max_urg = max(max_urg, ucush)
                if by >= -0.12 or csw > 0.25 or (how > 0.25 and lat_r > -0.68):
                    pos_off = max(pos_off, min(1.80, max(0.0, nb_cte + 0.82)) * ucush)
                    repel_bias += 0.36 * ucush
                else:
                    neg_off = min(neg_off, max(-1.75, min(0.0, nb_cte - 0.82)) * ucush)
                    repel_bias -= 0.34 * ucush
                if bx > 0.08 and d < 1.02:
                    slow_frac = max(slow_frac, 0.36 * _smoothstep((1.02 - d) / 0.45))

        self._nearest_distance = nearest
        self.last_engaged = max_urg > 1e-3 or abs(repel_bias) > 1e-3

        # --- 3. Constant-Curvature True-Circle Cornering + Overdamped Laser-Straight Cruising (a32) ---
        # Solves ALL THREE remaining visual issues simultaneously:
        # 1) NO mid-turn outward surge & NO flattened exit arc:
        #    - Hold corner throttle low ALL THE WAY down to `|he_route| <= 36°` (`0.63 rad`), ramping back to cruise
        #      only across `36° -> 14°` (`0.24..0.63 rad`)! At `he = 51°`, `turn_slow_open = 1.00` (was `0.50` -> `sc=0.80`!).
        #    - Keep corner speed steady at `scale ~ 0.44` (`v ~ 0.18 m/s`) paired with gentle `0.38 rad/s` turn rate
        #      so BOTH the first half and second half of the turn trace a uniform `R ~ 0.48..0.58m` true circular arc!
        # 2) Overdamped laser-straight line tracking (`L_straight = 1.95..2.35m`, `gain_straight = 0.64`):
        #    - Increases straight-line damping ratio `zeta` from `0.55` -> `1.12` under 2D sway + yaw lag, completely
        #      eliminating S-wave oscillation around the planned line!
        omega_cap = self.max_angular_velocity if self.turn_omega_limit <= 0.0 else min(self.max_angular_velocity, self.turn_omega_limit)
        mov_dom = _smoothstep((mov_urg - 0.14) / 0.24)
        head_nudge += stat_port_nudge * (1.0 - mov_dom)
        repel_bias += stat_port_repel * (1.0 - mov_dom)
        if mov_dom < 0.25:
            omega_min_guard = max(omega_min_guard, stat_omega_min_guard)
        if pos_off > 0.0 and neg_off < 0.0:
            eff_neg = neg_off * (1.0 - mov_dom)
            net_off = pos_off + eff_neg if mov_dom > 0.05 else (pos_off if pos_off >= abs(neg_off) else neg_off)
        else:
            net_off = pos_off + neg_off * (1.0 - mov_dom)

        he_abs = abs(he_route)
        avoid_blend = _smoothstep((max_urg - 0.06) / 0.28)
        turn_gate_open = _smoothstep((he_abs - 0.22) / 0.28)   # 0 below 12.6° (pure straight damping!), 1 above 28.6° (36° circle + corners)
        turn_gate_sharp = _smoothstep((he_abs - 0.92) / 0.42)
        turn_gate = (1.0 - avoid_blend) * turn_gate_open + avoid_blend * turn_gate_sharp

        # Delayed Exit Throttle (`0.25..0.68 rad` = `14°..39°`! Still 100% full brake at `he >= 39°`,
        # while a 30-36° polygon circle only touches `0.15..0.28 * 0.56` = ~88-92% cruise):
        turn_slow_open = _smoothstep((he_abs - 0.25) / 0.43)
        turn_slow_avoid = _smoothstep((he_abs - 0.78) / 0.85)
        arc_slow = 0.56 * ((1.0 - avoid_blend) * turn_slow_open + avoid_blend * turn_slow_avoid)

        # Gentle constant-curvature open-water turn cap `0.39 rad/s` (`R = 0.20 / 0.39 = 0.51m` uniform round circle!),
        # expanding smoothly to `0.58 rad/s` only when close-quarters avoidance demands crisp agility:
        smooth_turn_cap = min(omega_cap, 0.39 + 0.19 * _smoothstep((max_urg - 0.16) / 0.44))

        v_eff = 0.50 * own_v + 0.50 * (1.0 - arc_slow) * self.cruise_speed
        r_arc = v_eff / max(0.88 * smooth_turn_cap, 0.32)
        turn_sgn = 1.0 if he_route >= 0.0 else -1.0

        # Suppress inside-of-chord pull during the entire body of the arc (`|base_cte_err| > 0.12m`) so the second half
        # of the turn NEVER flattens out early, landing tangentially right onto `raw_cte = 0`!
        base_cte_err = raw_cte - net_off
        inside_suppress = 1.0 - 0.92 * turn_gate * _smoothstep((-turn_sgn * base_cte_err - 0.04) / 0.22)
        eff_cte_err = base_cte_err * inside_suppress + turn_gate * (
            0.16 * own_v * math.sin(he_route)
            + 0.14 * turn_sgn * r_arc * (1.0 - math.cos(min(he_abs, 1.35)))
        )

        # Overdamped straight-line lookahead `1.95..2.30m` when cruising freely, tightening to `1.45..1.85m`
        # during active avoidance so head-on / Z-shape lane shifts build crisply without sluggish lag!
        L_straight = (1.0 - avoid_blend) * min(2.25, max(1.88, 0.66 * d2g)) + avoid_blend * min(1.85, max(1.38, 0.58 * d2g))
        L_corner = min(1.55, max(1.12, 0.56 * d2g))
        L_lh = (1.0 - turn_gate) * L_straight + turn_gate * L_corner

        cte_err = float(np.clip(eff_cte_err, -2.4, 2.4))
        lb = (1.0 - 0.62 * _smoothstep((he_abs - 1.40) / 0.72)) * (1.0 - _smoothstep((he_abs - 2.30) / 0.45))
        los_corr = math.atan2(cte_err, L_lh)
        he_tr = _wrap(he_route + lb * los_corr)

        total_bias = float(np.clip(head_nudge + repel_bias, -0.54, 0.54))
        gain_straight = 0.64 * (1.0 - avoid_blend) + 0.78 * avoid_blend
        gain = gain_straight * (1.0 - turn_gate) + 0.88 * turn_gate
        omega = float(smooth_turn_cap * math.tanh((gain * (he_tr - total_bias)) / max(smooth_turn_cap, 0.19)))
        if omega_min_guard <= omega_max_guard:
            omega = float(np.clip(omega, max(-omega_cap, omega_min_guard), min(omega_cap, omega_max_guard)))

        total_slow_frac = float(np.clip(max(slow_frac, arc_slow), 0.0, 0.60))

        saved_gov = self.turn_speed_governor
        self.turn_speed_governor = False
        try:
            return self._format_action(omega, self.cruise_speed * total_slow_frac)
        finally:
            self.turn_speed_governor = saved_gov

    def act(self, obs: AgentLocalObservation) -> np.ndarray:
        """Produce the teacher action for a single agent's observation."""
        self._last_heading_error = float(obs.heading_error)
        if self.close_brush:
            return self._act_close_brush(obs)
        if self.sleek_direct:
            return self._act_sleek_direct(obs)
        if self.veteran_flow:
            return self._act_veteran_flow(obs)
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
            self._duck_dir, self._convoy_slow = 1.0, 0.0
            urgency = self._duck_urgency(obs)
            nearest, closing = self._nearest_closing(obs)
            self._nearest_distance = nearest
            if closing and nearest < self.escape_distance:
                urgency = 1.0
            self.last_engaged = urgency > 0.0 or self._convoy_slow > 0.0
            clearance_mode = self.avoid_clearance > 0.0 and self._duck_has_cpa
            need = 1.0
            if clearance_mode:
                need = float(np.clip(1.0 - self._duck_dir * self._duck_left / self.avoid_clearance, 0.0, 1.0))
            offset = self.duck_offset * urgency * self._duck_dir * need
            cte_term = self.cte_p_gain * (float(obs.cross_track_error) - offset)
            if self.cte_term_limit is not None:
                cte_term = float(np.clip(cte_term, -self.cte_term_limit, self.cte_term_limit))
            he = float(obs.heading_error)
            if self.avoid_heading > 0.0 and urgency > 0.0:
                # The bias fades out as the pass-side offset is reached, so the boat settles on the offset instead of drifting on.
                if clearance_mode:
                    progress = 1.0 - need
                else:
                    progress = float(obs.cross_track_error) * self._duck_dir / max(self.duck_offset, 1e-3)
                he -= self.avoid_heading * urgency * float(np.clip(1.0 - progress, 0.0, 1.0)) * self._duck_dir
            if self.heading_sqrt_gain > 0.0:
                # Sqrt law keeps small heading errors above the ~0.15 rad/s yaw deadband instead of stalling.
                heading_term = self.heading_sqrt_gain * math.copysign(math.sqrt(abs(he)), he)
            else:
                heading_term = self.heading_p_gain * he
            omega_cap = max_omega if self.turn_omega_limit <= 0.0 else min(max_omega, self.turn_omega_limit)
            omega = float(np.clip(heading_term + cte_term, -omega_cap, omega_cap))
            slow = self.duck_slow * urgency + 0.4 * self._convoy_slow + self.park_slow * self._park_near * urgency
            return self._format_action(omega, self.cruise_speed * slow)
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

    def _duck_urgency_smooth(self, obs: AgentLocalObservation) -> float:
        """Continuous duck urgency: give-way side from the other hull's course (stable through the encounter), smooth ramp in before CPA and out after it."""
        cos_yaw, sin_yaw = math.cos(float(obs.yaw)), math.sin(float(obs.yaw))
        own_v = max(float(obs.speed), 0.0)
        urgency = 0.0
        self._duck_dir, self._convoy_slow, self._park_near = 1.0, 0.0, 0.0
        self._duck_left, self._duck_has_cpa = 0.0, False
        phi = float(obs.yaw) + float(obs.heading_error)  # route direction ~ bearing to the goal
        cos_phi, sin_phi = math.cos(phi), math.sin(phi)
        for nb in obs.neighbors:
            d = float(nb.distance)
            if d <= 1e-6 or d > self.giveway_distance:
                continue
            rx, ry, rvx, rvy = float(nb.rel_x), float(nb.rel_y), float(nb.rel_vx), float(nb.rel_vy)
            lateral_pos = -sin_yaw * rx + cos_yaw * ry  # >0: neighbour on our port side
            nvx, nvy = rvx + own_v * cos_yaw, rvy + own_v * sin_yaw
            n_speed = math.hypot(nvx, nvy)
            same_w = 0.0
            if self.tandem_clean and n_speed >= 0.08:
                # Soft same-course weight: give-way fades out and convoy fades in together, so noisy velocity cannot flip the regime.
                same_w = _smoothstep(((nvx * cos_yaw + nvy * sin_yaw) / n_speed - 0.2) / 0.5)
                # Boats holding station on each other (small relative speed) are in convoy whatever their noisy course estimate says.
                same_w = max(same_w, _smoothstep((0.25 - math.hypot(rvx, rvy)) / 0.15))
                if same_w > 0.0 and self.convoy_distance > 0.0 and d < self.convoy_distance:
                    conv = same_w * _smoothstep((self.convoy_distance - d) / (0.45 * self.convoy_distance))
                    along_pos = cos_yaw * rx + sin_yaw * ry
                    ahead_w = _smoothstep(along_pos / 0.6)
                    self._convoy_slow = max(self._convoy_slow, conv * ahead_w)
                    push = conv * (1.0 - ahead_w) * _smoothstep((along_pos + 0.6) / 0.3)
                    if push > urgency:
                        urgency = push
                        self._duck_dir = 1.0 if lateral_pos >= 0.0 else -1.0
                        self._duck_has_cpa = False
                        self._park_near = 0.0
            if not self.tandem_clean and self.convoy_distance > 0.0 and d < self.convoy_distance and n_speed >= 0.08:
                same_course = (nvx * cos_yaw + nvy * sin_yaw) / n_speed
                conv = _smoothstep((same_course - 0.5) / 0.3) * _smoothstep((self.convoy_distance - d) / (0.45 * self.convoy_distance))
                if conv > urgency:
                    urgency = conv
                    self._duck_dir = 1.0 if lateral_pos > 0.0 else -1.0
                    ahead = cos_yaw * rx + sin_yaw * ry >= -0.2
                    self._convoy_slow = conv if ahead else 0.0
            if self.static_hull and n_speed < 0.08:
                along_r = cos_phi * rx + sin_phi * ry
                lat_r = -sin_phi * rx + cos_phi * ry  # >0: hull left of the route
                miss = _smoothstep((self.giveway_dcpa - abs(lat_r)) / (0.4 * self.giveway_dcpa))
                if along_r >= 0.0:
                    timing = _smoothstep((self.giveway_distance - along_r) / (0.4 * self.giveway_distance))
                else:
                    timing = _smoothstep(1.0 + along_r / 2.4)
                candidate = miss * timing
                if candidate > urgency:
                    urgency = candidate
                    self._convoy_slow = 0.0
                    self._duck_dir = 1.0 if lat_r > -0.6 else -1.0
                    self._duck_left = lat_r
                    self._duck_has_cpa = True
                    self._park_near = _smoothstep((3.2 - along_r) / 2.0) if along_r >= 0.0 else 0.0
                continue
            v_sq = rvx * rvx + rvy * rvy
            if v_sq < 0.12 ** 2:
                continue
            parked = n_speed < 0.08
            if parked:
                side = 1.0
            else:
                along = (nvx * cos_yaw + nvy * sin_yaw) / n_speed
                lateral = (-nvx * sin_yaw + nvy * cos_yaw) / n_speed
                along_r = (nvx * cos_phi + nvy * sin_phi) / n_speed
                lateral_r = (-nvx * sin_phi + nvy * cos_phi) / n_speed
                # Head-on, or crossing right-to-left (other hull on our starboard): we give way.
                # Evaluate in both body and route frames so turning to avoid never extinguishes urgency mid-pass.
                side = max(
                    _smoothstep((-along - 0.55) / 0.30),
                    _smoothstep((-along_r - 0.55) / 0.30),
                    _smoothstep((lateral - 0.1) / 0.4),
                    _smoothstep((lateral_r - 0.1) / 0.4),
                )
            if side <= 0.0:
                continue
            tcpa = -(rx * rvx + ry * rvy) / v_sq
            dcpa = math.hypot(rx + rvx * tcpa, ry + rvy * tcpa)
            miss = _smoothstep((self.giveway_dcpa - dcpa) / (0.4 * self.giveway_dcpa))
            if tcpa >= 0.0:
                timing = _smoothstep((self.giveway_tcpa - tcpa) / (0.6 * self.giveway_tcpa))
            else:
                timing = _smoothstep(1.0 + tcpa / self.duck_release)
            candidate = side * miss * timing * (1.0 - same_w)
            if candidate > urgency:
                urgency = candidate
                self._convoy_slow = 0.0
                self._park_near = 0.0
                # A hull that cannot move (HOLD / waiting) must be passed on its clear side; moving give-way stays starboard.
                self._duck_dir = (1.0 if lateral_pos > -0.6 else -1.0) if parked else 1.0
                t_cpa = max(tcpa, 0.0)
                self._duck_left = -sin_phi * (rx + rvx * t_cpa) + cos_phi * (ry + rvy * t_cpa)
                self._duck_has_cpa = True
        return urgency

    def _duck_urgency(self, obs: AgentLocalObservation) -> float:
        """0..1 ramp over the last giveway_tcpa seconds before CPA with a hull this boat must give way to."""
        if self.duck_smooth:
            return self._duck_urgency_smooth(obs)
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
            parked = math.hypot(nvx, nvy) < 0.08  # a parked hull can never give way, whatever side it is on
            if not -112.5 <= bearing <= (90.0 if parked else (40.0 if head_on else 10.0)):
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
