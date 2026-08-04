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

import numpy as np

from .collect_scripted_dataset import _scripted_policy_action_cluster_colregs_v2
from .multi_agent_types import AgentLocalObservation


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
    ) -> None:
        if action_mode not in ('speed_scale', 'full', 'angular_only'):
            raise ValueError(f'unsupported action_mode={action_mode!r}')
        self.action_mode = action_mode
        self.cruise_speed = float(cruise_speed)
        self.max_angular_velocity = float(max_angular_velocity)
        self.action_speed_scale_min = float(np.clip(action_speed_scale_min, 0.0, 1.0))
        # For 'full' mode the absolute forward speed ceiling.
        self.linear_speed_limit = float(linear_speed_limit if linear_speed_limit is not None else cruise_speed)

    def _avoidance_intent(self, obs: AgentLocalObservation) -> tuple[float, float]:
        """Return (slowdown >= 0, starboard_rate >= 0) from the v2 heuristic."""
        vec = _build_teacher_layout_vector(obs)
        raw = _scripted_policy_action_cluster_colregs_v2(vec)
        slowdown = max(0.0, -float(raw[0]))
        starboard_rate = max(0.0, -float(raw[1]))
        return slowdown, starboard_rate

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
        slowdown, starboard_rate = self._avoidance_intent(obs)
        max_omega = self.max_angular_velocity

        # Close-range safety augmentation (fixes overtaking/random marginal
        # collisions at min_sep ~0.73 < 0.75). The base v2 heuristic turns too
        # mildly for a catching-up overtaker; here we (a) ramp up the starboard
        # turn early (from ~2.4m) so separation opens BEFORE close quarters, and
        # (b) brake only at very close range (<1.3m) as a last resort.
        nearest, closing = self._nearest_closing(obs)
        if closing and nearest < 2.4:
            prox = float(np.clip((2.4 - nearest) / (2.4 - 0.85), 0.0, 1.0))
            starboard_rate = max(starboard_rate, 0.18 * prox + 0.34 * prox * prox)
            if nearest < 1.3:
                brake = float(np.clip((1.3 - nearest) / (1.3 - 0.75), 0.0, 1.0))
                slowdown = max(slowdown, 0.22 * brake)

        # BLEND goal-seeking with the starboard avoidance bias (do NOT override).
        # The closed-loop teacher eval showed a pure override makes the craft turn
        # away from every neighbour within lookahead and never reach the goal
        # (min_sep ~4.8m, goal~0). Subtracting the starboard rate from the nav
        # heading command keeps the craft heading to goal while biasing right for
        # COLREGS give-way, and lets it resume the route once the conflict clears.
        goal_omega = float(obs.raw_angular_z)
        omega = float(np.clip(goal_omega - starboard_rate, -max_omega, max_omega))

        # Throttle: full nav speed unless yielding; never stop / reverse.
        scale = float(np.clip(1.0 - slowdown / max(self.cruise_speed, 1e-3),
                              self.action_speed_scale_min, 1.0))

        if self.action_mode == 'angular_only':
            return np.asarray([omega], dtype=np.float32)
        if self.action_mode == 'speed_scale':
            return np.asarray([scale, omega], dtype=np.float32)
        # 'full': emit an absolute forward speed = scale * cruise.
        linear_x = float(np.clip(scale * self.cruise_speed, 0.0, self.linear_speed_limit))
        return np.asarray([linear_x, omega], dtype=np.float32)
