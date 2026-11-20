import argparse
from collections import deque
from dataclasses import replace
import json
import math
import os
import re
from datetime import datetime
from pathlib import Path
import time
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from common_interfaces.msg import FleetNeighborPoses, NavigationFeedback, NavigationGoal, NavigationResult
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int8, String

from .action_projection import project_policy_action as project_rl_policy_action
from .config import ActionBounds
from .multi_agent_types import ENCOUNTER_TYPE_COUNT, ROLE_GIVE_WAY, classify_encounter_role
from .observation_normalizer import ObservationNormalizer
from .policies import load_policy
from .policy_trace import PolicyTrace
from .route_frame import LEGACY_FRAME, OBSERVATION_FRAMES, ROUTE_CANONICAL_FRAME, canonical_route_fields, turn_governor_speed_cap
from .runtime_action_contract import RuntimeActionContract
from .types import NeighborObservation, NeighborState, UsvObservation, USV_NEIGHBOR_FEATURE_COUNT

_ENCOUNTER_TYPE_NAMES = {'head_on': 0, 'crossing': 1, 'overtaking': 2}

# ---------- Auto encounter-type classification constants ----------
# Detection range matches the training reward conflict_distance (5.0 m).  The
# previous 15 m radius kept the encounter one-hot active during nearly the
# entire 5-USV transit, biasing the policy into permanent avoidance mode.
_ENCOUNTER_DETECT_DISTANCE = 5.0    # metres – classify only when nearest neighbour is closer
_ENCOUNTER_HOLD_TIME = 2.0          # seconds – minimum time to hold a classification before switching
_ENCOUNTER_SPEED_THRESHOLD = 0.11   # m/s – below this (Gazebo wave bobbing <=0.09 m/s), vessel is holding/static
_ENCOUNTER_CLOSING_MIN = 0.01      # m/s – require actual closing before flagging an encounter

# ---------- Distance-aware speed scaling constants ----------
_SPEED_SCALE_DISTANCE = 3.0         # metres – linear speed scaling starts below this distance
_SPEED_SCALE_MIN = 0.35             # minimum speed scale factor at zero distance
_HEAD_ON_BEARING_DEG = 22.5         # |bearing| < this AND reciprocal courses → head_on
_HEAD_ON_COURSE_DEG = 135.0         # relative course > this → reciprocal
_OVERTAKING_COURSE_DEG = 45.0       # relative course < this AND ahead → overtaking
_OVERTAKING_ASTERN_DEG = 112.5      # |bearing| > this → astern sector

# ---------- Decisive give-way transit guard ----------
_GIVE_WAY_GUARD_DISTANCE = 5.0       # metres – align with the trained conflict distance
_GIVE_WAY_STATIC_TURN_FLOOR = 0.16   # rad/s – decisive turn around a holding vessel
_GIVE_WAY_MOVING_DCPA_TARGET = 0.80  # metres – moving pairs above this predicted CPA need no guard
_GIVE_WAY_MOVING_TURN_BASE = 0.03    # rad/s – subtle early correction for a moving conflict
_GIVE_WAY_MOVING_TURN_RISK = 0.06    # rad/s – additional turn for predicted CPA deficit
_GIVE_WAY_MOVING_TURN_NEAR = 0.03    # rad/s – additional turn as the pair closes
_GIVE_WAY_MOVING_STEERAGE_SPEED = 0.12  # m/s – keep crossing motion without an abrupt push
_GIVE_WAY_STEERAGE_SPEED = 0.15      # m/s – avoid a near-stationary close-pass plateau
_GIVE_WAY_STEERAGE_MIN_DISTANCE = 0.9  # metres – never force forward speed in emergency proximity

# ---------- Virtual sub-goal (long-transit OOD mitigation) constants ----------
# The fresh520 policy was trained in a ±8 m arena with distance-to-goal ≤ ~16 m.
# Real missions can issue goals 30-45 m away, pushing pose_x/pose_y and
# distance_to_goal far outside the normalizer statistics and degrading the
# policy into meandering.  When a route leg is long (or starts/ends outside the
# training arena), the node walks a virtual sub-goal along the route line and
# expresses the ego pose in a leg-local frame so every observation stays
# in-distribution.  Short legs keep the exact legacy behaviour.
_VIRTUAL_LEG_LENGTH = 8.0          # metres – virtual sub-goal lookahead along the route
_VIRTUAL_GOAL_TRIGGER = 12.0       # metres – enable virtual mode when route is longer
_POSE_IN_DIST_LIMIT = 9.0          # metres – training arena half-extent for pose features


def _give_way_guard_demand(observation, neighbor) -> Tuple[float, bool]:
    """Return starboard turn floor and static-blocker steerage demand."""
    if int(getattr(neighbor, 'role_index', -1)) != ROLE_GIVE_WAY:
        return 0.0, False
    if not (0.0 < neighbor.distance < _GIVE_WAY_GUARD_DISTANCE):
        return 0.0, False
    if abs(float(neighbor.bearing)) >= math.radians(_OVERTAKING_ASTERN_DEG):
        return 0.0, False

    range_rate = -(
        neighbor.rel_x * neighbor.rel_vx + neighbor.rel_y * neighbor.rel_vy
    ) / max(neighbor.distance, 1e-3)
    if range_rate <= _ENCOUNTER_CLOSING_MIN:
        return 0.0, False

    own_vx = observation.speed * math.cos(observation.yaw)
    own_vy = observation.speed * math.sin(observation.yaw)
    neighbor_speed = math.hypot(own_vx + neighbor.rel_vx, own_vy + neighbor.rel_vy)
    if neighbor_speed <= _ENCOUNTER_SPEED_THRESHOLD:
        keep_steerage = neighbor.distance > _GIVE_WAY_STEERAGE_MIN_DISTANCE
        return _GIVE_WAY_STATIC_TURN_FLOOR, keep_steerage

    relative_speed_sq = neighbor.rel_vx ** 2 + neighbor.rel_vy ** 2
    if relative_speed_sq <= 1e-6:
        return 0.0, False
    tcpa = -(
        neighbor.rel_x * neighbor.rel_vx + neighbor.rel_y * neighbor.rel_vy
    ) / relative_speed_sq
    if tcpa <= 0.0:
        return 0.0, False
    dcpa = math.hypot(
        neighbor.rel_x + neighbor.rel_vx * tcpa,
        neighbor.rel_y + neighbor.rel_vy * tcpa,
    )
    if dcpa >= _GIVE_WAY_MOVING_DCPA_TARGET:
        return 0.0, False

    dcpa_risk = min(1.0, (_GIVE_WAY_MOVING_DCPA_TARGET - dcpa) / _GIVE_WAY_MOVING_DCPA_TARGET)
    proximity = min(1.0, max(
        0.0,
        (_GIVE_WAY_GUARD_DISTANCE - neighbor.distance)
        / max(_GIVE_WAY_GUARD_DISTANCE - _GIVE_WAY_STEERAGE_MIN_DISTANCE, 1e-3),
    ))
    turn_floor = (
        _GIVE_WAY_MOVING_TURN_BASE
        + _GIVE_WAY_MOVING_TURN_RISK * dcpa_risk
        + _GIVE_WAY_MOVING_TURN_NEAR * proximity
    )
    return turn_floor, False


def _candidate_workspace_roots() -> list[Path]:
    roots: list[Path] = []
    module_path = Path(__file__).resolve()
    for parent in module_path.parents:
        if parent.name in ('install', 'src'):
            roots.append(parent.parent)

    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    for raw_prefix in ament_prefix_path.split(os.pathsep):
        if not raw_prefix:
            continue
        prefix = Path(raw_prefix).resolve()
        if prefix.name == 'install':
            roots.append(prefix.parent)
        elif prefix.parent.name == 'install':
            roots.append(prefix.parent.parent)

    unique_roots: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        if root not in seen:
            unique_roots.append(root)
            seen.add(root)
    return unique_roots


def _resolve_model_path(model_path: str) -> str:
    raw_path = Path(os.path.expandvars(os.path.expanduser(model_path)))
    candidates: list[Path] = []

    if raw_path.is_absolute():
        candidates.append(raw_path)
    else:
        candidates.append((Path.cwd() / raw_path).resolve())
        for workspace_root in _candidate_workspace_roots():
            candidates.append((workspace_root / raw_path).resolve())
            candidates.append((workspace_root / 'src' / raw_path).resolve())

    checked: list[str] = []
    for candidate in candidates:
        candidate_str = str(candidate)
        if candidate_str in checked:
            continue
        checked.append(candidate_str)
        if candidate.exists():
            return candidate_str

    checked_str = ', '.join(checked) if checked else str(raw_path)
    raise FileNotFoundError(
        f'Model file not found for input {model_path}. Checked: {checked_str}'
    )


def _quat_to_yaw(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _migrate_runtime_normalizer_state(state: dict, new_obs_dim: int) -> dict:
    mean = np.asarray(state.get('mean', ()), dtype=np.float64)
    var = np.asarray(state.get('var', ()), dtype=np.float64)
    if mean.ndim != 1 or var.ndim != 1 or mean.shape != var.shape:
        return state
    old_obs_dim = int(mean.shape[0])
    new_obs_dim = int(new_obs_dim)
    if old_obs_dim == new_obs_dim or old_obs_dim <= 0 or new_obs_dim <= old_obs_dim:
        return state

    layout_stats = _migrate_runtime_obs_stats_by_layout(mean, var, new_obs_dim)
    if layout_stats is not None:
        migrated = dict(state)
        migrated['mean'], migrated['var'] = layout_stats
        return migrated

    ego_dim = UsvObservation.ego_feature_size()
    old_neighbor_dim = 6
    old_neighbor_width = old_obs_dim - ego_dim - ENCOUNTER_TYPE_COUNT
    new_neighbor_width = new_obs_dim - ego_dim - ENCOUNTER_TYPE_COUNT
    if (
        old_neighbor_width >= 0
        and new_neighbor_width >= 0
        and old_neighbor_width % old_neighbor_dim == 0
    ):
        max_neighbors = old_neighbor_width // old_neighbor_dim
        if new_neighbor_width == max_neighbors * USV_NEIGHBOR_FEATURE_COUNT:
            new_mean_parts = [mean[:ego_dim]]
            new_var_parts = [var[:ego_dim]]
            old_pos = ego_dim
            for _ in range(max_neighbors):
                new_mean_parts.append(mean[old_pos:old_pos + old_neighbor_dim])
                new_var_parts.append(var[old_pos:old_pos + old_neighbor_dim])
                new_mean_parts.append(np.zeros(USV_NEIGHBOR_FEATURE_COUNT - old_neighbor_dim, dtype=np.float64))
                new_var_parts.append(np.ones(USV_NEIGHBOR_FEATURE_COUNT - old_neighbor_dim, dtype=np.float64))
                old_pos += old_neighbor_dim
            new_mean_parts.append(mean[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
            new_var_parts.append(var[old_pos:old_pos + ENCOUNTER_TYPE_COUNT])
            migrated = dict(state)
            migrated['mean'] = np.concatenate(new_mean_parts)
            migrated['var'] = np.concatenate(new_var_parts)
            return migrated

    delta = new_obs_dim - old_obs_dim
    migrated = dict(state)
    migrated['mean'] = np.concatenate([mean, np.zeros(delta, dtype=np.float64)])
    migrated['var'] = np.concatenate([var, np.ones(delta, dtype=np.float64)])
    return migrated


def _infer_runtime_obs_layout(obs_dim: int) -> tuple[int, int, int, int] | None:
    new_ego_dim = UsvObservation.ego_feature_size()
    for encounter_dim in (ENCOUNTER_TYPE_COUNT, 0):
        for ego_dim in (new_ego_dim, 17, 12, 11, 10):
            for neighbor_dim in (USV_NEIGHBOR_FEATURE_COUNT, 6):
                neighbor_width = int(obs_dim) - int(ego_dim) - int(encounter_dim)
                if neighbor_width >= 0 and neighbor_width % int(neighbor_dim) == 0:
                    return int(ego_dim), int(neighbor_dim), int(neighbor_width // int(neighbor_dim)), int(encounter_dim)
    return None


def _migrate_runtime_obs_stats_by_layout(mean: np.ndarray, var: np.ndarray, new_obs_dim: int) -> tuple[np.ndarray, np.ndarray] | None:
    old_layout = _infer_runtime_obs_layout(int(mean.shape[0]))
    new_layout = _infer_runtime_obs_layout(int(new_obs_dim))
    if old_layout is None or new_layout is None:
        return None
    old_ego_dim, old_neighbor_dim, neighbor_slots, old_encounter_dim = old_layout
    new_ego_dim, new_neighbor_dim, new_neighbor_slots, new_encounter_dim = new_layout
    if neighbor_slots != new_neighbor_slots or old_encounter_dim != new_encounter_dim:
        return None
    mean_parts = []
    var_parts = []
    ego_cols = min(old_ego_dim, new_ego_dim)
    mean_parts.append(mean[:ego_cols])
    var_parts.append(var[:ego_cols])
    if new_ego_dim > ego_cols:
        mean_parts.append(np.zeros(new_ego_dim - ego_cols, dtype=np.float64))
        var_parts.append(np.ones(new_ego_dim - ego_cols, dtype=np.float64))
    old_pos = old_ego_dim
    for _ in range(neighbor_slots):
        neighbor_cols = min(old_neighbor_dim, new_neighbor_dim)
        mean_parts.append(mean[old_pos:old_pos + neighbor_cols])
        var_parts.append(var[old_pos:old_pos + neighbor_cols])
        if new_neighbor_dim > neighbor_cols:
            mean_parts.append(np.zeros(new_neighbor_dim - neighbor_cols, dtype=np.float64))
            var_parts.append(np.ones(new_neighbor_dim - neighbor_cols, dtype=np.float64))
        old_pos += old_neighbor_dim
    if new_encounter_dim > 0:
        mean_parts.append(mean[old_pos:old_pos + new_encounter_dim])
        var_parts.append(var[old_pos:old_pos + new_encounter_dim])
    return np.concatenate(mean_parts), np.concatenate(var_parts)


def _adapt_runtime_observation_dim(observation: np.ndarray, target_dim: int) -> np.ndarray:
    obs = np.asarray(observation, dtype=np.float32)
    current_dim = int(obs.shape[-1]) if obs.ndim > 0 else 0
    target_dim = int(target_dim)
    if current_dim == target_dim:
        return obs
    current_layout = _infer_runtime_obs_layout(current_dim)
    target_layout = _infer_runtime_obs_layout(target_dim)
    if current_layout is not None and target_layout is not None:
        current_ego_dim, current_neighbor_dim, neighbor_slots, current_encounter_dim = current_layout
        target_ego_dim, target_neighbor_dim, target_neighbor_slots, target_encounter_dim = target_layout
        if neighbor_slots == target_neighbor_slots and current_encounter_dim == target_encounter_dim:
            parts = []
            ego_cols = min(current_ego_dim, target_ego_dim)
            parts.append(obs[..., :ego_cols])
            if target_ego_dim > ego_cols:
                pad_shape = obs.shape[:-1] + (target_ego_dim - ego_cols,)
                parts.append(np.zeros(pad_shape, dtype=np.float32))
            old_pos = current_ego_dim
            for _ in range(neighbor_slots):
                neighbor_cols = min(current_neighbor_dim, target_neighbor_dim)
                parts.append(obs[..., old_pos:old_pos + neighbor_cols])
                if target_neighbor_dim > neighbor_cols:
                    pad_shape = obs.shape[:-1] + (target_neighbor_dim - neighbor_cols,)
                    parts.append(np.zeros(pad_shape, dtype=np.float32))
                old_pos += current_neighbor_dim
            if target_encounter_dim > 0:
                parts.append(obs[..., old_pos:old_pos + target_encounter_dim])
            return np.concatenate(parts, axis=-1).astype(np.float32, copy=False)
    if target_dim > current_dim:
        pad_shape = obs.shape[:-1] + (target_dim - current_dim,)
        return np.concatenate([obs, np.zeros(pad_shape, dtype=np.float32)], axis=-1)
    return obs[..., :target_dim].astype(np.float32, copy=False)


class ZeroPolicy:
    def predict(self, observation: np.ndarray) -> np.ndarray:
        return np.zeros(2, dtype=np.float32)


class PpoPolicyRuntime:
    def __init__(self, model_path: str, device: str = 'cpu'):
        try:
            from stable_baselines3 import PPO
        except ImportError as exc:
            raise RuntimeError(
                'stable_baselines3 is required to load PPO policies for online inference.'
            ) from exc
        self._model = PPO.load(model_path, device=device)

    def predict(self, observation: np.ndarray) -> np.ndarray:
        action, _ = self._model.predict(observation, deterministic=True)
        return np.asarray(action, dtype=np.float32)


class MappoActorPolicyRuntime:
    def __init__(self, model_path: str, device: str = 'cpu'):
        try:
            import torch
            from torch import nn
        except ImportError as exc:
            raise RuntimeError(
                'torch is required to load MAPPO policies for online inference.'
            ) from exc

        checkpoint = torch.load(model_path, map_location=device, weights_only=False)
        hidden_sizes = tuple(int(value) for value in checkpoint.get('hidden_sizes', [128, 128]))
        self.obs_dim = int(checkpoint['local_observation_size'])
        self.action_dim = int(checkpoint['action_dim'])
        self.rl_control_mode = 'pure'
        action_bounds = checkpoint.get('action_bounds', {'linear_delta': 0.3, 'angular_delta': 0.4})
        linear_bound = float(action_bounds.get('linear_delta', 0.3))
        angular_bound = float(action_bounds.get('angular_delta', 0.4))
        linear_bound = max(linear_bound, float(checkpoint.get('cruise_speed', linear_bound)))
        angular_bound = max(angular_bound, float(checkpoint.get('max_angular_velocity', angular_bound)))
        self.action_bounds = ActionBounds(
            linear_delta=linear_bound,
            angular_delta=angular_bound,
        )
        self.action_contract = RuntimeActionContract(checkpoint, linear_bound)
        self.observation_frame = str(checkpoint.get('observation_frame', LEGACY_FRAME))
        if self.observation_frame not in OBSERVATION_FRAMES:
            raise RuntimeError(f'Unsupported observation_frame {self.observation_frame!r} in {model_path}.')
        # Models trained where these features were constant must see the same constants at deployment.
        self.neutral_conflict_features = bool(checkpoint.get('neutral_conflict_features', False))
        self.checkpoint_angular_authority_floor = checkpoint.get('angular_authority_floor')
        self.heading_omega_deadband = float(checkpoint.get('heading_omega_deadband', 0.06))
        self.heading_omega_reference = float(checkpoint.get('heading_omega_reference', 0.85))
        self.angular_authority_power = float(checkpoint.get('angular_authority_power', 1.6))
        self.angular_accel_limit = float(checkpoint.get('angular_accel_limit', 1.8))
        self.angular_decel_limit = float(checkpoint.get('angular_decel_limit', 2.4))
        self.conflict_turn_relief = float(checkpoint.get('conflict_turn_relief', 0.55))
        reward_config = checkpoint.get('reward_config', {}) or {}
        self.conflict_distance = float(reward_config.get('conflict_distance', 5.0))
        self.anticipation_distance = float(reward_config.get('anticipation_distance', self.conflict_distance))
        self._torch = torch
        self._device = torch.device(device)

        if checkpoint.get('neighbor_attention', False):
            from .neighbor_attention import build_attention_actor_from_checkpoint
            self._actor = build_attention_actor_from_checkpoint(
                checkpoint, nn, torch, self._device,
            )
            self.obs_dim = int(getattr(self._actor, 'obs_dim', self.obs_dim))
        else:
            layers = []
            current_dim = self.obs_dim
            for hidden_size in hidden_sizes:
                layers.append(nn.Linear(current_dim, hidden_size))
                layers.append(nn.Tanh())
                current_dim = hidden_size
            layers.append(nn.Linear(current_dim, self.action_dim))

            self._actor = nn.Sequential(*layers).to(self._device)
            self._actor.load_state_dict(checkpoint['actor_state_dict'])
            self._actor.eval()
        self._squash_actions = bool(checkpoint.get('squash_actions', False))
        if self._squash_actions:
            _low = checkpoint.get('action_low', [0.0, -0.4])
            _high = checkpoint.get('action_high', [0.4, 0.4])
            self._sq_action_low = torch.as_tensor(_low, dtype=torch.float32, device=self._device)
            self._sq_action_high = torch.as_tensor(_high, dtype=torch.float32, device=self._device)

        self._obs_normalizer = None
        if checkpoint.get('normalize_observations') and 'obs_normalizer' in checkpoint:
            self._obs_normalizer = ObservationNormalizer(self.obs_dim)
            norm_state = _migrate_runtime_normalizer_state(checkpoint['obs_normalizer'], self.obs_dim)
            self._obs_normalizer.load_state_dict(norm_state)

    def predict(self, observation: np.ndarray) -> np.ndarray:
        observation = _adapt_runtime_observation_dim(observation, self.obs_dim)
        if self._obs_normalizer is not None:
            observation = self._obs_normalizer.normalize(observation)
        obs_tensor = self._torch.as_tensor(
            observation,
            dtype=self._torch.float32,
            device=self._device,
        ).unsqueeze(0)
        with self._torch.no_grad():
            raw = self._actor(obs_tensor)
            if self._squash_actions:
                raw = raw.clamp(-3.0, 3.0)
                _half = (self._sq_action_high - self._sq_action_low) / 2.0
                _mid = (self._sq_action_high + self._sq_action_low) / 2.0
                action = (self._torch.tanh(raw) * _half + _mid).cpu().numpy()[0]
            else:
                action = raw.cpu().numpy()[0]
        return np.asarray(action, dtype=np.float32)


def _load_runtime_policy(model_path: str, policy_kind: str, device: str):
    resolved_model_path = _resolve_model_path(model_path)
    if policy_kind == 'zero':
        return ZeroPolicy()

    inferred_kind = policy_kind
    if inferred_kind == 'auto':
        if resolved_model_path.endswith('.npz'):
            inferred_kind = 'bc'
        elif resolved_model_path.endswith('.zip'):
            inferred_kind = 'ppo'
        elif resolved_model_path.endswith('.pt'):
            inferred_kind = 'mappo'
        else:
            raise RuntimeError(f'Unable to infer policy type from model path: {resolved_model_path}')

    if inferred_kind == 'bc':
        return load_policy(resolved_model_path)
    if inferred_kind == 'ppo':
        return PpoPolicyRuntime(resolved_model_path, device=device)
    if inferred_kind == 'mappo':
        return MappoActorPolicyRuntime(resolved_model_path, device=device)
    raise RuntimeError(f'Unsupported online policy type: {inferred_kind}')


def _load_solo_expert(model_path: str, device: str, base) -> 'MappoActorPolicyRuntime':
    """Load an optional no-neighbour expert that must share the base policy's interface exactly."""
    if not isinstance(base, MappoActorPolicyRuntime):
        raise RuntimeError('Solo expert routing requires a MAPPO base policy.')
    if base.observation_frame != LEGACY_FRAME:
        raise RuntimeError('The base policy must use the legacy observation frame.')
    expert = MappoActorPolicyRuntime(_resolve_model_path(model_path), device=device)
    shared = ('obs_dim', 'action_dim', 'heading_omega_deadband', 'heading_omega_reference',
              'angular_authority_power', 'angular_accel_limit', 'angular_decel_limit', 'conflict_turn_relief',
              '_squash_actions')
    for name in shared:
        if getattr(expert, name) != getattr(base, name):
            raise RuntimeError(f'Solo expert {name}={getattr(expert, name)!r} differs from base {getattr(base, name)!r}.')
    if vars(expert.action_contract) != vars(base.action_contract) or expert.action_bounds != base.action_bounds:
        raise RuntimeError('Solo expert action contract or bounds differ from the base policy.')
    for name in ('_sq_action_low', '_sq_action_high'):
        if hasattr(base, name) and not bool((getattr(expert, name) == getattr(base, name)).all()):
            raise RuntimeError(f'Solo expert {name} differs from the base policy.')
    return expert


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Run a trained policy online and publish to rl_policy/cmd_vel.')
    parser.add_argument('--model', required=True, help='Path to policy model (.npz for BC, .zip for PPO, .pt for MAPPO).')
    parser.add_argument('--solo-expert-model', default='',
                        help='Optional MAPPO actor used only when no neighbour is observed; the base model handles every multi-boat step.')
    parser.add_argument('--policy', choices=['auto', 'bc', 'ppo', 'mappo', 'zero'], default='auto', help='Policy backend.')
    parser.add_argument('--rl-control-mode', choices=['auto', 'pure'], default='pure', help='Deprecated compatibility flag. Only pure final-command policies are supported.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--device', default='cpu', help='Torch device for PPO inference.')
    parser.add_argument('--publish-rate', type=float, default=10.0, help='Policy publish rate in Hz.')
    parser.add_argument('--max-neighbors', type=int, default=3, help='Maximum neighbors encoded into the observation vector.')
    parser.add_argument('--disable-controller-param', action='store_true', help='Do not auto-enable rl_policy_enabled on velocity_controller_node.')
    parser.add_argument('--policy-trace-dir', default='~/usv_logs', help='Directory for per-step policy diagnostics.')
    parser.add_argument('--disable-policy-trace', action='store_true', help='Disable per-step policy diagnostics.')
    parser.add_argument('--encounter-type', choices=['auto', 'none', 'head_on', 'crossing', 'overtaking'], default='auto',
                        help='Encounter type for models trained with scenario conditioning (fresh54+). '
                             '"auto" classifies dynamically from neighbour geometry each step; '
                             'a fixed name uses a static one-hot; "none" sends all-zeros.')
    parser.add_argument('--clear-ahead-route-gate', action='store_true',
                        help='When the goal-direction forward cone is clear and no close neighbour exists, blend pure-RL output back toward raw route navigation.')
    parser.add_argument('--clear-ahead-route-distance', type=float, default=5.0,
                        help='Forward-cone blocker distance for online clear-ahead route gate.')
    parser.add_argument('--clear-ahead-route-bearing-deg', type=float, default=35.0,
                        help='Half-angle of the goal-direction forward cone for online clear-ahead route gate.')
    parser.add_argument('--clear-ahead-route-min-neighbor-separation', type=float, default=2.0,
                        help='All-around nearest-neighbor separation required before online clear-ahead route gate can activate.')
    parser.add_argument('--clear-ahead-route-max-conflict-level', type=float, default=0.30,
                        help='Maximum projection conflict level allowed before online clear-ahead route gate is disabled.')
    parser.add_argument('--clear-ahead-route-linear-blend', type=float, default=0.35,
                        help='Blend factor from RL linear speed toward raw clear-ahead route speed.')
    parser.add_argument('--clear-ahead-route-omega-blend', type=float, default=0.85,
                        help='Blend factor from RL yaw rate toward raw/heading clear-ahead yaw rate.')
    parser.add_argument('--clear-ahead-route-min-speed', type=float, default=0.18,
                        help='Minimum forward speed target used by online clear-ahead route gate.')
    parser.add_argument('--clear-ahead-route-max-omega', type=float, default=0.18,
                        help='Absolute yaw-rate cap used by online clear-ahead route gate.')
    parser.add_argument('--clear-ahead-route-heading-reference', type=float, default=0.55,
                        help='Heading-error magnitude mapped to max omega when raw yaw is unavailable.')
    parser.add_argument('--disable-turn-governor', action='store_true',
                        help='Disable the veteran-driver turn governor (heading-error speed cap).')
    parser.add_argument('--turn-governor-floor', type=float, default=0.45,
                        help='Turn governor: fraction of cruise kept at 90deg+ heading error (speed cap = cruise*(floor+(1-floor)*cos(he))). 0.45 keeps a gentle arc instead of a spin-in-place (user 2026-07-12).')
    parser.add_argument('--turn-governor-min-speed', type=float, default=0.14,
                        help='Turn governor: absolute lower bound of the speed cap (m/s) so the boat always keeps steerage way and turns on an arc.')
    parser.add_argument('--turn-governor-cruise', type=float, default=0.60,
                        help='Turn governor: default cruise speed reference (m/s), dynamically overridden by /set_velocity_cruise_speed.')
    parser.add_argument('--enable-safety-layers', action='store_true',
                        help='Re-enable node post-processing (turn governor/assist, proximity speed scaling, '
                             'push-through, deadlock/orbit breakers, give-way guard, route gate). Off by default.')
    parser.add_argument('--disable-turn-assist', action='store_true',
                        help='Disable the short-side turn assist (minimum-heading-error steering at waypoint switches).')
    parser.add_argument('--turn-assist-threshold-deg', type=float, default=45.0,
                        help='Turn assist engages when |heading error| exceeds this angle (deg).')
    parser.add_argument('--turn-assist-release-deg', type=float, default=25.0,
                        help='Turn assist releases when |heading error| drops below this angle (deg, hysteresis).')
    parser.add_argument('--turn-assist-min-separation', type=float, default=2.0,
                        help='Turn assist only engages when the nearest neighbour is farther than this (m), so avoidance geometry is never overridden.')
    parser.add_argument('--turn-assist-max-omega', type=float, default=0.60,
                        help='Turn assist: absolute yaw-rate command toward the short side (rad/s).')
    parser.add_argument('--turn-assist-blend', type=float, default=0.85,
                        help='Turn assist: blend factor from RL omega toward the short-side omega.')
    return parser.parse_args(argv)


class PolicyInferenceNode(Node):
    def __init__(
        self,
        *,
        namespace: str,
        model_path: str,
        policy_kind: str,
        rl_control_mode: str,
        device: str,
        publish_rate: float,
        max_neighbors: int,
        enable_controller_param: bool,
        encounter_type: str = 'none',
        clear_ahead_route_gate: bool = False,
        clear_ahead_route_distance: float = 5.0,
        clear_ahead_route_bearing_deg: float = 35.0,
        clear_ahead_route_min_neighbor_separation: float = 2.0,
        clear_ahead_route_max_conflict_level: float = 0.30,
        clear_ahead_route_linear_blend: float = 0.35,
        clear_ahead_route_omega_blend: float = 0.85,
        clear_ahead_route_min_speed: float = 0.18,
        clear_ahead_route_max_omega: float = 0.18,
        clear_ahead_route_heading_reference: float = 0.55,
        turn_governor_enabled: bool = True,
        turn_governor_floor: float = 0.45,
        turn_governor_min_speed: float = 0.14,
        turn_governor_cruise: float = 0.34,
        cruise_speed_override: Optional[float] = None,
        turn_assist_enabled: bool = True,
        turn_assist_threshold_deg: float = 45.0,
        turn_assist_release_deg: float = 25.0,
        turn_assist_min_separation: float = 2.0,
        turn_assist_max_omega: float = 0.60,
        turn_assist_blend: float = 0.85,
        safety_layers_enabled: bool = False,
        policy_trace_dir: Optional[str] = '~/usv_logs',
        solo_expert_model: str = '',
    ):
        resolved_namespace = namespace if namespace.startswith('/') else f'/{namespace}'
        super().__init__('policy_inference_node', namespace=resolved_namespace)
        self._namespace = resolved_namespace
        self._usv_id = resolved_namespace.strip('/')
        self._policy = _load_runtime_policy(model_path, policy_kind, device)
        self._solo_expert = _load_solo_expert(solo_expert_model, device, self._policy) if solo_expert_model else None
        self._conflict_latched: set[str] = set()
        self._solo_expert_path = _resolve_model_path(solo_expert_model) if solo_expert_model else None

        # ---- Model info logging ----
        resolved_path = _resolve_model_path(model_path)
        policy_type_name = type(self._policy).__name__
        self.get_logger().info('========== Model Info ==========')
        self.get_logger().info(f'Model path : {resolved_path}')
        self.get_logger().info(f'Policy type: {policy_type_name}')
        self.get_logger().info(f'Device     : {device}')
        try:
            fstat = os.stat(resolved_path)
            mod_time = datetime.fromtimestamp(fstat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
            size_mb = fstat.st_size / (1024 * 1024)
            self.get_logger().info(f'Model size : {size_mb:.2f} MB')
            self.get_logger().info(f'Modified   : {mod_time}')
        except OSError:
            pass
        step_match = re.search(r'step[_]?(\d+)', os.path.basename(resolved_path))
        if step_match:
            self.get_logger().info(f'Checkpoint step: {step_match.group(1)}')
        if hasattr(self._policy, 'obs_dim'):
            self.get_logger().info(f'Obs dim    : {self._policy.obs_dim}')
        if hasattr(self._policy, 'action_dim'):
            self.get_logger().info(f'Action dim : {self._policy.action_dim}')
        if hasattr(self._policy, 'action_bounds'):
            ab = self._policy.action_bounds
            self.get_logger().info(f'Action bounds: linear={ab.linear_delta:.3f}, angular={ab.angular_delta:.3f}')
        self.get_logger().info('================================')

        self._max_neighbors = max(1, max_neighbors)
        self._publish_rate = float(publish_rate)
        self._action_bounds = ActionBounds()
        if hasattr(self._policy, 'action_bounds'):
            self._action_bounds = getattr(self._policy, 'action_bounds')
        resolved_control_mode = 'pure'
        if int(getattr(self._policy, 'action_dim', 2)) != 2:
            raise RuntimeError('Pure RL online inference requires a policy with 2D action output.')
        self._rl_control_mode = resolved_control_mode
        self._enable_controller_param = enable_controller_param
        self._controller_node_name = f'{resolved_namespace}/velocity_controller_node'
        self._parameter_client = AsyncParameterClient(self, self._controller_node_name)
        self._controller_param_enabled = False
        self._last_enable_attempt = 0.0
        self._ready_logged = False
        self._first_action_logged = False
        self._startup_monotonic = time.monotonic()
        self._last_waiting_log = self._startup_monotonic
        self._last_waiting_signature: Optional[tuple] = None
        self._waiting_repeat_count = 0
        self._first_pose_logged = False
        self._first_velocity_logged = False
        self._first_feedback_logged = False
        self._first_raw_cmd_logged = False
        self._first_final_cmd_logged = False
        self._first_neighbor_update_logged = False
        self._no_neighbor_passthrough_logged = False
        self._active_goal_id: Optional[int] = None
        self._route_start: Optional[Tuple[float, float]] = None
        self._route_goal: Optional[Tuple[float, float]] = None
        self._raw_route_inputs: dict = {}
        self._virtual_goal_active = False
        self._first_goal_logged = False
        self._first_active_goal_feedback_logged = False
        self._head_on_guard_logged = False
        self._head_on_guard_active = False
        self._head_on_guard_linear_cap = 0.0
        self._head_on_guard_turn_floor = 0.0
        self._head_on_guard_hold_until = 0.0
        self._head_on_guard_last_update = self._startup_monotonic
        self._clear_ahead_route_gate = bool(clear_ahead_route_gate)
        self._clear_ahead_route_distance = max(0.0, float(clear_ahead_route_distance))
        self._clear_ahead_route_bearing = math.radians(max(1.0, min(179.0, float(clear_ahead_route_bearing_deg))))
        self._clear_ahead_route_min_neighbor_separation = max(0.0, float(clear_ahead_route_min_neighbor_separation))
        self._clear_ahead_route_max_conflict_level = max(0.0, float(clear_ahead_route_max_conflict_level))
        self._clear_ahead_route_linear_blend = max(0.0, min(1.0, float(clear_ahead_route_linear_blend)))
        self._clear_ahead_route_omega_blend = max(0.0, min(1.0, float(clear_ahead_route_omega_blend)))
        self._clear_ahead_route_min_speed = max(0.0, float(clear_ahead_route_min_speed))
        # fresh628 deployment governor (user 2026-07-12: "得到下一个航点后, 按转弯
        # 角度调整前进速度, 像老司机一样" / "不允许切换后继续满速直行"):
        # SITL evidence showed 30/30 waypoint switches at FULL cruise speed
        # regardless of required turn (3s-after-switch mean speed 0.30-0.35,
        # sail-away up to 3.2m). The policy observation has NO next-waypoint
        # preview so it cannot anticipate; this deterministic cap enforces
        # veteran-driver behaviour at the actuator: cap = cruise *
        # (floor + (1-floor)*max(0, cos(heading_error))), never below
        # turn_governor_min_speed (steerage way). he=0 -> full cruise;
        # he=60deg -> ~0.65*cruise; he>=90deg -> floor. The cap only LIMITS
        # the RL output (never raises it), so avoidance slowdowns remain.
        self._turn_governor_enabled = bool(turn_governor_enabled)
        self._turn_governor_floor = min(1.0, max(0.0, float(turn_governor_floor)))
        self._turn_governor_min_speed = max(0.0, float(turn_governor_min_speed))
        self._turn_governor_cruise = max(0.05, float(turn_governor_cruise))
        self._dynamic_cruise_speed: Optional[float] = (
            None if cruise_speed_override is None else max(0.05, float(cruise_speed_override))
        )
        # fresh628 short-side turn assist (user 2026-07-12: "应该按最小航向误差的
        # 转弯方向转向, 而不是转更大的角度"): SITL showed ~50% of waypoint
        # switches turned the FAR side and low-speed heading oscillation
        # (net turn 5-30x the required angle). Sign convention verified from
        # logs: omega SAME sign as heading_error converges (he_dot = -omega_yaw
        # with he = target_yaw - yaw). When |he| exceeds the threshold and no
        # neighbour is inside min_separation (avoidance geometry stays RL-owned),
        # omega is blended toward clamp(he/ref)*max_omega — a steady short-side
        # turn. Hysteresis: engage above threshold, release below release_deg.
        self._turn_assist_enabled = bool(turn_assist_enabled)
        self._turn_assist_threshold = math.radians(max(5.0, float(turn_assist_threshold_deg)))
        self._turn_assist_release = math.radians(max(2.0, min(float(turn_assist_release_deg), float(turn_assist_threshold_deg))))
        self._turn_assist_min_separation = max(0.0, float(turn_assist_min_separation))
        self._turn_assist_max_omega = max(0.05, float(turn_assist_max_omega))
        self._turn_assist_blend = min(1.0, max(0.0, float(turn_assist_blend)))
        self._turn_assist_active = False
        # Off = the published command is exactly the model's projected action, as in training.
        self._safety_layers_enabled = bool(safety_layers_enabled)
        # fresh628 role latch state: neighbour id -> (encounter_type, role).
        self._role_latch = {}
        self._role_latch_radius = 2.5
        self._role_latch_release = 3.5
        # fresh629 dual-give-way deadlock breaker state (SITL 2026-07-14
        # 104106): both boats latched give-way in a crossing and crept at the
        # 0.05 floor for 35s down to 0.15m separation.
        self._yield_stall_since = None
        self._yield_stall_logged = False
        # fresh630 ORBIT-LOCK BREAKER state (SITL 2026-07-15 175946 t=130-250:
        # 8-figure first encounter locked both boats into a rigid binary orbit
        # -- sep pinned at 0.42m for 120s while omegas held opposite signs
        # (-0.5 / +0.6) and both crept at ~0.11 m/s; encounter classifier
        # returned unset at that range so no COLREGS shaping applied, and the
        # 50% goal-omega blend of the push-through was too weak to break the
        # rotational symmetry). Detect "close + separation frozen" and break
        # the symmetry hard: smaller-ID boat steers to goal at near-full
        # blend and pushes out, larger-ID boat damps its turn and waits.
        self._orbit_lock_since = None
        self._orbit_lock_prev_sep = None
        self._orbit_lock_logged = False
        self._give_way_guard_logged = False
        self._clear_ahead_route_max_omega = max(0.0, float(clear_ahead_route_max_omega))
        self._clear_ahead_route_heading_reference = max(0.05, float(clear_ahead_route_heading_reference))
        self._clear_ahead_route_logged = False

        policy_obs_dim = None
        if hasattr(self._policy, 'obs_dim'):
            policy_obs_dim = int(getattr(self._policy, 'obs_dim'))
        else:
            policy_obs_mean = getattr(self._policy, 'obs_mean', None)
            if policy_obs_mean is not None:
                policy_obs_dim = int(np.asarray(policy_obs_mean).shape[0])

        # Detect whether the model uses encounter-type conditioning.
        self._encounter_type_auto = (encounter_type == 'auto')
        self._encounter_type_index = _ENCOUNTER_TYPE_NAMES.get(encounter_type, -1)
        self._encounter_type_enabled = False
        # Hysteresis state for auto classification
        self._auto_encounter_current: int = -1     # currently active index
        self._auto_encounter_candidate: int = -1   # pending switch
        self._auto_encounter_candidate_since: float = 0.0

        if policy_obs_dim is not None:
            _ego_dim = UsvObservation.ego_feature_size()
            # Check if dimension matches base layout: ego + N*neighbor_dim
            if policy_obs_dim >= _ego_dim and (policy_obs_dim - _ego_dim) % USV_NEIGHBOR_FEATURE_COUNT == 0:
                inferred_neighbors = max(1, (policy_obs_dim - _ego_dim) // USV_NEIGHBOR_FEATURE_COUNT)
                if inferred_neighbors != self._max_neighbors:
                    self.get_logger().info(
                        f'Overriding max_neighbors from {self._max_neighbors} to {inferred_neighbors} based on model observation dimension {policy_obs_dim}.'
                    )
                    self._max_neighbors = inferred_neighbors
            # Check if dimension matches encounter-type layout: ego + N*neighbor_dim + ENCOUNTER_TYPE_COUNT
            elif policy_obs_dim >= _ego_dim + ENCOUNTER_TYPE_COUNT and (policy_obs_dim - _ego_dim - ENCOUNTER_TYPE_COUNT) % USV_NEIGHBOR_FEATURE_COUNT == 0:
                inferred_neighbors = max(1, (policy_obs_dim - _ego_dim - ENCOUNTER_TYPE_COUNT) // USV_NEIGHBOR_FEATURE_COUNT)
                self._encounter_type_enabled = True
                if inferred_neighbors != self._max_neighbors:
                    self.get_logger().info(
                        f'Overriding max_neighbors from {self._max_neighbors} to {inferred_neighbors} based on model observation dimension {policy_obs_dim}.'
                    )
                    self._max_neighbors = inferred_neighbors
                if self._encounter_type_auto:
                    self.get_logger().info(
                        f'Encounter-type conditioning enabled in AUTO mode (obs_dim={policy_obs_dim}). '
                        'Will classify dynamically from neighbour geometry each step.'
                    )
                elif self._encounter_type_index < 0:
                    self.get_logger().warn(
                        f'Model expects encounter-type conditioning (obs_dim={policy_obs_dim}) but --encounter-type is "none". '
                        'The one-hot will be all zeros; behaviour may differ from training.'
                    )
                else:
                    self.get_logger().info(
                        f'Encounter-type conditioning enabled: {encounter_type} (index={self._encounter_type_index}).'
                    )
            else:
                # Backwards compatibility: try older ego layouts.
                _legacy_rawless_ego = 17
                if policy_obs_dim >= _legacy_rawless_ego and (policy_obs_dim - _legacy_rawless_ego) % USV_NEIGHBOR_FEATURE_COUNT == 0:
                    inferred_neighbors = max(1, (policy_obs_dim - _legacy_rawless_ego) // USV_NEIGHBOR_FEATURE_COUNT)
                    self.get_logger().warn(
                        f'Model uses legacy ego_dim=17 layout (obs_dim={policy_obs_dim}). '
                        'raw CTE/overflow observations will be removed before inference.'
                    )
                    if inferred_neighbors != self._max_neighbors:
                        self._max_neighbors = inferred_neighbors
                elif policy_obs_dim >= _legacy_rawless_ego + ENCOUNTER_TYPE_COUNT and (policy_obs_dim - _legacy_rawless_ego - ENCOUNTER_TYPE_COUNT) % USV_NEIGHBOR_FEATURE_COUNT == 0:
                    inferred_neighbors = max(1, (policy_obs_dim - _legacy_rawless_ego - ENCOUNTER_TYPE_COUNT) // USV_NEIGHBOR_FEATURE_COUNT)
                    self._encounter_type_enabled = True
                    self.get_logger().warn(
                        f'Model uses legacy ego_dim=17 layout with encounter type (obs_dim={policy_obs_dim}). '
                        'raw CTE/overflow observations will be removed before inference.'
                    )
                    if inferred_neighbors != self._max_neighbors:
                        self._max_neighbors = inferred_neighbors
                else:
                    # Backwards compatibility: try old ego_dim=12/11 layouts.
                    _old_ego = 12
                    if policy_obs_dim >= _old_ego and (policy_obs_dim - _old_ego) % 6 == 0:
                        inferred_neighbors = max(1, (policy_obs_dim - _old_ego) // 6)
                        self.get_logger().warn(
                            f'Model uses legacy ego_dim=12 layout (obs_dim={policy_obs_dim}). '
                            f'fresh96 route/timing observations will be ignored by the model.'
                        )
                        if inferred_neighbors != self._max_neighbors:
                            self._max_neighbors = inferred_neighbors
                    elif policy_obs_dim >= _old_ego + ENCOUNTER_TYPE_COUNT and (policy_obs_dim - _old_ego - ENCOUNTER_TYPE_COUNT) % 6 == 0:
                        inferred_neighbors = max(1, (policy_obs_dim - _old_ego - ENCOUNTER_TYPE_COUNT) // 6)
                        self._encounter_type_enabled = True
                        self.get_logger().warn(
                            f'Model uses legacy ego_dim=12 layout with encounter type (obs_dim={policy_obs_dim}). '
                            f'fresh96 route/timing observations will be ignored by the model.'
                        )
                        if inferred_neighbors != self._max_neighbors:
                            self._max_neighbors = inferred_neighbors
                    else:
                        # Try older ego_dim=11 layouts (scalar heading_error).
                        _oldest_ego = 11
                        if policy_obs_dim >= _oldest_ego and (policy_obs_dim - _oldest_ego) % 6 == 0:
                            inferred_neighbors = max(1, (policy_obs_dim - _oldest_ego) // 6)
                            self.get_logger().warn(
                                f'Model uses legacy ego_dim=11 layout (obs_dim={policy_obs_dim}). '
                                f'sin/cos heading_error observation will be collapsed to scalar for this model.'
                            )
                            if inferred_neighbors != self._max_neighbors:
                                self._max_neighbors = inferred_neighbors
                        elif policy_obs_dim >= _oldest_ego + ENCOUNTER_TYPE_COUNT and (policy_obs_dim - _oldest_ego - ENCOUNTER_TYPE_COUNT) % 6 == 0:
                            inferred_neighbors = max(1, (policy_obs_dim - _oldest_ego - ENCOUNTER_TYPE_COUNT) // 6)
                            self._encounter_type_enabled = True
                            self.get_logger().warn(
                                f'Model uses legacy ego_dim=11 layout with encounter type (obs_dim={policy_obs_dim}). '
                                f'sin/cos heading_error observation will be collapsed to scalar for this model.'
                            )
                            if inferred_neighbors != self._max_neighbors:
                                self._max_neighbors = inferred_neighbors
                        else:
                            raise RuntimeError(
                                f'Unsupported observation dimension {policy_obs_dim}; cannot map it to UsvObservation vector slots.'
                            )

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._pose_msg: Optional[PoseStamped] = None
        self._velocity_msg: Optional[TwistStamped] = None
        self._feedback_msg: Optional[NavigationFeedback] = None
        self._raw_cmd_msg: Optional[TwistStamped] = None
        self._final_cmd_msg: Optional[PositionTarget] = None
        self._neighbor_states: Dict[str, NeighborState] = {}
        self._telemetry_status = None
        self._telemetry_status_received = None
        self.declare_parameter('navigation_input_timeout_s', 1.5)
        self._navigation_input_timeout = float(self.get_parameter('navigation_input_timeout_s').value)
        if not math.isfinite(self._navigation_input_timeout) or self._navigation_input_timeout <= 0.0:
            raise ValueError('navigation_input_timeout_s must be positive and finite')
        self._navigation_inputs = {}
        self._completed_goal_ids = deque(maxlen=128)
        self._navigation_terminal_reason = None
        self._navigation_guard_reason = None
        self._last_guard_trace = -math.inf

        self.create_subscription(String, 'telemetry/status', self._telemetry_status_callback, qos_reliable)
        self.create_subscription(PoseStamped, 'local_position/pose_from_gps', self._pose_callback, qos_best_effort)
        self.create_subscription(TwistStamped, 'local_position/velocity_local', self._velocity_callback, qos_best_effort)
        self.create_subscription(NavigationGoal, 'set_usv_nav_goal', self._nav_goal_callback, qos_reliable)
        self.create_subscription(Float32, 'set_velocity_cruise_speed', self._cruise_speed_callback, qos_reliable)
        self.create_subscription(NavigationResult, 'navigation_result', self._nav_result_callback, qos_reliable)
        self.create_subscription(Bool, 'stop_navigation', self._stop_navigation_callback, qos_reliable)
        self.create_subscription(NavigationFeedback, 'navigation_feedback', self._feedback_callback, qos_best_effort)
        self.create_subscription(NavigationFeedback, 'velocity_controller/feedback', self._feedback_callback, qos_best_effort)
        self.create_subscription(TwistStamped, 'velocity_controller/raw_cmd', self._raw_cmd_callback, qos_best_effort)
        self.create_subscription(PositionTarget, 'setpoint_raw/local', self._final_cmd_callback, qos_best_effort)
        self.create_subscription(FleetNeighborPoses, 'apf/neighbors', self._fleet_neighbors_callback, qos_best_effort)

        self._action_pub = self.create_publisher(TwistStamped, 'rl_policy/cmd_vel', qos_best_effort)

        # Publish encounter-type index so log_collector can record it.
        self._encounter_type_pub = self.create_publisher(Int8, 'rl_policy/encounter_type', qos_best_effort)

        # Publish model metadata with transient_local so late subscribers can inspect the active model.
        qos_model_info = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._model_info_pub = self.create_publisher(String, 'rl_policy/model_info', qos_model_info)
        model_info_dict = {
            'model_path': resolved_path,
            'model_name': os.path.basename(resolved_path),
            'policy_type': policy_type_name,
            'device': device,
        }
        if step_match:
            model_info_dict['checkpoint_step'] = int(step_match.group(1))
        if hasattr(self._policy, 'obs_dim'):
            model_info_dict['obs_dim'] = self._policy.obs_dim
        if hasattr(self._policy, 'action_dim'):
            model_info_dict['action_dim'] = self._policy.action_dim
        if hasattr(self._policy, 'action_bounds'):
            ab = self._policy.action_bounds
            model_info_dict['action_bounds'] = {'linear': ab.linear_delta, 'angular': ab.angular_delta}
        if hasattr(self._policy, '_squash_actions'):
            model_info_dict['squash_actions'] = bool(self._policy._squash_actions)
        model_info_dict['runtime_action_contract_version'] = 1
        model_info_dict['solo_conflict_features_neutralized'] = True
        model_info_dict['navigation_input_guard_version'] = 1
        model_info_dict['navigation_input_timeout_s'] = self._navigation_input_timeout
        if isinstance(self._policy, MappoActorPolicyRuntime):
            model_info_dict['action_mode'] = self._policy.action_contract.action_mode
            model_info_dict['action_contract'] = vars(self._policy.action_contract).copy()
            model_info_dict['checkpoint_angular_authority_floor'] = self._policy.checkpoint_angular_authority_floor
        model_info_dict['projection'] = {
            'control_dt': 1.0 / max(1.0, self._publish_rate),
            'angular_authority_floor': 0.35,
            'min_forward_speed_floor': 0.0,
            **{name: getattr(self._policy, name, None) for name in (
                'heading_omega_deadband', 'heading_omega_reference', 'angular_authority_power',
                'angular_accel_limit', 'angular_decel_limit', 'conflict_turn_relief',
            )},
        }
        model_info_dict['solo_expert'] = {
            'enabled': self._solo_expert is not None,
            'model_path': self._solo_expert_path,
            'observation_frame': None if self._solo_expert is None else self._solo_expert.observation_frame,
            'route': 'solo expert when observation has no neighbours, base model otherwise',
        }
        model_info_dict['turn_assist'] = {
            'enabled': self._turn_assist_enabled,
            'threshold_rad': self._turn_assist_threshold,
            'release_rad': self._turn_assist_release,
            'min_separation': self._turn_assist_min_separation,
            'max_omega': self._turn_assist_max_omega,
            'blend': self._turn_assist_blend,
        }
        try:
            fstat = os.stat(resolved_path)
            model_info_dict['size_bytes'] = fstat.st_size
            model_info_dict['modified'] = datetime.fromtimestamp(fstat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
        except OSError:
            pass
        model_info_msg = String()
        model_info_msg.data = json.dumps(model_info_dict, ensure_ascii=False)
        self._model_info_pub.publish(model_info_msg)
        self.get_logger().info('Published model info on rl_policy/model_info')

        self._policy_trace = None
        if policy_trace_dir is not None:
            self._policy_trace = PolicyTrace(
                policy_trace_dir, self._usv_id,
                {'namespace': self._namespace, **model_info_dict}, self.get_logger().warning,
            )
            if self._policy_trace.enabled:
                self.get_logger().info(f'Policy trace: {self._policy_trace.path}')
        self._publish_timer = self.create_timer(1.0 / max(1.0, self._publish_rate), self._publish_action)

    def destroy_node(self):
        trace = getattr(self, '_policy_trace', None)
        if trace is not None:
            trace.close()
        return super().destroy_node()

    def _projection_conflict_level(self, observation) -> float:
        if observation is None or not observation.neighbors:
            return 0.0
        lookahead_distance = max(self._policy.anticipation_distance, self._policy.conflict_distance, 1e-3)
        min_neighbor_distance = observation.min_neighbor_distance()
        raw_level = float(np.clip((lookahead_distance - min_neighbor_distance) / lookahead_distance, 0.0, 1.0))
        # Reduce conflict level when the closest neighbour is separating,
        # so the agent regains turning authority to break free.
        if observation.neighbors and raw_level > 0.0:
            closest = min(observation.neighbors, key=lambda n: n.distance)
            d = max(closest.distance, 1e-3)
            range_rate = -(closest.rel_x * closest.rel_vx + closest.rel_y * closest.rel_vy) / d
            if range_rate < -0.02:
                separation_relief = min(1.0, abs(range_rate) / 0.15)
                raw_level *= max(0.25, 1.0 - 0.6 * separation_relief)
        return raw_level

    def _clear_ahead_route_gate_active(self, observation) -> bool:
        if not self._clear_ahead_route_gate or observation is None:
            return False
        if self._clear_ahead_route_distance <= 0.0:
            return False
        if observation.distance_to_goal <= 1.0:
            return False
        if observation.min_neighbor_distance() < self._clear_ahead_route_min_neighbor_separation:
            return False
        if self._projection_conflict_level(observation) > self._clear_ahead_route_max_conflict_level:
            return False

        # Neighbor bearings are ego-yaw-relative.  The route/goal direction is
        # heading_error radians away from ego heading, so subtract it to measure
        # whether a neighbor blocks the goal-direction forward cone.
        for neighbor in observation.neighbors:
            if neighbor.distance <= 1e-3 or neighbor.distance >= self._clear_ahead_route_distance:
                continue
            relative_to_goal = self._wrap_angle(float(neighbor.bearing) - float(observation.heading_error))
            if abs(relative_to_goal) <= self._clear_ahead_route_bearing:
                return False
        return True

    def _reset_head_on_guard_state(self):
        self._head_on_guard_logged = False
        self._head_on_guard_active = False
        self._head_on_guard_linear_cap = 0.0
        self._head_on_guard_turn_floor = 0.0
        self._head_on_guard_hold_until = 0.0
        self._head_on_guard_last_update = time.monotonic()

    def _telemetry_status_callback(self, msg: String):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict) or not isinstance(status.get('healthy'), bool):
                raise ValueError('Missing telemetry healthy flag')
            json.dumps(status, allow_nan=False)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f'Invalid telemetry status: {exc}', throttle_duration_sec=10.0)
            return
        self._telemetry_status = status
        self._telemetry_status_received = time.monotonic()

    def _pose_callback(self, msg: PoseStamped):
        self._pose_msg = msg
        if not self._first_pose_logged:
            self.get_logger().info(
                f'First pose sample received after {time.monotonic() - self._startup_monotonic:.2f}s.'
            )
            self._first_pose_logged = True

    def _velocity_callback(self, msg: TwistStamped):
        self._velocity_msg = msg
        if not self._first_velocity_logged:
            self.get_logger().info(
                f'First velocity sample received after {time.monotonic() - self._startup_monotonic:.2f}s.'
            )
            self._first_velocity_logged = True

    def _activate_goal(self, goal_id: int, *, reset_stream_state: bool, nav_mode: Optional[int] = None, target: Optional[Tuple[float, float]] = None, source: str = 'nav_goal'):
        prev_goal_id = self._active_goal_id
        prev_route_goal = self._route_goal
        seamless_pass_through = (
            reset_stream_state
            and prev_goal_id is not None
            and self._navigation_terminal_reason is None
            and self._pose_msg is not None
            and self._final_cmd_msg is not None
            and target is not None
            and int(goal_id) == int(prev_goal_id) + 1
            and self._navigation_input_status()['reason'] is None
        )
        is_new_goal = goal_id != self._active_goal_id
        self._active_goal_id = goal_id

        if not is_new_goal:
            return

        if reset_stream_state:
            if seamless_pass_through:
                # Bridge the 0.1s upstream feedback handover on smooth sequential
                # waypoint switches so the boat enters the arc seamlessly without
                # a [0.0, 0.0] inhibited brake spike.
                own_x = float(self._pose_msg.pose.position.x)
                own_y = float(self._pose_msg.pose.position.y)
                own_yaw = _quat_to_yaw(self._pose_msg)
                dx_g, dy_g = float(target[0]) - own_x, float(target[1]) - own_y
                synth_fb = NavigationFeedback()
                synth_fb.goal_id = int(goal_id)
                synth_fb.distance_to_goal = float(math.hypot(dx_g, dy_g))
                synth_fb.heading_error = float(self._wrap_angle(math.atan2(dy_g, dx_g) - own_yaw))
                synth_fb.timestamp = self.get_clock().now().to_msg()
                if self._record_navigation_input('feedback', synth_fb.timestamp):
                    self._feedback_msg = synth_fb
                else:
                    self._feedback_msg = None
                    self._navigation_inputs.pop('feedback', None)
            else:
                self._feedback_msg = None
                self._raw_cmd_msg = None
                self._final_cmd_msg = None
                self._navigation_inputs.clear()

        self._ready_logged = False
        self._first_action_logged = False
        self._first_active_goal_feedback_logged = False
        self._no_neighbor_passthrough_logged = False
        self._last_waiting_signature = None
        self._waiting_repeat_count = 0
        self._reset_head_on_guard_state()

        # Record route start and goal for CTE computation.
        # On sequential multi-waypoint transitions (wp_{k-1} -> wp_k), anchor
        # _route_start to the previous planned waypoint when within switch radius
        # so the reference line matches the true Ground Station XML route chord.
        if target is not None:
            self._route_goal = target
            if self._pose_msg is not None:
                own_pos = (
                    float(self._pose_msg.pose.position.x),
                    float(self._pose_msg.pose.position.y),
                )
                if (
                    prev_route_goal is not None
                    and prev_goal_id is not None
                    and int(goal_id) == int(prev_goal_id) + 1
                    and math.hypot(own_pos[0] - prev_route_goal[0], own_pos[1] - prev_route_goal[1]) <= 3.0
                    and math.hypot(target[0] - prev_route_goal[0], target[1] - prev_route_goal[1]) > 1.0
                ):
                    self._route_start = prev_route_goal
                else:
                    self._route_start = own_pos
            else:
                self._route_start = None
            # Decide once per goal whether the leg needs virtual sub-goals to
            # stay inside the training distribution (long leg, or leg outside
            # the ±9 m training arena).  Fixed per goal → no mid-leg frame jumps.
            self._virtual_goal_active = False
            if self._route_start is not None:
                sx, sy = self._route_start
                gx, gy = self._route_goal
                route_len = math.hypot(gx - sx, gy - sy)
                out_of_arena = max(abs(sx), abs(sy), abs(gx), abs(gy)) > _POSE_IN_DIST_LIMIT
                if route_len > _VIRTUAL_GOAL_TRIGGER or out_of_arena:
                    self._virtual_goal_active = True
                    self.get_logger().info(
                        f'Virtual sub-goal mode enabled for goal_id={goal_id}: '
                        f'route_len={route_len:.1f}m, out_of_arena={out_of_arena} '
                        f'(leg={_VIRTUAL_LEG_LENGTH:.0f}m).'
                    )

        details = [
            f'goal_id={goal_id}',
        ]
        if nav_mode is not None:
            details.append(f'nav_mode={nav_mode}')
        if target is not None:
            details.append(f'target=({target[0]:.2f}, {target[1]:.2f})')

        action = 'Activated' if reset_stream_state else 'Recovered'
        self.get_logger().info(
            f'{action} navigation goal after {time.monotonic() - self._startup_monotonic:.2f}s '
            f'from {source}: ' + ', '.join(details) + '.'
        )
        self._first_goal_logged = True

    def _nav_goal_callback(self, msg: NavigationGoal):
        goal_id = int(getattr(msg, 'goal_id', 0))
        self._navigation_terminal_reason = None
        if goal_id in self._completed_goal_ids:
            self._completed_goal_ids.remove(goal_id)
        self._activate_goal(
            goal_id,
            reset_stream_state=True,
            nav_mode=int(getattr(msg, 'nav_mode', 0)),
            target=(
                float(msg.target_pose.pose.position.x),
                float(msg.target_pose.pose.position.y),
            ),
            source='set_usv_nav_goal',
        )

    def _record_navigation_input(self, name, stamp):
        now = time.monotonic()
        source_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if source_stamp <= 0.0 or source_stamp > self.get_clock().now().nanoseconds * 1e-9 + 0.05:
            return False
        previous = self._navigation_inputs.get(name)
        if previous is not None and source_stamp <= previous['stamp']:
            previous['received'] = now
            return False
        self._navigation_inputs[name] = {'stamp': source_stamp, 'received': now, 'updated': now}
        return True

    def _navigation_input_status(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        status = {}
        reason = self._navigation_terminal_reason
        if reason is None and self._active_goal_id is None:
            reason = 'no_active_goal'
        for name in ('feedback', 'final_command'):
            sample = self._navigation_inputs.get(name)
            ages = {
                'receipt_age_s': None if sample is None else now - sample['received'],
                'source_update_age_s': None if sample is None else now - sample['updated'],
                'source_stamp_age_s': None if sample is None else ros_now - sample['stamp'],
            }
            fresh = sample is not None and sample['stamp'] > 0.0 and all(
                -0.05 <= age <= self._navigation_input_timeout for age in ages.values()
            )
            status[name] = {**ages, 'fresh': fresh}
            if reason is None and not fresh:
                reason = f'{name}_stale_or_missing'
        if reason is None and (self._feedback_msg is None or int(self._feedback_msg.goal_id) != self._active_goal_id):
            reason = 'feedback_goal_mismatch'
        return {'reason': reason, 'inputs': status}

    def _publish_navigation_stop(self, status):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        self._action_pub.publish(message)
        self._turn_assist_active = False
        self._reset_head_on_guard_state()
        now = time.monotonic()
        changed = status['reason'] != self._navigation_guard_reason
        if changed:
            self.get_logger().warning(f'Policy inhibited: {status["reason"]}; publishing zero command')
        if self._policy_trace is not None and (changed or now - self._last_guard_trace >= 1.0):
            self._policy_trace.write({
                'type': 'inhibited', 'timestamp': self.get_clock().now().nanoseconds * 1e-9,
                'wall_timestamp': time.time(), 'goal_id': self._active_goal_id,
                'navigation_input_status': status, 'published_action': [0.0, 0.0],
            })
            self._last_guard_trace = now
        self._navigation_guard_reason = status['reason']

    def _finish_navigation(self, reason):
        if self._active_goal_id is not None:
            self._completed_goal_ids.append(self._active_goal_id)
        self._active_goal_id = None
        self._feedback_msg = None
        self._final_cmd_msg = None
        self._navigation_inputs.clear()
        self._navigation_terminal_reason = reason
        self._publish_navigation_stop(self._navigation_input_status())

    def _nav_result_callback(self, msg: NavigationResult):
        if int(msg.goal_id) != self._active_goal_id:
            return
        if msg.success and ('已通过' in msg.message or '通过航点' in msg.message):
            return
        self._finish_navigation('navigation_completed' if msg.success else 'navigation_failed')

    def _stop_navigation_callback(self, msg: Bool):
        if msg.data:
            self._finish_navigation('navigation_stopped')

    def _feedback_callback(self, msg: NavigationFeedback):
        feedback_goal_id = int(msg.goal_id)
        if self._navigation_terminal_reason is not None or feedback_goal_id in self._completed_goal_ids:
            return
        if self._active_goal_id is not None and feedback_goal_id != self._active_goal_id:
            return
        if not math.isfinite(msg.distance_to_goal) or not math.isfinite(msg.heading_error):
            return
        if not self._record_navigation_input('feedback', msg.timestamp):
            return
        self._feedback_msg = msg
        if not self._first_feedback_logged:
            self.get_logger().info(
                f'First navigation feedback received after {time.monotonic() - self._startup_monotonic:.2f}s: distance_to_goal={float(msg.distance_to_goal):.2f}, heading_error={float(msg.heading_error):.2f}.'
            )
            self._first_feedback_logged = True

        feedback_goal_id = int(msg.goal_id)
        if self._active_goal_id is None and feedback_goal_id != 0:
            self._activate_goal(
                feedback_goal_id,
                reset_stream_state=False,
                source='navigation_feedback',
            )

        if (
            self._active_goal_id is not None
            and feedback_goal_id == self._active_goal_id
            and not self._first_active_goal_feedback_logged
        ):
            self.get_logger().info(
                f'First feedback for active goal received after {time.monotonic() - self._startup_monotonic:.2f}s: '
                f'goal_id={feedback_goal_id}, distance_to_goal={float(msg.distance_to_goal):.2f}, '
                f'heading_error={float(msg.heading_error):.2f}.'
            )
            self._first_active_goal_feedback_logged = True

    def _raw_cmd_callback(self, msg: TwistStamped):
        self._raw_cmd_msg = msg
        if not self._first_raw_cmd_logged:
            self.get_logger().info(
                f'First raw velocity command received after {time.monotonic() - self._startup_monotonic:.2f}s.'
            )
            self._first_raw_cmd_logged = True

    def _cruise_speed_callback(self, msg: Float32):
        speed = float(getattr(msg, 'data', 0.0))
        if math.isfinite(speed) and speed > 0.05:
            old = self._dynamic_cruise_speed or self._turn_governor_cruise
            self._dynamic_cruise_speed = speed
            self._turn_governor_cruise = speed
            if abs(speed - old) > 1e-3:
                self.get_logger().info(f'Updated RL cruise speed from GS: {old:.2f} -> {speed:.2f} m/s.')

    def _final_cmd_callback(self, msg: PositionTarget):
        if not self._record_navigation_input('final_command', msg.header.stamp):
            return
        self._final_cmd_msg = msg
        if not self._first_final_cmd_logged:
            self.get_logger().info(
                f'First final setpoint command received after {time.monotonic() - self._startup_monotonic:.2f}s.'
            )
            self._first_final_cmd_logged = True

    def _fleet_neighbors_callback(self, msg: FleetNeighborPoses):
        if msg.target_usv_id and msg.target_usv_id != self._usv_id:
            return

        updated: Dict[str, NeighborState] = {}
        for neighbor in msg.neighbors:
            if not neighbor.usv_id or neighbor.usv_id == self._usv_id:
                continue
            updated[neighbor.usv_id] = NeighborState(
                usv_id=str(neighbor.usv_id),
                x=float(neighbor.x),
                y=float(neighbor.y),
                yaw=float(neighbor.yaw),
                vx=float(neighbor.vx),
                vy=float(neighbor.vy),
            )
        self._neighbor_states = updated

        if not self._first_neighbor_update_logged and updated:
            self.get_logger().info(
                f'First neighbor update received after {time.monotonic() - self._startup_monotonic:.2f}s with {len(updated)} tracked neighbors.'
            )
            self._first_neighbor_update_logged = True

    def _maybe_log_waiting_for_observation(self):
        if self._ready_logged:
            return

        now = time.monotonic()
        since_start = now - self._startup_monotonic
        if since_start < 1.5:
            return
        if (now - self._last_waiting_log) < 2.0:
            return

        missing_inputs = []
        if self._pose_msg is None:
            missing_inputs.append('pose')
        if self._velocity_msg is None:
            missing_inputs.append('velocity')
        if self._active_goal_id is None:
            missing_inputs.append('active_goal')
        if not self._neighbor_states:
            missing_inputs.append('neighbors')
        if self._feedback_msg is None:
            missing_inputs.append('feedback')
        elif self._active_goal_id is not None and int(self._feedback_msg.goal_id) != self._active_goal_id:
            missing_inputs.append(f'feedback_for_goal({int(self._feedback_msg.goal_id)}!={self._active_goal_id})')
        if self._final_cmd_msg is None:
            missing_inputs.append('final_cmd')

        waiting_signature = (
            tuple(missing_inputs),
            len(self._neighbor_states),
            self._active_goal_id,
            self._final_cmd_msg is not None,
        )

        is_same_waiting_state = waiting_signature == self._last_waiting_signature
        min_interval = 15.0 if is_same_waiting_state else 2.0
        if (now - self._last_waiting_log) < min_interval:
            return

        if is_same_waiting_state:
            self._waiting_repeat_count += 1
            repeat_suffix = f', repeated={self._waiting_repeat_count}'
        else:
            self._waiting_repeat_count = 0
            repeat_suffix = ''

        self.get_logger().info(
            'Waiting for observation readiness '
            f'after {since_start:.2f}s: missing={missing_inputs or ["none"]}, '
            f'neighbors={len(self._neighbor_states)}, '
            f'active_goal_id={self._active_goal_id}, '
            f'final_cmd={self._final_cmd_msg is not None}{repeat_suffix}.'
        )
        self._last_waiting_log = now
        self._last_waiting_signature = waiting_signature

    def _maybe_enable_controller_param(self):
        if not self._enable_controller_param or self._controller_param_enabled:
            return

        now = time.monotonic()
        if (now - self._last_enable_attempt) < 1.0:
            return
        self._last_enable_attempt = now

        if not self._parameter_client.wait_for_services(timeout_sec=0.1):
            return

        future = self._parameter_client.set_parameters([
            Parameter('rl_policy_enabled', value=True),
            Parameter('rl_policy_fallback_to_raw', value=False),
            Parameter('rl_policy_allow_reverse', value=False),
            Parameter('apf_orca_enabled', value=False),
        ])

        def _done_callback(done_future):
            try:
                result = done_future.result()
            except Exception as exc:
                self.get_logger().warning(f'Failed to enable rl_policy_enabled: {exc}')
                return

            success = False
            if isinstance(result, (list, tuple)):
                success = all(item.successful for item in result)
            else:
                response_items = getattr(result, 'results', None)
                if response_items is not None:
                    success = all(item.successful for item in response_items)
                else:
                    single_result = getattr(result, 'result', None)
                    if single_result is not None and hasattr(single_result, 'successful'):
                        success = bool(single_result.successful)
            if success:
                self._controller_param_enabled = True
                self.get_logger().info('Enabled rl_policy_enabled on velocity_controller_node.')

        future.add_done_callback(_done_callback)

    def _build_observation(self) -> Optional[UsvObservation]:
        if (
            self._pose_msg is None
            or self._velocity_msg is None
            or self._feedback_msg is None
            or self._final_cmd_msg is None
            or self._active_goal_id is None
            or int(self._feedback_msg.goal_id) != self._active_goal_id
        ):
            return None

        yaw = _quat_to_yaw(self._pose_msg)
        own_x = float(self._pose_msg.pose.position.x)
        own_y = float(self._pose_msg.pose.position.y)
        own_vx = float(self._velocity_msg.twist.linear.x)
        own_vy = float(self._velocity_msg.twist.linear.y)
        speed = math.hypot(own_vx, own_vy)

        raw_linear_x = float(self._raw_cmd_msg.twist.linear.x) if self._raw_cmd_msg is not None else 0.0
        raw_angular_z = float(self._raw_cmd_msg.twist.angular.z) if self._raw_cmd_msg is not None else 0.0
        final_linear_x = float(self._final_cmd_msg.velocity.x) if self._final_cmd_msg is not None else 0.0
        final_angular_z = float(self._final_cmd_msg.yaw_rate) if self._final_cmd_msg is not None else 0.0

        neighbors = []
        for state in self._neighbor_states.values():
            rel_x = state.x - own_x
            rel_y = state.y - own_y
            rel_vx = state.vx - own_vx
            rel_vy = state.vy - own_vy
            distance = math.hypot(rel_x, rel_y)
            bearing = math.atan2(rel_y, rel_x) - yaw
            while bearing > math.pi:
                bearing -= 2.0 * math.pi
            while bearing < -math.pi:
                bearing += 2.0 * math.pi
            rel_speed_sq = (rel_vx * rel_vx) + (rel_vy * rel_vy)
            closing_rate = -((rel_x * rel_vx) + (rel_y * rel_vy)) / max(distance, 1e-3)
            # Smoothly blend out wave-jitter pseudo-TCPA when closing_rate < 0.045 m/s so a
            # parallel/escorting hull at 6..12m NEVER flips tcpa_norm 1.0 <-> 0.009 on 1 mm/s GPS noise!
            if rel_speed_sq > 1e-6 and closing_rate > 0.005:
                tcpa_seconds = -((rel_x * rel_vx) + (rel_y * rel_vy)) / rel_speed_sq
                cpa_x = rel_x + rel_vx * tcpa_seconds
                cpa_y = rel_y + rel_vy * tcpa_seconds
                dcpa_raw = math.hypot(cpa_x, cpa_y)
                tcpa_raw = max(0.0, min(1.0, tcpa_seconds / 20.0))
                t_blend = min(1.0, max(0.0, (closing_rate - 0.005) / 0.040))
                w_close = t_blend * t_blend * (3.0 - 2.0 * t_blend)
                tcpa_norm = (1.0 - w_close) * 1.0 + w_close * tcpa_raw
                dcpa = (1.0 - w_close) * distance + w_close * dcpa_raw
            else:
                tcpa_norm = 1.0
                dcpa = distance
            dcpa_norm = max(0.0, min(1.0, dcpa / 8.0))
            # Per-neighbor COLREGS classification — identical convention to
            # multi_agent_bridge._build_neighbor_observation. fresh628: rel_*
            # are WORLD-frame; pass own yaw for in-classifier body rotation and
            # ids for the deterministic strict-complement role tie-break.
            encounter_type_index, role_index = classify_encounter_role(
                body_x=rel_x,
                body_y=rel_y,
                body_vx=rel_vx,
                body_vy=rel_vy,
                distance=distance,
                own_speed=speed,
                ego_id=self._usv_id,
                neighbor_id=state.usv_id,
                own_yaw=yaw,
            )
            # fresh628 ROLE LATCH (SITL 2026-07-12 102710: usv_02/03 closed to
            # 0.16m at the final vertex): once both boats slowed for mutual
            # avoidance their speeds dropped below the classifier's motion
            # thresholds, the roles flickered 2->1->-1 and BOTH kept creeping
            # toward the shared waypoint -- the mutual-yield deadlock ended in
            # near-contact. Fix: once a role is assigned inside the latch
            # radius, KEEP it for that neighbour until separation exceeds the
            # release radius (hysteresis), regardless of the noisy per-frame
            # geometry. The engaged pair therefore keeps a stable
            # master/slave split through the whole close-quarters manoeuvre.
            latch_key = str(state.usv_id)
            if distance < self._role_latch_radius:
                if role_index >= 0:
                    self._role_latch[latch_key] = (encounter_type_index, role_index)
                elif latch_key in self._role_latch:
                    encounter_type_index, role_index = self._role_latch[latch_key]
            elif distance > self._role_latch_release and latch_key in self._role_latch:
                del self._role_latch[latch_key]
            neighbors.append(
                NeighborObservation(
                    usv_id=state.usv_id,
                    rel_x=rel_x,
                    rel_y=rel_y,
                    rel_vx=rel_vx,
                    rel_vy=rel_vy,
                    distance=distance,
                    bearing=bearing,
                    tcpa=tcpa_norm,
                    dcpa=dcpa_norm,
                    encounter_type_index=encounter_type_index,
                    role_index=role_index,
                )
            )

        # Compute route/timing features from spawn→goal line.
        cte = 0.0
        raw_cte = 0.0
        cte_overflow = 0.0
        route_progress = 0.0
        conflict_phase = 0.0
        conflict_eta = 1.0
        crossing_priority = 0.0
        crossing_eta_gap = 1.0
        obs_pose_x = own_x
        obs_pose_y = own_y
        obs_distance_to_goal = float(self._feedback_msg.distance_to_goal)
        obs_heading_error = float(self._feedback_msg.heading_error)
        self._raw_route_inputs = {
            'pose_x': own_x, 'pose_y': own_y, 'yaw': yaw,
            'feedback_distance': obs_distance_to_goal, 'feedback_heading_error': obs_heading_error,
        }
        if self._route_start is not None and self._route_goal is not None:
            sx, sy = self._route_start
            gx, gy = self._route_goal
            route_dx = gx - sx
            route_dy = gy - sy
            route_len = math.hypot(route_dx, route_dy)
            if route_len > 1e-6:
                unit_x = route_dx / route_len
                unit_y = route_dy / route_len
                rel_x = own_x - sx
                rel_y = own_y - sy
                progress_s = rel_x * unit_x + rel_y * unit_y
                raw_cte = (rel_x * route_dy - rel_y * route_dx) / route_len
                cte = max(-3.0, min(3.0, raw_cte))
                cte_overflow = max(0.0, abs(raw_cte) - 3.0)
                route_progress = max(0.0, min(1.0, progress_s / route_len))
                if neighbors:
                    conflict_s = route_len * 0.5
                    to_conflict = conflict_s - progress_s
                    conflict_phase = max(-1.0, min(1.0, (progress_s - conflict_s) / max(0.5 * route_len, 1e-3)))
                    along_speed = max(0.0, speed * math.cos(float(self._feedback_msg.heading_error)), final_linear_x)
                    eta_seconds = max(0.0, to_conflict) / max(along_speed, 0.03)
                    conflict_eta = max(0.0, min(1.0, eta_seconds / 25.0))
                    crossing_priority = {'usv_03': 1.0, 'usv_02': 0.0, 'usv_01': -1.0}.get(self._usv_id, 0.0)

                if self._virtual_goal_active:
                    # Virtual sub-goal: a point on the route line at most
                    # _VIRTUAL_LEG_LENGTH ahead of the ego projection.  Both
                    # distance_to_goal and heading_error are recomputed toward
                    # it with the exact velocity_controller feedback formulas,
                    # and the ego pose is expressed in a leg-local frame whose
                    # origin sits half a leg behind the virtual goal — keeping
                    # pose/d2g inside the training distribution.  CTE/route
                    # features are unchanged (virtual goals lie on the route).
                    s_v = max(0.0, min(route_len, progress_s + _VIRTUAL_LEG_LENGTH))
                    vg_x = sx + unit_x * s_v
                    vg_y = sy + unit_y * s_v
                    obs_distance_to_goal = math.hypot(vg_x - own_x, vg_y - own_y)
                    obs_heading_error = self._wrap_angle(
                        math.atan2(vg_y - own_y, vg_x - own_x) - yaw
                    )
                    o_s = s_v - 0.5 * _VIRTUAL_LEG_LENGTH
                    obs_pose_x = own_x - (sx + unit_x * o_s)
                    obs_pose_y = own_y - (sy + unit_y * o_s)

        return UsvObservation(
            pose_x=obs_pose_x,
            pose_y=obs_pose_y,
            yaw=yaw,
            speed=speed,
            distance_to_goal=obs_distance_to_goal,
            heading_error=obs_heading_error,
            raw_linear_x=raw_linear_x,
            raw_angular_z=raw_angular_z,
            final_linear_x=final_linear_x,
            final_angular_z=final_angular_z,
            cross_track_error=cte,
            raw_cross_track_error=raw_cte,
            cross_track_overflow=cte_overflow,
            route_progress=route_progress,
            conflict_phase=conflict_phase,
            conflict_eta=conflict_eta,
            crossing_priority=crossing_priority,
            crossing_eta_gap=crossing_eta_gap,
            neighbors=neighbors,
        )

    # ------------------------------------------------------------------
    #  Auto encounter-type classifier (COLREGS sectors + hysteresis)
    # ------------------------------------------------------------------

    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _classify_encounter_type(self, obs: 'UsvObservation') -> int:
        """Return encounter-type index (0=head_on, 1=crossing, 2=overtaking)
        for the closest neighbour, with hysteresis to avoid jitter.
        Returns -1 when no neighbour is within detection range."""

        raw_idx = self._classify_encounter_raw(obs)
        now = time.monotonic()

        # Hysteresis: hold the current classification until a *different*
        # candidate persists for _ENCOUNTER_HOLD_TIME seconds.
        if raw_idx == self._auto_encounter_current:
            self._auto_encounter_candidate = -1
            self._auto_encounter_candidate_since = 0.0
            return self._auto_encounter_current

        # First classification or no neighbour → accept immediately
        if self._auto_encounter_current < 0:
            self._auto_encounter_current = raw_idx
            return raw_idx

        # raw_idx differs — start or continue candidate timer
        if raw_idx != self._auto_encounter_candidate:
            self._auto_encounter_candidate = raw_idx
            self._auto_encounter_candidate_since = now
            return self._auto_encounter_current

        if now - self._auto_encounter_candidate_since >= _ENCOUNTER_HOLD_TIME:
            self._auto_encounter_current = raw_idx
            self._auto_encounter_candidate = -1
            self._auto_encounter_candidate_since = 0.0
        return self._auto_encounter_current

    def _classify_encounter_raw(self, obs: 'UsvObservation') -> int:
        """Instantaneous COLREGS-based classification without hysteresis."""
        if not obs.neighbors:
            return -1

        closest = min(obs.neighbors, key=lambda n: n.distance)
        if closest.distance > _ENCOUNTER_DETECT_DISTANCE:
            return -1

        # Closing-speed gate: if the pair is not actually converging there is
        # no encounter — e.g. matched-speed convoy following or diverging
        # tracks.  Prevents the one-hot from staying latched during transit.
        closing_speed = -(
            (closest.rel_x * closest.rel_vx) + (closest.rel_y * closest.rel_vy)
        ) / max(closest.distance, 1e-3)
        if closing_speed < _ENCOUNTER_CLOSING_MIN:
            return -1

        bearing = closest.bearing  # relative to own heading, [-pi, pi]

        # Reconstruct absolute velocities of neighbour
        own_vx = obs.speed * math.cos(obs.yaw)
        own_vy = obs.speed * math.sin(obs.yaw)
        nei_vx = own_vx + closest.rel_vx
        nei_vy = own_vy + closest.rel_vy

        # Courses
        own_course = obs.yaw if obs.speed < _ENCOUNTER_SPEED_THRESHOLD else math.atan2(own_vy, own_vx)
        nei_speed = math.hypot(nei_vx, nei_vy)
        abs_bearing = abs(bearing)
        if nei_speed <= _ENCOUNTER_SPEED_THRESHOLD:
            # Slow/stationary neighbour: course is unreliable.  A slow vessel
            # ahead that we are closing on is an overtaking situation (Rule
            # 13), NOT head-on.  The previous worst-case head-on assumption
            # caused permanent mutual-avoidance deadlocks in convoy following.
            if abs_bearing < math.radians(_OVERTAKING_ASTERN_DEG):
                return 2  # overtaking — ego keeps clear of the slow vessel
            return -1  # slow vessel astern — no encounter
        nei_course = math.atan2(nei_vy, nei_vx)

        rel_course = abs(self._wrap_angle(nei_course - own_course))

        # Astern sector: neighbour is behind us
        if abs_bearing > math.radians(_OVERTAKING_ASTERN_DEG):
            return 2  # overtaking

        # Ahead sector: distinguish head-on vs overtaking
        if abs_bearing < math.radians(_HEAD_ON_BEARING_DEG):
            if rel_course > math.radians(_HEAD_ON_COURSE_DEG):
                return 0  # head_on — nearly reciprocal courses
            if rel_course < math.radians(_OVERTAKING_COURSE_DEG):
                return 2  # overtaking — similar courses
            return 1  # crossing — intermediate angles

        # Side sector (22.5°–112.5°): crossing
        return 1  # crossing

    def _neighbor_conflict(self, observation) -> bool:
        """Predicted conflict (TCPA <= 12 s and DCPA < 1.6 m, or closing within 2.5 m), latched until past CPA and opening."""
        seen = set()
        for nb in observation.neighbors:
            nb_id = str(getattr(nb, 'usv_id', '') or getattr(nb, 'source_id', ''))
            seen.add(nb_id)
            d = float(nb.distance)
            if d <= 1e-6:
                continue
            rr = (nb.rel_x * nb.rel_vx + nb.rel_y * nb.rel_vy) / max(d, 1e-3)
            v_sq = nb.rel_vx * nb.rel_vx + nb.rel_vy * nb.rel_vy
            tcpa = -(nb.rel_x * nb.rel_vx + nb.rel_y * nb.rel_vy) / v_sq if v_sq >= 1e-6 else 0.0
            dcpa = math.hypot(nb.rel_x + nb.rel_vx * tcpa, nb.rel_y + nb.rel_vy * tcpa) if tcpa > 0 else d
            threat = d <= 7.8 and ((d < 2.5 and rr < -0.05) or (0.0 < tcpa <= 12.0 and dcpa < 1.6))
            if threat:
                self._conflict_latched.add(nb_id)
            elif nb_id in self._conflict_latched and (d > 8.4 or (tcpa <= 0.0 and rr > 0.02)):
                self._conflict_latched.discard(nb_id)
        self._conflict_latched &= seen
        return bool(self._conflict_latched)

    def _publish_action(self):
        self._maybe_enable_controller_param()
        navigation_status = self._navigation_input_status()
        if navigation_status['reason'] is not None:
            if self._active_goal_id is not None or self._navigation_terminal_reason is not None:
                self._publish_navigation_stop(navigation_status)
            self._maybe_log_waiting_for_observation()
            return
        if self._navigation_guard_reason is not None:
            self.get_logger().info('Navigation inputs fresh; resuming policy inference')
            self._navigation_guard_reason = None
        observation = self._build_observation()
        if observation is None:
            self._publish_navigation_stop({**navigation_status, 'reason': 'observation_missing'})
            self._maybe_log_waiting_for_observation()
            return

        if not self._ready_logged:
            self.get_logger().info('Observation stream ready; pure RL policy inference is active.')
            self._ready_logged = True

        enc_idx = -1
        if self._encounter_type_enabled:
            enc_idx = self._encounter_type_index
            if self._encounter_type_auto:
                enc_idx = self._classify_encounter_type(observation)
        # to_vector() now appends the trailing agent-level encounter one-hot
        # itself (full parity with multi_agent_types), so set the index on the
        # observation rather than concatenating a separate one-hot.
        observation.encounter_type_index = enc_idx
        observation_vector = observation.to_vector(self._max_neighbors)
        if getattr(self._policy, 'neutral_conflict_features', False):
            observation_vector = replace(
                observation, conflict_phase=0.0, conflict_eta=1.0, crossing_priority=0.0,
                crossing_eta_gap=1.0, encounter_type_index=-1).to_vector(self._max_neighbors)

        # Publish encounter-type index for downstream logging.
        enc_msg = Int8()
        enc_msg.data = int(enc_idx)
        self._encounter_type_pub.publish(enc_msg)

        action_policy = self._policy
        policy_expert = 'base'
        canonical_ready = (
            self._route_start is not None and self._route_goal is not None
            and math.hypot(self._route_goal[0] - self._route_start[0], self._route_goal[1] - self._route_start[1]) > 1e-6
        )
        if self._solo_expert is not None and not self._neighbor_conflict(observation) and (
                self._solo_expert.observation_frame == LEGACY_FRAME or canonical_ready):
            action_policy = self._solo_expert
            policy_expert = 'solo'
            if self._solo_expert.observation_frame == ROUTE_CANONICAL_FRAME:
                # The whole post-processing chain then sees the same window the expert was trained on.
                observation = replace(observation, **canonical_route_fields(
                    **self._raw_route_inputs, route_start=self._route_start, route_goal=self._route_goal))
            # The expert only saw open water; non-conflicting traffic stays visible to post-processing only.
            observation_vector = replace(
                observation, neighbors=[], conflict_phase=0.0, conflict_eta=1.0, crossing_priority=0.0,
                crossing_eta_gap=1.0, encounter_type_index=-1).to_vector(self._max_neighbors)
        action = np.asarray(action_policy.predict(observation_vector), dtype=np.float32).reshape(-1)
        if action.size != 2:
            raise RuntimeError(f'Pure RL mode requires 2D action output, got {action.size}.')
        projected_action = project_rl_policy_action(
            action,
            rl_control_mode='pure',
            action_mode='full',
            linear_delta_limit=self._action_bounds.linear_delta,
            angular_delta_limit=self._action_bounds.angular_delta,
            raw_linear_x=observation.raw_linear_x,
            heading_error=observation.heading_error,
            current_angular_z=observation.final_angular_z,
            control_dt=1.0 / max(1.0, float(self._publish_rate)),
            conflict_level=self._projection_conflict_level(observation),
            heading_omega_deadband=self._policy.heading_omega_deadband,
            heading_omega_reference=self._policy.heading_omega_reference,
            angular_authority_power=self._policy.angular_authority_power,
            angular_accel_limit=self._policy.angular_accel_limit,
            angular_decel_limit=self._policy.angular_decel_limit,
            conflict_turn_relief=self._policy.conflict_turn_relief,
            authority_floor=(0.35 if getattr(self._policy, 'checkpoint_angular_authority_floor', None) is None
                             else float(self._policy.checkpoint_angular_authority_floor)),
            forward_only=True,
        )
        expanded_action = projected_action
        if isinstance(self._policy, MappoActorPolicyRuntime):
            expanded_action = self._policy.action_contract.expand(
                action, projected_action, observation.distance_to_goal,
                cruise_speed_override=self._dynamic_cruise_speed,
            )
        linear_x = float(expanded_action[0])
        angular_z = float(expanded_action[1])
        model_linear_x, model_angular_z = linear_x, angular_z

        # Veteran-driver turn governor: speed cap as a function of heading
        # error (see __init__ comment). Applied FIRST so neighbour-distance
        # scaling can only lower it further.
        if self._turn_governor_enabled:
            cap = turn_governor_speed_cap(
                observation.heading_error, self._turn_governor_cruise,
                self._turn_governor_floor, self._turn_governor_min_speed,
            )
            if linear_x > cap:
                linear_x = cap

        # Short-side turn assist: force minimum-heading-error turn direction
        # when far off-heading and clear of traffic (see __init__ comment).
        if self._turn_assist_enabled:
            he = float(observation.heading_error)
            he_abs = abs(he)
            clear_of_traffic = observation.min_neighbor_distance() > self._turn_assist_min_separation
            if self._turn_assist_active:
                if he_abs < self._turn_assist_release or not clear_of_traffic:
                    self._turn_assist_active = False
            elif he_abs > self._turn_assist_threshold and clear_of_traffic:
                self._turn_assist_active = True
            if self._turn_assist_active:
                desired_omega = max(-1.0, min(1.0, he / max(self._turn_assist_release, 1e-3))) * self._turn_assist_max_omega
                angular_z = (
                    (1.0 - self._turn_assist_blend) * angular_z
                    + self._turn_assist_blend * desired_omega
                )

        # Distance-aware speed scaling: reduce linear speed when a neighbour
        # is dangerously close, preventing high-speed collisions.
        # fresh628 CONVOY-LOCK FIX (SITL 2026-07-12 100541: usv_01/usv_02 locked
        # at 1.0m for 40s+, BOTH crawling at ~0.12 m/s and drifting off-route
        # together). Parallel-track proximity classifies as ENC_NONE (no COLREGS
        # role), and the old SYMMETRIC governor slowed both boats identically —
        # a stable lock. Now the scaling is ROLE-AWARE: the stand-on boat (or,
        # when no role exists, the deterministic smaller-ID master) keeps a
        # 0.75 floor and sails through, while the give-way boat keeps the old
        # 0.35 floor and falls behind — separation grows and the lock breaks.
        min_neighbor_dist = observation.min_neighbor_distance()
        if min_neighbor_dist < _SPEED_SCALE_DISTANCE:
            is_master = False
            nearest = None
            if observation.neighbors:
                nearest = min(observation.neighbors, key=lambda n: n.distance)
            if nearest is not None:
                role = int(getattr(nearest, 'role_index', -1))
                if role == 1:
                    is_master = True
                elif role == 0:
                    is_master = False
                else:
                    nb_id = str(getattr(nearest, 'usv_id', '') or '')
                    is_master = bool(nb_id) and str(self._usv_id) < nb_id
            scale_floor = 0.75 if is_master else _SPEED_SCALE_MIN
            speed_scale = max(
                scale_floor,
                min_neighbor_dist / _SPEED_SCALE_DISTANCE,
            )
            linear_x *= speed_scale
            # Master push-through: the stand-on boat must actually CLEAR the
            # geometry, not creep alongside the yielding boat (SITL 102710:
            # both crept at 0.05-0.17 into a 0.16m near-contact). While the
            # role is engaged and the hulls are not touching, hold the master
            # at steerage-plus speed so the pair separates quickly.
            if is_master and min_neighbor_dist > 0.45:
                linear_x = max(linear_x, 0.18)
                # fresh629 ESCORT-LOCK FIX (SITL 2026-07-14 113839 t=308-340:
                # usv_02 pushed through at 0.18 but steered PARALLEL to the
                # yielding usv_03 for 32s, sailing AWAY from its own goal --
                # d2g grew 3.7->7.6m at a stable 1.0m separation). Speed alone
                # does not disengage the pair: while pushing through, also
                # blend omega toward the goal heading so the master's track
                # actually diverges from the escort course. Omega has the SAME
                # sign as heading_error to converge (see turn governor note).
                # fresh630 tune (SITL 2026-07-15 175946): 0.5 blend lost to the
                # RL orbit output (|omega| 0.5-0.6) -> raised to 0.85 and the
                # engage threshold lowered 0.35->0.20 rad.
                he = float(observation.heading_error)
                if abs(he) > 0.20:
                    goal_omega = max(-1.0, min(1.0, he / 0.5)) * 0.45
                    angular_z = 0.15 * angular_z + 0.85 * goal_omega

            # fresh629 DUAL-GIVE-WAY DEADLOCK BREAKER (SITL 2026-07-14 104106:
            # usv_01/usv_02 both classified crossing but BOTH latched role 0 =
            # give-way; the role latch kept the symmetric split until the 3.5m
            # release so the push-through above never fired, and both boats
            # crept at the 0.05 floor for 35s down to 0.15m separation). If we
            # stay close and slow for >3s without being master, fall back to
            # the deterministic smaller-ID election and push through anyway;
            # the higher-ID boat keeps yielding, so the pair separates.
            now_mono = time.monotonic()
            if min_neighbor_dist < 2.0 and linear_x < 0.12:
                if self._yield_stall_since is None:
                    self._yield_stall_since = now_mono
            else:
                self._yield_stall_since = None
                self._yield_stall_logged = False
            if (
                not is_master
                and nearest is not None
                and self._yield_stall_since is not None
                and (now_mono - self._yield_stall_since) > 3.0
                and min_neighbor_dist > 0.45
            ):
                nb_id = str(getattr(nearest, 'usv_id', '') or '')
                if nb_id and str(self._usv_id) < nb_id:
                    linear_x = max(linear_x, 0.22)
                    # Same escort-lock fix as the master push-through above:
                    # steer toward the goal while pushing through, otherwise
                    # the elected master just escorts the yielding boat.
                    # fresh630: blend 0.5->0.85, threshold 0.35->0.20 rad.
                    he = float(observation.heading_error)
                    if abs(he) > 0.20:
                        goal_omega = max(-1.0, min(1.0, he / 0.5)) * 0.45
                        angular_z = 0.15 * angular_z + 0.85 * goal_omega
                    if not self._yield_stall_logged:
                        self.get_logger().warning(
                            f'Dual-give-way stall detected (sep={min_neighbor_dist:.2f}m, '
                            f'>3s at creep speed): ID election overrides latched role, '
                            f'{self._usv_id} pushes through past {nb_id}.'
                        )
                        self._yield_stall_logged = True

            # fresh630 ORBIT-LOCK BREAKER (SITL 2026-07-15 175946 t=130-250):
            # two boats in a rigid binary orbit -- separation FROZEN at
            # 0.42+/-0.02m for 120s, opposite near-constant omegas, both
            # creeping. The dual-give-way breaker fired (usv_02 reached
            # 0.20-0.25 at t=190-206) but the pair never separated because
            # both hulls kept rotating around their common center. Detection
            # here is SEPARATION-RATE based, not speed based: close (<0.9m)
            # AND |d(sep)/dt| < 0.04 m/s sustained >4s = orbit lock.
            # Symmetry break by deterministic ID election:
            #   smaller ID  -> full goal steering (blend 0.9) + 0.25 m/s out
            #   larger ID   -> omega damped x0.2, hold 0.06 m/s (stop circling)
            now_mono2 = time.monotonic()
            if min_neighbor_dist < 0.9 and nearest is not None:
                prev = self._orbit_lock_prev_sep
                self._orbit_lock_prev_sep = (now_mono2, min_neighbor_dist)
                sep_rate = None
                if prev is not None and now_mono2 - prev[0] > 1e-3:
                    sep_rate = abs(min_neighbor_dist - prev[1]) / (now_mono2 - prev[0])
                if sep_rate is not None and sep_rate < 0.04:
                    if self._orbit_lock_since is None:
                        self._orbit_lock_since = now_mono2
                else:
                    self._orbit_lock_since = None
                if (
                    self._orbit_lock_since is not None
                    and (now_mono2 - self._orbit_lock_since) > 4.0
                ):
                    nb_id2 = str(getattr(nearest, 'usv_id', '') or '')
                    he2 = float(observation.heading_error)
                    if nb_id2 and str(self._usv_id) < nb_id2:
                        # Elected master: hard goal steering, push straight out.
                        goal_omega2 = max(-1.0, min(1.0, he2 / 0.5)) * 0.60
                        angular_z = 0.10 * angular_z + 0.90 * goal_omega2
                        linear_x = max(linear_x, 0.25)
                    else:
                        # Elected slave: stop feeding the orbit -- damp the turn
                        # and hold nearly still until the master clears out.
                        angular_z *= 0.2
                        linear_x = min(linear_x, 0.06)
                    if not self._orbit_lock_logged:
                        self.get_logger().warning(
                            f'Orbit lock detected (sep={min_neighbor_dist:.2f}m frozen '
                            f'>4s): {self._usv_id} '
                            f'{"pushes out to goal" if nb_id2 and str(self._usv_id) < nb_id2 else "damps turn and waits"} '
                            f'(peer={nb_id2}).'
                        )
                        self._orbit_lock_logged = True
            else:
                self._orbit_lock_since = None
                self._orbit_lock_prev_sep = None
                self._orbit_lock_logged = False

        if self._clear_ahead_route_gate_active(observation):
            raw_linear = max(self._clear_ahead_route_min_speed, float(observation.raw_linear_x))
            raw_linear = min(raw_linear, float(self._action_bounds.linear_delta))
            if abs(float(observation.raw_angular_z)) > 1e-4:
                route_omega = float(observation.raw_angular_z)
            else:
                # SIGN FIX (SITL 2026-07-14 135017 usv_02 RUNAWAY): omega must be
                # the SAME sign as heading_error to converge (he_dot = -omega_yaw,
                # see turn governor note). The old negative sign made he=180deg a
                # STABLE equilibrium: when the raw command source died (zeros from
                # t=483s) this fallback drove the boat straight AWAY from its goal
                # for 330s at 0.16 m/s until 37m off-field.
                route_omega = max(-1.0, min(1.0, float(observation.heading_error) / self._clear_ahead_route_heading_reference)) * self._clear_ahead_route_max_omega
            route_omega = max(-self._clear_ahead_route_max_omega, min(self._clear_ahead_route_max_omega, route_omega))
            linear_x = (
                (1.0 - self._clear_ahead_route_linear_blend) * linear_x
                + self._clear_ahead_route_linear_blend * raw_linear
            )
            angular_z = (
                (1.0 - self._clear_ahead_route_omega_blend) * angular_z
                + self._clear_ahead_route_omega_blend * route_omega
            )
            if not self._clear_ahead_route_logged:
                self.get_logger().info(
                    'Online clear-ahead route gate active: blending pure RL output toward raw route command.'
                )
                self._clear_ahead_route_logged = True

        # Static blockers still need a decisive pass, but moving encounters
        # use a smooth DCPA-based correction.  This avoids pinning ordinary
        # crossings at one yaw rate when their natural timing already yields
        # sufficient separation.  Evaluate every give-way neighbour rather
        # than letting a closer holding vessel hide an approaching moving pair.
        give_way_guard_active = False
        give_way_guard_neighbor = None
        give_way_turn_floor = 0.0
        keep_static_steerage = False
        for neighbor in observation.neighbors:
            turn_floor, keep_steerage = _give_way_guard_demand(observation, neighbor)
            if turn_floor > give_way_turn_floor:
                give_way_turn_floor = turn_floor
                give_way_guard_neighbor = neighbor
            keep_static_steerage = keep_static_steerage or keep_steerage

        if give_way_turn_floor > 0.0:
            angular_z = min(angular_z, -give_way_turn_floor)
            if give_way_guard_neighbor.distance > _GIVE_WAY_STEERAGE_MIN_DISTANCE:
                linear_x = max(linear_x, _GIVE_WAY_MOVING_STEERAGE_SPEED)
            give_way_guard_active = True
        if keep_static_steerage:
            linear_x = max(linear_x, _GIVE_WAY_STEERAGE_SPEED)
            give_way_guard_active = True

        if give_way_guard_active and not self._give_way_guard_logged:
            self.get_logger().info(
                f'Predictive give-way guard active for {give_way_guard_neighbor.usv_id}: '
                f'separation={give_way_guard_neighbor.distance:.2f}m, '
                f'turn_floor={give_way_turn_floor:.3f}rad/s.'
            )
            self._give_way_guard_logged = True
        elif not give_way_guard_active:
            self._give_way_guard_logged = False

        if not self._safety_layers_enabled:
            linear_x, angular_z = model_linear_x, model_angular_z
            give_way_guard_active = False
            self._turn_assist_active = False

        if not self._first_action_logged:
            self.get_logger().info(
                f'Publishing first pure RL action: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}'
            )
            self._first_action_logged = True

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self._action_pub.publish(msg)
        if self._policy_trace is not None and self._policy_trace.enabled:
            nearest = observation.min_neighbor_distance()
            self._policy_trace.write({
                'type': 'action',
                'timestamp': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
                'wall_timestamp': time.time(),
                'navigation_input_status': navigation_status,
                'telemetry_status': self._telemetry_status,
                'telemetry_status_age_s': (
                    None if self._telemetry_status_received is None
                    else time.monotonic() - self._telemetry_status_received
                ),
                'goal_id': self._active_goal_id,
                'route_start': self._route_start,
                'route_goal': self._route_goal,
                'virtual_goal_active': self._virtual_goal_active,
                'heading_error_rad': float(observation.heading_error),
                'cross_track_error': float(observation.cross_track_error),
                'raw_cross_track_error': float(observation.raw_cross_track_error),
                'distance_to_goal': float(observation.distance_to_goal),
                'feedback_heading_error_rad': float(self._feedback_msg.heading_error),
                'feedback_distance_to_goal': float(self._feedback_msg.distance_to_goal),
                'observation_vector': observation_vector.tolist(),
                'policy_expert': policy_expert,
                'actor_action': action.tolist(),
                'projected_action': projected_action.tolist(),
                'expanded_action': expanded_action.tolist(),
                'published_action': [linear_x, angular_z],
                'previous_final_action': [float(observation.final_linear_x), float(observation.final_angular_z)],
                'turn_assist_active': self._turn_assist_enabled and self._turn_assist_active,
                'turn_governor_enabled': self._turn_governor_enabled,
                'give_way_guard_active': give_way_guard_active,
                'nearest_neighbor_distance': float(nearest) if math.isfinite(nearest) else None,
                'projection_conflict_level': self._projection_conflict_level(observation),
            })


def main(argv=None):
    cli_args = rclpy.utilities.remove_ros_args(args=argv)
    args = parse_args(cli_args[1:])
    rclpy.init(args=argv)
    node = None
    try:
        node = PolicyInferenceNode(
            namespace=args.namespace,
            model_path=args.model,
            policy_kind=args.policy,
            rl_control_mode=args.rl_control_mode,
            device=args.device,
            publish_rate=args.publish_rate,
            max_neighbors=args.max_neighbors,
            enable_controller_param=not args.disable_controller_param,
            encounter_type=args.encounter_type,
            clear_ahead_route_gate=args.clear_ahead_route_gate,
            clear_ahead_route_distance=args.clear_ahead_route_distance,
            clear_ahead_route_bearing_deg=args.clear_ahead_route_bearing_deg,
            clear_ahead_route_min_neighbor_separation=args.clear_ahead_route_min_neighbor_separation,
            clear_ahead_route_max_conflict_level=args.clear_ahead_route_max_conflict_level,
            clear_ahead_route_linear_blend=args.clear_ahead_route_linear_blend,
            clear_ahead_route_omega_blend=args.clear_ahead_route_omega_blend,
            clear_ahead_route_min_speed=args.clear_ahead_route_min_speed,
            clear_ahead_route_max_omega=args.clear_ahead_route_max_omega,
            clear_ahead_route_heading_reference=args.clear_ahead_route_heading_reference,
            turn_governor_enabled=not args.disable_turn_governor,
            turn_governor_floor=args.turn_governor_floor,
            turn_governor_min_speed=args.turn_governor_min_speed,
            turn_governor_cruise=args.turn_governor_cruise,
            cruise_speed_override=args.turn_governor_cruise,
            turn_assist_enabled=not args.disable_turn_assist,
            turn_assist_threshold_deg=args.turn_assist_threshold_deg,
            turn_assist_release_deg=args.turn_assist_release_deg,
            turn_assist_min_separation=args.turn_assist_min_separation,
            turn_assist_max_omega=args.turn_assist_max_omega,
            turn_assist_blend=args.turn_assist_blend,
            safety_layers_enabled=args.enable_safety_layers,
            policy_trace_dir=None if args.disable_policy_trace else args.policy_trace_dir,
            solo_expert_model=args.solo_expert_model,
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()