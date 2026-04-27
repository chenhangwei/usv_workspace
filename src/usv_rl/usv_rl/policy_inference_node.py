import argparse
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
from common_interfaces.msg import FleetNeighborPoses, NavigationFeedback, NavigationGoal
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int8, String

from .action_projection import project_policy_action as project_rl_policy_action
from .config import ActionBounds
from .multi_agent_types import ENCOUNTER_TYPE_COUNT
from .observation_normalizer import ObservationNormalizer
from .policies import load_policy
from .types import NeighborObservation, NeighborState, UsvObservation, USV_NEIGHBOR_FEATURE_COUNT

_ENCOUNTER_TYPE_NAMES = {'head_on': 0, 'crossing': 1, 'overtaking': 2}

# ---------- Auto encounter-type classification constants ----------
_ENCOUNTER_DETECT_DISTANCE = 15.0   # metres – classify only when nearest neighbour is closer
_ENCOUNTER_HOLD_TIME = 2.0          # seconds – minimum time to hold a classification before switching
_ENCOUNTER_SPEED_THRESHOLD = 0.05   # m/s – below this, course is unreliable

# ---------- Distance-aware speed scaling constants ----------
_SPEED_SCALE_DISTANCE = 3.0         # metres – linear speed scaling starts below this distance
_SPEED_SCALE_MIN = 0.35             # minimum speed scale factor at zero distance
_HEAD_ON_BEARING_DEG = 22.5         # |bearing| < this AND reciprocal courses → head_on
_HEAD_ON_COURSE_DEG = 135.0         # relative course > this → reciprocal
_OVERTAKING_COURSE_DEG = 45.0       # relative course < this AND ahead → overtaking
_OVERTAKING_ASTERN_DEG = 112.5      # |bearing| > this → astern sector


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
            from usv_rl.neighbor_attention import build_attention_actor_from_checkpoint
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


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Run a trained policy online and publish to rl_policy/cmd_vel.')
    parser.add_argument('--model', required=True, help='Path to policy model (.npz for BC, .zip for PPO, .pt for MAPPO).')
    parser.add_argument('--policy', choices=['auto', 'bc', 'ppo', 'mappo', 'zero'], default='auto', help='Policy backend.')
    parser.add_argument('--rl-control-mode', choices=['auto', 'pure'], default='pure', help='Deprecated compatibility flag. Only pure final-command policies are supported.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--device', default='cpu', help='Torch device for PPO inference.')
    parser.add_argument('--publish-rate', type=float, default=10.0, help='Policy publish rate in Hz.')
    parser.add_argument('--max-neighbors', type=int, default=3, help='Maximum neighbors encoded into the observation vector.')
    parser.add_argument('--disable-controller-param', action='store_true', help='Do not auto-enable rl_policy_enabled on velocity_controller_node.')
    parser.add_argument('--encounter-type', choices=['auto', 'none', 'head_on', 'crossing', 'overtaking'], default='auto',
                        help='Encounter type for models trained with scenario conditioning (fresh54+). '
                             '"auto" classifies dynamically from neighbour geometry each step; '
                             'a fixed name uses a static one-hot; "none" sends all-zeros.')
    return parser.parse_args(argv)


class PolicyInferenceNode(Node):
    def __init__(self, *, namespace: str, model_path: str, policy_kind: str, rl_control_mode: str, device: str, publish_rate: float, max_neighbors: int, enable_controller_param: bool, encounter_type: str = 'none'):
        resolved_namespace = namespace if namespace.startswith('/') else f'/{namespace}'
        super().__init__('policy_inference_node', namespace=resolved_namespace)
        self._namespace = resolved_namespace
        self._usv_id = resolved_namespace.strip('/')
        self._policy = _load_runtime_policy(model_path, policy_kind, device)

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
        self._first_goal_logged = False
        self._first_active_goal_feedback_logged = False
        self._head_on_guard_logged = False
        self._head_on_guard_active = False
        self._head_on_guard_linear_cap = 0.0
        self._head_on_guard_turn_floor = 0.0
        self._head_on_guard_hold_until = 0.0
        self._head_on_guard_last_update = self._startup_monotonic

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

        self.create_subscription(PoseStamped, 'local_position/pose_from_gps', self._pose_callback, qos_best_effort)
        self.create_subscription(TwistStamped, 'local_position/velocity_local', self._velocity_callback, qos_best_effort)
        self.create_subscription(NavigationGoal, 'set_usv_nav_goal', self._nav_goal_callback, qos_reliable)
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

        self._publish_timer = self.create_timer(1.0 / max(1.0, self._publish_rate), self._publish_action)

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

    def _reset_head_on_guard_state(self):
        self._head_on_guard_logged = False
        self._head_on_guard_active = False
        self._head_on_guard_linear_cap = 0.0
        self._head_on_guard_turn_floor = 0.0
        self._head_on_guard_hold_until = 0.0
        self._head_on_guard_last_update = time.monotonic()

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
        is_new_goal = goal_id != self._active_goal_id
        self._active_goal_id = goal_id

        if not is_new_goal:
            return

        if reset_stream_state:
            self._feedback_msg = None
            self._raw_cmd_msg = None
            self._final_cmd_msg = None

        self._ready_logged = False
        self._first_action_logged = False
        self._first_active_goal_feedback_logged = False
        self._no_neighbor_passthrough_logged = False
        self._last_waiting_signature = None
        self._waiting_repeat_count = 0
        self._reset_head_on_guard_state()

        # Record route start (current position) and goal for CTE computation.
        if target is not None:
            self._route_goal = target
            if self._pose_msg is not None:
                self._route_start = (
                    float(self._pose_msg.pose.position.x),
                    float(self._pose_msg.pose.position.y),
                )
            else:
                self._route_start = None

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

    def _feedback_callback(self, msg: NavigationFeedback):
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

    def _final_cmd_callback(self, msg: PositionTarget):
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

        raw_linear_x = 0.0
        raw_angular_z = 0.0
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
            if rel_speed_sq > 1e-6:
                tcpa_seconds = -((rel_x * rel_vx) + (rel_y * rel_vy)) / rel_speed_sq
            else:
                tcpa_seconds = 1e3
            if tcpa_seconds <= 0.0:
                tcpa_norm = 1.0
                dcpa = distance
            else:
                cpa_x = rel_x + rel_vx * tcpa_seconds
                cpa_y = rel_y + rel_vy * tcpa_seconds
                dcpa = math.hypot(cpa_x, cpa_y)
                tcpa_norm = max(0.0, min(1.0, tcpa_seconds / 20.0))
            dcpa_norm = max(0.0, min(1.0, dcpa / 8.0))
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
                )
            )

        # Compute route/timing features from spawn→goal line.
        cte = 0.0
        route_progress = 0.0
        conflict_phase = 0.0
        conflict_eta = 1.0
        crossing_priority = 0.0
        crossing_eta_gap = 1.0
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
                cte = (rel_x * route_dy - rel_y * route_dx) / route_len
                cte = max(-3.0, min(3.0, cte))
                route_progress = max(0.0, min(1.0, progress_s / route_len))
                conflict_s = route_len * 0.5
                to_conflict = conflict_s - progress_s
                conflict_phase = max(-1.0, min(1.0, (progress_s - conflict_s) / max(0.5 * route_len, 1e-3)))
                along_speed = max(0.0, speed * math.cos(float(self._feedback_msg.heading_error)), final_linear_x)
                eta_seconds = max(0.0, to_conflict) / max(along_speed, 0.03)
                conflict_eta = max(0.0, min(1.0, eta_seconds / 25.0))
                crossing_priority = {'usv_03': 1.0, 'usv_02': 0.0, 'usv_01': -1.0}.get(self._usv_id, 0.0)

        return UsvObservation(
            pose_x=own_x,
            pose_y=own_y,
            yaw=yaw,
            speed=speed,
            distance_to_goal=float(self._feedback_msg.distance_to_goal),
            heading_error=float(self._feedback_msg.heading_error),
            raw_linear_x=raw_linear_x,
            raw_angular_z=raw_angular_z,
            final_linear_x=final_linear_x,
            final_angular_z=final_angular_z,
            cross_track_error=cte,
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

        bearing = closest.bearing  # relative to own heading, [-pi, pi]

        # Reconstruct absolute velocities of neighbour
        own_vx = obs.speed * math.cos(obs.yaw)
        own_vy = obs.speed * math.sin(obs.yaw)
        nei_vx = own_vx + closest.rel_vx
        nei_vy = own_vy + closest.rel_vy

        # Courses
        own_course = obs.yaw if obs.speed < _ENCOUNTER_SPEED_THRESHOLD else math.atan2(own_vy, own_vx)
        nei_speed = math.hypot(nei_vx, nei_vy)
        if nei_speed > _ENCOUNTER_SPEED_THRESHOLD:
            nei_course = math.atan2(nei_vy, nei_vx)
        else:
            # Stationary neighbour — assume course pointing toward us (worst case)
            nei_course = math.atan2(-closest.rel_y, -closest.rel_x)

        rel_course = abs(self._wrap_angle(nei_course - own_course))
        abs_bearing = abs(bearing)

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

    def _publish_action(self):
        self._maybe_enable_controller_param()
        observation = self._build_observation()
        if observation is None:
            self._maybe_log_waiting_for_observation()
            return

        if not self._ready_logged:
            self.get_logger().info('Observation stream ready; pure RL policy inference is active.')
            self._ready_logged = True

        observation_vector = observation.to_vector(self._max_neighbors)

        enc_idx = -1
        if self._encounter_type_enabled:
            enc_idx = self._encounter_type_index
            if self._encounter_type_auto:
                enc_idx = self._classify_encounter_type(observation)
            one_hot = np.zeros(ENCOUNTER_TYPE_COUNT, dtype=np.float32)
            if 0 <= enc_idx < ENCOUNTER_TYPE_COUNT:
                one_hot[enc_idx] = 1.0
            observation_vector = np.concatenate([observation_vector, one_hot])

        # Publish encounter-type index for downstream logging.
        enc_msg = Int8()
        enc_msg.data = int(enc_idx)
        self._encounter_type_pub.publish(enc_msg)

        action = np.asarray(self._policy.predict(observation_vector), dtype=np.float32).reshape(-1)
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
            forward_only=True,
        )
        linear_x = float(projected_action[0])
        angular_z = float(projected_action[1])

        # Distance-aware speed scaling: reduce linear speed when a neighbour
        # is dangerously close, preventing high-speed collisions.
        min_neighbor_dist = observation.min_neighbor_distance()
        if min_neighbor_dist < _SPEED_SCALE_DISTANCE:
            speed_scale = max(
                _SPEED_SCALE_MIN,
                min_neighbor_dist / _SPEED_SCALE_DISTANCE,
            )
            linear_x *= speed_scale

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