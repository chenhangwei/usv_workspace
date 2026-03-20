import argparse
import math
import os
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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .action_projection import project_residual_action
from .config import ActionBounds
from .policies import load_residual_policy
from .types import NeighborObservation, NeighborState, UsvObservation


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


class ZeroPolicy:
    def predict(self, observation: np.ndarray) -> np.ndarray:
        return np.zeros(2, dtype=np.float32)


class PpoResidualPolicyRuntime:
    def __init__(self, model_path: str, device: str = 'cpu'):
        try:
            from stable_baselines3 import PPO
        except ImportError as exc:
            raise RuntimeError(
                'stable_baselines3 is required to load PPO residual policies for online inference.'
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
                'torch is required to load MAPPO residual policies for online inference.'
            ) from exc

        checkpoint = torch.load(model_path, map_location=device, weights_only=False)
        hidden_sizes = tuple(int(value) for value in checkpoint.get('hidden_sizes', [128, 128]))
        self.obs_dim = int(checkpoint['local_observation_size'])
        self.action_dim = int(checkpoint['action_dim'])
        self._torch = torch
        self._device = torch.device(device)

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

    def predict(self, observation: np.ndarray) -> np.ndarray:
        obs_tensor = self._torch.as_tensor(
            observation,
            dtype=self._torch.float32,
            device=self._device,
        ).unsqueeze(0)
        with self._torch.no_grad():
            action = self._actor(obs_tensor).cpu().numpy()[0]
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
        return load_residual_policy(resolved_model_path)
    if inferred_kind == 'ppo':
        return PpoResidualPolicyRuntime(resolved_model_path, device=device)
    if inferred_kind == 'mappo':
        return MappoActorPolicyRuntime(resolved_model_path, device=device)
    raise RuntimeError(f'Unsupported online policy type: {inferred_kind}')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Run a trained residual policy online and publish to rl_policy/cmd_vel.')
    parser.add_argument('--model', required=True, help='Path to residual policy model (.npz for BC, .zip for PPO, .pt for MAPPO).')
    parser.add_argument('--policy', choices=['auto', 'bc', 'ppo', 'mappo', 'zero'], default='auto', help='Policy backend.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--device', default='cpu', help='Torch device for PPO inference.')
    parser.add_argument('--publish-rate', type=float, default=10.0, help='Residual action publish rate in Hz.')
    parser.add_argument('--max-neighbors', type=int, default=3, help='Maximum neighbors encoded into the observation vector.')
    parser.add_argument('--disable-controller-param', action='store_true', help='Do not auto-enable rl_policy_enabled on velocity_controller_node.')
    return parser.parse_args(argv)


class ResidualPolicyInferenceNode(Node):
    def __init__(self, *, namespace: str, model_path: str, policy_kind: str, device: str, publish_rate: float, max_neighbors: int, enable_controller_param: bool):
        resolved_namespace = namespace if namespace.startswith('/') else f'/{namespace}'
        super().__init__('residual_policy_node', namespace=resolved_namespace)
        self._namespace = resolved_namespace
        self._usv_id = resolved_namespace.strip('/')
        self._policy = _load_runtime_policy(model_path, policy_kind, device)
        self._max_neighbors = max(1, max_neighbors)
        self._action_bounds = ActionBounds()
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

        if policy_obs_dim is not None:
            if policy_obs_dim >= 10 and (policy_obs_dim - 10) % 6 == 0:
                inferred_neighbors = max(1, (policy_obs_dim - 10) // 6)
                if inferred_neighbors != self._max_neighbors:
                    self.get_logger().info(
                        f'Overriding max_neighbors from {self._max_neighbors} to {inferred_neighbors} based on model observation dimension {policy_obs_dim}.'
                    )
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
        self._publish_timer = self.create_timer(1.0 / max(1.0, publish_rate), self._publish_action)

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
        if self._raw_cmd_msg is None:
            missing_inputs.append('raw_cmd')
        if self._final_cmd_msg is None:
            missing_inputs.append('final_cmd')

        waiting_signature = (
            tuple(missing_inputs),
            len(self._neighbor_states),
            self._active_goal_id,
            self._raw_cmd_msg is not None,
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
            f'raw_cmd={self._raw_cmd_msg is not None}, '
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
            Parameter('rl_policy_use_residual', value=True),
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
            or self._raw_cmd_msg is None
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
            neighbors.append(
                NeighborObservation(
                    usv_id=state.usv_id,
                    rel_x=rel_x,
                    rel_y=rel_y,
                    rel_vx=rel_vx,
                    rel_vy=rel_vy,
                    distance=distance,
                    bearing=bearing,
                )
            )

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
            neighbors=neighbors,
        )

    def _apply_head_on_guard(self, observation: UsvObservation, *, linear_x: float, angular_z: float) -> Tuple[float, float]:
        now = time.monotonic()
        dt = min(max(now - self._head_on_guard_last_update, 1e-3), 0.3)
        self._head_on_guard_last_update = now

        own_speed = max(
            0.0,
            float(observation.speed),
            float(observation.final_linear_x),
            float(observation.raw_linear_x),
        )
        guidance_distance = 6.4
        target_offset = 1.45
        best_neighbor = None
        best_score = -1.0

        for neighbor in observation.neighbors:
            if neighbor.distance <= 1e-3 or neighbor.distance > guidance_distance:
                continue
            body_x = float(neighbor.rel_x)
            body_y = float(neighbor.rel_y)
            body_vx = float(neighbor.rel_vx)
            body_vy = float(neighbor.rel_vy)
            if body_x <= -0.1:
                continue

            closing_speed = -((body_x * body_vx) + (body_y * body_vy)) / max(neighbor.distance, 1e-3)
            if closing_speed <= 0.02:
                continue

            if abs(body_y) > max(1.6, 0.34 * neighbor.distance):
                continue

            neighbor_forward_speed = own_speed + body_vx
            if neighbor_forward_speed >= 0.05:
                continue

            proximity = max(0.0, min(1.0, (guidance_distance - neighbor.distance) / guidance_distance))
            centerline_exposure = max(0.0, 1.0 - min(1.0, abs(body_y) / max(0.8, target_offset)))
            score = 0.65 * proximity + 0.35 * centerline_exposure
            if score > best_score:
                best_score = score
                best_neighbor = neighbor

        raw_linear_x = max(0.0, float(observation.raw_linear_x))
        raw_angular_z = float(observation.raw_angular_z)
        distance = None
        closing_speed = 0.0

        if best_neighbor is not None:
            distance = float(best_neighbor.distance)
            body_y = float(best_neighbor.rel_y)
            body_vx = float(best_neighbor.rel_vx)
            body_vy = float(best_neighbor.rel_vy)
            body_x = float(best_neighbor.rel_x)
            closing_speed = -((body_x * body_vx) + (body_y * body_vy)) / max(distance, 1e-3)
            closing_weight = float(np.clip((closing_speed + 0.18) / 0.95, 0.0, 1.0))
            conflict_distance = 4.0
            commit_progress = float(np.clip((guidance_distance - distance) / max(guidance_distance - conflict_distance, 1e-3), 0.0, 1.0))
            close_quarters = float(np.clip((conflict_distance - distance) / max(conflict_distance, 1e-3), 0.0, 1.0))
            corridor_deficit = float(np.clip((target_offset + body_y) / target_offset, 0.0, 1.0))

            raw_starboard_rate = max(0.0, -raw_angular_z)
            sustain_floor = 0.9 * raw_starboard_rate if raw_starboard_rate > 0.03 else 0.0
            proactive_turn = 0.22 + 0.18 * commit_progress + 0.12 * closing_weight + 0.14 * corridor_deficit
            emergency_turn = 0.0
            if distance <= conflict_distance:
                emergency_turn = 0.10 + 0.12 * close_quarters + 0.10 * corridor_deficit

            desired_turn_floor = float(np.clip(max(sustain_floor, proactive_turn, emergency_turn), 0.18, 0.58))
            slowdown_ratio = 0.80 - 0.12 * commit_progress - 0.08 * closing_weight - 0.06 * corridor_deficit - 0.06 * close_quarters
            desired_linear_cap = float(np.clip(raw_linear_x * slowdown_ratio, 0.18, max(0.18, raw_linear_x)))

            if not self._head_on_guard_active:
                self._head_on_guard_active = True
                self._head_on_guard_linear_cap = desired_linear_cap
                self._head_on_guard_turn_floor = desired_turn_floor
            else:
                if desired_linear_cap < self._head_on_guard_linear_cap:
                    self._head_on_guard_linear_cap = max(desired_linear_cap, self._head_on_guard_linear_cap - 0.45 * dt)
                else:
                    self._head_on_guard_linear_cap = min(desired_linear_cap, self._head_on_guard_linear_cap + 0.10 * dt)

                if desired_turn_floor > self._head_on_guard_turn_floor:
                    self._head_on_guard_turn_floor = min(desired_turn_floor, self._head_on_guard_turn_floor + 1.8 * dt)
                else:
                    self._head_on_guard_turn_floor = max(desired_turn_floor, self._head_on_guard_turn_floor - 0.45 * dt)

            self._head_on_guard_hold_until = now + 1.2

        elif self._head_on_guard_active:
            if now >= self._head_on_guard_hold_until:
                self._head_on_guard_linear_cap = min(raw_linear_x, self._head_on_guard_linear_cap + 0.12 * dt)
                self._head_on_guard_turn_floor = max(0.0, self._head_on_guard_turn_floor - 0.40 * dt)
                if self._head_on_guard_turn_floor <= 1e-3 and self._head_on_guard_linear_cap >= raw_linear_x - 1e-3:
                    self._reset_head_on_guard_state()
                    return linear_x, angular_z
            distance = float(observation.min_neighbor_distance()) if observation.neighbors else None
        else:
            return linear_x, angular_z

        desired_final_linear_x = min(raw_linear_x, max(0.18, self._head_on_guard_linear_cap))
        desired_final_angular_z = min(raw_angular_z, -self._head_on_guard_turn_floor)

        guarded_linear_x = desired_final_linear_x - raw_linear_x
        guarded_angular_z = desired_final_angular_z - raw_angular_z

        if (
            not self._head_on_guard_logged
            and (guarded_linear_x != linear_x or guarded_angular_z != angular_z)
        ):
            self.get_logger().info(
                'Applying head-on residual guard: '
                f'distance={0.0 if distance is None else distance:.2f}, closing_speed={closing_speed:.2f}, '
                f'raw=({observation.raw_linear_x:.2f}, {observation.raw_angular_z:.2f}), '
                f'policy=({linear_x:.2f}, {angular_z:.2f}), '
                f'guarded=({guarded_linear_x:.2f}, {guarded_angular_z:.2f}), '
                f'target_final=({desired_final_linear_x:.2f}, {desired_final_angular_z:.2f}).'
            )
            self._head_on_guard_logged = True

        return guarded_linear_x, guarded_angular_z

    def _publish_action(self):
        self._maybe_enable_controller_param()
        observation = self._build_observation()
        if observation is None:
            self._maybe_log_waiting_for_observation()
            return

        if not self._ready_logged:
            self.get_logger().info('Observation stream ready; residual policy inference is active.')
            self._ready_logged = True

        observation_vector = observation.to_vector(self._max_neighbors)

        if not observation.neighbors:
            if not self._no_neighbor_passthrough_logged:
                self.get_logger().info(
                    'No tracked neighbors for the active goal; publishing zero residual and leaving raw navigation unchanged.'
                )
                self._no_neighbor_passthrough_logged = True

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self._action_pub.publish(msg)
            return

        action = np.asarray(self._policy.predict(observation_vector), dtype=np.float32).reshape(-1)
        if action.size == 1:
            projected_action = project_residual_action(
                action,
                action_mode='angular_only',
                linear_delta_limit=self._action_bounds.linear_delta,
                angular_delta_limit=self._action_bounds.angular_delta,
                raw_linear_x=observation.raw_linear_x,
                forward_only=True,
            )
            linear_x = 0.0
            angular_z = float(projected_action[0])
        elif action.size == 2:
            projected_action = project_residual_action(
                action,
                action_mode='full',
                linear_delta_limit=self._action_bounds.linear_delta,
                angular_delta_limit=self._action_bounds.angular_delta,
                raw_linear_x=observation.raw_linear_x,
                forward_only=True,
            )
            linear_x = float(projected_action[0])
            angular_z = float(projected_action[1])
        else:
            raise RuntimeError(f'Unsupported policy action dimension: {action.size}')

        linear_x, angular_z = self._apply_head_on_guard(
            observation,
            linear_x=linear_x,
            angular_z=angular_z,
        )

        if not self._first_action_logged:
            self.get_logger().info(
                f'Publishing first residual action: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}'
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
        node = ResidualPolicyInferenceNode(
            namespace=args.namespace,
            model_path=args.model,
            policy_kind=args.policy,
            device=args.device,
            publish_rate=args.publish_rate,
            max_neighbors=args.max_neighbors,
            enable_controller_param=not args.disable_controller_param,
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