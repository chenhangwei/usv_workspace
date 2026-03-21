import math
import time
from typing import Optional

from common_interfaces.msg import FleetNeighborPoses, NavigationFeedback, NavigationGoal, NeighborPose
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .config import EnvConfig
from .scenarios import EncounterScenario
from .types import NeighborObservation, NeighborState, UsvObservation


def _quat_to_yaw(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrainingBridge(Node):
    def __init__(self, config: EnvConfig):
        namespace = config.namespace if config.namespace.startswith('/') else f'/{config.namespace}'
        super().__init__('usv_rl_bridge', namespace=namespace)
        self._config = config
        self._namespace = namespace
        self._controller_node_name = f'{namespace}/velocity_controller_node'
        self._parameter_client = AsyncParameterClient(self, self._controller_node_name)

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._pose_msg: Optional[PoseStamped] = None
        self._velocity_msg: Optional[TwistStamped] = None
        self._feedback_msg: Optional[NavigationFeedback] = None
        self._raw_cmd_msg: Optional[TwistStamped] = None
        self._final_cmd_msg: Optional[PositionTarget] = None
        self._active_scenario: Optional[EncounterScenario] = None
        self._scenario_start_time: float = 0.0
        self._current_neighbor_states: list[NeighborState] = []
        self._goal_id_counter = 1000
        self._rl_backend_active = False

        self.create_subscription(PoseStamped, 'local_position/pose_from_gps', self._pose_callback, qos_best_effort)
        self.create_subscription(TwistStamped, 'local_position/velocity_local', self._velocity_callback, qos_best_effort)
        self.create_subscription(NavigationFeedback, 'navigation_feedback', self._feedback_callback, qos_best_effort)
        self.create_subscription(NavigationFeedback, 'velocity_controller/feedback', self._feedback_callback, qos_best_effort)
        self.create_subscription(TwistStamped, 'velocity_controller/raw_cmd', self._raw_cmd_callback, qos_best_effort)
        self.create_subscription(PositionTarget, 'setpoint_raw/local', self._final_cmd_callback, qos_best_effort)

        self._goal_pub = self.create_publisher(NavigationGoal, 'set_usv_nav_goal', qos_reliable)
        self._neighbor_pub = self.create_publisher(FleetNeighborPoses, 'apf/neighbors', qos_best_effort)
        self._rl_action_pub = self.create_publisher(TwistStamped, 'rl_policy/cmd_vel', qos_best_effort)
        self._neighbor_timer = self.create_timer(
            1.0 / max(1.0, config.neighbor_publish_rate),
            self._publish_synthetic_neighbors,
        )

    def _pose_callback(self, msg: PoseStamped):
        self._pose_msg = msg

    def _velocity_callback(self, msg: TwistStamped):
        self._velocity_msg = msg

    def _feedback_callback(self, msg: NavigationFeedback):
        self._feedback_msg = msg

    def _raw_cmd_callback(self, msg: TwistStamped):
        self._raw_cmd_msg = msg

    def _final_cmd_callback(self, msg: PositionTarget):
        self._final_cmd_msg = msg

    def wait_for_pose(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._pose_msg is not None and self._velocity_msg is not None:
                return True
            time.sleep(0.1)
        return False

    def reset_episode_state(self):
        self._feedback_msg = None
        self._raw_cmd_msg = None
        self._final_cmd_msg = None

    def wait_for_controller(self, timeout: float) -> bool:
        return self._parameter_client.wait_for_services(timeout_sec=timeout)

    def wait_for_goal_subscriber(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._goal_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return False

    def set_rl_backend_enabled(self, enabled: bool, timeout: float = 10.0) -> bool:
        if not self._parameter_client.wait_for_services(timeout_sec=timeout):
            return False

        self._rl_backend_active = bool(enabled)

        future = self._parameter_client.set_parameters([
            Parameter('rl_policy_enabled', value=enabled),
            Parameter('rl_policy_fallback_to_raw', value=False),
        ])
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                if result is None:
                    return False
                if isinstance(result, (list, tuple)):
                    return all(item.successful for item in result)
                response_items = getattr(result, 'results', None)
                if response_items is not None:
                    return all(item.successful for item in response_items)
                single_result = getattr(result, 'result', None)
                if single_result is not None and hasattr(single_result, 'successful'):
                    return bool(single_result.successful)
                return False
            time.sleep(0.05)
        return False

    def publish_goal(self, goal_x: float, goal_y: float, task_name: str) -> int:
        self.wait_for_goal_subscriber()
        self._goal_id_counter += 1
        goal = NavigationGoal()
        goal.task_name = task_name
        goal.goal_id = self._goal_id_counter
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = float(goal_x)
        goal.target_pose.pose.position.y = float(goal_y)
        goal.target_pose.pose.position.z = 0.0
        goal.enable_yaw = False
        goal.nav_mode = NavigationGoal.NAV_MODE_ASYNC
        goal.sync_timeout = 0.0
        goal.arrival_quality_threshold = 0.8
        goal.maneuver_type = NavigationGoal.MANEUVER_TYPE_NONE
        goal.maneuver_param = 0.0
        goal.timeout = 120.0
        for _ in range(5):
            stamp = self.get_clock().now().to_msg()
            goal.target_pose.header.stamp = stamp
            goal.timestamp = stamp
            self._goal_pub.publish(goal)
            time.sleep(0.1)
        return self._goal_id_counter

    def publish_rl_action(self, linear_delta: float, angular_delta: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_delta)
        msg.twist.angular.z = float(angular_delta)
        self._rl_action_pub.publish(msg)

    def activate_scenario(self, scenario: EncounterScenario):
        self._active_scenario = scenario
        self._scenario_start_time = time.monotonic()

    def clear_scenario(self):
        self._active_scenario = None
        self._current_neighbor_states = []
        msg = FleetNeighborPoses()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_usv_id = self._namespace.strip('/')
        self._neighbor_pub.publish(msg)

    def prepare_for_shutdown(self):
        self._active_scenario = None
        self._current_neighbor_states = []
        if self._neighbor_timer is not None:
            self._neighbor_timer.cancel()

    def _publish_synthetic_neighbors(self):
        if self._active_scenario is None:
            return

        elapsed = time.monotonic() - self._scenario_start_time
        self._current_neighbor_states = self._active_scenario.states_at(elapsed)
        msg = FleetNeighborPoses()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_usv_id = self._namespace.strip('/')
        for state in self._current_neighbor_states:
            neighbor = NeighborPose()
            neighbor.usv_id = state.usv_id
            neighbor.x = float(state.x)
            neighbor.y = float(state.y)
            neighbor.yaw = float(state.yaw)
            neighbor.vx = float(state.vx)
            neighbor.vy = float(state.vy)
            neighbor.stamp = msg.header.stamp
            msg.neighbors.append(neighbor)
        self._neighbor_pub.publish(msg)

    def get_teacher_action(self) -> Optional[tuple[float, float]]:
        if self._raw_cmd_msg is None or self._final_cmd_msg is None:
            return None

        linear_delta = float(self._final_cmd_msg.velocity.x) - float(self._raw_cmd_msg.twist.linear.x)
        angular_delta = float(self._final_cmd_msg.yaw_rate) - float(self._raw_cmd_msg.twist.angular.z)
        return linear_delta, angular_delta

    def get_pose_snapshot(self) -> Optional[tuple[float, float, float, float]]:
        if self._pose_msg is None or self._velocity_msg is None:
            return None

        yaw = _quat_to_yaw(self._pose_msg)
        own_x = float(self._pose_msg.pose.position.x)
        own_y = float(self._pose_msg.pose.position.y)
        own_vx = float(self._velocity_msg.twist.linear.x)
        own_vy = float(self._velocity_msg.twist.linear.y)
        speed = math.hypot(own_vx, own_vy)
        return own_x, own_y, yaw, speed

    def get_observation(self) -> Optional[UsvObservation]:
        if self._pose_msg is None or self._velocity_msg is None or self._feedback_msg is None:
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
        for state in self._current_neighbor_states:
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