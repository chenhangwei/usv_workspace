import math
import time
from typing import Dict, Optional

from common_interfaces.msg import FleetNeighborPoses, NavigationFeedback, NavigationGoal, NeighborPose
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .multi_agent_scenarios import FleetScenario
from .multi_agent_types import AgentLocalObservation, AgentNeighborObservation, FleetGlobalState
from .types import NeighborState


def _quat_to_yaw(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MultiAgentTrainingBridge(Node):
    def __init__(self, agent_namespaces: tuple[str, ...], neighbor_publish_rate: float):
        super().__init__('multi_usv_rl_bridge')
        self._agent_namespaces = tuple(agent_namespaces)
        self._parameter_clients: Dict[str, AsyncParameterClient] = {}
        self._pose_msgs: Dict[str, Optional[PoseStamped]] = {namespace: None for namespace in self._agent_namespaces}
        self._velocity_msgs: Dict[str, Optional[TwistStamped]] = {namespace: None for namespace in self._agent_namespaces}
        self._feedback_msgs: Dict[str, Optional[NavigationFeedback]] = {namespace: None for namespace in self._agent_namespaces}
        self._raw_cmd_msgs: Dict[str, Optional[TwistStamped]] = {namespace: None for namespace in self._agent_namespaces}
        self._final_cmd_msgs: Dict[str, Optional[PositionTarget]] = {namespace: None for namespace in self._agent_namespaces}
        self._goal_publishers = {}
        self._neighbor_publishers = {}
        self._rl_action_publishers = {}
        self._goal_id_counter = {namespace: 1000 + index * 1000 for index, namespace in enumerate(self._agent_namespaces)}
        self._rl_backend_active = False

        self._active_scenario: Optional[FleetScenario] = None
        self._scenario_start_time: float = 0.0

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        for namespace in self._agent_namespaces:
            self._parameter_clients[namespace] = AsyncParameterClient(self, f'/{namespace}/velocity_controller_node')
            self.create_subscription(PoseStamped, f'/{namespace}/local_position/pose_from_gps', self._make_pose_callback(namespace), qos_best_effort)
            self.create_subscription(TwistStamped, f'/{namespace}/local_position/velocity_local', self._make_velocity_callback(namespace), qos_best_effort)
            self.create_subscription(NavigationFeedback, f'/{namespace}/navigation_feedback', self._make_feedback_callback(namespace), qos_best_effort)
            self.create_subscription(NavigationFeedback, f'/{namespace}/velocity_controller/feedback', self._make_feedback_callback(namespace), qos_best_effort)
            self.create_subscription(TwistStamped, f'/{namespace}/velocity_controller/raw_cmd', self._make_raw_cmd_callback(namespace), qos_best_effort)
            self.create_subscription(PositionTarget, f'/{namespace}/setpoint_raw/local', self._make_final_cmd_callback(namespace), qos_best_effort)
            self._goal_publishers[namespace] = self.create_publisher(NavigationGoal, f'/{namespace}/set_usv_nav_goal', qos_reliable)
            self._neighbor_publishers[namespace] = self.create_publisher(FleetNeighborPoses, f'/{namespace}/apf/neighbors', qos_best_effort)
            self._rl_action_publishers[namespace] = self.create_publisher(TwistStamped, f'/{namespace}/rl_policy/cmd_vel', qos_best_effort)

        self._neighbor_timer = self.create_timer(1.0 / max(1.0, neighbor_publish_rate), self._publish_neighbor_views)

    def _make_pose_callback(self, namespace: str):
        def _callback(msg: PoseStamped):
            self._pose_msgs[namespace] = msg
        return _callback

    def _make_velocity_callback(self, namespace: str):
        def _callback(msg: TwistStamped):
            self._velocity_msgs[namespace] = msg
        return _callback

    def _make_feedback_callback(self, namespace: str):
        def _callback(msg: NavigationFeedback):
            self._feedback_msgs[namespace] = msg
        return _callback

    def _make_raw_cmd_callback(self, namespace: str):
        def _callback(msg: TwistStamped):
            self._raw_cmd_msgs[namespace] = msg
        return _callback

    def _make_final_cmd_callback(self, namespace: str):
        def _callback(msg: PositionTarget):
            self._final_cmd_msgs[namespace] = msg
        return _callback

    def wait_for_agents(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(self._pose_msgs[ns] is not None and self._velocity_msgs[ns] is not None for ns in self._agent_namespaces):
                return True
            time.sleep(0.1)
        return False

    def reset_episode_state(self):
        for namespace in self._agent_namespaces:
            self._feedback_msgs[namespace] = None
            self._raw_cmd_msgs[namespace] = None
            self._final_cmd_msgs[namespace] = None

    def wait_for_controllers(self, timeout: float) -> bool:
        return all(client.wait_for_services(timeout_sec=timeout) for client in self._parameter_clients.values())

    def wait_for_goal_subscribers(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(self._goal_publishers[ns].get_subscription_count() > 0 for ns in self._agent_namespaces):
                return True
            time.sleep(0.05)
        return False

    def set_rl_backend_enabled(self, enabled: bool, timeout: float = 10.0) -> bool:
        self._rl_backend_active = bool(enabled)
        success = True
        for namespace, client in self._parameter_clients.items():
            if not client.wait_for_services(timeout_sec=timeout):
                success = False
                continue
            future = client.set_parameters([
                Parameter('rl_policy_enabled', value=enabled),
                Parameter('rl_policy_fallback_to_raw', value=False),
            ])
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and not future.done():
                time.sleep(0.05)
            if not future.done():
                success = False
                continue
            result = future.result()
            if result is None:
                success = False
                continue
            response_items = getattr(result, 'results', result if isinstance(result, (list, tuple)) else None)
            if response_items is None:
                success = False
                continue
            success = success and all(item.successful for item in response_items)
        return success

    def publish_goals(self, task_prefix: str, goal_map: Dict[str, tuple[float, float]]) -> Dict[str, int]:
        self.wait_for_goal_subscribers(timeout=5.0)
        goal_ids = {}
        for namespace, (goal_x, goal_y) in goal_map.items():
            self._goal_id_counter[namespace] += 1
            goal_ids[namespace] = self._goal_id_counter[namespace]
            goal = NavigationGoal()
            goal.task_name = f'{task_prefix}_{namespace}'
            goal.goal_id = goal_ids[namespace]
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
                self._goal_publishers[namespace].publish(goal)
                time.sleep(0.05)
        return goal_ids

    def publish_actions(self, action_map: Dict[str, tuple[float, float]]):
        for namespace, (linear_delta, angular_delta) in action_map.items():
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(linear_delta)
            msg.twist.angular.z = float(angular_delta)
            self._rl_action_publishers[namespace].publish(msg)

    def activate_scenario(self, scenario: FleetScenario):
        self._active_scenario = scenario
        self._scenario_start_time = time.monotonic()

    def clear_scenario(self):
        self._active_scenario = None
        for namespace in self._agent_namespaces:
            msg = FleetNeighborPoses()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.target_usv_id = namespace
            self._neighbor_publishers[namespace].publish(msg)

    def _publish_neighbor_views(self):
        if self._active_scenario is None:
            return

        elapsed = time.monotonic() - self._scenario_start_time
        background_states = self._active_scenario.states_at(elapsed)
        current_states = self._collect_agent_states()
        if len(current_states) < len(self._agent_namespaces):
            return

        for namespace in self._agent_namespaces:
            msg = FleetNeighborPoses()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.target_usv_id = namespace

            for other_namespace, state in current_states.items():
                if other_namespace == namespace:
                    continue
                neighbor = NeighborPose()
                neighbor.usv_id = other_namespace
                neighbor.x = float(state.x)
                neighbor.y = float(state.y)
                neighbor.yaw = float(state.yaw)
                neighbor.vx = float(state.vx)
                neighbor.vy = float(state.vy)
                neighbor.stamp = msg.header.stamp
                msg.neighbors.append(neighbor)

            for state in background_states:
                neighbor = NeighborPose()
                neighbor.usv_id = state.usv_id
                neighbor.x = float(state.x)
                neighbor.y = float(state.y)
                neighbor.yaw = float(state.yaw)
                neighbor.vx = float(state.vx)
                neighbor.vy = float(state.vy)
                neighbor.stamp = msg.header.stamp
                msg.neighbors.append(neighbor)

            self._neighbor_publishers[namespace].publish(msg)

    def _collect_agent_states(self) -> Dict[str, NeighborState]:
        current_states = {}
        for namespace in self._agent_namespaces:
            pose_msg = self._pose_msgs[namespace]
            velocity_msg = self._velocity_msgs[namespace]
            if pose_msg is None or velocity_msg is None:
                continue
            current_states[namespace] = NeighborState(
                usv_id=namespace,
                x=float(pose_msg.pose.position.x),
                y=float(pose_msg.pose.position.y),
                yaw=_quat_to_yaw(pose_msg),
                vx=float(velocity_msg.twist.linear.x),
                vy=float(velocity_msg.twist.linear.y),
            )
        return current_states

    def get_local_observations(self, max_neighbors: int) -> Optional[Dict[str, AgentLocalObservation]]:
        current_states = self._collect_agent_states()
        if len(current_states) < len(self._agent_namespaces):
            return None

        observations = {}
        background_states = [] if self._active_scenario is None else self._active_scenario.states_at(time.monotonic() - self._scenario_start_time)
        for namespace in self._agent_namespaces:
            pose_msg = self._pose_msgs[namespace]
            velocity_msg = self._velocity_msgs[namespace]
            feedback_msg = self._feedback_msgs[namespace]
            if pose_msg is None or velocity_msg is None or feedback_msg is None:
                return None

            yaw = _quat_to_yaw(pose_msg)
            own_x = float(pose_msg.pose.position.x)
            own_y = float(pose_msg.pose.position.y)
            own_vx = float(velocity_msg.twist.linear.x)
            own_vy = float(velocity_msg.twist.linear.y)
            speed = math.hypot(own_vx, own_vy)

            raw_msg = self._raw_cmd_msgs[namespace]
            final_msg = self._final_cmd_msgs[namespace]
            raw_linear_x = 0.0
            raw_angular_z = 0.0
            final_linear_x = float(final_msg.velocity.x) if final_msg is not None else 0.0
            final_angular_z = float(final_msg.yaw_rate) if final_msg is not None else 0.0

            neighbors = []
            for other_namespace, state in current_states.items():
                if other_namespace == namespace:
                    continue
                neighbors.append(self._build_neighbor_observation(other_namespace, state, own_x, own_y, own_vx, own_vy, yaw))
            for state in background_states:
                neighbors.append(self._build_neighbor_observation(state.usv_id, state, own_x, own_y, own_vx, own_vy, yaw))

            observations[namespace] = AgentLocalObservation(
                agent_id=namespace,
                pose_x=own_x,
                pose_y=own_y,
                yaw=yaw,
                speed=speed,
                distance_to_goal=float(feedback_msg.distance_to_goal),
                heading_error=float(feedback_msg.heading_error),
                raw_linear_x=raw_linear_x,
                raw_angular_z=raw_angular_z,
                final_linear_x=final_linear_x,
                final_angular_z=final_angular_z,
                neighbors=sorted(neighbors, key=lambda item: item.distance)[:max_neighbors],
            )
        return observations

    def _build_neighbor_observation(
        self,
        source_id: str,
        state: NeighborState,
        own_x: float,
        own_y: float,
        own_vx: float,
        own_vy: float,
        own_yaw: float,
    ) -> AgentNeighborObservation:
        rel_x = state.x - own_x
        rel_y = state.y - own_y
        rel_vx = state.vx - own_vx
        rel_vy = state.vy - own_vy
        distance = math.hypot(rel_x, rel_y)
        bearing = math.atan2(rel_y, rel_x) - own_yaw
        while bearing > math.pi:
            bearing -= 2.0 * math.pi
        while bearing < -math.pi:
            bearing += 2.0 * math.pi
        return AgentNeighborObservation(
            source_id=source_id,
            rel_x=rel_x,
            rel_y=rel_y,
            rel_vx=rel_vx,
            rel_vy=rel_vy,
            distance=distance,
            bearing=bearing,
        )

    def get_global_state(
        self,
        max_agents: int,
        max_neighbors: int,
        *,
        active_agent_ids: tuple[str, ...] | None = None,
        goal_tolerance: float = 0.8,
    ) -> Optional[FleetGlobalState]:
        local_observations = self.get_local_observations(max_neighbors)
        if local_observations is None:
            return None
        return FleetGlobalState.from_local_observations(
            local_observations,
            active_agent_ids=active_agent_ids,
            goal_tolerance=goal_tolerance,
        )

    def prepare_for_shutdown(self):
        self._active_scenario = None
        if self._neighbor_timer is not None:
            self._neighbor_timer.cancel()
            self.destroy_timer(self._neighbor_timer)
            self._neighbor_timer = None