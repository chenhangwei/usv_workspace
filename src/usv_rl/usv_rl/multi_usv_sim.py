import argparse
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .multi_agent_scenarios import FleetScenario


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass
class _SimAgentState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    target_v: float = 0.0
    target_omega: float = 0.0
    last_cmd_time: float = 0.0


class MultiUsvSimNode(Node):
    def __init__(self, agent_namespaces: tuple[str, ...], update_rate: float = 20.0):
        super().__init__('multi_usv_sim')
        self._agent_namespaces = tuple(agent_namespaces)
        self._dt = 1.0 / max(update_rate, 1.0)
        self._tau_linear = 0.45
        self._tau_angular = 0.25
        self._command_timeout = 0.6

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._agent_states = {namespace: _SimAgentState() for namespace in self._agent_namespaces}
        self._pose_publishers = {}
        self._mavros_pose_publishers = {}
        self._velocity_publishers = {}
        self._state_publishers = {}

        for namespace in self._agent_namespaces:
            self._pose_publishers[namespace] = self.create_publisher(
                PoseStamped,
                f'/{namespace}/local_position/pose_from_gps',
                qos_best_effort,
            )
            self._mavros_pose_publishers[namespace] = self.create_publisher(
                PoseStamped,
                f'/{namespace}/local_position/pose',
                qos_best_effort,
            )
            self._velocity_publishers[namespace] = self.create_publisher(
                TwistStamped,
                f'/{namespace}/local_position/velocity_local',
                qos_best_effort,
            )
            self._state_publishers[namespace] = self.create_publisher(
                State,
                f'/{namespace}/state',
                qos_reliable,
            )
            self.create_subscription(
                PositionTarget,
                f'/{namespace}/setpoint_raw/local',
                self._make_cmd_callback(namespace),
                qos_best_effort,
            )

        self._update_timer = self.create_timer(self._dt, self._on_timer)

    def _make_cmd_callback(self, namespace: str):
        def _callback(msg: PositionTarget):
            state = self._agent_states[namespace]
            state.target_v = float(msg.velocity.x)
            state.target_omega = float(msg.yaw_rate)
            state.last_cmd_time = time.monotonic()
        return _callback

    def reset_agents(self, scenario: FleetScenario):
        now = time.monotonic()
        for namespace, state in self._agent_states.items():
            spawn = scenario.agent_spawns[namespace]
            state.x = float(spawn.x)
            state.y = float(spawn.y)
            state.yaw = float(spawn.yaw)
            state.v = 0.0
            state.omega = 0.0
            state.target_v = 0.0
            state.target_omega = 0.0
            state.last_cmd_time = now
            self._publish_agent_state(namespace, state)

    def get_agent_snapshot(self, namespace: str) -> tuple[float, float, float, float]:
        state = self._agent_states[namespace]
        return state.x, state.y, state.yaw, state.v

    def _on_timer(self):
        now = time.monotonic()
        for namespace, state in self._agent_states.items():
            if (now - state.last_cmd_time) > self._command_timeout:
                state.target_v = 0.0
                state.target_omega = 0.0

            linear_alpha = min(1.0, self._dt / max(self._tau_linear, 1e-3))
            angular_alpha = min(1.0, self._dt / max(self._tau_angular, 1e-3))
            state.v += linear_alpha * (state.target_v - state.v)
            state.omega += angular_alpha * (state.target_omega - state.omega)

            state.yaw += state.omega * self._dt
            while state.yaw > math.pi:
                state.yaw -= 2.0 * math.pi
            while state.yaw < -math.pi:
                state.yaw += 2.0 * math.pi

            vx_world = state.v * math.cos(state.yaw)
            vy_world = state.v * math.sin(state.yaw)
            state.x += vx_world * self._dt
            state.y += vy_world * self._dt
            self._publish_agent_state(namespace, state)

    def _publish_agent_state(self, namespace: str, state: _SimAgentState):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _yaw_to_quaternion(state.yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = state.x
        pose_msg.pose.position.y = state.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self._pose_publishers[namespace].publish(pose_msg)
        self._mavros_pose_publishers[namespace].publish(pose_msg)

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = stamp
        velocity_msg.header.frame_id = 'map'
        velocity_msg.twist.linear.x = state.v * math.cos(state.yaw)
        velocity_msg.twist.linear.y = state.v * math.sin(state.yaw)
        velocity_msg.twist.angular.z = state.omega
        self._velocity_publishers[namespace].publish(velocity_msg)

        state_msg = State()
        state_msg.connected = True
        state_msg.armed = True
        state_msg.guided = True
        state_msg.manual_input = False
        state_msg.mode = 'GUIDED'
        self._state_publishers[namespace].publish(state_msg)

    def prepare_for_shutdown(self):
        if self._update_timer is not None:
            self._update_timer.cancel()
            self.destroy_timer(self._update_timer)
            self._update_timer = None
        for state in self._agent_states.values():
            state.target_v = 0.0
            state.target_omega = 0.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Run the lightweight multi-USV simulator used for MAPPO migration.')
    parser.add_argument('--namespaces', nargs='+', default=['usv_01', 'usv_02'], help='Simulated USV namespaces.')
    parser.add_argument('--update-rate', type=float, default=20.0, help='Simulation update rate in Hz.')
    return parser.parse_args(argv)


def main(argv=None):
    cli_args = rclpy.utilities.remove_ros_args(args=argv)
    args = parse_args(cli_args[1:])
    rclpy.init(args=argv)
    node = None
    try:
        node = MultiUsvSimNode(tuple(args.namespaces), update_rate=args.update_rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.prepare_for_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()