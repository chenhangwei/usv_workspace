import math
import time
import argparse

import rclpy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SimpleUsvSimNode(Node):
    """最小训练动力学节点，用于在 RL 训练中驱动 velocity_controller_node。"""

    def __init__(self, namespace: str, update_rate: float = 20.0):
        resolved_namespace = namespace if namespace.startswith('/') else f'/{namespace}'
        super().__init__('simple_usv_sim', namespace=resolved_namespace)

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._dt = 1.0 / max(update_rate, 1.0)
        self._tau_linear = 0.45
        self._tau_angular = 0.25
        self._command_timeout = 0.6

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v = 0.0
        self._omega = 0.0
        self._target_v = 0.0
        self._target_omega = 0.0
        self._last_cmd_time = 0.0

        self._pose_pub = self.create_publisher(PoseStamped, 'local_position/pose_from_gps', qos_best_effort)
        self._mavros_pose_pub = self.create_publisher(PoseStamped, 'local_position/pose', qos_best_effort)
        self._velocity_pub = self.create_publisher(TwistStamped, 'local_position/velocity_local', qos_best_effort)
        self._state_pub = self.create_publisher(State, 'state', qos_reliable)

        self.create_subscription(PositionTarget, 'setpoint_raw/local', self._cmd_callback, qos_best_effort)
        self._update_timer = self.create_timer(self._dt, self._on_timer)

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self._x = float(x)
        self._y = float(y)
        self._yaw = float(yaw)
        self._v = 0.0
        self._omega = 0.0
        self._target_v = 0.0
        self._target_omega = 0.0
        self._last_cmd_time = time.monotonic()
        self._publish_pose(0.0, 0.0)
        self._publish_state()

    def _cmd_callback(self, msg: PositionTarget):
        self._target_v = float(msg.velocity.x)
        self._target_omega = float(msg.yaw_rate)
        self._last_cmd_time = time.monotonic()

    def _on_timer(self):
        now = time.monotonic()
        if (now - self._last_cmd_time) > self._command_timeout:
            self._target_v = 0.0
            self._target_omega = 0.0

        linear_alpha = min(1.0, self._dt / max(self._tau_linear, 1e-3))
        angular_alpha = min(1.0, self._dt / max(self._tau_angular, 1e-3))
        self._v += linear_alpha * (self._target_v - self._v)
        self._omega += angular_alpha * (self._target_omega - self._omega)

        self._yaw += self._omega * self._dt
        while self._yaw > math.pi:
            self._yaw -= 2.0 * math.pi
        while self._yaw < -math.pi:
            self._yaw += 2.0 * math.pi

        vx_world = self._v * math.cos(self._yaw)
        vy_world = self._v * math.sin(self._yaw)
        self._x += vx_world * self._dt
        self._y += vy_world * self._dt

        self._publish_pose(vx_world, vy_world)
        self._publish_state()

    def _publish_pose(self, vx_world: float, vy_world: float):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _yaw_to_quaternion(self._yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self._x
        pose_msg.pose.position.y = self._y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self._pose_pub.publish(pose_msg)
        self._mavros_pose_pub.publish(pose_msg)

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = stamp
        velocity_msg.header.frame_id = 'map'
        velocity_msg.twist.linear.x = vx_world
        velocity_msg.twist.linear.y = vy_world
        velocity_msg.twist.angular.z = self._omega
        self._velocity_pub.publish(velocity_msg)

    def _publish_state(self):
        state_msg = State()
        state_msg.connected = True
        state_msg.armed = True
        state_msg.guided = True
        state_msg.manual_input = False
        state_msg.mode = 'GUIDED'
        self._state_pub.publish(state_msg)

    def prepare_for_shutdown(self):
        if self._update_timer is not None:
            self._update_timer.cancel()
        self._target_v = 0.0
        self._target_omega = 0.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Run the lightweight USV simulator used for RL validation.')
    parser.add_argument('--namespace', default='usv_03', help='Target namespace for the simulated USV.')
    parser.add_argument('--update-rate', type=float, default=20.0, help='Simulation update rate in Hz.')
    parser.add_argument('--start-x', type=float, default=0.0, help='Initial X position in meters.')
    parser.add_argument('--start-y', type=float, default=0.0, help='Initial Y position in meters.')
    parser.add_argument('--start-yaw', type=float, default=0.0, help='Initial yaw in radians.')
    return parser.parse_args(argv)


def main(argv=None):
    cli_args = rclpy.utilities.remove_ros_args(args=argv)
    args = parse_args(cli_args[1:])
    rclpy.init(args=argv)
    node = None
    try:
        node = SimpleUsvSimNode(namespace=args.namespace, update_rate=args.update_rate)
        node.reset(x=args.start_x, y=args.start_y, yaw=args.start_yaw)
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