import argparse
import math
import time

import rclpy
from common_interfaces.msg import NavigationGoal
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Publish one NavigationGoal and exit.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--x', type=float, required=True, help='Goal X position in meters.')
    parser.add_argument('--y', type=float, required=True, help='Goal Y position in meters.')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw in radians.')
    parser.add_argument('--nav-mode', type=int, default=int(NavigationGoal.NAV_MODE_TERMINAL), help='NavigationGoal.nav_mode value.')
    parser.add_argument('--timeout', type=float, default=120.0, help='Navigation timeout in seconds.')
    parser.add_argument('--task-name', default='rl_validation_goal', help='Task name to embed in the goal message.')
    parser.add_argument('--wait-timeout', type=float, default=5.0, help='How long to wait for a subscriber before publishing anyway.')
    parser.add_argument('--publish-count', type=int, default=3, help='How many times to publish the same goal message to reduce startup delivery races.')
    parser.add_argument('--publish-interval', type=float, default=0.2, help='Delay in seconds between repeated goal publications.')
    return parser.parse_args(argv)


class PublishNavGoalOnceNode(Node):
    def __init__(self, *, namespace: str):
        resolved_namespace = namespace if namespace.startswith('/') else f'/{namespace}'
        super().__init__('publish_nav_goal_once', namespace=resolved_namespace)
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(NavigationGoal, 'navigation_goal', qos_reliable)

    def wait_for_subscriber(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if self._publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._publisher.get_subscription_count() > 0

    def publish_goal(self, *, x: float, y: float, yaw: float, nav_mode: int, timeout: float, task_name: str, publish_count: int, publish_interval: float):
        msg = NavigationGoal()
        msg.task_name = task_name
        msg.goal_id = int(time.time() * 1000) & 0xFFFFFFFF
        msg.enable_yaw = False
        msg.nav_mode = int(nav_mode)
        msg.sync_timeout = 0.0
        msg.arrival_quality_threshold = 0.8
        msg.maneuver_type = NavigationGoal.MANEUVER_TYPE_NONE
        msg.maneuver_param = 0.0
        msg.timeout = float(timeout)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        publish_count = max(1, int(publish_count))
        publish_interval = max(0.0, float(publish_interval))

        for attempt in range(1, publish_count + 1):
            msg.timestamp = self.get_clock().now().to_msg()
            pose.header.stamp = msg.timestamp
            msg.target_pose = pose
            self._publisher.publish(msg)
            self.get_logger().info(
                f'Published navigation goal attempt {attempt}/{publish_count} '
                f'goal_id={msg.goal_id} target=({x:.2f}, {y:.2f}) nav_mode={nav_mode}.'
            )
            if attempt != publish_count and publish_interval > 0.0:
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(publish_interval)


def main(argv=None):
    cli_args = rclpy.utilities.remove_ros_args(args=argv)
    args = parse_args(cli_args[1:])
    rclpy.init(args=argv)
    node = None
    try:
        node = PublishNavGoalOnceNode(namespace=args.namespace)
        has_subscriber = node.wait_for_subscriber(args.wait_timeout)
        if not has_subscriber:
            node.get_logger().warning('No navigation_goal subscriber discovered before timeout; publishing anyway.')
        node.publish_goal(
            x=args.x,
            y=args.y,
            yaw=args.yaw,
            nav_mode=args.nav_mode,
            timeout=args.timeout,
            task_name=args.task_name,
            publish_count=args.publish_count,
            publish_interval=args.publish_interval,
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()