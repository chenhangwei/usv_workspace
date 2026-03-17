import argparse
import time

import rclpy
from common_interfaces.msg import FleetNeighborPoses, NeighborPose
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .scenarios import ScenarioFactory


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Publish synthetic neighbor trajectories for online avoidance validation.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--scenario', choices=ScenarioFactory.all_available(), required=True, help='Encounter scenario to publish.')
    parser.add_argument('--goal-distance', type=float, default=8.0, help='Goal distance used to construct the scenario geometry.')
    parser.add_argument('--neighbor-speed', type=float, default=0.3, help='Synthetic neighbor speed in m/s.')
    parser.add_argument('--publish-rate', type=float, default=10.0, help='Neighbor publish rate in Hz.')
    parser.add_argument('--origin-x', type=float, default=0.0, help='Ownship initial X position in meters.')
    parser.add_argument('--origin-y', type=float, default=0.0, help='Ownship initial Y position in meters.')
    parser.add_argument('--origin-yaw', type=float, default=0.0, help='Ownship initial yaw in radians.')
    return parser.parse_args(argv)


class SyntheticNeighborsPublisher(Node):
    def __init__(
        self,
        *,
        namespace: str,
        scenario: str,
        goal_distance: float,
        neighbor_speed: float,
        publish_rate: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
    ):
        resolved_namespace = namespace if namespace.startswith('/') else f'/{namespace}'
        super().__init__('publish_synthetic_neighbors', namespace=resolved_namespace)
        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._usv_id = resolved_namespace.strip('/')
        self._publisher = self.create_publisher(FleetNeighborPoses, 'apf/neighbors', qos_best_effort)
        self._scenario = ScenarioFactory.create(
            scenario,
            origin_x=origin_x,
            origin_y=origin_y,
            yaw=origin_yaw,
            goal_distance=goal_distance,
            neighbor_speed=neighbor_speed,
        )
        self._start_time = time.monotonic()
        self._last_summary_log = 0.0
        self._timer = self.create_timer(1.0 / max(1.0, publish_rate), self._on_timer)
        self.get_logger().info(
            f'Started synthetic neighbor publisher scenario={scenario} goal=({self._scenario.goal_x:.2f}, {self._scenario.goal_y:.2f}) '
            f'neighbors={len(self._scenario.neighbors)}.'
        )

    def _on_timer(self):
        elapsed = time.monotonic() - self._start_time
        msg = FleetNeighborPoses()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_usv_id = self._usv_id

        if elapsed <= self._scenario.duration:
            for state in self._scenario.states_at(elapsed):
                neighbor = NeighborPose()
                neighbor.usv_id = state.usv_id
                neighbor.x = float(state.x)
                neighbor.y = float(state.y)
                neighbor.yaw = float(state.yaw)
                neighbor.vx = float(state.vx)
                neighbor.vy = float(state.vy)
                neighbor.stamp = msg.header.stamp
                msg.neighbors.append(neighbor)

        self._publisher.publish(msg)

        if msg.neighbors and (elapsed - self._last_summary_log) >= 2.0:
            closest = min(msg.neighbors, key=lambda item: ((item.x ** 2) + (item.y ** 2)) ** 0.5)
            self.get_logger().info(
                f'Synthetic neighbor update: count={len(msg.neighbors)} closest={closest.usv_id} '
                f'pos=({closest.x:.2f}, {closest.y:.2f}) vel=({closest.vx:.2f}, {closest.vy:.2f}).'
            )
            self._last_summary_log = elapsed


def main(argv=None):
    cli_args = rclpy.utilities.remove_ros_args(args=argv)
    args = parse_args(cli_args[1:])
    rclpy.init(args=argv)
    node = None
    try:
        node = SyntheticNeighborsPublisher(
            namespace=args.namespace,
            scenario=args.scenario,
            goal_distance=args.goal_distance,
            neighbor_speed=args.neighbor_speed,
            publish_rate=args.publish_rate,
            origin_x=args.origin_x,
            origin_y=args.origin_y,
            origin_yaw=args.origin_yaw,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()