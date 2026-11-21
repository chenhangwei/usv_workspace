#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time
import importlib
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import yaml

_common_msgs = importlib.import_module('common_interfaces.msg')
FleetNeighborPoses = getattr(_common_msgs, 'FleetNeighborPoses')
NeighborPose = getattr(_common_msgs, 'NeighborPose')


class ApfNeighborRelayNode(Node):
    """GS 侧统一邻船位姿聚合节点。"""

    def __init__(self):
        super().__init__('apf_neighbor_relay_node')

        self.declare_parameter('fleet_config_file', '')
        self.declare_parameter('usv_ids', [])
        self.declare_parameter('source_pose_topic_suffix', 'local_position/pose_from_gps')
        self.declare_parameter('source_velocity_topic_suffix', 'local_position/velocity_local')
        self.declare_parameter('apf_neighbors_topic_suffix', 'apf/neighbors')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('neighbor_timeout', 0.8)
        self.declare_parameter('max_neighbor_distance', 12.0)
        self.declare_parameter('prefer_velocity_topic', True)
        self.declare_parameter('velocity_estimation_min_dt', 0.05)
        self.declare_parameter('velocity_smoothing_alpha', 0.35)
        self.declare_parameter('max_estimated_speed', 3.0)

        self._fleet_config_file = str(self.get_parameter('fleet_config_file').value or '')
        self._usv_ids = [str(x) for x in (self.get_parameter('usv_ids').value or []) if str(x)]
        self._source_pose_topic_suffix = str(
            self.get_parameter('source_pose_topic_suffix').value or 'local_position/pose_from_gps'
        )
        self._source_velocity_topic_suffix = str(
            self.get_parameter('source_velocity_topic_suffix').value or 'local_position/velocity_local'
        )
        self._neighbors_topic_suffix = str(
            self.get_parameter('apf_neighbors_topic_suffix').value or 'apf/neighbors'
        )
        self._publish_rate = float(self.get_parameter('publish_rate').value or 10.0)
        self._neighbor_timeout = float(self.get_parameter('neighbor_timeout').value or 0.8)
        self._max_neighbor_distance = float(self.get_parameter('max_neighbor_distance').value or 12.0)
        self._prefer_velocity_topic = bool(self.get_parameter('prefer_velocity_topic').value)
        self._vel_min_dt = float(self.get_parameter('velocity_estimation_min_dt').value or 0.05)
        self._vel_alpha = float(self.get_parameter('velocity_smoothing_alpha').value or 0.35)
        self._max_estimated_speed = float(self.get_parameter('max_estimated_speed').value or 3.0)

        if not self._usv_ids:
            self._usv_ids = self._load_usv_ids_from_fleet_config(self._fleet_config_file)

        if not self._usv_ids:
            self.get_logger().warn('未检测到可用 USV 列表，邻船聚合节点不会发布数据。')

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self._latest_pose: Dict[str, Dict[str, float]] = {}
        self._pose_subs = []
        self._vel_subs = []
        self._neighbor_pubs = {}

        for usv_id in self._usv_ids:
            pose_topic = f'/{usv_id}/{self._source_pose_topic_suffix}'.replace('//', '/')
            sub = self.create_subscription(
                PoseStamped,
                pose_topic,
                lambda msg, uid=usv_id: self._pose_callback(msg, uid),
                qos_best_effort,
            )
            self._pose_subs.append(sub)

            vel_topic = f'/{usv_id}/{self._source_velocity_topic_suffix}'.replace('//', '/')
            vel_sub = self.create_subscription(
                TwistStamped,
                vel_topic,
                lambda msg, uid=usv_id: self._velocity_callback(msg, uid),
                qos_best_effort,
            )
            self._vel_subs.append(vel_sub)

            pub_topic = f'/{usv_id}/{self._neighbors_topic_suffix}'.replace('//', '/')
            self._neighbor_pubs[usv_id] = self.create_publisher(FleetNeighborPoses, pub_topic, qos_best_effort)

        if self._publish_rate > 0.0:
            self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_neighbors)

        self.get_logger().info(
            f'APF邻船聚合已启动: usv_ids={self._usv_ids}, pose_suffix={self._source_pose_topic_suffix}, '
            f'vel_suffix={self._source_velocity_topic_suffix}, neighbors_suffix={self._neighbors_topic_suffix}, '
            f'rate={self._publish_rate:.1f}Hz'
        )

    def _velocity_callback(self, msg, usv_id: str):
        now_sec = time.time()
        state = self._latest_pose.get(usv_id, {})

        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        speed = math.hypot(vx, vy)
        if self._max_estimated_speed > 0.0 and speed > self._max_estimated_speed:
            scale = self._max_estimated_speed / max(speed, 1e-6)
            vx *= scale
            vy *= scale

        state['vx'] = vx
        state['vy'] = vy
        state['vel_stamp_sec'] = now_sec
        self._latest_pose[usv_id] = state

    def _load_usv_ids_from_fleet_config(self, config_path: str) -> List[str]:
        candidate_paths = []
        if config_path:
            candidate_paths.append(config_path)
        candidate_paths.append(os.path.expanduser('~/usv_workspace/src/gs_bringup/config/usv_fleet.yaml'))

        for path in candidate_paths:
            if not path or not os.path.exists(path):
                continue
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    cfg = yaml.safe_load(f) or {}
                fleet = cfg.get('usv_fleet', {})
                ids = [uid for uid, item in fleet.items() if isinstance(item, dict) and item.get('enabled', True)]
                if ids:
                    self.get_logger().info(f'已从 fleet 配置加载 USV 列表: {ids}')
                    return sorted(ids)
            except Exception as exc:
                self.get_logger().warn(f'读取 fleet 配置失败: {path}, {exc}')

        return []

    def _pose_callback(self, msg, usv_id: str):
        now_sec = time.time()
        prev = self._latest_pose.get(usv_id)
        
        # DEBUG
        self.get_logger().debug(f"Received pose from {usv_id}")

        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        vx = 0.0
        vy = 0.0

        use_diff_fallback = True
        if prev is not None and self._prefer_velocity_topic:
            vel_stamp = float(prev.get('vel_stamp_sec', 0.0))
            if (now_sec - vel_stamp) <= self._neighbor_timeout:
                vx = float(prev.get('vx', 0.0))
                vy = float(prev.get('vy', 0.0))
                use_diff_fallback = False

        if prev is not None and use_diff_fallback:
            dt = now_sec - float(prev.get('stamp_sec', now_sec))
            if dt >= self._vel_min_dt:
                raw_vx = (x - float(prev['x'])) / dt
                raw_vy = (y - float(prev['y'])) / dt
                speed = math.hypot(raw_vx, raw_vy)
                if self._max_estimated_speed > 0.0 and speed > self._max_estimated_speed:
                    scale = self._max_estimated_speed / max(speed, 1e-6)
                    raw_vx *= scale
                    raw_vy *= scale
                prev_vx = float(prev.get('vx', 0.0))
                prev_vy = float(prev.get('vy', 0.0))
                alpha = max(0.0, min(1.0, self._vel_alpha))
                vx = (1.0 - alpha) * prev_vx + alpha * raw_vx
                vy = (1.0 - alpha) * prev_vy + alpha * raw_vy
            else:
                vx = float(prev.get('vx', 0.0))
                vy = float(prev.get('vy', 0.0))

        self._latest_pose[usv_id] = {
            'x': x,
            'y': y,
            'yaw': float(yaw),
            'vx': float(vx),
            'vy': float(vy),
            'vel_stamp_sec': float(prev.get('vel_stamp_sec', 0.0)) if prev else 0.0,
            'stamp_sec': now_sec,
        }

    def _publish_neighbors(self):
        now_sec = time.time()

        for target_usv in self._usv_ids:
            target_pose = self._latest_pose.get(target_usv)
            if not target_pose or 'stamp_sec' not in target_pose:
                self.get_logger().debug(f"No target pose for {target_usv}")
                continue

            out = FleetNeighborPoses()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'map'
            out.target_usv_id = target_usv

            neighbors = []
            for other_usv, other_pose in self._latest_pose.items():
                if other_usv == target_usv:
                    continue
                if 'stamp_sec' not in other_pose:
                    continue
                if now_sec - other_pose['stamp_sec'] > self._neighbor_timeout:
                    continue

                dx = target_pose['x'] - other_pose['x']
                dy = target_pose['y'] - other_pose['y']
                dist = math.hypot(dx, dy)
                if self._max_neighbor_distance > 0.0 and dist > self._max_neighbor_distance:
                    continue

                item = NeighborPose()
                item.usv_id = other_usv
                item.x = other_pose['x']
                item.y = other_pose['y']
                item.yaw = other_pose['yaw']
                item.vx = float(other_pose.get('vx', 0.0))
                item.vy = float(other_pose.get('vy', 0.0))
                item.stamp = out.header.stamp
                neighbors.append(item)

            out.neighbors = neighbors
            pub = self._neighbor_pubs.get(target_usv)
            if pub is not None:
                self.get_logger().debug(f"Publishing {len(neighbors)} neighbors for {target_usv}")
                pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ApfNeighborRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
