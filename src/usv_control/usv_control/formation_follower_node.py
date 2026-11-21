#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
#
# This file is part of the USV Workspace project.
#
# USV-side formation follower node (PX4 uXRCE-DDS + ENU UsvStatus version).
"""
编队跟随节点 (USV / 异构载具端)

该节点运行在每台载具命名空间下，负责：
1. 接收地面站下发的编队配置 (formation/config, TRANSIENT_LOCAL)
2. 订阅领队载具状态 (formation/leader_state, 由地面站中转)
3. 在本地高频 (默认 10Hz) 计算编队偏移目标坐标 (ENU)
4. 直接发布 NavigationGoal 到本地 set_usv_nav_goal (供 usv_control_node 与 policy_inference_node 执行)
"""

import json
import math
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from common_interfaces.msg import FormationConfig, NavigationGoal, UsvStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, String


class FormationType(IntEnum):
    V_SHAPE = 0    # 人字形
    LINE = 1       # 一字形
    DIAMOND = 2    # 菱形
    TRIANGLE = 3   # 三角形
    COLUMN = 4     # 纵列
    S_SHAPE = 5    # S形
    ESCORT = 6     # 护卫


@dataclass
class FormationOffset:
    """编队偏移量 (领队坐标系)"""
    along: float  # 前后偏移 (负 = 后方)
    cross: float  # 左右偏移 (正 = 右侧)


class FormationCalculator:
    """
    静态方法类，计算各编队类型中每个跟随者的偏移量（与地面站 FormationCalculator 严格对齐）。
    所有坐标基于领队坐标系：
        along: 沿领队航向 (负值 = 后方)
        cross: 垂直于航向 (正值 = 右侧, 负值 = 左侧)
    """

    @staticmethod
    def compute_offsets(
        formation_type: int,
        num_followers: int,
        spacing_along: float,
        spacing_cross: float,
    ) -> List[FormationOffset]:
        if num_followers <= 0:
            return []
        try:
            ftype = FormationType(formation_type)
        except ValueError:
            ftype = FormationType.V_SHAPE

        if ftype == FormationType.V_SHAPE:
            return FormationCalculator._v_shape(num_followers, spacing_along, spacing_cross)
        if ftype == FormationType.LINE:
            return FormationCalculator._line(num_followers, spacing_cross)
        if ftype == FormationType.DIAMOND:
            return FormationCalculator._diamond(num_followers, spacing_along, spacing_cross)
        if ftype == FormationType.TRIANGLE:
            return FormationCalculator._triangle(num_followers, spacing_along, spacing_cross)
        if ftype == FormationType.COLUMN:
            return FormationCalculator._column(num_followers, spacing_along)
        if ftype == FormationType.S_SHAPE:
            return FormationCalculator._s_shape(num_followers, spacing_along)
        if ftype == FormationType.ESCORT:
            return FormationCalculator._escort(num_followers, spacing_along)
        return FormationCalculator._v_shape(num_followers, spacing_along, spacing_cross)

    @staticmethod
    def _v_shape(n: int, dx: float, dy: float) -> List[FormationOffset]:
        offsets = []
        for i in range(n):
            row = (i // 2) + 1
            cross = -row * dy if i % 2 == 0 else row * dy
            offsets.append(FormationOffset(along=-row * dx, cross=cross))
        return offsets

    @staticmethod
    def _line(n: int, dy: float) -> List[FormationOffset]:
        offsets = []
        for i in range(n):
            rank = (i // 2) + 1
            cross = -rank * dy if i % 2 == 0 else rank * dy
            offsets.append(FormationOffset(along=0.0, cross=cross))
        return offsets

    @staticmethod
    def _column(n: int, dx: float) -> List[FormationOffset]:
        return [FormationOffset(along=-(i + 1) * dx, cross=0.0) for i in range(n)]

    @staticmethod
    def _s_shape(n: int, dx: float) -> List[FormationOffset]:
        return [FormationOffset(along=-(i + 1) * dx, cross=0.0) for i in range(n)]

    @staticmethod
    def _escort(n: int, radius: float) -> List[FormationOffset]:
        offsets = []
        for i in range(n):
            angle = math.pi + 2.0 * math.pi * i / max(n, 1)
            offsets.append(FormationOffset(along=radius * math.cos(angle), cross=radius * math.sin(angle)))
        return offsets

    @staticmethod
    def _diamond(n: int, dx: float, dy: float) -> List[FormationOffset]:
        offsets = []
        row = 1
        placed = 0
        while placed < n:
            if row % 2 == 1:
                if placed < n:
                    offsets.append(FormationOffset(along=-row * dx, cross=-dy))
                    placed += 1
                if placed < n:
                    offsets.append(FormationOffset(along=-row * dx, cross=dy))
                    placed += 1
            else:
                if placed < n:
                    offsets.append(FormationOffset(along=-row * dx, cross=0.0))
                    placed += 1
            row += 1
        return offsets

    @staticmethod
    def _triangle(n: int, dx: float, dy: float) -> List[FormationOffset]:
        offsets = []
        placed = 0
        row = 1
        while placed < n:
            count_in_row = row + 1
            total_width = (count_in_row - 1) * dy
            start_cross = -total_width / 2.0
            for j in range(count_in_row):
                if placed >= n:
                    break
                offsets.append(FormationOffset(along=-row * dx, cross=start_cross + j * dy))
                placed += 1
            row += 1
        return offsets


class FormationFollowerNode(Node):
    """USV / 异构载具端编队跟随节点"""

    DEVIATION_THRESHOLD = 5.0         # 偏差警告阈值 (m)
    DEVIATION_WARN_INTERVAL = 5.0     # 偏差警告限频 (s)
    TARGET_UPDATE_THRESHOLD = 0.05    # 目标更新最小变化量 (m)
    S_SHAPE_PERIOD = 10.0             # S形摆动周期 (s)
    S_SHAPE_PHASE_STEP = math.pi / 2  # S形相位步进 (rad)
    SPEED_GAIN_DEVIATION = 0.15       # 偏差追赶增益
    SPEED_MAX_MULTIPLIER = 2.0        # 最大速度倍率
    SPEED_MIN = 0.10                  # 最小速度 (m/s)
    SPEED_UPDATE_INTERVAL = 0.5       # 速度更新间隔 (s)
    YAW_RATE_SMOOTHING = 0.3          # 角速度 EMA 平滑系数
    DEFAULT_CRUISE_SPEED = 0.60       # 默认巡航速度 (m/s)

    def __init__(self):
        super().__init__('formation_follower_node')
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('usv_id', '')
        self.declare_parameter('platform_mode', '2d')
        self.declare_parameter('vehicle_type', 'usv_diff')

        param_usv_id = self.get_parameter('usv_id').get_parameter_value().string_value.strip()
        ns_usv_id = self.get_namespace().strip('/')
        self.usv_id: str = param_usv_id if param_usv_id else ns_usv_id

        raw_pmode = str(self.get_parameter('platform_mode').value).strip().lower()
        self.vehicle_type: str = str(self.get_parameter('vehicle_type').value).strip().lower()
        if raw_pmode in ('2d', '3d'):
            self.platform_mode = raw_pmode
        else:
            self.platform_mode = '2d' if self.vehicle_type in ('usv', 'usv_diff') else '3d'

        if not self.usv_id:
            self.get_logger().error('无法确定 usv_id（参数为空且未设置命名空间），编队节点不启用')
            return

        self.get_logger().info(
            f'编队跟随节点启动: {self.usv_id} (platform_mode={self.platform_mode}, vehicle_type={self.vehicle_type})'
        )

        # 编队状态
        self._active = False
        self._leader_id: str = ''
        self._my_index: int = -1
        self._offsets: List[FormationOffset] = []
        self._formation_type: int = 0
        self._spacing_along: float = 1.0
        self._spacing_cross: float = 1.0
        self._leader_timeout: float = 3.0
        self._follower_speed: float = 0.0
        self._update_rate: float = 10.0

        # 领队状态
        self._leader_x: float = 0.0
        self._leader_y: float = 0.0
        self._leader_z: float = 0.0
        self._leader_yaw: float = 0.0
        self._leader_vx: float = 0.0
        self._leader_vy: float = 0.0
        self._leader_vz: float = 0.0
        self._leader_speed: float = 0.0
        self._leader_last_update: float = 0.0
        self._leader_has_data: bool = False

        # 角速度跟踪
        self._last_leader_yaw: Optional[float] = None
        self._last_yaw_time: float = 0.0
        self._leader_yaw_rate: float = 0.0

        # 自身状态
        self._my_x: float = 0.0
        self._my_y: float = 0.0
        self._my_z: float = 0.0
        self._my_yaw: float = 0.0
        self._last_target: Optional[Tuple[float, float, float]] = None
        self._deviation: float = 0.0
        self._last_deviation_warn_time: float = 0.0
        self._goal_step: int = 0
        self._original_speed: float = self.DEFAULT_CRUISE_SPEED
        self._last_speed_update_time: float = 0.0
        self._s_shape_start_time: float = 0.0

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.config_sub = self.create_subscription(
            FormationConfig,
            'formation/config',
            self._config_callback,
            qos_transient,
            callback_group=self.callback_group,
        )
        self.leader_state_sub = self.create_subscription(
            UsvStatus,
            'formation/leader_state',
            self._leader_state_callback,
            qos_reliable,
            callback_group=self.callback_group,
        )
        self.own_state_sub = self.create_subscription(
            UsvStatus,
            'usv_state',
            self._own_state_callback,
            qos_reliable,
            callback_group=self.callback_group,
        )

        self.nav_goal_pub = self.create_publisher(
            NavigationGoal,
            'set_usv_nav_goal',
            qos_reliable,
        )
        self.speed_pub = self.create_publisher(
            Float32,
            'set_velocity_cruise_speed',
            qos_reliable,
        )
        self.stop_nav_pub = self.create_publisher(
            Bool,
            'stop_navigation',
            qos_reliable,
        )
        self.status_pub = self.create_publisher(
            String,
            'formation/follower_status',
            qos_reliable,
        )

        self._formation_timer = None
        self.get_logger().info('编队跟随节点初始化完成，等待编队配置...')

    def _config_callback(self, msg: FormationConfig) -> None:
        self.get_logger().info(
            f'收到编队配置: leader={msg.leader_id}, '
            f'followers={list(msg.follower_ids)}, '
            f'type={msg.formation_type}, active={msg.active}'
        )

        if not msg.active:
            if self._active:
                self._stop_formation('GS 停止指令')
            return

        if self.usv_id == msg.leader_id:
            self.get_logger().info(f'本机 {self.usv_id} 是领队，无需跟随')
            if self._active:
                self._stop_formation('角色变更为领队')
            return

        if self.usv_id not in msg.follower_ids:
            if self._active:
                self._stop_formation('已从编队移除')
            return

        self._my_index = list(msg.follower_ids).index(self.usv_id)
        self._leader_id = msg.leader_id
        self._formation_type = int(msg.formation_type)
        self._spacing_along = max(0.3, float(msg.spacing_along))
        self._spacing_cross = max(0.3, float(msg.spacing_cross))
        self._update_rate = max(1.0, min(50.0, float(msg.update_rate) if msg.update_rate > 0 else 10.0))
        self._leader_timeout = max(1.0, float(msg.leader_timeout) if msg.leader_timeout > 0 else 5.0)
        self._follower_speed = float(msg.follower_speed)

        self._offsets = FormationCalculator.compute_offsets(
            self._formation_type,
            len(msg.follower_ids),
            self._spacing_along,
            self._spacing_cross,
        )

        if self._my_index >= len(self._offsets):
            self.get_logger().error(
                f'偏移量不足: index={self._my_index}, offsets={len(self._offsets)}'
            )
            return

        my_offset = self._offsets[self._my_index]
        self.get_logger().info(
            f'编队角色: 跟随者 #{self._my_index}, '
            f'偏移 along={my_offset.along:.2f}m cross={my_offset.cross:.2f}m, '
            f'领队={self._leader_id}'
        )

        if self._active:
            # 运行中动态切换队形/间距：平滑更新，不重置停止信号
            self._last_target = None
            return

        self._start_formation()

    def _start_formation(self) -> None:
        self._last_target = None
        self._deviation = 0.0
        self._last_deviation_warn_time = 0.0
        self._last_speed_update_time = 0.0
        self._last_leader_yaw = None
        self._leader_yaw_rate = 0.0
        self._s_shape_start_time = self._now()

        self._goal_step += 1
        if self._goal_step > 99999:
            self._goal_step = 1

        if self._follower_speed > 0.01:
            self._original_speed = self._follower_speed
            speed_msg = Float32()
            speed_msg.data = float(self._follower_speed)
            self.speed_pub.publish(speed_msg)

        # 清空此前可能残留的普通单点导航目标
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_nav_pub.publish(stop_msg)

        if self._formation_timer is not None:
            self.destroy_timer(self._formation_timer)

        timer_period = 1.0 / self._update_rate
        self._formation_timer = self.create_timer(
            timer_period,
            self._formation_update_callback,
            callback_group=self.callback_group,
        )

        self._active = True
        try:
            fname = FormationType(self._formation_type).name
        except ValueError:
            fname = str(self._formation_type)
        self.get_logger().info(
            f'✅ 编队跟随启动: 领队={self._leader_id}, 更新频率={self._update_rate}Hz, 类型={fname}'
        )
        self._publish_status('started')

    def _stop_formation(self, reason: str) -> None:
        self.get_logger().info(f'编队跟随停止: {reason}')
        self._stop_formation_internal()

        # 停止本地导航目标，使载具立即原地驻留
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_nav_pub.publish(stop_msg)

        # 恢复默认巡航速度
        speed_msg = Float32()
        speed_msg.data = float(self._original_speed)
        self.speed_pub.publish(speed_msg)

        self._publish_status('stopped')

    def _stop_formation_internal(self) -> None:
        self._active = False
        if self._formation_timer is not None:
            self.destroy_timer(self._formation_timer)
            self._formation_timer = None

    def _own_state_callback(self, msg: UsvStatus) -> None:
        self._my_x = float(msg.pose.position.x)
        self._my_y = float(msg.pose.position.y)
        self._my_z = float(msg.pose.position.z)
        self._my_yaw = float(msg.yaw)

    def _leader_state_callback(self, msg: UsvStatus) -> None:
        self._leader_x = float(msg.pose.position.x)
        self._leader_y = float(msg.pose.position.y)
        self._leader_z = float(msg.pose.position.z)
        self._leader_yaw = float(msg.yaw)
        self._leader_vx = float(msg.twist.linear.x)
        self._leader_vy = float(msg.twist.linear.y)
        self._leader_vz = float(msg.twist.linear.z)
        self._leader_speed = math.hypot(self._leader_vx, self._leader_vy)
        self._leader_last_update = self._now()
        self._leader_has_data = True

    def _formation_update_callback(self) -> None:
        if not self._active:
            return

        now = self._now()
        if not self._leader_has_data or self._leader_last_update <= 0.0:
            return

        time_since_update = now - self._leader_last_update
        if time_since_update > self._leader_timeout:
            self.get_logger().warn(
                f'⚠️ 领队 {self._leader_id} 状态超时 ({time_since_update:.1f}s)，自动停止编队跟随'
            )
            self._stop_formation('领队超时')
            return

        # 航向角变化率 (EMA 平滑)
        if self._last_leader_yaw is not None:
            dt = now - self._last_yaw_time
            if dt > 0.01:
                dyaw = self._leader_yaw - self._last_leader_yaw
                while dyaw > math.pi:
                    dyaw -= 2.0 * math.pi
                while dyaw < -math.pi:
                    dyaw += 2.0 * math.pi
                raw_yaw_rate = dyaw / dt
                alpha = self.YAW_RATE_SMOOTHING
                self._leader_yaw_rate = alpha * raw_yaw_rate + (1.0 - alpha) * self._leader_yaw_rate
        self._last_leader_yaw = self._leader_yaw
        self._last_yaw_time = now

        if self._my_index < 0 or self._my_index >= len(self._offsets):
            return

        offset = self._offsets[self._my_index]

        # 限制外推窗口，防止短时抖动造成目标跳变
        dt_extrap = max(0.0, min(time_since_update, 0.5))
        est_leader_x = self._leader_x + self._leader_vx * dt_extrap
        est_leader_y = self._leader_y + self._leader_vy * dt_extrap
        est_leader_z = self._leader_z + self._leader_vz * dt_extrap
        leader_yaw = self._leader_yaw

        effective_cross = offset.cross
        if self._formation_type == int(FormationType.S_SHAPE):
            elapsed = now - self._s_shape_start_time
            omega = 2.0 * math.pi / self.S_SHAPE_PERIOD
            phase = self._my_index * self.S_SHAPE_PHASE_STEP
            effective_cross += self._spacing_cross * math.sin(omega * elapsed - phase)

        # 领队局部系 (along 前, cross 右) -> ENU 全局系 (X 东, Y 北, yaw 从东逆时针为正)
        target_x = (
            est_leader_x
            + offset.along * math.cos(leader_yaw)
            + effective_cross * math.sin(leader_yaw)
        )
        target_y = (
            est_leader_y
            + offset.along * math.sin(leader_yaw)
            - effective_cross * math.cos(leader_yaw)
        )

        # Z 高度策略：2D 水面船固定 0.0；3D 载具跟随空中领队高度，若领队在水面则保持安全飞行高度
        if self.platform_mode == '3d':
            if est_leader_z > 0.3:
                target_z = est_leader_z
            else:
                target_z = max(self._my_z, 1.5)
        else:
            target_z = 0.0

        self._deviation = math.hypot(self._my_x - target_x, self._my_y - target_y)
        if self._deviation > self.DEVIATION_THRESHOLD:
            if now - self._last_deviation_warn_time >= self.DEVIATION_WARN_INTERVAL:
                self.get_logger().warn(
                    f'⚠️ 编队偏差较大: {self._deviation:.2f}m (阈值 {self.DEVIATION_THRESHOLD}m)'
                )
                self._last_deviation_warn_time = now

        # 目标变化极小时跳过重发，但当跟随者距编队位超过到点阈值时持续保活（避免控制端判定到达后停船）
        if self._last_target is not None and self._deviation < 0.45:
            dx = target_x - self._last_target[0]
            dy = target_y - self._last_target[1]
            dz = target_z - self._last_target[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self.TARGET_UPDATE_THRESHOLD:
                if now - self._last_speed_update_time >= self.SPEED_UPDATE_INTERVAL:
                    self._last_speed_update_time = now
                    self._update_speed()
                return

        self._send_nav_goal(target_x, target_y, target_z, leader_yaw)
        self._last_target = (target_x, target_y, target_z)

        if now - self._last_speed_update_time >= self.SPEED_UPDATE_INTERVAL:
            self._last_speed_update_time = now
            self._update_speed()

    def _send_nav_goal(self, x: float, y: float, z: float, yaw: float) -> None:
        try:
            usv_num = int(self.usv_id.split('_')[-1])
        except (ValueError, IndexError):
            usv_num = 0
        goal_id = usv_num * 100000 + self._goal_step

        try:
            fname = FormationType(self._formation_type).name
        except ValueError:
            fname = str(self._formation_type)

        goal = NavigationGoal()
        goal.task_name = f'formation_follow_{fname}'
        goal.goal_id = int(goal_id)

        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = float(z)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)

        goal.enable_yaw = False
        goal.nav_mode = 0  # NAV_MODE_ASYNC
        goal.sync_timeout = 0.0
        goal.arrival_quality_threshold = 0.8
        goal.maneuver_type = 0
        goal.maneuver_param = 0.0
        goal.timeout = 120.0
        goal.timestamp = self.get_clock().now().to_msg()

        self.nav_goal_pub.publish(goal)

    def _update_speed(self) -> None:
        if self._my_index < 0 or self._my_index >= len(self._offsets):
            return

        cross_offset = self._offsets[self._my_index].cross
        abs_yaw_rate = abs(self._leader_yaw_rate)

        turn_multiplier = 1.0
        if abs_yaw_rate > 0.01 and self._leader_speed > 0.05:
            turn_radius = self._leader_speed / abs_yaw_rate
            signed_cross = cross_offset * (1.0 if self._leader_yaw_rate > 0 else -1.0)
            follower_radius = turn_radius + signed_cross
            if follower_radius > 0 and turn_radius > 0:
                turn_multiplier = follower_radius / turn_radius

        deviation_boost = 0.0
        if self._deviation > 0.8:
            deviation_boost = (self._deviation - 0.8) * self.SPEED_GAIN_DEVIATION

        base_speed = max(self._leader_speed, self._original_speed if self._deviation > 1.0 else self.SPEED_MIN)
        target_speed = base_speed * turn_multiplier + deviation_boost
        max_speed = max(self._leader_speed, self._original_speed, 0.5) * self.SPEED_MAX_MULTIPLIER
        target_speed = max(self.SPEED_MIN, min(target_speed, max_speed))

        speed_msg = Float32()
        speed_msg.data = float(target_speed)
        self.speed_pub.publish(speed_msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self, status: str) -> None:
        info = {
            'usv_id': self.usv_id,
            'status': status,
            'leader_id': self._leader_id,
            'formation_type': self._formation_type,
            'my_index': self._my_index,
            'deviation': round(self._deviation, 3),
            'active': self._active,
        }
        msg = String()
        msg.data = json.dumps(info)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FormationFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
