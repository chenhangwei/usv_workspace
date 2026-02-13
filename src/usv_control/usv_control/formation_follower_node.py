#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
#
# This file is part of the USV Workspace project.
#
# USV-side formation follower node.
# Runs on each USV, subscribes to leader state via domain_bridge relay,
# computes formation target locally at high frequency, and sends
# NavigationGoal directly to the local velocity_controller_node.
#
# Architecture (Scheme A):
#   GS (Domain 99) --FormationConfig--> bridge --> Follower USV (Domain 12/13)
#   Leader USV --> usv_state --> bridge --> GS (Domain 99) --> bridge --> Follower USV
#   Follower USV's formation_follower_node:
#     - Receives one-time FormationConfig from GS
#     - Subscribes to leader's usv_state (bridged)
#     - Computes target coordinates at 10Hz locally
#     - Publishes NavigationGoal to local set_usv_nav_goal (zero latency)
#     - Dynamically adjusts cruise speed for turns and deviation catch-up
#
# Author: chenhangwei
# Date: 2026-02-14
"""
编队跟随节点 (USV 端)

该节点运行在每艘 USV 上，负责：
1. 接收 GS 下发的编队配置 (FormationConfig)
2. 订阅领队 USV 的状态 (通过 domain_bridge 中转)
3. 在本地高频计算编队目标坐标
4. 直接发送导航目标给本地速度控制器

优势：
- 消除 GS 中转延迟 (原方案: USV→GS→USV, ~1.2-1.5s)
- 本地 10Hz 更新频率，编队跟随更精确
- GS 仅需一次性下发配置，减少通信量
"""

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from common_interfaces.msg import FormationConfig, NavigationGoal, UsvStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, Float32, String


# =============================================================================
# 编队类型枚举 (与 GS 端 FormationType 保持一致)
# =============================================================================

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


# =============================================================================
# 编队偏移量计算器 (从 GS 端 FormationCalculator 移植)
# =============================================================================

class FormationCalculator:
    """
    静态方法类，计算各编队类型中每个跟随者的偏移量。
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
        """根据编队类型计算所有跟随者的偏移量"""
        calculators = {
            FormationType.V_SHAPE: FormationCalculator._v_shape,
            FormationType.LINE: FormationCalculator._line,
            FormationType.COLUMN: FormationCalculator._column,
            FormationType.S_SHAPE: FormationCalculator._s_shape,
            FormationType.DIAMOND: FormationCalculator._diamond,
            FormationType.TRIANGLE: FormationCalculator._triangle,
            FormationType.ESCORT: FormationCalculator._escort,
        }
        calc_func = calculators.get(FormationType(formation_type), FormationCalculator._line)
        return calc_func(num_followers, spacing_along, spacing_cross)

    @staticmethod
    def _v_shape(n: int, sa: float, sc: float) -> List[FormationOffset]:
        """人字形: 跟随者交替排列在领队两侧偏后方
        偶数序号在左侧(cross<0)，奇数序号在右侧(cross>0)"""
        offsets = []
        for i in range(n):
            row = (i // 2) + 1
            if i % 2 == 0:
                cross = -row * sc  # 左侧
            else:
                cross = row * sc   # 右侧
            offsets.append(FormationOffset(along=-row * sa, cross=cross))
        return offsets

    @staticmethod
    def _line(n: int, sa: float, sc: float) -> List[FormationOffset]:
        """横排一字形: 跟随者横向排列，与领队航向垂直
        偶数序号在左侧(cross<0)，奇数序号在右侧(cross>0)"""
        offsets = []
        for i in range(n):
            rank = (i // 2) + 1
            if i % 2 == 0:
                cross = -rank * sc  # 左侧
            else:
                cross = rank * sc   # 右侧
            offsets.append(FormationOffset(along=0.0, cross=cross))
        return offsets

    @staticmethod
    def _column(n: int, sa: float, sc: float) -> List[FormationOffset]:
        """纵列一字形: 跟随者沿领队航向方向纵向排列在正后方"""
        return [FormationOffset(along=-(i + 1) * sa, cross=0.0) for i in range(n)]

    @staticmethod
    def _s_shape(n: int, sa: float, sc: float) -> List[FormationOffset]:
        """S形: 基础偏移为纵列，运行时叠加动态正弦摆动"""
        return [FormationOffset(along=-(i + 1) * sa, cross=0.0) for i in range(n)]

    @staticmethod
    def _escort(n: int, sa: float, sc: float) -> List[FormationOffset]:
        """护卫: 跟随者环绕领队"""
        offsets = []
        for i in range(n):
            angle = 2.0 * math.pi * i / max(n, 1) + math.pi  # 从后方开始
            r = max(sa, sc)
            offsets.append(FormationOffset(along=r * math.cos(angle), cross=r * math.sin(angle)))
        return offsets

    @staticmethod
    def _diamond(n: int, sa: float, sc: float) -> List[FormationOffset]:
        """菱形: 前-左-右-后"""
        positions = [
            FormationOffset(along=sa, cross=0.0),       # 前
            FormationOffset(along=0.0, cross=-sc),      # 左
            FormationOffset(along=0.0, cross=sc),       # 右
            FormationOffset(along=-sa, cross=0.0),      # 后
        ]
        return positions[:n]

    @staticmethod
    def _triangle(n: int, sa: float, sc: float) -> List[FormationOffset]:
        """三角形: 左后-右后-正后"""
        positions = [
            FormationOffset(along=-sa, cross=-sc),      # 左后
            FormationOffset(along=-sa, cross=sc),        # 右后
            FormationOffset(along=-2 * sa, cross=0.0),   # 正后
        ]
        return positions[:n]


# =============================================================================
# 编队跟随节点
# =============================================================================

class FormationFollowerNode(Node):
    """
    USV 端编队跟随节点

    生命周期：
    1. 启动时等待 FormationConfig (TRANSIENT_LOCAL QoS)
    2. 收到配置后判断自身角色 (领队/跟随者/无关)
    3. 若为跟随者: 订阅领队 usv_state, 启动 10Hz 编队计算定时器
    4. 持续追踪领队，动态调整速度
    5. 收到 active=false 的配置时停止跟随
    """

    # ==================== 控制参数 ====================
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
    DEFAULT_CRUISE_SPEED = 0.35       # 默认巡航速度 (m/s)

    def __init__(self):
        super().__init__('formation_follower_node')
        self.callback_group = ReentrantCallbackGroup()

        # ==================== 参数 ====================
        self.declare_parameter('usv_id', '')
        self.usv_id: str = self.get_parameter('usv_id').get_parameter_value().string_value

        if not self.usv_id:
            self.get_logger().error('参数 usv_id 未设置，无法启动')
            return

        self.get_logger().info(f'编队跟随节点启动: {self.usv_id}')

        # ==================== 编队状态 ====================
        self._active = False
        self._leader_id: str = ''
        self._my_index: int = -1           # 在 follower_ids 中的索引
        self._offsets: List[FormationOffset] = []
        self._formation_type: int = 0
        self._spacing_along: float = 1.0
        self._spacing_cross: float = 1.0
        self._leader_timeout: float = 3.0
        self._follower_speed: float = 0.0
        self._update_rate: float = 10.0

        # ==================== 领队状态 ====================
        self._leader_x: float = 0.0
        self._leader_y: float = 0.0
        self._leader_yaw: float = 0.0
        self._leader_speed: float = 0.0
        self._leader_last_update: float = 0.0
        self._leader_connected: bool = False

        # ==================== 角速度跟踪 ====================
        self._last_leader_yaw: Optional[float] = None
        self._last_yaw_time: float = 0.0
        self._leader_yaw_rate: float = 0.0

        # ==================== 跟随状态 ====================
        self._my_x: float = 0.0
        self._my_y: float = 0.0
        self._my_yaw: float = 0.0
        self._last_target: Optional[Tuple[float, float]] = None
        self._deviation: float = 0.0
        self._last_deviation_warn_time: float = 0.0
        self._goal_step: int = 0
        self._original_speed: float = self.DEFAULT_CRUISE_SPEED
        self._last_speed_update_time: float = 0.0
        self._s_shape_start_time: float = 0.0

        # ==================== QoS 定义 ====================
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ==================== 订阅: 编队配置 (GS 下发, TRANSIENT_LOCAL) ====================
        # 话题: /{usv_id}/formation/config (namespace 下解析为 formation/config)
        self.config_sub = self.create_subscription(
            FormationConfig,
            'formation/config',
            self._config_callback,
            qos_transient,
            callback_group=self.callback_group
        )

        # ==================== 订阅: 领队状态 (GS 中转, domain_bridge 桥接) ====================
        # GS 订阅领队 usv_state 后, 转发到 /{follower_id}/formation/leader_state
        # 话题: /{usv_id}/formation/leader_state (namespace 下解析为 formation/leader_state)
        self.leader_state_sub = self.create_subscription(
            UsvStatus,
            'formation/leader_state',
            self._leader_state_callback,
            qos_reliable,
            callback_group=self.callback_group
        )

        # ==================== 订阅: 自身状态 (本地话题, 无需桥接) ====================
        self.own_state_sub = self.create_subscription(
            UsvStatus,
            'usv_state',
            self._own_state_callback,
            qos_reliable,
            callback_group=self.callback_group
        )

        # ==================== 发布: 导航目标 (本地, 直接给 velocity_controller) ====================
        self.nav_goal_pub = self.create_publisher(
            NavigationGoal,
            'set_usv_nav_goal',
            qos_reliable
        )

        # ==================== 发布: 巡航速度调整 (本地) ====================
        self.speed_pub = self.create_publisher(
            Float32,
            'set_velocity_cruise_speed',
            qos_reliable
        )

        # ==================== 发布: 停止导航 (本地) ====================
        self.stop_nav_pub = self.create_publisher(
            Bool,
            'stop_navigation',
            qos_reliable
        )

        # ==================== 发布: 编队跟随状态 (桥接回 GS) ====================
        self.status_pub = self.create_publisher(
            String,
            'formation/follower_status',
            qos_reliable
        )

        # ==================== 领队速度分量 (用于航位推算) ====================
        self._leader_vx: float = 0.0
        self._leader_vy: float = 0.0

        # ==================== 编队计算定时器 (动态创建) ====================
        self._formation_timer = None

        self.get_logger().info(f'编队跟随节点初始化完成，等待编队配置...')

    # =========================================================================
    # 配置回调
    # =========================================================================

    def _config_callback(self, msg: FormationConfig) -> None:
        """
        处理 GS 下发的编队配置

        收到配置后:
        1. 判断自身角色 (领队 / 跟随者 / 无关)
        2. 若为跟随者: 计算偏移, 订阅领队状态, 启动定时器
        3. 若 active=False: 停止编队跟随
        """
        self.get_logger().info(
            f'收到编队配置: leader={msg.leader_id}, '
            f'followers={msg.follower_ids}, '
            f'type={msg.formation_type}, active={msg.active}'
        )

        # 停止命令
        if not msg.active:
            if self._active:
                self._stop_formation('GS 停止指令')
            return

        # 判断角色
        if self.usv_id == msg.leader_id:
            self.get_logger().info(f'本机 {self.usv_id} 是领队，无需跟随')
            if self._active:
                self._stop_formation('角色变更为领队')
            return

        if self.usv_id not in msg.follower_ids:
            self.get_logger().info(f'本机 {self.usv_id} 不在编队中，忽略')
            if self._active:
                self._stop_formation('已从编队移除')
            return

        # 本机是跟随者
        self._my_index = list(msg.follower_ids).index(self.usv_id)
        self._leader_id = msg.leader_id
        self._formation_type = msg.formation_type
        self._spacing_along = max(0.3, msg.spacing_along)
        self._spacing_cross = max(0.3, msg.spacing_cross)
        self._update_rate = max(1.0, min(50.0, msg.update_rate))
        self._leader_timeout = max(1.0, msg.leader_timeout)
        self._follower_speed = msg.follower_speed

        # 计算编队偏移量
        num_followers = len(msg.follower_ids)
        self._offsets = FormationCalculator.compute_offsets(
            self._formation_type,
            num_followers,
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

        # 如果已经在跟随 (重新配置), 先停止再重启
        if self._active:
            self._stop_formation_internal()

        self._start_formation()

    # =========================================================================
    # 编队启动 / 停止
    # =========================================================================

    def _start_formation(self) -> None:
        """启动编队跟随"""
        # 重置状态
        self._last_target = None
        self._deviation = 0.0
        self._last_deviation_warn_time = 0.0
        self._last_speed_update_time = 0.0
        self._last_leader_yaw = None
        self._leader_yaw_rate = 0.0
        self._s_shape_start_time = self._now()
        self._leader_connected = False

        # 为本次编队会话分配固定 goal_id
        # 每次启动编队递增一次, 确保 velocity_controller 将首个目标视为"新任务"
        # 后续同一会话内的所有更新使用同一个 goal_id → update_waypoint (保留MPC状态)
        self._goal_step += 1
        if self._goal_step > 99999:
            self._goal_step = 1

        # 保存原始巡航速度, 如果配置指定了速度则使用
        if self._follower_speed > 0.01:
            self._original_speed = self._follower_speed
            speed_msg = Float32()
            speed_msg.data = float(self._follower_speed)
            self.speed_pub.publish(speed_msg)

        # 停止当前导航任务 (清空队列)
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_nav_pub.publish(stop_msg)

        # 领队状态已通过 formation/leader_state 固定订阅 (GS domain_bridge 中转)
        # 无需动态订阅, leader_state_sub 在 __init__ 中已创建

        # 启动编队计算定时器
        if self._formation_timer is not None:
            self.destroy_timer(self._formation_timer)

        timer_period = 1.0 / self._update_rate
        self._formation_timer = self.create_timer(
            timer_period,
            self._formation_update_callback,
            callback_group=self.callback_group
        )

        self._active = True
        self.get_logger().info(
            f'✅ 编队跟随启动: 领队={self._leader_id}, '
            f'更新频率={self._update_rate}Hz, '
            f'类型={FormationType(self._formation_type).name}'
        )
        self._publish_status('started')

    def _stop_formation(self, reason: str) -> None:
        """停止编队跟随 (完整停止, 恢复速度)"""
        self.get_logger().info(f'编队跟随停止: {reason}')
        self._stop_formation_internal()

        # 恢复原始巡航速度
        speed_msg = Float32()
        speed_msg.data = float(self._original_speed)
        self.speed_pub.publish(speed_msg)
        self.get_logger().info(f'恢复巡航速度: {self._original_speed:.2f} m/s')

        self._publish_status('stopped')

    def _stop_formation_internal(self) -> None:
        """内部停止 (不恢复速度, 供重新配置使用)"""
        self._active = False

        if self._formation_timer is not None:
            self.destroy_timer(self._formation_timer)
            self._formation_timer = None

    # =========================================================================
    # 状态回调
    # =========================================================================

    def _own_state_callback(self, msg: UsvStatus) -> None:
        """自身状态回调 (本地话题)"""
        self._my_x = msg.position.x
        self._my_y = msg.position.y
        self._my_yaw = msg.yaw

        # 首次收到自身状态时记录巡航速度
        if msg.ground_speed > 0.01 and self._original_speed < 0.01:
            self._original_speed = self.DEFAULT_CRUISE_SPEED

    def _leader_state_callback(self, msg: UsvStatus) -> None:
        """领队状态回调 (通过 domain_bridge 中转)"""
        self._leader_x = msg.position.x
        self._leader_y = msg.position.y
        self._leader_yaw = msg.yaw
        self._leader_connected = msg.connected

        # 保存速度分量 (用于航位推算)
        self._leader_vx = msg.velocity.linear.x
        self._leader_vy = msg.velocity.linear.y
        self._leader_speed = math.sqrt(self._leader_vx ** 2 + self._leader_vy ** 2)

        self._leader_last_update = self._now()

    # =========================================================================
    # 编队核心循环 (10Hz 定时器回调)
    # =========================================================================

    def _formation_update_callback(self) -> None:
        """
        编队更新定时回调 (核心循环)

        每周期执行:
        1. 检查领队状态是否超时
        2. 计算领队航向变化率 (用于转弯补偿)
        3. 计算目标坐标 (领队位置 + 旋转后的偏移)
        4. 发送 NavigationGoal 给本地 velocity_controller
        5. 动态调整巡航速度
        """
        if not self._active:
            return

        now = self._now()

        # 1. 检查领队超时
        if not self._leader_connected or self._leader_last_update == 0.0:
            return  # 尚未收到领队数据

        time_since_update = now - self._leader_last_update
        if time_since_update > self._leader_timeout:
            self.get_logger().warn(
                f'⚠️ 领队 {self._leader_id} 超时 ({time_since_update:.1f}s), 自动停止'
            )
            self._stop_formation('领队超时')
            return

        # 2. 计算领队航向变化率 (EMA 平滑)
        if self._last_leader_yaw is not None:
            dt = now - self._last_yaw_time
            if dt > 0.01:
                dyaw = self._leader_yaw - self._last_leader_yaw
                while dyaw > math.pi:
                    dyaw -= 2 * math.pi
                while dyaw < -math.pi:
                    dyaw += 2 * math.pi
                raw_yaw_rate = dyaw / dt
                alpha = self.YAW_RATE_SMOOTHING
                self._leader_yaw_rate = (
                    alpha * raw_yaw_rate + (1 - alpha) * self._leader_yaw_rate
                )
        self._last_leader_yaw = self._leader_yaw
        self._last_yaw_time = now

        # 3. 获取自身偏移量
        if self._my_index < 0 or self._my_index >= len(self._offsets):
            return

        offset = self._offsets[self._my_index]

        # 航位推算: 在领队状态更新间隔内, 用速度外推领队位置
        # 这使得 10Hz 的控制循环在 1Hz 领队数据下也能平滑跟随
        time_since_update = now - self._leader_last_update
        est_leader_x = self._leader_x + self._leader_vx * time_since_update
        est_leader_y = self._leader_y + self._leader_vy * time_since_update
        leader_yaw = self._leader_yaw  # 航向保持最后已知值

        # S形: 在纵列基础上叠加动态横向正弦摆动
        effective_cross = offset.cross
        if self._formation_type == FormationType.S_SHAPE:
            elapsed = now - self._s_shape_start_time
            omega = 2.0 * math.pi / self.S_SHAPE_PERIOD
            phase = self._my_index * self.S_SHAPE_PHASE_STEP
            effective_cross += self._spacing_cross * math.sin(omega * elapsed - phase)

        # 4. 坐标变换: 编队坐标系 → 全局坐标系 (ENU)
        # 使用航位推算后的领队位置, 提高跟随精度
        # global_x = leader_x + along*cos(yaw) + cross*sin(yaw)
        # global_y = leader_y + along*sin(yaw) - cross*cos(yaw)
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

        # 5. 计算偏差
        self._deviation = math.sqrt(
            (self._my_x - target_x) ** 2 + (self._my_y - target_y) ** 2
        )

        # 偏差过大警告 (限频)
        if self._deviation > self.DEVIATION_THRESHOLD:
            if now - self._last_deviation_warn_time >= self.DEVIATION_WARN_INTERVAL:
                self.get_logger().warn(
                    f'⚠️ 偏差过大: {self._deviation:.2f}m '
                    f'(阈值: {self.DEVIATION_THRESHOLD}m)'
                )
                self._last_deviation_warn_time = now

        # 6. 目标去重 (变化太小不发送)
        if self._last_target is not None:
            dx = target_x - self._last_target[0]
            dy = target_y - self._last_target[1]
            if math.sqrt(dx ** 2 + dy ** 2) < self.TARGET_UPDATE_THRESHOLD:
                # 即使不发新目标, 仍然更新速度
                if now - self._last_speed_update_time >= self.SPEED_UPDATE_INTERVAL:
                    self._last_speed_update_time = now
                    self._update_speed()
                return

        # 7. 发送导航目标
        self._send_nav_goal(target_x, target_y, leader_yaw)
        self._last_target = (target_x, target_y)

        # 8. 动态调整速度 (限频)
        if now - self._last_speed_update_time >= self.SPEED_UPDATE_INTERVAL:
            self._last_speed_update_time = now
            self._update_speed()

    # =========================================================================
    # 导航目标发送
    # =========================================================================

    def _send_nav_goal(self, x: float, y: float, yaw: float) -> None:
        """
        向本地 velocity_controller 发送导航目标

        直接发布到 set_usv_nav_goal, 绕过 navigate_to_point_node 的队列,
        实现零延迟的编队跟随目标更新。

        使用固定 goal_id: 同一次编队会话内 goal_id 不变,
        velocity_controller 会调用 tracker.update_waypoint() 原地更新目标,
        保留 MPC 状态、角速度滤波器和路径历史, 实现平滑跟踪。
        """
        # 仅在编队启动时递增一次 goal_step (见 _start_formation)
        # 后续所有更新使用同一个 goal_id, 让 velocity_controller
        # 走 "same goal_id" 分支 → update_waypoint (而非 set_waypoint)
        try:
            usv_num = int(self.usv_id.split('_')[-1])
        except (ValueError, IndexError):
            usv_num = 0
        goal_id = usv_num * 100000 + self._goal_step

        goal = NavigationGoal()
        goal.task_name = f'formation_follow_{FormationType(self._formation_type).name}'
        goal.goal_id = goal_id

        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # 使用自动朝向 (跟随航向由 MPC 控制器自动计算)
        goal.enable_yaw = False

        # 异步模式 — 持续追踪, 不等待到达确认
        goal.nav_mode = 0  # NAV_MODE_ASYNC
        goal.sync_timeout = 0.0
        goal.arrival_quality_threshold = 0.8

        goal.maneuver_type = 0  # NONE
        goal.maneuver_param = 0.0
        goal.timeout = 120.0  # 编队跟随超时设长
        goal.timestamp = self.get_clock().now().to_msg()

        self.nav_goal_pub.publish(goal)

    # =========================================================================
    # 动态速度调整
    # =========================================================================

    def _update_speed(self) -> None:
        """
        根据转弯半径和偏差动态调整巡航速度

        原理:
        - 转弯时外侧跟随者需要更高速度, 内侧需要更低速度
        - 偏差大时额外加速追赶
        """
        if self._my_index < 0 or self._my_index >= len(self._offsets):
            return

        cross_offset = self._offsets[self._my_index].cross
        abs_yaw_rate = abs(self._leader_yaw_rate)

        # ---- 1. 转弯半径补偿 ----
        turn_multiplier = 1.0
        if abs_yaw_rate > 0.01 and self._leader_speed > 0.05:
            turn_radius = self._leader_speed / abs_yaw_rate
            signed_cross = cross_offset * (
                1.0 if self._leader_yaw_rate > 0 else -1.0
            )
            follower_radius = turn_radius + signed_cross
            if follower_radius > 0 and turn_radius > 0:
                turn_multiplier = follower_radius / turn_radius

        # ---- 2. 偏差追赶补偿 ----
        deviation_boost = 0.0
        if self._deviation > 1.0:
            deviation_boost = (self._deviation - 1.0) * self.SPEED_GAIN_DEVIATION

        # ---- 3. 计算最终速度 ----
        base_speed = max(self._leader_speed, self.SPEED_MIN)
        target_speed = base_speed * turn_multiplier + deviation_boost

        max_speed = max(self._leader_speed, base_speed) * self.SPEED_MAX_MULTIPLIER
        target_speed = max(self.SPEED_MIN, min(target_speed, max_speed))

        # ---- 4. 发布速度 ----
        speed_msg = Float32()
        speed_msg.data = float(target_speed)
        self.speed_pub.publish(speed_msg)

    # =========================================================================
    # 辅助方法
    # =========================================================================

    def _now(self) -> float:
        """当前 ROS 时钟秒值"""
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self, status: str) -> None:
        """发布编队跟随状态 (供 GS 监控)"""
        import json
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
