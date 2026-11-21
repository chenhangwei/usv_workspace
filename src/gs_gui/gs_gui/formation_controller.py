#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of formation controller.
#
# Author: chenhangwei
# Date: 2026-02-13
"""
编队控制器模块

实现 Leader-Follower 编队控制：
- 领队按正常导航任务运动
- 跟随者根据领队实时位置+航向+编队偏移量自动计算目标坐标
- 支持 人字形(V-Shape)、一字形(Line)、菱形(Diamond)、三角形(Triangle)、S形(S-Shape)

坐标系说明：
- 沿领队航向方向为 "along" (前后)
- 垂直于领队航向方向为 "cross" (左右)
- 左侧为负 cross，右侧为正 cross
- 跟随者在领队后方，along 为负值
"""

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict, List, Optional, Tuple

from common_utils import ThreadSafeDict


class FormationType(IntEnum):
    """编队类型枚举"""
    V_SHAPE = 0      # 人字形
    LINE = 1          # 横排一字形 (垂直于航向)
    DIAMOND = 2       # 菱形
    TRIANGLE = 3      # 三角形
    COLUMN = 4        # 纵列一字形 (沿航向方向)
    S_SHAPE = 5       # S形 (纵列+横向正弦摆动)
    ESCORT = 6        # 护卫队形 (环绕领队均匀分布)


@dataclass
class FormationOffset:
    """单个跟随者在编队坐标系中的偏移量"""
    along: float = 0.0   # 沿领队航向方向的偏移 (负值=在后方)
    cross: float = 0.0   # 垂直于航向方向的偏移 (正值=右侧, 负值=左侧)


@dataclass
class FormationState:
    """编队状态"""
    group_id: str = "default"
    active: bool = False
    leader_id: str = ""
    follower_ids: List[str] = field(default_factory=list)
    formation_type: FormationType = FormationType.V_SHAPE
    spacing_along: float = 1.0    # 前后间距 (m)
    spacing_cross: float = 1.0    # 左右间距 (m)
    update_rate: float = 10.0     # 更新频率 (Hz)
    leader_timeout: float = 3.0   # 领队超时 (s)
    follower_speed: float = 0.0   # 跟随者速度 (0=使用默认)

    # 运行时状态
    last_leader_update: float = 0.0
    leader_position: Optional[Tuple[float, float]] = None
    leader_yaw: float = 0.0
    leader_velocity: float = 0.0


class FormationCalculator:
    """
    编队偏移量计算器
    
    根据编队类型和跟随者数量，计算每个跟随者在编队坐标系中的偏移量。
    """

    @staticmethod
    def compute_offsets(
        formation_type: FormationType,
        num_followers: int,
        spacing_along: float,
        spacing_cross: float,
    ) -> List[FormationOffset]:
        """
        计算所有跟随者的编队偏移量
        
        Args:
            formation_type: 编队类型
            num_followers: 跟随者数量
            spacing_along: 前后间距 (m)
            spacing_cross: 左右间距 (m)
            
        Returns:
            每个跟随者的 FormationOffset 列表 (按跟随者顺序)
        """
        if num_followers <= 0:
            return []

        if formation_type == FormationType.V_SHAPE:
            return FormationCalculator._v_shape(num_followers, spacing_along, spacing_cross)
        elif formation_type == FormationType.LINE:
            return FormationCalculator._line(num_followers, spacing_cross)
        elif formation_type == FormationType.DIAMOND:
            return FormationCalculator._diamond(num_followers, spacing_along, spacing_cross)
        elif formation_type == FormationType.TRIANGLE:
            return FormationCalculator._triangle(num_followers, spacing_along, spacing_cross)
        elif formation_type == FormationType.COLUMN:
            return FormationCalculator._column(num_followers, spacing_along)
        elif formation_type == FormationType.S_SHAPE:
            return FormationCalculator._s_shape(num_followers, spacing_along)
        elif formation_type == FormationType.ESCORT:
            return FormationCalculator._escort(num_followers, spacing_along)
        else:
            # 默认使用人字形
            return FormationCalculator._v_shape(num_followers, spacing_along, spacing_cross)

    @staticmethod
    def _v_shape(n: int, dx: float, dy: float) -> List[FormationOffset]:
        """
        人字形 (V-Shape) 编队
        
        布局 (从上往下看，领队在最前方)：
        
                  Leader
                 ↗     ↖
            F1(-dx,-dy)  F2(-dx,+dy)
               ↗             ↖
        F3(-2dx,-2dy)  F4(-2dx,+2dy)
              ↗                  ↖
        F5(-3dx,-3dy)  F6(-3dx,+3dy)
        ...
        
        规则：
        - 奇数编号跟随者在左侧 (cross < 0)
        - 偶数编号跟随者在右侧 (cross > 0)
        - 每一排后退 dx，横向扩展 dy
        """
        offsets = []
        for i in range(n):
            row = (i // 2) + 1  # 第几排 (从1开始)
            if i % 2 == 0:
                # 左侧
                cross = -row * dy
            else:
                # 右侧
                cross = row * dy
            along = -row * dx
            offsets.append(FormationOffset(along=along, cross=cross))
        return offsets

    @staticmethod
    def _line(n: int, dy: float) -> List[FormationOffset]:
        """
        横排一字形 (Line) 编队 — 横向排列，与领队航向垂直
        
        布局：
        F3  F1  Leader  F2  F4
        
        规则：
        - 所有跟随者与领队在同一排 (along=0)
        - 奇数编号在左侧，偶数在右侧
        - 交替排列，间距递增
        """
        offsets = []
        for i in range(n):
            rank = (i // 2) + 1  # 距离中心的排名
            if i % 2 == 0:
                cross = -rank * dy
            else:
                cross = rank * dy
            offsets.append(FormationOffset(along=0.0, cross=cross))
        return offsets

    @staticmethod
    def _column(n: int, dx: float) -> List[FormationOffset]:
        """
        纵列一字形 (Column) 编队 — 沿领队航向方向纵向排列
        
        布局 (从上往下看，领队在最前方)：
        
            Leader
              F1    (-dx, 0)
              F2    (-2dx, 0)
              F3    (-3dx, 0)
              ...
        
        规则：
        - 所有跟随者在领队正后方 (cross=0)
        - 按顺序递增间距
        """
        offsets = []
        for i in range(n):
            along = -(i + 1) * dx
            offsets.append(FormationOffset(along=along, cross=0.0))
        return offsets

    @staticmethod
    def _s_shape(n: int, dx: float) -> List[FormationOffset]:
        """
        S形 (S-Shape) 编队 — 纵列一字形的动态变体
        
        静态基础偏移与 Column 相同 (所有跟随者在领队正后方)，
        动态横向正弦摆动在 FormationController._formation_update_callback() 中叠加。

        布局基础 (从上往下看)：
        
            Leader
              F1    (-dx, 0) + sin 摆动
              F2    (-2dx, 0) + sin 摆动
              F3    (-3dx, 0) + sin 摆动
              ...
        
        参数说明：
        - spacing_along (dx): 纵向间距，与 Column 一致
        - spacing_cross: 在运行时作为正弦摆动的振幅
        - 摆动周期和相位差由 FormationController 常量控制
        """
        offsets = []
        for i in range(n):
            along = -(i + 1) * dx
            offsets.append(FormationOffset(along=along, cross=0.0))
        return offsets

    @staticmethod
    def _escort(n: int, radius: float) -> List[FormationOffset]:
        """
        护卫 (Escort) 队形 — 跟随者均匀环绕领队
        
        布局 (从上往下看，领队在中心)：
        
                  F3 (前方)
                ↗      ↖
            F2            F4
              ↖          ↗
                 Leader
              ↗          ↖
            F1            F5
                ↖      ↗
                  F6 (后方)
        
        规则：
        - spacing_along 作为环绕半径
        - 所有跟随者均匀分布在以领队为圆心的圆上
        - 从正后方 (along=-radius) 开始，逆时针均匀分布
        - 角间距 = 2π / n
        """
        offsets = []
        for i in range(n):
            angle = math.pi + 2.0 * math.pi * i / n  # 从正后方起始
            along = radius * math.cos(angle)
            cross = radius * math.sin(angle)
            offsets.append(FormationOffset(along=along, cross=cross))
        return offsets

    @staticmethod
    def _diamond(n: int, dx: float, dy: float) -> List[FormationOffset]:
        """
        菱形 (Diamond) 编队
        
        布局 (适合奇数个跟随者；偶数时最后一排可能不对称)：
        
              Leader
             ↗     ↖
           F1       F2
          ↗    ↖  ↗    ↖
        F4    F3(后方中心)  F5
                ↖  ↗
                 F6
        
        实际实现为菱形网格：
        - 第1排 (2人): (-dx, ±dy)
        - 第2排 (1人): (-2dx, 0) — 中心
        - 第3排 (2人): (-3dx, ±dy)
        - 第4排 (1人): (-4dx, 0)
        - ...以此类推
        """
        offsets = []
        row = 1
        placed = 0
        while placed < n:
            if row % 2 == 1:
                # 奇数排：左右各一个
                # 左
                if placed < n:
                    offsets.append(FormationOffset(along=-row * dx, cross=-dy))
                    placed += 1
                # 右
                if placed < n:
                    offsets.append(FormationOffset(along=-row * dx, cross=dy))
                    placed += 1
            else:
                # 偶数排：中心一个
                if placed < n:
                    offsets.append(FormationOffset(along=-row * dx, cross=0.0))
                    placed += 1
            row += 1
        return offsets

    @staticmethod
    def _triangle(n: int, dx: float, dy: float) -> List[FormationOffset]:
        """
        三角形 (Triangle) 编队 — 倒三角形，领队在顶点
        
        布局：
                Leader
               ↗     ↖
             F1       F2
            ↗    ↖  ↗    ↖
          F3    F4    F5
         ↗  ↖  ↗  ↖  ↗  ↖
        F6  F7  F8  F9
        
        规则：
        - 第 k 排有 (k+1) 个位置，但领队占了第0排
        - 第 1 排：2人, 第 2 排：3人, 第 3 排：4人...
        - 每排均匀分布，中心对齐
        """
        offsets = []
        placed = 0
        row = 1  # 从第1排开始 (领队在第0排)
        while placed < n:
            count_in_row = row + 1  # 第 row 排可容纳的人数
            # 计算这排的起始横向位置 (居中对齐)
            total_width = (count_in_row - 1) * dy
            start_cross = -total_width / 2.0
            
            for j in range(count_in_row):
                if placed >= n:
                    break
                cross = start_cross + j * dy
                along = -row * dx
                offsets.append(FormationOffset(along=along, cross=cross))
                placed += 1
            row += 1
        return offsets


class FormationController:
    """
    编队控制器
    
    负责：
    1. 管理编队配置状态
    2. 订阅领队状态并计算跟随者目标点
    3. 通过 NavigationGoal 话题下发跟随者目标
    4. 安全保护 (领队超时/跟随者偏差过大)
    """

    # 安全距离 (m)：跟随者间距小于此值时警告
    SAFETY_DISTANCE = 0.5
    # 偏差阈值 (m)：跟随者与目标偏差超过此值时警告
    DEVIATION_THRESHOLD = 5.0
    # 目标更新最小距离阈值 (m)：新目标与上次发送目标距离低于此值时跳过发送，防止刷屏
    TARGET_UPDATE_THRESHOLD = 0.05
    # 偏差警告限频间隔 (s)：同一跟随者偏差过大警告最小打印间隔
    DEVIATION_WARN_INTERVAL = 5.0
    # S形编队参数
    S_SHAPE_PERIOD = 10.0        # S形摆动周期 (秒)，一个完整正弦周期
    S_SHAPE_PHASE_STEP = math.pi / 2  # 相邻跟随者之间的相位差 (rad)

    # 跟随者动态调速参数
    SPEED_GAIN_DEVIATION = 0.15    # 偏差追赶增益: 每米偏差增加的速度 (m/s per m)
    SPEED_MAX_MULTIPLIER = 2.0     # 最大速度倍率 (相对于领队速度)
    SPEED_MIN = 0.10               # 最小巡航速度 (m/s)
    SPEED_UPDATE_INTERVAL = 0.5    # 速度更新最小间隔 (s)
    YAW_RATE_SMOOTHING = 0.3       # 航向变化率平滑系数 (EMA alpha)

    def __init__(self, node, group_id: str = "default"):
        """
        初始化编队控制器
        
        Args:
            node: GroundStationNode 实例
            group_id: 编队组 ID
        """
        self.node = node
        self.group_id = group_id
        self.state = FormationState(group_id=group_id)
        self._offsets: List[FormationOffset] = []  # 缓存的偏移量
        self._timer = None  # 领队状态中转定时器
        self._follower_deviation = ThreadSafeDict()  # 跟随者偏差 (来自 follower_status)
        self._s_shape_start_time: float = 0.0  # S形编队启动时间

        # ===== 方案 A: USV 端计算 — GS 仅做配置下发 + 领队状态中转 =====
        # 编队配置发布者 (per follower): /{follower_id}/formation/config
        self._config_pubs: Dict = {}
        # 领队状态中转发布者 (per follower): /{follower_id}/formation/leader_state
        self._leader_relay_pubs: Dict = {}
        # 领队状态订阅 (订阅 /{leader_id}/usv_state)
        self._leader_state_sub = None
        # 跟随者状态订阅 (per follower): /{follower_id}/formation/follower_status
        self._follower_status_subs: Dict = {}
        # 领队状态监控
        self._last_leader_yaw: Optional[float] = None
        self._last_yaw_time: float = 0.0
        self._leader_yaw_rate: float = 0.0

    def configure(
        self,
        leader_id: str,
        follower_ids: List[str],
        formation_type: FormationType,
        spacing_along: float = 1.0,
        spacing_cross: float = 1.0,
        update_rate: float = 10.0,
        leader_timeout: float = 3.0,
        follower_speed: float = 0.0,
    ) -> bool:
        """
        配置编队参数
        
        Args:
            leader_id: 领队 USV ID
            follower_ids: 跟随者 USV ID 列表
            formation_type: 编队类型
            spacing_along: 前后间距 (m)
            spacing_cross: 左右间距 (m)
            update_rate: 更新频率 (Hz)
            leader_timeout: 领队超时 (s)
            follower_speed: 跟随者速度 (0=使用默认)
            
        Returns:
            配置是否成功
        """
        if not leader_id:
            self.node.get_logger().error("编队配置失败：未指定领队")
            return False
        if not follower_ids:
            self.node.get_logger().error("编队配置失败：无跟随者")
            return False
        if leader_id in follower_ids:
            self.node.get_logger().error("编队配置失败：领队不能同时是跟随者")
            return False

        self.state.leader_id = leader_id
        self.state.follower_ids = list(follower_ids)
        self.state.formation_type = FormationType(formation_type)
        self.state.spacing_along = max(0.3, spacing_along)  # 最小 0.3m
        self.state.spacing_cross = max(0.3, spacing_cross)
        self.state.update_rate = max(1.0, min(20.0, update_rate))
        self.state.leader_timeout = max(1.0, leader_timeout)
        self.state.follower_speed = max(0.0, follower_speed)

        # 预计算偏移量
        self._offsets = FormationCalculator.compute_offsets(
            self.state.formation_type,
            len(self.state.follower_ids),
            self.state.spacing_along,
            self.state.spacing_cross,
        )

        self.node.get_logger().info(
            f"编队配置完成: 领队={leader_id}, "
            f"跟随者={follower_ids}, "
            f"队形={self.state.formation_type.name}, "
            f"间距=({spacing_along}m, {spacing_cross}m)"
        )
        return True

    def set_leader_path(self, waypoints: List[Dict]) -> bool:
        """
        设置领队导航路径 (已废弃，领队现由集群任务控制)
        
        保留填充方法以兼容旧调用，无实际效果。
        """
        self.node.get_logger().info("编队模式：领队路径已由集群导航任务控制，忽略 set_leader_path 调用")
        return False

    def start(self) -> bool:
        """
        启动编队跟随 (方案 A: USV 端计算)
        
        GS 端仅执行:
        1. 向每个跟随者 USV 发布 FormationConfig
        2. 启动领队 usv_state → follower formation/leader_state 的中转
        3. 订阅跟随者的 formation/follower_status 用于监控
        
        Returns:
            是否成功启动
        """
        if self.state.active:
            self.node.get_logger().warn("编队已在运行中")
            return False

        if not self.state.leader_id or not self.state.follower_ids:
            self.node.get_logger().error("编队未配置，无法启动")
            return False

        # 检查领队是否在线
        leader_status = self.node.usv_states.get(self.state.leader_id)
        if not leader_status or not leader_status.get('connected', False):
            self.node.get_logger().error(f"领队 {self.state.leader_id} 不在线，无法启动编队")
            return False

        self.state.active = True
        self.state.last_leader_update = self._now()
        self._follower_deviation.clear()
        self._last_leader_yaw = None
        self._leader_yaw_rate = 0.0
        self._s_shape_start_time = self._now()

        # ========== 1. 创建 FormationConfig 发布者并发送配置 ==========
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from common_interfaces.msg import FormationConfig as FormationConfigMsg

        qos_transient = QoSProfile(depth=1)
        qos_transient.reliability = ReliabilityPolicy.RELIABLE
        qos_transient.durability = DurabilityPolicy.TRANSIENT_LOCAL

        for fid in self.state.follower_ids:
            config_topic = f'/{fid}/formation/config'
            if fid not in self._config_pubs:
                self._config_pubs[fid] = self.node.create_publisher(
                    FormationConfigMsg, config_topic, qos_transient
                )
            # 发送配置
            self._publish_formation_config(active=True)

        # ========== 2. 创建领队状态中转: leader usv_state → follower leader_state ==========
        from common_interfaces.msg import UsvStatus
        from rclpy.qos import HistoryPolicy

        qos_relay = QoSProfile(depth=10)
        qos_relay.reliability = ReliabilityPolicy.RELIABLE

        # 订阅领队的 usv_state (已通过 domain_bridge 到达 Domain 99)
        leader_topic = f'/{self.state.leader_id}/usv_state'
        if self._leader_state_sub is not None:
            self.node.destroy_subscription(self._leader_state_sub)
        self._leader_state_sub = self.node.create_subscription(
            UsvStatus,
            leader_topic,
            self._relay_leader_state_callback,
            qos_relay,
        )

        # 为每个跟随者创建 leader_state 发布者
        for fid in self.state.follower_ids:
            relay_topic = f'/{fid}/formation/leader_state'
            if fid not in self._leader_relay_pubs:
                self._leader_relay_pubs[fid] = self.node.create_publisher(
                    UsvStatus, relay_topic, qos_relay
                )

        # ========== 3. 订阅跟随者的编队状态反馈 ==========
        from std_msgs.msg import String
        for fid in self.state.follower_ids:
            status_topic = f'/{fid}/formation/follower_status'
            if fid not in self._follower_status_subs:
                self._follower_status_subs[fid] = self.node.create_subscription(
                    String,
                    status_topic,
                    lambda msg, f=fid: self._follower_status_callback(f, msg),
                    10,
                )

        # ========== 4. 启动领队超时监控定时器 (1Hz) ==========
        if self._timer is not None:
            self._timer.cancel()
            self.node.destroy_timer(self._timer)
        self._timer = self.node.create_timer(
            1.0,
            self._leader_timeout_check,
        )

        self.node.get_logger().info(
            f"🚀 编队模式启动 (USV端计算): {self.state.formation_type.name}, "
            f"领队={self.state.leader_id}, "
            f"跟随者={list(self.state.follower_ids)}, "
            f"配置已下发, 领队状态中转已启动"
        )

        # 更新跟随者的导航状态显示为 "跟随中"
        for fid in self.state.follower_ids:
            self.node.ros_signal.nav_status_update.emit(fid, "跟随中")

        self._emit_formation_status("running")
        return True

    def stop(self, reason: str = "手动停止") -> None:
        """停止编队跟随 (方案 A: 通知 USV 端停止)"""
        if not self.state.active:
            return

        self.state.active = False

        # 1. 向跟随者发送 active=false 的配置 (USV 端收到后自动停止)
        self._publish_formation_config(active=False)

        # 2. 停止领队超时监控定时器
        if self._timer is not None:
            self._timer.cancel()
            self.node.destroy_timer(self._timer)
            self._timer = None

        # 3. 停止领队状态中转
        if self._leader_state_sub is not None:
            self.node.destroy_subscription(self._leader_state_sub)
            self._leader_state_sub = None

        # 4. 清理中转发布者
        for pub in self._leader_relay_pubs.values():
            self.node.destroy_publisher(pub)
        self._leader_relay_pubs.clear()

        # 5. 清理配置发布者
        for pub in self._config_pubs.values():
            self.node.destroy_publisher(pub)
        self._config_pubs.clear()

        # 6. 清理跟随者状态订阅
        for sub in self._follower_status_subs.values():
            self.node.destroy_subscription(sub)
        self._follower_status_subs.clear()

        self._follower_deviation.clear()

        # 恢复跟随者的导航状态显示为 "空闲"
        for fid in self.state.follower_ids:
            self.node.ros_signal.nav_status_update.emit(fid, "空闲")

        self.node.get_logger().info(f"编队模式已停止: {reason}")
        self._emit_formation_status("stopped")

    def update_formation_type(self, formation_type: FormationType) -> None:
        """运行中切换队形"""
        self.state.formation_type = formation_type
        self._offsets = FormationCalculator.compute_offsets(
            formation_type,
            len(self.state.follower_ids),
            self.state.spacing_along,
            self.state.spacing_cross,
        )
        # 重新下发配置到 USV 端
        if self.state.active:
            self._publish_formation_config(active=True)
        self.node.get_logger().info(f"编队队形切换为: {formation_type.name}")
        self._emit_formation_status("running")

    def update_spacing(self, spacing_along: float, spacing_cross: float) -> None:
        """运行中调整间距"""
        self.state.spacing_along = max(0.3, spacing_along)
        self.state.spacing_cross = max(0.3, spacing_cross)
        self._offsets = FormationCalculator.compute_offsets(
            self.state.formation_type,
            len(self.state.follower_ids),
            self.state.spacing_along,
            self.state.spacing_cross,
        )
        # 重新下发配置到 USV 端
        if self.state.active:
            self._publish_formation_config(active=True)
        self.node.get_logger().info(
            f"编队间距更新: along={self.state.spacing_along}m, cross={self.state.spacing_cross}m"
        )

    def is_active(self) -> bool:
        """编队是否在运行"""
        return self.state.active

    def get_state(self) -> FormationState:
        """获取编队状态"""
        return self.state

    def get_formation_info(self) -> Dict:
        """获取编队信息 (用于 GUI 显示)"""
        type_names = {
            FormationType.V_SHAPE: "人字形",
            FormationType.LINE: "一字形",
            FormationType.COLUMN: "纵列",
            FormationType.DIAMOND: "菱形",
            FormationType.TRIANGLE: "三角形",
            FormationType.S_SHAPE: "S形",
            FormationType.ESCORT: "护卫",
        }
        return {
            'group_id': self.group_id,
            'active': self.state.active,
            'leader_id': self.state.leader_id,
            'follower_ids': list(self.state.follower_ids),
            'formation_type': self.state.formation_type,
            'formation_type_name': type_names.get(self.state.formation_type, "未知"),
            'spacing_along': self.state.spacing_along,
            'spacing_cross': self.state.spacing_cross,
            'follower_count': len(self.state.follower_ids),
            'offsets': [(o.along, o.cross) for o in self._offsets],
            'deviations': dict(self._follower_deviation),
        }

    # ==================== 内部方法 ====================

    def _now(self) -> float:
        """当前 ROS 时钟秒值"""
        return self.node.get_clock().now().nanoseconds / 1e9

    def _formation_update_callback(self) -> None:
        """
        [已废弃] 原 GS 端编队计算循环，已迁移到 USV 端 formation_follower_node。
        保留空方法以兼容旧代码引用。
        """
        pass

    def _relay_leader_state_callback(self, msg) -> None:
        """
        领队状态中转回调

        订阅领队的 usv_state，原样转发到每个跟随者的 formation/leader_state 话题。
        GS 不做任何计算，仅作为 Domain Bridge 的补充中转。
        """
        # 更新 GS 端监控用的领队状态
        self.state.leader_position = (msg.position.x, msg.position.y)
        self.state.leader_yaw = msg.yaw
        vel = msg.velocity.linear
        self.state.leader_velocity = math.sqrt(vel.x ** 2 + vel.y ** 2)
        self.state.last_leader_update = self._now()

        # 转发到每个跟随者
        for fid, pub in self._leader_relay_pubs.items():
            try:
                pub.publish(msg)
            except Exception as e:
                self.node.get_logger().warn(f"领队状态中转失败 → {fid}: {e}")

    def _leader_timeout_check(self) -> None:
        """
        领队超时监控 (1Hz)

        如果领队状态长时间未更新，通知 USV 端停止编队。
        USV 端也有独立的超时检测，这里是 GS 端的冗余保护。
        """
        if not self.state.active:
            return

        now = self._now()
        if self.state.last_leader_update > 0:
            elapsed = now - self.state.last_leader_update
            if elapsed > self.state.leader_timeout * 2:
                self.node.get_logger().warn(
                    f"⚠️ 领队 {self.state.leader_id} 超时 "
                    f"({elapsed:.1f}s)，GS 端主动停止编队"
                )
                self.stop("领队超时 (GS 端检测)")

    def _publish_formation_config(self, active: bool) -> None:
        """
        向所有跟随者发布/更新 FormationConfig

        使用 TRANSIENT_LOCAL QoS，确保后加入的跟随者也能收到最新配置。
        """
        from common_interfaces.msg import FormationConfig as FormationConfigMsg

        config = FormationConfigMsg()
        config.leader_id = self.state.leader_id
        config.follower_ids = list(self.state.follower_ids)
        config.formation_type = int(self.state.formation_type)
        config.spacing_along = float(self.state.spacing_along)
        config.spacing_cross = float(self.state.spacing_cross)
        config.update_rate = float(self.state.update_rate)
        config.leader_timeout = float(self.state.leader_timeout)
        config.follower_speed = float(self.state.follower_speed)
        config.active = active
        config.timestamp = self.node.get_clock().now().to_msg()

        for fid, pub in self._config_pubs.items():
            try:
                pub.publish(config)
                self.node.get_logger().info(
                    f"编队配置{'下发' if active else '停止'} → {fid}: "
                    f"type={self.state.formation_type.name}, "
                    f"spacing=({self.state.spacing_along}m, {self.state.spacing_cross}m)"
                )
            except Exception as e:
                self.node.get_logger().error(f"编队配置发布失败 → {fid}: {e}")

    def _follower_status_callback(self, follower_id: str, msg) -> None:
        """
        跟随者编队状态反馈回调

        解析跟随者报告的偏差等信息，更新 GS 端监控状态。
        """
        import json
        try:
            info = json.loads(msg.data)
            dev = info.get('deviation', 0.0)
            self._follower_deviation[follower_id] = dev
            status = info.get('status', '')
            if status == 'stopped':
                self.node.get_logger().info(f"跟随者 {follower_id} 报告编队已停止")
        except (json.JSONDecodeError, Exception) as e:
            self.node.get_logger().warn(f"解析跟随者 {follower_id} 状态失败: {e}")

    def _emit_formation_status(self, status: str) -> None:
        """向 GUI 发送编队状态"""
        info = self.get_formation_info()
        info['status'] = status
        info['group_id'] = self.group_id
        self.node.ros_signal.formation_status_update.emit(info)

    # ==================== 队形预览 (静态方法，供 UI 使用) ====================

    @staticmethod
    def get_formation_preview(
        formation_type: FormationType,
        num_followers: int,
        spacing_along: float,
        spacing_cross: float,
    ) -> List[Tuple[float, float]]:
        """
        获取编队预览坐标 (领队在原点)
        
        Returns:
            坐标列表 [(x,y), ...]，第一个为领队 (0,0)，后续为跟随者
        """
        offsets = FormationCalculator.compute_offsets(
            formation_type, num_followers, spacing_along, spacing_cross
        )
        # 在预览中，along 对应 Y 轴 (向上为前), cross 对应 X 轴 (向右为正)
        result = [(0.0, 0.0)]  # 领队
        for idx, off in enumerate(offsets):
            cross = off.cross
            # S形预览：在静态列基础上叠加正弦波快照，让用户看到S形效果
            if formation_type == FormationType.S_SHAPE:
                phase = idx * (math.pi / 2)
                cross = spacing_cross * math.sin(phase)
            result.append((cross, off.along))
        return result


class FormationManager:
    """
    多编队组管理器
    
    管理多个独立的编队组，每组有自己的领队、跟随者、队形和路径。
    每个组使用独立的 FormationController 实例。
    一个 USV 只能属于一个组（作为领队或跟随者）。
    """

    def __init__(self, node):
        self.node = node
        self._groups: Dict[str, FormationController] = {}  # group_id → controller

    def configure_groups(self, group_configs: List[Dict]) -> bool:
        """
        配置所有编队组
        
        Args:
            group_configs: 编队组配置列表，每个元素为 dict，包含:
                group_id, leader_id, follower_ids, formation_type,
                spacing_along, spacing_cross, update_rate, leader_timeout,
                follower_speed
        
        Returns:
            是否所有组配置成功
        """
        # 验证：每个 USV 只能属于一个组
        usv_assignments: Dict[str, str] = {}  # usv_id → group_id
        for cfg in group_configs:
            group_id = cfg.get('group_id', '')
            leader = cfg.get('leader_id', '')
            followers = cfg.get('follower_ids', [])
            for usv_id in [leader] + followers:
                if usv_id in usv_assignments:
                    self.node.get_logger().error(
                        f"USV {usv_id} 同时在编队组 '{usv_assignments[usv_id]}' "
                        f"和 '{group_id}' 中，配置失败"
                    )
                    return False
                usv_assignments[usv_id] = group_id

        # 停止现有组
        self.stop_all("重新配置")
        self._groups.clear()

        # 创建各组控制器
        for cfg in group_configs:
            group_id = cfg.get('group_id', f"group_{len(self._groups) + 1}")
            controller = FormationController(self.node, group_id=group_id)
            success = controller.configure(
                leader_id=cfg.get('leader_id', ''),
                follower_ids=cfg.get('follower_ids', []),
                formation_type=cfg.get('formation_type', 0),
                spacing_along=cfg.get('spacing_along', 1.0),
                spacing_cross=cfg.get('spacing_cross', 1.0),
                update_rate=cfg.get('update_rate', 10.0),
                leader_timeout=cfg.get('leader_timeout', 3.0),
                follower_speed=cfg.get('follower_speed', 0.0),
            )
            if not success:
                self.node.get_logger().error(f"编队组 '{group_id}' 配置失败")
                return False

            self._groups[group_id] = controller

        self.node.get_logger().info(
            f"多编队配置完成: {len(self._groups)} 个组, "
            f"共 {len(usv_assignments)} 艘 USV"
        )
        return True

    def start_all(self) -> bool:
        """启动所有编队组"""
        success = True
        for gid, ctrl in self._groups.items():
            if not ctrl.start():
                self.node.get_logger().error(f"编队组 '{gid}' 启动失败")
                success = False
        return success

    def stop_all(self, reason: str = "手动停止") -> None:
        """停止所有编队组"""
        for ctrl in self._groups.values():
            if ctrl.is_active():
                ctrl.stop(reason)

    def stop_group(self, group_id: str, reason: str = "手动停止") -> None:
        """停止指定编队组"""
        if group_id in self._groups:
            self._groups[group_id].stop(reason)

    def is_any_active(self) -> bool:
        """是否有任何编队组在运行"""
        return any(ctrl.is_active() for ctrl in self._groups.values())

    def get_active_count(self) -> int:
        """获取活跃组数"""
        return sum(1 for ctrl in self._groups.values() if ctrl.is_active())

    def get_group_count(self) -> int:
        """获取总组数"""
        return len(self._groups)

    def get_group(self, group_id: str) -> Optional[FormationController]:
        """获取指定编队组控制器"""
        return self._groups.get(group_id)

    def get_all_groups_info(self) -> List[Dict]:
        """获取所有组信息"""
        return [ctrl.get_formation_info() for ctrl in self._groups.values()]

    def update_formation_type_all(self, formation_type: FormationType) -> None:
        """所有活跃组切换队形"""
        for ctrl in self._groups.values():
            if ctrl.is_active():
                ctrl.update_formation_type(formation_type)

    def update_spacing_all(self, along: float, cross: float) -> None:
        """所有活跃组调整间距"""
        for ctrl in self._groups.values():
            if ctrl.is_active():
                ctrl.update_spacing(along, cross)

    def get_all_follower_ids(self) -> set:
        """获取所有编队组的跟随者 ID 集合"""
        result = set()
        for ctrl in self._groups.values():
            result.update(ctrl.state.follower_ids)
        return result

    def get_all_leader_ids(self) -> set:
        """获取所有编队组的领队 ID 集合"""
        result = set()
        for ctrl in self._groups.values():
            if ctrl.state.leader_id:
                result.add(ctrl.state.leader_id)
        return result

