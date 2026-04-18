#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 Chen Hangwei
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
无人船双目摄像头云台控制节点

该节点负责控制 Pan-Tilt 云台上的两个舵机 (通过 PCA9685 PWM 驱动板)，
实现以下功能:

1. **巡视模式 (PATROL)**: 自动水平扫描 + 垂直阶梯，覆盖前方视野
2. **跟踪模式 (TRACK)**: PID 闭环跟踪视觉检测到的目标
3. **手动模式 (MANUAL)**: 接收外部角度/角速度指令

工作流程:
- 默认进入巡视模式，云台自动扫描
- 当视觉节点检测到目标 (/vision_obstacles)，自动切换到跟踪模式
- 跟踪丢失超时后，自动恢复巡视
- 可通过 /gimbal_command 话题手动控制

硬件连接:
- PCA9685 通过 I2C 连接 (与 usv_head_action_node 共用同一块板)
- Pan 舵机 → CH1 (水平旋转, -90°~+90°)
- Tilt 舵机 → CH2 (垂直俯仰, -45°~+30°)
- 25kg PWM 舵机, 脉宽 500~2500µs, 50Hz
"""

import math
import time
from typing import Any, cast

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from common_interfaces.msg import (
    GimbalCommand, GimbalState, VisionObstacle, VisionObstacleArray)

# ---- 可选硬件依赖 ----
HARDWARE_ADAFRUIT = True
busio = None
SCL = None
SDA = None
PCA9685 = None
try:
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
except Exception:
    HARDWARE_ADAFRUIT = False

# smbus2 后端 (USB-to-I2C, 如 CH341A)
HARDWARE_SMBUS = True
try:
    from usv_drivers.pca9685_smbus import PCA9685SMBus, SMBUS2_AVAILABLE
    if not SMBUS2_AVAILABLE:
        HARDWARE_SMBUS = False
except Exception:
    HARDWARE_SMBUS = False

# ---- 模式常量 ----
MODE_IDLE = 0
MODE_PATROL = 1
MODE_TRACK = 2
MODE_MANUAL = 3

MODE_NAMES = {MODE_IDLE: 'IDLE', MODE_PATROL: 'PATROL',
              MODE_TRACK: 'TRACK', MODE_MANUAL: 'MANUAL'}


class PIDController:
    """简单增量式 PID 控制器"""

    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        self._integral += error * dt
        # 积分饱和限幅
        integral_limit = self.output_limit / max(self.ki, 1e-6)
        self._integral = max(-integral_limit,
                             min(integral_limit, self._integral))
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))


class UsvGimbalNode(Node):
    """无人船云台控制节点"""

    def __init__(self):
        super().__init__('usv_gimbal_node')

        # ---- 参数声明 ----
        # PCA9685 通道
        self.declare_parameter('pan_channel', 1)
        self.declare_parameter('tilt_channel', 2)

        # 角度范围 (度)
        self.declare_parameter('pan_min', -90.0)
        self.declare_parameter('pan_max', 90.0)
        self.declare_parameter('tilt_min', -45.0)
        self.declare_parameter('tilt_max', 30.0)

        # 舵机 PWM 脉宽 (µs)
        self.declare_parameter('pulse_min_us', 500)
        self.declare_parameter('pulse_max_us', 2500)
        self.declare_parameter('pwm_frequency', 50)

        # 运动限制
        self.declare_parameter('max_angular_speed', 60.0)  # 度/秒

        # 控制频率
        self.declare_parameter('control_rate', 20.0)  # Hz

        # 巡视参数
        self.declare_parameter('patrol_pan_speed', 20.0)      # 度/秒
        self.declare_parameter('patrol_pan_range', 80.0)       # 巡视半幅 (度)
        self.declare_parameter('patrol_tilt_angles', [-30.0, 0.0, 15.0])  # 垂直阶梯
        self.declare_parameter('patrol_edge_hold_time', 0.5)   # 到端点后停留 (秒)

        # 跟踪参数
        self.declare_parameter('track_kp', 0.05)
        self.declare_parameter('track_ki', 0.001)
        self.declare_parameter('track_kd', 0.02)
        self.declare_parameter('track_lost_timeout', 3.0)   # 丢失目标超时 (秒)
        self.declare_parameter('track_lock_iou_thresh', 0.3)  # IOU 匹配阈值

        # 图像参数 (与视觉节点一致)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # 目标选择策略: 'nearest', 'largest', 'highest_conf'
        self.declare_parameter('target_selection', 'nearest')

        # 启动模式
        self.declare_parameter('initial_mode', 1)  # 默认巡视

        # 硬件开关
        self.declare_parameter('enable_hardware', True)

        # I2C 后端: 'auto'(先 adafruit 后 smbus), 'adafruit', 'smbus', 'none'
        self.declare_parameter('i2c_backend', 'auto')
        # smbus2 总线编号 (/dev/i2c-N)
        self.declare_parameter('i2c_bus', 0)

        # ---- 读取参数 ----
        self.pan_channel = self.get_parameter('pan_channel').value
        self.tilt_channel = self.get_parameter('tilt_channel').value

        self.pan_min = self.get_parameter('pan_min').value
        self.pan_max = self.get_parameter('pan_max').value
        self.tilt_min = self.get_parameter('tilt_min').value
        self.tilt_max = self.get_parameter('tilt_max').value

        self.pulse_min_us = self.get_parameter('pulse_min_us').value
        self.pulse_max_us = self.get_parameter('pulse_max_us').value
        self.pwm_freq = self.get_parameter('pwm_frequency').value

        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.control_rate = self.get_parameter('control_rate').value

        self.patrol_pan_speed = self.get_parameter('patrol_pan_speed').value
        self.patrol_pan_range = self.get_parameter('patrol_pan_range').value
        self.patrol_tilt_angles = list(
            self.get_parameter('patrol_tilt_angles').value)
        self.patrol_edge_hold = self.get_parameter('patrol_edge_hold_time').value

        self.track_lost_timeout = self.get_parameter('track_lost_timeout').value
        self.track_iou_thresh = self.get_parameter('track_lock_iou_thresh').value

        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.target_selection = self.get_parameter('target_selection').value

        enable_hw = self.get_parameter('enable_hardware').value

        # ---- PID 控制器 ----
        kp = self.get_parameter('track_kp').value
        ki = self.get_parameter('track_ki').value
        kd = self.get_parameter('track_kd').value
        self.pan_pid = PIDController(kp, ki, kd, self.max_angular_speed)
        self.tilt_pid = PIDController(kp, ki, kd, self.max_angular_speed)

        # ---- PCA9685 硬件初始化 ----
        i2c_backend = self.get_parameter('i2c_backend').value
        i2c_bus = self.get_parameter('i2c_bus').value

        self.pca = None
        if enable_hw and i2c_backend != 'none':
            self.pca = self._init_pca9685(i2c_backend, i2c_bus)
        else:
            if not enable_hw:
                self.get_logger().info('参数 enable_hardware=false，以无硬件模式运行')
            if i2c_backend == 'none':
                self.get_logger().info('参数 i2c_backend=none，以无硬件模式运行')

        # ---- 状态变量 ----
        self.current_pan = 0.0     # 当前 Pan 角度 (度)
        self.current_tilt = 0.0    # 当前 Tilt 角度 (度)
        self.mode = self.get_parameter('initial_mode').value

        # 巡视状态
        self._patrol_direction = 1       # 1=向右, -1=向左
        self._patrol_tilt_idx = 0        # 当前垂直阶梯索引
        self._patrol_holding = False     # 端点等待中
        self._patrol_hold_until = 0.0    # 等待结束时刻

        # 跟踪状态
        self._track_target_bbox = None       # 锁定目标的 bbox [x1,y1,x2,y2]
        self._track_target_class = ''
        self._track_target_distance = 0.0
        self._track_last_seen = 0.0          # 上次看到目标的时刻
        self._tracking = False

        # 手动控制目标
        self._manual_target_pan = 0.0
        self._manual_target_tilt = 0.0
        self._manual_vel_pan = 0.0
        self._manual_vel_tilt = 0.0
        self._manual_is_velocity = False

        # 上一个控制周期的时间
        self._last_time = self.get_clock().now().nanoseconds / 1e9

        # ---- ROS 2 订阅/发布 ----
        qos_best = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.obstacle_sub = self.create_subscription(
            VisionObstacleArray, 'vision_obstacles',
            self._on_vision_obstacles, qos_best)

        self.cmd_sub = self.create_subscription(
            GimbalCommand, 'gimbal_command',
            self._on_gimbal_command, 10)

        self.state_pub = self.create_publisher(GimbalState, 'gimbal_state', 10)

        # ---- 控制定时器 ----
        self.timer = self.create_timer(
            1.0 / self.control_rate, self._control_loop)

        # ---- 初始位置 ----
        self._apply_angles(0.0, 0.0)

        mode_str = MODE_NAMES.get(self.mode, '?')
        hw_str = '硬件控制' if self.pca else '无硬件模式'
        self.get_logger().info(
            f'云台节点已启动 ({hw_str}) | 模式={mode_str} '
            f'| Pan CH{self.pan_channel} [{self.pan_min}°,{self.pan_max}°] '
            f'| Tilt CH{self.tilt_channel} [{self.tilt_min}°,{self.tilt_max}°] '
            f'| 频率={self.control_rate}Hz')

    # ==================================================================
    # PCA9685 硬件控制
    # ==================================================================

    def _init_pca9685(self, backend, bus_num):
        """根据后端类型初始化 PCA9685"""
        pca = None

        backends_to_try = []
        if backend == 'auto':
            backends_to_try = ['adafruit', 'smbus']
        elif backend in ('adafruit', 'smbus'):
            backends_to_try = [backend]

        for be in backends_to_try:
            if be == 'adafruit' and HARDWARE_ADAFRUIT:
                try:
                    i2c = cast(Any, busio).I2C(SCL, SDA)
                    pca = cast(Any, PCA9685)(i2c)
                    self._set_pwm_freq_with_retry(self.pwm_freq, pca)
                    self.get_logger().info(
                        'PCA9685 已通过 adafruit (板载 I2C) 初始化')
                    return pca
                except Exception as e:
                    self.get_logger().warn(
                        f'adafruit 后端初始化失败: {e}')

            elif be == 'smbus' and HARDWARE_SMBUS:
                try:
                    pca = PCA9685SMBus(bus=bus_num, address=0x40)
                    self._set_pwm_freq_with_retry(self.pwm_freq, pca)
                    self.get_logger().info(
                        f'PCA9685 已通过 smbus2 (/dev/i2c-{bus_num}) 初始化'
                        f' — 适用于 CH341/CP2112 USB-I2C 适配器')
                    return pca
                except Exception as e:
                    self.get_logger().warn(
                        f'smbus2 后端初始化失败 (bus={bus_num}): {e}')

        self.get_logger().warn(
            f'所有 I2C 后端均失败 (尝试: {backends_to_try})，以无硬件模式运行')
        return None

    def _set_pwm_freq_with_retry(self, freq, pca=None, attempts=3, delay=0.1):
        target = pca if pca is not None else self.pca
        if target is None:
            return
        last_err = None
        for i in range(1, attempts + 1):
            try:
                target.frequency = freq
                if i > 1:
                    self.get_logger().info(
                        f'PCA9685 频率设置在第 {i} 次重试成功: {freq}Hz')
                return
            except Exception as e:
                last_err = e
                time.sleep(delay)
        self.get_logger().warn(
            f'PCA9685 频率设置失败 (尝试 {attempts} 次): {last_err}')

    def _angle_to_duty(self, angle_deg, angle_min, angle_max):
        """
        将角度 (度) 映射到 PCA9685 duty_cycle 值

        角度范围 [angle_min, angle_max] → 脉宽 [pulse_min_us, pulse_max_us]
        → PCA9685 tick [0, 0xFFFF]
        """
        # 归一化到 [0, 1]
        ratio = (angle_deg - angle_min) / (angle_max - angle_min)
        ratio = max(0.0, min(1.0, ratio))

        # 脉宽 (µs)
        pulse_us = self.pulse_min_us + ratio * (
            self.pulse_max_us - self.pulse_min_us)

        # 转为 duty_cycle (0~0xFFFF)
        period_us = 1e6 / self.pwm_freq  # 50Hz → 20000µs
        duty = int(pulse_us / period_us * 0xFFFF)
        return max(0, min(0xFFFF, duty))

    def _set_servo(self, channel, angle_deg, angle_min, angle_max):
        """设置指定通道的舵机角度"""
        if self.pca is None:
            self.get_logger().debug(
                f'[NO-HW] servo CH{channel} = {angle_deg:.1f}°')
            return
        duty = self._angle_to_duty(angle_deg, angle_min, angle_max)
        try:
            self.pca.channels[channel].duty_cycle = duty
        except Exception as e:
            self.get_logger().warn(f'舵机 CH{channel} 写入失败: {e}')

    def _apply_angles(self, pan, tilt):
        """限幅并应用 Pan/Tilt 角度到舵机"""
        self.current_pan = max(self.pan_min, min(self.pan_max, pan))
        self.current_tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        self._set_servo(self.pan_channel, self.current_pan,
                        self.pan_min, self.pan_max)
        self._set_servo(self.tilt_channel, self.current_tilt,
                        self.tilt_min, self.tilt_max)

    # ==================================================================
    # 视觉障碍物回调
    # ==================================================================

    def _on_vision_obstacles(self, msg: VisionObstacleArray):
        """处理视觉检测结果，选择跟踪目标"""
        now = self.get_clock().now().nanoseconds / 1e9

        if not msg.obstacles:
            return

        # 选择最佳目标
        target = self._select_target(msg.obstacles)
        if target is None:
            return

        # 更新跟踪信息
        self._track_target_bbox = list(target.bbox)
        self._track_target_class = target.class_name
        self._track_target_distance = target.distance
        self._track_last_seen = now
        self._tracking = True

        # 巡视模式下发现目标 → 自动切换到跟踪
        if self.mode == MODE_PATROL:
            self.mode = MODE_TRACK
            self.pan_pid.reset()
            self.tilt_pid.reset()
            self.get_logger().info(
                f'发现目标 [{target.class_name}] '
                f'{target.distance:.1f}m → 切换到跟踪模式')

    def _select_target(self, obstacles):
        """根据策略选择跟踪目标"""
        valid = [o for o in obstacles if o.confidence > 0.3]
        if not valid:
            return None

        # 如果已有锁定目标，优先 IOU 匹配维持锁定
        if self._track_target_bbox is not None and self._tracking:
            best_iou = 0.0
            best_match = None
            for obs in valid:
                iou = self._compute_iou(self._track_target_bbox,
                                        list(obs.bbox))
                if iou > best_iou:
                    best_iou = iou
                    best_match = obs
            if best_match is not None and best_iou >= self.track_iou_thresh:
                return best_match

        # 无锁定目标或 IOU 匹配失败 → 按策略选择
        if self.target_selection == 'nearest':
            return min(valid, key=lambda o: o.distance)
        elif self.target_selection == 'largest':
            return max(valid,
                       key=lambda o: (o.bbox[2] - o.bbox[0]) *
                                     (o.bbox[3] - o.bbox[1]))
        elif self.target_selection == 'highest_conf':
            return max(valid, key=lambda o: o.confidence)
        else:
            return min(valid, key=lambda o: o.distance)

    @staticmethod
    def _compute_iou(box_a, box_b):
        """计算两个 bbox 的 IOU"""
        x1 = max(box_a[0], box_b[0])
        y1 = max(box_a[1], box_b[1])
        x2 = min(box_a[2], box_b[2])
        y2 = min(box_a[3], box_b[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
        area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
        union = area_a + area_b - inter
        if union <= 0:
            return 0.0
        return inter / union

    # ==================================================================
    # 云台指令回调
    # ==================================================================

    def _on_gimbal_command(self, msg: GimbalCommand):
        """处理外部控制指令"""
        if msg.command_type == GimbalCommand.CMD_SET_MODE:
            old_mode = self.mode
            self.mode = msg.target_mode
            if self.mode == MODE_TRACK:
                self.pan_pid.reset()
                self.tilt_pid.reset()
            if self.mode == MODE_PATROL:
                self._patrol_direction = 1
                self._patrol_tilt_idx = 0
                self._patrol_holding = False
            self.get_logger().info(
                f'模式切换: {MODE_NAMES.get(old_mode, "?")} → '
                f'{MODE_NAMES.get(self.mode, "?")}')

        elif msg.command_type == GimbalCommand.CMD_SET_ANGLE:
            self.mode = MODE_MANUAL
            self._manual_target_pan = msg.pan_angle
            self._manual_target_tilt = msg.tilt_angle
            self._manual_is_velocity = False

        elif msg.command_type == GimbalCommand.CMD_SET_VELOCITY:
            self.mode = MODE_MANUAL
            self._manual_vel_pan = msg.pan_velocity
            self._manual_vel_tilt = msg.tilt_velocity
            self._manual_is_velocity = True

    # ==================================================================
    # 主控制循环
    # ==================================================================

    def _control_loop(self):
        """20Hz 主控制回调"""
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self._last_time
        self._last_time = now

        if dt <= 0 or dt > 1.0:
            dt = 1.0 / self.control_rate

        if self.mode == MODE_PATROL:
            self._update_patrol(dt, now)
        elif self.mode == MODE_TRACK:
            self._update_track(dt, now)
        elif self.mode == MODE_MANUAL:
            self._update_manual(dt)
        # MODE_IDLE: 保持当前位置

        # 发布状态
        self._publish_state()

    # ------------------------------------------------------------------
    # 巡视模式
    # ------------------------------------------------------------------

    def _update_patrol(self, dt, now):
        """巡视模式更新：水平往返扫描 + 垂直阶梯"""
        # 端点等待
        if self._patrol_holding:
            if now >= self._patrol_hold_until:
                self._patrol_holding = False
                # 反向
                self._patrol_direction *= -1
                # 如果从右到左，切换下一个 tilt 阶梯
                if self._patrol_direction == 1:
                    self._patrol_tilt_idx = (
                        (self._patrol_tilt_idx + 1)
                        % len(self.patrol_tilt_angles))
            else:
                return  # 继续等待

        # 水平移动
        pan_delta = self.patrol_pan_speed * dt * self._patrol_direction
        new_pan = self.current_pan + pan_delta

        # 检查是否到达端点
        pan_limit = self.patrol_pan_range
        if new_pan >= pan_limit:
            new_pan = pan_limit
            self._patrol_holding = True
            self._patrol_hold_until = now + self.patrol_edge_hold
        elif new_pan <= -pan_limit:
            new_pan = -pan_limit
            self._patrol_holding = True
            self._patrol_hold_until = now + self.patrol_edge_hold

        # 垂直角度
        if self.patrol_tilt_angles:
            target_tilt = self.patrol_tilt_angles[self._patrol_tilt_idx]
        else:
            target_tilt = 0.0

        # 平滑过渡 tilt
        tilt_speed = 30.0  # 度/秒
        tilt_diff = target_tilt - self.current_tilt
        max_tilt_step = tilt_speed * dt
        if abs(tilt_diff) <= max_tilt_step:
            new_tilt = target_tilt
        else:
            new_tilt = self.current_tilt + max_tilt_step * (
                1.0 if tilt_diff > 0 else -1.0)

        self._apply_angles(new_pan, new_tilt)

    # ------------------------------------------------------------------
    # 跟踪模式
    # ------------------------------------------------------------------

    def _update_track(self, dt, now):
        """跟踪模式更新：PID 闭环控制云台对准目标"""
        # 检查跟踪超时
        if now - self._track_last_seen > self.track_lost_timeout:
            self.get_logger().info(
                f'目标丢失 {self.track_lost_timeout:.1f}s → 恢复巡视')
            self._tracking = False
            self._track_target_bbox = None
            self.mode = MODE_PATROL
            self._patrol_direction = 1
            self._patrol_tilt_idx = 0
            self._patrol_holding = False
            self.pan_pid.reset()
            self.tilt_pid.reset()
            return

        if self._track_target_bbox is None:
            return

        bbox = self._track_target_bbox
        # 目标中心 (像素)
        target_cx = (bbox[0] + bbox[2]) / 2.0
        target_cy = (bbox[1] + bbox[3]) / 2.0

        # 误差: 目标偏离图像中心 (像素)
        # 正误差 → 目标在画面右侧 → pan 应向右增加
        error_x = target_cx - self.img_w / 2.0
        # 正误差 → 目标在画面下方 → tilt 应向下 (减小)
        error_y = target_cy - self.img_h / 2.0

        # PID 计算角速度 (度/秒)
        pan_rate = self.pan_pid.compute(error_x, dt)
        tilt_rate = self.tilt_pid.compute(error_y, dt)

        # 注意 tilt 方向: 画面下方 → tilt 应减小 (下俯)
        new_pan = self.current_pan + pan_rate * dt
        new_tilt = self.current_tilt - tilt_rate * dt

        self._apply_angles(new_pan, new_tilt)

    # ------------------------------------------------------------------
    # 手动模式
    # ------------------------------------------------------------------

    def _update_manual(self, dt):
        """手动模式更新"""
        if self._manual_is_velocity:
            # 角速度控制
            new_pan = self.current_pan + self._manual_vel_pan * dt
            new_tilt = self.current_tilt + self._manual_vel_tilt * dt
            self._apply_angles(new_pan, new_tilt)
        else:
            # 角度控制 (平滑过渡)
            pan_diff = self._manual_target_pan - self.current_pan
            tilt_diff = self._manual_target_tilt - self.current_tilt
            max_step = self.max_angular_speed * dt

            if abs(pan_diff) <= max_step:
                new_pan = self._manual_target_pan
            else:
                new_pan = self.current_pan + max_step * (
                    1.0 if pan_diff > 0 else -1.0)

            if abs(tilt_diff) <= max_step:
                new_tilt = self._manual_target_tilt
            else:
                new_tilt = self.current_tilt + max_step * (
                    1.0 if tilt_diff > 0 else -1.0)

            self._apply_angles(new_pan, new_tilt)

    # ==================================================================
    # 状态发布
    # ==================================================================

    def _publish_state(self):
        """发布云台状态"""
        msg = GimbalState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_link'
        msg.pan_angle = float(self.current_pan)
        msg.tilt_angle = float(self.current_tilt)
        msg.mode = self.mode
        msg.tracking = self._tracking
        msg.track_target_class = self._track_target_class
        msg.track_target_distance = float(self._track_target_distance)
        self.state_pub.publish(msg)

    # ==================================================================
    # 节点销毁
    # ==================================================================

    def destroy_node(self):
        """释放舵机，回到中位"""
        try:
            self.get_logger().info('云台回中...')
            self._apply_angles(0.0, 0.0)
            time.sleep(0.5)
            if self.pca is not None:
                try:
                    self.pca.channels[self.pan_channel].duty_cycle = 0
                    self.pca.channels[self.tilt_channel].duty_cycle = 0
                except Exception:
                    pass
                self.get_logger().info('已释放 PCA9685 通道')
        except Exception as e:
            self.get_logger().warn(f'释放资源时出错: {e}')
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsvGimbalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
