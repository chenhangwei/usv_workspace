import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import random
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from typing import Any, cast

# 可选硬件依赖（无则进入无硬件模式）
HARDWARE_AVAILABLE = True
busio = None
SCL = None
SDA = None
PCA9685 = None
try:
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
except Exception:
    HARDWARE_AVAILABLE = False

SERVO_CHANNEL = 0  # PCA9685的通道号，通常为0-15
RC_OVERRIDE_FREQ = 50  # 50Hz 舵机刷新频率

# 占空比范围：5% (0度) ~ 10% (180度)
MIN_DUTY = 0.05
MAX_DUTY = 0.10

def angle_to_duty(angle):
    angle = max(0, min(180, angle))
    pulse_percent = MIN_DUTY + (angle / 180.0) * (MAX_DUTY - MIN_DUTY)
    return int(pulse_percent * 0xFFFF)

class UsvHeadActionNode(Node):
    def __init__(self):
        super().__init__('usv_head_action_node')
        self.qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            String, 'gs_action_command', self.listener_callback, self.qos)

        # 允许用参数关停硬件控制（默认启用）
        self.declare_parameter('enable_head_actuator', True)
        # PWM 频率可配置（默认 50Hz）
        self.declare_parameter('pwm_frequency', 50)
        try:
            self.enable_hw = bool(self.get_parameter('enable_head_actuator').get_parameter_value().bool_value)
        except Exception:
            self.enable_hw = True
        try:
            self.pwm_frequency = int(self.get_parameter('pwm_frequency').get_parameter_value().integer_value)
            if self.pwm_frequency <= 0:
                raise ValueError('pwm_frequency must be > 0')
        except Exception:
            self.pwm_frequency = 50

        self.pca = None
        if self.enable_hw and HARDWARE_AVAILABLE:
            try:
                # busio/PCA9685 在缺少依赖时会为 None，这里已由 HARDWARE_AVAILABLE 保证可用
                i2c = cast(Any, busio).I2C(SCL, SDA)  # type: ignore[attr-defined]
                self.pca = cast(Any, PCA9685)(i2c)    # type: ignore[call-arg]
                # 设置 PWM 频率，加入重试降低 I2C 短暂忙碌导致的告警
                self._set_pwm_frequency_with_retry(self.pwm_frequency)
            except Exception as e:
                self.get_logger().warn(f'I2C/PCA9685 初始化失败，切换到无硬件模式: {e}')
                self.pca = None
        else:
            if not HARDWARE_AVAILABLE:
                self.get_logger().warn('未找到板卡依赖(board/busio/adafruit_pca9685)，以无硬件模式运行')
            if not self.enable_hw:
                self.get_logger().info('参数 enable_head_actuator=false，以无硬件模式运行')

        self.swinging = False
        self.current_angle = 90
        self.target_angle = 90
        self.angle_step = 2
        self.swing_phase = 'idle'
        self.swing_hold_until = None
        self.waiting_to_center = False
        self.center_time = None
        self.next_swing_direction = None  # 记录下一次摇摆的方向
        self.next_swing_angle = None      # 记录下一次摇摆的角度
        # 定时器与启动日志（修正缩进，必须在 __init__ 内）
        self.timer = self.create_timer(1.0 / RC_OVERRIDE_FREQ, self.timer_callback)
        mode = '硬件控制' if self.pca is not None else '无硬件模式'
        self.get_logger().info(f'摇摆器控制器节点已启动（{mode}）。')

    def _set_pwm_frequency_with_retry(self, freq: int, attempts: int = 3, delay: float = 0.1):
        if self.pca is None:
            return
        last_err = None
        for i in range(1, attempts + 1):
            try:
                self.pca.frequency = freq
                if i > 1:
                    self.get_logger().info(f'PCA9685 频率设置在第 {i} 次重试成功: {freq}Hz')
                return
            except Exception as e:
                last_err = e
                # 常见为 EAGAIN 资源忙，短暂等待重试
                try:
                    time.sleep(delay)
                except Exception:
                    pass
        self.get_logger().warn(f'I2C 设置 PWM 频率失败（尝试 {attempts} 次），继续以默认值运行: {last_err}')

    def set_servo_angle(self, channel, angle):
        if self.pca is None:
            self.get_logger().debug(f'[NO-HW] set_servo_angle ch={channel} angle={angle}')
            return
        duty = angle_to_duty(angle)
        try:
            self.pca.channels[channel].duty_cycle = duty
        except Exception as e:
            self.get_logger().warn(f'设置舵机角度失败: {e}')

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.current_angle != self.target_angle:
            diff = self.target_angle - self.current_angle
            step = self.angle_step
            if abs(diff) <= step:
                self.current_angle = self.target_angle
            else:
                self.current_angle += step if diff > 0 else -step
            self.set_servo_angle(SERVO_CHANNEL, self.current_angle)
            return

        if self.swinging:
            if self.swing_phase == 'idle':
                # 在idle阶段就决定下一次摇摆的方向和角度
                self.next_swing_direction = random.choice(['left', 'right'])
                self.next_swing_angle = random.uniform(20, 45)
                self.target_angle = 90
                self.swing_phase = 'to_center_start'
                self.get_logger().info(f'准备摇摆: {self.next_swing_direction} {self.next_swing_angle:.1f}°')
            elif self.swing_phase == 'to_center_start':
                if self.current_angle == 90:
                    # 使用预先决定的方向和角度（确保已初始化）
                    if self.next_swing_direction is None or self.next_swing_angle is None:
                        self.get_logger().warn('摇摆参数未初始化，重新生成')
                        self.next_swing_direction = random.choice(['left', 'right'])
                        self.next_swing_angle = random.uniform(20, 45)
                    
                    if self.next_swing_direction == 'left':
                        self.target_angle = 90 - self.next_swing_angle
                    else:
                        self.target_angle = 90 + self.next_swing_angle
                    self.swing_phase = 'to_target'
                    self.get_logger().info(f'开始摇摆: {self.next_swing_direction} {self.next_swing_angle:.1f}° (目标角度: {self.target_angle:.1f}°)')
            elif self.swing_phase == 'to_target':
                if self.current_angle == self.target_angle:
                    self.swing_hold_until = now + random.uniform(2, 4)
                    self.swing_phase = 'hold'
            elif self.swing_phase == 'hold':
                if (self.swing_hold_until is not None) and (now >= self.swing_hold_until):
                    self.target_angle = 90
                    self.swing_phase = 'to_center'
            elif self.swing_phase == 'to_center':
                if self.current_angle == 90:
                    self.swing_phase = 'idle'
        elif self.waiting_to_center:
            self.target_angle = 90
            if self.center_time is not None and now >= self.center_time:
                self.waiting_to_center = False
        else:
            self.set_servo_angle(SERVO_CHANNEL, self.current_angle)

    def move_angle(self, angle):
        angle = max(0, min(180, angle))
        self.target_angle = angle
        self.get_logger().info(f'转动到 {angle:.1f}°')

    def center(self):
        self.target_angle = 90
        self.get_logger().info('舵机回中 (90°)')

    def listener_callback(self, msg):
        cmd = msg.data.strip()
        if cmd == 'neck_swinging':
            if not self.swinging:
                self.swinging = True
                self.swing_phase = 'idle'
                self.get_logger().info('开始摇摆')
        elif cmd == 'neck_stop':
            self.swinging = False
            self.swing_phase = 'idle'
            self.waiting_to_center = True
            self.center_time = self.get_clock().now().nanoseconds / 1e9
            self.center()
            self.get_logger().info('停止摇摆并回中')

    def destroy_node(self):
        try:
            self.center()
            time.sleep(0.5)
            if self.pca is not None:
                try:
                    self.pca.channels[SERVO_CHANNEL].duty_cycle = 0
                except Exception:
                    pass
                self.get_logger().info('释放PCA9685通道')
        except Exception as e:
            self.get_logger().warn(f'释放资源时出现问题: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UsvHeadActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
