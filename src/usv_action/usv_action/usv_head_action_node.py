import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import random
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# === 新增：导入PCA9685控制库 ===
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# 舵机控制参数
SERVO_CHANNEL = 2  # PCA9685 通道编号（0-15）
PWM_MID = 1500     # 中位
PWM_MIN = 1100
PWM_MAX = 1900
PWM_PER_DEGREE = (PWM_MAX - PWM_MID) / 45
RC_OVERRIDE_FREQ = 50

class UsvHeadActionNode(Node):
    def __init__(self):
        super().__init__('usv_head_action_node')
        self.qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            String, 'gs_action_command', self.listener_callback, self.qos)

        # === 初始化PCA9685 ===
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # 设置为标准舵机频率

        self.swinging = False
        self.current_pwm = PWM_MID
        self.target_pwm = PWM_MID
        self.pwm_step = 5
        self.center_pwm_step = 2
        self.swing_phase = 'idle'
        self.swing_target_pwm = PWM_MID
        self.swing_hold_until = None
        self.waiting_to_center = False
        self.center_time = None

        self.timer = self.create_timer(1.0 / RC_OVERRIDE_FREQ, self.timer_callback)
        self.get_logger().info('摇摆器控制器（PCA9685）节点已启动。')

    def set_pwm_us(self, channel, pwm_us):
        # 将微秒PWM转换为16位值（占空比），PCA9685每周期约20ms（50Hz）
        pwm_us = max(PWM_MIN, min(PWM_MAX, pwm_us))
        duty_cycle = int(pwm_us / 20000 * 0xFFFF)  # 20000us = 20ms
        self.pca.channels[channel].duty_cycle = duty_cycle

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # 平滑移动到目标PWM
        if self.current_pwm != self.target_pwm:
            step = self.center_pwm_step if self.waiting_to_center else self.pwm_step
            diff = self.target_pwm - self.current_pwm
            if abs(diff) <= step:
                self.current_pwm = self.target_pwm
            else:
                self.current_pwm += step if diff > 0 else -step
            self.set_pwm_us(SERVO_CHANNEL, self.current_pwm)
            return

        # === 状态机控制 ===
        if self.swinging:
            if self.swing_phase == 'idle':
                self.target_pwm = PWM_MID
                self.swing_phase = 'to_center_start'
                return
            elif self.swing_phase == 'to_center_start':
                if self.current_pwm == PWM_MID:
                    direction = random.choice(['left', 'right'])
                    angle = random.uniform(20, 45)
                    if direction == 'left':
                        self.swing_target_pwm = int(PWM_MID - abs(angle) * PWM_PER_DEGREE)
                    else:
                        self.swing_target_pwm = int(PWM_MID + abs(angle) * PWM_PER_DEGREE)
                    self.target_pwm = self.swing_target_pwm
                    self.swing_phase = 'to_target'
                    self.get_logger().info(f'开始摇摆: {direction} {angle:.1f}° (PWM: {self.swing_target_pwm})')
                return
            elif self.swing_phase == 'to_target':
                if self.current_pwm == self.swing_target_pwm:
                    self.swing_hold_until = now + random.randint(2, 4)
                    self.swing_phase = 'hold'
                return
            elif self.swing_phase == 'hold':
                if self.swing_hold_until is not None and now >= self.swing_hold_until:
                    self.target_pwm = PWM_MID
                    self.swing_phase = 'to_center'
                return
            elif self.swing_phase == 'to_center':
                self.target_pwm = PWM_MID
                if self.current_pwm == PWM_MID:
                    self.swing_phase = 'idle'
                return

        # 回中逻辑
        if self.waiting_to_center:
            self.target_pwm = PWM_MID
            if self.center_time is not None and now >= self.center_time:
                self.waiting_to_center = False
            return

        # 持续发当前PWM
        self.set_pwm_us(SERVO_CHANNEL, self.current_pwm)

    def move_angle(self, angle):
        pwm = int(PWM_MID + angle * PWM_PER_DEGREE)
        pwm = max(PWM_MIN, min(PWM_MAX, pwm))
        self.target_pwm = pwm
        self.get_logger().info(f'转动到 {angle:.1f}° (PWM: {pwm})')

    def center(self):
        self.target_pwm = PWM_MID
        self.get_logger().info('舵机回中 (0°)')

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
        self.center()
        time.sleep(0.5)
        self.pca.channels[SERVO_CHANNEL].duty_cycle = 0
        self.get_logger().info('释放PCA9685通道')
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