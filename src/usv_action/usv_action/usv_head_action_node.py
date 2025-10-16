import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import random
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

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

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.swinging = False
        self.current_angle = 90
        self.target_angle = 90
        self.angle_step = 2
        self.swing_phase = 'idle'
        self.swing_hold_until = None
        self.waiting_to_center = False
        self.center_time = None

        self.timer = self.create_timer(1.0 / RC_OVERRIDE_FREQ, self.timer_callback)
        self.get_logger().info('摇摆器控制器（CircuitPython版）节点已启动。')

    def set_servo_angle(self, channel, angle):
        duty = angle_to_duty(angle)
        self.pca.channels[channel].duty_cycle = duty

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
                self.target_angle = 90
                self.swing_phase = 'to_center_start'
            elif self.swing_phase == 'to_center_start':
                if self.current_angle == 90:
                    direction = random.choice(['left', 'right'])
                    angle = random.uniform(20, 45)
                    if direction == 'left':
                        self.target_angle = 90 - angle
                    else:
                        self.target_angle = 90 + angle
                    self.swing_phase = 'to_target'
                    self.get_logger().info(f'开始摇摆: {direction} {angle:.1f}° (目标角度: {self.target_angle:.1f}°)')
            elif self.swing_phase == 'to_target':
                if self.current_angle == self.target_angle:
                    self.swing_hold_until = now + random.uniform(2, 4)
                    self.swing_phase = 'hold'
            elif self.swing_phase == 'hold':
                if now >= self.swing_hold_until:
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
