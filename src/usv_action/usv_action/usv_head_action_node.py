import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
import time
import random

SERVO_CHANNEL = 8  # MAIN 8通道，MAVROS下标从1开始
PWM_MID = 1500     # 中位
PWM_MIN = 1100
PWM_MAX = 1900
PWM_PER_DEGREE = (PWM_MAX - PWM_MID) / 45  # 每度PWM步进

class UsvHeadActionNode(Node):
    def __init__(self):
        super().__init__('usv_head_action_node')
        self.subscription = self.create_subscription(
            String,
            'gs_action_command',
            self.listener_callback,
            10)
        self.rc_pub = self.create_publisher(OverrideRCIn, 'rc/override', 10) # MAVROS RC覆盖话题
        self.swinging = False # 是否正在摇摆
        self.next_swing_time = None # 下次摇摆时间
        self.timer = self.create_timer(1.0, self.timer_callback)  # 每秒检查一次
        self.waiting_to_center = False # 是否等待回中
        self.center_time = None # 回中时间
        self.get_logger().info('摇摆器控制器节点已启动。')

    def listener_callback(self, msg):
        cmd = msg.data.strip()
        if cmd == 'neck_swinging':
            if not self.swinging:
                self.swinging = True
                self.schedule_next_swing()
                self.get_logger().info('开始摇摆')
        elif cmd == 'neck_stop':
            self.swinging = False
            self.waiting_to_center = False
            self.center()
            self.get_logger().info('停止摇摆并回中')

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9 # 获取当前时间（秒）
        if self.waiting_to_center:
            if now >= self.center_time:
                self.center()
                self.waiting_to_center = False
                self.schedule_next_swing()
            return

        if self.swinging and self.next_swing_time is not None and now >= self.next_swing_time: 
            direction = random.choice(['left', 'right'])
            angle = random.uniform(20, 45)
            if direction == 'left':
                self.move_angle(-angle)
            else:
                self.move_angle(angle)
            self.waiting_to_center = True
            ran = random.choice(2,4)  # 随机2-4秒后回中
            self.center_time = now + ran # 设置回中时间
    
    def schedule_next_swing(self): # 安排下次摇摆
        now = self.get_clock().now().nanoseconds / 1e9
        self.next_swing_time = now + random.uniform(60, 90) # 随机60-90秒后摇摆

    def move_angle(self, angle):
        pwm = int(PWM_MID + angle * PWM_PER_DEGREE) # 计算PWM值
            # 设置PWM上下限
      
        pwm = max(PWM_MIN, min(PWM_MAX, pwm))
        self.send_pwm(pwm)
        self.get_logger().info(f'Move neck to {angle:.1f}° (PWM: {pwm})')
    
    def center(self):
        self.send_pwm(PWM_MID)
        self.get_logger().info('Center neck (0°)')

    def send_pwm(self, pwm_value):
        msg = OverrideRCIn()
        msg.channels = [65535] * 18  # 18通道，65535为不覆盖
        msg.channels[SERVO_CHANNEL - 1] = pwm_value
        self.rc_pub.publish(msg)

    def destroy_node(self):
        self.center()
        time.sleep(0.5)
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