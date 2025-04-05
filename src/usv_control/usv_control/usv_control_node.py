import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool, Header
import math

class UsvControlNode(Node):
    def __init__(self):
        super().__init__('usv_control_node')
       
        # 发布目标点
        self.target_pub = self.create_publisher(PoseStamped, 'setpoint_position/local', 10)
        # 发布到达信号
        self.signal_pub = self.create_publisher(Bool, 'reached_target', 10)

        # 订阅目标点
        self.target_sub = self.create_subscription(
            PoseStamped, 'set_target', self.target_callback, 10)
        
        # 订阅当前位置
        self.position_sub = self.create_subscription(
            PoseStamped, 'local_position/pose', self.position_callback, 10)

        # 当前状态
        self.current_position = PoseStamped()
        self.current_target = PoseStamped()



    def target_callback(self, msg):
        if not isinstance(msg, PoseStamped):
            self.get_logger().info('目标点为空，忽略')
            return
        self.current_target = (msg.pose.position.x, msg.pose.position.y)
        self.send_target(msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'目标点: {self.current_target}')

    def send_target(self, x, y):
        if x is None or y is None:
            self.get_logger().info('目标点为空，忽略')
            return
        msg = PoseStamped()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.target_pub.publish(msg)

    def position_callback(self, msg):
        if not isinstance(msg, PoseStamped):
            self.get_logger().info('当前位置为空，忽略')
            return
        self.current_position = (msg.pose.position.x, msg.pose.position.y)

    def check_target_reached(self):
        if self.current_position and self.current_target:
            distance = math.hypot(
                self.current_position[0] - self.current_target[0],
                self.current_position[1] - self.current_target[1])
            if distance < 1.0:  # 靠近目标（1 米阈值）
                signal_msg = Bool()
                signal_msg.data = True
                self.signal_pub.publish(signal_msg)
                self.get_logger().info(f'Target reached: {self.current_target}')

def main(args=None):
    rclpy.init(args=args)
    node = UsvControlNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()