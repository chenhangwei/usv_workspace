import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool, Header
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class UsvControlNode(Node):
    def __init__(self):
        super().__init__('usv_control_node')

        qos = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

          # 从参数加载阈值，默认值为 1.0
        self.reach_threshold = self.declare_parameter('reach_threshold', 1.0).value


       
        # 发布目标点
        self.target_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)
        # 发布到达信号Bool
        self.signal_pub = self.create_publisher(Bool, 'reached_target', 10)



        #订阅当前状态
        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos)
        
        # 订阅目标点
        self.target_sub = self.create_subscription(
            PoseStamped, 'set_target', self.target_callback, qos)
        
        # 订阅当前位置
        self.current_position_sub = self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self.current_position_callback, qos)
        
        # 定时器检查是否到达目标点
        self.timer = self.create_timer(1, self.check_target_reached)

        # 当前状态
        self.current_state = State()
        self.current_position =None
        self.current_target = None

    def state_callback(self, msg):
        if isinstance(msg, State):
            self.get_logger().info("避障节点usv状态信息接收成功")
            self.current_state = msg
    #订阅到达目标点话题
    def target_callback(self, msg):
        if not isinstance(msg, PoseStamped):
            self.get_logger().info('目标点为空，忽略')
            return
        self.current_target = (msg.pose.position.x, msg.pose.position.y)
        self.send_target(msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'目标点: {self.current_target}')
    #发布目标点
    def send_target(self, x, y):
        if not self.current_state.armed or self.current_state.mode != "GUIDED":
                self.get_logger().warn("设备站未解锁或未在 GUIDED模式,将不会执行任何操作")
                return
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
    #订阅当前位置
    def current_position_callback(self, msg):
        if not isinstance(msg, PoseStamped):
            self.get_logger().info('当前位置为空，忽略')
            return
        self.current_position = (msg.pose.position.x, msg.pose.position.y)

    #检测无人船是否到达目标点
    def check_target_reached(self):
        if self.current_position and self.current_target:
            distance = math.hypot(
                self.current_position[0] - self.current_target[0],
                self.current_position[1] - self.current_target[1])
            if distance < self.reach_threshold:  # 靠近目标（1 米阈值）
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