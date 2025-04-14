import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # 假设定向雷达输出类似 LaserScan
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import Header
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class UsvAvoidanceNode(Node):
    def __init__(self):
        super().__init__('usv_avoidance_node')

        qos = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 从参数加载阈值，默认值为 1.0
        self.in_distance_value = self.declare_parameter('in_distance_value ', 2.0).value

        
        #订阅当前状态
        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos)
        # 订阅雷达数据
        self.radar_sub = self.create_subscription(
            LaserScan, 'radar_scan', self.radar_callback, qos)
        # 订阅当前目标点
        self.target_sub = self.create_subscription(
            PoseStamped, 'setpoint_position/local', self.target_callback, qos)
        # 订阅当前 UWB 位置
        self.position_sub = self.create_subscription(
            PoseStamped, 'local_position/pose', self.position_callback, qos)
        


        # 发布调整后的目标点
        self.target_pub = self.create_publisher(PoseStamped, 'setpoint_avoidance', 10)
        # 当前状态
        self.current_state = State()
        self.current_position = None
        self.current_target = None
        self.avoiding = False

    def radar_callback(self, msg):

        if  not self.current_state.armed or self.current_state.mode != "GUIDED":
                self.get_logger().warn("设备站未解锁或未在 GUIDED模式,避障将不会执行任何操作")
                return

        if isinstance(msg, LaserScan):
            self.get_logger().info("雷达数据接收成功")
            # 检测前方障碍（假设雷达正前方为 0 度）
            min_distance = min(msg.ranges)  # 最近障碍距离
            if min_distance < self.in_distance_value  and not self.avoiding:  # 2 米内有障碍
                self.get_logger().info('障碍物检测到，启动避让')
                self.avoiding = True
                self.adjust_path()
    def state_callback(self, msg):
        if isinstance(msg, State):
            self.get_logger().info("避障节点usv状态信息接收成功")
            self.current_state = msg
           


    def position_callback(self, msg):
        if isinstance(msg, PoseStamped):
            self.current_position = (msg.pose.position.x, msg.pose.position.y)

    def target_callback(self, msg):
        if isinstance(msg, PoseStamped):
            self.get_logger().info("目标点信息接收成功")
            self.current_target = (msg.pose.position.x, msg.pose.position.y)

    def adjust_path(self):
        if not self.current_position or not self.current_target:
            return
        # 计算当前到目标的方向
        dx = self.current_target[0] - self.current_position[0]
        dy = self.current_target[1] - self.current_position[1]
        heading = math.atan2(dy, dx)
        # 绕障：向右偏移 2 米（可根据雷达数据动态调整）
        avoid_x = self.current_position[0] + 2.0 * math.sin(heading)
        avoid_y = self.current_position[1] - 2.0 * math.cos(heading)
        # 发送临时避障目标点
        msg = PoseStamped()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        msg.pose.position.x = avoid_x
        msg.pose.position.y = avoid_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)
        self.get_logger().info(f'Avoidance target: ({avoid_x}, {avoid_y})')
        # 定时检查是否绕过障碍
        self.create_timer(2.0, self.check_avoidance_complete)

    def check_avoidance_complete(self):
        if self.avoiding and self.current_position:
            # 如果接近临时目标点，恢复原始目标
            dist_to_avoid = math.hypot(
                self.current_position[0] - self.current_target[0],
                self.current_position[1] - self.current_target[1])
            if dist_to_avoid < 1.0:  # 假设绕过障碍
                self.avoiding = False
                msg = PoseStamped()
                msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
                msg.pose.position.x = float(self.current_target[0])
                msg.pose.position.y = float(self.current_target[1])
                msg.pose.position.z = 0.0
                msg.pose.orientation.w = 1.0
                self.target_pub.publish(msg)
                self.get_logger().info(f'Resuming original target: {self.current_target}')

def main(args=None):
    rclpy.init(args=args)
    node = UsvAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()