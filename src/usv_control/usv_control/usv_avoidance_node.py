"""
无人船避障节点

该节点负责处理无人船的避障逻辑。通过订阅雷达数据、飞控状态、当前位置和目标位置，
当检测到障碍物时，自动调整目标点以避开障碍物，确保无人船安全航行。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import PoseStamped, Point
import math
from std_msgs.msg import Bool
from mavros_msgs.msg import State, PositionTarget
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class UsvAvoidanceNode(Node):
    """
    无人船避障节点类
    
    该节点实现基于超声波雷达的避障功能，当检测到障碍物时，
    自动调整无人船的目标位置以避开障碍物。
    """

    def __init__(self):
        """初始化无人船避障节点"""
        super().__init__('usv_avoidance_node')

        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 从参数服务器加载避障距离阈值，默认值为 1.2 米
        self.declare_parameter('in_distance_value', 1.2)
        self.in_distance_value = self.get_parameter('in_distance_value').get_parameter_value().double_value

        # 订阅飞控状态信息
        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos_best_effort)
        
        # 订阅超声波雷达数据
        self.radar_sub = self.create_subscription(
            Range, 'ultrasonic_radar_range', self.radar_callback, qos_best_effort)
        
        # 订阅当前目标点
        self.target_sub = self.create_subscription(
            PositionTarget, 'setpoint_raw/local', self.target_callback, qos_best_effort)
        
        # 订阅当前 UWB 位置
        self.position_sub = self.create_subscription(
            PoseStamped, 'local_position/pose', self.position_callback, qos_best_effort)

        # 发布调整后的目标点
        self.target_pub = self.create_publisher(PositionTarget, 'avoidance_position', 10)

        # 发布避障状态（true/false）
        self.avoidance_flag = self.create_publisher(Bool, 'avoidance_flag', qos_reliable)
    
        # 定时运行避障程序
        self.avoidance_timer = self.create_timer(0.2, self.avoidance_run)

        # 初始化状态变量
        self.current_laserscan = Range()     # 当前雷达数据
        self.current_state = State()         # 当前飞控设备状态
        self.current_position = Point()      # 当前位置坐标
        self.current_target = Point()        # 目标位置坐标
        self.obstacle_detected = False       # 雷达避障动作标志

        self.get_logger().info('USV 避障节点已启动')
        self.get_logger().info(f'避障距离阈值: {self.in_distance_value} 米')

    def radar_callback(self, msg):
        """
        雷达数据回调函数
        
        Args:
            msg (Range): 包含雷达距离信息的消息
        """
        if isinstance(msg, Range):
            self.current_laserscan = msg         

    def state_callback(self, msg):
        """
        飞控状态回调函数
        
        Args:
            msg (State): 包含飞控状态信息的消息
        """
        if isinstance(msg, State):
            self.current_state = msg

    def position_callback(self, msg):
        """
        位置信息回调函数
        
        Args:
            msg (PoseStamped): 包含当前位置信息的消息
        """
        if isinstance(msg, PoseStamped):
            self.current_position = msg.pose.position

    def target_callback(self, msg):
        """
        目标点回调函数
        
        Args:
            msg (PositionTarget): 包含目标点信息的消息
        """
        if isinstance(msg, PositionTarget):
            self.current_target = msg.position

    def avoidance_run(self):
        """
        避障主逻辑函数
        
        定期检查是否需要进行避障操作，并在必要时发布新的目标点。
        """
        try:
            # 获取最新的避障距离阈值参数
            self.in_distance_value = self.get_parameter("in_distance_value").get_parameter_value().double_value
            
            # 检查飞控是否已连接、已解锁且处于GUIDED模式
            if not self.current_state.connected or not self.current_state.armed or self.current_state.mode != "GUIDED":
                return
                 
            # 检测障碍物
            if self.current_laserscan.range < self.in_distance_value:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False   

            # 如果检测到障碍物且位置信息有效，则计算避障目标点
            if self.obstacle_detected and self.current_position and self.current_target:
                # 计算当前到目标的方向
                dx = self.current_target.x - self.current_position.x
                dy = self.current_target.y - self.current_position.y
                
                # 避免除零错误
                if dx == 0 and dy == 0:
                    heading = 0.0
                else:
                    heading = math.atan2(dy, dx)
                
                # 绕障：向右偏移 2 米（可根据雷达数据动态调整）
                avoid_x = self.current_position.x + 2.0 * math.sin(heading)
                avoid_y = self.current_position.y - 2.0 * math.cos(heading)

                # 构造并发布避障目标点消息
                msg = PositionTarget()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                msg.type_mask = (
                    PositionTarget.IGNORE_VX |
                    PositionTarget.IGNORE_VY |
                    PositionTarget.IGNORE_VZ |
                    PositionTarget.IGNORE_AFX |
                    PositionTarget.IGNORE_AFY |
                    PositionTarget.IGNORE_AFZ |
                    PositionTarget.FORCE |
                    PositionTarget.IGNORE_YAW |
                    PositionTarget.IGNORE_YAW_RATE
                )
                msg.position.x = avoid_x
                msg.position.y = avoid_y 
                msg.position.z = 0.0

                self.target_pub.publish(msg)
                self.get_logger().info(f'避障目标点已发布: ({avoid_x:.2f}, {avoid_y:.2f})')

            # 发布避障状态
            temp = Bool()
            temp.data = self.obstacle_detected
            self.avoidance_flag.publish(temp)

        except Exception as e:
            self.get_logger().error(f'避障程序运行异常: {str(e)}')


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = UsvAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()