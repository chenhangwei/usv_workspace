import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # 假设定向雷达输出类似 LaserScan
from geometry_msgs.msg import PoseStamped,Point
import math
from std_msgs.msg import Header,Bool
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class UsvAvoidanceNode(Node):
    def __init__(self):
        super().__init__('usv_avoidance_node')

        qos = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_a = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 从参数加载阈值，默认值为 1.0
        self.declare_parameter('in_distance_value', 2.0)

        
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
        self.target_pub = self.create_publisher(PoseStamped, 'avoidance_position', 10)

        # 发布避障状态（true/false）
        self.avoidance_flag=self.create_publisher(Bool,'avoidance_flag',qos_a)
    
        #定时运行避障程序
        self.avoidance_timer=self.create_timer(0.1,self. avoidance_run)

        self.current_laserscan=LaserScan() #初始化当前雷达数据
        self.current_state = State() #初始化当前飞控设备状态
        self.current_position =Point() #初始化当前位置坐标
        self.current_target = Point() #初始化目标位置坐标
        self.obstacle_detected  = False #初始化雷达避障动作标志

    # 获取雷达数据
    def radar_callback(self, msg):
        if isinstance(msg, LaserScan):
            self.get_logger().info("雷达数据接收成功")
            self.current_laserscan=msg          
    # 获取当前状态
    def state_callback(self, msg):
        if isinstance(msg, State):
            # self.get_logger().info("避障节点usv状态信息接收成功")
            self.current_state = msg
    # 获取当前坐标     
    def position_callback(self, msg):
        if isinstance(msg, PoseStamped):
            self.current_position= msg.pose.position
    # 获取目标坐标
    def target_callback(self, msg):
        if isinstance(msg, PoseStamped):
            # self.get_logger().info("目标点信息接收成功")
            self.current_target = msg.pose.position

 
    #避障循环
    def avoidance_run(self):
        self.in_distance_value=self.get_parameter("in_distance_value").value 
        if not self.current_state.connected or not self.current_state.armed or self.current_state.mode != "GUIDED":
                # self.get_logger().warn("设备站未解锁或未在 GUIDED模式,避障将不会执行任何操作")
                return
        if self.current_laserscan is not None:
            # 检测前方 ±30° 范围内的最小距离
            angle_min = -30.0 * math.pi / 180.0  # -30°
            angle_max = 30.0 * math.pi / 180.0   # 30°
            ranges = self.current_laserscan.ranges
            angles = [self.current_laserscan.angle_min + i * self.current_laserscan.angle_increment for i in range(len(ranges))]
            min_distance = float('inf')
                    
            for i, angle in enumerate(angles):
                if angle_min <= angle <= angle_max and ranges[i] < min_distance and ranges[i] > self.current_laserscan.range_min:
                    min_distance = ranges[i]
                    
                # 如果前方障碍物距离小于安全距离，触发避障
            if (
                min_distance is not None and
                self.in_distance_value is not None and
                min_distance < self.in_distance_value and
                min_distance > 0
            ):
                self.obstacle_detected = True
                self.get_logger().info(f"Obstacle detected at {min_distance:.2f} m, initiating avoidance")
            else:
                self.obstacle_detected = False   


        if not self.current_position and not self.current_target and self.obstacle_detected:              
            # 计算当前到目标的方向
            dx = self.current_target.x - self.current_position.x
            dy = self.current_target.y- self.current_position.y
            heading = math.atan2(dy, dx)
            # 绕障：向右偏移 2 米（可根据雷达数据动态调整）
            avoid_x = self.current_position.x+ 2.0 * math.sin(heading)
            avoid_y = self.current_position.y - 2.0 * math.cos(heading)
            # 发送临时避障目标点
            msg = PoseStamped()
            msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
            msg.pose.position.x = avoid_x
            msg.pose.position.y = avoid_y
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0

            self.target_pub.publish(msg)
            self.get_logger().info(f'Avoidance target: ({avoid_x}, {avoid_y})')

        temp = Bool()
        temp.data=self.obstacle_detected
        self.avoidance_flag.publish(temp)




def main(args=None):
    rclpy.init(args=args)
    node = UsvAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()