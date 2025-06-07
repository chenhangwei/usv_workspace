import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range  # 假设定向雷达输出类似 Range 
from geometry_msgs.msg import PoseStamped,Point
import math
from std_msgs.msg import Header,Bool
from mavros_msgs.msg import State,PositionTarget
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
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 从参数加载阈值，默认值为 1.0
        self.declare_parameter('in_distance_value', 1.0)
      
        #订阅当前状态
        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos)
        
        # 订阅雷达数据
        self.radar_sub = self.create_subscription(
            Range, 'ultrasonic_radar_range', self.radar_callback, qos)
        
        # 订阅当前目标点
        self.target_sub = self.create_subscription(
            PositionTarget, 'ssetpoint_raw/local', self.target_callback, qos)
        
        # 订阅当前 UWB 位置
        self.position_sub = self.create_subscription(
            PoseStamped, 'local_position/pose', self.position_callback, qos)
        


        # 发布调整后的目标点
        self.target_pub = self.create_publisher(PositionTarget,'avoidance_position', 10)

        # 发布避障状态（true/false）
        self.avoidance_flag=self.create_publisher(Bool,'avoidance_flag',qos_a)
    
        #定时运行避障程序
        self.avoidance_timer=self.create_timer(0.2,self. avoidance_run)

        self.current_laserscan=Range () #初始化当前雷达数据
        self.current_state = State() #初始化当前飞控设备状态
        self.current_position =Point() #初始化当前位置坐标
        self.current_target = Point() #初始化目标位置坐标
        self.obstacle_detected  = False #初始化雷达避障动作标志
        self.in_distance_value = 1.0

        self.get_logger().info('USV 避障节点已启动')


    # 获取雷达数据
    def radar_callback(self, msg):
        if isinstance(msg, Range):
            self.current_laserscan=msg         
    # 获取当前状态
    def state_callback(self, msg):
        if isinstance(msg, State):
            self.current_state = msg
    # 获取当前坐标     
    def position_callback(self, msg):
        if isinstance(msg, PoseStamped):
            self.current_position= msg.pose.position
    # 获取目标坐标
    def target_callback(self, msg):
        if isinstance(msg, PositionTarget):
            self.current_target = msg.position

 
    #避障循环
    def avoidance_run(self):

        try:
            self.in_distance_value=self.get_parameter("in_distance_value").value
            
            dis_value = float(self.in_distance_value)

            if not self.current_state.connected or not self.current_state.armed or self.current_state.mode != "GUIDED":
                    return
                 
            if self.current_laserscan.range < dis_value:
                self.obstacle_detected = True
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
                msg= PositionTarget()
                msg.header.stamp=self.get_clock().now().to_msg()
                msg.header.frame_id='map'
                msg.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
                msg.type_mask=(
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
                msg.position.x=avoid_x
                msg.position.y=avoid_y 
                msg.position.z=0.0


                self.target_pub.publish(msg)
                self.get_logger().info(f'避障目标点: ({avoid_x}, {avoid_y})')

            temp = Bool()
            temp.data=self.obstacle_detected
            self.avoidance_flag.publish(temp)

        except Exception as e:
            self.get_logger().error(f'避障程序运行异常: {str(e)}')





def main(args=None):
    rclpy.init(args=args)
    node = UsvAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()