import signal
import stat

from hamcrest import is_
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Point
from mavros_msgs.msg import State,OverrideRCIn #覆盖RC通道
from std_msgs.msg import Bool, Header,Float32,String
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class UsvControlNode(Node):
    def __init__(self):
        super().__init__('usv_control_node')

        qos = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.publish_qos = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.RELIABLE
        )

          # 从参数加载阈值，默认值为 1.0
        self.declare_parameter('reach_threshold', 1.0)


     
        # 发布目标点
        self.target_point_pub = self.create_publisher(PoseStamped, 'setpoint_position/local', self.publish_qos)    
        # 发布到达信号Bool
        self.signal_pub = self.create_publisher(Bool, 'reached_target', self.publish_qos)
        # 发布油门
        self.rc_override_pub=self.create_publisher(OverrideRCIn,'rc/override',self.publish_qos)

        self.is_running_pub=self.create_publisher(Bool,'is_running',self.publish_qos)

        #订阅当前状态
        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos)
        
        # 订阅需要运行的目标点
        self.target_point_sub = self.create_subscription(
            PoseStamped, 'set_usv_target_position', self.set_target_point_callback, qos)
        
        # 订阅需要运行的目标速度
        self.target_velocity_sub=self.create_subscription(
            Float32,'set_usv_target_velocity',self.set_target_velocity_callback,qos)

        #订阅避障目标点
        self.avoidance_target_point_sub=self.create_subscription(
            PoseStamped,'avoidance_position',self.set_avoidance_target_position_callback,qos)
        #订阅避障标记
        self.avoidance_flag_sub=self.create_subscription(
            Bool,'avoidance_flag',self.set_avoidance_flag_callback,qos)
        
        # 订阅当前USV位置
        self.current_position_sub = self.create_subscription(
            PoseStamped, 'local_position/pose', self.current_position_callback, qos)
        
           
        # 定时器检查是否到达目标点
        self.check_target_timer_reached = self.create_timer(1, self.check_target_reached)

        self.publish_target_timer=self.create_timer(0.1,self.publish_target)

        # 当前状态
        self.current_state = State()
        self.current_position =PoseStamped()
        self.current_point=Point()
        self.current_target_position =PoseStamped()
        self.current_target_point=Point()
        self.speed_value=0.0
        self.avoidance_postition=PoseStamped()
        self.avoidance_point=Point()
        self.avoidance_flag=Bool()
        self.is_running=Bool()

    def state_callback(self, msg):
        if isinstance(msg, State):
            # self.get_logger().info("避障节点usv状态信息接收成功")
            self.current_state = msg
  #订阅当前位置
    def current_position_callback(self, msg):       
        if not isinstance(msg, PoseStamped):
            self.get_logger().info('当前位置为空，忽略')
            return
        self.current_position=msg
        self.current_point = msg.pose.position

    #订阅到达目标点话题
    def set_target_point_callback(self, msg):
        # self.get_logger().info(f"目标点信息接收成功:{msg}")
        if not isinstance(msg, PoseStamped):
            self.get_logger().info('目标坐标为空，忽略')
            return
        self.current_target_position=msg
        self.current_target_point = msg.pose.position
    def set_target_velocity_callback(self,msg):
        self.get_logger().info(f"目标速度信息接收成功:{msg}")
        if not isinstance(msg,Float32):
            self.get_logger().info('目标速度为空，忽略')
            return
        self.speed_value=self.speed_to_pwm(msg.data,0.0,100,1100,1900)

    def set_avoidance_target_position_callback(self,msg):
        if not isinstance(msg,PoseStamped):
            self.get_logger().info('避障目标为空，忽略')
            return
        self.avoidance_position=msg.pose.position

    def set_avoidance_flag_callback(self,msg):
        if not isinstance(msg,Bool):
            self.get_logger().info('避障标记为空，忽略')
            return
        self.avoidance_flag.data=msg.data
        
    def speed_to_pwm(self, speed, speed_min=0.0, speed_max=100.0, pwm_min=1100, pwm_max=1900):
        # 限制 speed 范围
        speed = max(min(speed, speed_max), speed_min)
        return int(((speed - speed_min) / (speed_max - speed_min)) * (pwm_max - pwm_min) + pwm_min)
   
    #发布目标点
    def publish_target(self):
        if not self.current_state.connected or not self.current_state.armed or self.current_state.mode != "GUIDED":
                # self.get_logger().warn("设备站未解锁或未在 GUIDED模式,将不会执行任何操作")
                return  
        # if not self.avoidance_flag:    
        px=self.current_target_position.pose.position.x
        py=self.current_target_position.pose.position.y
        pz=self.current_target_position.pose.position.z

        ox=self.current_target_position.pose.orientation.x
        oy=self.current_target_position.pose.orientation.y
        oz=self.current_target_position.pose.orientation.z
        ow=self.current_target_position.pose.orientation.w
        # else :
        #     px=self.avoidance_postition.pose.position.x
        #     py=self.avoidance_postition.pose.position.y
        #     pz=self.avoidance_postition.pose.position.z  

        #     ox=self.avoidance_postition.pose.orientation.x
        #     oy=self.avoidance_postition.pose.orientation.y
        #     oz=self.avoidance_postition.pose.orientation.z
        #     ow=self.avoidance_postition.pose.orientation.w

        if px is None or py is None or pz is None :
            self.get_logger().info('目标点为空，忽略')
            return
        point_msg = PoseStamped()
        point_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')

        point_msg.pose.position.x=px
        point_msg.pose.position.y=py
        point_msg.pose.position.z=pz

        point_msg.pose.orientation.x = ox
        point_msg.pose.orientation.y = oy
        point_msg.pose.orientation.z = oz
        point_msg.pose.orientation.w = ow

        rc_msg=OverrideRCIn()

        rc_msg.channels=[0]*8

        rc_msg.channels[2]=int(self.speed_value)


        self.target_point_pub.publish(point_msg)
        self.rc_override_pub.publish(rc_msg)
        is_running_=Bool()
        is_running_.data=True
        self.is_running_pub.publish(is_running_)
        signal_=Bool()
        signal_.data=False
        self.signal_pub.publish(signal_)

  

    #检测无人船是否到达目标点
    def check_target_reached(self):
        self.reach_threshold = self.get_parameter('reach_threshold').value
        if (
            self.current_point
            and self.current_target_point
            and self.current_point.x is not None
            and self.current_point.y is not None
            and self.current_target_point.x is not None
            and self.current_target_point.y is not None
            and self.reach_threshold is not None
        ):
            distance = math.hypot(
                self.current_point.x - self.current_target_point.x,
                self.current_point.y - self.current_target_point.y)
            if distance < self.reach_threshold:  # 靠近目标（1 米阈值）
                signal_msg_=Bool()
                signal_msg_.data=True
                self.signal_pub.publish(signal_msg_)
                is_running_=Bool()
                is_running_.data=False
                self.is_running_pub.publish(is_running_)
                # self.get_logger().info(f'目标点已经到达')

def main(args=None):
    rclpy.init(args=args)
    node = UsvControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()