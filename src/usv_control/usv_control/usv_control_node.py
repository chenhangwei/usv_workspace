import signal
import stat
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Point
from mavros_msgs.msg import State,OverrideRCIn ,PositionTarget
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
    
        # 发布目标点
        self.target_point_pub = self.create_publisher(PoseStamped, 'setpoint_position/local', qos)    

        # 订阅当前状态
        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos)
        
        # 订阅需要运行的目标点
        self.target_point_sub = self.create_subscription(
            PoseStamped, 'set_usv_target_position', self.set_target_point_callback, qos)
        

        # 订阅避障目标点
        self.avoidance_target_point_sub=self.create_subscription(
            PoseStamped,'avoidance_position',self.set_avoidance_target_position_callback,qos)
        
        # 订阅避障标记
        self.avoidance_flag_sub=self.create_subscription(
            Bool,'avoidance_flag',self.set_avoidance_flag_callback,qos)
        
        # 发送目标位置循环     
        self.publish_target_timer=self.create_timer(0.1,self.publish_target)
    
        self.current_state = State() # 当前状态
        self.current_target_position =PoseStamped()#目标点
        self.avoidance_position=PoseStamped()#避障目标点
        self.avoidance_flag=Bool()#避障标记
 
    def state_callback(self, msg):
        if isinstance(msg, State):
            self.current_state = msg

    # 订阅到达目标点话题
    def set_target_point_callback(self, msg):
        if not isinstance(msg, PoseStamped):
            self.get_logger().info('目标坐标为空，忽略')
            return      
        self.current_target_position=msg
        # self.get_logger().info(f'接受到目标点：{self.current_target_position}')

    # 订阅目标速度   
    def set_target_velocity_callback(self,msg):
        if not isinstance(msg,Float32):
            self.get_logger().info('目标速度为空，忽略')
            return
        self.speed_value=self.speed_to_pwm(msg.data,0.0,100,1000,2000)

    # 订阅避障目标点
    def set_avoidance_target_position_callback(self,msg):
        if not isinstance(msg,PoseStamped):
            self.get_logger().info('避障目标为空，忽略')
            return
        self.avoidance_position.pose=msg.pose

    # 订阅避障标记
    def set_avoidance_flag_callback(self,msg):
        if not isinstance(msg,Bool):
            self.get_logger().info('避障标记为空，忽略')
            return
        self.avoidance_flag.data=msg.data

    # 限制速度范围    
    def speed_to_pwm(self, speed, speed_min=0.0, speed_max=100.0, pwm_min=1000, pwm_max=2000):
        # 限制 speed 范围
        speed = max(min(speed, speed_max), speed_min)
        return int(((speed - speed_min) / (speed_max - speed_min)) * (pwm_max - pwm_min) + pwm_min)
   
    # 发布目标点
    def publish_target(self):
        if not self.current_state.connected or not self.current_state.armed or self.current_state.mode != "GUIDED":
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
        #     px=self.avoidance_position.pose.position.x
        #     py=self.avoidance_position.pose.position.y
        #     pz=self.avoidance_position.pose.position.z  

        #     ox=self.avoidance_position.pose.orientation.x
        #     oy=self.avoidance_position.pose.orientation.y
        #     oz=self.avoidance_position.pose.orientation.z
        #     ow=self.avoidance_position.pose.orientation.w

        if px is None or py is None or pz is None :
            self.get_logger().info('目标点为空，忽略')
            return
        point_msg = PoseStamped()
        point_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')

        point_msg.pose.position.x= px
        point_msg.pose.position.y= py
        point_msg.pose.position.z= pz

        point_msg.pose.orientation.x=ox
        point_msg.pose.orientation.y=oy
        point_msg.pose.orientation.z=oz
        point_msg.pose.orientation.w=ow

        self.target_point_pub.publish(point_msg)
        # self.get_logger().info(f'发送给usv的目标点：{point_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = UsvControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()