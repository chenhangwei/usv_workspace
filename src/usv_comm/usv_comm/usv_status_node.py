from xmlrpc.client import Boolean
from sympy import false
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State,PositionTarget
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String,Bool
from common_interfaces.msg import UsvStatus
from geometry_msgs.msg import TwistStamped,PoseStamped,Vector3
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class UsvStatusNode(Node):
    def __init__(self):
        super().__init__('usv_status_node')   

        # 创建 QoS 配置
        qos = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.BEST_EFFORT # 可靠性策略

            
        )

           # 创建 QoS 配置
        qos_a = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.RELIABLE # 可靠性策略

            
        )

        self.state_publisher=self.create_publisher(UsvStatus,'usv_state',10)

        self.reached_target=Bool() #是否运行中
        self.usv_state=State() #状态信息
        self.usv_battery=BatteryState() #电池电压信息
        self.usv_velocity=TwistStamped() #速度信息      
        self.usv_pose=PoseStamped() #定位信息

        self.usv_state_msg= UsvStatus() #需要发送的状态信息

        

        self.get_logger().info('状态报告节点已启动')

        # 订阅 MAVROS 的状态主题
        self.state_sub = self.create_subscription(
            State,
            f'state',  # 使用命名空间
            self.usv_state_callback,
            qos
        )

        # 订阅 MAVROS 的电池状态主题
        self.battery_sub = self.create_subscription(
            BatteryState,
            f'battery',  # 使用命名空间
            self.usv_battery_callback,
            qos
        )

        # 订阅 usv 的速度主题
        self.velocity_local_sub=self.create_subscription(
            TwistStamped, 
            f'local_position/velocity_local',
            self.usv_velocity_callback,
            qos
        )
        # 订阅 usv 的定位主题
        self.pos_sub=self.create_subscription(
            PoseStamped,
            f'local_position/pose',
            self.usv_pose_callback,
            qos 
        )
        # 订阅usv的运行状态
        self.reached_sub=self.create_subscription(
            Bool,
            f'reached_target',
            self.reached_target_callback,
            qos
        )

        self.state_timer=self.create_timer(1,self.state_timer_callback)

    def reached_target_callback(self,msg):
        if isinstance(msg, Bool):
            self.reached_target.data=msg.data          
        else:
            self.get_logger().error('接收到的消息类型不正确')

    def usv_status_callback(self,msg):
        if isinstance(msg, String):
            self.status=msg.data
        else:
            self.get_logger().error('接收到的消息类型不正确')

    def usv_state_callback(self, msg):
            if isinstance(msg, State):
                self.usv_state=msg 


    def usv_battery_callback(self, msg):
        if  isinstance(msg, BatteryState):
            self.usv_battery=msg



    def usv_velocity_callback(self,msg):
        if isinstance(msg, TwistStamped):      
           self.usv_velocity=msg


    def usv_pose_callback(self,msg):
        if isinstance(msg,PoseStamped):
            self.usv_pose=msg 


    def state_timer_callback(self):
        self.usv_state_msg.header.stamp=self.usv_state.header.stamp
 
        self.usv_state_msg.header.frame_id=self.usv_state.header.frame_id

        self.usv_state_msg.armed=self.usv_state.armed

        self.usv_state_msg.connected=self.usv_state.connected

        self.usv_state_msg.mode=self.usv_state.mode

        self.usv_state_msg.guided=self.usv_state.guided

        self.usv_state_msg.battery_voltage=self.usv_battery.voltage

        self.usv_state_msg.battery_percentage=self.usv_battery.percentage

        self.usv_state_msg.power_supply_status=self.usv_battery.power_supply_status

        self.usv_state_msg.velocity=self.usv_velocity.twist

        self.usv_state_msg.position=self.usv_pose.pose.position


        # 获取四元数
        quaternion = (
            self.usv_pose.pose.orientation.x,
            self.usv_pose.pose.orientation.y,
            self.usv_pose.pose.orientation.z,
            self.usv_pose.pose.orientation.w
        )
        # 转换为欧拉角（roll, pitch, yaw）
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        # 赋值给 yaw（单位：弧度）
        self.usv_state_msg.yaw = float(yaw)  # 转换为 float32
        # self.get_logger().info(f'当前偏角：{yaw}')
        self.usv_state_msg.reached_target=self.reached_target.data
        self.state_publisher.publish( self.usv_state_msg)

        
    
def main():
    rclpy.init()
    node = UsvStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()