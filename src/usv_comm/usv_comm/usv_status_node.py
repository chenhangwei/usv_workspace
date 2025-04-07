import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State,PositionTarget
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
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

        self.state_publisher=self.create_publisher(UsvStatus,f'usv_current_state',10)

        self.is_running=False #是否运行中
        self.usv_state=State() #状态信息
        self.usv_battery=BatteryState() #电池电压信息
        self.usv_velocity=TwistStamped() #速度信息      
        self.usv_pose=PoseStamped() #定位信息

        self.usv_state_msg= UsvStatus() #需要发送的状态信息

        

        self.get_logger().info('状态报告节点已启动')

        # 订阅 MAVROS 的状态主题
        self.state_subscriber = self.create_subscription(
            State,
            f'mavros/state',  # 使用命名空间
            self.usv_state_callback,
            qos
        )

        # 订阅 MAVROS 的电池状态主题
        self.battery_subscriber = self.create_subscription(
            BatteryState,
            f'mavros/battery',  # 使用命名空间
            self.usv_battery_callback,
            qos
        )


        self.velocity_local_subscriber=self.create_subscription(
            TwistStamped, 
            f'mavros/local_position/velocity_local',
            self.usv_velocity_callback,
            qos
        )

        self.subscription_pose=self.create_subscription(
            PoseStamped,
            f'mavros/local_position/pose',
            self.usv_pose_callback,
            qos
        )
        self.state_timer=self.create_timer(1,self.state_timer_callback)


    def usv_state_callback(self, msg):
            if isinstance(msg, State):
                self.usv_state=msg 

    def usv_battery_callback(self, msg):
        if  isinstance(msg, BatteryState):
            self.usv_battery

    def usv_velocity_callback(self,msg):
        if isinstance(msg, TwistStamped):      
           self.usv_velocity=msg.twist.linear.x

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


        self.state_publisher.publish( self.usv_state_msg)
        
    
def main():
    rclpy.init()
    node = UsvStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()