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

        self.state_publisher=self.create_publisher(UsvStatus,'usv_state',10)

        self.is_runing=Bool() #是否运行中
        self.usv_state=State() #状态信息
        self.usv_battery=BatteryState() #电池电压信息
        self.usv_velocity=TwistStamped() #速度信息      
        self.usv_pose=PoseStamped() #定位信息

        self.usv_state_msg= UsvStatus() #需要发送的状态信息

        

        self.get_logger().info('状态报告节点已启动')

        # 订阅 MAVROS 的状态主题
        self.state_subscriber = self.create_subscription(
            State,
            f'state',  # 使用命名空间
            self.usv_state_callback,
            qos
        )

        # 订阅 MAVROS 的电池状态主题
        self.battery_subscriber = self.create_subscription(
            BatteryState,
            f'battery',  # 使用命名空间
            self.usv_battery_callback,
            qos
        )


        self.velocity_local_subscriber=self.create_subscription(
            TwistStamped, 
            f'local_position/velocity_local',
            self.usv_velocity_callback,
            qos
        )

        self.subscription_pose=self.create_subscription(
            PoseStamped,
            f'local_position/pose',
            self.usv_pose_callback,
            qos
        )

        self.subscription_state=self.create_subscription(
            String,
            f'status',
            self.usv_status_callback,
            qos
        )

        self.subscription_is_runing=self.create_subscription(
            Bool,
            f'is_running',
            self.usv_is_runing_callback,
            qos
        )

        self.state_timer=self.create_timer(1,self.state_timer_callback)

    def usv_is_runing_callback(self,msg):
        if isinstance(msg, Bool):
            self.is_runing.data=msg.data
            self.get_logger().info(f'当前运行状态：{self.is_runing.data}')
        else:
            self.get_logger().error('接收到的消息类型不正确')

    def usv_status_callback(self,msg):
        if isinstance(msg, String):
            self.status=msg.data
            self.get_logger().info(f'当前状态：{self.status}')
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
        self.get_logger().info(f'当前时间戳：{self.usv_state_msg.header.stamp}')    
        self.usv_state_msg.header.frame_id=self.usv_state.header.frame_id
        self.get_logger().info(f'当前帧ID:{self.usv_state_msg.header.frame_id}')
        self.usv_state_msg.armed=self.usv_state.armed
        self.get_logger().info(f'当前解锁状态：{self.usv_state_msg.armed}')
        self.usv_state_msg.connected=self.usv_state.connected
        self.get_logger().info(f'当前连接状态：{self.usv_state_msg.connected}')
        self.usv_state_msg.mode=self.usv_state.mode
        self.get_logger().info(f'当前模式：{self.usv_state_msg.mode}')
        self.usv_state_msg.guided=self.usv_state.guided
        self.get_logger().info(f'当前引导状态：{self.usv_state_msg.guided}')


        self.usv_state_msg.battery_voltage=self.usv_battery.voltage
        self.get_logger().info(f'当前电池电压：{self.usv_state_msg.battery_voltage}')
        self.usv_state_msg.battery_percentage=self.usv_battery.percentage
        self.get_logger().info(f'当前电池电量：{self.usv_state_msg.battery_percentage}')
        self.usv_state_msg.power_supply_status=self.usv_battery.power_supply_status
        self.get_logger().info(f'当前电池状态：{self.usv_state_msg.power_supply_status}')
        self.usv_state_msg.velocity=self.usv_velocity.twist
        self.get_logger().info(f'当前线速度：{self.usv_state_msg.velocity.linear}')
        self.get_logger().info(f'当前角速度：{self.usv_state_msg.velocity.angular}')
        self.usv_state_msg.position=self.usv_pose.pose.position
        self.get_logger().info(f'当前定位：{self.usv_state_msg.position}')

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
        self.get_logger().info(f'当前偏角：{yaw}')

        self.usv_state_msg.is_runing=self.is_runing.data
        self.get_logger().info(f'当前运行状态：{self.usv_state_msg.is_runing}')


        self.state_publisher.publish( self.usv_state_msg)

        
    
def main():
    rclpy.init()
    node = UsvStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()