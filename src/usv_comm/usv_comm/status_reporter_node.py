import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State,PositionTarget
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from common_interfaces.msg import UsvStatus
from geometry_msgs.msg import TwistStamped,PoseStamped

class StatusReporterNode(Node):
    def __init__(self):
        super().__init__('status_reporter_node')


        
        
        self.usv_battery=0.0
        self.usv_velocity=TwistStamped()
        self.state_msg = UsvStatus()
        self.usv_pose=PoseStamped()

        self.state_publisher=self.create_publisher(UsvStatus,f'state',10)
        

        # 订阅 MAVROS 的状态主题
        self.state_subscriber = self.create_subscription(
            State,
            f'mavros/state',  # 使用命名空间
            self.state_callback,
            10
        )

        # 订阅 MAVROS 的电池状态主题
        self.battery_subscriber = self.create_subscription(
            BatteryState,
            f'mavros/battery',  # 使用命名空间
            self.battery_callback,
            10
        )

        self.velocity_local_subscriber=self.create_subscription(
            TwistStamped, 
            f'mavros/local_position/velocity_local',
            self.velocity_local_callback,
            10
        )

        self.subscription_pose=self.create_subscription(
            PoseStamped,
            f'mavros/local_position/pose',
            self.pose_callback,
            10
        )
        self.state_timer=self.create_timer(1,self.state_timer_callback)

    def state_callback(self, msg):
            self.state_msg.mode = msg.mode
            self.state_msg.connected = msg.connected
            self.state_msg.armed = msg.armed
            self.state_msg.guided = msg.guided
            self.state_msg.manual_input = msg.manual_input

            self.state_msg.position=self.usv_pose.pose.position
            #self.state_msg.orientation=self.usv_pose.pose.orientation
            self.state_msg.battery_percentage = self.usv_battery

            self.state_msg.velocity=self.usv_velocity
        
 
        
     

    def battery_callback(self, msg):
        self.get_logger().info(f"电池状态：{msg}")
        self.usv_battery=msg.percentage

    def velocity_local_callback(self,msg):
        
        self.usv_velocity=msg

    def pose_callback(self,msg):
        self.get_logger().info(f"定位状态：{msg}")
        self.usv_pose=msg 

    def state_timer_callback(self):
         self.state_publisher.publish( self.state_msg)
        
    
def main():
    rclpy.init()
    node = StatusReporterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()