import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleCommand, TrajectorySetpoint
from geometry_msgs.msg import Pose, Twist

class PX4InterfaceNode(Node):
    def __init__(self):
        # 获取命名空间（如果未指定，则为空）
        namespace = self.get_namespace().strip('/')
        if not namespace:
            namespace = 'usv1'  # 默认值
        super().__init__('px4_interface_node', namespace=namespace)

        # 使用 usv_id 参数，默认为命名空间值
        self.declare_parameter('usv_id', namespace)
        self.usv_id = self.get_parameter('usv_id').get_parameter_value().string_value

        # 订阅 PX4 数据（无需命名空间前缀，自动继承）
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, 10)#订阅来自PX4的数据里程计数据
        #订阅来自ground_station的数据
        self.ground_station_sub_pose = self.create_subscription(
            Pose, f'/{self.usv_id}/ground_station/pose', self.ground_station_pose_callback, 10)#订阅来自地面站的pose
        self.ground_station_sub_velocity = self.create_subscription(
            Twist, f'/{self.usv_id}/ground_station/velocity', self.ground_station_velocity_callback, 10)#订阅来自地面站的速度
        self.ground_station_sub_command = self.create_subscription(
            VehicleCommand, f'/{self.usv_id}/ground_station/vehicle_command', self.ground_station_vehicle_command_callback, 10)#订阅来自地面站的控制指令
        # 发布到带命名空间的话题
        self.pose_px_to_ground_pub = self.create_publisher(Pose, f'/{self.usv_id}/pose', 10)#发布到pose话题
        self.velocity_px_to_ground_pub = self.create_publisher(Twist, f'/{self.usv_id}/velocity', 10)#发布到velocity话题
        # 发布到 PX4 的控制指令
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, f'/{self.usv_id}/fmu/in/vehicle_command', 10)#发布到vehicle_command话题
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, f'/{self.usv_id}/fmu/in/trajectory_setpoint', 10)#发布到trajectory_setpoint话题-轨迹点
        self.pose_ground_to_px_pub = self.create_publisher(Pose, f'/{self.usv_id}/fmu/in/pose', 10)#发布到pose话题
        self.velocity_ground_to_px_pub = self.create_publisher(Twist, f'/{self.usv_id}/fmu/in/velocity', 10)#发布到velocity话题


        self.timer = self.create_timer(0.1, self.control_callback)
        
        self.get_logger().info(f'{self.usv_id} PX4 Interface Node started in namespace /{namespace}')
    #回调函数
    def odom_callback(self, msg):
        pose = Pose()
        pose.position.x = msg.position[0]
        pose.position.y = msg.position[1]
        pose.position.z = msg.position[2]
        pose.orientation.x = msg.q[1]
        pose.orientation.y = msg.q[2]
        pose.orientation.z = msg.q[3]
        pose.orientation.w = msg.q[0]

        velocity = Twist()
        velocity.linear.x = msg.velocity[0]
        velocity.linear.y = msg.velocity[1]
        velocity.linear.z = msg.velocity[2]

        self.pose_px_to_ground_pub.publish(pose) #发布到pose话题 到地面站
        self.velocity_px_to_ground_pub.publish(velocity)#发布到velocity话题 到地面站

        self.get_logger().info(f'{self.usv_id} Pose: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}')
    #地面站的pose回调函数
    def ground_station_pose_callback(self, msg):
        pose=Pose()
        pose.position.x=msg.position[0]
        pose.position.y=msg.position[1]
        pose.position.z=msg.position[2]
        pose.orientation.x=msg.orientation[1]
        pose.orientation.y=msg.orientation[2]
        pose.orientation.z=msg.orientation[3]
        pose.orientation.w=msg.orientation[0]
        self.pose_ground_to_px_pub.publish(pose)#发布到pose话题 到px4
        self.get_logger().info(f'{self.usv_id} Ground Station Pose: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}')

    #地面站的速度回调函数
    def ground_station_velocity_callback(self, msg):
        velocity=Twist()
        velocity.linear.x=msg.linear[0]
        velocity.linear.y=msg.linear[1]
        velocity.linear.z=msg.linear[2]
        self.velocity_ground_to_px_pub.publish(velocity)#发布到velocity话题 到px4
        self.get_logger().info(f'{self.usv_id} Ground Station Velocity: vx={msg.linear.x}, vy={msg.linear.y}, vz={msg.linear.z}')
    
    def ground_station_vehicle_command_callback(self,msg):
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'{self.usv_id} Ground Station Command: command={msg.command}, param1={msg.param1}, param2={msg.param2}')


    #定时器
    def control_callback(self):
        setpoint = TrajectorySetpoint()
        setpoint.position = [float(self.usv_id[-1]), 0.0, 0.0]
        setpoint.velocity = [0.5, 0.0, 0.0]
        self.setpoint_pub.publish(setpoint)


def main(args=None):
    rclpy.init(args=args)
    node = PX4InterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()