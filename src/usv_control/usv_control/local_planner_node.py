import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool,SetMode
from tf_transformations import euler_from_quaternion
import numpy as np

class UsvLocalPlanner(Node):
    def __init__(self):
        super().__init__('usv_local_planner')

        self.subscription_pose=self.create_subscription(
            PoseStamped,
            f'mavros/local_position/pose',
            self.pose_callback,
            10
            )
        self.subscription_state=self.create_subscription(
            State,
            f'mavros/state',
            self.state_callback,
            10
        )
        self.publisher_velocity=self.create_publisher(
            TwistStamped,
            f'mavros/setpoint_velocity/cmd_vel',
            10
            )
        
        self.arming_client=self.create_client(
            CommandBool,
            f'mavros/cmd/arming'

        )

        self.mode_client=self.create_client(
            SetMode,
            f'mavros/set_mode'
        )

        #初始化变量
        self.current_pose=None
        self.current_state=None
        self.path=[(0.0,0.0),(0.0,0.0),(0.0,0.0)]
        self.target_index=0

        #定时器
        self.timer=self.create_timer(0.1,self.planner_callback)
        #初始化设备
        self.initialize_usv()

        def pose_callback(self,msg):
            self.current_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.orientation)
        def state_callback(self,msg):
            self.current_state = msg
        def initialize_usv(self):
            # 等待服务可用
            self.arming_client.wait_for_service(timeout_sec=5.0)
            self.mode_client.wait_for_service(timeout_sec=5.0)
             # 切换到 Offboard 模式
            mode_req = SetMode.Request()
            mode_req.custom_mode = "OFFBOARD"
            self.mode_client.call_async(mode_req)

             # 解锁漫游车
            arm_req = CommandBool.Request()
            arm_req.value = True
            self.arming_client.call_async(arm_req)

        def planner_callback(self):
            if  self.current_pose is None or self.current_state is None:
                self.get_logger().warn("等待位姿或状态数据...")
                return

            if  not self.current_state.armed or self.current_state.mode != "OFFBOARD":
                self.get_logger().warn("漫游车未解锁或未在 Offboard 模式")
                return
            # 获取当前位置和目标点
            curr_x, curr_y, curr_orient = self.current_pose
            target_x, target_y = self.path[self.target_index]

            # 计算方向和距离
            dx = target_x - curr_x
            dy = target_y - curr_y
            distance = np.sqrt(dx**2 + dy**2)
            target_angle = np.arctan2(dy, dx)

            # 假设朝向用 quaternion 转换为 yaw
            
            curr_yaw = euler_from_quaternion([curr_orient.x, curr_orient.y, curr_orient.z, curr_orient.w])[2]
            angle_error = target_angle - curr_yaw

            # 简单控制律
            linear_speed = min(0.1, distance)  # 最大 0.1 m/s
            angular_speed = np.clip(angle_error, -0.5, 0.5)  # 最大 ±0.5 rad/s

            # 接近目标时切换到下一个点
            if distance < 3:
                self.target_index += 1
                if self.target_index >= len(self.path):
                    linear_speed = 0.0
                    angular_speed = 0.0
                    self.get_logger().info("路径完成！")

            # 发布速度指令
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x = float(linear_speed)
            twist.twist.angular.z = float(angular_speed)
            self.publisher_velocity.publish(twist)        
                                                        
def main(args=None):
    rclpy.init(args=args)
    node = UsvLocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()