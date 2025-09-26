from math import sqrt
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget, WaypointReached
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool, Float32, Int32
from common_interfaces.msg import UsvStatus
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3, Point
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class UsvStatusNode(Node):
    """无人船状态节点类
    
    该节点负责收集无人船的各种状态信息并整合发布。
    主要功能包括：
    1. 订阅飞控状态、电池状态、位置和速度信息
    2. 整合所有状态信息并发布到统一的状态主题
    3. 获取并发布系统温度
    """

    def __init__(self):
        """初始化无人船状态节点"""
        super().__init__('usv_status_node')


        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 创建发布者
        self.state_publisher = self.create_publisher(UsvStatus, 'usv_state', 10)
        self.temperature_publisher = self.create_publisher(Float32, 'usv_temperature', 10)

        # 初始化状态变量
        self.target_point = Point()  # 目标点位置
        self.usv_state = State()     # 飞控状态信息
        self.usv_battery = BatteryState()  # 电池状态信息
        self.usv_velocity = TwistStamped()  # 速度信息
        self.usv_pose = PoseStamped()  # 定位信息
        self.usv_state_msg = UsvStatus()  # 要发布的状态信息

        self.get_logger().info('状态报告节点已启动')

        # 订阅 MAVROS 的状态主题
        self.state_sub = self.create_subscription(
            State,
            'state',
            self.usv_state_callback,
            qos_best_effort
        )

        # 订阅 MAVROS 的电池状态主题
        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery',
            self.usv_battery_callback,
            qos_best_effort
        )

        # 订阅无人船的速度主题
        self.velocity_local_sub = self.create_subscription(
            TwistStamped,
            'local_position/velocity_local',
            self.usv_velocity_callback,
            qos_best_effort
        )

        # 订阅无人船的定位主题
        self.pos_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',
            self.usv_pose_callback,
            qos_best_effort
        )

        # 订阅目标点位置
        self.target_sub = self.create_subscription(
            PositionTarget,
            'setpoint_raw/local',
            self.target_callback,
            qos_best_effort
        )

        # 定时器，定期发布状态信息
        self.state_timer = self.create_timer(1.0, self.state_timer_callback)

    def target_callback(self, msg):
        """目标点回调函数
        
        Args:
            msg (PositionTarget): 包含目标点位置的消息
        """
        if isinstance(msg, PositionTarget):
            self.target_point.x = msg.position.x
            self.target_point.y = msg.position.y
            self.target_point.z = msg.position.z
        else:
            self.get_logger().error('接收到的消息类型不正确')

    def usv_state_callback(self, msg):
        """飞控状态回调函数
        
        Args:
            msg (State): 包含飞控状态的消息
        """
        if isinstance(msg, State):
            self.usv_state = msg

    def usv_battery_callback(self, msg):
        """电池状态回调函数
        
        Args:
            msg (BatteryState): 包含电池状态的消息
        """
        if isinstance(msg, BatteryState):
            self.usv_battery = msg

    def usv_velocity_callback(self, msg):
        """速度回调函数
        
        Args:
            msg (TwistStamped): 包含速度信息的消息
        """
        if isinstance(msg, TwistStamped):
            self.usv_velocity = msg

    def usv_pose_callback(self, msg):
        """位置回调函数
        
        Args:
            msg (PoseStamped): 包含位置信息的消息
        """
        if isinstance(msg, PoseStamped):
            self.usv_pose = msg

    def state_timer_callback(self):
        """定时器回调函数，定期发布状态信息"""
        try:
            # 更新状态消息的时间戳和帧ID
            self.usv_state_msg.header.stamp = self.usv_state.header.stamp
            self.usv_state_msg.header.frame_id = self.usv_state.header.frame_id

            # 更新飞控状态信息
            self.usv_state_msg.armed = self.usv_state.armed
            self.usv_state_msg.connected = self.usv_state.connected
            self.usv_state_msg.mode = self.usv_state.mode
            self.usv_state_msg.guided = self.usv_state.guided

            # 更新电池状态信息
            self.usv_state_msg.battery_voltage = self.usv_battery.voltage
            self.usv_state_msg.battery_percentage = self.usv_battery.percentage
            self.usv_state_msg.power_supply_status = self.usv_battery.power_supply_status

            # 更新速度和位置信息
            self.usv_state_msg.velocity = self.usv_velocity.twist
            self.usv_state_msg.position = self.usv_pose.pose.position

            # 计算偏航角（四元数转欧拉角）
            quaternion = (
                self.usv_pose.pose.orientation.x,
                self.usv_pose.pose.orientation.y,
                self.usv_pose.pose.orientation.z,
                self.usv_pose.pose.orientation.w
            )
            # 转换为欧拉角（roll, pitch, yaw）
            _, _, yaw = euler_from_quaternion(quaternion)
            # 赋值给 yaw（单位：弧度）
            self.usv_state_msg.yaw = float(yaw)

            # 获取并发布温度信息
            temperature_msg = Float32()
            try:
                temperature_msg.data = self.get_temperature() / 1000.0  # 转换为摄氏度
                self.usv_state_msg.temperature = temperature_msg.data
                self.temperature_publisher.publish(temperature_msg)
            except Exception as e:
                self.get_logger().warn(f'获取温度信息失败: {e}')
                self.usv_state_msg.temperature = 0.0

            # 发布状态信息
            self.state_publisher.publish(self.usv_state_msg)

            # 记录日志信息
        except Exception as e:
            self.get_logger().error(f'状态信息发布过程中发生错误: {e}')

    def get_temperature(self):
        """获取系统温度
        
        Returns:
            float: 系统温度值（毫摄氏度）
            
        Raises:
            IOError: 无法读取温度文件时抛出
        """
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read())
            return temp
        except Exception as e:
            self.get_logger().warn(f'读取温度文件失败: {e}')
            raise IOError(f'无法读取系统温度: {e}')


def main():
    """主函数"""
    rclpy.init()
    node = UsvStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()