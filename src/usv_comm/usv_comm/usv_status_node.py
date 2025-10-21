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

        # 参数：目标到达阈值（米）与距离计算模式（2d/3d）
        self.declare_parameter('target_reach_threshold', 1.0)
        self.declare_parameter('distance_mode', '2d')
        # 根据节点命名空间推断 usv_id（例如 /usv_02 -> usv_02），允许通过参数覆盖
        ns_guess = self.get_namespace().lstrip('/') if self.get_namespace() else 'usv_01'
        self.declare_parameter('usv_id', ns_guess)
        try:
            self.target_reach_threshold = float(self.get_parameter('target_reach_threshold').get_parameter_value().double_value)
        except Exception:
            self.target_reach_threshold = 1.0
        try:
            self.distance_mode = str(self.get_parameter('distance_mode').get_parameter_value().string_value).lower()
        except Exception:
            self.distance_mode = '2d'

        # 读取 usv_id 参数（已声明），优先使用显式参数，回退为命名空间推断值
        try:
            usv_id_val = self.get_parameter('usv_id').get_parameter_value().string_value
            self.usv_id = usv_id_val.lstrip('/') if usv_id_val else ns_guess
        except Exception:
            self.usv_id = ns_guess
        # 若参数与命名空间不一致，记录警告以提醒用户
        try:
            ns_now = self.get_namespace().lstrip('/') if self.get_namespace() else ''
            if ns_now and ns_now != self.usv_id:
                self.get_logger().warn(f"usv_id 参数 ({self.usv_id}) 与节点命名空间 ({ns_now}) 不一致；优先使用参数值。若想按命名空间自动设置，请移除该参数。")
        except Exception:
            pass

        # 初始化状态变量
        self.target_point = Point()  # 目标点位置
        self.usv_state = State()     # 飞控状态信息
        self.usv_battery = BatteryState()  # 电池状态信息
        self.usv_velocity = TwistStamped()  # 速度信息
        self.usv_pose = PoseStamped()  # 定位信息

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
            # 每次创建新的消息实例，避免重用导致订阅端读取到中间态
            msg = UsvStatus()

            # 安全填充 header：优先使用定位 header，否则使用当前时钟
            # 使用初始化时缓存的 usv_id，避免每次查询参数
            try:
                msg.usv_id = getattr(self, 'usv_id', None) or str(self.get_parameter('usv_id').get_parameter_value().string_value)
            except Exception:
                msg.usv_id = 'usv_01'
            try:
                msg.header.stamp = self.usv_pose.header.stamp
                msg.header.frame_id = self.usv_pose.header.frame_id if self.usv_pose.header.frame_id else 'map'
            except Exception:
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'

            # 填写飞控状态信息（存在性检查以防止属性缺失）
            msg.armed = getattr(self.usv_state, 'armed', False)
            msg.connected = getattr(self.usv_state, 'connected', False)
            msg.mode = getattr(self.usv_state, 'mode', '')
            msg.guided = getattr(self.usv_state, 'guided', False)

            # 电池信息
            msg.battery_voltage = getattr(self.usv_battery, 'voltage', 0.0)
            msg.battery_percentage = getattr(self.usv_battery, 'percentage', 0.0)
            msg.power_supply_status = getattr(self.usv_battery, 'power_supply_status', 0)

            # 位置与速度
            try:
                msg.position = self.usv_pose.pose.position
            except Exception:
                # 为空时使用默认Point
                msg.position = Point()
            try:
                msg.velocity = self.usv_velocity.twist
            except Exception:
                pass

            # 计算偏航角
            try:
                quaternion = (
                    self.usv_pose.pose.orientation.x,
                    self.usv_pose.pose.orientation.y,
                    self.usv_pose.pose.orientation.z,
                    self.usv_pose.pose.orientation.w
                )
                _, _, yaw = euler_from_quaternion(quaternion)
                msg.yaw = float(yaw)
            except Exception:
                msg.yaw = float(getattr(msg, 'yaw', 0.0))

            # 计算到目标点的距离（支持2D/3D）并记录到本地变量（不写入消息，因为 UsvStatus 中无该字段）
            try:
                dx = msg.position.x - self.target_point.x
                dy = msg.position.y - self.target_point.y
                dz = msg.position.z - self.target_point.z
                if self.distance_mode == '3d':
                    distance = sqrt(dx*dx + dy*dy + dz*dz)
                else:
                    distance = sqrt(dx*dx + dy*dy)
            except Exception:
                distance = float('inf')

            # 获取并发布温度（安全方式，不抛异常）
            temp_val = 0.0
            try:
                temp_val = float(self.get_temperature())
            except Exception:
                temp_val = 0.0
            # 如果 get_temperature 返回的是毫度级别 (>1000)，尝试转换为摄氏度
            if temp_val > 1000:
                try:
                    temp_val = temp_val / 1000.0
                except Exception:
                    pass
            msg.temperature = temp_val
            temp_msg = Float32()
            temp_msg.data = float(temp_val)
            self.temperature_publisher.publish(temp_msg)

            # 发布状态消息
            self.state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'状态信息发布过程中发生错误: {e}')

    def get_temperature(self):
        """获取系统温度
        
        Returns:
            float: 系统温度值（毫摄氏度）
            
        Raises:
            IOError: 无法读取温度文件时抛出
        """
        # 尝试多个常见路径以提高兼容性，若均失败则返回0.0
        candidates = [
            '/sys/class/thermal/thermal_zone0/temp',
            '/sys/class/hwmon/hwmon0/temp1_input'
        ]
        for path in candidates:
            try:
                with open(path, 'r') as f:
                    raw = f.read().strip()
                    if not raw:
                        continue
                    val = float(raw)
                    return val
            except Exception:
                continue
        # 如果所有路径均失败，记录并返回0.0
        self.get_logger().warn('无法读取系统温度，返回0.0')
        return 0.0


def main():
    """主函数"""
    rclpy.init()
    node = UsvStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()