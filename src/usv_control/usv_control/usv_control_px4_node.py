"""
无人球控制节点 - PX4 uXRCE-DDS 版本

该节点负责处理无人球的目标点控制逻辑。
使用 TrajectorySetpoint 消息发送目标点，替代 MAVROS 的 PositionTarget。

话题映射：
- MAVROS /mavros/setpoint_raw/local -> /fmu/in/trajectory_setpoint
- MAVROS /mavros/local_position/pose -> /fmu/out/vehicle_local_position
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# PX4 消息类型
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
    VehicleOdometry
)

# 导入 common_utils 工具
from common_utils import ParamLoader, ParamValidator


class UsvControlPx4Node(Node):
    """
    无人球控制节点类 - PX4 uXRCE-DDS 版本
    
    该节点实现目标点控制逻辑，处理常规目标点和避障目标点，
    根据避障标志决定使用哪个目标点，并将选定的目标点发布给 PX4 飞控。
    """

    def __init__(self):
        """初始化无人球控制节点"""
        super().__init__('usv_control_px4_node')
        
        # =====================================================================
        # 参数加载
        # =====================================================================
        param_loader = ParamLoader(self)
        
        self.publish_rate = param_loader.load_param(
            'publish_rate',
            20.0,
            ParamValidator.frequency,
            '目标点发布频率(Hz)'
        )
        self.frame_id = param_loader.load_param(
            'frame_id',
            'map',
            ParamValidator.non_empty_string,
            '坐标系ID'
        )
        
        # 声明额外参数
        self.declare_parameter('target_reach_threshold', 1.0)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('coordinate_system', 'NED')  # NED 或 ENU
        
        self.target_reach_threshold = self.get_parameter('target_reach_threshold').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.coordinate_system = self.get_parameter('coordinate_system').value

        # =====================================================================
        # QoS 配置 - PX4 uXRCE-DDS 使用 BEST_EFFORT
        # =====================================================================
        qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # =====================================================================
        # 发布器 - 发送目标点到 PX4
        # =====================================================================
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos_px4
        )
        
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            qos_px4
        )

        # =====================================================================
        # 订阅器 - PX4 状态和位置
        # =====================================================================
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_px4
        )
        
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.status_callback,
            qos_px4
        )

        # =====================================================================
        # 订阅器 - 地面站命令（保持原有接口兼容）
        # =====================================================================
        self.target_sub = self.create_subscription(
            PoseStamped,
            'set_usv_target_position',
            self.target_callback,
            qos_reliable
        )
        
        self.avoidance_sub = self.create_subscription(
            PoseStamped,
            'avoidance_position',
            self.avoidance_target_callback,
            qos_reliable
        )
        
        self.avoidance_flag_sub = self.create_subscription(
            Bool,
            'avoidance_flag',
            self.avoidance_flag_callback,
            qos_reliable
        )
        
        self.clear_target_sub = self.create_subscription(
            Bool,
            'clear_target',
            self.clear_target_callback,
            qos_reliable
        )

        # =====================================================================
        # 状态变量
        # =====================================================================
        self.current_position = None           # 当前位置 (VehicleLocalPosition)
        self.target_position = None            # 常规目标点 (PoseStamped)
        self.avoidance_position = None         # 避障目标点 (PoseStamped)
        self.avoidance_active = False          # 避障模式是否激活
        self.vehicle_status = None             # 飞控状态
        self.target_active = False             # 目标点是否激活
        self.local_position_valid = False      # 本地位置是否有效
        self.offboard_mode_active = False      # OFFBOARD 模式是否激活

        # =====================================================================
        # 定时器
        # =====================================================================
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_setpoint)

        # =====================================================================
        # 日志记录
        # =====================================================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('PX4 uXRCE-DDS 控制节点已启动')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'坐标系: {self.coordinate_system}')
        self.get_logger().info(f'目标到达阈值: {self.target_reach_threshold} m')
        self.get_logger().info('📤 发布话题: fmu/in/trajectory_setpoint')
        self.get_logger().info('📥 订阅话题: fmu/out/vehicle_local_position')
        self.get_logger().info('=' * 60)

    def local_position_callback(self, msg: VehicleLocalPosition):
        """
        本地位置回调
        
        Args:
            msg (VehicleLocalPosition): PX4 本地位置消息
        """
        self.current_position = msg
        
        # 检查位置是否有效
        if msg.xy_valid and msg.z_valid:
            if not self.local_position_valid:
                self.local_position_valid = True
                self.get_logger().info(
                    f'✅ 本地位置有效: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})'
                )

    def status_callback(self, msg: VehicleStatus):
        """
        飞控状态回调
        
        Args:
            msg (VehicleStatus): PX4 飞控状态消息
        """
        self.vehicle_status = msg
        # nav_state == 14 表示 OFFBOARD 模式
        self.offboard_mode_active = (msg.nav_state == 14)

    def target_callback(self, msg: PoseStamped):
        """
        目标点回调
        
        Args:
            msg (PoseStamped): 目标位置消息
        """
        self.target_position = msg
        self.target_active = True
        
        self.get_logger().info(
            f'📍 收到目标点: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
        )

    def avoidance_target_callback(self, msg: PoseStamped):
        """
        避障目标点回调
        
        Args:
            msg (PoseStamped): 避障目标位置消息
        """
        self.avoidance_position = msg

    def avoidance_flag_callback(self, msg: Bool):
        """
        避障标志回调
        
        Args:
            msg (Bool): 避障模式标志
        """
        if msg.data != self.avoidance_active:
            self.avoidance_active = msg.data
            status = "激活" if msg.data else "停用"
            self.get_logger().info(f'🚧 避障模式 {status}')

    def clear_target_callback(self, msg: Bool):
        """
        清除目标点回调
        
        Args:
            msg (Bool): 清除标志
        """
        if msg.data:
            self.target_position = None
            self.avoidance_position = None
            self.target_active = False
            self.get_logger().info('🗑️ 目标点已清除')

    def publish_setpoint(self):
        """
        发布目标点到 PX4
        
        将选定的目标点（常规或避障）转换为 PX4 TrajectorySetpoint 格式并发布。
        同时发布 OffboardControlMode 以保持 OFFBOARD 模式。
        """
        # 如果没有目标点，不发布
        if not self.target_active:
            return
            
        # 选择目标点（避障优先）
        target = self.avoidance_position if self.avoidance_active and self.avoidance_position else self.target_position
        
        if target is None:
            return

        # =====================================================================
        # 发布 OffboardControlMode（必须持续发送以保持 OFFBOARD 模式）
        # =====================================================================
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)

        # =====================================================================
        # 发布 TrajectorySetpoint
        # =====================================================================
        setpoint = TrajectorySetpoint()
        
        # 坐标转换：ROS 使用 ENU，PX4 使用 NED
        # ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
        if self.coordinate_system == 'NED':
            # 如果输入已经是 NED，直接使用
            setpoint.position[0] = target.pose.position.x  # North
            setpoint.position[1] = target.pose.position.y  # East
            setpoint.position[2] = target.pose.position.z  # Down
        else:
            # ENU 到 NED 转换
            setpoint.position[0] = target.pose.position.y   # North = East_enu
            setpoint.position[1] = target.pose.position.x   # East = North_enu
            setpoint.position[2] = -target.pose.position.z  # Down = -Up_enu
        
        # 速度设为 NaN（使用位置控制）
        setpoint.velocity[0] = float('nan')
        setpoint.velocity[1] = float('nan')
        setpoint.velocity[2] = float('nan')
        
        # 加速度设为 NaN
        setpoint.acceleration[0] = float('nan')
        setpoint.acceleration[1] = float('nan')
        setpoint.acceleration[2] = float('nan')
        
        # 从四元数计算偏航角
        yaw = self._quaternion_to_yaw(target.pose.orientation)
        
        # ENU 到 NED 偏航角转换
        if self.coordinate_system != 'NED':
            # ENU yaw: 0 = East, 增加逆时针
            # NED yaw: 0 = North, 增加顺时针
            yaw = math.pi / 2.0 - yaw
        
        setpoint.yaw = yaw
        setpoint.yawspeed = float('nan')  # 使用偏航角控制
        
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.setpoint_pub.publish(setpoint)

    def _quaternion_to_yaw(self, q) -> float:
        """
        从四元数提取偏航角
        
        Args:
            q: 四元数 (geometry_msgs/Quaternion)
            
        Returns:
            float: 偏航角（弧度）
        """
        # 使用 atan2 计算偏航角
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_distance_to_target(self) -> float:
        """
        计算当前位置到目标点的距离
        
        Returns:
            float: 距离（米），如果数据无效返回 -1
        """
        if self.current_position is None or self.target_position is None:
            return -1.0
            
        target = self.avoidance_position if self.avoidance_active else self.target_position
        if target is None:
            return -1.0
        
        # 转换坐标系
        if self.coordinate_system == 'NED':
            dx = target.pose.position.x - self.current_position.x
            dy = target.pose.position.y - self.current_position.y
            dz = target.pose.position.z - self.current_position.z
        else:
            # ENU 输入，当前位置是 NED
            dx = target.pose.position.y - self.current_position.x
            dy = target.pose.position.x - self.current_position.y
            dz = -target.pose.position.z - self.current_position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def is_target_reached(self) -> bool:
        """
        检查是否到达目标点
        
        Returns:
            bool: 是否到达目标点
        """
        distance = self.get_distance_to_target()
        if distance < 0:
            return False
        return distance < self.target_reach_threshold


def main(args=None):
    """节点主函数"""
    rclpy.init(args=args)
    node = UsvControlPx4Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
