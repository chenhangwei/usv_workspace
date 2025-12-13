"""
坐标转换节点 - PX4 uXRCE-DDS 版本（室内 UWB 定位专用）

功能：
1. 订阅地面站发送的 XYZ 目标点 (set_usv_target_position)
2. 订阅避障节点发送的 XYZ 目标点 (avoidance_position)
3. 通过 TrajectorySetpoint 发送本地坐标目标点

适用于室内 UWB 定位系统，使用本地坐标系（ENU/NED）。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped

# PX4 消息类型
from px4_msgs.msg import (
    VehicleLocalPosition,
    TrajectorySetpoint,
    OffboardControlMode,
)


class CoordTransformPx4Node(Node):
    """
    坐标转换节点 - PX4 uXRCE-DDS 版本（室内 UWB 定位专用）
    
    将地面站/避障节点的 ENU 坐标转换为 PX4 的 NED 坐标，
    通过 TrajectorySetpoint 发送给飞控。
    """

    def __init__(self):
        super().__init__('coord_transform_node')
        
        # =====================================================================
        # QoS 配置
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
        # 参数配置
        # =====================================================================
        self.declare_parameter('enable_coord_transform', True)
        self.declare_parameter('coordinate_system', 'ENU')  # 'ENU' 或 'NED'
        
        self.enabled = bool(self.get_parameter('enable_coord_transform').value)
        self.coordinate_system = self.get_parameter('coordinate_system').value
        
        # =====================================================================
        # PX4 发布器
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
        # PX4 订阅器（获取当前位置用于验证）
        # =====================================================================
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_px4
        )
        
        # =====================================================================
        # 目标点订阅
        # =====================================================================
        if self.enabled:
            # 订阅地面站的 XYZ 目标点
            self.xyz_target_sub = self.create_subscription(
                PoseStamped,
                'set_usv_target_position',
                self.xyz_target_callback,
                qos_reliable
            )
            
            # 订阅避障 XYZ 目标点
            self.avoidance_target_sub = self.create_subscription(
                PoseStamped,
                'avoidance_position',
                self.avoidance_target_callback,
                qos_reliable
            )
        
        # =====================================================================
        # 状态变量
        # =====================================================================
        self.current_local_pos = None
        
        # 日志输出
        if self.enabled:
            self.get_logger().info('✅ 坐标转换节点 (室内 UWB 模式) 已启动')
            self.get_logger().info(f'坐标系: {self.coordinate_system}')
        else:
            self.get_logger().info('⏸️ 坐标转换功能已禁用')

    def local_position_callback(self, msg: VehicleLocalPosition):
        """本地位置回调"""
        self.current_local_pos = msg

    def avoidance_target_callback(self, msg: PoseStamped):
        """接收避障节点的 XYZ 目标点"""
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            self.get_logger().debug(
                f"🚨 收到避障目标点: ({x:.2f}, {y:.2f}, {z:.2f})"
            )
            
            self._publish_local_setpoint(x, y, z)
            
        except Exception as e:
            self.get_logger().error(f'避障目标点处理失败: {e}')

    def xyz_target_callback(self, msg: PoseStamped):
        """接收地面站的 XYZ 目标点"""
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            self.get_logger().info(
                f"📥 收到目标点: ({x:.2f}, {y:.2f}, {z:.2f})"
            )
            
            self._publish_local_setpoint(x, y, z)
            
        except Exception as e:
            self.get_logger().error(f'目标点处理失败: {e}')

    def _publish_local_setpoint(self, x: float, y: float, z: float):
        """
        发布本地坐标目标点
        
        PX4 使用 NED 坐标系，需要进行转换
        """
        # ENU -> NED 坐标转换
        if self.coordinate_system == 'ENU':
            # ENU(x=东, y=北, z=上) -> NED(x=北, y=东, z=下)
            ned_x = y   # 北 = ENU的y
            ned_y = x   # 东 = ENU的x
            ned_z = -z  # 下 = -ENU的z
        else:
            # 输入已经是 NED
            ned_x = x
            ned_y = y
            ned_z = z
        
        # 发送 OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)
        
        # 发送 TrajectorySetpoint
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.position = [ned_x, ned_y, ned_z]
        setpoint_msg.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint_msg.yaw = float('nan')  # 保持当前航向
        setpoint_msg.yawspeed = float('nan')
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.setpoint_pub.publish(setpoint_msg)
        
        self.get_logger().info(
            f'📤 发布本地目标点 (NED): ({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoordTransformPx4Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
