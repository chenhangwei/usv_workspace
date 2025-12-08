"""
坐标转换节点 - PX4 uXRCE-DDS 版本

功能：
1. 订阅地面站发送的 XYZ 目标点 (set_usv_target_position)
2. 订阅避障节点发送的 XYZ 目标点 (avoidance_position)
3. 将 XYZ 转换为 GPS 坐标（lat/lon/alt）
4. 通过 PX4 VehicleGlobalPosition 接口发送目标

话题映射：
- MAVROS setpoint_raw/global -> /fmu/in/vehicle_command (SET_GPS_GLOBAL_ORIGIN)
- 或直接使用 TrajectorySetpoint 进行本地坐标控制

注意：PX4 uXRCE-DDS 通常使用本地坐标系统，GPS 目标点通过 VehicleCommand 发送
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped

# PX4 消息类型
from px4_msgs.msg import (
    VehicleCommand,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    TrajectorySetpoint,
    OffboardControlMode,
)

# 导入common_utils工具
from common_utils import ParamLoader


class CoordTransformPx4Node(Node):
    """
    坐标转换节点 - PX4 uXRCE-DDS 版本
    
    支持两种工作模式：
    1. 本地坐标直传：直接使用 TrajectorySetpoint (推荐用于室内/UWB 定位)
    2. GPS坐标转换：将 XYZ 转换为 GPS 发送给飞控 (用于室外 GPS 定位)
    """
    
    # VehicleCommand 命令 ID
    VEHICLE_CMD_DO_SET_HOME = 179
    VEHICLE_CMD_DO_REPOSITION = 192

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
        param_loader = ParamLoader(self)
        
        # GPS 原点配置（A0基站坐标）
        gps_origin = param_loader.load_gps_origin(
            default_lat=22.5180977,
            default_lon=113.9007239,
            default_alt=-5.17
        )
        self.origin_lat = gps_origin['lat']
        self.origin_lon = gps_origin['lon']
        self.origin_alt = gps_origin['alt']
        
        # 工作模式
        self.declare_parameter('enable_coord_transform', True)
        self.declare_parameter('mode', 'local')  # 'local' 或 'gps'
        self.declare_parameter('coordinate_system', 'ENU')  # 'ENU' 或 'NED'
        
        self.enabled = bool(self.get_parameter('enable_coord_transform').value)
        self.mode = self.get_parameter('mode').value
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
        
        self.command_pub = self.create_publisher(
            VehicleCommand,
            'fmu/in/vehicle_command',
            qos_px4
        )
        
        # =====================================================================
        # PX4 订阅器（获取当前位置用于坐标验证）
        # =====================================================================
        self.global_pos_sub = self.create_subscription(
            VehicleGlobalPosition,
            'fmu/out/vehicle_global_position',
            self.global_position_callback,
            qos_px4
        )
        
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
        self.current_global_pos = None
        self.current_local_pos = None
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        
        # 日志输出
        if self.enabled:
            self.get_logger().info('✅ 坐标转换节点 (PX4 uXRCE-DDS) 已启动')
            self.get_logger().info(f'工作模式: {self.mode}')
            self.get_logger().info(f'坐标系: {self.coordinate_system}')
            self.get_logger().info(
                f'GPS 原点: ({self.origin_lat:.7f}°, {self.origin_lon:.7f}°, {self.origin_alt:.2f}m)'
            )
        else:
            self.get_logger().info('⏸️ 坐标转换功能已禁用')

    def global_position_callback(self, msg: VehicleGlobalPosition):
        """全局位置回调（用于获取 EKF 参考点）"""
        self.current_global_pos = msg
        
        # 记录 EKF 参考点（首次有效位置）
        if self.ref_lat is None and msg.lat != 0.0 and msg.lon != 0.0:
            self.ref_lat = msg.lat
            self.ref_lon = msg.lon
            self.ref_alt = msg.alt
            self.get_logger().info(
                f'📍 EKF 参考点已获取: ({self.ref_lat:.7f}°, {self.ref_lon:.7f}°, {self.ref_alt:.2f}m)'
            )

    def local_position_callback(self, msg: VehicleLocalPosition):
        """本地位置回调"""
        self.current_local_pos = msg

    def avoidance_target_callback(self, msg: PoseStamped):
        """
        接收避障节点的 XYZ 目标点
        """
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            self.get_logger().debug(
                f"🚨 收到避障目标点: ({x:.2f}, {y:.2f}, {z:.2f})"
            )
            
            self._process_target(x, y, z, is_avoidance=True)
            
        except Exception as e:
            self.get_logger().error(f'避障目标点处理失败: {e}')

    def xyz_target_callback(self, msg: PoseStamped):
        """
        接收地面站的 XYZ 目标点
        """
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            self.get_logger().info(
                f"📥 收到目标点: ({x:.2f}, {y:.2f}, {z:.2f})"
            )
            
            self._process_target(x, y, z, is_avoidance=False)
            
        except Exception as e:
            self.get_logger().error(f'目标点处理失败: {e}')

    def _process_target(self, x: float, y: float, z: float, is_avoidance: bool = False):
        """
        处理目标点
        
        Args:
            x, y, z: 输入坐标（ENU 或 NED，取决于 coordinate_system 参数）
            is_avoidance: 是否为避障目标点
        """
        if self.mode == 'local':
            # 本地坐标直传模式
            self._publish_local_setpoint(x, y, z)
        else:
            # GPS 坐标转换模式
            gps_coord = self._xyz_to_gps(x, y, z)
            self._publish_gps_target(gps_coord['lat'], gps_coord['lon'], gps_coord['alt'])

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

    def _publish_gps_target(self, lat: float, lon: float, alt: float):
        """
        发布 GPS 目标点（使用 VehicleCommand DO_REPOSITION）
        
        Args:
            lat: 纬度 (度)
            lon: 经度 (度)
            alt: 海拔 (米)
        """
        msg = VehicleCommand()
        msg.command = self.VEHICLE_CMD_DO_REPOSITION
        msg.param1 = -1.0  # Ground speed, -1 for default
        msg.param2 = 0.0   # Bitmask
        msg.param3 = 0.0   # Reserved
        msg.param4 = float('nan')  # Yaw, NaN for unchanged
        msg.param5 = lat   # Latitude
        msg.param6 = lon   # Longitude
        msg.param7 = alt   # Altitude
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(msg)
        
        self.get_logger().info(
            f'📤 发布 GPS 目标点: ({lat:.7f}°, {lon:.7f}°, {alt:.2f}m)'
        )

    def _xyz_to_gps(self, x: float, y: float, z: float) -> dict:
        """
        本地 XYZ (ENU) → GPS (lat/lon/alt)
        
        公式（适用于小范围 <100km，误差 <0.5%）：
        - 纬度1度 ≈ 111320米
        - 经度1度 ≈ 111320 * cos(纬度)米
        
        Args:
            x: 东向距离（米）
            y: 北向距离（米）
            z: 天向距离（米）
        
        Returns:
            {'lat': 纬度(度), 'lon': 经度(度), 'alt': 海拔(米)}
        """
        # 使用配置的 GPS 原点
        origin_lat = self.origin_lat
        origin_lon = self.origin_lon
        origin_alt = self.origin_alt
        
        # 如果已获取 EKF 参考点，可以使用它
        # 但通常我们希望所有 USV 使用统一的原点
        
        # 北向距离 → 纬度差
        dlat = y / 111320.0
        lat = origin_lat + dlat
        
        # 东向距离 → 经度差
        dlon = x / (111320.0 * math.cos(math.radians(origin_lat)))
        lon = origin_lon + dlon
        
        # 天向距离 → 海拔
        alt = z + origin_alt
        
        return {'lat': lat, 'lon': lon, 'alt': alt}

    def _gps_to_xyz(self, lat: float, lon: float, alt: float) -> dict:
        """
        GPS → 本地 XYZ (ENU)
        
        Args:
            lat: 纬度 (度)
            lon: 经度 (度)
            alt: 海拔 (米)
        
        Returns:
            {'x': 东向(m), 'y': 北向(m), 'z': 天向(m)}
        """
        origin_lat = self.origin_lat
        origin_lon = self.origin_lon
        origin_alt = self.origin_alt
        
        # 纬度差 → 北向距离
        y = (lat - origin_lat) * 111320.0
        
        # 经度差 → 东向距离
        x = (lon - origin_lon) * 111320.0 * math.cos(math.radians(origin_lat))
        
        # 海拔差 → 天向距离
        z = alt - origin_alt
        
        return {'x': x, 'y': y, 'z': z}

    def destroy_node(self):
        """节点销毁"""
        super().destroy_node()


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
