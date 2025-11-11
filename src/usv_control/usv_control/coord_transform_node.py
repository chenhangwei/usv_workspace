"""
坐标转换节点 - XYZ → GPS

功能：
1. 订阅地面站发送的 XYZ 目标点 (set_usv_target_position)
2. 将 XYZ 转换为 GPS 坐标（lat/lon/alt）
3. 发布 GPS 目标点给 MAVROS (setpoint_position/global)

注意：所有 USV 使用统一的 GPS 原点（A0基站）进行坐标转换
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math


class CoordTransformNode(Node):
    """
    坐标转换节点 - XYZ → GPS
    
    确保所有 USV 使用统一的 GPS 原点进行坐标转换
    """

    def __init__(self):
        super().__init__('coord_transform_node')
        
        # =============================================================================
        # 参数声明
        # =============================================================================
        
        # GPS 原点配置（A0基站坐标）
        self.declare_parameter('gps_origin_lat', 22.5180977)  # 北纬（度）
        self.declare_parameter('gps_origin_lon', 113.9007239) # 东经（度）
        self.declare_parameter('gps_origin_alt', -5.17)       # 海拔（米，实测值）
        
        # 是否启用坐标转换
        self.declare_parameter('enable_coord_transform', True)
        
        # 获取参数
        self.origin_lat = float(self.get_parameter('gps_origin_lat').value)
        self.origin_lon = float(self.get_parameter('gps_origin_lon').value)
        self.origin_alt = float(self.get_parameter('gps_origin_alt').value)
        self.enabled = bool(self.get_parameter('enable_coord_transform').value)
        
        # =============================================================================
        # QoS 配置
        # =============================================================================
        
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # =============================================================================
        # 订阅者和发布者
        # =============================================================================
        
        if self.enabled:
            # 订阅地面站的 XYZ 目标点
            self.xyz_target_sub = self.create_subscription(
                PoseStamped,
                'set_usv_target_position',
                self.xyz_target_callback,
                qos_reliable
            )
            
            # 发布 GPS 目标点给 MAVROS
            self.gps_target_pub = self.create_publisher(
                GeoPoseStamped,
                'setpoint_position/global',
                qos_best_effort
            )
            
            self.get_logger().info('✅ XYZ→GPS 坐标转换节点已启动')
            self.get_logger().info(
                f'📍 GPS 原点: ({self.origin_lat:.7f}°, {self.origin_lon:.7f}°, {self.origin_alt:.2f}m)'
            )
            self.get_logger().info('📥 订阅: set_usv_target_position (地面站 XYZ)')
            self.get_logger().info('📤 发布: setpoint_position/global (GPS 目标点)')
            self.get_logger().info('ℹ️  注意: 所有 USV 使用统一的 GPS 原点进行坐标转换')
        else:
            self.get_logger().info('⏸️  坐标转换功能已禁用（使用局部坐标系统）')
    
    def xyz_target_callback(self, msg: PoseStamped):
        """
        接收地面站的 XYZ 目标点，转换为 GPS 坐标发送给飞控
        
        Args:
            msg: XYZ 目标点 (PoseStamped)
        """
        try:
            # 提取 XYZ 坐标
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # 转换为 GPS 坐标
            gps_coord = self._xyz_to_gps(x, y, z)
            
            # 构造 GPS 目标点消息
            gps_msg = GeoPoseStamped()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'map'
            
            gps_msg.pose.position.latitude = gps_coord['lat']
            gps_msg.pose.position.longitude = gps_coord['lon']
            gps_msg.pose.position.altitude = gps_coord['alt']
            
            # 复制姿态（yaw）
            gps_msg.pose.orientation = msg.pose.orientation
            
            # 发布给 MAVROS
            self.gps_target_pub.publish(gps_msg)
            
            self.get_logger().info(
                f'🎯 XYZ→GPS: ({x:.2f}, {y:.2f}, {z:.2f})m → '
                f'({gps_coord["lat"]:.7f}°, {gps_coord["lon"]:.7f}°, {gps_coord["alt"]:.2f}m)'
            )
            
        except Exception as e:
            self.get_logger().error(f'XYZ→GPS 转换失败: {e}')
    
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
        # 北向距离 → 纬度差
        dlat = y / 111320.0
        lat = self.origin_lat + dlat
        
        # 东向距离 → 经度差
        dlon = x / (111320.0 * math.cos(math.radians(self.origin_lat)))
        lon = self.origin_lon + dlon
        
        # 天向距离 → 海拔
        alt = z + self.origin_alt
        
        return {'lat': lat, 'lon': lon, 'alt': alt}



def main(args=None):
    rclpy.init(args=args)
    node = CoordTransformNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
