"""
坐标转换节点 - XYZ → GPS (全局坐标)

功能：
1. 订阅地面站发送的 XYZ 目标点 (set_usv_target_position)
2. 订阅避障节点发送的 XYZ 目标点 (avoidance_position)
3. 将 XYZ 转换为 GPS 坐标（lat/lon/alt）
4. 发布全局GPS目标点给 MAVROS (setpoint_raw/global)

支持两种输出格式：
- GeoPoseStamped → setpoint_position/global (旧接口)
- GlobalPositionTarget → setpoint_raw/global (推荐，SET_POSITION_TARGET_GLOBAL_INT)

注意：所有 USV 使用统一的 GPS 原点（A0基站）进行坐标转换
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

# 导入common_utils工具
from common_utils import ParamLoader, GeoUtils


class CoordTransformNode(Node):
    """
    坐标转换节点 - XYZ → GPS
    
    确保所有 USV 使用统一的 GPS 原点进行坐标转换
    """

    def __init__(self):
        super().__init__('coord_transform_node')
        
        # 创建参数加载器
        param_loader = ParamLoader(self)
        
        # =============================================================================
        # 参数声明
        # =============================================================================
        
        # GPS 原点配置（A0基站坐标）- 使用统一加载方法
        gps_origin = param_loader.load_gps_origin(
            default_lat=22.5180977,
            default_lon=113.9007239,
            default_alt=-5.17
        )
        self.origin_lat = gps_origin['lat']
        self.origin_lon = gps_origin['lon']
        self.origin_alt = gps_origin['alt']
        
        # 是否启用坐标转换
        self.declare_parameter('enable_coord_transform', True)
        
        # 输出格式选择
        self.declare_parameter('use_global_position_target', True)  # true=GlobalPositionTarget, false=GeoPoseStamped
        
        self.enabled = bool(self.get_parameter('enable_coord_transform').value)
        self.use_global_position_target = bool(self.get_parameter('use_global_position_target').value)
        
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
            
            # 订阅避障 XYZ 目标点（从 usv_avoidance_node）
            self.avoidance_target_sub = self.create_subscription(
                PositionTarget,  # 接收 PositionTarget 类型
                'avoidance_position',
                self.avoidance_target_callback,
                qos_reliable
            )
            
            # 根据配置选择输出格式
            if self.use_global_position_target:
                # 发布 GlobalPositionTarget 到 setpoint_raw/global
                self.global_target_pub = self.create_publisher(
                    GlobalPositionTarget,
                    'setpoint_raw/global',
                    qos_best_effort
                )
                output_topic = 'setpoint_raw/global (GlobalPositionTarget)'
                output_mavlink = 'SET_POSITION_TARGET_GLOBAL_INT'
            else:
                # 发布 GeoPoseStamped 到 setpoint_position/global
                self.gps_target_pub = self.create_publisher(
                    GeoPoseStamped,
                    'setpoint_position/global',
                    qos_best_effort
                )
                output_topic = 'setpoint_position/global (GeoPoseStamped)'
                output_mavlink = 'SET_POSITION_TARGET_GLOBAL'
            
            self.get_logger().info('✅ XYZ→GPS 坐标转换节点已启动')
            self.get_logger().info(
                f'📍 GPS 原点: ({self.origin_lat:.7f}°, {self.origin_lon:.7f}°, {self.origin_alt:.2f}m)'
            )
            self.get_logger().info('📥 订阅: set_usv_target_position (地面站 XYZ)')
            self.get_logger().info('📥 订阅: avoidance_position (避障 XYZ)')
            self.get_logger().info(f'📤 发布: {output_topic}')
            self.get_logger().info(f'🌍 MAVLink: {output_mavlink}')
            self.get_logger().info('ℹ️  注意: 所有 USV 使用统一的 GPS 原点进行坐标转换')
        else:
            self.get_logger().info('⏸️  坐标转换功能已禁用（使用局部坐标系统）')
    
    def avoidance_target_callback(self, msg: PositionTarget):
        """
        接收避障节点的 XYZ 目标点，转换为 GPS 坐标发送给飞控
        
        Args:
            msg: 避障 XYZ 目标点 (PositionTarget)
        """
        try:
            # 提取 XYZ 坐标（注意：PositionTarget 使用 position，不是 pose.position）
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            
            # 🔍 调试日志：接收避障 XYZ 坐标
            self.get_logger().info(
                f"🚨 [坐标转换节点] 接收避障 XYZ 目标点\n"
                f"  ├─ X(东向): {x:.3f} m\n"
                f"  ├─ Y(北向): {y:.3f} m\n"
                f"  └─ Z(高度): {z:.3f} m"
            )
            
            # 转换为 GPS 坐标
            gps_coord = self._xyz_to_gps(x, y, z)
            
            if self.use_global_position_target:
                # ============ 发布 GlobalPositionTarget ============
                global_msg = GlobalPositionTarget()
                global_msg.header.stamp = self.get_clock().now().to_msg()
                global_msg.header.frame_id = 'map'
                global_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
                global_msg.type_mask = (
                    GlobalPositionTarget.IGNORE_VX |
                    GlobalPositionTarget.IGNORE_VY |
                    GlobalPositionTarget.IGNORE_VZ |
                    GlobalPositionTarget.IGNORE_AFX |
                    GlobalPositionTarget.IGNORE_AFY |
                    GlobalPositionTarget.IGNORE_AFZ |
                    GlobalPositionTarget.FORCE |
                    GlobalPositionTarget.IGNORE_YAW_RATE
                )
                
                # 设置GPS坐标（纬度/经度/海拔）
                global_msg.latitude = gps_coord['lat']
                global_msg.longitude = gps_coord['lon']
                global_msg.altitude = gps_coord['alt']
                
                # 发布
                self.global_target_pub.publish(global_msg)
                
                # 🔍 调试日志：发布避障 GlobalPositionTarget
                self.get_logger().info(
                    f"📤 [坐标转换节点] 发布避障 GlobalPositionTarget\n"
                    f"  ├─ 纬度: {gps_coord['lat']:.7f}°\n"
                    f"  ├─ 经度: {gps_coord['lon']:.7f}°\n"
                    f"  ├─ 海拔: {gps_coord['alt']:.2f} m\n"
                    f"  └─ 话题: setpoint_raw/global"
                )
                
            else:
                # ============ 发布 GeoPoseStamped ============
                gps_msg = GeoPoseStamped()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'map'
                
                gps_msg.pose.position.latitude = gps_coord['lat']
                gps_msg.pose.position.longitude = gps_coord['lon']
                gps_msg.pose.position.altitude = gps_coord['alt']
                
                # PositionTarget 没有 orientation，设置默认姿态
                gps_msg.pose.orientation.w = 1.0
                
                # 发布
                self.gps_target_pub.publish(gps_msg)
                
                # 🔍 调试日志：发布避障 GeoPoseStamped
                self.get_logger().info(
                    f"� [坐标转换节点] 发布避障 GeoPoseStamped\n"
                    f"  ├─ 纬度: {gps_coord['lat']:.7f}°\n"
                    f"  ├─ 经度: {gps_coord['lon']:.7f}°\n"
                    f"  └─ 海拔: {gps_coord['alt']:.2f} m"
                )
            
        except Exception as e:
            self.get_logger().error(f'避障XYZ→GPS 转换失败: {e}')
    
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
            
            # 🔍 调试日志：接收 XYZ 坐标
            self.get_logger().info(
                f"📥 [坐标转换节点] 接收 XYZ 目标点\n"
                f"  ├─ X(东向): {x:.3f} m\n"
                f"  ├─ Y(北向): {y:.3f} m\n"
                f"  └─ Z(高度): {z:.3f} m"
            )
            
            # 转换为 GPS 坐标
            gps_coord = self._xyz_to_gps(x, y, z)
            
            if self.use_global_position_target:
                # ============ 发布 GlobalPositionTarget ============
                global_msg = GlobalPositionTarget()
                global_msg.header.stamp = self.get_clock().now().to_msg()
                global_msg.header.frame_id = 'map'
                global_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
                global_msg.type_mask = (
                    GlobalPositionTarget.IGNORE_VX |
                    GlobalPositionTarget.IGNORE_VY |
                    GlobalPositionTarget.IGNORE_VZ |
                    GlobalPositionTarget.IGNORE_AFX |
                    GlobalPositionTarget.IGNORE_AFY |
                    GlobalPositionTarget.IGNORE_AFZ |
                    GlobalPositionTarget.FORCE |
                    GlobalPositionTarget.IGNORE_YAW_RATE
                )
                
                # 设置GPS坐标（纬度/经度/海拔）
                global_msg.latitude = gps_coord['lat']
                global_msg.longitude = gps_coord['lon']
                global_msg.altitude = gps_coord['alt']
                
                # 发布
                self.global_target_pub.publish(global_msg)
                
                # 🔍 调试日志：发布 GlobalPositionTarget
                self.get_logger().info(
                    f"📤 [坐标转换节点] 发布 GlobalPositionTarget\n"
                    f"  ├─ 纬度(Lat): {gps_coord['lat']:.7f}°\n"
                    f"  ├─ 经度(Lon): {gps_coord['lon']:.7f}°\n"
                    f"  ├─ 海拔(Alt): {gps_coord['alt']:.2f} m\n"
                    f"  ├─ 话题: setpoint_raw/global\n"
                    f"  └─ MAVLink: SET_POSITION_TARGET_GLOBAL_INT (ID:86)"
                )
                
            else:
                # ============ 发布 GeoPoseStamped ============
                gps_msg = GeoPoseStamped()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'map'
                
                gps_msg.pose.position.latitude = gps_coord['lat']
                gps_msg.pose.position.longitude = gps_coord['lon']
                gps_msg.pose.position.altitude = gps_coord['alt']
                
                # 复制姿态（yaw）
                gps_msg.pose.orientation = msg.pose.orientation
                
                # 发布
                self.gps_target_pub.publish(gps_msg)
                
                # 🔍 调试日志：发布 GeoPoseStamped
                self.get_logger().info(
                    f"📤 [坐标转换节点] 发布 GeoPoseStamped\n"
                    f"  ├─ 纬度: {gps_coord['lat']:.7f}°\n"
                    f"  ├─ 经度: {gps_coord['lon']:.7f}°\n"
                    f"  ├─ 海拔: {gps_coord['alt']:.2f} m\n"
                    f"  └─ 话题: setpoint_position/global"
                )
            
        except Exception as e:
            self.get_logger().error(f'XYZ→GPS 转换失败: {e}')
    
    def _xyz_to_gps(self, x: float, y: float, z: float) -> dict:
        """
        本地 XYZ (ENU) → GPS (lat/lon/alt)
        使用 common_utils.GeoUtils 进行转换
        """
        return GeoUtils.xyz_to_gps(x, y, z, self.origin_lat, self.origin_lon, self.origin_alt)


    def destroy_node(self):
        """节点销毁时的资源清理"""
        # 该节点没有 timer，只需调用父类方法
        super().destroy_node()


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
