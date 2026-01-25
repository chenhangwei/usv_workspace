"""
GPS 到本地坐标转换节点

功能：
1. 订阅飞控的 GPS 位置 (global_position/global)
2. 基于固定 GPS 原点 (A0基站) 计算本地 XYZ 坐标
3. 发布本地坐标 (local_position/pose_from_gps)
4. 替代 MAVROS 的 local_position/pose，实现多 USV 统一坐标系
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

# 导入common_utils工具
from common_utils import ParamLoader, GeoUtils


class GpsToLocalNode(Node):
    """GPS 到本地坐标转换节点"""

    def __init__(self):
        super().__init__('gps_to_local_node')
        
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
        
        # 发布频率
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # 是否启用此功能
        self.declare_parameter('enable_gps_to_local', True)
        
        # UWB/伪卫星坐标系偏移角（度）
        # 定义：UWB X轴 与 地磁东 的夹角（逆时针为正）
        self.declare_parameter('coordinate_yaw_offset_deg', 0.0)
        
        # 获取其他参数
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.enabled = bool(self.get_parameter('enable_gps_to_local').value)
        self.yaw_offset_rad = math.radians(
            float(self.get_parameter('coordinate_yaw_offset_deg').value)
        )
        
        if not self.enabled:
            self.get_logger().info('❌ GPS→本地坐标转换已禁用（使用飞控的 local_position）')
            return
        
        # 缓存最新的 GPS 位置
        self.latest_gps = None
        self.gps_received = False
        
        # 缓存最新的航向（从 MAVROS local_position/pose 获取）
        self.latest_orientation = None
        self.orientation_received = False
        
        # =============================================================================
        # QoS 配置
        # =============================================================================
        
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # =============================================================================
        # 订阅者和发布者
        # =============================================================================
        
        # 订阅 MAVROS GPS 位置
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'global_position/global',
            self.gps_callback,
            qos_best_effort
        )
        
        # 订阅 MAVROS 本地位置（获取航向）
        self.mavros_pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',
            self.mavros_pose_callback,
            qos_best_effort
        )
        
        # 发布本地坐标（替代 local_position/pose）
        self.local_pose_pub = self.create_publisher(
            PoseStamped,
            'local_position/pose_from_gps',
            qos_best_effort
        )
        
        # 定时发布器（确保稳定输出）
        timer_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(timer_period, self.publish_local_pose)
        
        # 计算并显示该纬度处的转换系数（调试信息）
        meters_per_lat = GeoUtils.meters_per_lat_degree(self.origin_lat)
        meters_per_lon = GeoUtils.meters_per_lon_degree(self.origin_lat)
        
        self.get_logger().info('✅ GPS→本地坐标转换节点已启动 (WGS84 椭球模型)')

        self.get_logger().info(
            f'📍 GPS 原点: ({self.origin_lat:.7f}°, {self.origin_lon:.7f}°, {self.origin_alt:.2f}m)'
        )
        self.get_logger().info(
            f'📏 转换系数 @ {self.origin_lat:.2f}°: '
            f'纬度 {meters_per_lat:.2f} m/°, 经度 {meters_per_lon:.2f} m/°'
        )
        self.get_logger().info(f'⏱️  发布频率: {self.publish_rate} Hz')
        yaw_offset_deg = math.degrees(self.yaw_offset_rad)
        if abs(yaw_offset_deg) > 0.1:
            self.get_logger().info(f'🧭 UWB坐标系偏移角: {yaw_offset_deg:.1f}° (已启用航向补偿)')
        self.get_logger().info('📥 订阅: global_position/global (GPS), local_position/pose (航向)')
        self.get_logger().info('📤 发布: local_position/pose_from_gps (XYZ + 航向)')
    
    def gps_callback(self, msg: NavSatFix):
        """GPS 位置回调函数"""
        # 检查 GPS 定位状态
        if msg.status.status < 0:
            # GPS 未定位
            if self.gps_received:
                self.get_logger().warn(
                    '⚠️  GPS 失去定位！',
                    throttle_duration_sec=5.0
                )
            return
        
        # 更新缓存
        self.latest_gps = msg
        
        if not self.gps_received:
            self.gps_received = True
            self.get_logger().info(
                f'✅ GPS 定位成功: '
                f'({msg.latitude:.7f}°, {msg.longitude:.7f}°, {msg.altitude:.2f}m)'
            )
    
    def mavros_pose_callback(self, msg: PoseStamped):
        """MAVROS 本地位置回调（获取航向）"""
        # 从四元数提取 yaw
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 应用 UWB 坐标系偏移角
        # 飞控 yaw 是相对地磁东，需要转换到 UWB 坐标系
        # 修正后的 yaw = 原始 yaw - 偏移角
        corrected_yaw = raw_yaw - self.yaw_offset_rad
        
        # 归一化到 [-π, π]
        while corrected_yaw > math.pi:
            corrected_yaw -= 2 * math.pi
        while corrected_yaw < -math.pi:
            corrected_yaw += 2 * math.pi
        
        # 将修正后的 yaw 转换回四元数（仅保留 yaw，roll/pitch 保持原样）
        # 简化处理：只修改 yaw 分量
        half_yaw = corrected_yaw / 2.0
        self.latest_orientation = type(q)()
        self.latest_orientation.w = math.cos(half_yaw)
        self.latest_orientation.x = 0.0
        self.latest_orientation.y = 0.0
        self.latest_orientation.z = math.sin(half_yaw)
        
        if not self.orientation_received:
            self.orientation_received = True
            raw_yaw_deg = math.degrees(raw_yaw)
            corrected_yaw_deg = math.degrees(corrected_yaw)
            offset_deg = math.degrees(self.yaw_offset_rad)
            if abs(offset_deg) < 0.1:
                self.get_logger().info(
                    f'✅ 航向数据接收成功 (from MAVROS local_position/pose), '
                    f'当前 yaw={corrected_yaw_deg:.1f}°'
                )
            else:
                self.get_logger().info(
                    f'✅ 航向数据接收成功 (from MAVROS local_position/pose), '
                    f'原始 yaw={raw_yaw_deg:.1f}°, 偏移={offset_deg:.1f}°, '
                    f'修正后 yaw={corrected_yaw_deg:.1f}°'
                )
    
    def publish_local_pose(self):
        """定时发布本地坐标"""
        if not self.enabled or not self.gps_received or self.latest_gps is None:
            return
        
        try:
            # 转换 GPS → 本地 XYZ
            xyz = self._gps_to_xyz(
                self.latest_gps.latitude,
                self.latest_gps.longitude,
                self.latest_gps.altitude
            )
            
            # 构造 PoseStamped 消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            pose_msg.pose.position.x = xyz['x']
            pose_msg.pose.position.y = xyz['y']
            pose_msg.pose.position.z = xyz['z']
            
            # 使用来自 MAVROS local_position/pose 的航向
            if self.latest_orientation is not None:
                pose_msg.pose.orientation = self.latest_orientation
            else:
                # 航向数据尚未收到，使用单位四元数并发出警告
                self.get_logger().warn(
                    '⚠️ 航向数据未收到 (local_position/pose)，使用默认 yaw=0',
                    throttle_duration_sec=5.0
                )
                pose_msg.pose.orientation.w = 1.0
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
            
            # 发布
            self.local_pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'GPS→本地坐标转换失败: {e}')
    

    def _gps_to_xyz(self, lat: float, lon: float, alt: float) -> dict:
        """
        GPS 坐标 → 本地 XYZ (ENU坐标系)
        使用 common_utils.GeoUtils 进行转换
        """
        return GeoUtils.gps_to_xyz(lat, lon, alt, self.origin_lat, self.origin_lon, self.origin_alt)


    def destroy_node(self):
        """节点销毁时的资源清理"""
        if hasattr(self, 'publish_timer'):
            self.publish_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsToLocalNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
