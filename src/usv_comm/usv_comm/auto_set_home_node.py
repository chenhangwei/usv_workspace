"""
自动设置 EKF 原点节点 - PX4 uXRCE-DDS 版本

该节点用于自动设置 PX4 的 EKF 全局原点，使无人球可以使用本地坐标系统。

PX4 设置 EKF 原点的方式：
1. 通过 VehicleCommand (MAV_CMD_DO_SET_HOME) 设置 Home 位置
2. 等待 GPS 锁定后自动设置（如果有 GPS）
3. 对于无 GPS 系统，需要手动设置原点

话题映射：
- MAVROS global_position/set_gp_origin -> /fmu/in/vehicle_command
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# PX4 消息类型
from px4_msgs.msg import (
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleGlobalPosition,
    SensorGps,
)


class AutoSetHomePx4Node(Node):
    """
    自动设置 Home/EKF 原点节点类 - PX4 uXRCE-DDS 版本
    
    该节点监控 PX4 状态，在满足条件时自动设置 EKF 原点。
    """
    
    # VehicleCommand 命令 ID
    VEHICLE_CMD_DO_SET_HOME = 179
    VEHICLE_CMD_DO_SET_MODE = 176

    def __init__(self):
        """初始化节点"""
        super().__init__('auto_set_home_node')

        # =====================================================================
        # QoS 配置
        # =====================================================================
        qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =====================================================================
        # 参数配置
        # =====================================================================
        self.declare_parameter('set_delay_sec', 3.0)
        self.declare_parameter('use_current_gps', True)
        self.declare_parameter('fixed_lat', 22.5180977)
        self.declare_parameter('fixed_lon', 113.9007239)
        self.declare_parameter('fixed_alt', -5.17)
        self.declare_parameter('wait_for_gps', True)
        self.declare_parameter('min_gps_satellites', 6)
        
        self.set_delay_sec = self.get_parameter('set_delay_sec').value
        self.use_current_gps = self.get_parameter('use_current_gps').value
        self.fixed_lat = self.get_parameter('fixed_lat').value
        self.fixed_lon = self.get_parameter('fixed_lon').value
        self.fixed_alt = self.get_parameter('fixed_alt').value
        self.wait_for_gps = self.get_parameter('wait_for_gps').value
        self.min_gps_satellites = self.get_parameter('min_gps_satellites').value

        # =====================================================================
        # 发布器
        # =====================================================================
        self.command_pub = self.create_publisher(
            VehicleCommand,
            'fmu/in/vehicle_command',
            qos_px4
        )

        # =====================================================================
        # 订阅器
        # =====================================================================
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_px4
        )
        
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_px4
        )
        
        self.global_pos_sub = self.create_subscription(
            VehicleGlobalPosition,
            'fmu/out/vehicle_global_position',
            self.global_position_callback,
            qos_px4
        )
        
        # 可选：订阅 GPS 状态
        self.gps_sub = self.create_subscription(
            SensorGps,
            'fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_px4
        )

        # =====================================================================
        # 状态变量
        # =====================================================================
        self.home_set_sent = False
        self.vehicle_connected = False
        self.local_position_valid = False
        self.global_position_valid = False
        self.gps_satellites = 0
        self.gps_fix_type = 0
        
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        self.delay_timer = None
        self.check_timer = None

        # 日志输出
        if self.use_current_gps:
            self.get_logger().info(
                f'AutoSetHomePx4Node 已初始化 - 将使用当前 GPS 位置作为 Home 点，'
                f'延迟 {self.set_delay_sec:.1f}s'
            )
        else:
            self.get_logger().info(
                f'AutoSetHomePx4Node 已初始化 - 将使用固定坐标 '
                f'({self.fixed_lat:.7f}, {self.fixed_lon:.7f}, {self.fixed_alt:.2f}) 作为 Home 点'
            )
        
        # 定期检查状态
        self.check_timer = self.create_timer(1.0, self._check_status)

    def vehicle_status_callback(self, msg: VehicleStatus):
        """飞控状态回调"""
        # PX4 总是"连接"的（如果能收到消息）
        self.vehicle_connected = True

    def local_position_callback(self, msg: VehicleLocalPosition):
        """本地位置回调"""
        if msg.xy_valid and msg.z_valid:
            if not self.local_position_valid:
                self.local_position_valid = True
                self.get_logger().info('✅ 本地位置有效')

    def global_position_callback(self, msg: VehicleGlobalPosition):
        """全局位置回调"""
        if msg.lat != 0.0 and msg.lon != 0.0:
            self.global_position_valid = True
            self.current_lat = msg.lat
            self.current_lon = msg.lon
            self.current_alt = msg.alt

    def gps_callback(self, msg: SensorGps):
        """GPS 状态回调"""
        self.gps_satellites = msg.satellites_used
        self.gps_fix_type = msg.fix_type

    def _check_status(self):
        """定期检查状态，决定是否设置 Home 点"""
        if self.home_set_sent:
            return
        
        # 检查是否满足条件
        ready = False
        
        if self.wait_for_gps:
            # 等待 GPS 锁定
            if self.gps_fix_type >= 3 and self.gps_satellites >= self.min_gps_satellites:
                ready = True
                self.get_logger().info(
                    f'📡 GPS 就绪: {self.gps_satellites} 颗卫星, fix_type={self.gps_fix_type}'
                )
        else:
            # 不等待 GPS，只要飞控连接就可以
            if self.vehicle_connected:
                ready = True
        
        if ready and self.delay_timer is None:
            self.get_logger().info(f'⏳ {self.set_delay_sec:.1f} 秒后设置 Home 点...')
            self.delay_timer = self.create_timer(self.set_delay_sec, self._set_home_delayed)

    def _set_home_delayed(self):
        """延迟设置 Home 点"""
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None
        
        if self.home_set_sent:
            return
        
        self._set_home_position()
        self.home_set_sent = True
        
        # 停止定期检查
        if self.check_timer is not None:
            self.check_timer.cancel()
            self.check_timer = None

    def _set_home_position(self):
        """设置 Home 位置"""
        try:
            msg = VehicleCommand()
            msg.command = self.VEHICLE_CMD_DO_SET_HOME
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            if self.use_current_gps:
                # 使用当前 GPS 位置
                # param1 = 1 表示使用当前位置
                msg.param1 = 1.0
                msg.param5 = 0.0  # 不使用，因为 param1=1
                msg.param6 = 0.0
                msg.param7 = 0.0
                
                self.command_pub.publish(msg)
                self.get_logger().info('📍 使用当前 GPS 位置设置 Home 点')
            else:
                # 使用固定坐标
                # param1 = 0 表示使用指定位置
                msg.param1 = 0.0
                msg.param5 = float(self.fixed_lat)
                msg.param6 = float(self.fixed_lon)
                msg.param7 = float(self.fixed_alt)
                
                self.command_pub.publish(msg)
                self.get_logger().info(
                    f'📍 使用固定坐标设置 Home 点: '
                    f'({self.fixed_lat:.7f}, {self.fixed_lon:.7f}, {self.fixed_alt:.2f})'
                )
            
            self.get_logger().info('✅ Home 点设置命令已发送')
            
        except Exception as e:
            self.get_logger().error(f'❌ 设置 Home 点失败: {e}')

    def destroy_node(self):
        """节点销毁"""
        if self.delay_timer:
            self.delay_timer.cancel()
        if self.check_timer:
            self.check_timer.cancel()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = AutoSetHomePx4Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
