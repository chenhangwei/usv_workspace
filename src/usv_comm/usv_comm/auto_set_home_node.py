"""
自动设置 EKF 原点节点 - 室内 UWB 版本

该节点用于室内 UWB 定位场景，等待本地位置有效后设置 Home 点。

PX4 设置 EKF 原点的方式（室内 UWB）：
1. 通过 VehicleCommand (MAV_CMD_DO_SET_HOME) 设置 Home 位置
2. 使用固定坐标作为虚拟原点（室内场景）
3. 不依赖 GPS，等待 UWB 定位系统就绪

话题映射：
- /fmu/in/vehicle_command - 发送设置 Home 命令
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# PX4 消息类型
from px4_msgs.msg import (
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)


class AutoSetHomePx4Node(Node):
    """
    自动设置 Home/EKF 原点节点类 - 室内 UWB 版本
    
    该节点监控 PX4 状态，在本地位置有效时自动设置 EKF 原点。
    """
    
    # VehicleCommand 命令 ID
    VEHICLE_CMD_DO_SET_HOME = 179

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
        # 室内场景使用固定坐标作为虚拟原点（不影响本地坐标系）
        self.declare_parameter('fixed_lat', 0.0)   # 虚拟纬度
        self.declare_parameter('fixed_lon', 0.0)   # 虚拟经度
        self.declare_parameter('fixed_alt', 0.0)   # 虚拟高度
        
        self.set_delay_sec = self.get_parameter('set_delay_sec').value
        self.fixed_lat = self.get_parameter('fixed_lat').value
        self.fixed_lon = self.get_parameter('fixed_lon').value
        self.fixed_alt = self.get_parameter('fixed_alt').value

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

        # =====================================================================
        # 状态变量
        # =====================================================================
        self.home_set_sent = False
        self.vehicle_connected = False
        self.local_position_valid = False
        
        self.delay_timer = None
        self.check_timer = None

        # 日志输出
        self.get_logger().info(
            f'AutoSetHomePx4Node 已初始化（室内 UWB 模式）\n'
            f'  ├─ 延迟: {self.set_delay_sec:.1f}s\n'
            f'  └─ 虚拟原点: ({self.fixed_lat:.7f}, {self.fixed_lon:.7f}, {self.fixed_alt:.2f})'
        )
        
        # 定期检查状态
        self.check_timer = self.create_timer(1.0, self._check_status)

    def vehicle_status_callback(self, msg: VehicleStatus):
        """飞控状态回调"""
        # PX4 总是"连接"的（如果能收到消息）
        self.vehicle_connected = True

    def local_position_callback(self, msg: VehicleLocalPosition):
        """本地位置回调 - 用于检测 UWB 定位是否就绪"""
        if msg.xy_valid and msg.z_valid:
            if not self.local_position_valid:
                self.local_position_valid = True
                self.get_logger().info('✅ UWB 本地位置有效')

    def _check_status(self):
        """定期检查状态，决定是否设置 Home 点"""
        if self.home_set_sent:
            return
        
        # 室内 UWB 模式：只要飞控连接且本地位置有效就可以
        ready = self.vehicle_connected and self.local_position_valid
        
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
        """设置 Home 位置（室内 UWB 使用虚拟坐标）"""
        try:
            msg = VehicleCommand()
            msg.command = self.VEHICLE_CMD_DO_SET_HOME
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            # 使用固定虚拟坐标（室内场景）
            # param1 = 0 表示使用指定位置
            msg.param1 = 0.0
            msg.param5 = float(self.fixed_lat)
            msg.param6 = float(self.fixed_lon)
            msg.param7 = float(self.fixed_alt)
            
            self.command_pub.publish(msg)
            self.get_logger().info(
                f'📍 使用虚拟坐标设置 Home 点: '
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
