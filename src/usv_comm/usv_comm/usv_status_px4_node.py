"""
无人球状态节点 - PX4 uXRCE-DDS 版本

该节点负责收集无人球的各种状态信息并整合发布。
通过 PX4 uXRCE-DDS 原生话题获取状态，替代 MAVROS。

话题映射：
- MAVROS /mavros/state        -> /fmu/out/vehicle_status
- MAVROS /mavros/battery      -> /fmu/out/battery_status
- MAVROS /mavros/local_position/pose -> /fmu/out/vehicle_local_position
- MAVROS /mavros/local_position/velocity_local -> /fmu/out/vehicle_local_position
"""

from math import sqrt, degrees, atan2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from common_interfaces.msg import UsvStatus
import psutil
import time
from collections import deque

# PX4 消息类型
from px4_msgs.msg import (
    VehicleStatus,
    VehicleLocalPosition,
    BatteryStatus,
    VehicleAttitude,
    SensorCombined,
    VehicleGpsPosition
)


class UsvStatusPx4Node(Node):
    """
    无人球状态节点类 - PX4 uXRCE-DDS 版本
    
    该节点负责收集无人球的各种状态信息并整合发布。
    主要功能包括：
    1. 订阅 PX4 飞控状态、电池状态、位置和姿态信息
    2. 整合所有状态信息并发布到统一的状态主题
    3. 获取并发布系统温度
    """
    
    # =========================================================================
    # PX4 导航状态枚举（对应 VehicleStatus.nav_state）
    # =========================================================================
    NAV_STATE_MANUAL = 0
    NAV_STATE_ALTCTL = 1
    NAV_STATE_POSCTL = 2
    NAV_STATE_AUTO_MISSION = 3
    NAV_STATE_AUTO_LOITER = 4
    NAV_STATE_AUTO_RTL = 5
    NAV_STATE_ACRO = 6
    NAV_STATE_OFFBOARD = 14
    NAV_STATE_STAB = 15
    NAV_STATE_AUTO_TAKEOFF = 17
    NAV_STATE_AUTO_LAND = 18
    
    # 导航状态到模式名称的映射
    NAV_STATE_NAMES = {
        0: 'MANUAL',
        1: 'ALTCTL',
        2: 'POSCTL',
        3: 'AUTO.MISSION',
        4: 'AUTO.LOITER',
        5: 'AUTO.RTL',
        6: 'ACRO',
        14: 'OFFBOARD',
        15: 'STABILIZED',
        17: 'AUTO.TAKEOFF',
        18: 'AUTO.LAND',
    }

    def __init__(self):
        """初始化无人球状态节点"""
        super().__init__('usv_status_px4_node')

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
        self.declare_parameter('target_reach_threshold', 1.0)
        self.declare_parameter('distance_mode', '2d')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('data_timeout', 5.0)
        self.declare_parameter('enable_system_monitor', True)
        
        # 电池电压范围参数
        self.declare_parameter('battery_voltage_full', 25.2)  # 6S 满电
        self.declare_parameter('battery_voltage_empty', 21.0) # 6S 空电
        self.declare_parameter('battery_avg_window', 10.0)
        
        # 根据节点命名空间推断 usv_id
        ns_guess = self.get_namespace().lstrip('/') if self.get_namespace() else 'usv_01'
        self.declare_parameter('usv_id', ns_guess)
        
        # 读取参数
        self.target_reach_threshold = self.get_parameter('target_reach_threshold').value
        self.distance_mode = self.get_parameter('distance_mode').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.data_timeout = self.get_parameter('data_timeout').value
        self.enable_system_monitor = self.get_parameter('enable_system_monitor').value
        self.battery_voltage_full = self.get_parameter('battery_voltage_full').value
        self.battery_voltage_empty = self.get_parameter('battery_voltage_empty').value
        self.battery_avg_window = self.get_parameter('battery_avg_window').value
        self.usv_id = self.get_parameter('usv_id').value

        # =====================================================================
        # 创建发布者
        # =====================================================================
        self.state_publisher = self.create_publisher(UsvStatus, 'usv_state', 10)
        self.temperature_publisher = self.create_publisher(Float32, 'usv_temperature', 10)
        self.low_voltage_mode_publisher = self.create_publisher(Bool, 'low_voltage_mode', qos_reliable)

        # =====================================================================
        # 创建订阅者 - PX4 话题
        # =====================================================================
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_px4
        )
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            'fmu/out/battery_status',
            self.battery_callback,
            qos_px4
        )
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_px4
        )
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            'fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_px4
        )
        
        # 可选：GPS 状态
        self.gps_sub = self.create_subscription(
            VehicleGpsPosition,
            'fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_px4
        )

        # =====================================================================
        # 订阅目标点（用于计算距离）
        # =====================================================================
        from geometry_msgs.msg import PoseStamped
        self.target_sub = self.create_subscription(
            PoseStamped,
            'set_usv_target_position',
            self.target_callback,
            qos_reliable
        )

        # =====================================================================
        # 初始化状态变量
        # =====================================================================
        self.target_point = Point()
        self.vehicle_status = None
        self.battery_status = None
        self.local_position = None
        self.attitude = None
        self.gps_position = None
        
        # 电压历史缓存
        self.voltage_history = deque()
        self.low_voltage_mode = False
        self.low_voltage_triggered = False
        
        # 数据时效性跟踪
        self.last_status_time = 0.0
        self.last_battery_time = 0.0
        self.last_position_time = 0.0
        self.last_attitude_time = 0.0
        
        # 系统启动时间
        self.start_time = time.time()
        
        # 消息计数器
        self.message_count = 0

        # =====================================================================
        # 定时器
        # =====================================================================
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_status)
        
        if self.enable_system_monitor:
            self.temperature_timer = self.create_timer(5.0, self.publish_temperature)

        # =====================================================================
        # 日志
        # =====================================================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('PX4 uXRCE-DDS 状态节点已启动')
        self.get_logger().info(f'USV ID: {self.usv_id}')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'电池电压范围: {self.battery_voltage_empty}V ~ {self.battery_voltage_full}V')
        self.get_logger().info('📥 订阅: fmu/out/vehicle_status, battery_status, vehicle_local_position')
        self.get_logger().info('📤 发布: usv_state (UsvStatus)')
        self.get_logger().info('=' * 60)

    # =========================================================================
    # PX4 话题回调
    # =========================================================================
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        """飞控状态回调"""
        self.vehicle_status = msg
        self.last_status_time = time.time()

    def battery_callback(self, msg: BatteryStatus):
        """电池状态回调"""
        self.battery_status = msg
        self.last_battery_time = time.time()
        
        # 更新电压历史
        current_time = time.time()
        self.voltage_history.append((current_time, msg.voltage_v))
        
        # 移除过期数据
        while self.voltage_history and (current_time - self.voltage_history[0][0]) > self.battery_avg_window:
            self.voltage_history.popleft()
        
        # 检查低电量
        self._check_low_voltage()

    def local_position_callback(self, msg: VehicleLocalPosition):
        """本地位置回调"""
        self.local_position = msg
        self.last_position_time = time.time()

    def attitude_callback(self, msg: VehicleAttitude):
        """姿态回调"""
        self.attitude = msg
        self.last_attitude_time = time.time()

    def gps_callback(self, msg: VehicleGpsPosition):
        """GPS 回调"""
        self.gps_position = msg

    def target_callback(self, msg):
        """目标点回调"""
        self.target_point = msg.pose.position

    # =========================================================================
    # 状态发布
    # =========================================================================
    
    def publish_status(self):
        """发布整合的状态信息"""
        msg = UsvStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 设置 USV ID
        msg.usv_id = self.usv_id
        
        # 飞控状态
        if self.vehicle_status is not None:
            msg.connected = True
            msg.armed = (self.vehicle_status.arming_state == 2)  # ARMING_STATE_ARMED
            msg.mode = self._get_mode_name(self.vehicle_status.nav_state)
        else:
            msg.connected = False
            msg.armed = False
            msg.mode = 'UNKNOWN'
        
        # 电池状态
        if self.battery_status is not None:
            msg.battery_voltage = self.battery_status.voltage_v
            msg.battery_current = self.battery_status.current_a
            msg.battery_percentage = self._calculate_battery_percentage(self.battery_status.voltage_v)
        else:
            msg.battery_voltage = 0.0
            msg.battery_current = 0.0
            msg.battery_percentage = 0.0
        
        # 位置信息（NED 转 ENU）
        if self.local_position is not None:
            # PX4 NED -> ROS ENU
            msg.position_x = self.local_position.y   # East = Y_ned
            msg.position_y = self.local_position.x   # North = X_ned
            msg.position_z = -self.local_position.z  # Up = -Down
            
            # 速度
            msg.velocity_x = self.local_position.vy
            msg.velocity_y = self.local_position.vx
            msg.velocity_z = -self.local_position.vz
            
            # 计算水平速度
            msg.velocity_horizontal = sqrt(
                self.local_position.vx ** 2 + self.local_position.vy ** 2
            )
            msg.velocity_vertical = -self.local_position.vz
            
            # 航向
            msg.heading = degrees(self.local_position.heading)
        
        # 姿态信息
        if self.attitude is not None:
            # 从四元数计算欧拉角
            roll, pitch, yaw = self._quaternion_to_euler(self.attitude.q)
            msg.roll = degrees(roll)
            msg.pitch = degrees(pitch)
            msg.yaw = degrees(yaw)
        
        # 计算到目标点的距离
        msg.distance_to_target = self._calculate_distance_to_target()
        msg.target_reached = msg.distance_to_target < self.target_reach_threshold
        
        # 数据时效性检查
        current_time = time.time()
        msg.data_valid = (
            (current_time - self.last_status_time) < self.data_timeout and
            (current_time - self.last_position_time) < self.data_timeout
        )
        
        # 低电量模式
        msg.low_voltage_mode = self.low_voltage_mode
        
        # 发布状态
        self.state_publisher.publish(msg)
        self.message_count += 1

    def publish_temperature(self):
        """发布系统温度"""
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                # 获取第一个可用的温度传感器
                for name, entries in temps.items():
                    if entries:
                        temp_msg = Float32()
                        temp_msg.data = float(entries[0].current)
                        self.temperature_publisher.publish(temp_msg)
                        break
        except Exception as e:
            pass  # 静默处理，避免刷屏

    # =========================================================================
    # 辅助函数
    # =========================================================================
    
    def _get_mode_name(self, nav_state: int) -> str:
        """获取导航状态对应的模式名称"""
        return self.NAV_STATE_NAMES.get(nav_state, f'UNKNOWN({nav_state})')

    def _calculate_battery_percentage(self, voltage: float) -> float:
        """计算电池百分比"""
        if self.voltage_history:
            # 使用平均电压
            avg_voltage = sum(v for _, v in self.voltage_history) / len(self.voltage_history)
        else:
            avg_voltage = voltage
        
        # 线性映射
        voltage_range = self.battery_voltage_full - self.battery_voltage_empty
        if voltage_range <= 0:
            return 0.0
        
        percentage = (avg_voltage - self.battery_voltage_empty) / voltage_range * 100.0
        return max(0.0, min(100.0, percentage))

    def _check_low_voltage(self):
        """检查是否进入低电量模式"""
        if self.battery_status is None:
            return
            
        percentage = self._calculate_battery_percentage(self.battery_status.voltage_v)
        
        # 低于 5% 触发低电量模式
        if percentage < 5.0 and not self.low_voltage_mode:
            self.low_voltage_mode = True
            self.low_voltage_triggered = True
            
            msg = Bool()
            msg.data = True
            self.low_voltage_mode_publisher.publish(msg)
            
            self.get_logger().warn(f'⚠️ 低电量警告！电量: {percentage:.1f}%')
        
        # 高于 10% 解除低电量模式
        elif percentage > 10.0 and self.low_voltage_mode:
            self.low_voltage_mode = False
            
            msg = Bool()
            msg.data = False
            self.low_voltage_mode_publisher.publish(msg)

    def _calculate_distance_to_target(self) -> float:
        """计算到目标点的距离"""
        if self.local_position is None:
            return -1.0
        
        # PX4 NED -> ROS ENU
        current_x = self.local_position.y
        current_y = self.local_position.x
        current_z = -self.local_position.z
        
        dx = self.target_point.x - current_x
        dy = self.target_point.y - current_y
        dz = self.target_point.z - current_z
        
        if self.distance_mode == '2d':
            return sqrt(dx * dx + dy * dy)
        else:
            return sqrt(dx * dx + dy * dy + dz * dz)

    def _quaternion_to_euler(self, q):
        """四元数转欧拉角"""
        # q = [w, x, y, z] in PX4
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = 1.5707963 if sinp > 0 else -1.5707963  # ±90°
        else:
            pitch = atan2(sinp, 1)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


def main(args=None):
    """节点主函数"""
    rclpy.init(args=args)
    node = UsvStatusPx4Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
