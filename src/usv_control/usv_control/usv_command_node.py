"""
无人球命令控制节点 - PX4 uXRCE-DDS 版本

该节点负责处理来自地面站的命令，包括模式切换和解锁/上锁操作。
通过 px4_msgs/VehicleCommand 直接与 PX4 飞控通信，替代 MAVROS 服务调用。

话题映射：
- MAVROS /mavros/cmd/arming  -> /fmu/in/vehicle_command (VehicleCommand)
- MAVROS /mavros/set_mode    -> /fmu/in/vehicle_command (VehicleCommand)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Bool

# PX4 消息类型
from px4_msgs.msg import (
    VehicleCommand, 
    VehicleStatus, 
    OffboardControlMode, 
    TrajectorySetpoint,
    VehicleLocalPosition
)


class UsvCommandPx4Node(Node):
    """
    无人球命令控制节点类 - PX4 uXRCE-DDS 版本
    
    该节点订阅地面站发送的模式切换和解锁/上锁命令，
    通过 PX4 uXRCE-DDS 接口直接与飞控通信。
    """
    
    # =========================================================================
    # PX4 命令 ID (来自 MAVLink VEHICLE_CMD)
    # =========================================================================
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_NAV_TAKEOFF = 22
    VEHICLE_CMD_NAV_LAND = 21
    VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20
    
    # =========================================================================
    # PX4 自定义模式 ID
    # =========================================================================
    # 主模式
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1
    PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
    PX4_CUSTOM_MAIN_MODE_POSCTL = 3
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_MAIN_MODE_ACRO = 5
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
    
    # 自动模式子模式
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
    PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
    
    # =========================================================================
    # 模式名称到 PX4 模式 ID 的映射
    # =========================================================================
    MODE_MAP = {
        'MANUAL': (PX4_CUSTOM_MAIN_MODE_MANUAL, 0),
        'ALTCTL': (PX4_CUSTOM_MAIN_MODE_ALTCTL, 0),
        'POSCTL': (PX4_CUSTOM_MAIN_MODE_POSCTL, 0),
        'STABILIZED': (PX4_CUSTOM_MAIN_MODE_STABILIZED, 0),
        'ACRO': (PX4_CUSTOM_MAIN_MODE_ACRO, 0),
        'OFFBOARD': (PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0),
        'AUTO.READY': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_READY),
        'AUTO.TAKEOFF': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF),
        'AUTO.LOITER': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
        'AUTO.MISSION': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION),
        'AUTO.RTL': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL),
        'AUTO.LAND': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND),
    }

    def __init__(self):
        """初始化无人球命令控制节点"""
        super().__init__('usv_command_node')

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
        # 参数声明
        # =====================================================================
        self.declare_parameter('supported_modes', [
            'OFFBOARD', 'MANUAL', 'STABILIZED', 'POSCTL', 'ALTCTL',
            'AUTO.LOITER', 'AUTO.MISSION', 'AUTO.RTL', 'AUTO.LAND', 'AUTO.TAKEOFF'
        ])
        self.declare_parameter('command_timeout_sec', 5.0)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)

        # 获取参数值
        self.supported_modes = self.get_parameter('supported_modes').get_parameter_value().string_array_value
        self.command_timeout_sec = self.get_parameter('command_timeout_sec').get_parameter_value().double_value
        self.target_system = self.get_parameter('target_system').get_parameter_value().integer_value
        self.target_component = self.get_parameter('target_component').get_parameter_value().integer_value

        # =====================================================================
        # 状态变量
        # =====================================================================
        self.current_status = None
        self.last_mode_command = None
        self.last_mode_time = 0.0
        self.mode_debounce_sec = 0.5
        self.mode_switching = False

        # =====================================================================
        # 发布器 - 发送命令到 PX4 飞控
        # =====================================================================
        self.command_pub = self.create_publisher(
            VehicleCommand,
            'fmu/in/vehicle_command',
            qos_px4
        )
        
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            qos_px4
        )
        
        # OFFBOARD 模式需要持续发送 TrajectorySetpoint
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos_px4
        )

        # =====================================================================
        # 订阅器 - 飞控状态
        # =====================================================================
        # 注意：PX4 v1.15+ 发布的是 vehicle_status_v1 话题
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_px4
        )
        
        # 订阅本地位置，用于 OFFBOARD 模式保持当前位置
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_px4
        )

        # =====================================================================
        # 订阅器 - 地面站命令（保持原有接口兼容）
        # =====================================================================
        self.sub_mode = self.create_subscription(
            String,
            'set_usv_mode',
            self.set_mode_callback,
            qos_reliable
        )

        self.sub_arming = self.create_subscription(
            String,
            'set_usv_arming',
            self.set_arming_callback,
            qos_reliable
        )
        
        # 新增：Bool 类型的解锁订阅（更简洁的接口）
        self.sub_arm_bool = self.create_subscription(
            Bool,
            'set_arm',
            self.set_arm_bool_callback,
            qos_reliable
        )

        # =====================================================================
        # OFFBOARD 模式心跳定时器
        # =====================================================================
        self.offboard_heartbeat_timer = self.create_timer(0.1, self.publish_offboard_heartbeat)
        self.offboard_mode_active = False
        
        # OFFBOARD 模式预切换状态（在切换前需要先发送心跳）
        self.offboard_pre_switch = False
        self.offboard_pre_switch_count = 0
        self.offboard_pre_switch_target = 15  # 发送约 1.5 秒心跳后再切换
        
        # 当前位置（用于 OFFBOARD 模式保持位置）
        self.current_position = [0.0, 0.0, 0.0]  # NED 坐标

        self.get_logger().info('=' * 60)
        self.get_logger().info('PX4 uXRCE-DDS 命令控制节点已启动')
        self.get_logger().info(f'支持的模式: {", ".join(self.supported_modes)}')
        self.get_logger().info(f'目标系统: {self.target_system}, 目标组件: {self.target_component}')
        self.get_logger().info('=' * 60)

    def vehicle_status_callback(self, msg: VehicleStatus):
        """
        飞控状态回调
        
        Args:
            msg (VehicleStatus): PX4 飞控状态消息
        """
        self.current_status = msg
        
        # 检查是否处于 OFFBOARD 模式
        # nav_state == 14 表示 OFFBOARD
        self.offboard_mode_active = (msg.nav_state == 14)

    def local_position_callback(self, msg: VehicleLocalPosition):
        """
        本地位置回调
        
        用于更新当前位置，在 OFFBOARD 模式下保持当前位置。
        
        Args:
            msg (VehicleLocalPosition): PX4 本地位置消息
        """
        # 只有当位置有效时才更新
        if msg.xy_valid and msg.z_valid:
            self.current_position = [msg.x, msg.y, msg.z]

    def publish_offboard_heartbeat(self):
        """
        发布 OFFBOARD 模式心跳
        
        PX4 要求在 OFFBOARD 模式下持续接收 OffboardControlMode 消息，
        否则会自动切换到 Hold 模式。
        
        同时在切换到 OFFBOARD 模式之前也需要发送心跳（预切换阶段）。
        """
        # 只在 OFFBOARD 激活或预切换阶段发送
        if not self.offboard_mode_active and not self.offboard_pre_switch:
            return
        
        # 发送 OffboardControlMode
        ocm = OffboardControlMode()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(ocm)
        
        # 发送 TrajectorySetpoint（保持当前位置）
        sp = TrajectorySetpoint()
        sp.position = self.current_position  # NED 坐标
        sp.yaw = float('nan')  # 保持当前航向
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(sp)
        
        # 预切换阶段计数
        if self.offboard_pre_switch:
            self.offboard_pre_switch_count += 1
            if self.offboard_pre_switch_count >= self.offboard_pre_switch_target:
                # 发送模式切换命令
                self.get_logger().info('📡 OFFBOARD 预热完成，发送模式切换命令')
                self._send_mode_command('OFFBOARD')
                self.offboard_pre_switch = False
                self.offboard_pre_switch_count = 0

    def set_mode_callback(self, msg: String):
        """
        处理模式切换命令回调函数
        
        Args:
            msg (String): 包含目标模式的字符串消息
        """
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的模式消息类型')
            return

        mode_name = msg.data.upper()

        # 检查模式是否受支持
        if mode_name not in self.MODE_MAP:
            self.get_logger().error(
                f'不支持的模式: {mode_name}，支持的模式: {", ".join(self.MODE_MAP.keys())}'
            )
            return

        # 防抖检查
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (self.last_mode_command == mode_name and 
            current_time - self.last_mode_time < self.mode_debounce_sec):
            return

        # 如果已经在切换中或预切换中，拒绝新请求
        if self.mode_switching or self.offboard_pre_switch:
            return
        
        self.get_logger().info(f'收到模式切换命令: {mode_name}')
        self.last_mode_command = mode_name
        self.last_mode_time = current_time
        
        # OFFBOARD 模式需要特殊处理：先发送心跳，再切换模式
        if mode_name == 'OFFBOARD':
            if self.offboard_mode_active:
                # 已经在 OFFBOARD 模式，不需要再切换
                self.get_logger().info('已处于 OFFBOARD 模式')
                return
            
            self.get_logger().info('🔄 开始 OFFBOARD 模式预热...')
            self.offboard_pre_switch = True
            self.offboard_pre_switch_count = 0
            # 模式切换会在 publish_offboard_heartbeat 中完成
            return
        
        self.mode_switching = True

        # 发送模式切换命令
        self._send_mode_command(mode_name)
        
        self.mode_switching = False

    def set_arming_callback(self, msg: String):
        """
        处理解锁/上锁命令回调函数（String 类型，兼容原有接口）
        
        Args:
            msg (String): 包含 "arm", "disarm" 或 "force_disarm" 的字符串消息
        """
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的解锁消息类型')
            return

        command = msg.data.lower()
        
        if command == 'arm':
            self._send_arm_command(True)
        elif command == 'disarm':
            # 默认使用强制 disarm，与 QGC 行为一致
            self._send_arm_command(False, force=True)
        elif command == 'force_disarm':
            self._send_arm_command(False, force=True)
        elif command == 'safe_disarm':
            # 安全 disarm（非强制，会被安全检查拒绝）
            self._send_arm_command(False, force=False)
        else:
            self.get_logger().error(f'无效的解锁命令: {command}，应为 "arm", "disarm" 或 "force_disarm"')
    
    def set_arm_bool_callback(self, msg: Bool):
        """
        处理解锁/上锁命令回调函数（Bool 类型，新接口）
        
        Args:
            msg (Bool): True 表示解锁，False 表示上锁（强制）
        """
        self._send_arm_command(msg.data, force=not msg.data)

    def _send_arm_command(self, arm: bool, force: bool = False):
        """
        发送解锁/上锁命令到 PX4
        
        Args:
            arm (bool): True 表示解锁，False 表示上锁
            force (bool): 是否强制执行（21196 = 强制解锁/上锁魔术值）
        """
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0 if arm else 0.0  # 1 = arm, 0 = disarm
        cmd.param2 = 21196.0 if force else 0.0  # 21196 = 强制标志（与 QGC 一致）
        cmd.target_system = self.target_system
        cmd.target_component = self.target_component
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(cmd)
        
        action = "解锁" if arm else "上锁"
        force_str = "(强制)" if force else ""
        self.get_logger().info(f'✈️ 发送{action}命令{force_str}')

    def _send_mode_command(self, mode_name: str):
        """
        发送模式切换命令到 PX4
        
        Args:
            mode_name (str): 目标模式名称
        """
        if mode_name not in self.MODE_MAP:
            self.get_logger().error(f'未知模式: {mode_name}')
            return
            
        main_mode, sub_mode = self.MODE_MAP[mode_name]
        
        # 构建 custom_mode 值
        # PX4 custom_mode 格式: [reserved(8bit)][main_mode(8bit)][sub_mode(8bit)][reserved(8bit)]
        custom_mode = float((main_mode << 16) | (sub_mode << 24))
        
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0  # 使用自定义模式
        cmd.param2 = float(main_mode)
        cmd.param3 = float(sub_mode)
        cmd.target_system = self.target_system
        cmd.target_component = self.target_component
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(cmd)
        
        self.get_logger().info(f'🔄 发送模式切换命令: {mode_name} (main={main_mode}, sub={sub_mode})')

    def send_takeoff_command(self, altitude: float = 10.0):
        """
        发送起飞命令
        
        Args:
            altitude (float): 目标起飞高度（米）
        """
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_NAV_TAKEOFF
        cmd.param1 = -1.0  # Pitch angle (unused)
        cmd.param2 = 0.0   # Empty
        cmd.param3 = 0.0   # Empty
        cmd.param4 = float('nan')  # Yaw angle (NaN = current heading)
        cmd.param5 = float('nan')  # Latitude (NaN = current position)
        cmd.param6 = float('nan')  # Longitude (NaN = current position)
        cmd.param7 = altitude      # Altitude
        cmd.target_system = self.target_system
        cmd.target_component = self.target_component
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(cmd)
        
        self.get_logger().info(f'🛫 发送起飞命令，目标高度: {altitude}m')

    def send_land_command(self):
        """发送降落命令"""
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_NAV_LAND
        cmd.param5 = float('nan')  # Latitude (NaN = current position)
        cmd.param6 = float('nan')  # Longitude (NaN = current position)
        cmd.param7 = 0.0           # Altitude (ground level)
        cmd.target_system = self.target_system
        cmd.target_component = self.target_component
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(cmd)
        
        self.get_logger().info('🛬 发送降落命令')

    def send_rtl_command(self):
        """发送返航命令"""
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        cmd.target_system = self.target_system
        cmd.target_component = self.target_component
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(cmd)
        
        self.get_logger().info('🏠 发送返航命令')


def main(args=None):
    """节点主函数"""
    rclpy.init(args=args)
    node = UsvCommandPx4Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
