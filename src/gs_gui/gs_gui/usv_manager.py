"""
USV 管理器 PX4 适配层 - PX4 uXRCE-DDS 版本

该模块提供与原 usv_manager 兼容的接口，
但使用 PX4 原生消息类型替代 MAVROS 消息。

用于替代 usv_manager.py 中的 MAVROS 依赖。
"""

from typing import Dict, Optional, Callable, Any
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

# PX4 消息类型
from px4_msgs.msg import (
    VehicleStatus,
    BatteryStatus,
    VehicleLocalPosition,
    LogMessage,  # 替代 StatusText
    Event,       # PX4 事件消息（包含 arming denied 等）
)

# 自定义消息
from common_interfaces.msg import UsvStatus


class UsvStatusInfo:
    """单个 USV 状态信息"""
    
    # 导航状态名称映射
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
    
    def __init__(self, usv_id: str):
        self.usv_id = usv_id
        self.connected = False
        self.armed = False
        self.mode = ""
        self.nav_state = 0
        self.arming_state = 0
        self.battery_voltage = 0.0
        self.battery_percent = 0.0
        self.battery_remaining = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.heading = 0.0
        self.last_update_time = 0.0
        
        # PX4 特有状态字段
        self.in_air = False
        self.landed = True
        self.pre_flight_checks_pass = False
        self.failsafe = False
        
        # 健康状态
        self.health_warning_flags = 0
        self.health_error_flags = 0
        self.arming_check_error_flags = 0
        self.gcs_connection_lost = False
        
        # 飞控消息
        self.last_status_text = ""
        self.last_status_severity = 7
        
        # 系统状态
        self.cpu_load = 0.0
        self.errors_count = 0
        self.warnings = []
    
    def update_from_vehicle_status(self, msg: VehicleStatus):
        """从 VehicleStatus 更新"""
        self.connected = True
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
        self.mode = self.NAV_STATE_NAMES.get(msg.nav_state, f'UNKNOWN({msg.nav_state})')
        
        # PX4 特有状态字段
        self.in_air = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED and 
                      msg.takeoff_time > 0)
        self.landed = (msg.nav_state == 12 or  # DESCEND
                      msg.nav_state == 18)     # AUTO_LAND
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass
        self.failsafe = msg.failsafe
    
    def update_from_battery(self, msg: BatteryStatus):
        """从 BatteryStatus 更新"""
        self.battery_voltage = msg.voltage_v
        self.battery_remaining = msg.remaining
        # 计算百分比（如果 remaining 有效）
        if msg.remaining >= 0:
            self.battery_percent = msg.remaining * 100.0
    
    def update_from_local_position(self, msg: VehicleLocalPosition):
        """从 VehicleLocalPosition 更新"""
        if msg.xy_valid and msg.z_valid:
            # PX4 使用 NED，转换为 ENU 显示
            self.position_x = msg.y   # ENU_x = NED_y
            self.position_y = msg.x   # ENU_y = NED_x
            self.position_z = -msg.z  # ENU_z = -NED_z
            
            self.velocity_x = msg.vy
            self.velocity_y = msg.vx
            self.velocity_z = -msg.vz
            
            self.heading = msg.heading

    def update_from_usv_status(self, msg: UsvStatus):
        """从 UsvStatus 聚合消息更新（使用 ROS 2 标准消息结构）"""
        import time
        self.connected = msg.connected
        self.armed = msg.armed
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
        self.mode = msg.mode if msg.mode else self.NAV_STATE_NAMES.get(msg.nav_state, f'UNKNOWN({msg.nav_state})')
        
        # PX4 特有状态字段
        self.in_air = msg.in_air
        self.landed = msg.landed
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass
        self.failsafe = msg.failsafe
        
        # 电池信息
        self.battery_voltage = msg.battery_voltage
        self.battery_percent = msg.battery_percentage
        
        # 位置信息（来自 geometry_msgs/Pose）
        self.position_x = msg.pose.position.x
        self.position_y = msg.pose.position.y
        self.position_z = msg.pose.position.z
        
        # 速度信息（来自 geometry_msgs/Twist）
        self.velocity_x = msg.twist.linear.x
        self.velocity_y = msg.twist.linear.y
        self.velocity_z = msg.twist.linear.z
        
        # 航向
        self.heading = msg.heading
        
        # 健康状态
        self.health_warning_flags = msg.health_warning_flags
        self.health_error_flags = msg.health_error_flags
        self.arming_check_error_flags = msg.arming_check_error_flags
        self.gcs_connection_lost = msg.gcs_connection_lost
        
        # 飞控消息
        self.last_status_text = msg.last_status_text
        self.last_status_severity = msg.last_status_severity
        
        # 更新时间
        self.last_update_time = time.time()


class UsvManager:
    """
    USV 管理器 - PX4 uXRCE-DDS 版本
    
    管理多个 USV 的状态订阅和命令发布。
    """

    def __init__(self, node: Node):
        """
        初始化管理器
        
        Args:
            node: ROS 2 节点实例
        """
        self.node = node
        self.logger = node.get_logger()
        
        # QoS 配置
        self.qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # USV 状态缓存
        self._usv_status: Dict[str, UsvStatusInfo] = {}
        
        # 订阅器字典
        self._vehicle_status_subs: Dict[str, Any] = {}
        self._battery_subs: Dict[str, Any] = {}
        self._local_pos_subs: Dict[str, Any] = {}
        self._log_message_subs: Dict[str, Any] = {}
        self._event_subs: Dict[str, Any] = {}  # PX4 Event 订阅器
        self._usv_state_subs: Dict[str, Any] = {}  # usv_state 聚合话题订阅器
        
        # 远程模式标志（通过 Zenoh 桥接的 USV 使用 usv_state）
        self._remote_mode_usvs: set = set()
        
        # 命令发布器字典
        self._mode_pubs: Dict[str, Any] = {}
        self._arming_pubs: Dict[str, Any] = {}
        
        # 兼容性别名（用于 ground_station_node.py）
        self.usv_state_subs: Dict[str, Any] = self._vehicle_status_subs
        self.set_usv_mode_pubs: Dict[str, Any] = self._mode_pubs
        self.set_usv_arming_pubs: Dict[str, Any] = self._arming_pubs
        self.led_pubs: Dict[str, Any] = {}
        self.sound_pubs: Dict[str, Any] = {}
        self.action_pubs: Dict[str, Any] = {}
        self.navigation_goal_pubs: Dict[str, Any] = {}
        self.clear_target_pubs: Dict[str, Any] = {}  # 清除目标点发布器
        self.nav_goal_pubs: Dict[str, Any] = self.navigation_goal_pubs  # 别名兼容
        self.attitude_cmd_pubs: Dict[str, Any] = {}  # 姿态动作指令发布器（单向）
        
        # 回调函数
        self._on_status_update: Optional[Callable[[str, UsvStatusInfo], None]] = None
        self._on_log_message: Optional[Callable[[str, str], None]] = None
        self._on_event: Optional[Callable[[str, int, bytes], None]] = None  # Event 回调
        
        self.logger.info('✅ USV 管理器 (PX4 版本) 已初始化')

    def set_callbacks(
        self,
        on_status_update: Optional[Callable[[str, UsvStatusInfo], None]] = None,
        on_log_message: Optional[Callable[[str, str], None]] = None,
        on_event: Optional[Callable[[str, int, bytes], None]] = None
    ):
        """
        设置回调函数
        
        Args:
            on_status_update: 状态更新回调 (usv_id, status_info)
            on_log_message: 日志消息回调 (usv_id, message)
            on_event: PX4 事件回调 (usv_id, event_id, arguments)
        """
        self._on_status_update = on_status_update
        self._on_log_message = on_log_message
        self._on_event = on_event

    def add_usv_namespace(self, namespace: str, remote_mode: bool = None):
        """
        添加 USV 命名空间并创建订阅器
        
        Args:
            namespace: USV 命名空间（如 'usv_01'）
            remote_mode: 是否使用远程模式（订阅 usv_state）
                        None = 自动检测（检查 PX4 话题是否存在）
                        True = 强制远程模式
                        False = 强制本地模式
        """
        # 移除前导斜杠
        usv_id = namespace.lstrip('/').replace('/', '')
        
        if usv_id in self._usv_status:
            self.logger.warning(f'USV {usv_id} 已存在')
            return
        
        # 创建状态对象
        self._usv_status[usv_id] = UsvStatusInfo(usv_id)
        
        prefix = f'/{usv_id}'
        
        # 自动检测模式：检查 PX4 话题是否存在
        if remote_mode is None:
            topic_list = [name for name, _ in self.node.get_topic_names_and_types()]
            px4_topic = f'{prefix}/fmu/out/vehicle_status'
            usv_state_topic = f'{prefix}/usv_state'
            
            if px4_topic in topic_list:
                remote_mode = False  # PX4 话题存在，使用本地模式
            elif usv_state_topic in topic_list:
                remote_mode = True   # 只有 usv_state，使用远程模式
            else:
                remote_mode = True   # 默认使用远程模式（Zenoh 桥接）
        
        if remote_mode:
            # 远程模式：订阅 usv_state 聚合话题
            self._remote_mode_usvs.add(usv_id)
            self._subscribe_usv_state(usv_id, prefix)
            self.logger.info(f'✅ 已添加 USV: {usv_id} (远程模式 - usv_state)')
        else:
            # 本地模式：订阅 PX4 原生话题
            self._subscribe_px4_topics(usv_id, prefix)
            self.logger.info(f'✅ 已添加 USV: {usv_id} (本地模式 - PX4)')
        
        # 创建命令发布器（两种模式都需要）
        self._create_command_publishers(usv_id, prefix)
    
    def _subscribe_usv_state(self, usv_id: str, prefix: str):
        """订阅 usv_state 聚合话题（远程模式）"""
        self._usv_state_subs[usv_id] = self.node.create_subscription(
            UsvStatus,
            f'{prefix}/usv_state',
            lambda msg, uid=usv_id: self._usv_state_callback(uid, msg),
            10  # RELIABLE QoS
        )
    
    def _subscribe_px4_topics(self, usv_id: str, prefix: str):
        """订阅 PX4 原生话题（本地模式）"""
        # VehicleStatus 订阅
        self._vehicle_status_subs[usv_id] = self.node.create_subscription(
            VehicleStatus,
            f'{prefix}/fmu/out/vehicle_status',
            lambda msg, uid=usv_id: self._vehicle_status_callback(uid, msg),
            self.qos_px4
        )
        
        # BatteryStatus 订阅
        self._battery_subs[usv_id] = self.node.create_subscription(
            BatteryStatus,
            f'{prefix}/fmu/out/battery_status',
            lambda msg, uid=usv_id: self._battery_callback(uid, msg),
            self.qos_px4
        )
        
        # VehicleLocalPosition 订阅
        self._local_pos_subs[usv_id] = self.node.create_subscription(
            VehicleLocalPosition,
            f'{prefix}/fmu/out/vehicle_local_position',
            lambda msg, uid=usv_id: self._local_position_callback(uid, msg),
            self.qos_px4
        )
        
        # LogMessage 订阅
        try:
            self._log_message_subs[usv_id] = self.node.create_subscription(
                LogMessage,
                f'{prefix}/fmu/out/log_message',
                lambda msg, uid=usv_id: self._log_message_callback(uid, msg),
                self.qos_px4
            )
        except Exception as e:
            self.logger.debug(f'LogMessage 订阅失败: {e}')
        
        # Event 订阅（用于接收 arming denied 等飞控事件消息）
        try:
            self._event_subs[usv_id] = self.node.create_subscription(
                Event,
                f'{prefix}/fmu/out/event',
                lambda msg, uid=usv_id: self._event_callback(uid, msg),
                self.qos_px4
            )
            self.logger.debug(f'{usv_id}: 已订阅 Event 话题')
        except Exception as e:
            self.logger.debug(f'Event 订阅失败: {e}')
    
    def _create_command_publishers(self, usv_id: str, prefix: str):
        """创建命令发布器"""
        from common_interfaces.msg import NavigationGoal, AttitudeCommand
        from std_msgs.msg import Bool
        
        self._mode_pubs[usv_id] = self.node.create_publisher(
            String,
            f'{prefix}/set_usv_mode',
            self.qos_reliable
        )
        
        self._arming_pubs[usv_id] = self.node.create_publisher(
            String,
            f'{prefix}/set_usv_arming',
            self.qos_reliable
        )
        
        self.navigation_goal_pubs[usv_id] = self.node.create_publisher(
            NavigationGoal,
            f'{prefix}/navigation/goal',
            self.qos_reliable
        )
        
        self.clear_target_pubs[usv_id] = self.node.create_publisher(
            Bool,
            f'{prefix}/navigation/clear_target',
            self.qos_reliable
        )

        self.attitude_cmd_pubs[usv_id] = self.node.create_publisher(
            AttitudeCommand,
            f'{prefix}/attitude/command',
            self.qos_reliable
        )

    def remove_usv_namespace(self, namespace: str):
        """
        移除 USV 命名空间
        
        Args:
            namespace: USV 命名空间
        """
        usv_id = namespace.replace('/', '')
        
        if usv_id not in self._usv_status:
            return
        
        # 销毁订阅器（本地模式）
        if usv_id in self._vehicle_status_subs:
            self.node.destroy_subscription(self._vehicle_status_subs.pop(usv_id))
        if usv_id in self._battery_subs:
            self.node.destroy_subscription(self._battery_subs.pop(usv_id))
        if usv_id in self._local_pos_subs:
            self.node.destroy_subscription(self._local_pos_subs.pop(usv_id))
        if usv_id in self._log_message_subs:
            self.node.destroy_subscription(self._log_message_subs.pop(usv_id))
        if usv_id in self._event_subs:
            self.node.destroy_subscription(self._event_subs.pop(usv_id))
        
        # 销毁订阅器（远程模式）
        if usv_id in self._usv_state_subs:
            self.node.destroy_subscription(self._usv_state_subs.pop(usv_id))
        if usv_id in self._remote_mode_usvs:
            self._remote_mode_usvs.discard(usv_id)
        
        # 销毁发布器
        if usv_id in self._mode_pubs:
            self.node.destroy_publisher(self._mode_pubs.pop(usv_id))
        if usv_id in self._arming_pubs:
            self.node.destroy_publisher(self._arming_pubs.pop(usv_id))
        if usv_id in self.navigation_goal_pubs:
            self.node.destroy_publisher(self.navigation_goal_pubs.pop(usv_id))
        if usv_id in self.clear_target_pubs:
            self.node.destroy_publisher(self.clear_target_pubs.pop(usv_id))
        if usv_id in self.attitude_cmd_pubs:
            self.node.destroy_publisher(self.attitude_cmd_pubs.pop(usv_id))
        
        # 移除状态
        del self._usv_status[usv_id]
        
        self.logger.info(f'已移除 USV: {usv_id}')

    def _usv_state_callback(self, usv_id: str, msg: UsvStatus):
        """UsvStatus 聚合消息回调（远程模式）"""
        if usv_id in self._usv_status:
            self._usv_status[usv_id].update_from_usv_status(msg)
            
            if self._on_status_update:
                self._on_status_update(usv_id, self._usv_status[usv_id])

    def _vehicle_status_callback(self, usv_id: str, msg: VehicleStatus):
        """VehicleStatus 回调"""
        if usv_id in self._usv_status:
            self._usv_status[usv_id].update_from_vehicle_status(msg)
            self._usv_status[usv_id].last_update_time = self.node.get_clock().now().nanoseconds / 1e9
            
            if self._on_status_update:
                self._on_status_update(usv_id, self._usv_status[usv_id])

    def _battery_callback(self, usv_id: str, msg: BatteryStatus):
        """BatteryStatus 回调"""
        if usv_id in self._usv_status:
            self._usv_status[usv_id].update_from_battery(msg)

    def _local_position_callback(self, usv_id: str, msg: VehicleLocalPosition):
        """VehicleLocalPosition 回调"""
        if usv_id in self._usv_status:
            self._usv_status[usv_id].update_from_local_position(msg)

    def _log_message_callback(self, usv_id: str, msg: LogMessage):
        """LogMessage 回调"""
        if self._on_log_message:
            try:
                # LogMessage 包含 text 字段
                text = ''.join(chr(c) for c in msg.text if c != 0)
                self._on_log_message(usv_id, text)
            except Exception:
                pass

    def _event_callback(self, usv_id: str, msg: Event):
        """
        PX4 Event 回调
        
        处理飞控事件消息，如 arming denied、mode changes 等
        """
        if self._on_event:
            try:
                event_id = msg.id
                arguments = bytes(msg.arguments)
                self._on_event(usv_id, event_id, arguments)
            except Exception:
                pass

    def get_usv_status(self, usv_id: str) -> Optional[UsvStatusInfo]:
        """
        获取指定 USV 的状态
        
        Args:
            usv_id: USV ID
            
        Returns:
            状态信息，不存在返回 None
        """
        return self._usv_status.get(usv_id)

    def get_all_usv_ids(self) -> list:
        """获取所有 USV ID"""
        return list(self._usv_status.keys())

    def set_mode(self, usv_id: str, mode: str) -> bool:
        """
        设置 USV 模式
        
        Args:
            usv_id: USV ID
            mode: 目标模式
            
        Returns:
            是否成功发送
        """
        if usv_id not in self._mode_pubs:
            self.logger.warning(f'USV {usv_id} 不存在')
            return False
        
        msg = String()
        msg.data = mode
        self._mode_pubs[usv_id].publish(msg)
        self.logger.info(f'发送模式切换命令: {usv_id} -> {mode}')
        return True

    def set_arming(self, usv_id: str, arm: bool) -> bool:
        """
        设置 USV 解锁/上锁状态
        
        Args:
            usv_id: USV ID
            arm: True=解锁, False=上锁
            
        Returns:
            是否成功发送
        """
        if usv_id not in self._arming_pubs:
            self.logger.warning(f'USV {usv_id} 不存在')
            return False
        
        msg = String()
        msg.data = 'arm' if arm else 'disarm'
        self._arming_pubs[usv_id].publish(msg)
        self.logger.info(f'发送{"解锁" if arm else "上锁"}命令: {usv_id}')
        return True

    def cleanup(self):
        """清理所有资源"""
        for usv_id in list(self._usv_status.keys()):
            self.remove_usv_namespace(usv_id)
        
        self.logger.info('USV 管理器已清理')
