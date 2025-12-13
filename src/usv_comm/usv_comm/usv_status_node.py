"""
无人球状态节点 - PX4 uXRCE-DDS 版本

该节点负责收集无人球的各种状态信息并整合发布。
通过 PX4 uXRCE-DDS 原生话题获取状态，替代 MAVROS。

话题映射：
- MAVROS /mavros/state        -> /fmu/out/vehicle_status_v1  (PX4 v1.15+)
- MAVROS /mavros/battery      -> /fmu/out/battery_status
- MAVROS /mavros/local_position/pose -> /fmu/out/vehicle_local_position
- MAVROS /mavros/local_position/velocity_local -> /fmu/out/vehicle_local_position
"""

from math import sqrt, atan2
import math
import glob
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from common_interfaces.msg import UsvStatus
import psutil
import time
import json
import os
from collections import deque

# PX4 消息类型
from px4_msgs.msg import (
    VehicleStatus,
    VehicleLocalPosition,
    BatteryStatus,
    VehicleAttitude,
    FailsafeFlags,    # 已修复：添加了 reserved0 字段以匹配 184 字节
    EstimatorStatusFlags,  # 传感器状态标志
    Event,  # PX4 事件消息（arming denied 等）
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
        super().__init__('usv_status_node')

        # =====================================================================
        # QoS 配置
        # =====================================================================
        # PX4 uXRCE-DDS 使用 BEST_EFFORT + VOLATILE
        qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
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
        # 注意：PX4 v1.15+ 发布的是 vehicle_status_v1 话题
        # px4_msgs/msg/VehicleStatus 的 MESSAGE_VERSION=1 对应 vehicle_status_v1
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
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
        
        # 失控保护标志订阅 - 已启用（px4_msgs 已添加兼容字段）
        self.failsafe_flags_sub = self.create_subscription(
            FailsafeFlags,
            'fmu/out/failsafe_flags',
            self.failsafe_flags_callback,
            qos_px4
        )
        
        # 估计器状态标志订阅 - 用于传感器健康状态
        self.estimator_status_flags_sub = self.create_subscription(
            EstimatorStatusFlags,
            'fmu/out/estimator_status_flags',
            self.estimator_status_flags_callback,
            qos_px4
        )
        
        # Event 订阅 - 用于接收飞控事件（arming denied 等）
        self.event_sub = self.create_subscription(
            Event,
            'fmu/out/event',
            self.event_callback,
            qos_px4
        )
        
        # 加载事件定义文件（用于解码事件 ID）
        self._event_definitions = {}
        self._load_event_definitions()

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
        self.failsafe_flags = None    # 已启用
        self.estimator_status_flags = None  # 传感器健康状态
        
        # 事件消息缓存（用于显示最近的飞控事件）
        self._recent_events = deque(maxlen=5)  # 最多保存5条最近事件
        self._last_event_time = 0.0
        
        # 电压历史缓存
        self.voltage_history = deque()
        self.low_voltage_mode = False
        self.low_voltage_triggered = False

        # 系统温度缓存（摄氏度）。未知时为 NaN，便于在消息里表达“无数据”。
        self.latest_temperature_c = math.nan
        
        # 数据时效性跟踪
        self.last_status_time = 0.0
        self.last_battery_time = 0.0
        self.last_position_time = 0.0
        self.last_attitude_time = 0.0
        self.last_failsafe_time = 0.0
        
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
        self.get_logger().info('📥 订阅: vehicle_status, battery, position, attitude, failsafe_flags, event')
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

    def failsafe_flags_callback(self, msg: FailsafeFlags):
        """失控保护标志回调 - 提供详细的预检和安全状态"""
        self.failsafe_flags = msg
        self.last_failsafe_time = time.time()

    def estimator_status_flags_callback(self, msg: EstimatorStatusFlags):
        """估计器状态标志回调 - 提供传感器健康状态"""
        self.estimator_status_flags = msg

    def event_callback(self, msg: Event):
        """PX4 Event 回调 - 处理飞控事件消息（arming denied 等）"""
        try:
            event_id = msg.id
            decoded_msg = self._decode_event(event_id)
            
            if decoded_msg:
                # 根据消息内容判断严重性
                msg_upper = decoded_msg.upper()
                if 'DENIED' in msg_upper or 'FAIL' in msg_upper or 'CRITICAL' in msg_upper:
                    severity = 3  # ERROR
                elif 'WARN' in msg_upper:
                    severity = 4  # WARNING
                else:
                    severity = 6  # INFO
                
                # 添加到最近事件缓存
                self._recent_events.appendleft({
                    'text': decoded_msg,
                    'severity': severity,
                    'time': time.time()
                })
                self._last_event_time = time.time()
                
                # 记录日志
                self.get_logger().info(f'[FCU-EVENT] {decoded_msg}')
        except Exception as e:
            self.get_logger().debug(f'事件解码失败: {e}')

    def _load_event_definitions(self):
        """加载 PX4 事件定义文件"""
        # 尝试多个可能的路径
        from ament_index_python.packages import get_package_share_directory
        
        possible_paths = []
        
        # 1. 尝试从 ROS 2 包路径加载
        try:
            usv_comm_share = get_package_share_directory('usv_comm')
            possible_paths.append(os.path.join(usv_comm_share, 'config', 'px4_events.json'))
        except Exception:
            pass
        
        try:
            gs_gui_share = get_package_share_directory('gs_gui')
            possible_paths.append(os.path.join(gs_gui_share, 'config', 'px4_events.json'))
        except Exception:
            pass
        
        # 2. 尝试从常见路径加载
        possible_paths.extend([
            os.path.expanduser('~/usv_workspace/src/usv_comm/config/px4_events.json'),
            os.path.expanduser('~/usv_workspace/src/gs_gui/config/px4_events.json'),
            os.path.expanduser('~/px4_events.json'),
            '/opt/ros/humble/share/px4_msgs/config/px4_events.json',
        ])
        
        for path in possible_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    # 解析事件定义
                    count = 0
                    if 'components' in data:
                        for comp_id, comp_data in data['components'].items():
                            # 直接在 component 下的 events
                            if 'events' in comp_data:
                                for event_id, event_def in comp_data['events'].items():
                                    try:
                                        eid = int(event_id)
                                        self._event_definitions[eid] = event_def
                                        count += 1
                                    except ValueError:
                                        pass
                            
                            # 在 event_groups 下的 events
                            if 'event_groups' in comp_data:
                                for group_name, group_data in comp_data['event_groups'].items():
                                    if 'events' in group_data:
                                        for event_id, event_def in group_data['events'].items():
                                            try:
                                                eid = int(event_id)
                                                self._event_definitions[eid] = event_def
                                                count += 1
                                            except ValueError:
                                                pass
                    
                    self.get_logger().info(f'已加载 {count} 条事件定义: {path}')
                    return
                except Exception as e:
                    self.get_logger().warn(f'加载事件定义失败: {e}')
        
        self.get_logger().warn('未找到 px4_events.json，事件消息将显示为原始 ID')

    def _decode_event(self, event_id: int) -> str:
        """解码事件 ID 为可读消息
        
        PX4 事件 ID 结构: (component_id << 24) | sub_id
        px4_events.json 中使用的是 sub_id（24位）
        """
        # 提取 sub_id（低24位）用于查找事件定义
        sub_id = event_id & 0xFFFFFF
        
        # 先尝试用 sub_id 查找
        event_def = self._event_definitions.get(sub_id)
        if event_def:
            return event_def.get('message', event_def.get('name', f'Event {sub_id}'))
        
        # 如果没找到，尝试用完整的 event_id 查找（兼容旧格式）
        event_def = self._event_definitions.get(event_id)
        if event_def:
            return event_def.get('message', event_def.get('name', f'Event {event_id}'))
        
        return None

    def _get_recent_event(self) -> dict:
        """获取最近的有效事件消息（5秒内）"""
        current_time = time.time()
        # 只返回5秒内的事件
        if self._recent_events and (current_time - self._last_event_time) < 5.0:
            return self._recent_events[0]
        return None

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
        
        # 判断飞控连接状态：vehicle_status 或 vehicle_local_position 任一有效即认为连接
        # 因为某些 PX4 配置可能不发布 vehicle_status，但位置数据是必须的
        current_time = time.time()
        has_vehicle_status = (self.vehicle_status is not None and 
                              (current_time - self.last_status_time) < 2.0)
        has_position = (self.local_position is not None and 
                        (current_time - self.last_position_time) < 2.0)
        
        # 飞控连接：有 vehicle_status 或有位置数据
        fc_connected = has_vehicle_status or has_position
        
        # 飞控状态
        if has_vehicle_status:
            msg.connected = True
            msg.armed = (self.vehicle_status.arming_state == 2)  # ARMING_STATE_ARMED
            msg.mode = self._get_mode_name(self.vehicle_status.nav_state)
            
            # PX4 特有状态字段
            msg.nav_state = self.vehicle_status.nav_state
            msg.arming_state = self.vehicle_status.arming_state
            
            # 判断是否在空中：已解锁且起飞时间已设置
            msg.in_air = (self.vehicle_status.arming_state == 2 and 
                         self.vehicle_status.takeoff_time > 0)
            
            # 判断是否已着陆：处于降落或悬停等待着陆状态
            msg.landed = (self.vehicle_status.nav_state == 12 or  # DESCEND
                         self.vehicle_status.nav_state == 18)     # AUTO_LAND
            
            # 预检通过标志和失控保护
            msg.pre_flight_checks_pass = self.vehicle_status.pre_flight_checks_pass
            msg.failsafe = self.vehicle_status.failsafe
        elif has_position:
            # 有位置数据但没有 vehicle_status：飞控连接但状态未知
            msg.connected = True  # 有数据就认为连接
            msg.armed = False
            msg.mode = 'UNKNOWN'  # 无法确定模式
            
            # PX4 特有状态字段默认值
            msg.nav_state = 0
            msg.arming_state = 0
            msg.in_air = False
            msg.landed = True
            msg.pre_flight_checks_pass = False
            msg.failsafe = False
        else:
            # 完全断开：没有任何数据
            msg.connected = False
            msg.armed = False
            msg.mode = 'UNKNOWN'
            
            # PX4 特有状态字段默认值
            msg.nav_state = 0
            msg.arming_state = 0
            msg.in_air = False
            msg.landed = True
            msg.pre_flight_checks_pass = False
            msg.failsafe = False
        
        # 电池状态
        if self.battery_status is not None:
            msg.battery_voltage = self.battery_status.voltage_v
            msg.battery_current = self.battery_status.current_a
            msg.battery_percentage = self._calculate_battery_percentage(self.battery_status.voltage_v)
            msg.power_supply_status = UsvStatus.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.battery_voltage = 0.0
            msg.battery_current = 0.0
            msg.battery_percentage = 0.0
            msg.power_supply_status = UsvStatus.POWER_SUPPLY_STATUS_UNKNOWN
        
        # 位置信息（NED 转 ENU）- 使用 geometry_msgs/Pose
        if self.local_position is not None:
            # PX4 NED -> ROS ENU
            msg.pose.position.x = float(self.local_position.y)   # East = Y_ned
            msg.pose.position.y = float(self.local_position.x)   # North = X_ned
            msg.pose.position.z = float(-self.local_position.z)  # Up = -Down
            
            # 速度 - 使用 geometry_msgs/Twist
            msg.twist.linear.x = float(self.local_position.vy)
            msg.twist.linear.y = float(self.local_position.vx)
            msg.twist.linear.z = float(-self.local_position.vz)
            
            # 航向（弧度，按消息定义归一化到 [0, 2π)）
            heading = float(self.local_position.heading)
            msg.heading = (heading + 2.0 * math.pi) % (2.0 * math.pi)
            
            # 高度
            msg.altitude_relative = float(-self.local_position.z)
        
        # 姿态信息
        if self.attitude is not None:
            # 四元数：PX4 VehicleAttitude.q 为 FRD->NED。
            # 为与 ROS 生态兼容，这里转换为 FLU->ENU 后再填充到 geometry_msgs/Pose。
            q_w, q_x, q_y, q_z = self._px4_attitude_quat_to_ros_enu_flu(self.attitude.q)
            msg.pose.orientation.w = float(q_w)
            msg.pose.orientation.x = float(q_x)
            msg.pose.orientation.y = float(q_y)
            msg.pose.orientation.z = float(q_z)

            # 从转换后的四元数计算欧拉角（弧度，ROS ENU 语义）
            roll, pitch, yaw = self._quaternion_to_euler([q_w, q_x, q_y, q_z])
            msg.roll = float(roll)
            msg.pitch = float(pitch)
            msg.yaw = float(yaw)
            
            # 角速度
            if hasattr(self.attitude, 'angular_velocity'):
                msg.twist.angular.x = float(self.attitude.angular_velocity[0])
                msg.twist.angular.y = float(self.attitude.angular_velocity[1])
                msg.twist.angular.z = float(self.attitude.angular_velocity[2])
        
        # 计算到目标点的距离
        msg.distance_to_target = self._calculate_distance_to_target()
        msg.target_reached = msg.distance_to_target < self.target_reach_threshold

        # 系统温度（摄氏度）。未知时保持 NaN。
        msg.temperature = float(self.latest_temperature_c)
        
        # =====================================================================
        # 健康状态信息（来自 FailsafeFlags）
        # =====================================================================
        msg.health_warning_flags = 0
        msg.health_error_flags = 0
        msg.arming_check_error_flags = 0
        msg.gcs_connection_lost = False
        
        # 检查 failsafe_flags 是否可用且新鲜
        has_failsafe_flags = (self.failsafe_flags is not None and
                              (current_time - self.last_failsafe_time) < 2.0)
        
        if has_failsafe_flags:
            # 使用 FailsafeFlags 获取详细状态
            ff = self.failsafe_flags
            
            # 构建警告标志（非关键但应注意）
            warning_bits = 0
            if hasattr(ff, 'wind_limit_exceeded') and ff.wind_limit_exceeded:
                warning_bits |= (1 << 0)
            if hasattr(ff, 'flight_time_limit_exceeded') and ff.flight_time_limit_exceeded:
                warning_bits |= (1 << 1)
            if hasattr(ff, 'position_accuracy_low') and ff.position_accuracy_low:
                warning_bits |= (1 << 2)
            if hasattr(ff, 'battery_warning') and ff.battery_warning > 0:
                warning_bits |= (1 << 3)
            msg.health_warning_flags = warning_bits
            
            # 构建错误标志（关键问题）
            error_bits = 0
            if hasattr(ff, 'angular_velocity_invalid') and ff.angular_velocity_invalid:
                error_bits |= (1 << 0)
            if hasattr(ff, 'attitude_invalid') and ff.attitude_invalid:
                error_bits |= (1 << 1)
            if hasattr(ff, 'local_position_invalid') and ff.local_position_invalid:
                error_bits |= (1 << 2)
            if hasattr(ff, 'local_velocity_invalid') and ff.local_velocity_invalid:
                error_bits |= (1 << 3)
            if hasattr(ff, 'fd_critical_failure') and ff.fd_critical_failure:
                error_bits |= (1 << 4)
            if hasattr(ff, 'fd_esc_arming_failure') and ff.fd_esc_arming_failure:
                error_bits |= (1 << 5)
            if hasattr(ff, 'fd_motor_failure') and ff.fd_motor_failure:
                error_bits |= (1 << 6)
            if hasattr(ff, 'battery_unhealthy') and ff.battery_unhealthy:
                error_bits |= (1 << 7)
            msg.health_error_flags = error_bits
            
            # 解锁检查错误
            arming_bits = 0
            if hasattr(ff, 'manual_control_signal_lost') and ff.manual_control_signal_lost:
                arming_bits |= (1 << 0)
            if hasattr(ff, 'home_position_invalid') and ff.home_position_invalid:
                arming_bits |= (1 << 1)
            if hasattr(ff, 'geofence_breached') and ff.geofence_breached:
                arming_bits |= (1 << 2)
            msg.arming_check_error_flags = arming_bits
            
            # GCS 连接状态
            msg.gcs_connection_lost = hasattr(ff, 'gcs_connection_lost') and ff.gcs_connection_lost
            
            # 生成状态文本
            status_texts = []
            if ff.angular_velocity_invalid:
                status_texts.append('角速度无效')
            if ff.attitude_invalid:
                status_texts.append('姿态无效')
            if ff.local_position_invalid:
                status_texts.append('位置无效')
            if ff.manual_control_signal_lost:
                status_texts.append('遥控器信号丢失')
            if ff.gcs_connection_lost:
                status_texts.append('地面站连接断开')
            if hasattr(ff, 'battery_warning') and ff.battery_warning >= 2:
                status_texts.append(f'电池警告级别{ff.battery_warning}')
            if ff.fd_critical_failure:
                status_texts.append('严重故障')
            
            # 添加最近的事件消息（如 arming denied）
            recent_event = self._get_recent_event()
            if recent_event:
                status_texts.insert(0, recent_event['text'])
            
            if status_texts:
                msg.last_status_text = '; '.join(status_texts)
                # 如果有事件消息，使用事件的严重性级别
                if recent_event:
                    msg.last_status_severity = recent_event['severity']
                else:
                    msg.last_status_severity = 2 if error_bits else 4  # ERROR or WARNING
            else:
                msg.last_status_text = '系统正常'
                msg.last_status_severity = 6  # INFO
        elif has_vehicle_status:
            # 回退到 vehicle_status 基本信息
            status_texts = []
            if not self.vehicle_status.pre_flight_checks_pass:
                status_texts.append('预检未通过')
            if self.vehicle_status.failsafe:
                status_texts.append('失控保护激活')
            
            # 添加最近的事件消息
            recent_event = self._get_recent_event()
            if recent_event:
                status_texts.insert(0, recent_event['text'])
            
            if status_texts:
                msg.last_status_text = '; '.join(status_texts)
                if recent_event:
                    msg.last_status_severity = recent_event['severity']
                else:
                    msg.last_status_severity = 4  # WARNING
            else:
                msg.last_status_text = '系统正常'
                msg.last_status_severity = 6  # INFO
        else:
            msg.last_status_text = '无飞控数据'
            msg.last_status_severity = 4  # WARNING
        
        # =====================================================================
        # 传感器状态信息（来自 EstimatorStatusFlags）
        # =====================================================================
        if self.estimator_status_flags is not None:
            esf = self.estimator_status_flags
            # 陀螺仪和加速度计通过 tilt_align 判断（IMU 正常时 tilt_align 为 True）
            msg.sensor_gyro_ok = esf.cs_tilt_align
            msg.sensor_accel_ok = esf.cs_tilt_align
            # 磁罗盘状态
            msg.sensor_mag_ok = esf.cs_mag or esf.cs_mag_hdg
            # 气压计状态
            msg.sensor_baro_ok = esf.cs_baro_hgt and not esf.cs_baro_fault
            # GPS 状态
            msg.sensor_gps_ok = esf.cs_gnss_pos or esf.cs_gps_hgt
        else:
            # 未知状态
            msg.sensor_gyro_ok = False
            msg.sensor_accel_ok = False
            msg.sensor_mag_ok = False
            msg.sensor_baro_ok = False
            msg.sensor_gps_ok = False
        
        # 数据时效性检查
        current_time = time.time()
        data_age = current_time - max(self.last_status_time, self.last_position_time)
        msg.data_valid = data_age < self.data_timeout
        
        # 数据年龄 (builtin_interfaces/Duration)
        msg.data_age.sec = int(data_age)
        msg.data_age.nanosec = int((data_age - int(data_age)) * 1e9)
        
        # 消息序号
        msg.sequence = self.message_count
        
        # 低电量模式
        msg.low_voltage_mode = self.low_voltage_mode
        
        # 发布状态
        self.state_publisher.publish(msg)
        self.message_count += 1

    def publish_temperature(self):
        """发布系统温度"""
        try:
            temp_c = self._get_system_temperature_c()
            self.latest_temperature_c = float(temp_c)
            if not math.isnan(self.latest_temperature_c):
                temp_msg = Float32()
                temp_msg.data = self.latest_temperature_c
                self.temperature_publisher.publish(temp_msg)
        except Exception as e:
            pass  # 静默处理，避免刷屏

    # =========================================================================
    # 坐标系/四元数工具
    # =========================================================================

    def _mat3_mul(self, a, b):
        """3x3 矩阵乘法：a @ b"""
        return [
            [
                a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
                for j in range(3)
            ]
            for i in range(3)
        ]

    def _quat_wxyz_to_rot(self, w: float, x: float, y: float, z: float):
        """四元数(w,x,y,z)转旋转矩阵(3x3)。"""
        xx = x * x
        yy = y * y
        zz = z * z
        wx = w * x
        wy = w * y
        wz = w * z
        xy = x * y
        xz = x * z
        yz = y * z

        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]

    def _rot_to_quat_wxyz(self, r):
        """旋转矩阵(3x3)转四元数(w,x,y,z)。"""
        tr = r[0][0] + r[1][1] + r[2][2]
        if tr > 0.0:
            s = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * s
            x = (r[2][1] - r[1][2]) / s
            y = (r[0][2] - r[2][0]) / s
            z = (r[1][0] - r[0][1]) / s
        elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
            s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
            w = (r[2][1] - r[1][2]) / s
            x = 0.25 * s
            y = (r[0][1] + r[1][0]) / s
            z = (r[0][2] + r[2][0]) / s
        elif r[1][1] > r[2][2]:
            s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
            w = (r[0][2] - r[2][0]) / s
            x = (r[0][1] + r[1][0]) / s
            y = 0.25 * s
            z = (r[1][2] + r[2][1]) / s
        else:
            s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
            w = (r[1][0] - r[0][1]) / s
            x = (r[0][2] + r[2][0]) / s
            y = (r[1][2] + r[2][1]) / s
            z = 0.25 * s

        # 归一化，避免数值误差
        n = math.sqrt(w * w + x * x + y * y + z * z)
        if n > 0.0:
            w /= n
            x /= n
            y /= n
            z /= n
        return w, x, y, z

    def _px4_attitude_quat_to_ros_enu_flu(self, q_wxyz):
        """PX4 VehicleAttitude.q(FRD->NED) 转 ROS Pose 四元数(FLU->ENU)。

        约定：
        - PX4 本体坐标：FRD (x前, y右, z下)
        - PX4 世界坐标：NED (x北, y东, z下)
        - ROS base_link：FLU (x前, y左, z上)
        - ROS 世界(地图/里程计)：ENU (x东, y北, z上)
        """
        w, x, y, z = float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])
        r_frd_to_ned = self._quat_wxyz_to_rot(w, x, y, z)

        # 常量变换矩阵
        r_ned_to_enu = [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
        r_flu_to_frd = [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ]

        r_flu_to_enu = self._mat3_mul(self._mat3_mul(r_ned_to_enu, r_frd_to_ned), r_flu_to_frd)
        return self._rot_to_quat_wxyz(r_flu_to_enu)

    def _get_system_temperature_c(self) -> float:
        """获取系统温度（摄氏度）。优先 psutil，其次 sysfs；失败返回 NaN。"""
        # 1) psutil (更通用，但在某些环境可能返回空)
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                for _, entries in temps.items():
                    if entries and entries[0] is not None and entries[0].current is not None:
                        return float(entries[0].current)
        except Exception:
            pass

        # 2) sysfs fallback: /sys/class/thermal/thermal_zone*/temp (常见于 Linux SBC)
        sysfs_temp = self._read_sysfs_temperature_c()
        if sysfs_temp is not None:
            return sysfs_temp

        return math.nan

    def _read_sysfs_temperature_c(self) -> float | None:
        """从 sysfs 读取温度（摄氏度）。返回 None 表示无可用传感器。"""
        try:
            for path in sorted(glob.glob('/sys/class/thermal/thermal_zone*/temp')):
                try:
                    with open(path, 'r', encoding='utf-8') as f:
                        raw = f.read().strip()
                    if not raw:
                        continue
                    val = float(raw)
                    # 大多数平台该值为毫摄氏度（例如 42000 表示 42.0°C）
                    if val > 1000.0:
                        val = val / 1000.0
                    # 简单过滤明显无效值
                    if -50.0 <= val <= 150.0:
                        return float(val)
                except Exception:
                    continue
        except Exception:
            return None
        return None

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
        # 标准实现：pitch = asin(sinp)，并对数值误差做 clamp
        sinp = 2 * (w * y - z * x)
        if sinp > 1.0:
            sinp = 1.0
        elif sinp < -1.0:
            sinp = -1.0
        pitch = math.asin(sinp)
        
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
