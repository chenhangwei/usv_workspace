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
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)

# 导入 Trigger 服务用于优雅关闭
from std_srvs.srv import Trigger


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
        'HOLD': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
        'LOITER': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
        'AUTO.READY': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_READY),
        'AUTO.TAKEOFF': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF),
        'AUTO.LOITER': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
        'AUTO.MISSION': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION),
        'AUTO.RTL': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL),
        'RTL': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL),
        'AUTO.LAND': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND),
        'LAND': (PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND),
    }

    # 需要遥控摇杆输入的 PX4 nav_state 集合（无 RC 时 PX4 Commander 会拒绝解锁）
    MANUAL_RC_NAV_STATES = {0, 1, 2, 6, 15, 19}

    def __init__(self):
        """初始化无人球命令控制节点"""
        super().__init__('usv_command_node')

        # =====================================================================
        # QoS 配置 - PX4 uXRCE-DDS 必须使用 BEST_EFFORT + VOLATILE
        # =====================================================================
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
        # 参数声明
        # =====================================================================
        self.declare_parameter('supported_modes', [
            'OFFBOARD', 'HOLD', 'MANUAL', 'STABILIZED', 'POSCTL', 'ALTCTL',
            'AUTO.LOITER', 'AUTO.MISSION', 'AUTO.RTL', 'AUTO.LAND', 'AUTO.TAKEOFF'
        ])
        self.declare_parameter('command_timeout_sec', 5.0)
        # 0 表示从飞控 VehicleStatus.system_id 动态学习（兼容多实例 PX4 SITL 与任意 MAV_SYS_ID 实机）
        self.declare_parameter('target_system', 0)
        self.declare_parameter('target_component', 1)

        # 获取参数值
        self.supported_modes = self.get_parameter('supported_modes').get_parameter_value().string_array_value
        self.command_timeout_sec = self.get_parameter('command_timeout_sec').get_parameter_value().double_value
        self.configured_target_system = int(self.get_parameter('target_system').get_parameter_value().integer_value)
        self.target_system = self.configured_target_system
        self.target_component = int(self.get_parameter('target_component').get_parameter_value().integer_value)

        # =====================================================================
        # 状态变量
        # =====================================================================
        self.current_status = None
        self.failsafe_flags = None
        self.local_position_valid = False
        self.last_mode_command = None
        self.last_mode_time = 0.0
        self.mode_debounce_sec = 0.5
        self.mode_switching = False
        self.pending_arm_after_offboard = False
        self.pending_arm_force = False
        self.pending_hold_fallback = False

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
        
        # 仅在 usv_control_node 未启动或预切换阶段兜底发送 TrajectorySetpoint
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos_px4
        )

        # =====================================================================
        # 订阅器 - 飞控状态与命令应答
        # =====================================================================
        # 注意：PX4 v1.15+ 发布的是 vehicle_status_v1 话题
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_px4
        )

        self.failsafe_sub = self.create_subscription(
            FailsafeFlags,
            'fmu/out/failsafe_flags',
            self.failsafe_flags_callback,
            qos_px4
        )

        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            'fmu/out/vehicle_command_ack',
            self.command_ack_callback,
            qos_px4
        )
        
        # 订阅本地位置，用于预切换兜底保持当前位置
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
        # OFFBOARD 模式预切换定时器（仅在预切换阶段工作，常规保活由 usv_control_node 负责）
        # =====================================================================
        self.offboard_heartbeat_timer = self.create_timer(0.05, self.publish_offboard_heartbeat)
        self.offboard_mode_active = False
        
        # OFFBOARD 模式预切换状态（当飞控尚未收到有效 Offboard 信号时快速预热）
        self.offboard_pre_switch = False
        self.offboard_pre_switch_count = 0
        self.offboard_pre_switch_target = 6  # 50ms * 6 = 0.3 秒预热
        
        # 当前位置（用于预切换兜底保持位置）
        self.current_position = [0.0, 0.0, 0.0]  # NED 坐标

        # =====================================================================
        # 服务 - 优雅关闭
        # =====================================================================
        self.shutdown_srv = self.create_service(
            Trigger,
            'shutdown_all',
            self.shutdown_callback
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('PX4 uXRCE-DDS 命令控制节点已启动')
        self.get_logger().info(f'支持的模式: {", ".join(self.supported_modes)}')
        target_desc = 'auto(动态跟随 VehicleStatus.system_id)' if self.configured_target_system == 0 else str(self.target_system)
        self.get_logger().info(f'目标系统: {target_desc}, 目标组件: {self.target_component}')
        self.get_logger().info('=' * 60)

    def shutdown_callback(self, request, response):
        """优雅关闭所有节点的回调函数"""
        self.get_logger().warn('🛑 收到关闭指令，正在准备退出系统...')
        response.success = True
        response.message = "USV 节点正在关闭"
        
        # 延迟 1 秒执行退出，确保响应能发出去
        import threading
        import sys
        def delayed_exit():
            import time
            time.sleep(1.0)
            self.get_logger().info('👋 系统退出')
            # 在 ROS 2 中，通常通过 context.shutdown() 或直接 sys.exit()
            # 如果 launch 文件配置了 on_exit=Shutdown()，则会关闭整个 launch
            sys.exit(0)
            
        threading.Thread(target=delayed_exit).start()
        return response

    def vehicle_status_callback(self, msg: VehicleStatus):
        """
        飞控状态回调
        
        Args:
            msg (VehicleStatus): PX4 飞控状态消息
        """
        prev_sys_id = self.target_system
        self.current_status = msg

        # 动态对齐 PX4 system_id / component_id，避免多实例 SITL (-i 1 -> sys_id=2 等) 丢弃命令
        if self.configured_target_system <= 0 and int(msg.system_id) > 0:
            self.target_system = int(msg.system_id)
            if prev_sys_id != self.target_system:
                self.get_logger().info(f'🔗 已同步飞控 target_system={self.target_system}')
        if int(msg.component_id) > 0:
            self.target_component = int(msg.component_id)
        
        # 检查是否处于 OFFBOARD 模式 (nav_state == 14)
        self.offboard_mode_active = (msg.nav_state == 14)

        # 若此前因“无遥控器处于手动模式”触发了先切 OFFBOARD 再解锁，则切入 OFFBOARD 后立即补发解锁
        if self.pending_arm_after_offboard and self.offboard_mode_active:
            self.pending_arm_after_offboard = False
            force = self.pending_arm_force
            self.pending_arm_force = False
            self.get_logger().info('✅ 已进入 OFFBOARD 模式，继续执行自动解锁')
            self._publish_arm_vehicle_command(True, force=force)

    def failsafe_flags_callback(self, msg: FailsafeFlags):
        """接收飞控故障保护与信号丢失标志"""
        self.failsafe_flags = msg

    def command_ack_callback(self, msg: VehicleCommandAck):
        """打印 PX4 命令执行回执，便于排查拒绝原因"""
        if msg.command not in (
            self.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            self.VEHICLE_CMD_DO_SET_MODE,
        ):
            return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            if msg.command == self.VEHICLE_CMD_DO_SET_MODE:
                self.pending_hold_fallback = False
            self.get_logger().info(f'✅ PX4 接受命令 cmd={msg.command}')
        else:
            self.get_logger().warn(
                f'⚠️ PX4 拒绝命令 cmd={msg.command}, result={msg.result}, reason={msg.result_param1}'
            )
            if msg.command == self.VEHICLE_CMD_DO_SET_MODE and self.pending_hold_fallback:
                self.pending_hold_fallback = False
                self.get_logger().info(
                    '🔄 HOLD(AUTO.LOITER) 被飞控拒绝（通常因室内 UWB 无全局 GPS），自动回退 OFFBOARD 原地驻留'
                )
                self._request_offboard_mode()

    def local_position_callback(self, msg: VehicleLocalPosition):
        """
        本地位置回调
        
        用于更新当前位置，在预切换阶段兜底保持当前位置。
        
        Args:
            msg (VehicleLocalPosition): PX4 本地位置消息
        """
        if msg.xy_valid:
            z_val = msg.z if msg.z_valid else 0.0
            self.current_position = [float(msg.x), float(msg.y), float(z_val)]
            self.local_position_valid = True

    def _has_control_node_offboard_publisher(self) -> bool:
        """检查当前命名空间下是否已有其他节点（如 usv_control_node）发布 offboard_control_mode"""
        try:
            topic_name = self.offboard_mode_pub.topic_name
            return self.count_publishers(topic_name) > 1
        except Exception:
            return False

    def publish_offboard_heartbeat(self):
        """
        OFFBOARD 预切换辅助定时器
        
        常规运行期间由 usv_control_node 以 20Hz 独占发布 OffboardControlMode 与 TrajectorySetpoint，
        本节点绝不再持续发送“锁定原地”的 TrajectorySetpoint，彻底消除双发布者抢夺控制权问题！
        仅当处于 offboard_pre_switch 阶段（且无其他控制节点在发流时）补充预热信号。
        """
        if not self.offboard_pre_switch:
            return

        # 若 usv_control_node 未运行，才由本节点兜底发送预热设定值；否则不干扰 usv_control_node 的设定值流
        if not self._has_control_node_offboard_publisher():
            now_us = int(self.get_clock().now().nanoseconds / 1000)
            ocm = OffboardControlMode()
            ocm.position = True
            ocm.velocity = False
            ocm.acceleration = False
            ocm.attitude = False
            ocm.body_rate = False
            ocm.timestamp = now_us
            self.offboard_mode_pub.publish(ocm)

            sp = TrajectorySetpoint()
            sp.position = list(self.current_position)
            sp.yaw = float('nan')
            sp.timestamp = now_us
            self.trajectory_setpoint_pub.publish(sp)

        self.offboard_pre_switch_count += 1
        if self.offboard_pre_switch_count >= self.offboard_pre_switch_target:
            self.offboard_pre_switch = False
            self.offboard_pre_switch_count = 0
            self.get_logger().info('📡 OFFBOARD 预热完成，发送模式切换命令')
            self._send_mode_command('OFFBOARD')
            if self.pending_arm_after_offboard:
                force = self.pending_arm_force
                self.pending_arm_force = False
                self.pending_arm_after_offboard = False
                self._publish_arm_vehicle_command(True, force=force)

    def _request_offboard_mode(self):
        """请求切入 OFFBOARD：若已有持续 Offboard 流则立即切换，否则短暂预热后切换"""
        if self.offboard_mode_active:
            self.get_logger().info('已处于 OFFBOARD 模式')
            return

        offboard_signal_missing = (
            self.failsafe_flags is None or bool(self.failsafe_flags.offboard_control_signal_lost)
        )
        if not offboard_signal_missing or self._has_control_node_offboard_publisher():
            # usv_control_node 已经在 20Hz 广播 Offboard 流，直接下发 DO_SET_MODE 即可秒切！
            if not offboard_signal_missing:
                self._send_mode_command('OFFBOARD')
                return

        self.get_logger().info('🔄 开始 OFFBOARD 模式预热...')
        self.offboard_pre_switch = True
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

        mode_name = msg.data.strip().upper()

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
        
        if mode_name == 'OFFBOARD':
            self.pending_hold_fallback = False
            self._request_offboard_mode()
            return
        
        self.pending_hold_fallback = mode_name in ('HOLD', 'LOITER', 'AUTO.LOITER')
        self.mode_switching = True
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

        command = msg.data.strip().lower()
        
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

    def _effective_target_system(self) -> int:
        """获取有效的 PX4 target_system（0 表示广播到当前 DDS 会话内的飞控实例）"""
        if self.target_system > 0:
            return int(self.target_system)
        if self.current_status is not None and int(self.current_status.system_id) > 0:
            return int(self.current_status.system_id)
        return 0

    def _send_arm_command(self, arm: bool, force: bool = False):
        """
        发送解锁/上锁命令到 PX4。
        针对无遥控器（COM_RC_IN_MODE=4 / manual_control_signal_lost）场景：
        若当前处于需要摇杆的手动模式（STABILIZED / MANUAL / POSCTL / ALTCTL 等），
        PX4 Commander 会因 "No manual control input" 拒绝解锁。
        此时自动先切到无需摇杆的 OFFBOARD（若无 Offboard 信号则切 HOLD/AUTO.LOITER）再执行解锁！
        """
        if not arm:
            self.pending_arm_after_offboard = False
            self.pending_arm_force = False
            self._publish_arm_vehicle_command(False, force=force)
            return

        if self.current_status is not None:
            nav_state = int(self.current_status.nav_state)
            rc_missing = (
                self.failsafe_flags is None or bool(self.failsafe_flags.manual_control_signal_lost)
            )
            if nav_state in self.MANUAL_RC_NAV_STATES and rc_missing:
                offboard_ready = (
                    self.failsafe_flags is not None
                    and not bool(self.failsafe_flags.offboard_control_signal_lost)
                )
                if offboard_ready:
                    self.get_logger().info(
                        f'🔄 当前模式(nav_state={nav_state})需要遥控器输入，自动先切换 OFFBOARD 再解锁'
                    )
                    self.pending_arm_after_offboard = True
                    self.pending_arm_force = force
                    self._send_mode_command('OFFBOARD')
                    # 同时直接发一次解锁（PX4 同帧/紧邻帧处理完模式切换即可解锁；若未生效由状态回调补发）
                    self._publish_arm_vehicle_command(True, force=force)
                    return
                elif self._has_control_node_offboard_publisher() or self.local_position_valid:
                    self.get_logger().info(
                        f'🔄 当前模式(nav_state={nav_state})需要遥控器输入，预热切换 OFFBOARD 后自动解锁'
                    )
                    self.pending_arm_after_offboard = True
                    self.pending_arm_force = force
                    self.offboard_pre_switch = True
                    self.offboard_pre_switch_count = 0
                    return
                else:
                    self.get_logger().info(
                        f'🔄 当前模式(nav_state={nav_state})需要遥控器输入且无位置流，先切 HOLD(AUTO.LOITER) 再解锁'
                    )
                    self._send_mode_command('HOLD')

        self._publish_arm_vehicle_command(True, force=force)

    def _publish_arm_vehicle_command(self, arm: bool, force: bool = False):
        """底层发布 VEHICLE_CMD_COMPONENT_ARM_DISARM"""
        target_sys = self._effective_target_system()
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0 if arm else 0.0  # 1 = arm, 0 = disarm
        cmd.param2 = 21196.0 if force else 0.0  # 21196 = 强制标志（与 QGC 一致）
        cmd.target_system = target_sys
        cmd.target_component = self.target_component
        cmd.source_system = 255
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(cmd)
        
        action = "解锁" if arm else "上锁"
        force_str = "(强制)" if force else ""
        self.get_logger().info(f'✈️ 发送{action}命令{force_str} (target_system={target_sys})')

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
        target_sys = self._effective_target_system()
        
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0  # 使用自定义模式
        cmd.param2 = float(main_mode)
        cmd.param3 = float(sub_mode)
        cmd.target_system = target_sys
        cmd.target_component = self.target_component
        cmd.source_system = 255
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(cmd)
        
        self.get_logger().info(
            f'🔄 发送模式切换命令: {mode_name} (main={main_mode}, sub={sub_mode}, target_system={target_sys})'
        )

    def send_takeoff_command(self, altitude: float = 10.0):
        """
        发送起飞命令
        
        Args:
            altitude (float): 目标起飞高度（米）
        """
        target_sys = self._effective_target_system()
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_NAV_TAKEOFF
        cmd.param1 = -1.0  # Pitch angle (unused)
        cmd.param2 = 0.0   # Empty
        cmd.param3 = 0.0   # Empty
        cmd.param4 = float('nan')  # Yaw angle (NaN = current heading)
        cmd.param5 = float('nan')  # Latitude (NaN = current position)
        cmd.param6 = float('nan')  # Longitude (NaN = current position)
        cmd.param7 = altitude      # Altitude
        cmd.target_system = target_sys
        cmd.target_component = self.target_component
        cmd.source_system = 255
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_pub.publish(cmd)
        
        self.get_logger().info(f'🛫 发送起飞命令，目标高度: {altitude}m (target_system={target_sys})')

    def send_land_command(self):
        """发送降落命令"""
        target_sys = self._effective_target_system()
        cmd = VehicleCommand()
        cmd.command = self.VEHICLE_CMD_NAV_LAND
        cmd.param5 = float('nan')  # Latitude (NaN = current position)
        cmd.param6 = float('nan')  # Longitude (NaN = current position)
        cmd.param7 = 0.0           # Altitude (ground level)
        cmd.target_system = target_sys
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
