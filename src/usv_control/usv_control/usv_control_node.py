"""
无人球控制节点 - PX4 uXRCE-DDS 版本

该节点负责处理无人球的目标点控制逻辑。
使用 TrajectorySetpoint 消息发送目标点，替代 MAVROS 的 PositionTarget。

话题映射：
- MAVROS /mavros/setpoint_raw/local -> /fmu/in/trajectory_setpoint
- MAVROS /mavros/local_position/pose -> /fmu/out/vehicle_local_position
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, Float32, String
import json
import time

# PX4 消息类型
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
)
try:
    from px4_msgs.msg import TrajectorySetpoint6dof
except ImportError:
    TrajectorySetpoint6dof = None

# 自定义接口（地面站导航目标 / 姿态动作指令）
from common_interfaces.msg import NavigationGoal, NavigationFeedback, NavigationResult, NavigationAck, AttitudeCommand

# 导入 common_utils 工具
from common_utils import ParamLoader, ParamValidator


class UsvControlPx4Node(Node):
    """
    无人球控制节点类 - PX4 uXRCE-DDS 版本
    
    该节点实现目标点控制逻辑，处理常规目标点和避障目标点，
    根据避障标志决定使用哪个目标点，并将选定的目标点发布给 PX4 飞控。
    """

    def __init__(self):
        """初始化无人球控制节点"""
        super().__init__('usv_control_node')
        
        # =====================================================================
        # 参数加载
        # =====================================================================
        param_loader = ParamLoader(self)
        
        self.publish_rate = param_loader.load_param(
            'publish_rate',
            20.0,
            ParamValidator.frequency,
            '目标点发布频率(Hz)'
        )
        self.frame_id = param_loader.load_param(
            'frame_id',
            'map',
            ParamValidator.non_empty_string,
            '坐标系ID'
        )
        
        # 声明额外参数
        self.declare_parameter('target_reach_threshold', 1.0)
        # 3D 平台到达判定阈值：水平(XY)与高度(Z)分别判断。
        # 默认使用 target_reach_threshold 以尽量保持旧行为。
        self.declare_parameter('reach_xy_threshold', 1.0)
        self.declare_parameter('reach_z_threshold', 1.0)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('coordinate_system', 'NED')  # NED 或 ENU
        # 仿真多机世界坐标原点偏移（ENU，米）；实机默认为 0.0
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)
        self.declare_parameter('spawn_offset_z', 0.0)

        # 平台执行模式：
        # - '3d'：允许姿态(roll/pitch/yaw)目标（是否真正下发给 PX4 由 use_setpoint_6dof 控制）
        # - '2d'：忽略 roll/pitch，只保留位置 + yaw
        self.declare_parameter('platform_mode', '3d')
        # 载具类型：bionic_airship / bionic_dolphin / flying_sphere / usv / usv_diff
        self.declare_parameter('vehicle_type', 'flying_sphere')
        # 导航控制模式：'position'（默认位置轨迹）或 'rl'（由 RL 策略输出 (v, omega) 速度控制）
        self.declare_parameter('nav_mode', 'position')
        self.declare_parameter('rl_cmd_timeout', 0.8)
        self.declare_parameter('rl_z_kp', 0.85)
        self.declare_parameter('rl_z_max_vel', 1.5)
        # 3D 载体（飞行球/海豚/飞艇）起飞预高度与降落保护锁桨高度（ENU，米；<=0 表示按机型自动选择默认高度）
        self.declare_parameter('takeoff_altitude', -1.0)
        self.declare_parameter('landing_altitude', -1.0)
        self.declare_parameter('vertical_climb_speed', 0.8)
        self.declare_parameter('vertical_descent_speed', 0.6)
        # 是否发布 TrajectorySetpoint6dof（位置+quaternion）。默认关闭以保持现有行为。
        self.declare_parameter('use_setpoint_6dof', False)

        # 姿态动作期间的运动承载方式：
        # - 'overlay'：位置仍由 navigation setpoint 控制，姿态(roll/pitch)尝试叠加到 6DoF quaternion
        # - 'attitude_velocity'：姿态动作期间切到 velocity 控制（position=false, velocity=true），位移由速度承载
        #   注意：是否生效取决于 PX4 对 setpoint 的支持情况（见项目文档/实机验证）。
        self.declare_parameter('attitude_action_motion_mode', 'attitude_velocity')
        self.declare_parameter('attitude_action_min_velocity', 0.2)
        # 姿态动作期间速度方向：
        # - 'to_goal'：速度朝向导航目标点（更易收敛，但可能被认为“不沿机头/翻滚轴方向”）
        # - 'body_forward'：速度沿“机体前向/头部方向”(body X forward)，由当前 yaw/pitch 决定
        self.declare_parameter('attitude_action_velocity_direction', 'body_forward')
        
        self.target_reach_threshold = self.get_parameter('target_reach_threshold').value
        self.reach_xy_threshold = float(self.get_parameter('reach_xy_threshold').value)
        self.reach_z_threshold = float(self.get_parameter('reach_z_threshold').value)
        self.max_velocity = self.get_parameter('max_velocity').value
        self.coordinate_system = self.get_parameter('coordinate_system').value
        self.spawn_offset_x = float(self.get_parameter('spawn_offset_x').value)
        self.spawn_offset_y = float(self.get_parameter('spawn_offset_y').value)
        self.spawn_offset_z = float(self.get_parameter('spawn_offset_z').value)

        # 若用户未显式设置 XY/Z 阈值，则沿用 target_reach_threshold
        # （launch/yaml 没配时，这两个参数通常为默认 1.0）
        if not isinstance(self.reach_xy_threshold, (int, float)) or not math.isfinite(self.reach_xy_threshold):
            self.reach_xy_threshold = float(self.target_reach_threshold)
        if not isinstance(self.reach_z_threshold, (int, float)) or not math.isfinite(self.reach_z_threshold):
            self.reach_z_threshold = float(self.target_reach_threshold)

        self.platform_mode = str(self.get_parameter('platform_mode').value).strip().lower()
        if self.platform_mode not in ('2d', '3d'):
            self.get_logger().warn(
                f"platform_mode='{self.platform_mode}' 非法，回退为 '3d'"
            )
            self.platform_mode = '3d'

        self.vehicle_type = str(self.get_parameter('vehicle_type').value).strip().lower()
        if self.vehicle_type not in ('bionic_airship', 'bionic_dolphin', 'flying_sphere', 'usv', 'usv_diff'):
            self.vehicle_type = 'flying_sphere'

        self.nav_mode = str(self.get_parameter('nav_mode').value).strip().lower()
        if self.nav_mode not in ('position', 'pid', 'rl'):
            self.nav_mode = 'position'
        self.rl_cmd_timeout = max(0.1, float(self.get_parameter('rl_cmd_timeout').value))
        self.rl_z_kp = max(0.05, float(self.get_parameter('rl_z_kp').value))
        self.rl_z_max_vel = max(0.1, float(self.get_parameter('rl_z_max_vel').value))

        # 按机型自动补全默认到点阈值（<=0 或未显式配置时生效）
        _default_xy_by_type = {
            'bionic_airship': 4.0,
            'bionic_dolphin': 1.5,
            'flying_sphere': 0.8,
            'usv': float(self.target_reach_threshold),
            'usv_diff': 0.55,
        }
        _default_z_by_type = {
            'bionic_airship': 1.5,
            'bionic_dolphin': 0.8,
            'flying_sphere': 0.5,
            'usv': float(self.target_reach_threshold),
            'usv_diff': 1.0,
        }
        if self.reach_xy_threshold <= 0.0:
            self.reach_xy_threshold = _default_xy_by_type[self.vehicle_type]
        if self.reach_z_threshold <= 0.0:
            self.reach_z_threshold = _default_z_by_type[self.vehicle_type]

        # 按机型补全默认起飞预高度与降落保护高度（ENU, m）
        _default_takeoff_z_by_type = {
            'bionic_airship': 15.0,
            'bionic_dolphin': 6.0,
            'flying_sphere': 2.0,
            'usv': 0.0,
            'usv_diff': 0.0,
        }
        _default_landing_z_by_type = {
            'bionic_airship': 0.5,
            'bionic_dolphin': 0.3,
            'flying_sphere': 0.25,
            'usv': 0.0,
            'usv_diff': 0.0,
        }
        raw_takeoff_z = float(self.get_parameter('takeoff_altitude').value)
        raw_landing_z = float(self.get_parameter('landing_altitude').value)
        self.takeoff_altitude = (
            raw_takeoff_z if (math.isfinite(raw_takeoff_z) and raw_takeoff_z > 0.0)
            else _default_takeoff_z_by_type[self.vehicle_type]
        )
        self.landing_altitude = (
            raw_landing_z if (math.isfinite(raw_landing_z) and raw_landing_z >= 0.0)
            else _default_landing_z_by_type[self.vehicle_type]
        )
        self.vertical_climb_speed = max(0.15, float(self.get_parameter('vertical_climb_speed').value))
        self.vertical_descent_speed = max(0.15, float(self.get_parameter('vertical_descent_speed').value))

        # 非完整单向推进载体（无倒车，需防止近点打转）
        self.is_forward_only_vehicle = self.vehicle_type in ('bionic_airship', 'bionic_dolphin')
        self.enable_yaw = True

        raw_6dof = self.get_parameter('use_setpoint_6dof').value
        if isinstance(raw_6dof, str):
            self.use_setpoint_6dof = raw_6dof.strip().lower() in ('1', 'true', 'yes', 'y', 'on')
        else:
            self.use_setpoint_6dof = bool(raw_6dof)

        raw_motion_mode = str(self.get_parameter('attitude_action_motion_mode').value).strip().lower()
        if raw_motion_mode not in ('overlay', 'attitude_velocity'):
            self.get_logger().warn(
                f"attitude_action_motion_mode='{raw_motion_mode}' 非法，回退为 'attitude_velocity'"
            )
            raw_motion_mode = 'attitude_velocity'
        self.attitude_action_motion_mode = raw_motion_mode

        raw_vel_dir = str(self.get_parameter('attitude_action_velocity_direction').value).strip().lower()
        if raw_vel_dir not in ('to_goal', 'body_forward'):
            self.get_logger().warn(
                f"attitude_action_velocity_direction='{raw_vel_dir}' 非法，回退为 'body_forward'"
            )
            raw_vel_dir = 'body_forward'
        self.attitude_action_velocity_direction = raw_vel_dir

        try:
            self.attitude_action_min_velocity = float(self.get_parameter('attitude_action_min_velocity').value)
        except Exception:
            self.attitude_action_min_velocity = 0.2
        if not math.isfinite(self.attitude_action_min_velocity) or self.attitude_action_min_velocity < 0.0:
            self.attitude_action_min_velocity = 0.2

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
        # 发布器 - 发送目标点与模式指令到 PX4
        # =====================================================================
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos_px4
        )
        
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            qos_px4
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            'fmu/in/vehicle_command',
            qos_px4
        )

        # 可选：6DoF setpoint（位置 + quaternion）
        self.setpoint6dof_pub = (
            self.create_publisher(
                TrajectorySetpoint6dof,
                'fmu/in/trajectory_setpoint6dof',
                qos_px4
            )
            if TrajectorySetpoint6dof is not None
            else None
        )

        # =====================================================================
        # 订阅器 - PX4 状态和位置
        # =====================================================================
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_px4
        )
        
        # 注意：PX4 v1.15+ 发布的是 vehicle_status_v1 话题
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_px4
        )

        # =====================================================================
        # 订阅器 - 地面站命令（保持原有接口兼容）
        # =====================================================================
        self.target_sub = self.create_subscription(
            PoseStamped,
            'set_usv_target_position',
            self.target_callback,
            qos_reliable
        )
        
        # 新版导航目标订阅（NavigationGoal 消息类型，兼容 navigation/goal 与 set_usv_nav_goal）
        self.nav_goal_sub = self.create_subscription(
            NavigationGoal,
            'navigation/goal',
            self.nav_goal_callback,
            qos_reliable
        )
        self.nav_goal_compat_sub = self.create_subscription(
            NavigationGoal,
            'set_usv_nav_goal',
            self.nav_goal_callback,
            qos_reliable
        )
        
        # 导航反馈和结果发布器
        self.nav_feedback_pub = self.create_publisher(
            NavigationFeedback,
            'navigation/feedback',
            qos_reliable
        )

        # 导航应答发布器：收到/接受层（用于 GS 停止 step_timeout 重发）
        self.nav_ack_pub = self.create_publisher(
            NavigationAck,
            'navigation/ack',
            qos_reliable
        )
        
        self.nav_result_pub = self.create_publisher(
            NavigationResult,
            'navigation/result',
            qos_reliable
        )

        # 单向姿态动作指令（不发布完成度反馈）
        self.attitude_cmd_sub = self.create_subscription(
            AttitudeCommand,
            'attitude/command',
            self.attitude_command_callback,
            qos_reliable
        )
        
        self.avoidance_sub = self.create_subscription(
            PoseStamped,
            'avoidance_position',
            self.avoidance_target_callback,
            qos_reliable
        )
        
        self.avoidance_flag_sub = self.create_subscription(
            Bool,
            'avoidance_flag',
            self.avoidance_flag_callback,
            qos_reliable
        )
        
        self.clear_target_sub = self.create_subscription(
            Bool,
            'clear_target',
            self.clear_target_callback,
            qos_reliable
        )
        
        # 清除目标订阅（新版导航接口 & 地面站停止/取消导航兼容接口）
        self.nav_clear_sub = self.create_subscription(
            Bool,
            'navigation/clear_target',
            self.clear_target_callback,
            qos_reliable
        )
        self.stop_nav_sub = self.create_subscription(
            Bool,
            'stop_navigation',
            self.clear_target_callback,
            qos_reliable
        )
        self.cancel_nav_sub = self.create_subscription(
            Bool,
            'cancel_navigation',
            self.clear_target_callback,
            qos_reliable
        )

        # 3D 起飞 / 降落控制命令与手动解锁监听（用于清除降落保护锁桨锁存）
        self.takeoff_cmd_sub = self.create_subscription(
            Float32,
            'takeoff_command',
            self.takeoff_command_callback,
            qos_reliable,
        )
        self.land_cmd_sub = self.create_subscription(
            Float32,
            'land_command',
            self.land_command_callback,
            qos_reliable,
        )
        self.arming_str_sub = self.create_subscription(
            String,
            'set_usv_arming',
            self._manual_arming_str_callback,
            qos_reliable,
        )
        self.arming_bool_sub = self.create_subscription(
            Bool,
            'set_arm',
            self._manual_arming_bool_callback,
            qos_reliable,
        )
        self.flight_state_pub = self.create_publisher(
            String,
            'flight_state',
            qos_reliable,
        )

        # RL 策略输出订阅与闭环反馈发布
        qos_rl = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.rl_cmd_sub = self.create_subscription(
            TwistStamped,
            'rl_policy/cmd_vel',
            self.rl_cmd_callback,
            qos_rl,
        )
        self.raw_cmd_pub = self.create_publisher(TwistStamped, 'velocity_controller/raw_cmd', qos_rl)
        self.final_cmd_pub = self.create_publisher(TwistStamped, 'velocity_controller/final_cmd', qos_rl)

        # =====================================================================
        # 状态变量
        # =====================================================================
        self.current_position = None           # 当前位置 (VehicleLocalPosition)
        self.target_position = None            # 常规目标点 (PoseStamped)
        self.avoidance_position = None         # 避障目标点 (PoseStamped)
        self.avoidance_active = False          # 避障模式是否激活
        self.vehicle_status = None             # 飞控状态
        self.target_active = False             # 目标点是否激活
        self.local_position_valid = False      # 本地位置是否有效
        self.offboard_mode_active = False      # OFFBOARD 模式是否激活
        
        # 导航目标跟踪
        self.current_goal_id = None            # 当前导航目标 ID
        self.goal_start_time = None            # 目标开始时间
        self.goal_timeout = 300.0              # 默认超时时间
        self._goal_reach_xy_threshold = None   # 当前 goal 覆盖阈值（可选）
        self._goal_reach_z_threshold = None
        self._rl_cmd_msg = None                # 最新 RL 速度动作 (TwistStamped)
        self._rl_cmd_mono_time = 0.0
        self._integrated_yaw_ned = None        # 3D 载体 RL 角速度积分航向
        self._idle_hold_ned = None             # 空闲悬停锚点 [x, y, z, yaw_ned]

        # 3D 垂直起飞/降落与降落锁桨保护状态
        self._flight_action = None             # None | 'takeoff' | 'land'
        self._flight_lock_ned_xy_yaw = None    # [ned_x, ned_y, yaw_ned]
        self._flight_target_enu_z = None       # 当前起飞或降落的目标 ENU Z 高度
        self._protection_locked = False        # 降落到位后自动锁桨保护，需手动 Arming 解锁才清除
        self._takeoff_ready = (self.platform_mode == '2d')
        self._last_flight_state_pub_mono = 0.0

        # 姿态动作指令状态（roll/pitch，可选 yaw）
        self._att_cmd_active = False
        self._att_cmd_roll = 0.0
        self._att_cmd_pitch = 0.0
        self._att_cmd_yaw = float('nan')
        self._att_cmd_end_time = None  # rclpy.time.Time | None

        self.flight_state_timer = self.create_timer(0.2, self._publish_flight_state)

        # =====================================================================
        # 日志记录
        # =====================================================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('PX4 uXRCE-DDS 控制节点已启动')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'坐标系: {self.coordinate_system}')
        self.get_logger().info(f'平台模式: {self.platform_mode} (2d忽略roll/pitch)')
        self.get_logger().info(f'6DoF setpoint: {"启用" if self.use_setpoint_6dof else "禁用"} (fmu/in/trajectory_setpoint6dof)')
        self.get_logger().info(f'目标到达阈值: {self.target_reach_threshold} m')
        self.get_logger().info(f'3D到达判定阈值: XY<{self.reach_xy_threshold:.2f}m, |Z|<{self.reach_z_threshold:.2f}m')
        if self.platform_mode == '3d':
            self.get_logger().info(
                f'3D 起降参数: 起飞预高度={self.takeoff_altitude:.2f}m, 降落保护高度={self.landing_altitude:.2f}m'
            )
        self.get_logger().info('📤 发布话题: fmu/in/trajectory_setpoint')
        if self.use_setpoint_6dof:
            self.get_logger().info('📤 发布话题: fmu/in/trajectory_setpoint6dof')
        self.get_logger().info('📥 订阅话题: fmu/out/vehicle_local_position')
        self.get_logger().info('📥 订阅话题: navigation/goal (NavigationGoal)')
        self.get_logger().info('=' * 60)

        # 只打印一次的提醒
        self._warned_enu_rpy = False

    def attitude_command_callback(self, msg: AttitudeCommand):
        """姿态动作指令回调（单向，不回传完成度）。

        约定：roll/pitch/yaw 均为弧度。yaw 可为 NaN 表示“保持当前航向”。
        duration:
          - >0  : 保持 duration 秒后自动失效
          - <=0 : 持续生效直到下一条覆盖
        """
        try:
            self._att_cmd_roll = float(msg.roll)
            self._att_cmd_pitch = float(msg.pitch)
            self._att_cmd_yaw = float(msg.yaw)
            duration = float(msg.duration)
        except Exception as e:
            self.get_logger().warn(f'解析 attitude/command 失败: {e}')
            return

        self._att_cmd_active = True
        if duration > 0.0 and math.isfinite(duration):
            self._att_cmd_end_time = self.get_clock().now() + Duration(seconds=duration)
        else:
            self._att_cmd_end_time = None

        yaw_str = f'{self._att_cmd_yaw:.3f}' if math.isfinite(self._att_cmd_yaw) else 'nan'

        self.get_logger().info(
            f'🎛️ 收到姿态动作: roll={self._att_cmd_roll:.3f} rad, '
            f'pitch={self._att_cmd_pitch:.3f} rad, '
            f'yaw={yaw_str}, '
            f'duration={duration:.2f}s'
        )

    def local_position_callback(self, msg: VehicleLocalPosition):
        """
        本地位置回调
        
        Args:
            msg (VehicleLocalPosition): PX4 本地位置消息
        """
        self.current_position = msg
        
        # 检查位置是否有效
        if msg.xy_valid and msg.z_valid:
            if not self.local_position_valid:
                self.local_position_valid = True
                self.get_logger().info(
                    f'✅ 本地位置有效: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})'
                )

    def status_callback(self, msg: VehicleStatus):
        """
        飞控状态回调
        
        Args:
            msg (VehicleStatus): PX4 飞控状态消息
        """
        prev_armed = (
            self.vehicle_status is not None
            and int(self.vehicle_status.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        )
        now_armed = (int(msg.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED))
        self.vehicle_status = msg
        # nav_state == 14 表示 OFFBOARD 模式
        self.offboard_mode_active = (msg.nav_state == 14)
        # 未解锁或刚解锁瞬间刷新空闲锚点，避免带着历史偏移
        if not now_armed or (now_armed and not prev_armed):
            self._idle_hold_ned = None
        if not now_armed:
            if self.platform_mode == '3d':
                self._takeoff_ready = False
            if self._flight_action == 'takeoff':
                self._flight_action = None
        elif now_armed and not prev_armed and self._protection_locked:
            # 飞控已实际重新解锁（例如通过手动命令解锁生效），同步解除保护锁存
            self._protection_locked = False

    def _current_enu_z(self):
        """获取当前 ENU 世界坐标系下的 Z 高度（米），无效时返回 None"""
        if self.current_position is None or not bool(self.current_position.z_valid):
            return None
        return float(-float(self.current_position.z) + self.spawn_offset_z)

    def _enu_z_to_ned_z(self, enu_z: float) -> float:
        """将 ENU Z 高度转换为当前飞控实例局部 NED Z（Down）"""
        return -(float(enu_z) - self.spawn_offset_z)

    def _takeoff_z_tolerance(self) -> float:
        """起飞预高度到位容差（米）"""
        return max(0.20, min(float(self.reach_z_threshold), 0.60))

    def _send_force_disarm_command(self, reason: str = ''):
        """向 PX4 立即发送强制上锁锁桨指令（用于到达降落高度后的电机保护锁止）"""
        target_sys = 0
        target_comp = 1
        if self.vehicle_status is not None:
            if int(self.vehicle_status.system_id) > 0:
                target_sys = int(self.vehicle_status.system_id)
            if int(self.vehicle_status.component_id) > 0:
                target_comp = int(self.vehicle_status.component_id)
        cmd = VehicleCommand()
        cmd.command = 400      # VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 0.0       # Disarm
        cmd.param2 = 21196.0   # 强制上锁锁桨
        cmd.target_system = target_sys
        cmd.target_component = target_comp
        cmd.source_system = 255
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(cmd)
        self.get_logger().warn(f'🔒 触发降落电机保护锁止，已发送强制上锁命令 ({reason})')

    def _manual_arming_str_callback(self, msg: String):
        """监听地面站手动解锁命令，仅手动 arm 时解除降落保护锁定"""
        cmd = str(msg.data or '').strip().lower()
        if cmd == 'arm':
            if self._protection_locked:
                self.get_logger().info('🔓 收到手动 Arming 解锁指令，解除降落电机保护锁止状态')
            self._protection_locked = False
            self._flight_action = None
        elif cmd in ('disarm', 'force_disarm', 'safe_disarm'):
            self._flight_action = None
            self.target_active = False
            if self.platform_mode == '3d':
                self._takeoff_ready = False
        self._publish_flight_state()

    def _manual_arming_bool_callback(self, msg: Bool):
        """监听 Bool 解锁接口，True 时解除降落保护锁定"""
        if bool(msg.data):
            if self._protection_locked:
                self.get_logger().info('🔓 收到手动 set_arm=True 解锁指令，解除降落电机保护锁止状态')
            self._protection_locked = False
            self._flight_action = None
        else:
            self._flight_action = None
            self.target_active = False
            if self.platform_mode == '3d':
                self._takeoff_ready = False
        self._publish_flight_state()

    def takeoff_command_callback(self, msg: Float32):
        """
        处理 3D 载体（飞行球/海豚/飞艇）起飞指令：
        - msg.data > 0: 指定本次起飞预高度（ENU Z，米），并更新 self.takeoff_altitude
        - msg.data <= 0: 使用当前 self.takeoff_altitude 默认预高度
        - 若处于降落电机保护锁止或未解锁状态，拒绝起飞并要求先手动点击 Arming 解锁
        """
        if self.platform_mode == '2d' or self.vehicle_type in ('usv', 'usv_diff'):
            self.get_logger().info('ℹ️ 2D 水面船无需执行 Z 向起飞动作，直接标记就绪')
            self._takeoff_ready = True
            self._publish_flight_state()
            return

        if self._protection_locked:
            self.get_logger().warn('🔒 当前处于降落电机保护锁止状态，请先手动点击 Arming 解锁后再执行起飞！')
            self._publish_flight_state()
            return

        is_armed = (
            self.vehicle_status is not None
            and int(self.vehicle_status.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        )
        if not is_armed:
            self.get_logger().warn('⚠️ 当前未解锁（Disarmed），请先手动点击 Arming 解锁后再执行起飞！')
            self._publish_flight_state()
            return

        if self.current_position is None or not bool(self.current_position.xy_valid):
            self.get_logger().warn('⚠️ 本地位置无效，暂时无法执行起飞')
            return

        req_z = float(msg.data)
        if math.isfinite(req_z) and req_z > 0.0:
            self.takeoff_altitude = req_z
        target_z = max(0.3, float(self.takeoff_altitude))

        # 停止任何进行中的水平导航目标，原地锁定水平 XY 与航向，纯垂直爬升至预高度
        self.target_active = False
        self.target_position = None
        self.current_goal_id = None
        self._att_cmd_active = False

        cur_x = float(self.current_position.x)
        cur_y = float(self.current_position.y)
        cur_yaw = float(self.current_position.heading) if math.isfinite(float(self.current_position.heading)) else 0.0
        self._flight_lock_ned_xy_yaw = [cur_x, cur_y, cur_yaw]
        self._flight_target_enu_z = target_z
        self._flight_action = 'takeoff'
        self._idle_hold_ned = None
        self._ensure_offboard_if_armed()
        self.get_logger().info(f'🛫 开始垂直起飞：Z 爬升至设定预高度 {target_z:.2f}m')
        self._publish_flight_state()

    def land_command_callback(self, msg: Float32):
        """
        处理 3D 载体（飞行球/海豚/飞艇）降落指令：
        - msg.data >= 0: 指定本次降落保护高度（ENU Z，米），并更新 self.landing_altitude
        - msg.data < 0: 使用当前 self.landing_altitude 默认降落高度
        - Z 下降到 <= landing_altitude 时立即强制上锁电机并进入 _protection_locked 保护锁止状态
        """
        if self.platform_mode == '2d' or self.vehicle_type in ('usv', 'usv_diff'):
            self.get_logger().info('ℹ️ 2D 水面船无需执行 Z 向降落动作，仅清除当前导航目标')
            self.target_active = False
            self.target_position = None
            self.current_goal_id = None
            return

        req_land_z = float(msg.data)
        if math.isfinite(req_land_z) and req_land_z >= 0.0:
            self.landing_altitude = req_land_z
        land_z = max(0.0, float(self.landing_altitude))

        # 立即终止水平导航目标，转为原地垂直下降
        if self.target_active and self.current_goal_id is not None:
            self._send_nav_result(False, '已切换至降落动作')
        self.target_active = False
        self.target_position = None
        self.current_goal_id = None
        self._att_cmd_active = False
        self._takeoff_ready = False

        cur_enu_z = self._current_enu_z()
        is_armed = (
            self.vehicle_status is not None
            and int(self.vehicle_status.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        )

        # 若当前高度已经 <= 设定降落高度，直接触发电机保护锁止！
        if cur_enu_z is not None and cur_enu_z <= land_z:
            self._flight_action = None
            self._protection_locked = True
            if is_armed:
                self._send_force_disarm_command(f'当前高度 Z={cur_enu_z:.2f}m <= 降落高度 {land_z:.2f}m')
            self._publish_flight_state()
            return

        if not is_armed:
            self._flight_action = None
            self._protection_locked = True
            self.get_logger().info(f'🔒 当前已上锁，直接进入降落电机保护锁止状态 (land_z={land_z:.2f}m)')
            self._publish_flight_state()
            return

        if self.current_position is None or not bool(self.current_position.xy_valid):
            self._send_force_disarm_command('位置无效紧急降落锁桨')
            self._protection_locked = True
            self._publish_flight_state()
            return

        cur_x = float(self.current_position.x)
        cur_y = float(self.current_position.y)
        cur_yaw = float(self.current_position.heading) if math.isfinite(float(self.current_position.heading)) else 0.0
        self._flight_lock_ned_xy_yaw = [cur_x, cur_y, cur_yaw]
        self._flight_target_enu_z = land_z
        self._flight_action = 'land'
        self._idle_hold_ned = None
        self._ensure_offboard_if_armed()
        self.get_logger().info(f'🛬 开始垂直降落：Z 下降至设定降落高度 {land_z:.2f}m 后电机自动保护锁止')
        self._publish_flight_state()

    def _publish_flight_state(self):
        """定期发布 3D 起降与电机保护锁止状态（供 usv_status_node 与地面站同步）"""
        cur_enu_z = self._current_enu_z()
        is_armed = (
            self.vehicle_status is not None
            and int(self.vehicle_status.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        )
        if self.platform_mode == '2d' or self.vehicle_type in ('usv', 'usv_diff'):
            takeoff_ready = bool(is_armed)
        else:
            tol = self._takeoff_z_tolerance()
            z_ok = (cur_enu_z is not None) and (cur_enu_z >= max(0.2, float(self.takeoff_altitude) - tol))
            takeoff_ready = bool(is_armed and (not self._protection_locked) and z_ok and self._flight_action != 'land')
            self._takeoff_ready = takeoff_ready

        payload = {
            'flight_action': self._flight_action or 'idle',
            'protection_locked': bool(self._protection_locked),
            'takeoff_ready': bool(takeoff_ready),
            'takeoff_altitude': float(self.takeoff_altitude),
            'landing_altitude': float(self.landing_altitude),
            'current_z': round(float(cur_enu_z), 3) if cur_enu_z is not None else None,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.flight_state_pub.publish(msg)

    def _execute_vertical_flight_action(self):
        """执行 3D 垂直起飞或垂直降落（20Hz Offboard 定点垂直轨迹）"""
        if self.current_position is None or not bool(self.current_position.xy_valid):
            return

        cur_enu_z = self._current_enu_z()
        if cur_enu_z is None:
            return

        if self._flight_lock_ned_xy_yaw is None:
            cur_yaw = float(self.current_position.heading) if math.isfinite(float(self.current_position.heading)) else 0.0
            self._flight_lock_ned_xy_yaw = [float(self.current_position.x), float(self.current_position.y), cur_yaw]

        lock_x, lock_y, lock_yaw = self._flight_lock_ned_xy_yaw
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        if self._flight_action == 'land':
            land_z = float(self._flight_target_enu_z if self._flight_target_enu_z is not None else self.landing_altitude)
            # 一旦 Z 降到设定降落高度及以下，立即强制锁桨并锁存保护状态！
            if cur_enu_z <= land_z:
                self._flight_action = None
                self._protection_locked = True
                self._takeoff_ready = False
                self._idle_hold_ned = None
                self._send_force_disarm_command(f'到达降落高度 Z={cur_enu_z:.2f}m <= {land_z:.2f}m')
                self._publish_flight_state()
                return

            # 平滑垂直下降：目标高度略低于 land_z (land_z - 0.10m) 以确保干净利落穿过阈值触发锁桨
            step_dt = 1.0 / max(1.0, float(self.publish_rate))
            if not hasattr(self, '_land_ramp_enu_z') or self._land_ramp_enu_z is None:
                self._land_ramp_enu_z = cur_enu_z
            self._land_ramp_enu_z = max(land_z - 0.10, self._land_ramp_enu_z - self.vertical_descent_speed * step_dt)
            ned_target_z = self._enu_z_to_ned_z(self._land_ramp_enu_z)

            offboard_msg = OffboardControlMode()
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            offboard_msg.timestamp = now_us
            self.offboard_mode_pub.publish(offboard_msg)

            sp = TrajectorySetpoint()
            sp.position[0] = float(lock_x)
            sp.position[1] = float(lock_y)
            sp.position[2] = float(ned_target_z)
            sp.velocity[0] = float('nan')
            sp.velocity[1] = float('nan')
            sp.velocity[2] = float('nan')
            sp.acceleration[0] = float('nan')
            sp.acceleration[1] = float('nan')
            sp.acceleration[2] = float('nan')
            sp.yaw = float(lock_yaw)
            sp.yawspeed = float('nan')
            sp.timestamp = now_us
            self.setpoint_pub.publish(sp)
            return

        if self._flight_action == 'takeoff':
            self._land_ramp_enu_z = None
            target_z = float(self._flight_target_enu_z if self._flight_target_enu_z is not None else self.takeoff_altitude)
            ned_final_z = self._enu_z_to_ned_z(target_z)
            tol = self._takeoff_z_tolerance()

            if abs(cur_enu_z - target_z) <= tol:
                self._flight_action = None
                self._takeoff_ready = True
                self._idle_hold_ned = [float(lock_x), float(lock_y), float(ned_final_z), float(lock_yaw)]
                self.get_logger().info(
                    f'✅ 起飞到位：当前 Z={cur_enu_z:.2f}m (目标预高度={target_z:.2f}m)，允许启动集群路径任务！'
                )
                self._publish_flight_state()
                self._publish_idle_hold_setpoint()
                return

            offboard_msg = OffboardControlMode()
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            offboard_msg.timestamp = now_us
            self.offboard_mode_pub.publish(offboard_msg)

            sp = TrajectorySetpoint()
            sp.position[0] = float(lock_x)
            sp.position[1] = float(lock_y)
            sp.position[2] = float(ned_final_z)
            sp.velocity[0] = float('nan')
            sp.velocity[1] = float('nan')
            sp.velocity[2] = float('nan')
            sp.acceleration[0] = float('nan')
            sp.acceleration[1] = float('nan')
            sp.acceleration[2] = float('nan')
            sp.yaw = float(lock_yaw)
            sp.yawspeed = float('nan')
            sp.timestamp = now_us
            self.setpoint_pub.publish(sp)
            return

    def _ensure_offboard_if_armed(self):
        """当飞控已解锁但处于非 OFFBOARD（如 HOLD/STABILIZED）时，收到导航目标自动切回 OFFBOARD"""
        if self._protection_locked:
            return
        if self.vehicle_status is None:
            return
        is_armed = int(self.vehicle_status.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        if not is_armed or int(self.vehicle_status.nav_state) == 14:
            return

        target_sys = int(self.vehicle_status.system_id) if int(self.vehicle_status.system_id) > 0 else 0
        target_comp = int(self.vehicle_status.component_id) if int(self.vehicle_status.component_id) > 0 else 1
        cmd = VehicleCommand()
        cmd.command = 176  # VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        cmd.param3 = 0.0
        cmd.target_system = target_sys
        cmd.target_component = target_comp
        cmd.source_system = 255
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(cmd)
        self.get_logger().info(
            f'🔄 已解锁且收到新导航目标，自动切换至 OFFBOARD (target_system={target_sys})'
        )

    def target_callback(self, msg: PoseStamped):
        """
        目标点回调
        
        Args:
            msg (PoseStamped): 目标位置消息
        """
        if self._protection_locked:
            self.get_logger().warn('🔒 降落电机保护锁止中，已忽略目标点指令（请先手动点击 Arming 解锁）')
            return
        if self._flight_action == 'land':
            self.get_logger().warn('🛬 正在执行降落动作，已忽略新目标点指令')
            return
        self._flight_action = None
        self.target_position = msg
        self.target_active = True
        self._idle_hold_ned = None
        if self.current_goal_id is None:
            self.current_goal_id = 1
            self.goal_start_time = self.get_clock().now()
        self._integrated_yaw_ned = None
        self._ensure_offboard_if_armed()
        
        self.get_logger().info(
            f'📍 收到目标点: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
        )

    def rl_cmd_callback(self, msg: TwistStamped):
        """接收 RL 推理节点发布的机体速度与角速度 (linear.x, angular.z, ENU/FLU)"""
        self._rl_cmd_msg = msg
        self._rl_cmd_mono_time = time.monotonic()

    def nav_goal_callback(self, msg: NavigationGoal):
        """
        导航目标回调（新版接口，来自地面站或本地编队跟随节点）
        
        Args:
            msg (NavigationGoal): 导航目标消息
        """
        if self._protection_locked:
            self.get_logger().warn(
                f'🔒 降落电机保护锁止中，拒绝导航目标 [ID={msg.goal_id}]（请先手动点击 Arming 解锁）'
            )
            try:
                ack = NavigationAck()
                ack.goal_id = int(msg.goal_id)
                ack.accepted = False
                ack.execute_mask = 0
                ack.message = '电机处于降落保护锁止状态，需手动解锁'
                ack.timestamp = self.get_clock().now().to_msg()
                self.nav_ack_pub.publish(ack)
            except Exception:
                pass
            return
        if self._flight_action == 'land':
            self.get_logger().warn(f'🛬 正在执行降落动作，拒绝导航目标 [ID={msg.goal_id}]')
            return

        self._flight_action = None
        is_same_goal = (
            self.target_active
            and self.current_goal_id is not None
            and int(self.current_goal_id) == int(msg.goal_id)
        )
        is_continuous_follow = str(getattr(msg, 'task_name', '')).startswith('formation_follow_')

        # 提取 PoseStamped
        self.target_position = msg.target_pose
        self.target_active = True
        self._is_continuous_follow = is_continuous_follow
        self._idle_hold_ned = None
        
        # 记录目标信息
        self.current_goal_id = msg.goal_id
        self.goal_start_time = self.get_clock().now()
        self.goal_timeout = msg.timeout if msg.timeout > 0 else 300.0
        self.enable_yaw = bool(getattr(msg, 'enable_yaw', False))
        if not is_same_goal:
            self._integrated_yaw_ned = None
            self._ensure_offboard_if_armed()

        # 记录当前目标的到达判定阈值（GS 可按平台模式下发）
        try:
            xy_thr = float(getattr(msg, 'reach_xy_threshold', 0.0))
            z_thr = float(getattr(msg, 'reach_z_threshold', 0.0))
            self._goal_reach_xy_threshold = xy_thr if xy_thr > 0.0 else None
            self._goal_reach_z_threshold = z_thr if z_thr > 0.0 else None
        except Exception:
            self._goal_reach_xy_threshold = None
            self._goal_reach_z_threshold = None
        
        if not is_same_goal:
            pos = msg.target_pose.pose.position
            self.get_logger().info(
                f'🎯 收到导航目标 [ID={msg.goal_id}]: '
                f'({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), 超时={self.goal_timeout:.0f}s'
            )

            # 立即发送“收到/接受”应答（与完成结果解耦）
            try:
                execute_mask = int(NavigationAck.EXECUTE_POS) | int(NavigationAck.EXECUTE_YAW)
                if self.platform_mode == '3d' and self.use_setpoint_6dof:
                    execute_mask |= int(NavigationAck.EXECUTE_ROLL) | int(NavigationAck.EXECUTE_PITCH)

                ack = NavigationAck()
                ack.goal_id = int(msg.goal_id)
                ack.accepted = True
                ack.execute_mask = int(execute_mask)
                if self.platform_mode == '2d':
                    ack.message = '2D 平台：执行位置+yaw，忽略 roll/pitch'
                elif self.platform_mode == '3d' and not self.use_setpoint_6dof:
                    ack.message = '3D 平台：已收到目标；未启用 6DoF，下发 roll/pitch 将被忽略'
                else:
                    ack.message = '已收到目标，开始执行'
                ack.timestamp = self.get_clock().now().to_msg()
                self.nav_ack_pub.publish(ack)
            except Exception as e:
                self.get_logger().warn(f'发布 navigation/ack 失败: {e}')
        
        # 发送实时反馈（让 RL 推理节点与地面站实时获取最新距离与航向误差）
        distance = self._calculate_distance_to_target()
        self._send_nav_feedback(distance if distance else 0.0)

    def publish_nav_feedback(self):
        """
        定时发布导航反馈
        """
        if not self.target_active or self.current_goal_id is None:
            return
        
        # 计算到目标的距离（用于反馈/ETA；2D=水平距离，3D=欧氏距离）
        distance = self._calculate_distance_to_target()

        # 检查是否到达目标（编队持续跟随模式下不自动终结目标，由 formation_follower_node 持续驱动）
        if not getattr(self, '_is_continuous_follow', False) and self._is_target_reached():
            self._send_nav_result(True, '到达目标')
            self.target_active = False
            self.current_goal_id = None
            return
        
        # 检查超时
        if self.goal_start_time is not None:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if elapsed > self.goal_timeout:
                self._send_nav_result(False, '超时')
                self.target_active = False
                self.current_goal_id = None
                return
        
        # 发送执行中反馈
        self._send_nav_feedback(distance if distance else 0.0)

    def _calculate_xy_distance_and_z_error(self):
        """计算到目标的水平距离与高度误差（基于当前(px4)NED位置）。

        Returns:
            (xy_dist, z_err):
              - xy_dist: 水平距离(米)
              - z_err: 高度方向误差(米，NED Down 方向；仅用于绝对值判定)
            若数据不足返回 (None, None)
        """
        if self.current_position is None or self.target_position is None:
            return None, None

        target = self.target_position.pose.position

        # 当前(px4)为 NED。目标点可能是 NED(直接输入) 或 ENU(需要转换并扣除世界出生点偏移)。
        if self.coordinate_system == 'NED':
            dx = (float(target.x) - self.spawn_offset_y) - float(self.current_position.x)
            dy = (float(target.y) - self.spawn_offset_x) - float(self.current_position.y)
            dz = (float(target.z) + self.spawn_offset_z) - float(self.current_position.z)
        else:
            # ENU 输入，当前位置是 NED
            tx_enu = float(target.x) - self.spawn_offset_x
            ty_enu = float(target.y) - self.spawn_offset_y
            tz_enu = float(target.z) - self.spawn_offset_z
            dx = ty_enu - float(self.current_position.x)
            dy = tx_enu - float(self.current_position.y)
            dz = -tz_enu - float(self.current_position.z)

        return math.hypot(dx, dy), dz

    def _is_target_reached(self) -> bool:
        """到达判定：2D 用水平距离；3D 用 XY+Z 双阈值。"""
        if not self.target_active or self.current_goal_id is None:
            return False

        xy_dist, z_err = self._calculate_xy_distance_and_z_error()
        if xy_dist is None or z_err is None:
            return False

        if self.platform_mode == '2d':
            return float(xy_dist) < float(self.target_reach_threshold)

        # 3D：水平和高度分别收敛
        xy_thr = float(self._goal_reach_xy_threshold) if self._goal_reach_xy_threshold is not None else float(self.reach_xy_threshold)
        z_thr = float(self._goal_reach_z_threshold) if self._goal_reach_z_threshold is not None else float(self.reach_z_threshold)
        return (float(xy_dist) < xy_thr) and (abs(float(z_err)) < z_thr)

    def _send_nav_feedback(self, distance: float):
        """发送导航反馈"""
        if self.current_goal_id is None:
            return

        xy_distance = None
        z_err = None
        try:
            xy_distance, z_err = self._calculate_xy_distance_and_z_error()
        except Exception:
            xy_distance, z_err = None, None

        heading_error_deg = 0.0
        estimated_time = 0.0

        # 计算航向误差（度）
        # 优先：使用目标姿态里的 yaw（即 GS/任务下发的 yaw）
        # 回退：使用“朝向目标点”的方位角
        try:
            if self.current_position is not None and self.target_position is not None:
                cur_heading = float(self.current_position.heading)
                if math.isfinite(cur_heading):
                    target_yaw = None

                    # 1) 尝试从目标四元数中取 yaw
                    q = self.target_position.pose.orientation
                    if any(math.isfinite(float(v)) for v in (q.x, q.y, q.z, q.w)):
                        target_yaw = self._quaternion_to_yaw(q)
                        # ENU -> NED yaw 转换（与 publish_setpoint 保持一致）
                        if self.coordinate_system != 'NED':
                            target_yaw = math.pi / 2.0 - target_yaw

                    # 2) 若目标 yaw 不可用，回退到朝向目标点方位角（需扣除出生点偏移并对齐 NED）
                    if target_yaw is None or not math.isfinite(float(target_yaw)) or not self.enable_yaw:
                        target = self.target_position.pose.position
                        if self.coordinate_system == 'NED':
                            dx = (float(target.x) - self.spawn_offset_y) - float(self.current_position.x)
                            dy = (float(target.y) - self.spawn_offset_x) - float(self.current_position.y)
                        else:
                            tx_enu = float(target.x) - self.spawn_offset_x
                            ty_enu = float(target.y) - self.spawn_offset_y
                            dx = ty_enu - float(self.current_position.x)  # North
                            dy = tx_enu - float(self.current_position.y)  # East
                        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                            target_yaw = math.atan2(dy, dx)
                        else:
                            target_yaw = cur_heading

                    heading_error = float(target_yaw) - cur_heading
                    heading_error = (heading_error + math.pi) % (2.0 * math.pi) - math.pi
                    heading_error_deg = math.degrees(heading_error)

                # 简单的 ETA 估算：水平距离 / 水平速度
                # 说明：PX4 vx/vy 为水平速度分量，因此 ETA 用 xy_distance 更合理。
                eta_distance = xy_distance if (xy_distance is not None and math.isfinite(float(xy_distance))) else distance
                if eta_distance is not None and self.current_position is not None:
                    vx = float(self.current_position.vx)
                    vy = float(self.current_position.vy)
                    if math.isfinite(vx) and math.isfinite(vy):
                        speed = math.hypot(vx, vy)
                        if speed > 0.1 and math.isfinite(float(eta_distance)):
                            estimated_time = float(eta_distance) / speed
        except Exception:
            heading_error_deg = 0.0
            estimated_time = 0.0
        
        feedback = NavigationFeedback()
        feedback.goal_id = self.current_goal_id
        feedback.distance_to_goal = distance
        feedback.xy_distance_to_goal = float(xy_distance) if xy_distance is not None else float(distance)
        feedback.z_error = float(z_err) if z_err is not None else 0.0
        feedback.heading_error = float(heading_error_deg)
        feedback.estimated_time = float(estimated_time)
        feedback.timestamp = self.get_clock().now().to_msg()
        
        self.nav_feedback_pub.publish(feedback)

    def _quaternion_to_rpy(self, q):
        """从四元数提取 roll/pitch/yaw（弧度）。"""
        x = float(q.x)
        y = float(q.y)
        z = float(q.z)
        w = float(q.w)

        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """从 roll/pitch/yaw 生成四元数 (x,y,z,w)。"""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

    def _send_nav_result(self, success: bool, message: str):
        """发送导航结果"""
        if self.current_goal_id is None:
            return
        
        result = NavigationResult()
        result.goal_id = self.current_goal_id
        result.success = success
        # error_code: 0=成功, 1=超时, 2=取消, 3=其他错误
        if success:
            result.error_code = 0
        elif '超时' in message:
            result.error_code = 1
        elif '取消' in message:
            result.error_code = 2
        else:
            result.error_code = 3
        result.message = message
        result.timestamp = self.get_clock().now().to_msg()
        
        self.nav_result_pub.publish(result)
        
        status_str = '✅ 成功' if success else '❌ 失败'
        self.get_logger().info(f'{status_str} 导航目标 [ID={self.current_goal_id}]: {message}')

    def _calculate_distance_to_target(self) -> float:
        """计算到目标的距离"""
        if self.current_position is None or self.target_position is None:
            return None

        target = self.target_position.pose.position

        # 当前(px4)为 NED。目标点可能是 NED(直接输入) 或 ENU(需要转换并扣除世界出生点偏移)。
        if self.coordinate_system == 'NED':
            dx = (float(target.x) - self.spawn_offset_y) - float(self.current_position.x)
            dy = (float(target.y) - self.spawn_offset_x) - float(self.current_position.y)
            dz = (float(target.z) + self.spawn_offset_z) - float(self.current_position.z)
        else:
            # ENU 输入，当前位置是 NED
            tx_enu = float(target.x) - self.spawn_offset_x
            ty_enu = float(target.y) - self.spawn_offset_y
            tz_enu = float(target.z) - self.spawn_offset_z
            dx = ty_enu - float(self.current_position.x)
            dy = tx_enu - float(self.current_position.y)
            dz = -tz_enu - float(self.current_position.z)

        if self.platform_mode == '2d':
            return math.hypot(dx, dy)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def avoidance_target_callback(self, msg: PoseStamped):
        """
        避障目标点回调
        
        Args:
            msg (PoseStamped): 避障目标位置消息
        """
        self.avoidance_position = msg

    def avoidance_flag_callback(self, msg: Bool):
        """
        避障标志回调
        
        Args:
            msg (Bool): 避障模式标志
        """
        if msg.data != self.avoidance_active:
            self.avoidance_active = msg.data
            status = "激活" if msg.data else "停用"
            self.get_logger().info(f'🚧 避障模式 {status}')

    def clear_target_callback(self, msg: Bool):
        """
        清除目标点回调
        
        Args:
            msg (Bool): 清除标志
        """
        if msg.data:
            self.target_position = None
            self.avoidance_position = None
            self.target_active = False
            self._idle_hold_ned = None
            self.get_logger().info('🗑️ 目标点已清除')

    def _publish_idle_hold_setpoint(self):
        """
        空闲状态持续发布 Offboard 保活与驻留设定值（20Hz）：
        - 保证 PX4 始终收到有效 Offboard 信号，杜绝 `[FCU-EVENT] No offboard signal`
        - 允许地面站随时一键切换 OFFBOARD / 一键解锁
        - 到达目标或清除目标后平滑驻留在当前锚点（差速船发送零速驻留，3D 载体锁定当前悬停锚点）
        """
        if self.current_position is None or not bool(self.current_position.xy_valid):
            return

        is_armed = (
            self.vehicle_status is not None
            and int(self.vehicle_status.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        )
        cur_x = float(self.current_position.x)
        cur_y = float(self.current_position.y)
        cur_z = float(self.current_position.z) if bool(self.current_position.z_valid) else 0.0
        cur_yaw = float(self.current_position.heading) if math.isfinite(float(self.current_position.heading)) else 0.0

        # 未解锁时实时跟随载体真实位置；已解锁进入空闲瞬间锁存悬停锚点防止风流漂移
        if (not is_armed) or (self._idle_hold_ned is None):
            hold_z = 0.0 if self.platform_mode == '2d' else cur_z
            self._idle_hold_ned = [cur_x, cur_y, hold_z, cur_yaw]

        now_us = int(self.get_clock().now().nanoseconds / 1000)
        is_2d_rover = (self.vehicle_type in ('usv', 'usv_diff')) or (self.platform_mode == '2d')

        offboard_msg = OffboardControlMode()
        if is_2d_rover:
            offboard_msg.position = False
            offboard_msg.velocity = True
        else:
            offboard_msg.position = True
            offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = now_us
        self.offboard_mode_pub.publish(offboard_msg)

        sp = TrajectorySetpoint()
        if is_2d_rover:
            sp.position[0] = float('nan')
            sp.position[1] = float('nan')
            sp.position[2] = float('nan')
            sp.velocity[0] = 0.0
            sp.velocity[1] = 0.0
            sp.velocity[2] = 0.0
            sp.yaw = float('nan')
            sp.yawspeed = 0.0
        else:
            sp.position[0] = float(self._idle_hold_ned[0])
            sp.position[1] = float(self._idle_hold_ned[1])
            sp.position[2] = float(self._idle_hold_ned[2])
            sp.velocity[0] = float('nan')
            sp.velocity[1] = float('nan')
            sp.velocity[2] = float('nan')
            sp.yaw = float(self._idle_hold_ned[3])
            sp.yawspeed = float('nan')

        sp.acceleration[0] = float('nan')
        sp.acceleration[1] = float('nan')
        sp.acceleration[2] = float('nan')
        sp.timestamp = now_us
        self.setpoint_pub.publish(sp)

    def publish_setpoint(self):
        """
        发布目标点到 PX4
        
        将选定的目标点（常规或避障）转换为 PX4 TrajectorySetpoint 格式并发布。
        同时发布 OffboardControlMode 以保持 OFFBOARD 模式。
        """
        # 降落电机保护锁止状态：仅维持空闲零速/当前点保活，严禁执行任何机动或攀升
        if self._protection_locked:
            self.target_active = False
            self._flight_action = None
            self._publish_idle_hold_setpoint()
            return

        # 3D 垂直起飞 / 降落状态机最高优先级（屏蔽 RL 水平策略输出，保证纯 Z 向垂直动作）
        if self._flight_action in ('takeoff', 'land'):
            self._execute_vertical_flight_action()
            return

        # 如果既没有导航目标也没有姿态动作指令，持续发送空闲驻留设定值保活 Offboard
        if (not self.target_active) and (not self._att_cmd_active):
            self._publish_idle_hold_setpoint()
            return

        self._idle_hold_ned = None

        # 姿态动作过期判定
        if self._att_cmd_active and self._att_cmd_end_time is not None:
            try:
                if self.get_clock().now() >= self._att_cmd_end_time:
                    self._att_cmd_active = False
                    self._att_cmd_end_time = None
            except Exception:
                # 保守处理：异常时不让它一直卡住
                self._att_cmd_active = False
                self._att_cmd_end_time = None

        # 选择目标点（避障优先）
        target = None
        if self.target_active:
            target = self.avoidance_position if self.avoidance_active and self.avoidance_position else self.target_position

        # 判断是否使用 RL (v, omega) Offboard 速度模式
        rl_fresh = (
            self.nav_mode == 'rl'
            and self.target_active
            and target is not None
            and (not self._att_cmd_active)
            and self._rl_cmd_msg is not None
            and (time.monotonic() - self._rl_cmd_mono_time) <= self.rl_cmd_timeout
            and self.current_position is not None
        )

        if rl_fresh:
            cur_heading_ned = float(self.current_position.heading) if math.isfinite(float(self.current_position.heading)) else 0.0
            v_fwd = float(self._rl_cmd_msg.twist.linear.x)
            omega_enu = float(self._rl_cmd_msg.twist.angular.z)  # FLU/ENU 左转为正
            yawspeed_ned = -omega_enu                            # FRD/NED 右转为正

            # 将前向线速度投影到 NED 水平面
            vn = v_fwd * math.cos(cur_heading_ned)
            ve = v_fwd * math.sin(cur_heading_ned)

            # 3D 载体叠加目标高度跟踪速度 vd (NED Down)，2D 差速船 vd=0
            vd = 0.0
            if self.platform_mode == '3d':
                _, z_err = self._calculate_xy_distance_and_z_error()
                if z_err is not None and math.isfinite(float(z_err)):
                    vd = max(-self.rl_z_max_vel, min(self.rl_z_max_vel, self.rl_z_kp * float(z_err)))

            offboard_msg = OffboardControlMode()
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_mode_pub.publish(offboard_msg)

            sp = TrajectorySetpoint()
            sp.position[0] = float('nan')
            sp.position[1] = float('nan')
            sp.position[2] = float('nan')
            sp.velocity[0] = float(vn)
            sp.velocity[1] = float(ve)
            sp.velocity[2] = float(vd)
            sp.acceleration[0] = float('nan')
            sp.acceleration[1] = float('nan')
            sp.acceleration[2] = float('nan')

            if self.vehicle_type in ('usv', 'usv_diff') or self.platform_mode == '2d':
                # PX4 RoverDifferential 使用 yawspeed 直控偏航角速度
                sp.yaw = float('nan')
                sp.yawspeed = float(yawspeed_ned)
            else:
                # PX4 多旋翼 mc_pos_control 需有限 yaw 配合 yawspeed 前馈以获得精确转向响应
                dt = 1.0 / max(1.0, float(self.publish_rate))
                lead_yaw = (cur_heading_ned + yawspeed_ned * max(dt * 4.0, 0.25) + math.pi) % (2.0 * math.pi) - math.pi
                sp.yaw = float(lead_yaw)
                sp.yawspeed = float(yawspeed_ned)

            sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.setpoint_pub.publish(sp)

            # 发布闭环 final_cmd 供 policy_inference_node 自洽跟踪
            fc_msg = TwistStamped()
            fc_msg.header.stamp = self.get_clock().now().to_msg()
            fc_msg.header.frame_id = 'base_link'
            fc_msg.twist.linear.x = float(v_fwd)
            fc_msg.twist.linear.z = float(-vd)
            fc_msg.twist.angular.z = float(omega_enu)
            self.final_cmd_pub.publish(fc_msg)
            return

        # 姿态动作“叠加”策略：
        # - 有导航目标：继续按导航目标产生位置 setpoint（允许产生位移），同时叠加 roll/pitch（可选 yaw）
        # - 无导航目标：不做“保持当前位置”，仅发布姿态(6DoF quaternion)，位置字段置 NaN
        attitude_enabled = bool((self.use_setpoint_6dof or self._att_cmd_active) and self.platform_mode == '3d')
        attitude_only = bool(self._att_cmd_active and target is None)
        attitude_velocity_mode = bool(
            self._att_cmd_active
            and (target is not None)
            and (self.platform_mode == '3d')
            and attitude_enabled
            and (self.attitude_action_motion_mode == 'attitude_velocity')
        )

        # =====================================================================
        # 发布 OffboardControlMode（必须持续发送以保持 OFFBOARD 模式）
        # =====================================================================
        offboard_msg = OffboardControlMode()
        # 无导航目标且仅姿态动作时：不启用 position 控制，避免“锁在当前位置”
        offboard_msg.position = bool((target is not None) and (not attitude_velocity_mode))
        offboard_msg.velocity = bool(attitude_velocity_mode)
        offboard_msg.acceleration = False
        # 当发布 6DoF（含姿态）setpoint 时，开启 attitude 标志，避免 PX4 侧忽略姿态部分
        offboard_msg.attitude = bool(attitude_enabled)
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)

        # =====================================================================
        # 姿态动作 + 导航目标：切到 velocity 控制，位移由速度承载（更确定地避免位置控制器覆盖 roll/pitch）
        # =====================================================================
        if attitude_velocity_mode:
            if self.current_position is None:
                return

            # 目标点转 NED
            if self.coordinate_system == 'NED':
                tgt_x = float(target.pose.position.x)
                tgt_y = float(target.pose.position.y)
                tgt_z = float(target.pose.position.z)
            else:
                tgt_x = float(target.pose.position.y)
                tgt_y = float(target.pose.position.x)
                tgt_z = -float(target.pose.position.z)

            cur_x = float(self.current_position.x)
            cur_y = float(self.current_position.y)
            cur_z = float(self.current_position.z)

            dx = tgt_x - cur_x
            dy = tgt_y - cur_y
            dz = tgt_z - cur_z

            xy_dist = math.hypot(dx, dy)
            z_err = abs(dz)
            if self.platform_mode == '2d':
                dz = 0.0
                z_err = 0.0

            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            # yaw 基准：来自导航目标姿态；姿态动作可选覆盖 yaw
            roll, pitch, yaw = 0.0, 0.0, 0.0
            try:
                roll, pitch, yaw = self._quaternion_to_rpy(target.pose.orientation)
            except Exception:
                yaw = self._quaternion_to_yaw(target.pose.orientation)

            roll = float(self._att_cmd_roll)
            pitch = float(self._att_cmd_pitch)
            if math.isfinite(float(self._att_cmd_yaw)):
                yaw = float(self._att_cmd_yaw)

            if self.coordinate_system != 'NED':
                yaw = math.pi / 2.0 - yaw

            # 到达阈值内：速度置 0
            if (xy_dist <= float(self.reach_xy_threshold)) and (z_err <= float(self.reach_z_threshold)):
                vx = 0.0
                vy = 0.0
                vz = 0.0
            else:
                if dist < 1e-6:
                    vx = 0.0
                    vy = 0.0
                    vz = 0.0
                else:
                    # 幅值随距离增长并限幅
                    speed = min(float(self.max_velocity), max(float(self.attitude_action_min_velocity), 0.8 * dist))

                    # 方向选择：朝目标(to_goal) 或 沿机体前向(body_forward)
                    if self.attitude_action_velocity_direction == 'body_forward':
                        # PX4 Body FRD 中前向为 X；在 NED 中的单位前向向量由 yaw/pitch 决定。
                        # 说明：roll 不影响前向方向；pitch>0(抬头) 会让 NED 的 down 分量为负。
                        fx = math.cos(float(pitch)) * math.cos(float(yaw))
                        fy = math.cos(float(pitch)) * math.sin(float(yaw))
                        fz = -math.sin(float(pitch))
                        vx = speed * fx
                        vy = speed * fy
                        vz = speed * fz
                    else:
                        vx = speed * dx / dist
                        vy = speed * dy / dist
                        vz = speed * dz / dist

            # 发送 TrajectorySetpoint（velocity）
            sp = TrajectorySetpoint()
            sp.position[0] = float('nan')
            sp.position[1] = float('nan')
            sp.position[2] = float('nan')
            sp.velocity[0] = float(vx)
            sp.velocity[1] = float(vy)
            sp.velocity[2] = float(vz)
            sp.acceleration[0] = float('nan')
            sp.acceleration[1] = float('nan')
            sp.acceleration[2] = float('nan')
            sp.yaw = float(yaw)
            sp.yawspeed = float('nan')
            sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.setpoint_pub.publish(sp)

            # 同时发送 6DoF quaternion（并携带同样的 velocity，便于 PX4 端选择性使用）
            sp6 = TrajectorySetpoint6dof()
            sp6.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            sp6.position[0] = float('nan')
            sp6.position[1] = float('nan')
            sp6.position[2] = float('nan')
            sp6.velocity[0] = float(vx)
            sp6.velocity[1] = float(vy)
            sp6.velocity[2] = float(vz)
            sp6.acceleration[0] = float('nan')
            sp6.acceleration[1] = float('nan')
            sp6.acceleration[2] = float('nan')
            sp6.jerk[0] = float('nan')
            sp6.jerk[1] = float('nan')
            sp6.jerk[2] = float('nan')
            sp6.angular_velocity[0] = float('nan')
            sp6.angular_velocity[1] = float('nan')
            sp6.angular_velocity[2] = float('nan')

            qx, qy, qz, qw = self._rpy_to_quaternion(float(roll), float(pitch), float(yaw))
            sp6.quaternion[0] = qx
            sp6.quaternion[1] = qy
            sp6.quaternion[2] = qz
            sp6.quaternion[3] = qw
            self.setpoint6dof_pub.publish(sp6)
            return

        # =====================================================================
        # 仅姿态动作（无导航目标）：只发布 6DoF quaternion，不发布位置 setpoint
        # =====================================================================
        if attitude_only:
            # 2D 平台不支持 roll/pitch：直接忽略该姿态动作（不产生“定点保持”的副作用）
            if not attitude_enabled:
                self._att_cmd_active = False
                self._att_cmd_end_time = None
                return

            # yaw 基准：优先用当前 heading（可被指令覆盖）
            yaw = 0.0
            try:
                if self.current_position is not None and math.isfinite(float(self.current_position.heading)):
                    yaw = float(self.current_position.heading)
            except Exception:
                yaw = 0.0

            roll = float(self._att_cmd_roll)
            pitch = float(self._att_cmd_pitch)
            if math.isfinite(float(self._att_cmd_yaw)):
                yaw = float(self._att_cmd_yaw)

            # ENU 到 NED 偏航角转换（roll/pitch 不做隐式转换）
            if self.coordinate_system != 'NED':
                yaw = math.pi / 2.0 - yaw

            sp6 = TrajectorySetpoint6dof()
            sp6.timestamp = int(self.get_clock().now().nanoseconds / 1000)

            # 不控制位置：置 NaN
            sp6.position[0] = float('nan')
            sp6.position[1] = float('nan')
            sp6.position[2] = float('nan')

            # 其余量设为 NaN（由 PX4/控制器选择性使用）
            sp6.velocity[0] = float('nan')
            sp6.velocity[1] = float('nan')
            sp6.velocity[2] = float('nan')
            sp6.acceleration[0] = float('nan')
            sp6.acceleration[1] = float('nan')
            sp6.acceleration[2] = float('nan')
            sp6.jerk[0] = float('nan')
            sp6.jerk[1] = float('nan')
            sp6.jerk[2] = float('nan')
            sp6.angular_velocity[0] = float('nan')
            sp6.angular_velocity[1] = float('nan')
            sp6.angular_velocity[2] = float('nan')

            qx, qy, qz, qw = self._rpy_to_quaternion(float(roll), float(pitch), float(yaw))
            sp6.quaternion[0] = qx
            sp6.quaternion[1] = qy
            sp6.quaternion[2] = qz
            sp6.quaternion[3] = qw

            self.setpoint6dof_pub.publish(sp6)
            return

        # =====================================================================
        # 发布 TrajectorySetpoint
        # =====================================================================
        setpoint = TrajectorySetpoint()

        # 坐标转换：ROS 使用 ENU，PX4 使用 NED（扣除自身世界出生点偏移）
        # ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
        if target is not None:
            if self.coordinate_system == 'NED':
                # 如果输入已经是 NED，直接使用（扣除偏移）
                setpoint.position[0] = target.pose.position.x - self.spawn_offset_y  # North
                setpoint.position[1] = target.pose.position.y - self.spawn_offset_x  # East
                setpoint.position[2] = target.pose.position.z + self.spawn_offset_z  # Down
            else:
                # ENU 到 NED 转换（扣除 ENU 出生点偏移）
                tx_enu = float(target.pose.position.x) - self.spawn_offset_x
                ty_enu = float(target.pose.position.y) - self.spawn_offset_y
                tz_enu = float(target.pose.position.z) - self.spawn_offset_z
                setpoint.position[0] = ty_enu   # North = East_enu
                setpoint.position[1] = tx_enu   # East = North_enu
                setpoint.position[2] = -tz_enu  # Down = -Up_enu
        else:
            # 理论上不会到这里：target is None 且 attitude_only 已提前 return
            if self.current_position is None:
                return
            setpoint.position[0] = float(self.current_position.x)
            setpoint.position[1] = float(self.current_position.y)
            setpoint.position[2] = float(self.current_position.z)

        # 2D 平台：不下发高度，强制 z=0
        if self.platform_mode == '2d':
            setpoint.position[2] = 0.0
        
        # 速度设为 NaN（使用位置控制）
        setpoint.velocity[0] = float('nan')
        setpoint.velocity[1] = float('nan')
        setpoint.velocity[2] = float('nan')
        
        # 加速度设为 NaN
        setpoint.acceleration[0] = float('nan')
        setpoint.acceleration[1] = float('nan')
        setpoint.acceleration[2] = float('nan')
        
        # 从四元数计算目标姿态（roll/pitch/yaw）
        # 说明：
        # - 2D 平台：忽略 roll/pitch，仅保留 yaw
        # - 3D 平台：允许 roll/pitch/yaw（是否下发 6DoF 由 use_setpoint_6dof 控制）
        # 默认：来自导航目标姿态；若仅姿态动作则回退为当前 heading
        roll, pitch, yaw = 0.0, 0.0, 0.0
        if target is not None:
            try:
                roll, pitch, yaw = self._quaternion_to_rpy(target.pose.orientation)
            except Exception:
                roll, pitch, yaw = 0.0, 0.0, self._quaternion_to_yaw(target.pose.orientation)
        else:
            try:
                if self.current_position is not None and math.isfinite(float(self.current_position.heading)):
                    yaw = float(self.current_position.heading)
            except Exception:
                yaw = 0.0

        # 姿态动作指令：覆盖 roll/pitch，可选覆盖 yaw
        if self._att_cmd_active and self.platform_mode == '3d':
            roll = float(self._att_cmd_roll)
            pitch = float(self._att_cmd_pitch)
            if math.isfinite(float(self._att_cmd_yaw)):
                yaw = float(self._att_cmd_yaw)

        if self.platform_mode == '2d' or self.vehicle_type == 'flying_sphere':
            # 2D 平台与飞行球严禁翻滚和俯仰
            roll = 0.0
            pitch = 0.0

        # 当未显式启用目标航向约束 (not enable_yaw) 且无姿态动作覆盖时：
        # 远距离自动对准目标水平方位角（消除全系载具无横向推进器时的锁头漂移），近距离锁定当前航向防止近点打转
        if (not self.enable_yaw) and (not self._att_cmd_active) and (target is not None) and (self.current_position is not None):
            xy_dist, _ = self._calculate_xy_distance_and_z_error()
            xy_thr = float(self._goal_reach_xy_threshold) if self._goal_reach_xy_threshold is not None else float(self.reach_xy_threshold)
            dx_ned = float(setpoint.position[0]) - float(self.current_position.x)
            dy_ned = float(setpoint.position[1]) - float(self.current_position.y)
            if xy_dist is not None and xy_dist >= max(xy_thr, 0.35) and (abs(dx_ned) > 1e-4 or abs(dy_ned) > 1e-4):
                yaw_ned_auto = math.atan2(dy_ned, dx_ned)
                yaw = (math.pi / 2.0 - yaw_ned_auto) if self.coordinate_system != 'NED' else yaw_ned_auto
            elif math.isfinite(float(self.current_position.heading)):
                cur_h = float(self.current_position.heading)
                yaw = (math.pi / 2.0 - cur_h) if self.coordinate_system != 'NED' else cur_h
        
        # ENU 到 NED 偏航角转换
        if self.coordinate_system != 'NED':
            # ENU yaw: 0 = East, 增加逆时针
            # NED yaw: 0 = North, 增加顺时针
            yaw = math.pi / 2.0 - yaw

            # roll/pitch 的 ENU->NED 映射在不同约定下容易出错；这里不做隐式转换。
            # 如需要在 ENU 输入下执行 3D roll/pitch，请在边界层统一坐标系约定。
            if (not self._warned_enu_rpy) and self.platform_mode == '3d' and self.use_setpoint_6dof:
                self.get_logger().warn(
                    'platform_mode=3d 且 use_setpoint_6dof=true，但 coordinate_system=ENU：当前实现仅转换 yaw，未转换 roll/pitch。'
                )
                self._warned_enu_rpy = True
        
        setpoint.yaw = yaw
        setpoint.yawspeed = float('nan')  # 使用偏航角控制
        
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.setpoint_pub.publish(setpoint)

        # =====================================================================
        # 可选：发布 6DoF setpoint（仅当存在非零 roll/pitch 或显式姿态动作时发布，避免无意义覆盖飞控航向引导）
        # =====================================================================
        has_nonzero_rp = (abs(float(roll)) > 1e-4) or (abs(float(pitch)) > 1e-4)
        if (
            (TrajectorySetpoint6dof is not None)
            and (self.setpoint6dof_pub is not None)
            and (self._att_cmd_active or (self.use_setpoint_6dof and has_nonzero_rp))
            and self.platform_mode == '3d'
        ):
            sp6 = TrajectorySetpoint6dof()
            sp6.timestamp = int(self.get_clock().now().nanoseconds / 1000)

            sp6.position[0] = setpoint.position[0]
            sp6.position[1] = setpoint.position[1]
            sp6.position[2] = setpoint.position[2]

            # 其余量设为 NaN（由 PX4/控制器选择性使用）
            sp6.velocity[0] = float('nan')
            sp6.velocity[1] = float('nan')
            sp6.velocity[2] = float('nan')
            sp6.acceleration[0] = float('nan')
            sp6.acceleration[1] = float('nan')
            sp6.acceleration[2] = float('nan')
            sp6.jerk[0] = float('nan')
            sp6.jerk[1] = float('nan')
            sp6.jerk[2] = float('nan')
            sp6.angular_velocity[0] = float('nan')
            sp6.angular_velocity[1] = float('nan')
            sp6.angular_velocity[2] = float('nan')

            qx, qy, qz, qw = self._rpy_to_quaternion(float(roll), float(pitch), float(yaw))
            sp6.quaternion[0] = qx
            sp6.quaternion[1] = qy
            sp6.quaternion[2] = qz
            sp6.quaternion[3] = qw

            self.setpoint6dof_pub.publish(sp6)

    def _quaternion_to_yaw(self, q) -> float:
        """
        从四元数提取偏航角
        
        Args:
            q: 四元数 (geometry_msgs/Quaternion)
            
        Returns:
            float: 偏航角（弧度）
        """
        # 使用 atan2 计算偏航角
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_distance_to_target(self) -> float:
        """
        计算当前位置到目标点的距离
        
        Returns:
            float: 距离（米），如果数据无效返回 -1
        """
        if self.current_position is None or self.target_position is None:
            return -1.0
            
        target = self.avoidance_position if self.avoidance_active else self.target_position
        if target is None:
            return -1.0
        
        # 转换坐标系
        if self.coordinate_system == 'NED':
            dx = (target.pose.position.x - self.spawn_offset_y) - self.current_position.x
            dy = (target.pose.position.y - self.spawn_offset_x) - self.current_position.y
            dz = (target.pose.position.z + self.spawn_offset_z) - self.current_position.z
        else:
            # ENU 输入，当前位置是 NED
            tx_enu = target.pose.position.x - self.spawn_offset_x
            ty_enu = target.pose.position.y - self.spawn_offset_y
            tz_enu = target.pose.position.z - self.spawn_offset_z
            dx = ty_enu - self.current_position.x
            dy = tx_enu - self.current_position.y
            dz = -tz_enu - self.current_position.z
        
        if self.platform_mode == '2d':
            return math.hypot(dx, dy)
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def is_target_reached(self) -> bool:
        """
        检查是否到达目标点
        
        Returns:
            bool: 是否到达目标点
        """
        distance = self.get_distance_to_target()
        if distance < 0:
            return False
        return distance < self.target_reach_threshold


def main(args=None):
    """节点主函数"""
    rclpy.init(args=args)
    node = UsvControlPx4Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
