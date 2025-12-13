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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# PX4 消息类型
from px4_msgs.msg import (
    TrajectorySetpoint,
    TrajectorySetpoint6dof,
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
)

# 自定义接口（地面站导航目标）
from common_interfaces.msg import NavigationGoal, NavigationFeedback, NavigationResult, NavigationAck

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
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('coordinate_system', 'NED')  # NED 或 ENU

        # 平台执行模式：
        # - '3d'：允许姿态(roll/pitch/yaw)目标（是否真正下发给 PX4 由 use_setpoint_6dof 控制）
        # - '2d'：忽略 roll/pitch，只保留位置 + yaw
        self.declare_parameter('platform_mode', '3d')
        # 是否发布 TrajectorySetpoint6dof（位置+quaternion）。默认关闭以保持现有行为。
        self.declare_parameter('use_setpoint_6dof', False)
        
        self.target_reach_threshold = self.get_parameter('target_reach_threshold').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.coordinate_system = self.get_parameter('coordinate_system').value

        self.platform_mode = str(self.get_parameter('platform_mode').value).strip().lower()
        if self.platform_mode not in ('2d', '3d'):
            self.get_logger().warn(
                f"platform_mode='{self.platform_mode}' 非法，回退为 '3d'"
            )
            self.platform_mode = '3d'

        raw_6dof = self.get_parameter('use_setpoint_6dof').value
        if isinstance(raw_6dof, str):
            self.use_setpoint_6dof = raw_6dof.strip().lower() in ('1', 'true', 'yes', 'y', 'on')
        else:
            self.use_setpoint_6dof = bool(raw_6dof)

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
        # 发布器 - 发送目标点到 PX4
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

        # 可选：6DoF setpoint（位置 + quaternion）
        self.setpoint6dof_pub = self.create_publisher(
            TrajectorySetpoint6dof,
            'fmu/in/trajectory_setpoint6dof',
            qos_px4
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
        
        # 新版导航目标订阅（NavigationGoal 消息类型）
        self.nav_goal_sub = self.create_subscription(
            NavigationGoal,
            'navigation/goal',
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
        
        # 清除目标订阅（新版导航接口）
        self.nav_clear_sub = self.create_subscription(
            Bool,
            'navigation/clear_target',
            self.clear_target_callback,
            qos_reliable
        )

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

        # =====================================================================
        # 定时器
        # =====================================================================
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_setpoint)
        
        # 导航反馈定时器（1Hz）
        self.feedback_timer = self.create_timer(1.0, self.publish_nav_feedback)

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
        self.get_logger().info('📤 发布话题: fmu/in/trajectory_setpoint')
        if self.use_setpoint_6dof:
            self.get_logger().info('📤 发布话题: fmu/in/trajectory_setpoint6dof')
        self.get_logger().info('📥 订阅话题: fmu/out/vehicle_local_position')
        self.get_logger().info('📥 订阅话题: navigation/goal (NavigationGoal)')
        self.get_logger().info('=' * 60)

        # 只打印一次的提醒
        self._warned_enu_rpy = False

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
        self.vehicle_status = msg
        # nav_state == 14 表示 OFFBOARD 模式
        self.offboard_mode_active = (msg.nav_state == 14)

    def target_callback(self, msg: PoseStamped):
        """
        目标点回调
        
        Args:
            msg (PoseStamped): 目标位置消息
        """
        self.target_position = msg
        self.target_active = True
        
        self.get_logger().info(
            f'📍 收到目标点: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
        )

    def nav_goal_callback(self, msg: NavigationGoal):
        """
        导航目标回调（新版接口，来自地面站）
        
        Args:
            msg (NavigationGoal): 导航目标消息
        """
        # 提取 PoseStamped
        self.target_position = msg.target_pose
        self.target_active = True
        
        # 记录目标信息
        self.current_goal_id = msg.goal_id
        self.goal_start_time = self.get_clock().now()
        self.goal_timeout = msg.timeout if msg.timeout > 0 else 300.0
        
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
        
        # 发送初始反馈（距离待计算）
        distance = self._calculate_distance_to_target()
        self._send_nav_feedback(distance if distance else 0.0)

    def publish_nav_feedback(self):
        """
        定时发布导航反馈
        """
        if not self.target_active or self.current_goal_id is None:
            return
        
        # 计算到目标的距离
        distance = self._calculate_distance_to_target()
        
        # 检查是否到达目标
        if distance is not None and distance < self.target_reach_threshold:
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

    def _send_nav_feedback(self, distance: float):
        """发送导航反馈"""
        if self.current_goal_id is None:
            return

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

                    # 2) 若目标 yaw 不可用，回退到朝向目标点方位角
                    if target_yaw is None or not math.isfinite(float(target_yaw)):
                        target = self.target_position.pose.position
                        dx = float(target.x) - float(self.current_position.x)  # North
                        dy = float(target.y) - float(self.current_position.y)  # East
                        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                            target_yaw = math.atan2(dy, dx)
                        else:
                            target_yaw = cur_heading

                    heading_error = float(target_yaw) - cur_heading
                    heading_error = (heading_error + math.pi) % (2.0 * math.pi) - math.pi
                    heading_error_deg = math.degrees(heading_error)

                # 简单的 ETA 估算：距离 / 水平速度
                if distance is not None and self.current_position is not None:
                    vx = float(self.current_position.vx)
                    vy = float(self.current_position.vy)
                    if math.isfinite(vx) and math.isfinite(vy):
                        speed = math.hypot(vx, vy)
                        if speed > 0.1 and math.isfinite(float(distance)):
                            estimated_time = float(distance) / speed
        except Exception:
            heading_error_deg = 0.0
            estimated_time = 0.0
        
        feedback = NavigationFeedback()
        feedback.goal_id = self.current_goal_id
        feedback.distance_to_goal = distance
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
        dx = self.current_position.x - target.x
        dy = self.current_position.y - target.y
        dz = self.current_position.z - target.z
        
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
            self.get_logger().info('🗑️ 目标点已清除')

    def publish_setpoint(self):
        """
        发布目标点到 PX4
        
        将选定的目标点（常规或避障）转换为 PX4 TrajectorySetpoint 格式并发布。
        同时发布 OffboardControlMode 以保持 OFFBOARD 模式。
        """
        # 如果没有目标点，不发布
        if not self.target_active:
            return
            
        # 选择目标点（避障优先）
        target = self.avoidance_position if self.avoidance_active and self.avoidance_position else self.target_position
        
        if target is None:
            return

        # =====================================================================
        # 发布 OffboardControlMode（必须持续发送以保持 OFFBOARD 模式）
        # =====================================================================
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)

        # =====================================================================
        # 发布 TrajectorySetpoint
        # =====================================================================
        setpoint = TrajectorySetpoint()
        
        # 坐标转换：ROS 使用 ENU，PX4 使用 NED
        # ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
        if self.coordinate_system == 'NED':
            # 如果输入已经是 NED，直接使用
            setpoint.position[0] = target.pose.position.x  # North
            setpoint.position[1] = target.pose.position.y  # East
            setpoint.position[2] = target.pose.position.z  # Down
        else:
            # ENU 到 NED 转换
            setpoint.position[0] = target.pose.position.y   # North = East_enu
            setpoint.position[1] = target.pose.position.x   # East = North_enu
            setpoint.position[2] = -target.pose.position.z  # Down = -Up_enu
        
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
        try:
            roll, pitch, yaw = self._quaternion_to_rpy(target.pose.orientation)
        except Exception:
            roll, pitch, yaw = 0.0, 0.0, self._quaternion_to_yaw(target.pose.orientation)

        if self.platform_mode == '2d':
            roll = 0.0
            pitch = 0.0
        
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
        # 可选：发布 6DoF setpoint（位置 + quaternion）
        # =====================================================================
        if self.use_setpoint_6dof and self.platform_mode == '3d':
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
            dx = target.pose.position.x - self.current_position.x
            dy = target.pose.position.y - self.current_position.y
            dz = target.pose.position.z - self.current_position.z
        else:
            # ENU 输入，当前位置是 NED
            dx = target.pose.position.y - self.current_position.x
            dy = target.pose.position.x - self.current_position.y
            dz = -target.pose.position.z - self.current_position.z
        
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
