"""
无人船控制节点

该节点负责处理无人船的目标点控制逻辑。它订阅常规目标点和避障目标点，
根据避障标志决定使用哪个目标点，并将选定的目标点发布给飞控系统。
"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget, HomePosition, GlobalPositionTarget
from std_msgs.msg import Bool
from common_interfaces.msg import NavigationGoal
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# 导入common_utils工具
from common_utils import ParamLoader, GeoUtils


class UsvControlNode(Node):
    """
    无人船控制节点类
    
    该节点实现目标点控制逻辑，处理常规目标点和避障目标点，
    根据避障标志决定使用哪个目标点，并将选定的目标点发布给飞控系统。
    """

    def __init__(self):
        """初始化无人船控制节点"""
        super().__init__('usv_control_node')

        # 创建参数加载器
        param_loader = ParamLoader(self)
        
        # 声明参数
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('frame_id', 'map')
        
        # 控制模式: 'position' (发送GPS坐标) 或 'velocity' (由 velocity_controller_node 处理)
        # 当设置为 'velocity' 时，本节点仅处理特殊机动（如旋转），常规导航由速度控制器处理
        self.declare_parameter('control_mode', 'position')
        
        # 日志控制：避免发布循环刷屏
        self.declare_parameter('log_publish_setpoint', True)
        self.declare_parameter('log_publish_setpoint_on_change_only', True)
        self.declare_parameter('log_publish_setpoint_throttle_sec', 2.0)
        self.declare_parameter('log_publish_setpoint_epsilon', 0.01)
        # 默认不发送Z轴高度（水面船）
        
        # 获取参数值
        publish_rate_param = self.get_parameter('publish_rate').value
        publish_rate = 20.0 if publish_rate_param is None else float(publish_rate_param)
        self.frame_id = self.get_parameter('frame_id').value
        
        # 控制模式
        self.control_mode = self.get_parameter('control_mode').value
        if self.control_mode not in ['position', 'velocity']:
            self.get_logger().warn(f'未知控制模式 {self.control_mode}，使用默认 position 模式')
            self.control_mode = 'position'

        # 日志参数
        self._log_publish_setpoint = bool(self.get_parameter('log_publish_setpoint').value)
        self._log_publish_setpoint_on_change_only = bool(
            self.get_parameter('log_publish_setpoint_on_change_only').value
        )
        self._log_publish_setpoint_throttle_sec = float(
            self.get_parameter('log_publish_setpoint_throttle_sec').value
        )
        self._log_publish_setpoint_epsilon = float(
            self.get_parameter('log_publish_setpoint_epsilon').value
        )

        # 加载 GPS 原点参数（用于 Global 模式转换）
        gps_origin = param_loader.load_gps_origin(
            default_lat=22.5180977,
            default_lon=113.9007239,
            default_alt=-5.17
        )
        self.origin_lat = gps_origin['lat']
        self.origin_lon = gps_origin['lon']
        self.origin_alt = gps_origin['alt']
        
        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # 创建发布器: 
        # 1. setpoint_raw/local: 仅用于特殊机动（如原地旋转 yaw_rate 控制）
        self.target_point_pub = self.create_publisher(PositionTarget, 'setpoint_raw/local', qos_best_effort)
        
        # 2. setpoint_raw/global: 主要导航控制（GlobalPositionTarget）
        self.global_target_pub = self.create_publisher(GlobalPositionTarget, 'setpoint_raw/global', qos_best_effort)
        
        # 根据控制模式显示不同的启动信息
        if self.control_mode == 'velocity':
            self.get_logger().info('='*60)
            self.get_logger().info('USV 控制节点启动 (速度模式 - 待机)')
            self.get_logger().info('   编队任务由 velocity_controller_node 处理')
            self.get_logger().info('   本节点仅处理离群单点导航 (NAV_MODE_TERMINAL)')
            self.get_logger().info('='*60)
        else:
            self.get_logger().info('='*60)
            self.get_logger().info('✅ USV 控制节点启动 (位置模式)')
            self.get_logger().info('   常规导航: GPS 坐标 → setpoint_raw/global')
            self.get_logger().info('   避障导航: GPS 坐标 → setpoint_raw/global')
            self.get_logger().info('   旋转机动: 由 velocity_controller_node 处理')
            self.get_logger().info('='*60)
        self.get_logger().info(f'📍 GPS 原点: ({self.origin_lat:.7f}, {self.origin_lon:.7f}, {self.origin_alt:.2f})')
        self.get_logger().info(f'🎮 控制模式: {self.control_mode.upper()}')

        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos_best_effort)
        
        # 订阅需要运行的目标点 (来自地面站)
        self.target_point_sub = self.create_subscription(
            NavigationGoal, 'set_usv_nav_goal', self.set_target_point_callback, qos_best_effort)
        
        # 订阅避障目标点
        self.avoidance_target_point_sub = self.create_subscription(
            PositionTarget, 'avoidance_position', self.set_avoidance_target_position_callback, qos_best_effort)
        
        # 订阅避障标记
        self.avoidance_flag_sub = self.create_subscription(
            Bool, 'avoidance_flag', self.set_avoidance_flag_callback, qos_reliable)
        
        # 订阅 Home Position（用于检查 EKF 原点是否设置）
        self.home_position_sub = self.create_subscription(
            HomePosition, 'home_position/home', self.home_position_callback, qos_best_effort)
        
        # 订阅本地位置（使用 GPS 转换的统一坐标系）
        self.local_position_sub = self.create_subscription(
            PoseStamped, 'local_position/pose_from_gps', self.local_position_callback, qos_best_effort)
        
        # 发送目标位置循环     
        self.publish_target_timer = self.create_timer(1.0/publish_rate, self.publish_target)
    
        # 初始化状态变量
        self.current_state = State()              # 当前状态
        self.current_target_position = PoseStamped()  # 常规目标点
        self.current_goal_id = 0                  # 当前目标ID
        
        # 注意: 旋转机动由 velocity_controller_node 处理
        # 本节点只负责 GPS 导航和避障
        
        self.use_yaw = False                          # 是否使用偏航角
        self.avoidance_position = PositionTarget()    # 避障目标点
        self.avoidance_flag = Bool(data=False)        # 避障标记，默认为False
        self.home_position_set = False                # Home Position 是否已设置
        self.local_position_valid = False             # 本地位置是否有效（验证 EKF Origin）
        self.ekf_origin_ready = False                 # EKF 原点就绪标志（Home + LocalPos 都有效）

        self.current_local_pose = None            # 当前本地位姿 (Pose)
        
        # 离群单点导航状态（速度模式下处理 NAV_MODE_TERMINAL）
        self._departed_target_active = False      # 离群目标是否激活
        self._departed_goal_id = 0                # 离群目标ID
        
        # 初始化消息对象和状态跟踪
        self.point_msg = PositionTarget()         # 目标点消息
        self.last_published_position = None       # 跟踪最后发布的坐标，避免重复发布

        # 发布循环日志节流/去重状态
        self._last_publish_log_time_sec = None
        self._last_publish_log_key = None
        self._publish_log_suppressed_count = 0
        
        # 日志记录
        self.get_logger().info(f'USV 控制节点已启动')
        self.get_logger().info(f'发布频率: {publish_rate} Hz')
        self.get_logger().info(f'坐标系: {self.frame_id}')
        
        # 日志已在各自分支打印

    def state_callback(self, msg):
        """
        状态回调函数
        
        Args:
            msg (State): 包含飞控状态信息的消息
        """
        if isinstance(msg, State):
            self.current_state = msg

    def home_position_callback(self, msg):
        """
        Home Position 回调函数（检查 Home 位置是否设置）
        
        Args:
            msg (HomePosition): 包含 Home 位置信息的消息
        """
        if isinstance(msg, HomePosition):
            # 检查是否为有效的 Home Position（纬度/经度不为 0）
            if abs(msg.geo.latitude) > 0.0001 or abs(msg.geo.longitude) > 0.0001:
                if not self.home_position_set:
                    self.home_position_set = True
                    self.get_logger().info(
                        f'✅ Home Position 已设置: '
                        f'({msg.geo.latitude:.7f}, {msg.geo.longitude:.7f}, {msg.geo.altitude:.2f}m)'
                    )
                    self._check_ekf_origin_ready()
    
    def local_position_callback(self, msg):
        """
        本地位置回调函数（验证 EKF 原点是否生效）
        
        Args:
            msg (PoseStamped): 包含本地位置信息的消息
        """
        if isinstance(msg, PoseStamped):
            self.current_local_pose = msg.pose # 保存当前位姿

            # 检查本地位置是否有效（不是全0或NaN）
            pos = msg.pose.position
            if not (pos.x == 0.0 and pos.y == 0.0 and pos.z == 0.0):
                if not self.local_position_valid:
                    self.local_position_valid = True
                    self.get_logger().info(
                        f'✅ Local Position 有效: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})'
                    )
                    self._check_ekf_origin_ready()
    
    def _check_ekf_origin_ready(self):
        """检查 EKF 原点是否完全就绪（Home + LocalPos 都有效）"""
        if self.home_position_set and self.local_position_valid and not self.ekf_origin_ready:
            self.ekf_origin_ready = True
            self.get_logger().info('🎯 EKF Origin 完全就绪，可以安全发布目标点！')

    def set_target_point_callback(self, msg):
        """
        设置目标点回调函数
        
        Args:
            msg (NavigationGoal): 包含目标点信息的消息
        
        注意：
        - 位置模式(position)：处理所有导航目标
        - 速度模式(velocity)：仅处理 NAV_MODE_TERMINAL (离群单点导航)
        """
        if not isinstance(msg, NavigationGoal):
            self.get_logger().warn('收到无效的目标点消息类型')
            return
        
        # 获取导航模式
        nav_mode = getattr(msg, 'nav_mode', 0)
        NAV_MODE_TERMINAL = 3  # 定义在 NavigationGoal.msg
        
        # 速度模式下，仅处理离群单点导航 (NAV_MODE_TERMINAL)
        # 其他导航目标由 velocity_controller_node 处理
        if self.control_mode == 'velocity' and nav_mode != NAV_MODE_TERMINAL:
            self.get_logger().info(
                f'⏭️ 跳过目标 [ID={msg.goal_id}]: 速度模式下非TERMINAL模式(nav_mode={nav_mode})由velocity_controller处理')
            return
        
        # 如果是离群单点导航且在速度模式，激活离群目标处理
        if self.control_mode == 'velocity' and nav_mode == NAV_MODE_TERMINAL:
            self._departed_target_active = True  # 激活离群目标处理标志
            self._departed_goal_id = msg.goal_id
            self.get_logger().info(
                f'📍 位置模式处理离群目标 [ID={msg.goal_id}]: '
                f'({msg.target_pose.pose.position.x:.2f}, {msg.target_pose.pose.position.y:.2f})'
            )
        
        # 检查目标点坐标有效性
        if (msg.target_pose.pose.position.x is None or msg.target_pose.pose.position.y is None or 
            msg.target_pose.pose.position.z is None):
            self.get_logger().warn('收到的目标点坐标无效')
            return
            
        old_position = self.current_target_position.pose.position
        new_position = msg.target_pose.pose.position
        
        # 更新目标点和Yaw标志
        self.current_target_position = msg.target_pose
        self.use_yaw = msg.enable_yaw
        
        # 更新目标ID
        if msg.goal_id != self.current_goal_id:
            self.current_goal_id = msg.goal_id
            
            # 注意: 旋转机动由 velocity_controller_node 处理
            # 如果收到旋转指令，位置模式不处理，只记录日志
            maneuver_type = getattr(msg, 'maneuver_type', 0)
            if maneuver_type == NavigationGoal.MANEUVER_TYPE_ROTATE:
                self.get_logger().info(
                    f'⚠️ 收到旋转指令 [ID={msg.goal_id}]，由 velocity_controller_node 处理'
                )
                return  # 不处理旋转，退出

        # 只有当目标点发生变化时才更新
        # 优化日志：无论是否变化，只要收到新指令都打印调试信息，并在变化时打印INFO
        log_msg = (f'接收目标 [ID={msg.goal_id}]: XY({new_position.x:.1f}, {new_position.y:.1f}), '
                   f'Yaw({math.degrees(self.current_target_position.pose.orientation.z):.1f}°)')

        if (old_position.x != new_position.x or 
            old_position.y != new_position.y or 
            old_position.z != new_position.z or
            self.current_goal_id != msg.goal_id):
            
            self.get_logger().info(f'🆕 {log_msg}')
        else:
            self.get_logger().debug(f'♻️ {log_msg} (无变化)')

    def set_avoidance_target_position_callback(self, msg):
        """
        设置避障目标点回调函数
        
        Args:
            msg (PositionTarget): 包含避障目标点信息的消息
        """
        if not isinstance(msg, PositionTarget):
            self.get_logger().warn('收到无效的避障目标点消息类型')
            return
            
        # 检查避障目标点坐标有效性
        if (msg.position.x is None or msg.position.y is None or 
            msg.position.z is None):
            self.get_logger().warn('收到的避障目标点坐标无效')
            return
            
        old_position = self.avoidance_position.position
        new_position = msg.position
        
        # 只有当避障目标点发生变化时才更新
        if (old_position.x != new_position.x or 
            old_position.y != new_position.y or 
            old_position.z != new_position.z):
            
            self.avoidance_position = msg
            self.get_logger().info(f'更新避障目标点: ({new_position.x:.2f}, {new_position.y:.2f}, {new_position.z:.2f})')

    def set_avoidance_flag_callback(self, msg):
        """
        设置避障标志回调函数
        
        Args:
            msg (Bool): 包含避障标志信息的消息
        """
        if not isinstance(msg, Bool):
            self.get_logger().warn('收到无效的避障标志消息类型')
            return
            
        # 只有当避障标志状态发生变化时才处理
        if self.avoidance_flag.data != msg.data:
            self.avoidance_flag = msg
            mode = "避障模式" if msg.data else "常规模式"
            self.get_logger().info(f'切换到: {mode}')

    def publish_target(self):
        """
        发布目标点函数
        
        - position 模式: 发布 GPS 坐标到 setpoint_raw/global（包含避障和旋转）
        - velocity 模式: 不处理，所有功能由 velocity_controller_node 处理
        """
        try:
            # 检查飞控连接状态
            if not self.current_state.connected:
                self.get_logger().debug('飞控未连接，等待连接...')
                return
                
            if not self.current_state.armed:
                self.get_logger().debug('飞控未解锁，等待解锁...')
                return
                
            if self.current_state.mode != "GUIDED":
                self.get_logger().debug(f'当前模式: {self.current_state.mode}，需要GUIDED模式')
                return
            
            # ============================================================
            # 速度模式: 主要由 velocity_controller_node 处理
            # 但离群单点导航 (NAV_MODE_TERMINAL) 使用本节点的位置模式处理
            # 检查当前目标是否是离群目标
            # ============================================================
            if self.control_mode == 'velocity':
                # 检查是否有待处理的离群目标（通过标志位判断）
                # 如果没有离群目标，则跳过
                if not getattr(self, '_departed_target_active', False):
                    return
                # 调试：确认 velocity 模式下因离群目标而发布
                self.get_logger().debug(
                    f'[velocity模式] 离群目标激活，发布GPS目标 [ID={self._departed_goal_id}]')
            
            # ============================================================
            # 位置模式: GPS 坐标导航（包含避障和旋转）
            # ============================================================
            
            # 根据避障标志选择目标点
            if not self.avoidance_flag.data:    
                # 使用常规目标点
                px = self.current_target_position.pose.position.x
                py = self.current_target_position.pose.position.y
                pz = self.current_target_position.pose.position.z
                source = "常规"
            else:
                # 使用避障目标点
                px = self.avoidance_position.position.x
                py = self.avoidance_position.position.y
                pz = self.avoidance_position.position.z  
                source = "避障"
            
            # 检查目标点坐标是否有效
            if any(coord is None for coord in [px, py, pz]):
                self.get_logger().warn(f'{source}目标点坐标无效，忽略')
                return
            
            # 计算距离目标的距离 (2D)
            dist_to_target = 0.0
            if self.current_local_pose:
                dx = px - self.current_local_pose.position.x
                dy = py - self.current_local_pose.position.y
                dist_to_target = math.sqrt(dx*dx + dy*dy)

            # ============================================================
            # GPS 导航模式: GlobalPositionTarget
            # 注意: 旋转机动由 velocity_controller_node 处理
            # ============================================================
            
            # 使用 GeoUtils 进行高精度 WGS84 转换 (XYZ -> GPS)
            gps_coord = GeoUtils.xyz_to_gps(
                px, py, pz, 
                self.origin_lat, self.origin_lon, self.origin_alt
            )
            
            global_msg = GlobalPositionTarget()
            global_msg.header.stamp = self.get_clock().now().to_msg()
            global_msg.header.frame_id = 'map'
            global_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
            global_msg.type_mask = (
                GlobalPositionTarget.IGNORE_VX |
                GlobalPositionTarget.IGNORE_VY |
                GlobalPositionTarget.IGNORE_VZ |
                GlobalPositionTarget.IGNORE_AFX |
                GlobalPositionTarget.IGNORE_AFY |
                GlobalPositionTarget.IGNORE_AFZ |
                GlobalPositionTarget.IGNORE_YAW_RATE |
                GlobalPositionTarget.IGNORE_ALTITUDE # 忽略高度控制 (水面船)
            )

            #测试使用
            yaw_active = self.use_yaw and not self.avoidance_flag.data


            if self.use_yaw and not self.avoidance_flag.data:
                # 仅在非避障模式下使用Yaw控制
                q = self.current_target_position.pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                global_msg.yaw = math.atan2(siny_cosp, cosy_cosp)
            else:
                global_msg.type_mask |= GlobalPositionTarget.IGNORE_YAW
            
            # 设置GPS坐标
            global_msg.latitude = gps_coord['lat']
            global_msg.longitude = gps_coord['lon']
            global_msg.altitude = gps_coord['alt'] # 虽然设置了值，但 mask 已忽略之
            
            self.global_target_pub.publish(global_msg)
            self._maybe_log_publish_setpoint(source=source, px=px, py=py, pz=pz, yaw_active=yaw_active)
            
            # ============================================================
            # 离群目标到达检测：到达后重置标志
            # ============================================================
            if self._departed_target_active and self.control_mode == 'velocity':
                DEPARTED_ARRIVAL_THRESHOLD = 1.0  # 离群目标到达阈值 (米)
                if dist_to_target <= DEPARTED_ARRIVAL_THRESHOLD:
                    self.get_logger().info(
                        f'✅ 离群目标到达 [ID={self._departed_goal_id}]: 距离={dist_to_target:.2f}m'
                    )
                    self._departed_target_active = False  # 重置标志

        except Exception as e:

            self.get_logger().error(f'发布目标点时发生异常: {str(e)}')

    # 注意: 旋转机动 (_handle_rotation_maneuver) 已移至 velocity_controller_node
    # usv_control_node 只负责 GPS 导航和避障

    def _publish_global_position_target(self, px: float, py: float, pz: float, source: str = "常规"):
        """
        发布全局GPS位置目标（辅助方法）
        
        Args:
            px, py, pz: 本地坐标
            source: 日志来源标识
        """
        # 使用 GeoUtils 进行 WGS84 转换
        gps_coord = GeoUtils.xyz_to_gps(
            px, py, pz,
            self.origin_lat, self.origin_lon, self.origin_alt
        )

        global_msg = GlobalPositionTarget()
        global_msg.header.stamp = self.get_clock().now().to_msg()
        global_msg.header.frame_id = 'map'
        global_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        global_msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX |
            GlobalPositionTarget.IGNORE_VY |
            GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX |
            GlobalPositionTarget.IGNORE_AFY |
            GlobalPositionTarget.IGNORE_AFZ |
            GlobalPositionTarget.IGNORE_YAW_RATE |
            GlobalPositionTarget.IGNORE_ALTITUDE |
            GlobalPositionTarget.IGNORE_YAW
        )

        global_msg.latitude = gps_coord['lat']
        global_msg.longitude = gps_coord['lon']
        global_msg.altitude = gps_coord['alt']

        self.global_target_pub.publish(global_msg)
        self._maybe_log_publish_setpoint(source=source, px=px, py=py, pz=pz, yaw_active=False)

    def _maybe_log_publish_setpoint(
        self,
        *,
        source: str,
        px: float,
        py: float,
        pz: float,
        yaw_active: bool,
    ) -> None:
        """
        智能日志输出方法：发布目标点时的日志节流与去重
        
        该方法在 publish_target() 定时器中被调用（默认 20Hz），用于：
        1. 避免高频发布时产生大量重复日志刷屏
        2. 通过坐标量化去重，忽略微小位置变化
        3. 通过时间节流，限制日志输出频率
        
        日志输出条件（需同时满足）：
        - _log_publish_setpoint = True（总开关）
        - 坐标/来源发生变化 或 _log_publish_setpoint_on_change_only = False
        - 距离上次输出时间 >= _log_publish_setpoint_throttle_sec
        
        抑制的日志会累计计数，在下次输出时显示 "(已抑制N条重复日志)"
        
        Args:
            source: 目标点来源标识，如 "常规" 或 "避障"
            px, py, pz: 目标点本地坐标 (米)
            yaw_active: 是否启用偏航角控制
            
        配置参数（在 __init__ 中从 ROS 参数加载）：
            log_publish_setpoint: 日志总开关 (默认 True)
            log_publish_setpoint_on_change_only: 仅变化时输出 (默认 True)
            log_publish_setpoint_throttle_sec: 最小输出间隔秒数 (默认 2.0s)
            log_publish_setpoint_epsilon: 坐标量化精度 (默认 0.01m)
        """
        # ==================== 1. 总开关检查 ====================
        if not self._log_publish_setpoint:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        eps = max(0.0, float(self._log_publish_setpoint_epsilon))

        # ==================== 2. 坐标量化（去重用） ====================
        # 将浮点坐标量化到网格上，忽略 epsilon 范围内的微小变化
        # 例如 epsilon=0.01 时，(1.001, 2.002) 和 (1.009, 2.008) 视为相同
        if eps > 0.0:
            qx = int(round(px / eps))
            qy = int(round(py / eps))
            qz = int(round(pz / eps))
        else:
            qx, qy, qz = px, py, pz

        # 生成唯一键：(来源, 量化X, 量化Y, 量化Z, Yaw状态)
        key = (source, qx, qy, qz, yaw_active)

        # ==================== 3. 内容去重检查 ====================
        # 如果启用了"仅变化时输出"且内容未变化，则抑制本次日志
        unchanged = (self._last_publish_log_key == key)
        if self._log_publish_setpoint_on_change_only and unchanged:
            self._publish_log_suppressed_count += 1
            return

        # ==================== 4. 时间节流检查 ====================
        # 如果距上次输出时间不足阈值，则抑制本次日志
        if (
            self._last_publish_log_time_sec is not None
            and self._log_publish_setpoint_throttle_sec > 0.0
        ):
            if (
                (now_sec - self._last_publish_log_time_sec)
                < self._log_publish_setpoint_throttle_sec
            ):
                self._publish_log_suppressed_count += 1
                return

        # ==================== 5. 输出日志 ====================
        # 构建抑制计数后缀（如有）
        suppressed_suffix = ''
        if self._publish_log_suppressed_count > 0:
            suppressed_suffix = f' (已抑制{self._publish_log_suppressed_count}条重复日志)'

        self.get_logger().info(
            f'发布{source}目标点(GPS): XYZ({px:.2f}, {py:.2f}, {pz:.2f}) → '
            f'Yaw{"启用" if yaw_active else "忽略"}{suppressed_suffix}'
        )

        # ==================== 6. 更新状态 ====================
        self._last_publish_log_time_sec = now_sec  # 记录本次输出时间
        self._last_publish_log_key = key           # 记录本次输出内容
        self._publish_log_suppressed_count = 0     # 重置抑制计数

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if hasattr(self, 'publish_target_timer'):
            self.publish_target_timer.cancel()
        super().destroy_node()


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = UsvControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
