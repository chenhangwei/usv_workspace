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
        # 默认不发送Z轴高度（水面船）
        
        # 获取参数值
        publish_rate_param = self.get_parameter('publish_rate').value
        publish_rate = 20.0 if publish_rate_param is None else float(publish_rate_param)
        self.frame_id = self.get_parameter('frame_id').value

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
        
        self.get_logger().info('✅ USV 控制节点启动 (全局GPS导航模式)')
        self.get_logger().info(f'📍 GPS 原点: ({self.origin_lat:.7f}, {self.origin_lon:.7f}, {self.origin_alt:.2f})')

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
        self.current_maneuver_type = 0            # 当前机动类型
        self.current_maneuver_param = 0.0         # 当前机动参数
        self.current_goal_id = 0                  # 当前目标ID

        # 旋转机动相关变量
        self.rotating = False                     # 是否正在旋转
        self.rotation_accumulated_yaw = 0.0       # 累积旋转角度
        self.rotation_target_yaw = 0.0            # 目标总旋转量
        self.rotation_last_yaw = 0.0              # 上一次的航向
        self.rotation_initialized = False         # 旋转是否已初始化（记录初始Yaw）
        
        self.use_yaw = False                          # 是否使用偏航角
        self.avoidance_position = PositionTarget()    # 避障目标点
        self.avoidance_flag = Bool(data=False)        # 避障标记，默认为False
        self.home_position_set = False                # Home Position 是否已设置
        self.local_position_valid = False             # 本地位置是否有效（验证 EKF Origin）
        self.ekf_origin_ready = False                 # EKF 原点就绪标志（Home + LocalPos 都有效）

        self.current_local_pose = None            # 当前本地位姿 (Pose)
        
        # 初始化消息对象和状态跟踪
        self.point_msg = PositionTarget()         # 目标点消息
        self.last_published_position = None       # 跟踪最后发布的坐标，避免重复发布
        
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
        """
        if not isinstance(msg, NavigationGoal):
            self.get_logger().warn('收到无效的目标点消息类型')
            return
        
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
        
        # 更新机动参数
        new_maneuver_type = getattr(msg, 'maneuver_type', 0)
        new_maneuver_param = getattr(msg, 'maneuver_param', 0.0)
        
        # 如果是新的目标ID，或者机动类型变化，重置机动状态
        if msg.goal_id != self.current_goal_id:
            self.current_goal_id = msg.goal_id
            self.current_maneuver_type = new_maneuver_type
            self.current_maneuver_param = new_maneuver_param
            
            if self.current_maneuver_type == NavigationGoal.MANEUVER_TYPE_ROTATE:
                self.get_logger().info(f"收到旋转指令: 圈数 {self.current_maneuver_param}")
                self.rotating = True
                self.rotation_accumulated_yaw = 0.0
                self.rotation_initialized = False # 等待下一次循环获取初始Yaw
                # 计算目标总角度 (param * 2PI)
                self.rotation_target_yaw = self.current_maneuver_param * 2 * math.pi
            else:
                self.rotating = False

        # 只有当目标点发生变化时才更新
        # 优化日志：无论是否变化，只要收到新指令都打印调试信息，并在变化时打印INFO
        log_msg = (f'接收目标 [ID={msg.goal_id}]: XY({new_position.x:.1f}, {new_position.y:.1f}), '
                   f'Yaw({math.degrees(self.current_target_position.pose.orientation.z):.1f}°), '
                   f'Maneuver({new_maneuver_type}, {new_maneuver_param:.1f})')

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
        
        发布 PositionTarget 到 setpoint_raw/local (局部坐标)
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
            
            # 移除 EKF Origin 强制检查 (全局导航模式下主要依赖 GPS Fix)
            # 但仍需 local_pose 存在才能计算距离进行模式切换
            
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
            
            # 设定开始机动的距离阈值 (米)
            MANEUVER_Is_CLOSE_ENOUGH = 1.0 

            # ============================================================
            # 根据模式发布不同类型的消息
            # ============================================================

            # 优先处理特殊机动: 原地旋转
            # 只有当 1. 处于旋转请求状态 2. 机动类型正确 3. 已经到达目标点附近 时才执行旋转
            should_rotate = (self.rotating and 
                           self.current_maneuver_type == NavigationGoal.MANEUVER_TYPE_ROTATE and
                           dist_to_target <= MANEUVER_Is_CLOSE_ENOUGH)

            if should_rotate:
                if self.current_local_pose is None:
                    return # 等待定位数据

                # 获取当前Yaw
                q = self.current_local_pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                current_yaw = math.atan2(siny_cosp, cosy_cosp)

                if not self.rotation_initialized:
                    self.rotation_last_yaw = current_yaw
                    self.rotation_initialized = True
                    self.rotation_accumulated_yaw = 0.0
                    return

                # 计算Yaw delta
                delta_yaw = current_yaw - self.rotation_last_yaw
                # 处理角度跳变 (-PI <-> PI)
                if delta_yaw > math.pi:
                    delta_yaw -= 2*math.pi
                elif delta_yaw < -math.pi:
                    delta_yaw += 2*math.pi
                
                self.rotation_accumulated_yaw += delta_yaw
                self.rotation_last_yaw = current_yaw

                # 检查是否完成
                # 目标是 param * 2PI. param 可以是正负.
                # 如果 param > 0, 我们期望 accumulated_yaw 增加到 target_yaw
                # 如果 param < 0, 我们期望 accumulated_yaw 减小到 target_yaw
                
                done = False
                target_yaw_rate = 0.5 # rad/s 默认
                
                if self.rotation_target_yaw > 0:
                    if self.rotation_accumulated_yaw >= self.rotation_target_yaw:
                        done = True
                    else:
                        target_yaw_rate = 0.5
                else:
                    if self.rotation_accumulated_yaw <= self.rotation_target_yaw:
                        done = True
                    else:
                        target_yaw_rate = -0.5
                
                msg = PositionTarget()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'body' # 使用机体坐标系? 或者 FRAME_LOCAL_NED
                msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
                
                # 发送角速度
                msg.type_mask = (
                    PositionTarget.IGNORE_PX |
                    PositionTarget.IGNORE_PY |
                    PositionTarget.IGNORE_PZ |
                    PositionTarget.IGNORE_VX |
                    PositionTarget.IGNORE_VY |
                    PositionTarget.IGNORE_VZ |
                    PositionTarget.IGNORE_AFX |
                    PositionTarget.IGNORE_AFY |
                    PositionTarget.IGNORE_AFZ |
                    PositionTarget.IGNORE_YAW 
                    # 不忽略 RATE
                )

                if done:
                    msg.yaw_rate = 0.0
                    self.rotating = False
                    self.current_maneuver_type = 0 # 结束机动
                    self.get_logger().info("旋转机动完成")
                else:
                    msg.yaw_rate = target_yaw_rate
                    # 调试进度
                    # self.get_logger().info(f"旋转中: {self.rotation_accumulated_yaw:.2f}/{self.rotation_target_yaw:.2f}")

                # 发布到本地控制 (Rotation usually local)
                self.target_point_pub.publish(msg)
                return

            # ========== 全局GPS模式: GlobalPositionTarget ==========
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
            # 修改为INFO级别并在日志中显示更高精度的经纬度，以便调试厘米级误差
            self.get_logger().info(
                f'发布{source}目标点(GPS): XYZ({px:.2f}, {py:.2f}, {pz:.2f}) → '
                f'GPS({gps_coord["lat"]:.9f}°, {gps_coord["lon"]:.9f}°, {gps_coord["alt"]:.3f}m)')

        except Exception as e:

            self.get_logger().error(f'发布目标点时发生异常: {str(e)}')

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
