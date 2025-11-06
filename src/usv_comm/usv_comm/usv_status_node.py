from math import sqrt, degrees, atan2
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget, WaypointReached, Altitude, ExtendedState, GPSRAW
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String, Bool, Float32, Int32, Float64
from common_interfaces.msg import UsvStatus
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3, Point
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import psutil
import time
from collections import deque


class UsvStatusNode(Node):
    """无人船状态节点类
    
    该节点负责收集无人船的各种状态信息并整合发布。
    主要功能包括：
    1. 订阅飞控状态、电池状态、位置和速度信息
    2. 整合所有状态信息并发布到统一的状态主题
    3. 获取并发布系统温度
    """

    def __init__(self):
        """初始化无人船状态节点"""
        super().__init__('usv_status_node')

        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 创建发布者
        self.state_publisher = self.create_publisher(UsvStatus, 'usv_state', 10)
        self.temperature_publisher = self.create_publisher(Float32, 'usv_temperature', 10)

        # 参数配置
        self.declare_parameter('target_reach_threshold', 1.0)
        self.declare_parameter('distance_mode', '2d')
        self.declare_parameter('publish_rate', 1.0)  # 发布频率（Hz）
        self.declare_parameter('data_timeout', 5.0)  # 数据超时时间（秒）
        self.declare_parameter('enable_system_monitor', True)  # 是否启用系统监控
        
        # 电池电压范围参数
        self.declare_parameter('battery_voltage_full', 12.6)   # 满电电压（V）
        self.declare_parameter('battery_voltage_empty', 10.5)  # 空电电压（V）
        self.declare_parameter('battery_avg_window', 10.0)     # 电压平均窗口（秒）
        
        # 根据节点命名空间推断 usv_id
        ns_guess = self.get_namespace().lstrip('/') if self.get_namespace() else 'usv_01'
        self.declare_parameter('usv_id', ns_guess)
        
        # 读取参数
        try:
            self.target_reach_threshold = float(self.get_parameter('target_reach_threshold').get_parameter_value().double_value)
        except Exception:
            self.target_reach_threshold = 1.0
        try:
            self.distance_mode = str(self.get_parameter('distance_mode').get_parameter_value().string_value).lower()
        except Exception:
            self.distance_mode = '2d'
        try:
            publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        except Exception:
            publish_rate = 1.0
        try:
            self.data_timeout = float(self.get_parameter('data_timeout').get_parameter_value().double_value)
        except Exception:
            self.data_timeout = 5.0
        try:
            self.enable_system_monitor = bool(self.get_parameter('enable_system_monitor').get_parameter_value().bool_value)
        except Exception:
            self.enable_system_monitor = True
        
        # 读取电池电压范围参数
        try:
            self.battery_voltage_full = float(self.get_parameter('battery_voltage_full').get_parameter_value().double_value)
        except Exception:
            self.battery_voltage_full = 12.6
        try:
            self.battery_voltage_empty = float(self.get_parameter('battery_voltage_empty').get_parameter_value().double_value)
        except Exception:
            self.battery_voltage_empty = 10.5
        try:
            self.battery_avg_window = float(self.get_parameter('battery_avg_window').get_parameter_value().double_value)
        except Exception:
            self.battery_avg_window = 10.0

        # 读取 usv_id 参数
        try:
            usv_id_val = self.get_parameter('usv_id').get_parameter_value().string_value
            self.usv_id = usv_id_val.lstrip('/') if usv_id_val else ns_guess
        except Exception:
            self.usv_id = ns_guess
        
        # 检查命名空间一致性
        try:
            ns_now = self.get_namespace().lstrip('/') if self.get_namespace() else ''
            if ns_now and ns_now != self.usv_id:
                self.get_logger().warn(f"usv_id 参数 ({self.usv_id}) 与节点命名空间 ({ns_now}) 不一致")
        except Exception:
            pass
        
        # 记录电池电压配置
        self.get_logger().info(
            f"电池电压范围配置: 满电={self.battery_voltage_full}V (100%), "
            f"空电={self.battery_voltage_empty}V (0%), "
            f"平均窗口={self.battery_avg_window}秒"
        )

        # 初始化状态变量
        self.target_point = Point()
        self.usv_state = State()
        self.usv_battery = BatteryState()
        self.usv_velocity = TwistStamped()
        self.usv_pose = PoseStamped()
        self.usv_gps = NavSatFix()
        self.usv_gps_raw = GPSRAW()  # 添加 GPS RAW 数据
        self.usv_altitude = Altitude()
        self.usv_extended_state = ExtendedState()
        self.usv_compass_hdg = 0.0  # 罗盘航向（度数）
        
        # 电压历史缓存（用于计算平均电压）
        # 存储格式：(timestamp, voltage)
        self.voltage_history = deque()
        self.low_voltage_mode = False  # 低电压模式标志
        self.low_voltage_triggered = False  # 是否已触发低电压警告
        
        # 数据时效性跟踪（记录最后接收时间）
        self.last_state_time = 0.0
        self.last_battery_time = 0.0
        self.last_velocity_time = 0.0
        self.last_pose_time = 0.0
        self.last_gps_time = 0.0
        self.last_gps_raw_time = 0.0  # 添加 GPS RAW 时间戳
        self.last_altitude_time = 0.0
        self.last_compass_time = 0.0  # 罗盘航向时间戳
        
        # 消息计数器
        self.message_count = 0
        
        # 电量计算日志控制（每10条消息记录一次）
        self.battery_log_counter = 0
        
        # 系统启动时间
        self.start_time = time.time()
        
        # 温度缓存（避免频繁读文件）
        self.cached_temperature = 0.0
        self.last_temp_update = 0.0
        self.temp_cache_interval = 5.0  # 温度缓存5秒

        self.get_logger().info(f'状态报告节点已启动 (usv_id: {self.usv_id}, 发布频率: {publish_rate} Hz)')

        # 订阅 MAVROS 主题（注意：MAVROS 在相同命名空间下，topic 不需要 mavros/ 前缀）
        self.state_sub = self.create_subscription(
            State, 'state', self.usv_state_callback, qos_best_effort)
        
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery', self.usv_battery_callback, qos_best_effort)
        
        self.velocity_local_sub = self.create_subscription(
            TwistStamped, 'local_position/velocity_local', 
            self.usv_velocity_callback, qos_best_effort)
        
        self.pos_sub = self.create_subscription(
            PoseStamped, 'local_position/pose', 
            self.usv_pose_callback, qos_best_effort)
        
        self.target_sub = self.create_subscription(
            PositionTarget, 'setpoint_raw/local', 
            self.target_callback, qos_best_effort)
        
        self.extended_state_sub = self.create_subscription(
            ExtendedState, 'extended_state',
            self.extended_state_callback, qos_best_effort)
        
        # GPS 相关订阅（需要 MAVROS global_position 插件）
        # 如果 launch 文件中未加载该插件，这些订阅会失败但不影响节点运行
        self.has_gps = False
        self.has_gps_raw = False
        self.has_altitude = False
        
        try:
            self.gps_sub = self.create_subscription(
                NavSatFix, 'global_position/global',
                self.gps_callback, qos_best_effort)
            self.has_gps = True
            self.get_logger().info('GPS topic 订阅成功')
        except Exception as e:
            self.get_logger().info('GPS topic 不可用（需要 MAVROS global_position 插件）')
        
        # 订阅 GPS RAW 消息（包含卫星数）
        try:
            self.gps_raw_sub = self.create_subscription(
                GPSRAW, 'gpsstatus/gps1/raw',
                self.gps_raw_callback, qos_best_effort)
            self.has_gps_raw = True
            self.get_logger().info('GPS RAW topic 订阅成功（用于获取卫星数）')
        except Exception as e:
            self.get_logger().info(f'GPS RAW topic 不可用: {e}')
        
        try:
            self.altitude_sub = self.create_subscription(
                Altitude, 'altitude',
                self.altitude_callback, qos_best_effort)
            self.has_altitude = True
            self.get_logger().info('Altitude topic 订阅成功')
        except Exception as e:
            self.get_logger().info('Altitude topic 不可用（已包含在 global_position 中）')
        
        # 订阅罗盘航向（来自飞控的磁力计）
        self.has_compass = False
        try:
            self.compass_sub = self.create_subscription(
                Float64, 'global_position/compass_hdg',
                self.compass_callback, qos_best_effort)
            self.has_compass = True
            self.get_logger().info('罗盘航向 topic 订阅成功')
        except Exception as e:
            self.get_logger().info(f'罗盘航向 topic 不可用: {e}')

        # 定时器，定期发布状态信息
        timer_period = 1.0 / publish_rate
        self.state_timer = self.create_timer(timer_period, self.state_timer_callback)

    def target_callback(self, msg):
        """目标点回调函数
        
        Args:
            msg (PositionTarget): 包含目标点位置的消息
        """
        if isinstance(msg, PositionTarget):
            self.target_point.x = msg.position.x
            self.target_point.y = msg.position.y
            self.target_point.z = msg.position.z
        else:
            self.get_logger().error('接收到的消息类型不正确')

    def usv_state_callback(self, msg):
        """飞控状态回调函数"""
        if isinstance(msg, State):
            self.usv_state = msg
            self.last_state_time = time.time()

    def usv_battery_callback(self, msg):
        """电池状态回调函数"""
        if isinstance(msg, BatteryState):
            if msg.voltage <= 0.1 and not msg.location:
                return
            self.usv_battery = msg
            self.last_battery_time = time.time()
            
            # 记录电压历史（用于计算平均值）
            current_time = time.time()
            self.voltage_history.append((current_time, msg.voltage))
            
            # 清理超过时间窗口的旧数据
            while self.voltage_history and (current_time - self.voltage_history[0][0]) > self.battery_avg_window:
                self.voltage_history.popleft()

    def usv_velocity_callback(self, msg):
        """速度回调函数"""
        if isinstance(msg, TwistStamped):
            self.usv_velocity = msg
            self.last_velocity_time = time.time()

    def usv_pose_callback(self, msg):
        """位置回调函数"""
        if isinstance(msg, PoseStamped):
            self.usv_pose = msg
            self.last_pose_time = time.time()
    
    def gps_callback(self, msg):
        """GPS 回调函数"""
        if isinstance(msg, NavSatFix):
            self.usv_gps = msg
            self.last_gps_time = time.time()
    
    def gps_raw_callback(self, msg):
        """GPS RAW 回调函数（包含卫星数）"""
        if isinstance(msg, GPSRAW):
            self.usv_gps_raw = msg
            self.last_gps_raw_time = time.time()
    
    def altitude_callback(self, msg):
        """高度回调函数"""
        if isinstance(msg, Altitude):
            self.usv_altitude = msg
            self.last_altitude_time = time.time()
    
    def compass_callback(self, msg):
        """罗盘航向回调函数"""
        if isinstance(msg, Float64):
            self.usv_compass_hdg = msg.data
            self.last_compass_time = time.time()
    
    def extended_state_callback(self, msg):
        """扩展状态回调函数"""
        if isinstance(msg, ExtendedState):
            self.usv_extended_state = msg
    
    def get_average_voltage(self):
        """
        计算电压历史的平均值
        
        Returns:
            float: 平均电压，如果没有历史数据则返回当前电压
        """
        if not self.voltage_history:
            # 没有历史数据，返回当前电压
            return getattr(self.usv_battery, 'voltage', 0.0)
        
        # 计算平均值
        total_voltage = sum(voltage for _, voltage in self.voltage_history)
        avg_voltage = total_voltage / len(self.voltage_history)
        return avg_voltage
    
    def calculate_battery_percentage(self, voltage):
        """
        根据电压计算电量百分比
        
        公式：percentage = (12.6 - voltage) / (12.6 - 10.5) * 100
        注意：这个公式是反向的，电压越低，百分比越高是错误的
        正确公式应该是：percentage = (voltage - 10.5) / (12.6 - 10.5) * 100
        
        Args:
            voltage: 当前电压（V）
            
        Returns:
            float: 电量百分比 (0-100)
        """
        v_full = self.battery_voltage_full   # 12.6V
        v_empty = self.battery_voltage_empty  # 10.5V
        
        if voltage <= v_empty:
            return 0.0
        elif voltage >= v_full:
            return 100.0
        else:
            # 正确公式：电压越高，百分比越高
            percentage = (voltage - v_empty) / (v_full - v_empty) * 100.0
            return max(0.0, min(100.0, percentage))

    def state_timer_callback(self):
        """定时器回调函数，定期发布状态信息"""
        try:
            current_time = time.time()
            msg = UsvStatus()
            
            # ==================== Header 和基本信息 ====================
            msg.usv_id = self.usv_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            # 消息计数
            self.message_count += 1
            msg.message_count = self.message_count
            
            # ==================== 数据有效性检查 ====================
            pose_age = current_time - self.last_pose_time if self.last_pose_time > 0 else 999.0
            state_age = current_time - self.last_state_time if self.last_state_time > 0 else 999.0
            battery_age = current_time - self.last_battery_time if self.last_battery_time > 0 else 999.0
            
            msg.data_age_sec = max(pose_age, state_age, battery_age)
            msg.data_valid = (pose_age < self.data_timeout and 
                             state_age < self.data_timeout)
            
            if not msg.data_valid:
                self.get_logger().warn(
                    f'数据过期 - Pose: {pose_age:.1f}s, State: {state_age:.1f}s, Battery: {battery_age:.1f}s',
                    throttle_duration_sec=5.0)

            # ==================== 飞控状态 ====================
            msg.armed = getattr(self.usv_state, 'armed', False)
            msg.connected = getattr(self.usv_state, 'connected', False)
            msg.mode = getattr(self.usv_state, 'mode', 'UNKNOWN')
            msg.guided = getattr(self.usv_state, 'guided', False)
            
            # 系统状态映射
            system_status_map = {
                0: 'UNINIT', 1: 'BOOT', 2: 'CALIBRATING', 3: 'STANDBY',
                4: 'ACTIVE', 5: 'CRITICAL', 6: 'EMERGENCY', 7: 'POWEROFF',
                8: 'FLIGHT_TERMINATION'
            }
            system_status_id = getattr(self.usv_state, 'system_status', 0)
            msg.system_status = system_status_map.get(system_status_id, 'UNKNOWN')

            # ==================== 电池信息 ====================
            # 获取当前电压
            current_voltage = max(0.0, getattr(self.usv_battery, 'voltage', 0.0))
            msg.battery_voltage = current_voltage
            
            # 计算 10 秒平均电压
            avg_voltage = self.get_average_voltage()
            
            # 使用平均电压计算电量百分比
            # 公式：percentage = (voltage - 10.5) / (12.6 - 10.5) * 100
            battery_pct = self.calculate_battery_percentage(avg_voltage)
            msg.battery_percentage = battery_pct
            
            # 检查是否进入低电压模式（百分比 < 0 表示电压低于 10.5V）
            if avg_voltage < self.battery_voltage_empty:
                if not self.low_voltage_mode:
                    # 刚进入低电压模式
                    self.low_voltage_mode = True
                    self.get_logger().error(
                        f'[!][!][!] 低电压模式触发！ [!][!][!]\n'
                        f'当前电压: {current_voltage:.2f}V, '
                        f'平均电压(10s): {avg_voltage:.2f}V, '
                        f'临界电压: {self.battery_voltage_empty}V\n'
                        f'请立即返航或靠岸！'
                    )
            else:
                if self.low_voltage_mode:
                    # 退出低电压模式
                    self.low_voltage_mode = False
                    self.get_logger().info(
                        f'[OK] 退出低电压模式 - 平均电压: {avg_voltage:.2f}V'
                    )
            
            # 定期记录电量计算日志（每10条消息记录一次，避免刷屏）
            self.battery_log_counter += 1
            if self.battery_log_counter >= 10:
                self.battery_log_counter = 0
                voltage_samples = len(self.voltage_history)
                self.get_logger().debug(
                    f"电池: 当前={current_voltage:.2f}V, "
                    f"平均(10s)={avg_voltage:.2f}V, "
                    f"百分比={battery_pct:.1f}%, "
                    f"样本数={voltage_samples}, "
                    f"低电压模式={'是' if self.low_voltage_mode else '否'}"
                )
            
            msg.battery_current = getattr(self.usv_battery, 'current', 0.0)
            msg.battery_remaining = getattr(self.usv_battery, 'charge', 0.0)
            msg.power_supply_status = getattr(self.usv_battery, 'power_supply_status', 0)
            msg.low_voltage_mode = self.low_voltage_mode  # 设置低电压模式标志
            
            # 计算电池节数（假设单节电压3.7V）
            if msg.battery_voltage > 1.0:
                msg.battery_cell_count = int(msg.battery_voltage / 3.7 + 0.5)
            else:
                msg.battery_cell_count = 0

            # ==================== 位置和姿态 ====================
            try:
                msg.position = self.usv_pose.pose.position
            except Exception:
                msg.position = Point()
            
            # 完整的欧拉角
            # 优先使用罗盘航向，如果不可用则尝试从四元数计算
            compass_available = self.has_compass and (time.time() - self.last_compass_time) < self.data_timeout
            
            if compass_available:
                # 使用罗盘航向（最可靠的来源）
                compass_deg = self.usv_compass_hdg
                # 罗盘航向转换为 yaw（弧度，ENU坐标系）
                # 罗盘: 0°=北, 90°=东, 180°=南, 270°=西
                # ENU: 0=东(+X), 90°=北(+Y), 180°=西, 270°=南
                # 转换: yaw_enu = 90 - compass_deg
                import math
                yaw_deg = 90.0 - compass_deg
                msg.yaw = math.radians(yaw_deg)
                
                # 规范化yaw到[-π, π]
                while msg.yaw > math.pi:
                    msg.yaw -= 2 * math.pi
                while msg.yaw < -math.pi:
                    msg.yaw += 2 * math.pi
                
                # roll 和 pitch 从四元数获取（如果可用）
                try:
                    quaternion = (
                        self.usv_pose.pose.orientation.x,
                        self.usv_pose.pose.orientation.y,
                        self.usv_pose.pose.orientation.z,
                        self.usv_pose.pose.orientation.w
                    )
                    # 检查四元数是否有效（不是单位四元数）
                    if abs(quaternion[0]) > 0.01 or abs(quaternion[1]) > 0.01:
                        roll, pitch, _ = euler_from_quaternion(quaternion)
                        msg.roll = float(roll)
                        msg.pitch = float(pitch)
                    else:
                        msg.roll = 0.0
                        msg.pitch = 0.0
                except Exception:
                    msg.roll = 0.0
                    msg.pitch = 0.0
            else:
                # 回退：尝试从四元数计算完整欧拉角
                try:
                    quaternion = (
                        self.usv_pose.pose.orientation.x,
                        self.usv_pose.pose.orientation.y,
                        self.usv_pose.pose.orientation.z,
                        self.usv_pose.pose.orientation.w
                    )
                    roll, pitch, yaw = euler_from_quaternion(quaternion)
                    msg.roll = float(roll)
                    msg.pitch = float(pitch)
                    msg.yaw = float(yaw)
                    
                    # 规范化yaw到[-π, π]
                    import math
                    while msg.yaw > math.pi:
                        msg.yaw -= 2 * math.pi
                    while msg.yaw < -math.pi:
                        msg.yaw += 2 * math.pi
                        
                except Exception as e:
                    msg.roll = 0.0
                    msg.pitch = 0.0
                    msg.yaw = 0.0
            
            # 高度信息
            try:
                msg.altitude_relative = getattr(self.usv_altitude, 'relative', 0.0)
                msg.altitude_amsl = getattr(self.usv_altitude, 'amsl', 0.0)
            except Exception:
                msg.altitude_relative = 0.0
                msg.altitude_amsl = 0.0

            # ==================== 速度信息 ====================
            try:
                msg.velocity = self.usv_velocity.twist
                
                # 计算地速（水平速度模）
                vx = msg.velocity.linear.x
                vy = msg.velocity.linear.y
                vz = msg.velocity.linear.z
                msg.ground_speed = sqrt(vx*vx + vy*vy)
                msg.climb_rate = vz
                
                # 航向计算：优先使用罗盘，其次速度方向，最后使用yaw
                compass_available = self.has_compass and (time.time() - self.last_compass_time) < self.data_timeout
                
                if compass_available:
                    # 优先：直接使用罗盘航向（最可靠）
                    msg.heading = self.usv_compass_hdg
                elif msg.ground_speed > 0.1:
                    # 次选：高速时使用速度方向计算航向
                    heading_rad = atan2(vy, vx)
                    msg.heading = degrees(heading_rad)
                    if msg.heading < 0:
                        msg.heading += 360.0
                else:
                    # 最后：低速时使用偏航角作为航向
                    msg.heading = degrees(msg.yaw)
                    if msg.heading < 0:
                        msg.heading += 360.0
                
                msg.air_speed = 0.0  # 水面艇无空速
                
            except Exception as e:
                msg.ground_speed = 0.0
                msg.air_speed = 0.0
                msg.climb_rate = 0.0
                msg.heading = 0.0

            # ==================== GPS 信息 ====================
            try:
                # 优先使用 GPS RAW 的 fix_type（更准确，包含 3D Fix 等详细状态）
                # GPS RAW fix_type 定义：
                # 0: NO_FIX, 1: NO_FIX, 2: 2D_FIX, 3: 3D_FIX, 
                # 4: DGPS, 5: RTK_FLOAT, 6: RTK_FIXED
                if self.has_gps_raw and (time.time() - self.last_gps_raw_time) < self.data_timeout:
                    msg.gps_fix_type = int(self.usv_gps_raw.fix_type)
                else:
                    # 降级使用 NavSatFix 的 status.status（值域不同，需要映射）
                    # NavSatFix status: -1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX
                    # 映射到 GPS RAW 格式：-1→0, 0→3, 1→4, 2→6
                    gps_status = getattr(self.usv_gps, 'status', None)
                    if gps_status:
                        nav_status = getattr(gps_status, 'status', -1)
                        if nav_status < 0:
                            msg.gps_fix_type = 0  # NO_FIX
                        elif nav_status == 0:
                            msg.gps_fix_type = 3  # 假定为 3D_FIX
                        elif nav_status == 1:
                            msg.gps_fix_type = 4  # SBAS → DGPS
                        else:
                            msg.gps_fix_type = 6  # GBAS → RTK_FIXED
                    else:
                        msg.gps_fix_type = 0
                
                # 从 GPS RAW 获取 HDOP/VDOP（优先）和卫星数
                if self.has_gps_raw and (time.time() - self.last_gps_raw_time) < self.data_timeout:
                    # GPS RAW 的 eph/epv 是 uint16，单位通常是 cm (即 0.01)
                    # 如果值为 UINT16_MAX (65535)，表示未知
                    raw_eph = self.usv_gps_raw.eph
                    raw_epv = self.usv_gps_raw.epv
                    
                    if raw_eph < 65535:
                        msg.gps_eph = float(raw_eph) / 100.0  # 转换为米
                    else:
                        msg.gps_eph = 99.0  # 未知值
                    
                    if raw_epv < 65535:
                        msg.gps_epv = float(raw_epv) / 100.0  # 转换为米
                    else:
                        msg.gps_epv = 99.0  # 未知值
                    
                    # 卫星数
                    msg.gps_satellites_visible = int(self.usv_gps_raw.satellites_visible)
                else:
                    # 如果没有 GPS RAW，尝试从 NavSatFix 的 position_covariance 计算
                    cov = getattr(self.usv_gps, 'position_covariance', [])
                    if len(cov) >= 9 and (cov[0] + cov[4]) > 0:
                        # HDOP ≈ sqrt(cov[0] + cov[4])
                        msg.gps_eph = float(sqrt(cov[0] + cov[4]))
                        msg.gps_epv = float(sqrt(cov[8]) if cov[8] > 0 else 99.0)
                    else:
                        msg.gps_eph = 99.0
                        msg.gps_epv = 99.0
                    
                    msg.gps_satellites_visible = 0
                
            except Exception as e:
                self.get_logger().error(f'GPS信息处理错误: {e}', throttle_duration_sec=10.0)
                msg.gps_fix_type = 0
                msg.gps_satellites_visible = 0
                msg.gps_eph = 99.0
                msg.gps_epv = 99.0
                msg.gps_satellites_visible = 0
                msg.gps_eph = 99.0
                msg.gps_epv = 99.0

            # ==================== 系统健康 ====================
            # 温度（使用缓存）
            if current_time - self.last_temp_update > self.temp_cache_interval:
                self.cached_temperature = self.get_temperature()
                self.last_temp_update = current_time
            msg.temperature = self.cached_temperature
            
            # CPU和内存使用率（可选，避免频繁调用）
            if self.enable_system_monitor:
                try:
                    msg.cpu_percent = psutil.cpu_percent(interval=None)
                    msg.memory_percent = psutil.virtual_memory().percent
                except Exception:
                    msg.cpu_percent = 0.0
                    msg.memory_percent = 0.0
            else:
                msg.cpu_percent = 0.0
                msg.memory_percent = 0.0
            
            # 系统运行时间
            msg.uptime_sec = int(current_time - self.start_time)

            # ==================== 发布消息前验证 ====================
            # 验证关键字段类型
            if not isinstance(msg.gps_satellites_visible, int):
                self.get_logger().error(f'gps_satellites_visible类型错误: {type(msg.gps_satellites_visible)}')
                msg.gps_satellites_visible = int(msg.gps_satellites_visible) if msg.gps_satellites_visible is not None else 0

            # ==================== 发布消息 ====================
            self.state_publisher.publish(msg)
            
            # 发布温度到单独的topic（兼容旧代码）
            temp_msg = Float32()
            temp_msg.data = msg.temperature
            self.temperature_publisher.publish(temp_msg)
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'状态信息发布过程中发生错误: {e}', throttle_duration_sec=5.0)
            self.get_logger().error(f'完整堆栈:\n{traceback.format_exc()}', throttle_duration_sec=10.0)

    def get_temperature(self):
        """获取系统温度
        
        Returns:
            float: 系统温度值（毫摄氏度）
            
        Raises:
            IOError: 无法读取温度文件时抛出
        """
        # 尝试多个常见路径以提高兼容性，若均失败则返回0.0
        candidates = [
            '/sys/class/thermal/thermal_zone0/temp',
            '/sys/class/hwmon/hwmon0/temp1_input'
        ]
        for path in candidates:
            try:
                with open(path, 'r') as f:
                    raw = f.read().strip()
                    if not raw:
                        continue
                    val = float(raw)
                    return val
            except Exception:
                continue
        # 如果所有路径均失败，记录并返回0.0
        self.get_logger().warn('无法读取系统温度，返回0.0')
        return 0.0


def main():
    """主函数"""
    rclpy.init()
    node = UsvStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()