"""
地面站GUI ROS2节点实现文件
该文件实现了地面站GUI与USV系统通信的核心功能
包括状态监控、命令发送、导航控制等功能
"""

import json
from datetime import datetime
import rclpy 
from rclpy.node import Node
from rclpy.parameter import Parameter
from common_interfaces.msg import UsvStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import queue
import threading
from std_msgs.msg import String, Float32
import tf2_ros
from rcl_interfaces.msg import Log

# 导入分解后的模块
from .usv_manager import UsvManager
from .cluster_controller import ClusterController
from .command_processor import CommandProcessor
from .led_infection import LedInfectionHandler
from .formation_controller import FormationManager, FormationType
from .event_decoder import EventDecoder  # 导入事件解码器
from .px4_command_interface import Px4CommandInterface  # PX4 命令接口
from .navigation_handler import NavigationHandler  # 导航处理器
from .sensor_status_handler import SensorStatusHandler  # 传感器状态处理器
from .discovery_handler import DiscoveryHandler  # USV 发现处理器
from .system_command_handler import SystemCommandHandler  # 系统命令处理器

# 导入线程安全工具
from common_utils import ThreadSafeDict


class GroundStationNode(Node):
    """
    地面站节点类
    继承自rclpy.Node，实现地面站GUI与USV系统通信的核心功能
    """

    # 常量定义
    INFECTION_DISTANCE_SQUARED = 4.0  # 2米距离的平方
    DEFAULT_STEP_TIMEOUT = 25.0  # 默认步骤超时时间(秒) - 等待USV响应的时间，增加到25秒避免误判
    DEFAULT_MAX_RETRIES = 3      # 默认最大重试次数 - 增加到3次，给USV更多机会
    INFECTION_CHECK_PERIOD = 2.0 # 传染检查周期(秒)，增加周期减少CPU占用
    NAMESPACE_UPDATE_PERIOD = 2.0 # 命名空间更新周期(秒)，从 5.0 减少到 2.0，加快离线检测
    CLUSTER_TARGET_PUBLISH_PERIOD = 5 # 集群目标发布周期(秒)，增加周期减少CPU占用
    MIN_ACK_RATE_FOR_PROCEED = 0.8  # 最小确认率阈值，超过此值可进入下一步
    PREARM_WARNING_EXPIRY = 15.0  # PreArm 报警保留时长（秒）
    
    def __init__(self, signal, append_info=None, append_warning=None):
        """
        初始化地面站节点
        
        Args:
            signal: ROS信号对象，用于与GUI界面通信
            append_info: GUI 信息输出回调函数（可选）
            append_warning: GUI 警告输出回调函数（可选）
        """
        super().__init__('groundstationnode')  # 调用父类Node的初始化方法，设置节点名称为'groundstationnode'
        self.ros_signal = signal  # 保存ROS信号对象引用
        self.append_info = append_info if append_info else lambda x: None  # GUI 输出回调
        self.append_warning = append_warning if append_warning else lambda x: None  # GUI 警告回调
        self.qos_a = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)  # 创建QoS配置对象，深度为10，可靠性策略为可靠传输

        # 声明参数（必须在使用前声明）
        self.declare_parameter('discovery_interval', 3.0)  # 动态发现间隔（秒）
        self.declare_parameter('step_timeout', float(self.DEFAULT_STEP_TIMEOUT))
        self.declare_parameter('max_retries', int(self.DEFAULT_MAX_RETRIES))
        self.declare_parameter('min_ack_rate_for_proceed', float(self.MIN_ACK_RATE_FOR_PROCEED))
        self.declare_parameter('offline_grace_period', 5.0)
        self.declare_parameter('ack_resend_interval', 2.0)
        self.declare_parameter('cluster_action_timeout', 300.0)
        # 集群任务中 roll/pitch 的姿态动作下发时长（秒）；>0 表示持续秒数
        self.declare_parameter('attitude_command_duration', 1.0)
        # 方案A：先导航，再在接近目标时触发一次姿态动作（米）
        self.declare_parameter('attitude_trigger_distance', 1.0)
        # 导航到达判定阈值（按平台模式 2d/3d）
        self.declare_parameter('nav_reach_xy_threshold_2d', 1.0)
        self.declare_parameter('nav_reach_z_threshold_2d', 0.0)  # 2D 默认不需要 Z
        self.declare_parameter('nav_reach_xy_threshold_3d', 1.0)
        self.declare_parameter('nav_reach_z_threshold_3d', 1.0)
        self.declare_parameter('area_center_x', 0.0)
        self.declare_parameter('area_center_y', 0.0)
        self.declare_parameter('area_center_z', 0.0)
        self.declare_parameter('area_center_frame', 'map')
        
        # 围栏参数
        self.declare_parameter('fence_type', 0)  # 0: Cylinder, 1: Box
        self.declare_parameter('fence_radius', 50.0)
        self.declare_parameter('fence_length', 100.0)
        self.declare_parameter('fence_width', 100.0)
        self.declare_parameter('fence_height', 10.0)

        # 初始化子模块
        self.usv_manager = UsvManager(self)
        self.cluster_controller = ClusterController(self)
        self.command_processor = CommandProcessor(self)
        self.led_infection_handler = LedInfectionHandler(self)
        self.formation_manager = FormationManager(self)
        
        # 初始化模块化处理器
        self.navigation_handler = NavigationHandler(self, self.usv_manager, signal)
        self.sensor_handler = SensorStatusHandler(self, signal)
        self.discovery_handler = DiscoveryHandler(self, self.usv_manager, signal)
        self.system_command_handler = SystemCommandHandler(self, signal)
        
        # 设置 discovery_handler 对 sensor_handler 的引用
        self.discovery_handler.set_sensor_handler(self.sensor_handler)
        
        # 注册 UsvManager 的日志消息和事件回调
        self.usv_manager.set_callbacks(
            on_log_message=self._on_usv_log_message,
            on_event=self._on_usv_event
        )
        
        # 初始化事件解码器
        self.event_decoder = EventDecoder(self.get_logger())
        
        # 订阅 /rosout 以捕获 PX4 事件日志
        # 用于解码 FCU 事件消息
        from rclpy.qos import qos_profile_system_default
        rosout_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            rosout_qos
        )
        self.get_logger().info("已订阅 /rosout 用于事件解码")
        
        # USV rosout 订阅列表（将在 _register_new_usv 中动态初始化）
        self.usv_rosout_subs = []

        # 导航目标信息缓存（用于导航面板显示）
        self._usv_nav_target_cache = ThreadSafeDict()
        # 导航到达阈值缓存（用于导航面板显示，按 USV 保存最后一次下发值）
        self._nav_arrival_threshold_cache = ThreadSafeDict()
        self._user_cruise_speed_override = None

        # 每艇平台模式与异构机型配置（来自 usv_fleet.yaml，可缺省）
        self._usv_platform_mode = ThreadSafeDict()
        self._usv_vehicle_type = ThreadSafeDict()
        self._usv_safety_radius = ThreadSafeDict()
        self._usv_default_altitude = ThreadSafeDict()
        self._usv_landing_altitude = ThreadSafeDict()
        self._usv_reach_xy = ThreadSafeDict()
        self._usv_reach_z = ThreadSafeDict()
        
        # 新增: 基于话题的导航管理
        self._next_goal_id = 1  # 目标ID生成器
        self._goal_id_lock = threading.Lock()  # 目标ID锁
        self._goal_to_usv = ThreadSafeDict()  # 目标ID到USV的映射 {goal_id: usv_id} (线程安全)

        # 初始化USV状态和目标管理相关变量
        self.usv_states = ThreadSafeDict()  # USV状态字典 (线程安全)
        self.last_ns_list = []  # 上次命名空间列表
        self.is_runing = False
        self.run_step = 0  # 当前运行步骤
        self.usv_target_number = 0  # USV目标编号
        self.max_step = 1  # 最大步骤数
        self.current_targets = []  # 当前目标列表

        # 初始化发布队列和线程
        self.publish_queue = queue.Queue(maxsize=100)  # 创建消息发布队列，限制最大大小
        self.publish_thread = threading.Thread(target=self.process_publish_queue, daemon=True)  # 创建发布线程，设置为守护线程
        self.publish_thread.start()  # 启动发布线程

        # ========== 从参数服务器读取参数值 ==========
        # 注意：参数已在前面声明，这里只是读取值
        try:
            self._step_timeout = float(self.get_parameter('step_timeout').get_parameter_value().double_value)
        except Exception:
            self._step_timeout = float(self.DEFAULT_STEP_TIMEOUT)
        try:
            self._max_retries = int(self.get_parameter('max_retries').get_parameter_value().integer_value)
        except Exception:
            self._max_retries = int(self.DEFAULT_MAX_RETRIES)
        try:
            self.MIN_ACK_RATE_FOR_PROCEED = float(self.get_parameter('min_ack_rate_for_proceed').get_parameter_value().double_value)
        except Exception:
            pass
        try:
            self._ack_resend_interval = float(self.get_parameter('ack_resend_interval').get_parameter_value().double_value)
        except Exception:
            self._ack_resend_interval = 2.0
        try:
            self._cluster_action_timeout = float(self.get_parameter('cluster_action_timeout').get_parameter_value().double_value)
        except Exception:
            self._cluster_action_timeout = 300.0

        try:
            self._attitude_command_duration = float(self.get_parameter('attitude_command_duration').get_parameter_value().double_value)
        except Exception:
            self._attitude_command_duration = 1.0

        try:
            self._attitude_trigger_distance = float(self.get_parameter('attitude_trigger_distance').get_parameter_value().double_value)
        except Exception:
            self._attitude_trigger_distance = 1.0

        # 将最新参数同步给集群控制器
        self.cluster_controller.configure(
            resend_interval=self._ack_resend_interval,
            action_timeout=self._cluster_action_timeout,
        )

        # 读取 area_center 参数
        try:
            ax = float(self.get_parameter('area_center_x').get_parameter_value().double_value)
            ay = float(self.get_parameter('area_center_y').get_parameter_value().double_value)
            az = float(self.get_parameter('area_center_z').get_parameter_value().double_value)
            afr = str(self.get_parameter('area_center_frame').get_parameter_value().string_value)
            self._area_center = {'x': ax, 'y': ay, 'z': az, 'frame': afr}
        except Exception:
            self._area_center = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'frame': 'map'}

        # 围栏配置
        try:
            ft = int(self.get_parameter('fence_type').get_parameter_value().integer_value)
            fr = float(self.get_parameter('fence_radius').get_parameter_value().double_value)
            fl = float(self.get_parameter('fence_length').get_parameter_value().double_value)
            fw = float(self.get_parameter('fence_width').get_parameter_value().double_value)
            fh = float(self.get_parameter('fence_height').get_parameter_value().double_value)
            self._fence_config = {
                'type': ft, 'radius': fr, 'length': fl, 'width': fw, 'height': fh
            }
        except Exception:
            self._fence_config = {
                'type': 0,      # 0: 圆柱形, 1: 长方体
                'radius': 50.0, # 圆柱半径
                'length': 100.0,# 长方体长度 (X)
                'width': 100.0, # 长方体宽度 (Y)
                'height': 10.0  # 高度 (Z 范围)
            }

        try:
            self._ns_offline_grace_period = float(self.get_parameter('offline_grace_period').get_parameter_value().double_value)
        except Exception:
            self._ns_offline_grace_period = 5.0  # 从 20.0 减少到 5.0 秒，加快移除速度
        # 离线判定的宽限期（秒），在此时间内即便 ROS 图暂时看不到也不移除

        # 用于通知后台线程退出的事件
        self._stop_event = threading.Event()

        # 导航发送锁 (线程安全)
        self._send_locks = ThreadSafeDict()  # 每个 USV 的发送锁，防止并发冲突

        # 初始化传染机制相关变量 (线程安全)
        self._usv_led_modes = ThreadSafeDict()  # USV LED模式字典
        self._usv_infecting = set()  # 正在传染的USV集合
        # 维护本地 LED 状态 (线程安全)
        self._usv_current_led_state = ThreadSafeDict() # 维护 USV ID -> {'mode': str, 'color': [r,g,b]} 
        self._usv_infection_sources = ThreadSafeDict()  # 记录被传染USV的源映射
        # LED传染模式开关（默认关闭）
        self._led_infection_enabled = False
        
        # 随机运行模式状态
        self.random_run_enabled = False
        self.random_run_timer = None
        self.random_run_interval = 5.0  # 检查周期(秒)
     
        # 初始化命名空间检测历史记录
        self._ns_detection_history = []  # 用于存储命名空间检测历史记录的列表
        # 记录每个 USV 最后一次收到状态消息的时间戳（秒）(线程安全)
        self._ns_last_seen = ThreadSafeDict()
        
        # 已注册的 USV 集合（用于动态发现去重）
        self._registered_usvs = set()
        
        # 动态发现的 USV 列表（使用 discovery_handler 管理）
        self._discovered_usv_list = []

        # 获取动态发现配置
        self._discovery_interval = self.get_parameter('discovery_interval').value

        # 创建定时器
        # USV 话题可用性检查（使用 discovery_handler）
        self.ns_timer = self.create_timer(5.0, self._check_availability_wrapper)
        self.target_timer = self.create_timer(self.CLUSTER_TARGET_PUBLISH_PERIOD, self.publish_cluster_targets_callback)  # 集群目标发布定时器
        self.infect_check_timer = self.create_timer(self.INFECTION_CHECK_PERIOD, self.check_usv_infect)  # 传染检查定时器
        # 添加高频状态推送定时器，确保 Ready 检查等信息能快速更新到 GUI
        self.state_push_timer = self.create_timer(0.2, self.push_state_updates)  # 200ms = 5Hz
        
        # 动态发现定时器（使用 discovery_handler）
        self.discovery_timer = self.create_timer(self._discovery_interval, self._discover_wrapper)
        self.get_logger().info("🔍 动态发现模式已启用")
        
        # 预探测远程 USV（触发 Zenoh 桥接的 interest-based routing）
        self._probe_remote_usvs_from_config()

        # TF2: Buffer/Listener for coordinate transforms
        # 注意：使用 BEST_EFFORT QoS 以匹配 USV 发布的 /tf 话题
        try:
            self.tf_buffer = tf2_ros.Buffer()
            # 创建自定义 QoS 用于 TF 订阅
            tf_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self, qos=tf_qos
            )
            self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        except Exception as e:
            self.get_logger().warn(f"TF2 初始化失败: {e}")
            self.tf_buffer = None
            self.tf_listener = None
            self.static_broadcaster = None

        # 用于接收来自 GUI 的字符串命令的队列（由 GUI 线程快速入队，节点线程定期处理）
        self._incoming_str_commands = queue.Queue(maxsize=200)
        # 在节点线程中周期性处理入队的字符串命令，避免在 GUI 线程执行节点逻辑
        self._str_command_timer = self.create_timer(0.1, self._process_incoming_str_commands)

    def pause_cluster_task_callback(self):
        """处理来自 GUI 的集群暂停请求。"""
        self.get_logger().info("接收到集群暂停请求")
        self.cluster_controller.pause_cluster_task()

    def resume_cluster_task_callback(self):
        """处理来自 GUI 的集群恢复请求。"""
        if not self.cluster_controller.is_cluster_task_paused():
            self.get_logger().warn("集群任务未处于暂停状态，忽略恢复请求")
            return
        self.get_logger().info("接收到集群恢复请求")
        self.cluster_controller.resume_cluster_task()

    def stop_cluster_task_callback(self):
        """处理来自 GUI 的集群停止请求。"""
        self.get_logger().info("接收到集群停止请求")
        self.cluster_controller.stop_cluster_task("GUI 手动停止")

    def set_nav_arrival_threshold(self, usv_id_list, threshold_m):
        """设置 USV 导航到达阈值（米）。"""
        try:
            threshold = float(threshold_m)
        except Exception:
            self.get_logger().warn(f"nav_arrival_threshold 非法: {threshold_m}")
            return False
        if threshold <= 0.0 or not usv_id_list:
            return False
        sent = 0
        updated_any_state = False
        for usv_id in usv_id_list:
            pub = self.usv_manager.nav_arrival_threshold_pubs.get(usv_id)
            if pub is None:
                continue
            msg = Float32()
            msg.data = threshold
            try:
                self.publish_queue.put((pub, msg))
                sent += 1
                self._nav_arrival_threshold_cache[usv_id] = threshold
                if usv_id in self.usv_states:
                    try:
                        self.usv_states[usv_id]['nav_arrival_threshold'] = threshold
                        updated_any_state = True
                    except Exception:
                        pass
            except Exception as e:
                self.get_logger().warn(f"下发 nav_arrival_threshold 到 {usv_id} 失败: {e}")
        if updated_any_state:
            try:
                self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))
            except Exception:
                pass
        return sent > 0

    def set_nav_switch_threshold(self, usv_id_list, threshold_m):
        """设置 USV 导航切换阈值（米）。"""
        try:
            threshold = float(threshold_m)
        except Exception:
            return False
        if threshold <= 0.0 or not usv_id_list:
            return False
        sent = 0
        for usv_id in usv_id_list:
            pub = self.usv_manager.nav_switch_threshold_pubs.get(usv_id)
            if pub is None:
                continue
            msg = Float32()
            msg.data = threshold
            try:
                self.publish_queue.put((pub, msg))
                sent += 1
            except Exception as e:
                self.get_logger().warn(f"下发 nav_switch_threshold 到 {usv_id} 失败: {e}")
        return sent > 0

    def set_nav_smooth_navigation(self, usv_id_list, enabled: bool):
        """设置 USV 平滑导航开关。"""
        if not usv_id_list:
            return False
        from std_msgs.msg import Bool
        sent = 0
        for usv_id in usv_id_list:
            pub = self.usv_manager.nav_smooth_navigation_pubs.get(usv_id)
            if pub is None:
                continue
            msg = Bool()
            msg.data = bool(enabled)
            try:
                self.publish_queue.put((pub, msg))
                sent += 1
            except Exception as e:
                self.get_logger().warn(f"下发 smooth_navigation 到 {usv_id} 失败: {e}")
        return sent > 0

    def set_nav_settings(self, usv_id_list, settings: dict):
        """批量设置导航参数。"""
        results = {}
        if 'nav_arrival_threshold' in settings:
            results['arrival'] = self.set_nav_arrival_threshold(usv_id_list, settings['nav_arrival_threshold'])
        if 'switch_threshold' in settings:
            results['switch'] = self.set_nav_switch_threshold(usv_id_list, settings['switch_threshold'])
        if 'smooth_navigation' in settings:
            results['smooth'] = self.set_nav_smooth_navigation(usv_id_list, settings['smooth_navigation'])
        return all(results.values()) if results else False

    def set_velocity_settings(self, usv_id_list, settings: dict, *, from_user_dialog: bool = False):
        """批量设置速度控制器参数。"""
        if not usv_id_list:
            return False
        param_mapping = {
            'cruise_speed': (self.usv_manager.velocity_cruise_speed_pubs, 'cruise_speed'),
            'max_angular_velocity': (self.usv_manager.velocity_max_angular_pubs, 'max_angular_velocity'),
            'goal_tolerance': (self.usv_manager.velocity_goal_tolerance_pubs, 'goal_tolerance'),
            'switch_tolerance': (self.usv_manager.velocity_switch_tolerance_pubs, 'switch_tolerance'),
        }
        if from_user_dialog and 'cruise_speed' in settings:
            try:
                cs = float(settings['cruise_speed'])
                if cs > 0.0:
                    self._user_cruise_speed_override = cs
            except (ValueError, TypeError):
                pass
        total_sent = 0
        for param_name, (pubs_dict, settings_key) in param_mapping.items():
            if settings_key not in settings:
                continue
            try:
                value = float(settings[settings_key])
            except (ValueError, TypeError):
                continue
            if value <= 0.0:
                continue
            sent = 0
            for usv_id in usv_id_list:
                pub = pubs_dict.get(usv_id)
                if pub is None:
                    continue
                msg = Float32()
                msg.data = value
                try:
                    self.publish_queue.put((pub, msg))
                    sent += 1
                except Exception as e:
                    self.get_logger().warn(f"下发 {param_name} 到 {usv_id} 失败: {e}")
            total_sent += sent
        return total_sent > 0

    # ==================== 编队模式回调 ====================

    def start_formation_callback(self, group_configs: list):
        """处理多编队组启动请求（含 3D 载体起飞预高度门控检查）。"""
        involved_ids = []
        for g in (group_configs or []):
            if isinstance(g, dict):
                lid = str(g.get('leader_id', '')).lstrip('/')
                if lid:
                    involved_ids.append(lid)
                for f in (g.get('follower_ids') or g.get('followers') or []):
                    fid = str(f.get('follower_id', f) if isinstance(f, dict) else f).lstrip('/')
                    if fid:
                        involved_ids.append(fid)
        ok, reasons = self.validate_3d_cluster_takeoff_ready(involved_ids or None)
        if not ok:
            warn_msg = "🚫 3D 载体未达到设定起飞预高度或处于保护锁止，禁止启动编队：\n  • " + "\n  • ".join(reasons)
            self.get_logger().warn(warn_msg)
            self.ros_signal.node_warning.emit(warn_msg)
            return

        if self.formation_manager.is_any_active():
            self.formation_manager.stop_all("重新配置编队")
        success = self.formation_manager.configure_groups(group_configs)
        if success:
            follower_ids = self.formation_manager.get_all_follower_ids()
            self.cluster_controller.set_excluded_ids(follower_ids)
            self.get_logger().info(f"编队已启动，{len(follower_ids)} 艘跟随者已排除在集群任务之外")
            self.formation_manager.start_all()

    def stop_formation_callback(self):
        """处理编队停止请求。"""
        self.formation_manager.stop_all("GUI 手动停止")
        self.cluster_controller.clear_excluded_ids()
        self.get_logger().info("编队已停止，集群任务排除列表已清除")

    def change_formation_type_callback(self, formation_type: int):
        """处理编队队形切换 — 应用到所有活跃组。"""
        if self.formation_manager.is_any_active():
            self.formation_manager.update_formation_type_all(FormationType(formation_type))

    def change_formation_spacing_callback(self, along: float, cross: float):
        """处理编队间距变更 — 应用到所有活跃组。"""
        if self.formation_manager.is_any_active():
            self.formation_manager.update_spacing_all(along, cross)

    # 在独立线程中异步处理消息发布队列
    def process_publish_queue(self):
        """
        在独立线程中处理消息发布队列，避免阻塞主ROS循环
        
        该方法在单独的线程中运行，从发布队列中取出消息并发布
        这样可以避免在主ROS循环中进行耗时的发布操作
        """
        # 当ROS仍在运行且未收到停止事件时持续处理队列
        while rclpy.ok() and not getattr(self, '_stop_event', threading.Event()).is_set():
            try:
                # 从队列中获取发布任务，超时时间为1秒
                pub, msg = self.publish_queue.get(timeout=1.0)
                # 发布消息
                pub.publish(msg)
                # 记录调试日志
                self.get_logger().debug(f"发布消息到 {pub.topic}")
                # 标记任务完成
                self.publish_queue.task_done()
            # 如果队列为空，继续循环
            except queue.Empty:
                continue
            # 如果队列已满，丢弃旧消息以避免内存堆积
            except queue.Full:
                self.get_logger().warn("发布队列已满，丢弃旧消息")
                try:
                    # 尝试清空队列中的旧消息
                    while not self.publish_queue.empty():
                        self.publish_queue.get_nowait()
                        self.publish_queue.task_done()
                except queue.Empty:
                    pass
            # 捕获其他异常并记录错误日志
            except Exception as e:
                self.get_logger().error(f"发布消息失败: {e}")
        # 线程退出前做简单清理（如果队列中还有消息，这里不再处理）
        self.get_logger().debug('process_publish_queue 线程退出')

    # Boot Pose 功能已删除 - 系统直接使用 USV 当前状态进行坐标转换

    def shutdown(self):
        """
        优雅停止 GroundStationNode 的后台线程并做最小清理。

        说明：该方法不会销毁节点本身（destroy_node），调用者应在需要时负责调用
        node.destroy_node() 与 rclpy.shutdown()。
        """
        self.get_logger().info('GroundStationNode 正在关闭，通知后台线程退出')
        try:
            # 通知线程退出并等待其结束
            self._stop_event.set()
            if hasattr(self, 'publish_thread') and self.publish_thread.is_alive():
                self.publish_thread.join(timeout=2.0)
        except Exception as e:
            self.get_logger().warn(f'关闭后台线程时发生异常: {e}')
    
    # =========================================================================
    # 处理器包装方法（用于定时器回调）
    # =========================================================================
    
    def _discover_wrapper(self):
        """动态发现包装器 - 调用 discovery_handler"""
        try:
            self.discovery_handler.discover_usvs()
            # 同步状态到本地变量（兼容现有代码）
            self._discovered_usv_list = self.discovery_handler.get_discovered_usvs()
            self.usv_states = self.discovery_handler._usv_states
        except Exception as e:
            self.get_logger().error(f"动态发现失败: {e}")
    
    def _check_availability_wrapper(self):
        """可用性检查包装器 - 调用 discovery_handler"""
        try:
            self.discovery_handler.check_availability()
            # 同步状态
            self.usv_states = self.discovery_handler._usv_states
        except Exception as e:
            self.get_logger().error(f"可用性检查失败: {e}")
    
    def check_usv_topics_availability(self):
        """[已迁移到 discovery_handler] 保留用于兼容性"""
        self._check_availability_wrapper()
    
    def discover_new_usvs(self):
        """[已迁移到 discovery_handler] 保留用于兼容性"""
        self._discover_wrapper()
    
    def _probe_remote_usvs_from_config(self):
        """
        从配置文件读取 USV 列表并进行预探测
        
        用于触发 Zenoh Bridge 的 interest-based routing，
        使远程 USV 的话题能够被桥接到 GS 端。
        """
        import os
        import yaml
        
        # 尝试多个可能的配置文件路径
        possible_paths = [
            os.path.expanduser('~/usv_workspace/install/gs_bringup/share/gs_bringup/config/usv_fleet.yaml'),
            os.path.expanduser('~/usv_workspace/src/gs_bringup/config/usv_fleet.yaml'),
        ]
        
        usv_ids = []
        for config_path in possible_paths:
            if os.path.exists(config_path):
                try:
                    with open(config_path, 'r') as f:
                        config = yaml.safe_load(f)
                    
                    fleet_config = config.get('usv_fleet', {})
                    for usv_id, usv_config in fleet_config.items():
                        if usv_config.get('enabled', False):
                            usv_ids.append(usv_id)

                        # 读取平台模式与异构机型参数（可选）
                        try:
                            pm = str(usv_config.get('platform_mode', '')).strip().lower()
                            if pm in ('2d', '3d'):
                                self._usv_platform_mode[usv_id] = pm
                            vt = str(usv_config.get('vehicle_type', '')).strip().lower()
                            if vt:
                                self._usv_vehicle_type[usv_id] = vt
                            if 'safety_radius' in usv_config:
                                self._usv_safety_radius[usv_id] = float(usv_config['safety_radius'])
                            if 'default_altitude' in usv_config:
                                self._usv_default_altitude[usv_id] = float(usv_config['default_altitude'])
                            if 'takeoff_altitude' in usv_config and float(usv_config['takeoff_altitude']) > 0.0:
                                self._usv_default_altitude[usv_id] = float(usv_config['takeoff_altitude'])
                            if 'landing_altitude' in usv_config and float(usv_config['landing_altitude']) >= 0.0:
                                self._usv_landing_altitude[usv_id] = float(usv_config['landing_altitude'])
                            if 'reach_xy_threshold' in usv_config:
                                self._usv_reach_xy[usv_id] = float(usv_config['reach_xy_threshold'])
                            if 'reach_z_threshold' in usv_config:
                                self._usv_reach_z[usv_id] = float(usv_config['reach_z_threshold'])
                        except Exception:
                            pass
                    
                    self.get_logger().info(f"📋 从配置文件加载 USV 列表: {usv_ids}")
                    break
                except Exception as e:
                    self.get_logger().warning(f"读取 USV 配置文件失败: {e}")
        
        # 如果没有从配置文件读取到，使用默认探测列表
        if not usv_ids:
            usv_ids = ['usv_01', 'usv_02', 'usv_03']
            self.get_logger().info(f"📋 使用默认 USV 探测列表: {usv_ids}")
        
        # 调用 discovery_handler 进行预探测
        self.discovery_handler.probe_remote_usvs(usv_ids)
    
    def _register_new_usv(self, usv_id: str):
        """[已迁移到 discovery_handler] 保留用于兼容性"""
        self.discovery_handler._register_usv(usv_id)
        # 同步状态
        self._discovered_usv_list = self.discovery_handler.get_discovered_usvs()
        self.usv_states = self.discovery_handler._usv_states
    
    def _unregister_usv(self, usv_id: str):
        """[已迁移到 discovery_handler] 保留用于兼容性"""
        self.discovery_handler.unregister_usv(usv_id)
        # 同步状态
        self._discovered_usv_list = self.discovery_handler.get_discovered_usvs()
        self.usv_states = self.discovery_handler._usv_states

    # =========================================================================
    # 导航相关方法（委托给 navigation_handler）
    # =========================================================================
    
    def _validate_target_position(self, x, y, z):
        """验证目标点是否在安全范围内 - 委托给 navigation_handler"""
        self.navigation_handler.validate_target_position(x, y, z)

    # ==================== 基于话题的导航方法 ====================
    
    @staticmethod
    def _compute_goal_id(usv_id: str, step: int) -> int:
        """计算确定性 goal_id: usv编号 × 100000 + step步骤号。"""
        try:
            usv_num = int(usv_id.split('_')[-1])
        except (ValueError, IndexError):
            usv_num = 0
        return usv_num * 100000 + step

    @staticmethod
    def _parse_goal_id(goal_id: int) -> tuple:
        """从 goal_id 解析出 (usv编号, step步骤号)。"""
        return goal_id // 100000, goal_id % 100000

    def send_nav_goal_via_topic(
        self, usv_id, x, y, z=0.0, yaw=0.0, use_yaw=False, timeout=300.0,
        roll=0.0, pitch=0.0, maneuver_type=0, maneuver_param=0.0, step=None,
        nav_mode=0, sync_timeout=10.0, arrival_quality_threshold=0.8,
        is_lookahead=False, task_name=''
    ):
        """
        通过话题方式向指定USV发送导航目标点（兼容 3D 阈值与编排/同步/机动参数）
        """
        from common_interfaces.msg import NavigationGoal
        from geometry_msgs.msg import PoseStamped
        
        # 验证目标点
        try:
            self._validate_target_position(x, y, z)
        except ValueError as e:
            self.get_logger().error(f"目标点验证失败: {e}")
            self.ros_signal.nav_status_update.emit(usv_id, "失败")
            return False
        
        # 检查发布器是否存在
        if usv_id not in self.usv_manager.navigation_goal_pubs:
            self.get_logger().error(f"未找到USV {usv_id} 的导航目标发布器")
            self.ros_signal.nav_status_update.emit(usv_id, "失败")
            return False
        
        # 确定性 Goal ID
        current_step = step if step is not None else getattr(self, 'run_step', 1)
        goal_id = self._compute_goal_id(usv_id, current_step)
        is_retry = False
        cached = self._usv_nav_target_cache.get(usv_id)
        if cached and cached.get('goal_id') == goal_id:
            is_retry = True
        
        # 记录目标ID到USV的映射
        self._goal_to_usv[goal_id] = usv_id
        
        # 异构立体分层高度回填：若 3D 载具收到 Z=0 的 2D 航点，自动采用机型默认分层高度
        vt = str(self._usv_vehicle_type.get(usv_id, '')).strip().lower()
        default_pm = '2d' if vt in ('usv', 'usv_diff', 'usv_boat') else '3d'
        pm = str(self._usv_platform_mode.get(usv_id, default_pm)).strip().lower()
        if pm not in ('2d', '3d'):
            pm = default_pm
        effective_z = float(z)
        if pm == '2d':
            effective_z = 0.0
        elif pm == '3d' and abs(effective_z) < 1e-3:
            def_alt = self._usv_default_altitude.get(usv_id, None)
            if def_alt is not None and float(def_alt) > 0.0:
                effective_z = float(def_alt)

        # 构造目标消息
        goal_msg = NavigationGoal()
        goal_msg.goal_id = goal_id
        goal_msg.task_name = task_name or ''
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.pose.position.x = float(x)
        goal_msg.target_pose.pose.position.y = float(y)
        goal_msg.target_pose.pose.position.z = effective_z
        
        # 设置姿态 (Quaternion)
        goal_msg.enable_yaw = bool(use_yaw)
        from tf_transformations import quaternion_from_euler
        effective_yaw = float(yaw) if use_yaw or (abs(float(yaw)) > 1e-6) else 0.0
        q = quaternion_from_euler(float(roll), float(pitch), effective_yaw)
        goal_msg.target_pose.pose.orientation.x = q[0]
        goal_msg.target_pose.pose.orientation.y = q[1]
        goal_msg.target_pose.pose.orientation.z = q[2]
        goal_msg.target_pose.pose.orientation.w = q[3]

        goal_msg.maneuver_type = int(maneuver_type)
        goal_msg.maneuver_param = float(maneuver_param)
        goal_msg.nav_mode = int(nav_mode)
        goal_msg.sync_timeout = float(sync_timeout)
        goal_msg.arrival_quality_threshold = float(arrival_quality_threshold)
        goal_msg.timeout = float(timeout)

        # 按机型/平台模式下发 3D/2D 到达判定阈值（优先用 usv_fleet.yaml 机型专属阈值）
        try:
            xy_thr = self._usv_reach_xy.get(usv_id, None)
            if xy_thr is None or float(xy_thr) <= 0.0:
                xy_thr = float(self.get_parameter(f'nav_reach_xy_threshold_{pm}').value)
            z_thr = self._usv_reach_z.get(usv_id, None)
            if z_thr is None or float(z_thr) <= 0.0:
                z_thr = float(self.get_parameter(f'nav_reach_z_threshold_{pm}').value)

            goal_msg.reach_xy_threshold = float(xy_thr)
            goal_msg.reach_z_threshold = float(z_thr)
        except Exception:
            goal_msg.reach_xy_threshold = 0.0
            goal_msg.reach_z_threshold = 0.0
        goal_msg.timestamp = self.get_clock().now().to_msg()
        
        # 发布目标
        pub = self.usv_manager.navigation_goal_pubs[usv_id]
        pub.publish(goal_msg)
        
        # 更新缓存和状态（预发送时跳过）
        if not is_lookahead:
            self._usv_nav_target_cache[usv_id] = {
                'goal_id': goal_id,
                'x': float(x),
                'y': float(y),
                'z': float(effective_z),
                'yaw': float(yaw),
                'use_yaw': bool(use_yaw),
                'maneuver_type': int(maneuver_type),
                'maneuver_param': float(maneuver_param),
                'nav_mode': int(nav_mode),
                'sync_timeout': float(sync_timeout),
                'arrival_quality_threshold': float(arrival_quality_threshold),
                'step': current_step,
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'paused': False,
                'reached': False,
            }
            self.ros_signal.nav_status_update.emit(usv_id, "执行中")
        
        retry_tag = "(重试) " if is_retry else ""
        lookahead_tag = "📤预发送 " if is_lookahead else "📤 "
        self.get_logger().info(
            f"{lookahead_tag}{usv_id} 导航目标已发送 {retry_tag}[ID={goal_id}]: "
            f"({x:.1f}, {y:.1f}, {effective_z:.1f}), 超时={timeout:.0f}s")
        
        return True

    def send_attitude_command_via_topic(self, usv_id, roll=0.0, pitch=0.0, yaw=None, duration=1.0):
        """单向下发姿态动作指令（roll/pitch），不要求完成度反馈。

        约定：单位均为弧度(rad)。

        Args:
            usv_id (str): USV 标识符
            roll (float): 翻滚角(rad)
            pitch (float): 俯仰角(rad)
            yaw (float|None): 可选偏航(rad)。None 表示保持当前航向（将发送 NaN）。
            duration (float): >0 表示持续秒数；<=0 持续生效直到下一条覆盖。

        Returns:
            bool: 发送是否成功
        """
        from common_interfaces.msg import AttitudeCommand

        if usv_id not in self.usv_manager.attitude_cmd_pubs:
            self.get_logger().error(f"未找到USV {usv_id} 的姿态动作发布器")
            return False

        msg = AttitudeCommand()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float('nan') if yaw is None else float(yaw)
        msg.duration = float(duration)

        self.usv_manager.attitude_cmd_pubs[usv_id].publish(msg)
        self.get_logger().info(
            f"📤 {usv_id} 姿态动作已发送: roll={msg.roll:.3f} rad, pitch={msg.pitch:.3f} rad, "
            f"duration={msg.duration:.2f}s"
        )
        return True
    
    def navigation_feedback_callback(self, msg, usv_id):
        """
        导航反馈回调 (话题版本)
        
        Args:
            msg (NavigationFeedback): 导航反馈消息
            usv_id (str): USV标识符
        """
        # 检查是否是当前目标的反馈
        cached = self._usv_nav_target_cache.get(usv_id)
        if cached and cached.get('goal_id') != msg.goal_id:
            return  # 忽略旧目标的反馈
        
        # 简化日志输出
        self.get_logger().debug(
            f"{usv_id}: 距离={msg.distance_to_goal:.2f}m, "
            f"航向误差={msg.heading_error:.1f}°, "
            f"预计={msg.estimated_time:.0f}s")
        
        # 发射信号更新GUI
        # 转换为兼容格式
        feedback_obj = type('Feedback', (), {
            'goal_id': msg.goal_id,
            'step': cached.get('step', 0) if cached else 0,
            'distance_to_goal': msg.distance_to_goal,
            'xy_distance_to_goal': getattr(msg, 'xy_distance_to_goal', msg.distance_to_goal),
            'z_error': getattr(msg, 'z_error', 0.0),
            'heading_error': msg.heading_error,
            'estimated_time': msg.estimated_time,
            'queue_length': getattr(msg, 'queue_length', 0),
            'queue_capacity': getattr(msg, 'queue_capacity', 0),
            'smooth_navigation': getattr(msg, 'smooth_navigation', False),
            'nav_mode': getattr(msg, 'nav_mode', 0),
            'arrival_quality': getattr(msg, 'arrival_quality', 0.0),
            'rotate_progress': getattr(msg, 'rotate_progress', 0.0),
        })()
        self.ros_signal.navigation_feedback.emit(usv_id, feedback_obj)

        # 通知集群控制器（用于平滑导航预发送与到达质量确认）
        if hasattr(self.cluster_controller, 'on_navigation_feedback'):
            try:
                self.cluster_controller.on_navigation_feedback(usv_id, msg)
            except Exception:
                pass

        # 方案A：先导航，再在接近目标时触发一次姿态动作（每艇每step只触发一次）
        try:
            goal_step = cached.get('step') if cached else None
            dist_for_trigger = float(getattr(msg, 'xy_distance_to_goal', msg.distance_to_goal))
            self.cluster_controller.maybe_trigger_attitude_on_feedback(usv_id, dist_for_trigger, goal_step)
        except Exception:
            pass
    
    def navigation_result_callback(self, msg, usv_id):
        """
        导航结果回调 (话题版本)
        
        Args:
            msg (NavigationResult): 导航结果消息
            usv_id (str): USV标识符
        """
        # 详细调试日志
        self.get_logger().info(
            f"🔍 [DEBUG] 收到导航结果: usv_id={usv_id}, goal_id={msg.goal_id}, "
            f"success={msg.success}, message={msg.message}"
        )
        
        # 检查是否是当前目标的结果
        cached = self._usv_nav_target_cache.get(usv_id)
        if cached:
            self.get_logger().info(
                f"🔍 [DEBUG] 缓存目标信息: goal_id={cached.get('goal_id')}, "
                f"step={cached.get('step')}, x={cached.get('x'):.2f}, y={cached.get('y'):.2f}"
            )
        else:
            self.get_logger().warning(
                f"⚠️ {usv_id} 没有缓存目标，可能已被清除或过期"
            )
        
        if cached and cached.get('goal_id') != msg.goal_id:
            self.get_logger().warning(
                f"⚠️ {usv_id} 目标ID不匹配: cached={cached.get('goal_id')}, "
                f"received={msg.goal_id}，忽略此结果"
            )
            return  # 忽略旧目标的结果
        
        # 记录日志
        status_icon = "✅" if msg.success else "❌"
        self.get_logger().info(
            f"{status_icon} {usv_id} 导航完成 [ID={msg.goal_id}]: {msg.message}")
        
        # 获取目标的 step 信息
        goal_step = cached.get('step') if cached else None
        
        # 更新状态
        if msg.success:
            if cached is not None:
                cached['reached'] = True
            self.ros_signal.nav_status_update.emit(usv_id, "成功")
            self.cluster_controller.mark_usv_goal_result(usv_id, True, goal_step)
            
            # ✅ 修复：不在每个目标点完成时切换HOLD，让USV保持GUIDED模式继续执行后续步骤
            # 集群任务完成后会统一切换到HOLD（在_reset_cluster_task中处理）
            self.get_logger().info(f"✅ {usv_id} 导航成功，保持GUIDED模式等待下一步任务")
        else:
            self.ros_signal.nav_status_update.emit(usv_id, "失败")
            self.cluster_controller.mark_usv_goal_result(usv_id, False, goal_step)
        
        # 清理映射
        if msg.goal_id in self._goal_to_usv:
            del self._goal_to_usv[msg.goal_id]

    def navigation_ack_callback(self, msg, usv_id):
        """导航应答回调（收到/接受层，用于停止 step_timeout 重发）。"""
        cached = self._usv_nav_target_cache.get(usv_id)
        if cached and cached.get('goal_id') != msg.goal_id:
            return

        if cached is not None:
            cached['execute_mask'] = int(getattr(msg, 'execute_mask', 0))
            cached['ack_message'] = str(getattr(msg, 'message', ''))

        goal_step = cached.get('step') if cached else None
        try:
            self.cluster_controller.mark_usv_goal_ack(usv_id, bool(msg.accepted), goal_step)
        except Exception as e:
            self.get_logger().warning(f"处理 navigation/ack 失败: {e}")

    # 设置离群目标点回调
    def set_departed_target_point_callback(self, msg):
        """
        设置离群目标点
        
        Args:
            msg: 包含离群目标点的消息
        """
        # 记录日志信息
        self.get_logger().info("接收到离群目标点")
        try:
            # 检查msg对象是否具有targets属性
            usv_list = msg.targets if hasattr(msg, 'targets') else msg
            # 类型检查
            if not isinstance(usv_list, list):
                # 记录错误日志
                self.get_logger().error(f"usv_list 不是列表: {usv_list}")
                return

            # 遍历USV列表
            for ns in usv_list:
                # 类型检查
                if not isinstance(ns, dict):
                    # 记录警告日志
                    self.get_logger().warning(f"无效的目标格式: {ns}, 跳过")
                    continue
                # 获取USV ID
                usv_id = ns.get('usv_id')
                # 检查USV ID是否有效
                if not usv_id:
                    # 记录警告日志
                    self.get_logger().warning(f"无效 usv_id: {usv_id}, 跳过")
                    continue

                # 获取位置信息
                pos = ns.get('position', {})
                # 检查位置信息是否完整（包含x,y,z）
                if not all(k in pos for k in ('x', 'y', 'z')):
                    # 记录警告日志
                    self.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                    continue

                # 通过Action接口发送导航目标点（先转换坐标系）
                yaw = ns.get('yaw', 0.0)
                p_global = self.cluster_controller._area_to_global(pos)
                p_local = self.cluster_controller._global_to_usv_local(usv_id, p_global)
                
                # 🔍 调试日志：完整坐标转换链路
                self.get_logger().info(
                    f"📤 [地面站发送] {usv_id}\n"
                    f"  ├─ Area坐标(XML): X={pos.get('x', 0.0):.2f}, Y={pos.get('y', 0.0):.2f}, Z={pos.get('z', 0.0):.2f}\n"
                    f"  ├─ Global坐标: X={p_global.get('x', 0.0):.2f}, Y={p_global.get('y', 0.0):.2f}, Z={p_global.get('z', 0.0):.2f}\n"
                    f"  ├─ Local坐标: X={p_local.get('x', 0.0):.2f}, Y={p_local.get('y', 0.0):.2f}, Z={p_local.get('z', 0.0):.2f}\n"
                    f"  └─ Yaw: {yaw:.2f} rad"
                )
                
                # 支持z坐标（修复关键字参数传参，避免 300.0 误传入 use_yaw）
                use_yaw_flag = bool(ns.get('use_yaw', abs(float(yaw)) > 1e-6))
                self.send_nav_goal_via_topic(
                    usv_id,
                    p_local.get('x', 0.0),
                    p_local.get('y', 0.0),
                    p_local.get('z', 0.0),
                    yaw=yaw,
                    use_yaw=use_yaw_flag,
                    timeout=300.0,
                )
        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"处理离群目标点失败: {e}")

    def set_single_target_point_callback(self, payload: dict):
        """
        从 3D 主态势界面直接发送单艇/多艇三维目标点。
        支持 coord_frame='local'（地面站可视化 ENU 局部坐标）或 'area'（任务区域坐标）。
        """
        try:
            if not isinstance(payload, dict):
                return
            raw_ids = payload.get('usv_ids') or []
            if isinstance(raw_ids, str):
                raw_ids = [raw_ids]
            if not raw_ids and payload.get('usv_id'):
                raw_ids = [payload.get('usv_id')]
            usv_ids = [str(uid).lstrip('/') for uid in raw_ids if uid]
            if not usv_ids:
                self.get_logger().warning("直发三维目标缺少有效 usv_id")
                return

            tx = float(payload.get('x', 0.0))
            ty = float(payload.get('y', 0.0))
            tz = float(payload.get('z', 0.0))
            yaw = float(payload.get('yaw', 0.0))
            use_yaw = bool(payload.get('use_yaw', False))
            coord_frame = str(payload.get('coord_frame', 'local')).lower()
            timeout = float(payload.get('timeout', 300.0))

            for usv_id in usv_ids:
                if coord_frame == 'area':
                    p_global = self.cluster_controller._area_to_global({'x': tx, 'y': ty, 'z': tz})
                else:
                    p_global = {'x': tx, 'y': ty, 'z': tz}
                p_local = self.cluster_controller._global_to_usv_local(usv_id, p_global)
                self.send_nav_goal_via_topic(
                    usv_id,
                    p_local.get('x', tx),
                    p_local.get('y', ty),
                    p_local.get('z', tz),
                    yaw=yaw,
                    use_yaw=use_yaw,
                    timeout=timeout,
                )
        except Exception as e:
            self.get_logger().error(f"直发三维目标点失败: {e}")

    # 委托给子模块的方法 (PX4 模式)
    def set_hold_callback(self, msg):
        self.command_processor.set_hold_callback(msg)

    def set_offboard_callback(self, msg):
        self.command_processor.set_offboard_callback(msg)

    def set_stabilized_callback(self, msg):
        self.command_processor.set_stabilized_callback(msg)

    def set_posctl_callback(self, msg):
        self.command_processor.set_posctl_callback(msg)

    def set_altctl_callback(self, msg):
        self.command_processor.set_altctl_callback(msg)

    def set_arming_callback(self, msg):
        self.command_processor.set_arming_callback(msg)

    def set_disarming_callback(self, msg):
        self.command_processor.set_disarming_callback(msg)

    def set_takeoff_callback(self, payload):
        self.command_processor.set_takeoff_callback(payload)

    def set_land_callback(self, payload):
        self.command_processor.set_land_callback(payload)

    def is_3d_vehicle(self, usv_id: str) -> bool:
        """判断指定载具是否为需要垂直起降门控的 3D 载体（飞行球、海豚、飞艇）"""
        return self.command_processor._is_3d_vehicle(usv_id)

    def get_vehicle_required_takeoff_z(self, usv_id: str) -> float:
        """获取指定 3D 载具起飞所需达到的预高度（m）"""
        if not self.is_3d_vehicle(usv_id):
            return 0.0
        st = self.usv_states.get(usv_id, {}) or {}
        for cand in (st.get('takeoff_altitude'), self._usv_default_altitude.get(usv_id), st.get('default_altitude')):
            try:
                if cand is not None and float(cand) > 0.0:
                    return float(cand)
            except Exception:
                pass
        vt = str(st.get('vehicle_type') or self._usv_vehicle_type.get(usv_id, '')).strip().lower()
        if vt in ('bionic_airship', 'airship'):
            return 15.0
        if vt in ('bionic_dolphin', 'dolphin'):
            return 6.0
        return 2.0

    def validate_3d_cluster_takeoff_ready(self, usv_ids=None) -> tuple:
        """
        检查参与路径程序/集群/编队的 3D 载体（飞行球、海豚、飞艇）是否已起飞到达设定预高度且未处于保护锁止：
        返回 (ok: bool, reasons: list[str])
        """
        if usv_ids is None:
            usv_ids = [
                uid for uid, st in self.usv_states.items()
                if isinstance(st, dict) and st.get('connected', False)
            ]
        reasons = []
        for raw in usv_ids:
            uid = raw.get('namespace', '').lstrip('/') if isinstance(raw, dict) else str(raw).lstrip('/')
            if not uid:
                continue
            st = self.usv_states.get(uid)
            if not isinstance(st, dict) or not st.get('connected', False):
                continue
            if not self.is_3d_vehicle(uid):
                continue
            if bool(st.get('protection_locked', False)):
                reasons.append(f"{uid}: 处于降落电机保护锁止状态（需手动 Arming 解锁并点击 Takeoff 起飞）")
                continue
            if not bool(st.get('armed', False)):
                reasons.append(f"{uid}: 未解锁（Disarmed，需先解锁并起飞至预高度）")
                continue
            req_z = self.get_vehicle_required_takeoff_z(uid)
            tol_z = min(0.5, max(0.25, float(self._usv_reach_z.get(uid, 0.5)) * 0.5))
            pos = st.get('position') or {}
            cur_z = float(pos.get('z', st.get('position_z', 0.0)) or 0.0)
            # 若机载已汇报 takeoff_ready 且当前高度仍在预高度容差内，或实时高度 >= (req_z - tol_z)，视为已到位
            if cur_z < (req_z - tol_z):
                reasons.append(f"{uid}: 当前高度 Z={cur_z:.2f}m 未达到起飞预高度 {req_z:.2f}m（请先点击 Takeoff 起飞到位）")
        return (len(reasons) == 0, reasons)

    def set_cluster_target_point_callback(self, msg):
        self.cluster_controller.set_cluster_target_point_callback(msg)

    def publish_cluster_targets_callback(self):
        self.cluster_controller.publish_cluster_targets_callback()

    def _process_incoming_str_commands(self):
        self.command_processor.process_incoming_str_commands()

    def check_usv_infect(self):
        """定时检查USV传染逻辑（只有在传染模式开启时才执行）"""
        if self._led_infection_enabled:
            self.led_infection_handler.check_usv_infect()
        else:
            # 如果传染模式关闭，清理所有传染相关状态
            if self._usv_led_modes:
                # 恢复所有被传染USV的原始LED状态
                for dst_id in list(self._usv_led_modes.keys()):
                    mode, color = self._usv_led_modes[dst_id]
                    if dst_id in self.usv_manager.led_pubs:
                        if mode == 'color_select':
                            cmd = f"color_select|{color[0]},{color[1]},{color[2]}"
                        else:
                            cmd = mode
                        from std_msgs.msg import String
                        msg = String()
                        msg.data = cmd
                        self.publish_queue.put((self.usv_manager.led_pubs[dst_id], msg))
                # 清空传染状态字典
                self._usv_led_modes.clear()
                self._usv_infecting.clear()
                self._usv_infection_sources.clear()

    def _update_local_led_state(self, usv_id, command_str):
        """
        根据发送的 LED 命令更新本地维护的状态
        """
        if not command_str.data:
            return

        cmd_parts = command_str.data.split('|')
        mode = cmd_parts[0].lower()
        
        cached = self._usv_current_led_state.get(
            usv_id, {'mode': 'color_switching', 'color': [255, 0, 0]})

        new_state = {
            'mode': cached.get('mode', 'color_switching'),
            'color': list(cached.get('color', [255, 0, 0]))
        }
        updated = False

        if mode == 'color_select' and len(cmd_parts) > 1:
            try:
                color_parts = [max(0, min(255, int(c.strip()))) for c in cmd_parts[1].split(',')]
            except ValueError:
                self.get_logger().warn(f"解析 {usv_id} 的颜色命令失败: {command_str.data}")
                color_parts = None

            if color_parts and len(color_parts) == 3:
                if new_state['mode'] != 'color_select':
                    new_state['mode'] = 'color_select'
                    updated = True
                if new_state['color'] != color_parts:
                    new_state['color'] = color_parts
                    updated = True
        elif mode != 'color_infect':  # 传染模式不改变基础模式和颜色
            if new_state['mode'] != mode:
                new_state['mode'] = mode
                updated = True

        self._usv_current_led_state[usv_id] = new_state
        if updated:
            self.led_infection_handler.propagate_color_update(usv_id)

    def str_command_callback(self, msg):
        """
        字符串命令回调函数
        
        Args:
            msg: 字符串命令消息
        """
        self.command_processor.str_command_callback(msg)
    
    def reboot_autopilot_callback(self, usv_namespace):
        """飞控重启回调，委托给 system_command_handler."""
        self.system_command_handler.reboot_autopilot(usv_namespace)
    
    def reboot_companion_callback(self, usv_namespace):
        """机载计算机重启回调，委托给 system_command_handler."""
        self.system_command_handler.reboot_companion(usv_namespace)

    def set_home_position_callback(self, usv_namespace, use_current, coords):
        """设置 Home Position 回调，委托给 system_command_handler."""
        self.system_command_handler.set_home_position(usv_namespace, use_current, coords)

    def shutdown_usv_callback(self, usv_namespace):
        """优雅关闭 USV 节点回调，委托给 system_command_handler."""
        self.system_command_handler.shutdown_usv(usv_namespace)

    def _on_usv_log_message(self, usv_id: str, text: str):
        """
        处理来自 UsvManager 的日志消息回调
        
        将纯文本消息转换为类似 StatusText 的消息对象，
        然后传递给 sensor_handler 处理。
        
        Args:
            usv_id: USV 标识符
            text: 日志消息文本
        """
        if not text:
            return
        
        # 创建一个简单的消息对象，包含 text 和 severity 属性
        class SimpleStatusText:
            def __init__(self, text_content, severity_level=6):
                self.text = text_content
                self.severity = severity_level
        
        # 根据文本内容推断严重性级别
        text_upper = text.upper()
        if 'ERROR' in text_upper or 'FAIL' in text_upper or 'CRITICAL' in text_upper:
            severity = 3  # ERROR
        elif 'WARN' in text_upper or 'PREARM' in text_upper or 'PREFLIGHT' in text_upper:
            severity = 4  # WARNING
        elif 'INFO' in text_upper:
            severity = 6  # INFO
        else:
            severity = 6  # 默认 INFO
        
        msg = SimpleStatusText(text, severity)
        self.handle_status_text(usv_id, msg)

    def _on_usv_event(self, usv_id: str, event_id: int, arguments: bytes):
        """
        处理来自 UsvManager 的 PX4 Event 消息回调
        
        将 PX4 事件解码为人类可读的消息，如 "Arming denied: high throttle"
        
        Args:
            usv_id: USV 标识符
            event_id: PX4 事件 ID
            arguments: 事件参数（字节数组）
        """
        try:
            # 将参数字节转换为参数字符串
            # PX4 Event 参数格式：每个参数是一个字节或多个字节
            args_str = None
            if arguments:
                # 简化处理：将字节转换为 "-val1-val2-..." 格式
                args_list = [str(b) for b in arguments if b != 0]
                if args_list:
                    args_str = '-' + '-'.join(args_list)
            
            # 尝试使用事件解码器解码
            decoded_msg = self.event_decoder.decode(event_id, args_str)
            
            if decoded_msg:
                # 解码成功，创建状态文本消息
                self.get_logger().info(f"[FCU-EVENT] {usv_id}: {decoded_msg} (ID={event_id})")
                
                # 根据消息内容推断严重性级别
                msg_upper = decoded_msg.upper()
                if 'DENIED' in msg_upper or 'FAIL' in msg_upper or 'CRITICAL' in msg_upper or 'EMERGENCY' in msg_upper:
                    severity = 3  # ERROR
                elif 'WARN' in msg_upper or 'CAUTION' in msg_upper:
                    severity = 4  # WARNING
                else:
                    severity = 6  # INFO
                
                # 创建消息对象并处理
                class SimpleStatusText:
                    def __init__(self, text_content, severity_level=6):
                        self.text = text_content
                        self.severity = severity_level
                
                msg = SimpleStatusText(decoded_msg, severity)
                self.handle_status_text(usv_id, msg)
            else:
                # 解码失败，记录原始事件 ID
                self.get_logger().debug(f"[FCU-EVENT] {usv_id}: 未知事件 ID={event_id}")
                
        except Exception as e:
            self.get_logger().debug(f"事件处理失败: {e}")

    def handle_status_text(self, usv_id, msg):
        """处理飞控 status_text 消息，委托给 sensor_handler."""
        # 委托给 sensor_handler
        self.sensor_handler.handle_status_text(usv_id, msg)
        
        # 确保状态字典存在
        state = self.usv_states.get(usv_id)
        if state is None:
            state = {'namespace': usv_id}
            self.usv_states[usv_id] = state
        
        # 更新增强状态并推送到 GUI
        self.augment_state_payload(usv_id, state)

        try:
            self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))
        except Exception as exc:
            self.get_logger().warn(f"推送 {usv_id} 状态文本更新失败: {exc}")

    def handle_sys_status(self, usv_id, msg):
        """处理飞控 SYS_STATUS 消息，委托给 sensor_handler."""
        self.sensor_handler.handle_sys_status(usv_id, msg)

    def _extract_usv_id_from_log(self, node_name):
        """
        从日志节点名中提取 USV ID
        
        Args:
            node_name: 日志源节点名 (例如 "usv_01.sys", "usv_01.fmu")
            
        Returns:
            str: USV ID (例如 "usv_01") 或 "unknown"
        """
        import re
        
        # 尝试从 "usv_xx.xxx" 格式提取
        if '.' in node_name:
            parts = node_name.split('.')
            if parts[0].startswith('usv_'):
                return parts[0]
        
        # 尝试正则匹配 usv_xx 模式
        match = re.search(r'(usv_\d+)', node_name)
        if match:
            return match.group(1)
        
        # 如果无法从节点名提取，检查是否只有一个活跃 USV
        active_usvs = list(self.usv_manager.usv_state_subs.keys())
        if len(active_usvs) == 1:
            return active_usvs[0]
        
        return "unknown"

    def _parse_event_message(self, content):
        """
        解析 FCU EVENT 消息，提取事件 ID 和参数
        
        Args:
            content: 消息内容 (例如 "FCU: EVENT 12345 with args -4-0-0...")
            
        Returns:
            tuple: (event_id: int, args_str: str or None)
        """
        # 提取 "EVENT " 后面的数字
        start_idx = content.find('EVENT ') + 6
        space_idx = content.find(' ', start_idx)
        
        if space_idx == -1:
            # 没有参数，只有 ID
            event_id_str = content[start_idx:]
            return int(event_id_str.strip()), None
        else:
            event_id_str = content[start_idx:space_idx]
            # 提取参数部分 (在 "with args " 之后)
            args_marker = 'with args '
            args_idx = content.find(args_marker)
            if args_idx != -1:
                args_str = content[args_idx + len(args_marker):]
                return int(event_id_str), args_str
            return int(event_id_str), None

    def rosout_callback(self, msg):
        """
        处理 ROS 日志消息，用于捕获 PX4 的 FCU: EVENT 消息和其他重要日志
        """
        # 过滤出 USV 相关的日志
        # PX4 的日志节点名可能是:
        # - "usv_01.sys" (PX4 sys_status)
        # - "usv_01.fmu" (PX4 fmu)
        # - 包含 "usv_" 的其他名称
        if 'usv_' not in msg.name:
            return
        
        # 提取 USV ID
        usv_id = self._extract_usv_id_from_log(msg.name)
        if usv_id == "unknown":
            return
            
        # 检查是否是 FCU 事件消息
        # 格式通常为: "FCU: EVENT <id> with args <args>"
        if 'FCU: EVENT' in msg.msg:
            try:
                # 提取 USV ID (从节点名中提取，例如 usv_01.mavros)
                usv_id = self._extract_usv_id_from_log(msg.name)

                # 解析事件 ID 和参数
                # 格式: "FCU: EVENT <id>" 或 "FCU: EVENT <id> with args <args>"
                content = msg.msg
                event_id, args_str = self._parse_event_message(content)
                
                # 尝试解码
                decoded_msg = self.event_decoder.decode(event_id, args_str)
                
                if decoded_msg:
                    # 构造类似于 StatusText 的消息条目
                    log_text = f"[Event] {decoded_msg}"
                    self.get_logger().info(f"解码事件 {usv_id}: {log_text}")
                    
                    now_sec = self._now_seconds()
                    entry = {
                        'text': log_text,
                        'severity': msg.level,  # 使用原始日志级别
                        'severity_label': self._severity_to_label(msg.level),
                        'time': self._format_time(now_sec),
                        'timestamp': now_sec,
                    }
                    
                    # 如果是已知 USV，添加到消息列表 (通过 sensor_handler)
                    if usv_id != "unknown":
                        self.sensor_handler._vehicle_messages[usv_id].appendleft(entry)
                        # 触发 GUI 更新
                        self.ros_signal.status_text_received.emit(usv_id, log_text)
                    else:
                        # 如果 USV ID 未知，尝试广播给所有活跃的 USV
                        self.get_logger().warn(f"收到未关联 USV 的事件: {log_text}")
                        
            except Exception as e:
                # 解析失败则忽略
                self.get_logger().debug(f"事件解析失败: {e}")
            return  # 已处理 FCU EVENT，不再重复添加
        
        # 处理其他重要的 USV 日志消息 (WARN/ERROR 级别)
        # ROS Log 级别: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
        if msg.level >= 30:  # WARN 及以上
            try:
                now_sec = self._now_seconds()
                # 映射 ROS 日志级别到 MAVLink severity
                if msg.level >= 50:  # FATAL
                    severity = 0  # EMERGENCY
                elif msg.level >= 40:  # ERROR
                    severity = 3  # ERROR
                else:  # WARN
                    severity = 4  # WARNING
                
                entry = {
                    'text': msg.msg,
                    'severity': severity,
                    'severity_label': self._severity_to_label(severity),
                    'time': self._format_time(now_sec),
                    'timestamp': now_sec,
                }
                self.sensor_handler._vehicle_messages[usv_id].appendleft(entry)
                self.ros_signal.status_text_received.emit(usv_id, msg.msg)
            except Exception:
                pass

    def push_state_updates(self):
        """
        定期主动推送状态更新到 GUI（5Hz 优化频率）
        
        始终推送最新状态到 GUI，以确保远程 USV 的位置等实时数据能够显示。
        augment_state_payload 用于添加传感器/预检信息，有条件地执行以节省开销。
        """
        if not self.usv_states:
            return
        
        try:
            now_sec = self._now_seconds()
            
            # 遍历所有 USV，实时同步三维目标缓存与异构机型元数据，按需更新 augmented 数据
            for usv_id in list(self.usv_states.keys()):
                st = self.usv_states.get(usv_id)
                if isinstance(st, dict):
                    st['nav_target_cache'] = self._usv_nav_target_cache.get(usv_id)
                    if usv_id in self._usv_vehicle_type:
                        st['vehicle_type'] = self._usv_vehicle_type[usv_id]
                    if usv_id in self._usv_platform_mode:
                        st['platform_mode'] = self._usv_platform_mode[usv_id]
                    if usv_id in self._usv_default_altitude:
                        st['default_altitude'] = self._usv_default_altitude[usv_id]
                        st.setdefault('takeoff_altitude', self._usv_default_altitude[usv_id])
                    if usv_id in self._usv_landing_altitude:
                        st['landing_altitude'] = self._usv_landing_altitude[usv_id]
                    if usv_id in self._usv_current_led_state:
                        st['led_status'] = self._usv_current_led_state[usv_id]
                # 检查是否需要更新（有新消息、PreArm 警告变化、传感器状态变化）
                if self._should_update_augmented_state(usv_id, now_sec):
                    self.augment_state_payload(usv_id)
            
            # 始终推送状态列表（位置、速度等实时数据来自 discovery_handler）
            self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))
        except Exception as exc:
            # 使用 debug 级别避免刷屏，因为这是高频调用
            pass  # 静默失败，避免日志刷屏
    
    def augment_state_payload(self, usv_id, state_data=None):
        """
        为状态字典附加车辆消息、预检标记与传感器状态
        
        委托给 sensor_handler 实现 QGC 风格的综合 Ready 检查
        """
        if state_data is None:
            state_data = self.usv_states.get(usv_id)
            if state_data is None:
                return None

        now_sec = self._now_seconds()
        
        # 使用 sensor_handler 进行状态处理
        self.sensor_handler.cleanup_prearm_warnings(usv_id, now_sec)

        # 1. 收集所有消息 (从 sensor_handler)
        messages = self.sensor_handler.get_vehicle_messages(usv_id)
        
        # 2. 收集 PreArm 警告 (从 sensor_handler)
        prearm_warnings = self.sensor_handler.get_prearm_warnings(usv_id)
        
        # 3. 收集最近的 CRITICAL/ERROR 消息 (从 sensor_handler)
        critical_errors = self.sensor_handler.get_critical_errors(usv_id, within_seconds=30.0)
        
        # 4. 检查传感器健康状态 (从 sensor_handler)
        sensor_healthy, unhealthy_sensors = self.sensor_handler.check_sensor_health(usv_id)
        
        # 5. 综合判断 Ready 状态
        all_warnings = prearm_warnings.copy()
        
        # 将严重错误添加到警告列表
        if critical_errors:
            for err in critical_errors[:3]:
                all_warnings.append(f"[CRITICAL] {err}")
        
        # 将传感器问题添加到警告列表
        if not sensor_healthy:
            for sensor in unhealthy_sensors:
                all_warnings.append(f"[传感器] {sensor} 异常")
        
        # Ready 状态: 所有检查都通过
        ready = (len(prearm_warnings) == 0 and 
                 len(critical_errors) == 0 and 
                 sensor_healthy)
        
        # 缓存结果到 sensor_handler
        self.sensor_handler.set_prearm_ready(usv_id, ready)
        sensor_status = self.sensor_handler.build_sensor_status(usv_id, state_data)
        self.sensor_handler.cache_sensor_status(usv_id, sensor_status)

        # 更新状态数据
        state_data['vehicle_messages'] = messages
        state_data['prearm_ready'] = ready
        state_data['prearm_warnings'] = all_warnings
        state_data['sensor_status'] = sensor_status
        state_data['nav_target_cache'] = self._usv_nav_target_cache.get(usv_id)

        return state_data

    def _should_update_augmented_state(self, usv_id, now_sec):
        """
        检查是否需要重新计算 augmented state
        避免无变化时的重复计算
        """
        # 检查是否有新的 statustext 消息 (从 sensor_handler)
        last_msg_time = self.sensor_handler.get_last_statustext_time(usv_id)
        if now_sec - last_msg_time < 0.3:  # 300ms 内有新消息
            return True
        
        # 检查是否有传感器状态更新 (从 sensor_handler)
        sensor_cache = self.sensor_handler.get_sensor_health_cache(usv_id)
        if sensor_cache:
            if now_sec - sensor_cache.get('timestamp', 0) < 0.5:  # 500ms 内有更新
                return True
        
        # 默认每 2 秒强制更新一次
        last_update = getattr(self, '_last_augment_time', {}).get(usv_id, 0)
        if now_sec - last_update > 2.0:
            if not hasattr(self, '_last_augment_time'):
                self._last_augment_time = {}
            self._last_augment_time[usv_id] = now_sec
            return True
        
        return False

    def _now_seconds(self):
        try:
            return self.get_clock().now().nanoseconds / 1e9
        except Exception:
            return datetime.now().timestamp()

    def _format_time(self, seconds):
        try:
            return datetime.fromtimestamp(seconds).strftime('%H:%M:%S')
        except Exception:
            return '--:--:--'

    @staticmethod
    def _severity_to_label(severity):
        # MAVLink/PX4 日志级别 (0-7)
        mavlink_mapping = {
            0: 'EMERGENCY',
            1: 'ALERT',
            2: 'CRITICAL',
            3: 'ERROR',
            4: 'WARNING',
            5: 'NOTICE',
            6: 'INFO',
            7: 'DEBUG',
        }
        # ROS 2 日志级别 (10/20/30/40/50)
        ros2_mapping = {
            10: 'DEBUG',
            20: 'INFO',
            30: 'WARNING',
            40: 'ERROR',
            50: 'FATAL',
        }
        # 先尝试 MAVLink 映射，再尝试 ROS 2 映射
        if severity in mavlink_mapping:
            return mavlink_mapping[severity]
        if severity in ros2_mapping:
            return ros2_mapping[severity]
        return f'LEVEL {severity}'

    def handle_led_state_feedback(self, usv_id, msg):
        """处理来自USV的LED状态反馈。"""
        if msg is None:
            return

        payload_raw = getattr(msg, 'data', '')
        if not payload_raw:
            return

        try:
            payload = json.loads(payload_raw)
        except (TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"解析 {usv_id} 的LED状态失败: {exc}")
            return

        state = self._usv_current_led_state.get(
            usv_id, {'mode': 'color_switching', 'color': [255, 0, 0]})
        current_mode = state.get('mode', 'color_switching')
        current_color = list(state.get('color', [255, 0, 0]))

        updated = False

        mode_val = payload.get('mode')
        if isinstance(mode_val, str) and mode_val:
            mode_norm = mode_val.lower()
            if mode_norm != current_mode:
                current_mode = mode_norm
                updated = True

        color_val = payload.get('color')
        if isinstance(color_val, (list, tuple)) and len(color_val) >= 3:
            try:
                sanitized = [max(0, min(255, int(c))) for c in color_val[:3]]
            except (TypeError, ValueError):
                sanitized = None
            if sanitized and sanitized != current_color:
                current_color = sanitized
                updated = True

        self._usv_current_led_state[usv_id] = {
            'mode': current_mode,
            'color': current_color
        }

        if updated:
            self.led_infection_handler.propagate_color_update(usv_id)
    
    def update_area_center_callback(self, offset_dict):
        """
        更新任务坐标系偏移量（Area Center）
        
        Args:
            offset_dict: 偏移量字典 {'x': float, 'y': float, 'z': float}
        """
        try:
            # 更新内部存储的area_center
            self._area_center['x'] = float(offset_dict.get('x', 0.0))
            self._area_center['y'] = float(offset_dict.get('y', 0.0))
            self._area_center['z'] = float(offset_dict.get('z', 0.0))
            
            self.get_logger().info(
                f"已更新 Area Center 偏移量: "
                f"X={self._area_center['x']:.2f}, "
                f"Y={self._area_center['y']:.2f}, "
                f"Z={self._area_center['z']:.2f}"
            )
            
            # 可选：将新偏移量保存到参数服务器
            try:
                self.set_parameters([
                    Parameter('area_center_x', 
                        Parameter.Type.DOUBLE, 
                        self._area_center['x']),
                    Parameter('area_center_y', 
                        Parameter.Type.DOUBLE, 
                        self._area_center['y']),
                    Parameter('area_center_z', 
                        Parameter.Type.DOUBLE, 
                        self._area_center['z'])
                ])
            except Exception as e:
                self.get_logger().warn(f"更新参数服务器失败: {e}")
                
        except Exception as e:
            self.get_logger().error(f"更新 Area Center 偏移量失败: {e}")

    def update_fence_config_callback(self, fence_dict):
        """
        更新随机运行围栏配置
        
        Args:
            fence_dict: 围栏配置字典
        """
        try:
            # 同步更新内部字典
            self._fence_config.update(fence_dict)
            
            fence_type = int(fence_dict.get('type', 0))
            params = [Parameter('fence_type', Parameter.Type.INTEGER, fence_type)]
            
            log_msg = f"已更新围栏配置: 类型={'圆柱' if fence_type == 0 else '长方体'}"
            
            if fence_type == 0:
                radius = float(fence_dict.get('radius', 50.0))
                height = float(fence_dict.get('height', 10.0))
                params.append(Parameter('fence_radius', Parameter.Type.DOUBLE, radius))
                params.append(Parameter('fence_height', Parameter.Type.DOUBLE, height))
                log_msg += f", 半径={radius}m, 高度={height}m"
            else:
                length = float(fence_dict.get('length', 100.0))
                width = float(fence_dict.get('width', 100.0))
                height = float(fence_dict.get('height', 10.0))
                params.append(Parameter('fence_length', Parameter.Type.DOUBLE, length))
                params.append(Parameter('fence_width', Parameter.Type.DOUBLE, width))
                params.append(Parameter('fence_height', Parameter.Type.DOUBLE, height))
                log_msg += f", 长度={length}m, 宽度={width}m, 高度={height}m"
            
            self.set_parameters(params)
            self.get_logger().info(log_msg)
            self.append_info(f"✅ {log_msg}")
            
        except Exception as e:
            self.get_logger().error(f"更新围栏配置失败: {e}")
            self.append_warning(f"❌ 更新围栏配置失败: {e}")
    
    def set_led_infection_mode_callback(self, enabled):
        """
        设置LED传染模式开关
        
        Args:
            enabled: True开启传染模式，False关闭传染模式
        """
        try:
            self._led_infection_enabled = bool(enabled)
            status = "已开启" if self._led_infection_enabled else "已关闭"
            self.get_logger().info(f"LED传染模式{status}")
            
            # 如果关闭传染模式，恢复所有被传染USV的原始LED状态
            if not self._led_infection_enabled and self._usv_led_modes:
                self.get_logger().info("正在恢复所有被传染USV的原始LED状态...")
                for dst_id in list(self._usv_led_modes.keys()):
                    mode, color = self._usv_led_modes[dst_id]
                    if dst_id in self.usv_manager.led_pubs:
                        if mode == 'color_select':
                            cmd = f"color_select|{color[0]},{color[1]},{color[2]}"
                        else:
                            cmd = mode
                        from std_msgs.msg import String
                        msg = String()
                        msg.data = cmd
                        self.publish_queue.put((self.usv_manager.led_pubs[dst_id], msg))
                        self.get_logger().info(f"已恢复 {dst_id} 的LED状态: {cmd}")
                # 清空传染状态
                self._usv_led_modes.clear()
                self._usv_infecting.clear()
                
        except Exception as e:
            self.get_logger().error(f"设置LED传染模式失败: {e}")

    def toggle_random_run(self, enabled):
        """切换随机运行模式"""
        self.random_run_enabled = enabled
        status = "开启" if enabled else "关闭"
        self.get_logger().info(f"🎲 随机运行模式已{status}")
        
        if enabled:
            if self.random_run_timer is None:
                self.random_run_timer = self.create_timer(self.random_run_interval, self._random_run_callback)
        else:
            if self.random_run_timer is not None:
                self.destroy_timer(self.random_run_timer)
                self.random_run_timer = None

    def _random_run_callback(self):
        """随机运行模式定时回调"""
        if not self.random_run_enabled:
            return
            
        import random
        import math
        
        # 获取在线 USV
        for usv_id, state in self.usv_manager.usv_states.items():
            # 检查是否在线且处于 OFFBOARD 模式且已解锁
            if not state.connected or not state.armed or state.mode != 'OFFBOARD':
                continue
                
            # 检查是否正在导航
            nav_cache = self.navigation_handler.nav_target_cache.get(usv_id)
            is_idle = True
            if nav_cache:
                status = nav_cache.get('status')
                # 如果状态是 PENDING 或 ACTIVE，且距离目标还远，则认为不空闲
                if status in ['PENDING', 'ACTIVE']:
                    dist = nav_cache.get('distance_remaining', 999.0)
                    if dist > 1.5: # 1.5米阈值，认为还没到
                        is_idle = False
            
            if is_idle:
                # 相对于 area_center 生成随机位置
                center_x = self._area_center.get('x', 0.0)
                center_y = self._area_center.get('y', 0.0)
                center_z = self._area_center.get('z', 0.0)
                
                fence_type = self._fence_config.get('type', 0)
                
                if fence_type == 0: # 圆柱形
                    radius = self._fence_config.get('radius', 50.0)
                    height = self._fence_config.get('height', 10.0)
                    
                    # 极坐标生成以保证在圆内
                    r = random.uniform(0, radius * 0.9) # 留 10% 余量
                    theta = random.uniform(0, 2 * math.pi)
                    
                    target_x = center_x + r * math.cos(theta)
                    target_y = center_y + r * math.sin(theta)
                    target_z = center_z + random.uniform(-height/2, height/2)
                else: # 长方体
                    lx = self._fence_config.get('length', 100.0)
                    wy = self._fence_config.get('width', 100.0)
                    hz = self._fence_config.get('height', 10.0)
                    
                    target_x = center_x + random.uniform(-lx/2 * 0.9, lx/2 * 0.9)
                    target_y = center_y + random.uniform(-wy/2 * 0.9, wy/2 * 0.9)
                    target_z = center_z + random.uniform(-hz/2 * 0.9, hz/2 * 0.9)
                
                # 随机姿态 (弧度)
                yaw = random.uniform(-math.pi, math.pi)
                roll = random.uniform(-0.15, 0.15) # 限制 roll/pitch 范围，避免倾覆
                pitch = random.uniform(-0.15, 0.15)
                
                self.get_logger().info(
                    f"🎲 Random Run: Sending goal to {usv_id} -> "
                    f"pos=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), yaw={yaw:.2f}"
                )
                self.send_nav_goal_via_topic(
                    usv_id, target_x, target_y, target_z,
                    yaw=yaw, use_yaw=True, timeout=300.0, roll=roll, pitch=pitch
                )

    # 销毁节点资源
    def destroy_node(self):
        """
        销毁节点资源
        """
        # 记录日志信息
        self.get_logger().info("销毁节点资源...")
        # 销毁所有订阅者
        for usv_id in list(self.usv_manager.usv_state_subs.keys()):
            self.destroy_subscription(self.usv_manager.usv_state_subs[usv_id])
            del self.usv_manager.usv_state_subs[usv_id]
        # 销毁所有模式发布者
        for usv_id in list(self.usv_manager.set_usv_mode_pubs.keys()):
            self.destroy_publisher(self.usv_manager.set_usv_mode_pubs[usv_id])
            del self.usv_manager.set_usv_mode_pubs[usv_id]
        # 销毁所有武装状态发布者
        for usv_id in list(self.usv_manager.set_usv_arming_pubs.keys()):
            self.destroy_publisher(self.usv_manager.set_usv_arming_pubs[usv_id])
            del self.usv_manager.set_usv_arming_pubs[usv_id]
        # 销毁所有LED发布者
        for usv_id in list(self.usv_manager.led_pubs.keys()):
            self.destroy_publisher(self.usv_manager.led_pubs[usv_id])
            del self.usv_manager.led_pubs[usv_id]
        # 销毁所有声音发布者
        for usv_id in list(self.usv_manager.sound_pubs.keys()):
            self.destroy_publisher(self.usv_manager.sound_pubs[usv_id])
            del self.usv_manager.sound_pubs[usv_id]
        # 销毁所有动作发布者
        for usv_id in list(self.usv_manager.action_pubs.keys()):
            self.destroy_publisher(self.usv_manager.action_pubs[usv_id])
            del self.usv_manager.action_pubs[usv_id]
        # 调用父类的销毁方法
        super().destroy_node()


def main(args=None):
    """地面站节点入口函数"""
    rclpy.init(args=args)
    
    node = GroundStationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()