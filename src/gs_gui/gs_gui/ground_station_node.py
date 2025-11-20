"""
地面站GUI ROS2节点实现文件
该文件实现了地面站GUI与USV系统通信的核心功能
包括状态监控、命令发送、导航控制等功能
"""

import json
from collections import defaultdict, deque
from datetime import datetime
import yaml
import os
import rclpy 
from rclpy.node import Node  # 从rclpy.node模块导入Node类，用于创建ROS2节点
from rclpy.parameter import Parameter  # 导入Parameter类，用于参数设置
from geometry_msgs.msg import PoseStamped  # 从geometry_msgs.msg模块导入PoseStamped消息类型，用于位姿信息
from common_interfaces.msg import UsvStatus  # 从common_interfaces.msg模块导入UsvStatus消息类型
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # 从rclpy.qos模块导入QoSProfile和QoSReliabilityPolicy，用于设置服务质量
import queue  # 导入queue模块，用于创建消息队列
import threading  # 导入threading模块，用于多线程处理
from std_msgs.msg import String # 导入 String 消息类型
import weakref  # 导入weakref模块，用于弱引用
import tf2_ros
from geometry_msgs.msg import TransformStamped

# 导入分解后的模块
from .usv_manager import UsvManager
from .cluster_controller import ClusterController
from .command_processor import CommandProcessor
from .led_infection import LedInfectionHandler

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
        self.declare_parameter('fleet_config_file', '')
        self.declare_parameter('step_timeout', float(self.DEFAULT_STEP_TIMEOUT))
        self.declare_parameter('max_retries', int(self.DEFAULT_MAX_RETRIES))
        self.declare_parameter('min_ack_rate_for_proceed', float(self.MIN_ACK_RATE_FOR_PROCEED))
        self.declare_parameter('offline_grace_period', 5.0)
        self.declare_parameter('ack_resend_interval', 2.0)
        self.declare_parameter('cluster_action_timeout', 300.0)
        self.declare_parameter('area_center_x', 0.0)
        self.declare_parameter('area_center_y', 0.0)
        self.declare_parameter('area_center_z', 0.0)
        self.declare_parameter('area_center_frame', 'map')

        # 初始化子模块
        self.usv_manager = UsvManager(self)
        self.cluster_controller = ClusterController(self)
        self.command_processor = CommandProcessor(self)
        self.led_infection_handler = LedInfectionHandler(self)

        # 预检与状态文本缓存 (线程安全)
        self._vehicle_messages = defaultdict(lambda: deque(maxlen=50))
        self._prearm_warnings = defaultdict(dict)
        self._prearm_ready = ThreadSafeDict()
        self._sensor_status_cache = ThreadSafeDict()
        # 传感器健康缓存 (SYS_STATUS)
        self._sensor_health_cache = ThreadSafeDict()
        self._heartbeat_status_cache = ThreadSafeDict()
        # 导航目标信息缓存（用于导航面板显示）
        self._usv_nav_target_cache = ThreadSafeDict()
        
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

        # 将最新参数同步给集群控制器
        self.cluster_controller.configure(
            resend_interval=self._ack_resend_interval,
            action_timeout=self._cluster_action_timeout,
        )

        # ========== 从配置文件加载USV列表（Domain隔离架构）==========
        # 注意：必须在参数声明之后调用
        self._fleet_config = self._load_fleet_config()
        self._static_usv_list = self._extract_usv_list_from_config()

        # 读取 area_center 参数
        try:
            ax = float(self.get_parameter('area_center_x').get_parameter_value().double_value)
            ay = float(self.get_parameter('area_center_y').get_parameter_value().double_value)
            az = float(self.get_parameter('area_center_z').get_parameter_value().double_value)
            afr = str(self.get_parameter('area_center_frame').get_parameter_value().string_value)
            self._area_center = {'x': ax, 'y': ay, 'z': az, 'frame': afr}
        except Exception:
            self._area_center = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'frame': 'map'}

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
        # LED传染模式开关（默认开启）
        self._led_infection_enabled = True
     
        # 初始化命名空间检测历史记录
        self._ns_detection_history = []  # 用于存储命名空间检测历史记录的列表
        # 记录每个 USV 最后一次收到状态消息的时间戳（秒）(线程安全)
        self._ns_last_seen = ThreadSafeDict()

        # 创建定时器
        # 注意：在Domain隔离架构下，不再使用动态节点发现，而是从配置文件读取USV列表
        # 定时器仅用于验证topic是否可用（检测USV离线状态）
        self.ns_timer = self.create_timer(5.0, self.check_usv_topics_availability)  # USV话题可用性检查定时器
        self.target_timer = self.create_timer(self.CLUSTER_TARGET_PUBLISH_PERIOD, self.publish_cluster_targets_callback)  # 集群目标发布定时器，定期发布集群目标
        self.infect_check_timer = self.create_timer(self.INFECTION_CHECK_PERIOD, self.check_usv_infect)  # 传染检查定时器，定期检查USV之间的传染逻辑
        # 添加高频状态推送定时器，确保 Ready 检查等信息能快速更新到 GUI（类似 QGC 的灵敏响应）
        self.state_push_timer = self.create_timer(0.2, self.push_state_updates)  # 200ms = 5Hz，优化性能
        
        # 使用静态配置初始化USV订阅和发布者（Domain隔离架构）
        self.initialize_usv_from_config()

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

    def _load_fleet_config(self):
        """
        从配置文件加载USV集群配置（用于Domain隔离架构）
        
        Returns:
            dict: 配置字典，如果加载失败则返回None
        """
        try:
            # 1. 优先从ROS参数获取配置文件路径
            fleet_config_file = self.get_parameter('fleet_config_file').get_parameter_value().string_value
            
            # 2. 如果参数为空，使用默认路径
            if not fleet_config_file:
                # 尝试从install目录读取
                try:
                    from ament_index_python.packages import get_package_share_directory
                    share_dir = get_package_share_directory('gs_bringup')
                    fleet_config_file = os.path.join(share_dir, 'config', 'usv_fleet.yaml')
                except Exception:
                    # 如果失败，使用相对路径
                    fleet_config_file = os.path.expanduser('~/usv_workspace/src/gs_bringup/config/usv_fleet.yaml')
            
            # 3. 加载YAML文件
            if os.path.exists(fleet_config_file):
                with open(fleet_config_file, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                    self.get_logger().info(f"✓ 已加载fleet配置文件: {fleet_config_file}")
                    return config
            else:
                self.get_logger().warn(f"配置文件不存在: {fleet_config_file}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"加载fleet配置文件失败: {e}")
            return None
    
    def _extract_usv_list_from_config(self):
        """
        从配置中提取已启用的USV列表
        
        Returns:
            list: USV命名空间列表，例如 ['usv_01', 'usv_02', 'usv_03']
        """
        usv_list = []
        
        if not self._fleet_config:
            self.get_logger().warn("⚠️  未加载fleet配置，将使用空USV列表")
            return usv_list
        
        try:
            fleet = self._fleet_config.get('usv_fleet', {})
            for usv_id, config in fleet.items():
                # 只添加启用的USV
                if config.get('enabled', False):
                    namespace = config.get('namespace', usv_id)
                    usv_list.append(namespace)
                    self.get_logger().info(f"  ├─ {namespace} (已启用)")
                else:
                    self.get_logger().info(f"  ├─ {usv_id} (已禁用)")
            
            self.get_logger().info(f"✓ 从配置文件读取到 {len(usv_list)} 艘USV: {usv_list}")
            
        except Exception as e:
            self.get_logger().error(f"解析USV列表失败: {e}")
        
        return usv_list
    
    def initialize_usv_from_config(self):
        """
        基于配置文件静态初始化所有USV的订阅者和发布者
        （适用于Domain隔离架构，不依赖DDS节点发现）
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 初始化USV订阅者和发布者（静态配置模式）")
        self.get_logger().info("=" * 60)
        
        if not self._static_usv_list:
            self.get_logger().warn("⚠️  USV列表为空，请检查配置文件")
            return
        
        # 为每个配置的USV创建订阅和发布
        for usv_id in self._static_usv_list:
            try:
                # 添加命名空间（需要/前缀）
                ns = f"/{usv_id}" if not usv_id.startswith('/') else usv_id
                self.usv_manager.add_usv_namespace(ns)
                
                # 记录当前时间
                try:
                    now_sec = self.get_clock().now().nanoseconds / 1e9
                except Exception:
                    now_sec = 0.0
                self._ns_last_seen[usv_id] = now_sec
                
                self.get_logger().info(f"✓ {usv_id} 初始化完成")
                
            except Exception as e:
                self.get_logger().error(f"✗ {usv_id} 初始化失败: {e}")
        
        # 更新last_ns_list
        self.last_ns_list = self._static_usv_list.copy()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"✓ 完成初始化 {len(self._static_usv_list)} 艘USV")
        self.get_logger().info("=" * 60)
    
    def check_usv_topics_availability(self):
        """
        定期检查USV topic是否可用（用于检测离线状态）
        
        在Domain隔离架构下，无法通过节点发现来检测USV上下线，
        而是通过检查topic上是否有数据来判断USV是否在线。
        
        注意：这个方法不会添加或删除USV，只会标记离线状态。
        """
        if not self._static_usv_list:
            return
        
        try:
            now_sec = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            now_sec = 0.0
        
        # 检查每个USV的最后接收时间
        offline_threshold = 10.0  # 10秒未收到数据认为离线
        state_changed = False  # 标记是否有状态变化
        
        for usv_id in self._static_usv_list:
            last_seen = self._ns_last_seen.get(usv_id, 0.0)
            elapsed = now_sec - last_seen
            
            # 如果USV还没有状态条目，创建初始状态
            if usv_id not in self.usv_states:
                self.usv_states[usv_id] = {
                    'namespace': usv_id,
                    'connected': False,  # 初始为离线，等待第一次数据
                    'mode': 'UNKNOWN',
                    'armed': False,
                }
            
            # 更新状态字典中的连接状态
            if elapsed > offline_threshold:
                # 标记为离线
                if self.usv_states[usv_id].get('connected', True):
                    self.usv_states[usv_id]['connected'] = False
                    state_changed = True
                    self.get_logger().warn(f"⚠️  {usv_id} 已离线（{elapsed:.1f}s未收到数据）")
            else:
                # 标记为在线
                if not self.usv_states[usv_id].get('connected', False):
                    self.usv_states[usv_id]['connected'] = True
                    state_changed = True
                    self.get_logger().info(f"✓ {usv_id} 已上线")
        
        # 如果有状态变化，通知GUI更新
        if state_changed:
            try:
                self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))
            except Exception as e:
                self.get_logger().debug(f"推送状态更新失败: {e}")
    
    # =====================================================
    # 以下是原有的动态节点发现方法（保留用于兼容性）
    # 在Domain隔离架构下不再使用
    # =====================================================
    
    def update_subscribers_and_publishers(self):
        """
        [已废弃 - 仅用于兼容性] 动态发现USV节点
        
        ⚠️  注意：在Domain隔离架构下，此方法不再使用！
        
        原理：通过DDS节点发现机制动态检测USV上下线
        限制：在每个USV使用独立Domain ID的架构下，地面站无法通过DDS发现USV节点
        
        新方案：使用 initialize_usv_from_config() 从配置文件静态加载USV列表
        
        该方法定期检查系统中的节点命名空间，
        为新连接的USV创建订阅者和发布者，
        为断开的USV销毁订阅者和发布者
        """
        # 在Domain隔离架构下，直接返回不执行
        if self._static_usv_list:
            return
        # 获取当前系统中的节点名称和命名空间
        node_names_and_namespaces = self.get_node_names_and_namespaces()

        # 筛选出命名空间以 '/usv_' 开头的节点（忽略进一步的子命名空间），这些命名空间代表在线 USV
        current_nodes_info = []
        for name, ns in node_names_and_namespaces:
            if not ns.startswith('/usv_'):
                continue
            # 仅保留一级命名空间（例如 /usv_01），避免嵌套命名空间干扰
            if ns.count('/') > 1:
                continue
            current_nodes_info.append((name, ns))
        # 提取唯一的 USV 命名空间列表
        current_ns_list = list({ns for _, ns in current_nodes_info})
        
        # 添加稳定性检测机制：多次检测相同结果才确认变化
        # 记录当前检测结果到历史记录中
        self._ns_detection_history.append(set(current_ns_list))
        # 限制历史记录长度，避免列表无限增长
        if len(self._ns_detection_history) > 5:
            self._ns_detection_history.pop(0)

        # 需要连续多次检测得到一致结果才进行变更处理
        required_samples = 2  # 从 3 减少到 2，加快检测确认速度
        if len(self._ns_detection_history) < required_samples:
            return

        recent_ns_sets = self._ns_detection_history[-required_samples:]
        reference_set = recent_ns_sets[0]
        if not all(ns_set == reference_set for ns_set in recent_ns_sets[1:]):
            return

        # 使用稳定的结果进行后续处理
        stable_ns_set = set(reference_set)
        ns_map = {ns.lstrip('/'): ns for ns in stable_ns_set}
        normalized_ns_list = list(ns_map.keys())

        previous_last_ns = list(self.last_ns_list)

        # 如果命名空间列表没有变化，直接返回
        if set(normalized_ns_list) == set(self.last_ns_list):
            return
        
        # 获取已存在的命名空间集合
        existing_ns = set(self.usv_manager.usv_state_subs.keys())
        # 计算新增的命名空间集合
        new_ns = set(normalized_ns_list) - existing_ns
        # 计算移除的命名空间集合
        removed_ns = existing_ns - set(normalized_ns_list)

        # 处理新增的USV命名空间 - 只显示汇总日志
        if new_ns:
            # 统计该 USV 下有多少个节点
            node_counts = {}
            for node_name, ns in current_nodes_info:
                norm_ns = ns.lstrip('/')
                if norm_ns in new_ns:
                    node_counts[norm_ns] = node_counts.get(norm_ns, 0) + 1
            
            # 为每个新 USV 输出一条汇总日志
            for usv_id in new_ns:
                ns = ns_map.get(usv_id, f"/{usv_id}")
                node_count = node_counts.get(usv_id, 0)
                self.usv_manager.add_usv_namespace(ns)
                # 记录当前时间，避免刚加入后立刻因为没有状态消息被误判离线
                try:
                    now_sec = self.get_clock().now().nanoseconds / 1e9
                except Exception:
                    now_sec = 0.0
                self._ns_last_seen[usv_id] = now_sec
                # 输出简洁的汇总日志
                self.get_logger().info(f"[OK] USV上线: {ns} (检测到 {node_count} 个节点)")

        # 处理移除的USV命名空间
        safe_removed_ns = []
        for usv_id in removed_ns:
            last_seen = self._ns_last_seen.get(usv_id)
            allow_remove = False
            if last_seen is None:
                # 从未收到过状态消息，说明订阅尚未建立成功，可以直接移除
                allow_remove = True
            else:
                try:
                    now_sec = self.get_clock().now().nanoseconds / 1e9
                except Exception:
                    now_sec = last_seen
                elapsed = now_sec - last_seen
                if elapsed >= self._ns_offline_grace_period:
                    allow_remove = True
                else:
                    # 在宽限期内仍有状态消息，暂不移除，等待后续检测
                    self.get_logger().debug(
                        f"命名空间 {usv_id} 暂未从 ROS 图中检测到，但在 {elapsed:.1f}s 前仍有状态更新，延迟移除")
            if allow_remove:
                ns = ns_map.get(usv_id, f"/{usv_id}")
                self.usv_manager.remove_usv_namespace(ns)
                self._ns_last_seen.pop(usv_id, None)
                safe_removed_ns.append(ns)
                self.get_logger().info(f"USV节点断开连接，已移除命名空间: {ns}")

        # 计算最终的逻辑在线列表：稳定检测结果 + 当前仍在宽限期内的命名空间
        removed_now = {ns.lstrip('/') for ns in safe_removed_ns}
        postponed_ns = removed_ns - removed_now
        effective_ns = list(normalized_ns_list)
        if postponed_ns:
            for ns in previous_last_ns:
                if ns in postponed_ns and ns not in effective_ns:
                    effective_ns.append(ns)
        self.last_ns_list = effective_ns

    # 通过Action方式发送导航目标点
    def _validate_target_position(self, x, y, z):
        """
        验证目标点是否在安全范围内
        
        Args:
            x (float): 目标点X坐标
            y (float): 目标点Y坐标
            z (float): 目标点Z坐标
            
        Raises:
            ValueError: 如果目标点超出安全范围
        """
        import math
        
        # 定义安全范围参数（可根据实际需求调整）
        MAX_DISTANCE = 500.0  # 最大水平距离 500m
        MAX_ALTITUDE = 10.0   # 最大高度 10m（USV 通常在水面）
        
        # 计算2D距离
        distance_2d = math.sqrt(x**2 + y**2)
        
        # 检查水平距离
        if distance_2d > MAX_DISTANCE:
            raise ValueError(
                f"目标点距离过远: {distance_2d:.2f}m > {MAX_DISTANCE}m"
            )
        
        # 检查高度（通常 USV 不应该有太大的Z坐标）
        if abs(z) > MAX_ALTITUDE:
            raise ValueError(
                f"目标点高度异常: {abs(z):.2f}m > {MAX_ALTITUDE}m"
            )

    # ==================== 基于话题的导航方法 ====================
    
    def send_nav_goal_via_topic(self, usv_id, x, y, z=0.0, yaw=0.0, timeout=300.0):
        """
        通过话题方式向指定USV发送导航目标点 (新版本,替代Action)
        
        优势:
        - 更适合跨Domain通信
        - 不依赖Action的复杂服务机制
        - 在Domain Bridge中更容易配置和调试
        
        Args:
            usv_id (str): USV标识符
            x (float): 目标点X坐标
            y (float): 目标点Y坐标
            z (float): 目标点Z坐标
            yaw (float): 目标偏航角(弧度)
            timeout (float): 超时时间(秒)
        
        Returns:
            bool: 发送是否成功
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
        
        # 生成唯一的目标ID
        with self._goal_id_lock:
            goal_id = self._next_goal_id
            self._next_goal_id += 1
        
        # 记录目标ID到USV的映射
        self._goal_to_usv[goal_id] = usv_id
        
        # 构造目标消息
        goal_msg = NavigationGoal()
        goal_msg.goal_id = goal_id
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.pose.position.x = float(x)
        goal_msg.target_pose.pose.position.y = float(y)
        goal_msg.target_pose.pose.position.z = float(z)
        
        # 设置航向 (Quaternion)
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.target_pose.pose.orientation.x = q[0]
        goal_msg.target_pose.pose.orientation.y = q[1]
        goal_msg.target_pose.pose.orientation.z = q[2]
        goal_msg.target_pose.pose.orientation.w = q[3]
        
        goal_msg.timeout = timeout
        goal_msg.timestamp = self.get_clock().now().to_msg()
        
        # 发布目标
        pub = self.usv_manager.navigation_goal_pubs[usv_id]
        pub.publish(goal_msg)
        
        # 更新缓存和状态
        self._usv_nav_target_cache[usv_id] = {
            'goal_id': goal_id,
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'yaw': float(yaw),
            'step': self.run_step,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        # 更新导航状态为执行中
        self.ros_signal.nav_status_update.emit(usv_id, "执行中")
        
        self.get_logger().info(
            f"📤 {usv_id} 导航目标已发送 [ID={goal_id}]: "
            f"({x:.1f}, {y:.1f}, {z:.1f}), 超时={timeout:.0f}s")
        
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
            'distance_to_goal': msg.distance_to_goal,
            'heading_error': msg.heading_error,
            'estimated_time': msg.estimated_time
        })()
        self.ros_signal.navigation_feedback.emit(usv_id, feedback_obj)
    
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
                
                # 支持z坐标
                self.send_nav_goal_via_topic(usv_id, p_local.get('x', 0.0), p_local.get('y', 0.0), p_local.get('z', 0.0), yaw, 300.0)
        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"处理离群目标点失败: {e}")

    # 委托给子模块的方法
    def set_manual_callback(self, msg):
        self.command_processor.set_manual_callback(msg)

    def set_hold_callback(self, msg):
        self.command_processor.set_hold_callback(msg)

    def set_guided_callback(self, msg):
        self.command_processor.set_guided_callback(msg)

    def set_arco_callback(self, msg):
        self.command_processor.set_arco_callback(msg)

    def set_steering_callback(self, msg):
        self.command_processor.set_steering_callback(msg)

    def set_arming_callback(self, msg):
        self.command_processor.set_arming_callback(msg)

    def set_disarming_callback(self, msg):
        self.command_processor.set_disarming_callback(msg)

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
        """
        飞控重启回调
        
        发送 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 命令重启飞控
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        try:
            # 导入 MAVROS 命令服务
            from mavros_msgs.srv import CommandLong
            
            # 创建服务客户端（MAVROS 命令服务在节点命名空间下，不需要 mavros 子命名空间）
            service_name = f'/{usv_namespace}/cmd/command'
            client = self.create_client(CommandLong, service_name)
            
            # 等待服务可用
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f'[X] 服务不可用: {service_name}')
                try:
                    self.ros_signal.node_info.emit(f'[X] {usv_namespace} 飞控重启失败：服务不可用')
                except Exception:
                    pass
                return
            
            # 构建重启命令
            request = CommandLong.Request()
            request.broadcast = False
            request.command = 246  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            request.confirmation = 0
            request.param1 = 1.0   # 重启飞控 (1=reboot autopilot)
            request.param2 = 0.0   # 不重启机载计算机
            request.param3 = 0.0
            request.param4 = 0.0
            request.param5 = 0.0
            request.param6 = 0.0
            request.param7 = 0.0
            
            # 异步发送命令
            future = client.call_async(request)
            future.add_done_callback(
                lambda f: self._handle_reboot_response(f, usv_namespace)
            )
            
            self.get_logger().info(f'[OK] 已向 {usv_namespace} 发送飞控重启命令 (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)')
            try:
                self.ros_signal.node_info.emit(f'[OK] 已向 {usv_namespace} 发送飞控重启命令')
            except Exception:
                pass
            
        except Exception as e:
            self.get_logger().error(f'[X] 发送重启命令失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] 发送重启命令失败: {e}')
            except Exception:
                pass
    
    def _handle_reboot_response(self, future, usv_namespace):
        """处理重启命令响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[OK] {usv_namespace} 飞控重启命令已确认')
                try:
                    self.ros_signal.node_info.emit(f'[OK] {usv_namespace} 飞控重启命令已确认，请等待 10-20 秒')
                except Exception:
                    pass
            else:
                self.get_logger().warn(
                    f'[!] {usv_namespace} 飞控重启命令失败: result={response.result}'
                )
        except Exception as e:
            self.get_logger().error(f'[X] 处理重启命令响应失败: {e}')
    
    def reboot_companion_callback(self, usv_namespace):
        """
        机载计算机重启回调
        
        通过 SSH 直接重启机载计算机（更可靠的方式）
        备选方案：MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN（某些飞控可能不支持）
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        try:
            # 方法 1: 通过 SSH 直接重启（推荐，更可靠）
            import subprocess
            import os
            import yaml
            
            # 读取 usv_fleet.yaml 获取机载计算机信息
            workspace_path = os.path.expanduser('~/usv_workspace')
            config_file = os.path.join(
                workspace_path,
                'install/gs_bringup/share/gs_bringup/config/usv_fleet.yaml'
            )
            
            if os.path.exists(config_file):
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                    fleet_config = config.get('usv_fleet', {})
                    
                    if usv_namespace in fleet_config:
                        usv_config = fleet_config[usv_namespace]
                        hostname = usv_config.get('hostname')
                        username = usv_config.get('username')
                        
                        if hostname and username:
                            # 构建 SSH 重启命令（使用 systemctl reboot，无需 sudo）
                            # 注意：某些系统可能需要 sudo，如果失败会自动回退到 MAVLink
                            ssh_cmd = [
                                'ssh',
                                '-o', 'StrictHostKeyChecking=no',
                                '-o', 'ConnectTimeout=5',
                                f'{username}@{hostname}',
                                'systemctl reboot || sudo reboot'  # 先尝试 systemctl，失败则用 sudo
                            ]
                            
                            # 异步执行 SSH 命令
                            subprocess.Popen(
                                ssh_cmd,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL
                            )
                            
                            self.get_logger().info(f'[OK] 已向 {usv_namespace} ({hostname}) 发送 SSH 重启命令')
                            try:
                                self.ros_signal.node_info.emit(
                                    f'[OK] 已向 {usv_namespace} 发送重启命令，系统将在 30-60 秒后重新上线'
                                )
                            except Exception:
                                pass
                            return
                        else:
                            self.get_logger().error(f'[X] {usv_namespace} 配置缺少 hostname 或 username')
                    else:
                        self.get_logger().error(f'[X] 未找到 {usv_namespace} 的配置')
            else:
                self.get_logger().error(f'[X] 配置文件不存在: {config_file}')
            
            # 方法 2: 备选 - 通过 MAVLink 命令（如果 SSH 不可用或配置文件缺失）
            self.get_logger().warn(f'[!] SSH 重启失败，尝试 MAVLink 命令（可能不被支持）')
            self._reboot_companion_via_mavlink(usv_namespace)
            
        except Exception as e:
            self.get_logger().error(f'[X] 机载计算机重启失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] 机载计算机重启失败: {e}')
            except Exception:
                pass
    
    def _reboot_companion_via_mavlink(self, usv_namespace):
        """
        通过 MAVLink 命令重启机载计算机（备选方案）
        
        注意：某些飞控固件可能不支持此命令
        """
        try:
            from mavros_msgs.srv import CommandLong
            
            service_name = f'/{usv_namespace}/cmd/command'
            client = self.create_client(CommandLong, service_name)
            
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f'[X] MAVROS 服务不可用: {service_name}')
                return
            
            # 构建 MAVLink 重启命令
            request = CommandLong.Request()
            request.broadcast = False
            request.command = 246  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            request.confirmation = 0
            request.param1 = 0.0   # 不重启飞控
            request.param2 = 3.0   # 重启机载计算机
            request.param3 = 0.0
            request.param4 = 0.0
            request.param5 = 0.0
            request.param6 = 0.0
            request.param7 = 0.0
            
            future = client.call_async(request)
            future.add_done_callback(
                lambda f: self._handle_companion_reboot_response(f, usv_namespace)
            )
            
            self.get_logger().info(f'[OK] 已发送 MAVLink 重启命令到 {usv_namespace}')
            
        except Exception as e:
            self.get_logger().error(f'[X] MAVLink 重启命令失败: {e}')
    
    def _handle_companion_reboot_response(self, future, usv_namespace):
        """处理机载计算机重启命令响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[OK] {usv_namespace} 机载计算机重启命令已确认')
                try:
                    self.ros_signal.node_info.emit(f'[OK] {usv_namespace} 机载计算机重启命令已确认，系统将在 30-60 秒后重新上线')
                except Exception:
                    pass
            else:
                self.get_logger().warn(
                    f'[!] {usv_namespace} 机载计算机重启命令失败: result={response.result}'
                )
                try:
                    self.ros_signal.node_info.emit(f'[!] {usv_namespace} 机载计算机重启命令失败')
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f'[X] 重启命令响应处理失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] 重启命令响应处理失败: {e}')
            except Exception:
                pass

    def set_home_position_callback(self, usv_namespace, use_current, coords):
        """
        设置 Home Position 回调
        
        发送 MAV_CMD_DO_SET_HOME 命令设置 Home Position
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
            use_current: 是否使用当前位置（True=使用当前位置, False=使用指定坐标）
            coords: 坐标字典 {'lat': float, 'lon': float, 'alt': float}（仅当 use_current=False 时使用）
        """
        try:
            # 导入 MAVROS 命令服务
            from mavros_msgs.srv import CommandLong
            
            # 创建服务客户端
            service_name = f'/{usv_namespace}/cmd/command'
            client = self.create_client(CommandLong, service_name)
            
            # 等待服务可用
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f'[X] 服务不可用: {service_name}')
                try:
                    self.ros_signal.node_info.emit(f'[X] {usv_namespace} 设置 Home Position 失败：服务不可用')
                except Exception:
                    pass
                return
            
            # 构建 MAV_CMD_DO_SET_HOME 命令
            request = CommandLong.Request()
            request.broadcast = False
            request.command = 179  # MAV_CMD_DO_SET_HOME
            request.confirmation = 0
            
            if use_current:
                # 使用当前位置作为 Home Position
                request.param1 = 1.0  # 1=使用当前位置
                request.param2 = 0.0
                request.param3 = 0.0
                request.param4 = 0.0
                request.param5 = 0.0  # 纬度（使用当前位置时忽略）
                request.param6 = 0.0  # 经度（使用当前位置时忽略）
                request.param7 = 0.0  # 高度（使用当前位置时忽略）
                
                self.get_logger().info(f'[OK] 设置 {usv_namespace} Home Position 为当前位置')
            else:
                # 使用指定坐标作为 Home Position
                request.param1 = 0.0  # 0=使用指定坐标
                request.param2 = 0.0
                request.param3 = 0.0
                request.param4 = 0.0
                request.param5 = float(coords.get('lat', 0.0))  # 纬度
                request.param6 = float(coords.get('lon', 0.0))  # 经度
                request.param7 = float(coords.get('alt', 0.0))  # 高度
                
                self.get_logger().info(
                    f'[OK] 设置 {usv_namespace} Home Position 为指定坐标: '
                    f'lat={request.param5:.7f}, lon={request.param6:.7f}, alt={request.param7:.2f}m'
                )
            
            # 异步发送命令
            future = client.call_async(request)
            future.add_done_callback(
                lambda f: self._handle_set_home_response(f, usv_namespace, use_current, coords)
            )
            
            try:
                if use_current:
                    self.ros_signal.node_info.emit(f'[OK] 已向 {usv_namespace} 发送设置 Home Position 命令（使用当前位置）')
                else:
                    self.ros_signal.node_info.emit(
                        f'[OK] 已向 {usv_namespace} 发送设置 Home Position 命令\n'
                        f'    坐标: {coords.get("lat"):.7f}, {coords.get("lon"):.7f}, {coords.get("alt"):.2f}m'
                    )
            except Exception:
                pass
            
        except Exception as e:
            self.get_logger().error(f'[X] 发送设置 Home Position 命令失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] 发送设置 Home Position 命令失败: {e}')
            except Exception:
                pass
    
    def _handle_set_home_response(self, future, usv_namespace, use_current, coords):
        """处理设置 Home Position 命令响应"""
        try:
            response = future.result()
            if response.success:
                if use_current:
                    msg = f'[OK] {usv_namespace} Home Position 已设置为当前位置'
                else:
                    msg = (
                        f'[OK] {usv_namespace} Home Position 已设置为指定坐标\n'
                        f'    坐标: {coords.get("lat"):.7f}, {coords.get("lon"):.7f}, {coords.get("alt"):.2f}m'
                    )
                self.get_logger().info(msg)
                try:
                    self.ros_signal.node_info.emit(msg)
                except Exception:
                    pass
            else:
                self.get_logger().warn(
                    f'[!] {usv_namespace} 设置 Home Position 命令失败: result={response.result}'
                )
                try:
                    self.ros_signal.node_info.emit(f'[!] {usv_namespace} 设置 Home Position 命令失败')
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f'[X] 处理设置 Home Position 命令响应失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] 处理设置 Home Position 命令响应失败: {e}')
            except Exception:
                pass

    def shutdown_usv_callback(self, usv_namespace):
        """
        优雅关闭USV节点回调（通过ROS服务）
        
        调用USV端的shutdown_service来优雅关闭所有节点
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        try:
            # 导入Trigger服务
            from std_srvs.srv import Trigger
            
            # 创建服务客户端
            service_name = f'/{usv_namespace}/shutdown_all'
            client = self.create_client(Trigger, service_name)
            
            # 等待服务可用（3秒超时）
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f'[X] 关闭服务不可用: {service_name}')
                try:
                    self.ros_signal.node_info.emit(f'[X] {usv_namespace} 关闭失败：服务不可用（USV可能已离线）')
                except Exception:
                    pass
                return
            
            # 构建请求
            request = Trigger.Request()
            
            self.get_logger().info(f'[->] 正在关闭 {usv_namespace} 的所有节点...')
            try:
                self.ros_signal.node_info.emit(f'[->] 正在关闭 {usv_namespace} 的所有节点...')
            except Exception:
                pass
            
            # 异步调用服务
            future = client.call_async(request)
            future.add_done_callback(
                lambda f: self._handle_shutdown_response(f, usv_namespace)
            )
            
        except Exception as e:
            self.get_logger().error(f'[X] 发送关闭命令失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] {usv_namespace} 发送关闭命令失败: {e}')
            except Exception:
                pass
    
    def _handle_shutdown_response(self, future, usv_namespace):
        """处理USV关闭服务响应"""
        try:
            response = future.result()
            if response.success:
                msg = f'[OK] {usv_namespace} 节点关闭成功: {response.message}'
                self.get_logger().info(msg)
                try:
                    self.ros_signal.node_info.emit(msg)
                except Exception:
                    pass
            else:
                msg = f'[!] {usv_namespace} 节点关闭失败: {response.message}'
                self.get_logger().warn(msg)
                try:
                    self.ros_signal.node_warning.emit(msg)
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f'[X] 处理关闭命令响应失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] {usv_namespace} 关闭命令响应处理失败: {e}')
            except Exception:
                pass

    def handle_status_text(self, usv_id, msg):
        """处理飞控 status_text 消息，收集预检提示与车辆消息."""
        if msg is None:
            return

        text_raw = getattr(msg, 'text', '') or ''
        text = text_raw.strip()
        if not text:
            return
        
        try:
            severity = int(getattr(msg, 'severity', 6))
        except (TypeError, ValueError):
            severity = 6
        
        # 根据 severity 输出到不同窗口
        # 飞控状态文本消息（STATUSTEXT）不发送到 warning 窗口，也不发送到 info 窗口
        # 只以日志形式记录，避免干扰用户操作
        # 其他模块的错误和警告仍会正常显示在 warning 窗口
        
        # 所有飞控消息仅记录到 ROS 日志，不显示在 GUI 窗口
        # 用户可以通过日志文件或 rqt_console 查看这些消息
        
        # 根据严重性级别记录到不同的日志等级
        if severity <= 2:  # EMERGENCY/ALERT/CRITICAL
            self.get_logger().error(f"[FCU-CRITICAL] {usv_id}: {text}")
        elif severity == 3:  # ERROR
            self.get_logger().error(f"[FCU-ERROR] {usv_id}: {text}")
        elif severity == 4:  # WARNING
            self.get_logger().warn(f"[FCU-WARNING] {usv_id}: {text}")
        else:  # NOTICE/INFO/DEBUG
            self.get_logger().info(f"[FCU-INFO] {usv_id}: {text}")

        now_sec = self._now_seconds()
        entry = {
            'text': text,
            'severity': severity,
            'severity_label': self._severity_to_label(severity),
            'time': self._format_time(now_sec),
            'timestamp': now_sec,
        }
        self._vehicle_messages[usv_id].appendleft(entry)

        upper_text = text.upper()
        warnings = self._prearm_warnings[usv_id]
        if 'PREARM' in upper_text:
            if severity <= 4 and 'PASS' not in upper_text and 'OK' not in upper_text:
                warnings[text] = now_sec
            else:
                warnings.clear()

        self._cleanup_prearm_warnings(usv_id, now_sec)

        state = self.usv_states.get(usv_id)
        if state is None:
            state = {'namespace': usv_id}
            self.usv_states[usv_id] = state

        # 记录消息到达时间（用于变化检测）
        if not hasattr(self, '_last_statustext_time'):
            self._last_statustext_time = {}
        self._last_statustext_time[usv_id] = now_sec
        
        self.augment_state_payload(usv_id, state)

        try:
            self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))
        except Exception as exc:
            self.get_logger().warn(f"推送 {usv_id} 状态文本更新失败: {exc}")

    def handle_sys_status(self, usv_id, msg):
        """
        处理飞控 SYS_STATUS 消息，缓存传感器健康状态
        
        根据 QGC 实现，使用 onboard_control_sensors_health 位掩码来判断传感器健康状态。
        关键传感器位定义 (MAV_SYS_STATUS_SENSOR):
            0x01 (bit 0): 3D gyro
            0x02 (bit 1): 3D accelerometer  
            0x04 (bit 2): 3D magnetometer
            0x08 (bit 3): absolute pressure
            0x20 (bit 5): GPS
        
        Args:
            usv_id: USV标识符
            msg: mavros_msgs/SysStatus 消息
        """
        if msg is None:
            return
        
        # 缓存原始传感器状态位掩码
        self._sensor_health_cache[usv_id] = {
            'onboard_control_sensors_present': msg.onboard_control_sensors_present,
            'onboard_control_sensors_enabled': msg.onboard_control_sensors_enabled,
            'onboard_control_sensors_health': msg.onboard_control_sensors_health,
            'timestamp': self._now_seconds()
        }
        
        # 记录日志以便调试（仅首次或状态变化时）
        if not hasattr(self, '_last_sensor_health_log'):
            self._last_sensor_health_log = {}
        
        prev = self._last_sensor_health_log.get(usv_id)
        curr_health = msg.onboard_control_sensors_health
        if prev != curr_health:
            self.get_logger().info(
                f"[SYS_STATUS] {usv_id} 传感器健康更新: "
                f"present=0x{msg.onboard_control_sensors_present:08X}, "
                f"enabled=0x{msg.onboard_control_sensors_enabled:08X}, "
                f"health=0x{curr_health:08X}"
            )
            self._last_sensor_health_log[usv_id] = curr_health

    def push_state_updates(self):
        """
        定期主动推送状态更新到 GUI（5Hz 优化频率）
        
        只在数据有变化时才重新计算，避免不必要的开销。
        """
        if not self.usv_states:
            return
        
        try:
            now_sec = self._now_seconds()
            updated = False
            
            # 只更新有变化的 USV
            for usv_id in list(self.usv_states.keys()):
                # 检查是否需要更新（有新消息、PreArm 警告变化、传感器状态变化）
                if self._should_update_augmented_state(usv_id, now_sec):
                    self.augment_state_payload(usv_id)
                    updated = True
            
            # 只在有更新时推送
            if updated:
                self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))
        except Exception as exc:
            # 使用 debug 级别避免刷屏，因为这是高频调用
            pass  # 静默失败，避免日志刷屏
    
    def augment_state_payload(self, usv_id, state_data=None):
        """
        为状态字典附加车辆消息、预检标记与传感器状态
        
        实现 QGC 风格的综合 Ready 检查:
        1. PreArm 警告检查 (来自 STATUSTEXT)
        2. CRITICAL/ERROR 消息检查 (来自 STATUSTEXT)
        3. 传感器健康检查 (来自 SYS_STATUS)
        4. 系统状态检查 (来自 HEARTBEAT, 暂未实现)
        """
        if state_data is None:
            state_data = self.usv_states.get(usv_id)
            if state_data is None:
                return None

        now_sec = self._now_seconds()
        self._cleanup_prearm_warnings(usv_id, now_sec)

        # 1. 收集所有消息
        messages = [dict(item) for item in self._vehicle_messages.get(usv_id, [])]
        
        # 2. 收集 PreArm 警告
        prearm_warnings = list(self._prearm_warnings.get(usv_id, {}).keys())
        
        # 3. 收集最近的 CRITICAL/ERROR 消息 (30秒内)
        critical_errors = []
        for msg_entry in self._vehicle_messages.get(usv_id, []):
            severity = msg_entry.get('severity', 6)
            timestamp = msg_entry.get('timestamp', 0)
            # severity <= 3 表示 EMERGENCY/ALERT/CRITICAL/ERROR
            if severity <= 3 and (now_sec - timestamp) <= 30.0:
                critical_errors.append(msg_entry.get('text', ''))
        
        # 4. 检查传感器健康状态
        sensor_healthy, unhealthy_sensors = self._check_sensor_health(usv_id)
        
        # 5. 综合判断 Ready 状态
        # 必须同时满足: 无 PreArm 警告 + 无严重错误 + 传感器健康
        all_warnings = prearm_warnings.copy()
        
        # 将严重错误添加到警告列表
        if critical_errors:
            for err in critical_errors[:3]:  # 最多显示 3 条严重错误
                all_warnings.append(f"[CRITICAL] {err}")
        
        # 将传感器问题添加到警告列表
        if not sensor_healthy:
            for sensor in unhealthy_sensors:
                all_warnings.append(f"[传感器] {sensor} 异常")
        
        # Ready 状态: 所有检查都通过
        ready = (len(prearm_warnings) == 0 and 
                 len(critical_errors) == 0 and 
                 sensor_healthy)
        
        # 缓存结果
        self._prearm_ready[usv_id] = ready
        self._sensor_status_cache[usv_id] = self._build_sensor_status(usv_id, state_data)

        # 更新状态数据
        state_data['vehicle_messages'] = messages
        state_data['prearm_ready'] = ready
        state_data['prearm_warnings'] = all_warnings  # 包含所有警告来源
        state_data['sensor_status'] = self._sensor_status_cache[usv_id]
        # 附加导航目标缓存（用于导航面板显示）
        state_data['nav_target_cache'] = self._usv_nav_target_cache.get(usv_id)

        return state_data

    def _build_sensor_status(self, usv_id, state):
        """根据当前状态评估关键传感器的健康状况."""
        statuses = []

        fix_type = state.get('gps_fix_type')
        sat = state.get('gps_satellites_visible')
        eph = state.get('gps_eph')

        gps_label = self._describe_gps_fix(fix_type)
        detail_parts = []
        try:
            sat_int = int(sat) if sat is not None else None
        except (TypeError, ValueError):
            sat_int = None
        if sat_int is not None:
            detail_parts.append(f"{sat_int} sats")
        try:
            eph_val = float(eph) if eph is not None else None
        except (TypeError, ValueError):
            eph_val = None
        if eph_val is not None and eph_val > 0:
            detail_parts.append(f"HDOP {eph_val:.1f}")

        if fix_type is None:
            gps_level = 'warn'
        else:
            try:
                fix_int = int(fix_type)
            except (TypeError, ValueError):
                fix_int = -1
            
            # 综合判断：fix_type + 卫星数 + HDOP
            # 优先级：卫星数 > HDOP > fix_type
            if sat_int is not None and sat_int < 4:
                # 卫星数少于4颗，无法可靠定位 → 错误
                gps_level = 'error'
            elif eph_val is not None and eph_val > 10.0:
                # HDOP > 10（精度极差）→ 错误
                gps_level = 'error'
            elif fix_int <= 1:
                # No GPS 或 No Fix → 错误
                gps_level = 'error'
            elif fix_int == 2 or (eph_val is not None and eph_val > 5.0):
                # 2D Fix 或 HDOP > 5（精度较差）→ 警告
                gps_level = 'warn'
            else:
                # 3D Fix 及以上，且卫星数≥4，且 HDOP ≤ 5 → 正常
                gps_level = 'ok'

        statuses.append({
            'name': 'GPS Fix',
            'status': gps_label,
            'detail': ', '.join(detail_parts),
            'level': gps_level,
        })

        if sat_int is not None:
            if sat_int >= 4:
                sat_level = 'ok'      # 4颗及以上可定位，显示绿色
            else:
                sat_level = 'error'   # 少于4颗无法定位，显示红色
            statuses.append({
                'name': 'Satellites',
                'status': f'{sat_int}',
                'detail': '',
                'level': sat_level,
            })

        battery_pct = state.get('battery_percentage')
        battery_voltage = state.get('battery_voltage')
        try:
            battery_pct_val = float(battery_pct) if battery_pct is not None else None
        except (TypeError, ValueError):
            battery_pct_val = None

        if battery_pct_val is not None:
            if battery_pct_val >= 30.0:
                battery_level = 'ok'
            elif battery_pct_val >= 15.0:
                battery_level = 'warn'
            else:
                battery_level = 'error'
            detail = f"{battery_pct_val:.0f}%"
            if battery_voltage is not None:
                try:
                    detail += f" @ {float(battery_voltage):.1f}V"
                except (TypeError, ValueError):
                    pass
            statuses.append({
                'name': 'Battery',
                'status': 'OK' if battery_level == 'ok' else 'Low',
                'detail': detail,
                'level': battery_level,
            })

        # 温度检查（从毫摄氏度转换为摄氏度）
        temperature = state.get('temperature')
        try:
            temp_raw = float(temperature) if temperature is not None else None
            temp_val = temp_raw / 1000.0 if temp_raw is not None else None  # 毫度 → 度
        except (TypeError, ValueError):
            temp_val = None
        if temp_val is not None and temp_val > 0:
            if temp_val >= 75.0:
                temp_level = 'error'
            elif temp_val >= 60.0:
                temp_level = 'warn'
            else:
                temp_level = 'ok'
            statuses.append({
                'name': 'CPU Temp',
                'status': f"{temp_val:.1f}°C",
                'detail': '',
                'level': temp_level,
            })

        return statuses

    def _check_sensor_health(self, usv_id):
        """
        检查关键传感器是否健康 (基于 SYS_STATUS 位掩码)
        
        根据 QGC 实现方式，检查所有已启用且需要的传感器是否健康。
        MAV_SYS_STATUS_SENSOR 位定义:
            0x01: 3D gyro
            0x02: 3D accelerometer
            0x04: 3D magnetometer (可选，ArduPilot可在无磁罗盘时飞行)
            0x08: absolute pressure (气压计)
            0x20: GPS
        
        Returns:
            (bool, list): (是否健康, 不健康传感器列表)
        """
        sensor_data = self._sensor_health_cache.get(usv_id)
        if not sensor_data:
            # 如果还没有收到 SYS_STATUS 消息，暂时认为传感器健康
            # 这样 Ready 检查只依赖于 PreArm 警告和严重错误
            # 注意：这是临时方案，理想情况下应该确保收到 SYS_STATUS 消息
            return True, []
        
        present = sensor_data['onboard_control_sensors_present']
        enabled = sensor_data['onboard_control_sensors_enabled']
        health = sensor_data['onboard_control_sensors_health']
        
        # 定义关键传感器位掩码
        SENSOR_GYRO = 0x01
        SENSOR_ACCEL = 0x02
        SENSOR_MAG = 0x04
        SENSOR_BARO = 0x08
        SENSOR_GPS = 0x20
        
        # 定义必需传感器（陀螺仪、加速度计、气压计必需，磁罗盘可选）
        # GPS 根据飞行模式可能是必需的，但在 PreArm 阶段检查
        required_sensors = SENSOR_GYRO | SENSOR_ACCEL | SENSOR_BARO
        
        unhealthy_sensors = []
        sensor_names = {
            SENSOR_GYRO: "陀螺仪",
            SENSOR_ACCEL: "加速度计",
            SENSOR_MAG: "磁罗盘",
            SENSOR_BARO: "气压计",
            SENSOR_GPS: "GPS"
        }
        
        # 检查每个传感器
        for bit, name in sensor_names.items():
            # 如果传感器存在且已启用
            if (present & bit) and (enabled & bit):
                # 检查是否健康
                if not (health & bit):
                    unhealthy_sensors.append(name)
        
        # 如果有不健康的传感器，返回 False
        if unhealthy_sensors:
            return False, unhealthy_sensors
        
        # 检查必需传感器是否都已启用
        required_enabled = (enabled & required_sensors)
        if required_enabled != (present & required_sensors):
            missing = []
            for bit, name in sensor_names.items():
                if (bit & required_sensors) and (present & bit) and not (enabled & bit):
                    missing.append(f"{name}(未启用)")
            if missing:
                return False, missing
        
        return True, []

    def _should_update_augmented_state(self, usv_id, now_sec):
        """
        检查是否需要重新计算 augmented state
        避免无变化时的重复计算
        """
        # 检查是否有新的 statustext 消息
        last_msg_time = getattr(self, '_last_statustext_time', {}).get(usv_id, 0)
        if now_sec - last_msg_time < 0.3:  # 300ms 内有新消息
            return True
        
        # 检查是否有 PreArm 警告即将过期
        warnings = self._prearm_warnings.get(usv_id, {})
        if warnings:
            for ts in warnings.values():
                if now_sec - ts > self.PREARM_WARNING_EXPIRY - 1.0:  # 即将过期
                    return True
        
        # 检查是否有传感器状态更新
        sensor_cache = self._sensor_health_cache.get(usv_id)
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
    
    def _cleanup_prearm_warnings(self, usv_id, now_sec):
        warnings = self._prearm_warnings.get(usv_id)
        if not warnings:
            return
        for key, ts in list(warnings.items()):
            if now_sec - ts > self.PREARM_WARNING_EXPIRY:
                warnings.pop(key, None)

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
        mapping = {
            0: 'EMERGENCY',
            1: 'ALERT',
            2: 'CRITICAL',
            3: 'ERROR',
            4: 'WARNING',
            5: 'NOTICE',
            6: 'INFO',
            7: 'DEBUG',
        }
        return mapping.get(severity, f'LEVEL {severity}')

    @staticmethod
    def _describe_gps_fix(fix_type):
        mapping = {
            0: 'No GPS',
            1: 'No Fix',
            2: '2D Fix',
            3: '3D Fix',
            4: 'DGPS',
            5: 'RTK Float',
            6: 'RTK Fixed',
        }
        try:
            fix_int = int(fix_type)
        except (TypeError, ValueError):
            fix_int = None
        if fix_int is None:
            return 'Unknown'
        return mapping.get(fix_int, 'Unknown')

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