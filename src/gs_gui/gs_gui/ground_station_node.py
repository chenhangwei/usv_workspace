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
from rcl_interfaces.msg import Log  # 导入 ROS 日志消息类型

# 导入分解后的模块
from .usv_manager import UsvManager
from .cluster_controller import ClusterController
from .command_processor import CommandProcessor
from .led_infection import LedInfectionHandler
from .event_decoder import EventDecoder  # 导入事件解码器
from .px4_command_interface import Px4CommandInterface  # PX4 命令接口
from .navigation_handler import NavigationHandler  # 导航处理器
from .sensor_status_handler import SensorStatusHandler  # 传感器状态处理器
from .discovery_handler import DiscoveryHandler  # USV 发现处理器

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
        self.declare_parameter('area_center_x', 0.0)
        self.declare_parameter('area_center_y', 0.0)
        self.declare_parameter('area_center_z', 0.0)
        self.declare_parameter('area_center_frame', 'map')

        # 初始化子模块
        self.usv_manager = UsvManager(self)
        self.cluster_controller = ClusterController(self)
        self.command_processor = CommandProcessor(self)
        self.led_infection_handler = LedInfectionHandler(self)
        
        # 初始化模块化处理器
        self.navigation_handler = NavigationHandler(self, self.usv_manager, signal)
        self.sensor_handler = SensorStatusHandler(self, signal)
        self.discovery_handler = DiscoveryHandler(self, self.usv_manager, signal)
        
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
        
        通过 PX4 VehicleCommand 发送 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 命令重启飞控
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        try:
            # 使用 PX4 命令接口发送重启命令
            px4_cmd = Px4CommandInterface(self, usv_namespace)
            
            success = px4_cmd.reboot_autopilot()
            
            if success:
                self.get_logger().info(f'[OK] 已向 {usv_namespace} 发送飞控重启命令 (VehicleCommand)')
                try:
                    self.ros_signal.node_info.emit(f'[OK] 已向 {usv_namespace} 发送飞控重启命令，请等待 10-20 秒')
                except Exception:
                    pass
            else:
                self.get_logger().error(f'[X] {usv_namespace} 飞控重启命令发送失败')
                try:
                    self.ros_signal.node_info.emit(f'[X] {usv_namespace} 飞控重启命令发送失败')
                except Exception:
                    pass
            
        except Exception as e:
            self.get_logger().error(f'[X] 发送重启命令失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] 发送重启命令失败: {e}')
            except Exception:
                pass
    
    def _handle_reboot_response(self, future, usv_namespace):
        """处理重启命令响应（PX4 模式下不使用，保留兼容性）"""
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
        通过 PX4 VehicleCommand 重启机载计算机（备选方案）
        
        注意：某些飞控固件可能不支持此命令
        """
        try:
            # 使用 PX4 命令接口发送重启命令
            px4_cmd = Px4CommandInterface(self, usv_namespace)
            
            success = px4_cmd.reboot_companion()
            
            if success:
                self.get_logger().info(f'[OK] 已发送机载计算机重启命令到 {usv_namespace}')
                try:
                    self.ros_signal.node_info.emit(f'[OK] {usv_namespace} 机载计算机重启命令已发送，系统将在 30-60 秒后重新上线')
                except Exception:
                    pass
            else:
                self.get_logger().warn(f'[!] {usv_namespace} 机载计算机重启命令发送失败')
            
        except Exception as e:
            self.get_logger().error(f'[X] 机载计算机重启命令失败: {e}')
    
    def _handle_companion_reboot_response(self, future, usv_namespace):
        """处理机载计算机重启命令响应（PX4 模式下不使用，保留兼容性）"""
        pass

    def set_home_position_callback(self, usv_namespace, use_current, coords):
        """
        设置 Home Position 回调
        
        通过 PX4 HomePosition 消息设置 Home Position（局部坐标系）
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
            use_current: 是否使用当前位置（True=使用当前位置, False=使用指定坐标）
            coords: 坐标字典 {'x': float, 'y': float, 'z': float}（仅当 use_current=False 时使用）
        """
        try:
            # 使用 PX4 命令接口发送设置 Home 命令
            px4_cmd = Px4CommandInterface(self, usv_namespace)
            
            if use_current:
                success = px4_cmd.set_home_position(use_current=True)
                self.get_logger().info(f'[OK] 设置 {usv_namespace} Home Position 为当前位置')
            else:
                x = float(coords.get('x', 0.0))
                y = float(coords.get('y', 0.0))
                z = float(coords.get('z', 0.0))
                success = px4_cmd.set_home_position(use_current=False, x=x, y=y, z=z)
                self.get_logger().info(
                    f'[OK] 设置 {usv_namespace} Home Position 为局部坐标: '
                    f'X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m'
                )
            
            if success:
                try:
                    if use_current:
                        self.ros_signal.node_info.emit(f'[OK] 已向 {usv_namespace} 发送设置 Home Position 命令（使用当前位置）')
                    else:
                        self.ros_signal.node_info.emit(
                            f'[OK] 已向 {usv_namespace} 发送设置 Home Position 命令\n'
                            f'    局部坐标: X={coords.get("x"):.2f}m, Y={coords.get("y"):.2f}m, Z={coords.get("z"):.2f}m'
                        )
                except Exception:
                    pass
            else:
                self.get_logger().error(f'[X] {usv_namespace} 设置 Home Position 命令发送失败')
                try:
                    self.ros_signal.node_info.emit(f'[X] {usv_namespace} 设置 Home Position 命令发送失败')
                except Exception:
                    pass
            
        except Exception as e:
            self.get_logger().error(f'[X] 发送设置 Home Position 命令失败: {e}')
            try:
                self.ros_signal.node_info.emit(f'[X] 发送设置 Home Position 命令失败: {e}')
            except Exception:
                pass
    
    def _handle_set_home_response(self, future, usv_namespace, use_current, coords):
        """处理设置 Home Position 命令响应（PX4 模式下不使用，保留兼容性）"""
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
        处理 ROS 日志消息，用于捕获 PX4 的 FCU: EVENT 消息
        """
        # 过滤出 USV 相关的日志
        # PX4 的日志节点名可能是:
        # - "usv_01.sys" (PX4 sys_status)
        # - "usv_01.fmu" (PX4 fmu)
        # - 包含 "usv_" 的其他名称
        if 'usv_' not in msg.name:
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