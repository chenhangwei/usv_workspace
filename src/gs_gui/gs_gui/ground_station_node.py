"""
地面站GUI ROS2节点实现文件
该文件实现了地面站GUI与USV系统通信的核心功能
包括状态监控、命令发送、导航控制等功能
"""

import rclpy 
import rclpy.action
from rclpy.node import Node  # 从rclpy.node模块导入Node类，用于创建ROS2节点
from rclpy.parameter import Parameter  # 导入Parameter类，用于参数设置
from geometry_msgs.msg import PoseStamped  # 从geometry_msgs.msg模块导入PoseStamped消息类型，用于位姿信息
from common_interfaces.msg import UsvStatus  # 从common_interfaces.msg模块导入UsvStatus消息类型
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # 从rclpy.qos模块导入QoSProfile和QoSReliabilityPolicy，用于设置服务质量
from rclpy.action import ActionClient  # 从rclpy.action模块导入ActionClient，用于创建动作客户端
from common_interfaces.action import NavigateToPoint  # 从common_interfaces.action模块导入NavigateToPoint动作类型
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


class GroundStationNode(Node):
    """
    地面站节点类
    继承自rclpy.Node，实现地面站GUI与USV系统通信的核心功能
    """

    # 常量定义
    INFECTION_DISTANCE_SQUARED = 4.0  # 2米距离的平方
    DEFAULT_STEP_TIMEOUT = 20.0  # 默认步骤超时时间(秒)
    DEFAULT_MAX_RETRIES = 1      # 默认最大重试次数
    INFECTION_CHECK_PERIOD = 2.0 # 传染检查周期(秒)，增加周期减少CPU占用
    NAMESPACE_UPDATE_PERIOD = 2.0 # 命名空间更新周期(秒)，从 5.0 减少到 2.0，加快离线检测
    CLUSTER_TARGET_PUBLISH_PERIOD = 5 # 集群目标发布周期(秒)，增加周期减少CPU占用
    MIN_ACK_RATE_FOR_PROCEED = 0.8  # 最小确认率阈值，超过此值可进入下一步
    
    def __init__(self, signal):
        """
        初始化地面站节点
        
        Args:
            signal: ROS信号对象，用于与GUI界面通信
        """
        super().__init__('groundstationnode')  # 调用父类Node的初始化方法，设置节点名称为'groundstationnode'
        self.ros_signal = signal  # 保存ROS信号对象引用
        self.qos_a = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)  # 创建QoS配置对象，深度为10，可靠性策略为可靠传输

        # 初始化子模块
        self.usv_manager = UsvManager(self)
        self.cluster_controller = ClusterController(self)
        self.command_processor = CommandProcessor(self)
        self.led_infection_handler = LedInfectionHandler(self)

        # 初始化USV状态和目标管理相关变量
        self.usv_states = {}  # USV状态字典
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

        # 初始化集群控制相关变量
        self._usv_ack_map = {}  # USV确认映射表
        self._cluster_start_time = None  # 集群开始时间
        # 将超时/重试等参数化，支持通过参数/launch调整
        self.declare_parameter('step_timeout', float(self.DEFAULT_STEP_TIMEOUT))
        self.declare_parameter('max_retries', int(self.DEFAULT_MAX_RETRIES))
        self.declare_parameter('min_ack_rate_for_proceed', float(self.MIN_ACK_RATE_FOR_PROCEED))
        self.declare_parameter('offline_grace_period', 5.0)  # 从 20.0 减少到 5.0 秒，加快移除速度
        # area center 参数（任务坐标系原点在全局 map 中的位置）
        self.declare_parameter('area_center_x', 0.0)
        self.declare_parameter('area_center_y', 0.0)
        self.declare_parameter('area_center_z', 0.0)
        self.declare_parameter('area_center_frame', 'map')
        # 从参数服务器读取当前值（可被 launch/参数文件覆盖）
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

        self._cluster_task_paused = False  # 集群任务是否已暂停
        # 用于通知后台线程退出的事件
        self._stop_event = threading.Event()

        #Action 任务跟踪
        self._usv_active_goals = {} # 跟踪每个 USV 当前活动的 Action 句柄

        # 初始化传染机制相关变量
        self._usv_led_modes = {}  # USV LED模式字典
        self._usv_infecting = set()  # 正在传染的USV集合
        # 维护本地 LED 状态 
        self._usv_current_led_state = {} # 维护 USV ID -> {'mode': str, 'color': [r,g,b]} 
        # LED传染模式开关（默认开启）
        self._led_infection_enabled = True
     
        # 初始化命名空间检测历史记录
        self._ns_detection_history = []  # 用于存储命名空间检测历史记录的列表
        # 记录每个 USV 最后一次收到状态消息的时间戳（秒）
        self._ns_last_seen = {}

        # 创建定时器
        self.ns_timer = self.create_timer(self.NAMESPACE_UPDATE_PERIOD, self.update_subscribers_and_publishers)  # 命名空间更新定时器，定期更新订阅者和发布者
        self.target_timer = self.create_timer(self.CLUSTER_TARGET_PUBLISH_PERIOD, self.publish_cluster_targets_callback)  # 集群目标发布定时器，定期发布集群目标
        self.infect_check_timer = self.create_timer(self.INFECTION_CHECK_PERIOD, self.check_usv_infect)  # 传染检查定时器，定期检查USV之间的传染逻辑
        
        self.update_subscribers_and_publishers()

        # TF2: Buffer/Listener for coordinate transforms
        try:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        except Exception:
            self.tf_buffer = None
            self.tf_listener = None
            self.static_broadcaster = None

        # 用于接收来自 GUI 的字符串命令的队列（由 GUI 线程快速入队，节点线程定期处理）
        self._incoming_str_commands = queue.Queue(maxsize=200)
        # 在节点线程中周期性处理入队的字符串命令，避免在 GUI 线程执行节点逻辑
        self._str_command_timer = self.create_timer(0.1, self._process_incoming_str_commands)

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

    # 获取当前节点的名称和命名空间，为新的 USV 节点创建订阅和发布器
    def update_subscribers_and_publishers(self):
        """
        更新订阅者和发布者列表，处理USV的连接和断开
        
        该方法定期检查系统中的节点命名空间，
        为新连接的USV创建订阅者和发布者，
        为断开的USV销毁订阅者和发布者
        """
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
                self.get_logger().info(f"✅ USV上线: {ns} (检测到 {node_count} 个节点)")

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
    def send_nav_goal_via_action(self, usv_id, x, y, z=0.0, yaw=0.0, timeout=300.0):
        """
        通过Action方式向指定USV发送导航目标点
        
        Args:
            usv_id (str): USV标识符
            x (float): 目标点X坐标
            y (float): 目标点Y坐标
            z (float): 目标点Z坐标，默认为0.0
            yaw (float): 目标偏航角(弧度)
            timeout (float): 超时时间(秒)
            
        Returns:
            bool: 发送是否成功
        """
        # 检查USV是否存在
        if usv_id not in self.usv_manager.navigate_to_point_clients:
            self.get_logger().error(f"未找到USV {usv_id} 的导航客户端")
            # 更新导航状态为失败
            self.ros_signal.nav_status_update.emit(usv_id, "失败")
            return False

        # 检查Action服务器是否可用
        action_client = self.usv_manager.navigate_to_point_clients[usv_id]
        if not action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"USV {usv_id} 的导航Action服务器未响应")
            # 更新导航状态为失败
            self.ros_signal.nav_status_update.emit(usv_id, "失败")
            return False

        # 构造目标消息
        goal_msg = NavigateToPoint.Goal()
        goal_msg.goal = PoseStamped()
        # 设置时间戳为当前时间
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        # 设置坐标系为map
        goal_msg.goal.header.frame_id = 'map'
        # 设置目标点位置
        goal_msg.goal.pose.position.x = float(x)
        goal_msg.goal.pose.position.y = float(y)
        goal_msg.goal.pose.position.z = float(z)  # 添加z坐标

        # 使用四元数表示偏航角
        from tf_transformations import quaternion_from_euler
        # 计算四元数
        quat = quaternion_from_euler(0, 0, float(yaw))
        # 设置四元数
        goal_msg.goal.pose.orientation.x = quat[0]
        goal_msg.goal.pose.orientation.y = quat[1]
        goal_msg.goal.pose.orientation.z = quat[2]
        goal_msg.goal.pose.orientation.w = quat[3]

        # 设置超时时间
        goal_msg.timeout = float(timeout)

        # 更新导航状态为执行中
        self.ros_signal.nav_status_update.emit(usv_id, "执行中")

        # 取消旧任务
        self._cancel_active_goal(usv_id) # 取消旧任务

        # 异步发送目标
        send_goal_future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, uid=usv_id: self.nav_feedback_callback(feedback_msg, uid))

        # 添加结果回调
        send_goal_future.add_done_callback(
            lambda future, uid=usv_id: self.nav_goal_response_callback(future, uid))

        # 记录日志信息
        self.get_logger().info(f"向USV {usv_id} 发送导航目标点: ({x}, {y}, {z}), 偏航: {yaw}, 超时: {timeout}")
        return True

    # 导航目标响应回调
    def nav_goal_response_callback(self, future, usv_id):
        """
        导航目标响应回调
        
        Args:
            future: 异步操作的future对象
            usv_id (str): USV标识符
        """
        try:
            # 获取目标句柄
            goal_handle = future.result()
            # 检查目标是否被接受
            if not goal_handle.accepted:
                self.get_logger().warn(f"USV {usv_id} 拒绝了导航目标")
                # 更新导航状态为失败
                self.ros_signal.nav_status_update.emit(usv_id, "失败")
                return

            self.get_logger().info(f"USV {usv_id} 接受了导航目标")

            # 存储活动的 Action 句柄 ---
            self._usv_active_goals[usv_id] = goal_handle 
           






            # 获取结果
            get_result_future = goal_handle.get_result_async()
            # 添加结果回调
            get_result_future.add_done_callback(
                lambda result_future, uid=usv_id: self.nav_get_result_callback(result_future, uid))

        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"处理USV {usv_id} 导航目标响应时出错: {e}")
            # 更新导航状态为失败
            self.ros_signal.nav_status_update.emit(usv_id, "失败")

    # 导航结果回调
    def nav_get_result_callback(self, future, usv_id):
        """
        导航结果回调
        
        Args:
            future: 异步操作的future对象
            usv_id (str): USV标识符
        """
        try:
            # 获取结果
            result = future.result().result
            # 记录日志信息
            self.get_logger().info(
                f"USV {usv_id} 导航任务完成 - 成功: {result.success}, "
                f"错误码: {result.error_code}, 消息: {result.message}")

            # 根据结果更新导航状态
            if result.success:
                self.ros_signal.nav_status_update.emit(usv_id, "成功")
                # 标记为已确认
                if usv_id in self._usv_ack_map:
                    self._usv_ack_map[usv_id]['acked'] = True
                    self._usv_ack_map[usv_id]['ack_time'] = self.get_clock().now().nanoseconds / 1e9
            else:
                self.ros_signal.nav_status_update.emit(usv_id, "失败")

            # 可以在这里发射信号通知GUI更新状态
            # self.ros_signal.navigation_completed.emit(usv_id, result.success, result.error_code)

            # 任务完成后清除句柄 ---
            if usv_id in self._usv_active_goals:
                 del self._usv_active_goals[usv_id]



        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"处理USV {usv_id} 导航结果时出错: {e}")
            # 更新导航状态为失败
            self.ros_signal.nav_status_update.emit(usv_id, "失败")

    def _cancel_active_goal(self, usv_id):
        """
        取消指定 USV 当前活动的 Action 任务
        """
        if usv_id in self._usv_active_goals:
            goal_handle = self._usv_active_goals[usv_id]
            # 检查句柄是否仍然有效
            if goal_handle.status in [rclpy.action.client.GoalStatus.STATUS_EXECUTING, rclpy.action.client.GoalStatus.STATUS_ACCEPTED]:
                self.get_logger().warn(f"正在取消 USV {usv_id} 的上一个导航任务...")
                cancel_future = goal_handle.cancel_goal_async()
                
                # 可选：等待取消结果以确保取消请求已发送，但这里不做阻塞处理
                # cancel_future.add_done_callback(...)
                
            # 无论是否取消成功，都从跟踪字典中删除
            del self._usv_active_goals[usv_id]

    # 导航反馈回调
    def nav_feedback_callback(self, feedback_msg, usv_id):
        """
        导航反馈回调
        
        Args:
            feedback_msg: 反馈消息
            usv_id (str): USV标识符
        """
        try:
            # 获取反馈数据
            feedback = feedback_msg.feedback
            # 记录日志信息
            self.get_logger().info(
                f"USV {usv_id} 导航反馈 - 距离目标: {feedback.distance_to_goal:.2f}m, "
                f"航向误差: {feedback.heading_error:.2f}度, "
                f"预计剩余时间: {feedback.estimated_time:.2f}秒")

            # 发射信号通知GUI更新进度条等
            self.ros_signal.navigation_feedback.emit(usv_id, feedback)

        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"处理USV {usv_id} 导航反馈时出错: {e}")

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
                # 支持z坐标
                self.send_nav_goal_via_action(usv_id, p_local.get('x', 0.0), p_local.get('y', 0.0), p_local.get('z', 0.0), yaw, 300.0)
                # 记录日志信息（记录全局和本地坐标以便调试）
                self.get_logger().info(f"已下发目标点到 {usv_id}: global={p_global}, local={p_local}, yaw: {yaw}")
        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"处理离群目标点失败: {e}")

    # 委托给子模块的方法
    def set_manual_callback(self, msg):
        self.command_processor.set_manual_callback(msg)

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

    def _update_local_led_state(self, usv_id, command_str):
        """
        根据发送的 LED 命令更新本地维护的状态
        """
        if not command_str.data:
            return

        cmd_parts = command_str.data.split('|')
        mode = cmd_parts[0].lower()
        
        state = self._usv_current_led_state.get(usv_id, {'mode': 'color_switching', 'color': [255, 0, 0]})
        
        if mode == 'color_select' and len(cmd_parts) > 1:
            try:
                color_parts = [int(c.strip()) for c in cmd_parts[1].split(',')]
                if len(color_parts) == 3:
                    state['mode'] = 'color_select'
                    state['color'] = color_parts
            except ValueError:
                self.get_logger().warn(f"解析 {usv_id} 的颜色命令失败: {command_str.data}")
        elif mode != 'color_infect': # 传染模式不改变基础模式和颜色
             state['mode'] = mode
             # 可以根据模式设置默认颜色，这里保持不变
        
        self._usv_current_led_state[usv_id] = state

    def str_command_callback(self, msg):
        """
        字符串命令回调函数
        
        Args:
            msg: 字符串命令消息
        """
        self.command_processor.str_command_callback(msg)
    
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