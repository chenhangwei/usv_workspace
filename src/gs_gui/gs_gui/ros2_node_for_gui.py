"""
地面站GUI ROS2节点实现文件
该文件实现了地面站GUI与USV系统通信的核心功能
包括状态监控、命令发送、导航控制等功能
"""

import rclpy 
import rclpy.action
from rclpy.node import Node  # 从rclpy.node模块导入Node类，用于创建ROS2节点
from geometry_msgs.msg import PoseStamped  # 从geometry_msgs.msg模块导入PoseStamped消息类型，用于位姿信息
from common_interfaces.msg import UsvStatus  # 从common_interfaces.msg模块导入UsvStatus消息类型
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # 从rclpy.qos模块导入QoSProfile和QoSReliabilityPolicy，用于设置服务质量
from rclpy.action import ActionClient  # 从rclpy.action模块导入ActionClient，用于创建动作客户端
from common_interfaces.action import NavigateToPoint  # 从common_interfaces.action模块导入NavigateToPoint动作类型
import queue  # 导入queue模块，用于创建消息队列
import threading  # 导入threading模块，用于多线程处理
from std_msgs.msg import String # 导入 String 消息类型
import weakref  # 导入weakref模块，用于弱引用



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
    NAMESPACE_UPDATE_PERIOD = 5.0 # 命名空间更新周期(秒)，增加周期减少CPU占用
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

        # 初始化ROS发布者和订阅者字典
        self.usv_state_subs = {}  # USV状态订阅者字典
        self.set_usv_target_position_pubs = {}  # 设置USV目标位置发布者字典
        self.set_usv_target_velocity_pubs = {}  # 设置USV目标速度发布者字典
        self.set_usv_mode_pubs = {}  # 设置USV模式发布者字典
        self.set_usv_arming_pubs = {}  # 设置USV武装状态发布者字典
        self.led_pubs = {}  # LED控制发布者字典
        self.sound_pubs = {}  # 声音控制发布者字典
        self.action_pubs = {}  # 动作控制发布者字典
        self.navigate_to_point_clients = {}  # 添加导航动作客户端字典

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
        self._step_timeout = self.DEFAULT_STEP_TIMEOUT  # 步骤超时时间
        self._max_retries = self.DEFAULT_MAX_RETRIES  # 最大重试次数
        self._cluster_task_paused = False  # 集群任务是否已暂停

        #Action 任务跟踪
        self._usv_active_goals = {} # 跟踪每个 USV 当前活动的 Action 句柄

        # 初始化传染机制相关变量
        self._usv_led_modes = {}  # USV LED模式字典
        self._usv_infecting = set()  # 正在传染的USV集合
        # 维护本地 LED 状态 
        self._usv_current_led_state = {} # 维护 USV ID -> {'mode': str, 'color': [r,g,b]} 
     

        # 创建定时器
        self.ns_timer = self.create_timer(self.NAMESPACE_UPDATE_PERIOD, self.update_subscribers_and_publishers)  # 命名空间更新定时器，定期更新订阅者和发布者
        self.target_timer = self.create_timer(self.CLUSTER_TARGET_PUBLISH_PERIOD, self.publish_cluster_targets_callback)  # 集群目标发布定时器，定期发布集群目标
        self.infect_check_timer = self.create_timer(self.INFECTION_CHECK_PERIOD, self.check_usv_infect)  # 传染检查定时器，定期检查USV之间的传染逻辑
        
        self.update_subscribers_and_publishers()

    # 在独立线程中异步处理消息发布队列
    def process_publish_queue(self):
        """
        在独立线程中处理消息发布队列，避免阻塞主ROS循环
        
        该方法在单独的线程中运行，从发布队列中取出消息并发布
        这样可以避免在主ROS循环中进行耗时的发布操作
        """
        # 当ROS仍在运行时持续处理队列
        while rclpy.ok():
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

    # 获取当前节点的名称和命名空间，为新的 USV 节点创建订阅和发布器
    def update_subscribers_and_publishers(self):
        """
        更新订阅者和发布者列表，处理USV的连接和断开
        
        该方法定期检查系统中的节点命名空间，
        为新连接的USV创建订阅者和发布者，
        为断开的USV销毁订阅者和发布者
        """
        # 获取当前系统中的节点名称和命名空间
        namespaces = self.get_node_names_and_namespaces()
        # 筛选出以'/usv_'开头的命名空间
        current_ns_list = list(set([ns for _, ns in namespaces if ns.startswith('/usv_')]))
        # 如果命名空间列表没有变化，直接返回
        if current_ns_list == self.last_ns_list:
            return

        # 更新上次命名空间列表
        self.last_ns_list = current_ns_list
        # 获取已存在的命名空间集合
        existing_ns = set(self.usv_state_subs.keys())
        # 计算新增的命名空间集合
        new_ns = set(current_ns_list) - existing_ns
        # 计算移除的命名空间集合
        removed_ns = existing_ns - set(current_ns_list)

        # 处理新增的USV命名空间
        for ns in new_ns:
            self._add_usv_namespace(ns)

        # 处理移除的USV命名空间
        for ns in removed_ns:
            self._remove_usv_namespace(ns)

    # 添加一个新的USV命名空间
    def _add_usv_namespace(self, ns):
        """
        为指定的USV命名空间添加订阅者和发布者
        
        Args:
            ns (str): USV的命名空间
        """
        # 从命名空间中提取USV ID（去掉开头的'/')
        usv_id = ns.lstrip('/')
        # 构造各种主题名称
        topic_state = f"{ns}/usv_state"  # USV状态主题
        topic_mode = f"{ns}/set_usv_mode"  # 设置USV模式主题
        topic_arming = f"{ns}/set_usv_arming"  # 设置USV武装状态主题
        topic_led = f"{ns}/gs_led_command"  # LED控制主题
        topic_sound = f"{ns}/gs_sound_command"  # 声音控制主题
        topic_action = f"{ns}/gs_action_command"  # 动作控制主题
        action_server_name = f"{ns}/navigate_to_point"  # 导航动作服务器名称

        # 为USV创建各种订阅者和发布者
        # 创建USV状态订阅者
        self.usv_state_subs[usv_id] = self.create_subscription(
            UsvStatus, topic_state, lambda msg, id=usv_id: self.usv_state_callback(msg, id), self.qos_a)
        # 创建设置USV模式发布者
        self.set_usv_mode_pubs[usv_id] = self.create_publisher(
            String, topic_mode, self.qos_a)
        # 创建设置USV武装状态发布者
        self.set_usv_arming_pubs[usv_id] = self.create_publisher(
            String, topic_arming, self.qos_a)
        # 创建LED控制发布者
        self.led_pubs[usv_id] = self.create_publisher(
            String, topic_led, self.qos_a)
        # 创建声音控制发布者
        self.sound_pubs[usv_id] = self.create_publisher(
            String, topic_sound, self.qos_a)
        # 创建动作控制发布者
        self.action_pubs[usv_id] = self.create_publisher(
            String, topic_action, self.qos_a)
        # 为每个USV创建一个NavigateToPoint动作客户端
        self.navigate_to_point_clients[usv_id] = ActionClient(self, NavigateToPoint, action_server_name)
        # 记录日志信息
        self.get_logger().info(f"添加新的USV命名空间: {usv_id}")

    # 移除一个USV命名空间
    def _remove_usv_namespace(self, ns):
        """
        移除指定的USV命名空间的订阅者和发布者
        
        Args:
            ns (str): USV的命名空间
        """
        # 从命名空间中提取USV ID
        usv_id = ns.lstrip('/')
        # 销毁并删除USV状态订阅者
        if usv_id in self.usv_state_subs:
            self.destroy_subscription(self.usv_state_subs[usv_id])
            del self.usv_state_subs[usv_id]
        # 销毁并删除设置USV模式发布者
        if usv_id in self.set_usv_mode_pubs:
            self.destroy_publisher(self.set_usv_mode_pubs[usv_id])
            del self.set_usv_mode_pubs[usv_id]
        # 销毁并删除设置USV武装状态发布者
        if usv_id in self.set_usv_arming_pubs:
            self.destroy_publisher(self.set_usv_arming_pubs[usv_id])
            del self.set_usv_arming_pubs[usv_id]
        # 销毁并删除LED控制发布者
        if usv_id in self.led_pubs:
            self.destroy_publisher(self.led_pubs[usv_id])
            del self.led_pubs[usv_id]
        # 销毁并删除声音控制发布者
        if usv_id in self.sound_pubs:
            self.destroy_publisher(self.sound_pubs[usv_id])
            del self.sound_pubs[usv_id]
        # 销毁并删除动作控制发布者
        if usv_id in self.action_pubs:
            self.destroy_publisher(self.action_pubs[usv_id])
            del self.action_pubs[usv_id]
        # 移除对应的导航动作客户端
        if usv_id in self.navigate_to_point_clients:
            del self.navigate_to_point_clients[usv_id]
        # 记录日志信息
        self.get_logger().info(f"移除USV命名空间: {usv_id}")

    # 处理 USV 状态回调
    def usv_state_callback(self, msg, usv_id):
        """
        处理USV状态更新回调
        
        Args:
            msg (UsvStatus): USV状态消息
            usv_id (str): USV标识符
        """
        # 检查消息类型是否正确
        if isinstance(msg, UsvStatus):
            # 构造状态数据字典
            state_data = {
                'namespace': usv_id,  # 命名空间
                'mode': msg.mode,  # 当前模式
                'connected': msg.connected,  # 连接状态
                'armed': msg.armed,  # 武装状态
                'guided': msg.guided,  # 引导模式状态
                'battery_voltage': msg.battery_voltage,  # 电池电压
                'battery_prcentage': msg.battery_percentage,  # 电池电量百分比
                'power_supply_status': msg.power_supply_status,  # 电源状态
                'position': {
                    'x': round(msg.position.x, 2),  # 保留两位小数减少数据量
                    'y': round(msg.position.y, 2),
                    'z': round(msg.position.z, 2)
                },  # 位置信息
                'velocity': {
                    'linear': {
                        'x': round(msg.velocity.linear.x, 2),
                        'y': round(msg.velocity.linear.y, 2),
                        'z': round(msg.velocity.linear.z, 2)
                    }
                },  # 速度信息
                'yaw': round(msg.yaw, 2),  # 偏航角
                'temperature': round(msg.temperature, 1),  # 温度
            }
            
            # 只有当状态发生变化时才更新和发送信号
            if usv_id not in self.usv_states or self.usv_states[usv_id] != state_data:
                # 更新USV状态字典
                self.usv_states[usv_id] = state_data
                # 发射信号，将更新后的USV状态列表发送给GUI界面
                # 限制信号发射频率，避免过于频繁的更新
                self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))

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
        if usv_id not in self.navigate_to_point_clients:
            self.get_logger().error(f"未找到USV {usv_id} 的导航客户端")
            # 更新导航状态为失败
            self.ros_signal.nav_status_update.emit(usv_id, "失败")
            return False

        # 检查Action服务器是否可用
        action_client = self.navigate_to_point_clients[usv_id]
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

    # 设置 USV 手动模式回调
    def set_manual_callback(self, msg):
        """
        设置USV为手动模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.get_logger().info("接收到手动模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "MANUAL")

    # 设置 USV 导航模式回调
    def set_guided_callback(self, msg):
        """
        设置USV为导航模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.get_logger().info("接收到导航模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "GUIDED")

    # 设置 USV ARCO 模式回调
    def set_arco_callback(self, msg):
        """
        设置USV为ARCO模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.get_logger().info("接收到ARCO模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "ARCO")

    # 设置 USV 舵机模式回调
    def set_steering_callback(self, msg):
        """
        设置USV为舵机模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.get_logger().info("接收到舵机模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "STEERING")

    # 通用设置模式方法
    def _set_mode_for_usvs(self, msg, mode):
        """
        为USV列表设置指定模式
        
        Args:
            msg: 包含USV列表的消息
            mode (str): 要设置的模式
        """
        # 如果消息是列表则直接使用，否则创建包含单个元素的列表
        usv_list = msg if isinstance(msg, list) else [msg]
        # 遍历USV列表
        for ns in usv_list:
            # 提取USV ID
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            # 检查USV是否存在对应的模式发布者
            if usv_id in self.set_usv_mode_pubs:
                # 创建模式消息
                mode_msg = String()
                mode_msg.data = mode
                # 将消息添加到发布队列
                self.publish_queue.put((self.set_usv_mode_pubs[usv_id], mode_msg))
            else:
                # 记录警告日志
                self.get_logger().warn(f"无效的命名空间 {usv_id}，跳过")

    # 设置 USV 武装回调
    def set_arming_callback(self, msg):
        """
        武装USV
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.get_logger().info("接收到武装命令")
        # 调用通用设置武装状态方法
        self._set_arming_for_usvs(msg, "ARMING")

    # 设置 USV 解除武装回调
    def set_disarming_callback(self, msg):
        """
        解除USV武装
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.get_logger().info("接收到解除武装命令")
        # 调用通用设置武装状态方法
        self._set_arming_for_usvs(msg, "DISARMING")

    # 通用设置武装状态方法
    def _set_arming_for_usvs(self, msg, arming_state):
        """
        为USV列表设置武装状态
        
        Args:
            msg: 包含USV列表的消息
            arming_state (str): 要设置的武装状态
        """
        # 如果消息是列表则直接使用，否则创建包含单个元素的列表
        usv_list = msg if isinstance(msg, list) else [msg]
        # 遍历USV列表
        for ns in usv_list:
            # 提取USV ID
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            # 检查USV是否存在对应的武装状态发布者
            if usv_id in self.set_usv_arming_pubs:
                # 创建武装状态消息
                arming_msg = String()
                arming_msg.data = arming_state
                # 将消息添加到发布队列
                self.publish_queue.put((self.set_usv_arming_pubs[usv_id], arming_msg))
            else:
                # 记录警告日志
                self.get_logger().warn(f"无效的命名空间 {usv_id}，跳过")

    # 设置集群目标点回调
    def set_cluster_target_point_callback(self, msg):
        """
        设置集群目标点
        
        Args:
            msg: 包含集群目标点的消息
        """
        try:
            # 记录日志信息
            self.get_logger().info("接收到集群目标点")
            # 检查msg对象是否具有targets属性，若有则使用该属性值，否则直接使用msg本身
            temp_list = msg.targets if hasattr(msg, 'targets') else msg
            # 验证消息格式是否正确，确保temp_list是一个列表类型
            if not isinstance(temp_list, list):
                # 记录错误日志
                self.get_logger().error("集群目标点格式错误")
                return

            # 检查是否为空列表，用于暂停/停止任务
            if not temp_list:
                self.get_logger().info("接收到空列表，暂停/停止集群任务")
                # 取消所有活动的Action任务
                for usv_id in list(self._usv_active_goals.keys()):
                    self._cancel_active_goal(usv_id)
                
                # 清空当前目标列表，结束整个集群任务
                self.current_targets = []
                # 重置任务状态
                self.run_step = 0
                self.usv_target_number = 0
                self.max_step = 1
                # 清空确认状态映射表
                self._usv_ack_map.clear()
                # 重置集群开始时间
                self._cluster_start_time = None
                return

            # 更新目标点和步骤信息，保存当前接收到的目标列表
            self.current_targets = temp_list
            # 初始化运行步骤为1，表示开始执行第一个步骤
            self.run_step = 1
            # 初始化USV目标编号为0
            self.usv_target_number = 0
            # 计算最大步骤数，遍历所有目标点获取step值的最大值，若列表为空则默认为1
            self.max_step = max(target.get('step', 1) for target in temp_list) if temp_list else 1

            # 初始化每艇 ack 状态，为每个USV设备初始化确认状态
            # 清空之前的确认状态映射表，准备记录新的状态
            self._usv_ack_map.clear()
            # 根据当前步骤获取相关的USV列表
            cluster_usv_list = self.get_usvs_by_step(self.current_targets, self.run_step)
            # 获取当前时间戳，用于记录状态更新时间
            now = self.get_clock().now().nanoseconds / 1e9
            # 为每个USV初始化确认状态，遍历USV列表为每个设备设置初始状态
            for ns in cluster_usv_list:
                # 确保ns是字典类型，避免类型错误
                if not isinstance(ns, dict):
                    continue
                # 从字典中获取USV的ID标识
                usv_id = ns.get('usv_id', None)
                # 检查是否成功获取到USV ID
                if usv_id is None:
                    continue
                # 为USV初始化确认状态：未确认、无确认时间、重试次数为0
                self._usv_ack_map[usv_id] = {'acked': False, 'last_send_time': None, 'retry': 0}
            # 记录集群操作开始时间，用于超时判断
            self._cluster_start_time = now

        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"处理集群目标点消息失败: {e}")

    # 发布集群目标点回调
    def publish_cluster_targets_callback(self):
        """
        定时向每艇下发当前 step 的目标，并基于每艇的 reached_target 做独立 ack / 超时处理
        """
        # 检查实例是否存在current_targets属性且不为None，避免在未初始化或重置状态下执行后续逻辑
        # getattr(self, 'current_targets', None)是一种安全的属性访问方式：
        # 1. 如果实例存在current_targets属性，返回该属性的值
        # 2. 如果实例不存在current_targets属性，返回默认值None
        # 3. 相比直接访问self.current_targets，可以避免AttributeError异常
        if not getattr(self, 'current_targets', None):
            return
            
        # 检查任务是否已暂停
        if self._cluster_task_paused:
            return

        try:
            # 根据当前步骤获取相关的USV列表，确定本步骤需要操作的无人艇
            cluster_usv_list = self.get_usvs_by_step(self.current_targets, self.run_step)
            # 检查当前步骤的USV列表是否为空，为空表示没有需要操作的无人艇
            if not cluster_usv_list:
                # 记录警告日志，提示当前步骤无USV需要操作
                self.get_logger().warn(f"步骤 {self.run_step} 的USV列表为空")
                # 判断是否已达到最大步骤数，若是则表示整个任务完成
                if self.run_step >= self.max_step:
                    # 清空当前目标列表，结束整个集群任务
                    self.current_targets = []
                    # 清空确认状态映射表
                    self._usv_ack_map.clear()
                    # 重置集群开始时间
                    self._cluster_start_time = None
                    # 重置任务暂停状态
                    self._cluster_task_paused = False
                    self.get_logger().info("集群任务已完成")
                else:
                    # 否则进入下一步，增加步骤计数器
                    self.run_step += 1
                    # 重置集群开始时间
                    self._cluster_start_time = self.get_clock().now().nanoseconds / 1e9
                # 处理完空列表情况后直接返回，不执行后续逻辑
                return

            # 确保 ack_map 已为当前 step 的艇初始化
            # 调用方法确保当前步骤的所有USV都在确认映射表中正确初始化
            self._initialize_ack_map_for_step(cluster_usv_list)

            # 更新每艇 ack 状态并处理超时/重试
            # 处理每艘USV的确认状态、超时和重试逻辑，确保指令可靠传递
            self._process_usv_ack_and_timeouts(cluster_usv_list)

            # 判断是否所有艇已 ack
            # 检查是否所有USV都已确认接收到目标点，用于判断是否可以进入下一步
            all_acked = all(info['acked'] for info in self._usv_ack_map.values()) if self._usv_ack_map else False
            # 如果所有USV都已确认，则准备进入下一步
            if all_acked:
                # 调用方法进入下一步操作，更新步骤状态和相关变量
                self._proceed_to_next_step()

            # 继续下发尚未 ack 的艇的目标点
            # 向尚未确认的USV重新发布目标点，确保所有艇都能接收到指令
            self._publish_targets_for_unacked_usvs(cluster_usv_list)

        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"发布集群目标点失败: {e}")

    # 为当前步骤初始化ack映射
    def _initialize_ack_map_for_step(self, cluster_usv_list):
        """
        为当前步骤初始化确认映射
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        # 获取当前系统时间戳，用于记录操作开始时间
        # 将纳秒转换为秒，便于后续时间计算和比较
        now = self.get_clock().now().nanoseconds / 1e9
        # 遍历当前步骤涉及的所有USV，为每艘船初始化确认状态
        for ns in cluster_usv_list:
            # 类型检查：确保当前元素是字典类型，避免因数据格式错误导致异常
            # 这是一种防御性编程实践，提高代码健壮性
            if not isinstance(ns, dict):
                # 检查ns是否为字典类型，如果不是则跳过当前循环
                continue
            # 从字典中安全地提取USV ID，使用get方法避免KeyError异常
            # 如果'usv_id'键不存在，返回None而不是抛出异常
            usv_id = ns.get('usv_id', None)
            # 检查是否成功获取到有效的USV ID，如果ID为空则跳过当前元素
            if usv_id is None:
                # 检查USV ID是否有效，无效则跳过
                continue
            # 检查当前USV是否已经在确认映射表中，避免重复初始化
            # 只有新的USV才会被添加到映射表中
            if usv_id not in self._usv_ack_map:
                # 为新USV初始化确认状态，包含三个关键信息：
                # 1. 'acked': False - 表示尚未确认接收到目标点
                # 2. 'ack_time': None - 表示尚未记录确认时间
                # 3. 'retry': 0 - 表示重试次数为0
                # 使用 last_send_time 替换 ack_time ---
                self._usv_ack_map[usv_id] = {'acked': False, 'last_send_time': None, 'retry': 0}
                # ----------------------------------------------------
                # 检查集群开始时间是否已设置，只在第一次初始化时设置
                # 确保集群开始时间记录的是整个集群任务的起始时间
                if self._cluster_start_time is None:
                    # 记录集群操作开始时间，用于后续的超时判断和时间计算
                    self._cluster_start_time = now

    # 处理USV确认和超时
    def _process_usv_ack_and_timeouts(self, cluster_usv_list):
        """
        处理USV确认和超时逻辑
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        # 获取当前时间戳，用于确认时间和超时计算
        now = self.get_clock().now().nanoseconds / 1e9
        # 遍历当前步骤的所有USV，检查每艘船的确认状态和超时情况
        for ns in cluster_usv_list:
            # 类型检查：确保当前元素是字典类型，避免因数据格式错误导致异常
            if not isinstance(ns, dict):
                # 检查元素是否为字典类型，防止数据格式错误
                continue
            # 安全提取USV ID，使用get方法避免KeyError异常
            usv_id = ns.get('usv_id')
            # 检查是否成功获取到有效的USV ID
            if not usv_id:
                # 无效ID则跳过当前循环
                continue


              # 检查该USV是否已经确认过
            info = self._usv_ack_map[usv_id]
            if not info['acked']:
                last_send_time = info.get('last_send_time')
                
                #  核心超时逻辑修改 ---
                # 只有在发送目标后才进行超时检查
                if last_send_time is not None: 
                    elapsed = now - last_send_time 
                    if elapsed > self._step_timeout:
                        self._handle_usv_timeout(usv_id, ns)

            # 由于我们使用Action接口处理导航任务，不再依赖is_reached_target字段
            # Action的反馈和结果机制已经提供了任务完成状态
            # 因此，我们使用内部确认机制跟踪任务状态
            # 检查该USV是否已经确认过
            #if not self._usv_ack_map[usv_id]['acked']:
                # 计算从集群开始到现在的经过时间，用于超时判断
                # 使用"or now"确保_cluster_start_time为None时不会出现异常
               # elapsed = now - (self._cluster_start_time or now)
                # 检查是否超过设定的步骤超时时间
                #if elapsed > self._step_timeout:
                    # 调用超时处理函数，执行超时后的相应操作
                    #self._handle_usv_timeout(usv_id, ns)
        
        # 检查是否达到最小确认率阈值，如果是则可以进入下一步
        self._check_and_proceed_on_ack_rate(cluster_usv_list)

    # 处理USV超时
    def _handle_usv_timeout(self, usv_id, ns):
        """
        处理USV超时情况
        
        Args:
            usv_id (str): USV标识符
            ns (dict): USV目标信息
        """
        now = self.get_clock().now().nanoseconds / 1e9
        # 获取指定USV的确认信息，包含确认状态、确认时间和重试次数
        info = self._usv_ack_map[usv_id]

        # 取消 Action 任务
        self._cancel_active_goal(usv_id) 
        
        # 检查重试次数是否小于最大重试次数，决定是否继续重试
        if info['retry'] < self._max_retries:
            # 增加重试次数计数器
            info['retry'] += 1

            # 更新最后发送时间 
            info['last_send_time'] = now 

            # 记录重试日志，包含USV ID和当前重试次数
            self.get_logger().warn(f"{usv_id} 超时，重试第 {info['retry']} 次")
            # 通过Action接口发送导航目标点
            pos = ns.get('position', {})
            yaw = ns.get('yaw', 0.0)
            # 检查位置信息是否完整
            if not all(k in pos for k in ('x', 'y')):
                self.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                return
            # 支持z坐标
            self.send_nav_goal_via_action(usv_id, pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0), yaw, 300.0)
        else:
            # 达到最大重试次数，标记为已确认并记录日志，不再重试
            info['acked'] = True
            #info['ack_time'] = self.get_clock().now().nanoseconds / 1e9
            # 记录错误日志，说明该USV已超时且达到最大重试次数
            self.get_logger().error(f"{usv_id} 超时且已达最大重试次数，跳过并继续下一步")

    # 推进到下一步
    def _proceed_to_next_step(self):
        """
        推进到下一个步骤
        """
        # 记录日志信息
        self.get_logger().info(f"步骤 {self.run_step} 完成，推进到下一步")
        # 增加步骤计数器
        self.run_step += 1
        # 重置USV目标编号
        self.usv_target_number = 0
        
        # 取消所有活动的Action任务（无论是否是最后一步）
        for usv_id in list(self._usv_active_goals.keys()):
            self._cancel_active_goal(usv_id)
        
        # 检查是否已完成所有步骤
        if self.run_step > self.max_step:
            # 清空当前目标列表
            self.current_targets = []
            # 清空确认映射表
            self._usv_ack_map.clear()
            # 重置集群开始时间
            self._cluster_start_time = None
            # 重置任务暂停状态
            self._cluster_task_paused = False
            # 记录日志信息
            self.get_logger().info("全部步骤完成")
        else:
            # 获取下一步的USV列表
            next_list = self.get_usvs_by_step(self.current_targets, self.run_step)
            # 获取当前时间
            now = self.get_clock().now().nanoseconds / 1e9
            # 重置并初始化确认映射表
            self._usv_ack_map.clear()
            # 为下一步的USV初始化确认映射表
            for ns in next_list:
                uid = ns.get('usv_id')
                if uid:
                    self._usv_ack_map[uid] = {'acked': False, 'last_send_time': None, 'retry': 0}
            # 更新集群开始时间为当前时间
            self._cluster_start_time = now
            
    def pause_cluster_task(self):
        """
        暂停集群任务
        """
        self._cluster_task_paused = True
        # 取消所有活动的Action任务
        for usv_id in list(self._usv_active_goals.keys()):
            self._cancel_active_goal(usv_id)
        self.get_logger().info("集群任务已暂停")
        
    def resume_cluster_task(self):
        """
        恢复集群任务
        """
        if self._cluster_task_paused:
            self._cluster_task_paused = False
            # 重置集群开始时间
            self._cluster_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("集群任务已恢复")
        else:
            self.get_logger().warn("集群任务未处于暂停状态，无需恢复")
            
    def is_cluster_task_paused(self):
        """
        检查集群任务是否已暂停
        
        Returns:
            bool: 如果任务已暂停返回True，否则返回False
        """
        return self._cluster_task_paused

    def _check_and_proceed_on_ack_rate(self, cluster_usv_list):
        """
        检查确认率是否达到阈值，如果达到则进入下一步
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        # 只有当当前步骤有USV时才进行检查
        if not cluster_usv_list:
            return
            
        # 计算当前确认率
        total_usvs = len(cluster_usv_list)
        acked_usvs = sum(1 for usv in cluster_usv_list 
                         if isinstance(usv, dict) and 
                         usv.get('usv_id') in self._usv_ack_map and 
                         self._usv_ack_map[usv.get('usv_id')]['acked'])
        
        # 避免除零错误
        if total_usvs == 0:
            return
            
        ack_rate = acked_usvs / total_usvs
        
        # 发送进度信息到UI
        progress_info = {
            'current_step': self.run_step,
            'total_steps': self.max_step,
            'total_usvs': total_usvs,
            'acked_usvs': acked_usvs,
            'ack_rate': ack_rate,
            'start_time': self._cluster_start_time,
            'elapsed_time': (self.get_clock().now().nanoseconds / 1e9) - (self._cluster_start_time or 0)
        }
        self.ros_signal.cluster_progress_update.emit(progress_info)
        
        # 如果确认率超过阈值且当前步骤尚未完成，则进入下一步
        if ack_rate >= self.MIN_ACK_RATE_FOR_PROCEED:
            # 检查是否所有USV都已处理（确认或超时）
            all_processed = all(
                isinstance(usv, dict) and 
                usv.get('usv_id') in self._usv_ack_map and 
                (self._usv_ack_map[usv.get('usv_id')]['acked'] or 
                 self._usv_ack_map[usv.get('usv_id')]['retry'] >= self._max_retries)
                for usv in cluster_usv_list
            )
            
            # 只有当所有USV都已处理时才进入下一步
            if all_processed and ack_rate >= self.MIN_ACK_RATE_FOR_PROCEED:
                self.get_logger().info(
                    f"确认率达到 {ack_rate*100:.1f}% (阈值: {self.MIN_ACK_RATE_FOR_PROCEED*100:.1f}%)，"
                    f"其中 {acked_usvs}/{total_usvs} 个USV已确认，进入下一步"
                )
                self._proceed_to_next_step()
            
    # 为未确认的USV发布目标点
    def _publish_targets_for_unacked_usvs(self, cluster_usv_list):
        """
        为未确认的USV发布目标点（仅在首次进入步骤时）
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        now = self.get_clock().now().nanoseconds / 1e9
        # 遍历USV列表
        for ns in cluster_usv_list:
            # 类型检查
            if not isinstance(ns, dict):
                continue
            # 获取USV ID
            usv_id = ns.get('usv_id')
            # 检查USV ID是否有效
            if not usv_id:
                continue
            # 检查USV是否已确认且是首次尝试（retry == 0）
            info = self._usv_ack_map.get(usv_id, {})
            if not info.get('acked', False) and info.get('retry', 0) == 0:
                # 获取位置信息
                pos = ns.get('position', {})
                # 检查位置信息是否完整
                if not all(k in pos for k in ('x', 'y')):
                    self.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                    continue
                # 更新最后发送时间 ---
                self._usv_ack_map[usv_id]['last_send_time'] = now 
               
                # 通过Action接口发送导航目标点
                yaw = ns.get('yaw', 0.0)
                # 支持z坐标
                self.send_nav_goal_via_action(usv_id, pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0), yaw, 300.0)

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
                # 检查位置信息是否完整
                if not all(k in pos for k in ('x', 'y')):
                    # 记录警告日志
                    self.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                    continue

                # 通过Action接口发送导航目标点
                yaw = ns.get('yaw', 0.0)
                # 支持z坐标
                self.send_nav_goal_via_action(usv_id, pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0), yaw, 300.0)
                # 记录日志信息
                self.get_logger().info(f"已下发目标点到 {usv_id}: {pos}, yaw: {yaw}")
        # 捕获异常并记录错误日志
        except Exception as e:
            self.get_logger().error(f"处理离群目标点失败: {e}")

    # 从USV目标列表中筛选出指定步骤(step)的USV目标
    def get_usvs_by_step(self, cluster_usv_list, step):
        """
        从USV目标列表中筛选出指定步骤的USV目标
        
        Args:
            cluster_usv_list (list): USV目标列表
            step (int): 步骤编号
            
        Returns:
            list: 指定步骤的USV目标列表
        """
        # 使用列表推导式筛选出指定步骤的USV目标
        return [usv for usv in cluster_usv_list if usv.get('step', 0) == step]

    # 处理 LED ,声音,转头命令回调
    def str_command_callback(self, msg):
        """
        处理字符串命令（LED、声音、转头等）
        
        Args:
            msg (str): 命令字符串
        """
        # 记录日志信息
        self.get_logger().info(f"接收到命令: {msg}")
        # 类型检查
        if not isinstance(msg, str):
            # 记录警告日志
            self.get_logger().warn("命令不是字符串，跳过")
            return

        # 创建命令消息
        command_str = String()
        command_str.data = msg

        # 识别命令类型
        command_type = self._identify_command_type(msg)

        # 遍历命名空间列表
        for ns in self.last_ns_list:
            # 提取USV ID
            usv_id = ns.lstrip('/')
            # 根据命令类型发送到对应的发布者
            if command_type == 'led' and usv_id in self.led_pubs:
                # 更新本地 LED 状态 
                self._update_local_led_state(usv_id, command_str)
                self.publish_queue.put((self.led_pubs[usv_id], command_str))
            if command_type == 'sound' and usv_id in self.sound_pubs:
                self.publish_queue.put((self.sound_pubs[usv_id], command_str))
            if command_type == 'action' and usv_id in self.action_pubs:
                self.publish_queue.put((self.action_pubs[usv_id], command_str))

    # 识别命令类型
    def _identify_command_type(self, msg):
        """
        识别命令类型
        
        Args:
            msg (str): 命令字符串
            
        Returns:
            str: 命令类型 ('led', 'sound', 'action', 'unknown')
        """
        # 转换为小写
        msg_lower = msg.lower()
        # 根据关键字识别命令类型
        if "sound" in msg_lower:
            return 'sound'
        elif "led" in msg_lower or "color" in msg_lower:
            return 'led'
        elif "neck" in msg_lower:
            return 'action'
        else:
            return 'unknown'

    # 检查 USV 之间的传染逻辑
    def check_usv_infect(self):
        """
        检查USV之间的LED传染逻辑
        """
        # 获取USV状态列表
        usv_list = list(self.usv_states.values())
        # 获取USV数量
        n = len(usv_list)
        # 如果USV数量少于2个，直接返回
        if n < 2:
            return

        # 创建传染对集合
        infect_pairs = set()
        # 检查所有USV对之间的距离
        for i in range(n):
            for j in range(i + 1, n):
                # 获取两个USV的状态
                usv_a = usv_list[i]
                usv_b = usv_list[j]
                # 获取USV ID
                id_a = usv_a['namespace']
                id_b = usv_b['namespace']

                # 计算两个USV之间的距离平方
                distance_squared = self._calculate_distance_squared(usv_a, usv_b)
                # 如果距离小于传染距离
                if distance_squared <= self.INFECTION_DISTANCE_SQUARED:
                    # 确定源和目标USV（按ID排序）
                    src_id, dst_id, src_color, src_mode = self._determine_infection_source(
                        id_a, id_b)
                    # 添加到传染对集合
                    infect_pairs.add((src_id, dst_id))

                    # 记录目标USV的原始LED模式
                    self._record_original_led_mode(dst_id)

                    # 发送传染指令
                    self._send_infection_command(dst_id, src_color)

        # 恢复离开传染范围的USV
        self._restore_led_modes(infect_pairs)

    # 计算两个USV之间的距离平方
    def _calculate_distance_squared(self, usv_a, usv_b):
        """
        计算两个USV之间的距离平方
        
        Args:
            usv_a (dict): 第一个USV的状态
            usv_b (dict): 第二个USV的状态
            
        Returns:
            float: 距离平方，如果无法计算则返回无穷大
        """
        # 获取位置信息
        pos_a = usv_a.get('position', {})
        pos_b = usv_b.get('position', {})
        try:
            # 提取坐标值
            xa, ya = float(pos_a.get('x', 0)), float(pos_a.get('y', 0))
            xb, yb = float(pos_b.get('x', 0)), float(pos_b.get('y', 0))
            # 计算距离平方
            return (xa - xb) ** 2 + (ya - yb) ** 2
        # 捕获异常并返回无穷大
        except Exception:
            return float('inf')

    # 确定传染源
    def _determine_infection_source(self, id_a, id_b):
        """
        确定两个USV之间的LED传染源和目标。
        传染源基于 USV ID 的字符串排序决定（ID靠前为源）。
        """
         # 从本地状态字典中获取 LED 状态 
        state_a = self._usv_current_led_state.get(id_a, {'mode': 'color_switching', 'color': [255, 0, 0]})
        state_b = self._usv_current_led_state.get(id_b, {'mode': 'color_switching', 'color': [255, 0, 0]})
        
         # 以编号字符串排序，靠前为主
        if id_a < id_b:
            src_id, dst_id = id_a, id_b
            src_color = state_a['color'] # 使用本地状态的颜色
            src_mode = state_a['mode'] # 使用本地状态的模式
        else:
            src_id, dst_id = id_b, id_a
            src_color = state_b['color'] # 使用本地状态的颜色
            src_mode = state_b['mode'] # 使用本地状态的模式
        # 以编号字符串排序，靠前为主
        #if id_a < id_b:
            #src_id, dst_id = id_a, id_b
            #src_color = usv_a.get('led_color', [255, 0, 0])
           # src_mode = usv_a.get('led_mode', 'color_switching')
        #else:
           # src_id, dst_id = id_b, id_a
            #src_color = usv_b.get('led_color', [255, 0, 0])
            #src_mode = usv_b.get('led_mode', 'color_switching')
        # 返回传染源信息
        return src_id, dst_id, src_color, src_mode

    # 记录原始LED模式
    def _record_original_led_mode(self, dst_id):
        """
        在开始传染前，记录目标 USV 的原始 LED 模式和颜色。
        """
        # 从本地状态字典中获取原始 LED 状态 ---
        if dst_id not in self._usv_led_modes:
            # 始终使用本地维护的状态作为原始状态
            original_state = self._usv_current_led_state.get(dst_id, {'mode': 'color_switching', 'color': [255, 0, 0]})
            dst_led_mode = original_state['mode']
            dst_led_color = original_state['color']
            
            self._usv_led_modes[dst_id] = (dst_led_mode, dst_led_color)



        # 如果目标USV不在LED模式字典中
        #if dst_id not in self._usv_led_modes:
            # 根据ID确定目标USV的状态
            #if id_b == dst_id:
                #dst_led_mode = usv_b.get('led_mode', 'color_switching')
                #dst_led_color = usv_b.get('led_color', [255, 0, 0])
            #else:
                #dst_led_mode = usv_a.get('led_mode', 'color_switching')
                #dst_led_color = usv_a.get('led_color', [255, 0, 0])
            # 记录LED模式
            #self._usv_led_modes[dst_id] = (dst_led_mode, dst_led_color)

    # 发送传染命令
    def _send_infection_command(self, dst_id, src_color):
        """
        发送LED传染命令
        """
        # 构造传染命令
        infect_cmd = f"color_infect|{src_color[0]},{src_color[1]},{src_color[2]}"
        # 检查目标USV是否存在对应的LED发布者
        if dst_id in self.led_pubs:
            # 创建消息
            msg = String()
            msg.data = infect_cmd
            # 将消息添加到发布队列
            self.publish_queue.put((self.led_pubs[dst_id], msg))

    # 恢复LED模式
    def _restore_led_modes(self, infect_pairs):
        """
        恢复离开传染范围的USV的LED模式
        """
        # 遍历LED模式字典
        for dst_id in list(self._usv_led_modes.keys()):
            # 检查目标USV是否在传染对中
            if not any(dst_id == pair[1] for pair in infect_pairs):
                # 获取原始LED模式和颜色
                mode, color = self._usv_led_modes[dst_id]
                # 检查目标USV是否存在对应的LED发布者
                if dst_id in self.led_pubs:
                    # 根据模式构造命令
                    if mode == 'color_select':
                        cmd = f"color_select|{color[0]},{color[1]},{color[2]}"
                    else:
                        cmd = mode
                    # 创建消息
                    msg = String()
                    msg.data = cmd
                    # 将消息添加到发布队列
                    self.publish_queue.put((self.led_pubs[dst_id], msg))
                # 删除记录的LED模式
                del self._usv_led_modes[dst_id]
    # 更新本地 LED 状态
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

    # 销毁节点资源
    def destroy_node(self):
        """
        销毁节点资源
        """
        # 记录日志信息
        self.get_logger().info("销毁节点资源...")
        # 销毁所有订阅者
        for usv_id in list(self.usv_state_subs.keys()):
            self.destroy_subscription(self.usv_state_subs[usv_id])
            del self.usv_state_subs[usv_id]
        # 销毁所有模式发布者
        for usv_id in list(self.set_usv_mode_pubs.keys()):
            self.destroy_publisher(self.set_usv_mode_pubs[usv_id])
            del self.set_usv_mode_pubs[usv_id]
        # 销毁所有武装状态发布者
        for usv_id in list(self.set_usv_arming_pubs.keys()):
            self.destroy_publisher(self.set_usv_arming_pubs[usv_id])
            del self.set_usv_arming_pubs[usv_id]
        # 销毁所有LED发布者
        for usv_id in list(self.led_pubs.keys()):
            self.destroy_publisher(self.led_pubs[usv_id])
            del self.led_pubs[usv_id]
        # 销毁所有声音发布者
        for usv_id in list(self.sound_pubs.keys()):
            self.destroy_publisher(self.sound_pubs[usv_id])
            del self.sound_pubs[usv_id]
        # 销毁所有动作发布者
        for usv_id in list(self.action_pubs.keys()):
            self.destroy_publisher(self.action_pubs[usv_id])
            del self.action_pubs[usv_id]
        # 调用父类的销毁方法
        super().destroy_node()