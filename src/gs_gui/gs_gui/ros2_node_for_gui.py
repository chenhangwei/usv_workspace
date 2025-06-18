from math import dist
from scipy import cluster
from sympy import Quaternion, public
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from gs_gui.ros_signal import ROSSignal
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from common_interfaces.msg import UsvStatus, UsvSetPoint
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import tf_transformations
import queue
import threading

class GroundStationNode(Node):
    # 初始化
    def __init__(self, signal):
        super().__init__('groundstationnode')
        self.ros_signal = signal
        self.qos_a = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        
        self.usv_state_subs = {}
        self.set_usv_target_position_pubs = {}
        self.set_usv_target_velocity_pubs = {}
        self.set_usv_mode_pubs = {}
        self.set_usv_arming_pubs = {}
        self.led_pubs = {}
        self.sound_pubs = {}
        self.action_pubs = {}
        self.usv_states = {}
        self.last_ns_list = []
        self.is_runing = False
        self.run_step = 0
        self.usv_target_number = 0
        self.current_targets = []
        self.max_step = 1
        
        self.publish_queue = queue.Queue()
        self.publish_thread = threading.Thread(target=self.process_publish_queue, daemon=True)
        self.publish_thread.start()
        
        self.ns_timer = self.create_timer(2.0, self.update_subscribers_and_publishers)
        self.target_timer = self.create_timer(0.5, self.publish_cluster_targets_callback)
        self.infect_check_timer = self.create_timer(1.0, self.check_usv_infect)
        
        self._usv_led_modes = {}  # 记录每个USV的LED原始模式和颜色
        self._usv_infecting = set()  # 当前处于传染状态的USV对

        self.update_subscribers_and_publishers()
       
    # 在独立线程中异步处理消息发布队列
    def process_publish_queue(self):
        while rclpy.ok():
            try:
                pub, msg = self.publish_queue.get(timeout=1.0)
                pub.publish(msg)
                self.get_logger().info(f"发布消息到 {pub.topic}")
                self.publish_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"发布消息失败: {e}")
    
    # 获取当前节点的名称和命名空间，为新的 USV 节点创建订阅和发布器
    def update_subscribers_and_publishers(self):
        namespaces = self.get_node_names_and_namespaces()
        current_ns_list = list(set([ns for _, ns in namespaces if ns.startswith('/usv_')]))        
        if current_ns_list == self.last_ns_list:
            return
        
        self.last_ns_list = current_ns_list
        existing_ns = set(self.usv_state_subs.keys())
        new_ns = set(current_ns_list) - existing_ns
        removed_ns = existing_ns - set(current_ns_list)
        
        for ns in new_ns:

            usv_id = ns.lstrip('/')
            topic_state = f"{ns}/usv_state"
            topic_position = f"{ns}/set_usv_target_position"
            topic_velocity = f"{ns}/set_usv_target_velocity"
            topic_mode = f"{ns}/set_usv_mode"
            topic_arming = f"{ns}/set_usv_arming"
            topic_led = f"{ns}/gs_led_command"
            topic_sound = f"{ns}/gs_sound_command"
            topic_action = f"{ns}/gs_action_command"
            
            self.usv_state_subs[usv_id] = self.create_subscription(
                UsvStatus, topic_state, lambda msg, id=usv_id: self.usv_state_callback(msg, id), self.qos_a)
            self.set_usv_target_position_pubs[usv_id] = self.create_publisher(
                PoseStamped, topic_position, self.qos_a)
            self.set_usv_target_velocity_pubs[usv_id] = self.create_publisher(
                Float32, topic_velocity, self.qos_a)
            self.set_usv_mode_pubs[usv_id] = self.create_publisher(
                String, topic_mode, self.qos_a)
            self.set_usv_arming_pubs[usv_id] = self.create_publisher(
                String, topic_arming, self.qos_a)
            self.led_pubs[usv_id] = self.create_publisher(
                String, topic_led, self.qos_a)
            self.sound_pubs[usv_id] = self.create_publisher(
                String, topic_sound, self.qos_a)
            self.action_pubs[usv_id] = self.create_publisher(
                String, topic_action, self.qos_a)
            self.get_logger().info(f"New subscribers and publishers for {usv_id}")

        for ns in removed_ns:
            usv_id = ns.lstrip('/')
            if usv_id in self.usv_state_subs:
                self.destroy_subscription(self.usv_state_subs[usv_id])
                del self.usv_state_subs[usv_id]
            if usv_id in self.set_usv_target_position_pubs:
                self.destroy_publisher(self.set_usv_target_position_pubs[usv_id])
                del self.set_usv_target_position_pubs[usv_id]
            if usv_id in self.set_usv_target_velocity_pubs:
                self.destroy_publisher(self.set_usv_target_velocity_pubs[usv_id])
                del self.set_usv_target_velocity_pubs[usv_id]
            if usv_id in self.set_usv_mode_pubs:
                self.destroy_publisher(self.set_usv_mode_pubs[usv_id])
                del self.set_usv_mode_pubs[usv_id]
            if usv_id in self.set_usv_arming_pubs:
                self.destroy_publisher(self.set_usv_arming_pubs[usv_id])
                del self.set_usv_arming_pubs[usv_id]
            if usv_id in self.led_pubs:
                self.destroy_publisher(self.led_pubs[usv_id])
                del self.led_pubs[usv_id]
            if usv_id in self.sound_pubs:
                self.destroy_publisher(self.sound_pubs[usv_id])
                del self.sound_pubs[usv_id]
            if usv_id in self.action_pubs:
                self.destroy_publisher(self.action_pubs[usv_id])
                del self.action_pubs[usv_id]
            self.get_logger().info(f"Destroying subscribers and publishers for {usv_id}")
    # 处理 USV 状态回调
    def usv_state_callback(self, msg, usv_id):
        if isinstance(msg, UsvStatus):
            state_data = {
                'namespace': usv_id,
                'mode': msg.mode,
                'connected': msg.connected,
                'armed': msg.armed,
                'guided': msg.guided,
                'battery_voltage': msg.battery_voltage,
                'battery_prcentage': msg.battery_percentage,
                'power_supply_status': msg.power_supply_status,
                'position': msg.position,
                'velocity': msg.velocity,
                'yaw': msg.yaw,
                'is_reached_target': msg.reached_target, # 是否到达目标点
                'temperature': msg.temperature,# 温度
            }
            self.usv_states[usv_id] = state_data

            self.ros_signal.receive_state_list.emit(list(self.usv_states.values())) # 通过Qt信号将所有USV状态推送给界面，触发界面刷新
            
    # 设置 USV 模式回调
    def set_manual_callback(self, msg):
        self.get_logger().info("Received manual mode command")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_mode_pubs:
                mode_msg = String()
                mode_msg.data = "MANUAL"
                self.publish_queue.put((self.set_usv_mode_pubs[usv_id], mode_msg))
            else:
                self.get_logger().warn(f"Invalid namespace {usv_id}, skip")
    
    # 设置 USV 导航模式回调
    def set_guided_callback(self, msg):
        self.get_logger().info("Received guided mode command")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_mode_pubs:
                mode_msg = String()
                mode_msg.data = "GUIDED"
                self.publish_queue.put((self.set_usv_mode_pubs[usv_id], mode_msg))
            else:
                self.get_logger().warn(f"Invalid namespace {usv_id}, skip")
    
    # 设置 USV ARCO 模式回调
    def set_arco_callback(self, msg):
        self.get_logger().info("Received ARCO mode command")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_mode_pubs:
                mode_msg = String()
                mode_msg.data = "ARCO"
                self.publish_queue.put((self.set_usv_mode_pubs[usv_id], mode_msg))
            else:
                self.get_logger().warn(f"Invalid namespace {usv_id}, skip")
   
    # 设置 USV 舵机模式回调
    def set_steering_callback(self, msg):
        self.get_logger().info("Received steering mode command")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_mode_pubs:
                mode_msg = String()
                mode_msg.data = "STEERING"
                self.publish_queue.put((self.set_usv_mode_pubs[usv_id], mode_msg))
            else:
                self.get_logger().warn(f"Invalid namespace {usv_id}, skip")

    # 设置 USV 武装回调
    def set_arming_callback(self, msg):
        self.get_logger().info("Received armed command")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_arming_pubs:
                arming_msg = String()
                arming_msg.data = "ARMING"
                self.publish_queue.put((self.set_usv_arming_pubs[usv_id], arming_msg))
            else:
                self.get_logger().warn(f"Invalid namespace {usv_id}, skip")
    
    # 设置 USV 解除武装回调
    def set_disarming_callback(self, msg):
        self.get_logger().info("Received disarmed command")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_arming_pubs:
                disarming_msg = String()
                disarming_msg.data = "DISARMING"
                self.publish_queue.put((self.set_usv_arming_pubs[usv_id], disarming_msg))
            else:
                self.get_logger().warn(f"Invalid namespace {usv_id}, skip")

    # 设置集群目标点回调
    def set_cluster_target_point_callback(self, msg):
        self.get_logger().info("Received cluster target point")
        try:
            temp_list = msg.targets if hasattr(msg, 'targets') else msg
            if not isinstance(temp_list, list):
                return

            self.current_targets = temp_list
            self.run_step = 1
            self.usv_target_number = 0
            self.max_step = max(target.get('step', 1) for target in temp_list) if temp_list else 1

            # for ns in temp_list:
            #     if not isinstance(ns, dict):
            #         continue
            #     usv_id = ns.get('usv_id', None)
            #     if usv_id is None:
            #         continue
            #     if usv_id in self.usv_states and not self.usv_states[usv_id].get('is_runing', True):
            #         self.usv_target_number += 1
            
        except Exception as e:
            self.get_logger().error(f"Failed to process cluster target point: {e}")

    # 发布集群目标点回调
    def publish_cluster_targets_callback(self):
        if not self.current_targets:
            return

        try:
            cluster_usv_list = self.get_usvs_by_step(self.current_targets, self.run_step)
            if not cluster_usv_list:
                self.get_logger().warn(f"The USV list for step {self. run_step} is empty")
                if self.run_step >= self.max_step:
                    self.current_targets = []
                else:
                    self.run_step += 1
                return

            arrived_count = sum(1 for usv_id in self.usv_states if not self.usv_states.get(usv_id, {}).get('is_runing', True))
            for ns in cluster_usv_list:
                if not isinstance(ns, dict):
                    self.get_logger().warn(f"Invalid target format: {ns}, skip")
                    continue
                usv_id = ns.get('usv_id', None)
                if usv_id is None:
                    self.get_logger().warn("USV_ID not found, skip")
                    continue
                if usv_id not in self.set_usv_target_position_pubs:
                    self.get_logger().warn(f"Publisher does not exist for {usv_id}, current publisher: {list(self.set_usv_target_position_pubs.keys())}")
                    continue

                target_point_msg = PoseStamped()
                target_point_msg.header.frame_id = "map"
                target_point_msg.header.stamp = self.get_clock().now().to_msg()
                target_point_msg.pose.position.x = ns.get('position', {}).get('x', 0.0)
                target_point_msg.pose.position.y = ns.get('position', {}).get('y', 0.0)
                target_point_msg.pose.position.z = ns.get('position', {}).get('z', 0.0)
                yaw = ns.get('yaw', 0.0)
                quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                target_point_msg.pose.orientation.x = quaternion[0]
                target_point_msg.pose.orientation.y = quaternion[1]
                target_point_msg.pose.orientation.z = quaternion[2]
                target_point_msg.pose.orientation.w = quaternion[3]
                self.publish_queue.put((self.set_usv_target_position_pubs[usv_id], target_point_msg))
                self.get_logger().info(f'发送坐标：{target_point_msg}')

            if arrived_count >= len(cluster_usv_list):
                self.run_step += 1
                self.usv_target_number = 0
                if self.run_step > self.max_step:
                    self.current_targets = []
        except Exception as e:
            self.get_logger().error(f"Failed to publish cluster target point: {e}")

   
        self.get_logger().info("接收到集群目标速度")
        try:
            # 假设 msg 是 UsvSetPoint 或 UsvSetPoint 列表
            usv_list = [msg] if not isinstance(msg, list) else msg
            for target in usv_list:
                if not hasattr(target, 'usv_id'):
                    self.get_logger().warn(f"无效的目标格式: {target}, 跳过")
                    continue
                usv_id = target.usv_id
                if usv_id not in self.set_usv_target_velocity_pubs:
                    self.get_logger().warn(f"无效 usv_id 或发布器不存在: {usv_id}, 跳过")
                    continue
                velocity = target.velocity
                target_velocity_msg = Float32()
                target_velocity_msg.data = float(velocity)
                self.publish_queue.put((self.set_usv_target_velocity_pubs[usv_id], target_velocity_msg))
                # self.get_logger().info(f"发布速度到 {usv_id}: {velocity}")
        except Exception as e:
            self.get_logger().error(f"处理集群目标速度失败: {e}")

    # 设置离群目标点回调
    def set_departed_target_point_callback(self, msg):
        self.get_logger().info("接收到离群目标点")
        try:
            usv_list = msg.targets if hasattr(msg, 'targets') else msg
            if not isinstance(usv_list, list):
                self.get_logger().error(f"usv_list 不是列表: {usv_list}")
                return

            for ns in usv_list:
                if not isinstance(ns, dict):
                    self.get_logger().warning(f"无效的目标格式: {ns}, 跳过")
                    continue
                usv_id = ns.get('usv_id')
                if not usv_id or usv_id not in self.set_usv_target_position_pubs:
                    self.get_logger().warning(f"无效 usv_id 或发布器不存在: {usv_id}, 跳过")
                    continue

                pos = ns.get('position', {})
                if not all(k in pos for k in ('x', 'y', 'z')):
                    self.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                    continue

                target_point_msg = PoseStamped()
                target_point_msg.header.frame_id = "map"
                target_point_msg.header.stamp = self.get_clock().now().to_msg()
                target_point_msg.pose.position.x = pos.get('x', 0.0)
                target_point_msg.pose.position.y = pos.get('y', 0.0)
                target_point_msg.pose.position.z = pos.get('z', 0.0)
                yaw = ns.get('yaw', 0.0)
                quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                target_point_msg.pose.orientation.x = quaternion[0]
                target_point_msg.pose.orientation.y = quaternion[1]
                target_point_msg.pose.orientation.z = quaternion[2]
                target_point_msg.pose.orientation.w = quaternion[3]
                self.publish_queue.put((self.set_usv_target_position_pubs[usv_id], target_point_msg))
                self.get_logger().info(f"已下发目标点到 {usv_id}: {pos}, yaw: {yaw}")
        except Exception as e:
            self.get_logger().error(f"处理离群目标点失败: {e}")
    
    # 从USV目标列表中筛选出指定步骤(step)的USV目标
    def get_usvs_by_step(self, cluster_usv_list, step):
        return [usv for usv in cluster_usv_list if usv.get('step', 0) == step]
    
    # 处理 LED ,声音,转头命令回调
    def str_command_callback(self, msg):
        self.get_logger().info(f"接收到 LED 或声音命令: {msg}")
        if not isinstance(msg, str):
            self.get_logger().warn("命令不是字符串，跳过")
            return
        command_str = String()
        command_str.data = msg

        is_sound = "sound" in msg.lower() # 是否为声音命令
        is_led = ("led" in msg.lower()) or ("color" in msg.lower())# 是否为LED命令
        is_action = "neck" in msg.lower()

        for ns in self.last_ns_list:
            usv_id = ns.lstrip('/')
            if is_led and usv_id in self.led_pubs:
                self.publish_queue.put((self.led_pubs[usv_id], command_str))
            if is_sound and usv_id in self.sound_pubs:
                self.publish_queue.put((self.sound_pubs[usv_id], command_str))
            if is_action and usv_id in self.action_pubs:
                self.publish_queue.put((self.action_pubs[usv_id], command_str))

    # 检查 USV 之间的传染逻辑
    def check_usv_infect(self):
        usv_list = list(self.usv_states.values())
        n = len(usv_list)
        if n < 2:
            return
        infect_pairs = set()
        for i in range(n):
            for j in range(i+1, n):
                usv_a = usv_list[i]
                usv_b = usv_list[j]
                id_a = usv_a['namespace']
                id_b = usv_b['namespace']
                pos_a = usv_a.get('position', {})
                pos_b = usv_b.get('position', {})
                try:
                    xa, ya = float(pos_a.get('x', 0)), float(pos_a.get('y', 0))
                    xb, yb = float(pos_b.get('x', 0)), float(pos_b.get('y', 0))
                except Exception:
                    continue
                dist2 = (xa-xb)**2 + (ya-yb)**2
                if dist2 <= 4.0:  # 2米内
                    # 以编号字符串排序，靠前为主
                    if id_a < id_b:
                        src, dst = id_a, id_b
                        src_color = usv_a.get('led_color', [255,0,0])
                        src_mode = usv_a.get('led_mode', 'color_switching')
                    else:
                        src, dst = id_b, id_a
                        src_color = usv_b.get('led_color', [255,0,0])
                        src_mode = usv_b.get('led_mode', 'color_switching')
                    infect_pairs.add((src, dst))
                    # 记录原始LED模式
                    if dst not in self._usv_led_modes:
                        dst_led_mode = usv_b.get('led_mode', 'color_switching') if id_b == dst else usv_a.get('led_mode', 'color_switching')
                        dst_led_color = usv_b.get('led_color', [255,0,0]) if id_b == dst else usv_a.get('led_color', [255,0,0])
                        self._usv_led_modes[dst] = (dst_led_mode, dst_led_color)
                    # 发送 color_infect 指令
                    infect_cmd = f"color_infect|{src_color[0]},{src_color[1]},{src_color[2]}"
                    if dst in self.led_pubs:
                        msg = String()
                        msg.data = infect_cmd
                        self.publish_queue.put((self.led_pubs[dst], msg))
        # 恢复离开2米的USV
        for dst in list(self._usv_led_modes.keys()):
            if not any(dst == pair[1] for pair in infect_pairs):
                mode, color = self._usv_led_modes[dst]
                if dst in self.led_pubs:
                    if mode == 'color_select':
                        cmd = f"color_select|{color[0]},{color[1]},{color[2]}"
                    else:
                        cmd = mode
                    msg = String()
                    msg.data = cmd
                    self.publish_queue.put((self.led_pubs[dst], msg))
                del self._usv_led_modes[dst]

    # 销毁节点资源
    def destroy_node(self):
        self.get_logger().info("销毁节点资源...")
        for usv_id in list(self.usv_state_subs.keys()):
            self.destroy_subscription(self.usv_state_subs[usv_id])
            del self.usv_state_subs[usv_id]
        for usv_id in list(self.set_usv_target_position_pubs.keys()):
            self.destroy_publisher(self.set_usv_target_position_pubs[usv_id])
            del self.set_usv_target_position_pubs[usv_id]
        for usv_id in list(self.set_usv_target_velocity_pubs.keys()):
            self.destroy_publisher(self.set_usv_target_velocity_pubs[usv_id])
            del self.set_usv_target_velocity_pubs[usv_id]
        for usv_id in list(self.set_usv_mode_pubs.keys()):
            self.destroy_publisher(self.set_usv_mode_pubs[usv_id])
            del self.set_usv_mode_pubs[usv_id]
        for usv_id in list(self.set_usv_arming_pubs.keys()):
            self.destroy_publisher(self.set_usv_arming_pubs[usv_id])
            del self.set_usv_arming_pubs[usv_id]
        for usv_id in list(self.led_pubs.keys()):
            self.destroy_publisher(self.led_pubs[usv_id])
            del self.led_pubs[usv_id]
        for usv_id in list(self.sound_pubs.keys()):
            self.destroy_publisher(self.sound_pubs[usv_id])
            del self.sound_pubs[usv_id]
        for usv_id in list(self.action_pubs.keys()):
            self.destroy_publisher(self.action_pubs[usv_id])
            del self.action_pubs[usv_id]
        super().destroy_node()