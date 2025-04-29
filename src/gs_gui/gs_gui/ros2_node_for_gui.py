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
        
        self.update_subscribers_and_publishers()
        
        # # 订阅集群目标点
        # self.cluster_target_sub = self.create_subscription(
        #     UsvSetPoint, '/usv_cluster/target_points', self.set_cluster_target_point_callback, self.qos_a)

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

    def update_subscribers_and_publishers(self):
        namespaces = self.get_node_names_and_namespaces()
        current_ns_list = list(set([ns for _, ns in namespaces if ns.startswith('/usv_')]))
        # self.get_logger().info(f"检测到的命名空间: {current_ns_list}")
        
        if current_ns_list == self.last_ns_list:
            return
        
        self.last_ns_list = current_ns_list
        existing_ns = set(self.usv_state_subs.keys())
        new_ns = set(current_ns_list) - existing_ns
        removed_ns = existing_ns - set(current_ns_list)
        
        for ns in new_ns:
            # 存储不带斜杠的 usv_id（usv_01 而非 /usv_01）
            usv_id = ns.lstrip('/')
            topic_state = f"{ns}/usv_state"
            topic_position = f"{ns}/set_usv_target_position"
            topic_velocity = f"{ns}/set_usv_target_velocity"
            topic_mode = f"{ns}/set_usv_mode"
            topic_arming = f"{ns}/set_usv_arming"
            topic_led = f"{ns}/gs_led_command"
            topic_sound = f"{ns}/gs_sound_command"
            
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
            self.get_logger().info(f"新建订阅者和发布者 for {usv_id}")

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
            self.get_logger().info(f"销毁订阅者和发布者 for {usv_id}")

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
                'is_runing': msg.is_runing,
            }
            self.usv_states[usv_id] = state_data
            # self.get_logger().info(f"更新 USV 状态: {usv_id}, is_runing: {msg.is_runing}")
            self.ros_signal.receive_state_list.emit(list(self.usv_states.values()))

    def set_manual_callback(self, msg):
        self.get_logger().info("接收到手动模式命令")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_mode_pubs:
                mode_msg = String()
                mode_msg.data = "MANUAL"
                self.publish_queue.put((self.set_usv_mode_pubs[usv_id], mode_msg))
            else:
                self.get_logger().warn(f"无效命名空间 {usv_id}，跳过")

    def set_guided_callback(self, msg):
        self.get_logger().info("接收到引导模式命令")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_mode_pubs:
                mode_msg = String()
                mode_msg.data = "GUIDED"
                self.publish_queue.put((self.set_usv_mode_pubs[usv_id], mode_msg))
            else:
                self.get_logger().warn(f"无效命名空间 {usv_id}，跳过")

    def set_arming_callback(self, msg):
        self.get_logger().info("接收到武装命令")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_arming_pubs:
                arming_msg = String()
                arming_msg.data = "ARMING"
                self.publish_queue.put((self.set_usv_arming_pubs[usv_id], arming_msg))
            else:
                self.get_logger().warn(f"无效命名空间 {usv_id}，跳过")

    def set_disarming_callback(self, msg):
        self.get_logger().info("接收到解除武装命令")
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            if usv_id in self.set_usv_arming_pubs:
                disarming_msg = String()
                disarming_msg.data = "DISARMING"
                self.publish_queue.put((self.set_usv_arming_pubs[usv_id], disarming_msg))
            else:
                self.get_logger().warn(f"无效命名空间 {usv_id}，跳过")

    def set_cluster_target_point_callback(self, msg):
        self.get_logger().info("接收到集群目标点")
        try:
            temp_list = msg.targets if hasattr(msg, 'targets') else msg
            if not isinstance(temp_list, list):
                self.get_logger().error(f"temp_list 不是列表: {temp_list}")
                return

            self.current_targets = temp_list
            self.run_step = 1
            self.usv_target_number = 0
            self.max_step = max(target.get('step', 1) for target in temp_list) if temp_list else 1

            for ns in temp_list:
                if not isinstance(ns, dict):
                    self.get_logger().warn(f"无效的目标格式: {ns}, 跳过")
                    continue
                usv_id = ns.get('usv_id', None)
                if usv_id is None:
                    self.get_logger().warn("未找到 usv_id, 跳过")
                    continue
                if usv_id in self.usv_states and not self.usv_states[usv_id].get('is_runing', True):
                    self.usv_target_number += 1
            # self.get_logger().info(f"目标点: {temp_list}, 发布器: {list(self.set_usv_target_position_pubs.keys())}")
        except Exception as e:
            self.get_logger().error(f"处理集群目标点失败: {e}")

    def publish_cluster_targets_callback(self):
        if not self.current_targets:
            return

        try:
            cluster_usv_list = self.get_usvs_by_step(self.current_targets, self.run_step)
            if not cluster_usv_list:
                self.get_logger().warn(f"步骤 {self.run_step} 的 USV 列表为空")
                if self.run_step >= self.max_step:
                    self.current_targets = []
                else:
                    self.run_step += 1
                return

            arrived_count = sum(1 for usv_id in self.usv_states if not self.usv_states.get(usv_id, {}).get('is_runing', True))
            for ns in cluster_usv_list:
                if not isinstance(ns, dict):
                    self.get_logger().warn(f"无效的目标格式: {ns}, 跳过")
                    continue
                usv_id = ns.get('usv_id', None)
                if usv_id is None:
                    self.get_logger().warn("未找到 usv_id, 跳过")
                    continue
                if usv_id not in self.set_usv_target_position_pubs:
                    self.get_logger().warn(f"发布器不存在 for {usv_id}, 当前发布器: {list(self.set_usv_target_position_pubs.keys())}")
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
                # self.get_logger().info(f"发布目标点到 {usv_id}: {target_point_msg.pose.position}")

            if arrived_count >= len(cluster_usv_list):
                self.run_step += 1
                self.usv_target_number = 0
                if self.run_step > self.max_step:
                    self.current_targets = []
        except Exception as e:
            self.get_logger().error(f"发布集群目标点失败: {e}")

    def set_cluster_target_velocity_callback(self, msg):
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

    def set_departed_target_point_callback(self, msg):
        self.get_logger().info("接收到离群目标点")
        try:
            usv_list = msg.targets if hasattr(msg, 'targets') else msg
            if not isinstance(usv_list, list):
                self.get_logger().error(f"usv_list 不是列表: {usv_list}")
                return

            for ns in usv_list:
                if not isinstance(ns, dict):
                    self.get_logger().warn(f"无效的目标格式: {ns}, 跳过")
                    continue
                usv_id = ns.get('usv_id', None)
                if usv_id is None or usv_id not in self.set_usv_target_position_pubs:
                    self.get_logger().warn(f"无效 usv_id 或发布器不存在: {usv_id}, 跳过")
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
        except Exception as e:
            self.get_logger().error(f"处理离群目标点失败: {e}")

    def set_departed_target_velocity_callback(self, msg):
        self.get_logger().info("接收到离群目标速度")
        try:
            usv_list = msg.targets if hasattr(msg, 'targets') else msg
            if not isinstance(usv_list, list):
                self.get_logger().error(f"usv_list 不是列表: {usv_list}")
                return

            for ns in usv_list:
                if not isinstance(ns, dict):
                    self.get_logger().warn(f"无效的目标格式: {ns}, 跳过")
                    continue
                usv_id = ns.get('usv_id', None)
                if usv_id is None or usv_id not in self.set_usv_target_velocity_pubs:
                    self.get_logger().warn(f"无效 usv_id 或发布器不存在: {usv_id}, 跳过")
                    continue
                velocity = ns.get('velocity', 0.0)
                target_velocity_msg = Float32()
                target_velocity_msg.data = float(velocity)
                self.publish_queue.put((self.set_usv_target_velocity_pubs[usv_id], target_velocity_msg))
        except Exception as e:
            self.get_logger().error(f"处理离群目标速度失败: {e}")

    def get_usvs_by_step(self, cluster_usv_list, step):
        return [usv for usv in cluster_usv_list if usv.get('step', 0) == step]

    def str_command_callback(self, msg):
        self.get_logger().info(f"接收到 LED 或声音命令: {msg}")
        if not isinstance(msg, str):
            self.get_logger().warn("命令不是字符串，跳过")
            return
        command_str = String()
        command_str.data = msg
        for ns in self.last_ns_list:
            usv_id = ns.lstrip('/')
            if usv_id in self.led_pubs:
                self.publish_queue.put((self.led_pubs[usv_id], command_str))
            if usv_id in self.sound_pubs:
                self.publish_queue.put((self.sound_pubs[usv_id], command_str))

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
        super().destroy_node()