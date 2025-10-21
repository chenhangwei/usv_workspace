"""
USV管理模块
处理USV的发现、状态管理和通信
"""

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from common_interfaces.msg import UsvStatus
from std_msgs.msg import String
from common_interfaces.action import NavigateToPoint
from rclpy.action import ActionClient


class UsvManager:
    def __init__(self, node):
        self.node = node
        # USV通信相关的发布者和订阅者
        self.usv_state_subs = {}
        self.set_usv_mode_pubs = {}
        self.set_usv_arming_pubs = {}
        self.led_pubs = {}
        self.sound_pubs = {}
        self.action_pubs = {}
        self.navigate_to_point_clients = {}
        self.qos_a = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

    # 添加USV命名空间
    def add_usv_namespace(self, ns):
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
        self.usv_state_subs[usv_id] = self.node.create_subscription(
            UsvStatus, topic_state, lambda msg, id=usv_id: self.usv_state_callback(msg, id), self.qos_a)
        # 创建设置USV模式发布者
        self.set_usv_mode_pubs[usv_id] = self.node.create_publisher(
            String, topic_mode, self.qos_a)
        # 创建设置USV武装状态发布者
        self.set_usv_arming_pubs[usv_id] = self.node.create_publisher(
            String, topic_arming, self.qos_a)
        # 创建LED控制发布者
        self.led_pubs[usv_id] = self.node.create_publisher(
            String, topic_led, self.qos_a)
        # 创建声音控制发布者
        self.sound_pubs[usv_id] = self.node.create_publisher(
            String, topic_sound, self.qos_a)
        # 创建动作控制发布者
        self.action_pubs[usv_id] = self.node.create_publisher(
            String, topic_action, self.qos_a)
        # 为每个USV创建一个NavigateToPoint动作客户端
        self.navigate_to_point_clients[usv_id] = ActionClient(self.node, NavigateToPoint, action_server_name)
        # 记录日志信息
        self.node.get_logger().info(f"为USV {usv_id} 添加订阅者和发布者")

    # 移除USV命名空间
    def remove_usv_namespace(self, ns):
        """
        移除指定的USV命名空间的订阅者和发布者
        
        Args:
            ns (str): USV的命名空间
        """
        # 从命名空间中提取USV ID
        usv_id = ns.lstrip('/')
        # 销毁并删除USV状态订阅者
        if usv_id in self.usv_state_subs:
            self.node.destroy_subscription(self.usv_state_subs[usv_id])
            del self.usv_state_subs[usv_id]
        # 销毁并删除设置USV模式发布者
        if usv_id in self.set_usv_mode_pubs:
            self.node.destroy_publisher(self.set_usv_mode_pubs[usv_id])
            del self.set_usv_mode_pubs[usv_id]
        # 销毁并删除设置USV武装状态发布者
        if usv_id in self.set_usv_arming_pubs:
            self.node.destroy_publisher(self.set_usv_arming_pubs[usv_id])
            del self.set_usv_arming_pubs[usv_id]
        # 销毁并删除LED控制发布者
        if usv_id in self.led_pubs:
            self.node.destroy_publisher(self.led_pubs[usv_id])
            del self.led_pubs[usv_id]
        # 销毁并删除声音控制发布者
        if usv_id in self.sound_pubs:
            self.node.destroy_publisher(self.sound_pubs[usv_id])
            del self.sound_pubs[usv_id]
        # 销毁并删除动作控制发布者
        if usv_id in self.action_pubs:
            self.node.destroy_publisher(self.action_pubs[usv_id])
            del self.action_pubs[usv_id]
        # 移除对应的导航动作客户端
        if usv_id in self.navigate_to_point_clients:
            del self.navigate_to_point_clients[usv_id]
        # 记录日志信息
        self.node.get_logger().info(f"移除USV {usv_id} 的订阅者和发布者")

        # 额外：从节点的 usv_states 中移除该 USV，并通知 GUI 刷新列表
        try:
            # 清理状态与辅助记录
            if usv_id in getattr(self.node, 'usv_states', {}):
                del self.node.usv_states[usv_id]
            if hasattr(self.node, 'usv_boot_pose'):
                self.node.usv_boot_pose.pop(usv_id, None)
            # 若 GUI 信号可用，发射最新状态列表，触发表格行删除
            if hasattr(self.node, 'ros_signal') and getattr(self.node.ros_signal, 'receive_state_list', None) is not None:
                self.node.ros_signal.receive_state_list.emit(list(self.node.usv_states.values()))
        except Exception as e:
            self.node.get_logger().warn(f"清理 {usv_id} 状态或通知 GUI 失败: {e}")

    # USV状态回调
    def usv_state_callback(self, msg, usv_id):
        """
        处理USV状态更新回调
        
        Args:
            msg (UsvStatus): USV状态消息
            usv_id (str): USV标识符
        """
        # 检查消息类型是否正确
        if isinstance(msg, UsvStatus):
            # 更新最后一次收到状态的时间戳，供离线判定逻辑使用
            try:
                now_sec = self.node.get_clock().now().nanoseconds / 1e9
            except Exception:
                now_sec = 0.0
            self.node._ns_last_seen[usv_id] = now_sec
            # 构造状态数据字典
            state_data = {
                'namespace': usv_id,  # 命名空间
                'mode': msg.mode,  # 当前模式
                'connected': msg.connected,  # 连接状态
                'armed': msg.armed,  # 武装状态
                'guided': msg.guided,  # 引导模式状态
                'battery_voltage': msg.battery_voltage,  # 电池电压
                'battery_percentage': msg.battery_percentage,  # 电池电量百分比
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
            first_time = usv_id not in self.node.usv_states
            if first_time or self.node.usv_states.get(usv_id) != state_data:
                # 更新USV状态字典
                self.node.usv_states[usv_id] = state_data
                # 如果是首次收到该USV的状态，记录为 boot pose（上电时的原点）
                if first_time:
                    try:
                        bp = state_data.get('position', {})
                        byaw = float(state_data.get('yaw', 0.0))
                        self.node.usv_boot_pose[usv_id] = {'x': float(bp.get('x', 0.0)), 'y': float(bp.get('y', 0.0)), 'z': float(bp.get('z', 0.0)), 'yaw': byaw}
                        self.node.get_logger().info(f"记录 USV {usv_id} 的 boot_pose: {self.node.usv_boot_pose[usv_id]}")
                    except Exception:
                        pass
                # 发射信号，将更新后的USV状态列表发送给GUI界面
                # 限制信号发射频率，避免过于频繁的更新
                self.node.ros_signal.receive_state_list.emit(list(self.node.usv_states.values()))