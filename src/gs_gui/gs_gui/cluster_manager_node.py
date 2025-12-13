"""
集群管理节点 - 处理 40+ 无人球状态聚合

该节点负责：
1. 动态发现和注册 USV
2. 聚合所有 USV 的状态信息
3. 分组管理和编队命令分发
4. 心跳监测与故障检测
5. 发布集群整体状态
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from typing import Dict, List
from dataclasses import dataclass
from enum import Enum
import time
import threading

# 自定义消息
from common_interfaces.msg import UsvStatus, ClusterStatus

# PX4 消息（用于直接订阅 PX4 话题）
from px4_msgs.msg import VehicleStatus, BatteryStatus


class UsvState(Enum):
    """USV 状态枚举"""
    UNKNOWN = 0
    OFFLINE = 1
    STANDBY = 2
    ARMED = 3
    IN_MISSION = 4
    ERROR = 5
    LOW_BATTERY = 6


@dataclass
class UsvInfo:
    """单个 USV 信息"""
    namespace: str
    group_id: str = 'A'
    state: UsvState = UsvState.UNKNOWN
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    heading: float = 0.0
    velocity: float = 0.0
    battery_percent: float = 0.0
    battery_voltage: float = 0.0
    last_heartbeat: float = 0.0
    armed: bool = False
    connected: bool = False
    mode: str = ""
    
    def is_alive(self, timeout: float = 5.0) -> bool:
        """检查是否在线"""
        return (time.time() - self.last_heartbeat) < timeout
    
    def update_state(self):
        """根据各项指标更新状态"""
        if not self.is_alive():
            self.state = UsvState.OFFLINE
        elif self.battery_percent < 5.0:
            self.state = UsvState.LOW_BATTERY
        elif not self.connected:
            self.state = UsvState.ERROR
        elif self.armed:
            if self.mode == 'OFFBOARD' or 'AUTO' in self.mode:
                self.state = UsvState.IN_MISSION
            else:
                self.state = UsvState.ARMED
        else:
            self.state = UsvState.STANDBY


class ClusterManagerNode(Node):
    """集群管理节点"""
    
    # 分组配置：每组 8 台
    GROUP_SIZE = 8
    GROUP_NAMES = ['A', 'B', 'C', 'D', 'E', 'F']  # 支持最多 48 台

    def __init__(self):
        super().__init__('cluster_manager_node')
        
        # =====================================================================
        # 参数配置
        # =====================================================================
        self.declare_parameter('usv_discovery_enabled', True)
        self.declare_parameter('usv_discovery_timeout', 10.0)
        self.declare_parameter('heartbeat_timeout', 5.0)
        self.declare_parameter('status_publish_rate', 2.0)
        self.declare_parameter('max_usv_count', 50)
        
        # 静态配置的 USV 列表（可选）
        self.declare_parameter('usv_namespaces', [])
        
        self.discovery_enabled = self.get_parameter('usv_discovery_enabled').value
        self.discovery_timeout = self.get_parameter('usv_discovery_timeout').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.status_rate = self.get_parameter('status_publish_rate').value
        self.max_usv_count = self.get_parameter('max_usv_count').value
        self.static_namespaces = self.get_parameter('usv_namespaces').value
        
        # =====================================================================
        # USV 注册表
        # =====================================================================
        self.usv_registry: Dict[str, UsvInfo] = {}
        self.registry_lock = threading.Lock()
        
        # 初始化静态配置的 USV
        for ns in self.static_namespaces:
            group = self._get_group_from_namespace(ns)
            self.usv_registry[ns] = UsvInfo(namespace=ns, group_id=group)
        
        # =====================================================================
        # QoS 配置
        # =====================================================================
        self.qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # =====================================================================
        # 动态订阅器存储
        # =====================================================================
        self.status_subs: Dict[str, any] = {}
        self.position_subs: Dict[str, any] = {}
        self.battery_subs: Dict[str, any] = {}
        
        # 命令发布器（动态创建）
        self.command_pubs: Dict[str, Dict[str, any]] = {}
        
        # =====================================================================
        # 发布器
        # =====================================================================
        self.cluster_status_pub = self.create_publisher(
            ClusterStatus, 'cluster_status', 10)
        
        # USV 状态聚合话题（供 GUI 使用）
        self.usv_status_array_pub = self.create_publisher(
            String, 'cluster_usv_list', 10)
        
        # =====================================================================
        # 订阅器 - 地面站命令
        # =====================================================================
        self.group_arm_sub = self.create_subscription(
            String, 'group_arm_command', self.group_arm_callback, qos_reliable)
        self.group_mode_sub = self.create_subscription(
            String, 'group_mode_command', self.group_mode_callback, qos_reliable)
        self.all_arm_sub = self.create_subscription(
            Bool, 'all_arm', self.all_arm_callback, qos_reliable)
        self.all_disarm_sub = self.create_subscription(
            Bool, 'all_disarm', self.all_disarm_callback, qos_reliable)
        
        # =====================================================================
        # 定时器
        # =====================================================================
        self.status_timer = self.create_timer(
            1.0 / self.status_rate, self.publish_cluster_status)
        
        self.heartbeat_timer = self.create_timer(
            1.0, self.check_heartbeats)
        
        if self.discovery_enabled:
            self.discovery_timer = self.create_timer(
                5.0, self.discover_usvs)
        
        # 初始化订阅
        self._setup_subscriptions()
        
        # =====================================================================
        # 日志
        # =====================================================================
        self.get_logger().info('=' * 60)
        self.get_logger().info('集群管理节点已启动')
        self.get_logger().info(f'自动发现: {"启用" if self.discovery_enabled else "禁用"}')
        self.get_logger().info(f'心跳超时: {self.heartbeat_timeout}s')
        self.get_logger().info(f'静态配置 USV: {len(self.static_namespaces)} 个')
        self.get_logger().info('=' * 60)

    def _get_group_from_namespace(self, namespace: str) -> str:
        """
        根据命名空间获取分组
        
        usv_01 ~ usv_08 -> A
        usv_09 ~ usv_16 -> B
        ...
        """
        try:
            num = int(namespace.split('_')[1])
            group_idx = (num - 1) // self.GROUP_SIZE
            if group_idx < len(self.GROUP_NAMES):
                return self.GROUP_NAMES[group_idx]
            return 'X'  # 超出范围
        except (ValueError, IndexError):
            return 'A'

    def _setup_subscriptions(self):
        """设置话题订阅"""
        for ns in self.usv_registry.keys():
            self._create_usv_subscriptions(ns)

    def _create_usv_subscriptions(self, namespace: str):
        """
        为单个 USV 创建订阅
        
        支持两种模式：
        1. 订阅 UsvStatus（usv_status_node 发布的聚合状态）
        2. 直接订阅 PX4 话题
        """
        # 方式 1：订阅聚合的 UsvStatus
        try:
            self.status_subs[namespace] = self.create_subscription(
                UsvStatus,
                f'/{namespace}/usv_state',
                lambda msg, ns=namespace: self.usv_status_callback(msg, ns),
                10
            )
        except Exception as e:
            self.get_logger().warn(f'无法订阅 {namespace}/usv_state: {e}')
        
        # 创建命令发布器
        self.command_pubs[namespace] = {
            'arm': self.create_publisher(
                Bool, f'/{namespace}/set_arm', 10),
            'mode': self.create_publisher(
                String, f'/{namespace}/set_usv_mode', 10),
            'target': self.create_publisher(
                PoseStamped, f'/{namespace}/set_usv_target_position', 10),
        }
        
        self.get_logger().info(f'✅ 已注册 USV: {namespace}')

    def usv_status_callback(self, msg: UsvStatus, namespace: str):
        """USV 状态回调"""
        with self.registry_lock:
            if namespace not in self.usv_registry:
                group = self._get_group_from_namespace(namespace)
                self.usv_registry[namespace] = UsvInfo(namespace=namespace, group_id=group)
            
            usv = self.usv_registry[namespace]
            usv.last_heartbeat = time.time()
            usv.connected = msg.connected
            usv.armed = msg.armed
            usv.mode = msg.mode
            usv.position_x = msg.pose.position.x
            usv.position_y = msg.pose.position.y
            usv.position_z = msg.pose.position.z
            usv.heading = msg.heading
            usv.velocity = msg.twist.linear.x  # 使用线速度 x 分量
            usv.battery_percent = msg.battery_percentage
            usv.battery_voltage = msg.battery_voltage
            usv.update_state()

    def discover_usvs(self):
        """
        动态发现 USV
        
        通过检测 PX4 话题（/usv_xx/fmu/out/vehicle_status）自动发现新 USV
        """
        if not self.discovery_enabled:
            return
            
        # 通过话题列表发现新的 USV
        topic_list = self.get_topic_names_and_types()
        
        for topic, types in topic_list:
            # 查找 PX4 VehicleStatus 话题（优先）或 usv_state 话题（兼容）
            is_px4_status = '/fmu/out/vehicle_status' in topic and 'px4_msgs/msg/VehicleStatus' in str(types)
            is_usv_state = '/usv_state' in topic and 'common_interfaces/msg/UsvStatus' in str(types)
            
            if is_px4_status or is_usv_state:
                # 提取命名空间
                parts = topic.split('/')
                if len(parts) >= 2:
                    ns = parts[1]
                    if ns.startswith('usv_') and ns not in self.usv_registry:
                        if len(self.usv_registry) >= self.max_usv_count:
                            self.get_logger().warn(f'已达到最大 USV 数量限制: {self.max_usv_count}')
                            return
                            
                        self.get_logger().info(f'🔍 发现新 USV: {ns}')
                        group = self._get_group_from_namespace(ns)
                        
                        with self.registry_lock:
                            self.usv_registry[ns] = UsvInfo(namespace=ns, group_id=group)
                        
                        self._create_usv_subscriptions(ns)

    def check_heartbeats(self):
        """检查心跳，标记离线 USV"""
        with self.registry_lock:
            for ns, usv in self.usv_registry.items():
                was_alive = usv.state != UsvState.OFFLINE
                usv.update_state()
                
                if was_alive and usv.state == UsvState.OFFLINE:
                    self.get_logger().warn(f'⚠️ {ns} 离线')
                elif not was_alive and usv.state != UsvState.OFFLINE:
                    self.get_logger().info(f'✅ {ns} 恢复在线')

    def publish_cluster_status(self):
        """发布集群状态"""
        msg = ClusterStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        with self.registry_lock:
            msg.total_count = len(self.usv_registry)
            msg.online_count = sum(
                1 for usv in self.usv_registry.values() 
                if usv.is_alive(self.heartbeat_timeout)
            )
            msg.armed_count = sum(
                1 for usv in self.usv_registry.values() if usv.armed
            )
            msg.in_mission_count = sum(
                1 for usv in self.usv_registry.values()
                if usv.state == UsvState.IN_MISSION
            )
            msg.low_battery_count = sum(
                1 for usv in self.usv_registry.values()
                if usv.state == UsvState.LOW_BATTERY
            )
            msg.error_count = sum(
                1 for usv in self.usv_registry.values()
                if usv.state in [UsvState.ERROR, UsvState.OFFLINE]
            )
            
            # 分组统计
            groups: Dict[str, List[str]] = {}
            for ns, usv in self.usv_registry.items():
                if usv.group_id not in groups:
                    groups[usv.group_id] = []
                groups[usv.group_id].append(ns)
            
            msg.group_ids = list(groups.keys())
            msg.group_counts = [len(v) for v in groups.values()]
            
            # 告警信息
            warnings = []
            for ns, usv in self.usv_registry.items():
                if usv.state == UsvState.LOW_BATTERY:
                    warnings.append(f'{ns}: 低电量 ({usv.battery_percent:.0f}%)')
                elif usv.state == UsvState.OFFLINE:
                    warnings.append(f'{ns}: 离线')
                elif usv.state == UsvState.ERROR:
                    warnings.append(f'{ns}: 通信异常')
            
            msg.warnings = warnings
        
        self.cluster_status_pub.publish(msg)
        
        # 发布 USV 列表（供 GUI 使用）
        usv_list_msg = String()
        usv_list_msg.data = ','.join(self.usv_registry.keys())
        self.usv_status_array_pub.publish(usv_list_msg)

    # =========================================================================
    # 编队命令处理
    # =========================================================================
    
    def group_arm_callback(self, msg: String):
        """
        分组解锁命令
        
        格式: 'A:true' 或 'all:false'
        """
        try:
            group_id, arm_str = msg.data.split(':')
            arm_value = arm_str.lower() == 'true'
            
            with self.registry_lock:
                targets = self._get_group_members(group_id)
            
            for ns in targets:
                if ns in self.command_pubs:
                    arm_msg = Bool()
                    arm_msg.data = arm_value
                    self.command_pubs[ns]['arm'].publish(arm_msg)
            
            action = "解锁" if arm_value else "上锁"
            self.get_logger().info(f'📤 分组 {group_id} {action}: {len(targets)} 个 USV')
            
        except ValueError:
            self.get_logger().error(f'无效命令格式: {msg.data}，应为 "GROUP:true/false"')

    def group_mode_callback(self, msg: String):
        """
        分组模式切换命令
        
        格式: 'A:OFFBOARD' 或 'all:AUTO.LOITER'
        """
        try:
            group_id, mode = msg.data.split(':')
            
            with self.registry_lock:
                targets = self._get_group_members(group_id)
            
            for ns in targets:
                if ns in self.command_pubs:
                    mode_msg = String()
                    mode_msg.data = mode
                    self.command_pubs[ns]['mode'].publish(mode_msg)
            
            self.get_logger().info(f'📤 分组 {group_id} 切换到 {mode}: {len(targets)} 个 USV')
            
        except ValueError:
            self.get_logger().error(f'无效命令格式: {msg.data}，应为 "GROUP:MODE"')

    def all_arm_callback(self, msg: Bool):
        """全部解锁"""
        if msg.data:
            with self.registry_lock:
                for ns in self.usv_registry.keys():
                    if ns in self.command_pubs:
                        arm_msg = Bool()
                        arm_msg.data = True
                        self.command_pubs[ns]['arm'].publish(arm_msg)
            
            self.get_logger().info(f'📤 全部解锁: {len(self.usv_registry)} 个 USV')

    def all_disarm_callback(self, msg: Bool):
        """全部上锁"""
        if msg.data:
            with self.registry_lock:
                for ns in self.usv_registry.keys():
                    if ns in self.command_pubs:
                        arm_msg = Bool()
                        arm_msg.data = False
                        self.command_pubs[ns]['arm'].publish(arm_msg)
            
            self.get_logger().info(f'📤 全部上锁: {len(self.usv_registry)} 个 USV')

    def _get_group_members(self, group_id: str) -> List[str]:
        """获取分组成员"""
        group_id = group_id.upper()
        if group_id == 'ALL':
            return list(self.usv_registry.keys())
        return [
            ns for ns, usv in self.usv_registry.items() 
            if usv.group_id == group_id
        ]

    def send_target_to_group(self, group_id: str, target: PoseStamped, offset_mode: str = 'none'):
        """
        向分组发送目标点
        
        Args:
            group_id: 分组 ID
            target: 目标位置
            offset_mode: 偏移模式 ('none', 'line', 'grid')
        """
        with self.registry_lock:
            members = self._get_group_members(group_id)
        
        for i, ns in enumerate(members):
            if ns not in self.command_pubs:
                continue
                
            target_msg = PoseStamped()
            target_msg.header = target.header
            target_msg.pose = target.pose
            
            # 根据偏移模式调整位置
            if offset_mode == 'line':
                # 一字排列，间隔 3 米
                target_msg.pose.position.y += i * 3.0
            elif offset_mode == 'grid':
                # 网格排列
                row = i // 4
                col = i % 4
                target_msg.pose.position.x += row * 3.0
                target_msg.pose.position.y += col * 3.0
            
            self.command_pubs[ns]['target'].publish(target_msg)


def main(args=None):
    """节点主函数"""
    rclpy.init(args=args)
    node = ClusterManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
