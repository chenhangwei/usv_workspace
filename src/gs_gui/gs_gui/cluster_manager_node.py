"""
集群管理节点。

负责聚合网络中所有 USV 的状态，进行逻辑分组，并分发编队控制命令。
该节点是地面站与多艇集群之间的核心逻辑桥梁。
"""

import time
import threading
import logging
from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from common_interfaces.msg import UsvStatus, ClusterStatus
from common_utils import ParamLoader, ParamValidator

# 模块级日志记录器
_logger = logging.getLogger("gs_gui.cluster_manager")


class UsvState(Enum):
    """USV 逻辑状态枚举。"""
    UNKNOWN = 0
    OFFLINE = 1
    STANDBY = 2
    ARMED = 3
    IN_MISSION = 4
    ERROR = 5
    LOW_BATTERY = 6


@dataclass
class UsvInfo:
    """单个 USV 的聚合信息。"""
    namespace: str
    group_id: str = 'A'
    state: UsvState = UsvState.UNKNOWN
    position: Dict[str, float] = None
    heading: float = 0.0
    velocity: float = 0.0
    battery_percent: float = 0.0
    last_heartbeat: float = 0.0
    armed: bool = False
    connected: bool = False
    mode: str = ""
    
    def __post_init__(self):
        if self.position is None:
            self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def is_alive(self, timeout: float = 5.0) -> bool:
        """检查设备是否在线。"""
        return (time.time() - self.last_heartbeat) < timeout
    
    def update_logic_state(self):
        """根据物理指标更新逻辑状态。"""
        if not self.is_alive():
            self.state = UsvState.OFFLINE
        elif self.battery_percent < 10.0:
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
    """集群管理 ROS 2 节点。"""
    
    GROUP_SIZE = 8
    GROUP_NAMES = ['A', 'B', 'C', 'D', 'E', 'F']

    def __init__(self):
        super().__init__('cluster_manager_node')
        
        # 1. 参数加载
        loader = ParamLoader(self)
        self.discovery_enabled = loader.load_param('usv_discovery_enabled', True)
        self.heartbeat_timeout = loader.load_param('heartbeat_timeout', 5.0)
        self.status_rate = loader.load_param('status_publish_rate', 2.0, ParamValidator.frequency)
        self.max_usv_count = loader.load_param('max_usv_count', 50)
        self.static_namespaces = loader.load_param('usv_namespaces', [])
        
        # 2. 状态注册表
        self.usv_registry: Dict[str, UsvInfo] = {}
        self.registry_lock = threading.Lock()
        
        # 3. 话题管理
        self.status_subs: Dict[str, Any] = {}
        self.command_pubs: Dict[str, Dict[str, Any]] = {}
        
        # 4. 发布器
        self.cluster_status_pub = self.create_publisher(ClusterStatus, 'cluster_status', 10)
        self.usv_list_pub = self.create_publisher(String, 'cluster_usv_list', 10)
        
        # 5. 订阅器 (地面站全局命令)
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'group_arm_command', self.group_arm_callback, qos_reliable)
        self.create_subscription(String, 'group_mode_command', self.group_mode_callback, qos_reliable)
        self.create_subscription(Bool, 'all_arm', self.all_arm_callback, qos_reliable)
        self.create_subscription(Bool, 'all_disarm', self.all_disarm_callback, qos_reliable)
        
        # 6. 定时器
        self.create_timer(1.0 / self.status_rate, self.publish_cluster_status)
        self.create_timer(1.0, self.check_heartbeats)
        if self.discovery_enabled:
            self.create_timer(5.0, self.discover_usvs)
            
        # 初始化静态配置
        for ns in self.static_namespaces:
            self._register_usv(ns)
            
        self.get_logger().info(f"🚀 集群管理节点启动，已注册 {len(self.static_namespaces)} 个静态设备")

    def _get_group_id(self, ns: str) -> str:
        """根据命名空间分配分组。"""
        try:
            num = int(ns.split('_')[1])
            idx = (num - 1) // self.GROUP_SIZE
            return self.GROUP_NAMES[idx] if idx < len(self.GROUP_NAMES) else 'X'
        except Exception: return 'A'

    def _register_usv(self, ns: str):
        """注册新设备并建立订阅。"""
        with self.registry_lock:
            if ns in self.usv_registry: return
            
            self.usv_registry[ns] = UsvInfo(namespace=ns, group_id=self._get_group_id(ns))
            
            # 订阅状态
            self.status_subs[ns] = self.create_subscription(
                UsvStatus, f'/{ns}/usv_state',
                lambda msg, n=ns: self.usv_status_callback(msg, n), 10
            )
            
            # 建立命令发布器
            self.command_pubs[ns] = {
                'arm': self.create_publisher(Bool, f'/{ns}/set_arm', 10),
                'mode': self.create_publisher(String, f'/{ns}/set_usv_mode', 10),
                'target': self.create_publisher(PoseStamped, f'/{ns}/set_usv_target_position', 10),
            }
            self.get_logger().info(f"✅ 已注册设备: {ns}")

    def usv_status_callback(self, msg: UsvStatus, ns: str):
        """处理单艇状态更新。"""
        with self.registry_lock:
            if ns not in self.usv_registry: return
            usv = self.usv_registry[ns]
            usv.last_heartbeat = time.time()
            usv.connected = msg.connected
            usv.armed = msg.armed
            usv.mode = msg.mode
            usv.position = {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z}
            usv.heading = msg.heading
            usv.velocity = msg.twist.linear.x
            usv.battery_percent = msg.battery_percentage
            usv.update_logic_state()

    def discover_usvs(self):
        """动态发现新设备。"""
        topics = self.get_topic_names_and_types()
        for topic, _ in topics:
            if topic.endswith('/usv_state'):
                parts = topic.split('/')
                if len(parts) >= 2 and parts[1].startswith('usv_'):
                    ns = parts[1]
                    if ns not in self.usv_registry and len(self.usv_registry) < self.max_usv_count:
                        self.get_logger().info(f"🔍 发现新设备: {ns}")
                        self._register_usv(ns)

    def check_heartbeats(self):
        """心跳检测。"""
        with self.registry_lock:
            for ns, usv in self.usv_registry.items():
                usv.update_logic_state()

    def publish_cluster_status(self):
        """发布集群整体状态。"""
        msg = ClusterStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        with self.registry_lock:
            msg.total_count = len(self.usv_registry)
            msg.online_count = sum(1 for u in self.usv_registry.values() if u.is_alive(self.heartbeat_timeout))
            msg.armed_count = sum(1 for u in self.usv_registry.values() if u.armed)
            msg.in_mission_count = sum(1 for u in self.usv_registry.values() if u.state == UsvState.IN_MISSION)
            msg.low_battery_count = sum(1 for u in self.usv_registry.values() if u.state == UsvState.LOW_BATTERY)
            msg.error_count = sum(1 for u in self.usv_registry.values() if u.state in [UsvState.ERROR, UsvState.OFFLINE])
            
            # 分组统计
            groups = {}
            for u in self.usv_registry.values():
                groups[u.group_id] = groups.get(u.group_id, 0) + 1
            msg.group_ids = list(groups.keys())
            msg.group_counts = list(groups.values())
            
            # 警告汇总
            msg.warnings = [f"{ns}: 离线" for ns, u in self.usv_registry.items() if u.state == UsvState.OFFLINE]
            
        self.cluster_status_pub.publish(msg)
        
        # 发布 ID 列表
        list_msg = String()
        list_msg.data = ','.join(self.usv_registry.keys())
        self.usv_list_pub.publish(list_msg)

    # --- 命令回调 ---
    def group_arm_callback(self, msg: String):
        """处理分组解锁命令。格式 'GROUP:true/false'"""
        try:
            gid, arm_str = msg.data.split(':')
            arm = arm_str.lower() == 'true'
            targets = self._get_group_members(gid)
            for ns in targets:
                if ns in self.command_pubs:
                    self.command_pubs[ns]['arm'].publish(Bool(data=arm))
        except Exception as e: self.get_logger().error(f"Group arm error: {e}")

    def group_mode_callback(self, msg: String):
        """处理分组模式切换。格式 'GROUP:MODE'"""
        try:
            gid, mode = msg.data.split(':')
            targets = self._get_group_members(gid)
            for ns in targets:
                if ns in self.command_pubs:
                    self.command_pubs[ns]['mode'].publish(String(data=mode))
        except Exception as e: self.get_logger().error(f"Group mode error: {e}")

    def all_arm_callback(self, msg: Bool):
        """全群解锁。"""
        if msg.data:
            for pubs in self.command_pubs.values(): pubs['arm'].publish(Bool(data=True))

    def all_disarm_callback(self, msg: Bool):
        """全群上锁。"""
        if msg.data:
            for pubs in self.command_pubs.values(): pubs['arm'].publish(Bool(data=False))

    def _get_group_members(self, gid: str) -> List[str]:
        """获取分组成员。"""
        gid = gid.upper()
        if gid == 'ALL': return list(self.usv_registry.keys())
        return [ns for ns, u in self.usv_registry.items() if u.group_id == gid]


def main(args=None):
    rclpy.init(args=args)
    node = ClusterManagerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
