"""
USV 发现处理模块

负责处理 USV 的动态发现和注册：
- 通过 PX4 话题自动发现 USV
- 注册和注销 USV
- 管理 USV 在线状态
"""

from rcl_interfaces.msg import Log
from common_utils import ThreadSafeDict


class DiscoveryHandler:
    """USV 发现处理器类"""
    
    def __init__(self, node, usv_manager, ros_signal):
        """
        初始化发现处理器
        
        Args:
            node: ROS 节点实例
            usv_manager: USV 管理器实例
            ros_signal: ROS 信号对象
        """
        self.node = node
        self.usv_manager = usv_manager
        self.ros_signal = ros_signal
        self.logger = node.get_logger()
        
        # 发现状态
        self._discovered_usv_list = []
        self._ns_last_seen = ThreadSafeDict()
        self._usv_states = ThreadSafeDict()
        
        # rosout 订阅列表
        self._usv_rosout_subs = []
        
        # 离线检测阈值
        self.offline_threshold = 10.0  # 秒
    
    def discover_usvs(self):
        """
        动态发现新的 USV
        
        通过检测 `/usv_xx/fmu/out/vehicle_status` 话题来发现新上线的 USV
        """
        try:
            # 获取当前所有话题
            topic_names_and_types = self.node.get_topic_names_and_types()
            
            # 筛选出 USV 的 vehicle_status 话题
            discovered_usvs = set()
            for topic_name, _ in topic_names_and_types:
                if '/fmu/out/vehicle_status' in topic_name:
                    # 提取命名空间
                    parts = topic_name.split('/')
                    if len(parts) >= 2 and parts[1].startswith('usv_'):
                        usv_id = parts[1]
                        discovered_usvs.add(usv_id)
            
            # 获取已注册的 USV 列表
            registered_usvs = set(self._discovered_usv_list)
            
            # 发现新的 USV
            new_usvs = discovered_usvs - registered_usvs
            
            for usv_id in new_usvs:
                self.logger.info(f"🔍 发现新 USV: {usv_id}")
                self._register_usv(usv_id)
                
        except Exception as e:
            self.logger.error(f"动态发现 USV 失败: {e}")
    
    def _register_usv(self, usv_id: str):
        """
        注册新发现的 USV
        
        Args:
            usv_id: USV 标识符（不带斜杠），如 'usv_01'
        """
        try:
            if usv_id in self._discovered_usv_list:
                return
            
            # 添加到已发现列表
            self._discovered_usv_list.append(usv_id)
            
            # 添加命名空间
            ns = f"/{usv_id}"
            self.usv_manager.add_usv_namespace(ns)
            
            # 记录发现时间
            now_sec = self._now_seconds()
            self._ns_last_seen[usv_id] = now_sec
            
            # 初始化状态
            self._usv_states[usv_id] = {
                'namespace': usv_id,
                'connected': True,
                'mode': 'UNKNOWN',
                'armed': False,
            }
            
            # 订阅该 USV 的 rosout
            self._subscribe_usv_rosout(usv_id)
            
            self.logger.info(f"✓ {usv_id} 注册完成（动态发现）")
            
            # 通知 GUI 更新
            self._emit_state_update()
                
        except Exception as e:
            self.logger.error(f"✗ 注册 USV {usv_id} 失败: {e}")
    
    def _subscribe_usv_rosout(self, usv_id: str):
        """订阅 USV 的 rosout 话题"""
        topic = f"/{usv_id}/rosout"
        self.logger.info(f"  ├─ 订阅远程日志: {topic}")
        sub = self.node.create_subscription(
            Log,
            topic,
            lambda msg, uid=usv_id: self._rosout_callback(msg, uid),
            10
        )
        self._usv_rosout_subs.append(sub)
    
    def _rosout_callback(self, msg, usv_id):
        """处理 USV rosout 消息"""
        # 更新最后见到时间
        self._ns_last_seen[usv_id] = self._now_seconds()
    
    def unregister_usv(self, usv_id: str):
        """
        移除离线的 USV
        
        Args:
            usv_id: USV 标识符
        """
        try:
            if usv_id not in self._discovered_usv_list:
                return
            
            self._discovered_usv_list.remove(usv_id)
            
            # 从状态中移除
            if usv_id in self._usv_states:
                del self._usv_states[usv_id]
            
            # 从 usv_manager 移除
            ns = f"/{usv_id}"
            if hasattr(self.usv_manager, 'remove_usv_namespace'):
                self.usv_manager.remove_usv_namespace(ns)
            
            self.logger.info(f"✗ {usv_id} 已移除（长时间离线）")
            
            # 通知 GUI 更新
            self._emit_state_update()
                
        except Exception as e:
            self.logger.error(f"移除 USV {usv_id} 失败: {e}")
    
    def check_availability(self):
        """
        检查 USV 话题可用性，更新连接状态
        """
        if not self._discovered_usv_list:
            return
        
        now_sec = self._now_seconds()
        state_changed = False
        
        for usv_id in self._discovered_usv_list:
            last_seen = self._ns_last_seen.get(usv_id, 0.0)
            elapsed = now_sec - last_seen
            
            # 确保状态条目存在
            if usv_id not in self._usv_states:
                self._usv_states[usv_id] = {
                    'namespace': usv_id,
                    'connected': False,
                    'mode': 'UNKNOWN',
                    'armed': False,
                }
            
            # 更新连接状态
            if elapsed > self.offline_threshold:
                if self._usv_states[usv_id].get('connected', True):
                    self._usv_states[usv_id]['connected'] = False
                    state_changed = True
                    self.logger.warn(f"⚠️  {usv_id} 已离线（{elapsed:.1f}s未收到数据）")
            else:
                if not self._usv_states[usv_id].get('connected', False):
                    self._usv_states[usv_id]['connected'] = True
                    state_changed = True
                    self.logger.info(f"✓ {usv_id} 已上线")
        
        if state_changed:
            self._emit_state_update()
    
    def update_last_seen(self, usv_id: str):
        """
        更新 USV 最后见到时间
        
        Args:
            usv_id: USV 标识符
        """
        self._ns_last_seen[usv_id] = self._now_seconds()
    
    def get_discovered_usvs(self):
        """获取已发现的 USV 列表"""
        return list(self._discovered_usv_list)
    
    def get_usv_states(self):
        """获取所有 USV 状态"""
        return dict(self._usv_states)
    
    def get_usv_state(self, usv_id: str):
        """获取指定 USV 状态"""
        return self._usv_states.get(usv_id)
    
    def update_usv_state(self, usv_id: str, **kwargs):
        """
        更新 USV 状态
        
        Args:
            usv_id: USV 标识符
            **kwargs: 要更新的状态字段
        """
        if usv_id not in self._usv_states:
            self._usv_states[usv_id] = {
                'namespace': usv_id,
                'connected': False,
                'mode': 'UNKNOWN',
                'armed': False,
            }
        
        self._usv_states[usv_id].update(kwargs)
    
    def _emit_state_update(self):
        """通知 GUI 状态更新"""
        try:
            self.ros_signal.receive_state_list.emit(list(self._usv_states.values()))
        except Exception as e:
            self.logger.debug(f"推送状态更新失败: {e}")
    
    def _now_seconds(self):
        """获取当前时间（秒）"""
        try:
            return self.node.get_clock().now().nanoseconds / 1e9
        except Exception:
            return 0.0
