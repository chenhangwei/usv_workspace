"""
USV 发现处理模块

负责处理 USV 的动态发现和注册：
- 通过 PX4 话题自动发现 USV
- 注册和注销 USV
- 管理 USV 在线状态
"""

import json
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from common_interfaces.msg import UsvStatus
from common_utils import ThreadSafeDict

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from common_interfaces.msg import NavigationFeedback, NavigationResult, NavigationAck


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
        
        # 传感器处理器引用（用于处理飞控消息）
        self._sensor_handler = None
        
        # 上一条飞控消息缓存（用于去重）
        self._last_status_text_cache = ThreadSafeDict()
        
        # rosout 订阅列表
        self._usv_rosout_subs = []
        
        # usv_state 订阅列表（用于更新在线状态）
        self._usv_state_subs = []

        # 导航话题订阅（每艇一组）
        self._nav_feedback_subs = {}
        self._nav_result_subs = {}
        self._nav_ack_subs = {}
        self._telemetry_status_subs = {}
        self._flight_state_subs = {}
        
        # 预探测订阅列表（用于触发 Zenoh 桥接）
        self._probe_subs = []
        
        # 离线检测阈值
        self.offline_threshold = 10.0  # 秒
    
    def set_sensor_handler(self, sensor_handler):
        """
        设置传感器处理器引用
        
        Args:
            sensor_handler: SensorStatusHandler 实例，用于处理飞控消息
        """
        self._sensor_handler = sensor_handler
        self.logger.debug("已设置 sensor_handler 引用")
    
    def probe_remote_usvs(self, usv_ids: list):
        """
        预探测远程 USV（用于触发 Zenoh 桥接的 interest-based routing）
        
        Zenoh Bridge v1.7+ 使用 interest-based routing，只有当本地有订阅者时
        才会从远端桥接数据。此方法预先创建订阅者来触发 Zenoh 数据桥接。
        
        Args:
            usv_ids: USV 标识符列表，如 ['usv_01', 'usv_02']
        """
        self.logger.info(f"🔍 预探测远程 USV: {usv_ids}")
        
        for usv_id in usv_ids:
            if usv_id in self._discovered_usv_list:
                continue
                
            topic = f"/{usv_id}/usv_state"
            try:
                sub = self.node.create_subscription(
                    UsvStatus,
                    topic,
                    lambda msg, uid=usv_id: self._probe_callback(msg, uid),
                    10
                )
                self._probe_subs.append((usv_id, sub))
                self.logger.debug(f"  ├─ 探测话题: {topic}")
            except Exception as e:
                self.logger.warning(f"  ├─ 探测话题 {topic} 失败: {e}")
    
    def _probe_callback(self, msg: UsvStatus, usv_id: str):
        """
        处理探测消息回调
        
        当收到远程 USV 的消息时，触发正式注册流程
        """
        # 检查是否已经有探测订阅
        has_probe_sub = any(uid == usv_id for uid, _ in self._probe_subs)
        
        # 如果没有探测订阅（说明已经转为正式订阅），直接返回
        if not has_probe_sub:
            return
            
        self.logger.info(f"📡 探测到远程 USV: {usv_id}")
        
        # 移除探测订阅（无论是否已注册都要移除）
        self._remove_probe_subscription(usv_id)
        
        # 如果尚未注册，进行注册
        if usv_id not in self._discovered_usv_list:
            self._register_usv(usv_id)
    
    def _remove_probe_subscription(self, usv_id: str):
        """移除指定 USV 的探测订阅"""
        for item in self._probe_subs[:]:  # 创建副本遍历
            uid, sub = item
            if uid == usv_id:
                try:
                    self.node.destroy_subscription(sub)
                    self._probe_subs.remove(item)
                    self.logger.debug(f"  └─ 移除探测订阅: {usv_id}")
                except Exception as e:
                    self.logger.warning(f"移除探测订阅失败: {e}")
                break

    def discover_usvs(self):
        """
        动态发现新的 USV
        
        通过检测以下话题来发现新上线的 USV:
        - `/usv_xx/fmu/out/vehicle_status` (本地 PX4)
        - `/usv_xx/usv_state` (通过 Zenoh 桥接的远程 USV)
        """
        try:
            # 获取当前所有话题
            topic_names_and_types = self.node.get_topic_names_and_types()
            
            # 筛选出 USV 话题（支持 vehicle_status 和 usv_state）
            discovered_usvs = set()
            for topic_name, _ in topic_names_and_types:
                # 匹配 /usv_xx/fmu/out/vehicle_status 或 /usv_xx/usv_state
                if '/fmu/out/vehicle_status' in topic_name or topic_name.endswith('/usv_state'):
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
            
            # 订阅该 USV 的 rosout（本地 USV）
            self._subscribe_usv_rosout(usv_id)
            
            # 订阅该 USV 的 usv_state（通过 Zenoh 桥接的远程 USV）
            self._subscribe_usv_state(usv_id)

            # 订阅异构机型元数据（vehicle_type / platform_mode）与起降保护状态
            self._subscribe_telemetry_status(usv_id)
            self._subscribe_flight_state(usv_id)

            # 订阅导航反馈/结果/应答（用于任务推进与停止 step_timeout 重发）
            self._subscribe_navigation_topics(usv_id)
            
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
    
    def _subscribe_usv_state(self, usv_id: str):
        """订阅 USV 的 usv_state 话题（用于远程 USV 的在线检测）"""
        topic = f"/{usv_id}/usv_state"
        self.logger.info(f"  ├─ 订阅状态话题: {topic}")
        sub = self.node.create_subscription(
            UsvStatus,
            topic,
            lambda msg, uid=usv_id: self._usv_state_callback(msg, uid),
            10
        )
        self._usv_state_subs.append(sub)

    def _subscribe_telemetry_status(self, usv_id: str):
        """订阅 USV 的 telemetry/status 话题，动态同步机型与 2d/3d 平台模式"""
        if usv_id in self._telemetry_status_subs:
            return
        topic = f"/{usv_id}/telemetry/status"
        sub = self.node.create_subscription(
            String,
            topic,
            lambda msg, uid=usv_id: self._telemetry_status_callback(msg, uid),
            10
        )
        self._telemetry_status_subs[usv_id] = sub

    def _subscribe_flight_state(self, usv_id: str):
        """订阅 USV 的 flight_state 话题，实时同步 3D 起飞/降落与电机保护锁止状态"""
        if usv_id in self._flight_state_subs:
            return
        topic = f"/{usv_id}/flight_state"
        sub = self.node.create_subscription(
            String,
            topic,
            lambda msg, uid=usv_id: self._flight_state_callback(msg, uid),
            10
        )
        self._flight_state_subs[usv_id] = sub

    def _apply_flight_meta(self, usv_id: str, data: dict):
        """将机载上报的起降高度、动作与电机保护锁止标记同步到地面站缓存"""
        if not isinstance(data, dict):
            return
        st = self._usv_states.get(usv_id)
        if 'takeoff_altitude' in data and data['takeoff_altitude'] is not None:
            try:
                ta = float(data['takeoff_altitude'])
                if ta >= 0.0:
                    if hasattr(self.node, '_usv_default_altitude'):
                        self.node._usv_default_altitude[usv_id] = ta
                    if isinstance(st, dict):
                        st['takeoff_altitude'] = ta
                        st['default_altitude'] = ta
            except Exception:
                pass
        if 'landing_altitude' in data and data['landing_altitude'] is not None:
            try:
                la = float(data['landing_altitude'])
                if la >= 0.0:
                    if hasattr(self.node, '_usv_landing_altitude'):
                        self.node._usv_landing_altitude[usv_id] = la
                    if isinstance(st, dict):
                        st['landing_altitude'] = la
            except Exception:
                pass
        if 'protection_locked' in data and isinstance(st, dict):
            prev_lock = bool(st.get('protection_locked', False))
            now_lock = bool(data['protection_locked'])
            st['protection_locked'] = now_lock
            if now_lock and not prev_lock:
                try:
                    self.ros_signal.nav_status_update.emit(usv_id, "🔒降落锁桨保护")
                except Exception:
                    pass
        if 'flight_action' in data and isinstance(st, dict):
            fa = str(data.get('flight_action') or 'idle')
            prev_fa = str(st.get('flight_action') or 'idle')
            st['flight_action'] = fa
            if fa != prev_fa:
                try:
                    if fa == 'takeoff':
                        self.ros_signal.nav_status_update.emit(usv_id, "🛫正在起飞")
                    elif fa == 'land':
                        self.ros_signal.nav_status_update.emit(usv_id, "🛬正在降落")
                    elif prev_fa == 'takeoff' and bool(data.get('takeoff_ready', False)):
                        self.ros_signal.nav_status_update.emit(usv_id, "✅起飞到位")
                except Exception:
                    pass
        if 'takeoff_ready' in data and isinstance(st, dict):
            st['takeoff_ready'] = bool(data['takeoff_ready'])

    def _flight_state_callback(self, msg: String, usv_id: str):
        """解析 flight_state JSON"""
        try:
            data = json.loads(msg.data) if msg and msg.data else {}
            self._apply_flight_meta(usv_id, data)
        except Exception:
            pass

    def _telemetry_status_callback(self, msg: String, usv_id: str):
        """解析 telemetry/status JSON 并更新机型/平台模式与起降保护状态"""
        try:
            data = json.loads(msg.data) if msg and msg.data else {}
            vt = str(data.get('vehicle_type', '')).strip().lower()
            pm = str(data.get('platform_mode', '')).strip().lower()
            if vt:
                if hasattr(self.node, '_usv_vehicle_type'):
                    self.node._usv_vehicle_type[usv_id] = vt
                if usv_id in self._usv_states:
                    self._usv_states[usv_id]['vehicle_type'] = vt
            if pm in ('2d', '3d'):
                if hasattr(self.node, '_usv_platform_mode'):
                    self.node._usv_platform_mode[usv_id] = pm
                if usv_id in self._usv_states:
                    self._usv_states[usv_id]['platform_mode'] = pm
            self._apply_flight_meta(usv_id, data)
        except Exception:
            pass

    def _subscribe_navigation_topics(self, usv_id: str) -> None:
        """订阅 USV 的导航话题：feedback/result/ack。"""
        try:
            qos = getattr(self.node, 'qos_a', None)
            if qos is None:
                qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

            feedback_topic = f"/{usv_id}/navigation/feedback"
            result_topic = f"/{usv_id}/navigation/result"
            ack_topic = f"/{usv_id}/navigation/ack"

            if usv_id not in self._nav_feedback_subs:
                self._nav_feedback_subs[usv_id] = self.node.create_subscription(
                    NavigationFeedback,
                    feedback_topic,
                    lambda msg, uid=usv_id: self.node.navigation_feedback_callback(msg, uid),
                    qos,
                )
            if usv_id not in self._nav_result_subs:
                self._nav_result_subs[usv_id] = self.node.create_subscription(
                    NavigationResult,
                    result_topic,
                    lambda msg, uid=usv_id: self.node.navigation_result_callback(msg, uid),
                    qos,
                )
            if usv_id not in self._nav_ack_subs:
                self._nav_ack_subs[usv_id] = self.node.create_subscription(
                    NavigationAck,
                    ack_topic,
                    lambda msg, uid=usv_id: self.node.navigation_ack_callback(msg, uid),
                    qos,
                )

            self.logger.info(f"  ├─ 订阅导航话题: {feedback_topic}, {result_topic}, {ack_topic}")
        except Exception as e:
            self.logger.warning(f"订阅 {usv_id} 导航话题失败: {e}")
    
    def _usv_state_callback(self, msg: UsvStatus, usv_id: str):
        """处理 USV usv_state 消息，更新在线状态和状态信息"""
        # 更新最后见到时间
        self._ns_last_seen[usv_id] = self._now_seconds()
        
        # 更新状态信息
        if usv_id not in self._usv_states:
            self._usv_states[usv_id] = {
                'namespace': usv_id,
                'connected': True,  # 网络连接状态（收到数据即为在线）
                'mode': 'UNKNOWN',
                'armed': False,
            }
        
        # 从 UsvStatus 消息更新状态
        state = self._usv_states[usv_id]
        
        # 注意：收到消息即表示网络在线，不要用 msg.connected 覆盖
        # msg.connected 表示的是飞控连接状态，保存到单独字段
        state['fc_connected'] = msg.connected  # 飞控连接状态
        state['mode'] = msg.mode if msg.mode else 'UNKNOWN'
        state['armed'] = msg.armed
        
        # 位置信息（使用 table_manager 期望的嵌套格式）
        state['position'] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        # 同时保留扁平格式（兼容其他地方使用）
        state['position_x'] = msg.pose.position.x
        state['position_y'] = msg.pose.position.y
        state['position_z'] = msg.pose.position.z
        
        # 速度信息（使用 table_manager 期望的嵌套格式）
        state['velocity'] = {
            'linear': {
                'x': msg.twist.linear.x,
                'y': msg.twist.linear.y,
                'z': msg.twist.linear.z
            }
        }
        
        # 航向和姿态
        state['heading'] = msg.heading
        state['yaw'] = msg.yaw
        
        # 电池信息
        state['battery_percentage'] = msg.battery_percentage
        state['battery_voltage'] = msg.battery_voltage
        state['battery_current'] = msg.battery_current
        state['power_supply_status'] = msg.power_supply_status
        
        # 飞控状态
        state['nav_state'] = msg.nav_state
        state['arming_state'] = msg.arming_state
        state['in_air'] = msg.in_air
        state['landed'] = msg.landed
        state['failsafe'] = msg.failsafe
        state['altitude_relative'] = msg.altitude_relative
        
        # 引导模式状态 (PX4 的 OFFBOARD 模式对应 MAVROS 的 GUIDED 模式)
        # nav_state == 14 表示 OFFBOARD/GUIDED 模式
        state['guided'] = (msg.nav_state == 14)
        
        # 温度（如果有的话）
        state['temperature'] = msg.temperature
        
        # 传感器状态（来自 EstimatorStatusFlags）
        state['sensor_gyro_ok'] = msg.sensor_gyro_ok
        state['sensor_accel_ok'] = msg.sensor_accel_ok
        state['sensor_mag_ok'] = msg.sensor_mag_ok
        state['sensor_baro_ok'] = msg.sensor_baro_ok
        state['sensor_gps_ok'] = msg.sensor_gps_ok
        
        # 处理飞控状态消息（去重后添加到消息列表）
        self._handle_status_text(usv_id, msg.last_status_text, msg.last_status_severity)
    
    def _handle_status_text(self, usv_id: str, status_text: str, severity: int):
        """
        处理飞控状态消息，去重后添加到消息列表
        
        Args:
            usv_id: USV 标识符
            status_text: 状态消息文本
            severity: 消息严重性级别 (0-7)
        """
        # 忽略空消息
        if not status_text or not status_text.strip():
            return
        
        # 检查是否是重复消息（与上一条相同）
        last_text = self._last_status_text_cache.get(usv_id)
        if last_text == status_text:
            return
        
        # 更新缓存
        self._last_status_text_cache[usv_id] = status_text
        
        # 如果有 sensor_handler，委托处理
        if self._sensor_handler:
            try:
                # 创建简单的状态消息对象
                class SimpleStatusText:
                    def __init__(self, text, sev):
                        self.text = text
                        self.severity = sev
                
                msg_obj = SimpleStatusText(status_text, severity)
                self._sensor_handler.handle_status_text(usv_id, msg_obj)
                
                # 发送信号通知 GUI 更新
                try:
                    self.ros_signal.status_text_received.emit(usv_id, status_text)
                except Exception:
                    pass
                    
                self.logger.debug(f"[{usv_id}] 飞控消息: {status_text} (级别: {severity})")
            except Exception as e:
                self.logger.warning(f"处理飞控状态消息失败: {e}")
    
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

            # 销毁导航订阅（避免重复回调）
            try:
                sub = self._nav_feedback_subs.pop(usv_id, None)
                if sub is not None:
                    self.node.destroy_subscription(sub)
                sub = self._nav_result_subs.pop(usv_id, None)
                if sub is not None:
                    self.node.destroy_subscription(sub)
                sub = self._nav_ack_subs.pop(usv_id, None)
                if sub is not None:
                    self.node.destroy_subscription(sub)
                sub = self._telemetry_status_subs.pop(usv_id, None)
                if sub is not None:
                    self.node.destroy_subscription(sub)
            except Exception:
                pass
            
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
