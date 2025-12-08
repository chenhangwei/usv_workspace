"""
传感器状态处理模块

负责处理所有与传感器状态相关的功能：
- 传感器健康状态监测
- 预检状态管理
- 状态文本解析
"""

from collections import defaultdict, deque
from common_utils import ThreadSafeDict


class SensorStatusHandler:
    """传感器状态处理器类"""
    
    # PX4 传感器位掩码定义
    SENSOR_MASKS = {
        'GYRO': 0x01,
        'ACCEL': 0x02,
        'MAG': 0x04,
        'BARO': 0x08,
        'GPS': 0x20,
        'OPTICAL_FLOW': 0x40,
        'VISION_POSITION': 0x80,
        'LASER_POSITION': 0x100,
        'EXTERNAL_GROUND_TRUTH': 0x200,
        'MOTOR_OUTPUTS': 0x1000,
        'RC_RECEIVER': 0x4000,
        'BATTERY': 0x20000,
    }
    
    # 状态文本严重级别
    SEVERITY_LABELS = {
        0: 'EMERGENCY',
        1: 'ALERT',
        2: 'CRITICAL',
        3: 'ERROR',
        4: 'WARNING',
        5: 'NOTICE',
        6: 'INFO',
        7: 'DEBUG',
    }
    
    def __init__(self, node, ros_signal):
        """
        初始化传感器状态处理器
        
        Args:
            node: ROS 节点实例
            ros_signal: ROS 信号对象
        """
        self.node = node
        self.ros_signal = ros_signal
        self.logger = node.get_logger()
        
        # 状态缓存
        self._vehicle_messages = defaultdict(lambda: deque(maxlen=50))
        self._prearm_warnings = defaultdict(dict)
        self._prearm_ready = ThreadSafeDict()
        self._sensor_status_cache = ThreadSafeDict()
        self._sensor_health_cache = ThreadSafeDict()
        self._heartbeat_status_cache = ThreadSafeDict()
        
        # 用于去重的健康状态日志缓存
        self._last_sensor_health_log = {}
    
    def handle_status_text(self, usv_id, msg):
        """
        处理状态文本消息
        
        Args:
            usv_id: USV 标识符
            msg: StatusText 消息
        """
        try:
            severity = msg.severity if hasattr(msg, 'severity') else 6
            text = msg.text if hasattr(msg, 'text') else str(msg)
            
            # 记录到消息历史
            timestamp = self._now_seconds()
            self._vehicle_messages[usv_id].append({
                'time': timestamp,
                'severity': severity,
                'text': text
            })
            
            # 解析预检警告
            self._parse_prearm_warning(usv_id, text, severity)
            
            # 日志输出
            severity_label = self.SEVERITY_LABELS.get(severity, 'UNKNOWN')
            self.logger.debug(f"[{usv_id}] {severity_label}: {text}")
            
        except Exception as e:
            self.logger.error(f"处理状态文本失败: {e}")
    
    def _parse_prearm_warning(self, usv_id, text, severity):
        """
        解析预检警告
        
        Args:
            usv_id: USV 标识符
            text: 状态文本
            severity: 严重级别
        """
        # 检查是否是预检相关消息
        prearm_keywords = ['Preflight', 'PreArm', 'prearm', 'preflight', 'Check']
        is_prearm = any(kw in text for kw in prearm_keywords)
        
        if not is_prearm:
            return
        
        # 更新预检警告缓存
        now = self._now_seconds()
        warning_key = text[:50]  # 使用前50个字符作为键
        
        self._prearm_warnings[usv_id][warning_key] = {
            'text': text,
            'severity': severity,
            'time': now
        }
        
        # 检查是否ready
        if 'ready' in text.lower() or 'passed' in text.lower():
            self._prearm_ready[usv_id] = True
        elif severity <= 4:  # WARNING 或更严重
            self._prearm_ready[usv_id] = False
    
    def handle_sys_status(self, usv_id, msg):
        """
        处理系统状态消息
        
        Args:
            usv_id: USV 标识符
            msg: SysStatus 消息
        """
        try:
            # 更新传感器健康缓存
            curr_health = msg.sensors_health if hasattr(msg, 'sensors_health') else 0
            
            # 只在健康状态变化时记录日志
            if usv_id not in self._last_sensor_health_log or \
               self._last_sensor_health_log[usv_id] != curr_health:
                self.logger.debug(
                    f"[SYS_STATUS] {usv_id} 传感器健康更新: "
                    f"present=0x{msg.sensors_present:08X}, "
                    f"enabled=0x{msg.sensors_enabled:08X}, "
                    f"health=0x{curr_health:08X}"
                )
                self._last_sensor_health_log[usv_id] = curr_health
            
            # 缓存传感器状态
            self._sensor_health_cache[usv_id] = {
                'present': msg.sensors_present,
                'enabled': msg.sensors_enabled,
                'health': curr_health,
                'time': self._now_seconds()
            }
            
        except Exception as e:
            self.logger.error(f"处理系统状态失败: {e}")
    
    def check_sensor_health(self, usv_id):
        """
        检查传感器健康状态
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            dict: 传感器健康状态
        """
        cache = self._sensor_health_cache.get(usv_id)
        if not cache:
            return {'status': 'UNKNOWN', 'details': {}}
        
        present = cache.get('present', 0)
        enabled = cache.get('enabled', 0)
        health = cache.get('health', 0)
        
        sensor_status = {}
        for name, mask in self.SENSOR_MASKS.items():
            is_present = bool(present & mask)
            is_enabled = bool(enabled & mask)
            is_healthy = bool(health & mask)
            
            if is_present:
                if is_healthy:
                    sensor_status[name] = 'OK'
                elif is_enabled:
                    sensor_status[name] = 'ERROR'
                else:
                    sensor_status[name] = 'DISABLED'
            else:
                sensor_status[name] = 'NOT_PRESENT'
        
        # 计算总体状态
        errors = [k for k, v in sensor_status.items() if v == 'ERROR']
        if errors:
            overall_status = 'ERROR'
        elif all(v in ['OK', 'NOT_PRESENT', 'DISABLED'] for v in sensor_status.values()):
            overall_status = 'OK'
        else:
            overall_status = 'UNKNOWN'
        
        return {
            'status': overall_status,
            'details': sensor_status,
            'errors': errors
        }
    
    def build_sensor_status(self, usv_id, state):
        """
        构建传感器状态信息
        
        Args:
            usv_id: USV 标识符
            state: USV 状态字典
            
        Returns:
            dict: 传感器状态信息
        """
        sensor_health = self.check_sensor_health(usv_id)
        prearm_ready = self._prearm_ready.get(usv_id, False)
        
        return {
            'sensor_health': sensor_health,
            'prearm_ready': prearm_ready,
            'prearm_warnings': dict(self._prearm_warnings.get(usv_id, {})),
            'messages': list(self._vehicle_messages.get(usv_id, []))[-10:]  # 最近10条消息
        }
    
    def get_prearm_status(self, usv_id):
        """
        获取预检状态
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            dict: 预检状态信息
        """
        return {
            'ready': self._prearm_ready.get(usv_id, False),
            'warnings': dict(self._prearm_warnings.get(usv_id, {}))
        }
    
    def cleanup_old_warnings(self, usv_id, max_age=30.0):
        """
        清理过期的预检警告
        
        Args:
            usv_id: USV 标识符
            max_age: 最大保留时间（秒）
        """
        now = self._now_seconds()
        warnings = self._prearm_warnings.get(usv_id, {})
        
        expired_keys = [
            k for k, v in warnings.items()
            if now - v.get('time', 0) > max_age
        ]
        
        for key in expired_keys:
            del warnings[key]
    
    def _now_seconds(self):
        """获取当前时间（秒）"""
        try:
            return self.node.get_clock().now().nanoseconds / 1e9
        except Exception:
            return 0.0
    
    @staticmethod
    def severity_to_label(severity):
        """
        将严重级别转换为标签
        
        Args:
            severity: 严重级别数值
            
        Returns:
            str: 严重级别标签
        """
        return SensorStatusHandler.SEVERITY_LABELS.get(severity, 'UNKNOWN')
