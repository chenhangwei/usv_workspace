"""
传感器状态处理模块

负责处理所有与传感器状态相关的功能：
- 传感器健康状态监测 (SYS_STATUS)
- 预检状态管理 (PreArm/Preflight)
- 状态文本解析 (STATUSTEXT)
- Ready 状态综合判断
"""

from collections import defaultdict, deque
from datetime import datetime
from common_utils import ThreadSafeDict


class SensorStatusHandler:
    """
    传感器状态处理器类
    
    实现 QGC 风格的综合 Ready 检查:
    1. PreArm 警告检查 (来自 STATUSTEXT)
    2. CRITICAL/ERROR 消息检查 (来自 STATUSTEXT)
    3. 传感器健康检查 (来自 SYS_STATUS)
    """
    
    # MAV_SYS_STATUS_SENSOR 位掩码定义
    SENSOR_GYRO = 0x01       # 3D gyro
    SENSOR_ACCEL = 0x02      # 3D accelerometer
    SENSOR_MAG = 0x04        # 3D magnetometer
    SENSOR_BARO = 0x08       # absolute pressure
    SENSOR_GPS = 0x20        # GPS
    
    # 必需传感器（陀螺仪、加速度计、气压计）
    REQUIRED_SENSORS = SENSOR_GYRO | SENSOR_ACCEL | SENSOR_BARO
    
    # 传感器定义列表 (位掩码, 英文名, 中文名)
    SENSORS_DEF = [
        (0x01, 'Gyro', '陀螺仪'),
        (0x02, 'Accel', '加速度计'),
        (0x04, 'Mag', '磁罗盘'),
        (0x08, 'Baro', '气压计'),
    ]
    
    # 传感器名称映射
    SENSOR_NAMES = {
        0x01: "陀螺仪",
        0x02: "加速度计",
        0x04: "磁罗盘",
        0x08: "气压计",
        0x20: "GPS"
    }
    
    # MAVLink/PX4 日志级别 (0-7)
    MAVLINK_SEVERITY = {
        0: 'EMERGENCY',
        1: 'ALERT',
        2: 'CRITICAL',
        3: 'ERROR',
        4: 'WARNING',
        5: 'NOTICE',
        6: 'INFO',
        7: 'DEBUG',
    }
    
    # ROS 2 日志级别 (10/20/30/40/50)
    ROS2_SEVERITY = {
        10: 'DEBUG',
        20: 'INFO',
        30: 'WARNING',
        40: 'ERROR',
        50: 'FATAL',
    }
    
    # PreArm 警告过期时间（秒）
    PREARM_WARNING_EXPIRY = 15.0
    
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
        
        # 状态缓存 (线程安全)
        self._vehicle_messages = defaultdict(lambda: deque(maxlen=50))
        self._prearm_warnings = defaultdict(dict)
        self._prearm_ready = ThreadSafeDict()
        self._sensor_status_cache = ThreadSafeDict()
        self._sensor_health_cache = ThreadSafeDict()
        
        # 用于去重的健康状态日志缓存
        self._last_sensor_health_log = {}
        
        # 用于变化检测的时间戳
        self._last_statustext_time = {}
    
    def handle_status_text(self, usv_id, msg):
        """
        处理飞控 status_text 消息，收集预检提示与车辆消息
        
        Args:
            usv_id: USV 标识符
            msg: StatusText 消息
        """
        if msg is None:
            return

        text_raw = getattr(msg, 'text', '') or ''
        text = text_raw.strip()
        if not text:
            return
        
        try:
            severity = int(getattr(msg, 'severity', 6))
        except (TypeError, ValueError):
            severity = 6
        
        # 根据严重性级别记录到不同的日志等级
        if severity <= 2:  # EMERGENCY/ALERT/CRITICAL
            self.logger.error(f"[FCU-CRITICAL] {usv_id}: {text}")
        elif severity == 3:  # ERROR
            self.logger.error(f"[FCU-ERROR] {usv_id}: {text}")
        elif severity == 4:  # WARNING
            self.logger.warn(f"[FCU-WARNING] {usv_id}: {text}")
        else:  # NOTICE/INFO/DEBUG
            self.logger.info(f"[FCU-INFO] {usv_id}: {text}")

        now_sec = self._now_seconds()
        entry = {
            'text': text,
            'severity': severity,
            'severity_label': self.severity_to_label(severity),
            'time': self._format_time(now_sec),
            'timestamp': now_sec,
        }
        self._vehicle_messages[usv_id].appendleft(entry)

        upper_text = text.upper()
        warnings = self._prearm_warnings[usv_id]
        # 支持 ArduPilot (PREARM) 和 PX4 (PREFLIGHT) 的预检消息
        if 'PREARM' in upper_text or 'PREFLIGHT' in upper_text:
            if severity <= 4 and 'PASS' not in upper_text and 'OK' not in upper_text:
                warnings[text] = now_sec
            else:
                warnings.clear()

        self.cleanup_prearm_warnings(usv_id, now_sec)
        
        # 记录消息到达时间（用于变化检测）
        self._last_statustext_time[usv_id] = now_sec

    def handle_sys_status(self, usv_id, msg):
        """
        处理飞控 SYS_STATUS 消息，缓存传感器健康状态
        
        根据 QGC 实现，使用 onboard_control_sensors_health 位掩码来判断传感器健康状态。
        
        Args:
            usv_id: USV 标识符
            msg: SysStatus 消息
        """
        if msg is None:
            return
        
        # 缓存原始传感器状态位掩码
        self._sensor_health_cache[usv_id] = {
            'onboard_control_sensors_present': msg.sensors_present,
            'onboard_control_sensors_enabled': msg.sensors_enabled,
            'onboard_control_sensors_health': msg.sensors_health,
            'timestamp': self._now_seconds()
        }
        
        # 记录日志以便调试（仅首次或状态变化时）
        prev = self._last_sensor_health_log.get(usv_id)
        curr_health = msg.sensors_health
        if prev != curr_health:
            self.logger.info(
                f"[SYS_STATUS] {usv_id} 传感器健康更新: "
                f"present=0x{msg.sensors_present:08X}, "
                f"enabled=0x{msg.sensors_enabled:08X}, "
                f"health=0x{curr_health:08X}"
            )
            self._last_sensor_health_log[usv_id] = curr_health
    
    def check_sensor_health(self, usv_id):
        """
        检查关键传感器是否健康 (基于 SYS_STATUS 位掩码)
        
        根据 QGC 实现方式，检查所有已启用且需要的传感器是否健康。
        适用于 ArduPilot 和 PX4。
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            (bool, list): (是否健康, 不健康传感器列表)
        """
        sensor_data = self._sensor_health_cache.get(usv_id)
        if not sensor_data:
            # 如果还没有收到 SYS_STATUS 消息，暂时认为传感器健康
            return True, []
        
        present = sensor_data['onboard_control_sensors_present']
        enabled = sensor_data['onboard_control_sensors_enabled']
        health = sensor_data['onboard_control_sensors_health']
        
        unhealthy_sensors = []
        
        # 检查每个传感器
        for bit, name in self.SENSOR_NAMES.items():
            # 如果传感器存在且已启用
            if (present & bit) and (enabled & bit):
                # 检查是否健康
                if not (health & bit):
                    unhealthy_sensors.append(name)
        
        # 如果有不健康的传感器，返回 False
        if unhealthy_sensors:
            return False, unhealthy_sensors
        
        # 检查必需传感器是否都已启用
        required_enabled = (enabled & self.REQUIRED_SENSORS)
        if required_enabled != (present & self.REQUIRED_SENSORS):
            missing = []
            for bit, name in self.SENSOR_NAMES.items():
                if (bit & self.REQUIRED_SENSORS) and (present & bit) and not (enabled & bit):
                    missing.append(f"{name}(未启用)")
            if missing:
                return False, missing
        
        return True, []

    def build_sensor_status(self, usv_id, state):
        """
        根据当前状态评估关键传感器的健康状况
        
        Args:
            usv_id: USV 标识符
            state: USV 状态字典
            
        Returns:
            list: 传感器状态列表
        """
        statuses = []

        # 1. 传感器健康状态 (IMU, Baro, Mag)
        sensor_data = self._sensor_health_cache.get(usv_id)

        if sensor_data:
            present = sensor_data.get('onboard_control_sensors_present', 0)
            enabled = sensor_data.get('onboard_control_sensors_enabled', 0)
            health = sensor_data.get('onboard_control_sensors_health', 0)
            
            for bit, name_en, name_cn in self.SENSORS_DEF:
                # 只显示存在的传感器
                if present & bit:
                    if not (enabled & bit):
                        # 存在但未启用
                        status = 'Disabled'
                        level = 'warn'
                        detail = f"{name_cn} (未启用)"
                    elif health & bit:
                        # 健康 (位为1表示健康)
                        status = 'OK'
                        level = 'ok'
                        detail = name_cn
                    else:
                        # 不健康
                        status = 'Error'
                        level = 'error'
                        detail = f"{name_cn} 故障"
                    
                    statuses.append({
                        'name': name_en,
                        'status': status,
                        'detail': detail,
                        'level': level
                    })
        else:
            # 如果没有传感器数据，显示未知状态
            for _, name_en, name_cn in self.SENSORS_DEF:
                statuses.append({
                    'name': name_en,
                    'status': 'Unknown',
                    'detail': f"{name_cn} (未知)",
                    'level': 'warn'
                })

        # 2. 电池信息
        battery_pct = state.get('battery_percentage')
        battery_voltage = state.get('battery_voltage')
        try:
            battery_pct_val = float(battery_pct) if battery_pct is not None else None
        except (TypeError, ValueError):
            battery_pct_val = None

        if battery_pct_val is not None:
            if battery_pct_val >= 30.0:
                battery_level = 'ok'
            elif battery_pct_val >= 15.0:
                battery_level = 'warn'
            else:
                battery_level = 'error'
            detail = f"{battery_pct_val:.0f}%"
            if battery_voltage is not None:
                try:
                    detail += f" @ {float(battery_voltage):.1f}V"
                except (TypeError, ValueError):
                    pass
            statuses.append({
                'name': 'Battery',
                'status': 'OK' if battery_level == 'ok' else 'Low',
                'detail': detail,
                'level': battery_level,
            })

        # 3. 温度检查（从毫摄氏度转换为摄氏度）
        temperature = state.get('temperature')
        try:
            temp_raw = float(temperature) if temperature is not None else None
            temp_val = temp_raw / 1000.0 if temp_raw is not None else None  # 毫度 → 度
        except (TypeError, ValueError):
            temp_val = None
        if temp_val is not None and temp_val > 0:
            if temp_val >= 75.0:
                temp_level = 'error'
            elif temp_val >= 60.0:
                temp_level = 'warn'
            else:
                temp_level = 'ok'
            statuses.append({
                'name': 'CPU Temp',
                'status': f"{temp_val:.1f}°C",
                'detail': '',
                'level': temp_level,
            })

        return statuses
    
    def cleanup_prearm_warnings(self, usv_id, now_sec=None):
        """
        清理过期的预检警告
        
        Args:
            usv_id: USV 标识符
            now_sec: 当前时间（秒），如果未提供则自动获取
        """
        if now_sec is None:
            now_sec = self._now_seconds()
            
        warnings = self._prearm_warnings.get(usv_id)
        if not warnings:
            return
        for key, ts in list(warnings.items()):
            if now_sec - ts > self.PREARM_WARNING_EXPIRY:
                warnings.pop(key, None)
    
    def get_prearm_warnings(self, usv_id):
        """
        获取当前预检警告列表
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            list: 预检警告文本列表
        """
        return list(self._prearm_warnings.get(usv_id, {}).keys())
    
    def get_vehicle_messages(self, usv_id):
        """
        获取车辆消息列表
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            list: 消息列表
        """
        return [dict(item) for item in self._vehicle_messages.get(usv_id, [])]
    
    def get_critical_errors(self, usv_id, within_seconds=30.0):
        """
        获取最近的严重错误消息
        
        Args:
            usv_id: USV 标识符
            within_seconds: 时间窗口（秒）
            
        Returns:
            list: 严重错误消息列表
        """
        now_sec = self._now_seconds()
        critical_errors = []
        for msg_entry in self._vehicle_messages.get(usv_id, []):
            severity = msg_entry.get('severity', 6)
            timestamp = msg_entry.get('timestamp', 0)
            # severity <= 3 表示 EMERGENCY/ALERT/CRITICAL/ERROR
            if severity <= 3 and (now_sec - timestamp) <= within_seconds:
                critical_errors.append(msg_entry.get('text', ''))
        return critical_errors
    
    def get_last_statustext_time(self, usv_id):
        """
        获取最后一次接收状态文本的时间
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            float: 时间戳（秒）
        """
        return self._last_statustext_time.get(usv_id, 0)
    
    def get_sensor_health_cache(self, usv_id):
        """
        获取传感器健康缓存
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            dict: 传感器健康数据
        """
        return self._sensor_health_cache.get(usv_id)
    
    def set_prearm_ready(self, usv_id, ready):
        """
        设置预检 Ready 状态
        
        Args:
            usv_id: USV 标识符
            ready: 是否 Ready
        """
        self._prearm_ready[usv_id] = ready
    
    def get_prearm_ready(self, usv_id):
        """
        获取预检 Ready 状态
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            bool: 是否 Ready
        """
        return self._prearm_ready.get(usv_id, False)
    
    def cache_sensor_status(self, usv_id, status):
        """
        缓存传感器状态
        
        Args:
            usv_id: USV 标识符
            status: 传感器状态
        """
        self._sensor_status_cache[usv_id] = status
    
    def get_sensor_status_cache(self, usv_id):
        """
        获取缓存的传感器状态
        
        Args:
            usv_id: USV 标识符
            
        Returns:
            传感器状态
        """
        return self._sensor_status_cache.get(usv_id)

    def _now_seconds(self):
        """获取当前时间（秒）"""
        try:
            return self.node.get_clock().now().nanoseconds / 1e9
        except Exception:
            return datetime.now().timestamp()
    
    def _format_time(self, seconds):
        """格式化时间戳为时间字符串"""
        try:
            return datetime.fromtimestamp(seconds).strftime('%H:%M:%S')
        except Exception:
            return '--:--:--'

    @staticmethod
    def severity_to_label(severity):
        """
        将严重级别转换为标签
        
        Args:
            severity: 严重级别数值
            
        Returns:
            str: 严重级别标签
        """
        # 先尝试 MAVLink 映射，再尝试 ROS 2 映射
        if severity in SensorStatusHandler.MAVLINK_SEVERITY:
            return SensorStatusHandler.MAVLINK_SEVERITY[severity]
        if severity in SensorStatusHandler.ROS2_SEVERITY:
            return SensorStatusHandler.ROS2_SEVERITY[severity]
        return f'LEVEL {severity}'
