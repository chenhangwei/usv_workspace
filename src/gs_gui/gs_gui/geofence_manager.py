"""
电子围栏管理器
负责监控USV位置,当越界时触发动作
"""
import logging
from PyQt5.QtCore import QObject

class GeofenceManager(QObject):
    def __init__(self, ros_signal, notification_callback):
        """
        初始化电子围栏管理器
        
        Args:
            ros_signal: ROS信号对象,用于发送控制命令
            notification_callback: 通知回调函数
        """
        super().__init__()
        self.ros_signal = ros_signal
        self.notification_callback = notification_callback
        
        # 围栏状态
        self.enabled = False
        self.rect = {
            'x_min': -50.0, 
            'x_max': 50.0, 
            'y_min': -50.0, 
            'y_max': 50.0
        }
        
        # 记录正在违规的USV,避免重复发送命令
        self.violating_usvs = set()
        
    def set_bounds(self, bounds):
        """设置围栏边界 {'x_min': float, ...}"""
        if bounds == self.rect:
            return
        self.rect = bounds
        self.notification_callback(f"电子围栏边界已更新: X[{bounds['x_min']}:{bounds['x_max']}], Y[{bounds['y_min']}:{bounds['y_max']}]")
    
    def set_enabled(self, enabled):
        """启用/禁用电子围栏"""
        if enabled == self.enabled:
            return
            
        self.enabled = enabled
        if enabled:
            self.notification_callback("⚠️ 电子围栏已启用")
        else:
            self.notification_callback("电子围栏已禁用")
            self.violating_usvs.clear()

    def check_usv_states(self, usv_list):
        """
        检查USV位置是否越界
        Args:
            usv_list: USV状态列表 (来自 receive_state_list 信号)
        """
        if not self.enabled:
            return

        for usv_data in usv_list:
            if not isinstance(usv_data, dict):
                continue
            
            usv_id = usv_data.get('namespace')
            x = usv_data.get('x')
            y = usv_data.get('y')
            connected = usv_data.get('connected', False)
            
            # 忽略数据无效或未连接的节点
            if not usv_id or x is None or y is None or not connected:
                continue

            # 检查边界
            is_inside = (self.rect['x_min'] <= x <= self.rect['x_max']) and \
                        (self.rect['y_min'] <= y <= self.rect['y_max'])
            
            if not is_inside:
                if usv_id not in self.violating_usvs:
                    # 发现新的越界
                    self.violating_usvs.add(usv_id)
                    self.notification_callback(f"⚠️ [警告] {usv_id} 越出电子围栏 ({x:.1f}, {y:.1f})! 执行HOLD模式。")
                    
                    # 发送 HOLD 命令
                    try:
                        self.ros_signal.hold_command.emit([usv_id])
                    except Exception as e:
                        self.notification_callback(f"发送HOLD命令失败: {e}")
            else:
                if usv_id in self.violating_usvs:
                    # 回到安全区域
                    self.violating_usvs.remove(usv_id)
                    self.notification_callback(f"✅ {usv_id} 已回到电子围栏内。")
