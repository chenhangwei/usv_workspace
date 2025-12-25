"""
USV 命令处理模块。

负责封装并发送各类控制指令，包括飞行模式切换、武装控制、外设（灯光、声音、颈部）控制等。
支持集群与离群的批量指令下发。
"""

import logging
from typing import List, Dict, Any, Callable, Optional, Tuple
from PyQt5.QtCore import QTimer, QObject
from PyQt5.QtWidgets import QColorDialog

# 模块级日志记录器
_logger = logging.getLogger("gs_gui.commands")


class USVCommandHandler(QObject):
    """
    USV 命令处理器。
    
    将 UI 交互转换为 ROS 信号，支持彩虹灯光循环等定时任务。
    """
    
    def __init__(self, ros_signal: Any, info_callback: Callable[[str], None]) -> None:
        """
        初始化命令处理器。
        
        Args:
            ros_signal: ROS 信号桥接对象。
            info_callback: 信息日志输出回调。
        """
        super().__init__()
        self.ros_signal = ros_signal
        self.append_info = info_callback
        
        # 彩虹灯光配置
        self._rainbow_colors: List[Tuple[int, int, int]] = [
            (255, 0, 0), (255, 127, 0), (255, 255, 0), 
            (0, 255, 0), (0, 255, 255), (0, 0, 255), (148, 0, 211)
        ]
        self._rainbow_index: int = 0
        self._rainbow_timer = QTimer()
        self._rainbow_timer.setInterval(5000)  # 5秒切换一次
        self._rainbow_timer.timeout.connect(self._send_next_rainbow_color)
        
        _logger.info("USVCommandHandler 初始化完成")

    def _extract_namespaces(self, usv_list: List[Dict[str, Any]]) -> List[str]:
        """从 USV 字典列表中提取命名空间。"""
        return [item['namespace'] for item in usv_list if isinstance(item, dict) and 'namespace' in item]

    def _send_batch_command(self, usv_list: List[Dict[str, Any]], signal: Any, cmd_name: str) -> None:
        """通用批量指令发送逻辑。"""
        try:
            namespaces = self._extract_namespaces(usv_list)
            if not namespaces:
                self.append_info(f"⚠️ 未选中任何 USV，无法发送 {cmd_name}")
                return
            
            signal.emit(namespaces)
            self.append_info(f"✅ {cmd_name} 指令已发送至: {', '.join(namespaces)}")
        except Exception as e:
            self.append_info(f"❌ 发送 {cmd_name} 失败: {e}")
            _logger.error(f"Batch command {cmd_name} error: {e}")

    # ============== 模式与武装控制 ==============
    def set_cluster_arming(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.arm_command, "集群解锁")

    def cluster_disarming(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.disarm_command, "集群加锁")

    def set_cluster_offboard(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.offboard_command, "集群 OFFBOARD")

    def set_cluster_hold(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.hold_command, "集群 HOLD")

    def set_cluster_stabilized(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.stabilized_command, "集群 STABILIZED")

    def departed_arming(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.arm_command, "离群解锁")

    def departed_disarming(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.disarm_command, "离群加锁")

    def set_departed_offboard(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.offboard_command, "离群 OFFBOARD")

    def set_departed_stabilized(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.stabilized_command, "离群 STABILIZED")

    def set_departed_posctl(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.posctl_command, "离群 POSCTL")

    def set_departed_altctl(self, usv_list: List[Dict[str, Any]]) -> None:
        self._send_batch_command(usv_list, self.ros_signal.altctl_command, "离群 ALTCTL")

    # ============== 外设控制 ==============
    def sound_start(self) -> None:
        self.ros_signal.str_command.emit('sound_start')
        self.append_info("🔊 发送指令: 开启声音")

    def sound_stop(self) -> None:
        self.ros_signal.str_command.emit('sound_stop')
        self.append_info("🔇 发送指令: 停止声音")

    def neck_swinging(self) -> None:
        self.ros_signal.str_command.emit('neck_swinging')
        self.append_info("🦒 发送指令: 颈部摆动")

    def neck_stop(self) -> None:
        self.ros_signal.str_command.emit('neck_stop')
        self.append_info("🦒 发送指令: 颈部停止")

    # ============== LED 控制 ==============
    def led_color_switching(self) -> bool:
        """切换彩虹循环状态。"""
        if self._rainbow_timer.isActive():
            self._rainbow_timer.stop()
            self.append_info("🌈 LED 彩虹循环已停止")
            return False
        
        self._rainbow_index = 0
        self._send_next_rainbow_color()
        self._rainbow_timer.start()
        self.append_info("🌈 LED 彩虹循环已启动 (5s 步进)")
        return True

    def led_random_color(self) -> None:
        """触发随机颜色。"""
        self._stop_rainbow_cycle()
        self.ros_signal.str_command.emit('random_color_change')
        self.append_info("🎨 发送指令: 随机颜色")

    def led_select_color(self, parent: Optional[Any] = None) -> None:
        """打开颜色选择对话框。"""
        self._stop_rainbow_cycle()
        color = QColorDialog.getColor(parent=parent)
        if color.isValid():
            r, g, b = color.red(), color.green(), color.blue()
            cmd = f"color_select|{r},{g},{b}"
            self.ros_signal.str_command.emit(cmd)
            self.append_info(f"🎨 发送指令: 设置颜色 RGB({r},{g},{b})")

    def led_off(self) -> None:
        """关闭所有 LED。"""
        self._stop_rainbow_cycle()
        self.ros_signal.str_command.emit('led_off')
        self.append_info("💡 发送指令: 关闭灯光")

    def _send_next_rainbow_color(self) -> None:
        """发送下一个彩虹颜色。"""
        color = self._rainbow_colors[self._rainbow_index]
        cmd = f"color_select|{color[0]},{color[1]},{color[2]}"
        self.ros_signal.str_command.emit(cmd)
        self._rainbow_index = (self._rainbow_index + 1) % len(self._rainbow_colors)

    def _stop_rainbow_cycle(self) -> None:
        """内部停止彩虹循环。"""
        if self._rainbow_timer.isActive():
            self._rainbow_timer.stop()
