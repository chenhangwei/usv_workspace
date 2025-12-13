"""
信息面板样式更新函数

提供各种状态的样式更新函数。
"""
from PyQt5.QtWidgets import QLabel, QProgressBar
from typing import Tuple


def update_mode_style(label: QLabel, mode: str) -> None:
    """
    根据模式更新标签样式
    
    Args:
        label: 模式标签
        mode: 模式字符串
    """
    mode_str = str(mode).upper()
    if "GUIDED" in mode_str or "OFFBOARD" in mode_str:
        color = "#27ae60"  # 绿色
    elif "MANUAL" in mode_str:
        color = "#f39c12"  # 橙色
    elif "AUTO" in mode_str:
        color = "#3498db"  # 蓝色
    elif "STABILIZED" in mode_str or "POSCTL" in mode_str:
        color = "#9b59b6"  # 紫色
    else:
        color = "#95a5a6"  # 灰色
    
    label.setStyleSheet(f"""
        QLabel {{
            color: white;
            background-color: {color};
            font-weight: bold;
            padding: 5px;
            border-radius: 4px;
        }}
    """)


def update_status_style(label: QLabel, status: str) -> None:
    """
    根据状态更新标签样式
    
    Args:
        label: 状态标签
        status: 状态字符串 ("在线", "离线", "飞控断开" 等)
    """
    status_str = str(status).upper()
    if "在线" in status or "ACTIVE" in status_str or "ONLINE" in status_str:
        color = "#27ae60"  # 绿色
    elif "飞控断开" in status or "DISCONN" in status_str:
        color = "#f39c12"  # 橙色 - 网络在线但飞控断开
    elif "离线" in status or "OFFLINE" in status_str:
        color = "#e74c3c"  # 红色
    elif "STANDBY" in status_str:
        color = "#3498db"  # 蓝色
    elif "CRITICAL" in status_str or "EMERGENCY" in status_str:
        color = "#e74c3c"  # 红色
    else:
        color = "#95a5a6"  # 灰色
    
    label.setStyleSheet(f"""
        QLabel {{
            color: white;
            background-color: {color};
            font-weight: bold;
            padding: 5px;
            border-radius: 4px;
        }}
    """)


def update_armed_style(label: QLabel, armed) -> None:
    """
    根据解锁状态更新标签样式
    
    Args:
        label: 解锁状态标签
        armed: 是否解锁
    """
    armed_str = str(armed).upper()
    if "TRUE" in armed_str or "ARMED" in armed_str or armed is True:
        color = "#e74c3c"  # 红色
        text = "已解锁"
    else:
        color = "#27ae60"  # 绿色
        text = "已锁定"
    
    label.setText(text)
    label.setStyleSheet(f"""
        QLabel {{
            color: white;
            background-color: {color};
            font-weight: bold;
            padding: 5px;
            border-radius: 4px;
        }}
    """)


def update_battery_style(progress_bar: QProgressBar, percentage: float) -> None:
    """
    根据电池百分比更新进度条样式
    
    Args:
        progress_bar: 电池进度条
        percentage: 电池百分比 (0-100)
    """
    try:
        pct = float(percentage)
    except (ValueError, TypeError):
        pct = 0
    
    if pct > 50:
        color = "#27ae60"  # 绿色
    elif pct > 20:
        color = "#f39c12"  # 橙色
    else:
        color = "#e74c3c"  # 红色
    
    progress_bar.setStyleSheet(f"""
        QProgressBar {{
            border: 1px solid #bdc3c7;
            border-radius: 3px;
            text-align: center;
            background-color: #ecf0f1;
        }}
        QProgressBar::chunk {{
            background-color: {color};
            border-radius: 2px;
        }}
    """)
    progress_bar.setValue(int(pct))


def get_temperature_style(
    temp_celsius: float, 
    is_high_temperature: bool
) -> Tuple[str, str, bool]:
    """
    根据温度返回样式（带滞后效果）
    
    Args:
        temp_celsius: 温度（摄氏度）
        is_high_temperature: 当前是否处于高温状态
        
    Returns:
        (文字颜色, 背景颜色, 新的高温状态)
    """
    try:
        temp = float(temp_celsius)
    except (ValueError, TypeError):
        return ("#2c3e50", "#ecf0f1", False)
    
    # 使用滞后逻辑
    HIGH_THRESHOLD = 65.0
    LOW_THRESHOLD = 55.0
    
    if is_high_temperature:
        # 当前高温，需要降到 55°C 以下才恢复
        if temp < LOW_THRESHOLD:
            is_high_temperature = False
    else:
        # 当前正常，需要升到 65°C 以上才报警
        if temp >= HIGH_THRESHOLD:
            is_high_temperature = True
    
    if is_high_temperature:
        return ("#ffffff", "#e74c3c", True)  # 红色背景
    elif temp >= 50:
        return ("#000000", "#f39c12", is_high_temperature)  # 橙色背景
    else:
        return ("#2c3e50", "#ecf0f1", is_high_temperature)  # 正常
