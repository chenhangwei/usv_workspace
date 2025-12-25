"""
信息面板通用组件

提供 USV 信息面板使用的通用 UI 组件和样式工具。
"""
from PyQt5.QtWidgets import (
    QLabel, QListWidget, QPushButton, QAbstractItemView, QListWidgetItem
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor

# Qt 兼容性定义
try:
    AlignRight = Qt.AlignmentFlag.AlignRight
    AlignLeft = Qt.AlignmentFlag.AlignLeft
    AlignVCenter = Qt.AlignmentFlag.AlignVCenter
except AttributeError:
    AlignRight = Qt.AlignRight
    AlignLeft = Qt.AlignLeft
    AlignVCenter = Qt.AlignVCenter


# 统一的 QGroupBox 样式
GROUPBOX_STYLE = """
    QGroupBox {
        font-weight: bold;
        font-size: 12px;
        border: 1.5px solid #3498db;
        border-radius: 5px;
        margin-top: 6px;
        padding-top: 6px;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 3px;
    }
"""


def create_key_label(text: str) -> QLabel:
    """
    创建键标签（左侧描述文字）
    
    Args:
        text: 标签文字
        
    Returns:
        配置好样式的 QLabel
    """
    label = QLabel(text)
    label.setStyleSheet("""
        QLabel {
            color: #7f8c8d;
            font-size: 12px;
            padding: 2px 5px;
        }
    """)
    label.setAlignment(AlignLeft | AlignVCenter)
    return label


def create_value_label(text: str, large: bool = False) -> QLabel:
    """
    创建值标签（右侧数值显示）
    
    Args:
        text: 标签文字
        large: 是否使用大字体
        
    Returns:
        配置好样式的 QLabel
    """
    label = QLabel(text)
    font_size = 16 if large else 14
    label.setStyleSheet(f"""
        QLabel {{
            color: #2c3e50;
            font-size: {font_size}px;
            font-weight: bold;
            padding: 2px 5px;
        }}
    """)
    label.setAlignment(AlignRight | AlignVCenter)
    return label


def create_section_label(text: str) -> QLabel:
    """
    创建分节标签
    
    Args:
        text: 标签文字
        
    Returns:
        配置好样式的 QLabel
    """
    label = QLabel(text)
    label.setStyleSheet("""
        QLabel {
            color: #34495e;
            font-size: 12px;
            font-weight: bold;
            padding: 4px 0;
            border-bottom: 1px solid #ecf0f1;
        }
    """)
    return label


def configure_list_widget(widget: QListWidget, allow_selection: bool = False) -> None:
    """
    配置列表控件的通用样式
    
    Args:
        widget: QListWidget 实例
        allow_selection: 是否允许选择
    """
    widget.setStyleSheet("""
        QListWidget {
            background-color: #f8f9fa;
            border: 1px solid #e0e0e0;
            border-radius: 4px;
            padding: 5px;
        }
        QListWidget::item {
            padding: 3px 5px;
            border-bottom: 1px solid #eee;
        }
        QListWidget::item:last-child {
            border-bottom: none;
        }
        QListWidget::item:selected {
            background-color: #e3f2fd;
            color: #1976d2;
        }
    """)
    
    if allow_selection:
        widget.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
    else:
        widget.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
    
    widget.setFocusPolicy(Qt.FocusPolicy.NoFocus)
    widget.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
    widget.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    widget.setWordWrap(True)


def set_list_placeholder(widget: QListWidget, text: str) -> None:
    """
    设置列表占位符文本
    
    Args:
        widget: QListWidget 实例
        text: 占位符文字
    """
    widget.clear()
    item = QListWidgetItem(text)
    item.setForeground(QColor("#95a5a6"))
    widget.addItem(item)


def apply_button_style(
    button: QPushButton, 
    background: str, 
    text_color: str = "#ffffff"
) -> None:
    """
    应用按钮样式
    
    Args:
        button: QPushButton 实例
        background: 背景颜色
        text_color: 文字颜色
    """
    button.setStyleSheet(f"""
        QPushButton {{
            background-color: {background};
            color: {text_color};
            border: none;
            padding: 5px 10px;
            border-radius: 3px;
            font-weight: bold;
        }}
        QPushButton:hover {{
            background-color: {background}dd;
        }}
        QPushButton:pressed {{
            background-color: {background}aa;
        }}
    """)


def format_float(value, precision: int = 2) -> str:
    """
    格式化浮点数显示
    
    Args:
        value: 要格式化的值
        precision: 小数位数
        
    Returns:
        格式化后的字符串
    """
    try:
        return f"{float(value):.{precision}f}"
    except (ValueError, TypeError):
        return "--"


def level_to_palette(level) -> tuple:
    """
    将日志级别转换为调色板颜色（深色背景 + 彩色文字）
    
    Args:
        level: 日志级别，支持以下格式:
               - 整数 (0-7 MAVLink 或 10-50 ROS2)
               - 字符串 ('error', 'warn', 'warning', 'info', 'ok', 'good', 'debug')
        
    Returns:
        (背景色, 前景色) 元组
    """
    # 深色背景统一使用
    dark_bg = "#2d2d2d"
    
    # 如果是字符串，转换为对应的颜色
    if isinstance(level, str):
        level_lower = level.lower()
        if level_lower in ('error', 'critical', 'emergency', 'alert'):
            return (dark_bg, "#ff4444")  # 亮红色文字
        elif level_lower in ('warn', 'warning'):
            return (dark_bg, "#ffcc00")  # 亮黄色文字
        elif level_lower in ('ok', 'good', 'success'):
            return (dark_bg, "#44ff44")  # 亮绿色文字
        elif level_lower in ('info', 'notice'):
            return (dark_bg, "#4da6ff")  # 亮蓝色文字
        else:  # debug, unknown, 等
            return (dark_bg, "#aaaaaa")  # 浅灰色文字
    
    # 如果是整数，使用 MAVLink 级别映射
    try:
        level_int = int(level)
        if level_int <= 2:  # EMERGENCY, ALERT, CRITICAL
            return (dark_bg, "#ff4444")  # 亮红色文字
        elif level_int <= 3:  # ERROR
            return (dark_bg, "#ff6600")  # 亮橙色文字
        elif level_int <= 4:  # WARNING
            return (dark_bg, "#ffcc00")  # 亮黄色文字
        elif level_int <= 5:  # NOTICE
            return (dark_bg, "#4da6ff")  # 亮蓝色文字
        else:  # INFO, DEBUG
            return (dark_bg, "#aaaaaa")  # 浅灰色文字
    except (ValueError, TypeError):
        # 无法转换，返回默认颜色
        return (dark_bg, "#aaaaaa")


def severity_palette(severity: str) -> tuple:
    """
    根据严重性返回调色板（深色背景 + 彩色文字）
    
    Args:
        severity: 严重性字符串
        
    Returns:
        (前景色, 背景色) 元组
    """
    # 深色背景统一使用
    dark_bg = "#2d2d2d"
    
    severity_map = {
        # 严重级别：亮红色文字
        'EMERGENCY': ("#ff0000", dark_bg),
        'ALERT': ("#ff2222", dark_bg),
        'CRITICAL': ("#ff4444", dark_bg),
        # 错误：亮橙色文字
        'ERROR': ("#ff6600", dark_bg),
        # 警告：亮黄色文字
        'WARNING': ("#ffcc00", dark_bg),
        # 通知：亮蓝色文字
        'NOTICE': ("#4da6ff", dark_bg),
        # 信息：浅灰色文字
        'INFO': ("#cccccc", dark_bg),
        # 调试：暗灰色文字
        'DEBUG': ("#888888", dark_bg),
    }
    return severity_map.get(severity.upper(), ("#cccccc", dark_bg))
