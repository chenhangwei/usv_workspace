"""
参数管理串口连接对话框

用于配置和建立与飞控的串口连接。
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QComboBox, QGroupBox, QFormLayout, QMessageBox
)
from PyQt5.QtCore import Qt
import glob
import serial.tools.list_ports


class ParamConnectionDialog(QDialog):
    """串口连接配置对话框"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.port = None
        self.baudrate = 115200
        self.system_id = 1
        self.component_id = 1
        
        self._setup_ui()
        self._load_available_ports()
    
    def _setup_ui(self):
        """设置 UI"""
        self.setWindowTitle("连接到飞控（串口）")
        self.setMinimumWidth(450)
        
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # ==================== 串口配置 ====================
        serial_group = QGroupBox("串口配置")
        serial_layout = QFormLayout()
        serial_layout.setSpacing(10)
        
        # 串口选择
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(250)
        serial_layout.addRow("串口设备:", self.port_combo)
        
        # 刷新按钮
        refresh_btn = QPushButton("🔄 刷新")
        refresh_btn.clicked.connect(self._load_available_ports)
        refresh_btn.setMaximumWidth(80)
        serial_layout.addRow("", refresh_btn)
        
        # 波特率选择
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems([
            "57600",
            "115200",
            "230400",
            "460800",
            "921600"
        ])
        self.baudrate_combo.setCurrentText("115200")
        serial_layout.addRow("波特率:", self.baudrate_combo)
        
        serial_group.setLayout(serial_layout)
        layout.addWidget(serial_group)
        
        # ==================== MAVLink 配置 ====================
        mavlink_group = QGroupBox("MAVLink 配置")
        mavlink_layout = QFormLayout()
        mavlink_layout.setSpacing(10)
        
        # 系统 ID
        self.system_id_edit = QLineEdit("1")
        self.system_id_edit.setMaximumWidth(100)
        mavlink_layout.addRow("目标系统 ID:", self.system_id_edit)
        
        # 组件 ID
        self.component_id_edit = QLineEdit("1")
        self.component_id_edit.setMaximumWidth(100)
        mavlink_layout.addRow("目标组件 ID:", self.component_id_edit)
        
        mavlink_group.setLayout(mavlink_layout)
        layout.addWidget(mavlink_group)
        
        # ==================== 提示信息 ====================
        info_label = QLabel(
            "💡 提示：\n"
            "• 确保飞控已通过 USB 连接到计算机\n"
            "• 推荐使用 115200 波特率（稳定性最佳）\n"
            "• 系统 ID 通常为 1（与飞控 SYSID_THISMAV 一致）"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #555; font-size: 10pt;")
        layout.addWidget(info_label)
        
        # ==================== 按钮 ====================
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        cancel_btn = QPushButton("取消")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(cancel_btn)
        
        connect_btn = QPushButton("连接")
        connect_btn.setDefault(True)
        connect_btn.clicked.connect(self._on_connect)
        button_layout.addWidget(connect_btn)
        
        layout.addLayout(button_layout)
    
    def _load_available_ports(self):
        """加载可用串口列表"""
        self.port_combo.clear()
        
        # 使用 pyserial 扫描串口
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            self.port_combo.addItem("未检测到串口设备")
            return
        
        for port in ports:
            # 显示格式: /dev/ttyACM0 - USB Serial Device
            display_text = f"{port.device}"
            if port.description and port.description != "n/a":
                display_text += f" - {port.description}"
            
            self.port_combo.addItem(display_text, port.device)
    
    def _on_connect(self):
        """连接按钮点击处理"""
        # 验证输入
        if self.port_combo.count() == 0 or self.port_combo.currentText() == "未检测到串口设备":
            QMessageBox.warning(self, "错误", "请先选择有效的串口设备")
            return
        
        try:
            self.system_id = int(self.system_id_edit.text())
            self.component_id = int(self.component_id_edit.text())
        except ValueError:
            QMessageBox.warning(self, "错误", "系统 ID 和组件 ID 必须是整数")
            return
        
        # 获取选中的串口
        self.port = self.port_combo.currentData()
        if not self.port:
            self.port = self.port_combo.currentText().split(' - ')[0]
        
        self.baudrate = int(self.baudrate_combo.currentText())
        
        self.accept()
    
    def get_connection_params(self):
        """获取连接参数"""
        return {
            'port': self.port,
            'baudrate': self.baudrate,
            'system_id': self.system_id,
            'component_id': self.component_id
        }
