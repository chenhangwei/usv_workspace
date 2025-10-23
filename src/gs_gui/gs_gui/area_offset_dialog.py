"""
Area Center 偏移量设置对话框
用于设置任务坐标系原点（area_center）在全局地图坐标系中的位置
"""

from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                              QDoubleSpinBox, QPushButton, QGroupBox, QMessageBox)
from PyQt5.QtCore import Qt


class AreaOffsetDialog(QDialog):
    """Area Center 偏移量设置对话框"""
    
    def __init__(self, parent=None, current_offset=None):
        """
        初始化对话框
        
        Args:
            parent: 父窗口
            current_offset: 当前偏移量 {'x': float, 'y': float, 'z': float}
        """
        super().__init__(parent)
        self.setWindowTitle("设置任务坐标系原点偏移量")
        self.setModal(True)
        self.resize(400, 250)
        
        # 初始化偏移量
        if current_offset is None:
            current_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        self.offset_x = current_offset.get('x', 0.0)
        self.offset_y = current_offset.get('y', 0.0)
        self.offset_z = current_offset.get('z', 0.0)
        
        # 创建UI
        self._setup_ui()
        
    def _setup_ui(self):
        """设置用户界面"""
        layout = QVBoxLayout(self)
        
        # 说明标签
        info_label = QLabel(
            "设置任务坐标系原点（Area Center）在全局地图坐标系中的位置：\n"
            "• 任务文件中的坐标是相对于Area Center的\n"
            "• 该偏移量将Area坐标转换为全局Map坐标\n"
            "• USV会自动转换为本地坐标执行"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #555; font-size: 10pt; padding: 10px;")
        layout.addWidget(info_label)
        
        # 坐标输入组
        coord_group = QGroupBox("偏移量坐标 (单位: 米)")
        coord_layout = QVBoxLayout()
        
        # X坐标
        x_layout = QHBoxLayout()
        x_label = QLabel("X 偏移量:")
        x_label.setMinimumWidth(80)
        self.x_spinbox = QDoubleSpinBox()
        self.x_spinbox.setRange(-10000.0, 10000.0)
        self.x_spinbox.setDecimals(2)
        self.x_spinbox.setValue(self.offset_x)
        self.x_spinbox.setSuffix(" m")
        x_layout.addWidget(x_label)
        x_layout.addWidget(self.x_spinbox)
        coord_layout.addLayout(x_layout)
        
        # Y坐标
        y_layout = QHBoxLayout()
        y_label = QLabel("Y 偏移量:")
        y_label.setMinimumWidth(80)
        self.y_spinbox = QDoubleSpinBox()
        self.y_spinbox.setRange(-10000.0, 10000.0)
        self.y_spinbox.setDecimals(2)
        self.y_spinbox.setValue(self.offset_y)
        self.y_spinbox.setSuffix(" m")
        y_layout.addWidget(y_label)
        y_layout.addWidget(self.y_spinbox)
        coord_layout.addLayout(y_layout)
        
        # Z坐标
        z_layout = QHBoxLayout()
        z_label = QLabel("Z 偏移量:")
        z_label.setMinimumWidth(80)
        self.z_spinbox = QDoubleSpinBox()
        self.z_spinbox.setRange(-1000.0, 1000.0)
        self.z_spinbox.setDecimals(2)
        self.z_spinbox.setValue(self.offset_z)
        self.z_spinbox.setSuffix(" m")
        z_layout.addWidget(z_label)
        z_layout.addWidget(self.z_spinbox)
        coord_layout.addLayout(z_layout)
        
        coord_group.setLayout(coord_layout)
        layout.addWidget(coord_group)
        
        # 按钮组
        button_layout = QHBoxLayout()
        
        # 重置按钮
        reset_button = QPushButton("重置为0")
        reset_button.clicked.connect(self._reset_values)
        button_layout.addWidget(reset_button)
        
        button_layout.addStretch()
        
        # 确定按钮
        ok_button = QPushButton("确定")
        ok_button.setDefault(True)
        ok_button.clicked.connect(self.accept)
        button_layout.addWidget(ok_button)
        
        # 取消按钮
        cancel_button = QPushButton("取消")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        layout.addLayout(button_layout)
        
    def _reset_values(self):
        """重置所有值为0"""
        self.x_spinbox.setValue(0.0)
        self.y_spinbox.setValue(0.0)
        self.z_spinbox.setValue(0.0)
        
    def get_offset(self):
        """
        获取用户设置的偏移量
        
        Returns:
            dict: {'x': float, 'y': float, 'z': float}
        """
        return {
            'x': self.x_spinbox.value(),
            'y': self.y_spinbox.value(),
            'z': self.z_spinbox.value()
        }
