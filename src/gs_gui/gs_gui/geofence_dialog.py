from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                             QDoubleSpinBox, QCheckBox, QPushButton, QGroupBox)
from PyQt5.QtCore import Qt

class GeofenceDialog(QDialog):
    def __init__(self, parent=None, current_bounds=None, current_enabled=False):
        super().__init__(parent)
        self.setWindowTitle("电子围栏设置")
        self.current_bounds = current_bounds or {'x_min': -50.0, 'x_max': 50.0, 'y_min': -50.0, 'y_max': 50.0}
        self.current_enabled = current_enabled
        self.resize(400, 300)
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout()
        
        # 说明
        info_label = QLabel("设置矩形电子围栏。\n当USV超出此范围时，将自动切换到HOLD模式。")
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #AAAAAA; margin-bottom: 10px;")
        layout.addWidget(info_label)

        # 启用开关
        self.enable_checkbox = QCheckBox("启用电子围栏监控")
        self.enable_checkbox.setChecked(self.current_enabled)
        self.enable_checkbox.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self.enable_checkbox)
        
        # 边界设置
        group_box = QGroupBox("矩形边界 (坐标系: Local Frame / 米)")
        grid = QVBoxLayout()
        
        # X Range
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X Min:"))
        self.x_min_spin = QDoubleSpinBox()
        self.x_min_spin.setRange(-10000.0, 10000.0)
        self.x_min_spin.setValue(float(self.current_bounds['x_min']))
        self.x_min_spin.setSingleStep(10.0)
        x_layout.addWidget(self.x_min_spin)
        
        x_layout.addWidget(QLabel("X Max:"))
        self.x_max_spin = QDoubleSpinBox()
        self.x_max_spin.setRange(-10000.0, 10000.0)
        self.x_max_spin.setValue(float(self.current_bounds['x_max']))
        self.x_max_spin.setSingleStep(10.0)
        x_layout.addWidget(self.x_max_spin)
        grid.addLayout(x_layout)
        
        # Y Range
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y Min:"))
        self.y_min_spin = QDoubleSpinBox()
        self.y_min_spin.setRange(-10000.0, 10000.0)
        self.y_min_spin.setValue(float(self.current_bounds['y_min']))
        self.y_min_spin.setSingleStep(10.0)
        y_layout.addWidget(self.y_min_spin)
        
        y_layout.addWidget(QLabel("Y Max:"))
        self.y_max_spin = QDoubleSpinBox()
        self.y_max_spin.setRange(-10000.0, 10000.0)
        self.y_max_spin.setValue(float(self.current_bounds['y_max']))
        self.y_max_spin.setSingleStep(10.0)
        y_layout.addWidget(self.y_max_spin)
        grid.addLayout(y_layout)
        
        group_box.setLayout(grid)
        layout.addWidget(group_box)
        
        layout.addStretch()
        
        # Buttons
        btn_layout = QHBoxLayout()
        self.ok_btn = QPushButton("确定")
        self.ok_btn.clicked.connect(self.accept)
        self.ok_btn.setStyleSheet("background-color: #2D5887; color: white; padding: 6px;")
        self.cancel_btn = QPushButton("取消")
        self.cancel_btn.clicked.connect(self.reject)
        
        btn_layout.addWidget(self.cancel_btn)
        btn_layout.addWidget(self.ok_btn)
        layout.addLayout(btn_layout)
        
        self.setLayout(layout)
        
    def get_settings(self):
        bounds = {
            'x_min': self.x_min_spin.value(),
            'x_max': self.x_max_spin.value(),
            'y_min': self.y_min_spin.value(),
            'y_max': self.y_max_spin.value()
        }
        enabled = self.enable_checkbox.isChecked()
        return bounds, enabled
