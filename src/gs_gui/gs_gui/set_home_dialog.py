"""
Home Position 设置对话框
使用局部坐标系 (x, y, z)
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QRadioButton, QButtonGroup, QComboBox,
    QGroupBox, QMessageBox
)


class SetHomeDialog(QDialog):
    """设置 Home Position 对话框（局部坐标系版本）"""
    
    def __init__(self, usv_list, parent=None):
        """
        初始化对话框
        
        Args:
            usv_list: 在线 USV 列表（字典列表，包含 namespace 字段）
            parent: 父窗口
        """
        super().__init__(parent)
        self.setWindowTitle("设置 Home Position")
        self.resize(500, 320)
        
        # 存储 USV 列表
        self.usv_list = usv_list
        
        # 初始化 UI
        self._init_ui()
        
        # 默认坐标（原点）
        self.default_x = 0.0
        self.default_y = 0.0
        self.default_z = 0.0
        
        # 设置默认值
        self._set_default_coords()
    
    def _init_ui(self):
        """初始化 UI 组件"""
        layout = QVBoxLayout(self)
        
        # 说明文本
        info_label = QLabel(
            "Home Position 用于 RTL（返航）和 Failsafe（失联保护）功能。\n"
            "使用局部坐标系 (ENU: 东-北-天)，单位为米。"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #888; font-size: 11px; padding: 10px; background-color: #2a2a2a; border-radius: 5px;")
        layout.addWidget(info_label)
        
        # USV 选择
        usv_group = QGroupBox("选择 USV")
        usv_layout = QVBoxLayout(usv_group)
        
        self.usv_combo = QComboBox()
        self.usv_combo.setStyleSheet("font-size: 12px; padding: 5px;")
        
        if self.usv_list:
            for usv in self.usv_list:
                ns = usv.get('namespace', '')
                if ns:
                    self.usv_combo.addItem(ns)
        else:
            self.usv_combo.addItem("（无在线 USV）")
            self.usv_combo.setEnabled(False)
        
        usv_layout.addWidget(self.usv_combo)
        layout.addWidget(usv_group)
        
        # 坐标来源选择
        coord_group = QGroupBox("坐标来源")
        coord_layout = QVBoxLayout(coord_group)
        
        self.button_group = QButtonGroup()
        
        self.radio_current = QRadioButton("使用 USV 当前位置")
        self.radio_current.setToolTip("将 Home Position 设置为 USV 当前局部坐标位置")
        self.button_group.addButton(self.radio_current)
        coord_layout.addWidget(self.radio_current)
        
        self.radio_custom = QRadioButton("指定坐标")
        self.radio_custom.setToolTip("手动输入 Home Position 局部坐标")
        self.button_group.addButton(self.radio_custom)
        coord_layout.addWidget(self.radio_custom)
        
        # 默认选择"使用当前位置"
        self.radio_current.setChecked(True)
        
        # 连接信号
        self.radio_current.toggled.connect(self._on_mode_changed)
        self.radio_custom.toggled.connect(self._on_mode_changed)
        
        layout.addWidget(coord_group)
        
        # 坐标输入（局部坐标系）
        coords_group = QGroupBox("坐标输入（局部坐标系 ENU）")
        coords_layout = QVBoxLayout(coords_group)
        
        # X 坐标（东向）
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X (m):"))
        self.x_input = QLineEdit()
        self.x_input.setPlaceholderText("东向位置，例如: 0.0")
        self.x_input.setToolTip("东向位置（米），正值向东")
        x_layout.addWidget(self.x_input)
        coords_layout.addLayout(x_layout)
        
        # Y 坐标（北向）
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y (m):"))
        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("北向位置，例如: 0.0")
        self.y_input.setToolTip("北向位置（米），正值向北")
        y_layout.addWidget(self.y_input)
        coords_layout.addLayout(y_layout)
        
        # Z 坐标（高度）
        z_layout = QHBoxLayout()
        z_layout.addWidget(QLabel("Z (m):"))
        self.z_input = QLineEdit()
        self.z_input.setPlaceholderText("高度，例如: 0.0")
        self.z_input.setToolTip("高度（米），正值向上")
        z_layout.addWidget(self.z_input)
        coords_layout.addLayout(z_layout)
        
        # 快捷按钮
        shortcut_layout = QHBoxLayout()
        self.btn_origin = QPushButton("使用原点坐标 (0, 0, 0)")
        self.btn_origin.setToolTip("快速填入原点坐标")
        self.btn_origin.clicked.connect(self._set_default_coords)
        shortcut_layout.addWidget(self.btn_origin)
        coords_layout.addLayout(shortcut_layout)
        
        layout.addWidget(coords_group)
        self.coords_group = coords_group
        
        # 初始状态：禁用坐标输入
        self._on_mode_changed()
        
        # 按钮
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        self.btn_ok = QPushButton("确定")
        self.btn_ok.setStyleSheet("background-color: #4a90e2; color: white; font-weight: bold; padding: 8px 20px;")
        self.btn_ok.clicked.connect(self.accept)
        button_layout.addWidget(self.btn_ok)
        
        self.btn_cancel = QPushButton("取消")
        self.btn_cancel.setStyleSheet("padding: 8px 20px;")
        self.btn_cancel.clicked.connect(self.reject)
        button_layout.addWidget(self.btn_cancel)
        
        layout.addLayout(button_layout)
    
    def _on_mode_changed(self):
        """切换坐标来源时的回调"""
        use_current = self.radio_current.isChecked()
        
        # 禁用/启用坐标输入
        self.coords_group.setEnabled(not use_current)
        self.x_input.setEnabled(not use_current)
        self.y_input.setEnabled(not use_current)
        self.z_input.setEnabled(not use_current)
        self.btn_origin.setEnabled(not use_current)
    
    def _set_default_coords(self):
        """设置默认坐标（原点）"""
        self.x_input.setText(str(self.default_x))
        self.y_input.setText(str(self.default_y))
        self.z_input.setText(str(self.default_z))
    
    def get_result(self):
        """
        获取对话框结果
        
        Returns:
            tuple: (usv_namespace, use_current, coords)
                - usv_namespace: str, USV 命名空间
                - use_current: bool, 是否使用当前位置
                - coords: dict, 局部坐标字典 {'x': float, 'y': float, 'z': float}
        """
        usv_namespace = self.usv_combo.currentText()
        use_current = self.radio_current.isChecked()
        
        coords = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        }
        
        if not use_current:
            try:
                coords['x'] = float(self.x_input.text())
                coords['y'] = float(self.y_input.text())
                coords['z'] = float(self.z_input.text())
            except ValueError as e:
                QMessageBox.warning(
                    self,
                    "输入错误",
                    f"坐标输入格式错误，请输入有效的数字。\n错误: {e}"
                )
                return None, None, None
        
        return usv_namespace, use_current, coords
    
    def accept(self):
        """确定按钮回调"""
        if not self.usv_list:
            QMessageBox.warning(self, "警告", "没有在线的 USV，无法设置 Home Position。")
            return
        
        usv_namespace, use_current, coords = self.get_result()
        
        if usv_namespace is None:
            return  # 验证失败
        
        # 验证坐标范围（局部坐标一般不需要严格限制，但可以设置合理范围）
        if not use_current:
            max_range = 10000.0  # 10公里范围
            if abs(coords['x']) > max_range or abs(coords['y']) > max_range:
                QMessageBox.warning(
                    self, 
                    "输入警告", 
                    f"坐标值超出 ±{max_range}m 范围，请确认输入是否正确。"
                )
                # 这里只是警告，不阻止确认
        
        super().accept()

