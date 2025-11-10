"""
Home Position 设置对话框
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QRadioButton, QButtonGroup, QComboBox,
    QGroupBox, QMessageBox
)
from PyQt5.QtCore import Qt


class SetHomeDialog(QDialog):
    """设置 Home Position 对话框"""
    
    def __init__(self, usv_list, parent=None):
        """
        初始化对话框
        
        Args:
            usv_list: 在线 USV 列表（字典列表，包含 namespace 字段）
            parent: 父窗口
        """
        super().__init__(parent)
        self.setWindowTitle("设置 Home Position")
        self.resize(500, 350)
        
        # 存储 USV 列表
        self.usv_list = usv_list
        
        # 初始化 UI
        self._init_ui()
        
        # 默认坐标（A0 基站）
        self.default_lat = 22.5180977
        self.default_lon = 113.9007239
        self.default_alt = 0.0  # 修改为与 usv_params.yaml 一致
        
        # 设置默认值
        self._set_default_coords()
    
    def _init_ui(self):
        """初始化 UI 组件"""
        layout = QVBoxLayout(self)
        
        # 说明文本
        info_label = QLabel(
            "Home Position 用于 RTL（返航）和 Failsafe（失联保护）功能。\n"
            "与 EKF Origin 不同，Home Position 可以在任务执行中动态修改。"
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
        self.radio_current.setToolTip("将 Home Position 设置为 USV 当前 GPS 位置")
        self.button_group.addButton(self.radio_current)
        coord_layout.addWidget(self.radio_current)
        
        self.radio_custom = QRadioButton("指定坐标")
        self.radio_custom.setToolTip("手动输入 Home Position 坐标")
        self.button_group.addButton(self.radio_custom)
        coord_layout.addWidget(self.radio_custom)
        
        # 默认选择"使用当前位置"
        self.radio_current.setChecked(True)
        
        # 连接信号
        self.radio_current.toggled.connect(self._on_mode_changed)
        self.radio_custom.toggled.connect(self._on_mode_changed)
        
        layout.addWidget(coord_group)
        
        # 坐标输入
        coords_group = QGroupBox("坐标输入")
        coords_layout = QVBoxLayout(coords_group)
        
        # 纬度
        lat_layout = QHBoxLayout()
        lat_layout.addWidget(QLabel("纬度 (°):"))
        self.lat_input = QLineEdit()
        self.lat_input.setPlaceholderText("例如: 22.5180977")
        self.lat_input.setToolTip("北纬为正，南纬为负")
        lat_layout.addWidget(self.lat_input)
        coords_layout.addLayout(lat_layout)
        
        # 经度
        lon_layout = QHBoxLayout()
        lon_layout.addWidget(QLabel("经度 (°):"))
        self.lon_input = QLineEdit()
        self.lon_input.setPlaceholderText("例如: 113.9007239")
        self.lon_input.setToolTip("东经为正，西经为负")
        lon_layout.addWidget(self.lon_input)
        coords_layout.addLayout(lon_layout)
        
        # 高度
        alt_layout = QHBoxLayout()
        alt_layout.addWidget(QLabel("高度 (m):"))
        self.alt_input = QLineEdit()
        self.alt_input.setPlaceholderText("例如: 0.0")
        self.alt_input.setToolTip("相对于海平面的高度（米）")
        alt_layout.addWidget(self.alt_input)
        coords_layout.addLayout(alt_layout)
        
        # 快捷按钮
        shortcut_layout = QHBoxLayout()
        self.btn_a0 = QPushButton("使用 A0 基站坐标")
        self.btn_a0.setToolTip("快速填入 A0 基站坐标 (22.5180977, 113.9007239, 0.0)")
        self.btn_a0.clicked.connect(self._set_default_coords)
        shortcut_layout.addWidget(self.btn_a0)
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
        self.lat_input.setEnabled(not use_current)
        self.lon_input.setEnabled(not use_current)
        self.alt_input.setEnabled(not use_current)
        self.btn_a0.setEnabled(not use_current)
    
    def _set_default_coords(self):
        """设置默认坐标（A0 基站）"""
        self.lat_input.setText(str(self.default_lat))
        self.lon_input.setText(str(self.default_lon))
        self.alt_input.setText(str(self.default_alt))
    
    def get_result(self):
        """
        获取对话框结果
        
        Returns:
            tuple: (usv_namespace, use_current, coords)
                - usv_namespace: str, USV 命名空间
                - use_current: bool, 是否使用当前位置
                - coords: dict, 坐标字典 {'lat': float, 'lon': float, 'alt': float}
        """
        usv_namespace = self.usv_combo.currentText()
        use_current = self.radio_current.isChecked()
        
        coords = {
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0
        }
        
        if not use_current:
            try:
                coords['lat'] = float(self.lat_input.text())
                coords['lon'] = float(self.lon_input.text())
                coords['alt'] = float(self.alt_input.text())
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
        
        # 验证坐标范围
        if not use_current:
            if not (-90 <= coords['lat'] <= 90):
                QMessageBox.warning(self, "输入错误", "纬度必须在 -90° 到 90° 之间。")
                return
            if not (-180 <= coords['lon'] <= 180):
                QMessageBox.warning(self, "输入错误", "经度必须在 -180° 到 180° 之间。")
                return
        
        super().accept()
