"""
Area Center 偏移量设置对话框
用于设置任务坐标系原点（area_center）在全局地图坐标系中的位置
"""

from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                              QDoubleSpinBox, QPushButton, QGroupBox, QMessageBox)


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
        self.parent_window = parent  # 保存父窗口引用，用于获取USV位置
        self.setWindowTitle("设置任务坐标系原点偏移量")
        self.setModal(True)
        self.resize(400, 280)
        
        # 初始化偏移量
        if current_offset is None:
            current_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        self.offset_x = current_offset.get('x', 0.0)
        self.offset_y = current_offset.get('y', 0.0)
        self.offset_z = current_offset.get('z', 0.0)
        
        # 创建UI
        self._setup_ui()
        
        # 窗口居中显示
        self._center_on_screen()

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
        info_label.setStyleSheet("color: #555; font-size: 14pt; padding: 10px;")
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
        
        # 获取当前USV位置按钮
        get_position_button = QPushButton("获取选中USV位置")
        get_position_button.setToolTip("将当前选中USV的位置自动填充到偏移量坐标栏")
        get_position_button.clicked.connect(self._get_usv_position)
        button_layout.addWidget(get_position_button)
        
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
    
    def _get_usv_position(self):
        """从父窗口获取当前选中USV的位置并填充到坐标栏"""
        if self.parent_window is None:
            QMessageBox.warning(self, "警告", "无法访问主窗口")
            return
        
        # 检查父窗口是否有获取USV位置的方法
        if not hasattr(self.parent_window, 'get_selected_usv_position'):
            QMessageBox.warning(self, "警告", "主窗口不支持获取USV位置")
            return
        
        # 调用父窗口的方法获取选中USV的位置
        position = self.parent_window.get_selected_usv_position()
        
        if position is None:
            QMessageBox.warning(self, "警告", "未选中任何USV或无法获取位置信息\n请先选择一个USV")
            return
        
        # 将位置填充到坐标输入框
        self.x_spinbox.setValue(position.get('x', 0.0))
        self.y_spinbox.setValue(position.get('y', 0.0))
        self.z_spinbox.setValue(position.get('z', 0.0))
        
        # 显示成功提示
        usv_id = position.get('usv_id', 'Unknown')
        QMessageBox.information(
            self, 
            "成功", 
            f"已获取 {usv_id} 的位置:\n"
            f"X: {position.get('x', 0.0):.2f} m\n"
            f"Y: {position.get('y', 0.0):.2f} m\n"
            f"Z: {position.get('z', 0.0):.2f} m"
        )
        
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


    def _center_on_screen(self):
        """将窗口居中显示在屏幕上"""
        from PyQt5.QtWidgets import QApplication
        screen = QApplication.desktop().screenGeometry()
        size = self.geometry()
        self.move(
            (screen.width() - size.width()) // 2,
            (screen.height() - size.height()) // 2
        )
