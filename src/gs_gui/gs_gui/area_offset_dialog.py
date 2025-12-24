"""
Area Center & Fence 偏移量与围栏设置对话框
用于设置任务坐标系原点（area_center）以及随机运行的围栏范围
"""

from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                              QDoubleSpinBox, QPushButton, QGroupBox, QMessageBox,
                              QComboBox, QStackedWidget, QWidget)
from PyQt5.QtCore import Qt


class AreaOffsetDialog(QDialog):
    """Area Center & Fence 设置对话框"""
    
    def __init__(self, parent=None, current_offset=None, current_fence=None):
        """
        初始化对话框
        
        Args:
            parent: 父窗口
            current_offset: 当前偏移量 {'x': float, 'y': float, 'z': float}
            current_fence: 当前围栏配置 {'type': int, ...}
        """
        super().__init__(parent)
        self.parent_window = parent
        self.setWindowTitle("任务区域与围栏设置")
        self.setModal(True)
        self.resize(450, 500)
        
        # 初始化偏移量
        if current_offset is None:
            current_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.offset_x = current_offset.get('x', 0.0)
        self.offset_y = current_offset.get('y', 0.0)
        self.offset_z = current_offset.get('z', 0.0)
        
        # 初始化围栏
        if current_fence is None:
            current_fence = {
                'type': 0, 
                'radius': 50.0, 
                'length': 100.0, 
                'width': 100.0, 
                'height': 10.0
            }
        self.fence_config = current_fence
        
        self._setup_ui()
        self._center_on_screen()

    def _setup_ui(self):
        """设置用户界面"""
        layout = QVBoxLayout(self)
        
        # 说明标签
        info_label = QLabel(
            "配置任务区域原点及随机运行围栏范围：\n"
            "• Area Center: 任务坐标系原点在全局 Map 中的位置\n"
            "• Fence: 随机运行模式下的活动范围约束"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #555; font-size: 12pt; padding: 5px;")
        layout.addWidget(info_label)
        
        # 1. Area Center 坐标设置
        coord_group = QGroupBox("任务坐标系原点 (Area Center)")
        coord_layout = QVBoxLayout()
        
        self.x_spinbox = self._create_spinbox(self.offset_x)
        self.y_spinbox = self._create_spinbox(self.offset_y)
        self.z_spinbox = self._create_spinbox(self.offset_z, -1000, 1000)
        
        coord_layout.addLayout(self._create_row("X 偏移量:", self.x_spinbox))
        coord_layout.addLayout(self._create_row("Y 偏移量:", self.y_spinbox))
        coord_layout.addLayout(self._create_row("Z 偏移量:", self.z_spinbox))
        
        # 获取位置按钮
        get_pos_btn = QPushButton("获取选中 USV 位置")
        get_pos_btn.setToolTip("将当前选中 USV 的位置自动填充到偏移量坐标栏")
        get_pos_btn.clicked.connect(self._get_usv_position)
        coord_layout.addWidget(get_pos_btn)
        
        coord_group.setLayout(coord_layout)
        layout.addWidget(coord_group)
        
        # 2. Fence 围栏设置
        fence_group = QGroupBox("随机运行围栏设置 (Fence)")
        fence_layout = QVBoxLayout()
        
        type_layout = QHBoxLayout()
        type_layout.addWidget(QLabel("围栏形状:"))
        self.type_combo = QComboBox()
        self.type_combo.addItems(["圆柱形 (Cylinder)", "长方体 (Box)"])
        self.type_combo.setCurrentIndex(self.fence_config.get('type', 0))
        self.type_combo.currentIndexChanged.connect(self._on_type_changed)
        type_layout.addWidget(self.type_combo)
        fence_layout.addLayout(type_layout)
        
        # Stacked widget for different shapes
        self.fence_stack = QStackedWidget()
        
        # Cylinder page
        self.cylinder_page = QWidget()
        cyl_layout = QVBoxLayout(self.cylinder_page)
        self.radius_spinbox = self._create_spinbox(self.fence_config.get('radius', 50.0), 1, 5000)
        self.cyl_height_spinbox = self._create_spinbox(self.fence_config.get('height', 10.0), 1, 1000)
        cyl_layout.addLayout(self._create_row("半径 (Radius):", self.radius_spinbox))
        cyl_layout.addLayout(self._create_row("高度 (Height):", self.cyl_height_spinbox))
        self.fence_stack.addWidget(self.cylinder_page)
        
        # Box page
        self.box_page = QWidget()
        box_layout = QVBoxLayout(self.box_page)
        self.length_spinbox = self._create_spinbox(self.fence_config.get('length', 100.0), 1, 5000)
        self.width_spinbox = self._create_spinbox(self.fence_config.get('width', 100.0), 1, 5000)
        self.box_height_spinbox = self._create_spinbox(self.fence_config.get('height', 10.0), 1, 1000)
        box_layout.addLayout(self._create_row("长度 (X):", self.length_spinbox))
        box_layout.addLayout(self._create_row("宽度 (Y):", self.width_spinbox))
        box_layout.addLayout(self._create_row("高度 (Z):", self.box_height_spinbox))
        self.fence_stack.addWidget(self.box_page)
        
        self.fence_stack.setCurrentIndex(self.type_combo.currentIndex())
        fence_layout.addWidget(self.fence_stack)
        
        fence_group.setLayout(fence_layout)
        layout.addWidget(fence_group)
        
        # 3. Buttons
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        ok_btn = QPushButton("确定")
        ok_btn.setDefault(True)
        ok_btn.clicked.connect(self.accept)
        cancel_btn = QPushButton("取消")
        cancel_btn.clicked.connect(self.reject)
        btn_layout.addWidget(ok_btn)
        btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)

    def _create_spinbox(self, value, min_val=-10000, max_val=10000):
        sb = QDoubleSpinBox()
        sb.setRange(min_val, max_val)
        sb.setDecimals(2)
        sb.setValue(value)
        sb.setSuffix(" m")
        return sb

    def _create_row(self, label_text, widget):
        row = QHBoxLayout()
        lbl = QLabel(label_text)
        lbl.setMinimumWidth(120)
        row.addWidget(lbl)
        row.addWidget(widget)
        return row

    def _on_type_changed(self, index):
        self.fence_stack.setCurrentIndex(index)

    def _get_usv_position(self):
        """从父窗口获取当前选中 USV 的位置"""
        if self.parent_window and hasattr(self.parent_window, 'get_selected_usv_position'):
            pos = self.parent_window.get_selected_usv_position()
            if pos:
                self.x_spinbox.setValue(pos.get('x', 0.0))
                self.y_spinbox.setValue(pos.get('y', 0.0))
                self.z_spinbox.setValue(pos.get('z', 0.0))
                QMessageBox.information(self, "成功", f"已获取 {pos.get('usv_id')} 的位置")
            else:
                QMessageBox.warning(self, "警告", "未选中任何 USV 或无法获取位置信息")

    def get_config(self):
        """获取配置结果"""
        offset = {
            'x': self.x_spinbox.value(),
            'y': self.y_spinbox.value(),
            'z': self.z_spinbox.value()
        }
        fence_type = self.type_combo.currentIndex()
        if fence_type == 0:
            fence = {
                'type': 0,
                'radius': self.radius_spinbox.value(),
                'height': self.cyl_height_spinbox.value()
            }
        else:
            fence = {
                'type': 1,
                'length': self.length_spinbox.value(),
                'width': self.width_spinbox.value(),
                'height': self.box_height_spinbox.value()
            }
        return offset, fence

    def get_offset(self):
        """兼容旧接口"""
        return {
            'x': self.x_spinbox.value(),
            'y': self.y_spinbox.value(),
            'z': self.z_spinbox.value()
        }

    def _center_on_screen(self):
        from PyQt5.QtWidgets import QApplication
        screen = QApplication.desktop().screenGeometry()
        size = self.geometry()
        self.move((screen.width() - size.width()) // 2, (screen.height() - size.height()) // 2)
