"""
USV 信息面板模块
提供美观、信息丰富的 USV 详细信息显示界面

该模块使用以下子模块：
- info_panel_widgets: 通用 UI 组件和工具函数
- info_panel_styles: 样式更新函数
"""

import logging
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                              QGroupBox, QGridLayout, QFrame, QProgressBar,
                              QScrollArea, QSizePolicy, QPushButton,
                              QListWidget, QListWidgetItem, QAbstractItemView,
                              QMenu, QApplication, QCheckBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor

_logger = logging.getLogger("gs_gui.info_panel")


# 导入子模块
from .info_panel_widgets import (
    GROUPBOX_STYLE,
    create_key_label, create_value_label, create_section_label,
    configure_list_widget, set_list_placeholder, apply_button_style,
    format_float, level_to_palette, severity_palette,
    AlignRight, AlignLeft, AlignVCenter
)
from .info_panel_styles import (
    update_mode_style, update_status_style, update_armed_style,
    update_battery_style, get_temperature_style
)


class UsvInfoPanel(QWidget):
    """
    USV 信息面板（响应式设计）
    
    提供美观的 USV 详细信息显示，包括：
    - 基本信息（ID、模式、状态）
    - 位置信息（X, Y, Z, Yaw）
    - 电池信息（电压、百分比）
    - GPS 信息（卫星数、精度）
    - 速度信息（地速、航速）
    
    特性：
    - 滚动条支持：内容超出时自动显示滚动条
    - 响应式布局：小窗口下自动调整字体和间距
    - 弹性设计：避免内容被压扁
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 温度状态跟踪（用于实现滞后效果）
        self._is_high_temperature = False  # False=低温(绿色), True=高温(红色)
        
        # 当前状态缓存
        self._current_state = None
        
        # 重入保护标志，防止 update_state 在执行过程中被重复调用
        self._is_updating = False
        
        # 设置主布局（包含滚动区域）
        self._setup_ui()
        
        # 更新定时器（用于动态效果）
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._update_dynamic_styles)
        self._update_timer.start(1000)  # 每秒更新一次
    
    def _setup_ui(self):
        """设置UI布局（带滚动条）"""
        # 主容器布局（外层）
        main_container_layout = QVBoxLayout(self)
        main_container_layout.setContentsMargins(0, 0, 0, 0)
        main_container_layout.setSpacing(0)
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)  # 自动调整内容大小
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)  # 无边框
        
        # 创建滚动内容容器
        scroll_content = QWidget()
        scroll_content.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        
        # 内容布局
        content_layout = QVBoxLayout(scroll_content)
        content_layout.setContentsMargins(10, 10, 10, 10)
        content_layout.setSpacing(10)
        
        # ==================== 基本信息组 ====================
        basic_group = self._create_basic_info_group()
        content_layout.addWidget(basic_group)

        # ==================== 位置信息组 ====================
        position_group = self._create_position_info_group()
        content_layout.addWidget(position_group)
        
        # ==================== 电池信息组 ====================
        battery_group = self._create_battery_info_group()
        content_layout.addWidget(battery_group)
        
        # ==================== Ready 状态组 ====================
        readiness_group = self._create_readiness_group()
        content_layout.addWidget(readiness_group)

        # ==================== 飞控消息组 ====================
        messages_group = self._create_vehicle_message_group()
        content_layout.addWidget(messages_group)
        
        # 添加弹性空间（自动填充剩余空间）
        content_layout.addStretch()
        
        # 将内容容器设置到滚动区域
        scroll_area.setWidget(scroll_content)
        
        # 将滚动区域添加到主布局
        main_container_layout.addWidget(scroll_area)
        
        # 设置滚动条样式
        scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QScrollBar:vertical {
                border: none;
                background: #2c3e50;
                width: 10px;
                margin: 0px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background: #3498db;
                min-height: 30px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical:hover {
                background: #5dade2;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                border: none;
                background: none;
                height: 0px;
            }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
            }
            
            QScrollBar:horizontal {
                border: none;
                background: #2c3e50;
                height: 10px;
                margin: 0px;
                border-radius: 5px;
            }
            QScrollBar::handle:horizontal {
                background: #3498db;
                min-width: 30px;
                border-radius: 5px;
            }
            QScrollBar::handle:horizontal:hover {
                background: #5dade2;
            }
            QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
                border: none;
                background: none;
                width: 0px;
            }
            QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
                background: none;
            }
        """)
    
    def _create_basic_info_group(self):
        """创建基本信息组"""
        group = QGroupBox("📝 基本信息")
        group.setStyleSheet(GROUPBOX_STYLE)
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # USV ID
        self.id_label = create_value_label("--", large=True)
        self.id_label.setStyleSheet("""
            QLabel {
                color: #2c3e50;
                font-weight: bold;
                font-size: 16px;
                background-color: #ecf0f1;
                padding: 5px;
                border-radius: 3px;
            }
        """)
        layout.addWidget(QLabel("📋 USV ID:"), 0, 0)
        layout.addWidget(self.id_label, 0, 1)
        
        # 模式
        self.mode_label = create_value_label("--")
        self.mode_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                padding: 5px;
                border-radius: 4px;
            }
        """)
        layout.addWidget(QLabel("📋 模式:"), 1, 0)
        layout.addWidget(self.mode_label, 1, 1)
        
        # 状态
        self.status_label = create_value_label("--")
        self.status_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                padding: 5px;
                border-radius: 4px;
            }
        """)
        layout.addWidget(QLabel("📋 状态:"), 2, 0)
        layout.addWidget(self.status_label, 2, 1)
        
        # 解锁状态
        self.armed_label = create_value_label("--")
        self.armed_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                padding: 5px;
                border-radius: 4px;
            }
        """)
        layout.addWidget(QLabel("📋 解锁:"), 3, 0)
        layout.addWidget(self.armed_label, 3, 1)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_readiness_group(self):
        """创建 Ready 状态展示组"""
        group = QGroupBox("🎯 Ready 检查")
        group.setStyleSheet(GROUPBOX_STYLE.replace("#3498db", "#16a085"))

        layout = QVBoxLayout()
        layout.setSpacing(6)
        layout.setContentsMargins(10, 12, 10, 10)

        # Ready 按钮（仅显示用途）
        self.ready_button = QPushButton("等待数据…")
        self.ready_button.setEnabled(False)
        self.ready_button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.ready_button.setMinimumHeight(44)
        apply_button_style(self.ready_button, "#95a5a6")
        layout.addWidget(self.ready_button)
        
        # 飞控重启功能已移至参数配置窗口菜单：工具 → 🔄 重启飞控

        # Ready 摘要信息（参数配置功能已移至菜单栏：工具 → 飞控参数配置）
        self.ready_summary_label = QLabel("未接收到预检数据")
        self.ready_summary_label.setWordWrap(True)
        try:
            alignment = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        except AttributeError:
            alignment = AlignLeft | AlignVCenter
        self.ready_summary_label.setAlignment(alignment)  # type: ignore[arg-type]
        self.ready_summary_label.setStyleSheet("color: #7f8c8d; font-size: 16px;")
        layout.addWidget(self.ready_summary_label)

        # 传感器状态列表
        layout.addWidget(create_section_label("传感器状态"))
        self.sensor_list = QListWidget()
        configure_list_widget(self.sensor_list)
        self.sensor_list.setFixedHeight(120)
        layout.addWidget(self.sensor_list)
        set_list_placeholder(self.sensor_list, "等待传感器数据")

        # PreArm 警告列表
        layout.addWidget(create_section_label("PreArm 警告"))
        self.warning_list = QListWidget()
        configure_list_widget(self.warning_list)
        self.warning_list.setFixedHeight(100)
        layout.addWidget(self.warning_list)
        set_list_placeholder(self.warning_list, "无预检警告")

        group.setLayout(layout)
        return group
    
    def _create_position_info_group(self):
        """创建位置信息组"""
        group = QGroupBox("📋 位置信息")
        group.setStyleSheet(GROUPBOX_STYLE.replace("#3498db", "#27ae60"))
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # X 坐标
        self.x_label = create_value_label("--")
        layout.addWidget(create_key_label("X:"), 0, 0)
        layout.addWidget(self.x_label, 0, 1)
        layout.addWidget(QLabel("m"), 0, 2)
        
        # Y 坐标
        self.y_label = create_value_label("--")
        layout.addWidget(create_key_label("Y:"), 1, 0)
        layout.addWidget(self.y_label, 1, 1)
        layout.addWidget(QLabel("m"), 1, 2)
        
        # Z 坐标
        self.z_label = create_value_label("--")
        layout.addWidget(create_key_label("Z:"), 2, 0)
        layout.addWidget(self.z_label, 2, 1)
        layout.addWidget(QLabel("m"), 2, 2)
        
        # Yaw 角度
        self.yaw_label = create_value_label("--")
        layout.addWidget(create_key_label("Yaw:"), 3, 0)
        layout.addWidget(self.yaw_label, 3, 1)
        layout.addWidget(QLabel("°"), 3, 2)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group
    
    def _create_battery_info_group(self):
        """创建电池信息组"""
        group = QGroupBox("📋 电池信息")
        group.setStyleSheet(GROUPBOX_STYLE.replace("#3498db", "#f39c12"))
        
        layout = QVBoxLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # 电池进度条（紧凑版）
        self.battery_bar = QProgressBar()
        self.battery_bar.setMinimum(0)
        self.battery_bar.setMaximum(100)
        self.battery_bar.setValue(0)
        self.battery_bar.setTextVisible(True)
        self.battery_bar.setFormat("%p%")
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 1.5px solid #bdc3c7;
                border-radius: 4px;
                text-align: center;
                height: 20px;
                font-weight: bold;
                font-size: 16px;
            }
            QProgressBar::chunk {
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.battery_bar)
        
        # 电压和电流信息
        info_layout = QGridLayout()
        info_layout.setSpacing(4)
        
        self.voltage_label = create_value_label("--")
        info_layout.addWidget(create_key_label("电压:"), 0, 0)
        info_layout.addWidget(self.voltage_label, 0, 1)
        info_layout.addWidget(QLabel("V"), 0, 2)
        
        self.current_label = create_value_label("--")
        info_layout.addWidget(create_key_label("电流:"), 1, 0)
        info_layout.addWidget(self.current_label, 1, 1)
        info_layout.addWidget(QLabel("A"), 1, 2)
        
        # 温度信息
        self.temperature_label = create_value_label("--")
        info_layout.addWidget(create_key_label("温度:"), 2, 0)
        info_layout.addWidget(self.temperature_label, 2, 1)
        info_layout.addWidget(QLabel("℃"), 2, 2)
        
        info_layout.setColumnStretch(1, 1)
        layout.addLayout(info_layout)
        
        group.setLayout(layout)
        return group

    def _create_vehicle_message_group(self):
        """创建飞控消息展示组（支持筛选和复制）"""
        group = QGroupBox("📋 飞控消息")
        group.setStyleSheet(GROUPBOX_STYLE.replace("#3498db", "#34495e"))

        layout = QVBoxLayout()
        layout.setSpacing(6)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # ==================== 控制栏 ====================
        control_layout = QHBoxLayout()
        control_layout.setSpacing(8)
        
        # 消息类型筛选复选框
        filter_label = QLabel("显示:")
        filter_label.setStyleSheet("color: #7f8c8d; font-size: 12px; font-weight: bold;")
        control_layout.addWidget(filter_label)
        
        # 初始化消息类型筛选状态（默认只显示重要消息）
        self._message_filters = {
            'critical': True,   # EMERGENCY/ALERT/CRITICAL (0-2)
            'error': True,      # ERROR (3)
            'warning': True,    # WARNING (4)
            'notice': False,    # NOTICE (5)
            'info': False,      # INFO (6)
            'debug': False,     # DEBUG (7)
        }
        
        # 重要消息复选框（CRITICAL/ERROR/WARNING）- 默认勾选
        self.filter_critical_cb = QCheckBox("❗严重")
        self.filter_critical_cb.setChecked(True)
        self.filter_critical_cb.setToolTip("显示 EMERGENCY/ALERT/CRITICAL 级别消息")
        self.filter_critical_cb.stateChanged.connect(lambda s: self._on_filter_changed('critical', s))
        self._style_filter_checkbox(self.filter_critical_cb, "#e74c3c")
        control_layout.addWidget(self.filter_critical_cb)
        
        self.filter_error_cb = QCheckBox("⚠️错误")
        self.filter_error_cb.setChecked(True)
        self.filter_error_cb.setToolTip("显示 ERROR 级别消息")
        self.filter_error_cb.stateChanged.connect(lambda s: self._on_filter_changed('error', s))
        self._style_filter_checkbox(self.filter_error_cb, "#e67e22")
        control_layout.addWidget(self.filter_error_cb)
        
        self.filter_warning_cb = QCheckBox("⚠警告")
        self.filter_warning_cb.setChecked(True)
        self.filter_warning_cb.setToolTip("显示 WARNING 级别消息")
        self.filter_warning_cb.stateChanged.connect(lambda s: self._on_filter_changed('warning', s))
        self._style_filter_checkbox(self.filter_warning_cb, "#f39c12")
        control_layout.addWidget(self.filter_warning_cb)
        
        # 一般消息复选框（NOTICE/INFO/DEBUG）- 默认不勾选
        self.filter_info_cb = QCheckBox("ℹ️信息")
        self.filter_info_cb.setChecked(False)
        self.filter_info_cb.setToolTip("显示 NOTICE/INFO 级别消息（不影响解锁）")
        self.filter_info_cb.stateChanged.connect(lambda s: self._on_filter_changed('info', s))
        self._style_filter_checkbox(self.filter_info_cb, "#3498db")
        control_layout.addWidget(self.filter_info_cb)
        
        self.filter_debug_cb = QCheckBox("🔍调试")
        self.filter_debug_cb.setChecked(False)
        self.filter_debug_cb.setToolTip("显示 DEBUG 级别消息（不影响解锁）")
        self.filter_debug_cb.stateChanged.connect(lambda s: self._on_filter_changed('debug', s))
        self._style_filter_checkbox(self.filter_debug_cb, "#95a5a6")
        control_layout.addWidget(self.filter_debug_cb)
        
        control_layout.addStretch()
        
        layout.addLayout(control_layout)
        
        # ==================== 消息列表 ====================
        self.message_list = QListWidget()
        configure_list_widget(self.message_list, allow_selection=True)
        self.message_list.setMinimumHeight(140)
        # 启用右键菜单
        try:
            self.message_list.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        except AttributeError:
            self.message_list.setContextMenuPolicy(Qt.CustomContextMenu)
        self.message_list.customContextMenuRequested.connect(self._show_message_context_menu)
        layout.addWidget(self.message_list)
        set_list_placeholder(self.message_list, "尚未收到飞控消息")
        
        # 保存原始消息列表（用于筛选）
        self._all_vehicle_messages = []

        group.setLayout(layout)
        return group
    
    def _style_filter_checkbox(self, checkbox, color):
        """设置筛选复选框样式（带颜色标识）"""
        checkbox.setStyleSheet(f"""
            QCheckBox {{
                font-size: 12px;
                font-weight: bold;
                color: {color};
                spacing: 4px;
                padding: 2px 4px;
                border-radius: 3px;
            }}
            QCheckBox:hover {{
                background-color: rgba(0, 0, 0, 0.05);
            }}
            QCheckBox::indicator {{
                width: 16px;
                height: 16px;
                border-radius: 3px;
                border: 2px solid {color};
            }}
            QCheckBox::indicator:checked {{
                background-color: {color};
                border-color: {color};
                image: none;
            }}
            QCheckBox::indicator:unchecked {{
                background-color: white;
                border-color: {color};
            }}
        """)
    
    def _on_filter_changed(self, filter_type, state):
        """处理筛选复选框状态变化"""
        # 兼容 PyQt5 不同版本的 CheckState
        try:
            is_checked = state == Qt.CheckState.Checked.value
        except AttributeError:
            try:
                is_checked = state == Qt.Checked
            except AttributeError:
                is_checked = state == 2  # Qt.Checked 的值
        
        if filter_type == 'info':
            # info 复选框同时控制 NOTICE 和 INFO
            self._message_filters['notice'] = is_checked
            self._message_filters['info'] = is_checked
        else:
            self._message_filters[filter_type] = is_checked
        
        # 重新筛选并显示消息
        self._refresh_filtered_messages()
    
    def _refresh_filtered_messages(self):
        """根据当前筛选设置刷新消息列表"""
        if not hasattr(self, 'message_list'):
            return
        
        filtered = self._filter_messages(self._all_vehicle_messages)
        self._display_messages(filtered)
    
    def _filter_messages(self, messages):
        """根据当前筛选设置过滤消息
        
        支持两种级别系统：
        - MAVLink (0-7): 0-2=CRITICAL, 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG
        - ROS 2 (10-50): 50=FATAL, 40=ERROR, 30=WARNING, 20=INFO, 10=DEBUG
        """
        if not messages:
            return []
        
        filtered = []
        for entry in messages:
            severity = entry.get('severity', 6)
            
            # 判断级别类型并转换为统一的分类
            if severity >= 10:
                # ROS 2 日志级别 (10-50)
                if severity >= 50:  # FATAL -> critical
                    category = 'critical'
                elif severity >= 40:  # ERROR
                    category = 'error'
                elif severity >= 30:  # WARNING
                    category = 'warning'
                elif severity >= 20:  # INFO
                    category = 'info'
                else:  # DEBUG (10)
                    category = 'debug'
            else:
                # MAVLink 级别 (0-7)
                if severity <= 2:  # EMERGENCY/ALERT/CRITICAL
                    category = 'critical'
                elif severity == 3:  # ERROR
                    category = 'error'
                elif severity == 4:  # WARNING
                    category = 'warning'
                elif severity == 5:  # NOTICE
                    category = 'notice'
                elif severity == 6:  # INFO
                    category = 'info'
                else:  # DEBUG (7+)
                    category = 'debug'
            
            # 检查该分类是否被启用
            # notice 和 info 共用 info 筛选器
            if category == 'notice':
                if self._message_filters.get('notice', False):
                    filtered.append(entry)
            elif self._message_filters.get(category, category in ['critical', 'error', 'warning']):
                filtered.append(entry)
        
        return filtered
    
    def _display_messages(self, messages):
        """显示过滤后的消息列表"""
        self.message_list.clear()
        
        if not messages:
            set_list_placeholder(self.message_list, "无匹配的消息（调整筛选条件）")
            return
        
        max_items = 30
        for entry in messages[:max_items]:
            severity = entry.get('severity', 6)
            label = entry.get('severity_label') or f"LEVEL {severity}"
            time_str = entry.get('time') or "--:--:--"
            text = entry.get('text', '')
            combined = f"[{time_str}] {label}: {text}"
            item = QListWidgetItem(combined)
            # 设置可选择标志，允许复制
            try:
                item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
            except AttributeError:
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)  # type: ignore[attr-defined]
            item.setToolTip(text)
            # severity_palette 返回 (前景色, 背景色)
            fg_color, bg_color = severity_palette(label)
            item.setBackground(QColor(bg_color))
            item.setForeground(QColor(fg_color))
            self.message_list.addItem(item)
    
    def _show_message_context_menu(self, pos):
        """显示飞控消息右键菜单"""
        menu = QMenu(self.message_list)
        
        item = self.message_list.itemAt(pos)
        
        # 复制当前消息（仅当有选中项时）
        if item is not None:
            copy_action = menu.addAction("📋 复制消息")
        else:
            copy_action = None
        
        copy_all_action = menu.addAction("📄 复制全部消息")
        
        action = menu.exec_(self.message_list.mapToGlobal(pos))
        if action == copy_action and item is not None:
            QApplication.clipboard().setText(item.text())
        elif action == copy_all_action:
            all_text = []
            for i in range(self.message_list.count()):
                all_text.append(self.message_list.item(i).text())
            QApplication.clipboard().setText('\n'.join(all_text))
    
    def update_state(self, state):
        """
        更新USV状态显示
        
        Args:
            state: USV状态字典，包含所有状态信息
        """
        # 重入保护：如果正在更新中，直接返回
        if self._is_updating:
            return
        
        self._is_updating = True
        try:
            self._do_update_state(state)
        finally:
            self._is_updating = False
    
    def _do_update_state(self, state):
        """实际执行状态更新的内部方法"""
        if state is None:
            self._clear_display()
            return
        
        self._current_state = state
        
        try:
            pos = state.get('position', {}) or {}
            
            vehicle_messages = state.get('vehicle_messages') or []
            prearm_warnings = state.get('prearm_warnings') or []
            sensor_status = state.get('sensor_status') or []
            prearm_ready = bool(state.get('prearm_ready', False))

            # 更新基本信息
            self.id_label.setText(str(state.get('namespace', '--')))
            
            mode = state.get('mode', '--')
            self.mode_label.setText(str(mode))
            update_mode_style(self.mode_label, mode)
            
            # Ready 指示与连接状态
            # connected: 网络连接（能收到 Zenoh 桥接的消息）
            # fc_connected: 飞控连接（USV 端与 PX4 飞控的连接状态）
            connected = state.get('connected', False)
            fc_connected = state.get('fc_connected', False)
            armed = state.get('armed', False)
            self._update_ready_view(prearm_ready, prearm_warnings, fc_connected, armed)
            self._update_sensor_list(sensor_status)
            self._update_vehicle_messages(vehicle_messages)

            # 状态显示: 区分网络在线和飞控连接
            if not connected:
                status = "离线"
            elif not fc_connected:
                status = "飞控断开"
            else:
                status = "在线"
            self.status_label.setText(str(status))
            update_status_style(self.status_label, status)
            
            # Armed 标签更新（armed 已在上面获取）
            self.armed_label.setText(str(armed))
            update_armed_style(self.armed_label, armed)
            
            # 更新位置信息
            pos = state.get('position', {}) or {}
            x_val = format_float(pos.get('x'), precision=2)
            y_val = format_float(pos.get('y'), precision=2)
            z_val = format_float(pos.get('z'), precision=2)
            self.x_label.setText(x_val)
            self.y_label.setText(y_val)
            self.z_label.setText(z_val)
            
            # Yaw 角度（从弧度转换为度数显示）
            yaw_rad = state.get('yaw')
            if yaw_rad is not None:
                try:
                    import math
                    yaw_deg = math.degrees(float(yaw_rad))
                    self.yaw_label.setText(format_float(yaw_deg, precision=1))
                except (ValueError, TypeError):
                    self.yaw_label.setText("--")
            else:
                self.yaw_label.setText("--")
            
            # 更新电池信息
            battery_pct = state.get('battery_percentage', 0)
            try:
                battery_val = float(battery_pct)
            except (ValueError, TypeError):
                battery_val = 0
            
            self.battery_bar.setValue(int(battery_val))
            update_battery_style(self.battery_bar, battery_val)
            
            voltage = state.get('battery_voltage', '--')
            self.voltage_label.setText(format_float(voltage, precision=2))
            
            current = state.get('battery_current', None)
            self.current_label.setText(format_float(current, precision=1))
            
            # 温度信息（摄氏度）
            try:
                import math
                temp_celsius = float(state.get('temperature'))
                if math.isnan(temp_celsius):
                    temp_celsius = None
            except (ValueError, TypeError):
                temp_celsius = None
            if temp_celsius is not None:
                self.temperature_label.setText(format_float(temp_celsius, precision=1))
                self._update_temperature_style(temp_celsius)
            else:
                self.temperature_label.setText("--")
                self.temperature_label.setStyleSheet("")
                self._is_high_temperature = False
            
        except Exception as e:
            _logger.error(f"更新 USV 信息面板失败: {e}")
    
    def _clear_display(self):
        """清空显示"""
        self.id_label.setText("--")
        self.mode_label.setText("--")
        self.status_label.setText("--")
        self.armed_label.setText("--")
        
        self.x_label.setText("--")
        self.y_label.setText("--")
        self.z_label.setText("--")
        self.yaw_label.setText("--")
        
        self.battery_bar.setValue(0)
        self.voltage_label.setText("--")
        self.current_label.setText("--")
        self.temperature_label.setText("--")
        
        # 重置温度状态标志
        self._is_high_temperature = False
        
        self._current_state = None

        if hasattr(self, 'ready_button'):
            self.ready_button.setText("等待数据…")
            apply_button_style(self.ready_button, "#95a5a6")
        if hasattr(self, 'ready_summary_label'):
            self.ready_summary_label.setText("未接收到预检数据")
            self.ready_summary_label.setToolTip("")
        if hasattr(self, 'sensor_list'):
            set_list_placeholder(self.sensor_list, "等待传感器数据")
        if hasattr(self, 'warning_list'):
            set_list_placeholder(self.warning_list, "无预检警告")
        if hasattr(self, 'message_list'):
            set_list_placeholder(self.message_list, "尚未收到飞控消息")

    def _update_ready_view(self, ready, warnings, connected, armed=False):
        """根据预检结果和解锁状态更新 Ready 按钮和警告列表
        
        Args:
            ready: 预检是否通过
            warnings: 预检警告列表
            connected: 是否连接
            armed: 是否已解锁（USV Arm 成功后为 True）
        """
        if not hasattr(self, 'ready_button'):
            return

        if not connected:
            button_text = "USV 离线"
            summary = "车辆离线，等待连接..."
            button_bg, button_fg = "#95a5a6", "#ffffff"
        elif armed:
            # 已解锁状态优先显示（USV Arm 成功后）
            button_text = "Armed"
            summary = "无人球已解锁，准备运行"
            button_bg, button_fg = "#27ae60", "#ffffff"
        elif ready:
            button_text = "Ready to Sail"
            summary = "所有预检检查通过"
            button_bg, button_fg = "#27ae60", "#ffffff"
        elif warnings:
            button_text = "PreArm Checks Required"
            summary = f"{len(warnings)} 条预检警告待处理"
            button_bg, button_fg = "#e67e22", "#ffffff"
        else:
            button_text = "等待预检结果…"
            summary = "等待飞控返回预检结论"
            button_bg, button_fg = "#f1c40f", "#2c3e50"

        self.ready_button.setText(button_text)
        apply_button_style(self.ready_button, button_bg, button_fg)
        self.ready_summary_label.setText(summary)
        tooltip_lines = warnings[:8]
        self.ready_summary_label.setToolTip("\n".join(tooltip_lines) if tooltip_lines else "")

        if warnings:
            self.warning_list.clear()
            warning_bg, warning_fg = level_to_palette('error')
            for warning_text in warnings:
                item = QListWidgetItem(warning_text)
                item.setToolTip(warning_text)
                item.setBackground(QColor(warning_bg))
                item.setForeground(QColor(warning_fg))
                self.warning_list.addItem(item)
        else:
            placeholder = "无预检警告" if connected else "等待预检数据"
            set_list_placeholder(self.warning_list, placeholder)

    def _update_sensor_list(self, sensor_status):
        """更新传感器健康列表"""
        if not hasattr(self, 'sensor_list'):
            return

        if not sensor_status:
            set_list_placeholder(self.sensor_list, "等待传感器数据")
            return

        # 确保 sensor_status 是列表
        if not isinstance(sensor_status, list):
            set_list_placeholder(self.sensor_list, "等待传感器数据")
            return
        
        # 预先构建所有 items，然后一次性添加
        items_data = []
        for entry in sensor_status:
            name = entry.get('name', 'Sensor')
            status_text = entry.get('status', '--')
            detail = entry.get('detail')
            combined = f"{name}: {status_text}"
            if detail:
                combined += f"  ({detail})"
            bg_color, fg_color = level_to_palette(entry.get('level'))
            items_data.append((combined, detail, bg_color, fg_color))
        
        # 禁用更新，快速添加所有项
        self.sensor_list.setUpdatesEnabled(False)
        try:
            self.sensor_list.clear()
            for combined, detail, bg_color, fg_color in items_data:
                item = QListWidgetItem(combined)
                if detail:
                    item.setToolTip(detail)
                item.setBackground(QColor(bg_color))
                item.setForeground(QColor(fg_color))
                self.sensor_list.addItem(item)
        finally:
            self.sensor_list.setUpdatesEnabled(True)

    def _update_vehicle_messages(self, messages):
        """更新飞控消息列表(支持筛选)"""
        if not hasattr(self, 'message_list'):
            return

        # 保存原始消息列表
        self._all_vehicle_messages = list(messages) if messages else []
        
        if not messages:
            set_list_placeholder(self.message_list, "尚未收到飞控消息")
            return

        # 应用筛选并显示
        filtered = self._filter_messages(messages)
        self._display_messages(filtered)

    def _update_dynamic_styles(self):
        """更新动态样式（由定时器调用）"""
        # 可以在这里添加动画效果或闪烁提醒
        # 例如：低电量时闪烁、信号差时闪烁等
        pass
