"""
USV 信息面板模块
提供美观、信息丰富的 USV 详细信息显示界面
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                              QGroupBox, QGridLayout, QFrame, QProgressBar,
                              QScrollArea, QSizePolicy, QPushButton,
                              QListWidget, QListWidgetItem, QAbstractItemView)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QFont, QColor, QPalette
from .compass_widget import CompassWidget

# 兼容性定义
try:
    AlignRight = Qt.AlignmentFlag.AlignRight  # type: ignore
    AlignLeft = Qt.AlignmentFlag.AlignLeft  # type: ignore
    AlignVCenter = Qt.AlignmentFlag.AlignVCenter  # type: ignore
except AttributeError:
    AlignRight = Qt.AlignRight  # type: ignore
    AlignLeft = Qt.AlignLeft  # type: ignore
    AlignVCenter = Qt.AlignVCenter  # type: ignore


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
    
    # 统一的 QGroupBox 样式（紧凑版）
    GROUPBOX_STYLE = """
        QGroupBox {
            font-weight: bold;
            font-size: 16px;
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
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 温度状态跟踪（用于实现滞后效果）
        self._is_high_temperature = False  # False=低温(绿色), True=高温(红色)
        
        # 当前状态缓存
        self._current_state = None
        
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
        
        # ==================== GPS 信息组 ====================
        gps_group = self._create_gps_info_group()
        content_layout.addWidget(gps_group)
        
        # ==================== 速度信息组 ====================
        velocity_group = self._create_velocity_info_group()
        content_layout.addWidget(velocity_group)
        
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
        group.setStyleSheet(self.GROUPBOX_STYLE)
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # USV ID
        self.id_label = self._create_value_label("--", large=True)
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
        self.mode_label = self._create_value_label("--")
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
        self.status_label = self._create_value_label("--")
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
        self.armed_label = self._create_value_label("--")
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
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#3498db", "#16a085"))

        layout = QVBoxLayout()
        layout.setSpacing(6)
        layout.setContentsMargins(10, 12, 10, 10)

        # Ready 按钮（仅显示用途）
        self.ready_button = QPushButton("等待数据…")
        self.ready_button.setEnabled(False)
        self.ready_button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.ready_button.setMinimumHeight(44)
        self._apply_button_style(self.ready_button, "#95a5a6")
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
        layout.addWidget(self._create_section_label("传感器状态"))
        self.sensor_list = QListWidget()
        self._configure_list_widget(self.sensor_list)
        self.sensor_list.setFixedHeight(120)
        layout.addWidget(self.sensor_list)
        self._set_list_placeholder(self.sensor_list, "等待传感器数据")

        # PreArm 警告列表
        layout.addWidget(self._create_section_label("PreArm 警告"))
        self.warning_list = QListWidget()
        self._configure_list_widget(self.warning_list)
        self.warning_list.setFixedHeight(100)
        layout.addWidget(self.warning_list)
        self._set_list_placeholder(self.warning_list, "无预检警告")

        group.setLayout(layout)
        return group
    
    def _create_position_info_group(self):
        """创建位置信息组"""
        group = QGroupBox("📋 位置信息")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#3498db", "#27ae60"))
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # X 坐标
        self.x_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("X:"), 0, 0)
        layout.addWidget(self.x_label, 0, 1)
        layout.addWidget(QLabel("m"), 0, 2)
        
        # Y 坐标
        self.y_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("Y:"), 1, 0)
        layout.addWidget(self.y_label, 1, 1)
        layout.addWidget(QLabel("m"), 1, 2)
        
        # Z 坐标
        self.z_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("Z:"), 2, 0)
        layout.addWidget(self.z_label, 2, 1)
        layout.addWidget(QLabel("m"), 2, 2)
        
        # Yaw 角度
        self.yaw_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("Yaw:"), 3, 0)
        layout.addWidget(self.yaw_label, 3, 1)
        layout.addWidget(QLabel("°"), 3, 2)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group
    
    def _create_battery_info_group(self):
        """创建电池信息组"""
        group = QGroupBox("📋 电池信息")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#3498db", "#f39c12"))
        
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
        
        self.voltage_label = self._create_value_label("--")
        info_layout.addWidget(self._create_key_label("电压:"), 0, 0)
        info_layout.addWidget(self.voltage_label, 0, 1)
        info_layout.addWidget(QLabel("V"), 0, 2)
        
        self.current_label = self._create_value_label("--")
        info_layout.addWidget(self._create_key_label("电流:"), 1, 0)
        info_layout.addWidget(self.current_label, 1, 1)
        info_layout.addWidget(QLabel("A"), 1, 2)
        
        # 温度信息
        self.temperature_label = self._create_value_label("--")
        info_layout.addWidget(self._create_key_label("温度:"), 2, 0)
        info_layout.addWidget(self.temperature_label, 2, 1)
        info_layout.addWidget(QLabel("℃"), 2, 2)
        
        info_layout.setColumnStretch(1, 1)
        layout.addLayout(info_layout)
        
        group.setLayout(layout)
        return group
    
    def _create_gps_info_group(self):
        """创建GPS信息组"""
        group = QGroupBox("📋 GPS 信息")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#3498db", "#9b59b6"))
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # 卫星数量
        self.satellite_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("卫星数:"), 0, 0)
        layout.addWidget(self.satellite_label, 0, 1)
        
        # GPS精度
        self.gps_accuracy_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("精度:"), 1, 0)
        layout.addWidget(self.gps_accuracy_label, 1, 1)
        layout.addWidget(QLabel("m"), 1, 2)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group
    
    def _create_velocity_info_group(self):
        """创建速度信息组（带罗盘显示）"""
        group = QGroupBox("📋 速度 & 航向")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#3498db", "#e74c3c"))
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 12, 10, 10)
        
        # 上半部分：文字信息
        info_layout = QGridLayout()
        info_layout.setSpacing(5)
        
        # 地速
        self.ground_speed_label = self._create_value_label("--")
        info_layout.addWidget(self._create_key_label("地速:"), 0, 0)
        info_layout.addWidget(self.ground_speed_label, 0, 1)
        info_layout.addWidget(QLabel("m/s"), 0, 2)
        
        # 航向数字
        self.heading_speed_label = self._create_value_label("--")
        info_layout.addWidget(self._create_key_label("航向:"), 1, 0)
        info_layout.addWidget(self.heading_speed_label, 1, 1)
        info_layout.addWidget(QLabel("°"), 1, 2)
        
        info_layout.setColumnStretch(1, 1)
        main_layout.addLayout(info_layout)
        
        # 下半部分：罗盘图形显示
        compass_container = QHBoxLayout()
        compass_container.addStretch()
        self.compass_widget = CompassWidget()
        self.compass_widget.setFixedSize(140, 140)  # 固定尺寸，避免拉伸
        compass_container.addWidget(self.compass_widget)
        compass_container.addStretch()
        main_layout.addLayout(compass_container)
        
        group.setLayout(main_layout)
        return group

    def _create_vehicle_message_group(self):
        """创建飞控消息展示组"""
        group = QGroupBox("📋 飞控消息")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#3498db", "#34495e"))

        layout = QVBoxLayout()
        layout.setSpacing(6)
        layout.setContentsMargins(10, 12, 10, 10)

        self.message_list = QListWidget()
        self._configure_list_widget(self.message_list)
        self.message_list.setMinimumHeight(160)
        layout.addWidget(self.message_list)
        self._set_list_placeholder(self.message_list, "尚未收到飞控消息")

        group.setLayout(layout)
        return group
    
    def _create_key_label(self, text):
        """创建键标签（紧凑版）"""
        label = QLabel(text)
        if hasattr(Qt, "AlignmentFlag"):
            alignment = Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        else:
            alignment = AlignRight | AlignVCenter
        label.setAlignment(alignment)  # type: ignore[arg-type]
        label.setStyleSheet("""
            QLabel {
                color: #7f8c8d;
                font-weight: bold;
                font-size: 16px;
                min-width: 40px;
            }
        """)
        return label
    
    def _create_value_label(self, text, large=False):
        """创建值标签（响应式字体）"""
        label = QLabel(text)
        if hasattr(Qt, "AlignmentFlag"):
            alignment = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        else:
            alignment = AlignLeft | AlignVCenter
        label.setAlignment(alignment)  # type: ignore[arg-type]
        if large:
            label.setStyleSheet("""
                QLabel {
                    color: #2c3e50;
                    font-size: 16px;
                    font-weight: bold;
                }
            """)
        else:
            label.setStyleSheet("""
                QLabel {
                    color: #34495e;
                    font-size: 16px;
                    font-weight: 600;
                }
            """)
        return label

    def _create_section_label(self, text):
        """创建分组内的小节标题"""
        label = QLabel(text)
        try:
            alignment = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        except AttributeError:
            alignment = AlignLeft | AlignVCenter
        label.setAlignment(alignment)  # type: ignore[arg-type]
        label.setStyleSheet("color: #2c3e50; font-size: 16px; font-weight: bold; margin-top: 4px;")
        return label

    def _configure_list_widget(self, widget):
        """统一配置列表控件样式"""
        try:
            widget.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        except AttributeError:
            widget.setFocusPolicy(Qt.NoFocus)  # type: ignore[attr-defined]
        try:
            widget.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        except AttributeError:
            widget.setSelectionMode(QListWidget.NoSelection)
        widget.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        try:
            widget.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        except AttributeError:
            widget.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        widget.setAlternatingRowColors(False)
        widget.setWordWrap(True)
        widget.setStyleSheet("""
            QListWidget {
                background-color: #ffffff;
                border: 1px solid #ecf0f1;
                border-radius: 6px;
            }
            QListWidget::item {
                padding: 6px 8px;
            }
        """)

    def _set_list_placeholder(self, widget, text):
        """在列表内显示占位提示"""
        widget.clear()
        placeholder = QListWidgetItem(text)
        try:
            placeholder.setFlags(Qt.ItemFlag.ItemIsEnabled)
        except AttributeError:
            placeholder.setFlags(Qt.ItemIsEnabled)  # type: ignore[attr-defined]
        placeholder.setForeground(QColor("#7f8c8d"))
        widget.addItem(placeholder)

    def _apply_button_style(self, button, background, text_color="#ffffff"):
        """统一按钮样式"""
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {background};
                color: {text_color};
                font-weight: bold;
                font-size: 16px;
                border: none;
                border-radius: 6px;
                padding: 10px 14px;
            }}
            QPushButton:disabled {{
                color: {text_color};
                background-color: {background};
            }}
        """)
    
    def update_state(self, state):
        """
        更新USV状态显示
        
        Args:
            state: USV状态字典，包含所有状态信息
        """
        if state is None:
            self._clear_display()
            return
        
        self._current_state = state
        
        try:
            vehicle_messages = state.get('vehicle_messages') or []
            prearm_warnings = state.get('prearm_warnings') or []
            sensor_status = state.get('sensor_status') or []
            prearm_ready = bool(state.get('prearm_ready', False))

            # 更新基本信息
            self.id_label.setText(str(state.get('namespace', '--')))
            
            mode = state.get('mode', '--')
            self.mode_label.setText(str(mode))
            self._update_mode_style(mode)
            
            # Ready 指示与连接状态
            connected = state.get('connected', False)
            self._update_ready_view(prearm_ready, prearm_warnings, connected)
            self._update_sensor_list(sensor_status)
            self._update_vehicle_messages(vehicle_messages)

            # 连接状态作为 status 显示
            status = "在线" if connected else "离线"
            self.status_label.setText(str(status))
            self._update_status_style(status)
            
            armed = state.get('armed', False)
            self.armed_label.setText(str(armed))
            self._update_armed_style(armed)
            
            # 更新位置信息
            # 更新位置信息
            pos = state.get('position', {}) or {}
            self.x_label.setText(self._format_float(pos.get('x'), precision=2))
            self.y_label.setText(self._format_float(pos.get('y'), precision=2))
            self.z_label.setText(self._format_float(pos.get('z'), precision=2))
            
            # Yaw 角度（从弧度转换为度数显示）
            yaw_rad = state.get('yaw')
            if yaw_rad is not None:
                try:
                    import math
                    yaw_deg = math.degrees(float(yaw_rad))
                    self.yaw_label.setText(self._format_float(yaw_deg, precision=1))
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
            self._update_battery_style(battery_val)
            
            voltage = state.get('battery_voltage', '--')
            self.voltage_label.setText(self._format_float(voltage, precision=2))
            
            current = state.get('battery_current', None)
            self.current_label.setText(self._format_float(current, precision=1))
            
            # 温度信息（从毫摄氏度转换为摄氏度）
            try:
                temp_raw = float(state.get('temperature'))
                temp_celsius = temp_raw / 1000.0  # 转换：毫度 → 度
            except (ValueError, TypeError):
                temp_celsius = None
            if temp_celsius is not None:
                self.temperature_label.setText(self._format_float(temp_celsius, precision=1))
                self._update_temperature_style(temp_celsius)
            else:
                self.temperature_label.setText("--")
                self.temperature_label.setStyleSheet("")
                self._is_high_temperature = False
            
            sat_count = state.get('gps_satellites_visible')
            if sat_count is None:
                self.satellite_label.setText("--")
            else:
                try:
                    self.satellite_label.setText(str(int(sat_count)))
                except (ValueError, TypeError):
                    self.satellite_label.setText(self._format_float(sat_count, precision=0))
            self._update_satellite_style(sat_count)

            self.gps_accuracy_label.setText(self._format_float(state.get('gps_eph'), precision=1))
            
            # 更新速度信息（从 velocity 计算）
            vel = state.get('velocity', {}) or {}
            linear = vel.get('linear', {}) or {}
            
            # 计算地速（水平速度的模）
            try:
                vx = float(linear.get('x', 0.0))
                vy = float(linear.get('y', 0.0))
                ground_speed = (vx ** 2 + vy ** 2) ** 0.5
                self.ground_speed_label.setText(self._format_float(ground_speed, precision=2))
            except (ValueError, TypeError):
                self.ground_speed_label.setText("--")
            
            # 航向（直接从 heading 字段获取，单位为度）
            try:
                heading_deg = float(state.get('heading', 0.0))
                self.heading_speed_label.setText(self._format_float(heading_deg, precision=1))
                # 更新罗盘显示
                self.compass_widget.set_heading(heading_deg)
            except (ValueError, TypeError):
                self.heading_speed_label.setText("--")
                self.compass_widget.set_heading(0.0)
            
        except Exception as e:
            print(f"更新 USV 信息面板失败: {e}")
    
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
        
        self.satellite_label.setText("--")
        self.gps_accuracy_label.setText("--")
        
        self.ground_speed_label.setText("--")
        self.heading_speed_label.setText("--")
        self.compass_widget.set_heading(0.0)  # 重置罗盘显示
        
        # 重置温度状态标志
        self._is_high_temperature = False
        
        self._current_state = None

        if hasattr(self, 'ready_button'):
            self.ready_button.setText("等待数据…")
            self._apply_button_style(self.ready_button, "#95a5a6")
        if hasattr(self, 'ready_summary_label'):
            self.ready_summary_label.setText("未接收到预检数据")
            self.ready_summary_label.setToolTip("")
        if hasattr(self, 'sensor_list'):
            self._set_list_placeholder(self.sensor_list, "等待传感器数据")
        if hasattr(self, 'warning_list'):
            self._set_list_placeholder(self.warning_list, "无预检警告")
        if hasattr(self, 'message_list'):
            self._set_list_placeholder(self.message_list, "尚未收到飞控消息")
    
    def _format_float(self, value, precision=2):
        """格式化浮点数"""
        try:
            if value is None or value == '--':
                return "--"
            return f"{float(value):.{precision}f}"
        except (ValueError, TypeError):
            return "--"
    
    def _update_mode_style(self, mode):
        """根据模式更新样式"""
        mode_str = str(mode).upper()
        if "GUIDED" in mode_str:
            color = "#27ae60"  # 绿色
        elif "MANUAL" in mode_str:
            color = "#f39c12"  # 橙色
        elif "AUTO" in mode_str:
            color = "#3498db"  # 蓝色
        else:
            color = "#95a5a6"  # 灰色
        
        self.mode_label.setStyleSheet(f"""
            QLabel {{
                color: white;
                background-color: {color};
                font-weight: bold;
                padding: 5px;
                border-radius: 4px;
            }}
        """)
    
    def _update_status_style(self, status):
        """根据状态更新样式"""
        status_str = str(status).upper()
        if "STANDBY" in status_str:
            color = "#3498db"  # 蓝色
        elif "ACTIVE" in status_str:
            color = "#27ae60"  # 绿色
        elif "CRITICAL" in status_str or "EMERGENCY" in status_str:
            color = "#e74c3c"  # 红色
        else:
            color = "#95a5a6"  # 灰色
        
        self.status_label.setStyleSheet(f"""
            QLabel {{
                color: white;
                background-color: {color};
                font-weight: bold;
                padding: 5px;
                border-radius: 4px;
            }}
        """)
    
    def _update_armed_style(self, armed):
        """根据解锁状态更新样式"""
        armed_str = str(armed).upper()
        if "TRUE" in armed_str or "ARMED" in armed_str:
            color = "#e74c3c"  # 红色
            text = "已解锁"
        else:
            color = "#27ae60"  # 绿色
            text = "已锁定"
        
        self.armed_label.setText(text)
        self.armed_label.setStyleSheet(f"""
            QLabel {{
                color: white;
                background-color: {color};
                font-weight: bold;
                padding: 5px;
                border-radius: 4px;
            }}
        """)

    def _update_ready_view(self, ready, warnings, connected):
        """根据预检结果更新 Ready 按钮和警告列表"""
        if not hasattr(self, 'ready_button'):
            return

        if not connected:
            button_text = "USV 离线"
            summary = "车辆离线，等待连接..."
            button_bg, button_fg = "#95a5a6", "#ffffff"
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
        self._apply_button_style(self.ready_button, button_bg, button_fg)
        self.ready_summary_label.setText(summary)
        tooltip_lines = warnings[:8]
        self.ready_summary_label.setToolTip("\n".join(tooltip_lines) if tooltip_lines else "")

        if warnings:
            self.warning_list.clear()
            warning_bg, warning_fg = self._level_to_palette('error')
            for warning_text in warnings:
                item = QListWidgetItem(warning_text)
                item.setToolTip(warning_text)
                item.setBackground(QColor(warning_bg))
                item.setForeground(QColor(warning_fg))
                self.warning_list.addItem(item)
        else:
            placeholder = "无预检警告" if connected else "等待预检数据"
            self._set_list_placeholder(self.warning_list, placeholder)

    def _update_sensor_list(self, sensor_status):
        """更新传感器健康列表"""
        if not hasattr(self, 'sensor_list'):
            return

        if not sensor_status:
            self._set_list_placeholder(self.sensor_list, "等待传感器数据")
            return

        self.sensor_list.clear()
        for entry in sensor_status:
            name = entry.get('name', 'Sensor')
            status_text = entry.get('status', '--')
            detail = entry.get('detail')
            combined = f"{name}: {status_text}"
            if detail:
                combined += f"  ({detail})"
            item = QListWidgetItem(combined)
            if detail:
                item.setToolTip(detail)
            bg_color, fg_color = self._level_to_palette(entry.get('level'))
            item.setBackground(QColor(bg_color))
            item.setForeground(QColor(fg_color))
            self.sensor_list.addItem(item)

    def _update_vehicle_messages(self, messages):
        """更新飞控消息列表"""
        if not hasattr(self, 'message_list'):
            return

        if not messages:
            self._set_list_placeholder(self.message_list, "尚未收到飞控消息")
            return

        self.message_list.clear()
        max_items = 30
        for entry in messages[:max_items]:
            severity = entry.get('severity', 6)
            label = entry.get('severity_label') or f"LEVEL {severity}"
            time_str = entry.get('time') or "--:--:--"
            text = entry.get('text', '')
            combined = f"[{time_str}] {label}: {text}"
            item = QListWidgetItem(combined)
            item.setToolTip(text)
            bg_color, fg_color = self._severity_palette(severity)
            item.setBackground(QColor(bg_color))
            item.setForeground(QColor(fg_color))
            self.message_list.addItem(item)
    
    def _update_battery_style(self, percentage):
        """根据电池百分比更新进度条样式"""
        if percentage > 60:
            color = "#27ae60"  # 绿色
        elif percentage > 30:
            color = "#f39c12"  # 橙色
        else:
            color = "#e74c3c"  # 红色
        
        self.battery_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                text-align: center;
                height: 25px;
                font-weight: bold;
                font-size: 16px;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 3px;
            }}
        """)
    
    def _update_temperature_style(self, temp_celsius):
        """
        根据温度更新样式（带滞后效果）
        
        滞后逻辑：
        - 温度 >= 50°C 时切换到红色
        - 温度 < 48°C 时切换到绿色
        - 在 48-50°C 之间保持当前颜色（2°C死区）
        
        这样可以防止温度在50°C附近波动时颜色频繁闪烁
        
        Args:
            temp_celsius: 温度（摄氏度）
        """
        try:
            temp = float(temp_celsius)
            
            # 滞后逻辑实现
            if self._is_high_temperature:
                # 当前是高温状态（红色）
                if temp < 48:  # 温度降到48°C以下才切换到绿色
                    color = "#27ae60"  # 绿色
                    self._is_high_temperature = False
                else:
                    color = "#e74c3c"  # 保持红色
            else:
                # 当前是低温状态（绿色）
                if temp >= 50:  # 温度升到50°C及以上才切换到红色
                    color = "#e74c3c"  # 红色
                    self._is_high_temperature = True
                else:
                    color = "#27ae60"  # 保持绿色
            
            self.temperature_label.setStyleSheet(f"""
                QLabel {{
                    color: {color};
                    font-weight: bold;
                    font-size: 16px;
                }}
            """)
        except (ValueError, TypeError):
            self.temperature_label.setStyleSheet("")
    
    def _update_satellite_style(self, satellite_count):
        """根据卫星数量更新样式"""
        try:
            count = int(satellite_count)
            if count >= 4:
                color = "#27ae60"  # 绿色 - 正常（4颗及以上可定位）
            else:
                color = "#e74c3c"  # 红色 - 信号弱（少于4颗无法定位）
            
            self.satellite_label.setStyleSheet(f"""
                QLabel {{
                    color: white;
                    background-color: {color};
                    font-weight: bold;
                    padding: 3px 8px;
                    border-radius: 3px;
                    font-size: 16px;
                }}
            """)
        except (ValueError, TypeError):
            self.satellite_label.setStyleSheet("""
                QLabel {
                    color: #34495e;
                    font-size: 16px;
                    font-weight: 600;
                }
            """)

    def _level_to_palette(self, level):
        """根据 level 返回背景/前景颜色"""
        key = str(level).lower() if level is not None else ''
        mapping = {
            'ok': ("#ecfdf3", "#1d8348"),
            'warn': ("#fff6e5", "#b9770e"),
            'warning': ("#fff6e5", "#b9770e"),
            'error': ("#fdecea", "#c0392b"),
            'critical': ("#fdecea", "#c0392b"),
        }
        return mapping.get(key, ("#f4f6f7", "#2c3e50"))

    def _severity_palette(self, severity):
        """根据 MAVROS severity 返回配色"""
        try:
            sev = int(severity)
        except (ValueError, TypeError):
            sev = 6

        if sev <= 2:
            return "#fdecea", "#c0392b"
        if sev in (3, 4):
            return "#fef5e6", "#b9770e"
        if sev == 5:
            return "#ebf5fb", "#1f618d"
        if sev == 6:
            return "#f4f6f7", "#2c3e50"
        return "#f4f6f7", "#2c3e50"
    
    def _update_dynamic_styles(self):
        """更新动态样式（由定时器调用）"""
        # 可以在这里添加动画效果或闪烁提醒
        # 例如：低电量时闪烁、GPS信号差时闪烁等
        pass
