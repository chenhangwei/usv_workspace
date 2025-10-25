"""
USV 信息面板模块
提供美观、信息丰富的 USV 详细信息显示界面
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                              QGroupBox, QGridLayout, QFrame, QProgressBar,
                              QScrollArea, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QFont, QColor, QPalette

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
            font-size: 11px;
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
        group = QGroupBox("📌 基本信息")
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
                font-size: 12px;
                background-color: #ecf0f1;
                padding: 5px;
                border-radius: 3px;
            }
        """)
        layout.addWidget(QLabel("🆔 USV ID:"), 0, 0)
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
        layout.addWidget(QLabel("🎯 模式:"), 1, 0)
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
        layout.addWidget(QLabel("📊 状态:"), 2, 0)
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
        layout.addWidget(QLabel("🔓 解锁:"), 3, 0)
        layout.addWidget(self.armed_label, 3, 1)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group
    
    def _create_position_info_group(self):
        """创建位置信息组"""
        group = QGroupBox("🗺️ 位置信息")
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
        group = QGroupBox("🔋 电池信息")
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
                font-size: 11px;
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
        group = QGroupBox("🛰️ GPS 信息")
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
        """创建速度信息组"""
        group = QGroupBox("💨 速度信息")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#3498db", "#e74c3c"))
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # 地速
        self.ground_speed_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("地速:"), 0, 0)
        layout.addWidget(self.ground_speed_label, 0, 1)
        layout.addWidget(QLabel("m/s"), 0, 2)
        
        # 航向速度
        self.heading_speed_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("航向:"), 1, 0)
        layout.addWidget(self.heading_speed_label, 1, 1)
        layout.addWidget(QLabel("°"), 1, 2)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group
    
    def _create_key_label(self, text):
        """创建键标签（紧凑版）"""
        label = QLabel(text)
        label.setAlignment(AlignRight | AlignVCenter)
        label.setStyleSheet("""
            QLabel {
                color: #7f8c8d;
                font-weight: bold;
                font-size: 11px;
                min-width: 40px;
            }
        """)
        return label
    
    def _create_value_label(self, text, large=False):
        """创建值标签（响应式字体）"""
        label = QLabel(text)
        label.setAlignment(AlignLeft | AlignVCenter)
        if large:
            label.setStyleSheet("""
                QLabel {
                    color: #2c3e50;
                    font-size: 13px;
                    font-weight: bold;
                }
            """)
        else:
            label.setStyleSheet("""
                QLabel {
                    color: #34495e;
                    font-size: 11px;
                    font-weight: 600;
                }
            """)
        return label
    
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
            # 更新基本信息
            self.id_label.setText(str(state.get('namespace', '--')))
            
            mode = state.get('mode', '--')
            self.mode_label.setText(str(mode))
            self._update_mode_style(mode)
            
            # 连接状态作为 status
            connected = state.get('connected', False)
            status = "在线" if connected else "离线"
            self.status_label.setText(str(status))
            self._update_status_style(status)
            
            armed = state.get('armed', False)
            self.armed_label.setText(str(armed))
            self._update_armed_style(armed)
            
            # 更新位置信息
            pos = state.get('position', {}) or {}
            self.x_label.setText(self._format_float(pos.get('x')))
            self.y_label.setText(self._format_float(pos.get('y')))
            self.z_label.setText(self._format_float(pos.get('z')))
            
            yaw = state.get('yaw')
            self.yaw_label.setText(self._format_float(yaw, precision=1))
            
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
            
            # 电流字段不存在，显示为 --
            self.current_label.setText("--")
            
            # 温度信息（除以1000转换为摄氏度）
            try:
                temp_raw = float(state.get('temperature', 0.0))
                temp_celsius = temp_raw / 1000.0
                self.temperature_label.setText(self._format_float(temp_celsius, precision=1))
                self._update_temperature_style(temp_celsius)
            except (ValueError, TypeError):
                self.temperature_label.setText("--")
            
            # GPS 信息字段不存在，显示为 --
            self.satellite_label.setText("--")
            self.gps_accuracy_label.setText("--")
            
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
            
            # 航向（从 yaw 获取，转换为度）
            try:
                import math
                yaw_rad = float(state.get('yaw', 0.0))
                heading_deg = math.degrees(yaw_rad)
                self.heading_speed_label.setText(self._format_float(heading_deg, precision=1))
            except (ValueError, TypeError):
                self.heading_speed_label.setText("--")
            
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
        
        # 重置温度状态标志
        self._is_high_temperature = False
        
        self._current_state = None
    
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
                font-size: 13px;
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
                    font-size: 14px;
                }}
            """)
        except (ValueError, TypeError):
            self.temperature_label.setStyleSheet("")
    
    def _update_satellite_style(self, satellite_count):
        """根据卫星数量更新样式"""
        try:
            count = int(satellite_count)
            if count >= 10:
                color = "#27ae60"  # 绿色
            elif count >= 6:
                color = "#f39c12"  # 橙色
            else:
                color = "#e74c3c"  # 红色
            
            self.satellite_label.setStyleSheet(f"""
                QLabel {{
                    color: white;
                    background-color: {color};
                    font-weight: bold;
                    padding: 3px 8px;
                    border-radius: 3px;
                    font-size: 13px;
                }}
            """)
        except (ValueError, TypeError):
            self.satellite_label.setStyleSheet("""
                QLabel {
                    color: #34495e;
                    font-size: 13px;
                    font-weight: 600;
                }
            """)
    
    def _update_dynamic_styles(self):
        """更新动态样式（由定时器调用）"""
        # 可以在这里添加动画效果或闪烁提醒
        # 例如：低电量时闪烁、GPS信号差时闪烁等
        pass
