"""
USV 导航信息面板模块
提供美观、信息丰富的 USV 导航详细信息显示界面
"""

import logging
import math
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                              QGroupBox, QGridLayout, QFrame, QScrollArea, 
                              QSizePolicy)
from PyQt5.QtCore import Qt
from .compass_widget import CompassWidget
from .angle_utils import rad_to_normalized_deg_0_360

_logger = logging.getLogger("gs_gui.nav_panel")


# 兼容性定义
try:
    AlignRight = Qt.AlignmentFlag.AlignRight  # type: ignore
    AlignLeft = Qt.AlignmentFlag.AlignLeft  # type: ignore
    AlignVCenter = Qt.AlignmentFlag.AlignVCenter  # type: ignore
except AttributeError:
    AlignRight = Qt.AlignRight  # type: ignore
    AlignLeft = Qt.AlignLeft  # type: ignore
    AlignVCenter = Qt.AlignVCenter  # type: ignore


class UsvNavigationPanel(QWidget):
    """
    USV 导航信息面板（响应式设计）
    
    提供美观的 USV 导航详细信息显示，包括：
    - 速度信息（地速、垂直速度、总速度）
    - 航向信息（当前航向、目标航向、航向误差）
    - 任务信息（当前 Step、目标点坐标）
    - 导航反馈（距离目标点、预计剩余时间）
    
    特性：
    - 滚动条支持：内容超出时自动显示滚动条
    - 响应式布局：小窗口下自动调整字体和间距
    - 弹性设计：避免内容被压扁
    """
    
    # 统一的 QGroupBox 样式（紧凑版）
    GROUPBOX_STYLE = """
        QGroupBox {
            font-weight: bold;
            font-size: 14px;
            border: 1.5px solid #e67e22;
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
        
        # 当前导航状态缓存
        self._current_navigation_state = None
        self._current_feedback = None
        
        # 设置主布局（包含滚动区域）
        self._setup_ui()
    
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
        
        # ==================== 速度信息组 ====================
        velocity_group = self._create_velocity_info_group()
        content_layout.addWidget(velocity_group)
        
        # ==================== 航向信息组 ====================
        heading_group = self._create_heading_info_group()
        content_layout.addWidget(heading_group)
        
        # ==================== 任务信息组 ====================
        mission_group = self._create_mission_info_group()
        content_layout.addWidget(mission_group)
        
        # ==================== 导航反馈组 ====================
        feedback_group = self._create_navigation_feedback_group()
        content_layout.addWidget(feedback_group)
        
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
                background: #e67e22;
                min-height: 30px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical:hover {
                background: #f39c12;
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
                background: #e67e22;
                min-width: 30px;
                border-radius: 5px;
            }
            QScrollBar::handle:horizontal:hover {
                background: #f39c12;
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
    
    def _create_velocity_info_group(self):
        """创建速度信息组"""
        group = QGroupBox("🚤 速度信息")
        group.setStyleSheet(self.GROUPBOX_STYLE)
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # 地速（水平速度）
        self.ground_speed_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("地速:"), 0, 0)
        layout.addWidget(self.ground_speed_label, 0, 1)
        layout.addWidget(QLabel("m/s"), 0, 2)
        
        # 垂直速度
        self.vertical_speed_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("垂直速度:"), 1, 0)
        layout.addWidget(self.vertical_speed_label, 1, 1)
        layout.addWidget(QLabel("m/s"), 1, 2)
        
        # 总速度
        self.total_speed_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("总速度:"), 2, 0)
        layout.addWidget(self.total_speed_label, 2, 1)
        layout.addWidget(QLabel("m/s"), 2, 2)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group
    
    def _create_heading_info_group(self):
        """创建航向信息组（带罗盘显示）"""
        group = QGroupBox("🧭 航向信息")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#e67e22", "#3498db"))
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 12, 10, 10)
        
        # 上半部分：文字信息
        info_layout = QGridLayout()
        info_layout.setSpacing(5)
        
        # 当前航向
        self.current_heading_label = self._create_value_label("--")
        info_layout.addWidget(self._create_key_label("当前航向:"), 0, 0)
        info_layout.addWidget(self.current_heading_label, 0, 1)
        info_layout.addWidget(QLabel("°"), 0, 2)
        
        # 目标航向（如果有的话）
        self.target_heading_label = self._create_value_label("--")
        info_layout.addWidget(self._create_key_label("目标航向:"), 1, 0)
        info_layout.addWidget(self.target_heading_label, 1, 1)
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
    
    def _create_mission_info_group(self):
        """创建任务信息组"""
        group = QGroupBox("📋 任务信息")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#e67e22", "#27ae60"))
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # 当前 Step（如果集群任务正在执行）
        self.current_step_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("当前 Step:"), 0, 0)
        layout.addWidget(self.current_step_label, 0, 1, 1, 2)
        
        # 目标点 X 坐标
        self.target_x_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("目标 X:"), 1, 0)
        layout.addWidget(self.target_x_label, 1, 1)
        layout.addWidget(QLabel("m"), 1, 2)
        
        # 目标点 Y 坐标
        self.target_y_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("目标 Y:"), 2, 0)
        layout.addWidget(self.target_y_label, 2, 1)
        layout.addWidget(QLabel("m"), 2, 2)
        
        # 目标点 Z 坐标
        self.target_z_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("目标 Z:"), 3, 0)
        layout.addWidget(self.target_z_label, 3, 1)
        layout.addWidget(QLabel("m"), 3, 2)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group
    
    def _create_navigation_feedback_group(self):
        """创建导航反馈组"""
        group = QGroupBox("📡 导航反馈")
        group.setStyleSheet(self.GROUPBOX_STYLE.replace("#e67e22", "#9b59b6"))
        
        layout = QGridLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(10, 12, 10, 10)
        
        # 距离目标点
        self.distance_to_goal_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("距离目标:"), 0, 0)
        layout.addWidget(self.distance_to_goal_label, 0, 1)
        layout.addWidget(QLabel("m"), 0, 2)
        
        # 预计剩余时间
        self.estimated_time_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("预计时间:"), 1, 0)
        layout.addWidget(self.estimated_time_label, 1, 1)
        layout.addWidget(QLabel("s"), 1, 2)
        
        # 导航状态
        self.nav_status_label = self._create_value_label("--")
        self.nav_status_label.setStyleSheet("""
            QLabel {
                font-weight: bold;
                padding: 5px;
                border-radius: 4px;
            }
        """)
        layout.addWidget(self._create_key_label("导航状态:"), 2, 0)
        layout.addWidget(self.nav_status_label, 2, 1, 1, 2)
        
        layout.setColumnStretch(1, 1)
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
                font-size: 14px;
                min-width: 70px;
            }
        """)
        return label
    
    def _create_value_label(self, text):
        """创建值标签（响应式字体）"""
        label = QLabel(text)
        if hasattr(Qt, "AlignmentFlag"):
            alignment = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        else:
            alignment = AlignLeft | AlignVCenter
        label.setAlignment(alignment)  # type: ignore[arg-type]
        label.setStyleSheet("""
            QLabel {
                color: #34495e;
                font-size: 14px;
                font-weight: 600;
            }
        """)
        return label
    
    def update_navigation_state(self, state, feedback=None, nav_status=None):
        """
        更新 USV 导航状态显示
        
        Args:
            state: USV 状态字典，包含速度、航向、位置等信息
            feedback: 导航反馈数据对象,包含距离、航向误差、预计时间等
            nav_status: 导航状态字符串（"执行中"、"成功"、"失败"等）
        """
        if state is None:
            self._clear_display()
            return
        
        self._current_navigation_state = state
        self._current_feedback = feedback
        
        try:
            # ==================== 更新速度信息 ====================
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
            
            # 垂直速度
            try:
                vz = float(linear.get('z', 0.0))
                self.vertical_speed_label.setText(self._format_float(vz, precision=2))
            except (ValueError, TypeError):
                self.vertical_speed_label.setText("--")
            
            # 总速度（3D 速度）
            try:
                vx = float(linear.get('x', 0.0))
                vy = float(linear.get('y', 0.0))
                vz = float(linear.get('z', 0.0))
                total_speed = (vx ** 2 + vy ** 2 + vz ** 2) ** 0.5
                self.total_speed_label.setText(self._format_float(total_speed, precision=2))
            except (ValueError, TypeError):
                self.total_speed_label.setText("--")
            
            # ==================== 更新航向信息 ====================
            # 当前航向（UsvStatus.heading 单位为弧度；此处转换为度用于显示/罗盘）
            current_heading = None
            try:
                heading_rad = float(state.get('heading', 0.0))
                heading_deg = rad_to_normalized_deg_0_360(heading_rad)
                current_heading = heading_deg
                self.current_heading_label.setText(self._format_float(heading_deg, precision=1))
            except (ValueError, TypeError):
                self.current_heading_label.setText("--")
            
            # 目标航向和航向误差（从导航反馈中获取）
            target_heading = None
            heading_error = None
            
            if feedback is not None and hasattr(feedback, 'heading_error'):
                try:
                    heading_error = float(feedback.heading_error)
                    
                    # 计算目标航向 = 当前航向 + 航向误差
                    if current_heading is not None:
                        target_heading = current_heading + heading_error
                        # 归一化到 0-360 度
                        target_heading = target_heading % 360.0
                        self.target_heading_label.setText(self._format_float(target_heading, precision=1))
                    else:
                        self.target_heading_label.setText("--")
                except (ValueError, TypeError, AttributeError):
                    self.target_heading_label.setText("--")
            else:
                self.target_heading_label.setText("--")
            
            # 更新罗盘显示（传递当前航向、目标航向和航向误差）
            if current_heading is not None:
                self.compass_widget.set_heading(current_heading, target_heading, heading_error)
            else:
                self.compass_widget.set_heading(0.0)
            
            # ==================== 更新任务信息 ====================
            # 当前 Step 和目标点坐标（从 state 中获取缓存的导航目标信息）
            nav_target = state.get('nav_target_cache')
            if nav_target and isinstance(nav_target, dict):
                # 显示当前 Step
                step = nav_target.get('step')
                if step is not None:
                    self.current_step_label.setText(str(step))
                else:
                    self.current_step_label.setText("--")
                
                # 显示目标点坐标
                try:
                    x = float(nav_target.get('x', 0.0))
                    y = float(nav_target.get('y', 0.0))
                    z = float(nav_target.get('z', 0.0))
                    self.target_x_label.setText(self._format_float(x, precision=2))
                    self.target_y_label.setText(self._format_float(y, precision=2))
                    self.target_z_label.setText(self._format_float(z, precision=2))
                except (ValueError, TypeError):
                    self.target_x_label.setText("--")
                    self.target_y_label.setText("--")
                    self.target_z_label.setText("--")
            else:
                # 没有导航目标信息
                self.current_step_label.setText("--")
                self.target_x_label.setText("--")
                self.target_y_label.setText("--")
                self.target_z_label.setText("--")
            
            # ==================== 更新导航反馈 ====================
            if feedback is not None:
                # 距离目标点
                if hasattr(feedback, 'distance_to_goal'):
                    try:
                        distance = float(feedback.distance_to_goal)
                        self.distance_to_goal_label.setText(self._format_float(distance, precision=2))
                    except (ValueError, TypeError, AttributeError):
                        self.distance_to_goal_label.setText("--")
                else:
                    self.distance_to_goal_label.setText("--")
                
                # 预计剩余时间
                if hasattr(feedback, 'estimated_time'):
                    try:
                        time_remaining = float(feedback.estimated_time)
                        self.estimated_time_label.setText(self._format_float(time_remaining, precision=1))
                    except (ValueError, TypeError, AttributeError):
                        self.estimated_time_label.setText("--")
                else:
                    self.estimated_time_label.setText("--")
            else:
                self.distance_to_goal_label.setText("--")
                self.estimated_time_label.setText("--")
            
            # 导航状态
            if nav_status:
                self.nav_status_label.setText(str(nav_status))
                self._update_nav_status_style(nav_status)
            else:
                self.nav_status_label.setText("空闲")
                self._update_nav_status_style("空闲")
                
        except Exception as e:
            _logger.error(f"更新 USV 导航面板失败: {e}")
    
    def _clear_display(self):
        """清空显示"""
        # 速度信息
        self.ground_speed_label.setText("--")
        self.vertical_speed_label.setText("--")
        self.total_speed_label.setText("--")
        
        # 航向信息
        self.current_heading_label.setText("--")
        self.target_heading_label.setText("--")
        self.compass_widget.set_heading(0.0)  # 重置罗盘显示
        
        # 任务信息
        self.current_step_label.setText("--")
        self.target_x_label.setText("--")
        self.target_y_label.setText("--")
        self.target_z_label.setText("--")
        
        # 导航反馈
        self.distance_to_goal_label.setText("--")
        self.estimated_time_label.setText("--")
        self.nav_status_label.setText("--")
        
        self._current_navigation_state = None
        self._current_feedback = None
    
    def _format_float(self, value, precision=2):
        """格式化浮点数"""
        try:
            if value is None or value == '--':
                return "--"
            return f"{float(value):.{precision}f}"
        except (ValueError, TypeError):
            return "--"
    
    def _update_nav_status_style(self, status):
        """根据导航状态更新样式"""
        status_str = str(status).upper()
        if "执行中" in status_str or "ACTIVE" in status_str:
            color = "#3498db"  # 蓝色
        elif "成功" in status_str or "SUCCESS" in status_str:
            color = "#27ae60"  # 绿色
        elif "失败" in status_str or "FAILED" in status_str or "失败" in status_str:
            color = "#e74c3c"  # 红色
        else:
            color = "#95a5a6"  # 灰色（空闲状态）
        
        self.nav_status_label.setStyleSheet(f"""
            QLabel {{
                color: white;
                background-color: {color};
                font-weight: bold;
                padding: 5px;
                border-radius: 4px;
            }}
        """)
