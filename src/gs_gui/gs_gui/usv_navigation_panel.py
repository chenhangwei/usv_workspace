#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of usv navigation panel.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
USV 导航信息面板模块
提供美观、信息丰富的 USV 导航详细信息显示界面
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                              QGroupBox, QGridLayout, QFrame, QScrollArea, 
                              QSizePolicy)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor
import math
from .compass_widget import CompassWidget
from gs_gui.style_manager import is_dark_theme

# 兼容性定义
try:
    AlignRight = Qt.AlignmentFlag.AlignRight  # type: ignore
    AlignLeft = Qt.AlignmentFlag.AlignLeft  # type: ignore
    AlignVCenter = Qt.AlignmentFlag.AlignVCenter  # type: ignore
except AttributeError:
    AlignRight = Qt.AlignRight  # type: ignore
    AlignLeft = Qt.AlignLeft  # type: ignore
    AlignVCenter = Qt.AlignVCenter  # type: ignore


class AngleSmoother:
    """
    角度平滑器
    使用指数移动平均(EMA)处理角度数据，解决 0/360 度跳变问题
    """
    def __init__(self, alpha=0.15):
        self.alpha = alpha
        self.current_rad = None

    def update(self, target_deg):
        """
        更新角度值
        Args:
            target_deg: 目标角度（度）
        Returns:
            平滑后的角度（度，0-360）
        """
        if target_deg is None:
            return 0.0
            
        target_rad = math.radians(target_deg)
        
        if self.current_rad is None:
            self.current_rad = target_rad
            return target_deg
            
        # 计算角度差（使用 atan2 处理周期性，确保是最短路径）
        # diff 范围是 -pi 到 pi
        diff = math.atan2(math.sin(target_rad - self.current_rad), 
                         math.cos(target_rad - self.current_rad))
        
        # 更新当前角度
        self.current_rad += self.alpha * diff
        
        # 返回度数 (0-360)
        return math.degrees(self.current_rad) % 360.0

    def reset(self):
        self.current_rad = None


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
            font-size: 16px;
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

    def _get_groupbox_style(self, border_color="#e67e22"):
        """根据主题返回 QGroupBox 样式"""
        dark = is_dark_theme()
        title_color = "#e0e0e0" if dark else "#333333"
        return f"""
            QGroupBox {{
                font-weight: bold;
                font-size: 16px;
                border: 1.5px solid {border_color};
                border-radius: 5px;
                margin-top: 6px;
                padding-top: 6px;
                color: {title_color};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }}
        """

    def _get_scrollbar_stylesheet(self):
        """根据主题返回滚动条样式"""
        dark = is_dark_theme()
        if dark:
            bg = "#2c3e50"
            handle = "#e67e22"
            handle_hover = "#f39c12"
        else:
            bg = "#e8e8e8"
            handle = "#b0b0b0"
            handle_hover = "#909090"
        return f"""
            QScrollArea {{
                border: none;
                background-color: transparent;
            }}
            QScrollBar:vertical {{
                border: none;
                background: {bg};
                width: 10px;
                margin: 0px;
                border-radius: 5px;
            }}
            QScrollBar::handle:vertical {{
                background: {handle};
                min-height: 30px;
                border-radius: 5px;
            }}
            QScrollBar::handle:vertical:hover {{
                background: {handle_hover};
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                border: none;
                background: none;
                height: 0px;
            }}
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
                background: none;
            }}
            QScrollBar:horizontal {{
                border: none;
                background: {bg};
                height: 10px;
                margin: 0px;
                border-radius: 5px;
            }}
            QScrollBar::handle:horizontal {{
                background: {handle};
                min-width: 30px;
                border-radius: 5px;
            }}
            QScrollBar::handle:horizontal:hover {{
                background: {handle_hover};
            }}
            QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
                border: none;
                background: none;
                width: 0px;
            }}
            QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
                background: none;
            }}
        """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 当前导航状态缓存
        self._current_navigation_state = None
        self._current_feedback = None
        
        # 初始化平滑器 (alpha越小越平滑但延迟越高)
        self.heading_smoother = AngleSmoother(alpha=0.15)
        # 误差平滑器 (误差可能在+/-180跳变，也作为角度处理)
        self.error_smoother = AngleSmoother(alpha=0.15) 
        
        # 设置主布局（包含滚动区域）
        self._setup_ui()

    def set_theme(self, theme_name):
        """切换主题时刷新面板内硬编码的样式"""
        if hasattr(self, '_scroll_area'):
            self._scroll_area.setStyleSheet(self._get_scrollbar_stylesheet())
        group_colors = {
            'velocity_group': '#e67e22',
            'heading_group': '#3498db',
            'mission_group': '#27ae60',
            'feedback_group': '#9b59b6',
        }
        for attr, color in group_colors.items():
            w = getattr(self, attr, None)
            if w:
                w.setStyleSheet(self._get_groupbox_style(color))
        dark = is_dark_theme()
        key_color = "#b0b0b0" if dark else "#7f8c8d"
        val_color = "#e0e0e0" if dark else "#34495e"
        for label in getattr(self, '_key_labels', []):
            label.setStyleSheet(f"QLabel {{ color: {key_color}; font-weight: bold; font-size: 16px; min-width: 70px; }}")
        for label in getattr(self, '_value_labels', []):
            label.setStyleSheet(f"QLabel {{ color: {val_color}; font-size: 16px; font-weight: 600; }}")
    
    def _setup_ui(self):
        """设置UI布局（带滚动条）"""
        # 用于 set_theme() 批量刷新
        self._key_labels = []
        self._value_labels = []

        # 主容器布局（外层）
        main_container_layout = QVBoxLayout(self)
        main_container_layout.setContentsMargins(0, 0, 0, 0)
        main_container_layout.setSpacing(0)
        
        # 创建滚动区域
        self._scroll_area = QScrollArea()
        self._scroll_area.setWidgetResizable(True)
        self._scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self._scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self._scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        
        # 创建滚动内容容器
        scroll_content = QWidget()
        scroll_content.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        
        # 内容布局
        content_layout = QVBoxLayout(scroll_content)
        content_layout.setContentsMargins(10, 10, 10, 10)
        content_layout.setSpacing(10)
        
        # ==================== 速度信息组 ====================
        self.velocity_group = self._create_velocity_info_group()
        content_layout.addWidget(self.velocity_group)
        
        # ==================== 航向信息组 ====================
        self.heading_group = self._create_heading_info_group()
        content_layout.addWidget(self.heading_group)
        
        # ==================== 任务信息组 ====================
        self.mission_group = self._create_mission_info_group()
        content_layout.addWidget(self.mission_group)
        
        # ==================== 导航反馈组 ====================
        self.feedback_group = self._create_navigation_feedback_group()
        content_layout.addWidget(self.feedback_group)
        
        # 添加弹性空间（自动填充剩余空间）
        content_layout.addStretch()
        
        # 将内容容器设置到滚动区域
        self._scroll_area.setWidget(scroll_content)
        
        # 将滚动区域添加到主布局
        main_container_layout.addWidget(self._scroll_area)
        
        # 设置滚动条样式（主题感知）
        self._scroll_area.setStyleSheet(self._get_scrollbar_stylesheet())
    
    def _create_velocity_info_group(self):
        """创建速度信息组"""
        group = QGroupBox("🚤 速度信息")
        group.setStyleSheet(self._get_groupbox_style("#e67e22"))
        
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
        group.setStyleSheet(self._get_groupbox_style("#3498db"))
        
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
        group.setStyleSheet(self._get_groupbox_style("#27ae60"))
        
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

        # 到达阈值（米）
        self.arrival_threshold_label = self._create_value_label("--")
        layout.addWidget(self._create_key_label("到达阈值(设定):"), 4, 0)
        layout.addWidget(self.arrival_threshold_label, 4, 1)
        layout.addWidget(QLabel("m"), 4, 2)
        
        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group
    
    def _create_navigation_feedback_group(self):
        """创建导航反馈组"""
        group = QGroupBox("📡 导航反馈")
        group.setStyleSheet(self._get_groupbox_style("#9b59b6"))
        
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
        dark = is_dark_theme()
        key_color = "#b0b0b0" if dark else "#7f8c8d"
        label.setStyleSheet(f"""
            QLabel {{
                color: {key_color};
                font-weight: bold;
                font-size: 16px;
                min-width: 70px;
            }}
        """)
        self._key_labels.append(label)
        return label
    
    def _create_value_label(self, text):
        """创建值标签（响应式字体）"""
        label = QLabel(text)
        if hasattr(Qt, "AlignmentFlag"):
            alignment = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        else:
            alignment = AlignLeft | AlignVCenter
        label.setAlignment(alignment)  # type: ignore[arg-type]
        dark = is_dark_theme()
        val_color = "#e0e0e0" if dark else "#34495e"
        label.setStyleSheet(f"""
            QLabel {{
                color: {val_color};
                font-size: 16px;
                font-weight: 600;
            }}
        """)
        self._value_labels.append(label)
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
            # 1. 获取并平滑当前航向
            current_heading = None
            try:
                raw_heading = float(state.get('heading', 0.0))
                # 使用平滑器处理
                current_heading = self.heading_smoother.update(raw_heading)
                self.current_heading_label.setText(self._format_float(current_heading, precision=1))
            except (ValueError, TypeError):
                self.current_heading_label.setText("--")
                self.heading_smoother.reset()
            
            # 2. 获取并平滑航向误差，然后计算目标航向
            target_heading = None
            heading_error = None
            
            if feedback is not None and hasattr(feedback, 'heading_error'):
                try:
                    # 误差也是角度性质的，需要平滑
                    raw_error = float(feedback.heading_error)
                    heading_error = raw_error # 显示用的误差可以保持原始值或也平滑
                    
                    # 为了计算稳定的目标航向，我们使用平滑后的误差
                    # 注意：这里我们单独维护误差的平滑，或者直接用平滑后的航向+原始误差计算目标航向再平滑
                    # 策略：Target = Smooth(Current) + Smooth(Error)
                    # 但Error的平滑需要特殊处理，因为它是在0附近震荡。
                    # 实际上，Error本身就是 Target - Current。
                    # 如果我们信任 Feedback 中的 Error，那么 Target = Current + Error。
                    # 为了稳定，我们计算出瞬时 Target，然后对 Target 进行平滑。
                    
                    if current_heading is not None:
                        # 瞬时目标航向
                        inst_target = raw_heading + raw_error
                        
                        # 我们不需要单独平滑 Target，因为如果我们平滑了 Current，
                        # 且假设 Error 只是控制偏差，直接加会导致 Target 随 Current 波动。
                        # 正确的做法是：由于 Target 通常是固定的（或者变化很慢），
                        # 我们应该直接根据 (Current + Error) 计算出 Target，然后对 Target 进行强力平滑。
                        
                        # 这里我们复用 error_smoother 来作为 "Target Heading Smoother"
                        # 改名可能更合适，但为了最小修改，我们直接用它存 Target 的状态
                        target_heading = self.error_smoother.update(inst_target)
                        
                        self.target_heading_label.setText(self._format_float(target_heading, precision=1))
                    else:
                        self.target_heading_label.setText("--")
                        self.error_smoother.reset()
                except (ValueError, TypeError, AttributeError):
                    self.target_heading_label.setText("--")
                    self.error_smoother.reset()
            else:
                self.target_heading_label.setText("--")
                # 如果没有反馈，重置平滑器状态，避免保留旧值
                self.error_smoother.reset() 
            
            # 更新罗盘显示
            if current_heading is not None:
                # 重新计算用于显示的误差 (Target - Current) 以保持一致性
                display_error = heading_error # 默认显示原始误差
                if target_heading is not None:
                    # 计算平滑后的显示误差
                    diff_rad = math.radians(target_heading) - math.radians(current_heading)
                    diff_rad = math.atan2(math.sin(diff_rad), math.cos(diff_rad))
                    display_error = math.degrees(diff_rad)
                
                self.compass_widget.set_heading(current_heading, target_heading, display_error)
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

            # 到达阈值（从 state 中获取）
            try:
                threshold = state.get('nav_arrival_threshold', None)
                if threshold is None:
                    self.arrival_threshold_label.setText("--")
                else:
                    threshold_f = float(threshold)
                    if threshold_f > 0.0:
                        self.arrival_threshold_label.setText(self._format_float(threshold_f, precision=2))
                    else:
                        self.arrival_threshold_label.setText("--")
            except Exception:
                self.arrival_threshold_label.setText("--")
            
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
            print(f"更新 USV 导航面板失败: {e}")
    
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
        self.arrival_threshold_label.setText("--")
        
        # 导航反馈
        self.distance_to_goal_label.setText("--")
        self.estimated_time_label.setText("--")
        self.nav_status_label.setText("--")
        
        self._current_navigation_state = None
        self._current_feedback = None
        
        # 重置平滑器
        self.heading_smoother.reset()
        self.error_smoother.reset()
    
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
