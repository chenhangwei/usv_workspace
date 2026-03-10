#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of velocity settings dialog.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
速度控制器设置对话框 - 在线参数配置

支持在线设置:
- 巡航速度 (cruise_speed)
- 到达阈值 (goal_tolerance)
- 切换阈值 (switch_tolerance)
- 最大角速度 (max_angular_velocity)
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QPushButton, QGroupBox,
    QFormLayout, QFrame, QComboBox, QTabWidget, QWidget
)
from PyQt5.QtCore import Qt
from gs_gui.style_manager import is_dark_theme


class VelocitySettingsDialog(QDialog):
    """速度控制器设置对话框"""
    
    def __init__(self, parent=None, current_settings=None):
        """
        初始化对话框
        
        Args:
            parent: 父窗口
            current_settings: 当前设置字典
        """
        super().__init__(parent)
        self.setWindowTitle("速度控制器参数设置")
        self.resize(500, 550)
        
        # 默认设置
        self.current_settings = current_settings or {
            'cruise_speed': 0.5,
            'max_angular_velocity': 0.5,
            'goal_tolerance': 0.5,
            'switch_tolerance': 1.5,
        }
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout()
        
        # ==================== 说明 ====================
        info_label = QLabel(
            "配置 USV 速度控制器 (MPC) 的参数。\n"
            "速度模式直接发送速度指令，可实现平滑连续导航。"
        )
        info_label.setWordWrap(True)
        _info_color = '#AAAAAA' if is_dark_theme() else '#666666'
        info_label.setStyleSheet(f"color: {_info_color}; margin-bottom: 10px; font-size: 12px;")
        layout.addWidget(info_label)
        
        # ==================== 选项卡 ====================
        tab_widget = QTabWidget()
        
        # --- 速度设置 Tab ---
        speed_tab = QWidget()
        speed_layout = QFormLayout()
        speed_layout.setSpacing(12)
        
        # 巡航速度
        self.cruise_speed_spin = QDoubleSpinBox()
        self.cruise_speed_spin.setRange(0.1, 3.0)
        self.cruise_speed_spin.setSingleStep(0.1)
        self.cruise_speed_spin.setDecimals(2)
        self.cruise_speed_spin.setValue(self.current_settings.get('cruise_speed', 0.5))
        self.cruise_speed_spin.setSuffix(" m/s")
        self.cruise_speed_spin.setToolTip("默认巡航速度，USV 追踪路径时的目标速度")
        speed_layout.addRow("巡航速度:", self.cruise_speed_spin)
        
        # 最大角速度
        self.max_angular_spin = QDoubleSpinBox()
        self.max_angular_spin.setRange(0.1, 2.0)
        self.max_angular_spin.setSingleStep(0.1)
        self.max_angular_spin.setDecimals(2)
        self.max_angular_spin.setValue(self.current_settings.get('max_angular_velocity', 0.5))
        self.max_angular_spin.setSuffix(" rad/s")
        self.max_angular_spin.setToolTip("转向时的最大角速度限制")
        speed_layout.addRow("最大角速度:", self.max_angular_spin)
        
        speed_tab.setLayout(speed_layout)
        tab_widget.addTab(speed_tab, "🚀 速度")
        
        # --- 阈值设置 Tab ---
        threshold_tab = QWidget()
        threshold_layout = QFormLayout()
        threshold_layout.setSpacing(12)
        
        # 到达阈值
        self.goal_tolerance_spin = QDoubleSpinBox()
        self.goal_tolerance_spin.setRange(0.1, 5.0)
        self.goal_tolerance_spin.setSingleStep(0.1)
        self.goal_tolerance_spin.setDecimals(2)
        self.goal_tolerance_spin.setValue(self.current_settings.get('goal_tolerance', 0.5))
        self.goal_tolerance_spin.setSuffix(" m")
        self.goal_tolerance_spin.setToolTip("最终目标点的到达判定距离")
        threshold_layout.addRow("到达阈值:", self.goal_tolerance_spin)
        
        # 切换阈值
        self.switch_tolerance_spin = QDoubleSpinBox()
        self.switch_tolerance_spin.setRange(0.5, 10.0)
        self.switch_tolerance_spin.setSingleStep(0.5)
        self.switch_tolerance_spin.setDecimals(2)
        self.switch_tolerance_spin.setValue(self.current_settings.get('switch_tolerance', 1.5))
        self.switch_tolerance_spin.setSuffix(" m")
        self.switch_tolerance_spin.setToolTip(
            "中间航点的切换距离\n"
            "USV 距离中间航点 < 此值时，提前切换到下一航点"
        )
        threshold_layout.addRow("切换阈值:", self.switch_tolerance_spin)
        
        threshold_tab.setLayout(threshold_layout)
        tab_widget.addTab(threshold_tab, "📍 阈值")
        
        layout.addWidget(tab_widget)
        
        # ==================== 提示信息 ====================
        tip_frame = QFrame()
        if is_dark_theme():
            tip_frame.setStyleSheet(
                "QFrame { background-color: #2D3A4D; border-radius: 5px; padding: 10px; }"
            )
        else:
            tip_frame.setStyleSheet(
                "QFrame { background-color: #e6f7ff; border-radius: 5px; padding: 10px; }"
            )
        tip_layout = QVBoxLayout(tip_frame)
        
        tip_label = QLabel(
            "💡 参数调优建议:\n"
            "• 巡航速度: 根据水域条件和任务需求调整\n"
            "• 最大角速度: 越大转向越快，但可能不稳定\n"
            "• 切换阈值 > 到达阈值，确保中间航点平滑过渡"
        )
        tip_label.setWordWrap(True)
        _tip_color = '#88CCFF' if is_dark_theme() else '#0078d7'
        tip_label.setStyleSheet(f"color: {_tip_color}; font-size: 11px;")
        tip_layout.addWidget(tip_label)
        
        layout.addWidget(tip_frame)
        
        layout.addStretch()
        
        # ==================== 按钮 ====================
        btn_layout = QHBoxLayout()
        
        self.reset_btn = QPushButton("恢复默认")
        self.reset_btn.clicked.connect(self._reset_to_default)
        self.reset_btn.setStyleSheet("padding: 6px 12px;")
        
        self.cancel_btn = QPushButton("取消")
        self.cancel_btn.clicked.connect(self.reject)
        self.cancel_btn.setStyleSheet("padding: 6px 12px;")
        
        self.ok_btn = QPushButton("应用")
        self.ok_btn.clicked.connect(self.accept)
        if is_dark_theme():
            self.ok_btn.setStyleSheet(
                "background-color: #2D5887; color: white; padding: 6px 16px; font-weight: bold;"
            )
        else:
            self.ok_btn.setStyleSheet(
                "background-color: #0078d7; color: white; padding: 6px 16px; font-weight: bold;"
            )
        
        btn_layout.addWidget(self.reset_btn)
        btn_layout.addStretch()
        btn_layout.addWidget(self.cancel_btn)
        btn_layout.addWidget(self.ok_btn)
        
        layout.addLayout(btn_layout)
        
        self.setLayout(layout)
    
    def _reset_to_default(self):
        """恢复默认设置"""
        self.cruise_speed_spin.setValue(0.5)
        self.max_angular_spin.setValue(0.5)
        self.goal_tolerance_spin.setValue(0.5)
        self.switch_tolerance_spin.setValue(1.5)
    
    def get_settings(self):
        """
        获取用户设置
        
        Returns:
            dict: 包含所有设置的字典
        """
        return {
            'cruise_speed': self.cruise_speed_spin.value(),
            'max_angular_velocity': self.max_angular_spin.value(),
            'goal_tolerance': self.goal_tolerance_spin.value(),
            'switch_tolerance': self.switch_tolerance_spin.value(),
        }
