#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of nav settings dialog.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
导航设置对话框 - 平滑导航参数配置

支持在线设置:
- 到达阈值 (nav_arrival_threshold)
- 切换阈值 (switch_threshold)  
- 平滑导航开关 (smooth_navigation)
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QCheckBox, QPushButton, QGroupBox,
    QFormLayout, QFrame
)
from PyQt5.QtCore import Qt


class NavSettingsDialog(QDialog):
    """导航设置对话框"""
    
    def __init__(self, parent=None, current_settings=None):
        """
        初始化对话框
        
        Args:
            parent: 父窗口
            current_settings: 当前设置字典，包含:
                - nav_arrival_threshold: 到达阈值 (米)
                - switch_threshold: 切换阈值 (米)
                - smooth_navigation: 是否启用平滑导航
        """
        super().__init__(parent)
        self.setWindowTitle("导航参数设置")
        self.resize(450, 350)
        
        # 默认设置
        self.current_settings = current_settings or {
            'nav_arrival_threshold': 1.0,
            'switch_threshold': 1.5,
            'smooth_navigation': True
        }
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout()
        
        # ==================== 说明 ====================
        info_label = QLabel(
            "配置 USV 导航节点的平滑导航参数。\n\n"
            "• 到达阈值: 最终目标点的到达判定距离\n"
            "• 切换阈值: 中间航点的提前切换距离 (平滑导航)\n"
            "• 平滑导航: 中间航点提前切换，避免减速停止"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #AAAAAA; margin-bottom: 10px; font-size: 12px;")
        layout.addWidget(info_label)
        
        # ==================== 平滑导航开关 ====================
        self.smooth_nav_checkbox = QCheckBox("启用平滑导航模式")
        self.smooth_nav_checkbox.setChecked(self.current_settings.get('smooth_navigation', True))
        self.smooth_nav_checkbox.setStyleSheet("font-weight: bold; font-size: 14px; margin: 10px 0;")
        self.smooth_nav_checkbox.setToolTip(
            "启用后，USV 在中间航点会提前切换到下一个目标，不会减速停止。\n"
            "仅在最终航点才会精确到达。"
        )
        self.smooth_nav_checkbox.stateChanged.connect(self._on_smooth_nav_changed)
        layout.addWidget(self.smooth_nav_checkbox)
        
        # ==================== 阈值设置组 ====================
        threshold_group = QGroupBox("阈值设置 (单位: 米)")
        threshold_layout = QFormLayout()
        threshold_layout.setSpacing(15)
        
        # 到达阈值
        self.arrival_threshold_spin = QDoubleSpinBox()
        self.arrival_threshold_spin.setRange(0.1, 50.0)
        self.arrival_threshold_spin.setSingleStep(0.5)
        self.arrival_threshold_spin.setDecimals(2)
        self.arrival_threshold_spin.setValue(
            self.current_settings.get('nav_arrival_threshold', 2.0)
        )
        self.arrival_threshold_spin.setToolTip(
            "最终目标点的到达判定距离。\n"
            "当 USV 与最终目标的距离 < 此值时，判定为到达。"
        )
        arrival_label = QLabel("到达阈值:")
        arrival_label.setToolTip(self.arrival_threshold_spin.toolTip())
        threshold_layout.addRow(arrival_label, self.arrival_threshold_spin)
        
        # 切换阈值
        self.switch_threshold_spin = QDoubleSpinBox()
        self.switch_threshold_spin.setRange(0.1, 50.0)
        self.switch_threshold_spin.setSingleStep(0.5)
        self.switch_threshold_spin.setDecimals(2)
        self.switch_threshold_spin.setValue(
            self.current_settings.get('switch_threshold', 1.0)
        )
        self.switch_threshold_spin.setToolTip(
            "中间航点的切换距离 (仅平滑导航模式有效)。\n"
            "当 USV 与中间航点的距离 < 此值时，立即切换到下一航点。\n\n"
            "建议: 此值应 >= 到达阈值，以确保中间航点在触发到达前就已切换。"
        )
        switch_label = QLabel("切换阈值:")
        switch_label.setToolTip(self.switch_threshold_spin.toolTip())
        threshold_layout.addRow(switch_label, self.switch_threshold_spin)
        
        threshold_group.setLayout(threshold_layout)
        layout.addWidget(threshold_group)
        
        # ==================== 提示信息 ====================
        tip_frame = QFrame()
        tip_frame.setStyleSheet(
            "QFrame { background-color: #2D3A4D; border-radius: 5px; padding: 10px; }"
        )
        tip_layout = QVBoxLayout(tip_frame)
        
        tip_label = QLabel(
            "💡 提示:\n"
            "• 平滑导航适合连续路径追踪 (如编队航行、舞蹈)\n"
            "• 切换阈值建议 ≥ 到达阈值，避免中间航点意外到达\n"
            "• 对于需要精确定位的航点，可临时关闭平滑导航"
        )
        tip_label.setWordWrap(True)
        tip_label.setStyleSheet("color: #88CCFF; font-size: 11px;")
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
        self.ok_btn.setStyleSheet(
            "background-color: #2D5887; color: white; padding: 6px 16px; font-weight: bold;"
        )
        
        btn_layout.addWidget(self.reset_btn)
        btn_layout.addStretch()
        btn_layout.addWidget(self.cancel_btn)
        btn_layout.addWidget(self.ok_btn)
        
        layout.addLayout(btn_layout)
        
        self.setLayout(layout)
        
        # 初始化启用状态
        self._on_smooth_nav_changed()
    
    def _on_smooth_nav_changed(self):
        """平滑导航开关变化时，更新切换阈值的启用状态"""
        enabled = self.smooth_nav_checkbox.isChecked()
        self.switch_threshold_spin.setEnabled(enabled)
        
        # 视觉提示
        if enabled:
            self.switch_threshold_spin.setStyleSheet("")
        else:
            self.switch_threshold_spin.setStyleSheet("color: #666666;")
    
    def _reset_to_default(self):
        """恢复默认设置"""
        self.arrival_threshold_spin.setValue(1.0)
        self.switch_threshold_spin.setValue(1.5)
        self.smooth_nav_checkbox.setChecked(True)
    
    def get_settings(self):
        """
        获取用户设置
        
        Returns:
            dict: 包含所有设置的字典
        """
        return {
            'nav_arrival_threshold': self.arrival_threshold_spin.value(),
            'switch_threshold': self.switch_threshold_spin.value(),
            'smooth_navigation': self.smooth_nav_checkbox.isChecked()
        }
