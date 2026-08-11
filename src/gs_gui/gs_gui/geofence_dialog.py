#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of geofence dialog.
#
# Author: chenhangwei
# Date: 2026-01-26
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                             QDoubleSpinBox, QCheckBox, QPushButton, QGroupBox)
from PyQt5.QtCore import Qt
from gs_gui.style_manager import is_dark_theme

class GeofenceDialog(QDialog):
    PURPOSE_TEXT = (
        "作用：设置 USV 在本地坐标系中的矩形安全活动范围。\n"
        "启用后，如果 USV 超出设定边界，系统会自动切换到 HOLD 模式。"
    )
    USAGE_TEXT = (
        "使用方法：\n"
        "1. 勾选“启用电子围栏监控”打开围栏保护；\n"
        "2. 根据任务区域填写 X/Y 的最小值和最大值，单位为米；\n"
        "3. 确认 X 最小值 < X 最大值、Y 最小值 < Y 最大值；\n"
        "4. 点击“确定”保存，点击“取消”放弃本次修改。"
    )

    def __init__(self, parent=None, current_bounds=None, current_enabled=False):
        super().__init__(parent)
        self.setWindowTitle("电子围栏设置")
        self.current_bounds = current_bounds or {'x_min': -50.0, 'x_max': 50.0, 'y_min': -50.0, 'y_max': 50.0}
        self.current_enabled = current_enabled
        self.resize(400, 300)
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout()
        
        # 说明
        info_label = QLabel(f"{self.PURPOSE_TEXT}\n\n{self.USAGE_TEXT}")
        info_label.setWordWrap(True)
        _info_color = '#AAAAAA' if is_dark_theme() else '#666666'
        info_label.setStyleSheet(f"color: {_info_color}; margin-bottom: 10px;")
        layout.addWidget(info_label)

        # 启用开关
        self.enable_checkbox = QCheckBox("启用电子围栏监控")
        self.enable_checkbox.setChecked(self.current_enabled)
        self.enable_checkbox.setToolTip("开启后会持续检查 USV 是否超出下方矩形边界")
        self.enable_checkbox.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self.enable_checkbox)
        
        # 边界设置
        group_box = QGroupBox("矩形边界（本地坐标系 / 米）")
        grid = QVBoxLayout()
        
        # X Range
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X 最小值:"))
        self.x_min_spin = QDoubleSpinBox()
        self.x_min_spin.setRange(-10000.0, 10000.0)
        self.x_min_spin.setValue(float(self.current_bounds['x_min']))
        self.x_min_spin.setSingleStep(10.0)
        self.x_min_spin.setToolTip("围栏左边界，单位：米")
        x_layout.addWidget(self.x_min_spin)
        
        x_layout.addWidget(QLabel("X 最大值:"))
        self.x_max_spin = QDoubleSpinBox()
        self.x_max_spin.setRange(-10000.0, 10000.0)
        self.x_max_spin.setValue(float(self.current_bounds['x_max']))
        self.x_max_spin.setSingleStep(10.0)
        self.x_max_spin.setToolTip("围栏右边界，单位：米")
        x_layout.addWidget(self.x_max_spin)
        grid.addLayout(x_layout)
        
        # Y Range
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y 最小值:"))
        self.y_min_spin = QDoubleSpinBox()
        self.y_min_spin.setRange(-10000.0, 10000.0)
        self.y_min_spin.setValue(float(self.current_bounds['y_min']))
        self.y_min_spin.setSingleStep(10.0)
        self.y_min_spin.setToolTip("围栏下边界，单位：米")
        y_layout.addWidget(self.y_min_spin)
        
        y_layout.addWidget(QLabel("Y 最大值:"))
        self.y_max_spin = QDoubleSpinBox()
        self.y_max_spin.setRange(-10000.0, 10000.0)
        self.y_max_spin.setValue(float(self.current_bounds['y_max']))
        self.y_max_spin.setSingleStep(10.0)
        self.y_max_spin.setToolTip("围栏上边界，单位：米")
        y_layout.addWidget(self.y_max_spin)
        grid.addLayout(y_layout)
        
        group_box.setLayout(grid)
        layout.addWidget(group_box)
        
        layout.addStretch()
        
        # Buttons
        btn_layout = QHBoxLayout()
        self.ok_btn = QPushButton("确定")
        self.ok_btn.clicked.connect(self.accept)
        _btn_bg = '#2D5887' if is_dark_theme() else '#0078d7'
        self.ok_btn.setStyleSheet(f"background-color: {_btn_bg}; color: white; padding: 6px;")
        self.cancel_btn = QPushButton("取消")
        self.cancel_btn.clicked.connect(self.reject)
        
        btn_layout.addWidget(self.cancel_btn)
        btn_layout.addWidget(self.ok_btn)
        layout.addLayout(btn_layout)
        
        self.setLayout(layout)
        
    def get_settings(self):
        bounds = {
            'x_min': self.x_min_spin.value(),
            'x_max': self.x_max_spin.value(),
            'y_min': self.y_min_spin.value(),
            'y_max': self.y_max_spin.value()
        }
        enabled = self.enable_checkbox.isChecked()
        return bounds, enabled
