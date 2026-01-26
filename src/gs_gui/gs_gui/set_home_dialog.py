#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of set home dialog.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
Home Position 设置对话框
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QRadioButton, QButtonGroup, QComboBox,
    QGroupBox, QMessageBox
)
from PyQt5.QtCore import Qt


class SetHomeDialog(QDialog):
    """设置 Home Position 对话框"""
    
    def __init__(self, usv_list, parent=None):
        """
        初始化对话框
        
        Args:
            usv_list: 在线 USV 列表（字典列表，包含 namespace 字段）
            parent: 父窗口
        """
        super().__init__(parent)
        self.setWindowTitle("设置 Home Position")
        self.resize(500, 350)
        
        # 存储 USV 列表
        self.usv_list = usv_list
        
        # 初始化 UI
        self._init_ui()
        
        # 默认坐标（A0 基站）
        # 注意: 这些默认值应与 usv_params.yaml 中的 gps_origin_* 参数保持一致
        # 如需修改,请同步更新配置文件以确保所有节点使用统一的GPS原点
        self.default_lat = 22.5180977  # 与ParamLoader.DEFAULT_GPS_ORIGIN_LAT一致
        self.default_lon = 113.9007239  # 与ParamLoader.DEFAULT_GPS_ORIGIN_LON一致
        self.default_alt = 0.0  # 修改为与 usv_params.yaml 一致
        
        # 设置默认值
        self._set_default_coords()
    
    def _init_ui(self):
        """初始化 UI 组件"""
        layout = QVBoxLayout(self)
        
        # 说明文本
        info_label = QLabel(
            "Home Position 用于 RTL（返航）和 Failsafe（失联保护）功能。\n"
            "与 EKF Origin 不同，Home Position 可以在任务执行中动态修改。"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #888; font-size: 11px; padding: 10px; background-color: #2a2a2a; border-radius: 5px;")
        layout.addWidget(info_label)
        
        # USV 选择
        usv_group = QGroupBox("选择 USV")
        usv_layout = QVBoxLayout(usv_group)
        
        self.usv_combo = QComboBox()
        self.usv_combo.setStyleSheet("font-size: 12px; padding: 5px;")
        
        if self.usv_list:
            for usv in self.usv_list:
                ns = usv.get('namespace', '')
                if ns:
                    self.usv_combo.addItem(ns)
        else:
            self.usv_combo.addItem("（无在线 USV）")
            self.usv_combo.setEnabled(False)
        
        usv_layout.addWidget(self.usv_combo)
        layout.addWidget(usv_group)
        
        # 坐标来源选择
        coord_group = QGroupBox("坐标来源")
        coord_layout = QVBoxLayout(coord_group)
        
        self.button_group = QButtonGroup()
        
        self.radio_current = QRadioButton("使用 USV 当前位置")
        self.radio_current.setToolTip("将 Home Position 设置为 USV 当前 GPS 位置")
        self.button_group.addButton(self.radio_current)
        coord_layout.addWidget(self.radio_current)
        
        self.radio_custom = QRadioButton("指定坐标")
        self.radio_custom.setToolTip("手动输入 Home Position 坐标")
        self.button_group.addButton(self.radio_custom)
        coord_layout.addWidget(self.radio_custom)
        
        # 默认选择"使用当前位置"
        self.radio_current.setChecked(True)
        
        # 连接信号
        self.radio_current.toggled.connect(self._on_mode_changed)
        self.radio_custom.toggled.connect(self._on_mode_changed)
        
        layout.addWidget(coord_group)
        
        # 坐标输入
        coords_group = QGroupBox("局部坐标输入 (ENU)")
        coords_layout = QVBoxLayout(coords_group)
        
        # X轴 (东向)
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X (东向 East):"))
        self.x_input = QLineEdit()
        self.x_input.setPlaceholderText("例如: 0.0")
        self.x_input.setToolTip("相对于原点的东向距离 (米)")
        x_layout.addWidget(self.x_input)
        coords_layout.addLayout(x_layout)
        
        # Y轴 (北向)
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y (北向 North):"))
        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("例如: 0.0")
        self.y_input.setToolTip("相对于原点的北向距离 (米)")
        y_layout.addWidget(self.y_input)
        coords_layout.addLayout(y_layout)
        
        # Z轴 (高度)
        z_layout = QHBoxLayout()
        z_layout.addWidget(QLabel("Z (高度 Up):"))
        self.z_input = QLineEdit()
        self.z_input.setPlaceholderText("例如: 0.0")
        self.z_input.setToolTip("相对于原点的高度 (米)")
        z_layout.addWidget(self.z_input)
        coords_layout.addLayout(z_layout)
        
        # 快捷按钮
        shortcut_layout = QHBoxLayout()
        self.btn_origin = QPushButton("设为原点 (0,0,0)")
        self.btn_origin.setToolTip("快速填入 (0, 0, 0)")
        self.btn_origin.clicked.connect(self._set_zero_coords)
        shortcut_layout.addWidget(self.btn_origin)
        coords_layout.addLayout(shortcut_layout)
        
        layout.addWidget(coords_group)
        self.coords_group = coords_group
        
        # 初始状态：禁用坐标输入
        self._on_mode_changed()
        
        # 按钮
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        self.btn_ok = QPushButton("确定")
        self.btn_ok.setStyleSheet("background-color: #4a90e2; color: white; font-weight: bold; padding: 8px 20px;")
        self.btn_ok.clicked.connect(self.accept)
        button_layout.addWidget(self.btn_ok)
        
        self.btn_cancel = QPushButton("取消")
        self.btn_cancel.setStyleSheet("padding: 8px 20px;")
        self.btn_cancel.clicked.connect(self.reject)
        button_layout.addWidget(self.btn_cancel)
        
        layout.addLayout(button_layout)
    
    def _on_mode_changed(self):
        """切换坐标来源时的回调"""
        use_current = self.radio_current.isChecked()
        
        # 禁用/启用坐标输入
        self.coords_group.setEnabled(not use_current)
    
    def _set_zero_coords(self):
        """填入 (0,0,0)"""
        self.x_input.setText("0.0")
        self.y_input.setText("0.0")
        self.z_input.setText("0.0")

    def _set_default_coords(self):
        """设置默认坐标"""
        self._set_zero_coords()
    
    def get_result(self):
        """
        获取对话框结果
        
        Returns:
            tuple: (usv_namespace, use_current, coords)
                - usv_namespace: str, USV 命名空间
                - use_current: bool, 是否使用当前位置
                - coords: dict, 坐标字典 {'x': float, 'y': float, 'z': float}
        """
        usv_namespace = self.usv_combo.currentText()
        use_current = self.radio_current.isChecked()
        
        coords = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        }
        
        if not use_current:
            try:
                coords['x'] = float(self.x_input.text() or 0.0)
                coords['y'] = float(self.y_input.text() or 0.0)
                coords['z'] = float(self.z_input.text() or 0.0)
            except ValueError as e:
                QMessageBox.warning(
                    self,
                    "输入错误",
                    f"坐标输入格式错误，请输入有效的数字。\n错误: {e}"
                )
                return None, None, None
        
        return usv_namespace, use_current, coords
    
    def accept(self):
        """确定按钮回调"""
        if not self.usv_list:
            QMessageBox.warning(self, "警告", "没有在线的 USV，无法设置 Home Position。")
            return
        
        usv_namespace, use_current, coords = self.get_result()
        
        if usv_namespace is None:
            return  # 验证失败
        
        # 验证坐标范围
        if not use_current:
            # 简单范围验证 (例如避免过大的数值)
            if abs(coords['x']) > 50000 or abs(coords['y']) > 50000:
                 QMessageBox.warning(self, "输入警告", "输入的局部坐标距离过大 (>50km)，请确认是否正确。")
                 # 依然允许通过，只是警告
        
        super().accept()
