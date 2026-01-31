#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of area offset dialog.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
Area Center 偏移量设置对话框
用于设置任务坐标系原点（area_center）在全局地图坐标系中的位置
"""

from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                              QDoubleSpinBox, QSpinBox, QPushButton, QGroupBox, QMessageBox,
                              QComboBox, QSizePolicy)
from PyQt5.QtCore import Qt

from . import location_db


class AreaOffsetDialog(QDialog):
    """Area Center 偏移量设置对话框"""
    
    def __init__(self, parent=None, current_offset=None):
        """
        初始化对话框
        
        Args:
            parent: 父窗口
            current_offset: 当前偏移量 {'x': float, 'y': float, 'z': float, 'angle': int}
        """
        super().__init__(parent)
        self.parent_window = parent  # 保存父窗口引用，用于获取USV位置
        self.setWindowTitle("设置任务坐标系原点偏移量及偏转")
        self.setModal(True)
        self.resize(400, 320)
        
        # 初始化偏移量
        if current_offset is None:
            current_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'angle': 0}
        
        self.offset_x = current_offset.get('x', 0.0)
        self.offset_y = current_offset.get('y', 0.0)
        self.offset_z = current_offset.get('z', 0.0)
        self.offset_angle = int(current_offset.get('angle', 0))
        
        # 创建UI
        self._setup_ui()
        
        # 窗口居中显示
        self._center_on_screen()

    def _setup_ui(self):
        """设置用户界面"""
        layout = QVBoxLayout(self)
        
        # 说明标签
        info_label = QLabel(
            "设置任务坐标系原点（Area Center）在全局地图坐标系中的位置及偏转：\n"
            "• 任务文件中的坐标是相对于Area Center的\n"
            "• 该偏移量和角度将Area坐标转换为全局Map坐标\n"
            "• 旋转基于Area Center原点\n"
            "• USV会自动转换为本地坐标执行"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #555; font-size: 14pt; padding: 10px;")
        layout.addWidget(info_label)
        
        # 坐标输入组
        # 地点数据组（位于说明与坐标组之间）
        location_group = QGroupBox("地点数据")
        loc_layout = QVBoxLayout()

        # 第一行：地点名称（标签 + 下拉/输入），下拉框水平弹性伸缩
        name_row = QHBoxLayout()
        name_label = QLabel("地点名称:")
        name_label.setMinimumWidth(80)
        self.name_combo = QComboBox()
        self.name_combo.setEditable(True)
        self.name_combo.setInsertPolicy(QComboBox.NoInsert)
        self.name_combo.setToolTip("选择或输入地点名称；选择后会加载对应偏移量数据")
        # 当从下拉列表选择或索引改变时，加载对应记录
        self.name_combo.activated.connect(lambda idx: self._on_location_selected(self.name_combo.currentText()))
        self.name_combo.currentIndexChanged.connect(lambda idx: self._on_location_selected(self.name_combo.currentText()))
        # 如果用户手动编辑文本后完成编辑，也尝试加载对应记录
        try:
            le = self.name_combo.lineEdit()
            if le is not None:
                le.editingFinished.connect(lambda: self._on_location_selected(self.name_combo.currentText()))
        except Exception:
            pass
        self.name_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        name_row.addWidget(name_label)
        name_row.addWidget(self.name_combo)

        # 第二行：按钮（增加、删除），单独一行放在下方
        btn_row = QHBoxLayout()
        add_button = QPushButton("增加")
        add_button.setToolTip("将下面的偏移量保存到数据库，名称相同则覆盖")
        add_button.clicked.connect(self._save_location)
        del_button = QPushButton("删除")
        del_button.setToolTip("从数据库删除当前地点，并重置偏移量为0")
        del_button.clicked.connect(self._delete_location)
        btn_row.addWidget(add_button)
        btn_row.addWidget(del_button)
        btn_row.addStretch()

        loc_layout.addLayout(name_row)
        loc_layout.addLayout(btn_row)
        location_group.setLayout(loc_layout)
        layout.addWidget(location_group)

        # 填充地点名称
        try:
            location_db.init_db()
        except Exception:
            pass
        self._populate_location_names()

        coord_group = QGroupBox("偏移量坐标 (单位: 米) 与 旋转")
        coord_layout = QVBoxLayout()
        
        # X坐标
        x_layout = QHBoxLayout()
        x_label = QLabel("X 偏移量:")
        x_label.setMinimumWidth(80)
        self.x_spinbox = QDoubleSpinBox()
        self.x_spinbox.setRange(-10000.0, 10000.0)
        self.x_spinbox.setDecimals(2)
        self.x_spinbox.setValue(self.offset_x)
        self.x_spinbox.setSuffix(" m")
        x_layout.addWidget(x_label)
        x_layout.addWidget(self.x_spinbox)
        coord_layout.addLayout(x_layout)
        
        # Y坐标
        y_layout = QHBoxLayout()
        y_label = QLabel("Y 偏移量:")
        y_label.setMinimumWidth(80)
        self.y_spinbox = QDoubleSpinBox()
        self.y_spinbox.setRange(-10000.0, 10000.0)
        self.y_spinbox.setDecimals(2)
        self.y_spinbox.setValue(self.offset_y)
        self.y_spinbox.setSuffix(" m")
        y_layout.addWidget(y_label)
        y_layout.addWidget(self.y_spinbox)
        coord_layout.addLayout(y_layout)
        
        # Z坐标
        z_layout = QHBoxLayout()
        z_label = QLabel("Z 偏移量:")
        z_label.setMinimumWidth(80)
        self.z_spinbox = QDoubleSpinBox()
        self.z_spinbox.setRange(-1000.0, 1000.0)
        self.z_spinbox.setDecimals(2)
        self.z_spinbox.setValue(self.offset_z)
        self.z_spinbox.setSuffix(" m")
        z_layout.addWidget(z_label)
        z_layout.addWidget(self.z_spinbox)
        coord_layout.addLayout(z_layout)
        
        # 旋转角度 (Angle)
        angle_layout = QHBoxLayout()
        angle_label = QLabel("偏转角度:")
        angle_label.setMinimumWidth(80)
        self.angle_spinbox = QSpinBox()
        self.angle_spinbox.setRange(0, 359)
        self.angle_spinbox.setValue(self.offset_angle)
        self.angle_spinbox.setSuffix(" °")
        self.angle_spinbox.setToolTip("围绕任务坐标系原点旋转的角度 (0-359)")
        angle_layout.addWidget(angle_label)
        angle_layout.addWidget(self.angle_spinbox)
        coord_layout.addLayout(angle_layout)
        
        coord_group.setLayout(coord_layout)
        layout.addWidget(coord_group)
        
        # 按钮组
        button_layout = QHBoxLayout()
        
        # 获取当前USV位置按钮
        get_position_button = QPushButton("获取选中USV位置")
        get_position_button.setToolTip("将当前选中USV的位置自动填充到偏移量坐标栏")
        get_position_button.clicked.connect(self._get_usv_position)
        button_layout.addWidget(get_position_button)
        
        # 重置按钮
        reset_button = QPushButton("重置为0")
        reset_button.clicked.connect(self._reset_values)
        button_layout.addWidget(reset_button)
        
        button_layout.addStretch()
        
        # 确定按钮
        ok_button = QPushButton("确定")
        ok_button.setDefault(True)
        ok_button.clicked.connect(self.accept)
        button_layout.addWidget(ok_button)
        
        # 取消按钮
        cancel_button = QPushButton("取消")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        layout.addLayout(button_layout)
        
    def _reset_values(self):
        """重置所有值为0"""
        self.x_spinbox.setValue(0.0)
        self.y_spinbox.setValue(0.0)
        self.z_spinbox.setValue(0.0)
        self.angle_spinbox.setValue(0)
    
    def _get_usv_position(self):
        """从父窗口获取当前选中USV的位置并填充到坐标栏"""
        if self.parent_window is None:
            QMessageBox.warning(self, "警告", "无法访问主窗口")
            return
        
        # 检查父窗口是否有获取USV位置的方法
        if not hasattr(self.parent_window, 'get_selected_usv_position'):
            QMessageBox.warning(self, "警告", "主窗口不支持获取USV位置")
            return
        
        # 调用父窗口的方法获取选中USV的位置
        position = self.parent_window.get_selected_usv_position()
        
        if position is None:
            QMessageBox.warning(self, "警告", "未选中任何USV或无法获取位置信息\n请先选择一个USV")
            return
        
        # 将位置填充到坐标输入框
        self.x_spinbox.setValue(position.get('x', 0.0))
        self.y_spinbox.setValue(position.get('y', 0.0))
        self.z_spinbox.setValue(position.get('z', 0.0))
        
        # 显示成功提示
        usv_id = position.get('usv_id', 'Unknown')
        QMessageBox.information(
            self, 
            "成功", 
            f"已获取 {usv_id} 的位置:\n"
            f"X: {position.get('x', 0.0):.2f} m\n"
            f"Y: {position.get('y', 0.0):.2f} m\n"
            f"Z: {position.get('z', 0.0):.2f} m"
        )
        
    def get_offset(self):
        """
        获取用户设置的偏移量
        
        Returns:
            dict: {'x': float, 'y': float, 'z': float, 'angle': int}
        """
        return {
            'x': self.x_spinbox.value(),
            'y': self.y_spinbox.value(),
            'z': self.z_spinbox.value(),
            'angle': self.angle_spinbox.value()
        }


    def _populate_location_names(self):
        """从数据库加载所有地点名称到下拉框"""
        try:
            names = location_db.get_all_names()
        except Exception:
            names = []
        self.name_combo.blockSignals(True)
        self.name_combo.clear()
        for n in names:
            self.name_combo.addItem(n)
        self.name_combo.blockSignals(False)


    def _on_location_selected(self, text: str):
        """当下拉框文本改变时尝试加载该地点的数据"""
        if not text:
            return
        try:
            data = location_db.get_location_by_name(text)
        except Exception:
            data = None
        if data is None:
            return
        # 将数据加载到输入框
        self.x_spinbox.setValue(data.get('x', 0.0))
        self.y_spinbox.setValue(data.get('y', 0.0))
        self.z_spinbox.setValue(data.get('z', 0.0))
        self.angle_spinbox.setValue(int(data.get('angle', 0)))


    def _save_location(self):
        """保存/覆盖当前输入框数据到数据库，名称相同则覆盖"""
        name = self.name_combo.currentText().strip()
        if not name:
            QMessageBox.warning(self, "警告", "请先输入地点名称")
            return
        x = float(self.x_spinbox.value())
        y = float(self.y_spinbox.value())
        z = float(self.z_spinbox.value())
        angle = int(self.angle_spinbox.value())
        try:
            location_db.upsert_location(name, x, y, z, angle)
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存地点失败: {e}")
            return
        QMessageBox.information(self, "成功", f"已保存地点: {name}")
        # 刷新下拉框并选中
        self._populate_location_names()
        index = self.name_combo.findText(name)
        if index >= 0:
            self.name_combo.setCurrentIndex(index)


    def _delete_location(self):
        """删除数据库中当前名称的记录，并重置输入框为0"""
        name = self.name_combo.currentText().strip()
        if not name:
            QMessageBox.warning(self, "警告", "请先选择或输入要删除的地点名称")
            return
        confirm = QMessageBox.question(self, "确认", f"确定要删除地点 '{name}' 吗？")
        if confirm != QMessageBox.Yes:
            return
        try:
            location_db.delete_location(name)
        except Exception as e:
            QMessageBox.critical(self, "错误", f"删除地点失败: {e}")
            return
        QMessageBox.information(self, "已删除", f"地点 '{name}' 已从数据库删除")
        # 刷新并清除输入框
        self._populate_location_names()
        self._reset_values()


    def _center_on_screen(self):
        """将窗口居中显示在屏幕上"""
        from PyQt5.QtWidgets import QApplication
        screen = QApplication.desktop().screenGeometry()
        size = self.geometry()
        self.move(
            (screen.width() - size.width()) // 2,
            (screen.height() - size.height()) // 2
        )
