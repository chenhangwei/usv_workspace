#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of uwb offset dialog.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
UWB 坐标系偏移角设置对话框

用于设置 UWB/伪卫星定位系统的坐标系与地磁坐标系的偏移角。
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox,
    QPushButton, QGroupBox, QComboBox, QMessageBox
)
from PyQt5.QtCore import Qt


class UwbOffsetDialog(QDialog):
    """UWB 坐标系偏移角设置对话框"""
    
    def __init__(self, usv_list, ros_node=None, parent=None):
        """
        初始化对话框
        
        Args:
            usv_list: 在线 USV 列表
            ros_node: ROS 节点（用于设置参数）
            parent: 父窗口
        """
        super().__init__(parent)
        self.setWindowTitle("🧭 UWB 坐标系偏移角设置")
        self.resize(450, 300)
        
        self.usv_list = usv_list
        self.ros_node = ros_node
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化 UI 组件"""
        layout = QVBoxLayout(self)
        
        # 说明文本
        info_label = QLabel(
            "UWB/伪卫星定位系统的坐标系可能与地磁坐标系存在偏移。\n"
            "此设置用于补偿 UWB X轴 与 地磁东方向 的夹角。\n\n"
            "测量方法：\n"
            "1. 将船头对准 UWB X轴正方向\n"
            "2. 观察日志中显示的 yaw 值\n"
            "3. 将该 yaw 值填入下方"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet(
            "color: #888; font-size: 11px; padding: 10px; "
            "background-color: #2a2a2a; border-radius: 5px;"
        )
        layout.addWidget(info_label)
        
        # USV 选择
        usv_group = QGroupBox("选择 USV")
        usv_layout = QVBoxLayout(usv_group)
        
        self.usv_combo = QComboBox()
        self.usv_combo.setStyleSheet("font-size: 12px; padding: 5px;")
        
        # 添加"所有 USV"选项
        self.usv_combo.addItem("📡 所有 USV（全局设置）")
        
        if self.usv_list:
            for usv in self.usv_list:
                ns = usv.get('namespace', '')
                if ns:
                    self.usv_combo.addItem(ns)
        
        usv_layout.addWidget(self.usv_combo)
        layout.addWidget(usv_group)
        
        # 偏移角设置
        offset_group = QGroupBox("偏移角设置")
        offset_layout = QVBoxLayout(offset_group)
        
        angle_layout = QHBoxLayout()
        angle_layout.addWidget(QLabel("偏移角 (度):"))
        
        self.angle_spinbox = QDoubleSpinBox()
        self.angle_spinbox.setRange(-180.0, 180.0)
        self.angle_spinbox.setDecimals(1)
        self.angle_spinbox.setSingleStep(1.0)
        self.angle_spinbox.setValue(0.0)
        self.angle_spinbox.setSuffix(" °")
        self.angle_spinbox.setToolTip(
            "UWB X轴 与 地磁东方向 的夹角（逆时针为正）\n"
            "例如：如果船头对准 UWB X轴时，yaw 显示 30°，则填 30"
        )
        angle_layout.addWidget(self.angle_spinbox)
        
        offset_layout.addLayout(angle_layout)
        
        # 快捷按钮
        quick_layout = QHBoxLayout()
        quick_layout.addWidget(QLabel("快捷设置:"))
        
        btn_0 = QPushButton("0°")
        btn_0.clicked.connect(lambda: self.angle_spinbox.setValue(0.0))
        quick_layout.addWidget(btn_0)
        
        btn_90 = QPushButton("90°")
        btn_90.clicked.connect(lambda: self.angle_spinbox.setValue(90.0))
        quick_layout.addWidget(btn_90)
        
        btn_180 = QPushButton("180°")
        btn_180.clicked.connect(lambda: self.angle_spinbox.setValue(180.0))
        quick_layout.addWidget(btn_180)
        
        btn_n90 = QPushButton("-90°")
        btn_n90.clicked.connect(lambda: self.angle_spinbox.setValue(-90.0))
        quick_layout.addWidget(btn_n90)
        
        offset_layout.addLayout(quick_layout)
        layout.addWidget(offset_group)
        
        # 按钮
        btn_layout = QHBoxLayout()
        
        self.btn_apply = QPushButton("应用")
        self.btn_apply.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 8px 20px;"
        )
        self.btn_apply.clicked.connect(self._on_apply)
        btn_layout.addWidget(self.btn_apply)
        
        self.btn_cancel = QPushButton("取消")
        self.btn_cancel.clicked.connect(self.reject)
        btn_layout.addWidget(self.btn_cancel)
        
        layout.addLayout(btn_layout)
    
    def _on_apply(self):
        """应用设置"""
        offset_deg = self.angle_spinbox.value()
        target = self.usv_combo.currentText()
        
        if self.ros_node is None:
            QMessageBox.warning(
                self,
                "ROS 未就绪",
                "ROS 节点尚未初始化，无法设置参数。"
            )
            return
        
        try:
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            
            # 确定目标节点
            if target.startswith("📡"):
                # 所有 USV - 需要逐个设置
                namespaces = [usv.get('namespace') for usv in self.usv_list if usv.get('namespace')]
            else:
                namespaces = [target]
            
            success_count = 0
            for ns in namespaces:
                node_name = f'{ns}/gps_to_local_node'
                service_name = f'{node_name}/set_parameters'
                
                # 创建服务客户端
                client = self.ros_node.create_client(SetParameters, service_name)
                
                if not client.wait_for_service(timeout_sec=2.0):
                    continue
                
                # 创建参数
                param = Parameter()
                param.name = 'coordinate_yaw_offset_deg'
                param.value = ParameterValue()
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = offset_deg
                
                # 发送请求
                request = SetParameters.Request()
                request.parameters = [param]
                
                future = client.call_async(request)
                # 简单等待（在 Qt 中不推荐，但对于简短操作可接受）
                import time
                timeout = 2.0
                start = time.time()
                while not future.done() and (time.time() - start) < timeout:
                    time.sleep(0.1)
                
                if future.done():
                    result = future.result()
                    if result and result.results and result.results[0].successful:
                        success_count += 1
            
            if success_count > 0:
                QMessageBox.information(
                    self,
                    "设置成功",
                    f"已成功设置 {success_count} 个 USV 的坐标系偏移角为 {offset_deg:.1f}°\n\n"
                    f"注意：此设置立即生效，但不会持久化。\n"
                    f"如需永久生效，请修改 usv_params.yaml 中的\n"
                    f"coordinate_yaw_offset_deg 参数。"
                )
                self.accept()
            else:
                QMessageBox.warning(
                    self,
                    "设置失败",
                    "无法连接到目标节点的参数服务。\n"
                    "请确保 USV 节点正在运行。"
                )
        
        except Exception as e:
            QMessageBox.critical(
                self,
                "错误",
                f"设置参数时发生错误: {e}"
            )
    
    def get_result(self):
        """获取对话框结果"""
        return {
            'target': self.usv_combo.currentText(),
            'offset_deg': self.angle_spinbox.value()
        }
