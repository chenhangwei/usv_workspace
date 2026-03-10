#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of multi-group formation dialog.
#
# Author: chenhangwei
# Date: 2026-02-13
"""
多编队配置对话框模块

提供多编队组的配置 UI：
- 多个编队组，通过标签页管理
- 每组独立选择领队、跟随者、队形、间距
- 领队由集群导航任务控制，跟随者被排除在集群任务之外
- USV 不能同时属于多个编队组
- 实时队形预览
"""

import math
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QDoubleSpinBox,
    QListWidget, QListWidgetItem, QAbstractItemView, QMessageBox,
    QFrame, QWidget, QSplitter, QSpinBox, QTabBar
)
from PyQt5.QtCore import Qt, QSize, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QFont, QPen, QBrush

from .formation_controller import FormationType, FormationController
from .style_manager import is_dark_theme


class FormationPreviewWidget(QWidget):
    """
    编队队形实时预览控件
    
    在一个 2D 坐标系中显示领队和跟随者的相对位置。
    - 领队以红色大圆点表示，朝上为航向
    - 跟随者以蓝色圆点表示
    - 标注 USV ID
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(250, 250)
        self._positions = []  # [(x, y), ...] 第一个为领队
        self._labels = []     # ["L", "F1", "F2", ...]
        self._formation_name = ""

    def set_preview_data(self, positions, labels, formation_name=""):
        """
        更新预览数据
        
        Args:
            positions: [(x,y), ...] 坐标列表（领队在索引0）
            labels: 标签列表
            formation_name: 编队名称
        """
        self._positions = positions
        self._labels = labels
        self._formation_name = formation_name
        self.update()

    def paintEvent(self, event):
        """绘制编队预览"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # 背景
        if is_dark_theme():
            painter.fillRect(0, 0, w, h, QColor(30, 30, 35))
        else:
            painter.fillRect(0, 0, w, h, QColor(245, 245, 250))

        if not self._positions:
            # 无数据提示
            painter.setPen(QColor(100, 100, 100) if is_dark_theme() else QColor(150, 150, 150))
            painter.setFont(QFont("Arial", 12))
            painter.drawText(self.rect(), Qt.AlignCenter, "请先配置编队")
            return

        # 计算缩放
        all_x = [p[0] for p in self._positions]
        all_y = [p[1] for p in self._positions]
        
        max_range_x = max(abs(min(all_x)), abs(max(all_x)), 1.0) * 1.5
        max_range_y = max(abs(min(all_y)), abs(max(all_y)), 1.0) * 1.5
        max_range = max(max_range_x, max_range_y)

        cx = w / 2
        cy = h / 2
        scale = min(w, h) / (2 * max_range + 1) * 0.4

        # 绘制网格
        _grid_color = QColor(50, 50, 55) if is_dark_theme() else QColor(210, 210, 220)
        painter.setPen(QPen(_grid_color, 1, Qt.DotLine))
        for i in range(-10, 11):
            grid_x = cx + i * scale
            grid_y = cy - i * scale
            if 0 <= grid_x <= w:
                painter.drawLine(int(grid_x), 0, int(grid_x), h)
            if 0 <= grid_y <= h:
                painter.drawLine(0, int(grid_y), w, int(grid_y))

        # 绘制坐标轴
        _axis_color = QColor(80, 80, 90) if is_dark_theme() else QColor(160, 160, 180)
        painter.setPen(QPen(_axis_color, 1))
        painter.drawLine(int(cx), 0, int(cx), h)
        painter.drawLine(0, int(cy), w, int(cy))

        # 航向指示箭头 (向上)
        _arrow_color = QColor(100, 100, 120) if is_dark_theme() else QColor(140, 140, 160)
        painter.setPen(QPen(_arrow_color, 1))
        painter.drawLine(int(cx), int(cy), int(cx), max(0, int(cy - 40)))
        painter.drawLine(int(cx), max(0, int(cy - 40)), int(cx - 5), max(0, int(cy - 33)))
        painter.drawLine(int(cx), max(0, int(cy - 40)), int(cx + 5), max(0, int(cy - 33)))
        
        # 标注方向
        painter.setFont(QFont("Arial", 8))
        painter.setPen(_arrow_color)
        painter.drawText(int(cx + 5), max(10, int(cy - 42)), "航向")

        # 绘制USV
        for idx, (px, py) in enumerate(self._positions):
            # 转换到屏幕坐标 (注意 Y 轴翻转)
            sx = cx + px * scale
            sy = cy - py * scale

            if idx == 0:
                # 领队 - 红色大圆
                radius = 14
                painter.setBrush(QBrush(QColor(220, 60, 60)))
                painter.setPen(QPen(QColor(255, 100, 100), 2))
            else:
                # 跟随者 - 蓝色圆
                radius = 10
                painter.setBrush(QBrush(QColor(60, 120, 220)))
                painter.setPen(QPen(QColor(100, 160, 255), 2))

            painter.drawEllipse(int(sx - radius), int(sy - radius), radius * 2, radius * 2)

            # 标签
            label = self._labels[idx] if idx < len(self._labels) else f"F{idx}"
            _label_color = QColor(230, 230, 230) if is_dark_theme() else QColor(40, 40, 40)
            painter.setPen(_label_color)
            painter.setFont(QFont("Arial", 9, QFont.Bold))
            painter.drawText(int(sx - 15), int(sy + radius + 15), label)

        # 连线 (领队到每个跟随者)
        if len(self._positions) > 1:
            _line_color = QColor(80, 80, 100, 100) if is_dark_theme() else QColor(140, 140, 170, 120)
            painter.setPen(QPen(_line_color, 1, Qt.DashLine))
            lx = cx + self._positions[0][0] * scale
            ly = cy - self._positions[0][1] * scale
            for px, py in self._positions[1:]:
                sx = cx + px * scale
                sy = cy - py * scale
                painter.drawLine(int(lx), int(ly), int(sx), int(sy))

        # 编队名称
        if self._formation_name:
            _name_color = QColor(180, 180, 200) if is_dark_theme() else QColor(60, 60, 80)
            painter.setPen(_name_color)
            painter.setFont(QFont("Arial", 11, QFont.Bold))
            painter.drawText(10, 22, f"队形: {self._formation_name}")


class FormationDialog(QDialog):
    """
    多编队配置对话框
    
    功能：
    1. 支持多个编队组，通过顶部标签页管理
    2. 每组独立选择领队、跟随者、队形、间距
    3. 每组可独立加载领队导航路径
    4. USV 不会重复分配到多个组
    5. 实时预览当前组的队形
    6. 全局高级参数（更新频率、领队超时、跟随速度）
    """

    # 配置确认信号 — 发出编队组配置列表
    formation_confirmed = pyqtSignal(list)

    FORMATION_TYPE_NAMES = {
        FormationType.V_SHAPE: "人字形 (V-Shape)",
        FormationType.LINE: "横排一字形 (Line)",
        FormationType.COLUMN: "纵列一字形 (Column)",
        FormationType.S_SHAPE: "S形 (S-Shape)",
        FormationType.DIAMOND: "菱形 (Diamond)",
        FormationType.TRIANGLE: "三角形 (Triangle)",
        FormationType.ESCORT: "护卫队形 (Escort)",
    }

    def __init__(self, usv_cluster_list, parent=None):
        """
        Args:
            usv_cluster_list: 集群 USV 列表, [{namespace: "usv_xx", ...}, ...] 或 ["usv_xx", ...]
            parent: 父窗口
        """
        super().__init__(parent)
        self.setWindowTitle("多编队配置")
        self.setMinimumSize(750, 600)
        self.resize(850, 650)

        # 提取 USV ID 列表
        self._usv_ids = []
        for item in usv_cluster_list:
            if isinstance(item, dict):
                uid = item.get('namespace', '')
                if uid:
                    self._usv_ids.append(uid)
            elif isinstance(item, str):
                self._usv_ids.append(item)

        # 编队组数据模型
        self._groups = []          # List[dict] — 每个元素为一个编队组的配置
        self._current_idx = -1     # 当前选中的组索引
        self._is_loading = False   # 防止信号递归标志

        self._setup_ui()
        self._connect_signals()
        self._add_group()  # 默认创建一个编队组

    def _setup_ui(self):
        """构建 UI"""
        main_layout = QHBoxLayout(self)

        # ===== 左侧：配置区域 =====
        config_widget = QWidget()
        config_layout = QVBoxLayout(config_widget)
        config_layout.setSpacing(6)

        # --- 编队组标签管理 ---
        group_mgmt_layout = QHBoxLayout()
        self.group_tab_bar = QTabBar()
        self.group_tab_bar.setTabsClosable(True)
        self.group_tab_bar.setExpanding(False)
        self.btn_add_group = QPushButton("+ 添加编队组")
        self.btn_add_group.setAutoDefault(False)
        self.btn_add_group.setDefault(False)
        self.btn_add_group.setMaximumWidth(120)
        group_mgmt_layout.addWidget(self.group_tab_bar, 1)
        group_mgmt_layout.addWidget(self.btn_add_group)
        config_layout.addLayout(group_mgmt_layout)

        # --- 领队选择 ---
        leader_group = QGroupBox("领队选择")
        leader_layout = QHBoxLayout(leader_group)
        leader_layout.addWidget(QLabel("领队 USV:"))
        self.leader_combo = QComboBox()
        leader_layout.addWidget(self.leader_combo, 1)
        config_layout.addWidget(leader_group)

        # --- 编队说明 ---
        info_label = QLabel("💡 领队导航由集群任务控制，编队跟随者将自动排除在集群任务之外")
        _info_color = '#AAAAAA' if is_dark_theme() else '#666666'
        info_label.setStyleSheet(f"color: {_info_color}; font-size: 11px;")
        info_label.setWordWrap(True)
        config_layout.addWidget(info_label)

        # --- 跟随者列表 ---
        follower_group = QGroupBox("跟随者列表 (拖拽排序)")
        follower_layout = QVBoxLayout(follower_group)

        self.follower_list = QListWidget()
        self.follower_list.setDragDropMode(QAbstractItemView.InternalMove)
        self.follower_list.setSelectionMode(QAbstractItemView.MultiSelection)
        follower_layout.addWidget(self.follower_list)

        btn_sel_layout = QHBoxLayout()
        self.btn_select_all = QPushButton("全选")
        self.btn_deselect_all = QPushButton("取消全选")
        self.btn_select_all.setAutoDefault(False)
        self.btn_deselect_all.setAutoDefault(False)
        btn_sel_layout.addWidget(self.btn_select_all)
        btn_sel_layout.addWidget(self.btn_deselect_all)
        follower_layout.addLayout(btn_sel_layout)

        config_layout.addWidget(follower_group)

        # --- 队形选择 ---
        formation_group = QGroupBox("队形选择")
        formation_layout = QHBoxLayout(formation_group)
        formation_layout.addWidget(QLabel("队形:"))
        self.formation_combo = QComboBox()
        for ft, name in self.FORMATION_TYPE_NAMES.items():
            self.formation_combo.addItem(name, ft)
        formation_layout.addWidget(self.formation_combo, 1)
        config_layout.addWidget(formation_group)

        # --- 间距设置 ---
        spacing_group = QGroupBox("间距设置")
        spacing_layout = QGridLayout(spacing_group)

        spacing_layout.addWidget(QLabel("前后间距 (m):"), 0, 0)
        self.spacing_along_spin = QDoubleSpinBox()
        self.spacing_along_spin.setRange(0.3, 20.0)
        self.spacing_along_spin.setValue(1.0)
        self.spacing_along_spin.setSingleStep(0.1)
        self.spacing_along_spin.setDecimals(2)
        spacing_layout.addWidget(self.spacing_along_spin, 0, 1)

        spacing_layout.addWidget(QLabel("左右间距 (m):"), 1, 0)
        self.spacing_cross_spin = QDoubleSpinBox()
        self.spacing_cross_spin.setRange(0.3, 20.0)
        self.spacing_cross_spin.setValue(1.0)
        self.spacing_cross_spin.setSingleStep(0.1)
        self.spacing_cross_spin.setDecimals(2)
        spacing_layout.addWidget(self.spacing_cross_spin, 1, 1)

        config_layout.addWidget(spacing_group)

        # --- 高级参数 (全局) ---
        advanced_group = QGroupBox("高级参数 (全部组共用)")
        advanced_layout = QGridLayout(advanced_group)

        advanced_layout.addWidget(QLabel("更新频率 (Hz):"), 0, 0)
        self.update_rate_spin = QDoubleSpinBox()
        self.update_rate_spin.setRange(1.0, 20.0)
        self.update_rate_spin.setValue(10.0)
        self.update_rate_spin.setSingleStep(1.0)
        advanced_layout.addWidget(self.update_rate_spin, 0, 1)

        advanced_layout.addWidget(QLabel("领队超时 (s):"), 1, 0)
        self.leader_timeout_spin = QDoubleSpinBox()
        self.leader_timeout_spin.setRange(1.0, 30.0)
        self.leader_timeout_spin.setValue(3.0)
        self.leader_timeout_spin.setSingleStep(0.5)
        advanced_layout.addWidget(self.leader_timeout_spin, 1, 1)

        advanced_layout.addWidget(QLabel("跟随者速度 (m/s):"), 2, 0)
        self.follower_speed_spin = QDoubleSpinBox()
        self.follower_speed_spin.setRange(0.0, 5.0)
        self.follower_speed_spin.setValue(0.0)
        self.follower_speed_spin.setSingleStep(0.1)
        self.follower_speed_spin.setDecimals(2)
        self.follower_speed_spin.setToolTip("0 = 使用默认巡航速度")
        advanced_layout.addWidget(self.follower_speed_spin, 2, 1)

        config_layout.addWidget(advanced_group)

        # --- 按钮 ---
        btn_layout = QHBoxLayout()
        self.btn_confirm = QPushButton("✅ 确认启动所有编队")
        self.btn_confirm.setMinimumHeight(40)
        self.btn_confirm.setStyleSheet("""
            QPushButton {
                background-color: #2E7D32;
                color: white;
                font-weight: bold;
                font-size: 14px;
                border-radius: 6px;
            }
            QPushButton:hover {
                background-color: #388E3C;
            }
        """)
        self.btn_cancel = QPushButton("取消")
        self.btn_cancel.setMinimumHeight(40)
        self.btn_confirm.setAutoDefault(False)
        self.btn_cancel.setAutoDefault(False)
        btn_layout.addWidget(self.btn_confirm)
        btn_layout.addWidget(self.btn_cancel)
        config_layout.addLayout(btn_layout)

        main_layout.addWidget(config_widget, 1)

        # ===== 右侧：预览区域 =====
        preview_group = QGroupBox("队形预览")
        preview_layout = QVBoxLayout(preview_group)
        self.preview_widget = FormationPreviewWidget()
        preview_layout.addWidget(self.preview_widget)

        self.preview_info_label = QLabel("选择跟随者后更新预览")
        self.preview_info_label.setAlignment(Qt.AlignCenter)
        _info_color2 = '#AAAAAA' if is_dark_theme() else '#666666'
        self.preview_info_label.setStyleSheet(f"color: {_info_color2}; font-size: 11px;")
        preview_layout.addWidget(self.preview_info_label)

        main_layout.addWidget(preview_group, 1)

    def _connect_signals(self):
        """连接信号"""
        # 编队组管理
        self.group_tab_bar.currentChanged.connect(self._on_tab_changed)
        self.group_tab_bar.tabCloseRequested.connect(self._remove_group)
        self.btn_add_group.clicked.connect(self._add_group)

        # 配置变更
        self.leader_combo.currentIndexChanged.connect(self._on_leader_changed)
        self.follower_list.itemSelectionChanged.connect(self._on_config_changed)
        self.follower_list.model().rowsMoved.connect(self._on_config_changed)
        self.formation_combo.currentIndexChanged.connect(self._on_config_changed)
        self.spacing_along_spin.valueChanged.connect(self._on_config_changed)
        self.spacing_cross_spin.valueChanged.connect(self._on_config_changed)

        # 按钮
        self.btn_select_all.clicked.connect(self._select_all_followers)
        self.btn_deselect_all.clicked.connect(self._deselect_all_followers)

        self.btn_confirm.clicked.connect(self._on_confirm)
        self.btn_cancel.clicked.connect(self.reject)

    # ==================== 编队组管理 ====================

    def _add_group(self):
        """添加新编队组"""
        # 先保存当前组
        self._save_current_group()

        # 检查可用 USV
        available = self._get_available_usvs(exclude_group_idx=None)
        if len(available) < 2:
            QMessageBox.warning(
                self, "无法添加",
                "没有足够的可用 USV 创建新编队组。\n"
                "至少需要 2 艘未分配的 USV（1 领队 + 1 跟随者）。\n"
                "请先从现有组中减少 USV 分配。"
            )
            return

        group = {
            'group_id': f'编队组 {len(self._groups) + 1}',
            'leader_id': available[0],
            'follower_ids': list(available[1:]),
            'formation_type': 0,
            'spacing_along': 1.0,
            'spacing_cross': 1.0,

        }
        self._groups.append(group)
        idx = self.group_tab_bar.addTab(group['group_id'])
        self.group_tab_bar.setCurrentIndex(idx)

    def _remove_group(self, index):
        """移除编队组"""
        if len(self._groups) <= 1:
            QMessageBox.warning(self, "无法移除", "至少需要保留一个编队组")
            return

        reply = QMessageBox.question(
            self, "确认移除",
            f"确定要移除 '{self._groups[index]['group_id']}' 吗？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        self._groups.pop(index)
        self._is_loading = True
        self.group_tab_bar.removeTab(index)
        self._is_loading = False

        # 重新编号
        self._update_tab_labels()

        # 调整当前索引
        if self._current_idx >= len(self._groups):
            self._current_idx = len(self._groups) - 1
        if self._current_idx >= 0:
            self.group_tab_bar.setCurrentIndex(self._current_idx)
            self._load_current_group()

    def _update_tab_labels(self):
        """更新所有标签页标题编号"""
        for i in range(len(self._groups)):
            self._groups[i]['group_id'] = f'编队组 {i + 1}'
            self.group_tab_bar.setTabText(i, f'编队组 {i + 1}')

    def _on_tab_changed(self, index):
        """标签页切换事件"""
        if self._is_loading or index < 0 or index >= len(self._groups):
            return
        self._save_current_group()
        self._current_idx = index
        self._load_current_group()

    def _save_current_group(self):
        """保存当前编辑控件的值到当前组数据"""
        if self._current_idx < 0 or self._current_idx >= len(self._groups):
            return

        g = self._groups[self._current_idx]
        leader = self.leader_combo.currentText()
        g['leader_id'] = leader

        followers = self._get_selected_followers()
        # 确保领队不在跟随者列表中
        if leader in followers:
            followers.remove(leader)
        g['follower_ids'] = followers

        data = self.formation_combo.currentData()
        g['formation_type'] = int(data) if data is not None else 0
        g['spacing_along'] = round(self.spacing_along_spin.value(), 2)
        g['spacing_cross'] = round(self.spacing_cross_spin.value(), 2)

    def _load_current_group(self):
        """加载当前组数据到编辑控件"""
        if self._current_idx < 0 or self._current_idx >= len(self._groups):
            return

        self._is_loading = True
        try:
            g = self._groups[self._current_idx]
            available = self._get_available_usvs(exclude_group_idx=self._current_idx)

            # --- 领队下拉框 ---
            self.leader_combo.clear()
            leader_options = list(available)
            # 确保当前组的领队在选项中
            if g['leader_id'] and g['leader_id'] not in leader_options:
                leader_options.insert(0, g['leader_id'])
            self.leader_combo.addItems(leader_options)
            if g['leader_id']:
                idx = self.leader_combo.findText(g['leader_id'])
                if idx >= 0:
                    self.leader_combo.setCurrentIndex(idx)

            # --- 跟随者列表 ---
            leader_id = g['leader_id']
            follower_options = [uid for uid in available if uid != leader_id]
            # 确保当前组的跟随者在列表中
            for fid in g.get('follower_ids', []):
                if fid not in follower_options and fid != leader_id:
                    follower_options.append(fid)

            self.follower_list.clear()
            for uid in follower_options:
                item = QListWidgetItem(uid)
                item.setSelected(uid in g.get('follower_ids', []))
                self.follower_list.addItem(item)

            # --- 队形 ---
            ft = g.get('formation_type', 0)
            for i in range(self.formation_combo.count()):
                if self.formation_combo.itemData(i) == ft:
                    self.formation_combo.setCurrentIndex(i)
                    break

            # --- 间距 ---
            self.spacing_along_spin.setValue(g.get('spacing_along', 1.0))
            self.spacing_cross_spin.setValue(g.get('spacing_cross', 1.0))


        finally:
            self._is_loading = False

        self._update_preview()

    def _get_available_usvs(self, exclude_group_idx=None):
        """
        获取未被其他编队组使用的 USV 列表
        
        Args:
            exclude_group_idx: 排除的组索引（该组的 USV 不计入已使用）。
                              传 None 表示所有组都计入（用于检查还剩多少空闲 USV）。
        
        Returns:
            可用的 USV ID 列表
        """
        used = set()
        for i, g in enumerate(self._groups):
            if i == exclude_group_idx:
                continue
            if g.get('leader_id'):
                used.add(g['leader_id'])
            used.update(g.get('follower_ids', []))
        return [uid for uid in self._usv_ids if uid not in used]

    # ==================== UI 交互 ====================

    def _on_leader_changed(self, index):
        """领队切换时重新加载当前组（刷新跟随者可选列表）"""
        if self._is_loading:
            return
        self._save_current_group()
        self._load_current_group()

    def _on_config_changed(self):
        """配置变更（跟随者选择/队形/间距）— 更新预览"""
        if self._is_loading:
            return
        self._update_preview()

    def _select_all_followers(self):
        """全选跟随者"""
        for i in range(self.follower_list.count()):
            self.follower_list.item(i).setSelected(True)

    def _deselect_all_followers(self):
        """取消全选"""
        for i in range(self.follower_list.count()):
            self.follower_list.item(i).setSelected(False)

    def _get_selected_followers(self) -> list:
        """获取已选中的跟随者列表 (按列表顺序)"""
        followers = []
        for i in range(self.follower_list.count()):
            item = self.follower_list.item(i)
            if item.isSelected():
                followers.append(item.text())
        return followers

    def _update_preview(self):
        """更新队形预览"""
        followers = self._get_selected_followers()
        formation_type = self.formation_combo.currentData()
        spacing_along = self.spacing_along_spin.value()
        spacing_cross = self.spacing_cross_spin.value()
        leader_id = self.leader_combo.currentText()

        if not followers:
            self.preview_widget.set_preview_data([], [])
            self.preview_info_label.setText("请选择至少一个跟随者")
            return

        # 计算预览坐标
        positions = FormationController.get_formation_preview(
            formation_type, len(followers), spacing_along, spacing_cross
        )

        # 标签
        labels = [f"L: {leader_id}"]
        for i, fid in enumerate(followers):
            labels.append(f"F{i+1}: {fid}")

        formation_name = self.FORMATION_TYPE_NAMES.get(formation_type, "")
        group_label = ""
        if 0 <= self._current_idx < len(self._groups):
            group_label = self._groups[self._current_idx]['group_id']

        self.preview_widget.set_preview_data(
            positions, labels, f"{group_label} - {formation_name}"
        )

        self.preview_info_label.setText(
            f"{group_label}: 领队 {leader_id} | {len(followers)} 艘跟随 | "
            f"间距 {spacing_along:.2f}m × {spacing_cross:.2f}m"
        )

    # ==================== 确认启动 ====================

    def _on_confirm(self):
        """确认所有编队组配置"""
        # 保存当前编辑
        self._save_current_group()

        # 全局参数
        update_rate = round(self.update_rate_spin.value(), 2)
        leader_timeout = round(self.leader_timeout_spin.value(), 2)
        follower_speed = round(self.follower_speed_spin.value(), 2)

        # 验证所有编队组
        errors = []
        for i, g in enumerate(self._groups):
            gname = g['group_id']
            if not g.get('leader_id'):
                errors.append(f"{gname}: 未选择领队")
            if not g.get('follower_ids'):
                errors.append(f"{gname}: 未选择跟随者")

        if errors:
            QMessageBox.warning(
                self, "配置错误",
                "以下编队组配置有误:\n\n" + "\n".join(errors)
            )
            return

        # 构建配置列表
        type_names = {
            0: "人字形", 1: "横排一字形", 2: "菱形", 3: "三角形",
            4: "纵列一字形", 5: "S形", 6: "护卫",
        }
        group_configs = []
        summary_lines = []

        for i, g in enumerate(self._groups):
            cfg = {
                'group_id': g['group_id'],
                'leader_id': g['leader_id'],
                'follower_ids': list(g['follower_ids']),
                'formation_type': g.get('formation_type', 0),
                'spacing_along': round(g.get('spacing_along', 1.0), 2),
                'spacing_cross': round(g.get('spacing_cross', 1.0), 2),
                'update_rate': update_rate,
                'leader_timeout': leader_timeout,
                'follower_speed': follower_speed,
            }
            group_configs.append(cfg)

            ft_name = type_names.get(cfg['formation_type'], "未知")
            summary_lines.append(
                f"  {g['group_id']}:\n"
                f"    队形: {ft_name}\n"
                f"    领队: {cfg['leader_id']}\n"
                f"    跟随者: {', '.join(cfg['follower_ids'])} ({len(cfg['follower_ids'])} 艘)\n"
                f"    间距: {cfg['spacing_along']:.2f}m × {cfg['spacing_cross']:.2f}m"
            )

        total_usvs = sum(1 + len(g['follower_ids']) for g in group_configs)
        msg = (
            f"即将启动 {len(group_configs)} 个编队组 "
            f"(共 {total_usvs} 艘 USV):\n\n"
            + "\n\n".join(summary_lines)
            + f"\n\n全局参数:\n"
            + f"  更新频率: {update_rate:.1f}Hz\n"
            + f"  领队超时: {leader_timeout:.1f}s\n"
            + f"  跟随速度: "
            + ("默认" if follower_speed == 0 else f"{follower_speed:.2f} m/s")
            + "\n\n确认启动？"
        )

        reply = QMessageBox.question(
            self, "确认启动多编队", msg,
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.formation_confirmed.emit(group_configs)
            self.accept()
