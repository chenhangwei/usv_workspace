#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
#
# This file is part of the USV Workspace project.
#
# Implementation of mission queue widget for multi-task navigation GUI.
#
# Author: chenhangwei
# Date: 2026-03-09
"""
多任务导航队列 GUI 组件

提供可视化的任务队列管理界面，支持：
- 多列信息显示（序号、状态、名称、步骤、USV数、航程、速度、模式、过渡、耗时、来源）
- 可拖拽排序的任务列表
- 添加/移除/上移/下移任务
- 开始/暂停/停止/跳过队列执行
- 保存/加载队列
- 右键菜单设置过渡模式和重命名
"""

from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTreeWidget, QTreeWidgetItem,
    QPushButton, QLabel, QMenu, QFileDialog, QInputDialog,
    QAbstractItemView, QMessageBox, QFrame, QHeaderView
)
from PyQt5.QtGui import QColor, QBrush, QDropEvent

from gs_gui.mission_queue_manager import (
    MissionQueueManager, TaskStatus, TransitionMode
)
from gs_gui.style_manager import is_dark_theme


class FlatTreeWidget(QTreeWidget):
    """不允许嵌套的 QTreeWidget —— 仅支持同级拖拽排序。"""

    reorder_occurred = pyqtSignal()  # 拖拽排序完成后发射

    def dropEvent(self, event: QDropEvent):
        # 只接受 AboveItem / BelowItem，禁止 OnItem（变成子项）
        drop_indicator = self.dropIndicatorPosition()
        if drop_indicator == QAbstractItemView.OnItem:
            event.ignore()
            return
        super().dropEvent(event)
        # 防止 drop 把 item 嵌套到子级：把所有子项提升到顶层
        self._flatten()
        self.reorder_occurred.emit()

    def _flatten(self):
        """确保所有 item 都在顶层，不存在嵌套。"""
        changed = True
        while changed:
            changed = False
            for i in range(self.topLevelItemCount()):
                parent_item = self.topLevelItem(i)
                if parent_item is None:
                    continue
                while parent_item.childCount() > 0:
                    child = parent_item.takeChild(0)
                    idx = self.indexOfTopLevelItem(parent_item)
                    self.insertTopLevelItem(idx + 1, child)
                    changed = True


# 任务状态对应的颜色
STATUS_COLORS = {
    TaskStatus.PENDING.value: QColor(170, 170, 170),      # 灰色
    TaskStatus.RUNNING.value: QColor(0, 200, 100),         # 绿色
    TaskStatus.PAUSED.value: QColor(255, 180, 0),          # 橙色
    TaskStatus.COMPLETED.value: QColor(80, 140, 220),      # 蓝色
    TaskStatus.FAILED.value: QColor(220, 60, 60),          # 红色
}

STATUS_TEXT = {
    TaskStatus.PENDING.value: "⏳ 待执行",
    TaskStatus.RUNNING.value: "▶ 运行中",
    TaskStatus.PAUSED.value: "⏸ 已暂停",
    TaskStatus.COMPLETED.value: "✅ 已完成",
    TaskStatus.FAILED.value: "❌ 失败",
}

TRANSITION_LABELS = {
    TransitionMode.SEAMLESS.value: "无缝",
    TransitionMode.WAIT_CONFIRM.value: "确认",
}

# 列定义
COL_INDEX = 0       # 序号
COL_STATUS = 1      # 状态
COL_NAME = 2        # 任务名称
COL_STEPS = 3       # 步骤
COL_USV = 4         # USV 数
COL_DISTANCE = 5    # 航程(m)
COL_SPEED = 6       # 速度(m/s)
COL_NAV_MODE = 7    # 导航模式
COL_TRANSITION = 8  # 过渡
COL_ELAPSED = 9     # 耗时
COL_SOURCE = 10     # 来源文件
COL_COUNT = 11      # 列总数

COLUMN_HEADERS = [
    "#", "状态", "任务名称", "步骤", "USV",
    "航程(m)", "速度", "模式", "过渡", "耗时", "来源"
]


def _format_elapsed(seconds: float) -> str:
    """将秒数格式化为 mm:ss 或 hh:mm:ss"""
    if seconds <= 0:
        return "-"
    seconds = int(seconds)
    if seconds < 3600:
        return f"{seconds // 60:02d}:{seconds % 60:02d}"
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h}:{m:02d}:{s:02d}"


class MissionQueueWidget(QWidget):
    """多任务导航队列管理面板"""

    # 选中任务变更信号（用于更新 2D 预览）
    task_selected = pyqtSignal(list)   # position_list

    def __init__(self, queue_manager: MissionQueueManager, parent=None):
        super().__init__(parent)
        self.queue_manager = queue_manager
        self._setup_ui()
        self._connect_signals()
        # 定时器：运行时每秒刷新耗时列
        self._elapsed_timer = QTimer(self)
        self._elapsed_timer.timeout.connect(self._tick_elapsed)
        self._elapsed_timer.setInterval(1000)

    def _setup_ui(self):
        """构建 UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # ---- 工具栏 ----
        toolbar = QHBoxLayout()
        toolbar.setSpacing(3)

        self.btn_add = QPushButton("+")
        self.btn_add.setToolTip("添加 XML 任务文件")
        self.btn_add.setFixedWidth(36)

        self.btn_remove = QPushButton("−")
        self.btn_remove.setToolTip("移除选中任务")
        self.btn_remove.setFixedWidth(36)

        self.btn_up = QPushButton("▲")
        self.btn_up.setToolTip("上移")
        self.btn_up.setFixedWidth(36)

        self.btn_down = QPushButton("▼")
        self.btn_down.setToolTip("下移")
        self.btn_down.setFixedWidth(36)

        sep1 = QFrame()
        sep1.setFrameShape(QFrame.VLine)
        sep1.setFixedWidth(2)

        self.btn_start = QPushButton("⏩ 开始")
        self.btn_start.setToolTip("开始执行任务队列")

        self.btn_pause = QPushButton("⏸ 暂停")
        self.btn_pause.setToolTip("暂停/恢复当前任务")

        self.btn_stop = QPushButton("⏹ 停止")
        self.btn_stop.setToolTip("停止队列执行")

        self.btn_skip = QPushButton("⏭ 跳过")
        self.btn_skip.setToolTip("跳过当前任务")

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.VLine)
        sep2.setFixedWidth(2)

        self.btn_save = QPushButton("💾")
        self.btn_save.setToolTip("保存队列")
        self.btn_save.setFixedWidth(36)

        self.btn_load = QPushButton("📂")
        self.btn_load.setToolTip("加载队列")
        self.btn_load.setFixedWidth(36)

        self.btn_clear = QPushButton("🗑")
        self.btn_clear.setToolTip("清除已完成的任务")
        self.btn_clear.setFixedWidth(36)

        for w in [self.btn_add, self.btn_remove, self.btn_up, self.btn_down,
                  sep1, self.btn_start, self.btn_pause, self.btn_stop, self.btn_skip,
                  sep2, self.btn_save, self.btn_load, self.btn_clear]:
            toolbar.addWidget(w)
        toolbar.addStretch()
        layout.addLayout(toolbar)

        # ---- 任务列表 (FlatTreeWidget 多列，禁止嵌套) ----
        self.task_tree = FlatTreeWidget()
        self.task_tree.setColumnCount(COL_COUNT)
        self.task_tree.setHeaderLabels(COLUMN_HEADERS)
        self.task_tree.setRootIsDecorated(False)
        self.task_tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self.task_tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.task_tree.setMinimumHeight(120)
        self.task_tree.setIndentation(0)
        # 拖拽排序（仅平级移动）
        self.task_tree.setDragDropMode(QAbstractItemView.InternalMove)
        self.task_tree.setDefaultDropAction(Qt.MoveAction)
        self.task_tree.setDragEnabled(True)
        self.task_tree.setAcceptDrops(True)

        # 列宽设置
        header = self.task_tree.header()
        header.setStretchLastSection(False)
        header.setSectionResizeMode(COL_NAME, QHeaderView.Stretch)  # 任务名称自适应
        self.task_tree.setColumnWidth(COL_INDEX, 30)
        self.task_tree.setColumnWidth(COL_STATUS, 75)
        self.task_tree.setColumnWidth(COL_STEPS, 55)
        self.task_tree.setColumnWidth(COL_USV, 35)
        self.task_tree.setColumnWidth(COL_DISTANCE, 60)
        self.task_tree.setColumnWidth(COL_SPEED, 60)
        self.task_tree.setColumnWidth(COL_NAV_MODE, 65)
        self.task_tree.setColumnWidth(COL_TRANSITION, 40)
        self.task_tree.setColumnWidth(COL_ELAPSED, 55)
        self.task_tree.setColumnWidth(COL_SOURCE, 120)

        self.task_tree.setStyleSheet(self._get_tree_stylesheet())
        self.task_tree.setAlternatingRowColors(True)
        layout.addWidget(self.task_tree, 1)

        # ---- 确认栏（等待确认时显示）----
        self.confirm_bar = QWidget()
        confirm_layout = QHBoxLayout(self.confirm_bar)
        confirm_layout.setContentsMargins(5, 2, 5, 2)
        self.confirm_label = QLabel("⏳ 当前任务已完成，等待确认开始下一个任务")
        _warn_color = '#f9a825' if is_dark_theme() else '#d48806'
        self.confirm_label.setStyleSheet(f"color: {_warn_color}; font-weight: bold;")
        self.btn_confirm = QPushButton("✅ 确认开始下一个")
        self.btn_confirm.setStyleSheet("""
            QPushButton {
                background: #2e7d32;
                color: white;
                border-radius: 4px;
                padding: 5px 12px;
                font-weight: bold;
            }
            QPushButton:hover { background: #388e3c; }
        """)
        confirm_layout.addWidget(self.confirm_label)
        confirm_layout.addStretch()
        confirm_layout.addWidget(self.btn_confirm)
        self.confirm_bar.hide()
        layout.addWidget(self.confirm_bar)

        # ---- 进度显示 ----
        self.progress_label = QLabel("队列空闲")
        self.progress_label.setAlignment(Qt.AlignCenter)
        self.progress_label.setStyleSheet(self._get_progress_label_stylesheet())
        self.progress_label.setMinimumHeight(28)
        layout.addWidget(self.progress_label)

        # 按钮样式
        btn_style = self._get_btn_stylesheet()
        for btn in [self.btn_add, self.btn_remove, self.btn_up, self.btn_down,
                    self.btn_start, self.btn_pause, self.btn_stop, self.btn_skip,
                    self.btn_save, self.btn_load, self.btn_clear]:
            btn.setStyleSheet(btn_style)

    def _get_tree_stylesheet(self):
        """获取树控件样式"""
        if is_dark_theme():
            return """
                QTreeWidget {
                    background: #1e1e1e;
                    border: 1px solid #444;
                    color: #ddd;
                    font-size: 12px;
                    alternate-background-color: #252525;
                }
                QTreeWidget::item {
                    padding: 3px 4px;
                    border-bottom: 1px solid #333;
                }
                QTreeWidget::item:selected {
                    background: #2a4a6b;
                    color: #fff;
                }
                QTreeWidget::item:hover {
                    background: #2a2a2a;
                }
                QHeaderView::section {
                    background: #2d2d2d;
                    color: #aaa;
                    border: 1px solid #444;
                    padding: 3px 4px;
                    font-size: 11px;
                    font-weight: bold;
                }
            """
        else:
            return """
                QTreeWidget {
                    background: #ffffff;
                    border: 1px solid #d0d0d0;
                    color: #333;
                    font-size: 12px;
                    alternate-background-color: #f9f9f9;
                }
                QTreeWidget::item {
                    padding: 3px 4px;
                    border-bottom: 1px solid #e0e0e0;
                }
                QTreeWidget::item:selected {
                    background: #e6f7ff;
                    color: #0078d7;
                }
                QTreeWidget::item:hover {
                    background: #f0f8ff;
                }
                QHeaderView::section {
                    background: #f0f0f0;
                    color: #333;
                    border: 1px solid #d0d0d0;
                    padding: 3px 4px;
                    font-size: 11px;
                    font-weight: bold;
                }
            """

    def _get_progress_label_stylesheet(self):
        if is_dark_theme():
            return """
                QLabel {
                    background: #2d2d2d;
                    color: #aaa;
                    border: 1px solid #444;
                    border-radius: 4px;
                    padding: 4px;
                    font-size: 12px;
                    font-weight: bold;
                }
            """
        else:
            return """
                QLabel {
                    background: #f0f0f0;
                    color: #666;
                    border: 1px solid #d0d0d0;
                    border-radius: 4px;
                    padding: 4px;
                    font-size: 12px;
                    font-weight: bold;
                }
            """

    def _get_btn_stylesheet(self):
        if is_dark_theme():
            return """
                QPushButton {
                    background: #333;
                    border: 1px solid #555;
                    color: #fff;
                    border-radius: 3px;
                    padding: 4px 8px;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover { background: #444; color: #fff; }
                QPushButton:pressed { background: #555; }
                QPushButton:disabled { color: #666; }
            """
        else:
            return """
                QPushButton {
                    background: #ffffff;
                    border: 1px solid #cccccc;
                    color: #333;
                    border-radius: 3px;
                    padding: 4px 8px;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover { background: #e6f7ff; color: #0078d7; }
                QPushButton:pressed { background: #cceeff; }
                QPushButton:disabled { color: #a0a0a0; }
            """

    def _get_context_menu_stylesheet(self):
        if is_dark_theme():
            return """
                QMenu {
                    background: #2d2d2d;
                    color: #ddd;
                    border: 1px solid #555;
                }
                QMenu::item:selected { background: #3a5a8a; }
            """
        else:
            return """
                QMenu {
                    background: #ffffff;
                    color: #333;
                    border: 1px solid #d0d0d0;
                }
                QMenu::item:selected { background: #e6f7ff; color: #0078d7; }
            """

    def set_theme(self, theme_name):
        """切换主题"""
        self.task_tree.setStyleSheet(self._get_tree_stylesheet())
        self.progress_label.setStyleSheet(self._get_progress_label_stylesheet())
        btn_style = self._get_btn_stylesheet()
        for btn in [self.btn_add, self.btn_remove, self.btn_up, self.btn_down,
                    self.btn_start, self.btn_pause, self.btn_stop, self.btn_skip,
                    self.btn_save, self.btn_load, self.btn_clear]:
            btn.setStyleSheet(btn_style)
        _warn_color = '#f9a825' if is_dark_theme() else '#d48806'
        self.confirm_label.setStyleSheet(f"color: {_warn_color}; font-weight: bold;")

    def _connect_signals(self):
        """连接按钮和信号"""
        # 工具栏按钮
        self.btn_add.clicked.connect(self._on_add)
        self.btn_remove.clicked.connect(self._on_remove)
        self.btn_up.clicked.connect(lambda: self._on_move(-1))
        self.btn_down.clicked.connect(lambda: self._on_move(1))
        self.btn_start.clicked.connect(self._on_start)
        self.btn_pause.clicked.connect(self._on_pause)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_skip.clicked.connect(self._on_skip)
        self.btn_save.clicked.connect(self._on_save)
        self.btn_load.clicked.connect(self._on_load)
        self.btn_clear.clicked.connect(self.queue_manager.clear_completed)
        self.btn_confirm.clicked.connect(self._on_confirm)

        # 右键菜单
        self.task_tree.customContextMenuRequested.connect(self._show_context_menu)

        # 双击任务 -> 跳转预览
        self.task_tree.itemDoubleClicked.connect(self._on_item_double_clicked)

        # 拖拽排序结束（FlatTreeWidget 自定义信号）
        self.task_tree.reorder_occurred.connect(self._on_drag_reorder)

        # 队列管理器信号
        if hasattr(self.queue_manager.ros_signal, 'mission_queue_updated'):
            self.queue_manager.ros_signal.mission_queue_updated.connect(self._refresh_list)
        if hasattr(self.queue_manager.ros_signal, 'mission_queue_progress'):
            self.queue_manager.ros_signal.mission_queue_progress.connect(self._update_progress)

    # =============================================================
    #  按钮事件处理
    # =============================================================

    def _on_add(self):
        """添加任务文件"""
        files, _ = QFileDialog.getOpenFileNames(
            self, "选择 XML 任务文件", "", "XML Files (*.xml)"
        )
        for fp in files:
            self.queue_manager.add_task_from_file(fp)

    def _on_remove(self):
        """移除选中任务"""
        task_id = self._get_selected_task_id()
        if task_id:
            self.queue_manager.remove_task(task_id)

    def _on_move(self, direction):
        """上移/下移"""
        task_id = self._get_selected_task_id()
        if task_id:
            self.queue_manager.move_task(task_id, direction)

    def _on_start(self):
        """开始队列"""
        if self.queue_manager.is_running:
            self.queue_manager.resume_queue()
        else:
            self.queue_manager.start_queue()

    def _on_pause(self):
        """暂停队列"""
        self.queue_manager.pause_queue()

    def _on_stop(self):
        """停止队列"""
        if not self.queue_manager.is_running:
            return
        reply = QMessageBox.question(
            self, "确认停止", "确定要停止任务队列吗？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.queue_manager.stop_queue()

    def _on_skip(self):
        """跳过当前任务"""
        self.queue_manager.skip_current()

    def _on_save(self):
        """保存队列：提示输入队列名称并选择保存位置"""
        queue_name, ok = QInputDialog.getText(
            self, "保存任务队列", "队列名称:", text="我的任务队列"
        )
        if not ok or not queue_name.strip():
            return
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存任务队列", queue_name.strip() + ".json",
            "JSON Files (*.json)"
        )
        if file_path:
            self.queue_manager.save_queue(file_path, queue_name.strip())

    def _on_load(self):
        """加载队列：打开文件对话框选择队列文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开任务队列", "", "JSON Files (*.json)"
        )
        if file_path:
            self.queue_manager.load_queue(file_path)

    def _on_confirm(self):
        """确认开始下一个任务"""
        self.queue_manager.confirm_next()

    def _on_drag_reorder(self):
        """拖拽排序后同步到管理器"""
        new_order = []
        for i in range(self.task_tree.topLevelItemCount()):
            item = self.task_tree.topLevelItem(i)
            task_id = item.data(COL_INDEX, Qt.UserRole)
            if task_id:
                new_order.append(task_id)
        if new_order:
            self.queue_manager.reorder(new_order)

    def _on_item_double_clicked(self, item, column):
        """双击任务时跳转到反馈界面并预览路径"""
        task_id = item.data(COL_INDEX, Qt.UserRole)
        if not task_id:
            return
        for t in self.queue_manager.get_queue():
            if t.task_id == task_id and t.position_list:
                self.task_selected.emit(t.position_list)
                break

    # =============================================================
    #  右键菜单
    # =============================================================

    def _show_context_menu(self, pos):
        """显示右键上下文菜单"""
        item = self.task_tree.itemAt(pos)
        if not item:
            return

        task_id = item.data(COL_INDEX, Qt.UserRole)
        if not task_id:
            return

        # 找到对应的任务
        task = None
        for t in self.queue_manager.get_queue():
            if t.task_id == task_id:
                task = t
                break
        if not task:
            return

        menu = QMenu(self)
        menu.setStyleSheet(self._get_context_menu_stylesheet())

        # 过渡模式设置
        mode_menu = menu.addMenu("过渡模式")
        act_seamless = mode_menu.addAction("🔄 无缝衔接")
        act_seamless.setCheckable(True)
        act_seamless.setChecked(task.transition_mode == TransitionMode.SEAMLESS.value)
        act_confirm = mode_menu.addAction("⏸ 等待确认")
        act_confirm.setCheckable(True)
        act_confirm.setChecked(task.transition_mode == TransitionMode.WAIT_CONFIRM.value)

        # 重命名
        act_rename = menu.addAction("✏️ 重命名")

        # 移除
        act_remove = menu.addAction("🗑 移除")

        # 显示菜单
        action = menu.exec_(self.task_tree.mapToGlobal(pos))

        if action == act_seamless:
            self.queue_manager.set_transition_mode(task_id, TransitionMode.SEAMLESS)
        elif action == act_confirm:
            self.queue_manager.set_transition_mode(task_id, TransitionMode.WAIT_CONFIRM)
        elif action == act_rename:
            new_name, ok = QInputDialog.getText(
                self, "重命名任务", "新任务名:", text=task.task_name
            )
            if ok and new_name.strip():
                self.queue_manager.rename_task(task_id, new_name.strip())
        elif action == act_remove:
            self.queue_manager.remove_task(task_id)

    # =============================================================
    #  列表刷新
    # =============================================================

    def _refresh_list(self, queue_data: list):
        """根据队列数据刷新列表显示"""
        # 保存当前选中
        current_task_id = self._get_selected_task_id()

        self.task_tree.blockSignals(True)
        self.task_tree.clear()

        for idx, td in enumerate(queue_data, start=1):
            status = td.get('status', TaskStatus.PENDING.value)
            total_steps = td.get('total_steps', 0)
            current_step = td.get('current_step', 0)

            # 步骤列文本
            if status == TaskStatus.RUNNING.value and total_steps > 0:
                steps_text = f"{current_step}/{total_steps}"
            else:
                steps_text = str(total_steps)

            item = QTreeWidgetItem([
                str(idx),                                                   # COL_INDEX
                STATUS_TEXT.get(status, "⏳ 待执行"),                         # COL_STATUS
                td.get('task_name', ''),                                    # COL_NAME
                steps_text,                                                 # COL_STEPS
                str(td.get('usv_count', 0)),                               # COL_USV
                str(td.get('estimated_distance', 0)),                      # COL_DISTANCE
                td.get('speed_range_str', '-'),                            # COL_SPEED
                td.get('nav_modes_str', ''),                               # COL_NAV_MODE
                TRANSITION_LABELS.get(
                    td.get('transition_mode', TransitionMode.SEAMLESS.value), "无缝"
                ),                                                          # COL_TRANSITION
                _format_elapsed(td.get('elapsed', 0)),                     # COL_ELAPSED
                td.get('source_file', ''),                                 # COL_SOURCE
            ])

            # 存储 task_id 到 UserRole
            item.setData(COL_INDEX, Qt.UserRole, td['task_id'])

            # Tooltip 显示完整路径
            item.setToolTip(COL_SOURCE, td.get('file_path', ''))
            item.setToolTip(COL_NAME, td.get('task_name', ''))

            # 居中对齐小列
            for col in [COL_INDEX, COL_STATUS, COL_STEPS, COL_USV,
                        COL_DISTANCE, COL_SPEED, COL_NAV_MODE,
                        COL_TRANSITION, COL_ELAPSED]:
                item.setTextAlignment(col, Qt.AlignCenter)

            # 状态颜色 — 应用到所有列
            color = STATUS_COLORS.get(status, QColor(170, 170, 170))
            brush = QBrush(color)
            for col in range(COL_COUNT):
                item.setForeground(col, brush)

            # 已完成/失败的任务不可拖拽
            if status in (TaskStatus.COMPLETED.value, TaskStatus.FAILED.value):
                item.setFlags(item.flags() & ~Qt.ItemIsDragEnabled)

            self.task_tree.addTopLevelItem(item)

        self.task_tree.blockSignals(False)

        # 恢复选中
        if current_task_id:
            for i in range(self.task_tree.topLevelItemCount()):
                if self.task_tree.topLevelItem(i).data(COL_INDEX, Qt.UserRole) == current_task_id:
                    self.task_tree.setCurrentItem(self.task_tree.topLevelItem(i))
                    break

        # 更新确认栏可见性
        self.confirm_bar.setVisible(self.queue_manager.is_waiting_confirm)

        # 更新按钮状态
        self._update_button_states()

        # 管理耗时定时器
        has_running = any(td.get('status') == TaskStatus.RUNNING.value for td in queue_data)
        if has_running and not self._elapsed_timer.isActive():
            self._elapsed_timer.start()
        elif not has_running and self._elapsed_timer.isActive():
            self._elapsed_timer.stop()

    def _tick_elapsed(self):
        """每秒刷新运行中任务的耗时列"""
        import time as _time
        now = _time.time()
        for i in range(self.task_tree.topLevelItemCount()):
            item = self.task_tree.topLevelItem(i)
            task_id = item.data(COL_INDEX, Qt.UserRole)
            if not task_id:
                continue
            for t in self.queue_manager.get_queue():
                if t.task_id == task_id and t.status == TaskStatus.RUNNING.value and t.start_time:
                    elapsed = now - t.start_time
                    item.setText(COL_ELAPSED, _format_elapsed(elapsed))
                    break

    def _update_progress(self, progress: dict):
        """更新底部进度显示"""
        if not progress.get('queue_running', False) and not progress.get('waiting_confirm', False):
            total = progress.get('total_tasks', 0)
            completed = progress.get('completed_tasks', 0)
            if total == 0:
                self.progress_label.setText("队列空闲")
            elif completed == total:
                self.progress_label.setText(f"🏁 全部完成 ({total} 个任务)")
            else:
                self.progress_label.setText(f"队列就绪: {total} 个任务")
            return

        idx = progress.get('current_task_index', 0)
        total = progress.get('total_tasks', 0)
        name = progress.get('current_task_name', '')
        step = progress.get('current_task_step', 0)
        total_steps = progress.get('current_task_total_steps', 0)
        waiting = progress.get('waiting_confirm', False)

        if waiting:
            self.progress_label.setText(f"⏳ 等待确认 | 任务 {idx}/{total}")
        else:
            self.progress_label.setText(
                f"▶ 任务 {idx}/{total}: {name} | 步骤 {step}/{total_steps}"
            )

        # 更新确认栏
        self.confirm_bar.setVisible(waiting)

    def _update_button_states(self):
        """根据队列状态更新按钮启用/禁用"""
        running = self.queue_manager.is_running
        has_tasks = len(self.queue_manager.get_queue()) > 0
        has_pending = any(t.status == TaskStatus.PENDING.value for t in self.queue_manager.get_queue())

        self.btn_start.setEnabled(not running and has_pending)
        self.btn_pause.setEnabled(running)
        self.btn_stop.setEnabled(running)
        self.btn_skip.setEnabled(running)
        self.btn_add.setEnabled(True)
        self.btn_remove.setEnabled(has_tasks and not running)
        self.btn_up.setEnabled(has_tasks)
        self.btn_down.setEnabled(has_tasks)

    # =============================================================
    #  辅助方法
    # =============================================================

    def _get_selected_task_id(self) -> str:
        """获取当前选中任务的 task_id"""
        item = self.task_tree.currentItem()
        if item:
            return item.data(COL_INDEX, Qt.UserRole) or ""
        return ""
