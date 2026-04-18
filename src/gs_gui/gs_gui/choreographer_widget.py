#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
表演编排 GUI 组件

提供自然语言输入界面，通过 LLM 将描述转换为集群路径 XML，
可直接保存到文件并加载到 ClusterTaskManager 执行。
"""

import logging
import os
import traceback
from functools import partial
from typing import Optional, Callable, List

from PyQt5.QtCore import Qt, pyqtSignal, QThread, QSettings
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QPlainTextEdit,
    QPushButton, QLabel, QComboBox, QSpinBox, QDoubleSpinBox,
    QLineEdit, QGroupBox, QGridLayout, QFileDialog, QMessageBox,
    QSplitter, QFrame, QTabWidget, QCheckBox, QScrollArea,
    QSizePolicy, QProgressBar
)
from PyQt5.QtGui import QFont, QTextCharFormat, QSyntaxHighlighter

from gs_gui.choreographer import (
    Choreographer, ChoreographyPlan, SegmentPlan,
    parse_plan_json, plan_to_steps, steps_to_xml, save_xml, SYSTEM_PROMPT,
)
from gs_gui.shape_library import SHAPE_REGISTRY, FORMATION_REGISTRY

logger = logging.getLogger(__name__)


# ────────────────────── XML 语法高亮 ──────────────────────

class XmlHighlighter(QSyntaxHighlighter):
    """简单的 XML 语法高亮。"""

    def highlightBlock(self, text: str):
        import re
        # 标签名
        fmt_tag = QTextCharFormat()
        fmt_tag.setForeground(Qt.blue)
        for match in re.finditer(r'</?(\w+)', text):
            self.setFormat(match.start(), match.end() - match.start(), fmt_tag)

        # 属性值
        fmt_attr = QTextCharFormat()
        fmt_attr.setForeground(Qt.darkGreen)
        for match in re.finditer(r'"[^"]*"', text):
            self.setFormat(match.start(), match.end() - match.start(), fmt_attr)

        # 注释
        fmt_comment = QTextCharFormat()
        fmt_comment.setForeground(Qt.gray)
        for match in re.finditer(r'<!--.*?-->', text):
            self.setFormat(match.start(), match.end() - match.start(), fmt_comment)


# ────────────────────── LLM 工作线程 ──────────────────────

class GenerateWorker(QThread):
    """在后台线程中执行 LLM 调用，避免阻塞 GUI。"""
    finished = pyqtSignal(str)       # XML 结果
    error = pyqtSignal(str)          # 错误信息
    plan_ready = pyqtSignal(str)     # 中间 JSON 计划（用于调试）

    def __init__(self, choreographer: Choreographer,
                 user_input: str, usv_ids: List[str]):
        super().__init__()
        self.choreographer = choreographer
        self.user_input = user_input
        self.usv_ids = usv_ids

    def run(self):
        try:
            xml_str = self.choreographer.generate_from_text(
                self.user_input, self.usv_ids
            )
            # 发送中间计划
            if self.choreographer.last_plan:
                import json
                plan = self.choreographer.last_plan
                plan_info = {
                    "title": plan.title,
                    "usv_ids": plan.usv_ids,
                    "segments": [
                        {"type": s.type, "shape": s.shape,
                         "description": s.description}
                        for s in plan.segments
                    ]
                }
                self.plan_ready.emit(json.dumps(plan_info, ensure_ascii=False, indent=2))
            self.finished.emit(xml_str)
        except Exception as e:
            self.error.emit(f"{type(e).__name__}: {e}\n{traceback.format_exc()}")


# ────────────────────── 主组件 ──────────────────────

class ChoreographerWidget(QWidget):
    """表演编排 Tab 组件。"""

    # 发给主窗口的信号
    xml_generated = pyqtSignal(str)     # 生成的 XML 文件路径
    status_message = pyqtSignal(str)    # 状态消息

    def __init__(self, task_manager=None, list_manager=None,
                 path_dir: str = "", parent=None):
        super().__init__(parent)
        self.task_manager = task_manager
        self.list_manager = list_manager
        self.path_dir = path_dir or os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            "Path File"
        )
        self.choreographer: Optional[Choreographer] = None
        self._worker: Optional[GenerateWorker] = None
        self._settings = QSettings("USV", "Choreographer")

        self._init_ui()
        self._load_settings()
        self._init_choreographer()

    # ────────────────── UI 构建 ──────────────────

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # ── LLM 设置区 ──
        settings_group = QGroupBox("🤖 LLM 设置")
        settings_layout = QGridLayout(settings_group)
        settings_layout.setContentsMargins(6, 6, 6, 6)

        settings_layout.addWidget(QLabel("服务地址:"), 0, 0)
        self.url_edit = QLineEdit("http://localhost:11434/v1")
        self.url_edit.setPlaceholderText("http://localhost:11434/v1")
        settings_layout.addWidget(self.url_edit, 0, 1, 1, 2)

        settings_layout.addWidget(QLabel("模型:"), 1, 0)
        self.model_edit = QLineEdit("gemma3:27b")
        settings_layout.addWidget(self.model_edit, 1, 1)

        settings_layout.addWidget(QLabel("API Key:"), 1, 2)
        self.apikey_edit = QLineEdit("ollama")
        self.apikey_edit.setEchoMode(QLineEdit.Password)
        settings_layout.addWidget(self.apikey_edit, 1, 3)

        settings_group.setMaximumHeight(100)
        main_layout.addWidget(settings_group)

        # ── USV 选择区 ──
        usv_layout = QHBoxLayout()
        usv_layout.addWidget(QLabel("参演 USV:"))
        self.usv_edit = QLineEdit("usv_01, usv_02, usv_03")
        self.usv_edit.setPlaceholderText("usv_01, usv_02, usv_03")
        usv_layout.addWidget(self.usv_edit)
        self.sync_usv_btn = QPushButton("同步已连接")
        self.sync_usv_btn.setMaximumWidth(90)
        self.sync_usv_btn.clicked.connect(self._sync_connected_usvs)
        usv_layout.addWidget(self.sync_usv_btn)
        main_layout.addLayout(usv_layout)

        # ── 中间分割区: 输入 + 预览 ──
        splitter = QSplitter(Qt.Vertical)

        # 输入面板
        input_widget = QWidget()
        input_layout = QVBoxLayout(input_widget)
        input_layout.setContentsMargins(0, 0, 0, 0)

        input_header = QHBoxLayout()
        input_header.addWidget(QLabel("🎭 表演描述:"))
        input_header.addStretch()
        # 快捷图形按钮
        for shape_key, shape_info in list(SHAPE_REGISTRY.items())[:6]:
            btn = QPushButton(shape_info["label"])
            btn.setMaximumWidth(60)
            btn.setToolTip(f"快速插入 {shape_info['label']} 描述")
            btn.clicked.connect(partial(self._insert_quick_shape, shape_key))
            input_header.addWidget(btn)
        input_layout.addLayout(input_header)

        self.input_text = QTextEdit()
        self.input_text.setPlaceholderText(
            "用自然语言描述你想要的表演...\n\n"
            "示例:\n"
            "• 3艘USV画一个五角星，半径6米，速度0.5，彩虹灯光\n"
            "• 先排成V字形，然后展开成圆形队列，最后一起画8字\n"
            "• usv_01和usv_02画心形，usv_03在中心旋转3圈"
        )
        self.input_text.setMaximumHeight(120)
        input_layout.addWidget(self.input_text)

        # 按钮栏
        btn_layout = QHBoxLayout()
        self.generate_btn = QPushButton("🚀 AI 生成")
        self.generate_btn.setMinimumHeight(32)
        self.generate_btn.clicked.connect(self._on_generate)
        btn_layout.addWidget(self.generate_btn)

        self.stop_btn = QPushButton("⏹ 停止")
        self.stop_btn.setMaximumWidth(70)
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._on_stop_generate)
        btn_layout.addWidget(self.stop_btn)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 0)  # 不确定进度
        self.progress_bar.setVisible(False)
        self.progress_bar.setMaximumHeight(16)
        btn_layout.addWidget(self.progress_bar)

        input_layout.addLayout(btn_layout)
        splitter.addWidget(input_widget)

        # 预览面板 (带子标签)
        preview_tabs = QTabWidget()
        preview_tabs.setTabPosition(QTabWidget.South)

        # XML 预览
        self.xml_preview = QPlainTextEdit()
        self.xml_preview.setReadOnly(True)
        self.xml_preview.setFont(QFont("Consolas", 9))
        self._xml_highlighter = XmlHighlighter(self.xml_preview.document())
        preview_tabs.addTab(self.xml_preview, "📄 XML")

        # JSON 计划预览
        self.plan_preview = QPlainTextEdit()
        self.plan_preview.setReadOnly(True)
        self.plan_preview.setFont(QFont("Consolas", 9))
        preview_tabs.addTab(self.plan_preview, "📋 计划")

        # 日志
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(500)
        preview_tabs.addTab(self.log_text, "📝 日志")

        splitter.addWidget(preview_tabs)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        main_layout.addWidget(splitter, 1)

        # ── 底部操作栏 ──
        action_layout = QHBoxLayout()

        self.save_btn = QPushButton("💾 保存 XML")
        self.save_btn.clicked.connect(self._on_save_xml)
        self.save_btn.setEnabled(False)
        action_layout.addWidget(self.save_btn)

        self.load_btn = QPushButton("📂 加载到任务")
        self.load_btn.clicked.connect(self._on_load_to_task)
        self.load_btn.setEnabled(False)
        action_layout.addWidget(self.load_btn)

        self.add_to_queue_btn = QPushButton("➕ 加入队列")
        self.add_to_queue_btn.clicked.connect(self._on_add_to_queue)
        self.add_to_queue_btn.setEnabled(False)
        action_layout.addWidget(self.add_to_queue_btn)

        main_layout.addLayout(action_layout)

    # ────────────────── 设置管理 ──────────────────

    def _load_settings(self):
        url = self._settings.value("llm_url", "http://localhost:11434/v1")
        model = self._settings.value("llm_model", "gemma3:27b")
        apikey = self._settings.value("llm_apikey", "ollama")
        usvs = self._settings.value("usv_ids", "usv_01, usv_02, usv_03")
        self.url_edit.setText(url)
        self.model_edit.setText(model)
        self.apikey_edit.setText(apikey)
        self.usv_edit.setText(usvs)

    def _save_settings(self):
        self._settings.setValue("llm_url", self.url_edit.text())
        self._settings.setValue("llm_model", self.model_edit.text())
        self._settings.setValue("llm_apikey", self.apikey_edit.text())
        self._settings.setValue("usv_ids", self.usv_edit.text())

    def _init_choreographer(self):
        self.choreographer = Choreographer(
            llm_base_url=self.url_edit.text().strip(),
            llm_model=self.model_edit.text().strip(),
            llm_api_key=self.apikey_edit.text().strip(),
        )

    # ────────────────── USV 管理 ──────────────────

    def _get_usv_ids(self) -> List[str]:
        """获取当前配置的 USV IDs。"""
        text = self.usv_edit.text().strip()
        return [uid.strip() for uid in text.split(",") if uid.strip()]

    def _sync_connected_usvs(self):
        """从 USVListManager 同步已连接的 USV。"""
        if not self.list_manager:
            self._log("未连接 USVListManager")
            return

        all_usvs = []
        for usv_state in self.list_manager.usv_cluster_list:
            ns = usv_state if isinstance(usv_state, str) else usv_state.get("namespace", "")
            if ns:
                all_usvs.append(ns)
        for usv_state in self.list_manager.usv_departed_list:
            ns = usv_state if isinstance(usv_state, str) else usv_state.get("namespace", "")
            if ns:
                all_usvs.append(ns)

        if all_usvs:
            self.usv_edit.setText(", ".join(sorted(set(all_usvs))))
            self._log(f"已同步 {len(all_usvs)} 艘 USV: {', '.join(all_usvs)}")
        else:
            self._log("未发现已连接的 USV")

    # ────────────────── 快捷图形 ──────────────────

    def _insert_quick_shape(self, shape_key: str):
        """快捷插入图形描述到输入框。"""
        shape_info = SHAPE_REGISTRY[shape_key]
        usv_ids = self._get_usv_ids()
        count = len(usv_ids)
        label = shape_info["label"]
        params = shape_info["params"]

        # 提取核心参数
        radius = params.get("radius", params.get("size", params.get("a", 6.0)))
        text = f"{count}艘USV画{label}，半径{radius}米，速度0.5 m/s，异步导航"
        self.input_text.setPlainText(text)

    # ────────────────── AI 生成 ──────────────────

    def _on_generate(self):
        """AI 生成按钮回调。"""
        user_input = self.input_text.toPlainText().strip()
        if not user_input:
            QMessageBox.warning(self, "提示", "请输入表演描述")
            return

        usv_ids = self._get_usv_ids()
        if not usv_ids:
            QMessageBox.warning(self, "提示", "请输入至少一个 USV ID")
            return

        # 先保存设置并重新初始化
        self._save_settings()
        self._init_choreographer()

        # 切换 UI 状态
        self.generate_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.progress_bar.setVisible(True)
        self.save_btn.setEnabled(False)
        self.load_btn.setEnabled(False)
        self.add_to_queue_btn.setEnabled(False)
        self.xml_preview.clear()
        self.plan_preview.clear()

        self._log(f"开始生成: {user_input[:80]}...")
        self._log(f"USV: {', '.join(usv_ids)}")
        self._log(f"LLM: {self.model_edit.text()} @ {self.url_edit.text()}")

        # 启动工作线程
        self._worker = GenerateWorker(self.choreographer, user_input, usv_ids)
        self._worker.finished.connect(self._on_generate_finished)
        self._worker.error.connect(self._on_generate_error)
        self._worker.plan_ready.connect(self._on_plan_ready)
        self._worker.start()

    def _on_stop_generate(self):
        """停止生成。"""
        if self._worker and self._worker.isRunning():
            self._worker.terminate()
            self._worker.wait(2000)
            self._log("生成已停止")
        self._reset_ui_state()

    def _on_generate_finished(self, xml_str: str):
        """生成完成回调。"""
        self.xml_preview.setPlainText(xml_str)
        self._log(f"生成完成! XML 共 {len(xml_str)} 字符")
        self._reset_ui_state()
        self.save_btn.setEnabled(True)
        self.load_btn.setEnabled(True)
        self.add_to_queue_btn.setEnabled(True)

    def _on_generate_error(self, error_msg: str):
        """生成错误回调。"""
        self._log(f"❌ 生成失败: {error_msg}")
        self._reset_ui_state()
        QMessageBox.critical(self, "生成失败", error_msg.split("\n")[0])

    def _on_plan_ready(self, plan_json: str):
        """中间计划就绪。"""
        self.plan_preview.setPlainText(plan_json)

    def _reset_ui_state(self):
        """重置 UI 为空闲状态。"""
        self.generate_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.progress_bar.setVisible(False)

    # ────────────────── 文件操作 ──────────────────

    def _on_save_xml(self):
        """保存 XML 到文件。"""
        xml_str = self.xml_preview.toPlainText()
        if not xml_str:
            return

        # 从计划标题生成默认文件名
        title = "表演"
        if self.choreographer and self.choreographer.last_plan:
            title = self.choreographer.last_plan.title

        default_path = os.path.join(self.path_dir, f"{title}.xml")

        filepath, _ = QFileDialog.getSaveFileName(
            self, "保存表演路径", default_path,
            "XML 文件 (*.xml);;所有文件 (*)"
        )
        if filepath:
            try:
                save_xml(xml_str, filepath)
                self._log(f"已保存: {filepath}")
                self.xml_generated.emit(filepath)
                self.status_message.emit(f"已保存表演路径: {os.path.basename(filepath)}")
            except OSError as e:
                QMessageBox.critical(self, "保存失败", str(e))

    def _on_load_to_task(self):
        """将生成的 XML 加载到 ClusterTaskManager。"""
        xml_str = self.xml_preview.toPlainText()
        if not xml_str or not self.task_manager:
            return

        # 先保存到临时文件
        title = "表演"
        if self.choreographer and self.choreographer.last_plan:
            title = self.choreographer.last_plan.title

        filepath = os.path.join(self.path_dir, f"_temp_{title}.xml")
        try:
            save_xml(xml_str, filepath)
            # 调用 task_manager 解析文件
            positions = self.task_manager.parse_file(filepath)
            if positions:
                self.task_manager.cluster_position_list = positions
                self._log(f"已加载到集群任务: {len(positions)} 个目标点")
                self.status_message.emit(f"表演路径已加载: {len(positions)} 个目标点")
            else:
                self._log("加载失败: 解析结果为空")
        except Exception as e:
            self._log(f"加载失败: {e}")
            QMessageBox.critical(self, "加载失败", str(e))

    def _on_add_to_queue(self):
        """保存并添加到任务队列。"""
        xml_str = self.xml_preview.toPlainText()
        if not xml_str:
            return

        title = "表演"
        if self.choreographer and self.choreographer.last_plan:
            title = self.choreographer.last_plan.title

        default_path = os.path.join(self.path_dir, f"{title}.xml")
        filepath, _ = QFileDialog.getSaveFileName(
            self, "保存并添加到队列", default_path,
            "XML 文件 (*.xml);;所有文件 (*)"
        )
        if filepath:
            try:
                save_xml(xml_str, filepath)
                self._log(f"已保存并加入队列: {filepath}")
                self.xml_generated.emit(filepath)
            except OSError as e:
                QMessageBox.critical(self, "保存失败", str(e))

    # ────────────────── 日志 ──────────────────

    def _log(self, message: str):
        """向日志面板写入消息。"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.appendPlainText(f"[{timestamp}] {message}")
        logger.info(message)

    # ────────────────── 主题 ──────────────────

    def set_theme(self, theme_name: str):
        """适配主题切换。"""
        pass  # 使用全局样式表，无需额外处理
