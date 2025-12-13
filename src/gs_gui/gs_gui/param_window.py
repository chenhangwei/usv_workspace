"""
飞控参数配置窗口

类似 QGroundControl 的参数管理界面，提供参数读取、编辑、保存功能。

修改记录 (2025-11-05):
- 改为菜单栏形式（替代工具栏按钮）
- 使用串口直接通信（替代 MAVROS）
- 支持手动连接/断开飞控
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QLineEdit, QLabel, QListWidget, QSplitter, QHeaderView,
    QProgressBar, QMessageBox, QAbstractItemView, QFileDialog,
    QMenuBar, QAction, QMenu
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor, QFont, QKeySequence
from typing import Optional, Dict, List
from .param_serial_manager import ParamSerialManager, ParamInfo, ParamType
from .param_connection_dialog import ParamConnectionDialog
from .param_item_delegate import ParamItemDelegate
from .param_metadata import get_param_metadata
from .param_validator import ParamValidator
from .param_import_export import ParamImportExport, ImportResult
from .param_compare import ParamCompare, ParamDiff, DiffType


class ParamWindow(QDialog):
    """
    飞控参数配置窗口
    
    提供类似 QGC 的参数管理界面：
    - 菜单栏：连接、参数操作、工具
    - 左侧：参数分组列表
    - 右侧：参数详情表格
    - 底部：状态栏
    """
    
    # 自定义信号
    param_changed = pyqtSignal(str, float)  # 参数名, 新值
    
    def __init__(self, parent=None):
        """
        初始化参数窗口
        
        Args:
            parent: 父窗口（可选）
        """
        super().__init__(parent)
        
        # 使用串口管理器（不再依赖 MAVROS）
        self.param_manager = ParamSerialManager()
        
        # UI 状态
        self._current_group = "全部"
        self._search_text = ""
        self._is_loading = False
        self._connected = False
        
        # 设置窗口
        self.setWindowTitle("飞控参数配置（串口模式）")
        self.resize(1100, 750)
        
        # 初始化 UI
        self._setup_ui()
        
        # 尝试从缓存加载参数
        QTimer.singleShot(500, self._try_load_from_cache)
        
        # 窗口居中显示
        self._center_on_screen()

    def _setup_ui(self):
        """设置 UI 布局"""
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(0, 0, 0, 0)  # 移除边距，最大化显示空间
     
        
        # ==================== 顶部工具栏 ====================
        toolbar_layout = QHBoxLayout()
        toolbar_layout.setSpacing(10)
        toolbar_layout.setContentsMargins(10, 10, 10, 5)  # 为工具栏添加内边距
        
        # 搜索框
        search_label = QLabel("🚀 搜索:")
        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("输入参数名称...")
        self.search_box.textChanged.connect(self._on_search_changed)
        self.search_box.setMaximumWidth(250)
        
        toolbar_layout.addWidget(search_label)
        toolbar_layout.addWidget(self.search_box)
        toolbar_layout.addStretch()
        
        # 刷新按钮
        self.refresh_button = QPushButton("🔄 刷新")
        self.refresh_button.clicked.connect(self._load_params)
        self.refresh_button.setToolTip("从飞控重新加载参数")
        toolbar_layout.addWidget(self.refresh_button)
        
        # 保存按钮
        self.save_button = QPushButton("💾 保存")
        self.save_button.clicked.connect(self._save_modified_params)
        self.save_button.setEnabled(False)
        self.save_button.setToolTip("保存所有修改到飞控")
        toolbar_layout.addWidget(self.save_button)
        
        # 重置按钮
        self.reset_button = QPushButton("↺ 重置")
        self.reset_button.clicked.connect(self._reset_modified_params)
        self.reset_button.setEnabled(False)
        self.reset_button.setToolTip("撤销所有修改")
        toolbar_layout.addWidget(self.reset_button)
        
        # 恢复默认值按钮
        self.restore_default_button = QPushButton("🔄 恢复默认")
        self.restore_default_button.clicked.connect(self._restore_default_values)
        self.restore_default_button.setToolTip("将选中参数恢复到出厂默认值")
        toolbar_layout.addWidget(self.restore_default_button)
        
        toolbar_layout.addStretch()
        
        # 导入按钮
        self.import_button = QPushButton("📥 导入")
        self.import_button.clicked.connect(self._import_params)
        self.import_button.setToolTip("从文件导入参数（支持 .param 和 .json 格式）")
        toolbar_layout.addWidget(self.import_button)
        
        # 导出按钮
        self.export_button = QPushButton("📤 导出")
        self.export_button.clicked.connect(self._export_params)
        self.export_button.setToolTip("导出参数到文件（支持 .param 和 .json 格式）")
        toolbar_layout.addWidget(self.export_button)
        
        # 对比按钮
        self.compare_button = QPushButton("🚀 对比")
        self.compare_button.clicked.connect(self._show_compare_dialog)
        self.compare_button.setToolTip("对比当前值与默认值，或与其他 USV/文件对比")
        toolbar_layout.addWidget(self.compare_button)
        
        # 清除缓存按钮
        self.clear_cache_button = QPushButton("🗑️ 清除缓存")
        self.clear_cache_button.clicked.connect(self._clear_cache)
        self.clear_cache_button.setToolTip("清除本地参数缓存，下次将从飞控重新加载")
        toolbar_layout.addWidget(self.clear_cache_button)
        
        main_layout.addLayout(toolbar_layout)
        
        # ==================== 中间内容区 ====================
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧：分组列表
        self.group_list = QListWidget()
        self.group_list.setMinimumWidth(150)  # 减小最小宽度
        self.group_list.setMaximumWidth(200)  # 减小最大宽度
        self.group_list.currentTextChanged.connect(self._on_group_changed)
        splitter.addWidget(self.group_list)
        
        # 右侧：参数表格
        self.param_table = QTableWidget()
        self._setup_param_table()
        splitter.addWidget(self.param_table)
        
        splitter.setStretchFactor(0, 0)  # 分组列表不拉伸
        splitter.setStretchFactor(1, 1)  # 参数表格占据所有剩余空间
        
        main_layout.addWidget(splitter)
        
        # ==================== 底部状态栏 ====================
        #状态栏布局高度最小为30ox，以适应较小的屏幕怎么操作
        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(10, 5, 10, 8)  # 减小底部边距
        
        self.status_label = QLabel("准备就绪")
        self.status_label.setStyleSheet("font-size: 14pt;")  # 减小字体
        status_layout.addWidget(self.status_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximumWidth(300)
        self.progress_bar.setMaximumHeight(20)  # 限制进度条高度
        self.progress_bar.setVisible(False)
        status_layout.addWidget(self.progress_bar)
        
        status_layout.addStretch()
        
        # 统计信息
        self.stats_label = QLabel("参数: 0 | 已修改: 0")
        self.stats_label.setStyleSheet("font-size: 14pt;")  # 减小字体
        status_layout.addWidget(self.stats_label)
        
        main_layout.addLayout(status_layout)
        
        # 应用样式
        self._apply_styles()
    
    def _setup_param_table(self):
        """设置参数表格"""
        # 列定义 - 新增"单位"和"默认值"列
        headers = ["参数名称", "当前值", "单位", "默认值", "原始值", "分组", "描述"]
        self.param_table.setColumnCount(len(headers))
        self.param_table.setHorizontalHeaderLabels(headers)
        
        # 设置自定义编辑器委托
        delegate = ParamItemDelegate(self.param_manager, self)
        self.param_table.setItemDelegate(delegate)
        
        # 表格样式
        self.param_table.setAlternatingRowColors(True)
        self.param_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.param_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.param_table.setEditTriggers(QAbstractItemView.DoubleClicked)
        
        # 列宽（增大以适应更大字体）
        header = self.param_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)  # 参数名
        header.setSectionResizeMode(1, QHeaderView.Fixed)             # 当前值
        header.setSectionResizeMode(2, QHeaderView.Fixed)             # 单位
        header.setSectionResizeMode(3, QHeaderView.Fixed)             # 默认值
        header.setSectionResizeMode(4, QHeaderView.Fixed)             # 原始值
        header.setSectionResizeMode(5, QHeaderView.ResizeToContents)  # 分组
        header.setSectionResizeMode(6, QHeaderView.Stretch)           # 描述
        
        self.param_table.setColumnWidth(1, 150)  # 当前值
        self.param_table.setColumnWidth(2, 80)   # 单位
        self.param_table.setColumnWidth(3, 100)  # 默认值
        self.param_table.setColumnWidth(4, 100)  # 原始值
        
        # 连接信号
        self.param_table.itemChanged.connect(self._on_param_value_changed)
    
    def _apply_styles(self):
        """应用样式表"""
        # 设置全局字体大小
        font = QFont()
        font.setPointSize(14)  # 增大全局字体
        self.setFont(font)
        
        self.setStyleSheet("""
            QDialog {
                background-color: #ecf0f1;
            }
            QLineEdit {
                padding: 8px;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                background-color: white;
                color: black;
                font-size: 14pt;
            }
            QPushButton {
                padding: 8px 16px;
                border: none;
                border-radius: 4px;
                background-color: #3498db;
                color: white;
                font-weight: bold;
                font-size: 14pt;
                min-height: 32px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
                color: #ecf0f1;
            }
            QTableWidget {
                background-color: white;
                gridline-color: #ecf0f1;
                border: 1px solid #bdc3c7;
                font-size: 14pt;
                color: black;
            }
            QTableWidget::item {
                padding: 6px;
                min-height: 28px;
                color: black;
            }
            QTableWidget::item:selected {
                background-color: #3498db;
                color: white;
            }
            QListWidget {
                background-color: white;
                border: 1px solid #bdc3c7;
                font-size: 14pt;
                color: black;
            }
            QListWidget::item {
                padding: 8px;
                min-height: 28px;
                color: black;
            }
            QListWidget::item:selected {
                background-color: #3498db;
                color: white;
            }
            QHeaderView::section {
                background-color: #2c3e50;
                color: white;
                padding: 10px 6px;
                border: none;
                font-weight: bold;
                font-size: 14pt;
                min-height: 35px;
            }
            QLabel {
                font-size: 14pt;
                color: black;
            }
            QProgressBar {
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                text-align: center;
                font-size: 14pt;
                font-weight: bold;
                color: black;
            }
            QProgressBar::chunk {
                background-color: #3498db;
            }
        """)
    
    def _try_load_from_cache(self):
        """尝试从缓存加载参数"""
        # 检查缓存是否有效
        cache_info = self.param_manager.get_cache_info()
        
        if cache_info and cache_info['is_valid']:
            # 缓存有效，直接加载
            age_hours = int(cache_info['age_hours'])
            age_minutes = int((cache_info['age_hours'] - age_hours) * 60)
            
            self.status_label.setText(
                f"从缓存加载参数... (缓存时间: {age_hours}小时{age_minutes}分钟前)"
            )
            
            if self.param_manager.load_cache():
                # 刷新 UI
                self._refresh_group_list()
                self._refresh_param_table()
                self._update_stats()
                
                self.status_label.setText(
                    f"已从缓存加载 {len(self.param_manager.get_all_params())} 个参数 "
                    f"(缓存时间: {age_hours}小时{age_minutes}分钟前)"
                )
            else:
                # 缓存加载失败，从飞控加载
                self._load_params()
        else:
            # 缓存不存在或已过期，从飞控加载
            if cache_info:
                self.status_label.setText("缓存已过期，正在从飞控加载...")
            self._load_params()
    
    def _load_params(self):
        """从飞控加载参数"""
        if self._is_loading:
            QMessageBox.warning(self, "警告", "参数加载已在进行中...")
            return
        
        # 检查是否有未保存的修改
        modified_params = self.param_manager.get_modified_params()
        if modified_params:
            reply = QMessageBox.question(
                self,
                "确认",
                "有未保存的修改，重新加载将丢失这些修改。是否继续？",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return
        
        self._is_loading = True
        self.status_label.setText("正在从飞控加载参数...")
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 100)  # 百分比模式
        self.progress_bar.setValue(0)
        self.refresh_button.setEnabled(False)
        
        # 异步加载
        self.param_manager.pull_all_params_async(
            on_progress=self._on_load_progress,
            on_complete=self._on_load_complete
        )
    
    def _on_load_progress(self, current: int, total: int):
        """
        加载进度回调
        
        注意：这个回调可能在后台线程调用，需要使用 QTimer 调度到主线程
        """
        def update_progress():
            if total > 0:
                progress_percent = int((current / total) * 100)
                self.progress_bar.setValue(progress_percent)
                self.status_label.setText(
                    f"正在加载参数... {current}/{total} ({progress_percent}%)"
                )
        
        # 使用 QTimer 调度到主线程
        QTimer.singleShot(0, update_progress)
    
    def _on_load_complete(self, success: bool, message: str):
        """
        加载完成回调
        
        注意：这个回调可能在后台线程调用，需要使用 QTimer 调度到主线程
        """
        QTimer.singleShot(0, lambda: self._update_ui_after_load(success, message))
    
    def _update_ui_after_load(self, success: bool, message: str):
        """在主线程更新 UI"""
        self._is_loading = False
        self.progress_bar.setVisible(False)
        self.progress_bar.setValue(0)
        self.refresh_button.setEnabled(True)
        
        if success:
            # 加载成功
            self.status_label.setText(message)
            self._refresh_group_list()
            self._refresh_param_table()
            self._update_stats()
            
            QMessageBox.information(
                self,
                "成功",
                message,
                QMessageBox.Ok
            )
        else:
            # 加载失败
            self.status_label.setText("参数加载失败")
            
            # 创建更清晰的错误提示
            error_dialog = QMessageBox(self)
            error_dialog.setIcon(QMessageBox.Critical)
            
            # 设置样式表以确保文字清晰可见
            error_dialog.setStyleSheet("""
                QMessageBox {
                    background-color: white;
                }
                QLabel {
                    color: black;
                    font-size: 14pt;
                    background-color: transparent;
                }
                QPushButton {
                    padding: 8px 16px;
                    border: none;
                    border-radius: 4px;
                    background-color: #3498db;
                    color: white;
                    font-weight: bold;
                    font-size: 14pt;
                    min-height: 32px;
                    min-width: 80px;
                }
                QPushButton:hover {
                    background-color: #2980b9;
                }
            """)
            
            # 检查是否是"功能未实现"错误
            if "ParamPull 服务不可用" in message:
                error_dialog.setWindowTitle("无法连接飞控")
                error_dialog.setText("无法连接到飞控参数服务")
                error_dialog.setInformativeText(
                    "请检查以下事项：\n\n"
                    "1. PX4 飞控是否已正确连接\n"
                    "2. uXRCE-DDS Agent 是否正常运行\n"
                    "3. 串口连接是否正常\n"
                    "4. USV 是否在线"
                )
                error_dialog.setDetailedText(
                    f"详细错误信息：\n{message}\n\n"
                    f"命名空间：{self.usv_namespace}\n"
                    f"提示：使用串口直接通信获取参数"
                )
            else:
                error_dialog.setWindowTitle("参数加载失败")
                error_dialog.setText("无法加载飞控参数")
                error_dialog.setInformativeText(
                    "加载参数时发生错误，请重试。\n\n"
                    "如问题持续，请检查飞控连接状态。"
                )
                error_dialog.setDetailedText(f"错误详情：\n{message}")
            
            error_dialog.setStandardButtons(QMessageBox.Ok)
            error_dialog.exec_()
    
    def _refresh_group_list(self):
        """刷新分组列表"""
        self.group_list.clear()
        
        # 添加"全部"选项
        self.group_list.addItem("全部")
        
        # 添加分组
        groups = self.param_manager.get_param_groups()
        for group in groups:
            self.group_list.addItem(group)
        
        # 默认选中"全部"
        self.group_list.setCurrentRow(0)
    
    def _refresh_param_table(self):
        """刷新参数表格"""
        # 暂时断开信号，避免触发修改事件
        self.param_table.itemChanged.disconnect(self._on_param_value_changed)
        
        # 清空表格
        self.param_table.setRowCount(0)
        
        # 获取要显示的参数
        params = self._get_filtered_params()
        
        # 填充表格
        for i, param in enumerate(params):
            self.param_table.insertRow(i)
            
            # 获取元数据
            meta = get_param_metadata(param.name)
            
            # 参数名称（只读）
            name_item = QTableWidgetItem(param.name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
            # 添加工具提示
            tooltip = self._build_param_tooltip(param, meta)
            name_item.setToolTip(tooltip)
            self.param_table.setItem(i, 0, name_item)
            
            # 当前值（可编辑）
            value_item = QTableWidgetItem(f"{param.value:.6g}")
            value_item.setToolTip(tooltip)
            # 如果有枚举值，显示描述
            if meta and meta.values:
                value_desc = ParamValidator.get_value_description(param, param.value)
                if value_desc:
                    value_item.setText(f"{param.value:.6g} ({value_desc})")
            self.param_table.setItem(i, 1, value_item)
            
            # 单位（只读）
            unit_text = meta.unit if (meta and meta.unit) else ""
            unit_item = QTableWidgetItem(unit_text)
            unit_item.setFlags(unit_item.flags() & ~Qt.ItemIsEditable)
            unit_item.setToolTip(tooltip)
            self.param_table.setItem(i, 2, unit_item)
            
            # 默认值（只读）
            default_text = f"{meta.default_value:.6g}" if (meta and meta.default_value is not None) else ""
            default_item = QTableWidgetItem(default_text)
            default_item.setFlags(default_item.flags() & ~Qt.ItemIsEditable)
            default_item.setToolTip(tooltip)
            self.param_table.setItem(i, 3, default_item)
            
            # 原始值（只读）
            orig_item = QTableWidgetItem(f"{param.original_value:.6g}")
            orig_item.setFlags(orig_item.flags() & ~Qt.ItemIsEditable)
            orig_item.setToolTip(tooltip)
            self.param_table.setItem(i, 4, orig_item)
            
            # 分组（只读）
            group_item = QTableWidgetItem(param.group)
            group_item.setFlags(group_item.flags() & ~Qt.ItemIsEditable)
            self.param_table.setItem(i, 5, group_item)
            
            # 描述（只读）
            desc_text = meta.description if (meta and meta.description) else (param.description or "")
            desc_item = QTableWidgetItem(desc_text)
            desc_item.setFlags(desc_item.flags() & ~Qt.ItemIsEditable)
            desc_item.setToolTip(tooltip)
            self.param_table.setItem(i, 6, desc_item)
            
            # 高亮已修改的参数
            if param.is_modified:
                self._highlight_row(i, QColor(255, 255, 200))  # 淡黄色
            
            # 标记需要重启的参数
            if meta and meta.reboot_required:
                # 为需要重启的参数添加特殊标记
                for col in range(self.param_table.columnCount()):
                    item = self.param_table.item(i, col)
                    if item:
                        font = item.font()
                        font.setBold(True)
                        item.setFont(font)
                        item.setForeground(QColor(230, 126, 34))  # 橙色
        
        # 重新连接信号
        self.param_table.itemChanged.connect(self._on_param_value_changed)
    
    def _get_filtered_params(self) -> List[ParamInfo]:
        """获取过滤后的参数列表"""
        # 获取所有参数
        all_params = self.param_manager.get_all_params()
        
        # 按分组过滤
        if self._current_group == "全部":
            params = list(all_params.values())
        else:
            params = self.param_manager.get_params_by_group(self._current_group)
        
        # 按搜索文本过滤
        if self._search_text:
            search_lower = self._search_text.lower()
            params = [
                p for p in params
                if search_lower in p.name.lower() or
                   search_lower in p.description.lower()
            ]
        
        # 排序
        params.sort(key=lambda p: p.name)
        
        return params
    
    def _highlight_row(self, row: int, color: QColor):
        """高亮表格行"""
        for col in range(self.param_table.columnCount()):
            item = self.param_table.item(row, col)
            if item:
                item.setBackground(color)
    
    def _on_group_changed(self, group: str):
        """分组切换事件"""
        if group:
            self._current_group = group
            self._refresh_param_table()
            self._update_stats()
    
    def _on_search_changed(self, text: str):
        """搜索框文本变化事件"""
        self._search_text = text
        self._refresh_param_table()
        self._update_stats()
    
    def _on_param_value_changed(self, item: QTableWidgetItem):
        """参数值修改事件"""
        # 只处理值列的修改
        if item.column() != 1:
            return
        
        row = item.row()
        param_name = self.param_table.item(row, 0).text()
        
        try:
            # 解析新值
            new_value = float(item.text())
            
            # 更新参数管理器中的值
            param = self.param_manager.get_param(param_name)
            if param:
                param.value = new_value
                
                # 高亮已修改的行
                if param.is_modified:
                    self._highlight_row(row, QColor(255, 255, 200))
                else:
                    self._highlight_row(row, QColor(255, 255, 255))
                
                # 更新统计和按钮状态
                self._update_stats()
                self._update_button_states()
                
                # 发射信号
                self.param_changed.emit(param_name, new_value)
        
        except ValueError:
            # 输入无效，恢复原值
            QMessageBox.warning(self, "输入错误", "请输入有效的数字")
            param = self.param_manager.get_param(param_name)
            if param:
                item.setText(f"{param.value:.6g}")
    
    def _save_modified_params(self):
        """保存所有修改的参数"""
        modified = self.param_manager.get_modified_params()
        if not modified:
            QMessageBox.information(self, "提示", "没有需要保存的修改")
            return
        
        reply = QMessageBox.question(
            self,
            "确认保存",
            f"将保存 {len(modified)} 个修改的参数到飞控，是否继续？",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        self.status_label.setText(f"正在保存 {len(modified)} 个参数...")
        self.save_button.setEnabled(False)
        
        # 保存参数
        success = self.param_manager.save_all_modified()
        
        if success:
            self.status_label.setText(f"成功保存 {len(modified)} 个参数")
            QMessageBox.information(self, "成功", "所有参数已保存到飞控")
            self._refresh_param_table()
        else:
            self.status_label.setText("保存失败")
            QMessageBox.critical(self, "错误", "部分参数保存失败，请检查日志")
        
        self._update_button_states()
    
    def _reset_modified_params(self):
        """重置所有修改"""
        modified = self.param_manager.get_modified_params()
        if not modified:
            return
        
        reply = QMessageBox.question(
            self,
            "确认重置",
            f"将撤销 {len(modified)} 个参数的修改，是否继续？",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        self.param_manager.reset_all_modified()
        self._refresh_param_table()
        self._update_stats()
    
    def _clear_cache(self):
        """清除参数缓存"""
        cache_info = self.param_manager.get_cache_info()
        
        if not cache_info:
            QMessageBox.information(self, "提示", "没有缓存文件")
            return
        
        reply = QMessageBox.question(
            self,
            "确认清除缓存",
            f"将清除参数缓存文件。\n\n"
            f"缓存文件: {cache_info['file_path']}\n"
            f"缓存时间: {cache_info['timestamp'].strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"参数数量: {cache_info['param_count']}\n\n"
            f"清除后下次将从飞控重新加载参数，是否继续？",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        if self.param_manager.clear_cache():
            QMessageBox.information(self, "成功", "缓存已清除")
            self.status_label.setText("缓存已清除")
        else:
            QMessageBox.critical(self, "错误", "清除缓存失败")
        self._update_button_states()
        self.status_label.setText("已重置所有修改")
    
    def _update_stats(self):
        """更新统计信息"""
        all_params = self.param_manager.get_all_params()
        modified = self.param_manager.get_modified_params()
        
        self.stats_label.setText(
            f"参数: {len(all_params)} | 已修改: {len(modified)}"
        )
    
    def _update_button_states(self):
        """更新按钮状态"""
        has_modified = bool(self.param_manager.get_modified_params())
        self.save_button.setEnabled(has_modified)
        self.reset_button.setEnabled(has_modified)
    
    def _restore_default_values(self):
        """恢复选中参数到默认值"""
        selected_rows = set(item.row() for item in self.param_table.selectedItems())
        
        if not selected_rows:
            QMessageBox.information(self, "提示", "请先选择要恢复的参数")
            return
        
        # 获取选中的参数
        params_to_restore = []
        for row in selected_rows:
            param_name = self.param_table.item(row, 0).text()
            param = self.param_manager.get_param(param_name)
            meta = get_param_metadata(param_name)
            
            if param and meta and meta.default_value is not None:
                params_to_restore.append((param, meta))
        
        if not params_to_restore:
            QMessageBox.information(self, "提示", "选中的参数没有默认值信息")
            return
        
        # 确认对话框
        reply = QMessageBox.question(
            self,
            "确认恢复默认值",
            f"将 {len(params_to_restore)} 个参数恢复到出厂默认值，是否继续？",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        # 恢复默认值
        for param, meta in params_to_restore:
            param.value = meta.default_value
        
        # 刷新表格
        self._refresh_param_table()
        self._update_stats()
        self._update_button_states()
        
        QMessageBox.information(
            self,
            "成功",
            f"已将 {len(params_to_restore)} 个参数恢复到默认值"
        )
    
    def _build_param_tooltip(self, param: ParamInfo, meta) -> str:
        """构建参数工具提示"""
        lines = []
        
        # 参数名称
        lines.append(f"<b style='font-size: 14pt'>{param.name}</b>")
        lines.append("")
        
        # 显示名称
        if meta and meta.display_name and meta.display_name != param.name:
            lines.append(f"<b>名称：</b>{meta.display_name}")
        
        # 描述
        if meta and meta.description:
            lines.append(f"<b>描述：</b>{meta.description}")
            if meta.user_description:
                lines.append(f"<i>{meta.user_description}</i>")
            lines.append("")
        
        # 当前值和默认值
        lines.append(f"<b>当前值：</b>{param.value:.6g}")
        if meta and meta.default_value is not None:
            lines.append(f"<b>默认值：</b>{meta.default_value:.6g}")
            if abs(param.value - meta.default_value) > 1e-9:
                lines.append(f"<font color='orange'>⚠️ 已偏离默认值</font>")
        if param.is_modified:
            lines.append(f"<font color='#e74c3c'><b>📋 已修改（未保存）</b></font>")
        lines.append("")
        
        # 范围
        if param.min_value is not None or param.max_value is not None:
            min_str = f"{param.min_value:.6g}" if param.min_value is not None else "−∞"
            max_str = f"{param.max_value:.6g}" if param.max_value is not None else "+∞"
            lines.append(f"<b>范围：</b>{min_str} ~ {max_str}")
        
        # 单位
        if meta and meta.unit:
            lines.append(f"<b>单位：</b>{meta.unit}")
        
        # 步进值
        if meta and meta.increment:
            lines.append(f"<b>步进：</b>{meta.increment}")
        
        # 枚举值
        if meta and meta.values:
            lines.append("")
            lines.append(f"<b>枚举值：</b>")
            for value, desc in sorted(meta.values.items())[:5]:  # 只显示前5个
                current_mark = " ← <b>当前</b>" if int(param.value) == value else ""
                lines.append(f"  • {value}: {desc}{current_mark}")
            if len(meta.values) > 5:
                lines.append(f"  ... 共 {len(meta.values)} 个选项")
        
        # 位掩码
        if meta and meta.bitmask:
            lines.append("")
            lines.append(f"<b>位掩码：</b>")
            for bit, desc in sorted(meta.bitmask.items())[:5]:  # 只显示前5个
                lines.append(f"  • Bit {bit}: {desc}")
            if len(meta.bitmask) > 5:
                lines.append(f"  ... 共 {len(meta.bitmask)} 位")
        
        # 重启提示
        if meta and meta.reboot_required:
            lines.append("")
            lines.append("<font color='orange'><b>⚠️ 修改此参数需要重启飞控</b></font>")
        
        # 只读提示
        if meta and meta.read_only:
            lines.append("")
            lines.append("<font color='gray'><b>🔒 此参数为只读</b></font>")
        
        # 警告检查
        warning_level = ParamValidator.get_warning_level(param, param.value)
        if warning_level >= 2:
            warning_msg = ParamValidator.get_warning_message(param, param.value)
            lines.append("")
            lines.append(f"<font color='#e74c3c'><b>{warning_msg}</b></font>")
        
        return "<br>".join(lines)
    
    # ==================== 导入/导出功能 ====================
    
    def _import_params(self):
        """从文件导入参数"""
        # 选择文件
        file_path, file_type = QFileDialog.getOpenFileName(
            self,
            "导入参数",
            "",
            "参数文件 (*.param *.json);;QGC 格式 (*.param);;JSON 格式 (*.json);;所有文件 (*)"
        )
        
        if not file_path:
            return
        
        try:
            # 获取文件信息
            file_info = ParamImportExport.get_file_info(file_path)
            if not file_info:
                QMessageBox.warning(self, "导入失败", "无法识别的文件格式")
                return
            
            # 显示文件信息
            info_text = (
                f"文件格式：{file_info['format']}\n"
                f"机体类型：{file_info['vehicle_type']}\n"
                f"固件版本：{file_info['firmware_version']}\n"
                f"参数数量：{file_info['param_count']}\n"
                f"导出时间：{file_info['exported_at']}\n\n"
                f"确定要导入这些参数吗？\n"
                f"⚠️ 导入将覆盖当前参数值（但不会立即保存到飞控）"
            )
            
            reply = QMessageBox.question(
                self,
                "确认导入",
                info_text,
                QMessageBox.Yes | QMessageBox.No
            )
            
            if reply != QMessageBox.Yes:
                return
            
            # 执行导入
            self.status_label.setText("正在导入参数...")
            
            if file_path.endswith('.param'):
                result = ParamImportExport.import_from_param_file(
                    file_path,
                    self.param_manager.params,
                    validate=True
                )
            elif file_path.endswith('.json'):
                result = ParamImportExport.import_from_json_file(
                    file_path,
                    self.param_manager.params,
                    validate=True
                )
            else:
                QMessageBox.warning(self, "导入失败", "不支持的文件格式")
                return
            
            # 显示导入结果
            if result.success:
                # 构建结果消息
                msg_lines = [
                    f"✅ 导入成功！",
                    f"",
                    f"导入参数：{result.imported_count} 个",
                    f"跳过参数：{result.skipped_count} 个",
                    f"错误参数：{result.error_count} 个",
                ]
                
                # 显示冲突
                if result.conflicts:
                    msg_lines.append(f"\n⚠️ 发现 {len(result.conflicts)} 个参数值冲突：")
                    for param_name, file_value, current_value in result.conflicts[:10]:
                        msg_lines.append(
                            f"  • {param_name}: {current_value:.6g} → {file_value:.6g}"
                        )
                    if len(result.conflicts) > 10:
                        msg_lines.append(f"  ... 共 {len(result.conflicts)} 个冲突")
                
                # 显示详细消息
                if result.messages:
                    msg_lines.append(f"\n详细信息：")
                    for msg in result.messages[:5]:
                        msg_lines.append(f"  • {msg}")
                    if len(result.messages) > 5:
                        msg_lines.append(f"  ... 共 {len(result.messages)} 条消息")
                
                msg_lines.append(f"\n[*] 请点击\"保存\"按钮将修改写入飞控")
                
                QMessageBox.information(self, "导入完成", "\n".join(msg_lines))
                
                # 刷新界面
                self._refresh_param_table()
                self._update_button_states()
                self.status_label.setText(f"导入完成：{result.imported_count} 个参数")
                
            else:
                error_msg = "\n".join(result.messages)
                QMessageBox.critical(self, "导入失败", f"导入失败：\n{error_msg}")
                self.status_label.setText("导入失败")
                
        except Exception as e:
            QMessageBox.critical(self, "导入错误", f"导入过程中发生错误：\n{str(e)}")
            self.status_label.setText("导入错误")
    
    def _export_params(self):
        """导出参数到文件"""
        # 选择导出格式和文件
        file_path, file_type = QFileDialog.getSaveFileName(
            self,
            "导出参数",
            f"{self.usv_namespace}_params.param",
            "QGC 格式 (*.param);;JSON 格式 (*.json);;所有文件 (*)"
        )
        
        if not file_path:
            return
        
        try:
            # 确定文件格式
            if "*.param" in file_type or file_path.endswith('.param'):
                export_format = 'param'
                if not file_path.endswith('.param'):
                    file_path += '.param'
            elif "*.json" in file_type or file_path.endswith('.json'):
                export_format = 'json'
                if not file_path.endswith('.json'):
                    file_path += '.json'
            else:
                QMessageBox.warning(self, "导出失败", "请选择有效的文件格式")
                return
            
            # 询问是否包含元数据（仅 JSON 格式）
            include_metadata = True
            if export_format == 'json':
                reply = QMessageBox.question(
                    self,
                    "导出选项",
                    "是否包含完整元数据（描述、单位、范围等）？\n\n"
                    "• 是：导出完整信息（文件较大，便于查看）\n"
                    "• 否：仅导出参数值（文件较小）",
                    QMessageBox.Yes | QMessageBox.No
                )
                include_metadata = (reply == QMessageBox.Yes)
            
            # 执行导出
            self.status_label.setText("正在导出参数...")
            
            if export_format == 'param':
                success = ParamImportExport.export_to_param_file(
                    self.param_manager.params,
                    file_path,
                    vehicle_type="USV",
                    firmware_version="ArduPilot"
                )
            else:  # json
                success = ParamImportExport.export_to_json_file(
                    self.param_manager.params,
                    file_path,
                    include_metadata=include_metadata,
                    vehicle_type="USV",
                    firmware_version="ArduPilot"
                )
            
            if success:
                param_count = len(self.param_manager.params)
                QMessageBox.information(
                    self,
                    "导出成功",
                    f"✅ 成功导出 {param_count} 个参数到：\n{file_path}\n\n"
                    f"格式：{export_format.upper()}\n"
                    f"{'包含元数据' if include_metadata and export_format == 'json' else '仅参数值'}"
                )
                self.status_label.setText(f"导出完成：{param_count} 个参数")
            else:
                QMessageBox.critical(self, "导出失败", "导出参数时发生错误")
                self.status_label.setText("导出失败")
                
        except Exception as e:
            QMessageBox.critical(self, "导出错误", f"导出过程中发生错误：\n{str(e)}")
            self.status_label.setText("导出错误")
    
    # ==================== 对比功能 ====================
    
    def _show_compare_dialog(self):
        """显示参数对比对话框"""
        # 对比默认值
        diffs = ParamCompare.compare_with_default(self.param_manager.params)
        
        # 过滤出不同的参数
        different_diffs = ParamCompare.filter_diffs(
            diffs,
            show_same=False,
            show_different=True,
            show_missing=False
        )
        
        # 统计
        stats = ParamCompare.get_statistics(diffs)
        
        # 构建消息
        msg_lines = [
            "📋 参数对比结果（当前值 vs 默认值）\n",
            f"总参数：{stats['total']}",
            f"相同：{stats['same']} ✅",
            f"不同：{stats['different']} ⚠️",
            ""
        ]
        
        if different_diffs:
            msg_lines.append(f"差异参数（前 10 个）：\n")
            top_diffs = ParamCompare.get_top_diffs(different_diffs, top_n=10, by="percent")
            for diff in top_diffs:
                percent_str = f"{diff.diff_percent:.1f}%" if diff.diff_percent else "N/A"
                msg_lines.append(
                    f"• {diff.param_name}: {diff.left_value:.6g} → {diff.right_value:.6g} "
                    f"({percent_str})"
                )
        else:
            msg_lines.append("✅ 所有参数均为默认值")
        
        QMessageBox.information(self, "参数对比", "\n".join(msg_lines))




    def _center_on_screen(self):
        """将窗口居中显示在屏幕上"""
        from PyQt5.QtWidgets import QApplication
        screen = QApplication.desktop().screenGeometry()
        size = self.geometry()
        self.move(
            (screen.width() - size.width()) // 2,
            (screen.height() - size.height()) // 2
        )
