"""
飞控参数配置窗口

类似 QGroundControl 的参数管理界面，提供参数读取、编辑、保存功能。
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QLineEdit, QLabel, QListWidget, QSplitter, QHeaderView,
    QProgressBar, QMessageBox, QAbstractItemView, QFileDialog
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor, QFont
from typing import Optional, Dict, List
from .param_manager import ParamManager, ParamManagerAsync, ParamInfo


class ParamWindow(QDialog):
    """
    飞控参数配置窗口
    
    提供类似 QGC 的参数管理界面：
    - 左侧：参数分组列表
    - 右侧：参数详情表格
    - 顶部：搜索、刷新、保存按钮
    - 底部：状态栏
    """
    
    # 自定义信号
    param_changed = pyqtSignal(str, float)  # 参数名, 新值
    
    def __init__(self, usv_namespace: str, param_manager: ParamManagerAsync, parent=None):
        super().__init__(parent)
        
        self.usv_namespace = usv_namespace
        self.param_manager = param_manager
        
        # UI 状态
        self._current_group = "全部"
        self._search_text = ""
        self._is_loading = False
        
        # 设置窗口
        self.setWindowTitle(f"{usv_namespace} - 飞控参数配置")
        self.resize(1100, 700)  # 增大窗口尺寸以适应更大字体
        
        # 初始化 UI
        self._setup_ui()
        
        # 尝试从缓存加载参数
        QTimer.singleShot(500, self._try_load_from_cache)
    
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
        search_label = QLabel("🔍 搜索:")
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
        
        toolbar_layout.addStretch()
        
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
        self.status_label.setStyleSheet("font-size: 10pt;")  # 减小字体
        status_layout.addWidget(self.status_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximumWidth(300)
        self.progress_bar.setMaximumHeight(20)  # 限制进度条高度
        self.progress_bar.setVisible(False)
        status_layout.addWidget(self.progress_bar)
        
        status_layout.addStretch()
        
        # 统计信息
        self.stats_label = QLabel("参数: 0 | 已修改: 0")
        self.stats_label.setStyleSheet("font-size: 10pt;")  # 减小字体
        status_layout.addWidget(self.stats_label)
        
        main_layout.addLayout(status_layout)
        
        # 应用样式
        self._apply_styles()
    
    def _setup_param_table(self):
        """设置参数表格"""
        # 列定义
        headers = ["参数名称", "当前值", "原始值", "分组", "描述"]
        self.param_table.setColumnCount(len(headers))
        self.param_table.setHorizontalHeaderLabels(headers)
        
        # 表格样式
        self.param_table.setAlternatingRowColors(True)
        self.param_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.param_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.param_table.setEditTriggers(QAbstractItemView.DoubleClicked)
        
        # 列宽（增大以适应更大字体）
        header = self.param_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)  # 参数名
        header.setSectionResizeMode(1, QHeaderView.Fixed)             # 当前值
        header.setSectionResizeMode(2, QHeaderView.Fixed)             # 原始值
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)  # 分组
        header.setSectionResizeMode(4, QHeaderView.Stretch)           # 描述
        
        self.param_table.setColumnWidth(1, 150)  # 增大列宽
        self.param_table.setColumnWidth(2, 150)  # 增大列宽
        
        # 连接信号
        self.param_table.itemChanged.connect(self._on_param_value_changed)
    
    def _apply_styles(self):
        """应用样式表"""
        # 设置全局字体大小
        font = QFont()
        font.setPointSize(10)  # 增大全局字体
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
                font-size: 11pt;
            }
            QPushButton {
                padding: 8px 16px;
                border: none;
                border-radius: 4px;
                background-color: #3498db;
                color: white;
                font-weight: bold;
                font-size: 11pt;
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
                font-size: 11pt;
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
                font-size: 11pt;
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
                font-size: 12pt;
                min-height: 35px;
            }
            QLabel {
                font-size: 11pt;
                color: black;
            }
            QProgressBar {
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                text-align: center;
                font-size: 10pt;
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
                    font-size: 11pt;
                    background-color: transparent;
                }
                QPushButton {
                    padding: 8px 16px;
                    border: none;
                    border-radius: 4px;
                    background-color: #3498db;
                    color: white;
                    font-weight: bold;
                    font-size: 11pt;
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
                    "1. MAVROS 节点是否正常运行\n"
                    "2. MAVROS param 插件是否已启用\n"
                    "3. 飞控是否已正确连接\n"
                    "4. USV 是否在线"
                )
                error_dialog.setDetailedText(
                    f"详细错误信息：\n{message}\n\n"
                    f"命名空间：{self.usv_namespace}\n"
                    f"节点信息：检查 MAVROS 插件配置"
                )
            else:
                error_dialog.setWindowTitle("参数加载失败")
                error_dialog.setText("无法加载飞控参数")
                error_dialog.setInformativeText(
                    "加载参数时发生错误，请重试。\n\n"
                    "如问题持续，请检查 MAVROS 连接状态。"
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
            
            # 参数名称（只读）
            name_item = QTableWidgetItem(param.name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
            self.param_table.setItem(i, 0, name_item)
            
            # 当前值（可编辑）
            value_item = QTableWidgetItem(f"{param.value:.6g}")
            self.param_table.setItem(i, 1, value_item)
            
            # 原始值（只读）
            orig_item = QTableWidgetItem(f"{param.original_value:.6g}")
            orig_item.setFlags(orig_item.flags() & ~Qt.ItemIsEditable)
            self.param_table.setItem(i, 2, orig_item)
            
            # 分组（只读）
            group_item = QTableWidgetItem(param.group)
            group_item.setFlags(group_item.flags() & ~Qt.ItemIsEditable)
            self.param_table.setItem(i, 3, group_item)
            
            # 描述（只读）
            desc_item = QTableWidgetItem(param.description or "")
            desc_item.setFlags(desc_item.flags() & ~Qt.ItemIsEditable)
            self.param_table.setItem(i, 4, desc_item)
            
            # 高亮已修改的参数
            if param.is_modified:
                self._highlight_row(i, QColor(255, 255, 200))  # 淡黄色
        
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
