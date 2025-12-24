"""
基于串口的飞控参数配置窗口

直接通过串口与飞控通信，不依赖 MAVROS。
使用菜单栏替代工具栏按钮。

创建日期: 2025-11-05
"""

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QLineEdit, QLabel, QListWidget, QSplitter, QHeaderView,
    QProgressBar, QMessageBox, QAbstractItemView, QFileDialog,
    QMenuBar, QAction, QMenu, QApplication
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QThread, pyqtSlot
from PyQt5.QtGui import QColor, QFont, QKeySequence, QIcon
from typing import Optional, Dict, List
import threading

from .param_serial_manager import ParamSerialManager, ParamInfo
from .param_connection_dialog import ParamConnectionDialog
from .param_metadata import get_param_metadata, load_all_metadata


class ParamLoadThread(QThread):
    """参数加载线程"""
    progress = pyqtSignal(int, int, str)  # current, total, param_name
    finished = pyqtSignal(dict)  # params dict
    error = pyqtSignal(str)  # error message
    
    def __init__(self, param_manager: ParamSerialManager):
        super().__init__()
        self.param_manager = param_manager
    
    def run(self):
        try:
            params = self.param_manager.fetch_all_params(
                progress_callback=lambda cur, total, name: self.progress.emit(cur, total, name)
            )
            self.finished.emit(params)
        except Exception as e:
            self.error.emit(str(e))


class ParamWindowSerial(QMainWindow):
    """
    基于串口的飞控参数配置窗口
    
    功能：
    - 串口连接管理（手动连接/断开）
    - 参数读取、编辑、保存
    - 参数分组显示和搜索
    - 菜单栏操作（替代工具栏按钮）
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 加载参数元数据（描述、单位等）
        load_all_metadata()
        
        # 参数管理器
        self.param_manager = ParamSerialManager()
        self.params: Dict[str, ParamInfo] = {}
        
        # UI 状态
        self._current_group = "全部"
        self._search_text = ""
        self._connected = False
        
        # 设置窗口
        self.setWindowTitle("飞控参数配置 - 串口模式")
        self.resize(1200, 750)
        
        # 初始化 UI
        self._setup_ui()
        self._update_connection_status()
        
        # 窗口居中显示
        self._center_on_screen()
    
    def _setup_ui(self):
        """设置 UI"""
        # 创建中心 widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # ==================== 菜单栏 ====================
        menubar = self.menuBar()
        
        # 【连接】菜单
        connect_menu = menubar.addMenu("连接(&C)")
        
        self.connect_action = QAction("± 连接飞控...", self)
        self.connect_action.setShortcut(QKeySequence("Ctrl+O"))
        self.connect_action.triggered.connect(self._connect_to_fcu)
        connect_menu.addAction(self.connect_action)
        
        self.disconnect_action = QAction("⛔ 断开连接", self)
        self.disconnect_action.setEnabled(False)
        self.disconnect_action.triggered.connect(self._disconnect_from_fcu)
        connect_menu.addAction(self.disconnect_action)
        
        connect_menu.addSeparator()
        
        close_action = QAction("关闭", self)
        close_action.setShortcut(QKeySequence("Ctrl+W"))
        close_action.triggered.connect(self.close)
        connect_menu.addAction(close_action)
        
        # 【参数】菜单
        param_menu = menubar.addMenu("参数(&P)")
        
        self.load_action = QAction("🔄 刷新参数", self)
        self.load_action.setShortcut(QKeySequence("F5"))
        self.load_action.setEnabled(False)
        self.load_action.triggered.connect(self._load_params)
        param_menu.addAction(self.load_action)
        
        self.save_action = QAction("💾 保存修改", self)
        self.save_action.setShortcut(QKeySequence("Ctrl+S"))
        self.save_action.setEnabled(False)
        self.save_action.triggered.connect(self._save_modified_params)
        param_menu.addAction(self.save_action)
        
        self.reset_action = QAction("↺ 撤销修改", self)
        self.reset_action.setShortcut(QKeySequence("Ctrl+Z"))
        self.reset_action.setEnabled(False)
        self.reset_action.triggered.connect(self._reset_modified_params)
        param_menu.addAction(self.reset_action)
        
        param_menu.addSeparator()
        
        self.import_action = QAction("📥 导入参数...", self)
        self.import_action.setEnabled(False)
        self.import_action.triggered.connect(self._import_params)
        param_menu.addAction(self.import_action)
        
        self.export_action = QAction("📤 导出参数...", self)
        self.export_action.setEnabled(False)
        self.export_action.triggered.connect(self._export_params)
        param_menu.addAction(self.export_action)
        
        # 【工具】菜单
        tools_menu = menubar.addMenu("工具(&T)")
        
        self.reboot_action = QAction("🔄 重启飞控", self)
        self.reboot_action.setEnabled(False)
        self.reboot_action.triggered.connect(self._reboot_autopilot)
        tools_menu.addAction(self.reboot_action)
        
        tools_menu.addSeparator()
        
        search_action = QAction("🚀 查找参数...", self)
        search_action.setShortcut(QKeySequence("Ctrl+F"))
        search_action.triggered.connect(lambda: self.search_box.setFocus())
        tools_menu.addAction(search_action)
        
        # QMainWindow 的菜单栏已自动设置，无需手动添加到布局
        
        # ==================== 搜索栏 ====================
        search_layout = QHBoxLayout()
        search_layout.setContentsMargins(10, 5, 10, 5)
        
        search_label = QLabel("🚀 搜索:")
        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("输入参数名称...")
        self.search_box.textChanged.connect(self._on_search_changed)
        self.search_box.setMaximumWidth(300)
        
        search_layout.addWidget(search_label)
        search_layout.addWidget(self.search_box)
        search_layout.addStretch()
        
        main_layout.addLayout(search_layout)
        
        # ==================== 中间内容区 ====================
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # 左侧：分组列表
        self.group_list = QListWidget()
        self.group_list.setMinimumWidth(150)
        self.group_list.setMaximumWidth(200)
        self.group_list.currentTextChanged.connect(self._on_group_changed)
        splitter.addWidget(self.group_list)
        
        # 右侧：参数表格
        self.param_table = QTableWidget()
        self._setup_param_table()
        splitter.addWidget(self.param_table)
        
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        
        main_layout.addWidget(splitter)
        
        # ==================== 底部状态栏 ====================
        # 使用一个固定高度的 widget 包装状态布局，避免过高
        status_widget = QWidget()
        status_layout = QHBoxLayout(status_widget)
        status_layout.setContentsMargins(8, 2, 8, 2)  # 更小的垂直边距
        status_layout.setSpacing(8)

        self.status_label = QLabel("❌ 未连接")
        # 使用较小字体以避免过高的行高
        small_font = QFont()
        small_font.setPointSize(13)
        small_font.setBold(True)
        self.status_label.setFont(small_font)
        self.status_label.setFixedHeight(18)

        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximumWidth(300)
        self.progress_bar.setMaximumHeight(14)  # 限制进度条高度
        self.progress_bar.setMinimumHeight(14)
        self.progress_bar.setVisible(False)
        self.progress_bar.setTextVisible(True)  # 显示进度文本

        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        status_layout.addWidget(self.progress_bar)

        # 将 status_widget 设置为固定高度，使状态栏不再过高
        status_widget.setFixedHeight(24)
        main_layout.addWidget(status_widget)
    
    def _setup_param_table(self):
        """设置参数表格"""
        headers = ["参数名", "当前值", "单位", "默认值", "分组", "描述"]
        self.param_table.setColumnCount(len(headers))
        self.param_table.setHorizontalHeaderLabels(headers)
        
        # 表格样式
        self.param_table.setAlternatingRowColors(True)
        self.param_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.param_table.setEditTriggers(QAbstractItemView.EditTrigger.DoubleClicked)
        
        # 列宽
        header = self.param_table.horizontalHeader()
        if header:
            header.setStretchLastSection(True)
            self.param_table.setColumnWidth(0, 200)  # 参数名
            self.param_table.setColumnWidth(1, 120)  # 当前值
            self.param_table.setColumnWidth(2, 80)   # 单位
            self.param_table.setColumnWidth(3, 120)  # 默认值
            self.param_table.setColumnWidth(4, 100)  # 分组
        
        # 单元格修改监听
        self.param_table.itemChanged.connect(self._on_cell_changed)
    
    def _connect_to_fcu(self):
        """连接到飞控"""
        from PyQt5.QtWidgets import QDialog
        
        # 显示连接对话框
        dialog = ParamConnectionDialog(self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        
        conn_params = dialog.get_connection_params()
        
        # 显示进度
        self.status_label.setText("⏳ 正在连接...")
        QApplication.processEvents()
        
        # 尝试连接
        success = self.param_manager.connect(
            port=conn_params['port'],
            baudrate=conn_params['baudrate'],
            target_system=conn_params['system_id'],
            target_component=conn_params['component_id']
        )
        
        if success:
            self._connected = True
            self._update_connection_status()
            QMessageBox.information(
                self, "连接成功",
                f"已连接到飞控\n"
                f"串口: {conn_params['port']}\n"
                f"波特率: {conn_params['baudrate']}\n"
                f"系统 ID: {conn_params['system_id']}"
            )
            
            # 自动加载参数
            self._load_params()
        else:
            self._connected = False
            self._update_connection_status()
            QMessageBox.critical(
                self, "连接失败",
                f"无法连接到飞控\n\n"
                f"请检查：\n"
                f"• 串口设备是否正确\n"
                f"• 飞控是否上电\n"
                f"• 波特率是否匹配\n"
                f"• 串口权限（sudo usermod -a -G dialout $USER）"
            )
    
    def _disconnect_from_fcu(self):
        """断开飞控连接"""
        # 检查是否有未保存的修改
        modified = [p for p in self.params.values() if p.is_modified]
        if modified:
            reply = QMessageBox.question(
                self, "确认断开",
                f"有 {len(modified)} 个参数未保存，确定要断开连接吗？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
        
        self.param_manager.disconnect()
        self._connected = False
        self._update_connection_status()
        
        self.status_label.setText("❌ 已断开连接")
    
    def _update_connection_status(self):
        """更新连接状态"""
        self.connect_action.setEnabled(not self._connected)
        self.disconnect_action.setEnabled(self._connected)
        self.load_action.setEnabled(self._connected)
        self.save_action.setEnabled(self._connected)
        self.reset_action.setEnabled(self._connected)
        self.import_action.setEnabled(self._connected)
        self.export_action.setEnabled(self._connected)
        self.reboot_action.setEnabled(self._connected)
        
        if self._connected:
            self.status_label.setText("✅ 已连接")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.status_label.setText("❌ 未连接")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
    
    def _load_params(self):
        """加载参数"""
        if not self._connected:
            QMessageBox.warning(self, "未连接", "请先连接到飞控")
            return
        # 禁用相关动作，避免重复触发
        self.load_action.setEnabled(False)
        self.save_action.setEnabled(False)
        self.reset_action.setEnabled(False)

        # 显示进度条（初始为 0-100 百分比模式）
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("正在加载参数... %p%")
        self.status_label.setText("⏳ 正在加载参数...")

        # 立即刷新 UI，确保界面响应
        QApplication.processEvents()

        # 启动加载线程（后台执行）
        self.load_thread = ParamLoadThread(self.param_manager)
        self.load_thread.progress.connect(self._on_load_progress)
        self.load_thread.finished.connect(self._on_load_finished)
        self.load_thread.error.connect(self._on_load_error)
        self.load_thread.start()
    
    @pyqtSlot(int, int, str)
    def _on_load_progress(self, current: int, total: int, param_name: str):
        """加载进度回调"""
        if total > 0:
            self.progress_bar.setRange(0, total)
            self.progress_bar.setValue(current)
            percent = int((current / total) * 100)
            self.progress_bar.setFormat(f"{current}/{total} ({percent}%)")
            self.status_label.setText(f"⏳ 加载中: {param_name}")
        else:
            # 如果还不知道总数，显示当前数量
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(0)
            self.progress_bar.setFormat(f"已加载 {current} 个参数...")
            self.status_label.setText(f"⏳ 加载中: {param_name}")
    
    @pyqtSlot(dict)
    def _on_load_finished(self, params: Dict[str, ParamInfo]):
        """加载完成回调"""
        self.params = params
        self.progress_bar.setVisible(False)
        # 恢复动作
        self.load_action.setEnabled(True)
        self.save_action.setEnabled(True)
        self.reset_action.setEnabled(True)
        
        if params:
            self.status_label.setText(f"✅ 已加载 {len(params)} 个参数")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.status_label.setText("⚠️ 未加载到参数")
            self.status_label.setStyleSheet("color: orange; font-weight: bold;")
        
        # 更新 UI
        self._update_groups()
        self._display_params()
    
    @pyqtSlot(str)
    def _on_load_error(self, error_msg: str):
        """加载错误回调"""
        self.progress_bar.setVisible(False)
        # 恢复动作
        self.load_action.setEnabled(True)
        self.save_action.setEnabled(True)
        self.reset_action.setEnabled(True)
        self.status_label.setText("❌ 加载失败")
        QMessageBox.critical(self, "加载失败", f"参数加载失败：\n{error_msg}")
    
    def _update_groups(self):
        """更新分组列表"""
        self.group_list.clear()
        
        # 添加"全部"
        self.group_list.addItem("全部")
        
        # 提取所有分组
        groups = set()
        for param in self.params.values():
            groups.add(param.group)
        
        # 添加到列表
        for group in sorted(groups):
            self.group_list.addItem(group)
        
        # 默认选中"全部"
        self.group_list.setCurrentRow(0)
    
    def _display_params(self):
        """显示参数"""
        # 过滤参数
        filtered_params = []
        for param in self.params.values():
            # 分组过滤
            if self._current_group != "全部" and param.group != self._current_group:
                continue
            
            # 搜索过滤
            if self._search_text and self._search_text.lower() not in param.name.lower():
                continue
            
            filtered_params.append(param)
        
        # 清空表格
        self.param_table.setRowCount(0)
        self.param_table.blockSignals(True)  # 阻止信号
        
        # 填充表格
        for row, param in enumerate(sorted(filtered_params, key=lambda p: p.name)):
            self.param_table.insertRow(row)
            
            # 参数名（不可编辑）
            name_item = QTableWidgetItem(param.name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            if param.is_modified:
                name_item.setForeground(QColor("#FF6B00"))  # 橙色
                name_item.setFont(QFont("", -1, QFont.Weight.Bold))
            self.param_table.setItem(row, 0, name_item)
            
            # 当前值（可编辑）
            value_item = QTableWidgetItem(str(param.value))
            if param.is_modified:
                value_item.setBackground(QColor("#FFF3CD"))  # 淡黄色背景
            self.param_table.setItem(row, 1, value_item)
            
            # 单位（不可编辑）
            unit_item = QTableWidgetItem(param.unit)
            unit_item.setFlags(unit_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.param_table.setItem(row, 2, unit_item)
            
            # 默认值（不可编辑）
            metadata = get_param_metadata(param.name)
            if metadata and metadata.default_value is not None:
                default_val = str(metadata.default_value)
            else:
                default_val = '-'
            default_item = QTableWidgetItem(default_val)
            default_item.setFlags(default_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.param_table.setItem(row, 3, default_item)
            
            # 分组（不可编辑）
            group_item = QTableWidgetItem(param.group)
            group_item.setFlags(group_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.param_table.setItem(row, 4, group_item)
            
            # 描述（不可编辑）
            desc_item = QTableWidgetItem(param.description)
            desc_item.setFlags(desc_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.param_table.setItem(row, 5, desc_item)
        
        self.param_table.blockSignals(False)  # 恢复信号
    
    def _on_group_changed(self, group_name: str):
        """分组切换"""
        self._current_group = group_name
        self._display_params()
    
    def _on_search_changed(self, text: str):
        """搜索文本变化"""
        self._search_text = text
        self._display_params()
    
    def _on_cell_changed(self, item: QTableWidgetItem):
        """单元格修改"""
        if item.column() != 1:  # 只处理"当前值"列
            return
        
        row = item.row()
        param_name_item = self.param_table.item(row, 0)
        if not param_name_item:
            return
        
        param_name = param_name_item.text()
        if param_name not in self.params:
            return
        
        param = self.params[param_name]
        
        try:
            new_value = float(item.text())
            param.value = new_value
            
            # 刷新显示
            self._display_params()
            
        except ValueError:
            QMessageBox.warning(self, "无效值", "请输入有效的数值")
            item.setText(str(param.value))
    
    def _save_modified_params(self):
        """保存修改的参数"""
        modified = [p for p in self.params.values() if p.is_modified]
        
        if not modified:
            QMessageBox.information(self, "无修改", "没有需要保存的参数")
            return
        
        reply = QMessageBox.question(
            self, "确认保存",
            f"确定要保存 {len(modified)} 个修改的参数到飞控吗？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        # 显示进度
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, len(modified))
        
        success_count = 0
        for i, param in enumerate(modified, 1):
            self.progress_bar.setValue(i)
            self.status_label.setText(f"⏳ 正在保存 ({i}/{len(modified)}): {param.name}")
            QApplication.processEvents()
            
            if self.param_manager.set_param(param.name, param.value):
                success_count += 1
        
        self.progress_bar.setVisible(False)
        
        if success_count == len(modified):
            self.status_label.setText(f"✅ 已保存 {success_count} 个参数")
            QMessageBox.information(self, "保存成功", f"成功保存 {success_count} 个参数")
        else:
            failed = len(modified) - success_count
            self.status_label.setText(f"⚠️ 部分保存失败 ({failed} 个)")
            QMessageBox.warning(
                self, "部分失败",
                f"成功: {success_count}\n失败: {failed}"
            )
        
        # 刷新显示
        self._display_params()
    
    def _reset_modified_params(self):
        """撤销修改"""
        modified = [p for p in self.params.values() if p.is_modified]
        
        if not modified:
            QMessageBox.information(self, "无修改", "没有需要撤销的参数")
            return
        
        reply = QMessageBox.question(
            self, "确认撤销",
            f"确定要撤销 {len(modified)} 个参数的修改吗？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        for param in modified:
            param.reset()
        
        self._display_params()
        self.status_label.setText(f"↺ 已撤销 {len(modified)} 个修改")
    
    def _import_params(self):
        """导入参数"""
        QMessageBox.information(self, "功能开发中", "参数导入功能正在开发中...")
    
    def _export_params(self):
        """导出参数"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, "导出参数",
            f"params_{len(self.params)}.param",
            "参数文件 (*.param);;JSON 文件 (*.json)"
        )
        
        if not file_path:
            return
        
        try:
            with open(file_path, 'w') as f:
                f.write(f"# 参数导出\n# 参数数量: {len(self.params)}\n\n")
                for param in sorted(self.params.values(), key=lambda p: p.name):
                    f.write(f"{param.name}\t{param.value}\n")
            
            QMessageBox.information(self, "导出成功", f"已导出到:\n{file_path}")
        except Exception as e:
            QMessageBox.critical(self, "导出失败", f"导出参数失败:\n{e}")
    
    def _reboot_autopilot(self):
        """重启飞控"""
        reply = QMessageBox.question(
            self, "确认重启",
            "确定要重启飞控吗？\n\n重启后需要重新连接。",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply != QMessageBox.StandardButton.Yes:
            return
        
        if self.param_manager.reboot_autopilot():
            QMessageBox.information(
                self, "重启命令已发送",
                "飞控重启命令已发送\n\n请等待 10-20 秒后重新连接"
            )
            # 断开连接
            self._disconnect_from_fcu()
        else:
            QMessageBox.critical(self, "重启失败", "发送重启命令失败")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        # 检查未保存的修改
        modified = [p for p in self.params.values() if p.is_modified]
        if modified:
            reply = QMessageBox.question(
                self, "确认关闭",
                f"有 {len(modified)} 个参数未保存，确定要关闭吗？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
        
        # 断开连接
        if self._connected:
            self.param_manager.disconnect()
        
        event.accept()
    
    def _center_on_screen(self):
        """将窗口居中显示在屏幕上"""
        from PyQt5.QtWidgets import QApplication
        screen = QApplication.desktop().screenGeometry()
        size = self.geometry()
        self.move(
            (screen.width() - size.width()) // 2,
            (screen.height() - size.height()) // 2
        )


# ==================== 测试代码 ====================
if __name__ == '__main__':
    import sys
    app = QApplication(sys.argv)
    
    window = ParamWindowSerial()
    window.show()
    
    sys.exit(app.exec())
