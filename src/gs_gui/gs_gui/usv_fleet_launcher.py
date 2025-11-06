"""
USV 集群启动器对话框
提供图形化界面管理 USV 集群的启动、停止和监控

功能：
1. 显示在线设备列表（从 usv_fleet.yaml 和 ROS 节点检测）
2. 单独启动/停止单个 USV
3. 批量启动/停止选中的 USV
4. 实时状态监控（离线/启动中/运行中/已停止）
5. 美观的现代化 UI 设计
"""

import os
import yaml
import subprocess
import time
from typing import Dict, List, Optional
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QLabel, QHeaderView, QMessageBox, QCheckBox, QGroupBox,
    QProgressBar, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont


class UsvFleetLauncher(QDialog):
    """USV 集群启动器对话框"""
    
    # 信号定义
    status_updated = pyqtSignal(str, str)  # (usv_id, status)
    
    def __init__(self, parent=None, workspace_path=None):
        """
        初始化 USV 集群启动器
        
        Args:
            parent: 父窗口
            workspace_path: ROS 2 工作空间路径
        """
        super().__init__(parent)
        
        self.workspace_path = workspace_path or os.path.expanduser('~/usv_workspace')
        self.fleet_config = {}
        self.usv_processes = {}  # {usv_id: subprocess.Popen}
        self.usv_status = {}  # {usv_id: 'offline'|'launching'|'running'|'stopped'}
        
        # 初始化 UI
        self._init_ui()
        
        # 加载配置
        self._load_fleet_config()
        
        # 启动定时器检测状态
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_usv_status)
        self.status_timer.start(2000)  # 每 2 秒更新一次状态
        
        # 连接信号
        self.status_updated.connect(self._on_status_updated)
        
        # 窗口居中显示
        self._center_on_screen()
    
    def _center_on_screen(self):
        """将窗口居中显示在屏幕上"""
        from PyQt5.QtWidgets import QApplication
        screen = QApplication.desktop().screenGeometry()
        size = self.geometry()
        self.move(
            (screen.width() - size.width()) // 2,
            (screen.height() - size.height()) // 2
        )
    
    def _init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("USV 集群启动器")
        self.setMinimumSize(900, 600)
        
        # 主布局
        main_layout = QVBoxLayout()
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # ============== 标题区域 ==============
        title_label = QLabel("▶️ USV 集群管理")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # 副标题
        subtitle_label = QLabel("管理和监控所有 USV 节点的启动与停止")
        subtitle_label.setAlignment(Qt.AlignCenter)
        subtitle_label.setStyleSheet("color: #9e9e9e; font-size: 12px;")
        main_layout.addWidget(subtitle_label)
        
        # ============== USV 列表区域 ==============
        list_group = QGroupBox("📝 USV 设备列表")
        list_layout = QVBoxLayout()
        
        # USV 表格
        self.usv_table = QTableWidget()
        self.usv_table.setColumnCount(6)
        self.usv_table.setHorizontalHeaderLabels([
            "选择", "设备 ID", "主机地址", "状态", "操作", "详情"
        ])
        
        # 设置表格样式
        # 列宽度设置：所有列都可以拖拽调整
        self.usv_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)  # 选择列：自适应内容
        self.usv_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Interactive)       # 设备ID：可拖拽
        self.usv_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Interactive)       # 主机地址：可拖拽
        self.usv_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Interactive)       # 状态：可拖拽
        self.usv_table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Interactive)       # 操作：可拖拽（修改）
        self.usv_table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)           # 详情：拉伸填充
        
        # 设置初始列宽度
        self.usv_table.setColumnWidth(1, 100)   # 设备 ID
        self.usv_table.setColumnWidth(2, 150)   # 主机地址
        self.usv_table.setColumnWidth(3, 100)   # 状态
        self.usv_table.setColumnWidth(4, 180)   # 操作（减小到 180px，更紧凑）
        
        # 行为设置
        self.usv_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.usv_table.setAlternatingRowColors(True)
        self.usv_table.verticalHeader().setVisible(False)
        
        # 行高设置：固定为 55px
        self.usv_table.verticalHeader().setDefaultSectionSize(55)
        
        list_layout.addWidget(self.usv_table)
        list_group.setLayout(list_layout)
        main_layout.addWidget(list_group)
        
        # ============== 批量操作按钮区域 ==============
        batch_group = QGroupBox("🎯 批量操作")
        batch_layout = QHBoxLayout()
        
        self.select_all_btn = QPushButton("✓ 全选")
        self.select_all_btn.clicked.connect(self._select_all)
        batch_layout.addWidget(self.select_all_btn)
        
        self.deselect_all_btn = QPushButton("✗ 取消全选")
        self.deselect_all_btn.clicked.connect(self._deselect_all)
        batch_layout.addWidget(self.deselect_all_btn)
        
        batch_layout.addStretch()
        
        self.launch_selected_btn = QPushButton("▶️️ 启动选中")
        self.launch_selected_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 8px 16px;
                border-radius: 5px;
                border: 1px solid #388e3c;
                min-height: 28px;
            }
            QPushButton:hover {
                background-color: #66bb6a;
                border-color: #4caf50;
            }
            QPushButton:pressed {
                background-color: #388e3c;
            }
        """)
        self.launch_selected_btn.clicked.connect(self._launch_selected)
        batch_layout.addWidget(self.launch_selected_btn)
        
        batch_group.setLayout(batch_layout)
        main_layout.addWidget(batch_group)
        
        # ============== 日志输出区域 ==============
        log_group = QGroupBox("📋 操作日志")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #00ff00;
                font-family: 'Courier New', monospace;
                font-size: 11px;
            }
        """)
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)
        
        # ============== 底部按钮区域 ==============
        bottom_layout = QHBoxLayout()
        
        self.refresh_btn = QPushButton("🔄 刷新状态")
        self.refresh_btn.clicked.connect(self._refresh_status)
        bottom_layout.addWidget(self.refresh_btn)
        
        bottom_layout.addStretch()
        
        self.close_btn = QPushButton("关闭")
        self.close_btn.clicked.connect(self.close)
        bottom_layout.addWidget(self.close_btn)
        
        main_layout.addLayout(bottom_layout)
        
        # 设置主布局
        self.setLayout(main_layout)
        
        # 应用全局样式
        self._apply_styles()
    
    def _apply_styles(self):
        """应用全局样式（深色主题，与主界面一致）"""
        self.setStyleSheet("""
            /* 对话框主背景 - 深色主题 */
            QDialog {
                background-color: #1e1e1e;
                color: #e0e0e0;
            }
            
            /* 标签颜色 */
            QLabel {
                color: #e0e0e0;
            }
            
            /* 分组框样式 - 深色主题 */
            QGroupBox {
                font-weight: bold;
                border: 2px solid #3a3a3a;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 10px;
                background-color: #252525;
                color: #4fc3f7;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 15px;
                padding: 0 5px;
                color: #4fc3f7;
            }
            
            /* 表格样式 - 深色主题 */
            QTableWidget {
                border: 1px solid #3a3a3a;
                border-radius: 4px;
                background-color: #2b2b2b;
                color: #e0e0e0;
                gridline-color: #3a3a3a;
            }
            QTableWidget::item {
                padding: 5px;
                color: #e0e0e0;
            }
            QTableWidget::item:selected {
                background-color: #1976d2;
                color: #ffffff;
            }
            QTableWidget::item:alternate {
                background-color: #252525;
            }
            
            /* 表头样式 - 保持蓝色但适配深色主题 */
            QHeaderView::section {
                background-color: #1976d2;
                color: white;
                padding: 8px;
                border: 1px solid #3a3a3a;
                font-weight: bold;
            }
            
            /* 通用按钮样式 - 深色主题 */
            QPushButton {
                padding: 8px 16px;
                border-radius: 5px;
                border: 1px solid #555555;
                background-color: #424242;
                color: #e0e0e0;
                min-height: 28px;
            }
            QPushButton:hover {
                background-color: #4fc3f7;
                color: #000000;
            }
            QPushButton:pressed {
                background-color: #0277bd;
                color: #ffffff;
            }
            QPushButton:disabled {
                background-color: #2a2a2a;
                color: #666666;
            }
            
            /* 复选框样式 - 深色主题 */
            QCheckBox {
                color: #e0e0e0;
                spacing: 5px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border: 2px solid #555555;
                border-radius: 3px;
                background-color: #2b2b2b;
            }
            QCheckBox::indicator:checked {
                background-color: #4fc3f7;
                border-color: #4fc3f7;
            }
            QCheckBox::indicator:hover {
                border-color: #4fc3f7;
            }
        """)
    
    def _load_fleet_config(self):
        """加载 USV 集群配置"""
        config_file = os.path.join(
            self.workspace_path, 
            'install/gs_bringup/share/gs_bringup/config/usv_fleet.yaml'
        )
        
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                self.fleet_config = config.get('usv_fleet', {})
                self._log(f"✅ 加载配置成功: {len(self.fleet_config)} 艘 USV")
                self._populate_table()
        except FileNotFoundError:
            self._log(f"⚠️ 配置文件未找到: {config_file}")
            QMessageBox.warning(
                self,
                "配置文件未找到",
                f"未找到 USV 集群配置文件:\n{config_file}\n\n"
                "请确保已编译 gs_bringup 包。"
            )
        except Exception as e:
            self._log(f"❌ 加载配置失败: {e}")
            QMessageBox.critical(self, "错误", f"加载配置文件失败:\n{e}")
    
    def _populate_table(self):
        """填充 USV 表格"""
        self.usv_table.setRowCount(0)
        
        for usv_id, config in self.fleet_config.items():
            if not config.get('enabled', False):
                continue
            
            row = self.usv_table.rowCount()
            self.usv_table.insertRow(row)
            
            # 列 0: 复选框
            checkbox = QCheckBox()
            checkbox.setStyleSheet("margin-left: 10px;")
            self.usv_table.setCellWidget(row, 0, checkbox)
            
            # 列 1: 设备 ID
            id_item = QTableWidgetItem(usv_id)
            id_item.setTextAlignment(Qt.AlignCenter)
            id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
            self.usv_table.setItem(row, 1, id_item)
            
            # 列 2: 主机地址
            host_item = QTableWidgetItem(config.get('hostname', 'N/A'))
            host_item.setTextAlignment(Qt.AlignCenter)
            host_item.setFlags(host_item.flags() & ~Qt.ItemIsEditable)
            self.usv_table.setItem(row, 2, host_item)
            
            # 列 3: 状态
            status_item = QTableWidgetItem("检测中...")
            status_item.setTextAlignment(Qt.AlignCenter)
            status_item.setFlags(status_item.flags() & ~Qt.ItemIsEditable)
            self.usv_table.setItem(row, 3, status_item)
            
            # 列 4: 操作按钮
            btn_widget = self._create_action_buttons(usv_id)
            self.usv_table.setCellWidget(row, 4, btn_widget)
            
            # 列 5: 详情
            detail = f"FCU: {config.get('fcu_url', 'N/A')[:20]}..."
            detail_item = QTableWidgetItem(detail)
            detail_item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            detail_item.setFlags(detail_item.flags() & ~Qt.ItemIsEditable)
            self.usv_table.setItem(row, 5, detail_item)
            
            # 初始化状态
            self.usv_status[usv_id] = 'offline'
    
    def _create_action_buttons(self, usv_id):
        """创建操作按钮"""
        from PyQt5.QtWidgets import QWidget
        
        # 创建容器
        btn_container = QWidget()
        layout = QHBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)  # 适中的容器边距
        layout.setSpacing(6)  # 适中的按钮间距
        
        # 启动按钮 - 紧凑设计
        launch_btn = QPushButton("▶️️ 启动")
        launch_btn.setFixedHeight(38)  # 固定高度 38px
        launch_btn.setMinimumWidth(70)   # 最小宽度 70px
        launch_btn.setMaximumWidth(85)   # 最大宽度 85px
        launch_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                padding: 4px 8px;
                border-radius: 4px;
                border: 1px solid #388e3c;
                font-size: 11px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #66bb6a;
                border-color: #4caf50;
            }
            QPushButton:pressed {
                background-color: #388e3c;
            }
        """)
        launch_btn.clicked.connect(lambda: self._launch_single(usv_id))
        layout.addWidget(launch_btn)
        
        layout.addStretch()  # 添加弹性空间，使按钮靠左对齐
        
        btn_container.setLayout(layout)
        
        return btn_container
    
    def _log(self, message):
        """添加日志信息"""
        self.log_text.append(message)
        # 滚动到底部
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def _update_usv_status(self):
        """更新所有 USV 的状态"""
        try:
            # 获取所有 ROS 节点
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=3
            )
            
            online_nodes = result.stdout.strip().split('\n') if result.returncode == 0 else []
            
            # 更新每个 USV 的状态
            for usv_id in self.fleet_config.keys():
                if not self.fleet_config[usv_id].get('enabled', False):
                    continue
                
                # 检查该 USV 的关键节点是否在线
                namespace = f"/{usv_id}"
                has_nodes = any(namespace in node for node in online_nodes)
                
                if usv_id in self.usv_processes and self.usv_processes[usv_id].poll() is None:
                    # 进程正在运行
                    new_status = 'launching' if not has_nodes else 'running'
                else:
                    # 进程已停止
                    new_status = 'running' if has_nodes else 'offline'
                
                if self.usv_status.get(usv_id) != new_status:
                    self.usv_status[usv_id] = new_status
                    self.status_updated.emit(usv_id, new_status)
        
        except subprocess.TimeoutExpired:
            self._log("⚠️ 状态检测超时")
        except Exception as e:
            self._log(f"⚠️ 状态检测失败: {e}")
    
    def _on_status_updated(self, usv_id, status):
        """状态更新时的回调"""
        # 查找对应行
        for row in range(self.usv_table.rowCount()):
            if self.usv_table.item(row, 1).text() == usv_id:
                status_item = self.usv_table.item(row, 3)
                
                # 状态文本和颜色
                status_map = {
                    'offline': ('⚫ 离线', QColor(150, 150, 150)),
                    'launching': ('🔄 启动中...', QColor(255, 152, 0)),
                    'running': ('🟢 运行中', QColor(76, 175, 80)),
                    'stopped': ('🔴 已停止', QColor(244, 67, 54))
                }
                
                text, color = status_map.get(status, ('❓ 未知', QColor(100, 100, 100)))
                status_item.setText(text)
                status_item.setForeground(color)
                break
    
    def _launch_single(self, usv_id):
        """启动单个 USV"""
        self._log(f"🚀 正在启动 {usv_id}...")
        
        try:
            config = self.fleet_config[usv_id]
            
            # 构建 SSH 命令
            hostname = config['hostname']
            username = config['username']
            workspace = config['workspace']
            namespace = usv_id
            fcu_url = config['fcu_url']
            system_id = config['system_id']
            gcs_url = config.get('gcs_url', '')
            
            remote_cmd = (
                f"bash -c '"
                f"source /opt/ros/*/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash; "
                f"source {workspace}/install/setup.bash; "
                f"ros2 launch usv_bringup usv_launch.py "
                f"namespace:={namespace} "
                f"fcu_url:={fcu_url} "
                f"tgt_system:={system_id}"
            )
            
            if gcs_url:
                remote_cmd += f" gcs_url:={gcs_url}"
            
            remote_cmd += "'"
            
            ssh_cmd = [
                'ssh',
                '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=10',
                '-t',
                f'{username}@{hostname}',
                remote_cmd
            ]
            
            # 启动进程
            process = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.usv_processes[usv_id] = process
            self.usv_status[usv_id] = 'launching'
            self.status_updated.emit(usv_id, 'launching')
            
            self._log(f"✅ {usv_id} 启动命令已发送 (PID: {process.pid})")
        
        except Exception as e:
            self._log(f"❌ {usv_id} 启动失败: {e}")
            QMessageBox.critical(self, "启动失败", f"{usv_id} 启动失败:\n{e}")
    
    def _select_all(self):
        """全选所有 USV"""
        for row in range(self.usv_table.rowCount()):
            checkbox = self.usv_table.cellWidget(row, 0)
            if checkbox:
                checkbox.setChecked(True)
    
    def _deselect_all(self):
        """取消全选"""
        for row in range(self.usv_table.rowCount()):
            checkbox = self.usv_table.cellWidget(row, 0)
            if checkbox:
                checkbox.setChecked(False)
    
    def _get_selected_usvs(self):
        """获取选中的 USV ID 列表"""
        selected = []
        for row in range(self.usv_table.rowCount()):
            checkbox = self.usv_table.cellWidget(row, 0)
            if checkbox and checkbox.isChecked():
                usv_id = self.usv_table.item(row, 1).text()
                selected.append(usv_id)
        return selected
    
    def _launch_selected(self):
        """启动选中的 USV"""
        selected = self._get_selected_usvs()
        
        if not selected:
            QMessageBox.information(self, "提示", "请先选择要启动的 USV")
            return
        
        reply = QMessageBox.question(
            self,
            "确认启动",
            f"确定要启动以下 {len(selected)} 艘 USV 吗？\n\n" + "\n".join(selected),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self._log(f"🚀 批量启动: {', '.join(selected)}")
            for usv_id in selected:
                self._launch_single(usv_id)
                # 延迟避免同时启动
                QTimer.singleShot(2000 * selected.index(usv_id), lambda: None)
    
    def _refresh_status(self):
        """手动刷新状态"""
        self._log("🔄 手动刷新状态...")
        self._update_usv_status()
        self._log("✅ 刷新完成")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        # 停止定时器
        self.status_timer.stop()
        
        # 直接接受关闭事件，不再弹出确认对话框
        event.accept()
