"""
USV 集群启动器对话框（性能优化版）
提供图形化界面管理 USV 集群的启动、停止和监控

性能优化：
1. 异步状态检测：使用独立线程执行网络和 ROS 检测，避免阻塞 GUI
2. 并行 ping 检测：使用线程池并行执行多个主机的 ping 操作
3. 智能日志输出：减少冗余日志，使用日志级别控制
4. 批量信号更新：减少 UI 更新频率，提高响应速度

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
from concurrent.futures import ThreadPoolExecutor, as_completed
from threading import Thread, Lock

# 导入common_utils工具
from common_utils import ProcessTracker

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QLabel, QHeaderView, QMessageBox, QCheckBox, QGroupBox,
    QProgressBar, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont


class UsvFleetLauncher(QDialog):
    """USV 集群启动器对话框（性能优化版）"""
    
    # 信号定义
    status_updated = pyqtSignal(str, str)  # (usv_id, status)
    batch_status_updated = pyqtSignal(dict)  # {usv_id: status}
    log_message = pyqtSignal(str)  # 异步日志消息
    
    def __init__(self, parent=None, workspace_path=None):
        """
        初始化 USV 集群启动器
        
        Args:
            parent: 父窗口
            workspace_path: ROS 2 工作空间路径
        """
        super().__init__(parent)
        
        # 设置窗口标志，使其不会始终置顶
        from PyQt5.QtCore import Qt
        self.setWindowFlags(Qt.Window)
        
        self.workspace_path = workspace_path or os.path.expanduser('~/usv_workspace')
        self.fleet_config = {}
        self.usv_processes = {}  # {usv_id: subprocess.Popen}
        self.usv_status = {}  # {usv_id: 'offline'|'launching'|'running'|'stopped'}
        self.last_stop_time = {}  # {usv_id: timestamp} 用于防止停止时的状态闪烁
        
        # 初始化进程追踪器
        self.process_tracker = ProcessTracker()
        
        # 状态检测线程相关
        self.status_check_thread = None
        self.status_check_running = False
        self.status_lock = Lock()  # 保护 usv_status 字典
        
        # 线程池用于并行 ping 检测
        self.executor = ThreadPoolExecutor(max_workers=10)
        
        # 日志级别控制（减少冗余输出）
        self.verbose_logging = False
        
        # 初始化 UI
        self._init_ui()
        
        # 连接信号（必须在状态检测之前连接）
        self.status_updated.connect(self._on_status_updated)
        self.batch_status_updated.connect(self._on_batch_status_updated)
        self.log_message.connect(self._log_sync)
        
        # 加载配置
        self._load_fleet_config()
        
        #启动异步状态检测线程
        self._start_status_check_thread()
        
        # 立即执行首次状态检测（不等待 3 秒）
        self._log("⏱️ 立即触发首次状态检测...")
        Thread(target=self._update_usv_status_async, daemon=True).start()
        
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
        title_font.setPointSize(15)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
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
        self.usv_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.usv_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Interactive)
        self.usv_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Interactive)
        self.usv_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Interactive)
        self.usv_table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Interactive)
        self.usv_table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        
        # 设置初始列宽度
        self.usv_table.setColumnWidth(1, 100)   # 设备 ID
        self.usv_table.setColumnWidth(2, 150)   # 主机地址
        self.usv_table.setColumnWidth(3, 100)   # 状态
        self.usv_table.setColumnWidth(4, 300)   # 操作（3个按钮需要更宽）
        # 详情列(5)使用 Stretch 模式自动填充剩余空间
        
        # 行为设置
        self.usv_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.usv_table.setAlternatingRowColors(True)
        self.usv_table.verticalHeader().setVisible(False)
        
        # 行高设置
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
        
        self.reboot_selected_btn = QPushButton("🔄 重启选中")
        self.reboot_selected_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                font-weight: bold;
                padding: 8px 16px;
                border-radius: 5px;
                border: 1px solid #F57C00;
                min-height: 28px;
            }
            QPushButton:hover {
                background-color: #FFB74D;
                border-color: #FF9800;
            }
            QPushButton:pressed {
                background-color: #F57C00;
            }
        """)
        self.reboot_selected_btn.clicked.connect(self._reboot_selected)
        batch_layout.addWidget(self.reboot_selected_btn)
        
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
                font-size: 14px;
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
        
        # 详细日志开关
        self.verbose_checkbox = QCheckBox("显示详细日志")
        self.verbose_checkbox.stateChanged.connect(self._toggle_verbose_logging)
        bottom_layout.addWidget(self.verbose_checkbox)
        
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
            
            /* 表头样式 */
            QHeaderView::section {
                background-color: #1976d2;
                color: white;
                padding: 8px;
                border: 1px solid #3a3a3a;
                font-weight: bold;
            }
            
            /* 通用按钮样式 */
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
            
            /* 复选框样式 */
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
            
            # 列 3: 状态（初始化为离线，等待首次检测）
            status_item = QTableWidgetItem("⚫ 离线")
            status_item.setTextAlignment(Qt.AlignCenter)
            status_item.setFlags(status_item.flags() & ~Qt.ItemIsEditable)
            status_item.setForeground(QColor(150, 150, 150))
            self.usv_table.setItem(row, 3, status_item)
            
            # 初始化状态字典（设为 None，确保首次检测会触发更新）
            self.usv_status[usv_id] = None
            
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
            with self.status_lock:
                self.usv_status[usv_id] = 'offline'
    
    def _create_action_buttons(self, usv_id):
        """创建操作按钮"""
        from PyQt5.QtWidgets import QWidget
        
        # 创建容器
        btn_container = QWidget()
        layout = QHBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)
        
        # 启动按钮
        launch_btn = QPushButton("▶️️ 启动")
        launch_btn.setFixedHeight(38)
        launch_btn.setMinimumWidth(70)
        launch_btn.setMaximumWidth(85)
        launch_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                padding: 4px 8px;
                border-radius: 4px;
                border: 1px solid #388e3c;
                font-size: 14px;
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
        
        # 重启按钮
        reboot_btn = QPushButton("🔄 重启")
        reboot_btn.setFixedHeight(38)
        reboot_btn.setMinimumWidth(70)
        reboot_btn.setMaximumWidth(85)
        reboot_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                padding: 4px 8px;
                border-radius: 4px;
                border: 1px solid #F57C00;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #FFB74D;
                border-color: #FF9800;
            }
            QPushButton:pressed {
                background-color: #F57C00;
            }
        """)
        reboot_btn.clicked.connect(lambda: self._reboot_single(usv_id))
        layout.addWidget(reboot_btn)
        
        layout.addStretch()
        
        btn_container.setLayout(layout)
        
        return btn_container
    
    def _log(self, message):
        """异步日志输出（通过信号）"""
        self.log_message.emit(message)
    
    def _log_sync(self, message):
        """同步日志输出（在主线程中执行）"""
        self.log_text.append(message)
        # 滚动到底部
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def _toggle_verbose_logging(self, state):
        """切换详细日志模式"""
        self.verbose_logging = (state == Qt.Checked)
        if self.verbose_logging:
            self._log("🔍 详细日志模式已启用")
        else:
            self._log("🔇 详细日志模式已关闭")
    
    def _start_status_check_thread(self):
        """启动异步状态检测线程"""
        if self.status_check_thread and self.status_check_thread.is_alive():
            return
        
        self.status_check_running = True
        self.status_check_thread = Thread(target=self._status_check_loop, daemon=True)
        self.status_check_thread.start()
        self._log("🚀 异步状态检测线程已启动")
    
    def _status_check_loop(self):
        """状态检测循环（在独立线程中运行）"""
        while self.status_check_running:
            try:
                self._update_usv_status_async()
                # 延长查询时间，默认 10 秒检测一次，减少系统开销
                time.sleep(10)
            except Exception as e:
                import traceback
                self._log(f"⚠️ 状态检测异常: {e}")
                time.sleep(10)
    
    def _update_usv_status_async(self):
        """
        异步更新所有 USV 的状态
        
        优化策略：
        1. 准确判断：结合 ping (OS层)、进程追踪 (启动层) 和 ROS 状态 (应用层)
        2. 性能优化：移除昂贵的 'ros2 node list'，改用 GroundStationNode 已有的状态缓存
        3. 并行检测：使用线程池并行执行 ping 操作
        """
        try:
            self._log("🔍 开始状态检测 (应用层 + 网络层)...")
            
            # 步骤 1: 获取 ROS 层的在线状态（从 GroundStationNode 获取）
            # 这比运行 'ros2 node list' 快得多且更准确
            online_usvs_ros = set()
            parent = self.parent()
            if parent and hasattr(parent, 'ros_node'):
                try:
                    # 优先使用 discovery_handler，因为它包含超时掉线检测逻辑 (offline_threshold)
                    # usv_manager 仅作为底层消息接收器，可能无法及时感知掉线
                    if hasattr(parent.ros_node, 'discovery_handler'):
                        discovery = parent.ros_node.discovery_handler
                        states = discovery.get_usv_states()
                        for usv_id, state in states.items():
                            if state.get('connected', False):
                                online_usvs_ros.add(usv_id)
                    # 备选方案：如果 discovery_handler 不可用，回退到 usv_manager (可能存在状态滞后)
                    elif hasattr(parent.ros_node, 'usv_manager'):
                        usv_manager = parent.ros_node.usv_manager
                        for usv_id, state in usv_manager.usv_states.items():
                            if state.connected:
                                online_usvs_ros.add(usv_id)
                except Exception as e:
                    self._log(f"⚠️ 从 ROS 节点获取状态失败: {e}")
            
            # 步骤 2: 并行检测所有主机的网络在线状态 (Ping)
            host_status = {}  # {hostname: is_online}
            futures = {}
            for usv_id, config in self.fleet_config.items():
                if not config.get('enabled', False):
                    continue
                
                hostname = config.get('hostname', '')
                if hostname and hostname not in host_status:
                    future = self.executor.submit(self._check_host_online_fast, hostname)
                    futures[future] = hostname
            
            for future in as_completed(futures):
                hostname = futures[future]
                try:
                    host_status[hostname] = future.result()
                except Exception:
                    host_status[hostname] = False
            
            # 步骤 3: 综合判断状态
            status_updates = {}
            
            for usv_id, config in self.fleet_config.items():
                if not config.get('enabled', False):
                    continue
                
                hostname = config.get('hostname', '')
                
                # A. 应用层：ROS 节点是否已连接并发布状态
                is_running_ros = usv_id in online_usvs_ros
                
                # B. 启动层：本地是否有正在运行的 SSH 启动进程
                has_process = (usv_id in self.usv_processes and 
                             self.usv_processes[usv_id].poll() is None)
                
                # C. 网络层：主机是否可 Ping 通
                is_host_online = host_status.get(hostname, False)
                
                # D. 停止保护：如果刚刚发送了停止指令，在 5 秒内忽略 ROS 在线状态
                is_stopping = False
                if usv_id in self.last_stop_time:
                    if time.time() - self.last_stop_time[usv_id] < 5.0:
                        is_stopping = True
                
                # 状态优先级判断逻辑：
                # 1. 如果正在停止 -> 已停止 (stopped)
                # 2. 如果 Ping 不通 -> 离线 (offline) [新增优先级，解决重启后状态滞后问题]
                # 3. 如果 ROS 节点在线 -> 运行中 (running)
                # 4. 如果 ROS 不在线但进程还在 -> 启动中 (launching)
                # 5. 否则 -> 离线 (offline)
                if is_stopping:
                    new_status = 'stopped'
                elif not is_host_online:
                    # 如果物理网络都不通了，强制判定为离线
                    # 这能解决重启/断电后，ROS 心跳超时导致的状态滞后显示为"运行中"的问题
                    new_status = 'offline'
                elif is_running_ros:
                    new_status = 'running'
                elif has_process:
                    new_status = 'launching'
                else:
                    # 网络通，但无 ROS 节点运行且无启动进程 -> 待机在线
                    new_status = 'online'
                
                with self.status_lock:
                    old_status = self.usv_status.get(usv_id)
                    if old_status != new_status:
                        self.usv_status[usv_id] = new_status
                        status_updates[usv_id] = new_status
                        
                        if self.verbose_logging:
                            self._log(f"📊 {usv_id}: {old_status or 'init'} → {new_status} "
                                     f"[ROS={is_running_ros}, Proc={has_process}, Ping={is_host_online}]")
            
            # 步骤 4: 批量更新 UI
            if status_updates:
                self.batch_status_updated.emit(status_updates)
            
            if self.verbose_logging:
                self._log("✅ 状态检测完成")
        
        except Exception as e:
            self._log(f"❌ 状态检测失败: {e}")
    
    def _check_host_online_fast(self, hostname):
        """
        快速检查主机是否在线（优化版 ping）
        
        Args:
            hostname: 主机名或 IP 地址
        
        Returns:
            bool: 主机在线返回 True，否则返回 False
        """
        if not hostname:
            return False
            
        try:
            # 优化的 ping 命令：
            # -c 1: 发送 1 个包
            # -W 1: 超时 1 秒（减少等待时间）
            # -q: 安静模式，减少输出
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', '-q', hostname],
                capture_output=True,
                timeout=2  # 总超时 2 秒
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False
        except Exception as e:
            # 记录异常信息以便调试
            self._log(f"⚠️ Ping {hostname} 异常: {e}")
            return False
    
    def _on_status_updated(self, usv_id, status):
        """单个状态更新时的回调"""
        self._update_table_row(usv_id, status)
    
    def _on_batch_status_updated(self, status_dict):
        """批量状态更新时的回调"""
        self._log(f"🎨 UI 更新回调: {list(status_dict.items())}")
        for usv_id, status in status_dict.items():
            self._update_table_row(usv_id, status)
    
    def _update_table_row(self, usv_id, status):
        """更新表格中指定 USV 的状态"""
        for row in range(self.usv_table.rowCount()):
            if self.usv_table.item(row, 1).text() == usv_id:
                status_item = self.usv_table.item(row, 3)
                
                # 状态文本和颜色
                status_map = {
                    'offline': ('⚫ 离线', QColor(150, 150, 150)),
                    'online': ('🟡 在线', QColor(255, 193, 7)),
                    'launching': ('🔄 启动中...', QColor(255, 152, 0)),
                    'running': ('🟢 运行中', QColor(76, 175, 80)),
                    'stopped': ('🔴 已停止', QColor(244, 67, 54))
                }
                
                text, color = status_map.get(status, ('❓ 未知', QColor(100, 100, 100)))
                status_item.setText(text)
                status_item.setForeground(color)
                break
    
    def _launch_single(self, usv_id):
        """启动单个 USV（异步执行，避免阻塞 GUI）"""
        # 在独立线程中执行启动命令
        Thread(target=self._launch_single_async, args=(usv_id,), daemon=True).start()
    
    def _launch_single_async(self, usv_id):
        """异步启动单个 USV"""
        self._log(f"🚀 正在启动 {usv_id}...")
        
        try:
            config = self.fleet_config[usv_id]
            
            # 构建 SSH 命令
            hostname = config['hostname']
            username = config['username']
            workspace = config['workspace']
            namespace = usv_id
            fcu_url = str(config.get('fcu_url', '') or '')
            platform_mode = str(config.get('platform_mode', '3d')).strip().lower() or '3d'
            vehicle_type = str(config.get('vehicle_type', 'flying_sphere')).strip().lower() or 'flying_sphere'
            nav_mode = str(config.get('nav_mode', 'rl' if vehicle_type in ('usv_diff', 'usv') else 'position')).strip().lower()
            rl_model = str(config.get('rl_policy_model', '')).strip()
            reach_xy = config.get('reach_xy_threshold', -1.0)
            reach_z = config.get('reach_z_threshold', -1.0)

            # 从 fcu_url 推导 usv_launch.py 串口参数（仅支持 serial:///dev/xxx:baudrate 形式；否则保持 usv_launch.py 默认值）
            use_ethernet_arg = None
            serial_port_arg = None
            baudrate_arg = None
            if fcu_url.startswith('serial://'):
                try:
                    raw = fcu_url[len('serial://'):]
                    if raw.startswith('/'):
                        raw = raw[1:]
                    if ':' in raw:
                        port, baud = raw.rsplit(':', 1)
                        if not port.startswith('/'):
                            port = '/' + port
                        serial_port_arg = port
                        baudrate_arg = baud
                        use_ethernet_arg = 'false'
                except Exception:
                    pass
            
            remote_cmd = (
                f"bash -c '"
                f"source /opt/ros/*/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash; "
                f"source {workspace}/install/setup.bash; "
                f"ros2 launch usv_bringup usv_launch.py "
                f"namespace:={namespace} "
                f"platform_mode:={platform_mode} "
                f"vehicle_type:={vehicle_type} "
                f"nav_mode:={nav_mode} "
                f"rl_policy_enabled:={'true' if nav_mode == 'rl' else 'false'} "
                f"reach_xy_threshold:={reach_xy} "
                f"reach_z_threshold:={reach_z}"
            )
            if rl_model:
                remote_cmd += f" rl_policy_model:={rl_model}"

            if use_ethernet_arg is not None:
                remote_cmd += f" use_ethernet:={use_ethernet_arg}"
            if serial_port_arg is not None:
                remote_cmd += f" serial_port:={serial_port_arg}"
            if baudrate_arg is not None:
                remote_cmd += f" baudrate:={baudrate_arg}"
            
            remote_cmd += "'"
            
            ssh_cmd = [
                'ssh',
                '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=10',
                '-t',
                f'{username}@{hostname}',
                remote_cmd
            ]
            
            # 启动进程并追踪
            process = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # 追踪进程
            self.process_tracker.track(process, f'USV {usv_id} SSH Launch')
            
            self.usv_processes[usv_id] = process
            
            with self.status_lock:
                self.usv_status[usv_id] = 'launching'
            
            self.status_updated.emit(usv_id, 'launching')
            
            self._log(f"✅ {usv_id} 启动命令已发送 (PID: {process.pid})")
        
        except Exception as e:
            self._log(f"❌ {usv_id} 启动失败: {e}")
    
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
            for i, usv_id in enumerate(selected):
                # 使用定时器延迟启动，避免同时启动
                QTimer.singleShot(2000 * i, lambda uid=usv_id: self._launch_single(uid))
    
    def _refresh_status(self):
        """手动刷新状态"""
        self._log("🔄 手动刷新状态...")
        # 触发一次异步状态检测
        Thread(target=self._update_usv_status_async, daemon=True).start()
    
    def _reboot_single(self, usv_id):
        """重启单个 USV 的机载计算机"""
        reply = QMessageBox.question(
            self,
            "确认重启",
            f"确定要重启 {usv_id} 的机载计算机吗？\n\n"
            f"⚠️ 重启后系统需要 30-60 秒恢复在线\n"
            f"⚠️ 所有运行中的节点将被终止",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self._log(f"🔄 正在重启 {usv_id} 的机载计算机...")
            
            # 清理本地关联的启动进程，避免重启后显示为 'launching'
            if usv_id in self.usv_processes:
                try:
                    proc = self.usv_processes.pop(usv_id)
                    proc.terminate()
                    self._log(f"🧹 已清理 {usv_id} 的本地启动进程")
                except Exception:
                    pass
            
            try:
                parent = self.parent()
                if parent and hasattr(parent, 'ros_signal'):
                    parent.ros_signal.reboot_companion.emit(usv_id)
                    self._log(f"✅ {usv_id} 重启命令已发送")
                else:
                    self._log(f"❌ 无法获取 ROS 信号对象，重启失败")
                    QMessageBox.warning(
                        self,
                        "重启失败",
                        f"无法访问 ROS 通信接口\n请确保地面站已正常启动"
                    )
            except Exception as e:
                self._log(f"❌ {usv_id} 重启失败: {e}")
    
    def _reboot_selected(self):
        """批量重启选中的 USV 机载计算机"""
        selected = self._get_selected_usvs()
        
        if not selected:
            QMessageBox.information(self, "提示", "请先选择要重启的 USV")
            return
        
        reply = QMessageBox.question(
            self,
            "确认批量重启",
            f"确定要重启以下 {len(selected)} 艘 USV 的机载计算机吗？\n\n" + 
            "\n".join(selected) + "\n\n" +
            "⚠️ 重启后系统需要 30-60 秒恢复在线\n"
            "⚠️ 所有运行中的节点将被终止",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self._log(f"🔄 批量重启: {', '.join(selected)}")
            for usv_id in selected:
                # 清理本地进程
                if usv_id in self.usv_processes:
                    try:
                        self.usv_processes.pop(usv_id).terminate()
                    except: pass
                
                try:
                    parent = self.parent()
                    if parent and hasattr(parent, 'ros_signal'):
                        parent.ros_signal.reboot_companion.emit(usv_id)
                        self._log(f"✅ {usv_id} 重启命令已发送")
                    else:
                        self._log(f"❌ {usv_id}: 无法获取 ROS 信号对象")
                except Exception as e:
                    self._log(f"❌ {usv_id} 重启失败: {e}")
                
                time.sleep(2)  # 延迟 2 秒避免同时发送
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        # 停止状态检测线程
        self.status_check_running = False
        
        # 关闭线程池
        self.executor.shutdown(wait=False)
        
        # 清理所有进程
        self.process_tracker.cleanup_all()
        
        # 通知父窗口清理引用
        if self.parent():
            if hasattr(self.parent(), '_usv_fleet_launcher'):
                self.parent()._usv_fleet_launcher = None
        
        event.accept()
