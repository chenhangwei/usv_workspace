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
# 导入系统命令处理器
from gs_gui.system_command_handler import SystemCommandHandler

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
        
        # 初始化进程追踪器
        self.process_tracker = ProcessTracker()
        
        # 初始化系统命令处理器
        self.system_command_handler = SystemCommandHandler()
        
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
        self.setWindowTitle("USV 集群启动器 (优化版)")
        self.setMinimumSize(900, 600)
        
        # 主布局
        main_layout = QVBoxLayout()
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # ============== 标题区域 ==============
        title_label = QLabel("▶️ USV 集群管理 (性能优化)")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # 副标题
        subtitle_label = QLabel("管理和监控所有 USV 节点的启动与停止 | 异步检测 + 并行优化")
        subtitle_label.setAlignment(Qt.AlignCenter)
        subtitle_label.setStyleSheet("color: #9e9e9e; font-size: 14px;")
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
        self.log_text.setMinimumHeight(100)
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
        
        # 应用初始主题 (默认 Dark)
        self.set_theme('modern_dark')
    
    def set_theme(self, theme_name):
        """设置界面主题 (Light / Dark)"""
        is_dark = (theme_name == 'modern_dark')
        
        if is_dark:
            # Dark colors
            bg_color = "#1e1e1e"
            text_color = "#e0e0e0"
            group_bg = "#252525"
            group_border = "#3a3a3a"
            group_title = "#4fc3f7"
            table_bg = "#2b2b2b"
            table_alt = "#252525"
            table_header_bg = "#1976d2"
            table_header_text = "white"
            btn_bg = "#424242"
            btn_text = "#e0e0e0"
            log_bg = "#1e1e1e"
            log_text = "#00ff00"
        else:
            # Light colors
            bg_color = "#f5f5f5"
            text_color = "#333333"
            group_bg = "#ffffff"
            group_border = "#d0d0d0"
            group_title = "#0078d7"
            table_bg = "#ffffff"
            table_alt = "#f9f9f9"
            table_header_bg = "#f0f0f0"
            table_header_text = "#0078d7"
            btn_bg = "#ffffff"
            btn_text = "#333333"
            log_bg = "#ffffff"
            log_text = "#333333"
            
        style_sheet = f"""
            /* 对话框主背景 */
            QDialog {{
                background-color: {bg_color};
                color: {text_color};
            }}
            
            /* 标签颜色 */
            QLabel {{
                color: {text_color};
            }}
            
            /* 分组框样式 */
            QGroupBox {{
                font-weight: bold;
                border: 2px solid {group_border};
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 10px;
                background-color: {group_bg};
                color: {group_title};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 15px;
                padding: 0 5px;
                background-color: {group_bg};
                color: {group_title};
            }}
            
            /* 表格样式 */
            QTableWidget {{
                border: 1px solid {group_border};
                border-radius: 4px;
                background-color: {table_bg};
                color: {text_color};
                gridline-color: {group_border};
            }}
            QTableWidget::item {{
                padding: 5px;
                color: {text_color};
            }}
            QTableWidget::item:selected {{
                background-color: {('#1976d2' if is_dark else '#e6f7ff')};
                color: {('#ffffff' if is_dark else '#0078d7')};
            }}
            QTableWidget::item:alternate {{
                background-color: {table_alt};
            }}
            
            /* 表头样式 */
            QHeaderView::section {{
                background-color: {table_header_bg};
                color: {table_header_text};
                padding: 8px;
                border: 1px solid {group_border};
                font-weight: bold;
            }}
            
            /* 通用按钮样式 (Refresh, Close, Select All) */
            QPushButton {{
                padding: 8px 16px;
                border-radius: 5px;
                border: 1px solid {('#555555' if is_dark else '#cccccc')};
                background-color: {btn_bg};
                color: {btn_text};
                min-height: 28px;
            }}
            QPushButton:hover {{
                background-color: {('#4fc3f7' if is_dark else '#e6f7ff')};
                color: {('#000000' if is_dark else '#0078d7')};
            }}
            QPushButton:pressed {{
                background-color: {('#0277bd' if is_dark else '#cceeff')};
                color: {('#ffffff' if is_dark else '#005a9e')};
            }}
            
            /* 复选框样式 */
            QCheckBox {{
                color: {text_color};
                spacing: 5px;
            }}
            
            /* 日志框 */
            QTextEdit {{
                background-color: {log_bg};
                color: {log_text};
                font-family: 'Courier New', monospace;
                font-size: 14px;
                border: 1px solid {group_border};
            }}
        """
        self.setStyleSheet(style_sheet)

    
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
                time.sleep(5)  # 每 5 秒检测一次（降低频率）
            except Exception as e:
                # 始终记录异常（不管 verbose_logging）
                import traceback
                self._log(f"⚠️ 状态检测异常: {e}")
                self._log(f"详细堆栈:\n{traceback.format_exc()}")
                time.sleep(10)  # 异常后延长等待时间
    
    def _update_usv_status_async(self):
        """
        异步更新所有 USV 的状态
        
        优化策略：
        1. 使用 ThreadPoolExecutor 并行执行 ping 检测
        2. 一次性批量更新 UI，减少信号发送次数
        3. 使用锁保护共享数据结构
        """
        try:
            # 调试日志：开始检测（仅详细模式）
            if self.verbose_logging:
                self._log("🔍 开始状态检测...")
            
            # 步骤 1: 获取所有 ROS 节点（单次检测）
            # 增加超时时间到 5 秒，确保在网络复杂时能获取完整列表
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            online_nodes = result.stdout.strip().split('\n') if result.returncode == 0 else []
            
            # 步骤 2: 并行检测所有主机的在线状态
            host_status = {}  # {hostname: is_online}
            
            futures = {}
            for usv_id, config in self.fleet_config.items():
                if not config.get('enabled', False):
                    continue
                
                hostname = config.get('hostname', '')
                if hostname and hostname not in host_status:
                    # 提交 ping 任务到线程池
                    future = self.executor.submit(self._check_host_online_fast, hostname)
                    futures[future] = hostname
            
            if self.verbose_logging:
                self._log(f"📡 提交 {len(futures)} 个 ping 任务")
            
            # 等待所有 ping 任务完成
            for future in as_completed(futures):
                hostname = futures[future]
                try:
                    is_online = future.result()
                    host_status[hostname] = is_online
                    if self.verbose_logging:
                        self._log(f"  Ping {hostname}: {'✅ 在线' if is_online else '❌ 离线'}")
                except Exception as e:
                    self._log(f"⚠️ {hostname} ping 失败: {e}")
                    host_status[hostname] = False
            
            # 步骤 3: 批量更新所有 USV 状态
            status_updates = {}  # {usv_id: new_status}
            
            if self.verbose_logging:
                self._log(f"📋 检查 {len(self.fleet_config)} 个 USV 状态")
            
            for usv_id, config in self.fleet_config.items():
                if not config.get('enabled', False):
                    if self.verbose_logging:
                        self._log(f"  ⏭️ {usv_id}: 已禁用，跳过")
                    continue
                
                namespace = f"/{usv_id}" if not usv_id.startswith('/') else usv_id
                # 兼容性处理：配置中可能没有写 namespace, 默认为 /usv_id
                # 但实际运行的节点通常带有 namespace 前缀
                
                hostname = config.get('hostname', '')
                
                # 检查节点是否在线
                # 优化匹配逻辑：只要节点路径中包含该 usv_id 即可
                # 例如 /usv_02/mavros 分割后包含 'usv_02'
                # 这比 startswith 更通用，且比简单的 in 更安全（防止 usv_1 匹配到 usv_10）
                has_nodes = False
                raw_id = usv_id.lstrip('/') # 确保是纯 ID
                
                # 双重检查：
                # 1. 尝试从 main_app 获取实时连接状态（最准）
                if self.parent() and hasattr(self.parent(), 'state_handler'):
                    try:
                       state = self.parent().state_handler.get_usv_state(usv_id)
                       if state and state.get('connected'):
                           has_nodes = True
                    except Exception:
                        pass
                
                # 2. 如果方法1没结果，再查 ros2 node list
                if not has_nodes:
                    for node in online_nodes:
                        # 移除开头的 / 并按 / 分割
                        parts = node.lstrip('/').split('/')
                        if raw_id in parts:
                            has_nodes = True
                            break
                        
                        # 兼容处理：有些节点可能命名为 /usv_02_driver (下划线连接)
                        # 检查是否有以 id 开头的部分
                        for part in parts:
                             if part == raw_id or part.startswith(f"{raw_id}_"):
                                 has_nodes = True
                                 break
                        if has_nodes: break
                
                # 检查是否有正在运行的启动进程
                has_process = (usv_id in self.usv_processes and 
                             self.usv_processes[usv_id].poll() is None)
                
                # 检查主机是否在线
                is_host_online = host_status.get(hostname, False)
                
                # 状态判断逻辑
                # 修改判断逻辑：如果检测到 ROS 节点，则无论是否有 usv_process，都优先认为是 running
                # 这解决了外部已启动（非本启动器启动）场景下的状态显示问题
                if has_nodes:
                    new_status = 'running'
                elif has_process:
                    new_status = 'launching'
                elif is_host_online:
                    # 如果主机在线且没有检测到节点，也没有启动进程，则为 only online (就绪)
                    new_status = 'online'
                else:
                    new_status = 'offline'
                
                # 仅记录状态变化
                with self.status_lock:
                    old_status = self.usv_status.get(usv_id)
                    
                    # 调试：总是输出状态信息（仅详细模式）
                    if self.verbose_logging:
                        self._log(f"  [{usv_id}] old={old_status}, new={new_status}, "
                             f"host={is_host_online}, nodes={has_nodes}")
                    
                    if old_status != new_status:
                        self.usv_status[usv_id] = new_status
                        status_updates[usv_id] = new_status
                        
                        # 输出状态变化日志（首次检测或状态改变）- 这个始终保留，因为是关键变化
                        # 修改：响应用户需求，默认隐藏详细的进程状态变化日志，避免刷屏
                        # 仅在调试模式(verbose_logging)下显示
                        if self.verbose_logging:
                            self._log(f"📊 {usv_id}: {old_status or '(首次)'} → {new_status} "
                                     f"[nodes={has_nodes}, proc={has_process}, host={is_host_online}]")
            
            # 步骤 4: 批量发送状态更新信号（减少信号数量）
            if status_updates:
                if self.verbose_logging:
                    self._log(f"🔄 发送批量状态更新: {len(status_updates)} 个 USV")
                self.batch_status_updated.emit(status_updates)
            elif self.verbose_logging:
                self._log("✅ 状态检测完成，无变化")
        
        except subprocess.TimeoutExpired:
            self._log("⚠️ ROS 节点检测超时")
        except Exception as e:
            self._log(f"⚠️ 状态检测失败: {e}")
    
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
        if self.verbose_logging:
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
            fcu_url = config['fcu_url']
            system_id = config['system_id']
            gcs_url = config.get('gcs_url', '')
            domain_id = config.get('domain_id', '0')
            
            # FastDDS 单播配置文件路径（USV端）
            fastdds_config = config.get('fastdds_config', '/home/chenhangwei/fastdds_usv.xml')
            
            remote_cmd = (
                f"bash -c '"
                f"export ROS_DOMAIN_ID={domain_id}; "
                f"export FASTDDS_DEFAULT_PROFILES_FILE={fastdds_config}; "
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
            
            # 检查是否配置了密码
            password = config.get('password', '')
            use_sshpass = False
            
            if password:
                import shutil
                if shutil.which('sshpass'):
                    use_sshpass = True
                else:
                    self._log(f"⚠️ {usv_id} 配置了密码但系统未安装 sshpass，请运行: sudo apt install sshpass")
            
            # 构建 SSH 命令
            base_ssh_cmd = [
                'ssh',
                '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=10',
                '-t',
                f'{username}@{hostname}',
                remote_cmd
            ]
            
            if use_sshpass:
                ssh_cmd = ['sshpass', '-p', password] + base_ssh_cmd
                self._log(f"🔑 使用 sshpass 自动输入密码")
            else:
                ssh_cmd = base_ssh_cmd
            
            # 启动进程并追踪
            process = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, # 将 stderr 合并到 stdout
                text=True,
                bufsize=1  # 行缓冲
            )
            
            # 调试：立即检查进程是否存活并读取错误输出
            time.sleep(0.5)
            if process.poll() is not None:
                # 进程立即退出，捕获错误
                stdout, stderr = process.communicate()
                self._log(f"❌ {usv_id} 启动进程立即退出 (Code {process.returncode})")
                if stderr:
                    self._log(f"📋 错误详情:\n{stderr.strip()}")
                self.status_updated.emit(usv_id, 'offline')
                with self.status_lock:
                    self.usv_status[usv_id] = 'offline'
                return

            # 如果设置了显示详细日志，打印完整命令
            if self.verbose_logging:
                masked_cmd = ' '.join(ssh_cmd).replace(password, '******') if password else ' '.join(ssh_cmd)
                self._log(f"🔧 执行命令: {masked_cmd}")

            # 追踪进程
            self.process_tracker.track(process, f'USV {usv_id} SSH Launch')
            
            self.usv_processes[usv_id] = process
            
            with self.status_lock:
                self.usv_status[usv_id] = 'launching'
            
            self.status_updated.emit(usv_id, 'launching')
            
            self._log(f"✅ {usv_id} 启动命令已发送 (PID: {process.pid})")
            
            # 启动输出流读取线程
            Thread(target=self._read_process_output, args=(usv_id, process), daemon=True).start()
        
        except Exception as e:
            self._log(f"❌ {usv_id} 启动失败: {e}")

    def _read_process_output(self, usv_id, process):
        """读取 SSH 进程输出"""
        try:
            # 循环读取 stdout 和 stderr
            while process.poll() is None:
                # 读取一行 stdout
                line = process.stdout.readline()
                if line:
                    line = line.strip()
                    if line:
                        # 过滤掉一些没什么用的 SSH 警告
                        if "Connection to" in line and "closed" in line:
                            continue
                        self._log(f"[{usv_id}] {line}")
                
                # 读取 stderr (非阻塞方式比较麻烦，这里简单处理，或者是读完 stdout 再读 stderr)
                # 由于是 readline，可能会阻塞。
                # 更好的方式是使用 select 或者两个线程，这里简化为只读 stdout 
                # 因为 Popen 不合并 stderr，我们暂时只关注 stdout，或者将 stderr合并到 stdout
                
                # 稍微休眠避免 CPU 占用过高
                # time.sleep(0.01) 
                
                # 在 python subprocess 中最好合并 stderr 到 stdout，或者使用 communicate
                # 但我们需要实时流。
                pass
            
            # 进程结束后，读取剩余输出
            stdout, stderr = process.communicate()
            if stdout:
                for line in stdout.split('\n'):
                    if line.strip(): self._log(f"[{usv_id}] {line.strip()}")
            if stderr:
                for line in stderr.split('\n'):
                    if line.strip(): self._log(f"[{usv_id} ERR] {line.strip()}")
                    
            # 检查返回码
            if process.returncode != 0:
                 self._log(f"⚠️ {usv_id} 进程退出，返回码: {process.returncode}")
                 self.status_updated.emit(usv_id, 'offline')
            else:
                 self._log(f"ℹ️ {usv_id} 进程已结束") # 通常 ssh 命令结束意味着远程程序结束

        except Exception as e:
            self._log(f"⚠️读取 {usv_id} 输出出错: {e}")
    
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
    
    def _perform_ssh_reboot(self, usv_id):
        """执行 SSH 重启操作"""
        config = self.fleet_config.get(usv_id, {})
        hostname = config.get('hostname')
        username = config.get('username')
        password = config.get('password')

        if not hostname or not username:
             self.log_message.emit(f"❌ {usv_id} 配置错误: 缺少 hostname 或 username")
             return

        self.log_message.emit(f"🔄 正在通过 SSH 重启 {usv_id} ({hostname})...")
        
        def run_reboot():
            success, msg = self.system_command_handler.reboot_usv(hostname, username, password)
            if success:
                 self.log_message.emit(f"✅ {usv_id} 重启命令已发送")
            else:
                 self.log_message.emit(f"❌ {usv_id} 重启失败: {msg}")

        Thread(target=run_reboot, daemon=True).start()

    def _reboot_single(self, usv_id):
        """重启单个 USV 的机载计算机"""
        reply = QMessageBox.question(
            self,
            "确认重启",
            f"确定要重启 {usv_id} 的机载计算机吗？\n\n"
            f"⚠️ 将使用 SSH 发送重启命令\n"
            f"⚠️ 重启后系统需要 30-60 秒恢复在线\n"
            f"⚠️ 所有运行中的节点将被终止",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self._perform_ssh_reboot(usv_id)
    
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
            "⚠️ 将使用 SSH (带密码) 发送重启命令\n"
            "⚠️ 重启后系统需要一些时间恢复",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self._log(f"🔄 批量重启: {', '.join(selected)}")
            for usv_id in selected:
                self._perform_ssh_reboot(usv_id)
                time.sleep(1)  # 间隔防止拥塞

    
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
