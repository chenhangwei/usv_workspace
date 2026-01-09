"""
Ground Station主窗口应用
重构后的版本,使用模块化设计
"""
import sys
import threading
import os
import yaml
import logging
import subprocess
from logging.handlers import RotatingFileHandler

# 模块级 logger
_logger = logging.getLogger("gs_gui.main")

import rclpy
from rclpy.parameter import Parameter
from PyQt5.QtCore import QProcess, QTimer, Qt, QEvent
from PyQt5.QtWidgets import (QApplication, QMainWindow, QAbstractItemView, 
                             QMessageBox, QAction, QDialog, QPushButton, 
                             QHBoxLayout, QSpacerItem, QSizePolicy,
                             QTableWidget, QTableWidgetItem, QHeaderView,
                             QMenu, QTabWidget, QWidget, QVBoxLayout,
                             QFrame, QLabel, QProgressBar)
from PyQt5.QtGui import QFont, QColor
from gs_gui.ros_signal import ROSSignal
from gs_gui.ground_station_node import GroundStationNode
from gs_gui.ui import Ui_MainWindow

# 导入新创建的模块
from gs_gui.table_manager import TableManager
from gs_gui.usv_commands import USVCommandHandler
from gs_gui.cluster_task_manager import ClusterTaskManager
from gs_gui.usv_list_manager import USVListManager
from gs_gui.state_handler import StateHandler
from gs_gui.ui_utils import UIUtils
from gs_gui.area_offset_dialog import AreaOffsetDialog
from gs_gui.usv_info_panel import UsvInfoPanel
from gs_gui.usv_navigation_panel import UsvNavigationPanel
from gs_gui.style_manager import StyleManager
# 使用性能优化版本的集群启动器（异步检测 + 并行 ping）
from gs_gui.usv_fleet_launcher_optimized import UsvFleetLauncher


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, ros_signal):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("Ground Station GUI")
        self.resize(1024, 512)
        self.setGeometry(100, 100, 1124, 612)

        # 允许通过鼠标拖拽调整左右区域宽度（QSplitter 分隔条）
        # 说明：UI 使用 mainSplitter（横向）承载 3 个区域；这里加宽 handle 并设置伸缩因子，
        # 让拖拽更容易、比例更符合预期。
        try:
            self.ui.mainSplitter.setOrientation(Qt.Horizontal)
            self.ui.mainSplitter.setChildrenCollapsible(False)
            self.ui.mainSplitter.setHandleWidth(10)
            self.ui.mainSplitter.setStretchFactor(0, 6)  # 左侧列表/控制区
            self.ui.mainSplitter.setStretchFactor(1, 3)  # USV details
            self.ui.mainSplitter.setStretchFactor(2, 3)  # message
        except Exception:
            pass

        # 更新按钮文本以匹配新的彩虹循环行为
        try:
            self.ui.led1_pushButton.setText("彩虹循环")
        except Exception:
            pass
        
        # 更新按钮文本为 OFFBOARD
        try:
            self.ui.set_guided_pushButton.setText("OFFBOARD")
            self.ui.set_departed_guided_pushButton.setText("OFFBOARD")
        except Exception:
            pass
            
        # 设置操作按钮不获取焦点，避免点击时清除表格选择
        try:
            self.ui.add_cluster_pushButton.setFocusPolicy(Qt.NoFocus)
            self.ui.quit_cluster_pushButton.setFocusPolicy(Qt.NoFocus)
        except Exception:
            pass
        
        self.ros_signal = ros_signal
        
        # 优雅关闭标志：避免重复发送关闭命令
        self._shutdown_commands_sent = False
        
        # 初始化样式管理器并加载现代化主题
        self.style_manager = StyleManager(self)
        self.style_manager.load_theme('modern_dark')
        
        # 设置全局字体大小（增大 emoji 显示）
        # 必须在 StyleManager 之后设置，以避免被主题覆盖
        # 可选值：9(默认小), 10(稍大), 11(中等), 12(较大), 13(大), 14(很大)
        from PyQt5.QtGui import QFont
        app_font = QFont()
        app_font.setPointSize(11)  # 从 13pt 缩小到 11pt
        QApplication.instance().setFont(app_font)
        
        # 初始化UI工具
        self.ui_utils = UIUtils(self.ui, self)

        # 初始化额外菜单
        self._init_custom_menu()
        
        # 初始化侧边栏选项卡（合并详情与导航）
        self._init_side_tab_panel()
        
        # 初始化导航反馈表格（替换原有的文本框）
        self._init_navigation_feedback_table()
        
        # 初始化消息栏右键菜单
        self._init_message_context_menus()
        
        # 初始化表格管理器
        self.table_manager = TableManager(
            self.ui.cluster_tableView,
            self.ui.departed_tableView
        )
        
        # 安装事件过滤器，处理失去焦点清除选择的问题
        self.ui.cluster_tableView.installEventFilter(self)
        self.ui.departed_tableView.installEventFilter(self)
        
        # 设置表格为单行选择模式
        self.ui.cluster_tableView.setSelectionMode(QAbstractItemView.SingleSelection)
        self.ui.departed_tableView.setSelectionMode(QAbstractItemView.SingleSelection)
        
        # 设置表格为行选择模式
        self.ui.cluster_tableView.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.ui.departed_tableView.setSelectionBehavior(QAbstractItemView.SelectRows)
        
        # 设置表格为只读
        self.ui.cluster_tableView.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.ui.departed_tableView.setEditTriggers(QAbstractItemView.NoEditTriggers)
        
        # 初始化USV列表管理器
        self.list_manager = USVListManager(self.ui_utils.append_info)
        
        # 初始化状态处理器（传入信息面板和导航面板更新回调）
        self.state_handler = StateHandler(
            self.table_manager,
            self.list_manager,
            self.ui_utils.append_warning,
            self._refresh_selected_usv_info,  # 传入USV信息面板更新回调
            self._refresh_selected_usv_navigation  # 传入USV导航面板更新回调
        )
        
        # 初始化命令处理器
        self.command_handler = USVCommandHandler(
            self.ros_signal,
            self.ui_utils.append_info
        )
        
        # 初始化集群任务管理器
        self.task_manager = ClusterTaskManager(
            self.ros_signal,
            self.ui_utils.append_info,
            self.ui_utils.append_warning,
            self
        )
        
        # 连接ROS信号
        self._connect_ros_signals()
        
        # 连接UI按钮信号
        self._connect_ui_signals()
        
        # 在初始化最后刷新表格表头
        self.table_manager.refresh_table_header()

    def eventFilter(self, source, event):
        """事件过滤器：处理特定的UI事件"""
        if event.type() == QEvent.FocusOut:
            if source == self.nav_feedback_table:
                # 当导航反馈表格失去焦点时，清除选择
                self.nav_feedback_table.clearSelection()
                self.nav_feedback_table.setCurrentItem(None)
            elif source == self.ui.cluster_tableView:
                # 当集群表格失去焦点时
                self.ui.cluster_tableView.clearSelection()
                self.ui.cluster_tableView.setCurrentIndex(self.ui.cluster_tableView.rootIndex())
            elif source == self.ui.departed_tableView:
                # 当离群表格失去焦点时
                self.ui.departed_tableView.clearSelection()
                self.ui.departed_tableView.setCurrentIndex(self.ui.departed_tableView.rootIndex())
                
        return super().eventFilter(source, event)
    
    def _connect_ros_signals(self):
        """连接ROS信号到处理函数"""
        # 状态更新信号
        self.ros_signal.receive_state_list.connect(self.state_handler.receive_state_callback)
        
        # 集群任务进度信号
        self.ros_signal.cluster_progress_update.connect(self._handle_cluster_progress_update)
        
        # 导航状态更新信号
        self.ros_signal.nav_status_update.connect(self.state_handler.update_nav_status)
        
        # 导航反馈信号（连接到 StateHandler 进行缓存）
        self.ros_signal.navigation_feedback.connect(self.state_handler.update_navigation_feedback)
        
        # 导航反馈信号（连接到主窗口进行日志显示）
        self.ros_signal.navigation_feedback.connect(self.handle_navigation_feedback)
    
    def _connect_ui_signals(self):
        """连接UI按钮信号到处理函数"""
        # ============== 集群控制按钮 ==============
        self.ui.arming_pushButton.clicked.connect(self.set_cluster_arming_command)
        self.ui.disarming_pushButton.clicked.connect(self.cluster_disarming_command)
        self.ui.set_guided_pushButton.clicked.connect(self.set_cluster_offboard_command)
        self.ui.set_manual_pushButton.clicked.connect(self.set_cluster_hold_command)
        self.ui.set_stabilized_pushButton.clicked.connect(self.set_cluster_stabilized_command)
        self.ui.send_cluster_point_pushButton.clicked.connect(self.toggle_cluster_task)
        self.ui.stop_cluster_task_pushButton.clicked.connect(self.stop_cluster_task)
        
        # ============== 离群控制按钮 ==============
        self.ui.departed_arming_pushButton.clicked.connect(self.departed_arming_command)
        self.ui.departed_disarming_pushButton.clicked.connect(self.departed_disarming_command)
        self.ui.set_departed_guided_pushButton.clicked.connect(self.set_departed_offboard_command)
        self.ui.set_departed_manual_pushButton.clicked.connect(self.set_departed_stabilized_command)
        self.ui.set_departed_ARCO_pushButton.clicked.connect(self.set_departed_posctl_command)
        self.ui.set_departed_Steering_pushButton.clicked.connect(self.set_departed_altctl_command)
        self.ui.send_departed_point_pushButton.clicked.connect(self.send_departed_point_command)
        
        # ============== 集群列表管理按钮 ==============
        self.ui.add_cluster_pushButton.clicked.connect(self.add_cluster_command)
        self.ui.quit_cluster_pushButton.clicked.connect(self.quit_cluster_command)
        
        # ============== 声音和颈部按钮 ==============
        self.ui.sound_start_pushButton.clicked.connect(self.command_handler.sound_start)
        self.ui.sound_stop_pushButton.clicked.connect(self.command_handler.sound_stop)
        self.ui.neck_swinging_pushButton.clicked.connect(self.command_handler.neck_swinging)
        self.ui.neck_stop_pushButton.clicked.connect(self.command_handler.neck_stop)
        
        # ============== LED按钮 ==============
        self.ui.led1_pushButton.clicked.connect(self.toggle_led_rainbow_cycle)
        self.ui.led2_pushButton.clicked.connect(self.trigger_led_random_color)
        self.ui.led3_pushButton.clicked.connect(self.trigger_led_select_color)
        self.ui.light_stop_pushButton.clicked.connect(self.stop_all_led_effects)
        
        # ============== 菜单操作 ==============
        self.ui.actionopen.triggered.connect(self.task_manager.read_data_from_file)
        self.ui.actionrviz2.triggered.connect(self.ui_utils.start_rviz)
        
        # ============== 表格选择信号 ==============
        # 连接集群表格和离群表格的选择改变信号
        self.ui.cluster_tableView.selectionModel().selectionChanged.connect(
            lambda: self.update_usv_info_display(is_cluster=True)
        )
        self.ui.departed_tableView.selectionModel().selectionChanged.connect(
            lambda: self.update_usv_info_display(is_cluster=False)
        )
        self.ui.cluster_tableView.clicked.connect(
            lambda index: self._handle_table_clicked(index, is_cluster=True)
        )
        self.ui.departed_tableView.clicked.connect(
            lambda index: self._handle_table_clicked(index, is_cluster=False)
        )
        self.ui.action3D.triggered.connect(self.show_usv_plot_window)
        self.action_launch_usv_fleet.triggered.connect(self.launch_usv_fleet)
        self.action_set_area_offset.triggered.connect(self.set_area_offset_command)
        self.action_led_infection_mode.triggered.connect(self.toggle_led_infection_mode)
        self.action_set_home.triggered.connect(self.open_set_home_dialog)
        self.action_param_config.triggered.connect(self.open_param_config_window)

    def _init_custom_menu(self):
        """在菜单栏中增加坐标偏移设置入口、LED传染模式开关和工具菜单"""
        # USV控制菜单
        usv_menu = self.ui.menubar.addMenu("USV控制(&U)")
        self.action_launch_usv_fleet = QAction("🚀 启动 USV 集群", self)
        self.action_launch_usv_fleet.setShortcut("Ctrl+L")
        self.action_launch_usv_fleet.setToolTip("通过分布式 launch 启动所有 USV 节点")
        usv_menu.addAction(self.action_launch_usv_fleet)
        
        # 坐标系设置菜单
        coord_menu = self.ui.menubar.addMenu("坐标系设置")
        self.action_set_area_offset = QAction("设置任务坐标系偏移量", self)
        coord_menu.addAction(self.action_set_area_offset)
        
        # LED设置菜单
        led_menu = self.ui.menubar.addMenu("LED设置")
        self.action_led_infection_mode = QAction("LED传染模式", self)
        self.action_led_infection_mode.setCheckable(True)
        self.action_led_infection_mode.setChecked(False)  # 默认关闭
        led_menu.addAction(self.action_led_infection_mode)
        
        # 随机运行菜单
        random_run_menu = self.ui.menubar.addMenu("随机运行")
        self.action_random_run_mode = QAction("🎲 开启随机运行模式", self)
        self.action_random_run_mode.setCheckable(True)
        self.action_random_run_mode.setChecked(False)
        self.action_random_run_mode.triggered.connect(self._toggle_random_run_mode)
        random_run_menu.addAction(self.action_random_run_mode)
        
        # 工具菜单
        tools_menu = self.ui.menubar.addMenu("工具(&T)")
        
        # Home Position 设置
        self.action_set_home = QAction("🏠 设置 Home Position", self)
        self.action_set_home.setShortcut("Ctrl+H")
        self.action_set_home.setToolTip("设置 USV 的 Home Position（RTL 返航点）")
        tools_menu.addAction(self.action_set_home)
        
        # 分隔线
        tools_menu.addSeparator()
        
        # 飞控参数配置
        self.action_param_config = QAction("[+] 飞控参数配置...", self)
        self.action_param_config.setShortcut("Ctrl+P")
        self.action_param_config.setToolTip("通过串口直连配置飞控参数")
        tools_menu.addAction(self.action_param_config)
    
    def _init_side_tab_panel(self):
        """初始化侧边栏选项卡，合并 USV 详情、导航、反馈及日志"""
        # 1. 创建选项卡控件
        self.side_tab_widget = QTabWidget()
        self.side_tab_widget.setTabPosition(QTabWidget.North)
        self.side_tab_widget.setDocumentMode(True)  # 扁平化设计
        
        # 2. 创建并添加详情面板
        self.usv_info_panel = UsvInfoPanel()
        self.side_tab_widget.addTab(self.usv_info_panel, "📋 详情")
        
        # 3. 创建并添加导航面板
        self.usv_navigation_panel = UsvNavigationPanel()
        self.side_tab_widget.addTab(self.usv_navigation_panel, "🧭 导航")

        # 4. 添加导航反馈页
        self.nav_feedback_container = QWidget()
        self.nav_feedback_layout = QVBoxLayout(self.nav_feedback_container)
        self.nav_feedback_layout.setContentsMargins(5, 5, 5, 5)
        self.nav_feedback_layout.setSpacing(5)

        # --- 新增：集群任务进度仪表盘 (科幻风格) ---
        self.mission_dashboard = QFrame()
        self.mission_dashboard.setObjectName("missionDashboard")
        self.mission_dashboard.setStyleSheet("""
            QFrame#missionDashboard {
                background-color: #0a192f;
                border: 1px solid #00f2ff;
                border-radius: 4px;
                padding: 8px;
            }
            QLabel {
                color: #00f2ff;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 12pt;
            }
            .dashValue {
                color: #ffffff;
                font-weight: bold;
            }
        """)
        
        dash_main_layout = QVBoxLayout(self.mission_dashboard)
        dash_main_layout.setContentsMargins(5, 5, 5, 5)
        
        # 第一行：标题和状态
        header_layout = QHBoxLayout()
        title_label = QLabel(" MISSION STATUS ")
        title_label.setStyleSheet("background-color: #00f2ff; color: #0a192f; font-weight: bold; padding: 2px;")
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        self.dash_status_label = QLabel("IDLE")
        self.dash_status_label.setStyleSheet("color: #95a5a6; font-weight: bold;")
        header_layout.addWidget(self.dash_status_label)
        dash_main_layout.addLayout(header_layout)
        
        # 第二行：核心数据
        stats_layout = QHBoxLayout()
        self.dash_step_label = QLabel("STEP: <span class='dashValue'>--/--</span>")
        self.dash_units_label = QLabel("UNITS: <span class='dashValue'>--/--</span>")
        self.dash_time_label = QLabel("TIME: <span class='dashValue'>0.0s</span>")
        
        stats_layout.addWidget(self.dash_step_label)
        stats_layout.addStretch()
        stats_layout.addWidget(self.dash_units_label)
        stats_layout.addStretch()
        stats_layout.addWidget(self.dash_time_label)
        dash_main_layout.addLayout(stats_layout)
        
        # 第三行：进度条
        self.mission_progress_bar = QProgressBar()
        self.mission_progress_bar.setRange(0, 100)
        self.mission_progress_bar.setValue(0)
        self.mission_progress_bar.setTextVisible(True)
        self.mission_progress_bar.setFormat("MISSION PROGRESS: %p%")
        self.mission_progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #00f2ff;
                background-color: #05101e;
                height: 22px;
                text-align: center;
                color: #00f2ff;
                font-weight: bold;
                font-size: 10pt;
                border-radius: 2px;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                                  stop:0 #00d2ff, stop:0.5 #0072ff, stop:1 #00d2ff);
                width: 20px;
                margin: 1px;
            }
        """)
        dash_main_layout.addWidget(self.mission_progress_bar)
        
        self.nav_feedback_layout.addWidget(self.mission_dashboard)
        
        self.side_tab_widget.addTab(self.nav_feedback_container, "📊 反馈")

        # 5. 添加信息日志页
        self.side_tab_widget.addTab(self.ui.info_textEdit, "ℹ️ 信息")

        # 6. 添加警告日志页
        self.side_tab_widget.addTab(self.ui.warning_textEdit, "⚠️ 警告")
        
        # 7. 将选项卡控件放入 groupBox_usv_details
        # 清除 groupBox_usv_details 原有的布局内容
        layout = self.ui.verticalLayout_usv_details
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        
        layout.addWidget(self.side_tab_widget)
        self.ui.groupBox_usv_details.setTitle("系统监控与反馈")

        # 8. 隐藏原有的消息区域 groupBox_2
        self.ui.groupBox_2.hide()
        
        # 9. 调整主分隔条比例
        # 现在主分隔条只有 2 个主要部分：左侧列表(0)、右侧综合监控(1)
        main_splitter = self.ui.mainSplitter
        main_splitter.setStretchFactor(0, 3)  # 左侧列表/控制区
        main_splitter.setStretchFactor(1, 7)  # 右侧综合监控选项卡

    def _init_navigation_feedback_table(self):
        """初始化导航反馈表格，采用科幻风格设计"""
        self.nav_feedback_table = QTableWidget()
        # 安装事件过滤器以处理焦点丢失时的选择清除
        self.nav_feedback_table.installEventFilter(self)
        self.nav_feedback_table.setColumnCount(6)
        self.nav_feedback_table.setHorizontalHeaderLabels(["STATUS", "USV ID", "TARGET", "DISTANCE", "HEADING ERR", "ETA"])
        
        # 设置表头自适应
        header = self.nav_feedback_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents) # Status 宽度自适应
        
        # 设置表格属性
        self.nav_feedback_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.nav_feedback_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.nav_feedback_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.nav_feedback_table.verticalHeader().setVisible(False)
        self.nav_feedback_table.verticalHeader().setDefaultSectionSize(40)
        self.nav_feedback_table.setShowGrid(False)
        self.nav_feedback_table.setAlternatingRowColors(True)
        
        # 科幻风格 QSS
        self.nav_feedback_table.setStyleSheet("""
            QTableWidget {
                background-color: #1a1a1a;
                alternate-background-color: #222222;
                color: #e0e0e0;
                gridline-color: transparent;
                border: none;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 10pt;
            }
            QTableWidget::item {
                padding: 5px;
            }
            QTableWidget::item:hover {
                background-color: rgba(0, 242, 255, 0.05);
            }
            QTableWidget::item:selected {
                background-color: rgba(0, 242, 255, 0.15);
                color: #00f2ff;
                border-left: 2px solid #00f2ff;
            }
            QHeaderView::section {
                background-color: #0d1b2a;
                color: #00f2ff;
                padding: 8px;
                border: none;
                border-bottom: 1px solid #00f2ff;
                font-weight: bold;
                text-transform: uppercase;
                font-size: 9pt;
            }
        """)
        
        # 将表格放入选项卡的反馈页布局中
        if hasattr(self, 'nav_feedback_layout'):
            self.nav_feedback_layout.addWidget(self.nav_feedback_table)
            
        # 用于存储 usv_id 到行索引的映射
        self._nav_feedback_row_map = {}

    def _clear_nav_feedback_table(self):
        """清除导航反馈表格"""
        if hasattr(self, 'nav_feedback_table'):
            self.nav_feedback_table.setRowCount(0)
            self._nav_feedback_row_map = {}

    def _remove_nav_feedback_row(self, usv_id):
        """移除指定USV的导航反馈行"""
        if usv_id in self._nav_feedback_row_map:
            row = self._nav_feedback_row_map[usv_id]
            self.nav_feedback_table.removeRow(row)
            del self._nav_feedback_row_map[usv_id]
            
            # 更新其他行的映射索引
            for uid, r in self._nav_feedback_row_map.items():
                if r > row:
                    self._nav_feedback_row_map[uid] = r - 1

    def _init_message_context_menus(self):
        """为消息栏的窗口添加右键菜单清除功能"""
        # 1. 导航反馈表格
        self.nav_feedback_table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.nav_feedback_table.customContextMenuRequested.connect(
            lambda pos: self._show_clear_context_menu(pos, self.nav_feedback_table, self._clear_nav_feedback_table)
        )
        
        # 2. 信息窗口
        self.ui.info_textEdit.setContextMenuPolicy(Qt.CustomContextMenu)
        self.ui.info_textEdit.customContextMenuRequested.connect(
            lambda pos: self._show_clear_context_menu(pos, self.ui.info_textEdit, self.ui_utils.clear_info)
        )
        
        # 3. 警告窗口
        self.ui.warning_textEdit.setContextMenuPolicy(Qt.CustomContextMenu)
        self.ui.warning_textEdit.customContextMenuRequested.connect(
            lambda pos: self._show_clear_context_menu(pos, self.ui.warning_textEdit, self.ui_utils.clear_warning)
        )

    def _show_clear_context_menu(self, pos, widget, clear_callback):
        """显示清除右键菜单"""
        menu = QMenu(widget)
        clear_action = menu.addAction("清除内容")
        clear_action.triggered.connect(clear_callback)
        menu.exec_(widget.mapToGlobal(pos))

    # ============== 集群命令包装方法 ==============
    def set_cluster_arming_command(self):
        """集群解锁命令（带防抖）"""
        # 防抖：2秒内只允许一次 arm 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_arm_time'):
            self._last_arm_time = 0
        if now - self._last_arm_time < 2.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 2 秒后再试")
            return
        self._last_arm_time = now
        self.command_handler.set_cluster_arming(self.list_manager.usv_cluster_list)
    
    def cluster_disarming_command(self):
        """集群加锁命令（带防抖）"""
        # 防抖：2秒内只允许一次 disarm 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_disarm_time'):
            self._last_disarm_time = 0
        if now - self._last_disarm_time < 2.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 2 秒后再试")
            return
        self._last_disarm_time = now
        self.command_handler.cluster_disarming(self.list_manager.usv_cluster_list)
    
    def set_cluster_offboard_command(self):
        """集群设置OFFBOARD模式（带防抖）"""
        # 防抖：1秒内只允许一次 offboard 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_offboard_time'):
            self._last_offboard_time = 0
        if now - self._last_offboard_time < 1.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 1 秒后再试")
            return
        self._last_offboard_time = now
        self.command_handler.set_cluster_offboard(self.list_manager.usv_cluster_list)
    
    def set_cluster_hold_command(self):
        """集群设置HOLD模式（带防抖）"""
        # 防抖：1秒内只允许一次 HOLD 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_hold_time'):
            self._last_hold_time = 0
        if now - self._last_hold_time < 1.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 1 秒后再试")
            return
        self._last_hold_time = now
        self.command_handler.set_cluster_hold(self.list_manager.usv_cluster_list)
    
    def set_cluster_stabilized_command(self):
        """集群设置STABILIZED稳定模式（带防抖）"""
        # 防抖：1秒内只允许一次 STABILIZED 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_stabilized_time'):
            self._last_stabilized_time = 0
        if now - self._last_stabilized_time < 1.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 1 秒后再试")
            return
        self._last_stabilized_time = now
        self.command_handler.set_cluster_stabilized(self.list_manager.usv_cluster_list)
    
    # ============== 离群命令包装方法 ==============
    def departed_arming_command(self):
        """离群解锁命令（带防抖）"""
        # 防抖：2秒内只允许一次 arm 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_departed_arm_time'):
            self._last_departed_arm_time = 0
        if now - self._last_departed_arm_time < 2.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 2 秒后再试")
            return
        self._last_departed_arm_time = now
        self.command_handler.departed_arming(self.list_manager.usv_departed_list)
    
    def departed_disarming_command(self):
        """离群加锁命令（带防抖）"""
        # 防抖：2秒内只允许一次 disarm 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_departed_disarm_time'):
            self._last_departed_disarm_time = 0
        if now - self._last_departed_disarm_time < 2.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 2 秒后再试")
            return
        self._last_departed_disarm_time = now
        self.command_handler.departed_disarming(self.list_manager.usv_departed_list)
    
    def set_departed_offboard_command(self):
        """离群设置OFFBOARD模式（带防抖）"""
        # 防抖：1秒内只允许一次 offboard 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_departed_offboard_time'):
            self._last_departed_offboard_time = 0
        if now - self._last_departed_offboard_time < 1.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 1 秒后再试")
            return
        self._last_departed_offboard_time = now
        self.command_handler.set_departed_offboard(self.list_manager.usv_departed_list)
    
    def set_departed_stabilized_command(self):
        """离群设置STABILIZED稳定模式（带防抖）"""
        # 防抖：1秒内只允许一次 stabilized 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_departed_stabilized_time'):
            self._last_departed_stabilized_time = 0
        if now - self._last_departed_stabilized_time < 1.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 1 秒后再试")
            return
        self._last_departed_stabilized_time = now
        self.command_handler.set_departed_stabilized(self.list_manager.usv_departed_list)
    
    def set_departed_posctl_command(self):
        """离群设置POSCTL位置控制模式"""
        self.command_handler.set_departed_posctl(self.list_manager.usv_departed_list)
    
    def set_departed_altctl_command(self):
        """离群设置ALTCTL高度控制模式"""
        self.command_handler.set_departed_altctl(self.list_manager.usv_departed_list)
    
    # ============== 集群任务控制 ==============
    def toggle_cluster_task(self):
        """切换集群任务运行状态"""
        button_text = self.task_manager.toggle_task(
            self.list_manager.usv_departed_list,
            self.list_manager.usv_cluster_list
        )
        self.ui.send_cluster_point_pushButton.setText(button_text)
    
    def stop_cluster_task(self):
        """停止集群任务并刷新按钮文本"""
        self.task_manager.stop_task()
        self.ui.send_cluster_point_pushButton.setText(self.task_manager.get_button_text())
        
        # 清除导航反馈表格
        self._clear_nav_feedback_table()

    def _handle_cluster_progress_update(self, progress_info):
        """处理集群任务进度更新并同步按钮文本"""
        self.task_manager.update_progress(progress_info)
        self.ui.send_cluster_point_pushButton.setText(self.task_manager.get_button_text())
        
        # 更新科幻仪表盘
        self._update_mission_dashboard(progress_info)

    def _update_mission_dashboard(self, progress_info):
        """更新科幻风格的任务进度仪表盘"""
        if not hasattr(self, 'mission_dashboard'):
            return
            
        current_step = progress_info.get('current_step', 0)
        total_steps = progress_info.get('total_steps', 0)
        total_usvs = progress_info.get('total_usvs', 0)
        acked_usvs = progress_info.get('acked_usvs', 0)
        ack_rate = progress_info.get('ack_rate', 0.0)
        elapsed_time = progress_info.get('elapsed_time', 0.0)
        state = progress_info.get('state', 'idle')
        
        # 更新标签
        self.dash_step_label.setText(f"STEP: <span class='dashValue'>{current_step}/{total_steps}</span>")
        self.dash_units_label.setText(f"UNITS: <span class='dashValue'>{acked_usvs}/{total_usvs}</span>")
        self.dash_time_label.setText(f"TIME: <span class='dashValue'>{elapsed_time:.1f}s</span>")
        
        # 更新进度条
        self.mission_progress_bar.setValue(int(ack_rate * 100))
        
        # 更新状态样式
        state_map = {
            'running': ('ACTIVE', '#00f2ff'),
            'paused': ('PAUSED', '#f1c40f'),
            'completed': ('COMPLETED', '#2ecc71'),
            'idle': ('IDLE', '#95a5a6'),
            'failed': ('FAILED', '#e74c3c')
        }
        label_text, color = state_map.get(state, ('UNKNOWN', '#95a5a6'))
        self.dash_status_label.setText(label_text)
        self.dash_status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
        
        # 如果是运行中，给仪表盘边框加个呼吸灯效果（简单实现：切换边框颜色）
        if state == 'running':
            self.mission_dashboard.setStyleSheet(self.mission_dashboard.styleSheet().replace("border: 1px solid #00f2ff;", "border: 2px solid #00f2ff;"))
        else:
            self.mission_dashboard.setStyleSheet(self.mission_dashboard.styleSheet().replace("border: 2px solid #00f2ff;", "border: 1px solid #00f2ff;"))

    # ============== 离群目标点命令 ==============
    def send_departed_point_command(self):
        """发送离群目标点命令"""
        x = self.ui.set_departed_x_doubleSpinBox.value()
        y = self.ui.set_departed_y_doubleSpinBox.value()
        z = 0.0
        if hasattr(self.ui, 'set_departed_z_doubleSpinBox'):
            z = self.ui.set_departed_z_doubleSpinBox.value()
        
        departed_target_list = []
        for usv_item in self.list_manager.usv_departed_list:
            if isinstance(usv_item, dict):
                usv_id = usv_item.get('namespace')
                if usv_id:
                    self.ros_signal.nav_status_update.emit(usv_id, "执行中")
                    departed_target_list.append({
                        'usv_id': usv_id,
                        'position': {'x': x, 'y': y, 'z': z},
                        'yaw': 0.0
                    })
        
        self.ros_signal.departed_target_point_command.emit(departed_target_list)
        self.ui_utils.append_info(f"发送离群目标点: x={x}, y={y}, z={z} 到 {len(departed_target_list)} 个USV")
    
    # ============== 集群列表管理 ==============
    def add_cluster_command(self):
        """将选中的离群USV添加到集群列表"""
        usv_info = self.table_manager.get_selected_usv_info(is_cluster=False)
        if usv_info:
            if self.list_manager.add_to_cluster(usv_info):
                # 恢复该 USV 的集群任务资格（如果之前被排除）
                usv_id = usv_info['namespace']
                self.ros_signal.str_command.emit(f"INCLUDE_CLUSTER:{usv_id}")
                
                # 更新表格显示
                self.table_manager.update_cluster_table(
                    self.list_manager.usv_cluster_list,
                    self.state_handler.usv_nav_status
                )
                self.table_manager.update_departed_table(
                    self.list_manager.usv_departed_list,
                    self.state_handler.usv_nav_status
                )
                # 移除确认对话框，直接在info窗口输出
                self.ui_utils.append_info(f"✅ 设备 {usv_info['namespace']} 已添加到集群列表")
        else:
            self.ui_utils.append_info("请先选择一行")
    
    def quit_cluster_command(self):
        """将选中的集群USV移到离群列表"""
        usv_info = self.table_manager.get_selected_usv_info(is_cluster=True)
        if usv_info:
            usv_id = usv_info['namespace']
            
            # 1. 发送 HOLD 模式指令，确保物理静止
            self.ui_utils.append_info(f"🚦 正在将 {usv_id} 切换为 HOLD 模式 (集群脱离)")
            # set_cluster_hold 期望的是一个列表，所以这里转换为列表
            self.command_handler.set_cluster_hold([usv_info])
            
            # 2. 发送排除命令，确保节点逻辑不再等待/控制该 USV
            self.ros_signal.str_command.emit(f"EXCLUDE_CLUSTER:{usv_id}")

            # 3. 从导航反馈列表中移除该 USV
            self._remove_nav_feedback_row(usv_id)

            if self.list_manager.remove_from_cluster(usv_info):
                # 更新表格显示
                self.table_manager.update_cluster_table(
                    self.list_manager.usv_cluster_list,
                    self.state_handler.usv_nav_status
                )
                self.table_manager.update_departed_table(
                    self.list_manager.usv_departed_list,
                    self.state_handler.usv_nav_status
                )
                # 移除确认对话框，直接在info窗口输出
                self.ui_utils.append_info(f"✅ 设备 {usv_info['namespace']} 已添加到离群列表")
        else:
            self.ui_utils.append_info("请先选择一行")
    
    # ============== USV 集群启动 ==============
    def launch_usv_fleet(self):
        """启动 USV 集群启动器对话框"""
        try:
            # 查找工作空间路径
            current_file = os.path.abspath(__file__)
            search_path = current_file
            workspace_path = None
            
            for _ in range(10):
                search_path = os.path.dirname(search_path)
                if os.path.basename(search_path) == 'install':
                    workspace_path = os.path.dirname(search_path)
                    break
            
            if workspace_path is None:
                workspace_path = os.path.expanduser('~/usv_workspace')
            
            # 打开 USV 集群启动器对话框（非模态）
            # 如果已经打开，则激活窗口
            if hasattr(self, '_usv_fleet_launcher') and self._usv_fleet_launcher is not None:
                # 窗口已存在，激活并置顶
                self._usv_fleet_launcher.raise_()
                self._usv_fleet_launcher.activateWindow()
            else:
                # 创建新窗口（非模态）
                self._usv_fleet_launcher = UsvFleetLauncher(self, workspace_path)
                self._usv_fleet_launcher.show()  # 使用 show() 而非 exec_()，允许同时操作主界面
            
        except Exception as e:
            self.ui_utils.append_info(f"❌ 打开 USV 集群启动器失败: {e}")
            QMessageBox.critical(
                self,
                "错误",
                f"打开 USV 集群启动器时发生错误:\n{e}"
            )
    
    # ============== 坐标系设置命令 ==============
    def set_area_offset_command(self):
        """设置任务坐标系偏移量（Area Center）与围栏范围"""
        try:
            # 获取当前的偏移量
            current_offset = getattr(self.ros_node, '_area_center', {'x': 0.0, 'y': 0.0, 'z': 0.0})
            
            # 获取当前围栏配置
            current_fence = {
                'type': self.ros_node.get_parameter('fence_type').value,
                'radius': self.ros_node.get_parameter('fence_radius').value,
                'length': self.ros_node.get_parameter('fence_length').value,
                'width': self.ros_node.get_parameter('fence_width').value,
                'height': self.ros_node.get_parameter('fence_height').value
            }
            
            # 显示对话框
            dialog = AreaOffsetDialog(self, current_offset, current_fence)
            if dialog.exec_() == QDialog.Accepted:
                new_offset, new_fence = dialog.get_config()
                
                # 发送更新信号到ROS节点
                self.ros_signal.update_area_center.emit(new_offset)
                self.ros_signal.update_fence_config.emit(new_fence)
                
                self.ui_utils.append_info(
                    f"已更新任务区域配置: AreaCenter({new_offset['x']:.1f}, {new_offset['y']:.1f}), "
                    f"Fence({'圆柱' if new_fence['type']==0 else '长方体'})"
                )
        except Exception as e:
            self.ui_utils.append_info(f"设置坐标偏移量或围栏时发生错误: {e}")
    
    def get_selected_usv_position(self):
        """
        获取当前选中USV的位置信息
        供AreaOffsetDialog调用以实现一键获取当前USV位置
        
        Returns:
            dict: 包含位置信息的字典 {'x': float, 'y': float, 'z': float, 'usv_id': str}
                  如果没有选中USV或无法获取位置，返回None
        """
        try:
            # 先尝试从集群表格获取选中USV
            usv_info = self.table_manager.get_selected_usv_info(is_cluster=True)
            
            # 如果集群表格没有选中，再尝试从离群表格获取
            if usv_info is None:
                usv_info = self.table_manager.get_selected_usv_info(is_cluster=False)
            
            # 如果两个表格都没有选中，返回None
            if usv_info is None:
                return None
            
            usv_id = usv_info.get('namespace')
            if not usv_id:
                return None
            
            # 从state_handler的缓存中获取USV完整状态信息
            usv_state = self.state_handler._usv_state_cache.get(usv_id)
            if usv_state is None:
                return None
            
            # 提取位置信息
            position = usv_state.get('position', {})
            if not isinstance(position, dict):
                return None
            
            return {
                'x': position.get('x', 0.0),
                'y': position.get('y', 0.0),
                'z': position.get('z', 0.0),
                'usv_id': usv_id
            }
            
        except Exception as e:
            self.ui_utils.append_warning(f"获取USV位置失败: {e}")
            return None
    
    # ============== LED传染模式开关 ==============
    def toggle_led_infection_mode(self):
        """切换LED传染模式开关"""
        is_enabled = self.action_led_infection_mode.isChecked()
        self.ros_signal.led_infection_mode_changed.emit(is_enabled)
        status_text = "已开启" if is_enabled else "已关闭"
        # 移除确认对话框，直接在info窗口输出
        self.ui_utils.append_info(f"✅ LED传染模式{status_text}")

    def toggle_led_rainbow_cycle(self):
        """切换LED彩虹循环并更新按钮文本"""
        is_active = self.command_handler.led_color_switching()
        new_label = "停止彩虹" if is_active else "彩虹循环"
        self.ui.led1_pushButton.setText(new_label)

    def trigger_led_random_color(self):
        """触发随机颜色并确保停止彩虹循环"""
        self.command_handler.led_random_color()
        self.ui.led1_pushButton.setText("彩虹循环")

    def trigger_led_select_color(self):
        """弹出颜色选择器并确保停止彩虹循环"""
        self.command_handler.led_select_color(self)
        self.ui.led1_pushButton.setText("彩虹循环")

    def stop_all_led_effects(self):
        """停止所有LED效果并重置按钮文本"""
        self.command_handler.led_off()
        self.ui.led1_pushButton.setText("彩虹循环")
    
    # ============== 导航反馈处理 ==============
    def handle_navigation_feedback(self, usv_id, feedback):
        """
        处理导航反馈信息，更新到表格中（科幻增强版 V2）
        """
        # Feature: 如果USV在离群列表中，不显示反馈，并从表格中移除
        is_departed = any(u.get('namespace') == usv_id for u in self.list_manager.usv_departed_list)
        if is_departed:
            if usv_id in self._nav_feedback_row_map:
                self._remove_nav_feedback_row(usv_id)
            return

        # 检查是否已有该 USV 的行
        if usv_id not in self._nav_feedback_row_map:
            row = self.nav_feedback_table.rowCount()
            self.nav_feedback_table.insertRow(row)
            self._nav_feedback_row_map[usv_id] = row
            
            # 0. 状态 (STATUS) - 初始为等待
            status_item = QTableWidgetItem("●")
            status_item.setTextAlignment(Qt.AlignCenter)
            status_item.setForeground(QColor("#ff9800")) # 橙色
            self.nav_feedback_table.setItem(row, 0, status_item)
            
            # 1. ID
            id_item = QTableWidgetItem(usv_id)
            id_item.setTextAlignment(Qt.AlignCenter)
            id_item.setForeground(QColor("#00f2ff"))
            self.nav_feedback_table.setItem(row, 1, id_item)
        
        row = self._nav_feedback_row_map[usv_id]
        dist = feedback.distance_to_goal
        abs_err = abs(feedback.heading_error)
        
        # 更新状态颜色
        status_item = self.nav_feedback_table.item(row, 0)
        if dist < 1.5:
            status_item.setText("✔")
            status_item.setForeground(QColor("#4caf50")) # 绿色
        elif abs_err > 30.0:
            status_item.setText("⚠")
            status_item.setForeground(QColor("#f44336")) # 红色
        else:
            status_item.setText("●")
            status_item.setForeground(QColor("#00f2ff")) # 青色
            
        # 2. 目标ID (TARGET)
        # 优先显示 Step 步骤号，如果是单点导航(step=0)则显示 Goal ID
        step_val = getattr(feedback, 'step', 0)
        if step_val > 0:
            display_text = f"Step-{step_val}"
        else:
            display_text = f"T-{feedback.goal_id:02d}"
            
        goal_item = QTableWidgetItem(display_text)
        goal_item.setTextAlignment(Qt.AlignCenter)
        self.nav_feedback_table.setItem(row, 2, goal_item)
        
        # 3. 距离 (DISTANCE) - 使用进度条展示接近程度
        # 假设 30m 为满量程，越近进度条越满
        max_dist = 30.0
        progress_val = int(max(0, min(100, (1.0 - dist / max_dist) * 100)))
        
        bar = self.nav_feedback_table.cellWidget(row, 3)
        if not isinstance(bar, QProgressBar):
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setTextVisible(True)
            bar.setStyleSheet("""
                QProgressBar {
                    border: 1px solid #333;
                    border-radius: 2px;
                    background-color: #0a0a0a;
                    text-align: center;
                    color: #ffffff;
                    font-size: 8pt;
                    height: 16px;
                }
                QProgressBar::chunk {
                    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #004e92, stop:1 #00f2ff);
                }
            """)
            self.nav_feedback_table.setCellWidget(row, 3, bar)
        
        bar.setValue(progress_val)
        bar.setFormat(f"{dist:.1f}m")
        
        # 4. 航向误差 (HEADING ERR)
        dir_sym = "◀" if feedback.heading_error > 0 else "▶"
        if abs_err < 5.0: dir_sym = "◈"
        
        yaw_item = QTableWidgetItem(f"{dir_sym} {abs_err:.1f}°")
        yaw_item.setTextAlignment(Qt.AlignCenter)
        if abs_err > 30.0:
            yaw_item.setForeground(QColor("#f44336"))
        elif abs_err > 15.0:
            yaw_item.setForeground(QColor("#ff9800"))
        else:
            yaw_item.setForeground(QColor("#4caf50"))
        self.nav_feedback_table.setItem(row, 4, yaw_item)
        
        # 5. ETA
        eta = feedback.estimated_time
        eta_str = f"{int(eta)}s" if eta > 0 else "--"
        eta_item = QTableWidgetItem(eta_str)
        eta_item.setTextAlignment(Qt.AlignCenter)
        if 0 < eta < 10:
            eta_item.setForeground(QColor("#00f2ff"))
        self.nav_feedback_table.setItem(row, 5, eta_item)
    
    # ============== UI辅助方法 ==============
    def show_usv_plot_window(self):
        """显示USV绘图窗口"""
        self.ui_utils.show_usv_plot_window(lambda: self.list_manager.usv_online_list)
    
    def update_selected_table_row(self):
        """更新选中行数据"""
        self.ui_utils.update_selected_table_row(self.table_manager, self.state_handler)
    
    def update_usv_info_display(self, is_cluster=True):
        """
        更新USV详细信息显示（由表格选择改变时调用）
        
        Args:
            is_cluster: True表示从集群表格选择，False表示从离群表格选择
        """
        # 保存当前选择的表格类型
        self._current_selected_table = 'cluster' if is_cluster else 'departed'
        
        # 刷新显示
        self._refresh_selected_usv_info()
    
    def _toggle_random_run_mode(self, checked):
        """切换随机运行模式"""
        self.ros_signal.random_run_mode_changed.emit(checked)
        status = "开启" if checked else "关闭"
        self.ui_utils.append_info(f"🎲 随机运行模式已{status}")
        self.action_random_run_mode.setText(f"🎲 {'关闭' if checked else '开启'}随机运行模式")

    def _refresh_selected_usv_info(self):
        """
        刷新当前选中USV的详细信息（由状态更新定时器调用）
        
        该方法会根据当前选中的表格和行，实时更新USV信息面板
        """
        try:
            # 确定当前选中的是哪个表格
            is_cluster = getattr(self, '_current_selected_table', 'cluster') == 'cluster'
            
            # 获取选中的USV信息
            usv_info = self.table_manager.get_selected_usv_info(is_cluster)
            
            if not usv_info:
                # 没有选中时清空显示
                self.usv_info_panel.update_state(None)
                return
            
            # 获取 USV 的详细状态（实时从缓存中获取最新状态）
            namespace = usv_info.get('namespace')
            state = self.state_handler.get_usv_state(namespace)
            
            # 更新信息面板
            self.usv_info_panel.update_state(state)
                
        except Exception as e:
            # 出错时清空显示
            try:
                self.ui_utils.append_info(f"更新USV信息显示时出错: {e}")
            except Exception:
                pass
            self.usv_info_panel.update_state(None)
    
    def _refresh_selected_usv_navigation(self):
        """
        刷新当前选中USV的导航信息（由状态更新定时器调用）
        
        该方法会根据当前选中的表格和行，实时更新USV导航面板
        """
        try:
            # 确定当前选中的是哪个表格
            is_cluster = getattr(self, '_current_selected_table', 'cluster') == 'cluster'
            
            # 获取选中的USV信息
            usv_info = self.table_manager.get_selected_usv_info(is_cluster)
            
            if not usv_info:
                # 没有选中时清空显示
                self.usv_navigation_panel.update_navigation_state(None)
                return
            
            # 获取 USV 的详细状态（实时从缓存中获取最新状态）
            namespace = usv_info.get('namespace')
            state = self.state_handler.get_usv_state(namespace)
            
            # 获取导航反馈数据
            feedback = self.state_handler.get_usv_navigation_feedback(namespace)
            
            # 获取导航状态
            nav_status = self.state_handler.usv_nav_status.get(namespace, "空闲")
            
            # 更新导航面板
            self.usv_navigation_panel.update_navigation_state(state, feedback, nav_status)
                
        except Exception as e:
            # 出错时清空显示
            try:
                self.ui_utils.append_info(f"更新USV导航信息显示时出错: {e}")
            except Exception:
                pass
            self.usv_navigation_panel.update_navigation_state(None)

    def _handle_table_clicked(self, index, is_cluster):
        """处理表格单击事件，确保仅选中当前行并刷新详情"""
        try:
            table_view = self.ui.cluster_tableView if is_cluster else self.ui.departed_tableView
            if index is None or not index.isValid():
                return

            # 清理旧选择并强制选中当前行，避免残留多选状态
            selection_model = table_view.selectionModel()
            if selection_model is None:
                return
            selection_model.clearSelection()
            table_view.selectRow(index.row())

            # 记录当前表格并刷新详情
            self._current_selected_table = 'cluster' if is_cluster else 'departed'
            self._refresh_selected_usv_info()
        except Exception as exc:
            try:
                self.ui_utils.append_info(f"处理行选择时出错: {exc}")
            except Exception:
                pass
    
    def open_set_home_dialog(self):
        """打开设置 Home Position 对话框"""
        try:
            from .set_home_dialog import SetHomeDialog
            from PyQt5.QtWidgets import QMessageBox
            
            # 获取在线 USV 列表
            online_usvs = self.list_manager.usv_online_list
            
            if not online_usvs:
                QMessageBox.warning(
                    self,
                    "无在线 USV",
                    "当前没有在线的 USV，无法设置 Home Position。\n"
                    "请确保至少有一艘 USV 在线后再试。"
                )
                return
            
            # 创建并显示对话框
            dialog = SetHomeDialog(online_usvs, self)
            
            if dialog.exec_() == QDialog.Accepted:
                # 获取对话框结果
                usv_namespace, use_current, coords = dialog.get_result()
                
                if usv_namespace:
                    # 发送设置 Home Position 信号
                    self.ros_signal.set_home_position.emit(usv_namespace, use_current, coords)
                    
                    if use_current:
                        self.ui_utils.append_info(
                            f"📍 已向 {usv_namespace} 发送设置 Home Position 命令（使用当前位置）"
                        )
                    else:
                        self.ui_utils.append_info(
                            f"📍 已向 {usv_namespace} 发送设置 Home Position 命令\n"
                            f"    局部坐标: X={coords.get('x'):.2f}m, Y={coords.get('y'):.2f}m, Z={coords.get('z'):.2f}m"
                        )
        
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.critical(self, "错误", f"打开设置 Home Position 对话框失败: {e}")
            self.ui_utils.append_info(f"❌ 打开设置 Home Position 对话框失败: {e}")
    
    def open_param_config_window(self):
        """
        打开参数配置窗口（串口直连模式）
        
        通过 USB 串口直接与飞控通信，不依赖 MAVROS。
        """
        try:
            from .param_window_serial import ParamWindowSerial
            
            # 检查是否已有串口参数窗口打开（并且窗口仍然有效）
            if (hasattr(self, '_param_window_serial') and 
                self._param_window_serial is not None and 
                not self._param_window_serial.isHidden()):
                # 窗口存在且未被关闭，激活它
                self._param_window_serial.activateWindow()
                self._param_window_serial.raise_()
                return
            
            # 创建新窗口
            self._param_window_serial = ParamWindowSerial(self)
            
            # 窗口关闭时清理引用（QMainWindow 使用 destroyed 信号）
            def on_window_closed():
                self._param_window_serial = None
            
            self._param_window_serial.destroyed.connect(on_window_closed)
            
            # 显示窗口
            self._param_window_serial.show()
            self.ui_utils.append_info("✅ 已打开串口参数配置窗口")
            
        except ImportError as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.critical(
                self, "缺少依赖",
                f"串口参数模块加载失败:\n{e}\n\n"
                f"请安装 pymavlink 和 pyserial：\n"
                f"pip3 install pymavlink pyserial --break-system-packages"
            )
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.critical(self, "错误", f"打开串口参数窗口失败: {e}")
            self.ui_utils.append_info(f"❌ 打开参数窗口失败: {e}")

    def closeEvent(self, event):
        """
        窗口关闭事件处理器
        
        在关闭地面站之前，先发送关闭命令到所有在线USV：
        1. 关闭LED灯光
        2. 停止声音
        3. 停止扭头动作
        
        然后接受关闭事件
        
        Args:
            event: QCloseEvent对象
        """
        try:
            # 如果已经发送过关闭命令，直接接受关闭事件
            if self._shutdown_commands_sent:
                event.accept()
                return
            
            # 获取所有在线USV列表
            online_usvs = self.list_manager.usv_online_list
            
            if online_usvs:
                self.ui_utils.append_info("正在关闭所有USV外设（LED、声音、扭头）...")
                
                # 为所有在线USV发送关闭命令
                # 1. 关闭LED灯光
                try:
                    self.ros_signal.str_command.emit('led_off')
                except Exception as e:
                    _logger.warning(f"发送LED关闭命令失败: {e}")
                
                # 2. 停止声音
                try:
                    self.ros_signal.str_command.emit('sound_stop')
                except Exception as e:
                    _logger.warning(f"发送声音停止命令失败: {e}")
                
                # 3. 停止扭头动作
                try:
                    self.ros_signal.str_command.emit('neck_stop')
                except Exception as e:
                    _logger.warning(f"发送扭头停止命令失败: {e}")
                
                self.ui_utils.append_info("已发送外设关闭命令")
                
                # 标记已发送关闭命令，避免重复发送
                self._shutdown_commands_sent = True
                
                # 等待短暂时间确保命令被发送
                # 使用QTimer的singleShot来避免阻塞GUI线程
                from PyQt5.QtCore import QTimer
                QTimer.singleShot(500, lambda: self.close())
                event.ignore()  # 暂时忽略关闭事件，等待500ms后再关闭
            else:
                # 没有在线USV，直接接受关闭事件
                event.accept()
                
        except Exception as e:
            # 发生错误时也允许关闭
            _logger.error(f"closeEvent处理出错: {e}")
            try:
                self.ui_utils.append_info(f"关闭前处理出错: {e}，将直接关闭")
            except Exception:
                pass
            event.accept()


def main(argv=None):
    """主函数"""
    app = QApplication(sys.argv)
    ros_signal = ROSSignal()
    main_window = MainWindow(ros_signal)
    
    # 全局异常处理
    def _excepthook(type_, value, traceback_):
        try:
            msg = f"未捕获异常: {type_.__name__}: {value}"
            try:
                main_window.ui_utils.append_info(msg)
            except Exception:
                _logger.info(msg)
        finally:
            sys.__excepthook__(type_, value, traceback_)
    
    sys.excepthook = _excepthook
    
    # 设置持久化日志
    try:
        log_dir = os.path.abspath(os.path.join(os.getcwd(), '.logs'))
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, 'gs_gui.log')
        handler = RotatingFileHandler(log_file, maxBytes=5 * 1024 * 1024, backupCount=5, encoding='utf-8')
        fmt = logging.Formatter('%(asctime)s %(levelname)s %(name)s: %(message)s')
        handler.setFormatter(fmt)
        root_logger = logging.getLogger()
        if not any(isinstance(h, RotatingFileHandler) and getattr(h, 'baseFilename', None) == log_file for h in root_logger.handlers):
            root_logger.addHandler(handler)
        root_logger.setLevel(logging.INFO)
        root_logger.info('gs_gui 启动，日志记录到 %s' % log_file)
    except Exception:
        _logger.warning('无法设置持久化日志')
    
    # 初始化ROS节点（传入 append_info 和 append_warning 回调以输出到 GUI）
    rclpy.init(args=None)
    node = GroundStationNode(
        ros_signal, 
        append_info=main_window.ui_utils.append_info,
        append_warning=main_window.ui_utils.append_warning
    )
    
    # 将 ROS 节点传递给主窗口（用于参数管理功能）
    main_window.ros_node = node
    
    # 加载参数
    try:
        default_params_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), '..', '..', 'gs_bringup', 'config', 'gs_params.yaml'
        ))
        if os.path.isfile(default_params_path):
            with open(default_params_path, 'r') as f:
                params = yaml.safe_load(f) or {}
            try:
                param_list = []
                # 地面站集群控制参数白名单
                # 注意: target_reach_threshold 和 distance_mode 已移除
                # 这些参数由 USV 端的 navigate_to_point_server 控制
                whitelist = (
                    'step_timeout',
                    'max_retries',
                    'min_ack_rate_for_proceed'
                )
                for k in whitelist:
                    if k in params:
                        v = params[k]
                        try:
                            param_list.append(Parameter(k, value=v))
                        except Exception:
                            try:
                                if isinstance(v, str) and v.isdigit():
                                    val = int(v)
                                else:
                                    val = float(v)
                            except Exception:
                                val = v
                            param_list.append(Parameter(k, value=val))
                if param_list:
                    node.set_parameters(param_list)
            except Exception as e:
                node.get_logger().error(f"设置参数时出错: {e}")
        # gs_params.yaml 文件不存在时不输出任何信息
    except Exception as e:
        node.get_logger().error(f"加载 gs_params.yaml 时出错: {e}")
    
    # 连接ROS信号到节点
    ros_signal.hold_command.connect(node.command_processor.set_hold_callback)
    ros_signal.offboard_command.connect(node.command_processor.set_offboard_callback)
    ros_signal.stabilized_command.connect(node.command_processor.set_stabilized_callback)
    ros_signal.posctl_command.connect(node.command_processor.set_posctl_callback)
    ros_signal.altctl_command.connect(node.command_processor.set_altctl_callback)
    ros_signal.arm_command.connect(node.set_arming_callback)
    ros_signal.disarm_command.connect(node.set_disarming_callback)
    ros_signal.cluster_target_point_command.connect(node.set_cluster_target_point_callback)
    ros_signal.departed_target_point_command.connect(node.set_departed_target_point_callback)
    ros_signal.cluster_pause_request.connect(node.pause_cluster_task_callback)
    ros_signal.cluster_resume_request.connect(node.resume_cluster_task_callback)
    ros_signal.cluster_stop_request.connect(node.stop_cluster_task_callback)
    ros_signal.str_command.connect(node.str_command_callback)
    
    # 连接飞控重启信号
    ros_signal.reboot_autopilot.connect(node.reboot_autopilot_callback)
    
    # 连接机载计算机重启信号
    ros_signal.reboot_companion.connect(node.reboot_companion_callback)
    
    # 连接USV节点关闭信号
    ros_signal.shutdown_usv.connect(node.shutdown_usv_callback)
    
    # 连接 Home Position 设置信号
    ros_signal.set_home_position.connect(node.set_home_position_callback)
    
    # 连接节点信息信号
    try:
        node_info_sig = getattr(ros_signal, 'node_info', None)
        if node_info_sig is not None:
            node_info_sig.connect(main_window.ui_utils.append_info)
    except Exception:
        try:
            main_window.ui_utils.append_info('警告: 无法连接节点反馈信号 node_info')
        except Exception:
            pass
    
    # 连接坐标系偏移量更新信号
    try:
        sig_offset = getattr(ros_signal, 'update_area_center', None)
        cb_offset = getattr(node, 'update_area_center_callback', None)
        if sig_offset is not None and cb_offset is not None:
            sig_offset.connect(cb_offset)
            
        sig_fence = getattr(ros_signal, 'update_fence_config', None)
        cb_fence = getattr(node, 'update_fence_config_callback', None)
        if sig_fence is not None and cb_fence is not None:
            sig_fence.connect(cb_fence)
    except Exception:
        try:
            main_window.ui_utils.append_info('警告: 无法将坐标系/围栏信号连接到 GroundStationNode')
        except Exception:
            pass
    
    # 连接LED传染模式控制信号
    try:
        sig_led_infection = getattr(ros_signal, 'led_infection_mode_changed', None)
        cb_led_infection = getattr(node, 'set_led_infection_mode_callback', None)
        if sig_led_infection is not None and cb_led_infection is not None:
            sig_led_infection.connect(cb_led_infection)
    except Exception:
        try:
            main_window.ui_utils.append_info('警告: 无法将 led_infection_mode_changed 信号连接到 GroundStationNode')
        except Exception:
            pass
            
    # 连接随机运行模式控制信号
    try:
        sig_random_run = getattr(ros_signal, 'random_run_mode_changed', None)
        cb_random_run = getattr(node, 'toggle_random_run', None)
        if sig_random_run is not None and cb_random_run is not None:
            sig_random_run.connect(cb_random_run)
    except Exception:
        try:
            main_window.ui_utils.append_info('警告: 无法将 random_run_mode_changed 信号连接到 GroundStationNode')
        except Exception:
            pass
    
    # 启动ROS线程
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    # 显示主窗口
    main_window.show()
    
    try:
        exit_code = app.exec_()
    finally:
        # 优雅停止
        try:
            getattr(node, 'shutdown', lambda: None)()
        except Exception as e:
            _logger.warning(f"调用 node.shutdown() 时出错: {e}")
        try:
            node.destroy_node()
        except Exception as e:
            _logger.warning(f"销毁节点时出错: {e}")
        try:
            rclpy.shutdown()
        except Exception as e:
            _logger.warning(f"rclpy.shutdown() 时出错: {e}")
        try:
            if ros_thread.is_alive():
                ros_thread.join(timeout=2.0)
        except Exception:
            pass
    
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
