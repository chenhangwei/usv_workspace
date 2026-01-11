"""
Ground Station主窗口应用
重构后的版本,使用模块化设计
"""
from http.client import UNAVAILABLE_FOR_LEGAL_REASONS
import sys
import threading
import os
import yaml
import logging
import subprocess
from logging.handlers import RotatingFileHandler

import rclpy
from rclpy.parameter import Parameter
from PyQt5.QtCore import QProcess, QTimer, Qt, QSettings
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QAbstractItemView, QMessageBox, QAction, QDialog, QMenu,
    QTabWidget, QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem, 
    QHeaderView, QProgressBar, QFrame, QLabel
)
from PyQt5.QtGui import QFont, QColor, QLinearGradient, QGradient, QPalette, QBrush
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
from gs_gui.geofence_manager import GeofenceManager
from gs_gui.geofence_dialog import GeofenceDialog
# 使用性能优化版本的集群启动器（异步检测 + 并行 ping）
from gs_gui.usv_fleet_launcher_optimized import UsvFleetLauncher


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, ros_signal):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("Ground Station GUI")
        
        # 恢复窗口大小和位置
        self.settings = QSettings("USV_Team", "GroundStation")
        geometry = self.settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)
        else:
            self.resize(1024, 512)
            self.setGeometry(100, 100, 1124, 612)

        # 更新按钮文本以匹配新的彩虹循环行为
        try:
            self.ui.led1_pushButton.setText("彩虹循环")
        except Exception:
            pass
        
        self.ros_signal = ros_signal
        
        # 优雅关闭标志：避免重复发送关闭命令
        self._shutdown_commands_sent = False
        
        # 初始化样式管理器并加载现代化主题
        self.style_manager = StyleManager(self)
        self.style_manager.load_theme('modern_dark')
        
        # 设置全局字体大小
        # 必须在 StyleManager 之后设置，以避免被主题覆盖
        # 可选值：9(默认小), 10(稍大), 11(中等), 12(较大), 13(大), 14(很大)
        from PyQt5.QtGui import QFont
        app_font = QFont()
        app_font.setPointSize(11)  # 设置为 11pt，更精致紧凑
        QApplication.instance().setFont(app_font)
        
        # 初始化UI工具
        self.ui_utils = UIUtils(self.ui, self)

        # 初始化额外菜单
        self._init_custom_menu()
        
        # 初始化消息框右键菜单
        self._init_text_edit_context_menus()
        
        # 初始化表格管理器
        self.table_manager = TableManager(
            self.ui.cluster_tableView,
            self.ui.departed_tableView
        )
        
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

        # 初始化右侧侧边栏综合选项卡（合并详情、导航、反馈、日志）
        # 注意：这需要用到 list_manager 初始化 2D 绘图窗口
        self._init_side_tab_panel()
        
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
        
        # 初始化电子围栏管理器
        self.geofence_manager = GeofenceManager(
            self.ros_signal,
            self.ui_utils.append_warning  # 使用warning通道输出围栏警告
        )
        
        # 初始化初始化界面控件
        self._init_task_status_label()
        self._init_geofence_checkbox()
        
        # 连接ROS信号
        self._connect_ros_signals()
        
        # 连接UI按钮信号
        self._connect_ui_signals()
        
        # 在初始化最后刷新表格表头
        self.table_manager.refresh_table_header()
    
    def _init_task_status_label(self):
        """初始化集群任务状态标签"""
        # 创建标签
        self.cluster_status_label = QLabel("当前没有加载任务")
        self.cluster_status_label.setAlignment(Qt.AlignCenter)
        self.cluster_status_label.setMinimumHeight(30)
        
        # 初始样式
        default_style = """
            QLabel {
                background-color: #2D2D2D;
                color: #888888;
                border: 1px solid #444444;
                border-radius: 4px;
                padding: 4px;
                font-weight: bold;
                font-size: 13px;
            }
        """
        self.cluster_status_label.setStyleSheet(default_style)
        
        # 将标签插入到verticalLayout_5的最上方 (集群控制区域)
        # verticalLayout_5 是右侧面板中"集群控制"GroupBox的布局
        self.ui.verticalLayout_5.insertWidget(0, self.cluster_status_label)
        
        # 将更新回调函数传递给Task Manager
        if hasattr(self, 'task_manager'):
            self.task_manager.set_update_status_callback(self.update_cluster_status)
            self.task_manager.set_task_loaded_callback(self.update_plot_preview)

    def _init_geofence_checkbox(self):
        """初始化电子围栏复选框到主界面"""
        from PyQt5.QtWidgets import QCheckBox
        self.geofence_checkbox = QCheckBox("🛡️ 启用电子围栏保护")
        self.geofence_checkbox.setToolTip("开启后，若USV超出设定矩形范围将自动锁定(HOLD)")
        self.geofence_checkbox.setStyleSheet("""
            QCheckBox {
                color: #CCCCCC;
                spacing: 5px;
                margin-top: 5px;
                margin-bottom: 5px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
            }
        """)
        
        # 插入到集群控制区域 (在状态标签下面)
        self.ui.verticalLayout_5.insertWidget(1, self.geofence_checkbox)
        
        # 连接信号
        self.geofence_checkbox.clicked.connect(self._on_geofence_checkbox_toggled)

    def _on_geofence_checkbox_toggled(self, checked):
        """主界面复选框切换处理"""
        # 更新管理器
        if hasattr(self, 'geofence_manager'):
            self.geofence_manager.set_enabled(checked)
        # 同步菜单
        if hasattr(self, 'action_geofence_toggle'):
            self.action_geofence_toggle.blockSignals(True)
            self.action_geofence_toggle.setChecked(checked)
            self.action_geofence_toggle.blockSignals(False)

    def update_plot_preview(self, task_data):
        """当任务加载后，更新绘图窗口的预览路径"""
        # 此时窗口已嵌入在 Feedback Tab 中，直接更新数据
        if hasattr(self, 'usv_plot_window') and self.usv_plot_window:
             self.usv_plot_window.set_preview_path(task_data)
             
        # 自动切换到反馈选项卡 (Tab Index 2: 📊 反馈)
        if hasattr(self, 'right_tab_widget'):
            self.right_tab_widget.setCurrentIndex(2)

    def update_cluster_status(self, text, style_sheet=None):
        """更新集群任务状态标签"""
        if hasattr(self, 'cluster_status_label'):
            self.cluster_status_label.setText(text)
            if style_sheet:
                self.cluster_status_label.setStyleSheet(style_sheet)
    
    def _connect_ros_signals(self):
        """连接ROS信号到处理函数"""
        # 状态更新信号
        self.ros_signal.receive_state_list.connect(self.state_handler.receive_state_callback)
        # 连接电子围栏检查
        if hasattr(self, 'geofence_manager'):
            self.ros_signal.receive_state_list.connect(self.geofence_manager.check_usv_states)
        
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
        self.ui.set_guided_pushButton.clicked.connect(self.set_cluster_guided_command)
        self.ui.set_manual_pushButton.clicked.connect(self.set_cluster_hold_command)
        self.ui.send_cluster_point_pushButton.clicked.connect(self.toggle_cluster_task)
        self.ui.stop_cluster_task_pushButton.clicked.connect(self.stop_cluster_task)
        
        # ============== 离群控制按钮 ==============
        self.ui.departed_arming_pushButton.clicked.connect(self.departed_arming_command)
        self.ui.departed_disarming_pushButton.clicked.connect(self.departed_disarming_command)
        self.ui.set_departed_guided_pushButton.clicked.connect(self.set_departed_guided_command)
        self.ui.set_departed_manual_pushButton.clicked.connect(self.set_departed_manual_command)
        self.ui.set_departed_ARCO_pushButton.clicked.connect(self.set_departed_arco_command)
        self.ui.set_departed_Steering_pushButton.clicked.connect(self.set_departed_steering_command)
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
        self.action_geofence_settings.triggered.connect(self.open_geofence_dialog)
        self.action_geofence_toggle.toggled.connect(self.toggle_geofence_from_menu)
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
        
        # 工具菜单
        tools_menu = self.ui.menubar.addMenu("工具(&T)")
        
        # Home Position 设置
        self.action_set_home = QAction("🏠 设置 Home Position", self)
        self.action_set_home.setShortcut("Ctrl+H")
        self.action_set_home.setToolTip("设置 USV 的 Home Position（RTL 返航点）")
        tools_menu.addAction(self.action_set_home)

        # 电子围栏设置
        self.action_geofence_settings = QAction("🚧 电子围栏设置...", self)
        self.action_geofence_settings.setToolTip("设置矩形活动区域，越界自动HOLD")
        tools_menu.addAction(self.action_geofence_settings)
        
        # 电子围栏快速开关
        self.action_geofence_toggle = QAction("🛡️ 启用电子围栏", self)
        self.action_geofence_toggle.setCheckable(True)
        self.action_geofence_toggle.setChecked(False)
        self.action_geofence_toggle.setToolTip("快速开启/关闭电子围栏监控")
        tools_menu.addAction(self.action_geofence_toggle)
        
        # 分隔线
        tools_menu.addSeparator()
        
        # 飞控参数配置
        self.action_param_config = QAction("[+] 飞控参数配置...", self)
        self.action_param_config.setShortcut("Ctrl+P")
        self.action_param_config.setToolTip("通过串口直连配置飞控参数")
        tools_menu.addAction(self.action_param_config)

    def _init_text_edit_context_menus(self):
        """初始化消息框的右键清空功能"""
        
        # 定义通用的上下文菜单策略处理函数
        def setup_context_menu(text_edit):
            text_edit.setContextMenuPolicy(Qt.CustomContextMenu)
            text_edit.customContextMenuRequested.connect(
                lambda pos: show_context_menu(text_edit, pos)
            )

        def show_context_menu(text_edit, pos):
            # 创建标准菜单（包含复制/全选等）
            menu = text_edit.createStandardContextMenu()
            menu.addSeparator()
            # 添加清空动作
            clear_action = QAction("🗑️ 清除内容", menu)
            clear_action.triggered.connect(text_edit.clear)
            menu.addAction(clear_action)
            # 显示菜单
            menu.exec_(text_edit.mapToGlobal(pos))

        # 为三个文本框应用策略
        setup_context_menu(self.ui.cluster_navigation_feedback_info_textEdit)
        setup_context_menu(self.ui.info_textEdit)
        setup_context_menu(self.ui.warning_textEdit)
    
    def _init_side_tab_panel(self):
        """初始化右侧侧边栏综合选项卡（合并详情、导航、反馈、日志）"""
        # 1. 创建 TabWidget
        self.right_tab_widget = QTabWidget()
        self.right_tab_widget.setTabPosition(QTabWidget.North)
        self.right_tab_widget.setDocumentMode(True)  # 更现代的文档模式外观
        
        # 2. 准备各个面板
        # [Tab 1] USV 详情
        self.usv_info_panel = UsvInfoPanel()
        self.right_tab_widget.addTab(self.usv_info_panel, "📋 详情")
        
        # [Tab 2] USV 导航
        self.usv_navigation_panel = UsvNavigationPanel()
        self.right_tab_widget.addTab(self.usv_navigation_panel, "🧭 导航")
        
        # [Tab 3] 任务反馈 (Dashboard + Table)
        feedback_widget = self._init_feedback_tab()
        self.right_tab_widget.addTab(feedback_widget, "📊 反馈")
        
        # [Tab 4] 系统信息 (复用现有控件)
        if self.ui.info_textEdit.parent():
            self.ui.info_textEdit.setParent(None)
        self.right_tab_widget.addTab(self.ui.info_textEdit, "ℹ 信息")
        
        # [Tab 5] 系统警告 (复用现有控件)
        if self.ui.warning_textEdit.parent():
            self.ui.warning_textEdit.setParent(None)
        self.right_tab_widget.addTab(self.ui.warning_textEdit, "⚠ 警告")
        
        # 3. 清理旧布局
        # 隐藏原有的 groupBox_usv_details 和 groupBox_2 (Message)
        self.ui.groupBox_usv_details.hide()
        self.ui.groupBox_2.hide()
        
        # 4. 添加到 Splitter (替换右侧区域)
        main_splitter = self.ui.mainSplitter
        # 现在的 Splitter 应该有: [0: LeftControl, 1: OldDetails, 2: OldMessage]
        # 我们调整拉伸因子，因为现在只有两部分：List(0) 和 Tabs(1-added)
        main_splitter.addWidget(self.right_tab_widget)
        
        # 5. 设置科幻风格 QSS
        self.right_tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #444;
                background: #1e1e1e;
                top: -1px; 
            }
            QTabBar::tab {
                background: #2d2d2d;
                color: #aaa;
                padding: 8px 20px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                margin-right: 2px;
                font-family: "Segoe UI", sans-serif;
                font-size: 11pt;
            }
            QTabBar::tab:selected {
                background: #1e1e1e;
                color: #00f2ff;
                border-bottom: 2px solid #00f2ff;
            }
            QTabBar::tab:hover {
                background: #3d3d3d;
                color: #fff;
            }
        """)

    def _init_feedback_tab(self):
        """初始化反馈选项卡内容"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)
        
        # 1. 任务仪表盘 (Mission Dashboard)
        self._init_mission_dashboard(layout)
        
        # 2. 导航反馈表格 (Navigation Table)
        self._init_navigation_feedback_table()
        layout.addWidget(self.nav_feedback_table)
        
        # 3. 嵌入 2D 绘图窗口 (Embedded 2D Plot)
        from gs_gui.usv_plot_window import UsvPlotWindow
        # 我们在这里创建唯一的实例，放在表格下面
        self.usv_plot_window = UsvPlotWindow(self.list_manager.get_usv_list, self)
        layout.addWidget(self.usv_plot_window, stretch=1) # 占据剩余空间
        
        # 保存 layout 引用以便后续使用
        self.nav_feedback_layout = layout
        
        return widget

    def _init_mission_dashboard(self, parent_layout):
        """初始化科幻任务仪表盘"""
        self.mission_dashboard = QFrame()
        self.mission_dashboard.setFixedHeight(100) # 固定高度
        self.mission_dashboard.setObjectName("missionDashboard")
        
        db_layout = QVBoxLayout(self.mission_dashboard)
        db_layout.setContentsMargins(15, 10, 15, 10)
        
        # 上部分：标题与状态标签
        top_layout = QHBoxLayout()
        title_label = QLabel("MISSION STATUS")
        title_label.setObjectName("dbTitle")
        
        self.mission_state_label = QLabel("IDLE")
        self.mission_state_label.setObjectName("dbState")
        self.mission_state_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        top_layout.addWidget(title_label)
        top_layout.addStretch()
        top_layout.addWidget(self.mission_state_label)
        db_layout.addLayout(top_layout)
        
        # 中部分：数据概览
        info_layout = QHBoxLayout()
        self.mission_step_label = QLabel("STEP: 0/0")
        self.mission_units_label = QLabel("UNITS: 0/0")
        self.mission_time_label = QLabel("TIME: 0.0s")
        for lbl in [self.mission_step_label, self.mission_units_label, self.mission_time_label]:
            lbl.setObjectName("dbInfo")
            info_layout.addWidget(lbl)
            info_layout.addStretch()
        # 移除最后一个 stretch
        if info_layout.count() > 0:
            info_layout.takeAt(info_layout.count()-1)
        db_layout.addLayout(info_layout)
        
        # 下部分：进度条
        self.mission_progress_bar = QProgressBar()
        self.mission_progress_bar.setObjectName("dbProgress")
        self.mission_progress_bar.setValue(0)
        self.mission_progress_bar.setFormat("MISSION PROGRESS: %p%")
        self.mission_progress_bar.setAlignment(Qt.AlignCenter)
        self.mission_progress_bar.setFixedHeight(20)
        db_layout.addWidget(self.mission_progress_bar)
        
        # Dashboard QSS
        self.mission_dashboard.setStyleSheet("""
            QFrame#missionDashboard {
                background-color: #0a192f;
                border: 1px solid #00f2ff;
                border-radius: 5px;
            }
            QLabel#dbTitle {
                font-family: "Impact", sans-serif;
                font-size: 14pt;
                color: #000;
                background-color: #00f2ff;
                padding: 2px 5px;
                font-weight: bold;
            }
            QLabel#dbState {
                font-family: "Consolas", monospace;
                font-size: 12pt;
                color: #00f2ff;
                font-weight: bold;
            }
            QLabel#dbInfo {
                font-family: "Consolas", monospace;
                font-size: 10pt;
                color: #00f2ff;
            }
            QProgressBar#dbProgress {
                border: 1px solid #00f2ff;
                border-radius: 2px;
                text-align: center;
                color: #00f2ff;
                background-color: #001122;
                font-family: "Consolas", monospace;
                font-weight: bold;
            }
            QProgressBar#dbProgress::chunk {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #004e92, stop:1 #00f2ff);
            }
        """)
        
        parent_layout.addWidget(self.mission_dashboard)

    def _update_mission_dashboard(self, progress_info):
        """更新任务仪表盘数据"""
        # 1. 进度条
        ack_rate = progress_info.get('ack_rate', 0.0)
        self.mission_progress_bar.setValue(int(ack_rate * 100))
        
        # 2. 文本信息
        current_step = progress_info.get('current_step', 0)
        total_steps = progress_info.get('total_steps', 0)
        acked_usvs = progress_info.get('acked_usvs', 0)
        total_usvs = progress_info.get('total_usvs', 0)
        elapsed_time = progress_info.get('elapsed_time', 0.0)
        state = progress_info.get('state', 'unknown')
        
        self.mission_step_label.setText(f"STEP: {current_step}/{total_steps}")
        self.mission_units_label.setText(f"UNITS: {acked_usvs}/{total_usvs}")
        self.mission_time_label.setText(f"TIME: {elapsed_time:.1f}s")
        
        # 3. 状态与呼吸灯效果
        state_map = {'idle': 'IDLE', 'running': 'RUNNING', 'completed': 'COMPLETED', 'failed': 'FAILED'}
        state_text = state_map.get(state, state.upper())
        self.mission_state_label.setText(state_text)
        
        if state == 'running':
             self.mission_state_label.setStyleSheet("QLabel#dbState { color: #00f2ff; }") # 简单处理，可加定时器闪烁
        elif state == 'completed':
             self.mission_state_label.setStyleSheet("QLabel#dbState { color: #4caf50; }")
        elif state == 'failed':
             self.mission_state_label.setStyleSheet("QLabel#dbState { color: #f44336; }")

    def _init_navigation_feedback_table(self):
        """初始化导航反馈表格，采用科幻风格设计"""
        self.nav_feedback_table = QTableWidget()
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
        
        # 用于存储 usv_id 到行索引的映射
        self._nav_feedback_row_map = {}
    
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
    
    def set_cluster_guided_command(self):
        """集群设置guided模式（带防抖）"""
        # 防抖：1秒内只允许一次 guided 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_guided_time'):
            self._last_guided_time = 0
        if now - self._last_guided_time < 1.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 1 秒后再试")
            return
        self._last_guided_time = now
        self.command_handler.set_cluster_guided(self.list_manager.usv_cluster_list)
    
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
    
    def set_departed_guided_command(self):
        """离群设置guided模式（带防抖）"""
        # 防抖：1秒内只允许一次 guided 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_departed_guided_time'):
            self._last_departed_guided_time = 0
        if now - self._last_departed_guided_time < 1.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 1 秒后再试")
            return
        self._last_departed_guided_time = now
        self.command_handler.set_departed_guided(self.list_manager.usv_departed_list)
    
    def set_departed_manual_command(self):
        """离群设置manual模式（带防抖）"""
        # 防抖：1秒内只允许一次 manual 命令
        import time
        now = time.time()
        if not hasattr(self, '_last_departed_manual_time'):
            self._last_departed_manual_time = 0
        if now - self._last_departed_manual_time < 1.0:
            self.ui_utils.append_info("⚠️ 操作过快，请等待 1 秒后再试")
            return
        self._last_departed_manual_time = now
        self.command_handler.set_departed_manual(self.list_manager.usv_departed_list)
    
    def set_departed_arco_command(self):
        """离群设置ARCO模式"""
        self.command_handler.set_departed_arco(self.list_manager.usv_departed_list)
    
    def set_departed_steering_command(self):
        """离群设置Steering模式"""
        self.command_handler.set_departed_steering(self.list_manager.usv_departed_list)
    
    # ============== 集群任务控制 ==============
    def toggle_cluster_task(self):
        """切换集群任务运行状态"""
        button_text = self.task_manager.toggle_task(self.list_manager.usv_departed_list)
        self.ui.send_cluster_point_pushButton.setText(button_text)
    
    def stop_cluster_task(self):
        """停止集群任务并刷新按钮文本"""
        self.task_manager.stop_task()
        self.ui.send_cluster_point_pushButton.setText(self.task_manager.get_button_text())

    def _handle_cluster_progress_update(self, progress_info):
        """处理集群任务进度更新并同步按钮文本"""
        self.task_manager.update_progress(progress_info)
        self.ui.send_cluster_point_pushButton.setText(self.task_manager.get_button_text())
        
        # 更新科幻仪表盘
        if hasattr(self, 'mission_dashboard'):
            self._update_mission_dashboard(progress_info)

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
        """设置任务坐标系偏移量（Area Center）"""
        try:
            # 获取当前的偏移量（从参数文件或默认值）
            current_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            # 显示对话框
            dialog = AreaOffsetDialog(self, current_offset)
            if dialog.exec_() == QDialog.Accepted:
                new_offset = dialog.get_offset()
                # 发送更新信号到ROS节点
                self.ros_signal.update_area_center.emit(new_offset)
                self.ui_utils.append_info(
                    f"已更新任务坐标系偏移量: X={new_offset['x']:.2f}m, "
                    f"Y={new_offset['y']:.2f}m, Z={new_offset['z']:.2f}m"
                )
        except Exception as e:
            self.ui_utils.append_info(f"设置坐标偏移量时发生错误: {e}")
    
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
    def clear_navigation_feedback_table(self):
        """清空导航反馈列表"""
        self.nav_feedback_table.setRowCount(0)
        self._nav_feedback_row_map.clear()
        
    def handle_navigation_feedback(self, usv_id, feedback):
        """
        处理导航反馈信息，更新到表格中（科幻增强版）
        """
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
        # 优先显示 Step 数值（T-xx），如果是单点导航则显示 Goal ID
        target_val = getattr(feedback, 'step', 0)
        if target_val <= 0:
            target_val = feedback.goal_id
            
        goal_item = QTableWidgetItem(f"T-{target_val:02d}")
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
        """显示USV绘图窗口 (切换到反馈Tab)"""
        # 以前是弹窗，现在是切换到 Feedback Tab
        if hasattr(self, 'right_tab_widget'):
            self.right_tab_widget.setCurrentIndex(2) # Index 2 is Feedback tab
    
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
    
    def open_geofence_dialog(self):
        """打开电子围栏设置对话框"""
        dialog = GeofenceDialog(
            self,
            current_bounds=self.geofence_manager.rect,
            current_enabled=self.geofence_manager.enabled
        )
        if dialog.exec_() == QDialog.Accepted:
            bounds, enabled = dialog.get_settings()
            self.geofence_manager.set_bounds(bounds)
            self.geofence_manager.set_enabled(enabled)
            
            # 同步更新UI状态
            self._sync_geofence_ui(enabled)

    def toggle_geofence_from_menu(self, checked):
        """从菜单快速切换电子围栏状态"""
        if hasattr(self, 'geofence_manager'):
            self.geofence_manager.set_enabled(checked)
        # 同步复选框
        if hasattr(self, 'geofence_checkbox'):
            self.geofence_checkbox.blockSignals(True)
            self.geofence_checkbox.setChecked(checked)
            self.geofence_checkbox.blockSignals(False)
            
    def _sync_geofence_ui(self, enabled):
        """同步所有电子围栏相关的UI控件状态"""
        # 1. 菜单
        if hasattr(self, 'action_geofence_toggle'):
            self.action_geofence_toggle.blockSignals(True)
            self.action_geofence_toggle.setChecked(enabled)
            self.action_geofence_toggle.blockSignals(False)
        # 2. 主界面复选框
        if hasattr(self, 'geofence_checkbox'):
            self.geofence_checkbox.blockSignals(True)
            self.geofence_checkbox.setChecked(enabled)
            self.geofence_checkbox.blockSignals(False)

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
                            f"    坐标 (XYZ): {coords.get('x'):.2f}, {coords.get('y'):.2f}, {coords.get('z'):.2f}m"
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
        # 保存窗口大小和位置
        self.settings.setValue("geometry", self.saveGeometry())
        
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
                    print(f"发送LED关闭命令失败: {e}")
                
                # 2. 停止声音
                try:
                    self.ros_signal.str_command.emit('sound_stop')
                except Exception as e:
                    print(f"发送声音停止命令失败: {e}")
                
                # 3. 停止扭头动作
                try:
                    self.ros_signal.str_command.emit('neck_stop')
                except Exception as e:
                    print(f"发送扭头停止命令失败: {e}")
                
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
            print(f"closeEvent处理出错: {e}")
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
                print(msg)
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
        print('无法设置持久化日志')
    
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
    ros_signal.hold_command.connect(node.set_hold_callback)
    ros_signal.guided_command.connect(node.set_guided_callback)
    ros_signal.manual_command.connect(node.set_manual_callback)  # ✅ 修复：添加manual信号连接
    ros_signal.arm_command.connect(node.set_arming_callback)
    ros_signal.disarm_command.connect(node.set_disarming_callback)
    ros_signal.arco_command.connect(node.set_arco_callback)
    ros_signal.steering_command.connect(node.set_steering_callback)  # ✅ 修复：callback不是command
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
    except Exception:
        try:
            main_window.ui_utils.append_info('警告: 无法将 update_area_center 信号连接到 GroundStationNode')
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
            print(f"调用 node.shutdown() 时出错: {e}")
        try:
            node.destroy_node()
        except Exception as e:
            print(f"销毁节点时出错: {e}")
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"rclpy.shutdown() 时出错: {e}")
        try:
            if ros_thread.is_alive():
                ros_thread.join(timeout=2.0)
        except Exception:
            pass
    
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
