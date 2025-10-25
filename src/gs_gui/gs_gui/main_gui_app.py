"""
Ground Station主窗口应用
重构后的版本，使用模块化设计
"""
from http.client import UNAVAILABLE_FOR_LEGAL_REASONS
import sys
import threading
import os
import yaml
import logging
from logging.handlers import RotatingFileHandler

import rclpy
from rclpy.parameter import Parameter
from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QAbstractItemView, QMessageBox, QAction, QDialog
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
from gs_gui.style_manager import StyleManager


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, ros_signal):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle("Ground Station GUI")
        self.resize(1024, 512)
        self.setGeometry(100, 100, 1124, 612)
        
        self.ros_signal = ros_signal
        
        # 初始化样式管理器并加载现代化主题
        self.style_manager = StyleManager(self)
        self.style_manager.load_theme('modern_dark')
        
        # 初始化UI工具
        self.ui_utils = UIUtils(self.ui, self)

        # 初始化额外菜单
        self._init_custom_menu()
        
        # 初始化 USV 信息面板并替换原有的 groupBox_3
        self._init_usv_info_panel()
        
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
        
        # 初始化状态处理器
        self.state_handler = StateHandler(
            self.table_manager,
            self.list_manager,
            self.ui_utils.append_warning
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
    
    def _connect_ros_signals(self):
        """连接ROS信号到处理函数"""
        # 状态更新信号
        self.ros_signal.receive_state_list.connect(self.state_handler.receive_state_callback)
        
        # 集群任务进度信号
        self.ros_signal.cluster_progress_update.connect(self.task_manager.update_progress)
        
        # 导航状态更新信号
        self.ros_signal.nav_status_update.connect(self.state_handler.update_nav_status)
        
        # 导航反馈信号
        self.ros_signal.navigation_feedback.connect(self.handle_navigation_feedback)
    
    def _connect_ui_signals(self):
        """连接UI按钮信号到处理函数"""
        # ============== 集群控制按钮 ==============
        self.ui.arming_pushButton.clicked.connect(self.set_cluster_arming_command)
        self.ui.disarming_pushButton.clicked.connect(self.cluster_disarming_command)
        self.ui.set_guided_pushButton.clicked.connect(self.set_cluster_guided_command)
        self.ui.set_manual_pushButton.clicked.connect(self.set_cluster_manual_command)
        self.ui.send_cluster_point_pushButton.clicked.connect(self.toggle_cluster_task)
        
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
        self.ui.led1_pushButton.clicked.connect(self.command_handler.led_color_switching)
        self.ui.led2_pushButton.clicked.connect(self.command_handler.led_random_color)
        self.ui.led3_pushButton.clicked.connect(lambda: self.command_handler.led_select_color(self))
        self.ui.light_stop_pushButton.clicked.connect(self.command_handler.led_off)
        
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
        self.ui.action3D.triggered.connect(self.show_usv_plot_window)
        self.action_set_area_offset.triggered.connect(self.set_area_offset_command)
        self.action_led_infection_mode.triggered.connect(self.toggle_led_infection_mode)

    def _init_custom_menu(self):
        """在菜单栏中增加坐标偏移设置入口和LED传染模式开关"""
        # 坐标系设置菜单
        coord_menu = self.ui.menubar.addMenu("坐标系设置")
        self.action_set_area_offset = QAction("设置任务坐标系偏移量", self)
        coord_menu.addAction(self.action_set_area_offset)
        
        # LED设置菜单
        led_menu = self.ui.menubar.addMenu("LED设置")
        self.action_led_infection_mode = QAction("LED传染模式", self)
        self.action_led_infection_mode.setCheckable(True)
        self.action_led_infection_mode.setChecked(True)  # 默认打开
        led_menu.addAction(self.action_led_infection_mode)
    
    def _init_usv_info_panel(self):
        """初始化 USV 信息面板，替换原有的 groupBox_3"""
        # 创建 USV 信息面板
        self.usv_info_panel = UsvInfoPanel()
        
        # 获取原有的 groupBox_3 的父布局
        # groupBox_3 在 verticalLayout_10 中
        parent_layout = self.ui.groupBox_3.parent().layout()
        
        if parent_layout is not None:
            # 找到 groupBox_3 在布局中的索引
            index = parent_layout.indexOf(self.ui.groupBox_3)
            
            # 移除并隐藏原有的 groupBox_3
            parent_layout.removeWidget(self.ui.groupBox_3)
            self.ui.groupBox_3.hide()
            
            # 在相同位置插入新的信息面板
            if index >= 0:
                parent_layout.insertWidget(index, self.usv_info_panel)
            else:
                parent_layout.addWidget(self.usv_info_panel)
    
    # ============== 集群命令包装方法 ==============
    def set_cluster_arming_command(self):
        """集群解锁命令"""
        self.command_handler.set_cluster_arming(self.list_manager.usv_cluster_list)
    
    def cluster_disarming_command(self):
        """集群加锁命令"""
        self.command_handler.cluster_disarming(self.list_manager.usv_cluster_list)
    
    def set_cluster_guided_command(self):
        """集群设置guided模式"""
        self.command_handler.set_cluster_guided(self.list_manager.usv_cluster_list)
    
    def set_cluster_manual_command(self):
        """集群设置manual模式"""
        self.command_handler.set_cluster_manual(self.list_manager.usv_cluster_list)
    
    # ============== 离群命令包装方法 ==============
    def departed_arming_command(self):
        """离群解锁命令"""
        self.command_handler.departed_arming(self.list_manager.usv_departed_list)
    
    def departed_disarming_command(self):
        """离群加锁命令"""
        self.command_handler.departed_disarming(self.list_manager.usv_departed_list)
    
    def set_departed_guided_command(self):
        """离群设置guided模式"""
        self.command_handler.set_departed_guided(self.list_manager.usv_departed_list)
    
    def set_departed_manual_command(self):
        """离群设置manual模式"""
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
                QMessageBox.information(self, "操作成功", f"设备 {usv_info['namespace']} 已添加到集群列表")
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
                QMessageBox.information(self, "操作成功", f"设备 {usv_info['namespace']} 已添加到离群列表")
        else:
            self.ui_utils.append_info("请先选择一行")
    
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
    
    # ============== LED传染模式开关 ==============
    def toggle_led_infection_mode(self):
        """切换LED传染模式开关"""
        is_enabled = self.action_led_infection_mode.isChecked()
        self.ros_signal.led_infection_mode_changed.emit(is_enabled)
        status_text = "已开启" if is_enabled else "已关闭"
        self.ui_utils.append_info(f"LED传染模式{status_text}")
        QMessageBox.information(self, "LED传染模式", f"LED传染模式{status_text}")
    
    # ============== 导航反馈处理 ==============
    def handle_navigation_feedback(self, usv_id, feedback):
        """
        处理导航反馈信息
        
        Args:
            usv_id: USV标识符
            feedback: 导航反馈数据
        """
        self.ui.cluster_navigation_feedback_info_textEdit.append(
            f"USV {usv_id} 导航反馈 - "
            f"距离目标: {feedback.distance_to_goal:.2f}m, "
            f"航向误差: {feedback.heading_error:.2f}度, "
            f"预计剩余时间: {feedback.estimated_time:.2f}秒"
        )
    
    # ============== UI辅助方法 ==============
    def show_usv_plot_window(self):
        """显示USV绘图窗口"""
        self.ui_utils.show_usv_plot_window(lambda: self.list_manager.usv_online_list)
    
    def update_selected_table_row(self):
        """更新选中行数据"""
        self.ui_utils.update_selected_table_row(self.table_manager, self.state_handler)
    
    def update_usv_info_display(self, is_cluster=True):
        """
        更新USV详细信息显示
        
        Args:
            is_cluster: True表示从集群表格选择，False表示从离群表格选择
        """
        try:
            # 获取选中的USV信息
            usv_info = self.table_manager.get_selected_usv_info(is_cluster)
            
            if not usv_info:
                # 没有选中时清空显示
                self.usv_info_panel.update_state(None)
                return
            
            # 获取 USV 的详细状态
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
    
    # 初始化ROS节点
    rclpy.init(args=None)
    node = GroundStationNode(ros_signal)
    
    # 加载参数
    try:
        default_params_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), '..', '..', 'usv_bringup', 'config', 'gs_params.yaml'
        ))
        if os.path.isfile(default_params_path):
            with open(default_params_path, 'r') as f:
                params = yaml.safe_load(f) or {}
            try:
                param_list = []
                whitelist = (
                    'step_timeout',
                    'max_retries',
                    'min_ack_rate_for_proceed',
                    'target_reach_threshold',
                    'distance_mode'
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
                    print(f"已加载参数: {[p.name for p in param_list]}")
            except Exception as e:
                print(f"设置参数时出错: {e}")
        else:
            print(f"gs_params.yaml not found at {default_params_path}, skipping GUI param load")
    except Exception as e:
        print(f"加载 gs_params.yaml 时出错: {e}")
    
    # 连接ROS信号到节点
    ros_signal.manual_command.connect(node.set_manual_callback)
    ros_signal.guided_command.connect(node.set_guided_callback)
    ros_signal.arm_command.connect(node.set_arming_callback)
    ros_signal.disarm_command.connect(node.set_disarming_callback)
    ros_signal.arco_command.connect(node.set_arco_callback)
    ros_signal.steering_command.connect(node.set_steering_callback)
    ros_signal.cluster_target_point_command.connect(node.set_cluster_target_point_callback)
    ros_signal.departed_target_point_command.connect(node.set_departed_target_point_callback)
    ros_signal.str_command.connect(node.str_command_callback)
    
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
