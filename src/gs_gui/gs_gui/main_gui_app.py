from http.client import UNAVAILABLE_FOR_LEGAL_REASONS
import sys
import threading
import weakref  # 添加weakref模块用于弱引用
from collections import deque

import rclpy
from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow,  QAbstractItemView, QFileDialog, QMessageBox, QColorDialog
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from gs_gui.ros_signal import ROSSignal
# 更新导入语句，使用新的模块结构
from gs_gui.ground_station_node import GroundStationNode
import os
import yaml
from rclpy.parameter import Parameter
from gs_gui.ui import Ui_MainWindow
import xml.etree.ElementTree as ET
from gs_gui.usv_plot_window import UsvPlotWindow
import re


class MainWindow(QMainWindow):
    def __init__(self,ros_signal):
        super().__init__()
        self.ui=Ui_MainWindow()        
        self.ui.setupUi(self)
        self.setWindowTitle("Ground Station GUI")
        self.resize(1024, 512)
        self.setGeometry(100, 100, 1124, 612)

        self.ros_signal = ros_signal
        
        # 表格列标题
        self.TABLE_HEADERS = ["编号", "当前模式", "连接状态", "武装状态", "引导模式状态", '电压V', '电量%', 
                             '电池状态', '坐标', '速度', '偏角', '导航状态', '温度']

        # 导航状态常量
        self.NAV_STATUS_UNKNOWN = "未知"
        self.NAV_STATUS_IDLE = "空闲"
        self.NAV_STATUS_ACTIVE = "执行中"
        self.NAV_STATUS_SUCCEEDED = "成功"
        self.NAV_STATUS_FAILED = "失败"

        # 命名空间列表
        self.CLUSTER_NAMESPACE_LIST = "cluster"
        self.DEPARTED_NAMESPACE_LIST = "departed"
        
        # 存储USV导航状态
        self.usv_nav_status = {}
        # 添加集群任务进度信息
        self.cluster_progress_info = {}

        # 添加USV列表
        self.usv_cluster_list = []      # 集群USV列表
        self.usv_departed_list = []     # 离群USV列表
        self.usv_online_list = []       # 在线USV列表
        
        # 添加集群位置列表
        self.cluster_position_list = []  # 集群位置列表
        self.departed_position_list = []  # 离群目标点集合
        
        # 集群设备namespace列表
        self.usv_cluster_namespace_list = []
        # 离群设备namespace列表
        self.usv_departed_namespace_list = []

        # 初始化表格模型
        self.cluster_table_model = QStandardItemModel(self)
        self.departed_table_model = QStandardItemModel(self)
        
        # 设置表格模型
        self.ui.cluster_tableView.setModel(self.cluster_table_model)
        self.ui.departed_tableView.setModel(self.departed_table_model)
        
        # 设置表头
        self.cluster_table_model.setHorizontalHeaderLabels(self.TABLE_HEADERS)
        self.departed_table_model.setHorizontalHeaderLabels(self.TABLE_HEADERS)
        
        # 启用表格排序功能
        self.ui.cluster_tableView.setSortingEnabled(True)
        self.ui.departed_tableView.setSortingEnabled(True)
        
        # 添加集群任务控制状态
        self.cluster_task_running = False  # 集群任务是否正在运行
        self.cluster_task_paused = False   # 集群任务是否已暂停
        
        # 添加USV状态缓存
        self._usv_state_cache = {}  # USV状态缓存
        self._usv_state_dirty = False  # USV状态是否需要更新标志

        # 使用 QTimer 在 GUI 线程周期性刷新 UI，避免高频直接更新导致卡顿
        self._ui_refresh_timer = QTimer(self)
        self._ui_refresh_timer.setInterval(200)  # 每200毫秒检查一次更新
        self._ui_refresh_timer.timeout.connect(self._flush_state_cache_to_ui)
        self._ui_refresh_timer.start()

        # 日志缓冲：用于限制 info_textEdit 的写入速率和最大行数

        self._info_buffer = deque()
        self._info_max_lines = 500  # info_textEdit 最大行数
        self._info_flush_interval_ms = 500
        self._info_timer = QTimer(self)
        self._info_timer.setInterval(self._info_flush_interval_ms)
        self._info_timer.timeout.connect(self._flush_info_buffer)
        self._info_timer.start()
        
        # 设置表格为单行选择模式
        self.ui.cluster_tableView.setSelectionMode(QAbstractItemView.SingleSelection)
        self.ui.departed_tableView.setSelectionMode(QAbstractItemView.SingleSelection)
        
        # 设置表格为行选择模式
        self.ui.cluster_tableView.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.ui.departed_tableView.setSelectionBehavior(QAbstractItemView.SelectRows)
        
        # 设置表格为只读
        self.ui.cluster_tableView.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.ui.departed_tableView.setEditTriggers(QAbstractItemView.NoEditTriggers)

        # 添加USV绘图窗口的弱引用
        self.usv_plot_window_ref = None

        # 初始化集群任务按钮状态
        self.update_cluster_task_button_state()

        # 连接信号
        self.ros_signal.receive_state_list.connect(self.receive_state_callback)
        self.ros_signal.cluster_progress_update.connect(self.cluster_progress_callback)

        # 连接按钮信号
        self.ui.arming_pushButton.clicked.connect(self.set_cluster_arming_command)            # 集群解锁按钮
        self.ui.disarming_pushButton.clicked.connect(self.cluster_disarming_command)      # 集群加锁按钮
        self.ui.set_guided_pushButton.clicked.connect(self.set_cluster_guided_command)    # 集群切换到guided模式
        self.ui.set_manual_pushButton.clicked.connect(self.set_cluster_manual_command)    # 集群切换到manaul模式

        self.ui.departed_arming_pushButton.clicked.connect(self.departed_arming_command)  # 离群解锁按钮
        self.ui.departed_disarming_pushButton.clicked.connect(self.departed_disarming_command) #离群加锁按钮
        self.ui.set_departed_guided_pushButton.clicked.connect(self.set_departed_guided_command) #离群切换到guided模式
        self.ui.set_departed_manual_pushButton.clicked.connect(self.set_departed_manual_command) #离群切换到manual模式
        self.ui.set_departed_ARCO_pushButton.clicked.connect(self.set_departed_arco_command) #离群切换到ARCO模式

        self.ui.send_cluster_point_pushButton.clicked.connect(self.toggle_cluster_task) # 集群坐标发送        
        self.ui.send_departed_point_pushButton.clicked.connect(self.send_departed_point_command)#离群坐标发送
        
        self.ui.add_cluster_pushButton.clicked.connect(self.add_cluster_command)
        self.ui.quit_cluster_pushButton.clicked.connect(self.quit_cluster_command)

        self.ui.sound_start_pushButton.clicked.connect(self.sound_start_command)
        self.ui.sound_stop_pushButton.clicked.connect(self.sound_stop_command)
        self.ui.neck_swinging_pushButton.clicked.connect(self.neck_swinging_command)
        self.ui.neck_stop_pushButton.clicked.connect(self.neck_stop_command)

        self.ui.led1_pushButton.clicked.connect(self.led1_command)
        self.ui.led2_pushButton.clicked.connect(self.led2_command)
        self.ui.led3_pushButton.clicked.connect(self.led3_command)
        self.ui.light_stop_pushButton.clicked.connect(self.light_stop_command)

        # 菜单操作
        self.ui.actionopen.triggered.connect(self.read_data_from_file)
        self.ui.actionrviz2.triggered.connect(self.rviz_process)
        self.ui.action3D.triggered.connect(self.show_usv_plot_window)

        # 在 __init__ 的最后添加这行
        self.refresh_table_header()

    def _extract_namespaces(self, usv_list):
        """
        从USV列表中提取命名空间列表
        
        Args:
            usv_list: USV列表，每个元素是一个包含'namespace'键的字典
            
        Returns:
            list: 命名空间列表
        """
        namespaces = []
        for item in usv_list:
            if isinstance(item, dict) and 'namespace' in item:
                namespaces.append(item['namespace'])
        return namespaces

    def _update_tables_if_dirty(self):
        """
        如果有新的USV状态数据，更新表格显示
        """
        if self._usv_state_dirty:
            self._flush_state_cache_to_ui()

    def map_power_supply_status(self, status):
        """
        将电池状态数字映射成文字
        0:未知 1:充电 2:放电 3:充满 4:未充电
        
        Args:
            status: 电池状态数字
            
        Returns:
            str: 对应的文字描述
        """
        status_map = {
            0: "未知",
            1: "充电",
            2: "放电",
            3: "充满",
            4: "未充电"
        }
        try:
            return status_map.get(int(status), "未知")
        except (ValueError, TypeError):
            return "未知"

    # 从数据文件中读取数据，数据格式xml
    def read_data_from_file(self):
        """
        从XML文件中读取集群任务数据
        """
        # 打开文件对话框，选择XML文件
        file_dialog = QFileDialog(self)
        file_dialog.setNameFilter("XML Files (*.xml)")
        file_dialog.setFileMode(QFileDialog.ExistingFiles)
        if file_dialog.exec_():
            xml_files = file_dialog.selectedFiles()
            if xml_files:
                # 读取第一个选中的文件
                xml_file = xml_files[0]
                # 初始化缓存列表
                usv_list = []   
                try:
                    # 解析XML文件
                    tree = ET.parse(xml_file)
                    root = tree.getroot()
                    
                    # 获取step节点
                    step = root.find("step")
                    if step is not None:
                        step_number = step.get("number")
                        
                        # 遍历usvs下的所有usv节点
                        usvs_elem = step.find("usvs")
                        if usvs_elem is not None:
                            for usv in usvs_elem.findall("usv"):
                                usv_id_elem = usv.find("usv_id")
                                pos_x_elem = usv.find("position/x")
                                pos_y_elem = usv.find("position/y")
                                pos_z_elem = usv.find("position/z")  # 添加z坐标支持
                                yaw_elem = usv.find("yaw/value")
                                velocity_elem = usv.find("velocity/value")
                                usv_data = {
                                    "usv_id": usv_id_elem.text if usv_id_elem is not None else "",
                                    "position": {
                                        "x": float(pos_x_elem.text) if pos_x_elem is not None and pos_x_elem.text is not None else 0.0,
                                        "y": float(pos_y_elem.text) if pos_y_elem is not None and pos_y_elem.text is not None else 0.0,
                                        "z": float(pos_z_elem.text) if pos_z_elem is not None and pos_z_elem.text is not None else 0.0  # 添加z坐标
                                    },
                                    "yaw": float(yaw_elem.text) if yaw_elem is not None and yaw_elem.text is not None else 0.0,
                                    "velocity": float(velocity_elem.text) if velocity_elem is not None and velocity_elem.text is not None else 0.0,
                                    "step": int(step_number) if step_number is not None else 0
                                }
                                usv_list.append(usv_data)

                            self.cluster_position_list=usv_list
                            self.append_info(f"读取数据成功，共 {len(usv_list)} 个 USV 数据")
                            self.append_info(f"数据： {self.cluster_position_list} ")
                            # 重置任务状态
                            self.cluster_task_running = False
                            self.cluster_task_paused = False
                            self.update_cluster_task_button_state()
                    else:
                        error_msg = "XML文件格式错误：未找到step节点"
                        print(error_msg)
                        self.append_info(error_msg)
                        self.cluster_position_list=[]
                        self.update_cluster_task_button_state()
                    
                except ET.ParseError as e:
                    error_msg = f"XML解析错误: {e}"
                    print(error_msg)
                    self.append_info(error_msg)
                    QMessageBox.critical(self, "XML解析错误", error_msg)
                    self.cluster_position_list=[]

                except FileNotFoundError:
                    error_msg = f"文件未找到: {xml_file}"
                    print(error_msg)
                    self.append_info(error_msg)
                    QMessageBox.critical(self, "文件错误", error_msg)
                    self.cluster_position_list=[]

                except PermissionError:
                    error_msg = f"没有权限访问文件: {xml_file}"
                    print(error_msg)
                    self.append_info(error_msg)
                    QMessageBox.critical(self, "权限错误", error_msg)
                    self.cluster_position_list=[]
                    
                except ValueError as e:
                    error_msg = f"数据格式错误: {e}"
                    print(error_msg)
                    self.append_info(error_msg)
                    QMessageBox.critical(self, "数据格式错误", error_msg)
                    self.cluster_position_list=[]

                except Exception as e:
                    error_msg = f"读取文件时发生未知错误: {e}"
                    print(error_msg)
                    self.append_info(error_msg)
                    QMessageBox.critical(self, "未知错误", error_msg)
                    self.cluster_position_list=[]
    
    # 启动 RViz2
    def rviz_process(self):
        # 启动 RViz2
        self._rviz_process = QProcess(self)
        self._rviz_process.start("rviz2")
        self.append_info(f"启动RViz2")

    def set_boot_pose_command(self):
        """
        GUI 操作：标记当前选中 USV 的上电原点（发送信号到 GroundStationNode）
        """
        selected_indexes = self.ui.cluster_tableView.selectedIndexes()
        if not selected_indexes:
            self.append_info("请先在集群表格中选择一个USV来标记 boot pose")
            return
        selected_row = selected_indexes[0].row()
        model = self.ui.cluster_tableView.model()
        if model is None:
            self.append_info("集群表格模型未初始化，无法标记 boot pose")
            return
        index0 = model.index(selected_row, 0)
        usv_id = model.data(index0) if index0.isValid() else None
        if not usv_id:
            self.append_info("无法获取选中行的 USV ID")
            return
        try:
            # 通过 ros_signal 发出标记 boot pose 的信号
            self.ros_signal.set_boot_pose.emit(usv_id)
            self.append_info(f"已请求标记 USV {usv_id} 的 boot_pose（上电原点）")
        except Exception as e:
            self.append_info(f"标记 boot_pose 出错: {e}")

    def set_all_boot_pose_command(self):
        """
        GUI 操作：对当前集群列表中的所有 USV 一次性标记 boot pose。
        """
        try:
            if not self.usv_cluster_list:
                self.append_info("集群列表为空，无法批量标记 boot pose")
                return
            usv_ids = self._extract_namespaces(self.usv_cluster_list)
            if not usv_ids:
                self.append_info("未找到有效的 USV ID，取消批量标记")
                return
            # 通过 ros_signal 发出批量标记信号
            try:
                self.ros_signal.set_boot_pose_all.emit(usv_ids)
                self.append_info(f"已请求为以下 USV 批量标记 boot_pose: {usv_ids}")
            except Exception as e:
                self.append_info(f"批量标记 boot_pose 出错: {e}")
        except Exception as e:
            self.append_info(f"执行批量标记时发生错误: {e}")


    # 切换集群任务运行状态（运行/暂停）
    def toggle_cluster_task(self):
        """
        切换集群任务的运行状态：运行/暂停
        """
        if not self.cluster_task_running and not self.cluster_position_list:
            self.ui.warning_textEdit.append( "请先导入集群目标点数据")
            return
        # 如果任务未开始且有目标点数据，则开始任务
        if not self.cluster_task_running and self.cluster_position_list:
            self.start_cluster_task()
        # 如果任务正在运行，则切换暂停状态
        elif self.cluster_task_running:
            self.cluster_task_paused = not self.cluster_task_paused
            if self.cluster_task_paused:
                self.ui.send_cluster_point_pushButton.setText("cluster continue")
                self.append_info("集群任务已暂停")
                # 发送暂停信号给ROS节点（如果需要实现）
                # 这里可以发送一个特殊的信号或清空目标点列表来暂停任务
                self.ros_signal.cluster_target_point_command.emit([])  # 发送空列表表示暂停
            else:
                self.ui.send_cluster_point_pushButton.setText("cluster pause")
                self.append_info("集群任务已继续")
                # 重新发送任务数据以恢复任务
                self.ros_signal.cluster_target_point_command.emit(self.cluster_position_list)

    # 发送集群目标点命令
    def send_cluster_point_command(self):
        """
        发送集群目标点命令
        """
        # 检查是否有集群列表
        if not self.cluster_position_list:
            self.ui.warning_textEdit.append("集群列表为空")
            return
            
        # 创建副本以避免修改原始列表时的遍历问题
        filtered_list = self.cluster_position_list.copy()
        # 将 usv_departed_list 转换为集合以提高查找效率
        departed_ids = {item.get('namespace') if isinstance(item, dict) else item 
                        for item in self.usv_departed_list}
        # 移除匹配 usv_id 的 USV 数据
        filtered_list = [usv for usv in filtered_list if usv.get('usv_id', '') not in departed_ids]
        if not filtered_list:
            self.ui.warning_textEdit.append("集群列表为空（所有 USV 均在离群列表中）")          
            return 
            
        # 弹窗确认
        reply = QMessageBox.question(
            self,
            f"确认执行",
            f"即将执行 {len(filtered_list)} 个 USV 的集群任务。\n是否继续?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            try:
                # 更新 cluster_position_list
                self.cluster_position_list = filtered_list
                # 设置任务状态
                self.cluster_task_running = True
                self.cluster_task_paused = False
                # 更新按钮文本
                self.ui.send_cluster_point_pushButton.setText("cluster pause")
                # 发送 ROS 信号
                self.ros_signal.cluster_target_point_command.emit(self.cluster_position_list)
                self.append_info("集群任务已开始执行！")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"任务启动失败: {e}")
                # 出错时重置状态
                self.cluster_task_running = False
                self.cluster_task_paused = False
                self.update_cluster_task_button_state()
        else:
            QMessageBox.information(self, "取消", f"任务启动已取消")

    # 开始集群任务
    def start_cluster_task(self):
        """
        开始执行集群任务
        """
        self.append_info(f"开始执行集群目标点任务")
        # 检查是否有集群列表
        if not self.cluster_position_list:
            self.ui.warning_textEdit.append("集群列表为空")
            return
        # 创建副本以避免修改原始列表时的遍历问题
        filtered_list = self.cluster_position_list.copy()
        # 将 usv_departed_list 转换为集合以提高查找效率
        departed_ids = {item.get('namespace') if isinstance(item, dict) else item 
                        for item in self.usv_departed_list}
        # 移除匹配 usv_id 的 USV 数据
        filtered_list = [usv for usv in filtered_list if usv.get('usv_id', '') not in departed_ids]
        if not filtered_list:
            self.ui.warning_textEdit.append("集群列表为空（所有 USV 均在离群列表中）")          
            return 
        # 弹窗确认
        reply = QMessageBox.question(
            self,
            f"确认执行",
            f"即将执行 {len(filtered_list)} 个 USV 的集群任务。\n是否继续?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            try:
                # 更新 cluster_position_list
                self.cluster_position_list = filtered_list
                # 设置任务状态
                self.cluster_task_running = True
                self.cluster_task_paused = False
                # 更新按钮文本
                self.ui.send_cluster_point_pushButton.setText("cluster pause")
                # 发送 ROS 信号
                self.ros_signal.cluster_target_point_command.emit(self.cluster_position_list)
                self.append_info("集群任务已开始执行！")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"任务启动失败: {e}")
                # 出错时重置状态
                self.cluster_task_running = False
                self.cluster_task_paused = False
                self.update_cluster_task_button_state()
        else:
            QMessageBox.information(self, "取消", f"任务启动已取消")
    
    # 停止集群任务
    def stop_cluster_task(self):
        """
        停止集群任务执行
        """
        # 如果没有正在运行的集群任务，直接返回
        if not self.cluster_task_running:
            self.append_info("当前没有正在运行的集群任务")
            return
            
        # 弹窗确认
        reply = QMessageBox.question(
            self,
            f"确认停止",
            f"确定要停止当前集群任务吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            # 发送停止信号给ROS节点
            self.ros_signal.cluster_target_point_command.emit([])  # 发送空列表表示停止
            
            # 重置任务状态
            self.cluster_task_running = False
            self.cluster_task_paused = False
            # 更新按钮文本
            self.ui.send_cluster_point_pushButton.setText("cluster start")
            self.append_info("集群任务已停止！")
            
            # 清空集群位置列表
            self.cluster_position_list = []
        else:
            self.append_info("取消停止操作")

    def is_cluster_task_active(self):
        """
        检查集群任务是否处于活动状态（运行中且未暂停）
        
        Returns:
            bool: 如果任务正在运行且未暂停则返回True，否则返回False
        """
        return self.cluster_task_running and not self.cluster_task_paused

    def update_cluster_task_button_state(self):
        """
        根据任务状态更新集群任务按钮的文本
        """
        if not self.cluster_task_running:
            self.ui.send_cluster_point_pushButton.setText("cluster start")
        elif self.cluster_task_paused:
            self.ui.send_cluster_point_pushButton.setText("cluster continue")
        else:
            self.ui.send_cluster_point_pushButton.setText("cluster pause")

    # 离群目标点命令
    def send_departed_point_command(self):
        x = self.ui.set_departed_x_doubleSpinBox.value()
        y = self.ui.set_departed_y_doubleSpinBox.value() 
        z = 0.0  # 默认z坐标为0
        # 检查是否存在z坐标控件
        if hasattr(self.ui, 'set_departed_z_doubleSpinBox'):
            z = self.ui.set_departed_z_doubleSpinBox.value()  # 添加z坐标
        departed_target_list = []
        for usv_item in self.usv_departed_list:
            # 从usv_item中提取usv_id
            if isinstance(usv_item, dict):
                usv_id = usv_item.get('namespace')
                if usv_id:
                    # 使用Action接口发送导航目标点
                    self.ros_signal.nav_status_update.emit(usv_id, "执行中")
                    # 注意：这里需要调用Node中的方法来发送Action目标
                    # 由于GUI和Node分离，我们通过信号传递信息
                    # 实际的Action调用在Node中进行
                    departed_target_list.append({
                        'usv_id': usv_id,
                        'position': {'x': x, 'y': y, 'z': z},  # 添加z坐标
                        'yaw': 0.0
                    })
        # 发送离群目标点信号，让Node处理Action调用
        self.ros_signal.departed_target_point_command.emit(departed_target_list)
        self.append_info(f"发送离群目标点: x={x}, y={y}, z={z} 到 {len(departed_target_list)} 个USV")

    # 添加到集群列表
    def add_cluster_command(self):
        """
        将选中的离群USV添加到集群列表
        """
        self.append_info(f"添加到集群列表")
        selected_indexes = self.ui.departed_tableView.selectedIndexes()
        if not selected_indexes:
            self.append_info("请先选择一行")
            return

        selected_row = selected_indexes[0].row()
        model = self.ui.departed_tableView.model()
        if model is None:
            self.append_info("错误：离群表格模型未初始化")
            return

        # 获取整行状态数据
        state = {}
        headers = self.TABLE_HEADERS
        for col, key in enumerate(headers):
            index = model.index(selected_row, col)
            state[key] = model.data(index) or "Unknown"

        usv_id = state.get("编号")  # 编号对应namespace
        if not usv_id:
            self.append_info("错误：无法获取 USV ID")
            return

        try:
            self.usv_cluster_list.append(state)
            # 从 usv_departed_list 移除匹配的项
            for item in self.usv_departed_list:
                if item.get('namespace') == usv_id:
                    self.usv_departed_list.remove(item)               
            self.update_cluster_table(self.usv_cluster_list)
            self.update_departed_table(self.usv_departed_list)
            self.append_info(f"添加设备 {usv_id} 到集群列表")
            QMessageBox.information(self, "操作成功", f"设备 {usv_id} 已添加到集群列表")
        except Exception as e:
            error_msg = f"错误：添加设备失败 - {str(e)}"
            self.append_info(error_msg)
            QMessageBox.critical(self, "操作失败", error_msg)
   
    # 离开集群列表
    def quit_cluster_command(self):
        """
        将选中的集群USV移到离群列表
        """
        self.append_info(f"离开集群列表")
        selected_indexes = self.ui.cluster_tableView.selectedIndexes()
        self.append_info(f'选中的标签：{selected_indexes}')

        if not selected_indexes:
            self.append_info("请先选择一行")
            return

        selected_row = selected_indexes[0].row()
        model = self.ui.cluster_tableView.model()
        if model is None:
            self.append_info("错误：集群表格模型未初始化")
            return

        # 获取整行状态数据
        state = {}
        headers = self.TABLE_HEADERS
        for col, key in enumerate(headers):
            index = model.index(selected_row, col)
            state[key] = model.data(index) or "Unknown"

        usv_id = state.get("编号")  # 编号对应namespace
        if not usv_id:
            self.append_info("错误：无法获取 USV ID")
            return

        try:
            # 从 usv_cluster_list 移除匹配的字典
            for item in self.usv_cluster_list:
                if item.get('namespace') == usv_id:
                    self.usv_cluster_list.remove(item)
                    self.usv_departed_list.append(item)  # 添加状态字典
                    break
            self.update_cluster_table(self.usv_cluster_list)
            self.update_departed_table(self.usv_departed_list)
            self.append_info(f"添加设备 {usv_id} 到离群列表")
            QMessageBox.information(self, "操作成功", f"设备 {usv_id} 已添加到离群列表")
        except Exception as e:
            error_msg = f"错误：移除设备失败 - {str(e)}"
            self.append_info(error_msg)
            QMessageBox.critical(self, "操作失败", error_msg)
    
    def cluster_progress_callback(self, progress_info):
        """
        处理集群任务进度更新
        
        Args:
            progress_info (dict): 进度信息字典
        """
        # 保存进度信息
        self.cluster_progress_info = progress_info
        
        # 更新UI显示
        current_step = progress_info.get('current_step', 0)
        total_steps = progress_info.get('total_steps', 0)
        total_usvs = progress_info.get('total_usvs', 0)
        acked_usvs = progress_info.get('acked_usvs', 0)
        ack_rate = progress_info.get('ack_rate', 0)
        elapsed_time = progress_info.get('elapsed_time', 0)
        
        # 更新标签显示
        progress_text = (f"集群任务进度: 步骤 {current_step}/{total_steps}, "
                        f"完成 {acked_usvs}/{total_usvs} 个USV ({ack_rate*100:.1f}%), "
                        f"耗时 {elapsed_time:.1f}s")
        
        self.append_info(progress_text)
        
        # 如果任务已完成，重置按钮状态
        if current_step > total_steps or (current_step == total_steps and ack_rate >= 1.0):
            self.cluster_task_running = False
            self.cluster_task_paused = False
            self.ui.send_cluster_point_pushButton.setText("cluster start")
            
        # 如果有进度条控件，可以在这里更新进度条
        # 例如: self.ui.cluster_progress_bar.setValue(int(ack_rate * 100))

    # 接收所有在线的usv状态
    def receive_state_callback(self, msg):
        """
        接收并处理所有在线USV的状态信息
        
        Args:
            msg (list): 包含USV状态信息的字典列表
        """
        # 快速入缓存并返回（保证该回调非常快，避免阻塞 Qt 线程）
        if not isinstance(msg, list):
            # 可能偶发错误，记录到 warning 区并退出
            self.ui.warning_textEdit.append("接收到的数据类型错误，期望为列表")
            return

        # 将来自后端的列表视为“全量快照”，用其覆盖本地缓存，确保下线 USV 行能及时移除
        new_cache = {}
        for ns in msg:
            if isinstance(ns, dict) and ns.get('namespace'):
                new_cache[ns.get('namespace')] = ns
        self._usv_state_cache = new_cache

        # 标记需要刷新 UI（定时器将在 GUI 线程中批量刷新）
        self._usv_state_dirty = True
    
    def _update_departed_list_status(self):
        """
        更新离群USV列表状态信息
        """
        for i, departed in enumerate(self.usv_departed_list):
            departed_id = departed.get('namespace') if isinstance(departed, dict) else departed
            state = next((ns for ns in self.usv_online_list if ns.get('namespace') == departed_id), None)
            if state:
                self.usv_departed_list[i] = state
            else:
                # 如果离群 USV 已下线，保留现有数据
                if not isinstance(departed, dict):
                    self.usv_departed_list[i] = {'namespace': departed_id}  # 转换为字典
                self.append_info(f"警告：离群设备 {departed_id} 未在在线列表中")
    
    def _update_cluster_list(self):
        """
        更新集群USV列表，移除离群USV并保留手动添加的USV
        """
        # 创建 temp_list，移除离群 USV
        temp_list = self.usv_online_list.copy()
        departed_ids = {departed.get('namespace') for departed in self.usv_departed_list 
                       if isinstance(departed, dict)}
        
        # 移除离群USV
        temp_list = [ns for ns in temp_list if ns.get('namespace') not in departed_ids]
        
        # 保留现有 usv_cluster_list 的手动添加 USV
        current_cluster = self.usv_cluster_list.copy()
        current_cluster_ids = {item.get('namespace') for item in current_cluster 
                              if isinstance(item, dict) and item.get('namespace')}
        
        # 更新 usv_cluster_list
        self.usv_cluster_list = []
        for item in current_cluster:
            if isinstance(item, dict) and item.get('namespace'):
                # 仅在仍在线时保留该 USV（下线则从集群列表移除）
                state = next((ns for ns in self.usv_online_list 
                             if ns.get('namespace') == item.get('namespace')), None)
                if state is not None:
                    self.usv_cluster_list.append(state)
        for ns in temp_list:
            if ns.get('namespace') not in current_cluster_ids:
                self.usv_cluster_list.append(ns)

    def _flush_state_cache_to_ui(self):
        """
        将缓存中的 USV 状态批量刷新到 UI（在 GUI 线程中由 QTimer 周期性调用）
        """
        # 如果没有新的数据，不执行任何操作
        if not self._usv_state_dirty:
            return

        # 用缓存构建在线列表（保证顺序无关紧要）
        try:
            self.usv_online_list = list(self._usv_state_cache.values())

            # 更新离群和集群列表（复用现有逻辑）
            self._update_departed_list_status()
            self._update_cluster_list()

            # 批量更新表格 UI
            self.update_cluster_table(self.usv_cluster_list)
            self.update_departed_table(self.usv_departed_list)
            self.update_selected_table_row()

        except Exception as e:
            # 出错时在 warning 区提示并清理标志以避免持续异常
            try:
                self.ui.warning_textEdit.append(f"刷新 UI 时出错: {e}")
            except Exception:
                pass
        finally:
            # 清除脏标志（下一轮等待新的回调）
            self._usv_state_dirty = False

    # --------------------------------------------------
    # info_textEdit 缓冲写入 API（速率和大小限制）
    # --------------------------------------------------
    def append_info(self, text: str):
        """
        将文本放入 info 缓冲，由定时器周期性刷新到 UI，避免频繁写入导致卡顿。
        """
        try:
            # 将文本追加到缓冲队列
            self._info_buffer.append(str(text))
        except Exception:
            # 如果缓冲出错，作为回退直接写入（避免递归调用 append_info）
            try:
                self.ui.info_textEdit.append(str(text))
            except Exception:
                pass

    def _flush_info_buffer(self):
        """
        定时器回调：把缓冲中的若干行追加到 info_textEdit，并保持最大行数。
        """
        try:
            if not self._info_buffer:
                return

            # 合并若干条消息一次性写入，减少 UI 更新次数
            to_write = []
            # 每次最多写入 50 条，避免一次性写入太多
            max_batch = 50
            for _ in range(min(max_batch, len(self._info_buffer))):
                to_write.append(self._info_buffer.popleft())

            for line in to_write:
                try:
                    self.ui.info_textEdit.append(line)
                except Exception:
                    pass

            # 控制最大行数
            try:
                doc = self.ui.info_textEdit.document()
                block_count = None
                if doc is not None:
                    bc = getattr(doc, 'blockCount', None)
                    if callable(bc):
                        try:
                            block_count = bc()
                        except Exception:
                            block_count = None

                if block_count is not None:
                    if isinstance(block_count, (int, float, str)):
                        try:
                            block_count = int(block_count)
                        except (ValueError, TypeError):
                            block_count = None
                    else:
                        block_count = None

                if block_count is not None and block_count > self._info_max_lines:
                    # 删除最旧的若干行，保持大小
                    cursor = self.ui.info_textEdit.textCursor()
                    cursor.movePosition(cursor.Start)
                    # 计算要删除的块数
                    remove_count = block_count - self._info_max_lines
                    for _ in range(remove_count):
                        cursor.select(cursor.LineUnderCursor)
                        cursor.removeSelectedText()
                        cursor.deleteChar()  # 删除换行
            except Exception:
                # 修剪失败时不影响主流程
                pass

        except Exception:
            # 忽略刷新中的异常以保证稳定性
            try:
                self._info_buffer.clear()
            except Exception:
                pass
    
    def get_battery_status_text(self, status_value):
        """
        将电池状态数字映射为文字描述
        
        Args:
            status_value: 电池状态值（数字或字符串）
            
        Returns:
            str: 电池状态文字描述
        """
        # 定义数字到文字的映射表
        status_map = {
            0: "未知",
            1: "放电",
            2: "充电",
            3: "充满",
            4: "异常"
        }
        
        # 处理输入值，尝试转换为整数
        try:
            if isinstance(status_value, str):
                # 如果是"true"/"false"这样的字符串，先转为数字
                if status_value.lower() == 'true':
                    status_value = 1
                elif status_value.lower() == 'false':
                    status_value = 0
                else:
                    status_value = int(float(status_value))
            elif isinstance(status_value, bool):
                status_value = 1 if status_value else 0
            else:
                status_value = int(float(status_value))
        except (ValueError, TypeError):
            return "未知"
            
        return status_map.get(status_value, "未知")

    def update_cluster_table(self, state_list):
        """
        更新集群表格
        
        Args:
            state_list: 状态列表
        """
        try:
            model = self.cluster_table_model

            # 构建当前表中 namespace -> row 的映射
            current_ns_to_row = {}
            for row in range(model.rowCount()):
                item = model.item(row, 0)
                ns = item.text() if item is not None else ''
                current_ns_to_row[ns] = row

            # 新状态的 namespace 列表与集合
            new_ns_list = [s.get('namespace') for s in state_list if isinstance(s, dict) and s.get('namespace')]
            new_ns_set = set(new_ns_list)

            # 删除在当前表中但不在新状态集合的行（从后往前删除以保持索引正确）
            rows_to_remove = [r for ns, r in current_ns_to_row.items() if ns not in new_ns_set]
            for r in sorted(rows_to_remove, reverse=True):
                model.removeRow(r)

            # 重新构建映射（因为删除可能改变索引）
            current_ns_to_row = {}
            for row in range(model.rowCount()):
                item = model.item(row, 0)
                ns = item.text() if item is not None else ''
                current_ns_to_row[ns] = row

            # 更新已有行或追加新行
            for state in state_list:
                if not isinstance(state, dict):
                    continue
                ns = state.get('namespace', 'Unknown')
                # 格式化各列文本，使用安全转换以避免异常
                try:
                    bp = state.get('battery_percentage', 0)
                    bp_text = f"{float(bp):.1f}%"
                except Exception:
                    bp_text = "Unknown"

                voltage = state.get('battery_voltage', 'Unknown')
                try:
                    voltage_text = f"{float(voltage):.1f}"
                except Exception:
                    voltage_text = str(voltage) if voltage is not None else "Unknown"

                try:
                    px = float(state.get('position', {}).get('x', 0.0))
                    py = float(state.get('position', {}).get('y', 0.0))
                    pz = float(state.get('position', {}).get('z', 0.0))
                    pos_text = f"({px:.2f}, {py:.2f}, {pz:.2f})"
                except Exception:
                    pos_text = "(Unknown, Unknown, Unknown)"

                try:
                    vx = float(state.get('velocity', {}).get('linear', {}).get('x', 0.0))
                    vy = float(state.get('velocity', {}).get('linear', {}).get('y', 0.0))
                    vz = float(state.get('velocity', {}).get('linear', {}).get('z', 0.0))
                    vel_text = f"({vx:.2f}, {vy:.2f}, {vz:.2f})"
                except Exception:
                    vel_text = "(Unknown, Unknown, Unknown)"

                try:
                    yaw_text = f"{float(state.get('yaw', 0.0)):.2f}"
                except Exception:
                    yaw_text = "Unknown"

                nav_text = self.usv_nav_status.get(ns, self.NAV_STATUS_IDLE)

                try:
                    temp_text = f"{float(state.get('temperature', 0.0)):.1f}"
                except Exception:
                    temp_text = "Unknown"

                # 获取电池状态文字描述
                power_status = state.get('power_supply_status', 0)
                power_status_text = self.map_power_supply_status(power_status)

                cells = [
                    ns,
                    state.get('mode', 'Unknown'),
                    str(state.get('connected', 'Unknown')),
                    str(state.get('armed', 'Unknown')),
                    str(state.get('guided', 'Unknown')),  # 引导模式状态
                    voltage_text,  # 电压V
                    bp_text,  # 电量%
                    power_status_text,  # 电池状态
                    pos_text,  # 坐标
                    vel_text,  # 速度
                    yaw_text,  # 偏角
                    nav_text,  # 导航状态
                    temp_text,  # 温度
                ]

                if ns in current_ns_to_row:
                    row = current_ns_to_row[ns]
                    # 只更新变化的列，减少重绘
                    for col, text in enumerate(cells):
                        existing_item = model.item(row, col)
                        existing_text = existing_item.text() if existing_item is not None else None
                        if existing_text != str(text):
                            model.setItem(row, col, QStandardItem(str(text)))
                else:
                    # 追加新行
                    row = model.rowCount()
                    model.insertRow(row)
                    for col, text in enumerate(cells):
                        model.setItem(row, col, QStandardItem(str(text)))

        except Exception as e:
            try:
                self.append_info(f"更新集群表格失败: {e}")
            except Exception:
                pass

    def update_departed_table(self, state_list):
        """
        更新离群表格
        
        Args:
            state_list: 状态列表
        """
        try:
            model = self.departed_table_model

            # 当前表中 namespace -> row 映射
            current_ns_to_row = {}
            for row in range(model.rowCount()):
                item = model.item(row, 0)
                ns = item.text() if item is not None else ''
                current_ns_to_row[ns] = row

            # 新状态 ns 列表与集合
            new_ns_list = [s.get('namespace') for s in state_list if isinstance(s, dict) and s.get('namespace')]
            new_ns_set = set(new_ns_list)

            # 删除在表中但不在新数据中的行（倒序删除）
            rows_to_remove = [r for ns, r in current_ns_to_row.items() if ns not in new_ns_set]
            for r in sorted(rows_to_remove, reverse=True):
                model.removeRow(r)

            # 重新构建映射
            current_ns_to_row = {}
            for row in range(model.rowCount()):
                item = model.item(row, 0)
                ns = item.text() if item is not None else ''
                current_ns_to_row[ns] = row

            # 更新或追加
            for state in state_list:
                if not isinstance(state, dict):
                    continue
                ns = state.get('namespace', 'Unknown')

                try:
                    bp = state.get('battery_percentage', 0)
                    bp_text = f"{float(bp):.1f}%"
                except Exception:
                    bp_text = "Unknown"

                voltage = state.get('battery_voltage', 'Unknown')
                try:
                    voltage_text = f"{float(voltage):.1f}"
                except Exception:
                    voltage_text = str(voltage) if voltage is not None else "Unknown"

                try:
                    px = float(state.get('position', {}).get('x', 0.0))
                    py = float(state.get('position', {}).get('y', 0.0))
                    pz = float(state.get('position', {}).get('z', 0.0))
                    pos_text = f"({px:.2f}, {py:.2f}, {pz:.2f})"
                except Exception:
                    pos_text = "(Unknown, Unknown, Unknown)"

                try:
                    vx = float(state.get('velocity', {}).get('linear', {}).get('x', 0.0))
                    vy = float(state.get('velocity', {}).get('linear', {}).get('y', 0.0))
                    vz = float(state.get('velocity', {}).get('linear', {}).get('z', 0.0))
                    vel_text = f"({vx:.2f}, {vy:.2f}, {vz:.2f})"
                except Exception:
                    vel_text = "(Unknown, Unknown, Unknown)"

                try:
                    yaw_text = f"{float(state.get('yaw', 0.0)):.2f}"
                except Exception:
                    yaw_text = "Unknown"

                nav_text = self.usv_nav_status.get(ns, self.NAV_STATUS_IDLE)

                try:
                    temp_text = f"{float(state.get('temperature', 0.0)):.1f}"
                except Exception:
                    temp_text = "Unknown"

                # 获取电池状态文字描述
                power_status = state.get('power_supply_status', 0)
                power_status_text = self.map_power_supply_status(power_status)

                cells = [
                    ns,
                    state.get('mode', 'Unknown'),
                    str(state.get('connected', 'Unknown')),
                    str(state.get('armed', 'Unknown')),
                    str(state.get('guided', 'Unknown')),  # 引导模式状态
                    voltage_text,  # 电压V
                    bp_text,  # 电量%
                    power_status_text,  # 电池状态
                    pos_text,  # 坐标
                    vel_text,  # 速度
                    yaw_text,  # 偏角
                    nav_text,  # 导航状态
                    temp_text,  # 温度
                ]

                if ns in current_ns_to_row:
                    row = current_ns_to_row[ns]
                    for col, text in enumerate(cells):
                        existing_item = model.item(row, col)
                        existing_text = existing_item.text() if existing_item is not None else None
                        if existing_text != str(text):
                            model.setItem(row, col, QStandardItem(str(text)))
                else:
                    row = model.rowCount()
                    model.insertRow(row)
                    for col, text in enumerate(cells):
                        model.setItem(row, col, QStandardItem(str(text)))

        except Exception as e:
            try:
                self.append_info(f"更新离群表格失败: {e}")
            except Exception:
                pass

    # 更新选中行数据
    def update_selected_table_row(self):
        try:
            # 获取选中的行并找到对应的 namespace（第一列）
            selected_indexes = self.ui.cluster_tableView.selectedIndexes()
            if not selected_indexes:
                # 没有选中时不做任何更新
                return
            selected_row = selected_indexes[0].row()
            model = self.ui.cluster_tableView.model()
            if model is None:
                return

            index0 = model.index(selected_row, 0)
            namespace = model.data(index0) if index0.isValid() else None
            if not namespace:
                return

            # 优先从缓存获取最新状态，其次从 usv_cluster_list 查找
            state = self._usv_state_cache.get(namespace)
            if state is None:
                state = next((s for s in self.usv_cluster_list if isinstance(s, dict) and s.get('namespace') == namespace), None)

            if state is None:
                # 无状态数据时清空显示
                self.ui.usv_id_label.setText("Unknown")
                self.ui.usv_x_label.setText("Unknown")
                self.ui.usv_y_label.setText("Unknown")
                self.ui.usv_z_label.setText("Unknown")
                self.ui.usv_yaw_label.setText("Unknown")
                return

            # 直接读取并格式化数值
            self.ui.usv_id_label.setText(str(state.get('namespace', 'Unknown')))
            pos = state.get('position', {}) or {}
            try:
                x = float(pos.get('x', 0.0))
            except Exception:
                x = None
            try:
                y = float(pos.get('y', 0.0))
            except Exception:
                y = None
            try:
                z = float(pos.get('z', 0.0))
            except Exception:
                z = None

            yaw_val = state.get('yaw')
            try:
                yaw_val = float(yaw_val) if yaw_val is not None else None
            except Exception:
                yaw_val = None

            self.ui.usv_x_label.setText(f"{x:.2f}" if x is not None else "Unknown")
            self.ui.usv_y_label.setText(f"{y:.2f}" if y is not None else "Unknown")
            self.ui.usv_z_label.setText(f"{z:.2f}" if z is not None else "Unknown")
            self.ui.usv_yaw_label.setText(f"{yaw_val:.2f}" if yaw_val is not None else "Unknown")

        except Exception as e:
            try:
                self.append_info(f"错误：获取选中行数据失败 - {str(e)}")
            except Exception:
                pass
            self.ui.usv_id_label.setText("Unknown")
            self.ui.usv_x_label.setText("Unknown")
            self.ui.usv_y_label.setText("Unknown")
            self.ui.usv_z_label.setText("Unknown")
            self.ui.usv_yaw_label.setText("Unknown")
   
    # 声音开始命令
    def sound_start_command(self):
        self.ros_signal.str_command.emit('sound_start')
        self.append_info(f"发送命令: sound_start")
    
    # 声音停止命令
    def sound_stop_command(self):
        self.ros_signal.str_command.emit('sound_stop')
        self.append_info(f"发送命令:sound_stop")
   
    # 颈部摆动命令
    def neck_swinging_command(self):
        """
        发送颈部摆动命令
        """
        self.ros_signal.str_command.emit('neck_swinging')
        self.append_info(f"发送命令: neck_swinging")

   
    # 颈部停止命令
    def neck_stop_command(self):
        """
        发送颈部停止命令
        """
        self.ros_signal.str_command.emit('neck_stop')
        self.append_info(f"发送命令: neck_stop")
   
    # led1命令
    def led1_command(self):
        self.ros_signal.str_command.emit('color_switching')
        self.append_info(f"发送命令: color_switching")
    
    # led2命令
    def led2_command(self):
        self.ros_signal.str_command.emit('random_color_change')
        self.append_info(f"发送命令:random_color_change")
    
    # led3命令    
    def led3_command(self):
        try:
            # 使用非阻塞的 QColorDialog 实例，以避免阻塞 GUI 事件循环
            dlg = QColorDialog(self)
            # 保存对话框引用，避免被垃圾回收
            self._color_dialog = dlg
            dlg.setOption(QColorDialog.ShowAlphaChannel, False)
            # 当用户选择颜色时触发 color_selected_handler
            dlg.colorSelected.connect(self._color_selected_handler)
            dlg.open()
        except Exception:
            # 回退到同步调用（兼容旧平台）
            color = QColorDialog.getColor()  # type: ignore
            if not color.isValid():
                self.append_info("颜色选择无效")
                return
            r = color.red()
            g = color.green()
            b = color.blue()
            color_str = f"color_select|{r},{g},{b}"
            self.ros_signal.str_command.emit(color_str)
            self.append_info(f"发送led3命令: {color_str}")

    def _color_selected_handler(self, color):
        """
        颜色对话框回调：在 GUI 线程执行，将选中的颜色命令发送到 ROS。
        """
        try:
            if not color.isValid():
                self.append_info("颜色选择无效")
                return
            r = color.red()
            g = color.green()
            b = color.blue()
            color_str = f"color_select|{r},{g},{b}"
            # 通过信号发送命令（快速，不阻塞）
            self.ros_signal.str_command.emit(color_str)
            self.append_info(f"发送led3命令: {color_str}")
        except Exception as e:
            self.append_info(f"处理颜色选择时出错: {e}")
   
    # 停止灯光命令
    def light_stop_command(self):
        self.ros_signal.str_command.emit('led_off')
        self.append_info(f"发送命令: led_off")
    
    # 显示 USV 绘图窗口
    def show_usv_plot_window(self):
        # 检查是否已经存在绘图窗口实例
        if self.usv_plot_window_ref and self.usv_plot_window_ref():
            # 如果存在，将其激活到前台
            window = self.usv_plot_window_ref()
            if window:
                window.raise_()
                window.activateWindow()
        else:
            # 如果不存在，创建新窗口并保存弱引用
            usv_plot_window = UsvPlotWindow(lambda: self.usv_online_list, self)
            self.usv_plot_window_ref = weakref.ref(usv_plot_window)
            usv_plot_window.show()

    def refresh_table_header(self):
        """
        刷新表格表头
        """
        # 使用统一的 TABLE_HEADERS 类变量来设置表头
        self.cluster_table_model.setHorizontalHeaderLabels(self.TABLE_HEADERS)
        self.departed_table_model.setHorizontalHeaderLabels(self.TABLE_HEADERS)

    def update_nav_status(self, usv_id, status):
        """
        更新USV的导航状态
        
        Args:
            usv_id (str): USV标识符
            status (str): 导航状态
        """
        # 更新导航状态字典
        self.usv_nav_status[usv_id] = status
        
        # 更新表格显示
        self.update_cluster_table(self.usv_cluster_list)
        self.update_departed_table(self.usv_departed_list)

    def handle_navigation_feedback(self, usv_id, feedback):
        """
        处理导航反馈信息
        
        Args:
            usv_id (str): USV标识符
            feedback: 导航反馈数据
        """
        # 在信息文本框中显示导航反馈
        self.ui.cluster_navigation_feedback_info_textEdit.append(
            f"USV {usv_id} 导航反馈 - "
            f"距离目标: {feedback.distance_to_goal:.2f}m, "
            f"航向误差: {feedback.heading_error:.2f}度, "
            f"预计剩余时间: {feedback.estimated_time:.2f}秒"
        )
        
    # 集群解锁命令 
    def set_cluster_arming_command(self):
        """
        发送集群解锁命令给所有集群USV
        """
        try:
            self.usv_cluster_namespace_list = self._extract_namespaces(self.usv_cluster_list)
            self.ros_signal.arm_command.emit(self.usv_cluster_namespace_list)
            self.append_info(f"集群解锁命令已发送: {self.usv_cluster_namespace_list}")
        except Exception as e:
            error_msg = f"发送集群解锁命令失败: {e}"
            self.append_info(error_msg)

    # 集群设置manual模式命令
    def set_cluster_manual_command(self):   
        """
        发送集群设置manual模式命令给所有集群USV
        """
        try:
            self.usv_cluster_namespace_list = self._extract_namespaces(self.usv_cluster_list)
            self.ros_signal.manual_command.emit(self.usv_cluster_namespace_list)
            self.append_info(f"集群设置manual模式命令已发送: {self.usv_cluster_namespace_list}")
        except Exception as e:
            error_msg = f"发送集群manual模式命令失败: {e}"
            self.append_info(error_msg)

    # 集群加锁命令  
    def cluster_disarming_command(self):
        """
        发送集群加锁命令给所有集群USV
        """
        try:
            self.usv_cluster_namespace_list = self._extract_namespaces(self.usv_cluster_list)
            self.ros_signal.disarm_command.emit(self.usv_cluster_namespace_list)
            self.append_info(f"集群加锁命令已发送: {self.usv_cluster_namespace_list}")
        except Exception as e:
            error_msg = f"发送集群加锁命令失败: {e}"
            self.append_info(error_msg)

    # 集群设置guided模式命令 
    def set_cluster_guided_command(self):
        """
        发送集群设置guided模式命令给所有集群USV
        """
        try:
            self.usv_cluster_namespace_list = self._extract_namespaces(self.usv_cluster_list)
            # 发送 guided 模式命令
            self.ros_signal.guided_command.emit(self.usv_cluster_namespace_list)
            self.append_info(f"集群设置guided模式命令已发送: {self.usv_cluster_namespace_list}")
        except Exception as e:
            error_msg = f"发送集群guided模式命令失败: {e}"
            self.append_info(error_msg)

    # 离群解锁命令
    def departed_arming_command(self):
        """
        发送离群解锁命令给所有离群USV
        """
        try:
            # 发送离群解锁命令给所有离群USV
            self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
            # 发送离群解锁命令       
            self.ros_signal.arm_command.emit(self.usv_departed_namespace_list)
            self.append_info(f"离群解锁命令已发送: {self.usv_departed_namespace_list}") 
        except Exception as e:
            error_msg = f"发送离群解锁命令失败: {e}"
            self.append_info(error_msg)

    # 离群加锁命令
    def departed_disarming_command(self):
        """
        发送离群加锁命令给所有离群USV
        """
        try:
            self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
            self.ros_signal.disarm_command.emit(self.usv_departed_namespace_list)
            self.append_info(f"离群加锁命令已发送: {self.usv_departed_namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群加锁命令失败: {e}")

    # 离群设置Steering模式命令
    def set_departed_steering_command(self):
        """
        发送离群设置Steering模式命令给所有离群USV
        """
        try:
            self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
            # 发送离群设置Steering模式命令       
            self.ros_signal.steering_command.emit(self.usv_departed_namespace_list)
            self.append_info(f"离群设置Steering模式命令已发送: {self.usv_departed_namespace_list}") 
        except Exception as e:
            error_msg = f"发送离群Steering模式命令失败: {e}"
            self.append_info(error_msg)

    def set_departed_guided_command(self):
        """
        发送离群设置 guided 模式命令给所有离群 USV
        """
        try:
            self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
            self.ros_signal.guided_command.emit(self.usv_departed_namespace_list)
            self.append_info(f"离群设置guided模式命令已发送: {self.usv_departed_namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群guided模式命令失败: {e}")

    def set_departed_manual_command(self):
        """
        发送离群设置 manual 模式命令给所有离群 USV
        """
        try:
            self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
            self.ros_signal.manual_command.emit(self.usv_departed_namespace_list)
            self.append_info(f"离群设置manual模式命令已发送: {self.usv_departed_namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群manual模式命令失败: {e}")

    def set_departed_arco_command(self):
        """
        发送离群设置 ARCO 模式命令给所有离群 USV
        """
        try:
            self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
            self.ros_signal.arco_command.emit(self.usv_departed_namespace_list)
            self.append_info(f"离群设置ARCO模式命令已发送: {self.usv_departed_namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群ARCO模式命令失败: {e}")

# 主函数
def main(argv=None):
    app = QApplication(sys.argv)
    ros_signal = ROSSignal()
    main_window = MainWindow(ros_signal)

    # 全局异常处理：把未捕获的异常输出到 GUI 的 info 面板，便于调试运行时错误
    def _excepthook(type_, value, traceback_):
        try:
            msg = f"未捕获异常: {type_.__name__}: {value}"
            try:
                main_window.append_info(msg)
            except Exception:
                print(msg)
        finally:
            # 仍然调用默认行为
            sys.__excepthook__(type_, value, traceback_)

    sys.excepthook = _excepthook

    # 设置持久化日志（可滚动），便于在运行后分析问题
    try:
        import logging
        from logging.handlers import RotatingFileHandler
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

    rclpy.init(args=None)
    node=GroundStationNode(ros_signal)

    # 尝试从 usv_bringup/config/gs_params.yaml 加载参数并设置到 GroundStationNode
    try:
        # 构造默认参数文件路径（相对于本文件位置）
        default_params_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), '..', '..', 'usv_bringup', 'config', 'gs_params.yaml'
        ))
        if os.path.isfile(default_params_path):
            with open(default_params_path, 'r') as f:
                params = yaml.safe_load(f) or {}
            # 从 params dict 构造 rclpy Parameter 列表并设置到 node（更稳健）
            try:
                param_list = []
                # 白名单：只设置 GroundStationNode / 状态相关的参数
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
                        # 直接使用 Parameter(name, value) 由 rclpy 推断类型，兼容 int/float/str/bool
                        try:
                            param_list.append(Parameter(k, value=v))
                        except Exception:
                            # 尝试做基础类型转换后再设置
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

    ros_signal.manual_command.connect(node.set_manual_callback) # 手动信号连接到手动回调函数
  
    ros_signal.guided_command.connect(node.set_guided_callback)# 切换到guided模式信号连接到guided回调函数
    ros_signal.arm_command.connect(node.set_arming_callback) # 解锁信号连接到集群解锁回调函数
    ros_signal.disarm_command.connect(node.set_disarming_callback)# 加锁信号连接到集群加锁回调函数
    ros_signal.arco_command.connect(node.set_arco_callback) # 切换到arco模式信号连接到arco回调函数
    ros_signal.steering_command.connect(node.set_steering_callback) # 切换到steering模式信号连接到steering回调函数

    ros_signal.cluster_target_point_command.connect(node.set_cluster_target_point_callback)# 集群目标点信号连接到集群目标点回调函数
    ros_signal.departed_target_point_command.connect(node.set_departed_target_point_callback)# 离群目标点信号连接到离群目标点回调函数
   
    ros_signal.str_command.connect(node.str_command_callback) # 字符串命令信号连接到字符串命令回调函数

    # GUI 发来的手动标记 boot pose 信号，连接到节点回调（若支持）
    try:
        sig = getattr(ros_signal, 'set_boot_pose', None)
        cb = getattr(node, 'set_boot_pose_callback', None)
        if sig is not None and cb is not None:
            sig.connect(cb)
    except Exception:
        try:
            main_window.append_info('警告: 无法将 set_boot_pose 信号连接到 GroundStationNode')
        except Exception:
            pass

    # 连接节点->GUI 的反馈信号（node_info）到 GUI 的 append_info
    try:
        node_info_sig = getattr(ros_signal, 'node_info', None)
        if node_info_sig is not None:
            node_info_sig.connect(main_window.append_info)
    except Exception:
        try:
            main_window.append_info('警告: 无法连接节点反馈信号 node_info')
        except Exception:
            pass

    # 连接批量设置 boot pose 的信号
    try:
        sig_all = getattr(ros_signal, 'set_boot_pose_all', None)
        cb_all = getattr(node, 'set_boot_pose_all_callback', None)
        if sig_all is not None and cb_all is not None:
            sig_all.connect(cb_all)
    except Exception:
        try:
            main_window.append_info('警告: 无法将 set_boot_pose_all 信号连接到 GroundStationNode')
        except Exception:
            pass

    # 添加导航状态更新信号连接
    ros_signal.nav_status_update.connect(main_window.update_nav_status)
    # 添加导航反馈信号连接
    ros_signal.navigation_feedback.connect(main_window.handle_navigation_feedback)

    ros_thread = threading.Thread(target=lambda:rclpy.spin(node), daemon=True) # 创建一个线程来运行 ROS 事件循环
    ros_thread.start()
   
    main_window.show()
    try:
        exit_code = app.exec_()
    finally:
        # 应用退出，优雅停止 ROS 节点与后台线程
        try:
            # 使用 getattr 安全调用 shutdown（若不存在则调用空 lambda）
            getattr(node, 'shutdown', lambda: None)()
        except Exception as e:
            print(f"调用 node.shutdown() 时出错: {e}")
        try:
            # 销毁节点并关闭 rclpy
            node.destroy_node()
        except Exception as e:
            print(f"销毁节点时出错: {e}")
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"rclpy.shutdown() 时出错: {e}")
        # 等待 ros 线程退出（短超时），避免阻塞太久
        try:
            if ros_thread.is_alive():
                ros_thread.join(timeout=2.0)
        except Exception:
            pass
    sys.exit(exit_code)
# 如果这个脚本是主程序入口   
if __name__ == "__main__":
    main()
