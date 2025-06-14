from http.client import UNAVAILABLE_FOR_LEGAL_REASONS
import sys
import threading

from sympy import Point
import rclpy
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QTableWidgetItem, QAbstractItemView, QFileDialog, QMessageBox, QColorDialog
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from gs_gui.ros_signal import ROSSignal
from gs_gui.ros2_node_for_gui import GroundStationNode
from gs_gui.ui import Ui_MainWindow
from mavros_msgs.msg import State
from common_interfaces.msg import UsvStatus,UsvSetPoint

import xml.etree.ElementTree as ET
import requests
import json
from gs_gui.usv_plot_window import UsvPlotWindow




class MainWindow(QMainWindow):
    def __init__(self,ros_signal):
        super().__init__()
        self.ui=Ui_MainWindow()        
        self.ui.setupUi(self)
        self.setWindowTitle("Ground Station GUI")
        self.resize(1024, 512)
        self.setGeometry(100, 100, 1124, 612)

        self.ros_signal = ros_signal


        self.ros_signal.receive_state_list.connect(self.receive_state_callback)  # 连接信号

        self.ui.arming_pushButton.clicked.connect(self.set_cluster_arming_command)            # 集群解锁按钮
        self.ui.disarming_pushButton.clicked.connect(self.cluster_disarming_command)      # 集群加锁按钮
        self.ui.set_guided_pushButton.clicked.connect(self.set_cluster_guided_command)    # 集群切换到guided模式
        self.ui.set_manual_pushButton.clicked.connect(self.set_cluster_manual_command)    # 集群切换到manaul模式

        self.ui.departed_arming_pushButton.clicked.connect(self.departed_arming_command)  # 离群解锁按钮
        self.ui.departed_disarming_pushButton.clicked.connect(self.departed_disarming_command) #离群加锁按钮
        self.ui.set_departed_guided_pushButton.clicked.connect(self.set_departed_guided_command) #离群切换到guided模式
        self.ui.set_departed_manual_pushButton.clicked.connect(self.set_departed_manaul_command) #离群切换到manual模式
        self.ui.set_departed_ARCO_pushButton.clicked.connect(self.set_departed_arco_command) #离群切换到ARCO模式
        self.ui.set_departed_Steering_pushButton.clicked.connect(self.set_departed_steering_command) #离群切换到Steering模式

        self.ui.send_cluster_point_pushButton.clicked.connect(self.send_cluster_point_command) #集群坐标发送        
        self.ui.send_departed_point_pushButton.clicked.connect(self.send_departed_point_command)#离群坐标发送
        
        self.ui.add_cluster_pushButton.clicked.connect(self.add_cluster_command)#添加到集群list
        self.ui.quit_cluster_pushButton.clicked.connect(self.quit_cluster_command)#离开集群list，到离群list

        self.ui.sound_start_pushButton.clicked.connect(self.sound_start_command)
        self.ui.sound_stop_pushButton.clicked.connect(self.sound_stop_command)
        self.ui.neck_swinging_pushButton.clicked.connect(self.neck_swinging_command)
        self.ui.neck_stop_pushButton.clicked.connect(self.neck_swinging_command)

        self.ui.led1_pushButton.clicked.connect(self.led1_command)
        self.ui.led2_pushButton.clicked.connect(self.led2_command)
        self.ui.led3_pushButton.clicked.connect(self.led3_command)
        self.ui.light_stop_pushButton.clicked.connect(self.light_stop_command)


        # 点击open菜单
        self.ui.actionopen.triggered.connect(self.read_data_from_file)
        self.ui.actionrviz2.triggered.connect(self.rviz_process)

        # 在 __init__ 末尾添加菜单或按钮
        self.ui.action2D.triggered.connect(self.show_usv_plot_window)
        # 或者你可以加一个按钮 self.ui.show_usv_plot_pushButton.clicked.connect(self.show_usv_plot_window)


        # 在线设备列表
        self.usv_online_list=[]
        # 集群设备列表
        self.usv_cluster_list=[]

        # 集群设备namespace列表
        self.usv_cluster_namespace_list=[]

        # 离群设备列表
        self.usv_departed_list=[]

        # 离群设备namespace列表
        self.usv_departed_namespace_list=[]

        # 集群目标点集合
        self.cluster_position_list=[]

        # 离群目标点集合
        self.departed_position_list=[]
     
        # 集群表格模型
        self.cluster_table_model=QStandardItemModel(self)

        # 设置表格为单行选择模式
        self.ui.cluster_tableView.setSelectionMode(QAbstractItemView.SingleSelection)
        # 设置表格为行选择模式
        self.ui.cluster_tableView.setSelectionBehavior(QAbstractItemView.SelectRows)
        # 设置表格为只读
        self.ui.cluster_tableView.setEditTriggers(QAbstractItemView.NoEditTriggers)

        # 将模型设置到窗口表格
        self.ui.cluster_tableView.setModel(self.cluster_table_model)

        # 离群表格模型
        self.departed_table_model=QStandardItemModel(self)
        # 设置表格为只读
        self.ui.departed_tableView.setEditTriggers(QAbstractItemView.NoEditTriggers)
        # 设置表格为单行选择模式
        self.ui.departed_tableView.setSelectionMode(QAbstractItemView.SingleSelection)
        # 设置表格为行选择模式
        self.ui.departed_tableView.setSelectionBehavior(QAbstractItemView.SelectRows)

        # 将模型设置到窗口表格
        self.ui.departed_tableView.setModel(self.departed_table_model)

    # 集群解锁命令
    def set_cluster_arming_command(self):   
         
        for usv_id in self.usv_cluster_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_cluster_namespace_list.append(usv_namespace)
        self.ros_signal.arm_command.emit(self.usv_cluster_namespace_list)
        self.ui.info_textEdit.append(f"集群解锁命令已发送: {self.usv_cluster_namespace_list}")
        self.usv_cluster_namespace_list.clear()

    # 集群设置manaul模式命令
    def set_cluster_manual_command(self):   
        for usv_id in self.usv_cluster_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_cluster_namespace_list.append(usv_namespace)   
        self.ros_signal.manual_command.emit(self.usv_cluster_namespace_list)
        self.ui.info_textEdit.append(f"集群设置manual模式命令已发送: {self.usv_cluster_namespace_list}")
        self.usv_cluster_namespace_list.clear()

    # 集群加锁命令  
    def cluster_disarming_command(self):
        for usv_id in self.usv_cluster_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_cluster_namespace_list.append(usv_namespace)
        self.ros_signal.disarm_command.emit(self.usv_cluster_namespace_list)
        self.ui.info_textEdit.append(f"集群加锁命令已发送: {self.usv_cluster_namespace_list}")
        self.usv_cluster_namespace_list.clear()

    # 集群设置guided模式命令 
    def set_cluster_guided_command(self):
        for usv_id in self.usv_cluster_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_cluster_namespace_list.append(usv_namespace)
        # 发送 guided 模式命令
        self.ros_signal.guided_command.emit(self.usv_cluster_namespace_list)
        self.ui.info_textEdit.append(f"集群设置guided模式命令已发送: {self.usv_cluster_namespace_list}")
        self.usv_cluster_namespace_list.clear()

    # 离群解锁命令
    def departed_arming_command (self): 
        for usv_id in self.usv_departed_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_departed_namespace_list.append(usv_namespace)
        # 发送离群解锁命令       
        self.ros_signal.arm_command.emit(self.usv_departed_namespace_list)
        self.ui.info_textEdit.append(f"离群解锁命令已发送: {self.usv_departed_namespace_list}")
        self.usv_departed_namespace_list.clear()

    # 离群解锁命令
    def departed_disarming_command (self): 
        for usv_id in self.usv_departed_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_departed_namespace_list.append(usv_namespace)
        # 发送离群加锁命令       
        self.ros_signal.disarm_command.emit(self.usv_departed_namespace_list) 
        self.ui.info_textEdit.append(f"离群加锁命令已发送: {self.usv_departed_namespace_list}")
        self.usv_departed_namespace_list.clear()    
         
    # 离群设置manaul模式命令
    def set_departed_manaul_command (self): 
        for usv_id in self.usv_departed_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_departed_namespace_list.append(usv_namespace)
        # 发送离群设置manual模式命令       
        self.ros_signal.manaul_command.emit(self.usv_departed_namespace_list) 
        self.ui.info_textEdit.append(f"离群设置manual模式命令已发送: {self.usv_departed_namespace_list}") 
        self.usv_departed_namespace_list.clear()

    # 离群设置guided模式命令
    def set_departed_guided_command (self): 
        for usv_id in self.usv_departed_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_departed_namespace_list.append(usv_namespace)
        # 发送离群设置guided模式命令       
        self.ros_signal.guided_command.emit(self.usv_departed_namespace_list)
        self.ui.info_textEdit.append(f"离群设置guided模式命令已发送: {self.usv_departed_namespace_list}") 
        self.usv_departed_namespace_list.clear()  
    
    # 离群设置ARCO模式命令
    def set_departed_arco_command (self):
        for usv_id in self.usv_departed_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_departed_namespace_list.append(usv_namespace)
        # 发送离群设置ARCO模式命令       
        self.ros_signal.arco_command.emit(f"set_departed_arco_command:{self.usv_departed_namespace_list}")
        self.ui.info_textEdit.append(f"离群设置ARCO模式命令已发送: {self.usv_departed_namespace_list}") 
        self.usv_departed_namespace_list.clear()

    # 离群设置Steering模式命令
    def set_departed_steering_command (self):
        for usv_id in self.usv_departed_list:
            usv_namespace =usv_id.get('namespace', None)
            self.usv_departed_namespace_list.append(usv_namespace)
        # 发送离群设置Steering模式命令       
        self.ros_signal.steering_command.emit(f"set_departed_steering_command:{self.usv_departed_namespace_list}")
        self.ui.info_textEdit.append(f"离群设置Steering模式命令已发送: {self.usv_departed_namespace_list}") 
        self.usv_departed_namespace_list.clear()

    # 从数据文件中读取数据，数据格式xml
    def read_data_from_file(self):
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
                                yaw_elem = usv.find("yaw/value")
                                velocity_elem = usv.find("velocity/value")
                                usv_data = {
                                    "usv_id": usv_id_elem.text if usv_id_elem is not None else "",
                                    "position": {
                                        "x": float(pos_x_elem.text) if pos_x_elem is not None and pos_x_elem.text is not None else 0.0,
                                        "y": float(pos_y_elem.text) if pos_y_elem is not None and pos_y_elem.text is not None else 0.0
                                    },
                                    "yaw": float(yaw_elem.text) if yaw_elem is not None and yaw_elem.text is not None else 0.0,
                                    "velocity": float(velocity_elem.text) if velocity_elem is not None and velocity_elem.text is not None else 0.0,
                                    "step": int(step_number) if step_number is not None else 0
                                }
                                usv_list.append(usv_data)

                            self.cluster_position_list=usv_list
                            self.ui.info_textEdit.append(f"读取数据成功，共 {len(usv_list)} 个 USV 数据")
                            self.ui.info_textEdit.append(f"数据： {self.cluster_position_list} ")
                    else:
                        print("step 节点未找到")

                        self.cluster_position_list=[]
                    

                
                except ET.ParseError as e:
                    print(f"XML解析错误: {e}")
                    self.cluster_position_list=[]

                except FileNotFoundError:
                    print(f"文件 {xml_file} 未找到")
                    self.cluster_position_list=[]

                except Exception as e:
                    print(f"其他错误: {e}")
                    self.cluster_position_list=[]
    
    # 启动 RViz2
    def rviz_process(self):
        # 启动 RViz2
        self._rviz_process = QProcess(self)
        self._rviz_process.start("rviz2")
        self.ui.info_textEdit.append(f"启动RViz2")

    # 发送集群目标点命令
    def send_cluster_point_command(self):  
        self.ui.info_textEdit.append(f"发送集群目标点命令")
        # 检查是否有集群列表
        if not self.cluster_position_list:
            self.ui.warning_textEdit.append("集群列表为空")
            return
        # 创建副本以避免修改原始列表时的遍历问题
        filtered_list = self.cluster_position_list.copy()
        # 将 usv_departed_list 转换为集合以提高查找效率
        departed_ids = set(self.usv_departed_list)
        # 移除匹配 usv_id 的 USV 数据
        filtered_list = [usv for usv in filtered_list if usv.get('usv_id', '') not in departed_ids]
        if not filtered_list:
            self.ui.warning_textEdit.append("集群列表为空（所有 USV 均在离群列表中）")          
            return 
        # 弹窗确认
        reply = QMessageBox.question(
            self,
            f"确认发送",
            f"即将发送 {len(filtered_list)} 个 USV 的目标点数据。\n是否继续?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            try:
                # 更新 cluster_position_list
                self.cluster_position_list = filtered_list
                # 发送 ROS 信号
                self.ros_signal.cluster_target_point_command.emit(self.cluster_position_list)
                QMessageBox.information(self, "成功", "集群目标点数据已发送！")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"发送失败: {e}")
        else:
            QMessageBox.information(self, "取消", f"发送已取消")
   
    # 离群目标点命令
    def send_departed_point_command(self):
        x = self.ui.set_departed_x_doubleSpinBox.value()
        y = self.ui.set_departed_y_doubleSpinBox.value() 
        departed_target_list = []
        for usv_id in self.usv_departed_list:
            departed_target = UsvSetPoint()
            departed_target.usv_id = usv_id.get('namespace', None)
            departed_target.position.x = x
            departed_target.position.y = y
            departed_target_list.append(departed_target)
        self.ros_signal.departed_target_point_command.emit(departed_target_list)
        self.ui.info_textEdit.append(f"发送离群目标点: {departed_target_list}")

    # 添加到集群列表
    def add_cluster_command(self):
        self.ui.info_textEdit.append(f"添加到集群列表")
        selected_indexes = self.ui.departed_tableView.selectedIndexes()
        if not selected_indexes:
            self.ui.info_textEdit.append("请先选择一行")
            return

        selected_row = selected_indexes[0].row()
        model = self.ui.departed_tableView.model()
        if model is None:
            self.ui.info_textEdit.append("错误：离群表格模型未初始化")
            return

        # 获取整行状态数据
        state = {}
        headers = ["namespace", "mode", "connected", "armed", "battery_voltage",
                "battery_prcentage", "power_supply_status", "position",
                "velocity", "yaw", "is_reached_target"]
        for col, key in enumerate(headers):
            index = model.index(selected_row, col)
            state[key] = model.data(index) or "Unknown"

        usv_id = state.get("namespace")
        if not usv_id:
            self.ui.info_textEdit.append("错误：无法获取 USV ID")
            return

        try:
            self.usv_cluster_list.append(state)
            # 从 usv_departed_list 移除匹配的项
            for item in self.usv_departed_list:
                if item.get('namespace') == usv_id:
                    self.usv_departed_list.remove(item)               
            self.update_cluster_table(self.usv_cluster_list)
            self.update_departed_table(self.usv_departed_list)
            self.ui.info_textEdit.append(f"添加设备 {usv_id} 到集群列表")
        except Exception as e:
            self.ui.info_textEdit.append(f"错误：添加设备失败 - {str(e)}")
   
    # 离开集群列表
    def quit_cluster_command(self):
        self.ui.info_textEdit.append(f"离开集群列表")
        selected_indexes = self.ui.cluster_tableView.selectedIndexes()
        self.ui.info_textEdit.append(f'选中的标签：{selected_indexes}')

        if not selected_indexes:
            self.ui.info_textEdit.append("请先选择一行")
            return

        selected_row = selected_indexes[0].row()
        model = self.ui.cluster_tableView.model()
        if model is None:
            self.ui.info_textEdit.append("错误：集群表格模型未初始化")
            return

        # 获取整行状态数据
        state = {}
        headers = ["namespace", "mode", "connected", "armed", "battery_voltage", 
                "battery_prcentage", "power_supply_status", "position", 
                "velocity", "yaw", "is_reached_target"]
        for col, key in enumerate(headers):
            index = model.index(selected_row, col)
            state[key] = model.data(index) or "Unknown"

        usv_id = state.get("namespace")
        if not usv_id:
            self.ui.info_textEdit.append("错误：无法获取 USV ID")
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
            self.ui.info_textEdit.append(f"添加设备 {usv_id} 到离群列表")
        except Exception as e:
            self.ui.info_textEdit.append(f"错误：移除设备失败 - {str(e)}")
    
    # 接收所有在线的usv状态
    def receive_state_callback(self, msg):
        if not isinstance(msg, list):
            self.ui.warning_textEdit.append("接收到的数据类型错误，期望为列表")
            return
        
        # 验证并过滤 msg 数据
        self.usv_online_list = [ns for ns in msg if isinstance(ns, dict) and ns.get('namespace')]
        # print(f"Online List: {self.usv_online_list}")
        
        # 更新 usv_departed_list 状态
        for i, departed in enumerate(self.usv_departed_list):
            departed_id = departed.get('namespace') if isinstance(departed, dict) else departed
            state = next((ns for ns in self.usv_online_list if ns.get('namespace') == departed_id), None)
            if state:
                self.usv_departed_list[i] = state
            else:
                # 如果离群 USV 已下线，保留现有数据
                if not isinstance(departed, dict):
                    self.usv_departed_list[i] = {'namespace': departed_id}  # 转换为字典
                self.ui.info_textEdit.append(f"警告：离群设备 {departed_id} 未在在线列表中")
        
        # 创建 temp_list，移除离群 USV
        temp_list = self.usv_online_list.copy()
        for ns in self.usv_online_list:
            usv_id_set = ns.get('namespace')
            for departed in self.usv_departed_list:
                departed_id = departed.get('namespace') if isinstance(departed, dict) else departed
                if usv_id_set == departed_id:
                    if ns in temp_list:
                        temp_list.remove(ns)
                        break
        
        # 保留现有 usv_cluster_list 的手动添加 USV
        current_cluster = self.usv_cluster_list.copy()
        current_cluster_ids = {item.get('namespace') for item in current_cluster if isinstance(item, dict) and item.get('namespace')}
        # print(f"Current Cluster IDs: {current_cluster_ids}")
        
        # 更新 usv_cluster_list
        self.usv_cluster_list = []
        for item in current_cluster:
            if isinstance(item, dict) and item.get('namespace'):
                state = next((ns for ns in self.usv_online_list if ns.get('namespace') == item.get('namespace')), item)
                self.usv_cluster_list.append(state)
        for ns in temp_list:
            if ns.get('namespace') not in current_cluster_ids:
                self.usv_cluster_list.append(ns)
        
        # 更新表格
        self.update_cluster_table(self.usv_cluster_list)
        self.update_departed_table(self.usv_departed_list)
        self.update_selected_table_row()       
   
    # 更新集群表格
    def update_cluster_table(self, lists):

        if lists is None:
            lists = []

        # self.ui.info_textEdit.append(f"更新集群表格: {lists}")
        self.cluster_table_model.setRowCount(len(lists))
        self.cluster_table_model.setColumnCount(11)
        self.cluster_table_model.setHorizontalHeaderLabels(["编号", "当前模式",
                                                            "连接状态", "武装状态", '电压V', '电量%', '电池状态', '坐标', '速度', '偏角','是否到达目标'])
        for i, state in enumerate(lists):
            # 检查 state 是否为字典
            if isinstance(state, dict):
                self.cluster_table_model.setItem(i, 0, QStandardItem(str(state.get('namespace', 'Unknown'))))
                self.cluster_table_model.setItem(i, 1, QStandardItem(str(state.get('mode', 'Unknown'))))
                self.cluster_table_model.setItem(i, 2, QStandardItem(str(state.get('connected', 'Unknown'))))
                self.cluster_table_model.setItem(i, 3, QStandardItem(str(state.get('armed', 'Unknown'))))
                self.cluster_table_model.setItem(i, 4, QStandardItem(str(state.get('battery_voltage', 'Unknown'))))
                self.cluster_table_model.setItem(i, 5, QStandardItem(str(state.get('battery_prcentage', 'Unknown'))))
                self.cluster_table_model.setItem(i, 6, QStandardItem(str(state.get('power_supply_status', 'Unknown'))))
                self.cluster_table_model.setItem(i, 7, QStandardItem(str(state.get('position', 'Unknown'))))
                self.cluster_table_model.setItem(i, 8, QStandardItem(str(state.get('velocity', 'Unknown'))))
                self.cluster_table_model.setItem(i, 9, QStandardItem(str(state.get('yaw', 'Unknown'))))
                self.cluster_table_model.setItem(i, 10, QStandardItem(str(state.get('is_reached_target', 'Unknown'))))
    
    # 更新离群表格
    def update_departed_table(self, lists):
        if lists is None:
            lists = []
        self.departed_table_model.setRowCount(len(lists))
        self.departed_table_model.setColumnCount(11)
        self.departed_table_model.setHorizontalHeaderLabels(["编号",  "当前模式",
                                                            "连接状态", "武装状态", '电压V', '电量%', '电池状态', '坐标', '速度', '偏角','是否到达目标'])
        for i, state in enumerate(lists):
            if isinstance(state, dict):
                self.departed_table_model.setItem(i, 0, QStandardItem(str(state.get('namespace', 'Unknown'))))
                self.departed_table_model.setItem(i, 1, QStandardItem(str(state.get('mode', 'Unknown'))))
                self.departed_table_model.setItem(i, 2, QStandardItem(str(state.get('connected', 'Unknown'))))
                self.departed_table_model.setItem(i, 3, QStandardItem(str(state.get('armed', 'Unknown'))))
                self.departed_table_model.setItem(i, 4, QStandardItem(str(state.get('battery_voltage', 'Unknown'))))
                self.departed_table_model.setItem(i, 5, QStandardItem(str(state.get('battery_prcentage', 'Unknown'))))
                self.departed_table_model.setItem(i, 6, QStandardItem(str(state.get('power_supply_status', 'Unknown'))))
                self.departed_table_model.setItem(i, 7, QStandardItem(str(state.get('position', 'Unknown'))))
                self.departed_table_model.setItem(i, 8, QStandardItem(str(state.get('velocity', 'Unknown'))))
                self.departed_table_model.setItem(i, 9, QStandardItem(str(state.get('yaw', 'Unknown'))))
                self.departed_table_model.setItem(i, 10, QStandardItem(str(state.get('is_reached_target', 'Unknown'))))
   
    # 更新选中行数据
    def update_selected_table_row(self):
        try:
            # 获取选中的行
            selected_indexes = self.ui.cluster_tableView.selectedIndexes()
            if not selected_indexes:
                self.ui.info_textEdit.append("没有选中任何行")
                return
            selected_row = selected_indexes[0].row()
            model = self.ui.cluster_tableView.model()
            if model is None:
                self.ui.info_textEdit.append("集群列表为空")
                return
            # 获取整行状态数据
            headers = ["namespace", "mode", "connected", "armed", "battery_voltage", 
                       "battery_prcentage", "power_supply_status", "position", 
                       "velocity", "yaw", "is_reached_target"]
            state = {}
            for col, key in enumerate(headers):
                index = model.index(selected_row, col)
                state[key] = model.data(index) or "Unknown"
            self.ui.usv_id_label.setText(f"当前选中 USV ID: {state.get('namespace', 'Unknown')}")
            position_str = state.get('position', 'Unknown')
            yaw_str = state.get('yaw', 'Unknown')
            # 解析 position 字符串，格式如 x=...,y=...,z=...
            try:
                position_ = [s.strip() for s in position_str.split(',')]
                if len(position_) == 3 and all('=' in p for p in position_):
                    x = float(position_[0].split('=')[1])
                    y = float(position_[1].split('=')[1])
                    z = float(position_[2].split('=')[1])
                else:
                    raise ValueError('位置字段格式错误')
            except Exception:
                x = y = z = 'Unknown'
            # 解析 yaw 字符串，格式如 yaw=...
            try:
                if '=' in yaw_str:
                    yaw_val = float(yaw_str.split('=')[1])
                else:
                    raise ValueError('yaw字段格式错误')
            except Exception:
                yaw_val = 'Unknown'
            self.ui.usv_x_label.setText(f"{x}")
            self.ui.usv_y_label.setText(f"{y}")
            self.ui.usv_z_label.setText(f"{z}")
            self.ui.usv_yaw_label.setText(f"{yaw_val}")
        except Exception as e:
            self.ui.info_textEdit.append(f"错误：获取选中行数据失败 - {str(e)}")
            self.ui.usv_id_label.setText("当前选中 USV ID: Unknown")
            self.ui.usv_x_label.setText("当前选中 USV X: Unknown")
            self.ui.usv_y_label.setText("当前选中 USV Y: Unknown")
            self.ui.usv_z_label.setText("当前选中 USV Z: Unknown")
            self.ui.usv_yaw_label.setText("当前选中 USV Yaw: Unknown")
   
    # 声音开始命令
    def sound_start_command(self):
        self.ros_signal.str_command.emit('sound_start')
        self.ui.info_textEdit.append(f"发送命令: sound_start")
    
    # 声音停止命令
    def sound_stop_command(self):
        self.ros_signal.str_command.emit('sound_stop')
        self.ui.info_textEdit.append(f"发送命令:sound_stop")
   
    # 颈部摆动命令
    def neck_swinging_command(self):
        self.ros_signal.str_command.emit('neck_swinging')
        self.ui.info_textEdit.append(f"发送命令: neck_swinging")

   
    # 颈部停止命令
    def neck_stop_command(self):
        self.ros_signal.str_command.emit('neck_stop')
        self.ui.info_textEdit.append(f"发送命令: neck_stop")
   
    # led1命令
    def led1_command(self):
        self.ros_signal.str_command.emit('color_switching')
        self.ui.info_textEdit.append(f"发送命令: color_switching")
    
    # led2命令
    def led2_command(self):
        self.ros_signal.str_command.emit('random_color_change')
        self.ui.info_textEdit.append(f"发送命令:random_color_change")
    
    # led3命令    
    def led3_command(self):
        color=QColorDialog.getColor() # type: ignore
        if not color.isValid():
            self.ui.info_textEdit.append("颜色选择无效")
            return
        # 获取颜色的 RGB 值
        r = color.red()
        g = color.green()
        b = color.blue()
        # 将 RGB 值转换为字符串格式
        color_str = f"color_select|{r},{g},{b}"
        # 发送颜色选择命令
        self.ros_signal.str_command.emit(color_str)
        # 在信息文本框中显示发送的命令
        self.ui.info_textEdit.append(f"发送led3命令: {color_str}")
   
    # 停止灯光命令
    def light_stop_command(self):
        self.ros_signal.str_command.emit('led_off')
        self.ui.info_textEdit.append(f"发送命令: led_off")
    
    # 显示 USV 绘图窗口
    def show_usv_plot_window(self):
        # 传递一个获取usv列表的函数，保证窗口里能实时获取最新数据
        self.usv_plot_window = UsvPlotWindow(lambda: self.usv_online_list, self)
        self.usv_plot_window.show()

# 主函数
def main(argv=None):
    app = QApplication(sys.argv)
    ros_signal = ROSSignal()
    main_window = MainWindow(ros_signal)

    rclpy.init(args=None)
    node=GroundStationNode(ros_signal)

    ros_signal.manual_command.connect(node.set_manual_callback) # 手动信号连接到手动回调函数
  
    ros_signal.guided_command.connect(node.set_guided_callback)# 切换到guided模式信号连接到guided回调函数
    ros_signal.arm_command.connect(node.set_arming_callback) # 解锁信号连接到集群解锁回调函数
    ros_signal.disarm_command.connect(node.set_disarming_callback)# 加锁信号连接到集群加锁回调函数
    ros_signal.arco_command.connect(node.set_arco_callback) # 切换到arco模式信号连接到arco回调函数
    ros_signal.steering_command.connect(node.set_steering_callback) # 切换到steering模式信号连接到steering回调函数

    ros_signal.cluster_target_point_command.connect(node.set_cluster_target_point_callback)# 集群目标点信号连接到集群目标点回调函数
    ros_signal.departed_target_point_command.connect(node.set_departed_target_point_callback)# 离群目标点信号连接到离群目标点回调函数
   
    ros_signal.str_command.connect(node.str_command_callback) # 字符串命令信号连接到字符串命令回调函数

    ros_thread = threading.Thread(target=lambda:rclpy.spin(node), daemon=True) # 创建一个线程来运行 ROS 事件循环
    ros_thread.start()
   
    main_window.show()
    sys.exit(app.exec_())
# 如果这个脚本是主程序入口   
if __name__ == "__main__":
    main()
