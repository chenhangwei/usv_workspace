from http.client import UNAVAILABLE_FOR_LEGAL_REASONS
import sys
import threading

from sympy import Point
import rclpy
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QTableWidgetItem, QAbstractItemView, QFileDialog, QMessageBox
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from gs_gui.ros_signal import ROSSignal
from gs_gui.ros2_node_for_gui import GroundStationNode
from gs_gui.ui import Ui_MainWindow
from mavros_msgs.msg import State
from common_interfaces.msg import UsvStatus,UsvSetPoint

import xml.etree.ElementTree as ET
import requests
import json




class Mainwindow(QMainWindow):
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

        self.ui.send_cluster_point_pushButton.clicked.connect(self.send_cluster_point_command) #集群坐标发送
        self.ui.usv_cluster_velocity_horizontalSlider.valueChanged.connect(self.cluster_velocity_value_changed)#集群速度发送

        self.ui.send_departed_point_pushButton.clicked.connect(self.send_departed_point_command)#离群坐标发送
        self.ui.usv_departed_velocity_horizontalSlider.valueChanged.connect(self.departed_velocity_value_changed)#离群速度发送

        self.ui.add_cluster_pushButton.clicked.connect(self.add_cluster_command)#添加到集群list
        self.ui.quit_cluster_pushButton.clicked.connect(self.quit_cluster_command)#离开集群list，到离群list

        self.ui.gaga1_pushButton.clicked.connect(self.gaga1_command)
        self.ui.gaga2_pushButton.clicked.connect(self.gaga2_command)
        self.ui.gaga3_pushButton.clicked.connect(self.gaga3_command)
        self.ui.gaga4_pushButton.clicked.connect(self.gaga4_command)

        self.ui.led1_pushButton.clicked.connect(self.led1_command)
        self.ui.led2_pushButton.clicked.connect(self.led2_command)
        self.ui.led3_pushButton.clicked.connect(self.led3_command)
        self.ui.led4_pushButton.clicked.connect(self.led4_command)


        # 点击open菜单
        self.ui.actionopen.triggered.connect(self.read_data_from_file)
        self.ui.actionrviz2.triggered.connect(self.rviz_process)


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

        # 集群速度
        self.cluster_velocity=0.0

        # 离群速度
        self.departed_velocity=0.0
      
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
        self.rviz_process = QProcess(self)
        self.rviz_process.start("rviz2")
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

    
    # 集群速度命令
    def cluster_velocity_value_changed(self):

        cluster_velocity_list = []
        self.cluster_velocity=float(self.ui.usv_cluster_velocity_horizontalSlider.value())
        # 发送集群速度命令
        for usv_id in self.usv_cluster_list:       
            cluster_velocity_ = UsvSetPoint()
            cluster_velocity_.usv_id = usv_id.get('namespace', None)
            cluster_velocity_.velocity = self.cluster_velocity
            cluster_velocity_list.append(cluster_velocity_)
        self.ros_signal.cluster_target_velocity_command.emit( cluster_velocity_list)
        self.ui.info_textEdit.append(f"集群目标速度: {cluster_velocity_list}")
        cluster_velocity_list.clear()

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

    # 离群速度命令 
    def departed_velocity_value_changed (self):
        departed_velocity_list = []
        # 获取离群速度
        self.departed_velocity=float(self.ui.usv_departed_velocity_horizontalSlider.value())
        # 发送离群速度命令
        for usv_id in self.usv_departed_list:
            departed_velocity_ = UsvSetPoint()
            departed_velocity_.usv_id = usv_id.get('namespace', None)
            departed_velocity_.velocity = self.departed_velocity
            departed_velocity_list.append(departed_velocity_)
        self.ros_signal.departed_target_velocity_command.emit(departed_velocity_list)
        self.ui.info_textEdit.append(f"离群目标速度: {departed_velocity_list}")


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
                "velocity", "yaw", "is_running"]
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
                    break
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
                "velocity", "yaw", "is_running"]
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
        print(f"Current Cluster IDs: {current_cluster_ids}")
        
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
        
   
    def update_cluster_table(self, lists):

        if lists is None:
            lists = []

        # self.ui.info_textEdit.append(f"更新集群表格: {lists}")
        self.cluster_table_model.setRowCount(len(lists))
        self.cluster_table_model.setColumnCount(11)
        self.cluster_table_model.setHorizontalHeaderLabels(["编号", "当前模式",
                                                            "连接状态", "武装状态", '电压V', '电量%', '电池状态', '坐标', '速度', '偏角','是否运行中'])
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
                self.cluster_table_model.setItem(i, 10, QStandardItem(str(state.get('is_running', 'Unknown'))))

    # 更新离群表格
    def update_departed_table(self, lists):
        if lists is None:
            lists = []
        self.departed_table_model.setRowCount(len(lists))
        self.departed_table_model.setColumnCount(11)
        self.departed_table_model.setHorizontalHeaderLabels(["编号",  "当前模式",
                                                            "连接状态", "武装状态", '电压V', '电量%', '电池状态', '坐标', '速度', '偏角','是否运行中'])
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
                self.departed_table_model.setItem(i, 10, QStandardItem(str(state.get('is_running', 'Unknown'))))

    def gaga1_command(self):
        self.ros_signal.str_command.emit('gaga1')
        self.ui.info_textEdit.append(f"发送gaga1命令: gaga1")

    def gaga2_command(self):
        self.ros_signal.str_command.emit('gaga2')
        self.ui.info_textEdit.append(f"发送gaga2命令: gaga2")

    def gaga3_command(self):
        self.ros_signal.str_command.emit('gaga3')
        self.ui.info_textEdit.append(f"发送gaga3命令: gaga3")
    def gaga4_command(self):
        self.ros_signal.str_command.emit('gaga4')
        self.ui.info_textEdit.append(f"发送gaga4命令: gaga4")

    def led1_command(self):
        self.ros_signal.str_command.emit('led1')
        self.ui.info_textEdit.append(f"发送led1命令: led1")
    
    def led2_command(self):
        self.ros_signal.str_command.emit('led2')
        self.ui.info_textEdit.append(f"发送led2命令: led2")
        
    def led3_command(self):
        self.ros_signal.str_command.emit('led3')
        self.ui.info_textEdit.append(f"发送led3命令: led3")

    def led4_command(self):
        self.ros_signal.str_command.emit('led4')
        self.ui.info_textEdit.append(f"发送led4命令: led4")


def main(argv=None):
    app = QApplication(sys.argv)
    ros_signal = ROSSignal()
    main_window = Mainwindow(ros_signal)

    rclpy.init(args=None)
    node=GroundStationNode(ros_signal)


    ros_signal.manual_command.connect(node.set_manual_callback)#主线程连接
    ros_signal.guided_command.connect(node.set_guided_callback)
    ros_signal.arm_command.connect(node.set_arming_callback)#主线程连接
    ros_signal.disarm_command.connect(node.set_disarming_callback)

    ros_signal.cluster_target_point_command.connect(node.set_cluster_target_point_callback)
    ros_signal.cluster_target_velocity_command.connect(node.set_cluster_target_velocity_callback)
    ros_signal.departed_target_point_command.connect(node.set_departed_target_point_callback)
    ros_signal.departed_target_velocity_command.connect(node.set_departed_target_velocity_callback)

   
    ros_signal.str_command.connect(node.str_command_callback)




    ros_thread = threading.Thread(target=lambda:rclpy.spin(node), daemon=True)
    ros_thread.start()
   
    
    main_window.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
