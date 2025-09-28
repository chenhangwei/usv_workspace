from http.client import UNAVAILABLE_FOR_LEGAL_REASONS
import sys
import threading
import weakref  # 添加weakref模块用于弱引用

import rclpy
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication, QMainWindow,  QAbstractItemView, QFileDialog, QMessageBox, QColorDialog
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from gs_gui.ros_signal import ROSSignal
from gs_gui.ros2_node_for_gui import GroundStationNode
from gs_gui.ui import Ui_MainWindow
import xml.etree.ElementTree as ET
from gs_gui.usv_plot_window import UsvPlotWindow
import re


class MainWindow(QMainWindow):
    # 表格列标题常量
    TABLE_HEADERS = ["编号", "当前模式", "连接状态", "武装状态", '电压V', '电量%', 
                    '电池状态', '坐标', '速度', '偏角', '导航状态', '温度']
    
    # 命名空间列表常量
    CLUSTER_NAMESPACE_LIST = "cluster"
    DEPARTED_NAMESPACE_LIST = "departed"
    
    # 添加导航状态常量
    NAV_STATUS_UNKNOWN = "未知"
    NAV_STATUS_IDLE = "空闲"
    NAV_STATUS_ACTIVE = "执行中"
    NAV_STATUS_SUCCEEDED = "成功"
    NAV_STATUS_FAILED = "失败"
    
    def __init__(self,ros_signal):
        super().__init__()
        self.ui=Ui_MainWindow()        
        self.ui.setupUi(self)
        self.setWindowTitle("Ground Station GUI")
        self.resize(1024, 512)
        self.setGeometry(100, 100, 1124, 612)

        self.ros_signal = ros_signal
        
        # 存储USV导航状态
        self.usv_nav_status = {}
        # 添加集群任务进度信息
        self.cluster_progress_info = {}

        # 添加集群任务控制状态
        self.cluster_task_running = False  # 集群任务是否正在运行
        self.cluster_task_paused = False   # 集群任务是否已暂停
        
        # 初始化集群任务按钮状态
        self.update_cluster_task_button_state()

        self.ros_signal.receive_state_list.connect(self.receive_state_callback)  # 连接信号
        self.ros_signal.cluster_progress_update.connect(self.cluster_progress_callback)  # 连接集群进度信号

        self.ui.arming_pushButton.clicked.connect(self.set_cluster_arming_command)            # 集群解锁按钮
        self.ui.disarming_pushButton.clicked.connect(self.cluster_disarming_command)      # 集群加锁按钮
        self.ui.set_guided_pushButton.clicked.connect(self.set_cluster_guided_command)    # 集群切换到guided模式
        self.ui.set_manual_pushButton.clicked.connect(self.set_cluster_manual_command)    # 集群切换到manaul模式

        self.ui.departed_arming_pushButton.clicked.connect(self.departed_arming_command)  # 离群解锁按钮
        self.ui.departed_disarming_pushButton.clicked.connect(self.departed_disarming_command) #离群加锁按钮
        self.ui.set_departed_guided_pushButton.clicked.connect(self.set_departed_guided_command) #离群切换到guided模式
        self.ui.set_departed_manual_pushButton.clicked.connect(self.set_departed_manual_command) #离群切换到manual模式
        self.ui.set_departed_ARCO_pushButton.clicked.connect(self.set_departed_arco_command) #离群切换到ARCO模式
        self.ui.set_departed_Steering_pushButton.clicked.connect(self.set_departed_steering_command) #离群切换到Steering模式

        self.ui.send_cluster_point_pushButton.clicked.connect(self.toggle_cluster_task) # 集群任务运行/暂停切换
        self.ui.stop_cluster_task_pushButton.clicked.connect(self.stop_cluster_task) # 集群任务停止
        self.ui.send_departed_point_pushButton.clicked.connect(self.send_departed_point_command)#离群坐标发送
        
        self.ui.add_cluster_pushButton.clicked.connect(self.add_cluster_command)#添加到集群list
        self.ui.quit_cluster_pushButton.clicked.connect(self.quit_cluster_command)#离开集群list，到离群list

        self.ui.sound_start_pushButton.clicked.connect(self.sound_start_command)
        self.ui.sound_stop_pushButton.clicked.connect(self.sound_stop_command)
        self.ui.neck_swinging_pushButton.clicked.connect(self.neck_swinging_command)
        self.ui.neck_stop_pushButton.clicked.connect(self.neck_stop_command)

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

        # 添加USV绘图窗口的弱引用
        self.usv_plot_window_ref = None

    def _extract_namespaces(self, usv_list):
        """
        从USV列表中提取命名空间列表
        
        Args:
            usv_list (list): 包含USV信息的字典列表
            
        Returns:
            list: USV命名空间列表
        """
        namespaces = []
        for usv_info in usv_list:
            if isinstance(usv_info, dict):
                namespace = usv_info.get('namespace')
                if namespace:
                    namespaces.append(namespace)
        return namespaces

    # 集群解锁命令
    def set_cluster_arming_command(self):   
        """
        发送集群解锁命令给所有集群USV
        """
        self.usv_cluster_namespace_list = self._extract_namespaces(self.usv_cluster_list)
        self.ros_signal.arm_command.emit(self.usv_cluster_namespace_list)
        self.ui.info_textEdit.append(f"集群解锁命令已发送: {self.usv_cluster_namespace_list}")
        self.usv_cluster_namespace_list.clear()

    # 集群设置manual模式命令
    def set_cluster_manual_command(self):   
        """
        发送集群设置manual模式命令给所有集群USV
        """
        self.usv_cluster_namespace_list = self._extract_namespaces(self.usv_cluster_list)
        self.ros_signal.manual_command.emit(self.usv_cluster_namespace_list)
        self.ui.info_textEdit.append(f"集群设置manual模式命令已发送: {self.usv_cluster_namespace_list}")
        self.usv_cluster_namespace_list.clear()

    # 集群加锁命令  
    def cluster_disarming_command(self):
        """
        发送集群加锁命令给所有集群USV
        """
        self.usv_cluster_namespace_list = self._extract_namespaces(self.usv_cluster_list)
        self.ros_signal.disarm_command.emit(self.usv_cluster_namespace_list)
        self.ui.info_textEdit.append(f"集群加锁命令已发送: {self.usv_cluster_namespace_list}")
        self.usv_cluster_namespace_list.clear()

    # 集群设置guided模式命令 
    def set_cluster_guided_command(self):
        """
        发送集群设置guided模式命令给所有集群USV
        """
        self.usv_cluster_namespace_list = self._extract_namespaces(self.usv_cluster_list)
        # 发送 guided 模式命令
        self.ros_signal.guided_command.emit(self.usv_cluster_namespace_list)
        self.ui.info_textEdit.append(f"集群设置guided模式命令已发送: {self.usv_cluster_namespace_list}")
        self.usv_cluster_namespace_list.clear()

    # 离群解锁命令
    def departed_arming_command (self):
        """
        发送离群解锁命令给所有离群USV
        """ 
        self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
        # 发送离群解锁命令       
        self.ros_signal.arm_command.emit(self.usv_departed_namespace_list)
        self.ui.info_textEdit.append(f"离群解锁命令已发送: {self.usv_departed_namespace_list}")
        self.usv_departed_namespace_list.clear()

    # 离群解锁命令
    def departed_disarming_command (self):
        """
        发送离群加锁命令给所有离群USV
        """ 
        self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
        # 发送离群加锁命令       
        self.ros_signal.disarm_command.emit(self.usv_departed_namespace_list) 
        self.ui.info_textEdit.append(f"离群加锁命令已发送: {self.usv_departed_namespace_list}")
        self.usv_departed_namespace_list.clear()    
         
    # 离群设置manual模式命令
    def set_departed_manual_command (self):
        """
        发送离群设置manual模式命令给所有离群USV
        """ 
        self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
        # 发送离群设置manual模式命令       
        self.ros_signal.manual_command.emit(self.usv_departed_namespace_list) 
        self.ui.info_textEdit.append(f"离群设置manual模式命令已发送: {self.usv_departed_namespace_list}") 
        self.usv_departed_namespace_list.clear()

    # 离群设置guided模式命令
    def set_departed_guided_command (self):
        """
        发送离群设置guided模式命令给所有离群USV
        """ 
        self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
        # 发送离群设置guided模式命令       
        self.ros_signal.guided_command.emit(self.usv_departed_namespace_list)
        self.ui.info_textEdit.append(f"离群设置guided模式命令已发送: {self.usv_departed_namespace_list}") 
        self.usv_departed_namespace_list.clear()  
    
    # 离群设置ARCO模式命令
    def set_departed_arco_command (self):
        """
        发送离群设置ARCO模式命令给所有离群USV
        """
        self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
        # 发送离群设置ARCO模式命令       
        self.ros_signal.arco_command.emit(f"set_departed_arco_command:{self.usv_departed_namespace_list}")
        self.ui.info_textEdit.append(f"离群设置ARCO模式命令已发送: {self.usv_departed_namespace_list}") 
        self.usv_departed_namespace_list.clear()

    # 离群设置Steering模式命令
    def set_departed_steering_command (self):
        """
        发送离群设置Steering模式命令给所有离群USV
        """
        self.usv_departed_namespace_list = self._extract_namespaces(self.usv_departed_list)
        # 发送离群设置Steering模式命令       
        self.ros_signal.steering_command.emit(f"set_departed_steering_command:{self.usv_departed_namespace_list}")
        self.ui.info_textEdit.append(f"离群设置Steering模式命令已发送: {self.usv_departed_namespace_list}") 
        self.usv_departed_namespace_list.clear()

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
                            self.ui.info_textEdit.append(f"读取数据成功，共 {len(usv_list)} 个 USV 数据")
                            self.ui.info_textEdit.append(f"数据： {self.cluster_position_list} ")
                            # 重置任务状态
                            self.cluster_task_running = False
                            self.cluster_task_paused = False
                            self.update_cluster_task_button_state()
                    else:
                        error_msg = "XML文件格式错误：未找到step节点"
                        print(error_msg)
                        self.ui.info_textEdit.append(error_msg)
                        self.cluster_position_list=[]
                        self.update_cluster_task_button_state()
                    
                except ET.ParseError as e:
                    error_msg = f"XML解析错误: {e}"
                    print(error_msg)
                    self.ui.info_textEdit.append(error_msg)
                    QMessageBox.critical(self, "XML解析错误", error_msg)
                    self.cluster_position_list=[]

                except FileNotFoundError:
                    error_msg = f"文件未找到: {xml_file}"
                    print(error_msg)
                    self.ui.info_textEdit.append(error_msg)
                    QMessageBox.critical(self, "文件错误", error_msg)
                    self.cluster_position_list=[]

                except PermissionError:
                    error_msg = f"没有权限访问文件: {xml_file}"
                    print(error_msg)
                    self.ui.info_textEdit.append(error_msg)
                    QMessageBox.critical(self, "权限错误", error_msg)
                    self.cluster_position_list=[]
                    
                except ValueError as e:
                    error_msg = f"数据格式错误: {e}"
                    print(error_msg)
                    self.ui.info_textEdit.append(error_msg)
                    QMessageBox.critical(self, "数据格式错误", error_msg)
                    self.cluster_position_list=[]

                except Exception as e:
                    error_msg = f"读取文件时发生未知错误: {e}"
                    print(error_msg)
                    self.ui.info_textEdit.append(error_msg)
                    QMessageBox.critical(self, "未知错误", error_msg)
                    self.cluster_position_list=[]
    
    # 启动 RViz2
    def rviz_process(self):
        # 启动 RViz2
        self._rviz_process = QProcess(self)
        self._rviz_process.start("rviz2")
        self.ui.info_textEdit.append(f"启动RViz2")


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
                self.ui.info_textEdit.append("集群任务已暂停")
                # 发送暂停信号给ROS节点（如果需要实现）
                # 这里可以发送一个特殊的信号或清空目标点列表来暂停任务
                self.ros_signal.cluster_target_point_command.emit([])  # 发送空列表表示暂停
            else:
                self.ui.send_cluster_point_pushButton.setText("cluster pause")
                self.ui.info_textEdit.append("集群任务已继续")
                # 重新发送任务数据以恢复任务
                self.ros_signal.cluster_target_point_command.emit(self.cluster_position_list)

    # 开始集群任务
    def start_cluster_task(self):
        """
        开始执行集群任务
        """
        self.ui.info_textEdit.append(f"开始执行集群目标点任务")
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
                self.ui.info_textEdit.append("集群任务已开始执行！")
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
            self.ui.info_textEdit.append("当前没有正在运行的集群任务")
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
            self.ui.info_textEdit.append("集群任务已停止！")
            
            # 清空集群位置列表
            self.cluster_position_list = []
        else:
            self.ui.info_textEdit.append("取消停止操作")

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
        self.ui.info_textEdit.append(f"发送离群目标点: x={x}, y={y}, z={z} 到 {len(departed_target_list)} 个USV")

    # 添加到集群列表
    def add_cluster_command(self):
        """
        将选中的离群USV添加到集群列表
        """
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
        headers = self.TABLE_HEADERS
        for col, key in enumerate(headers):
            index = model.index(selected_row, col)
            state[key] = model.data(index) or "Unknown"

        usv_id = state.get("编号")  # 编号对应namespace
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
            QMessageBox.information(self, "操作成功", f"设备 {usv_id} 已添加到集群列表")
        except Exception as e:
            error_msg = f"错误：添加设备失败 - {str(e)}"
            self.ui.info_textEdit.append(error_msg)
            QMessageBox.critical(self, "操作失败", error_msg)
   
    # 离开集群列表
    def quit_cluster_command(self):
        """
        将选中的集群USV移到离群列表
        """
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
        headers = self.TABLE_HEADERS
        for col, key in enumerate(headers):
            index = model.index(selected_row, col)
            state[key] = model.data(index) or "Unknown"

        usv_id = state.get("编号")  # 编号对应namespace
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
            QMessageBox.information(self, "操作成功", f"设备 {usv_id} 已添加到离群列表")
        except Exception as e:
            error_msg = f"错误：移除设备失败 - {str(e)}"
            self.ui.info_textEdit.append(error_msg)
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
        
        self.ui.info_textEdit.append(progress_text)
        
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
        if not isinstance(msg, list):
            self.ui.warning_textEdit.append("接收到的数据类型错误，期望为列表")
            return
        
        # 验证并过滤 msg 数据
        self.usv_online_list = [ns for ns in msg if isinstance(ns, dict) and ns.get('namespace')]
        
        # 更新离群列表状态
        self._update_departed_list_status()
        
        # 更新集群列表
        self._update_cluster_list()
        
        # 更新表格
        self.update_cluster_table(self.usv_cluster_list)
        self.update_departed_table(self.usv_departed_list)
        self.update_selected_table_row()
    
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
                self.ui.info_textEdit.append(f"警告：离群设备 {departed_id} 未在在线列表中")
    
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
                state = next((ns for ns in self.usv_online_list 
                             if ns.get('namespace') == item.get('namespace')), item)
                self.usv_cluster_list.append(state)
        for ns in temp_list:
            if ns.get('namespace') not in current_cluster_ids:
                self.usv_cluster_list.append(ns)
    
    def update_cluster_table(self, state_list):
        """
        更新集群表格
        
        Args:
            state_list: 状态列表
        """
        try:
            self.cluster_table_model.setRowCount(len(state_list))
            for i, state in enumerate(state_list):
                # 填充表格数据
                self.cluster_table_model.setItem(i, 0, QStandardItem(state.get('namespace', 'Unknown')))
                self.cluster_table_model.setItem(i, 1, QStandardItem(state.get('mode', 'Unknown')))
                self.cluster_table_model.setItem(i, 2, QStandardItem(str(state.get('connected', 'Unknown'))))
                self.cluster_table_model.setItem(i, 3, QStandardItem(str(state.get('armed', 'Unknown'))))
                self.cluster_table_model.setItem(i, 4, QStandardItem(str(state.get('guided', 'Unknown'))))
                self.cluster_table_model.setItem(i, 5, QStandardItem(f"{state.get('battery_percentage', 'Unknown'):.1f}%"))
                self.cluster_table_model.setItem(i, 6, QStandardItem(f"({state.get('position', {}).get('x', 'Unknown'):.2f}, {state.get('position', {}).get('y', 'Unknown'):.2f})"))
                self.cluster_table_model.setItem(i, 7, QStandardItem(f"({state.get('velocity', {}).get('linear', {}).get('x', 'Unknown'):.2f}, {state.get('velocity', {}).get('linear', {}).get('y', 'Unknown'):.2f})"))
                self.cluster_table_model.setItem(i, 8, QStandardItem(f"{state.get('yaw', 'Unknown'):.2f}"))
                # 添加导航状态信息
                usv_id = state.get('namespace', 'Unknown')
                nav_status = self.usv_nav_status.get(usv_id, self.NAV_STATUS_IDLE)
                self.cluster_table_model.setItem(i, 9, QStandardItem(nav_status))
                self.cluster_table_model.setItem(i, 10, QStandardItem(f"{state.get('temperature', 'Unknown'):.1f}"))
                
        except Exception as e:
            self.ui.info_textEdit.append(f"更新集群表格失败: {e}")

    def update_departed_table(self, state_list):
        """
        更新离群表格
        
        Args:
            state_list: 状态列表
        """
        try:
            self.departed_table_model.setRowCount(len(state_list))
            for i, state in enumerate(state_list):
                # 填充表格数据
                self.departed_table_model.setItem(i, 0, QStandardItem(state.get('namespace', 'Unknown')))
                self.departed_table_model.setItem(i, 1, QStandardItem(state.get('mode', 'Unknown')))
                self.departed_table_model.setItem(i, 2, QStandardItem(str(state.get('connected', 'Unknown'))))
                self.departed_table_model.setItem(i, 3, QStandardItem(str(state.get('armed', 'Unknown'))))
                self.departed_table_model.setItem(i, 4, QStandardItem(str(state.get('guided', 'Unknown'))))
                self.departed_table_model.setItem(i, 5, QStandardItem(f"{state.get('battery_percentage', 'Unknown'):.1f}%"))
                self.departed_table_model.setItem(i, 6, QStandardItem(f"({state.get('position', {}).get('x', 'Unknown'):.2f}, {state.get('position', {}).get('y', 'Unknown'):.2f})"))
                self.departed_table_model.setItem(i, 7, QStandardItem(f"({state.get('velocity', {}).get('linear', {}).get('x', 'Unknown'):.2f}, {state.get('velocity', {}).get('linear', {}).get('y', 'Unknown'):.2f})"))
                self.departed_table_model.setItem(i, 8, QStandardItem(f"{state.get('yaw', 'Unknown'):.2f}"))
                # 添加导航状态信息
                usv_id = state.get('namespace', 'Unknown')
                nav_status = self.usv_nav_status.get(usv_id, self.NAV_STATUS_IDLE)
                self.departed_table_model.setItem(i, 9, QStandardItem(nav_status))
                self.departed_table_model.setItem(i, 10, QStandardItem(f"{state.get('temperature', 'Unknown'):.1f}"))
                
        except Exception as e:
            self.ui.info_textEdit.append(f"更新离群表格失败: {e}")

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
            headers = ["namespace", "mode", "connected", "armed", "guided", 
                       "battery_voltage", "battery_prcentage", "power_supply_status", 
                       "position", "velocity", "yaw", "nav_status", "temperature"]
            state = {}
            for col, key in enumerate(headers):
                index = model.index(selected_row, col)
                state[key] = model.data(index) or "Unknown"
            self.ui.usv_id_label.setText(f"{state.get('namespace', 'Unknown')}")
            position_str = str(state.get('position', 'Unknown'))
            yaw_str = str(state.get('yaw', 'Unknown'))
            # 正则提取 x, y, z
            x = y = z = 'Unknown'
            pos_match = re.findall(r'([xyz])\s*=\s*([-+]?\d*\.?\d+)', position_str)
            pos_dict = {k: v for k, v in pos_match}
            try:
                x = float(pos_dict['x']) if 'x' in pos_dict else 'Unknown'
                y = float(pos_dict['y']) if 'y' in pos_dict else 'Unknown'
                z = float(pos_dict['z']) if 'z' in pos_dict else 'Unknown'
            except Exception:
                x = y = z = 'Unknown'
            # 正则提取 yaw
            yaw_val = 'Unknown'
            yaw_match = re.search(r'yaw\s*=\s*([-+]?\d*\.?\d+)', yaw_str)
            if yaw_match:
                try:
                    yaw_val = float(yaw_match.group(1))
                except Exception:
                    yaw_val = 'Unknown'
            else:
                # 兼容直接数字
                try:
                    yaw_val = float(yaw_str)
                except Exception:
                    yaw_val = 'Unknown'
            self.ui.usv_x_label.setText(f"{x:.2f}")
            self.ui.usv_y_label.setText(f"{y:.2f}")
            self.ui.usv_z_label.setText(f"{z:.2f}")
            self.ui.usv_yaw_label.setText(f"{yaw_val:.2f}")
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
        """
        发送颈部摆动命令
        """
        self.ros_signal.str_command.emit('neck_swinging')
        self.ui.info_textEdit.append(f"发送命令: neck_swinging")

   
    # 颈部停止命令
    def neck_stop_command(self):
        """
        发送颈部停止命令
        """
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
        # 更新集群表格表头
        cluster_headers = ["命名空间", "模式", "连接状态", "解锁状态", "引导状态", "电池电量", "位置", "速度", "偏航角", "导航状态", "温度"]
        self.cluster_table_model.setHorizontalHeaderLabels(cluster_headers)
        
        # 更新离群表格表头
        departed_headers = ["命名空间", "模式", "连接状态", "解锁状态", "引导状态", "电池电量", "位置", "速度", "偏航角", "导航状态", "温度"]
        self.departed_table_model.setHorizontalHeaderLabels(departed_headers)

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
        self.ui.info_textEdit.append(
            f"USV {usv_id} 导航反馈 - "
            f"距离目标: {feedback.distance_to_goal:.2f}m, "
            f"航向误差: {feedback.heading_error:.2f}度, "
            f"预计剩余时间: {feedback.estimated_time:.2f}秒"
        )
        
        # 如果有专门的进度条或状态显示控件，可以在这里更新
        # 例如: self.ui.navigation_progress_bar.setValue(int((1 - feedback.distance_to_goal / initial_distance) * 100))

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

    # 添加导航状态更新信号连接
    ros_signal.nav_status_update.connect(main_window.update_nav_status)
    # 添加导航反馈信号连接
    ros_signal.navigation_feedback.connect(main_window.handle_navigation_feedback)

    ros_thread = threading.Thread(target=lambda:rclpy.spin(node), daemon=True) # 创建一个线程来运行 ROS 事件循环
    ros_thread.start()
   
    main_window.show()
    sys.exit(app.exec_())
# 如果这个脚本是主程序入口   
if __name__ == "__main__":
    main()
