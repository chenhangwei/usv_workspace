"""
集群任务管理模块
负责集群任务的创建、执行、暂停和停止
"""
import xml.etree.ElementTree as ET
from PyQt5.QtWidgets import QFileDialog, QMessageBox


class ClusterTaskManager:
    """集群任务管理器"""
    
    def __init__(self, ros_signal, info_callback, warning_callback, parent_widget=None):
        """
        初始化集群任务管理器
        
        Args:
            ros_signal: ROS信号对象
            info_callback: 信息输出回调
            warning_callback: 警告输出回调
            parent_widget: 父窗口部件
        """
        self.ros_signal = ros_signal
        self.append_info = info_callback
        self.append_warning = warning_callback
        self.parent_widget = parent_widget
        
        # 集群任务状态
        self.cluster_task_running = False
        self.cluster_task_paused = False
        
        # 集群位置列表
        self.cluster_position_list = []
        
        # 集群任务进度信息
        self.cluster_progress_info = {}
    
    def read_data_from_file(self):
        """从XML文件中读取集群任务数据"""
        # 打开文件对话框，选择XML文件
        file_dialog = QFileDialog(self.parent_widget)
        file_dialog.setNameFilter("XML Files (*.xml)")
        file_dialog.setFileMode(QFileDialog.ExistingFiles)
        
        if file_dialog.exec_():
            xml_files = file_dialog.selectedFiles()
            if xml_files:
                xml_file = xml_files[0]
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
                                pos_z_elem = usv.find("position/z")
                                yaw_elem = usv.find("yaw/value")
                                velocity_elem = usv.find("velocity/value")
                                
                                usv_data = {
                                    "usv_id": usv_id_elem.text if usv_id_elem is not None else "",
                                    "position": {
                                        "x": float(pos_x_elem.text) if pos_x_elem is not None and pos_x_elem.text is not None else 0.0,
                                        "y": float(pos_y_elem.text) if pos_y_elem is not None and pos_y_elem.text is not None else 0.0,
                                        "z": float(pos_z_elem.text) if pos_z_elem is not None and pos_z_elem.text is not None else 0.0
                                    },
                                    "yaw": float(yaw_elem.text) if yaw_elem is not None and yaw_elem.text is not None else 0.0,
                                    "velocity": float(velocity_elem.text) if velocity_elem is not None and velocity_elem.text is not None else 0.0,
                                    "step": int(step_number) if step_number is not None else 0
                                }
                                usv_list.append(usv_data)
                            
                            self.cluster_position_list = usv_list
                            self.append_info(f"读取数据成功，共 {len(usv_list)} 个 USV 数据")
                            self.append_info(f"数据： {self.cluster_position_list}")
                            
                            # 重置任务状态
                            self.cluster_task_running = False
                            self.cluster_task_paused = False
                    else:
                        error_msg = "XML文件格式错误：未找到step节点"
                        self.append_info(error_msg)
                        self.cluster_position_list = []
                
                except ET.ParseError as e:
                    error_msg = f"XML解析错误: {e}"
                    self.append_info(error_msg)
                    QMessageBox.critical(self.parent_widget, "XML解析错误", error_msg)
                    self.cluster_position_list = []
                
                except FileNotFoundError:
                    error_msg = f"文件未找到: {xml_file}"
                    self.append_info(error_msg)
                    QMessageBox.critical(self.parent_widget, "文件错误", error_msg)
                    self.cluster_position_list = []
                
                except PermissionError:
                    error_msg = f"没有权限访问文件: {xml_file}"
                    self.append_info(error_msg)
                    QMessageBox.critical(self.parent_widget, "权限错误", error_msg)
                    self.cluster_position_list = []
                
                except ValueError as e:
                    error_msg = f"数据格式错误: {e}"
                    self.append_info(error_msg)
                    QMessageBox.critical(self.parent_widget, "数据格式错误", error_msg)
                    self.cluster_position_list = []
                
                except Exception as e:
                    error_msg = f"读取文件时发生未知错误: {e}"
                    self.append_info(error_msg)
                    QMessageBox.critical(self.parent_widget, "未知错误", error_msg)
                    self.cluster_position_list = []
    
    def toggle_task(self, usv_departed_list):
        """
        切换集群任务的运行状态：运行/暂停
        
        Args:
            usv_departed_list: 离群USV列表
            
        Returns:
            str: 返回按钮应显示的文本
        """
        if not self.cluster_task_running and not self.cluster_position_list:
            self.append_warning("请先导入集群目标点数据")
            return self.get_button_text()
        
        # 如果任务未开始且有目标点数据，则开始任务
        if not self.cluster_task_running and self.cluster_position_list:
            self.start_task(usv_departed_list)
        # 如果任务正在运行，则切换暂停状态
        elif self.cluster_task_running:
            self.cluster_task_paused = not self.cluster_task_paused
            if self.cluster_task_paused:
                self.append_info("集群任务已暂停")
                # 发送暂停信号给ROS节点
                self.ros_signal.cluster_target_point_command.emit([])
            else:
                self.append_info("集群任务已继续")
                # 重新发送任务数据以恢复任务
                self.ros_signal.cluster_target_point_command.emit(self.cluster_position_list)
        
        return self.get_button_text()
    
    def start_task(self, usv_departed_list):
        """
        开始执行集群任务
        
        Args:
            usv_departed_list: 离群USV列表
            
        Returns:
            bool: 任务是否成功启动
        """
        self.append_info("开始执行集群目标点任务")
        
        # 检查是否有集群列表
        if not self.cluster_position_list:
            self.append_warning("集群列表为空")
            return False
        
        # 创建副本并过滤掉离群的USV
        filtered_list = self.cluster_position_list.copy()
        departed_ids = {item.get('namespace') if isinstance(item, dict) else item 
                       for item in usv_departed_list}
        filtered_list = [usv for usv in filtered_list if usv.get('usv_id', '') not in departed_ids]
        
        if not filtered_list:
            self.append_warning("集群列表为空（所有 USV 均在离群列表中）")
            return False
        
        # 弹窗确认
        reply = QMessageBox.question(
            self.parent_widget,
            "确认执行",
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
                # 发送 ROS 信号
                self.ros_signal.cluster_target_point_command.emit(self.cluster_position_list)
                self.append_info("集群任务已开始执行！")
                return True
            except Exception as e:
                QMessageBox.critical(self.parent_widget, "错误", f"任务启动失败: {e}")
                # 出错时重置状态
                self.cluster_task_running = False
                self.cluster_task_paused = False
                return False
        else:
            QMessageBox.information(self.parent_widget, "取消", "任务启动已取消")
            return False
    
    def stop_task(self):
        """停止集群任务执行"""
        # 如果没有正在运行的集群任务，直接返回
        if not self.cluster_task_running:
            self.append_info("当前没有正在运行的集群任务")
            return
        
        # 弹窗确认
        reply = QMessageBox.question(
            self.parent_widget,
            "确认停止",
            "确定要停止当前集群任务吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # 发送停止信号给ROS节点
            self.ros_signal.cluster_target_point_command.emit([])
            
            # 重置任务状态
            self.cluster_task_running = False
            self.cluster_task_paused = False
            self.append_info("集群任务已停止！")
            
            # 清空集群位置列表
            self.cluster_position_list = []
        else:
            self.append_info("取消停止操作")
    
    def is_task_active(self):
        """
        检查集群任务是否处于活动状态（运行中且未暂停）
        
        Returns:
            bool: 如果任务正在运行且未暂停则返回True
        """
        return self.cluster_task_running and not self.cluster_task_paused
    
    def get_button_text(self):
        """
        根据任务状态获取按钮应显示的文本
        
        Returns:
            str: 按钮文本
        """
        if not self.cluster_task_running:
            return "cluster start"
        elif self.cluster_task_paused:
            return "cluster continue"
        else:
            return "cluster pause"
    
    def update_progress(self, progress_info):
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
