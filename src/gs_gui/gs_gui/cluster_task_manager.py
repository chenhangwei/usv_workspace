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
        
        # 状态更新回调
        self.update_status_callback = None
        # 任务加载回调 (用于更新 2D 预览)
        self.task_loaded_callback = None
        
        # 集群任务状态
        self.cluster_task_running = False
        self.cluster_task_paused = False
        
        # 集群位置列表
        self.cluster_position_list = []
        
        # 集群任务进度信息
        self.cluster_progress_info = {}
    
    def set_update_status_callback(self, callback):
        """设置状态更新回调"""
        self.update_status_callback = callback

    def set_task_loaded_callback(self, callback):
        """设置任务加载回调"""
        self.task_loaded_callback = callback

    def _update_status(self, text, style="normal"):
        """内部方法：调用回调更新状态"""
        if self.update_status_callback:
            self.update_status_callback(text, style)

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
                    
                    # 获取所有 step 节点
                    steps = root.findall("step")
                    if not steps:
                        error_msg = "XML文件格式错误：未找到step节点"
                        self.append_info(error_msg)
                        self.cluster_position_list = []
                        return

                    combined_list = []
                    step_summaries = []

                    for idx, step in enumerate(steps, start=1):
                        raw_step_number = step.get("number")
                        try:
                            step_number = int(raw_step_number) if raw_step_number is not None else idx
                        except ValueError:
                            step_number = idx
                            self.append_warning(
                                f"Step 节点 number='{raw_step_number}' 解析失败，使用顺序值 {idx}"
                            )

                        # 获取 sync 属性，默认为 true
                        sync_attr = step.get("sync", "true").lower()
                        sync_val = (sync_attr == "true")
                        
                        usvs_elem = step.find("usvs")
                        if usvs_elem is None:
                            self.append_warning(f"step {step_number} 缺少 usvs 节点，已跳过")
                            continue

                        step_usv_count = 0
                        for usv in usvs_elem.findall("usv"):
                            usv_id_elem = usv.find("usv_id")
                            pos_x_elem = usv.find("position/x")
                            pos_y_elem = usv.find("position/y")
                            pos_z_elem = usv.find("position/z")
                            roll_elem = usv.find("roll/value")
                            pitch_elem = usv.find("pitch/value")
                            yaw_elem = usv.find("yaw/value")
                            velocity_elem = usv.find("velocity/value")
                            
                            # 获取 led 属性
                            led_val = usv.get("led", "")

                            usv_data = {
                                "usv_id": usv_id_elem.text if usv_id_elem is not None else "",
                                "position": {
                                    "x": float(pos_x_elem.text) if pos_x_elem is not None and pos_x_elem.text is not None else 0.0,
                                    "y": float(pos_y_elem.text) if pos_y_elem is not None and pos_y_elem.text is not None else 0.0,
                                    "z": float(pos_z_elem.text) if pos_z_elem is not None and pos_z_elem.text is not None else 0.0
                                },
                                "roll": float(roll_elem.text) if roll_elem is not None and roll_elem.text is not None else 0.0,
                                "pitch": float(pitch_elem.text) if pitch_elem is not None and pitch_elem.text is not None else 0.0,
                                "yaw": float(yaw_elem.text) if yaw_elem is not None and yaw_elem.text is not None else 0.0,
                                "velocity": float(velocity_elem.text) if velocity_elem is not None and velocity_elem.text is not None else 0.0,
                                "step": step_number,
                                "sync": sync_val,
                                "led": led_val
                            }
                            combined_list.append(usv_data)
                            step_usv_count += 1

                        if step_usv_count:
                            sync_str = "同步" if sync_val else "异步"
                            step_summaries.append(f"步骤 {step_number}: {step_usv_count} 艘 [{sync_str}]")

                    if combined_list:
                        combined_list.sort(key=lambda item: (item.get("step", 0), item.get("usv_id", "")))
                        self.cluster_position_list = combined_list
                        
                        # 统计信息
                        unique_steps = sorted(list({item.get("step", 0) for item in combined_list}))
                        total_steps = len(unique_steps)
                        unique_usvs = sorted(list({item.get("usv_id", "") for item in combined_list}))
                        usv_count = len(unique_usvs)
                        
                        # 格式验证
                        is_standard = (root.tag == "cluster")
                        format_status = "符合标准格式" if is_standard else "不符合标准格式 (根节点应为 <cluster>)"
                        
                        # 构建弹窗信息
                        msg_lines = [
                            f"解析结果: 成功",
                            f"格式检查: {format_status}",
                            f"----------------------------------------",
                            f"总步数: {total_steps}",
                            f"参与 USV 数量: {usv_count}",
                            f"USV ID 列表: {', '.join(unique_usvs)}",
                            f"----------------------------------------",
                            "各步骤摘要:"
                        ]
                        if step_summaries:
                            msg_lines.extend([f"  - {s}" for s in step_summaries])
                        
                        info_text = "\n".join(msg_lines)
                        
                        # 弹窗显示
                        QMessageBox.information(self.parent_widget, "XML 文件加载报告", info_text)
                        
                        # 更新状态指示器
                        import os
                        file_name = os.path.basename(xml_file)
                        self._update_status(f"✅ 任务就绪: {file_name} (共 {total_steps} 步)", "ready")

                        self.append_info(
                            f"读取数据成功，共 {usv_count} 艘 USV，涵盖 {total_steps} 个步骤"
                        )
                        if step_summaries:
                            self.append_info("步骤分布：" + "，".join(step_summaries))
                        else:
                            self.append_info(f"数据： {self.cluster_position_list}")

                        # 重置任务状态
                        self.cluster_task_running = False

                        # 触发任务加载回调
                        if self.task_loaded_callback:
                            self.task_loaded_callback(self.cluster_position_list)
                        self.cluster_task_paused = False
                    else:
                        error_msg = "XML文件中未找到任何 USV 数据"
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
            self._update_status("⚠️ 未加载任务", "warning")
            return self.get_button_text()
        
        # 如果任务未开始且有目标点数据，则开始任务
        if not self.cluster_task_running and self.cluster_position_list:
            self.start_task(usv_departed_list)
        # 如果任务正在运行，则切换暂停状态
        elif self.cluster_task_running:
            if not self.cluster_task_paused:
                self.cluster_task_paused = True
                self.append_info("集群任务已暂停")
                self.ros_signal.cluster_pause_request.emit()
                self._update_status("⏸️ 任务暂停", "paused")
            else:
                self.cluster_task_paused = False
                self.append_info("集群任务已继续")
                self.ros_signal.cluster_resume_request.emit()
                self._update_status("🚀 任务继续", "running")
        
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
        
        # 统计参与的唯一 USV 数量
        unique_usvs_in_task = {item.get('usv_id', '') for item in filtered_list}
        usv_count_in_task = len(unique_usvs_in_task)

        # 弹窗确认
        reply = QMessageBox.question(
            self.parent_widget,
            "确认执行",
            f"即将执行 {usv_count_in_task} 个 USV 的集群任务。\n是否继续?",
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
                
                # 通知 UI 清理任务反馈列表（通过回调或信号）
                if self.parent_widget and hasattr(self.parent_widget, 'clear_navigation_feedback_table'):
                    self.parent_widget.clear_navigation_feedback_table()
                
                self.append_info(f"集群任务已开始执行，共 {usv_count_in_task} 艘 USV")
                unique_steps = sorted(list({item.get("step", 0) for item in self.cluster_position_list}))
                self._update_status(f"🚀 正在执行: 第 {unique_steps[0] if unique_steps else 1} / {len(unique_steps)} 步", "running")
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
            self.ros_signal.cluster_stop_request.emit()
            
            # 重置任务状态
            self.cluster_task_running = False
            self.cluster_task_paused = False
            self.append_info("集群任务已停止！")
            self._update_status("⏹ 任务已停止", "stopped")
            
            # 清空集群位置列表
            self.cluster_position_list = []
            
            # 通知 UI 清理任务反馈列表
            if self.parent_widget and hasattr(self.parent_widget, 'clear_navigation_feedback_table'):
                self.parent_widget.clear_navigation_feedback_table()
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
        state = progress_info.get('state', 'unknown')
        state_label_map = {
            'running': '运行中',
            'paused': '已暂停',
            'completed': '已完成',
            'idle': '空闲',
        }
        state_label = state_label_map.get(state, '未知')
        
        # 更新标签显示
        progress_text = (f"集群任务进度: 步骤 {current_step}/{total_steps}, "
                        f"完成 {acked_usvs}/{total_usvs} 个USV ({ack_rate*100:.1f}%), "
                        f"耗时 {elapsed_time:.1f}s, 状态 {state_label}")
        
        self.append_info(progress_text)
        
        if state == 'running':
            self.cluster_task_running = True
            self.cluster_task_paused = False
        elif state == 'paused':
            self.cluster_task_running = True
            self.cluster_task_paused = True
        elif state in ('completed', 'idle'):
            self.cluster_task_running = False
            self.cluster_task_paused = False
        else:
            # 兼容旧信号：通过 ack_rate 判断
            if current_step > total_steps or (current_step == total_steps and ack_rate >= 1.0):
                self.cluster_task_running = False
                self.cluster_task_paused = False
