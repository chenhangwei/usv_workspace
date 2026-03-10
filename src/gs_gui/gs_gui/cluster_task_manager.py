#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of cluster task manager.
#
# Author: chenhangwei
# Date: 2026-01-26
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

    def _parse_xml_content(self, xml_file: str) -> list:
        """
        纯 XML 解析逻辑，返回解析后的位置列表。

        Args:
            xml_file: XML 文件路径

        Returns:
            list: USV 目标点字典列表，按 (step, usv_id) 排序
        
        Raises:
            ET.ParseError, FileNotFoundError, ValueError 等异常
        """
        tree = ET.parse(xml_file)
        root = tree.getroot()

        steps = root.findall("step")
        if not steps:
            return []

        combined_list = []
        NAV_MODE_MAP = {
            'async': 0, '0': 0,
            'sync': 1, '1': 1,
            'rotate': 2, '2': 2,
            'terminal': 3, '3': 3
        }

        for idx, step in enumerate(steps, start=1):
            raw_step_number = step.get("number")
            try:
                step_number = int(raw_step_number) if raw_step_number is not None else idx
            except ValueError:
                step_number = idx

            nav_mode_attr = step.get("nav_mode", "async").lower()
            step_nav_mode = NAV_MODE_MAP.get(nav_mode_attr, 0)

            explicit_sync = step.get("sync")
            if explicit_sync is not None:
                sync_val = (explicit_sync.lower() == "true")
            else:
                sync_val = (step_nav_mode != 0)

            step_sync_timeout = float(step.get("sync_timeout", "10.0"))
            step_arrival_quality = float(step.get("arrival_quality", "0.8"))

            usvs_elem = step.find("usvs")
            if usvs_elem is None:
                continue

            for usv in usvs_elem.findall("usv"):
                usv_id_elem = usv.find("usv_id")
                pos_x_elem = usv.find("position/x")
                pos_y_elem = usv.find("position/y")
                pos_z_elem = usv.find("position/z")

                yaw_val = 0.0
                use_yaw_val = False
                maneuver_type = 0
                maneuver_param = 0.0

                maneuver_node = usv.find("maneuver")
                if maneuver_node is not None:
                    m_type = maneuver_node.get("type", "").lower()
                    if m_type in ["rotate", "spin"]:
                        maneuver_type = 1
                        use_yaw_val = True
                        circles_attr = maneuver_node.get("circles")
                        direction_attr = maneuver_node.get("direction", "clockwise")
                        if circles_attr:
                            try:
                                maneuver_param = float(circles_attr)
                                if direction_attr.lower() in ["ccw", "counter_clockwise", "anticlockwise", "left"]:
                                    maneuver_param = -abs(maneuver_param)
                                else:
                                    maneuver_param = abs(maneuver_param)
                            except ValueError:
                                pass

                yaw_node = usv.find("yaw")
                if yaw_node is not None and maneuver_type == 0:
                    use_attr = yaw_node.get("use")
                    mode_attr = yaw_node.get("mode")
                    if (use_attr and use_attr.lower() == "true") or (mode_attr and mode_attr.lower() == "fixed"):
                        use_yaw_val = True
                    elif mode_attr and mode_attr.lower() == "auto":
                        use_yaw_val = False
                    val_child = yaw_node.find("value")
                    if val_child is not None and val_child.text:
                        try:
                            yaw_val = float(val_child.text)
                        except ValueError:
                            pass
                    elif yaw_node.text and yaw_node.text.strip():
                        try:
                            yaw_val = float(yaw_node.text)
                        except ValueError:
                            pass

                velocity_elem = usv.find("velocity/value")
                led_val = usv.get("led", "")

                effective_nav_mode = step_nav_mode
                if maneuver_type == 1:
                    effective_nav_mode = 2

                usv_data = {
                    "usv_id": usv_id_elem.text if usv_id_elem is not None else "",
                    "position": {
                        "x": float(pos_x_elem.text) if pos_x_elem is not None and pos_x_elem.text is not None else 0.0,
                        "y": float(pos_y_elem.text) if pos_y_elem is not None and pos_y_elem.text is not None else 0.0,
                        "z": float(pos_z_elem.text) if pos_z_elem is not None and pos_z_elem.text is not None else 0.0
                    },
                    "yaw": yaw_val,
                    "use_yaw": use_yaw_val,
                    "maneuver_type": maneuver_type,
                    "maneuver_param": maneuver_param,
                    "velocity": float(velocity_elem.text) if velocity_elem is not None and velocity_elem.text is not None else 0.0,
                    "step": step_number,
                    "sync": sync_val,
                    "nav_mode": effective_nav_mode,
                    "sync_timeout": step_sync_timeout,
                    "arrival_quality_threshold": step_arrival_quality,
                    "led": led_val
                }
                combined_list.append(usv_data)

        if combined_list:
            combined_list.sort(key=lambda item: (item.get("step", 0), item.get("usv_id", "")))

        return combined_list

    def parse_file(self, xml_file: str) -> list:
        """
        解析 XML 任务文件并返回位置列表（不触发 GUI 弹窗和状态更新）。
        供 MissionQueueManager 等外部调用使用。

        Args:
            xml_file: XML 文件路径

        Returns:
            list: 解析后的目标点列表，失败返回空列表
        """
        try:
            return self._parse_xml_content(xml_file)
        except Exception as e:
            self.append_warning(f"解析文件失败: {e}")
            return []

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

                        # ========== 导航模式解析 ==========
                        # 支持的导航模式: async(0), sync(1), rotate(2), terminal(3)
                        NAV_MODE_MAP = {
                            'async': 0, '0': 0,
                            'sync': 1, '1': 1,
                            'rotate': 2, '2': 2,
                            'terminal': 3, '3': 3
                        }
                        nav_mode_attr = step.get("nav_mode", "async").lower()
                        step_nav_mode = NAV_MODE_MAP.get(nav_mode_attr, 0)
                        
                        # 获取 GS 端步骤间同步属性
                        # 如果 XML 显式指定了 sync 属性，则使用该值
                        # 否则根据 nav_mode 自动推导:
                        #   - nav_mode="async" → sync=false (GS端不等待, USV到达即推进)
                        #   - nav_mode="sync"  → sync=true  (GS端等待所有USV完成当前step)
                        #   - nav_mode="rotate"/"terminal" → sync=true
                        explicit_sync = step.get("sync")
                        if explicit_sync is not None:
                            sync_val = (explicit_sync.lower() == "true")
                        else:
                            # 自动推导: async模式默认不同步, 其他模式默认同步
                            sync_val = (step_nav_mode != 0)
                        
                        # 同步模式参数
                        step_sync_timeout = float(step.get("sync_timeout", "10.0"))
                        step_arrival_quality = float(step.get("arrival_quality", "0.8"))
                        
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
                            
                            # Yaw & Maneuver Parsing Logic
                            yaw_val = 0.0
                            use_yaw_val = False
                            maneuver_type = 0
                            maneuver_param = 0.0
                            
                            # 1. Parse <maneuver> tag (Preferred for actions)
                            maneuver_node = usv.find("maneuver")
                            if maneuver_node is not None:
                                m_type = maneuver_node.get("type", "").lower()
                                if m_type in ["rotate", "spin"]:
                                    maneuver_type = 1 # ROTATE
                                    use_yaw_val = True # Maneuver implies specific yaw control
                                    
                                    circles_attr = maneuver_node.get("circles")
                                    direction_attr = maneuver_node.get("direction", "clockwise") # Default to clockwise

                                    if circles_attr:
                                        try:
                                            maneuver_param = float(circles_attr)
                                            # If logic requires counter-clockwise, negate the param (assuming positive is CW)
                                            if direction_attr.lower() in ["ccw", "counter_clockwise", "anticlockwise", "left"]:
                                                maneuver_param = -abs(maneuver_param)
                                            else:
                                                maneuver_param = abs(maneuver_param)
                                        except ValueError:
                                            self.append_warning(f"Step {step_number} USV {usv_id_elem.text}: circles value '{circles_attr}' invalid")

                            # 2. Parse <yaw> tag (Fallback or Static Heading)
                            # Only parse if no maneuver overrides it, or if maneuver allows parallel yaw setting (usually not for spin)
                            yaw_node = usv.find("yaw")
                            if yaw_node is not None and maneuver_type == 0:
                                # Check for attribute use="true" or mode="fixed"
                                use_attr = yaw_node.get("use")
                                mode_attr = yaw_node.get("mode")
                                
                                if (use_attr and use_attr.lower() == "true") or (mode_attr and mode_attr.lower() == "fixed"):
                                    use_yaw_val = True
                                elif mode_attr and mode_attr.lower() == "auto":
                                    use_yaw_val = False

                                # Try to get value from <value> child or direct text
                                val_child = yaw_node.find("value")
                                if val_child is not None and val_child.text:
                                    try:
                                        yaw_val = float(val_child.text)
                                    except ValueError:
                                        pass
                                elif yaw_node.text and yaw_node.text.strip():
                                    try:
                                        yaw_val = float(yaw_node.text)
                                    except ValueError:
                                        pass

                            velocity_elem = usv.find("velocity/value")
                            
                            # 获取 led 属性
                            led_val = usv.get("led", "")

                            # 如果有旋转机动，自动设置为旋转模式
                            effective_nav_mode = step_nav_mode
                            if maneuver_type == 1:  # ROTATE maneuver
                                effective_nav_mode = 2  # NAV_MODE_ROTATE
                            
                            usv_data = {
                                "usv_id": usv_id_elem.text if usv_id_elem is not None else "",
                                "position": {
                                    "x": float(pos_x_elem.text) if pos_x_elem is not None and pos_x_elem.text is not None else 0.0,
                                    "y": float(pos_y_elem.text) if pos_y_elem is not None and pos_y_elem.text is not None else 0.0,
                                    "z": float(pos_z_elem.text) if pos_z_elem is not None and pos_z_elem.text is not None else 0.0
                                },
                                "yaw": yaw_val,
                                "use_yaw": use_yaw_val,
                                "maneuver_type": maneuver_type,
                                "maneuver_param": maneuver_param,
                                "velocity": float(velocity_elem.text) if velocity_elem is not None and velocity_elem.text is not None else 0.0,
                                "step": step_number,
                                "sync": sync_val,
                                "nav_mode": effective_nav_mode,
                                "sync_timeout": step_sync_timeout,
                                "arrival_quality_threshold": step_arrival_quality,
                                "led": led_val
                            }
                            combined_list.append(usv_data)
                            step_usv_count += 1

                        if step_usv_count:
                            nav_mode_names = {0: '异步', 1: '同步', 2: '旋转', 3: '终止'}
                            mode_str = nav_mode_names.get(step_nav_mode, '异步')
                            step_summaries.append(f"步骤 {step_number}: {step_usv_count} 艘 [{mode_str}]")

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
        """停止集群任务执行（含编队模式）"""
        # 如果编队模式正在运行，先停止编队
        if self._is_formation_active():
            self._stop_formation_task()
            return

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
            
            # 保留集群位置列表以便重启
            # self.cluster_position_list = []
            
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
    

    # 状态对应的样式表（背景色 + 圆角）
    STATUS_STYLES = {
        'running': "QLabel { background-color: #2e7d32; color: white; border-radius: 4px; padding: 2px; }", # 深绿色
        'paused': "QLabel { background-color: #f9a825; color: black; border-radius: 4px; padding: 2px; }", # 黄色
        'completed': "QLabel { background-color: #1565c0; color: white; border-radius: 4px; padding: 2px; }", # 深蓝色
        'idle': "QLabel { background-color: #424242; color: #bdbdbd; border-radius: 4px; padding: 2px; }", # 灰色
        'ready': "QLabel { background-color: #00838f; color: white; border-radius: 4px; padding: 2px; }", # 青色
        'stopped': "QLabel { background-color: #c62828; color: white; border-radius: 4px; padding: 2px; }", # 红色
    }

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
        state_label_cn = state_label_map.get(state, state)
        
        # 构造详细日志文本
        progress_text = (f"集群任务进度: 步骤 {current_step}/{total_steps}, "
                        f"完成 {acked_usvs}/{total_usvs} 个USV ({ack_rate*100:.1f}%), "
                        f"耗时 {elapsed_time:.1f}s, 状态 {state_label_cn}")
        
        self.append_info(progress_text)
        
        # ============ 更新 UI 状态标签 ============
        label_text = f"🚀 {state_label_cn}: 第 {current_step} / {total_steps} 步 | {acked_usvs}/{total_usvs} 艘"
        if state == 'completed':
            label_text = f"🏁 任务已完成 (共{total_steps}步)"
        elif state == 'idle':
            label_text = "⏹ 任务未运行"
        elif state == 'paused':
            label_text = f"⏸️ 已暂停: 第 {current_step} / {total_steps} 步"

        # 获取对应的样式并更新标签
        style_css = self.STATUS_STYLES.get(state, self.STATUS_STYLES.get('idle'))
        self._update_status(label_text, style_css)
        # ===============================================

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

    # ==================== 编队模式管理方法 ====================

    def _is_formation_active(self):
        """检查编队模式是否正在运行"""
        return getattr(self, '_formation_running', False)

    def start_formation(self, group_configs: list):
        """
        启动多编队模式
        
        编队模式与集群任务共存：
        - 领队由集群导航任务控制
        - 跟随者被排除在集群任务之外，由编队控制器管理
        
        Args:
            group_configs: 编队组配置列表，每个元素为 dict
        """
        self._formation_running = True
        self._formation_config = group_configs

        # 发送编队启动信号
        self.ros_signal.formation_start_request.emit(group_configs)

        # 更新状态
        from .formation_controller import FormationType
        type_names = {
            0: "人字形",
            1: "横排一字形",
            2: "菱形",
            3: "三角形",
            4: "纵列一字形",
            5: "S形",
            6: "护卫",
        }

        n_groups = len(group_configs)
        total_usvs = sum(1 + len(g.get('follower_ids', [])) for g in group_configs)

        if n_groups == 1:
            cfg = group_configs[0]
            ft = cfg.get('formation_type', 0)
            type_name = type_names.get(ft, "未知")
            n_followers = len(cfg.get('follower_ids', []))
            leader = cfg.get('leader_id', '')
            self._update_status(
                f"🔱 编队模式: {type_name} | 领队: {leader} | {n_followers} 艘跟随",
                "running"
            )
            self.append_info(
                f"编队模式已启动: {type_name}, 领队={leader}, "
                f"跟随者={', '.join(cfg.get('follower_ids', []))}, "
                f"间距=({cfg.get('spacing_along', 1.0)}m × {cfg.get('spacing_cross', 1.0)}m)"
            )
        else:
            summaries = []
            for cfg in group_configs:
                ft = cfg.get('formation_type', 0)
                type_name = type_names.get(ft, "未知")
                leader = cfg.get('leader_id', '')
                n_f = len(cfg.get('follower_ids', []))
                summaries.append(f"{cfg.get('group_id', '')}: {type_name}/{leader}→{n_f}跟随")
            self._update_status(
                f"🔱 多编队模式: {n_groups} 组 | 共 {total_usvs} 艘 USV",
                "running"
            )
            self.append_info(
                f"多编队模式已启动: {n_groups} 组, 共 {total_usvs} 艘\n"
                + "\n".join(f"  {s}" for s in summaries)
            )

    def _stop_formation_task(self):
        """停止编队任务"""
        reply = QMessageBox.question(
            self.parent_widget,
            "确认停止编队",
            "确定要停止当前编队模式吗？\n所有跟随者将切换到 HOLD 模式。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.ros_signal.formation_stop_request.emit()
            self._formation_running = False
            self._formation_config = None
            self.append_info("❌ 编队已解散，所有 USV 已恢复集群任务参与")
            self._update_status("⏹ 编队已解散", "stopped")
            # 恢复 stop 按钮文本
            if self.parent_widget and hasattr(self.parent_widget, 'ui'):
                self.parent_widget.ui.stop_cluster_task_pushButton.setText("cluster stop")
            # 清除绘图窗口编队信息
            if self.parent_widget and hasattr(self.parent_widget, 'usv_plot_window'):
                self.parent_widget.usv_plot_window.set_formation_info([], [])
        else:
            self.append_info("取消停止操作")

    def update_formation_status(self, status_info: dict):
        """
        处理编队状态更新
        
        Args:
            status_info: 编队状态信息字典 (来自某个编队组)
        """
        status = status_info.get('status', 'unknown')
        group_id = status_info.get('group_id', '')
        if status == 'stopped':
            # 单个组停止不一定全部停止，由外部决定
            pass
        elif status == 'running':
            self._formation_running = True
            type_name = status_info.get('formation_type_name', '')
            leader = status_info.get('leader_id', '')
            n = status_info.get('follower_count', 0)
            if group_id:
                self._update_status(
                    f"🔱 {group_id}: {type_name} | 领队: {leader} | {n} 艘跟随",
                    "running"
                )
            else:
                self._update_status(
                    f"🔱 编队运行中: {type_name} | 领队: {leader} | {n} 艘跟随",
                    "running"
                )

    def get_button_text_with_formation(self):
        """获取按钮文本 (编队模式感知)"""
        if self._is_formation_active():
            return "停止编队"
        return self.get_button_text()
