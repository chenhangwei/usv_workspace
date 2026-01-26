#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of command processor.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
命令处理器模块
处理各种命令的发送和处理
"""

import queue
from std_msgs.msg import String, Bool


class CommandProcessor:
    def __init__(self, node):
        self.node = node

    def set_manual_callback(self, msg):
        """
        设置USV为手动模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到手动模式命令")
        
        # 手动切换到MANUAL模式时，应该停止所有导航任务
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            # 提取USV ID
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            
            # 发送取消导航消息，通知 USV 端结束导航任务
            # 这会让 velocity_controller_node 进入 CANCELLED 状态，不会自动恢复 GUIDED
            self._send_cancel_navigation(usv_id)
            
            # 如果该USV有正在执行的导航任务，取消它
            if usv_id in self.node._usv_nav_target_cache:
                self.node.get_logger().info(f"🛑 手动切换MANUAL模式，取消 {usv_id} 的导航任务")
                del self.node._usv_nav_target_cache[usv_id]
                # 更新导航状态显示为"已停止"
                self.node.ros_signal.nav_status_update.emit(usv_id, "已停止")
        
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "MANUAL")

    def set_hold_callback(self, msg):
        """
        设置USV为HOLD模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到HOLD模式命令")
        
        # 切换到HOLD模式时，应该停止所有导航任务
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            # 提取USV ID
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            
            # 发送取消导航消息，通知 USV 端结束导航任务
            # 这会让 velocity_controller_node 进入 CANCELLED 状态，不会自动恢复 GUIDED
            self._send_cancel_navigation(usv_id)
            
            # 如果该USV有正在执行的导航任务，取消它
            if usv_id in self.node._usv_nav_target_cache:
                self.node.get_logger().info(f"🛑 切换HOLD模式，取消 {usv_id} 的导航任务")
                del self.node._usv_nav_target_cache[usv_id]
                # 更新导航状态显示为"已停止"
                self.node.ros_signal.nav_status_update.emit(usv_id, "已停止")
            else:
                # 如果没有活动的导航任务，显示为"待命"
                self.node.ros_signal.nav_status_update.emit(usv_id, "待命")
        
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "HOLD")
    
    def _send_cancel_navigation(self, usv_id: str):
        """
        发送取消导航消息给指定的USV
        
        这会通知 USV 端的 velocity_controller_node 进入 CANCELLED 状态，
        从而不会自动恢复 GUIDED 模式。
        
        Args:
            usv_id: USV标识符
        """
        if usv_id in self.node.usv_manager.cancel_navigation_pubs:
            cancel_msg = Bool()
            cancel_msg.data = True
            try:
                self.node.publish_queue.put_nowait(
                    (self.node.usv_manager.cancel_navigation_pubs[usv_id], cancel_msg)
                )
                self.node.get_logger().info(f"📤 向 {usv_id} 发送取消导航消息")
            except Exception as e:
                self.node.get_logger().warn(f"发送取消导航消息失败: {e}")

    def set_guided_callback(self, msg):
        """
        设置USV为导航模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到导航模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "GUIDED")

    def set_arco_callback(self, msg):
        """
        设置USV为ARCO模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到ARCO模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "ARCO")

    def set_steering_callback(self, msg):
        """
        设置USV为舵机模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到舵机模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "STEERING")

    def set_rtl_callback(self, msg):
        """
        设置USV为RTL模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到RTL模式命令")
        
        # 切换到RTL模式时，应该停止所有导航任务
        usv_list = msg if isinstance(msg, list) else [msg]
        for ns in usv_list:
            # 提取USV ID
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            # 如果该USV有正在执行的导航任务，取消它
            if usv_id in self.node._usv_nav_target_cache:
                self.node.get_logger().info(f"🛑 切换RTL模式，取消 {usv_id} 的导航任务")
                del self.node._usv_nav_target_cache[usv_id]
                # 更新导航状态显示为"已停止" (或者"返航中"?)
                self.node.ros_signal.nav_status_update.emit(usv_id, "返航中")
        
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "RTL")

    def _set_mode_for_usvs(self, msg, mode):
        """
        为USV列表设置指定模式
        
        Args:
            msg: 包含USV列表的消息
            mode (str): 要设置的模式
        """
        # 如果消息是列表则直接使用，否则创建包含单个元素的列表
        usv_list = msg if isinstance(msg, list) else [msg]
        # 遍历USV列表
        for ns in usv_list:
            # 提取USV ID
            usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            # 检查USV是否存在对应的模式发布者
            if usv_id in self.node.usv_manager.set_usv_mode_pubs:
                # 创建模式消息
                mode_msg = String()
                mode_msg.data = mode
                # 将消息添加到发布队列
                self.node.publish_queue.put((self.node.usv_manager.set_usv_mode_pubs[usv_id], mode_msg))
            else:
                # 记录警告日志
                self.node.get_logger().warn(f"无效的命名空间 {usv_id}，跳过")

    def set_arming_callback(self, msg):
        """
        武装USV
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到武装命令")
        # 调用通用设置武装状态方法
        self._set_arming_for_usvs(msg, "ARMING")

    def set_disarming_callback(self, msg):
        """
        解除USV武装
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到解除武装命令")
        # 调用通用设置武装状态方法
        self._set_arming_for_usvs(msg, "DISARMING")

    def _set_arming_for_usvs(self, msg, arming_state):
        """
        为USV列表设置武装状态
        
        Args:
            msg: 包含USV列表的消息
            arming_state (str): 要设置的武装状态
        """
        # 如果消息是列表则直接使用，否则创建包含单个元素的列表
        usv_list = msg if isinstance(msg, list) else [msg]
        
        sent_count = 0
        offline_count = 0
        invalid_count = 0
        
        # 遍历USV列表
        for ns in usv_list:
            # 提取USV ID和namespace
            if isinstance(ns, dict):
                usv_id = ns.get('namespace', '').lstrip('/')
            else:
                usv_id = ns.lstrip('/') if isinstance(ns, str) else ns
            
            # 检查USV是否存在对应的武装状态发布者
            if usv_id not in self.node.usv_manager.set_usv_arming_pubs:
                invalid_count += 1
                self.node.get_logger().warn(f"⚠️  {usv_id}: 发布者不存在，跳过")
                continue
            
            # 检查USV是否在线（即使不在集群列表中，只要有状态信息就尝试发送）
            is_online = False
            if usv_id in self.node.usv_states:
                is_online = self.node.usv_states[usv_id].get('connected', False)
            
            if not is_online:
                offline_count += 1
                self.node.get_logger().warn(f"⚠️  {usv_id}: USV离线，跳过 {arming_state} 命令")
                continue
            
            # 创建武装状态消息并发送
            arming_msg = String()
            arming_msg.data = arming_state
            self.node.publish_queue.put((self.node.usv_manager.set_usv_arming_pubs[usv_id], arming_msg))
            sent_count += 1
            self.node.get_logger().info(f"✓ {usv_id}: 发送 {arming_state} 命令")
        
        # 输出统计信息
        if sent_count > 0:
            self.node.get_logger().info(f"📤 {arming_state} 命令已发送至 {sent_count} 艘在线USV")
        if offline_count > 0:
            self.node.get_logger().warn(f"⚠️  跳过 {offline_count} 艘离线USV")
        if invalid_count > 0:
            self.node.get_logger().warn(f"⚠️  跳过 {invalid_count} 个无效USV")

    def str_command_callback(self, msg):
        """
        处理字符串命令（LED、声音、转头等）
        
        Args:
            msg (str): 命令字符串
        """
        # 只把命令快速放入队列，由节点线程的定时器处理，避免在 GUI 线程执行节点逻辑
        try:
            self.node._incoming_str_commands.put_nowait(msg)
        except queue.Full:
            # 如果队列已满，丢弃并记录日志
            self.node.get_logger().warn("incoming str_command 队列已满，丢弃命令")

    def _identify_command_type(self, msg):
        """
        识别命令类型
        
        Args:
            msg (str): 命令字符串
            
        Returns:
            str: 命令类型 ('led', 'sound', 'action', 'unknown')
        """
        # 转换为小写
        msg_lower = msg.lower()
        # 根据关键字识别命令类型
        if "sound" in msg_lower:
            return 'sound'
        elif "led" in msg_lower or "color" in msg_lower:
            return 'led'
        elif "neck" in msg_lower:
            return 'action'
        else:
            return 'unknown'

    def process_incoming_str_commands(self):
        """
        在节点线程中处理从 GUI 入队的字符串命令。
        该方法将原先的 `str_command_callback` 的实际处理逻辑移到节点线程中执行，
        包括识别命令类型、更新本地 LED 状态并将消息放入发布队列。
        """
        try:
            # 处理若干命令以避免单次循环占用过多时间
            max_process = 20
            processed = 0
            while processed < max_process:
                try:
                    msg = self.node._incoming_str_commands.get_nowait()
                except queue.Empty:
                    break

                # 输出到 GUI info 窗口和 ROS logger
                if hasattr(self.node, 'append_info'):
                    self.node.append_info(f"📤 处理命令: {msg}")
                self.node.get_logger().info(f"处理入队命令: {msg}")
                
                # 类型检查
                if not isinstance(msg, str):
                    self.node.get_logger().warn("命令不是字符串，跳过")
                    continue

                # 创建命令消息
                command_str = String()
                command_str.data = msg

                # 识别命令类型
                command_type = self._identify_command_type(msg)

                # 遍历命名空间列表（在节点线程访问 last_ns_list 是安全的）
                sent_count = 0
                skipped_count = 0
                
                for ns in list(self.node.last_ns_list):
                    usv_id = ns.lstrip('/')
                    
                    # 检查USV是否在线（连接状态）
                    is_online = False
                    if usv_id in self.node.usv_states:
                        is_online = self.node.usv_states[usv_id].get('connected', False)
                    
                    # 如果USV离线，跳过命令发送
                    if not is_online:
                        skipped_count += 1
                        continue
                    
                    if command_type == 'led' and usv_id in self.node.usv_manager.led_pubs:
                        self.node._update_local_led_state(usv_id, command_str)
                        # 使用发布队列异步发布
                        try:
                            self.node.publish_queue.put_nowait((self.node.usv_manager.led_pubs[usv_id], command_str))
                            sent_count += 1
                        except queue.Full:
                            self.node.get_logger().warn('发布队列已满，无法发送 LED 命令')
                    if command_type == 'sound' and usv_id in self.node.usv_manager.sound_pubs:
                        try:
                            self.node.publish_queue.put_nowait((self.node.usv_manager.sound_pubs[usv_id], command_str))
                            sent_count += 1
                        except queue.Full:
                            self.node.get_logger().warn('发布队列已满，无法发送声音命令')
                    if command_type == 'action' and usv_id in self.node.usv_manager.action_pubs:
                        try:
                            self.node.publish_queue.put_nowait((self.node.usv_manager.action_pubs[usv_id], command_str))
                            sent_count += 1
                        except queue.Full:
                            self.node.get_logger().warn('发布队列已满，无法发送动作命令')
                
                # 记录发送统计
                if sent_count > 0:
                    self.node.get_logger().info(f"✓ 命令已发送至 {sent_count} 艘在线USV")
                if skipped_count > 0:
                    self.node.get_logger().warn(f"⚠️  跳过 {skipped_count} 艘离线USV")

                processed += 1
        except Exception as e:
            self.node.get_logger().error(f"处理入队字符串命令时出错: {e}")