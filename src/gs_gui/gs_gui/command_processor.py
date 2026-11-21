"""
命令处理器模块
处理各种命令的发送和处理
"""

import queue
from std_msgs.msg import String, Float32


class CommandProcessor:
    def __init__(self, node):
        self.node = node

    def _stop_vehicle_navigation(self, usv_list, reason_mode: str):
        """停止目标载具的集群任务/编队与机载导航目标，避免后续航点把载具重新拉回 OFFBOARD"""
        from std_msgs.msg import Bool

        normalized_ids = []
        for ns in (usv_list if isinstance(usv_list, list) else [usv_list]):
            if isinstance(ns, dict):
                uid = ns.get('namespace', '').lstrip('/')
            else:
                uid = ns.lstrip('/') if isinstance(ns, str) else str(ns)
            if uid:
                normalized_ids.append(uid)

        # 若编队正在运行且涉及目标载具，停止编队（防止 formation_follower_node 10Hz 继续发航点）
        fm = getattr(self.node, 'formation_manager', None)
        if fm is not None and fm.is_any_active():
            fm.stop_all(f"切换 {reason_mode}")
            if hasattr(self.node, 'cluster_controller'):
                self.node.cluster_controller.clear_excluded_ids()

        # 若集群任务正在运行且未暂停，自动暂停集群任务推进（防止下一步自动续发航点）
        cc = getattr(self.node, 'cluster_controller', None)
        if cc is not None and getattr(cc, 'is_cluster_task_running', False) and not getattr(cc, 'is_paused', False):
            cc.pause_cluster_task()
            self.node.get_logger().info(f"⏸️ 切换 {reason_mode}，已自动暂停正在执行的集群任务")

        for usv_id in normalized_ids:
            # 同时发送 clear_target 与 stop_navigation，让 usv_control_node 与 policy_inference_node 全部停航
            if usv_id in self.node.usv_manager.clear_target_pubs:
                clear_msg = Bool()
                clear_msg.data = True
                self.node.usv_manager.clear_target_pubs[usv_id].publish(clear_msg)
            if usv_id in self.node.usv_manager.stop_navigation_pubs:
                stop_msg = Bool()
                stop_msg.data = True
                self.node.usv_manager.stop_navigation_pubs[usv_id].publish(stop_msg)

            if usv_id in self.node._usv_nav_target_cache:
                del self.node._usv_nav_target_cache[usv_id]
                self.node.ros_signal.nav_status_update.emit(usv_id, "已停止")
            else:
                self.node.ros_signal.nav_status_update.emit(usv_id, "待命")

    def set_hold_callback(self, msg):
        """
        设置USV为HOLD模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到HOLD模式命令")
        self._stop_vehicle_navigation(msg, "HOLD")
        # 调用通用设置模式方法，切换到 HOLD（保持位置）
        self._set_mode_for_usvs(msg, "HOLD")

    def set_offboard_callback(self, msg):
        """
        设置USV为OFFBOARD模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到OFFBOARD模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "OFFBOARD")

    def set_stabilized_callback(self, msg):
        """
        设置USV为STABILIZED稳定模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到STABILIZED稳定模式命令")
        self._stop_vehicle_navigation(msg, "STABILIZED")
        # 调用通用设置模式方法 - PX4 STABILIZED
        self._set_mode_for_usvs(msg, "STABILIZED")

    def set_posctl_callback(self, msg):
        """
        设置USV为POSCTL位置控制模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到POSCTL位置控制模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "POSCTL")

    def set_altctl_callback(self, msg):
        """
        设置USV为ALTCTL高度控制模式
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到ALTCTL高度控制模式命令")
        # 调用通用设置模式方法
        self._set_mode_for_usvs(msg, "ALTCTL")

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
        # 调用通用设置武装状态方法 (USV端期望 "arm" 或 "disarm")
        self._set_arming_for_usvs(msg, "arm")

    def set_disarming_callback(self, msg):
        """
        解除USV武装
        
        Args:
            msg: 包含USV列表的消息
        """
        # 记录日志信息
        self.node.get_logger().info("接收到解除武装命令")
        # 上锁前先停止目标载具的导航/编队，避免上锁后仍继续推航点
        self._stop_vehicle_navigation(msg, "DISARM")
        # 调用通用设置武装状态方法 (USV端期望 "arm" 或 "disarm")
        self._set_arming_for_usvs(msg, "disarm")

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
            if arming_state == "arm" and usv_id in self.node.usv_states:
                self.node.usv_states[usv_id]['protection_locked'] = False
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

    def _is_3d_vehicle(self, usv_id: str) -> bool:
        """判断指定载具是否为 3D 载体（飞行球 / 仿生海豚 / 仿生飞艇）"""
        st = self.node.usv_states.get(usv_id, {}) or {}
        vt = str(st.get('vehicle_type') or self.node._usv_vehicle_type.get(usv_id, '')).strip().lower()
        if vt in ('usv', 'usv_diff', 'usv_boat', 'boat', 'diff_boat', 'rover_differential'):
            return False
        if vt in ('flying_sphere', 'bionic_dolphin', 'bionic_airship', 'sphere', 'dolphin', 'airship', 'uav'):
            return True
        pm = str(st.get('platform_mode') or self.node._usv_platform_mode.get(usv_id, '3d')).strip().lower()
        return pm != '2d'

    def set_takeoff_callback(self, payload):
        """
        处理 3D 载体垂直起飞命令：
        - 仅对在线、已手动解锁且未处于降落保护锁止的 3D 载体下发起飞至预高度指令
        - 未解锁或处于降落锁桨保护状态的载体给出明确提示，需用户先手动点击 Arming 解锁
        """
        if isinstance(payload, dict):
            usv_list = payload.get('usv_ids') or []
            req_alt = float(payload.get('altitude', -1.0))
        else:
            usv_list = payload if isinstance(payload, list) else [payload]
            req_alt = -1.0

        sent_3d = []
        skipped_2d = []
        blocked_locked = []
        blocked_disarmed = []

        for ns in usv_list:
            usv_id = ns.get('namespace', '').lstrip('/') if isinstance(ns, dict) else str(ns).lstrip('/')
            if not usv_id:
                continue
            st = self.node.usv_states.get(usv_id, {}) or {}
            if not st.get('connected', False):
                continue
            if not self._is_3d_vehicle(usv_id):
                skipped_2d.append(usv_id)
                continue
            if bool(st.get('protection_locked', False)):
                blocked_locked.append(usv_id)
                continue
            if not bool(st.get('armed', False)):
                blocked_disarmed.append(usv_id)
                continue
            if usv_id not in self.node.usv_manager.takeoff_cmd_pubs:
                continue

            # 更新地面站记录的目标起飞预高度（req_alt > 0 表示 GUI 显式指定统一预高度）
            if req_alt > 0.0:
                self.node._usv_default_altitude[usv_id] = req_alt
                st['takeoff_altitude'] = req_alt
                st['default_altitude'] = req_alt
            msg = Float32()
            msg.data = float(req_alt)
            self.node.publish_queue.put((self.node.usv_manager.takeoff_cmd_pubs[usv_id], msg))
            sent_3d.append(usv_id)

        if sent_3d:
            alt_desc = f"{req_alt:.2f}m" if req_alt > 0.0 else "机型默认预高度"
            info = f"🛫 已向 {', '.join(sent_3d)} 下发垂直起飞指令（目标高度: {alt_desc}）"
            self.node.get_logger().info(info)
            self.node.ros_signal.node_info.emit(info)
        if skipped_2d:
            self.node.get_logger().info(f"ℹ️ 跳过 2D 水面船（无需起飞）: {', '.join(skipped_2d)}")
        if blocked_locked:
            warn = f"🔒 {', '.join(blocked_locked)} 处于降落电机保护锁止状态！请先手动点击【Arming 解锁】后再点击【Takeoff 起飞】。"
            self.node.get_logger().warn(warn)
            self.node.ros_signal.node_warning.emit(warn)
        if blocked_disarmed:
            warn = f"⚠️ {', '.join(blocked_disarmed)} 当前未解锁（Disarmed），请先点击【Arming 解锁】后再点击【Takeoff 起飞】。"
            self.node.get_logger().warn(warn)
            self.node.ros_signal.node_warning.emit(warn)

    def set_land_callback(self, payload):
        """
        处理 3D 载体垂直降落命令：
        - 先停止目标载具的集群任务/编队与水平导航
        - 下发垂直降落至设定降落高度指令，高度到达及以下电机自动锁桨进入保护状态
        """
        if isinstance(payload, dict):
            usv_list = payload.get('usv_ids') or []
            req_land_alt = float(payload.get('altitude', -1.0))
        else:
            usv_list = payload if isinstance(payload, list) else [payload]
            req_land_alt = -1.0

        self._stop_vehicle_navigation(usv_list, "LAND")

        sent_3d = []
        skipped_2d = []
        for ns in usv_list:
            usv_id = ns.get('namespace', '').lstrip('/') if isinstance(ns, dict) else str(ns).lstrip('/')
            if not usv_id:
                continue
            st = self.node.usv_states.get(usv_id, {}) or {}
            if not st.get('connected', False):
                continue
            if not self._is_3d_vehicle(usv_id):
                skipped_2d.append(usv_id)
                continue
            if usv_id not in self.node.usv_manager.land_cmd_pubs:
                continue

            if req_land_alt >= 0.0:
                self.node._usv_landing_altitude[usv_id] = req_land_alt
                st['landing_altitude'] = req_land_alt
            msg = Float32()
            msg.data = float(req_land_alt)
            self.node.publish_queue.put((self.node.usv_manager.land_cmd_pubs[usv_id], msg))
            sent_3d.append(usv_id)

        if sent_3d:
            alt_desc = f"{req_land_alt:.2f}m" if req_land_alt >= 0.0 else "机型默认降落高度"
            info = f"🛬 已向 {', '.join(sent_3d)} 下发垂直降落指令（降落锁桨高度: {alt_desc}，到位后电机自动保护锁止，需手动解锁恢复）"
            self.node.get_logger().info(info)
            self.node.ros_signal.node_info.emit(info)
        if skipped_2d:
            self.node.get_logger().info(f"ℹ️ 跳过 2D 水面船（无需垂直降落）: {', '.join(skipped_2d)}")

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
        if msg_lower.startswith('exclude_cluster:'):
            return 'exclude_cluster'
        elif msg_lower.startswith('include_cluster:'):
            return 'include_cluster'
        elif "sound" in msg_lower:
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

                # 处理集群排除命令 (不通过通用流程遍历发送)
                if command_type == 'exclude_cluster':
                    parts = msg.split(':')
                    if len(parts) > 1:
                        usv_id = parts[1]
                        self.node.cluster_controller.exclude_usv(usv_id)
                    continue
                
                if command_type == 'include_cluster':
                    parts = msg.split(':')
                    if len(parts) > 1:
                        usv_id = parts[1]
                        self.node.cluster_controller.include_usv(usv_id)
                    continue

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