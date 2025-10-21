"""
集群控制器模块
处理集群任务的控制和管理
"""

import rclpy
from rclpy.action import ActionClient
from common_interfaces.action import NavigateToPoint
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class ClusterController:
    def __init__(self, node):
        self.node = node
        # 集群控制相关变量在主节点中管理

    def set_cluster_target_point_callback(self, msg):
        """
        设置集群目标点
        
        Args:
            msg: 包含集群目标点的消息
        """
        try:
            # 记录日志信息
            self.node.get_logger().info("接收到集群目标点")
            # 检查msg对象是否具有targets属性，若有则使用该属性值，否则直接使用msg本身
            temp_list = msg.targets if hasattr(msg, 'targets') else msg
            # 验证消息格式是否正确，确保temp_list是一个列表类型
            if not isinstance(temp_list, list):
                # 记录错误日志
                self.node.get_logger().error("集群目标点格式错误")
                return

            # 检查是否为空列表，用于暂停/停止任务
            if not temp_list:
                self.node.get_logger().info("接收到空列表，暂停/停止集群任务")
                # 取消所有活动的Action任务
                for usv_id in list(self.node._usv_active_goals.keys()):
                    self._cancel_active_goal(usv_id)
                
                # 清空当前目标列表，结束整个集群任务
                self.node.current_targets = []
                # 重置任务状态
                self.node.run_step = 0
                self.node.usv_target_number = 0
                self.node.max_step = 1
                # 清空确认状态映射表
                self.node._usv_ack_map.clear()
                # 重置集群开始时间
                self.node._cluster_start_time = None
                return

            # 更新目标点和步骤信息，保存当前接收到的目标列表
            self.node.current_targets = temp_list
            # 初始化运行步骤为1，表示开始执行第一个步骤
            self.node.run_step = 1
            # 初始化USV目标编号为0
            self.node.usv_target_number = 0
            # 计算最大步骤数，遍历所有目标点获取step值的最大值，若列表为空则默认为1
            self.node.max_step = max(target.get('step', 1) for target in temp_list) if temp_list else 1

            # 初始化每艇 ack 状态，为每个USV设备初始化确认状态
            # 清空之前的确认状态映射表，准备记录新的状态
            self.node._usv_ack_map.clear()
            # 根据当前步骤获取相关的USV列表
            cluster_usv_list = self._get_usvs_by_step(self.node.current_targets, self.node.run_step)
            # 获取当前时间戳，用于记录状态更新时间
            now = self.node.get_clock().now().nanoseconds / 1e9
            # 为每个USV初始化确认状态，遍历USV列表为每个设备设置初始状态
            for ns in cluster_usv_list:
                # 确保ns是字典类型，避免类型错误
                if not isinstance(ns, dict):
                    continue
                # 从字典中获取USV的ID标识
                usv_id = ns.get('usv_id', None)
                # 检查是否成功获取到USV ID
                if usv_id is None:
                    continue
                # 为USV初始化确认状态：未确认、无确认时间、重试次数为0
                self.node._usv_ack_map[usv_id] = {'acked': False, 'last_send_time': None, 'retry': 0}
            # 记录集群操作开始时间，用于超时判断
            self.node._cluster_start_time = now

        # 捕获异常并记录错误日志
        except Exception as e:
            self.node.get_logger().error(f"处理集群目标点消息失败: {e}")

    def publish_cluster_targets_callback(self):
        """
        定时向每艇下发当前 step 的目标，并基于每艇的 reached_target 做独立 ack / 超时处理
        """
        # 检查实例是否存在current_targets属性且不为None，避免在未初始化或重置状态下执行后续逻辑
        # getattr(self.node, 'current_targets', None)是一种安全的属性访问方式：
        # 1. 如果实例存在current_targets属性，返回该属性的值
        # 2. 如果实例不存在current_targets属性，返回默认值None
        # 3. 相比直接访问self.node.current_targets，可以避免AttributeError异常
        if not getattr(self.node, 'current_targets', None):
            return
            
        # 检查任务是否已暂停
        if self.node._cluster_task_paused:
            return

        try:
            # 根据当前步骤获取相关的USV列表，确定本步骤需要操作的无人艇
            cluster_usv_list = self._get_usvs_by_step(self.node.current_targets, self.node.run_step)
            # 检查当前步骤的USV列表是否为空，为空表示没有需要操作的无人艇
            if not cluster_usv_list:
                # 记录警告日志，提示当前步骤无USV需要操作
                self.node.get_logger().warn(f"步骤 {self.node.run_step} 的USV列表为空")
                # 判断是否已达到最大步骤数，若是则表示整个任务完成
                if self.node.run_step >= self.node.max_step:
                    # 清空当前目标列表，结束整个集群任务
                    self.node.current_targets = []
                    # 清空确认状态映射表
                    self.node._usv_ack_map.clear()
                    # 重置集群开始时间
                    self.node._cluster_start_time = None
                    # 重置任务暂停状态
                    self.node._cluster_task_paused = False
                    self.node.get_logger().info("集群任务已完成")
                else:
                    # 否则进入下一步，增加步骤计数器
                    self.node.run_step += 1
                    # 重置集群开始时间
                    self.node._cluster_start_time = self.node.get_clock().now().nanoseconds / 1e9
                # 处理完空列表情况后直接返回，不执行后续逻辑
                return

            # 确保 ack_map 已为当前 step 的艇初始化
            # 调用方法确保当前步骤的所有USV都在确认映射表中正确初始化
            self._initialize_ack_map_for_step(cluster_usv_list)

            # 更新每艇 ack 状态并处理超时/重试
            # 处理每艘USV的确认状态、超时和重试逻辑，确保指令可靠传递
            self._process_usv_ack_and_timeouts(cluster_usv_list)

            # 判断是否所有艇已 ack
            # 检查是否所有USV都已确认接收到目标点，用于判断是否可以进入下一步
            all_acked = all(info['acked'] for info in self.node._usv_ack_map.values()) if self.node._usv_ack_map else False
            # 如果所有USV都已确认，则准备进入下一步
            if all_acked:
                # 调用方法进入下一步操作，更新步骤状态和相关变量
                self._proceed_to_next_step()

            # 继续下发尚未 ack 的艇的目标点
            # 向尚未确认的USV重新发布目标点，确保所有艇都能接收到指令
            self._publish_targets_for_unacked_usvs(cluster_usv_list)

        # 捕获异常并记录错误日志
        except Exception as e:
            self.node.get_logger().error(f"发布集群目标点失败: {e}")

    def _initialize_ack_map_for_step(self, cluster_usv_list):
        """
        为当前步骤初始化确认映射
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        # 获取当前系统时间戳，用于记录操作开始时间
        # 将纳秒转换为秒，便于后续时间计算和比较
        now = self.node.get_clock().now().nanoseconds / 1e9
        # 遍历当前步骤涉及的所有USV，为每艘船初始化确认状态
        for ns in cluster_usv_list:
            # 类型检查：确保当前元素是字典类型，避免因数据格式错误导致异常
            # 这是一种防御性编程实践，提高代码健壮性
            if not isinstance(ns, dict):
                # 检查ns是否为字典类型，如果不是则跳过当前循环
                continue
            # 从字典中安全地提取USV ID，使用get方法避免KeyError异常
            # 如果'usv_id'键不存在，返回None而不是抛出异常
            usv_id = ns.get('usv_id', None)
            # 检查是否成功获取到有效的USV ID，如果ID为空则跳过当前元素
            if usv_id is None:
                # 检查USV ID是否有效，无效则跳过
                continue
            # 检查当前USV是否已经在确认映射表中，避免重复初始化
            # 只有新的USV才会被添加到映射表中
            if usv_id not in self.node._usv_ack_map:
                # 为新USV初始化确认状态，包含三个关键信息：
                # 1. 'acked': False - 表示尚未确认接收到目标点
                # 2. 'ack_time': None - 表示尚未记录确认时间
                # 3. 'retry': 0 - 表示重试次数为0
                # 使用 last_send_time 替换 ack_time ---
                self.node._usv_ack_map[usv_id] = {'acked': False, 'last_send_time': None, 'retry': 0}
                # ----------------------------------------------------
                # 检查集群开始时间是否已设置，只在第一次初始化时设置
                # 确保集群开始时间记录的是整个集群任务的起始时间
                if self.node._cluster_start_time is None:
                    # 记录集群操作开始时间，用于后续的超时判断和时间计算
                    self.node._cluster_start_time = now

    def _process_usv_ack_and_timeouts(self, cluster_usv_list):
        """
        处理USV确认和超时逻辑
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        # 获取当前时间戳，用于确认时间和超时计算
        now = self.node.get_clock().now().nanoseconds / 1e9
        # 遍历当前步骤的所有USV，检查每艘船的确认状态和超时情况
        for ns in cluster_usv_list:
            # 类型检查：确保当前元素是字典类型，避免因数据格式错误导致异常
            if not isinstance(ns, dict):
                # 检查元素是否为字典类型，防止数据格式错误
                continue
            # 安全提取USV ID，使用get方法避免KeyError异常
            usv_id = ns.get('usv_id')
            # 检查是否成功获取到有效的USV ID
            if not usv_id:
                # 无效ID则跳过当前循环
                continue

            # 检查该USV是否已经确认过
            info = self.node._usv_ack_map[usv_id]
            if not info['acked']:
                last_send_time = info.get('last_send_time')
                
                #  核心超时逻辑修改 ---
                # 只有在发送目标后才进行超时检查
                if last_send_time is not None: 
                    elapsed = now - last_send_time 
                    if elapsed > self.node._step_timeout:
                        self._handle_usv_timeout(usv_id, ns)

            # 由于我们使用Action接口处理导航任务，不再依赖is_reached_target字段
            # Action的反馈和结果机制已经提供了任务完成状态
            # 因此，我们使用内部确认机制跟踪任务状态
            # 检查该USV是否已经确认过
            #if not self.node._usv_ack_map[usv_id]['acked']:
                # 计算从集群开始到现在的经过时间，用于超时判断
                # 使用"or now"确保_cluster_start_time为None时不会出现异常
                #elapsed = now - (self.node._cluster_start_time or now)
                # 检查是否超过设定的步骤超时时间
                #if elapsed > self.node._step_timeout:
                    # 调用超时处理函数，执行超时后的相应操作
                    #self._handle_usv_timeout(usv_id, ns)
        
        # 检查是否达到最小确认率阈值，如果是则可以进入下一步
        self._check_and_proceed_on_ack_rate(cluster_usv_list)

    def _handle_usv_timeout(self, usv_id, ns):
        """
        处理USV超时情况
        
        Args:
            usv_id (str): USV标识符
            ns (dict): USV目标信息
        """
        now = self.node.get_clock().now().nanoseconds / 1e9
        # 获取指定USV的确认信息，包含确认状态、确认时间和重试次数
        info = self.node._usv_ack_map[usv_id]

        # 取消 Action 任务
        self._cancel_active_goal(usv_id) 
        
        # 检查重试次数是否小于最大重试次数，决定是否继续重试
        if info['retry'] < self.node._max_retries:
            # 增加重试次数计数器
            info['retry'] += 1

            # 更新最后发送时间 
            info['last_send_time'] = now 

            # 记录重试日志，包含USV ID和当前重试次数
            self.node.get_logger().warn(f"{usv_id} 超时，重试第 {info['retry']} 次")
            # 通过Action接口发送导航目标点
            pos = ns.get('position', {})
            yaw = ns.get('yaw', 0.0)
            # 检查位置信息是否完整
            if not all(k in pos for k in ('x', 'y')):
                self.node.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                return
            # 支持z坐标
            self.node.send_nav_goal_via_action(usv_id, pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0), yaw, 300.0)
        else:
            # 达到最大重试次数，标记为已确认并记录日志，不再重试
            info['acked'] = True
            #info['ack_time'] = self.node.get_clock().now().nanoseconds / 1e9
            # 记录错误日志，说明该USV已超时且达到最大重试次数
            self.node.get_logger().error(f"{usv_id} 超时且已达最大重试次数，跳过并继续下一步")

    def _area_to_global(self, p_area):
        """
        将相对于 area_center 的点转换为全局坐标（使用 self.node._area_center）。
        p_area: dict 包含 x,y,z
        返回 dict {'x','y','z'}
        """
        try:
            ax = float(self.node._area_center.get('x', 0.0))
            ay = float(self.node._area_center.get('y', 0.0))
            az = float(self.node._area_center.get('z', 0.0))
            gx = ax + float(p_area.get('x', 0.0))
            gy = ay + float(p_area.get('y', 0.0))
            gz = az + float(p_area.get('z', 0.0))
            return {'x': gx, 'y': gy, 'z': gz}
        except Exception:
            return {'x': float(p_area.get('x', 0.0)), 'y': float(p_area.get('y', 0.0)), 'z': float(p_area.get('z', 0.0))}

    def _global_to_usv_local(self, usv_id, p_global):
        """
        将全局坐标转换为指定 usv 的本地坐标。
        如果 usv 的 boot pose 不存在，则返回 p_global 作为回退。
        """
        try:
            # 优先使用 boot pose（上电时记录的原点），若不存在则退回到最新状态
            bp = self.node.usv_boot_pose.get(usv_id)
            if bp:
                bx = float(bp.get('x', 0.0))
                by = float(bp.get('y', 0.0))
                bz = float(bp.get('z', 0.0))
                theta = float(bp.get('yaw', 0.0))
            else:
                st = self.node.usv_states.get(usv_id)
                if not st:
                    return p_global
                bx = float(st.get('position', {}).get('x', 0.0))
                by = float(st.get('position', {}).get('y', 0.0))
                bz = float(st.get('position', {}).get('z', 0.0))
                theta = float(st.get('yaw', 0.0))

            dx = float(p_global.get('x', 0.0)) - bx
            dy = float(p_global.get('y', 0.0)) - by
            import math
            ct = math.cos(-theta); stt = math.sin(-theta)
            lx = ct * dx - stt * dy
            ly = stt * dx + ct * dy
            lz = float(p_global.get('z', 0.0)) - bz
            return {'x': lx, 'y': ly, 'z': lz}
        except Exception:
            return p_global

    def _proceed_to_next_step(self):
        """
        推进到下一个步骤
        """
        # 记录日志信息
        self.node.get_logger().info(f"步骤 {self.node.run_step} 完成，推进到下一步")
        # 增加步骤计数器
        self.node.run_step += 1
        # 重置USV目标编号
        self.node.usv_target_number = 0
        
        # 取消所有活动的Action任务（无论是否是最后一步）
        for usv_id in list(self.node._usv_active_goals.keys()):
            self._cancel_active_goal(usv_id)
        
        # 检查是否已完成所有步骤
        if self.node.run_step > self.node.max_step:
            # 清空当前目标列表
            self.node.current_targets = []
            # 清空确认映射表
            self.node._usv_ack_map.clear()
            # 重置集群开始时间
            self.node._cluster_start_time = None
            # 重置任务暂停状态
            self.node._cluster_task_paused = False
            # 记录日志信息
            self.node.get_logger().info("全部步骤完成")
        else:
            # 获取下一步的USV列表
            next_list = self._get_usvs_by_step(self.node.current_targets, self.node.run_step)
            # 获取当前时间
            now = self.node.get_clock().now().nanoseconds / 1e9
            # 重置并初始化确认映射表
            self.node._usv_ack_map.clear()
            # 为下一步的USV初始化确认映射表
            for ns in next_list:
                uid = ns.get('usv_id')
                if uid:
                    self.node._usv_ack_map[uid] = {'acked': False, 'last_send_time': None, 'retry': 0}
            # 更新集群开始时间为当前时间
            self.node._cluster_start_time = now

    def pause_cluster_task(self):
        """
        暂停集群任务
        """
        self.node._cluster_task_paused = True
        # 取消所有活动的Action任务
        for usv_id in list(self.node._usv_active_goals.keys()):
            self._cancel_active_goal(usv_id)
        self.node.get_logger().info("集群任务已暂停")
        
    def resume_cluster_task(self):
        """
        恢复集群任务
        """
        if self.node._cluster_task_paused:
            self.node._cluster_task_paused = False
            # 重置集群开始时间
            self.node._cluster_start_time = self.node.get_clock().now().nanoseconds / 1e9
            self.node.get_logger().info("集群任务已恢复")
        else:
            self.node.get_logger().warn("集群任务未处于暂停状态，无需恢复")
            
    def is_cluster_task_paused(self):
        """
        检查集群任务是否已暂停
        
        Returns:
            bool: 如果任务已暂停返回True，否则返回False
        """
        return self.node._cluster_task_paused

    def _check_and_proceed_on_ack_rate(self, cluster_usv_list):
        """
        检查确认率是否达到阈值，如果达到则进入下一步
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        # 只有当当前步骤有USV时才进行检查
        if not cluster_usv_list:
            return
            
        # 计算当前确认率
        total_usvs = len(cluster_usv_list)
        acked_usvs = sum(1 for usv in cluster_usv_list 
                         if isinstance(usv, dict) and 
                         usv.get('usv_id') in self.node._usv_ack_map and 
                         self.node._usv_ack_map[usv.get('usv_id')]['acked'])
        
        # 避免除零错误
        if total_usvs == 0:
            return
            
        ack_rate = acked_usvs / total_usvs
        
        # 发送进度信息到UI
        progress_info = {
            'current_step': self.node.run_step,
            'total_steps': self.node.max_step,
            'total_usvs': total_usvs,
            'acked_usvs': acked_usvs,
            'ack_rate': ack_rate,
            'start_time': self.node._cluster_start_time,
            'elapsed_time': (self.node.get_clock().now().nanoseconds / 1e9) - (self.node._cluster_start_time or 0)
        }
        self.node.ros_signal.cluster_progress_update.emit(progress_info)
        
        # 如果确认率超过阈值且当前步骤尚未完成，则进入下一步
        if ack_rate >= self.node.MIN_ACK_RATE_FOR_PROCEED:
            # 检查是否所有USV都已处理（确认或超时）
            all_processed = all(
                isinstance(usv, dict) and 
                usv.get('usv_id') in self.node._usv_ack_map and 
                (self.node._usv_ack_map[usv.get('usv_id')]['acked'] or 
                 self.node._usv_ack_map[usv.get('usv_id')]['retry'] >= self.node._max_retries)
                for usv in cluster_usv_list
            )
            
            # 只有当所有USV都已处理时才进入下一步
            if all_processed and ack_rate >= self.node.MIN_ACK_RATE_FOR_PROCEED:
                self.node.get_logger().info(
                    f"确认率达到 {ack_rate*100:.1f}% (阈值: {self.node.MIN_ACK_RATE_FOR_PROCEED*100:.1f}%)，"
                    f"其中 {acked_usvs}/{total_usvs} 个USV已确认，进入下一步"
                )
                self._proceed_to_next_step()

    def _publish_targets_for_unacked_usvs(self, cluster_usv_list):
        """
        为未确认的USV发布目标点（仅在首次进入步骤时）
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        now = self.node.get_clock().now().nanoseconds / 1e9
        # 遍历USV列表
        for ns in cluster_usv_list:
            # 类型检查
            if not isinstance(ns, dict):
                continue
            # 获取USV ID
            usv_id = ns.get('usv_id')
            # 检查USV ID是否有效
            if not usv_id:
                continue
            # 检查USV是否已确认且是首次尝试（retry == 0）
            info = self.node._usv_ack_map.get(usv_id, {})
            if not info.get('acked', False) and info.get('retry', 0) == 0:
                # 获取位置信息（文件里的点可能是相对于 area_center）
                pos = ns.get('position', {})
                # 检查位置信息是否完整（包含x,y,z）
                if not all(k in pos for k in ('x', 'y', 'z')):
                    self.node.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                    continue
                # 更新最后发送时间 ---
                self.node._usv_ack_map[usv_id]['last_send_time'] = now 
               
                # 通过Action接口发送导航目标点
                yaw = ns.get('yaw', 0.0)
                # 将 area-relative 转为全局，再转换为 usv 本地坐标（以 usv 启动点为0,0,0）
                p_global = self._area_to_global(pos)
                p_local = self._global_to_usv_local(usv_id, p_global)
                # 支持z坐标
                self.node.send_nav_goal_via_action(usv_id, p_local.get('x', 0.0), p_local.get('y', 0.0), p_local.get('z', 0.0), yaw, 300.0)

    # 从USV目标列表中筛选出指定步骤(step)的USV目标
    def _get_usvs_by_step(self, cluster_usv_list, step):
        """
        从USV目标列表中筛选出指定步骤的USV目标
        
        Args:
            cluster_usv_list (list): USV目标列表
            step (int): 步骤编号
            
        Returns:
            list: 指定步骤的USV目标列表
        """
        # 使用列表推导式筛选出指定步骤的USV目标
        return [usv for usv in cluster_usv_list if usv.get('step', 0) == step]

    def _cancel_active_goal(self, usv_id):
        """
        取消指定 USV 当前活动的 Action 任务
        """
        if usv_id in self.node._usv_active_goals:
            goal_handle = self.node._usv_active_goals[usv_id]
            # 检查句柄是否仍然有效
            if goal_handle.status in [rclpy.action.client.GoalStatus.STATUS_EXECUTING, rclpy.action.client.GoalStatus.STATUS_ACCEPTED]:
                self.node.get_logger().warn(f"正在取消 USV {usv_id} 的上一个导航任务...")
                cancel_future = goal_handle.cancel_goal_async()
                
                # 可选：等待取消结果以确保取消请求已发送，但这里不做阻塞处理
                # cancel_future.add_done_callback(...)
                
            # 无论是否取消成功，都从跟踪字典中删除
            del self.node._usv_active_goals[usv_id]