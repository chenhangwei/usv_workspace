#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of cluster controller.
#
# Author: chenhangwei
# Date: 2026-01-26
"""集群控制器模块，负责集群任务的协调与状态管理。"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple
import math

import rclpy

# 导入线程安全工具
from common_utils import ThreadSafeDict


@dataclass
class AckState:
    """跟踪单艘 USV 在当前集群步骤下的确认状态。"""

    step: int
    acked: bool = False
    last_send_time: Optional[float] = None
    retry: int = 0
    ack_time: Optional[float] = None
    received: bool = False  # 新增：目标点是否已被USV确认接收
    timeout_logged: bool = False  # 是否已输出超时日志（避免重复刷屏）



class ClusterTaskState(Enum):
    """集群任务生命周期状态。"""

    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"


class ClusterController:
    def __init__(self, node):
        self.node = node
        # 集群控制相关变量在主节点中管理 (线程安全)
        self._ack_states: Dict[str, AckState] = ThreadSafeDict()
        self._resend_interval = float(getattr(node, '_ack_resend_interval', 2.0))
        self._action_timeout = float(getattr(node, '_cluster_action_timeout', 300.0))
        self._excluded_ids = set()  # 排除的 USV ID 集合（编队跟随者）
        
        # 新增：用于跟踪每个 USV 当前执行到的步骤 (usv_id -> step_number)
        self._usv_step_progress: Dict[str, int] = ThreadSafeDict()

        # ==================== 航点预发送 (Lookahead) ====================
        self._lookahead_enabled = True        # 是否启用预发送
        self._lookahead_steps = 1             # 预发送步数 (当前步+N个后续步)
        self._lookahead_queue_threshold = 2   # 当 USV 队列剩余 < 此值时预发送
        self._lookahead_sent: Dict[str, set] = ThreadSafeDict()  # 已预发送的 (usv_id -> set of steps)
        self._lookahead_completed: Dict[str, set] = ThreadSafeDict()  # 预发送已完成的 (usv_id -> set of steps)

        # 初始化状态变量
        self._state = ClusterTaskState.IDLE
        self._cluster_start_time = None

        # 外部任务完成回调（多任务队列管理器使用）
        self._on_task_completed_callback = None

    def _now(self) -> float:
        """返回当前 ROS 时钟的秒值。"""
        return self.node.get_clock().now().nanoseconds / 1e9

    def set_excluded_ids(self, ids: set) -> None:
        """
        设置排除 USV 集合（编队跟随者）
        
        被排除的 USV 不会参与集群任务执行。
        
        Args:
            ids: 要排除的 USV ID 集合
        """
        self._excluded_ids = set(ids)
        self.node.get_logger().info(
            f"集群控制器: 已设置排除列表 ({len(self._excluded_ids)} 艘): "
            f"{', '.join(sorted(self._excluded_ids)) if self._excluded_ids else '无'}"
        )

    def clear_excluded_ids(self) -> None:
        """清除排除列表，恢复所有 USV 参与集群任务"""
        if self._excluded_ids:
            self.node.get_logger().info(
                f"集群控制器: 已清除排除列表 (移除 {len(self._excluded_ids)} 艘)"
            )
        self._excluded_ids.clear()

    def configure(self, resend_interval: Optional[float] = None, action_timeout: Optional[float] = None) -> None:
        """更新控制参数，可在节点参数加载后调用。"""
        if resend_interval is not None:
            try:
                self._resend_interval = float(resend_interval)
            except (TypeError, ValueError):
                self.node.get_logger().warn(f"ack_resend_interval 参数非法: {resend_interval}, 使用 {self._resend_interval}")
        if action_timeout is not None:
            try:
                self._action_timeout = float(action_timeout)
            except (TypeError, ValueError):
                self.node.get_logger().warn(f"cluster_action_timeout 参数非法: {action_timeout}, 使用 {self._action_timeout}")

    def set_area_context(self, area_center_dict):
        """
        更新区域中心上下文（主要用于日志或触发重新计算）
        注意：实际计算直接使用 self.node._area_center，此处仅作通知接口
        """
        # self.node.get_logger().info(f"ClusterController 区域中心更新通知: {area_center_dict}")
        pass

    def _set_state(self, new_state: ClusterTaskState, reason: Optional[str] = None) -> None:
        """切换集群任务状态并通知 UI。"""
        if new_state == self._state:
            return

        if reason:
            self.node.get_logger().info(
                f"集群任务状态 {self._state.value} -> {new_state.value}: {reason}"
            )
        else:
            self.node.get_logger().info(
                f"集群任务状态 {self._state.value} -> {new_state.value}"
            )

        self._state = new_state
        self._emit_current_progress()

    def _calculate_progress_metrics(self, cluster_usv_list: Optional[list] = None) -> Tuple[int, int, float]:
        """计算当前步骤的进度指标。"""
        if cluster_usv_list is None:
            cluster_usv_list = []
            if getattr(self.node, 'current_targets', None):
                cluster_usv_list = self._get_usvs_by_step(self.node.current_targets, self.node.run_step)

        usv_ids = []
        for usv in cluster_usv_list:
            if not isinstance(usv, dict):
                continue
            usv_id = usv.get('usv_id')
            if not isinstance(usv_id, str) or not usv_id:
                continue
            
            # 过滤掉不在线的 USV (connected=False)
            # 这样只统计在线USV的到达情况作为判断依据
            usv_state = self.node.usv_states.get(usv_id)
            if not usv_state or not usv_state.get('connected', False):
                continue

            usv_ids.append(usv_id)

        total_usvs = len(usv_ids)
        acked_usvs = 0
        for usv_id in usv_ids:
            state = self._ack_states.get(usv_id)
            if state and state.step == self.node.run_step and state.acked:
                acked_usvs += 1

        ack_rate = acked_usvs / total_usvs if total_usvs else 0.0
        return total_usvs, acked_usvs, ack_rate

    def _emit_progress_update(self, total_usvs: int, acked_usvs: int, ack_rate: float) -> None:
        """向 GUI 发送进度信息。"""
        if total_usvs == 0 and self._state == ClusterTaskState.COMPLETED:
            ack_rate = 1.0
        start_time = self._cluster_start_time or 0.0
        elapsed_time = self._now() - start_time if self._cluster_start_time is not None else 0.0

        progress_info = {
            'current_step': min(self.node.run_step, self.node.max_step),
            'total_steps': self.node.max_step,
            'total_usvs': total_usvs,
            'acked_usvs': acked_usvs,
            'ack_rate': ack_rate,
            'start_time': self._cluster_start_time,
            'elapsed_time': elapsed_time,
            'state': self._state.value,
        }
        self.node.ros_signal.cluster_progress_update.emit(progress_info)

    def _emit_current_progress(self, cluster_usv_list: Optional[list] = None) -> None:
        """计算并发送当前的最新进度。"""
        total_usvs, acked_usvs, ack_rate = self._calculate_progress_metrics(cluster_usv_list)
        self._emit_progress_update(total_usvs, acked_usvs, ack_rate)

    def _reset_cluster_task(self, target_state: ClusterTaskState, reason: str, cancel_active: bool = True) -> None:
        """统一重置集群任务状态。"""
        usv_ids_to_hold = []
        
        if cancel_active:
            # 清理所有 USV 的导航目标缓存，并发送 stop_navigation 消息
            for usv_id in list(self.node._usv_nav_target_cache.keys()):
                self._cancel_active_goal(usv_id)
                usv_ids_to_hold.append(usv_id)

        self.node.current_targets = []
        self.node.run_step = 0
        self.node.usv_target_number = 0
        self.node.max_step = 0
        self._ack_states.clear()
        self._cluster_start_time = None
        self._set_state(target_state, reason)
        self._emit_current_progress([])
        
        # 任务停止或完成后，将所有参与的USV设置为HOLD模式
        # 注意：这里直接调用模式切换，不通过 hold_command 信号，
        # 因为 _cancel_active_goal 已经发送了 stop_navigation 消息，
        # 不需要再发送 cancel_navigation (暂停) 消息
        if usv_ids_to_hold and target_state in (ClusterTaskState.IDLE, ClusterTaskState.COMPLETED):
            self.node.get_logger().info(f"集群任务{target_state.value}，将 {len(usv_ids_to_hold)} 个USV设置为HOLD模式")
            # 直接调用模式切换，不发送额外的取消导航消息
            self.node.command_processor._set_mode_for_usvs(usv_ids_to_hold, "HOLD")
            # 更新导航状态显示为"已停止"
            for usv_id in usv_ids_to_hold:
                self.node.ros_signal.nav_status_update.emit(usv_id, "已停止")

    def stop_cluster_task(self, reason: str = "手动停止") -> None:
        """外部请求停止集群任务。"""
        self.node.get_logger().info(f"停止集群任务: {reason}")
        self._reset_cluster_task(ClusterTaskState.IDLE, reason)

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
                self._reset_cluster_task(ClusterTaskState.IDLE, "收到空目标列表")
                return

            # 更新目标点和步骤信息，保存当前接收到的目标列表
            self.node.current_targets = temp_list
            # 初始化运行步骤为1，表示开始执行第一个步骤
            self.node.run_step = 1
            # 初始化USV目标编号为0
            self.node.usv_target_number = 0
            # 计算最大步骤数，遍历所有目标点获取step值的最大值，若列表为空则默认为1
            self.node.max_step = max(target.get('step', 1) for target in temp_list) if temp_list else 0

            # ========== 新任务前取消所有活动目标 ==========
            # 发送 stop_navigation 到所有 USV，清空残留队列
            for uid in list(self.node._usv_nav_target_cache.keys()):
                self._cancel_active_goal(uid)
            self.node.get_logger().info("🔄 新任务开始，已发送 stop_navigation 清空所有 USV 残留队列")

            # 初始化每艇 ack 状态
            self._ack_states.clear()
            self._usv_step_progress.clear()  # 清空步骤进度
            self._lookahead_sent.clear()     # 清空预发送记录
            self._lookahead_completed.clear() # 清空预发送完成记录
            
            # 初始化所有参与任务的 USV 进度
            all_usvs = set(t.get('usv_id') for t in temp_list if t.get('usv_id'))
            for uid in all_usvs:
                if uid not in self._excluded_ids:
                    self._usv_step_progress[uid] = 1

            # 获取当前时间戳，用于记录状态更新时间
            now = self._now()
            # 记录集群操作开始时间，用于超时判断
            self._cluster_start_time = now
            self._set_state(ClusterTaskState.RUNNING, "接收到新的集群目标")

        # 捕获异常并记录错误日志
        except Exception as e:
            self.node.get_logger().error(f"处理集群目标点消息失败: {e}")

    def _get_target_data(self, usv_id, step):
        """获取指定 USV 在指定步骤的目标数据"""
        if not getattr(self.node, 'current_targets', None):
            return None
        for t in self.node.current_targets:
            if t.get('usv_id') == usv_id and t.get('step') == step:
                return t
        return None

    def _get_peer_ids_for_step(self, step):
        """获取指定步骤涉及的所有 USV ID"""
        if not getattr(self.node, 'current_targets', None):
            return []
        return [t.get('usv_id') for t in self.node.current_targets if t.get('step') == step]

    def _send_led_command(self, usv_id, led_cmd):
        """发送 LED 控制命令"""
        if not led_cmd:
            return
            
        cmd_str = str(led_cmd).strip()
        # led=0 表示不动作
        if cmd_str == "0":
            return
            
        full_cmd = cmd_str
        
        if usv_id in self.node.usv_manager.led_pubs:
             from std_msgs.msg import String
             msg = String()
             msg.data = str(full_cmd)
             pub = self.node.usv_manager.led_pubs[usv_id]
             self.node.publish_queue.put((pub, msg))

    def publish_cluster_targets_callback(self):
        """
        定时向每艇下发当前 step 的目标（支持异步步骤和LED控制）
        """
        if not getattr(self.node, 'current_targets', None):
            return
            
        if self._state in (ClusterTaskState.PAUSED, ClusterTaskState.IDLE):
            return
            
        if self._state == ClusterTaskState.COMPLETED:
            return

        if self._cluster_start_time is None:
            self._cluster_start_time = self._now()

        all_task_usvs = set(t.get('usv_id') for t in self.node.current_targets if t.get('usv_id'))
        if not all_task_usvs:
            self._set_state(ClusterTaskState.COMPLETED, "无有效 USV 目标")
            return

        for uid in all_task_usvs:
            if uid not in self._usv_step_progress:
                self._usv_step_progress[uid] = 1

        active_usvs_count = 0
        min_step_progress = 999999
        max_mission_step = getattr(self.node, 'max_step', 1)

        for usv_id in list(self._usv_step_progress.keys()):
            # 增加离线检测：完全忽略如果 USV 不在线，不计入 active_usvs_count
            usv_status = self.node.usv_states.get(usv_id)
            if not usv_status or not usv_status.get('connected', False):
                 continue

            current_step = self._usv_step_progress[usv_id]
            
            if current_step > max_mission_step:
                continue
            
            active_usvs_count += 1
            min_step_progress = min(min_step_progress, current_step)

            target_data = self._get_target_data(usv_id, current_step)
            if not target_data:
                self._usv_step_progress[usv_id] += 1
                continue

            state = self._ack_states.get(usv_id)
            if state is None or state.step != current_step:
                state = AckState(step=current_step)
                self._ack_states[usv_id] = state
                # 自动确认已通过 lookahead 完成的步骤
                completed_set = self._lookahead_completed.get(usv_id)
                if completed_set and current_step in completed_set:
                    state.acked = True
                    state.ack_time = self._now()
                    completed_set.discard(current_step)
                    self.node.get_logger().info(
                        f"⏩ Step {current_step} for {usv_id} 已通过 lookahead 完成，自动确认")

            if state.acked:
                sync_enabled = target_data.get('sync', True)
                
                can_proceed = True
                if sync_enabled:
                    peers = self._get_peer_ids_for_step(current_step)
                    for pid in peers:
                        if pid == usv_id: continue
                        
                        # 增加离线检测：忽略不在线的同伴，不阻碍进度的判别
                        pid_status = self.node.usv_states.get(pid)
                        if not pid_status or not pid_status.get('connected', False):
                             continue

                        p_step = self._usv_step_progress.get(pid, 1)
                        if p_step < current_step:
                            can_proceed = False
                            break
                        elif p_step == current_step:
                            p_state = self._ack_states.get(pid)
                            if not p_state or not p_state.acked:
                                can_proceed = False
                                break
                
                if can_proceed:
                    self.node.get_logger().info(f"USV {usv_id} 完成步骤 {current_step}，进入下一步")
                    self._usv_step_progress[usv_id] += 1
                else:
                    pass 
            else:
                self._execute_usv_step(usv_id, target_data, state)

        if active_usvs_count == 0:
            self._set_state(ClusterTaskState.COMPLETED, "所有 USV 完成任务")
            self._ack_states.clear()
            self._usv_step_progress.clear()
            self.node.run_step = max_mission_step
            self.node.current_targets = []
        else:
            if min_step_progress != 999999:
                self.node.run_step = min_step_progress

    def _execute_usv_step(self, usv_id, target_data, state):
        """执行单个 USV 的步骤：发送目标、LED、处理超时"""
        # 增加离线检测：如果不在线，直接跳过处理，不计时也不报错
        usv_status = self.node.usv_states.get(usv_id)
        if not usv_status or not usv_status.get('connected', False):
             return

        now = self._now()
        should_send = False
        
        if state.last_send_time is None:
            should_send = True
        else:
            if not state.received:
                if (now - state.last_send_time) > self.node._step_timeout:
                    if state.retry < self.node._max_retries:
                        state.retry += 1
                        self.node.get_logger().warn(f"{usv_id} Step {state.step} 超时，重试 {state.retry}")
                        should_send = True
                    else:
                        pass # 超时但不强制失败，等待

        if should_send:
            state.last_send_time = now
            
            led_val = target_data.get('led')
            if led_val:
                self._send_led_command(usv_id, led_val)
                
            pos = target_data.get('position', {})
            if all(k in pos for k in ('x', 'y')):
                # ========== 检查是否已通过 lookahead 预发送 ==========
                # 注意: 仅首次发送时跳过，重试时必须重新发送（可能 USV 已完成/丢失原目标）
                is_retry = state.retry > 0
                sent_set = self._lookahead_sent.get(usv_id)
                if self._lookahead_enabled and sent_set and state.step in sent_set and not is_retry:
                    # 更新 GS 缓存到当前步骤，使反馈匹配能跟踪到 lookahead 步骤
                    la_goal_id = self.node._compute_goal_id(usv_id, state.step)
                    p_g = self._area_to_global(pos)
                    p_l = self._global_to_usv_local(usv_id, p_g)
                    self.node._usv_nav_target_cache[usv_id] = {
                        'goal_id': la_goal_id,
                        'x': float(p_l.get('x', 0.0)),
                        'y': float(p_l.get('y', 0.0)),
                        'z': float(p_l.get('z', 0.0)),
                        'step': state.step,
                        'timestamp': self._now(),
                        'paused': False,
                    }
                    self.node.ros_signal.nav_status_update.emit(usv_id, "执行中")
                    
                    # 已预发送，检查是否也已完成
                    completed_set = self._lookahead_completed.get(usv_id)
                    if completed_set and state.step in completed_set:
                        self.node.get_logger().info(
                            f"⏩ Step {state.step} 已通过 lookahead 预发送并完成 {usv_id}，自动确认")
                        state.acked = True
                        state.ack_time = self._now()
                        completed_set.discard(state.step)
                    else:
                        self.node.get_logger().info(
                            f"⏭️ Step {state.step} 已通过 lookahead 预发送给 {usv_id}，跳过重复发送")
                    
                    # 即使跳过了当前步骤的发送，仍需为后续步骤发送 lookahead
                    # 否则队列会在平滑切换后变空，导致中间航点被误判为"最终航点"
                    if self._lookahead_enabled:
                        self._send_lookahead_goals(usv_id, state.step)
                    return

                p_global = self._area_to_global(pos)
                p_local = self._global_to_usv_local(usv_id, p_global)
                
                yaw = float(target_data.get('yaw', 0.0))
                use_yaw = target_data.get('use_yaw', False)
                maneuver_type = target_data.get('maneuver_type', 0)
                maneuver_param = target_data.get('maneuver_param', 0.0)
                
                # 获取导航模式参数
                nav_mode = target_data.get('nav_mode', 0)
                sync_timeout = target_data.get('sync_timeout', 10.0)
                arrival_quality = target_data.get('arrival_quality_threshold', 0.8)
                
                retry_tag = f" (重试{state.retry})" if is_retry else ""
                self.node.get_logger().info(
                    f"📤执行 Step {state.step} {usv_id}{retry_tag}: Pos=({p_local['x']:.1f}, {p_local['y']:.1f})"
                )
                
                self.node.send_nav_goal_via_topic(
                    usv_id,
                    p_local['x'], p_local['y'], p_local.get('z', 0.0),
                    yaw,
                    use_yaw,
                    self._action_timeout,
                    maneuver_type=maneuver_type,
                    maneuver_param=maneuver_param,
                    step=state.step,
                    nav_mode=nav_mode,
                    sync_timeout=sync_timeout,
                    arrival_quality_threshold=arrival_quality,
                    task_name=target_data.get('task_name', '')
                )
                
                # ========== 航点预发送 (Lookahead) ==========
                if self._lookahead_enabled:
                    self._send_lookahead_goals(usv_id, state.step)
        else:
            if state.retry >= self.node._max_retries:
                 state.acked = False
                 # 只在首次到达最大重试次数时输出错误日志，避免刷屏
                 if not state.timeout_logged:
                     self.node.get_logger().error(f"{usv_id} 超时且已达最大重试次数，将在确认率检查时跳过该USV继续下一步")
                     state.timeout_logged = True

    def _old_publish_cluster_targets_callback(self):
        """保留原方法占位，避免接口问题 (已被新方法替代)"""
        pass

    def _send_lookahead_goals(self, usv_id: str, current_step: int):
        """
        预发送后续航点到 USV 队列 (Lookahead)
        
        当 USV 正在执行 current_step 时，提前发送 current_step+1 ~ current_step+N 的航点
        这样 USV 可以在完成当前航点时立即切换到下一个，无需等待 GS 发送
        
        Args:
            usv_id: USV 标识符
            current_step: 当前正在执行的步骤号
        """
        max_step = getattr(self.node, 'max_step', 1)
        
        # 初始化该 USV 的预发送记录
        if usv_id not in self._lookahead_sent:
            self._lookahead_sent[usv_id] = set()
        
        sent_steps = self._lookahead_sent[usv_id]
        
        # 预发送后续 N 个步骤
        for i in range(1, self._lookahead_steps + 1):
            next_step = current_step + i
            
            # 超出任务范围
            if next_step > max_step:
                break
            
            # 已经预发送过
            if next_step in sent_steps:
                continue
            
            # 获取下一步的目标数据
            target_data = self._get_target_data(usv_id, next_step)
            if not target_data:
                continue
            
            pos = target_data.get('position', {})
            if not all(k in pos for k in ('x', 'y')):
                continue
            
            # 转换坐标
            p_global = self._area_to_global(pos)
            p_local = self._global_to_usv_local(usv_id, p_global)
            
            yaw = float(target_data.get('yaw', 0.0))
            use_yaw = target_data.get('use_yaw', False)
            maneuver_type = target_data.get('maneuver_type', 0)
            maneuver_param = target_data.get('maneuver_param', 0.0)
            nav_mode = target_data.get('nav_mode', 0)
            sync_timeout = target_data.get('sync_timeout', 10.0)
            arrival_quality = target_data.get('arrival_quality_threshold', 0.8)
            
            # 预发送航点 (标记为预发送，不更新缓存)
            self.node.get_logger().info(
                f"📤预发送 Step {next_step} → {usv_id}: Pos=({p_local['x']:.1f}, {p_local['y']:.1f})"
            )
            
            self.node.send_nav_goal_via_topic(
                usv_id,
                p_local['x'], p_local['y'], p_local.get('z', 0.0),
                yaw,
                use_yaw,
                self._action_timeout,
                maneuver_type=maneuver_type,
                maneuver_param=maneuver_param,
                step=next_step,
                nav_mode=nav_mode,
                sync_timeout=sync_timeout,
                arrival_quality_threshold=arrival_quality,
                is_lookahead=True,  # 预发送标记，不更新目标缓存
                task_name=target_data.get('task_name', '')
            )
            
            # 记录已预发送
            sent_steps.add(next_step)


    def _initialize_ack_map_for_step(self, cluster_usv_list):
        """
        为当前步骤初始化确认映射
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        # 获取当前系统时间戳，用于记录操作开始时间
        # 将纳秒转换为秒，便于后续时间计算和比较
        now = self._now()
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
            state = self._ack_states.get(usv_id)
            if state is None or state.step != self.node.run_step:
                # 为新USV初始化确认状态
                self._ack_states[usv_id] = AckState(step=self.node.run_step)
                # ----------------------------------------------------
                # 检查集群开始时间是否已设置，只在第一次初始化时设置
                # 确保集群开始时间记录的是整个集群任务的起始时间
                if self._cluster_start_time is None:
                    # 记录集群操作开始时间，用于后续的超时判断和时间计算
                    self._cluster_start_time = now

    def _process_usv_ack_and_timeouts(self, cluster_usv_list):
        """
        处理USV确认和超时逻辑
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
            
        Returns:
            bool: True表示已进入下一步,False表示继续当前步骤
        """
        # 获取当前时间戳，用于确认时间和超时计算
        now = self._now()
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

            state = self._ack_states.get(usv_id)
            if state is None or state.step != self.node.run_step:
                continue

            if not state.acked:
                last_send_time = state.last_send_time

                # ⚠️ 超时机制说明：
                # - step_timeout：发送目标后等待USV响应的时间（默认25秒）
                # - 这个超时用于检测USV是否在线/是否收到目标
                # - 如果超时，会重试发送（最多max_retries次，默认3次）
                # - 注意：这不是导航完成的超时！导航可以用300秒（cluster_action_timeout）
                # 只有在发送目标后才进行超时检查
                if last_send_time is not None:
                    elapsed = now - last_send_time
                    if elapsed > self.node._step_timeout:
                        self._handle_usv_timeout(usv_id, ns)

        # 检查是否达到最小确认率阈值，如果是则可以进入下一步
        # 返回True表示已进入下一步,需要终止当前流程
        return self._check_and_proceed_on_ack_rate(cluster_usv_list)

    def _handle_usv_timeout(self, usv_id, ns):
        """
        处理USV超时情况
        
        Args:
            usv_id (str): USV标识符
            ns (dict): USV目标信息
        """
        # 增加离线检测：如果不在线，直接跳过处理
        usv_status = self.node.usv_states.get(usv_id)
        if not usv_status or not usv_status.get('connected', False):
             return

        now = self._now()
        # 获取指定USV的确认信息，包含确认状态、确认时间和重试次数
        state = self._ack_states.get(usv_id)
        if state is None:
            return

        # 取消 Action 任务
        self._cancel_active_goal(usv_id) 
        
        # 检查重试次数是否小于最大重试次数，决定是否继续重试
        if state.retry < self.node._max_retries:
            # 增加重试次数计数器
            state.retry += 1

            # 更新最后发送时间 
            state.last_send_time = now 

            # 记录重试日志，包含USV ID和当前重试次数
            self.node.get_logger().warn(f"{usv_id} 超时，重试第 {state.retry} 次")
            # 通过Action接口发送导航目标点
            pos = ns.get('position', {})
            # 不读取yaw参数,让ArduPilot自动调整航向
            # 检查位置信息是否完整
            if not all(k in pos for k in ('x', 'y')):
                self.node.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                return
            # 支持z坐标,yaw设为0让ArduPilot自动调整航向
            self.node.send_nav_goal_via_topic(
                usv_id,
                pos.get('x', 0.0),
                pos.get('y', 0.0),
                pos.get('z', 0.0),
                0.0,  # 不设置航向要求
                self._action_timeout,
            )
        else:
            # 达到最大重试次数，但不应标记为"已确认"
            # ⚠️ 修复：超时失败不等于成功确认，不应设置 acked=True
            # 只记录失败状态，让 _check_and_proceed_on_ack_rate 根据确认率判断是否进入下一步
            state.acked = False  # 明确标记为未确认
            # 只在首次到达最大重试次数时输出错误日志，避免刷屏
            if not state.timeout_logged:
                self.node.get_logger().error(f"{usv_id} 超时且已达最大重试次数，将在确认率检查时跳过该USV继续下一步")
                state.timeout_logged = True

    def _area_to_global(self, p_area):
        """
        将相对于 area_center 的点转换为全局坐标（使用 self.node._area_center）。
        p_area: dict 包含 x,y,z (局部任务坐标)
        返回 dict {'x','y','z'} (全局坐标)
        计算：Global = Rotate(Local) + Offset
        """
        try:
            # 1. 获取中心点参数 (Offset)
            ax = float(self.node._area_center.get('x', 0.0))
            ay = float(self.node._area_center.get('y', 0.0))
            az = float(self.node._area_center.get('z', 0.0))
            angle = float(self.node._area_center.get('angle', 0.0)) # 偏转角(度)
            
            # 2. 获取任务点局部坐标 (Local)
            lx = float(p_area.get('x', 0.0))
            ly = float(p_area.get('y', 0.0))
            lz = float(p_area.get('z', 0.0))
            
            # 3. 如果有角度偏转，先进行旋转 (绕局部原点)
            # 旋转公式 (CCW):
            # x' = x*cos - y*sin
            # y' = x*sin + y*cos
            if angle != 0.0:
                rad = math.radians(angle)
                cos_a = math.cos(rad)
                sin_a = math.sin(rad)
                
                rotated_x = lx * cos_a - ly * sin_a
                rotated_y = lx * sin_a + ly * cos_a
                
                lx = rotated_x
                ly = rotated_y
            
            # 4. 平移 (Offset)
            gx = ax + lx
            gy = ay + ly
            gz = az + lz
            
            result = {'x': gx, 'y': gy, 'z': gz}
            
            # 调试日志（可通过参数控制）
            if self.node.get_parameter('debug_coordinates').value if self.node.has_parameter('debug_coordinates') else False:
                self.node.get_logger().debug(
                    f"坐标转换 Area→Global: {p_area} + center{{'x':{ax},'y':{ay},'z':{az}}} = {result}"
                )
            
            return result
        except Exception as e:
            self.node.get_logger().error(f"Area→Global 转换失败: {e}, 使用原始坐标")
            return {'x': float(p_area.get('x', 0.0)), 'y': float(p_area.get('y', 0.0)), 'z': float(p_area.get('z', 0.0))}

    def _global_to_usv_local(self, usv_id, p_global):
        """
        全局坐标 → USV本地坐标（实际上是同一个坐标系）
        
        坐标系说明:
        - 全局坐标系 (Map/Global): 以定位基站A0为原点
        - USV本地坐标系 (Local): **也是以定位基站A0为原点**（通过set_home设置）
        - 两者是**同一个坐标系**，因此不需要转换！
        
        设计优势:
        - 所有USV共享同一个坐标系（A0为原点），便于集群协作
        - 导航目标点直接使用全局坐标（相对A0的偏移），飞控会正确处理
        - 无需复杂的坐标变换，简化系统架构
        
        Args:
            usv_id: USV标识符
            p_global: 全局坐标 {'x', 'y', 'z'}（相对A0基站）
        
        Returns:
            USV本地坐标 {'x', 'y', 'z'}（与全局坐标相同，因为都是相对A0）
        """
        import math
        
        # 全局坐标系 = USV本地坐标系（都以A0为原点），直接返回
        result = {
            'x': p_global.get('x', 0.0),
            'y': p_global.get('y', 0.0),
            'z': p_global.get('z', 0.0)
        }
        
        # 调试日志
        if self.node.get_parameter('debug_coordinates').value if self.node.has_parameter('debug_coordinates') else False:
            distance = math.sqrt(result['x']**2 + result['y']**2 + result['z']**2)
            
            self.node.get_logger().debug(
                f"📍 Global→Local({usv_id}) [无需转换，都以A0为原点]:\n"
                f"   输入坐标: ({p_global.get('x', 0):.2f}, {p_global.get('y', 0):.2f}, {p_global.get('z', 0):.2f})\n"
                f"   输出坐标: ({result['x']:.2f}, {result['y']:.2f}, {result['z']:.2f})\n"
                f"   距A0距离: {distance:.2f}m"
            )
        
        # 验证结果合理性（相对A0基站的距离）
        MAX_REASONABLE_DISTANCE = 1000.0  # 1km
        distance = math.sqrt(result['x']**2 + result['y']**2 + result['z']**2)
        if distance > MAX_REASONABLE_DISTANCE:
            self.node.get_logger().warning(
                f"[!] {usv_id} 目标点距A0基站距离异常: {distance:.2f}m > {MAX_REASONABLE_DISTANCE}m"
            )
        
        return result

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
        
        # 取消所有活动的导航任务（无论是否是最后一步）
        for usv_id in list(self.node._usv_nav_target_cache.keys()):
            self._cancel_active_goal(usv_id)
        
        # 检查是否已完成所有步骤
        if self.node.run_step > self.node.max_step:
            self._reset_cluster_task(ClusterTaskState.COMPLETED, "全部步骤完成", cancel_active=False)
            # 记录日志信息
            self.node.get_logger().info("全部步骤完成")
            # 通知外部任务完成回调（多任务队列管理器使用）
            if self._on_task_completed_callback:
                try:
                    self._on_task_completed_callback()
                except Exception as e:
                    self.node.get_logger().warn(f"任务完成回调执行失败: {e}")
            # ✅ 修复：任务完成后立即返回，不再执行后续逻辑
            return
        
        # 获取下一步的USV列表
        next_list = self._get_usvs_by_step(self.node.current_targets, self.node.run_step)
        # 获取当前时间
        now = self._now()
        # 重置并初始化确认映射表
        self._ack_states.clear()
        # 为下一步的USV初始化确认映射表
        for ns in next_list:
            uid = ns.get('usv_id')
            if uid:
                self._ack_states[uid] = AckState(step=self.node.run_step)
        # 更新集群开始时间为当前时间
        self._cluster_start_time = now
        self._emit_current_progress(next_list)

    def pause_cluster_task(self):
        """
        暂停集群任务
        
        暂停时仅让 USV 进入 HOLD 模式（保持当前位置），
        不发送 stop_navigation，保留导航缓存和 ack 状态，
        以便恢复时能重新下发目标继续任务。
        """
        if self._state == ClusterTaskState.PAUSED:
            self.node.get_logger().warn("集群任务已处于暂停状态")
            return

        self._set_state(ClusterTaskState.PAUSED, "用户请求暂停")

        # 仅切换到 HOLD 模式，不发送 stop_navigation
        hold_usvs = list(self.node._usv_nav_target_cache.keys())
        if hold_usvs:
            self.node.command_processor._set_mode_for_usvs(hold_usvs, "HOLD")
            for usv_id in hold_usvs:
                self.node.ros_signal.nav_status_update.emit(usv_id, "已暂停")

        self.node.get_logger().info(f"集群任务已暂停，{len(hold_usvs)} 艘 USV 进入 HOLD 模式")

    def resume_cluster_task(self):
        """
        恢复集群任务
        
        重置所有未完成步骤的发送时间和重试计数，
        使定时回调立即重新下发目标。同时清空预发送记录，
        确保 lookahead 目标也会被重新发送。
        """
        if self._state != ClusterTaskState.PAUSED:
            self.node.get_logger().warn("集群任务未处于暂停状态，无需恢复")
            return

        self._cluster_start_time = self._now()

        # 重置所有未确认 USV 的 ack 状态，强制重新下发目标
        resend_count = 0
        for usv_id, state in self._ack_states.items():
            if not state.acked:
                state.last_send_time = None
                state.retry = 0
                state.received = False
                resend_count += 1

        # 清空 lookahead 发送记录，确保预发送目标也会被重新发送
        self._lookahead_sent.clear()

        self._set_state(ClusterTaskState.RUNNING, "恢复集群任务")
        self.node.get_logger().info(
            f"集群任务已恢复，{resend_count} 个目标将被重新下发"
        )

    def is_cluster_task_paused(self):
        """
        检查集群任务是否已暂停
        
        Returns:
            bool: 如果任务已暂停返回True，否则返回False
        """
        return self._state == ClusterTaskState.PAUSED

    def _check_and_proceed_on_ack_rate(self, cluster_usv_list):
        """
        检查确认率是否达到阈值，如果达到则推进到下一步
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
            
        Returns:
            bool: True表示已进入下一步,False表示继续当前步骤
        """
        # 只有当当前步骤有USV时才进行检查
        if not cluster_usv_list:
            self._emit_current_progress(cluster_usv_list)
            return False

        total_usvs, acked_usvs, ack_rate = self._calculate_progress_metrics(cluster_usv_list)

        # 避免除零错误
        if total_usvs == 0:
            self._emit_progress_update(total_usvs, acked_usvs, ack_rate)
            return False

        self._emit_progress_update(total_usvs, acked_usvs, ack_rate)
        
        # 如果确认率超过阈值且当前步骤尚未完成，则进入下一步
        if ack_rate >= self.node.MIN_ACK_RATE_FOR_PROCEED:
            # 检查是否所有USV都已「处理完毕」
            # 已处理 = 成功确认(acked) 或 超时且重试次数耗尽(retry >= max_retries)
            all_resolved = True
            pending_usvs = []
            timeout_failed_usvs = []
            for usv in cluster_usv_list:
                if not isinstance(usv, dict):
                    continue
                usv_id = usv.get('usv_id')
                # 过滤不在线
                usv_status = self.node.usv_states.get(usv_id)
                if not usv_status or not usv_status.get('connected', False):
                    continue

                state = self._ack_states.get(usv_id) if usv_id else None
                if not state or state.step != self.node.run_step:
                    all_resolved = False
                    pending_usvs.append(f"{usv_id}(未初始化)")
                    break
                if state.acked:
                    # 成功确认
                    pass
                elif state.retry >= self.node._max_retries:
                    # 超时失败但重试已耗尽 → 视为「已处理」，不阻塞进入下一步
                    timeout_failed_usvs.append(usv_id)
                else:
                    # 仍在等待中
                    all_resolved = False
                    pending_usvs.append(f"{usv_id}(等待中)")
            
            # 所有USV都已处理（确认或超时耗尽）时进入下一步
            if all_resolved and ack_rate >= self.node.MIN_ACK_RATE_FOR_PROCEED:
                if timeout_failed_usvs:
                    self.node.get_logger().warning(
                        f"⚠️ {', '.join(timeout_failed_usvs)} 超时失败，跳过继续下一步"
                    )
                self.node.get_logger().info(
                    f"确认率达到 {ack_rate*100:.1f}% (阈值: {self.node.MIN_ACK_RATE_FOR_PROCEED*100:.1f}%)，"
                    f"其中 {acked_usvs}/{total_usvs} 个USV已确认，进入下一步"
                )
                self._proceed_to_next_step()
                return True
            elif pending_usvs:
                self.node.get_logger().debug(
                    f"等待USV完成: {', '.join(pending_usvs)}"
                )
        
        # 返回False表示未进入下一步,继续当前步骤
        return False

    def _publish_targets_for_unacked_usvs(self, cluster_usv_list):
        """
        为未确认的USV发布目标点（仅在首次进入步骤时）
        
        Args:
            cluster_usv_list (list): 当前步骤的USV列表
        """
        # 🔍 调试：检查传入的参数
        self.node.get_logger().info(
            f"🔍 [DEBUG] _publish_targets 接收到 {len(cluster_usv_list)} 个USV: "
            f"{[(u.get('usv_id'), u.get('position', {})) for u in cluster_usv_list if isinstance(u, dict)]}"
        )
        now = self._now()
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
            state = self._ack_states.get(usv_id)
            if state is None or state.step != self.node.run_step:
                state = AckState(step=self.node.run_step)
                self._ack_states[usv_id] = state

            # 已确认，跳过
            if state.acked:
                continue

            # ✅ 修复：只在首次发送（last_send_time为None）或超时重试时发送
            # 如果已经发送过（last_send_time不为None）且没有超时，则不再发送
            if state.last_send_time is not None:
                # 检查是否需要超时重试（由_process_usv_ack_and_timeouts处理）
                # 这里只处理首次发送，超时重试由_handle_usv_timeout处理
                continue

            # 首次发送目标点
            # 获取位置信息（文件里的点可能是相对于 area_center）
            pos = ns.get('position', {})
            # 检查位置信息是否完整（至少需要x和y，z可选）
            if not all(k in pos for k in ('x', 'y')):
                self.node.get_logger().warning(f"目标点缺少坐标: {ns}, 跳过")
                continue
            
            # 更新最后发送时间
            state.last_send_time = now

            # 通过Action接口发送导航目标点
            # 不设置航向要求,让ArduPilot在Guided模式下自动调整航向朝向目标点
            # 将 area-relative 转为全局，再转换为 usv 本地坐标（以 usv 启动点为0,0,0）
            p_global = self._area_to_global(pos)
            p_local = self._global_to_usv_local(usv_id, p_global)
            
            # 精简日志：集群控制器发送目标点
            self.node.get_logger().info(
                f"📤 Step {self.node.run_step} → {usv_id}: Local({p_local.get('x', 0.0):.2f}, {p_local.get('y', 0.0):.2f}, {p_local.get('z', 0.0):.2f}) [Auto-Yaw] [首次发送]"
            )
            
            # 支持z坐标,yaw设为0让ArduPilot自动调整航向
            self.node.send_nav_goal_via_topic(
                usv_id,
                p_local.get('x', 0.0),
                p_local.get('y', 0.0),
                p_local.get('z', 0.0),
                0.0,  # 不设置航向要求
                self._action_timeout,
            )

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
        result = [usv for usv in cluster_usv_list if usv.get('step', 0) == step]
        
        # 🔍 调试日志：输出筛选结果
        # if result:
        #     self.node.get_logger().info(
        #         f"🔍 [DEBUG] _get_usvs_by_step(step={step}): 找到 {len(result)} 个USV, "
        #         f"坐标={[(u.get('usv_id'), u.get('position', {}).get('x'), u.get('position', {}).get('y')) for u in result]}"
        #     )
        # else:
        #     self.node.get_logger().warning(f"⚠️  _get_usvs_by_step(step={step}): 未找到任何USV")
        
        return result

    def _cancel_active_goal(self, usv_id):
        """
        停止指定 USV 当前活动的导航任务 (Topic 版本)
        
        发送 stop_navigation 消息，完全清空 USV 的任务和队列。
        用于集群 STOP 操作。
        """
        # 清理目标缓存 (USV 端会自动超时)
        if usv_id in self.node._usv_nav_target_cache:
            self.node.get_logger().warn(f"⚠️  清除 {usv_id} 导航目标缓存...")
            del self.node._usv_nav_target_cache[usv_id]
        
        # 发送 stop_navigation 消息到 USV
        from std_msgs.msg import Bool
        if usv_id in self.node.usv_manager.stop_navigation_pubs:
            stop_msg = Bool()
            stop_msg.data = True
            try:
                self.node.publish_queue.put_nowait(
                    (self.node.usv_manager.stop_navigation_pubs[usv_id], stop_msg)
                )
                self.node.get_logger().info(f"🛑 发送停止导航请求到 {usv_id}")
            except Exception as e:
                self.node.get_logger().error(f"发送停止导航请求失败 {usv_id}: {e}")

    def mark_usv_goal_result(self, usv_id: str, success: bool, goal_step: Optional[int] = None) -> None:
        """根据导航结果更新指定 USV 的 ack 状态。"""
        # self.node.get_logger().info(
        #     f"🔍 [DEBUG] mark_usv_goal_result 被调用: usv_id={usv_id}, success={success}, goal_step={goal_step}, run_step={self.node.run_step}"
        # )
        
        state = self._ack_states.get(usv_id)
        # self.node.get_logger().info(
        #     f"🔍 [DEBUG] state查询结果: state={state}, state.step={state.step if state else 'N/A'}, state.acked={state.acked if state else 'N/A'}"
        # )
        
        # 如果提供了 goal_step，使用它来匹配；否则使用当前 run_step
        # 允许 goal_step 等于 state.step 或 state.step+1（任务可能已进入下一步）
        if state is None:
            self.node.get_logger().warning(
                f"⚠️  {usv_id} state为None，无法更新确认状态"
            )
            return
        
        expected_step = goal_step if goal_step is not None else self.node.run_step
        if state.step != expected_step and state.step != expected_step - 1:
            # 检查是否为 lookahead 预发送的步骤完成结果
            if success and expected_step is not None and expected_step > state.step:
                if usv_id not in self._lookahead_completed:
                    self._lookahead_completed[usv_id] = set()
                self._lookahead_completed[usv_id].add(expected_step)
                self.node.get_logger().info(
                    f"📋 {usv_id} 完成预发送 Step {expected_step} "
                    f"(当前 state.step={state.step})，记录待确认")
                return
            self.node.get_logger().warning(
                f"⚠️  {usv_id} step不匹配! state.step={state.step}, expected_step={expected_step}, run_step={self.node.run_step}"
            )
            return

        if success:
            if not state.acked:
                state.acked = True
                state.ack_time = self._now()
                # self.node.get_logger().info(f"✅ {usv_id} 标记为已确认 (step={state.step})")
                self._emit_current_progress()
        else:
            # 失败情况下保持未确认状态
            # 如果重试次数已耗尽，_check_and_proceed_on_ack_rate 会将其视为已处理并推进
            state.last_send_time = self._now()
            if state.retry >= self.node._max_retries:
                self.node.get_logger().warning(f"❌ {usv_id} 导航失败（重试已耗尽），等待确认率检查推进下一步")
            else:
                self.node.get_logger().warning(f"❌ {usv_id} 导航失败，保持未确认状态等待重试")
            self._emit_current_progress()


