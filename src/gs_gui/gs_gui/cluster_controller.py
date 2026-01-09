"""集群控制器模块，负责集群任务的协调与状态管理。"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple

import rclpy

# 导入线程安全工具
from common_utils import ThreadSafeDict


@dataclass
class AckState:
    """跟踪单艘 USV 在当前集群步骤下的确认状态。"""

    step: int
    # received/accepted 层：仅表示 USV 已收到并接受目标，用于停止 step_timeout 重发。
    received: bool = False
    received_time: Optional[float] = None
    acked: bool = False
    last_send_time: Optional[float] = None
    retry: int = 0
    ack_time: Optional[float] = None

    # 姿态动作（roll/pitch）触发状态：用于“接近目标再触发一次 attitude/command”
    attitude_roll: float = 0.0
    attitude_pitch: float = 0.0
    attitude_sent: bool = False


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
        self._cluster_start_time: Optional[float] = None
        self._state = ClusterTaskState.IDLE
        self._excluded_ids = set()  # 排除的 USV ID 集合
        
        # 新增：用于跟踪每个 USV 当前执行到的步骤 (usv_id -> step_number)
        self._usv_step_progress: Dict[str, int] = ThreadSafeDict()

    def _now(self) -> float:
        """返回当前 ROS 时钟的秒值。"""
        return self.node.get_clock().now().nanoseconds / 1e9

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

    def maybe_trigger_attitude_on_feedback(self, usv_id: str, distance_to_goal: float, goal_step: Optional[int] = None) -> None:
        """方案 A：先导航，再在接近目标时触发一次姿态动作。

        - 仅对当前 step 生效
        - 每艇每 step 只触发一次
        - 触发条件：distance_to_goal <= attitude_trigger_distance 且 roll/pitch 非零
        """
        try:
            step = int(goal_step) if goal_step is not None else int(getattr(self.node, 'run_step', 0))
        except Exception:
            step = int(getattr(self.node, 'run_step', 0))

        state = self._ack_states.get(usv_id)
        if state is None or state.step != step:
            return

        # 2D 平台：不触发 roll/pitch 姿态动作，减少无效指令
        try:
            pm = str(getattr(self.node, '_usv_platform_mode', {}).get(usv_id, '3d')).strip().lower()
            if pm == '2d':
                return
        except Exception:
            # 拿不到平台模式时，默认按 3d 处理
            pass

        # 已触发过则不重复
        if bool(getattr(state, 'attitude_sent', False)):
            return

        roll = float(getattr(state, 'attitude_roll', 0.0))
        pitch = float(getattr(state, 'attitude_pitch', 0.0))
        if abs(roll) <= 1e-6 and abs(pitch) <= 1e-6:
            return

        try:
            threshold = float(getattr(self.node, '_attitude_trigger_distance', 1.0))
        except Exception:
            threshold = 1.0
        if not (threshold > 0.0):
            return

        try:
            dist = float(distance_to_goal)
        except Exception:
            return
        if not (dist <= threshold):
            return

        # 触发一次姿态动作
        try:
            duration = float(getattr(self.node, '_attitude_command_duration', 1.0))
        except Exception:
            duration = 1.0

        try:
            self.node.send_attitude_command_via_topic(
                usv_id,
                roll=roll,
                pitch=pitch,
                yaw=None,
                duration=duration,
            )
            state.attitude_sent = True
            self.node.get_logger().info(
                f"🎛️ Step {step} {usv_id}: 距离 {dist:.2f}m ≤ {threshold:.2f}m，触发姿态动作 roll={roll:.3f}, pitch={pitch:.3f}"
            )
        except Exception as e:
            # 失败不置 sent，允许后续 feedback 再触发
            self.node.get_logger().debug(f"{usv_id} 姿态动作触发失败(忽略): {e}")

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
        from std_msgs.msg import Bool
        
        usv_ids_to_manual = []
        
        if cancel_active:
            # 清理所有 USV 的导航目标缓存
            for usv_id in list(self.node._usv_nav_target_cache.keys()):
                self._cancel_active_goal(usv_id)
                usv_ids_to_manual.append(usv_id)
            
            # 向所有在线的 USV 发送清除目标点命令，停止发送 setpoint
            for usv_id in self.node.usv_manager.clear_target_pubs.keys():
                clear_msg = Bool()
                clear_msg.data = True
                self.node.usv_manager.clear_target_pubs[usv_id].publish(clear_msg)
                self.node.get_logger().info(f"📤 发送清除目标点命令到 {usv_id}")
                if usv_id not in usv_ids_to_manual:
                    usv_ids_to_manual.append(usv_id)

        self.node.current_targets = []
        self.node.run_step = 0
        self.node.usv_target_number = 0
        self.node.max_step = 0
        self._ack_states.clear()
        self._cluster_start_time = None
        self._set_state(target_state, reason)
        self._emit_current_progress([])
        
        # 任务停止或完成后，将所有参与的USV设置为HOLD模式
        if usv_ids_to_manual and target_state in (ClusterTaskState.IDLE, ClusterTaskState.COMPLETED):
            self.node.get_logger().info(f"集群任务{target_state.value}，将 {len(usv_ids_to_manual)} 个USV设置为HOLD模式")
            self.node.ros_signal.hold_command.emit(usv_ids_to_manual)
            # 更新导航状态显示为"待命"
            for usv_id in usv_ids_to_manual:
                self.node.ros_signal.nav_status_update.emit(usv_id, "待命")

    def stop_cluster_task(self, reason: str = "手动停止") -> None:
        """外部请求停止集群任务。"""
        self.node.get_logger().info(f"停止集群任务: {reason}")
        self._reset_cluster_task(ClusterTaskState.IDLE, reason)

    def set_cluster_target_point_callback(self, msg):
        """设置集群目标点 (支持 Step Async & LED)"""
        try:
            self.node.get_logger().info("接收到集群目标点")
            temp_list = msg.targets if hasattr(msg, 'targets') else msg
            if not isinstance(temp_list, list):
                self.node.get_logger().error("集群目标点格式错误")
                return

            if not temp_list:
                self.node.get_logger().info("接收到空列表，暂停/停止集群任务")
                self._reset_cluster_task(ClusterTaskState.IDLE, "收到空目标列表")
                return

            # 更新数据
            self.node.current_targets = temp_list
            self.node.run_step = 1
            self.node.usv_target_number = 0
            self.node.max_step = max(target.get('step', 1) for target in temp_list) if temp_list else 0

            # 重置状态
            self._ack_states.clear()
            self._usv_step_progress.clear()
            self._cluster_start_time = self._now()

            # 初始化所有参与任务的 USV 进度
            all_usvs = set(t.get('usv_id') for t in temp_list if t.get('usv_id'))
            for uid in all_usvs:
                if uid not in self._excluded_ids:
                    self._usv_step_progress[uid] = 1

            self._set_state(ClusterTaskState.RUNNING, "接收到新的集群目标")
            
            # 可选：立即触发一次发布
            # self.publish_cluster_targets_callback()

        except Exception as e:
            self.node.get_logger().error(f"设置集群目标点失败: {e}")

        # 捕获异常并记录错误日志
        except Exception as e:
            self.node.get_logger().error(f"处理集群目标点消息失败: {e}")

    def _get_usvs_by_step(self, targets, step):
        """获取指定步骤的所有 USV 目标数据"""
        if not targets:
            return []
        return [t for t in targets if t.get('step') == step]

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
        
        # 用户需求：led=0 表示不动作
        if cmd_str == "0":
            return
            
        full_cmd = cmd_str
        
        # 构造消息并放入发送队列
        # 使用 GroundStationNode 的 publish_queue
        # 需要获取 topic name string for that usv
        # usv_manager.led_pubs has publishers, but queue expects (publisher, msg)
        
        if usv_id in self.node.usv_manager.led_pubs:
             from std_msgs.msg import String
             msg = String()
             msg.data = str(full_cmd)
             pub = self.node.usv_manager.led_pubs[usv_id]
             self.node.publish_queue.put((pub, msg))
             # self.node.get_logger().info(f"LED Cmd -> {usv_id}: {full_cmd}")

    def publish_cluster_targets_callback(self):
        """
        定时向每艇下发当前 step 的目标（支持异步步骤和LED控制）
        """
        if not getattr(self.node, 'current_targets', None):
            return
            
        if self._state in (ClusterTaskState.PAUSED, ClusterTaskState.IDLE):
            return
            
        if self._state == ClusterTaskState.COMPLETED:
            # 防止重复 Complete
            return

        # 初始化集群开始时间
        if self._cluster_start_time is None:
            self._cluster_start_time = self._now()

        # 1. 识别所有参与任务的 USV
        all_task_usvs = set(t.get('usv_id') for t in self.node.current_targets if t.get('usv_id'))
        if not all_task_usvs:
            self._set_state(ClusterTaskState.COMPLETED, "无有效 USV 目标")
            return

        # 2. 初始化进度 (如果没有)
        for uid in all_task_usvs:
            if uid not in self._usv_step_progress:
                self._usv_step_progress[uid] = 1

        active_usvs_count = 0
        min_step_progress = 999999
        max_mission_step = getattr(self.node, 'max_step', 1)

        # 3. 遍历每个 USV，处理其当前步骤
        # 使用 list(keys) 避免迭代中修改字典（虽然这里只修改值）
        for usv_id in list(self._usv_step_progress.keys()):
            current_step = self._usv_step_progress[usv_id]
            
            # 如果步骤超过最大值，视为完成
            if current_step > max_mission_step:
                continue
            
            active_usvs_count += 1
            min_step_progress = min(min_step_progress, current_step)

            # 获取目标数据
            target_data = self._get_target_data(usv_id, current_step)
            if not target_data:
                # 该 USV 在此步骤无任务，自动进入下一步
                self._usv_step_progress[usv_id] += 1
                continue

            # 获取或初始化 AckState
            state = self._ack_states.get(usv_id)
            if state is None or state.step != current_step:
                state = AckState(step=current_step)
                self._ack_states[usv_id] = state

            # 4. 检查完成状态与 Sync 逻辑
            if state.acked:
                sync_enabled = target_data.get('sync', True)
                
                can_proceed = True
                if sync_enabled:
                    # Sync=True: 检查该步骤的所有 peer 是否都准备好
                    peers = self._get_peer_ids_for_step(current_step)
                    for pid in peers:
                        if pid == usv_id: continue
                        
                        p_step = self._usv_step_progress.get(pid, 1)
                        if p_step < current_step:
                            # 队友还在后面，等待
                            can_proceed = False
                            break
                        elif p_step == current_step:
                            # 队友在同一步，检查是否 acked
                            p_state = self._ack_states.get(pid)
                            if not p_state or not p_state.acked:
                                can_proceed = False
                                break
                        # if p_step > current_step: 队友已通过，不阻碍
                
                if can_proceed:
                    self.node.get_logger().info(f"USV {usv_id} 完成步骤 {current_step}，进入下一步")
                    self._usv_step_progress[usv_id] += 1
                    # 清理旧状态以便下次重新初始化? 
                    # 不，AckState 会在下次循环时因 step 不匹配而重建
                else:
                    pass # 等待同步
            else:
                # 5. 未完成 (Ack=False)，执行发送逻辑 (含超时重试)
                self._execute_usv_step(usv_id, target_data, state)

        # 6. 更新全局状态
        if active_usvs_count == 0:
            self._set_state(ClusterTaskState.COMPLETED, "所有 USV 完成任务")
            # 清理
            self._ack_states.clear()
            self._usv_step_progress.clear()
            self.node.run_step = max_mission_step
            self.node.current_targets = []
        else:
            # 更新 GUI 显示的步骤 (显示进度的滞后沿)
            if min_step_progress != 999999:
                self.node.run_step = min_step_progress

        # 推送进度用于UI更新
        # 计算总体 Ack Rate? 
        # 这里简化：只要 active_usvs_count > 0 就是 Running
        pass

    def _execute_usv_step(self, usv_id, target_data, state):
        """执行单个 USV 的步骤：发送目标、LED、处理超时"""
        now = self._now()
        
        # 检查是否超时需要重试
        should_send = False
        
        if state.last_send_time is None:
            # 首次发送
            should_send = True
        else:
            # 这里仅做"是否收到"的超时检测 (step_timeout)
            if not state.received:
                if (now - state.last_send_time) > self.node._step_timeout:
                    if state.retry < self.node._max_retries:
                        state.retry += 1
                        self.node.get_logger().warn(f"{usv_id} Step {state.step} 超时，重试 {state.retry}")
                        should_send = True
                    else:
                        # 超过最大重试，标记失败但不卡死? 
                        # 原逻辑是标记失败，但不 ack
                        pass

        if should_send:
            state.last_send_time = now
            
            # 1. 发送 LED 指令
            led_val = target_data.get('led')
            if led_val:
                self._send_led_command(usv_id, led_val)
                
            # 2. 发送导航目标
            pos = target_data.get('position', {})
            if all(k in pos for k in ('x', 'y')):
                p_global = self._area_to_global(pos)
                p_local = self._global_to_usv_local(usv_id, p_global)
                
                roll = float(target_data.get('roll', 0.0))
                pitch = float(target_data.get('pitch', 0.0))
                yaw = float(target_data.get('yaw', 0.0))
                
                # 设置姿态触发 (用于 maybe_trigger_attitude_on_feedback)
                state.attitude_roll = roll
                state.attitude_pitch = pitch
                state.attitude_sent = False
                
                self.node.get_logger().info(
                    f"📤执行 Step {state.step} {usv_id}: Pos=({p_local['x']:.1f}, {p_local['y']:.1f}), LED={led_val}, Sync={target_data.get('sync', True)}"
                )
                
                self.node.send_nav_goal_via_topic(
                    usv_id,
                    p_local['x'], p_local['y'], p_local.get('z', 0.0),
                    yaw,
                    self._action_timeout
                )
        else:
            # 达到最大重试次数，但不应标记为"已确认"
            # ⚠️ 修复：超时失败不等于成功确认，不应设置 acked=True
            # 只记录失败状态，让 _check_and_proceed_on_ack_rate 根据确认率判断是否进入下一步
            state.acked = False  # 明确标记为未确认
            # 记录错误日志，说明该USV已超时且达到最大重试次数
            self.node.get_logger().error(f"{usv_id} 超时且已达最大重试次数，标记为失败（不进入下一步）")

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

    def pause_cluster_task(self):
        """
        暂停集群任务
        """
        if self._state == ClusterTaskState.PAUSED:
            self.node.get_logger().warn("集群任务已处于暂停状态")
            return

        self._set_state(ClusterTaskState.PAUSED, "用户请求暂停")
        # 取消所有活动的导航任务
        for usv_id in list(self.node._usv_nav_target_cache.keys()):
            self._cancel_active_goal(usv_id)
        self.node.get_logger().info("集群任务已暂停")

    def resume_cluster_task(self):
        """
        恢复集群任务
        """
        if self._state != ClusterTaskState.PAUSED:
            self.node.get_logger().warn("集群任务未处于暂停状态，无需恢复")
            return

        self._cluster_start_time = self._now()
        self._set_state(ClusterTaskState.RUNNING, "恢复集群任务")
        self.node.get_logger().info("集群任务已恢复")

    def is_cluster_task_paused(self):
        """
        检查集群任务是否已暂停
        
        Returns:
            bool: 如果任务已暂停返回True，否则返回False
        """
        return self._state == ClusterTaskState.PAUSED

    def _cancel_active_goal(self, usv_id):
        """
        取消指定 USV 当前活动的导航任务 (Topic 版本)
        """
        # 清理目标缓存 (USV 端会自动超时)
        if usv_id in self.node._usv_nav_target_cache:
            self.node.get_logger().warn(f"⚠️  清除 {usv_id} 导航目标缓存...")
            del self.node._usv_nav_target_cache[usv_id]

    def mark_usv_goal_result(self, usv_id: str, success: bool, goal_step: Optional[int] = None) -> None:
        """根据导航结果更新指定 USV 的 ack 状态。"""
        self.node.get_logger().info(
            f"🔍 [DEBUG] mark_usv_goal_result 被调用: usv_id={usv_id}, success={success}, goal_step={goal_step}, run_step={self.node.run_step}"
        )
        
        state = self._ack_states.get(usv_id)
        self.node.get_logger().info(
            f"🔍 [DEBUG] state查询结果: state={state}, state.step={state.step if state else 'N/A'}, state.acked={state.acked if state else 'N/A'}"
        )
        
        # 如果提供了 goal_step，使用它来匹配；否则使用当前 run_step
        # 允许 goal_step 等于 state.step 或 state.step+1（任务可能已进入下一步）
        if state is None:
            self.node.get_logger().warning(
                f"⚠️  {usv_id} state为None，无法更新确认状态"
            )
            return
        
        expected_step = goal_step if goal_step is not None else self.node.run_step
        if state.step != expected_step and state.step != expected_step - 1:
            self.node.get_logger().warning(
                f"⚠️  {usv_id} step不匹配! state.step={state.step}, expected_step={expected_step}, run_step={self.node.run_step}"
            )
            return

        if success:
            if not state.acked:
                state.acked = True
                state.ack_time = self._now()
                self.node.get_logger().info(f"✅ {usv_id} 标记为已确认 (step={state.step})")
                self._emit_current_progress()
        else:
            # 失败情况下保持未确认状态，等待重试或人工处理
            state.last_send_time = self._now()
            self.node.get_logger().warning(f"❌ {usv_id} 导航失败，保持未确认状态")
            self._emit_current_progress()

    def mark_usv_goal_ack(self, usv_id: str, accepted: bool, goal_step: Optional[int] = None) -> None:
        """根据 navigation/ack 更新指定 USV 的 received 状态（用于停止 step_timeout 级别的重发）。"""
        state = self._ack_states.get(usv_id)
        if state is None:
            return

        expected_step = goal_step if goal_step is not None else self.node.run_step
        if state.step != expected_step and state.step != expected_step - 1:
            return

        if accepted:
            if not state.received:
                state.received = True
                state.received_time = self._now()
        else:
            # 未接受：保持 received=False 以便继续重发
            state.received = False

    def exclude_usv(self, usv_id: str) -> None:
        """从集群任务中排除指定USV。"""
        self._excluded_ids.add(usv_id)
        # 清除 ACK 状态
        if usv_id in self._ack_states:
            del self._ack_states[usv_id]
        self.node.get_logger().info(f"🚫 {usv_id} 已从集群任务中排除")

    def include_usv(self, usv_id: str) -> None:
        """重新将指定USV纳入集群任务。"""
        self._excluded_ids.discard(usv_id)
        # 注意：重新纳入后，如果在任务进行中，可能需要等待下一个步骤或手动干预
        self.node.get_logger().info(f"✅ {usv_id} 已恢复集群任务资格")


