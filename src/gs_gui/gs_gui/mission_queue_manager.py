#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
#
# This file is part of the USV Workspace project.
#
# Implementation of mission queue manager for multi-task navigation.
#
# Author: chenhangwei
# Date: 2026-03-09
"""
多任务导航队列管理模块

在现有单任务集群导航基础上，新增多任务队列管理层，支持：
- 加载多个 XML 任务文件到有序队列
- 顺序连续执行
- 拖拽排序 / 上下移动调整执行顺序
- 可配置的任务间过渡策略（无缝衔接 / 等待确认）
- JSON 持久化保存与恢复
"""

import json
import math
import os
import uuid
import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import List, Optional, Callable


class TaskStatus(Enum):
    """任务状态枚举"""
    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"


class TransitionMode(Enum):
    """任务间过渡模式枚举"""
    SEAMLESS = "seamless"           # 无缝衔接：自动开始下一个任务
    WAIT_CONFIRM = "wait_confirm"   # 等待确认：暂停等待操作员确认


@dataclass
class MissionTask:
    """单个导航任务的数据模型"""
    task_id: str
    task_name: str
    file_path: str
    position_list: list = field(default_factory=list)
    status: str = TaskStatus.PENDING.value
    transition_mode: str = TransitionMode.SEAMLESS.value
    total_steps: int = 0
    current_step: int = 0
    added_time: float = 0.0
    completed_time: Optional[float] = None
    start_time: Optional[float] = None
    # 预计算指标
    usv_count: int = 0
    estimated_distance: float = 0.0
    nav_modes_str: str = ""
    speed_range_str: str = ""
    source_file: str = ""

    def to_dict(self):
        """序列化为可 JSON 存储的字典（不含运行时大数据）"""
        return {
            'task_id': self.task_id,
            'task_name': self.task_name,
            'file_path': self.file_path,
            'transition_mode': self.transition_mode,
            'status': self.status,
        }


# 默认持久化路径
DEFAULT_QUEUE_DIR = os.path.expanduser("~/.usv_gs")
DEFAULT_QUEUE_FILE = os.path.join(DEFAULT_QUEUE_DIR, "mission_queue.json")

# 导航模式名称映射
NAV_MODE_NAMES = {0: 'ASYNC', 1: 'SYNC', 2: 'ROTATE', 3: 'TERMINAL'}


def _compute_task_metrics(position_list: list, file_path: str) -> dict:
    """
    从 position_list 中提取任务指标。

    Returns:
        dict with keys: usv_count, estimated_distance, nav_modes_str, speed_range_str, source_file
    """
    if not position_list:
        return {'usv_count': 0, 'estimated_distance': 0.0,
                'nav_modes_str': '', 'speed_range_str': '', 'source_file': ''}

    # 参与 USV 数量
    usv_ids = {p.get('usv_id') for p in position_list if p.get('usv_id')}
    usv_count = len(usv_ids)

    # 预估航程：按每个 USV 计算步骤间欧氏距离之和,取最大值
    usv_positions = {}  # usv_id -> [(step, x, y)]
    for p in position_list:
        uid = p.get('usv_id', '')
        pos = p.get('position', {})
        usv_positions.setdefault(uid, []).append(
            (p.get('step', 0), pos.get('x', 0.0), pos.get('y', 0.0))
        )
    max_dist = 0.0
    for uid, waypoints in usv_positions.items():
        waypoints.sort(key=lambda w: w[0])
        dist = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i][1] - waypoints[i - 1][1]
            dy = waypoints[i][2] - waypoints[i - 1][2]
            dist += math.sqrt(dx * dx + dy * dy)
        max_dist = max(max_dist, dist)

    # 导航模式
    nav_modes_set = set()
    for p in position_list:
        mode = p.get('nav_mode', 0)
        nav_modes_set.add(NAV_MODE_NAMES.get(mode, str(mode)))
    nav_modes_str = '/'.join(sorted(nav_modes_set))

    # 速度范围
    velocities = [p.get('velocity', 0.0) for p in position_list if p.get('velocity', 0.0) > 0]
    if velocities:
        v_min, v_max = min(velocities), max(velocities)
        if abs(v_min - v_max) < 0.01:
            speed_range_str = f"{v_min:.1f}"
        else:
            speed_range_str = f"{v_min:.1f}~{v_max:.1f}"
    else:
        speed_range_str = "-"

    # 来源文件名
    source_file = os.path.basename(file_path) if file_path else ''

    return {
        'usv_count': usv_count,
        'estimated_distance': round(max_dist, 1),
        'nav_modes_str': nav_modes_str,
        'speed_range_str': speed_range_str,
        'source_file': source_file,
    }


class MissionQueueManager:
    """
    多任务导航队列管理器

    包装现有 ClusterTaskManager，实现多任务队列的管理与顺序执行。
    不修改 ClusterController / navigate_to_point_node 的核心逻辑。
    """

    def __init__(self, ros_signal, task_manager, info_callback=None, warning_callback=None):
        """
        Args:
            ros_signal: ROS信号对象
            task_manager: ClusterTaskManager 实例（复用其 XML 解析能力）
            info_callback: 信息输出回调 (str -> None)
            warning_callback: 警告输出回调 (str -> None)
        """
        self.ros_signal = ros_signal
        self.task_manager = task_manager
        self.append_info = info_callback or (lambda x: None)
        self.append_warning = warning_callback or (lambda x: None)

        # 任务队列（有序列表）
        self._queue: List[MissionTask] = []
        # 当前执行中的任务索引（-1 表示未执行）
        self._current_index: int = -1
        # 队列是否处于运行状态
        self._queue_running: bool = False
        # 是否等待操作员确认下一个任务
        self._waiting_confirm: bool = False
        # 离群 USV 列表（执行时需要过滤）
        self._departed_list: list = []
        # 失败时策略: True = 跳过继续, False = 暂停队列
        self.skip_on_failure: bool = True

        # 任务完成回调（供 ClusterController 通知）
        self._on_single_task_completed: Optional[Callable] = None
        # 预览路径更新回调（切换任务时更新反馈窗口预览）
        self._preview_callback: Optional[Callable] = None

        # 连接进度更新信号
        self.ros_signal.cluster_progress_update.connect(self._on_cluster_progress)

    # =============================================================
    #  回调设置
    # =============================================================

    def set_preview_callback(self, callback: Callable):
        """设置预览路径更新回调（任务切换时更新反馈窗口预览）"""
        self._preview_callback = callback

    # =============================================================
    #  队列管理
    # =============================================================

    def add_task_from_file(self, file_path: str) -> Optional[MissionTask]:
        """
        从 XML 文件解析并添加任务到队列尾部。

        Args:
            file_path: XML 文件路径

        Returns:
            添加成功返回 MissionTask，失败返回 None
        """
        position_list = self.task_manager.parse_file(file_path)
        if not position_list:
            self.append_warning(f"解析文件失败或为空: {os.path.basename(file_path)}")
            return None

        task_name = os.path.splitext(os.path.basename(file_path))[0]
        unique_steps = sorted({item.get('step', 0) for item in position_list})

        task = MissionTask(
            task_id=str(uuid.uuid4())[:8],
            task_name=task_name,
            file_path=file_path,
            position_list=position_list,
            total_steps=len(unique_steps),
            added_time=time.time(),
            source_file=os.path.basename(file_path),
        )
        # 计算预览指标
        metrics = _compute_task_metrics(position_list, file_path)
        task.usv_count = metrics['usv_count']
        task.estimated_distance = metrics['estimated_distance']
        task.nav_modes_str = metrics['nav_modes_str']
        task.speed_range_str = metrics['speed_range_str']

        self._queue.append(task)
        self.append_info(f"任务已添加: {task_name} ({task.total_steps} 步)")
        self._emit_queue_updated()
        self._auto_save()
        return task

    def remove_task(self, task_id: str) -> bool:
        """移除指定任务（不能移除正在运行的任务）"""
        for i, t in enumerate(self._queue):
            if t.task_id == task_id:
                if t.status == TaskStatus.RUNNING.value:
                    self.append_warning("无法移除正在运行的任务")
                    return False
                self._queue.pop(i)
                # 调整当前索引
                if self._current_index >= i and self._current_index > 0:
                    self._current_index -= 1
                self._emit_queue_updated()
                self._auto_save()
                return True
        return False

    def clear_completed(self):
        """清除所有已完成/失败的任务"""
        before = len(self._queue)
        self._queue = [t for t in self._queue
                       if t.status not in (TaskStatus.COMPLETED.value, TaskStatus.FAILED.value)]
        # 重新计算当前索引
        self._recalc_current_index()
        removed_count = before - len(self._queue)
        if removed_count > 0:
            self.append_info(f"已清除 {removed_count} 个已完成/失败的任务")
            self._emit_queue_updated()
            self._auto_save()

    def reorder(self, new_order: List[str]):
        """
        按给定的 task_id 列表重排队列顺序。

        Args:
            new_order: task_id 有序列表
        """
        id_map = {t.task_id: t for t in self._queue}
        reordered = []
        for tid in new_order:
            if tid in id_map:
                reordered.append(id_map[tid])
        # 保留 new_order 中未包含的任务（安全措施）
        for t in self._queue:
            if t.task_id not in {r.task_id for r in reordered}:
                reordered.append(t)
        self._queue = reordered
        self._recalc_current_index()
        self._emit_queue_updated()
        self._auto_save()

    def move_task(self, task_id: str, direction: int):
        """
        移动任务位置。

        Args:
            task_id: 任务 ID
            direction: -1 表示上移, +1 表示下移
        """
        for i, t in enumerate(self._queue):
            if t.task_id == task_id:
                new_pos = i + direction
                if 0 <= new_pos < len(self._queue):
                    self._queue[i], self._queue[new_pos] = self._queue[new_pos], self._queue[i]
                    self._recalc_current_index()
                    self._emit_queue_updated()
                    self._auto_save()
                return

    def set_transition_mode(self, task_id: str, mode: TransitionMode):
        """设置指定任务的过渡模式"""
        for t in self._queue:
            if t.task_id == task_id:
                t.transition_mode = mode.value
                self._emit_queue_updated()
                self._auto_save()
                return

    def rename_task(self, task_id: str, new_name: str):
        """重命名任务"""
        for t in self._queue:
            if t.task_id == task_id:
                t.task_name = new_name
                self._emit_queue_updated()
                self._auto_save()
                return

    def get_queue(self) -> List[MissionTask]:
        """获取当前队列的浅拷贝"""
        return list(self._queue)

    def get_current_task(self) -> Optional[MissionTask]:
        """获取当前执行中的任务"""
        if 0 <= self._current_index < len(self._queue):
            return self._queue[self._current_index]
        return None

    @property
    def is_running(self) -> bool:
        return self._queue_running

    @property
    def is_waiting_confirm(self) -> bool:
        return self._waiting_confirm

    # =============================================================
    #  执行控制
    # =============================================================

    def start_queue(self, departed_list: list = None):
        """
        开始执行任务队列（从第一个 PENDING 任务开始）。

        Args:
            departed_list: 离群 USV 列表
        """
        if departed_list is not None:
            self._departed_list = departed_list

        if self._queue_running:
            self.append_warning("任务队列已在运行中")
            return

        # 找到第一个 PENDING 的任务
        start_idx = self._find_next_pending()
        if start_idx is None:
            self.append_warning("队列中没有待执行的任务")
            return

        self._queue_running = True
        self._waiting_confirm = False
        self._current_index = start_idx
        self._start_task_at_index(start_idx)

    def pause_queue(self):
        """暂停当前任务（保持队列运行状态）"""
        if not self._queue_running:
            return
        current = self.get_current_task()
        if current and current.status == TaskStatus.RUNNING.value:
            current.status = TaskStatus.PAUSED.value
            self.ros_signal.cluster_pause_request.emit()
            self.append_info(f"任务暂停: {current.task_name}")
            self._emit_queue_updated()

    def resume_queue(self):
        """恢复暂停的任务"""
        if not self._queue_running:
            return
        current = self.get_current_task()
        if current and current.status == TaskStatus.PAUSED.value:
            current.status = TaskStatus.RUNNING.value
            self.ros_signal.cluster_resume_request.emit()
            self.append_info(f"任务恢复: {current.task_name}")
            self._emit_queue_updated()

    def stop_queue(self):
        """停止队列执行并取消当前任务"""
        if not self._queue_running and not self._waiting_confirm:
            return
        # 停止当前正在执行的任务
        current = self.get_current_task()
        if current and current.status in (TaskStatus.RUNNING.value, TaskStatus.PAUSED.value):
            self.ros_signal.cluster_stop_request.emit()
            current.status = TaskStatus.PENDING.value  # 重置为 PENDING，允许重新执行

        self._queue_running = False
        self._waiting_confirm = False

        # 显式重置 task_manager 状态，确保主界面恢复
        self.task_manager.cluster_task_running = False
        self.task_manager.cluster_task_paused = False

        # 更新状态标签为停止
        label_text = "⏹ 队列已停止"
        style_css = self.task_manager.STATUS_STYLES.get('stopped')
        self.task_manager._update_status(label_text, style_css)

        self.append_info("任务队列已停止")
        self._emit_queue_updated()
        self._emit_queue_progress()

    def skip_current(self):
        """跳过当前任务，执行下一个"""
        if not self._queue_running:
            return
        current = self.get_current_task()
        if current:
            # 停止当前任务
            self.ros_signal.cluster_stop_request.emit()
            current.status = TaskStatus.COMPLETED.value
            current.completed_time = time.time()
            self.append_info(f"已跳过任务: {current.task_name}")
        self._advance_to_next()

    def confirm_next(self):
        """操作员确认开始下一个任务（用于 WAIT_CONFIRM 过渡模式）"""
        if not self._waiting_confirm:
            return
        self._waiting_confirm = False
        self._advance_to_next()

    # =============================================================
    #  持久化
    # =============================================================

    def save_queue(self, file_path: str = None, queue_name: str = None):
        """保存队列到 JSON 文件"""
        file_path = file_path or DEFAULT_QUEUE_FILE
        dir_name = os.path.dirname(file_path)
        if dir_name:
            os.makedirs(dir_name, exist_ok=True)

        data = {
            'version': 1,
            'queue_name': queue_name or '',
            'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'tasks': [t.to_dict() for t in self._queue],
        }
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        display_name = queue_name or os.path.basename(file_path)
        self.append_info(f"任务队列 [{display_name}] 已保存 ({len(self._queue)} 个任务)")

    def load_queue(self, file_path: str = None):
        """从 JSON 文件加载队列"""
        file_path = file_path or DEFAULT_QUEUE_FILE
        if not os.path.isfile(file_path):
            self.append_info("没有找到已保存的任务队列")
            return

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            self.append_warning(f"加载队列文件失败: {e}")
            return

        tasks_data = data.get('tasks', [])
        loaded_count = 0
        for td in tasks_data:
            fp = td.get('file_path', '')
            if not os.path.isfile(fp):
                self.append_warning(f"跳过: 文件不存在 {os.path.basename(fp)}")
                continue
            task = self.add_task_from_file(fp)
            if task:
                # 恢复过渡模式
                mode_str = td.get('transition_mode', TransitionMode.SEAMLESS.value)
                task.transition_mode = mode_str
                # 恢复任务名（可能用户重命名过）
                saved_name = td.get('task_name')
                if saved_name:
                    task.task_name = saved_name
                loaded_count += 1

        self.append_info(f"已加载 {loaded_count}/{len(tasks_data)} 个任务")
        self._emit_queue_updated()

    # =============================================================
    #  内部方法
    # =============================================================

    def _auto_save(self):
        """队列变更时自动保存"""
        try:
            self.save_queue()
        except Exception:
            pass  # 自动保存失败不影响运行

    def _find_next_pending(self, start_from: int = 0) -> Optional[int]:
        """找到从 start_from 开始的第一个 PENDING 任务的索引"""
        for i in range(start_from, len(self._queue)):
            if self._queue[i].status == TaskStatus.PENDING.value:
                return i
        return None

    def _start_task_at_index(self, index: int):
        """开始执行指定索引的任务"""
        if index < 0 or index >= len(self._queue):
            self._complete_queue()
            return

        task = self._queue[index]
        self._current_index = index

        # 设置 ClusterTaskManager 的数据并启动
        self.task_manager.cluster_position_list = task.position_list.copy()
        task.status = TaskStatus.RUNNING.value
        task.current_step = 0
        task.start_time = time.time()

        # 标记为运行中并直接发送 ROS 信号（跳过 GUI 弹窗确认）
        self.task_manager.cluster_task_running = True
        self.task_manager.cluster_task_paused = False

        # 注入 task_name 到每个位置字典，便于日志记录区分不同任务
        position_list_with_name = []
        for p in task.position_list:
            p_copy = dict(p)
            p_copy['task_name'] = task.task_name
            position_list_with_name.append(p_copy)
        self.ros_signal.cluster_target_point_command.emit(position_list_with_name)

        # 更新反馈窗口预览路径（修复切换任务时预览不更新）
        if self._preview_callback:
            self._preview_callback(task.position_list)

        # 更新集群状态标签（修复队列运行时状态冻结）
        label_text = f"🚀 运行中: 任务 {index + 1}/{len(self._queue)} - {task.task_name}"
        style_css = self.task_manager.STATUS_STYLES.get('running')
        self.task_manager._update_status(label_text, style_css)

        self.append_info(f"▶ 开始任务 [{index + 1}/{len(self._queue)}]: {task.task_name}")
        self._emit_queue_updated()
        self._emit_queue_progress()

    def _advance_to_next(self):
        """推进到下一个任务"""
        next_idx = self._find_next_pending(self._current_index + 1)
        if next_idx is not None:
            self._start_task_at_index(next_idx)
        else:
            self._complete_queue()

    def _complete_queue(self):
        """所有任务执行完毕"""
        self._queue_running = False
        self._waiting_confirm = False
        self._current_index = -1

        # 显式重置 task_manager 状态，确保主界面恢复
        self.task_manager.cluster_task_running = False
        self.task_manager.cluster_task_paused = False
        self.task_manager.cluster_position_list = []

        # 更新状态标签为完成
        completed_count = sum(1 for t in self._queue if t.status == TaskStatus.COMPLETED.value)
        label_text = f"🏁 队列完成: {completed_count}/{len(self._queue)} 个任务"
        style_css = self.task_manager.STATUS_STYLES.get('completed')
        self.task_manager._update_status(label_text, style_css)

        self.append_info("🏁 任务队列全部完成")
        self._emit_queue_updated()
        self._emit_queue_progress()

    def _recalc_current_index(self):
        """在队列顺序变更后，重新计算当前执行任务的索引"""
        if not self._queue_running:
            self._current_index = -1
            return
        for i, t in enumerate(self._queue):
            if t.status == TaskStatus.RUNNING.value:
                self._current_index = i
                return
        self._current_index = -1

    def _on_cluster_progress(self, progress_info: dict):
        """
        监听 ClusterController 的进度更新信号。
        检测当前任务是否已完成，若完成则触发过渡逻辑。
        """
        if not self._queue_running:
            return

        current = self.get_current_task()
        if not current or current.status != TaskStatus.RUNNING.value:
            return

        state = progress_info.get('state', '')
        current.current_step = progress_info.get('current_step', 0)

        if state == 'completed':
            current.status = TaskStatus.COMPLETED.value
            current.completed_time = time.time()
            self.append_info(f"✅ 任务完成: {current.task_name}")

            # 更新 task_manager 状态
            self.task_manager.cluster_task_running = False
            self.task_manager.cluster_task_paused = False

            # 根据过渡模式决定下一步
            if current.transition_mode == TransitionMode.WAIT_CONFIRM.value:
                self._waiting_confirm = True
                self.append_info("⏳ 等待操作员确认以开始下一个任务...")
                self._emit_queue_updated()
                self._emit_queue_progress()
            else:
                self._advance_to_next()

        elif state == 'idle' and current.status == TaskStatus.RUNNING.value:
            # ClusterController 意外进入 IDLE（可能是手动停止）
            # 标记为失败
            current.status = TaskStatus.FAILED.value
            current.completed_time = time.time()
            self.append_warning(f"⚠ 任务异常中止: {current.task_name}")

            self.task_manager.cluster_task_running = False
            self.task_manager.cluster_task_paused = False

            if self.skip_on_failure:
                self._advance_to_next()
            else:
                self._queue_running = False
                self.append_warning("队列已暂停（任务失败）")
                self._emit_queue_updated()
                self._emit_queue_progress()

    def _emit_queue_updated(self):
        """发送队列内容更新信号"""
        if hasattr(self.ros_signal, 'mission_queue_updated'):
            now = time.time()
            queue_data = []
            for t in self._queue:
                # 计算耗时
                if t.status == TaskStatus.RUNNING.value and t.start_time:
                    elapsed = now - t.start_time
                elif t.status == TaskStatus.COMPLETED.value and t.start_time and t.completed_time:
                    elapsed = t.completed_time - t.start_time
                else:
                    elapsed = 0.0
                queue_data.append({
                    'task_id': t.task_id,
                    'task_name': t.task_name,
                    'status': t.status,
                    'transition_mode': t.transition_mode,
                    'total_steps': t.total_steps,
                    'current_step': t.current_step,
                    'file_path': t.file_path,
                    'usv_count': t.usv_count,
                    'estimated_distance': t.estimated_distance,
                    'nav_modes_str': t.nav_modes_str,
                    'speed_range_str': t.speed_range_str,
                    'source_file': t.source_file,
                    'elapsed': elapsed,
                })
            self.ros_signal.mission_queue_updated.emit(queue_data)

    def _emit_queue_progress(self):
        """发送整体队列进度信号"""
        if hasattr(self.ros_signal, 'mission_queue_progress'):
            total = len(self._queue)
            completed = sum(1 for t in self._queue if t.status == TaskStatus.COMPLETED.value)
            current = self.get_current_task()
            progress = {
                'total_tasks': total,
                'completed_tasks': completed,
                'current_task_index': self._current_index + 1 if self._current_index >= 0 else 0,
                'current_task_name': current.task_name if current else '',
                'current_task_step': current.current_step if current else 0,
                'current_task_total_steps': current.total_steps if current else 0,
                'queue_running': self._queue_running,
                'waiting_confirm': self._waiting_confirm,
            }
            self.ros_signal.mission_queue_progress.emit(progress)
