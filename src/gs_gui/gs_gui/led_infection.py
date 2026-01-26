#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of led infection.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
LED感染处理器模块
处理LED感染逻辑
"""

class LedInfectionHandler:
    def __init__(self, node):
        self.node = node
        # LED感染相关变量在主节点中管理

    def check_usv_infect(self):
        """
        检查USV之间的LED传染逻辑
        传染模式下，被传染的USV会实时跟随传染源的LED颜色变化
        """
        # 获取USV状态列表
        usv_list = list(self.node.usv_states.values())
        # 获取USV数量
        n = len(usv_list)
        # 如果USV数量少于2个，直接返回
        if n < 2:
            return

        # 创建传染对集合
        infect_pairs = set()
        # 检查所有USV对之间的距离
        for i in range(n):
            for j in range(i + 1, n):
                # 获取两个USV的状态
                usv_a = usv_list[i]
                usv_b = usv_list[j]
                # 获取USV ID
                id_a = usv_a['namespace']
                id_b = usv_b['namespace']

                # 计算两个USV之间的距离平方
                distance_squared = self._calculate_distance_squared(usv_a, usv_b)
                # 如果距离小于传染距离
                if distance_squared <= self.node.INFECTION_DISTANCE_SQUARED:
                    # 确定源和目标USV（按ID排序）
                    src_id, dst_id, src_color, src_mode = self._determine_infection_source(
                        id_a, id_b)
                    # 添加到传染对集合
                    infect_pairs.add((src_id, dst_id))
                    self.node._usv_infection_sources[dst_id] = src_id
                    self.node._usv_infecting.add(dst_id)

                    # 记录目标USV的原始LED模式（仅在首次传染时记录）
                    self._record_original_led_mode(dst_id)

                    # 发送传染指令（实时跟随传染源的颜色）
                    self._send_infection_command(dst_id, src_color)

        # 对每个传染源立即同步一次颜色，避免等待下一轮状态上报
        if infect_pairs:
            for src_id in {pair[0] for pair in infect_pairs}:
                self.propagate_color_update(src_id)

        # 恢复离开传染范围的USV
        self._restore_led_modes(infect_pairs)

    def _calculate_distance_squared(self, usv_a, usv_b):
        """
        计算两个USV之间的距离平方
        
        Args:
            usv_a (dict): 第一个USV的状态
            usv_b (dict): 第二个USV的状态
            
        Returns:
            float: 距离平方，如果无法计算则返回无穷大
        """
        # 获取位置信息
        pos_a = usv_a.get('position', {})
        pos_b = usv_b.get('position', {})
        try:
            # 提取坐标值
            xa, ya = float(pos_a.get('x', 0)), float(pos_a.get('y', 0))
            xb, yb = float(pos_b.get('x', 0)), float(pos_b.get('y', 0))
            # 计算距离平方
            return (xa - xb) ** 2 + (ya - yb) ** 2
        # 捕获异常并返回无穷大
        except Exception:
            return float('inf')

    def _determine_infection_source(self, id_a, id_b):
        """
        确定两个USV之间的LED传染源和目标。
        传染源基于 USV ID 的字符串排序决定（ID靠前为源）。
        """
        # 从本地状态字典中获取 LED 状态
        state_a = self.node._usv_current_led_state.get(
            id_a, {'mode': 'color_switching', 'color': [255, 0, 0]})
        state_b = self.node._usv_current_led_state.get(
            id_b, {'mode': 'color_switching', 'color': [255, 0, 0]})

        # 以编号字符串排序，靠前为主
        if id_a < id_b:
            src_id, dst_id = id_a, id_b
            src_color = list(state_a.get('color', [255, 0, 0]))
            src_mode = state_a.get('mode', 'color_switching')
        else:
            src_id, dst_id = id_b, id_a
            src_color = list(state_b.get('color', [255, 0, 0]))
            src_mode = state_b.get('mode', 'color_switching')

        # 返回传染源信息
        return src_id, dst_id, src_color, src_mode

    def _record_original_led_mode(self, dst_id):
        """
        在开始传染前，记录目标 USV 的原始 LED 模式和颜色。
        """
        # 从本地状态字典中获取原始 LED 状态 ---
        if dst_id not in self.node._usv_led_modes:
            # 始终使用本地维护的状态作为原始状态
            original_state = self.node._usv_current_led_state.get(
                dst_id, {'mode': 'color_switching', 'color': [255, 0, 0]})
            dst_led_mode = original_state.get('mode', 'color_switching')
            dst_led_color = list(original_state.get('color', [255, 0, 0]))

            self.node._usv_led_modes[dst_id] = (dst_led_mode, dst_led_color)

    def _send_infection_command(self, dst_id, src_color):
        """
        发送LED传染命令
        """
        if not isinstance(src_color, (list, tuple)) or len(src_color) < 3:
            return

        sanitized = [max(0, min(255, int(c))) for c in src_color[:3]]

        # 构造传染命令
        infect_cmd = f"color_infect|{sanitized[0]},{sanitized[1]},{sanitized[2]}"
        # 检查目标USV是否存在对应的LED发布者
        if dst_id in self.node.usv_manager.led_pubs:
            # 创建消息
            from std_msgs.msg import String
            msg = String()
            msg.data = infect_cmd
            # 将消息添加到发布队列
            self.node.publish_queue.put((self.node.usv_manager.led_pubs[dst_id], msg))

    def _restore_led_modes(self, infect_pairs):
        """
        恢复离开传染范围的USV的LED模式
        """
        # 遍历LED模式字典
        for dst_id in list(self.node._usv_led_modes.keys()):
            # 检查目标USV是否在传染对中
            if not any(dst_id == pair[1] for pair in infect_pairs):
                # 获取原始LED模式和颜色
                mode, color = self.node._usv_led_modes[dst_id]
                # 检查目标USV是否存在对应的LED发布者
                if dst_id in self.node.usv_manager.led_pubs:
                    # 根据模式构造命令
                    if mode == 'color_select':
                        cmd = f"color_select|{color[0]},{color[1]},{color[2]}"
                    else:
                        cmd = mode
                    # 创建消息
                    from std_msgs.msg import String
                    msg = String()
                    msg.data = cmd
                    # 将消息添加到发布队列
                    self.node.publish_queue.put((self.node.usv_manager.led_pubs[dst_id], msg))
                # 删除记录的LED模式
                del self.node._usv_led_modes[dst_id]
                self.node._usv_infection_sources.pop(dst_id, None)
                self.node._usv_infecting.discard(dst_id)

    def propagate_color_update(self, src_id):
        """当传染源颜色变化时，同步更新所有被传染的USV。"""
        state = self.node._usv_current_led_state.get(src_id)
        if not state:
            return

        color = state.get('color')
        if not isinstance(color, (list, tuple)) or len(color) < 3:
            return

        sanitized = [max(0, min(255, int(c))) for c in color[:3]]

        for dst_id, mapped_src in list(self.node._usv_infection_sources.items()):
            if mapped_src != src_id:
                continue
            self._record_original_led_mode(dst_id)
            self._send_infection_command(dst_id, sanitized)