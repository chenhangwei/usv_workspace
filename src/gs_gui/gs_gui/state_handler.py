#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of state handler.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
状态处理模块
负责处理USV状态的接收、缓存和更新
"""
from PyQt5.QtCore import QTimer


class StateHandler:
    """状态处理器，负责USV状态的接收和缓存管理"""
    
    # 导航状态常量
    NAV_STATUS_UNKNOWN = "未知"
    NAV_STATUS_IDLE = "空闲"
    NAV_STATUS_ACTIVE = "执行中"
    NAV_STATUS_SUCCEEDED = "成功"
    NAV_STATUS_FAILED = "失败"
    
    def __init__(self, table_manager, list_manager, warning_callback, info_panel_update_callback=None, navigation_panel_update_callback=None):
        """
        初始化状态处理器
        
        Args:
            table_manager: 表格管理器
            list_manager: 列表管理器
            warning_callback: 警告输出回调
            info_panel_update_callback: USV信息面板更新回调（可选）
            navigation_panel_update_callback: USV导航面板更新回调（可选）
        """
        self.table_manager = table_manager
        self.list_manager = list_manager
        self.append_warning = warning_callback
        self.info_panel_update_callback = info_panel_update_callback
        self.navigation_panel_update_callback = navigation_panel_update_callback
        
        # USV状态缓存
        self._usv_state_cache = {}
        self._usv_state_dirty = False
        
        # 存储USV导航状态
        self.usv_nav_status = {}
        
        # 存储USV导航反馈数据
        self._usv_navigation_feedback_cache = {}
        
        # 使用 QTimer 在 GUI 线程周期性刷新 UI
        self._ui_refresh_timer = QTimer()
        # 500ms (2Hz) 足够展示艇状态，同时避免多艇运行时频繁重绘拖满 GUI 线程。
        self._ui_refresh_timer.setInterval(500)
        self._ui_refresh_timer.timeout.connect(self._flush_state_cache_to_ui)
        self._ui_refresh_timer.start()
    
    def receive_state_callback(self, msg):
        """
        接收并处理所有在线USV的状态信息
        
        Args:
            msg (list): 包含USV状态信息的字典列表
        """
        # 快速入缓存并返回
        if not isinstance(msg, list):
            self.append_warning("接收到的数据类型错误，期望为列表")
            return
        
        # 将来自后端的列表视为"全量快照"，用其覆盖本地缓存
        new_cache = {}
        for ns in msg:
            if isinstance(ns, dict) and ns.get('namespace'):
                new_cache[ns.get('namespace')] = ns
        self._usv_state_cache = new_cache
        
        # 标记需要刷新 UI
        self._usv_state_dirty = True
    
    def _flush_state_cache_to_ui(self):
        """
        将缓存中的 USV 状态批量刷新到 UI（在 GUI 线程中由 QTimer 周期性调用）
        """
        # 如果没有新的数据，不执行任何操作
        if not self._usv_state_dirty:
            return
        
        try:
            # 用缓存构建在线列表（包含所有 USV，无论在线还是离线）
            # 在 list_manager 和 table_manager 中会根据 connected 状态进行显示控制
            online_list = list(self._usv_state_cache.values())
            
            # 更新列表管理器的在线列表
            self.list_manager.update_online_list(online_list)
            
            # 批量更新表格 UI
            self.table_manager.update_cluster_table(
                self.list_manager.usv_cluster_list,
                self.usv_nav_status
            )
            self.table_manager.update_departed_table(
                self.list_manager.usv_departed_list,
                self.usv_nav_status
            )
            
            # 🔥 新增：刷新选中的 USV 信息面板
            if self.info_panel_update_callback:
                try:
                    self.info_panel_update_callback()
                except Exception as e:
                    self.append_warning(f"更新USV信息面板时出错: {e}")
            
            # 🔥 新增：刷新选中的 USV 导航面板
            if self.navigation_panel_update_callback:
                try:
                    self.navigation_panel_update_callback()
                except Exception as e:
                    self.append_warning(f"更新USV导航面板时出错: {e}")
        
        except Exception as e:
            try:
                self.append_warning(f"刷新 UI 时出错: {e}")
            except Exception:
                pass
        finally:
            # 清除脏标志
            self._usv_state_dirty = False
    
    def update_nav_status(self, usv_id, status):
        """
        更新USV的导航状态
        
        Args:
            usv_id (str): USV标识符
            status (str): 导航状态
        """
        # 更新导航状态字典
        self.usv_nav_status[usv_id] = status
        
        # 标记需要更新
        self._usv_state_dirty = True
    
    def get_usv_state(self, usv_id):
        """
        获取指定USV的状态
        
        Args:
            usv_id: USV ID
            
        Returns:
            dict: USV状态字典，如果不存在则返回None
        """
        return self._usv_state_cache.get(usv_id)
    
    def update_navigation_feedback(self, usv_id, feedback):
        """
        更新USV的导航反馈数据
        
        Args:
            usv_id (str): USV标识符
            feedback: 导航反馈数据对象
        """
        # 更新导航反馈缓存
        self._usv_navigation_feedback_cache[usv_id] = feedback
        
        # 标记需要更新
        self._usv_state_dirty = True
    
    def get_usv_navigation_feedback(self, usv_id):
        """
        获取指定USV的导航反馈数据
        
        Args:
            usv_id: USV ID
            
        Returns:
            导航反馈数据对象，如果不存在则返回None
        """
        return self._usv_navigation_feedback_cache.get(usv_id)
    
    def stop_refresh_timer(self):
        """停止刷新定时器"""
        if self._ui_refresh_timer:
            self._ui_refresh_timer.stop()
