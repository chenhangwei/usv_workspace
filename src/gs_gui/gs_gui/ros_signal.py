#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of ros signal.
#
# Author: chenhangwei
# Date: 2026-01-26
from PyQt5.QtCore import pyqtSignal, QObject


class ROSSignal(QObject):
    """
    ROS信号类，用于GUI和ROS节点之间的通信
    
    该类定义了GUI和ROS节点之间通信所需的所有信号。
    信号按照功能分为几类：武装命令、模式切换、目标点设置、状态更新和通用命令。
    """

    # 模式常量定义
    MODE_MANUAL = "manual"          # 手动模式
    MODE_GUIDED = "guided"          # 制导模式
    MODE_ARCO = "arco"              # ARCO模式
    MODE_STEERING = "steering"      # 转向模式

    # 武装/解武装命令信号
    arm_command = pyqtSignal(list)      # 武装命令，参数：USV命名空间列表
    disarm_command = pyqtSignal(list)   # 解除武装命令，参数：USV命名空间列表

    # 模式切换命令信号
    hold_command = pyqtSignal(list)     # HOLD模式命令，参数：USV命名空间列表（集群使用）
    manual_command = pyqtSignal(list)   # 手动模式命令，参数：USV命名空间列表（离群使用）
    guided_command = pyqtSignal(list)   # 制导模式命令，参数：USV命名空间列表
    arco_command = pyqtSignal(list)     # ARCO模式命令，参数：USV命名空间列表
    steering_command = pyqtSignal(list) # 转向模式命令，参数：USV命名空间列表
    rtl_command = pyqtSignal(list)      # RTL模式命令，参数：USV命名空间列表

    # 目标点命令信号
    cluster_target_point_command = pyqtSignal(list)     # 集群目标点命令，参数：目标点列表
    departed_target_point_command = pyqtSignal(list)    # 离群目标点命令，参数：目标点列表
    cluster_pause_request = pyqtSignal()                # 集群暂停请求
    cluster_resume_request = pyqtSignal()               # 集群恢复请求
    cluster_stop_request = pyqtSignal()                 # 集群停止请求

    # 状态更新信号
    receive_state_list = pyqtSignal(list)  # 接收USV状态列表，参数：USV状态字典列表
    cluster_progress_update = pyqtSignal(dict)  # 集群任务进度更新，参数：进度信息字典

    # 添加导航状态更新信号
    nav_status_update = pyqtSignal(str, str)        # 导航状态更新信号，参数：USV ID, 状态字符串
    navigation_feedback = pyqtSignal(str, object)   # 导航反馈信号，参数：USV ID, 反馈数据对象

    # 通用字符串命令信号
    str_command = pyqtSignal(str)          # 通用字符串命令，参数：命令字符串
    # 节点 -> GUI 的信息反馈
    node_info = pyqtSignal(str)            # 参数：信息字符串，供节点向 GUI 发送状态/反馈
    
    # 坐标系偏移量设置信号
    update_area_center = pyqtSignal(dict)  # 更新任务坐标系偏移量，参数：{'x': float, 'y': float, 'z': float}
    
    # LED传染模式控制信号
    
    # Home Position 设置信号
    set_home_position = pyqtSignal(str, bool, dict)  # 参数：USV命名空间, 是否使用当前位置, 坐标字典{'lat':float,'lon':float,'alt':float}
    led_infection_mode_changed = pyqtSignal(bool)  # LED传染模式开关，参数：True开启/False关闭
    
    # 飞控重启命令信号
    reboot_autopilot = pyqtSignal(str)  # 飞控重启命令，参数：USV命名空间
    
    # 机载计算机重启命令信号
    reboot_companion = pyqtSignal(str)  # 机载计算机重启命令，参数：USV命名空间
    
    # USV节点优雅关闭命令信号
    shutdown_usv = pyqtSignal(str)  # USV节点关闭命令，参数：USV命名空间
    
    # GUI警告信号（用于显示错误和警告消息）
    node_warning = pyqtSignal(str)  # 参数：警告字符串

    # ==================== 编队模式信号 ====================
    formation_start_request = pyqtSignal(list)     # 编队启动请求，参数：编队组配置列表
    formation_stop_request = pyqtSignal()           # 编队停止请求
    formation_status_update = pyqtSignal(dict)      # 编队状态更新，参数：编队状态信息字典
    formation_type_change = pyqtSignal(int)         # 编队队形切换，参数：FormationType 整数值
    formation_spacing_change = pyqtSignal(float, float)  # 编队间距变更，参数：(along, cross)




