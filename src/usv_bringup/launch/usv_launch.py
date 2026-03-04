#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Launch script for usv_launch.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
无人船(Ultra Short Wave Vehicle, USV)启动文件
该文件用于启动完整的USV系统，包括飞控通信、传感器驱动、控制逻辑等模块

用法:
    # 普通启动
    ros2 launch usv_bringup usv_launch.py
    
    # 指定命名空间
    ros2 launch usv_bringup usv_launch.py namespace:=usv_01
    
    # 启用导航日志收集（用于调试）
    ros2 launch usv_bringup usv_launch.py enable_log_collector:=true
    
    # 组合使用
    ros2 launch usv_bringup usv_launch.py namespace:=usv_01 enable_log_collector:=true

日志文件保存位置: ~/usv_logs/nav_log_YYYYMMDD_HHMMSS.csv
分析脚本: python3 usv_control/scripts/analyze_nav_log.py
"""

import os
import socket
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution


# =============================================================================
# USV 自动识别配置表
# =============================================================================
# 每艘 USV 的固定参数映射表，新增 USV 时只需在此处添加一行
#
# ┌────────┬────────────┬───────────────────────────────────────────────────────┬────────────┬──────────────────┐
# │ USV ID │ namespace  │ gcs_url                                               │ system_id  │ target_system_id │
# ├────────┼────────────┼───────────────────────────────────────────────────────┼────────────┼──────────────────┤
# │   1    │ usv_01     │ udp://192.168.68.55:14551@192.168.68.53:14550         │    101     │        1         │
# │   2    │ usv_02     │ udp://192.168.68.54:14552@192.168.68.53:14560         │    102     │        2         │
# │   3    │ usv_03     │ udp://192.168.68.52:14553@192.168.68.53:14570         │    103     │        3         │
# └────────┴────────────┴───────────────────────────────────────────────────────┴────────────┴──────────────────┘
USV_CONFIG = {
    1: {
        'namespace':        'usv_01',
        'local_ip':         '192.168.68.55',
        'gcs_url':          'udp://192.168.68.55:14551@192.168.68.53:14550',
        'system_id':        101,
        'target_system_id': 1,
    },
    2: {
        'namespace':        'usv_02',
        'local_ip':         '192.168.68.54',
        'gcs_url':          'udp://192.168.68.54:14552@192.168.68.53:14560',
        'system_id':        102,
        'target_system_id': 2,
    },
    3: {
        'namespace':        'usv_03',
        'local_ip':         '192.168.68.52',
        'gcs_url':          'udp://192.168.68.52:14553@192.168.68.53:14570',
        'system_id':        103,
        'target_system_id': 3,
    },
}

# IP → USV ID 反向映射 (用于 IP 自动检测)
_IP_TO_USV = {cfg['local_ip']: uid for uid, cfg in USV_CONFIG.items()}


def detect_usv_id() -> int:
    """
    自动检测当前机载计算机对应的 USV ID

    检测优先级：
      1. /etc/usv_id 配置文件 (最可靠，推荐)
      2. ~/.usv_id   配置文件
      3. 主机名匹配  (如 usv-01, usv_01, usv01)
      4. 本机 IP 地址匹配
      5. 默认值 1    (兜底)

    首次部署时，在每台机载计算机上执行一次:
        echo 1 | sudo tee /etc/usv_id   # usv_01 的机载计算机
        echo 2 | sudo tee /etc/usv_id   # usv_02 的机载计算机
        echo 3 | sudo tee /etc/usv_id   # usv_03 的机载计算机
    """
    # ---- 方法 1 & 2: 配置文件 ----
    for path in ['/etc/usv_id', os.path.expanduser('~/.usv_id')]:
        if os.path.isfile(path):
            try:
                with open(path, 'r') as f:
                    usv_id = int(f.read().strip())
                if usv_id in USV_CONFIG:
                    print(f'[USV Auto-Detect] ✅ 从 {path} 读取 USV ID = {usv_id}')
                    return usv_id
                else:
                    print(f'[USV Auto-Detect] ⚠️ {path} 中的 ID={usv_id} 不在配置表中')
            except (ValueError, IOError) as e:
                print(f'[USV Auto-Detect] ⚠️ 读取 {path} 失败: {e}')

    # ---- 方法 3: 主机名 ----
    hostname = socket.gethostname().lower()
    for usv_id in USV_CONFIG:
        patterns = [f'usv-{usv_id:02d}', f'usv_{usv_id:02d}', f'usv{usv_id:02d}',
                    f'usv-{usv_id}', f'usv_{usv_id}', f'usv{usv_id}']
        if any(p in hostname for p in patterns):
            print(f'[USV Auto-Detect] ✅ 从主机名 "{hostname}" 检测 USV ID = {usv_id}')
            return usv_id

    # ---- 方法 4: 本机 IP ----
    try:
        result = subprocess.run(
            ['hostname', '-I'], capture_output=True, text=True, timeout=5
        )
        for ip in result.stdout.strip().split():
            if ip in _IP_TO_USV:
                usv_id = _IP_TO_USV[ip]
                print(f'[USV Auto-Detect] ✅ 从本机 IP {ip} 检测 USV ID = {usv_id}')
                return usv_id
    except Exception:
        pass

    # ---- 兜底 ----
    print('[USV Auto-Detect] ⚠️ 无法自动检测，使用默认 USV ID = 1')
    print('[USV Auto-Detect]    提示: echo <ID> | sudo tee /etc/usv_id')
    return 1


def generate_launch_description():
    """
    生成USV系统启动描述
    
    该函数定义了完整的USV系统启动配置，包括：
    1. 基础参数配置（命名空间、参数文件）
    2. 飞控通信模块（MAVROS）
    3. 状态管理模块
    4. 控制模块
    5. 传感器驱动模块
    6. 辅助功能模块（LED、声音、风扇等）
    
    可配置参数：
    - namespace: 节点命名空间
    - param_file: 参数文件路径
    - fcu_url: 飞控通信串口
    - gcs_url: 地面站通信地址
    - lidar_port: 激光雷达串口路径
    
    Returns:
        LaunchDescription: 包含所有节点和参数的启动描述对象
    """
    # =============================================================================
    # 自动检测 USV ID 并加载对应配置
    # =============================================================================
    usv_id = detect_usv_id()
    cfg = USV_CONFIG[usv_id]
    print(f'[USV Launch] 当前配置: namespace={cfg["namespace"]}, '
          f'system_id={cfg["system_id"]}, target_system_id={cfg["target_system_id"]}')
    print(f'[USV Launch] GCS URL: {cfg["gcs_url"]}')

    # =============================================================================
    # 参数声明
    # =============================================================================

    # 仿真模式参数
    # 'hardware' = 真实硬件, 'sitl' = ArduPilot SITL 仿真, 'simple' = 简单仿真
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='hardware',
        description='运行模式: hardware(真实硬件), sitl(ArduPilot SITL), simple(简单仿真)'
    )

    # 命名空间参数（自动检测，可通过 launch 参数覆盖）
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=cfg['namespace'],
        description='无人船节点的命名空间（自动检测）'
    )

    # SITL 目标系统 ID（多 USV 时每艘不同，匹配 SYSID_THISMAV）
    target_system_id_arg = DeclareLaunchArgument(
        'target_system_id',
        default_value=str(cfg['target_system_id']),
        description='MAVROS 目标飞控系统 ID（SITL 模式，需匹配 SYSID_THISMAV）'
    )

    # SITL 参数文件（用于覆盖 SITL 特有参数）
    sitl_param_file_arg = DeclareLaunchArgument(
        'sitl_param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('usv_bringup'),
            'config',
            'usv_params_sitl.yaml'
        ]),
        description='SITL 仿真模式的参数覆盖文件'
    )
    
    # 日志收集开关参数
    enable_log_collector_arg = DeclareLaunchArgument(
        'enable_log_collector',
        default_value='true',
        description='是否启用导航日志收集 (true/false)'
    )
    
    # 参数文件路径参数
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('usv_bringup'),
            'config',
            'usv_params.yaml'
        ]),
        description='设备站的参数文件路径'
    )

 
    
    # 飞控串口参数
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        #default_value='udp://192.168.10.1:14550@192.168.10.2:14550',
        default_value='serial:///dev/ttyACM0:921600',
        description='飞控通信串口和波特率'
    )
    
    # 地面站通信参数
    # ⚠️ 关键配置：多USV场景下，每个USV必须使用不同的地面站端口以避免MAVLink消息冲突
    # 格式: udp://[本机IP]:[本机监听端口]@[地面站IP]:[地面站接收端口]
    # 
    # 🔌 多USV端口分配方案 (每个USV连接到地面站的不同端口):kongz
    # ┌──────────┬─────────────┬─────────────┬──────────────────┐
    # │   USV    │  本机IP     │ 本机端口    │  地面站端口      │
    # ├──────────┼─────────────┼─────────────┼──────────────────┤
    # │ usv_01   │ .68.55      │ 14551       │ 14550 (QGC端口1) │
    # │ usv_02   │ .68.54      │ 14552       │ 14560 (QGC端口2) │
    # │ usv_03   │ .68.52      │ 14553       │ 14570 (QGC端口3) │ ✅当前
    # └──────────┴─────────────┴─────────────┴──────────────────┘
    # 
    # 📋 地面站QGroundControl配置 (需要添加3个UDP连接):
    #    1. Tools → Application Settings → Comm Links → Add
    #       - Name: USV_01, Type: UDP, Port: 14550
    #    2. Add 第二个: Name: USV_02, Type: UDP, Port: 14560
    #    3. Add 第三个: Name: USV_03, Type: UDP, Port: 14570
    # 
    # ⚠️ 重要: 启动前必须确保本机IP已正确配置为 192.168.68.52
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        # 以下 URL 已由 USV_CONFIG 自动选择，无需手动切换:
        # usv_01: udp://192.168.68.55:14551@192.168.68.53:14550
        # usv_02: udp://192.168.68.54:14552@192.168.68.53:14560
        # usv_03: udp://192.168.68.52:14553@192.168.68.53:14570
        default_value=cfg['gcs_url'],  # ← 自动检测
        description='地面站MAVLink通信地址（自动检测）'
    )
    
    # 激光雷达串口参数
   # lidar_port_arg = DeclareLaunchArgument(
      #  'lidar_po
      #  default_value='/dev/ttyUSB0',
      #  description='激光雷达串口路径'
    #)

    # =============================================================================
    # 参数配置加载
    # =============================================================================

    param_file = LaunchConfiguration('param_file')
    sitl_param_file = LaunchConfiguration('sitl_param_file')

    namespace = LaunchConfiguration('namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    enable_log_collector = LaunchConfiguration('enable_log_collector')
    simulation_mode = LaunchConfiguration('simulation_mode')
    target_system_id = LaunchConfiguration('target_system_id')
    #lidar_port = LaunchConfiguration('lidar_port')

    # =============================================================================
    # 通信与状态管理节点
    # =============================================================================

  

    # GPS 到本地坐标转换节点（新增）
    gps_to_local_node = Node(
        package='usv_comm',
        executable='gps_to_local_node',
        name='gps_to_local_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file]
        # ⚠️ 移除重映射，避免与 MAVROS 的 local_position/pose 冲突
        # 其他节点可以选择订阅：
        # - local_position/pose (MAVROS 原生，飞控 EKF Origin)
        # - local_position/pose_from_gps (GPS 转换，A0 基站原点)
    )

    # 状态处理节点
    usv_status_node = Node(
        package='usv_comm',
        executable='usv_status_node',
        name='usv_status_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file]
    )

    # 自动设置home点节点
    auto_set_home_node = Node(
        package='usv_comm',
        executable='auto_set_home_node',
        name='auto_set_home_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # =============================================================================
    # 控制相关节点
    # =============================================================================

    # 避障节点
    # usv_avoidance_node = Node(
    #     package='usv_control',
    #     executable='usv_avoidance_node',
    #     name='usv_avoidance_node',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[param_file]
    # )

    # mode和arm切换节点
    usv_command_node = Node(
        package='usv_control',
        executable='usv_command_node',
        name='usv_command_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file]
    )

    # 控制器节点
    usv_control_node = Node(
        package='usv_control',
        executable='usv_control_node',
        name='usv_control_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file]
    )

    # 速度控制器节点 (MPC 控制)
    # 仅在 usv_params.yaml 中 control_mode='velocity' 时需要启动
    # 可实现无减速的平滑航点跟踪
    velocity_controller_node = Node(
        package='usv_control',
        executable='velocity_controller_node',
        name='velocity_controller_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file]
    )

    # 坐标转换节点（XYZ → GPS）（新增）
    coord_transform_node = Node(
        package='usv_control',
        executable='coord_transform_node',
        name='coord_transform_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file]
    )
    
    # 编队跟随节点 (USV 端计算)
    # 接收 GS 下发的 FormationConfig，订阅领队状态中转，本地高频计算编队目标
    formation_follower_node = Node(
        package='usv_control',
        executable='formation_follower_node',
        name='formation_follower_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file, {'usv_id': namespace}]
    )

    # 日志收集节点（可选，用于调试导航）
    log_collector_node = Node(
        package='usv_control',
        executable='log_collector',
        name='log_collector',
        namespace=namespace,
        output='screen',
        condition=IfCondition(enable_log_collector)
    )

    # =============================================================================
    # 传感器驱动节点
    # =============================================================================

    # UWB定位节点
    usv_uwb_node = Node(
        package='usv_drivers',
        executable='usv_uwb_node',
        name='usv_uwb_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 激光雷达节点
    usv_laserscan_node = Node(
        package='usv_drivers',
        executable='usv_laserscan_node',
        name='usv_laserscan_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 超声波传感器节点
    usv_ultrasonic_node = Node(
        package='usv_drivers',
        executable='usv_ultrasonic_node',
        name='usv_ultrasonic_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # SU04超声波传感器节点
    usv_su04_node = Node(
        package='usv_drivers',
        executable='usv_su04_node',
        name='usv_su04_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 超声波雷达节点
    usv_ultrasonic_radar_node = Node(
        package='usv_drivers',
        executable='usv_ultrasonic_radar_node',
        name='usv_ultrasonic_radar_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # =============================================================================
    # 辅助功能节点
    # =============================================================================

    # LED控制节点（仅硬件模式）
    usv_led_node = Node(
        package='usv_led',
        executable='usv_led_node',
        name='usv_led_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'hardware')),
        parameters=[param_file]
    )

    # 声音控制节点（仅硬件模式）
    usv_sound_node = Node(
        package='usv_sound',
        executable='usv_sound_node',
        name='usv_sound_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'hardware')),
        parameters=[param_file]
    )

    # 风扇控制节点（仅硬件模式）
    usv_fan_node = Node(
        package='usv_fan',
        executable='usv_fan_node',
        name='usv_fan_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'hardware')),
        parameters=[param_file]
    )

    # 鸭头动作控制节点（仅硬件模式）
    usv_head_action_node = Node(
        package='usv_action',
        executable='usv_head_action_node',
        name='usv_head_action_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'hardware')),
        parameters=[param_file]
    )

    # NavigateToPoint 导航节点 (话题版本,替代Action服务器)
    # 使用话题通信更适合跨Domain通信场景
    navigate_to_point_node = Node(
        package='usv_comm',
        executable='navigate_to_point_node',
        name='navigate_to_point_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file, sitl_param_file]
    )



    # =============================================================================
    # 飞控通信节点
    # =============================================================================

    # =========================================================================
    # MAVROS 节点 - 硬件模式 (simulation_mode == 'hardware')
    # =========================================================================
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'hardware')),
        parameters=[
            param_file,
            {
                # 核心连接参数
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                
                # MAVLink 身份配置 (硬件模式)
                'system_id': cfg['system_id'],           # MAVROS 自身系统 ID (自动检测)
                'component_id': 191,                     # MAVROS 自身组件 ID
                'target_system_id': cfg['target_system_id'],  # 目标飞控系统 ID (自动检测)
                'target_component_id': 1,                # 目标飞控组件 ID (固定为1)
                
                # ==================== 插件黑名单（加速启动，关键优化！）====================
                # 禁用不需要的插件，从 50+ 个插件减少到 15 个核心插件
                # 预期节省启动时间：约 270 秒
                # ⚠️ 注意：ROS 2 中参数名为 plugin_denylist (不是 plugin_blacklist)
                'plugin_denylist': [
                    'actuator_control',      # 执行器控制（不需要）
                    'adsb',                  # ADS-B 防撞（水面船不需要）
                    'altitude',              # 高度传感器（已有 GPS）
                    'cam_imu_sync',          # 相机同步（无相机）
                    'camera',                # 相机控制（无相机）
                    'cellular_status',       # 蜂窝网络（无蜂窝模块）
                    'companion_process_status',  # 伴随计算机状态（已用 ROS）
                    'debug_value',           # 调试值（生产环境不需要）
                    'distance_sensor',       # 距离传感器（无超声波/激光高度计）
                    'esc_status',            # ESC 状态（不需要）
                    'esc_telemetry',         # ESC 遥测（不需要）
                    'fake_gps',              # 虚假 GPS（实际 GPS 工作）
                    'ftp',                   # FTP 文件传输（用 SSH）
                    'gimbal_control',        # 云台控制（无云台）
                    'gps_input',             # GPS 输入（使用飞控内置 GPS）
                    'gps_rtk',               # RTK GPS（使用飞控内置 GPS）
                    'guided_target',         # GUIDED 目标（用 setpoint_raw）
                    'hil',                   # 硬件在环（实际硬件）
                    'landing_target',        # 着陆目标（水面船不需要）
                    'log_transfer',          # 日志传输（用 SD 卡）
                    'mag_calibration_status',  # 磁力计校准（飞控上操作）
                    'manual_control',        # 手动控制（用遥控器）
                    'mocap_pose_estimate',   # 动捕姿态（无动捕系统）
                    'mount_control',         # 挂载控制（无云台）
                    'nav_controller_output', # 导航控制器（内部使用）
                    'obstacle_distance',     # 障碍物距离（已有超声波节点）
                    'odometry',              # 里程计（用 GPS）
                    'onboard_computer_status',  # 机载计算机（已用 ROS）
                    'open_drone_id',         # 无人机远程识别（水面船不需要）
                    'optical_flow',          # 光流（无光流传感器）
                    'param',                 # ⚠️ 已启用：参数同步（解决 QGC 参数加载失败问题）
                    'play_tune',             # 播放音调（无蜂鸣器）
                    'px4flow',               # PX4 光流（无光流）
                    'rangefinder',           # 测距仪（已有 GPS 高度）
                    'setpoint_accel',        # 加速度目标点（用 setpoint_raw）
                    'setpoint_attitude',     # 姿态目标点（用 setpoint_raw）
                    'setpoint_position',     # 位置目标点（用 setpoint_raw）
                    'setpoint_trajectory',   # 轨迹目标点（用 setpoint_raw）
                    'setpoint_velocity',     # 速度目标点（用 setpoint_raw）
                    'tdr_radio',             # 数传电台（不需要）
                    'terrain',               # 地形跟随（水面船不需要）
                    'trajectory',            # 轨迹规划（用 setpoint_raw）
                    'tunnel',                # MAVLink 隧道（不需要）
                    'vibration',             # 振动监测（生产环境不需要）
                    'vision_pose',           # 视觉定位（无视觉系统）
                    'vision_speed',          # 视觉速度（无视觉系统）
                    'vfr_hud',               # HUD 数据（不需要显示 HUD）
                    'waypoint',              # 航点任务（只用 GUIDED 模式，不需要航点）
                    'wheel_odometry',        # 轮式里程计（无编码器）
                    'wind_estimation',       # 风速估计（水面船不需要）
                    'rallypoint',            # 集结点（不需要任务功能）
                    'geofence',              # 地理围栏（不需要任务功能）
                ],
                
                # ==================== 保留的核心插件（11个）====================
                # ✅ sys_status      - 系统状态（连接、解锁、模式）
                # ✅ sys_time        - 时间同步
                # ✅ command         - 命令发送（解锁、模式切换）
                # ✅ local_position  - 本地位置（EKF 输出的 XYZ 坐标）
                # ✅ global_position - GPS 位置（经纬度、高度）
                # ✅ home_position   - Home 点（返航位置）
                # ✅ gps_status      - GPS 状态（卫星数、精度）
                # ✅ battery         - 电池状态（电压、电流、电量）
                # ✅ imu             - IMU 数据（姿态、角速度、加速度）
                # ✅ rc_io           - 遥控器通道（遥控输入/输出）
                # ✅ setpoint_raw    - 原始目标点（GUIDED 模式主要控制方式）
                
                # ==================== 性能优化参数 ====================
                'disable_diag': True,               # 禁用版本查询（节省 10-15 秒）
                # 注意: 由于已禁用 waypoint/rallypoint/geofence 插件，
                # 以下参数不再需要（插件都不加载了）
                # 'mission.pull_after_gcs': False,
                # 'waypoint.pull_after_gcs': False,
                # 'rallypoint.pull_after_gcs': False,
                # 'geofence.pull_after_gcs': False,
                
                # 连接参数 (MAVROS sys_status 插件参数)
                'conn_timeout': 30.0,               # 连接超时 30 秒 (增加超时时间，提升断连容忍度)
                'heartbeat_mav_type': 'SURFACE_BOAT',  # MAV_TYPE 字符串 (MAVROS 要求 string 类型)
                'heartbeat_rate': 1.0,              # 心跳频率 1 Hz
            },
        ]
    )

    # =========================================================================
    # MAVROS 节点 - SITL 仿真模式 (simulation_mode == 'sitl')
    # =========================================================================
    # SITL 模式关键差异：
    #   - system_id: 255 (标准 GCS ID)
    #   - target_system_id: 1 (SITL 默认 sysid=1)
    #   - fcu_url: tcp://127.0.0.1:5760 (由 sitl_launch.py 传入)
    #   - 减少不必要的插件禁用（SITL 中一些插件有用）
    mavros_node_sitl = Node(
        package='mavros',
        executable='mavros_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'sitl')),
        parameters=[
            param_file,
            sitl_param_file,
            {
                # 核心连接参数
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                
                # MAVLink 身份配置 (SITL 模式)
                # ⚠️ system_id 不能用 255，因为 MAVProxy 已经用了 255，会冲突
                # 多 USV: target_system_id 通过 launch 参数传入，匹配 SYSID_THISMAV
                'system_id': 240,           # MAVROS 系统 ID (避开 MAVProxy 的 255)
                'component_id': 191,        # MAVROS 组件 ID
                'target_system_id': target_system_id,   # 匹配 SYSID_THISMAV
                'target_component_id': 1,   # 飞控组件 ID
                
                # SITL 模式下的插件黑名单
                # ⚠️ param 插件已禁用: SITL 参数通过 .parm 文件加载，无需 MAVROS 拉取
                #    启用 param 会导致下载 1233 个参数，严重拖慢启动并引起心跳超时
                'plugin_denylist': [
                    'actuator_control',
                    'adsb',
                    'cam_imu_sync',
                    'camera',
                    'cellular_status',
                    'companion_process_status',
                    'debug_value',
                    'distance_sensor',
                    'esc_status',
                    'esc_telemetry',
                    'fake_gps',
                    'ftp',
                    'gimbal_control',
                    'gps_input',
                    'gps_rtk',
                    'guided_target',
                    'landing_target',
                    'log_transfer',
                    'mag_calibration_status',
                    'manual_control',
                    'mocap_pose_estimate',
                    'mount_control',
                    'nav_controller_output',
                    'obstacle_distance',
                    'odometry',
                    'onboard_computer_status',
                    'open_drone_id',
                    'optical_flow',
                    'param',                # ← SITL 不需要参数同步
                    'play_tune',
                    'px4flow',
                    'rangefinder',
                    'setpoint_accel',
                    'setpoint_attitude',
                    'setpoint_position',
                    'setpoint_trajectory',
                    'setpoint_velocity',
                    'tdr_radio',
                    'terrain',
                    'trajectory',
                    'tunnel',
                    'vibration',
                    'vision_pose',
                    'vision_speed',
                    'waypoint',             # ← SITL GUIDED 模式不需要航点
                    'wheel_odometry',
                    'wind_estimation',
                    'rallypoint',
                    'geofence',
                ],
                
                # 性能参数
                'disable_diag': True,
                
                # 连接参数 (MAVROS sys_status 插件参数)
                'conn_timeout': 30.0,               # 30s (多实例时 SITL 响应慢)
                'heartbeat_mav_type': 'SURFACE_BOAT',  # MAV_TYPE 字符串
                'heartbeat_rate': 1.0,
            },
        ]
    )

    # =============================================================================
    # 传感器相关节点
    # =============================================================================

    # RPLIDAR节点
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_composition',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                #'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'scan_frequency': 10.0,
                'frame_id': [
                    TextSubstitution(text='laser_frame_'), 
                    LaunchConfiguration('namespace')
                ]
            }
        ]
    )

    # =============================================================================
    # 坐标变换节点
    # =============================================================================

    # 静态坐标变换发布器
    static_tf_laser_node = Node(
        package='usv_tf',
        executable='static_tf_laser_node',
        name='static_tf_laser_node',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # 里程计到TF转换节点
    odom_to_tf = Node(
        package='usv_tf',
        executable='odom_to_tf',
        name='odom_to_tf',
        namespace=namespace,
        output='screen',
        parameters=[param_file]
    )

    # =============================================================================
    # 启动顺序控制：等待 MAVROS 连接后再启动依赖节点
    # =============================================================================
    
    # 第一批：延迟 2 秒启动 auto_set_home_node（确保 GPS 完全就绪）
    # ⚠️ 关键修复：延长启动时间，避免 GPS 高度未收敛导致 z 坐标偏移 -17m
    delayed_home_node = TimerAction(
        period=2.0,  # 等待 MAVROS + GPS 就绪（2秒确保 GPS 数据开始输出）
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'hardware')),
        actions=[
           auto_set_home_node,    # 自动设置 EKF Origin（必须在 GPS 高度稳定后）
        ]
    )
    
    # 第二批：延迟 13 秒启动其他节点（确保 EKF 原点已正确设置）
    # 计算：2s (auto_set_home启动) + 10s (GPS高度收敛) + 1s (EKF应用原点) = 13s
    # ⚠️ 关键：必须等待 GPS 高度充分收敛后再启动依赖 EKF 的节点
    delayed_control_nodes = TimerAction(
        period=13.0,
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'hardware')),
        actions=[
            gps_to_local_node,        # GPS→本地坐标转换（新增，优先启动）
            coord_transform_node,     # XYZ→GPS坐标转换（新增）
            navigate_to_point_node,   # NavigateToPoint 导航节点（话题版本）
            usv_status_node,          # 状态管理（依赖 MAVROS）
            usv_control_node,         # 核心控制器（依赖 MAVROS 和 EKF 原点）
            usv_command_node,         # 命令处理（依赖 MAVROS）
            velocity_controller_node, # 速度控制器（MPC，速度模式下使用）
            formation_follower_node, # 编队跟随（USV 端计算）
            # usv_avoidance_node,     # 避障功能（已注释）
        ]
    )

    # =========================================================================
    # SITL 模式启动序列（延迟更短，SITL GPS 秒定位）
    # =========================================================================
    delayed_home_node_sitl = TimerAction(
        period=1.0,  # SITL 中 GPS 立即就绪
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'sitl')),
        actions=[
           auto_set_home_node,
        ]
    )
    
    delayed_control_nodes_sitl = TimerAction(
        period=5.0,  # SITL 中 EKF 收敛更快，5 秒足够
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulation_mode'), 'sitl')),
        actions=[
            gps_to_local_node,
            coord_transform_node,
            navigate_to_point_node,
            usv_status_node,
            usv_control_node,
            usv_command_node,
            velocity_controller_node,
            formation_follower_node,  # 编队跟随（USV 端计算）
        ]
    )
    
    # =============================================================================
    # 启动描述配置
    # =============================================================================

    # =====================================================================
    # 硬件模式启动顺序（总时间约 13 秒，确保 GPS 高度充分收敛）：
    # 阶段 1 (t=0s):   启动 MAVROS（开始连接飞控，GPS 数据开始输出）
    # 阶段 2 (t=2s):   启动 auto_set_home_node（等待 GPS 水平定位就绪）
    # 阶段 3 (t=2-12s): auto_set_home_node 内部等待 10 秒（GPS 高度收敛）
    # 阶段 4 (t=13s):  启动控制节点（EKF 原点已正确设置，z 坐标无偏移）
    # 阶段 5 (t=0s):   辅助功能节点并行启动（不依赖 MAVROS）
    #
    # SITL 模式启动顺序（总时间约 5 秒）：
    # 阶段 1 (t=0s):   启动 MAVROS（TCP 连接 SITL）
    # 阶段 2 (t=1s):   启动 auto_set_home_node（SITL GPS 立即就绪）
    # 阶段 3 (t=5s):   启动控制节点
    # 辅助硬件节点: 不启动（SITL 无硬件外设）
    # =====================================================================
    return LaunchDescription([
        # 基础参数配置
        simulation_mode_arg,
        namespace_arg,
        param_file_arg,
        fcu_url_arg,
        gcs_url_arg,
        enable_log_collector_arg,
        target_system_id_arg,
        sitl_param_file_arg,
        #lidar_port_arg,
        
        # ============================
        # 硬件模式 (simulation_mode == 'hardware')
        # ============================
        mavros_node,               # 飞控通信 - 硬件模式（条件启动）
        delayed_home_node,         # 延迟启动 home 设置
        delayed_control_nodes,     # 延迟启动控制节点
        
        # 辅助功能节点（仅硬件模式，SITL 无硬件外设）
        usv_led_node,          # LED控制
        usv_sound_node,        # 声音控制
        usv_fan_node,          # 风扇控制
        usv_head_action_node,       # 鸭头动作控制
        log_collector_node,         # 导航日志收集
        
        # ============================
        # SITL 模式 (simulation_mode == 'sitl')
        # ============================
        mavros_node_sitl,          # 飞控通信 - SITL 模式（条件启动）
        delayed_home_node_sitl,    # 延迟启动 home 设置（SITL 延迟短）
        delayed_control_nodes_sitl,# 延迟启动控制节点（SITL 延迟短）
        
        # 可选节点（根据硬件配置启用）
        # usv_uwb_node,             # UWB定位
        # usv_laserscan_node,       # 激光雷达
        # usv_ultrasonic_node,      # 超声波传感器
        # usv_su04_node,            # SU04超声波传感器
        # rplidar_node,             # RPLIDAR激光雷达
        # static_tf_laser_node,     # 激光雷达坐标变换
        # odom_to_tf,               # 里程计坐标变换
    ])
