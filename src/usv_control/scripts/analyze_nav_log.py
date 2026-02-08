#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of analyze nav log.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
USV 导航日志分析脚本

分析日志文件，生成可视化图表，帮助调试导航问题。

用法:
    python3 analyze_nav_log.py <log_file.csv>
    python3 analyze_nav_log.py  # 自动使用最新的日志文件

输出:
    - 轨迹图 (位置 + 目标点)
    - 速度图 (速度大小 + 方向)
    - 航向对比图 (速度航向 vs 磁力计航向)
    - 控制指令图
    - 误差图 (距离误差 + 航向误差)

作者: Auto-generated
日期: 2026-01-25
"""

import sys
import csv
import math
from pathlib import Path
from datetime import datetime
from typing import Any

# 尝试导入可视化库
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("⚠️  matplotlib 未安装，将只输出文本分析")
    print("   安装: pip3 install matplotlib")


def load_csv(filepath: str) -> tuple:
    """加载 CSV 文件
    
    Returns:
        tuple: (data, header_info)
            - data: 数据行列表
            - header_info: 日志头信息字典 (USV ID, 参数等)
    """
    data = []
    header_info = {
        'usv_id': 'unknown',
        'version': 'v5',  # 默认 v5
        'params': {}
    }
    
    def _to_number(value: Any):
        if value is None:
            return None
        if isinstance(value, (int, float)):
            return float(value)
        s = str(value).strip()
        if s == "":
            return None
        try:
            return float(s)
        except ValueError:
            return s

    def _parse_header_line(line: str):
        """解析日志头注释行中的参数"""
        line = line.lstrip('#').strip()
        if ':' in line:
            key, _, value = line.partition(':')
            key = key.strip().lower().replace(' ', '_').replace('(', '').replace(')', '')
            value = value.strip()
            # 解析特定字段
            if 'usv_id' in key or key == 'usv_id':
                header_info['usv_id'] = value
            elif 'v6' in line.lower() or 'adaptive' in line.lower():
                header_info['version'] = 'v6'
            elif 'v7' in line.lower():
                header_info['version'] = 'v7'
            # 解析数值参数
            if any(k in key for k in ['tau', 'weight', 'threshold', 'q_', 'r_', 'w_max', 'n_steps']):
                try:
                    header_info['params'][key] = float(value.split()[0])
                except (ValueError, IndexError):
                    header_info['params'][key] = value

    with open(filepath, 'r', errors='replace', newline='') as f:
        # 兼容 v5/v6 日志：文件头可能包含若干以 # 开头的注释/参数行
        # 解析并找到第一行非注释作为 CSV header
        while True:
            pos = f.tell()
            line = f.readline()
            if not line:
                return [], header_info
            if line.strip() and not line.lstrip().startswith('#'):
                f.seek(pos)
                break
            # 解析日志头参数
            if line.lstrip().startswith('#'):
                _parse_header_line(line)

        reader = csv.DictReader(f)
        for row in reader:
            # 兼容：文件尾可能有 '# ...' 统计行，或不完整行
            ts = row.get('timestamp')
            ts_num = _to_number(ts)
            if not isinstance(ts_num, (int, float)):
                continue

            parsed = {}
            for key, value in row.items():
                parsed[key] = _to_number(value)
            data.append(parsed)
    
    # 检测 v6 特有字段
    if data and 'current_tau_omega' in data[0]:
        if header_info['version'] == 'v5':
            header_info['version'] = 'v6'
    
    return data, header_info


def analyze_statistics(data: list, header_info: dict = None):
    """统计分析"""
    print("\n" + "="*60)
    print("📊 统计分析")
    print("="*60)
    
    # 显示 USV ID 和版本信息 (v6+)
    if header_info:
        usv_id = header_info.get('usv_id', 'unknown')
        version = header_info.get('version', 'v5')
        print(f"\n🚢 USV: {usv_id}  (日志版本: {version})")
        
        # 显示 v6 自适应参数
        params = header_info.get('params', {})
        if params:
            tau_params = {k: v for k, v in params.items() if 'tau' in k}
            if tau_params:
                print(f"\n🔧 自适应 Tau 参数:")
                for k, v in tau_params.items():
                    print(f"   {k}: {v}")
    
    # 基本信息
    duration = data[-1]['timestamp'] - data[0]['timestamp']
    print(f"\n⏱️  记录时长: {duration:.1f} 秒 ({len(data)} 条记录)")

    # 采样率统计
    if len(data) >= 3:
        dts = [data[i]['timestamp'] - data[i - 1]['timestamp'] for i in range(1, len(data))]
        dts_sorted = sorted(dts)
        dt_med = dts_sorted[len(dts_sorted) // 2]
        dt_p95 = dts_sorted[int(len(dts_sorted) * 0.95)]
        dt_max = dts_sorted[-1]
        hz = (1.0 / dt_med) if dt_med > 1e-9 else 0.0
        print(f"\n🧾 采样信息:")
        print(f"   中位 dt: {dt_med:.3f}s (~{hz:.1f} Hz)")
        print(f"   P95 dt: {dt_p95:.3f}s")
        print(f"   最大 dt: {dt_max:.3f}s")
    
    # 速度统计
    speeds = [d['velocity_speed'] for d in data]
    avg_speed = sum(speeds) / len(speeds)
    max_speed = max(speeds)
    print(f"\n🚀 速度统计:")
    print(f"   平均速度: {avg_speed:.3f} m/s")
    print(f"   最大速度: {max_speed:.3f} m/s")
    
    # 航向差异统计
    yaw_diffs = [abs(d['yaw_diff_deg']) for d in data if d['velocity_speed'] > 0.1]
    if yaw_diffs:
        avg_diff = sum(yaw_diffs) / len(yaw_diffs)
        max_diff = max(yaw_diffs)
        print(f"\n🧭 航向差异 (速度航向 - 磁力计):")
        print(f"   平均差异: {avg_diff:.1f}°")
        print(f"   最大差异: {max_diff:.1f}°")
    
    # 距离误差统计（优先使用几何距离：target - pose）
    dist_calc = None
    if all(k in data[0] for k in ('pose_x', 'pose_y', 'target_x', 'target_y')):
        dist_calc = [
            math.hypot(d['target_x'] - d['pose_x'], d['target_y'] - d['pose_y'])
            for d in data
            if all(isinstance(d.get(k), (int, float)) for k in ('pose_x', 'pose_y', 'target_x', 'target_y'))
        ]

    dist_logged = [d.get('distance_to_goal') for d in data if isinstance(d.get('distance_to_goal'), (int, float))]
    distances = dist_calc if dist_calc else dist_logged

    if distances:
        min_dist = min(distances)
        end_dist = distances[-1]
        print(f"\n🎯 最小到达距离: {min_dist:.3f} m")
        print(f"   结束时距离: {end_dist:.3f} m")

        # 如果两者同时存在，做一致性检查（常见于 goal 切换时序不同步）
        if dist_calc and dist_logged and len(dist_calc) == len(dist_logged):
            diffs = [abs(a - b) for a, b in zip(dist_calc, dist_logged)]
            bad = sum(1 for x in diffs if x > 2.0)
            if bad:
                print(f"   ⚠️ distance_to_goal 与几何距离不一致: {bad}/{len(diffs)} 点 (最大差异 {max(diffs):.2f}m)")

        # 到达阈值（默认 1.5m，请与 usv_params.yaml 保持一致）
        threshold = 1.5
        reach_idx = next((i for i, d in enumerate(distances) if d <= threshold), None)
        if reach_idx is not None:
            reach_t = data[reach_idx]['timestamp'] - data[0]['timestamp']
            print(f"   首次进入 {threshold:.1f}m 阈值: t={reach_t:.1f}s")
        else:
            print(f"   ⚠️ 未进入 {threshold:.1f}m 到达阈值")

    # 横向偏差统计
    if 'cross_track_error' in data[0]:
        ctes = [abs(d['cross_track_error']) for d in data if isinstance(d.get('cross_track_error'), (int, float))]
        if ctes:
            rms = math.sqrt(sum(x * x for x in ctes) / len(ctes))
            print(f"\n📐 横向误差 |cross_track_error|:")
            print(f"   RMS: {rms:.3f} m")
            print(f"   最大: {max(ctes):.3f} m")

    # 航向误差统计
    if 'heading_error_deg' in data[0]:
        hes = [abs(d['heading_error_deg']) for d in data if isinstance(d.get('heading_error_deg'), (int, float))]
        if hes:
            print(f"\n🧭 航向误差 |heading_error_deg|:")
            print(f"   平均: {sum(hes)/len(hes):.1f}°")
            print(f"   最大: {max(hes):.1f}°")
    
    # 角速度统计
    omegas = [abs(d['cmd_omega']) for d in data]
    avg_omega = sum(omegas) / len(omegas)
    max_omega = max(omegas)
    print(f"\n🔄 角速度指令:")
    print(f"   平均: {avg_omega:.3f} rad/s ({math.degrees(avg_omega):.1f}°/s)")
    print(f"   最大: {max_omega:.3f} rad/s ({math.degrees(max_omega):.1f}°/s)")

    # MPC 统计
    if 'mpc_solve_time_ms' in data[0]:
        print("\n🤖 MPC 性能分析")
        solve_times = [d.get('mpc_solve_time_ms', 0) for d in data]
        avg_time = sum(solve_times) / len(solve_times)
        max_time = max(solve_times)
        st_sorted = sorted(solve_times)
        p95_time = st_sorted[int(len(st_sorted) * 0.95)]
        
        print(f"   平均求解时间: {avg_time:.2f} ms")
        print(f"   P95 求解时间: {p95_time:.2f} ms")
        print(f"   最大求解时间: {max_time:.2f} ms")
        if max_time > 50:
             print(f"   ⚠️  求解时间过长 (>50ms)")
             
        # 活跃控制器分布
        controllers = {}
        for d in data:
            ctrl = d.get('active_ctrl', 'unknown')
            controllers[ctrl] = controllers.get(ctrl, 0) + 1
            
        print(f"   控制器分布:")
        for ctrl, count in controllers.items():
            print(f"   - {ctrl}: {count} ({count/len(data)*100:.1f}%)")

    # 目标点/任务段统计
    goals = [int(d['goal_id']) for d in data if isinstance(d.get('goal_id'), (int, float))]
    if goals:
        unique_goals = []
        for g in goals:
            if not unique_goals or unique_goals[-1] != g:
                unique_goals.append(g)
        print(f"\n🗺️  goal_id 变化序列: {unique_goals}")

    # 控制输出范围与疑似饱和
    def _sat_ratio(values, limit, tol=1e-3):
        if not values:
            return 0.0
        hits = sum(1 for v in values if isinstance(v, (int, float)) and abs(abs(v) - limit) <= tol)
        return hits / len(values)

    vx_cmds = [d.get('cmd_vx') for d in data if isinstance(d.get('cmd_vx'), (int, float))]
    om_cmds = [d.get('cmd_omega') for d in data if isinstance(d.get('cmd_omega'), (int, float))]
    if vx_cmds:
        vmax = max(abs(v) for v in vx_cmds)
        print(f"\n🎮 指令范围:")
        print(f"   cmd_vx: [{min(vx_cmds):.3f}, {max(vx_cmds):.3f}] m/s")
        if vmax > 1e-6:
            print(f"   cmd_vx 触顶比例(~{vmax:.3f}): {_sat_ratio(vx_cmds, vmax)*100:.1f}%")
    if om_cmds:
        omax = max(abs(v) for v in om_cmds)
        print(f"   cmd_omega: [{min(om_cmds):.3f}, {max(om_cmds):.3f}] rad/s")
        if omax > 1e-6:
            print(f"   cmd_omega 触顶比例(~{omax:.3f}): {_sat_ratio(om_cmds, omax)*100:.1f}%")
        
        # 按配置的 w_max 计算真实饱和率 (v6+)
        if header_info:
            w_max_cfg = header_info.get('params', {}).get('w_max_max_angular_velocity', 0.5)
            if isinstance(w_max_cfg, (int, float)) and w_max_cfg > 0:
                true_sat = _sat_ratio(om_cmds, w_max_cfg, tol=0.01)
                print(f"   → 配置 w_max={w_max_cfg:.2f} 饱和率: {true_sat*100:.1f}%")
    
    # v6+ 振荡频率分析 (零交叉法)
    if om_cmds and len(om_cmds) > 20:
        zero_crossings = 0
        for i in range(1, len(om_cmds)):
            if om_cmds[i-1] * om_cmds[i] < 0:  # 符号变化
                zero_crossings += 1
        duration_s = data[-1]['timestamp'] - data[0]['timestamp']
        if duration_s > 0:
            osc_freq = zero_crossings / (2 * duration_s)  # 每个周期2次过零
            print(f"\n📈 振荡分析:")
            print(f"   角速度方向反转: {zero_crossings} 次")
            print(f"   估计振荡频率: {osc_freq:.2f} Hz ({1/osc_freq:.1f}s/周期)" if osc_freq > 0.01 else "   估计振荡频率: 无明显振荡")
            if osc_freq > 0.3:
                print(f"   ⚠️  振荡频率较高，可能存在 S 形轨迹问题")


def plot_errors(data: list, output_path: Path):
    """绘制误差相关曲线"""
    if not HAS_MATPLOTLIB:
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]

    ax = axes[0]
    if all(k in data[0] for k in ('pose_x', 'pose_y', 'target_x', 'target_y')):
        dist = [math.hypot(d['target_x'] - d['pose_x'], d['target_y'] - d['pose_y']) for d in data]
    else:
        dist = [d.get('distance_to_goal', 0) for d in data]
    ax.plot(t, dist, 'purple', label='Distance to Goal')
    # 到达阈值（默认 1.5m，请与 usv_params.yaml 保持一致）
    ax.axhline(y=1.5, color='r', linestyle='--', label='Arrival Threshold (1.5m)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Errors')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    he = [d.get('heading_error_deg', 0) for d in data]
    ax.plot(t, he, 'b-', label='Heading Error (deg)')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.set_ylabel('deg')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    cte = [d.get('cross_track_error', 0) for d in data]
    ax.plot(t, cte, 'g-', label='Cross Track Error (m)')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.set_ylabel('m')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path / 'errors.png', dpi=150)
    plt.close()
    print(f"   📈 误差图: {output_path / 'errors.png'}")


def find_yaw_offset(data: list) -> float:
    """查找稳定的航向偏移"""
    # 只取速度足够大的数据点
    valid_diffs = []
    for d in data:
        if d['velocity_speed'] > 0.15:
            valid_diffs.append(d['yaw_diff_deg'])
    
    if not valid_diffs:
        return 0.0
    
    # 简单统计
    avg = sum(valid_diffs) / len(valid_diffs)
    
    # 计算标准差
    variance = sum((x - avg) ** 2 for x in valid_diffs) / len(valid_diffs)
    std_dev = math.sqrt(variance)
    
    print(f"\n🔍 航向偏移分析:")
    print(f"   样本数: {len(valid_diffs)}")
    print(f"   平均偏移: {avg:.1f}°")
    print(f"   标准差: {std_dev:.1f}°")
    
    if std_dev < 20:
        print(f"   ✅ 偏移较稳定，可以考虑补偿 {avg:.0f}°")
    else:
        print(f"   ⚠️  偏移波动较大，可能坐标系已对齐")
    
    return avg


def plot_trajectory(data: list, output_path: Path):
    """绘制轨迹图"""
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Actual trajectory
    x = [d['pose_x'] for d in data]
    y = [d['pose_y'] for d in data]
    ax.plot(x, y, 'b-', linewidth=1, label='Trajectory', alpha=0.7)
    ax.scatter(x[0], y[0], c='green', s=100, marker='o', label='Start', zorder=5)
    ax.scatter(x[-1], y[-1], c='red', s=100, marker='s', label='End', zorder=5)
    
    # 目标点
    targets = []
    for d in data:
        if d['goal_id'] > 0:
            target = (d['target_x'], d['target_y'], int(d['goal_id']))
            if target not in targets:
                targets.append(target)
    
    for tx, ty, gid in targets:
        ax.scatter(tx, ty, c='orange', s=150, marker='*', zorder=4)
        ax.annotate(f'G{gid}', (tx, ty), textcoords='offset points',
                   xytext=(5, 5), fontsize=10)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('USV Navigation Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    plt.savefig(output_path / 'trajectory.png', dpi=150)
    plt.close()
    print(f"   📈 轨迹图: {output_path / 'trajectory.png'}")


def plot_velocity(data: list, output_path: Path):
    """绘制速度图"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    # Speed magnitude
    ax = axes[0]
    speed = [d['velocity_speed'] for d in data]
    ax.plot(t, speed, 'b-', label='Speed')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Velocity Analysis')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Velocity components
    ax = axes[1]
    vx = [d['velocity_vx'] for d in data]
    vy = [d['velocity_vy'] for d in data]
    ax.plot(t, vx, 'r-', label='Vx (East)', alpha=0.7)
    ax.plot(t, vy, 'g-', label='Vy (North)', alpha=0.7)
    ax.set_ylabel('Velocity (m/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Distance to goal (prefer geometric distance)
    ax = axes[2]
    if all(k in data[0] for k in ('pose_x', 'pose_y', 'target_x', 'target_y')):
        dist = [math.hypot(d['target_x'] - d['pose_x'], d['target_y'] - d['pose_y']) for d in data]
    else:
        dist = [d.get('distance_to_goal', 0) for d in data]
    ax.plot(t, dist, 'purple', label='Distance to Goal')
    ax.axhline(y=1.5, color='r', linestyle='--', label='Arrival Threshold (1.5m)')
    ax.set_ylabel('Distance (m)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path / 'velocity.png', dpi=150)
    plt.close()
    print(f"   📈 速度图: {output_path / 'velocity.png'}")


def plot_heading_comparison(data: list, output_path: Path):
    """绘制航向对比图"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    # Heading comparison
    ax = axes[0]
    vel_yaw = [d['velocity_yaw_deg'] for d in data]
    mag_yaw = [d['magnetometer_yaw_deg'] for d in data]
    ax.plot(t, vel_yaw, 'b-', label='Velocity Heading', alpha=0.7)
    ax.plot(t, mag_yaw, 'r-', label='Magnetometer Heading', alpha=0.7)
    ax.set_ylabel('Heading (deg)')
    ax.set_title('Heading Comparison: Velocity vs Magnetometer')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Heading difference
    ax = axes[1]
    yaw_diff = [d['yaw_diff_deg'] for d in data]
    speed = [d['velocity_speed'] for d in data]
    
    # Color by speed
    colors = ['blue' if s > 0.1 else 'gray' for s in speed]
    ax.scatter(t, yaw_diff, c=colors, s=2, alpha=0.5)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax.set_ylabel('Heading Diff (deg)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Heading Difference (Blue=Moving, Gray=Low Speed)')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path / 'heading_comparison.png', dpi=150)
    plt.close()
    print(f"   📈 航向对比图: {output_path / 'heading_comparison.png'}")


def plot_control_commands(data: list, output_path: Path):
    """绘制控制指令图"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    # Vx command
    ax = axes[0]
    cmd_vx = [d['cmd_vx'] for d in data]
    ax.plot(t, cmd_vx, 'b-')
    ax.set_ylabel('Cmd Vx (m/s)')
    ax.set_title('Control Commands')
    ax.grid(True, alpha=0.3)
    
    # Vy command
    ax = axes[1]
    cmd_vy = [d['cmd_vy'] for d in data]
    ax.plot(t, cmd_vy, 'g-')
    ax.set_ylabel('Cmd Vy (m/s)')
    ax.grid(True, alpha=0.3)
    
    # Angular velocity command
    ax = axes[2]
    cmd_omega = [d['cmd_omega'] for d in data]
    ax.plot(t, cmd_omega, 'r-')
    ax.set_ylabel('Cmd Omega (rad/s)')
    ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path / 'control_commands.png', dpi=150)
    plt.close()
    print(f"   📈 控制指令图: {output_path / 'control_commands.png'}")


def plot_mpc_debug(data: list, output_path: Path):
    """绘制 MPC 调试信息"""
    if 'mpc_solve_time_ms' not in data[0]:
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    # 1. 求解时间
    ax = axes[0]
    times = [d.get('mpc_solve_time_ms', 0) for d in data]
    ax.plot(t, times, 'b-', label='Solve Time')
    ax.axhline(y=50, color='r', linestyle='--', label='Warning (50ms)')
    ax.set_ylabel('Time (ms)')
    ax.set_title('MPC Performance')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 2. 代价函数
    ax = axes[1]
    costs = [d.get('mpc_cost', 0) for d in data]
    ax.plot(t, costs, 'g-', label='Optimization Cost')
    ax.set_ylabel('Cost')
    # ax.set_yscale('log') # 视情况开启
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 3. 预测航向 vs 实际航向
    ax = axes[2]
    pred_yaw = [d.get('mpc_pred_theta_deg', 0) for d in data]
    real_yaw = [d['pose_yaw_deg'] for d in data]
    
    ax.plot(t, real_yaw, 'k-', label='Actual Yaw', alpha=0.5)
    ax.plot(t, pred_yaw, 'r--', label='MPC Ref Yaw', alpha=0.8)
    ax.set_ylabel('Yaw (deg)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path / 'mpc_debug.png', dpi=150)
    plt.close()
    print(f"   📈 MPC 调试图: {output_path / 'mpc_debug.png'}")


def plot_v6_adaptive_tau(data: list, output_path: Path, header_info: dict = None):
    """绘制 V6 自适应 tau_omega 分析图"""
    if not HAS_MATPLOTLIB:
        return
    
    # 检查是否有 v6 字段
    if 'current_tau_omega' not in data[0]:
        return
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    # 获取 USV ID 用于标题
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    
    # 1. 当前 tau_omega vs 速度
    ax = axes[0]
    tau = [d.get('current_tau_omega', 0) for d in data]
    speed = [d.get('velocity_speed', 0) for d in data]
    
    ax2 = ax.twinx()
    ln1 = ax.plot(t, tau, 'b-', label='current_tau_omega', linewidth=1.5)
    ln2 = ax2.plot(t, speed, 'g-', alpha=0.5, label='speed')
    
    ax.set_ylabel('tau_omega (s)', color='blue')
    ax2.set_ylabel('speed (m/s)', color='green')
    ax.set_title(f'V6 Adaptive Tau Analysis - {usv_id}')
    
    # 绘制配置的阈值线
    if header_info:
        params = header_info.get('params', {})
        tau_low = params.get('tau_omega_low_speed', 0.8)
        tau_high = params.get('tau_omega_high_speed', 0.4)
        if isinstance(tau_low, (int, float)):
            ax.axhline(y=tau_low, color='b', linestyle='--', alpha=0.5, label=f'tau_low={tau_low:.1f}')
        if isinstance(tau_high, (int, float)):
            ax.axhline(y=tau_high, color='b', linestyle=':', alpha=0.5, label=f'tau_high={tau_high:.1f}')
    
    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax.legend(lns, labs, loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 2. 角速度命令 vs 实际角速度
    ax = axes[1]
    omega_cmd = [d.get('omega_cmd', 0) for d in data]
    omega_actual = [d.get('omega_actual', 0) for d in data]
    
    ax.plot(t, omega_cmd, 'r-', label='omega_cmd', alpha=0.7)
    ax.plot(t, omega_actual, 'b-', label='omega_actual', linewidth=1.5)
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 3. 横向偏差 CTE
    ax = axes[2]
    cte = [d.get('cross_track_error', 0) for d in data]
    ax.plot(t, cte, 'purple', label='Cross Track Error')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.fill_between(t, cte, alpha=0.3, color='purple')
    ax.set_ylabel('CTE (m)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path / 'v6_adaptive_tau.png', dpi=150)
    plt.close()
    print(f"   📈 V6 自适应 Tau 图: {output_path / 'v6_adaptive_tau.png'}")


def main():
    # 确定日志文件路径
    if len(sys.argv) > 1:
        log_file = Path(sys.argv[1])
    else:
        # 自动查找最新的日志文件
        log_dir = Path.home() / 'usv_logs'
        if not log_dir.exists():
            print("❌ 未找到日志目录: ~/usv_logs")
            sys.exit(1)
        # v5 实测日志可能按批次/艇号分目录保存，使用递归查找
        log_files = sorted(log_dir.rglob('nav_log_*.csv'))
        if not log_files:
            print("❌ 未找到日志文件")
            sys.exit(1)
        log_file = log_files[-1]
        print(f"📂 使用最新日志: {log_file}")

    if not log_file.exists():
        print(f"❌ 文件不存在: {log_file}")
        sys.exit(1)

    # 新建输出文件夹（与csv同名）
    output_path = log_file.parent / log_file.stem
    output_path.mkdir(parents=True, exist_ok=True)

    # 加载数据
    print(f"\n📖 加载日志: {log_file}")
    data, header_info = load_csv(str(log_file))
    print(f"   记录数: {len(data)}")

    if len(data) < 10:
        print("⚠️  数据量太少，无法分析")
        sys.exit(1)

    # 统计分析
    analyze_statistics(data, header_info)

    # 航向偏移分析
    find_yaw_offset(data)

    # 可视化
    if HAS_MATPLOTLIB:
        print("\n📊 生成图表...")
        plot_trajectory(data, output_path)
        plot_velocity(data, output_path)
        plot_heading_comparison(data, output_path)
        plot_control_commands(data, output_path)
        plot_mpc_debug(data, output_path)
        plot_errors(data, output_path)
        plot_v6_adaptive_tau(data, output_path, header_info)  # v6+ 新增
        print(f"\n✅ 图表已保存到: {output_path}")

    print("\n" + "="*60)
    print("分析完成!")
    print("="*60)


if __name__ == '__main__':
    main()
