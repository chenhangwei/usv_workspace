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

# 尝试导入可视化库
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("⚠️  matplotlib 未安装，将只输出文本分析")
    print("   安装: pip3 install matplotlib")


def load_csv(filepath: str) -> list:
    """加载 CSV 文件"""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # 转换为浮点数
            parsed = {}
            for key, value in row.items():
                try:
                    parsed[key] = float(value)
                except ValueError:
                    parsed[key] = value
            data.append(parsed)
    return data


def analyze_statistics(data: list):
    """统计分析"""
    print("\n" + "="*60)
    print("📊 统计分析")
    print("="*60)
    
    # 基本信息
    duration = data[-1]['timestamp'] - data[0]['timestamp']
    print(f"\n⏱️  记录时长: {duration:.1f} 秒 ({len(data)} 条记录)")
    
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
    
    # 距离误差统计
    distances = [d['distance_to_goal'] for d in data if d['goal_id'] > 0]
    if distances:
        min_dist = min(distances)
        print(f"\n🎯 最小到达距离: {min_dist:.3f} m")
    
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
        
        print(f"   平均求解时间: {avg_time:.2f} ms")
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
    
    # Distance to goal
    ax = axes[2]
    dist = [d['distance_to_goal'] for d in data]
    ax.plot(t, dist, 'purple', label='Distance to Goal')
    ax.axhline(y=0.8, color='r', linestyle='--', label='Arrival Threshold (0.8m)')
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
        
        log_files = sorted(log_dir.glob('nav_log_*.csv'))
        if not log_files:
            print("❌ 未找到日志文件")
            sys.exit(1)
        
        log_file = log_files[-1]
        print(f"📂 使用最新日志: {log_file}")
    
    if not log_file.exists():
        print(f"❌ 文件不存在: {log_file}")
        sys.exit(1)
    
    # 加载数据
    print(f"\n📖 加载日志: {log_file}")
    data = load_csv(str(log_file))
    print(f"   记录数: {len(data)}")
    
    if len(data) < 10:
        print("⚠️  数据量太少，无法分析")
        sys.exit(1)
    
    # 统计分析
    analyze_statistics(data)
    
    # 航向偏移分析
    find_yaw_offset(data)
    
    # 可视化
    if HAS_MATPLOTLIB:
        print("\n📊 生成图表...")
        output_path = log_file.parent
        plot_trajectory(data, output_path)
        plot_velocity(data, output_path)
        plot_heading_comparison(data, output_path)
        plot_control_commands(data, output_path)
        plot_mpc_debug(data, output_path)
        print(f"\n✅ 图表已保存到: {output_path}")
    
    print("\n" + "="*60)
    print("分析完成!")
    print("="*60)


if __name__ == '__main__':
    main()
