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
# Updated: 2026-02-10 - 增强: 综合仪表盘、模式着色轨迹、每航点统计、质量评分
"""
USV 导航日志分析脚本 (v2)

分析日志文件，生成可视化图表，帮助调试导航问题。

用法:
    python3 analyze_nav_log.py <log_file.csv>
    python3 analyze_nav_log.py <log_dir>   # 分析目录下全部日志
    python3 analyze_nav_log.py             # 自动使用最新的日志文件

输出图表:
    1. trajectory.png       - 轨迹图 (模式着色 + 目标点 + 方向箭头)
    2. velocity.png         - 速度分析 (实际 vs 指令 + 距离)
    3. heading_comparison.png - 航向对比 (速度航向 vs 磁力计航向)
    4. control_commands.png - 控制指令图 (含 omega 跟踪对比)
    5. errors.png           - 误差图 (距离/航向/CTE)
    6. mpc_debug.png        - MPC 调试 (求解时间/代价/预测航向)
    7. v6_adaptive_tau.png  - V6 自适应 Tau 分析 (v6+)
    8. v8_ampc_analysis.png - V8 AMPC 在线辨识 (v8+)
    9. dashboard.png        - 综合仪表盘 (一页总览)
    10. per_goal_stats.png  - 每航点统计柱状图

作者: chenhangwei
日期: 2026-02-10
"""

import sys
import csv
import math
import io
import contextlib
from pathlib import Path
from datetime import datetime
from typing import Any, Optional

# 尝试导入可视化库
try:
    import matplotlib
    matplotlib.use('Agg')  # 无头模式，适配服务器/SSH 环境
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.lines import Line2D
    import matplotlib.gridspec as gridspec
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("⚠️  matplotlib 未安装，将只输出文本分析")
    print("   安装: pip3 install matplotlib")

# 全局图表风格
if HAS_MATPLOTLIB:
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'WenQuanYi Micro Hei', 'Arial']
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['figure.dpi'] = 120
    plt.rcParams['savefig.bbox'] = 'tight'


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
            # 通用版本检测: 从 (vXX) 格式中提取版本号
            import re as _re
            version_match = _re.search(r'\(v(\d+)\)', line)
            if version_match:
                detected_ver = int(version_match.group(1))
                current_ver = int(header_info['version'].lstrip('v') or '5')
                if detected_ver > current_ver:
                    header_info['version'] = f'v{detected_ver}'
            elif 'v8' in line.lower() or 'ampc' in line.lower():
                header_info['version'] = 'v8'
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
    
    # 检测 v6/v8 特有字段
    if data and 'current_tau_omega' in data[0]:
        if header_info['version'] == 'v5':
            header_info['version'] = 'v6'
    if data and 'ampc_tau_estimated' in data[0]:
        header_info['version'] = 'v8'
    
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
    
    # GUIDED 模式过滤统计 (排除 HOLD 停留数据)
    guided_data = [d for d in data if d.get('flight_mode') == 'GUIDED']
    hold_data = [d for d in data if d.get('flight_mode') == 'HOLD']
    if guided_data:
        guided_speeds = [d['velocity_speed'] for d in guided_data]
        guided_pct = len(guided_data) / len(data) * 100
        hold_pct = len(hold_data) / len(data) * 100
        print(f"\n🎯 GUIDED模式统计 ({len(guided_data)} 点, {guided_pct:.1f}%):")
        print(f"   平均速度: {sum(guided_speeds)/len(guided_speeds):.3f} m/s")
        print(f"   最大速度: {max(guided_speeds):.3f} m/s")
        if 'cmd_vx' in guided_data[0]:
            cmd_vxs = [d['cmd_vx'] for d in guided_data if isinstance(d.get('cmd_vx'), (int, float))]
            if cmd_vxs:
                print(f"   平均cmd_vx: {sum(cmd_vxs)/len(cmd_vxs):.3f} m/s")
        print(f"   HOLD停留: {len(hold_data)} 点 ({hold_pct:.1f}%)")
    
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
    
    # v8 AMPC 统计
    if 'ampc_tau_estimated' in data[0]:
        ampc_on = [d for d in data if d.get('ampc_enabled', 0) == 1]
        if ampc_on:
            tau_ests = [d['ampc_tau_estimated'] for d in ampc_on
                        if isinstance(d.get('ampc_tau_estimated'), (int, float)) and d['ampc_tau_estimated'] > 0]
            confs = [d.get('ampc_tau_confidence', 0) for d in ampc_on
                     if isinstance(d.get('ampc_tau_confidence'), (int, float))]
            sat_ratios = [d.get('ampc_saturation_ratio', 0) for d in ampc_on
                          if isinstance(d.get('ampc_saturation_ratio'), (int, float))]
            noises = [d.get('ampc_heading_noise', 0) for d in ampc_on
                      if isinstance(d.get('ampc_heading_noise'), (int, float))]
            rebuild_counts = [d.get('ampc_rebuild_count', 0) for d in ampc_on
                              if isinstance(d.get('ampc_rebuild_count'), (int, float))]
            
            print(f"\n🧠 AMPC v8 在线辨识统计:")
            if tau_ests:
                print(f"   τ 估计范围: {min(tau_ests):.3f} ~ {max(tau_ests):.3f} s")
                print(f"   τ 最终值: {tau_ests[-1]:.3f} s")
            if confs:
                print(f"   置信度: {confs[-1]:.2f} (最终)")
            
            # 检查是否收敛
            converged_samples = [d for d in ampc_on if d.get('ampc_converged', 0) == 1]
            if converged_samples:
                first_converged_t = converged_samples[0]['timestamp'] - data[0]['timestamp']
                print(f"   收敛时间: t={first_converged_t:.1f}s")
            else:
                print(f"   ⚠️  本次任务未收敛")
            
            if sat_ratios:
                avg_sat = sum(sat_ratios) / len(sat_ratios)
                max_sat = max(sat_ratios)
                print(f"   饱和率: 平均 {avg_sat*100:.1f}%, 最大 {max_sat*100:.1f}%")
                if max_sat > 0.35:
                    print(f"   ⚠️  存在慢性饱和 (>35%)")
            
            if noises:
                avg_noise = sum(noises) / len(noises)
                print(f"   航向噪声: 平均 {avg_noise:.4f} rad/s")
                if avg_noise > 0.05:
                    print(f"   ⚠️  航向噪声偏高 (磁力计质量!)")
            
            if rebuild_counts:
                total_rebuilds = int(max(rebuild_counts))
                print(f"   求解器重建: {total_rebuilds} 次")
        else:
            print(f"\n🧠 AMPC: 未启用")

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

    # 每航点统计
    _print_per_goal_stats(data)
    
    # 质量评分总结
    _print_quality_score(data)


def _print_per_goal_stats(data: list):
    """打印每个航点的详细统计信息"""
    goals = {}
    for d in data:
        gid = d.get('goal_id')
        if not isinstance(gid, (int, float)):
            continue
        gid = int(gid)
        if gid not in goals:
            goals[gid] = []
        goals[gid].append(d)
    
    if not goals:
        return
    
    print(f"\n{'='*60}")
    print("📋 每航点详细统计")
    print("=" * 60)
    print(f"   {'GoalID':>6} {'时长(s)':>8} {'最近距(m)':>9} {'均CTE(m)':>9} "
          f"{'均航向误差°':>10} {'均速(m/s)':>9} {'模式':>8}")
    
    for gid in sorted(goals.keys()):
        gdata = goals[gid]
        t0 = gdata[0]['timestamp']
        t1 = gdata[-1]['timestamp']
        dur = t1 - t0
        
        dists = [d.get('distance_to_goal', 999) for d in gdata 
                 if isinstance(d.get('distance_to_goal'), (int, float))]
        min_dist = min(dists) if dists else float('nan')
        
        g_guided = [d for d in gdata if d.get('flight_mode') == 'GUIDED']
        
        ctes = [abs(d.get('cross_track_error', 0)) for d in g_guided
                if isinstance(d.get('cross_track_error'), (int, float))]
        avg_cte = sum(ctes) / len(ctes) if ctes else float('nan')
        
        hes = [abs(d.get('heading_error_deg', 0)) for d in g_guided
               if isinstance(d.get('heading_error_deg'), (int, float))]
        avg_he = sum(hes) / len(hes) if hes else float('nan')
        
        spds = [d.get('velocity_speed', 0) for d in g_guided
                if isinstance(d.get('velocity_speed'), (int, float))]
        avg_spd = sum(spds) / len(spds) if spds else float('nan')
        
        guided_pct = len(g_guided) / len(gdata) * 100 if gdata else 0
        mode_str = f"G{guided_pct:.0f}%"
        
        print(f"   {gid:>6} {dur:>8.1f} {min_dist:>9.3f} {avg_cte:>9.4f} "
              f"{avg_he:>10.1f} {avg_spd:>9.3f} {mode_str:>8}")


def _print_quality_score(data: list):
    """打印质量评分总结"""
    guided_data = [d for d in data if d.get('flight_mode') == 'GUIDED']
    if not guided_data:
        return
    
    print(f"\n{'='*60}")
    print("⭐ 质量评分总结")
    print("=" * 60)
    
    # CTE 评分
    ctes = [abs(d.get('cross_track_error', 0)) for d in guided_data
            if isinstance(d.get('cross_track_error'), (int, float))]
    avg_cte = sum(ctes) / len(ctes) if ctes else 999
    max_cte = max(ctes) if ctes else 999
    cte_rms = math.sqrt(sum(x*x for x in ctes) / len(ctes)) if ctes else 999
    cte_grade = "优秀" if avg_cte < 0.1 else "良好" if avg_cte < 0.3 else "一般" if avg_cte < 0.5 else "较差"
    
    # 航向误差评分
    hes = [abs(d.get('heading_error_deg', 0)) for d in guided_data
           if isinstance(d.get('heading_error_deg'), (int, float))]
    avg_he = sum(hes) / len(hes) if hes else 999
    he_grade = "优秀" if avg_he < 5 else "良好" if avg_he < 10 else "一般" if avg_he < 20 else "较差"
    
    # MPC 性能评分
    solve_times = [d.get('mpc_solve_time_ms', 0) for d in data
                   if isinstance(d.get('mpc_solve_time_ms'), (int, float))]
    avg_mpc = sum(solve_times) / len(solve_times) if solve_times else 999
    mpc_grade = "优秀" if avg_mpc < 15 else "正常" if avg_mpc < 30 else "偏高" if avg_mpc < 50 else "过高"
    
    # 速度评分
    guided_speeds = [d['velocity_speed'] for d in guided_data
                     if isinstance(d.get('velocity_speed'), (int, float))]
    avg_speed = sum(guided_speeds) / len(guided_speeds) if guided_speeds else 0
    
    print(f"   横向跟踪 (CTE):   均值={avg_cte:.4f}m  最大={max_cte:.4f}m  RMSE={cte_rms:.4f}m → 【{cte_grade}】")
    print(f"   航向误差:          均值={avg_he:.1f}°  → 【{he_grade}】")
    print(f"   MPC求解时间:       均值={avg_mpc:.1f}ms → 【{mpc_grade}】")
    print(f"   GUIDED平均速度:    {avg_speed:.3f} m/s")
    
    # 综合评分 (CTE 权重 40%, 航向 30%, MPC 20%, 速度 10%)
    score_cte = max(0, 100 - avg_cte * 200)       # 0.5m → 0分
    score_he = max(0, 100 - avg_he * 2.5)          # 40° → 0分
    score_mpc = max(0, 100 - avg_mpc * 1.5)        # 66ms → 0分
    score_spd = min(100, avg_speed / 0.3 * 100)    # 0.3m/s → 100分
    total = score_cte * 0.4 + score_he * 0.4 + score_mpc * 0.1 + score_spd * 0.1
    
    total_grade = "优秀" if total >= 80 else "良好" if total >= 60 else "一般" if total >= 40 else "较差"
    print(f"\n   📊 综合评分: {total:.0f}/100 → 【{total_grade}】")


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


class TeeStdout:
    """将 stdout 同时写入多个流"""

    def __init__(self, *streams):
        self._streams = streams

    def write(self, text: str) -> int:
        for stream in self._streams:
            stream.write(text)
        return len(text)

    def flush(self) -> None:
        for stream in self._streams:
            stream.flush()


def analyze_log_file(log_file: Path, batch_mode: bool = False) -> bool:
    if not log_file.exists():
        print(f"❌ 文件不存在: {log_file}")
        return False

    output_path = log_file.parent / log_file.stem
    output_path.mkdir(parents=True, exist_ok=True)

    report_path = output_path / 'analysis_report.txt'
    report_buffer = io.StringIO()
    report_buffer.write("USV nav log analysis report\n")
    report_buffer.write(f"Log file: {log_file}\n")
    report_buffer.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    report_buffer.write(f"Output dir: {output_path}\n")
    report_buffer.write("\n")

    print(f"\n📖 加载日志: {log_file}")
    data, header_info = load_csv(str(log_file))
    print(f"   记录数: {len(data)}")
    report_buffer.write(f"Records: {len(data)}\n\n")

    if len(data) < 10:
        print("⚠️  数据量太少，无法分析")
        if batch_mode:
            report_buffer.write("Data too small, analysis skipped.\n")
            report_path.write_text(report_buffer.getvalue(), encoding='utf-8')
            print(f"   📝 说明文件: {report_path}")
            return False
        return False

    with contextlib.redirect_stdout(TeeStdout(sys.stdout, report_buffer)):
        analyze_statistics(data, header_info)
        find_yaw_offset(data)

        if HAS_MATPLOTLIB:
            print("\n📊 生成图表...")
            plot_trajectory(data, output_path, header_info)
            plot_velocity(data, output_path, header_info)
            plot_heading_comparison(data, output_path)
            plot_control_commands(data, output_path, header_info)
            plot_mpc_debug(data, output_path)
            plot_errors(data, output_path)
            plot_v6_adaptive_tau(data, output_path, header_info)
            plot_v8_ampc(data, output_path, header_info)
            plot_dashboard(data, output_path, header_info)
            plot_per_goal_stats(data, output_path, header_info)
            print(f"\n✅ 图表已保存到: {output_path}")

    report_path.write_text(report_buffer.getvalue(), encoding='utf-8')
    print(f"   📝 说明文件: {report_path}")
    return True


def plot_trajectory(data: list, output_path: Path, header_info: dict = None):
    """绘制轨迹图 - 含模式着色和方向箭头"""
    fig, ax = plt.subplots(figsize=(10, 10))
    
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    
    x_all = [d['pose_x'] for d in data]
    y_all = [d['pose_y'] for d in data]
    
    # 按飞行模式分段着色绘制
    i = 0
    while i < len(data):
        mode = data[i].get('flight_mode', '')
        color = '#2196F3' if mode == 'GUIDED' else '#9E9E9E'
        alpha = 0.9 if mode == 'GUIDED' else 0.4
        lw = 1.5 if mode == 'GUIDED' else 1.0
        j = i
        while j < len(data) and data[j].get('flight_mode', '') == mode:
            j += 1
        end = min(j + 1, len(data))
        seg_x = [d['pose_x'] for d in data[i:end]]
        seg_y = [d['pose_y'] for d in data[i:end]]
        ax.plot(seg_x, seg_y, color=color, linewidth=lw, alpha=alpha)
        i = j
    
    # 起终点
    ax.scatter(x_all[0], y_all[0], c='green', s=150, marker='o', label='Start', zorder=5, edgecolors='black')
    ax.scatter(x_all[-1], y_all[-1], c='red', s=150, marker='s', label='End', zorder=5, edgecolors='black')
    
    # 方向箭头 (GUIDED模式下每隔一定距离画一个)
    arrow_step = max(1, len(data) // 20)
    for idx in range(0, len(data) - 1, arrow_step):
        d = data[idx]
        if d.get('flight_mode') != 'GUIDED' or d.get('velocity_speed', 0) < 0.05:
            continue
        yaw_rad = math.radians(d.get('pose_yaw_deg', 0))
        dx = math.cos(yaw_rad) * 0.3
        dy = math.sin(yaw_rad) * 0.3
        ax.annotate('', xy=(d['pose_x'] + dx, d['pose_y'] + dy),
                    xytext=(d['pose_x'], d['pose_y']),
                    arrowprops=dict(arrowstyle='->', color='#1565C0', lw=1.5))
    
    # 目标点
    targets = []
    for d in data:
        if isinstance(d.get('goal_id'), (int, float)) and d['goal_id'] > 0:
            target = (d['target_x'], d['target_y'], int(d['goal_id']))
            if target not in targets:
                targets.append(target)
    
    for tx, ty, gid in targets:
        ax.scatter(tx, ty, c='orange', s=200, marker='*', zorder=4, edgecolors='darkorange')
        ax.annotate(f'G{gid}', (tx, ty), textcoords='offset points',
                   xytext=(5, 5), fontsize=9, fontweight='bold', color='darkorange')
    
    # 图例
    legend_elements = [
        Line2D([0], [0], color='#2196F3', lw=2, label='GUIDED'),
        Line2D([0], [0], color='#9E9E9E', lw=1, alpha=0.5, label='HOLD'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='Start'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='red', markersize=10, label='End'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='orange', markersize=12, label='Goal'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=9)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'USV Navigation Trajectory - {usv_id}')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    plt.savefig(output_path / 'trajectory.png', dpi=150)
    plt.close()
    print(f"   📈 轨迹图: {output_path / 'trajectory.png'}")


def plot_velocity(data: list, output_path: Path, header_info: dict = None):
    """绘制速度图 - 含指令对比和模式背景"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    # 模式着色背景辅助函数
    def _shade_modes(ax):
        i = 0
        while i < len(data):
            mode = data[i].get('flight_mode', '')
            j = i
            while j < len(data) and data[j].get('flight_mode', '') == mode:
                j += 1
            if mode == 'HOLD':
                ax.axvspan(t[i], t[min(j-1, len(t)-1)], alpha=0.08, color='red')
            i = j
    
    # 1. 速度: 实际 vs 指令
    ax = axes[0]
    _shade_modes(ax)
    speed = [d['velocity_speed'] for d in data]
    ax.plot(t, speed, 'b-', label='Actual Speed', linewidth=1.2)
    if 'cmd_vx' in data[0]:
        cmd_vx = [d.get('cmd_vx', 0) for d in data]
        ax.plot(t, cmd_vx, 'r--', label='Cmd Vx', alpha=0.7, linewidth=1.0)
    ax.set_ylabel('Speed (m/s)')
    ax.set_title(f'Velocity Analysis - {usv_id}')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 2. 速度分量
    ax = axes[1]
    _shade_modes(ax)
    vx = [d['velocity_vx'] for d in data]
    vy = [d['velocity_vy'] for d in data]
    ax.plot(t, vx, 'r-', label='Vx (East)', alpha=0.7)
    ax.plot(t, vy, 'g-', label='Vy (North)', alpha=0.7)
    ax.set_ylabel('Velocity (m/s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 3. 距离目标
    ax = axes[2]
    _shade_modes(ax)
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


def plot_control_commands(data: list, output_path: Path, header_info: dict = None):
    """绘制控制指令图 - 含 omega 跟踪对比"""
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    has_omega = 'omega_actual' in data[0] and 'omega_cmd' in data[0]
    nrows = 4 if has_omega else 3
    fig, axes = plt.subplots(nrows, 1, figsize=(14, 3 * nrows), sharex=True)
    
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    # Vx command vs actual speed
    ax = axes[0]
    cmd_vx = [d['cmd_vx'] for d in data]
    speed = [d['velocity_speed'] for d in data]
    ax.plot(t, cmd_vx, 'r-', label='Cmd Vx', linewidth=1.2)
    ax.plot(t, speed, 'b-', label='Actual Speed', alpha=0.6, linewidth=1.0)
    ax.set_ylabel('Vx (m/s)')
    ax.set_title(f'Control Commands - {usv_id}')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # Vy command
    ax = axes[1]
    cmd_vy = [d['cmd_vy'] for d in data]
    ax.plot(t, cmd_vy, 'g-')
    ax.set_ylabel('Cmd Vy (m/s)')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.grid(True, alpha=0.3)
    
    # cmd_omega
    ax = axes[2]
    cmd_omega = [d['cmd_omega'] for d in data]
    ax.plot(t, cmd_omega, 'r-', label='Cmd Omega')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.set_ylabel('Cmd Omega (rad/s)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # omega_actual vs omega_cmd 跟踪对比 (v5+)
    if has_omega:
        ax = axes[3]
        omega_cmd = [d.get('omega_cmd', 0) for d in data]
        omega_actual = [d.get('omega_actual', 0) for d in data]
        ax.plot(t, omega_cmd, 'r-', label='ω_cmd (MPC)', alpha=0.7, linewidth=1.0)
        ax.plot(t, omega_actual, 'b-', label='ω_actual (estimated)', linewidth=1.2)
        ax.axhline(y=0, color='k', linewidth=0.5)
        ax.set_ylabel('ω (rad/s)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    axes[-1].set_xlabel('Time (s)')
    
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
    
    # 2. 代价函数 (自动对数缩放)
    ax = axes[1]
    costs = [d.get('mpc_cost', 0) for d in data]
    ax.plot(t, costs, 'g-', label='Optimization Cost')
    ax.set_ylabel('Cost')
    # 如果代价范围跨越2个数量级以上，使用对数缩放
    cost_min = min(c for c in costs if c > 0) if any(c > 0 for c in costs) else 1
    cost_max = max(costs) if costs else 1
    if cost_max / max(cost_min, 1e-6) > 100:
        ax.set_yscale('symlog', linthresh=max(cost_min, 1))
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


def plot_v8_ampc(data: list, output_path: Path, header_info: dict = None):
    """绘制 V8 AMPC 在线辨识分析图"""
    if not HAS_MATPLOTLIB:
        return
    
    # 检查是否有 v8 字段
    if 'ampc_tau_estimated' not in data[0]:
        return
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    fig.suptitle(f'AMPC v8 Online Identification - {usv_id}', fontsize=14, fontweight='bold')
    
    # 1. τ 估计值 + 置信度
    ax = axes[0]
    tau_est = [d.get('ampc_tau_estimated', 0) for d in data]
    tau_conf = [d.get('ampc_tau_confidence', 0) for d in data]
    tau_current = [d.get('current_tau_omega', 0) for d in data]
    converged = [d.get('ampc_converged', 0) for d in data]
    
    ax2 = ax.twinx()
    ln1 = ax.plot(t, tau_est, 'b-', label='τ estimated (RLS)', linewidth=1.5)
    ln2 = ax.plot(t, tau_current, 'r--', label='τ active (MPC)', linewidth=1.0, alpha=0.7)
    ln3 = ax2.plot(t, tau_conf, 'g-', label='confidence', alpha=0.6, linewidth=1.0)
    
    # 标记收敛点
    for i in range(1, len(converged)):
        if converged[i] == 1 and converged[i-1] == 0:
            ax.axvline(x=t[i], color='green', linestyle=':', alpha=0.8, linewidth=1.5)
            ax.annotate('converged', xy=(t[i], tau_est[i]),
                       fontsize=8, color='green', ha='left')
    
    ax.set_ylabel('τ_omega (s)', color='blue')
    ax2.set_ylabel('Confidence (0-1)', color='green')
    ax2.set_ylim(-0.05, 1.1)
    
    lns = ln1 + ln2 + ln3
    labs = [l.get_label() for l in lns]
    ax.legend(lns, labs, loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 2. 角速度: 实测 (AMPC) vs 命令
    ax = axes[1]
    omega_measured = [d.get('ampc_omega_measured', 0) for d in data]
    omega_cmd = [d.get('omega_cmd', 0) for d in data]
    omega_actual = [d.get('omega_actual', 0) for d in data]
    
    ax.plot(t, omega_cmd, 'r-', label='ω_cmd', alpha=0.5, linewidth=1.0)
    ax.plot(t, omega_actual, 'b--', label='ω_actual (MPC state)', alpha=0.5, linewidth=1.0)
    ax.plot(t, omega_measured, 'k-', label='ω_measured (observer)', linewidth=1.5)
    ax.axhline(y=0, color='grey', linewidth=0.5)
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 3. 饱和率 + 重建事件
    ax = axes[2]
    sat_ratio = [d.get('ampc_saturation_ratio', 0) for d in data]
    rebuild_count = [d.get('ampc_rebuild_count', 0) for d in data]
    
    ax.fill_between(t, sat_ratio, alpha=0.3, color='orange')
    ax.plot(t, sat_ratio, 'orange', label='saturation ratio', linewidth=1.0)
    ax.axhline(y=0.35, color='red', linestyle='--', alpha=0.5, label='chronic threshold (35%)')
    ax.set_ylabel('Saturation Ratio')
    ax.set_ylim(-0.05, 1.05)
    
    # 标记重建事件
    ax2 = ax.twinx()
    ax2.plot(t, rebuild_count, 'purple', label='rebuild count', linewidth=1.0, alpha=0.7)
    ax2.set_ylabel('Rebuild Count', color='purple')
    
    lns_a = ax.get_lines()
    lns_b = ax2.get_lines()
    labs_all = [l.get_label() for l in lns_a] + [l.get_label() for l in lns_b]
    ax.legend(list(lns_a) + list(lns_b), labs_all, loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 4. 航向噪声 + CTE
    ax = axes[3]
    noise = [d.get('ampc_heading_noise', 0) for d in data]
    cte = [d.get('cross_track_error', 0) for d in data]
    
    ax2 = ax.twinx()
    ln1 = ax.plot(t, noise, 'darkorange', label='heading noise (std)', linewidth=1.0)
    ax.axhline(y=0.05, color='darkorange', linestyle=':', alpha=0.5, label='noise baseline (0.05)')
    ln2 = ax2.plot(t, cte, 'purple', label='CTE', alpha=0.7, linewidth=1.0)
    ax2.axhline(y=0, color='grey', linewidth=0.5)
    
    ax.set_ylabel('Heading Noise (rad/s)', color='darkorange')
    ax2.set_ylabel('CTE (m)', color='purple')
    ax.set_xlabel('Time (s)')
    
    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax.legend(lns, labs, loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path / 'v8_ampc_analysis.png', dpi=150)
    plt.close()
    print(f"   📈 V8 AMPC 分析图: {output_path / 'v8_ampc_analysis.png'}")


def plot_dashboard(data: list, output_path: Path, header_info: dict = None):
    """绘制综合仪表盘 - 一页总览所有关键指标"""
    if not HAS_MATPLOTLIB:
        return
    
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    version = header_info.get('version', '?') if header_info else '?'
    
    fig = plt.figure(figsize=(18, 14))
    fig.suptitle(f'USV Navigation Dashboard - {usv_id} ({version})', fontsize=16, fontweight='bold')
    gs = gridspec.GridSpec(3, 3, hspace=0.35, wspace=0.3)
    
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    guided = [d for d in data if d.get('flight_mode') == 'GUIDED']
    duration = t[-1] if t else 0
    
    # ─── 1. 轨迹 (左上) ───
    ax = fig.add_subplot(gs[0, 0])
    x = [d['pose_x'] for d in data]
    y = [d['pose_y'] for d in data]
    # 按模式着色
    i = 0
    while i < len(data):
        mode = data[i].get('flight_mode', '')
        color = '#2196F3' if mode == 'GUIDED' else '#BDBDBD'
        j = i
        while j < len(data) and data[j].get('flight_mode', '') == mode:
            j += 1
        end = min(j + 1, len(data))
        ax.plot([d['pose_x'] for d in data[i:end]], [d['pose_y'] for d in data[i:end]],
                color=color, linewidth=1)
        i = j
    # 目标点
    targets = []
    for d in data:
        if isinstance(d.get('goal_id'), (int, float)) and d['goal_id'] > 0:
            tgt = (d['target_x'], d['target_y'], int(d['goal_id']))
            if tgt not in targets:
                targets.append(tgt)
    for tx, ty, gid in targets:
        ax.plot(tx, ty, '*', color='orange', markersize=8)
    ax.plot(x[0], y[0], 'go', markersize=8)
    ax.plot(x[-1], y[-1], 'rs', markersize=8)
    ax.set_title('Trajectory', fontsize=10)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # ─── 2. 速度 (中上) ───
    ax = fig.add_subplot(gs[0, 1])
    speed = [d['velocity_speed'] for d in data]
    ax.plot(t, speed, 'b-', linewidth=0.8, label='Speed')
    if 'cmd_vx' in data[0]:
        ax.plot(t, [d.get('cmd_vx', 0) for d in data], 'r--', linewidth=0.8, alpha=0.7, label='Cmd Vx')
    ax.set_title('Speed', fontsize=10)
    ax.set_ylabel('m/s')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # ─── 3. CTE (右上) ───
    ax = fig.add_subplot(gs[0, 2])
    if 'cross_track_error' in data[0]:
        cte = [d.get('cross_track_error', 0) for d in data]
        ax.plot(t, cte, 'purple', linewidth=0.8)
        ax.fill_between(t, cte, alpha=0.2, color='purple')
        ax.axhline(y=0, color='k', linewidth=0.5)
    ax.set_title('Cross Track Error', fontsize=10)
    ax.set_ylabel('m')
    ax.grid(True, alpha=0.3)
    
    # ─── 4. 距离到目标 (左中) ───
    ax = fig.add_subplot(gs[1, 0])
    dist = [d.get('distance_to_goal', 0) for d in data]
    ax.plot(t, dist, 'purple', linewidth=0.8)
    ax.axhline(y=1.5, color='r', linestyle='--', linewidth=0.8, label='Threshold')
    ax.set_title('Distance to Goal', fontsize=10)
    ax.set_ylabel('m')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    
    # ─── 5. 航向误差 (中中) ───
    ax = fig.add_subplot(gs[1, 1])
    if 'heading_error_deg' in data[0]:
        he = [d.get('heading_error_deg', 0) for d in data]
        ax.plot(t, he, 'b-', linewidth=0.8)
        ax.axhline(y=0, color='k', linewidth=0.5)
    ax.set_title('Heading Error', fontsize=10)
    ax.set_ylabel('deg')
    ax.grid(True, alpha=0.3)
    
    # ─── 6. MPC求解时间 (右中) ───
    ax = fig.add_subplot(gs[1, 2])
    if 'mpc_solve_time_ms' in data[0]:
        st = [d.get('mpc_solve_time_ms', 0) for d in data]
        ax.plot(t, st, 'b-', linewidth=0.8)
        ax.axhline(y=50, color='r', linestyle='--', linewidth=0.8)
    ax.set_title('MPC Solve Time', fontsize=10)
    ax.set_ylabel('ms')
    ax.grid(True, alpha=0.3)
    
    # ─── 7. omega 跟踪 (左下) ───
    ax = fig.add_subplot(gs[2, 0])
    if 'omega_cmd' in data[0] and 'omega_actual' in data[0]:
        ax.plot(t, [d.get('omega_cmd', 0) for d in data], 'r-', linewidth=0.8, alpha=0.7, label='ω_cmd')
        ax.plot(t, [d.get('omega_actual', 0) for d in data], 'b-', linewidth=0.8, label='ω_actual')
        ax.axhline(y=0, color='k', linewidth=0.5)
        ax.legend(fontsize=7)
    ax.set_title('Angular Velocity', fontsize=10)
    ax.set_ylabel('rad/s')
    ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)
    
    # ─── 8. Tau (中下) ───
    ax = fig.add_subplot(gs[2, 1])
    if 'current_tau_omega' in data[0]:
        tau = [d.get('current_tau_omega', 0) for d in data]
        ax.plot(t, tau, 'b-', linewidth=1.2, label='τ_omega')
        if 'ampc_tau_estimated' in data[0]:
            ax.plot(t, [d.get('ampc_tau_estimated', 0) for d in data], 'r--', 
                    linewidth=0.8, alpha=0.7, label='τ_est (AMPC)')
        ax.legend(fontsize=7)
    ax.set_title('Tau Omega', fontsize=10)
    ax.set_ylabel('s')
    ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)
    
    # ─── 9. 统计摘要 (右下) ───
    ax = fig.add_subplot(gs[2, 2])
    ax.axis('off')
    
    # 计算统计数据
    g_speeds = [d['velocity_speed'] for d in guided] if guided else [0]
    g_ctes = [abs(d.get('cross_track_error', 0)) for d in guided] if guided else [0]
    g_hes = [abs(d.get('heading_error_deg', 0)) for d in guided] if guided else [0]
    g_mpc = [d.get('mpc_solve_time_ms', 0) for d in data if isinstance(d.get('mpc_solve_time_ms'), (int,float))]
    
    avg_cte = sum(g_ctes) / len(g_ctes)
    avg_he = sum(g_hes) / len(g_hes)
    avg_spd = sum(g_speeds) / len(g_speeds)
    avg_mpc = sum(g_mpc) / len(g_mpc) if g_mpc else 0
    
    # 质量评分
    s_cte = max(0, 100 - avg_cte * 200)
    s_he = max(0, 100 - avg_he * 2.5)
    s_mpc = max(0, 100 - avg_mpc * 1.5)
    s_spd = min(100, avg_spd / 0.3 * 100)
    total = s_cte * 0.4 + s_he * 0.4 + s_mpc * 0.1 + s_spd * 0.1
    grade = "A+" if total >= 80 else "B" if total >= 60 else "C" if total >= 40 else "D"
    
    stats_text = (
        f"━━━ Summary ━━━\n"
        f"Duration:  {duration:.1f}s ({duration/60:.1f}min)\n"
        f"Records:   {len(data)}\n"
        f"Goals:     {len(targets)}\n"
        f"GUIDED:    {len(guided)/len(data)*100:.0f}%\n"
        f"\n━━━ GUIDED Stats ━━━\n"
        f"Avg Speed: {avg_spd:.3f} m/s\n"
        f"Avg CTE:   {avg_cte:.4f} m\n"
        f"Avg HdgErr:{avg_he:.1f}°\n"
        f"MPC Time:  {avg_mpc:.1f} ms\n"
        f"\n━━━ Score ━━━\n"
        f"Total: {total:.0f}/100 [{grade}]"
    )
    ax.text(0.1, 0.95, stats_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    
    plt.savefig(output_path / 'dashboard.png', dpi=150)
    plt.close()
    print(f"   📈 综合仪表盘: {output_path / 'dashboard.png'}")


def plot_per_goal_stats(data: list, output_path: Path, header_info: dict = None):
    """绘制每航点统计柱状图"""
    if not HAS_MATPLOTLIB:
        return
    
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    
    # 按 goal_id 分组
    goals = {}
    for d in data:
        gid = d.get('goal_id')
        if not isinstance(gid, (int, float)):
            continue
        gid = int(gid)
        if gid not in goals:
            goals[gid] = []
        goals[gid].append(d)
    
    if len(goals) < 2:
        return
    
    gids = sorted(goals.keys())
    durations = []
    min_dists = []
    avg_ctes = []
    avg_hes = []
    avg_speeds = []
    
    for gid in gids:
        gdata = goals[gid]
        dur = gdata[-1]['timestamp'] - gdata[0]['timestamp']
        durations.append(dur)
        
        dists = [d.get('distance_to_goal', 999) for d in gdata if isinstance(d.get('distance_to_goal'), (int, float))]
        min_dists.append(min(dists) if dists else float('nan'))
        
        g_guided = [d for d in gdata if d.get('flight_mode') == 'GUIDED']
        ctes = [abs(d.get('cross_track_error', 0)) for d in g_guided if isinstance(d.get('cross_track_error'), (int, float))]
        avg_ctes.append(sum(ctes)/len(ctes) if ctes else 0)
        
        hes = [abs(d.get('heading_error_deg', 0)) for d in g_guided if isinstance(d.get('heading_error_deg'), (int, float))]
        avg_hes.append(sum(hes)/len(hes) if hes else 0)
        
        spds = [d.get('velocity_speed', 0) for d in g_guided if isinstance(d.get('velocity_speed'), (int, float))]
        avg_speeds.append(sum(spds)/len(spds) if spds else 0)
    
    x = range(len(gids))
    labels = [f'G{g}' for g in gids]
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Per-Goal Statistics - {usv_id}', fontsize=14, fontweight='bold')
    
    # 1. CTE
    ax = axes[0, 0]
    bars = ax.bar(x, avg_ctes, color='#7E57C2', alpha=0.8)
    ax.axhline(y=0.3, color='orange', linestyle='--', linewidth=1, label='Good threshold')
    ax.set_ylabel('Avg |CTE| (m)')
    ax.set_title('Cross Track Error')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, fontsize=8)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, axis='y')
    # 超标着红
    for bar, v in zip(bars, avg_ctes):
        if v > 0.5:
            bar.set_color('#E53935')
    
    # 2. 航向误差
    ax = axes[0, 1]
    bars = ax.bar(x, avg_hes, color='#42A5F5', alpha=0.8)
    ax.axhline(y=10, color='orange', linestyle='--', linewidth=1, label='Good threshold')
    ax.set_ylabel('Avg |Heading Error| (°)')
    ax.set_title('Heading Error')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, fontsize=8)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, axis='y')
    for bar, v in zip(bars, avg_hes):
        if v > 30:
            bar.set_color('#E53935')
    
    # 3. 最近到达距离
    ax = axes[1, 0]
    bars = ax.bar(x, min_dists, color='#66BB6A', alpha=0.8)
    ax.axhline(y=1.5, color='red', linestyle='--', linewidth=1, label='Arrival threshold')
    ax.set_ylabel('Min Distance (m)')
    ax.set_title('Closest Approach')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, fontsize=8)
    ax.set_xlabel('Goal ID')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, axis='y')
    
    # 4. 速度
    ax = axes[1, 1]
    ax.bar(x, avg_speeds, color='#FFA726', alpha=0.8)
    ax.set_ylabel('Avg Speed (m/s)')
    ax.set_title('Average Speed (GUIDED)')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, fontsize=8)
    ax.set_xlabel('Goal ID')
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_path / 'per_goal_stats.png', dpi=150)
    plt.close()
    print(f"   📈 每航点统计: {output_path / 'per_goal_stats.png'}")


def main():
    # 确定日志路径
    if len(sys.argv) > 1:
        input_path = Path(sys.argv[1])
    else:
        log_dir = Path.home() / 'usv_logs'
        if not log_dir.exists():
            print("❌ 未找到日志目录: ~/usv_logs")
            sys.exit(1)
        log_files = sorted(log_dir.rglob('nav_log_*.csv'))
        if not log_files:
            print("❌ 未找到日志文件")
            sys.exit(1)
        input_path = log_files[-1]
        print(f"📂 使用最新日志: {input_path}")

    if not input_path.exists():
        print(f"❌ 路径不存在: {input_path}")
        sys.exit(1)

    if input_path.is_dir():
        log_files = sorted(input_path.rglob('nav_log_*.csv'))
        if not log_files:
            print(f"❌ 目录下未找到日志文件: {input_path}")
            sys.exit(1)
    else:
        log_files = [input_path]

    batch_mode = len(log_files) > 1
    for idx, log_file in enumerate(log_files, start=1):
        if batch_mode:
            print("\n" + "="*60)
            print(f"[{idx}/{len(log_files)}] {log_file}")
            print("="*60)
        analyze_log_file(log_file, batch_mode=batch_mode)

    print("\n" + "="*60)
    print("分析完成!")
    print("="*60)


if __name__ == '__main__':
    main()
