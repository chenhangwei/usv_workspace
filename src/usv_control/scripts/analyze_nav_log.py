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
    7. adaptive_tau.png    - 自适应 Tau 分析
    8. ampc_analysis.png    - AMPC 在线辨识分析
    9. dashboard.png        - 综合仪表盘 (一页总览)
    10. per_goal_stats.png  - 每航点统计柱状图
    11. orca_analysis.png   - ORCA/APF 避障分析 (v14+)

作者: chenhangwei
日期: 2026-02-10
Updated: 2026-02-27 - v14 新增 ORCA/APF 避障分析和 nav_mode 分析
"""

import sys
import csv
import math
import io
import contextlib
from pathlib import Path
from datetime import datetime
from typing import Any, Optional

# HTML 交互式报告生成器
try:
    from nav_report_html import extract_report_data, generate_html_report, generate_multi_html_report
    HAS_HTML_REPORT = True
except ImportError:
    try:
        # 当脚本从其他目录调用时，尝试相对路径导入
        import importlib.util
        _spec = importlib.util.spec_from_file_location(
            'nav_report_html', Path(__file__).parent / 'nav_report_html.py')
        if _spec and _spec.loader:
            _mod = importlib.util.module_from_spec(_spec)
            _spec.loader.exec_module(_mod)
            extract_report_data = _mod.extract_report_data
            generate_html_report = _mod.generate_html_report
            generate_multi_html_report = _mod.generate_multi_html_report
            HAS_HTML_REPORT = True
        else:
            HAS_HTML_REPORT = False
    except Exception:
        HAS_HTML_REPORT = False

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
    # 中文字体优先级：先尝试中文字体，再回退到英文字体
    # 系统已安装 WenQuanYi Micro Hei，必须排在 DejaVu Sans 前面
    _zh_fonts = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'SimHei',
                 'Noto Sans CJK SC', 'Microsoft YaHei', 'DejaVu Sans', 'Arial']
    plt.rcParams['font.sans-serif'] = _zh_fonts
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['figure.dpi'] = 120
    plt.rcParams['savefig.bbox'] = 'tight'
    # 清除 matplotlib 字体缓存（首次修正字体后可能需要）
    try:
        from matplotlib.font_manager import fontManager
        fontManager._load_fontmanager(try_read_cache=False) if hasattr(fontManager, '_load_fontmanager') else None
    except Exception:
        pass


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
    
    # 检测 v6/v8/v14 特有字段
    if data and 'current_tau_omega' in data[0]:
        if header_info['version'] == 'v5':
            header_info['version'] = 'v6'
    if data and 'ampc_tau_estimated' in data[0]:
        header_info['version'] = 'v8'
    if data and ('orca_active' in data[0] or 'wifi_rssi_dbm' in data[0]):
        header_info['version'] = 'v14'

    # 从文件路径提取 USV ID (如 .../usv_02/xxx.csv)
    if header_info['usv_id'] == 'unknown':
        import re as _re
        fp = str(filepath)
        # 尝试从路径中匹配 usv_XX
        m = _re.search(r'(usv_\d+)', fp)
        if m:
            header_info['usv_id'] = m.group(1)
        else:
            # 尝试从 goal_id 推断: 200001 → usv_02, 300001 → usv_03
            m2 = _re.search(r'goal_(\d)', fp)
            if m2:
                header_info['usv_id'] = f'usv_{m2.group(1).zfill(2)}'

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

    # WiFi 信号统计
    _print_wifi_stats(data)
    
    # 每航点统计
    _print_per_goal_stats(data)
    
    # v14 ORCA/APF 避障统计
    _print_orca_stats(data)
    
    # v14 导航模式统计
    _print_nav_mode_stats(data)
    
    # 质量评分总结
    _print_quality_score(data)


def _print_wifi_stats(data: list):
    """打印 WiFi 信号强度统计"""
    if 'wifi_rssi_dbm' not in data[0]:
        return
    
    wifi_values = [d.get('wifi_rssi_dbm', -100) for d in data
                   if isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100]
    
    if not wifi_values:
        return
    
    avg_rssi = sum(wifi_values) / len(wifi_values)
    min_rssi = min(wifi_values)
    max_rssi = max(wifi_values)
    
    # 标准差
    variance = sum((x - avg_rssi) ** 2 for x in wifi_values) / len(wifi_values)
    std_rssi = math.sqrt(variance)
    
    # 信号质量等级分布
    excellent = sum(1 for v in wifi_values if v >= -50)     # 优秀
    good = sum(1 for v in wifi_values if -60 <= v < -50)    # 良好
    fair = sum(1 for v in wifi_values if -70 <= v < -60)    # 一般
    weak = sum(1 for v in wifi_values if v < -70)           # 较弱
    total = len(wifi_values)
    
    # 链路质量
    wifi_quality = [d.get('wifi_link_quality', 0) for d in data
                    if isinstance(d.get('wifi_link_quality'), (int, float)) and d.get('wifi_rssi_dbm', -100) > -100]
    avg_quality = sum(wifi_quality) / len(wifi_quality) if wifi_quality else 0
    
    print(f"\n{'='*60}")
    print("📶 WiFi 信号强度统计")
    print("=" * 60)
    print(f"   有效样本: {total}/{len(data)} ({total/len(data)*100:.1f}%)")
    print(f"   平均 RSSI: {avg_rssi:.1f} dBm")
    print(f"   最强信号: {max_rssi:.0f} dBm")
    print(f"   最弱信号: {min_rssi:.0f} dBm")
    print(f"   信号波动: ±{std_rssi:.1f} dBm")
    print(f"   平均链路质量: {avg_quality:.0f}%")
    print(f"\n   信号质量分布:")
    print(f"   • 优秀 (≥-50dBm): {excellent:>5} ({excellent/total*100:>5.1f}%)")
    print(f"   • 良好 (-60~-50):  {good:>5} ({good/total*100:>5.1f}%)")
    print(f"   • 一般 (-70~-60):  {fair:>5} ({fair/total*100:>5.1f}%)")
    print(f"   • 较弱 (<-70dBm): {weak:>5} ({weak/total*100:>5.1f}%)")
    
    # 信号质量评价
    if avg_rssi >= -50:
        grade = "优秀"
    elif avg_rssi >= -60:
        grade = "良好"
    elif avg_rssi >= -70:
        grade = "一般"
    else:
        grade = "较弱"
    print(f"\n   📊 信号评级: 【{grade}】")
    
    if weak / total > 0.1:
        print(f"   ⚠️  较弱信号占比 {weak/total*100:.1f}%，建议优化天线位置或缩短通信距离")
    if std_rssi > 8:
        print(f"   ⚠️  信号波动较大 (±{std_rssi:.1f}dBm)，可能存在间歇遮挡")

    # 每航点WiFi统计
    goals = {}
    for d in data:
        gid = d.get('goal_id')
        if isinstance(gid, (int, float)) and isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100:
            gid = int(gid)
            if gid not in goals:
                goals[gid] = []
            goals[gid].append(d['wifi_rssi_dbm'])
    
    if len(goals) > 1:
        print(f"\n   每航点平均信号:")
        for gid in sorted(goals.keys()):
            vals = goals[gid]
            g_avg = sum(vals) / len(vals)
            g_min = min(vals)
            print(f"   • Goal {gid:>3}: 平均 {g_avg:.1f}dBm, 最弱 {g_min:.0f}dBm")


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
    
    has_wifi = 'wifi_rssi_dbm' in data[0]
    
    print(f"\n{'='*60}")
    print("📋 每航点详细统计")
    print("=" * 60)
    header = (f"   {'GoalID':>6} {'时长(s)':>8} {'最近距(m)':>9} {'均CTE(m)':>9} "
              f"{'均航向误差°':>10} {'均速(m/s)':>9} {'模式':>8}")
    if has_wifi:
        header += f" {'信号(dBm)':>10}"
    print(header)
    
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
        
        line = (f"   {gid:>6} {dur:>8.1f} {min_dist:>9.3f} {avg_cte:>9.4f} "
                f"{avg_he:>10.1f} {avg_spd:>9.3f} {mode_str:>8}")
        
        if has_wifi:
            wifi_vals = [d.get('wifi_rssi_dbm', -100) for d in gdata
                         if isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100]
            if wifi_vals:
                avg_wifi = sum(wifi_vals) / len(wifi_vals)
                line += f" {avg_wifi:>10.1f}"
            else:
                line += f" {'N/A':>10}"
        
        print(line)


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


def _print_orca_stats(data: list):
    """打印 ORCA/APF 避障统计 (v14+)"""
    if 'orca_active' not in data[0]:
        return

    orca_active_samples = [d for d in data if d.get('orca_active') == 1]
    total = len(data)
    active_count = len(orca_active_samples)

    if active_count == 0:
        print(f"\n{'='*60}")
        print("🛡️ ORCA/APF 避障统计")
        print("=" * 60)
        print(f"   ORCA 未触发 (0/{total} = 0%)")
        return

    print(f"\n{'='*60}")
    print("🛡️ ORCA/APF 避障统计")
    print("=" * 60)
    print(f"\n   ORCA 激活: {active_count}/{total} 次 ({active_count/total*100:.1f}%)")

    # 最近距离统计
    distances = [d.get('orca_closest_distance', -1) for d in orca_active_samples
                 if isinstance(d.get('orca_closest_distance'), (int, float)) and d['orca_closest_distance'] > 0]
    if distances:
        print(f"   最近邻居距离: 最小={min(distances):.2f}m, 平均={sum(distances)/len(distances):.2f}m")

    # 计算 ORCA 持续时间段
    duration_total = 0.0
    episode_count = 0
    i = 0
    while i < len(data):
        if data[i].get('orca_active') == 1:
            episode_start = data[i]['timestamp']
            j = i
            while j < len(data) and data[j].get('orca_active') == 1:
                j += 1
            episode_end = data[j - 1]['timestamp']
            duration_total += (episode_end - episode_start)
            episode_count += 1
            i = j
        else:
            i += 1
    if episode_count > 0:
        total_time = data[-1]['timestamp'] - data[0]['timestamp']
        print(f"   激活段数: {episode_count} 段, 总时长={duration_total:.1f}s ({duration_total/total_time*100:.1f}%)")
        print(f"   平均每段: {duration_total/episode_count:.1f}s")

    # 遭遇类型分布
    encounters = {}
    for d in orca_active_samples:
        etype = d.get('orca_encounter_type', 'none')
        if isinstance(etype, str) and etype != 'none' and etype != 'other':
            encounters[etype] = encounters.get(etype, 0) + 1
    if encounters:
        print(f"\n   遭遇类型 (COLREGS):")
        for etype, count in sorted(encounters.items(), key=lambda x: -x[1]):
            print(f"   - {etype}: {count} ({count/active_count*100:.1f}%)")

    # 选边统计
    sides = [d.get('orca_commit_side', 0) for d in orca_active_samples
             if isinstance(d.get('orca_commit_side'), (int, float)) and d['orca_commit_side'] != 0]
    if sides:
        left = sum(1 for s in sides if s > 0)
        right = sum(1 for s in sides if s < 0)
        print(f"\n   选边决策: 左转(+1)={left}, 右转(-1)={right}")

    # 修正量统计
    lin_corrs = [d.get('orca_linear_correction', 0) for d in orca_active_samples
                 if isinstance(d.get('orca_linear_correction'), (int, float))]
    ang_corrs = [d.get('orca_angular_correction', 0) for d in orca_active_samples
                 if isinstance(d.get('orca_angular_correction'), (int, float))]
    if lin_corrs:
        print(f"\n   线速度修正: 均值={sum(lin_corrs)/len(lin_corrs):.4f} m/s, "
              f"最大减速={min(lin_corrs):.4f} m/s")
    if ang_corrs:
        print(f"   角速度修正: 均值={sum(ang_corrs)/len(ang_corrs):.4f} rad/s, "
              f"最大={max(abs(a) for a in ang_corrs):.4f} rad/s")

    # 硬刹车统计
    hard_brakes = [d for d in data if d.get('orca_hard_brake') == 1]
    if hard_brakes:
        print(f"\n   ⚠️ 硬刹车触发: {len(hard_brakes)} 次 ({len(hard_brakes)/total*100:.1f}%)")

    # 邻居数统计
    neighbor_counts = [d.get('apf_neighbor_count', 0) for d in orca_active_samples
                       if isinstance(d.get('apf_neighbor_count'), (int, float))]
    if neighbor_counts:
        max_neighbors = max(neighbor_counts)
        print(f"   最大同时邻居数: {max_neighbors}")


def _print_nav_mode_stats(data: list):
    """打印导航模式统计 (v14+)"""
    if 'nav_mode' not in data[0]:
        return
    
    mode_names = {0: 'async(异步)', 1: 'sync(同步)', 2: 'rotate(旋转)', 3: 'terminal(末端)'}
    modes = {}
    for d in data:
        m = d.get('nav_mode')
        if isinstance(m, (int, float)):
            m = int(m)
            modes[m] = modes.get(m, 0) + 1
    
    if not modes or (len(modes) == 1 and 0 in modes):
        return  # 全部是 async(0)，不用特意显示
    
    total = len(data)
    print(f"\n{'='*60}")
    print("🔄 导航模式统计 (v14)")
    print("=" * 60)
    for m in sorted(modes.keys()):
        name = mode_names.get(m, f'unknown({m})')
        count = modes[m]
        print(f"   {name}: {count} ({count/total*100:.1f}%)")

    # 同步模式等待时间分析
    if 1 in modes:
        # 找同步模式的航点段，估算等待时间（HOLD期间的时长）
        sync_goals = {}
        for d in data:
            if int(d.get('nav_mode', 0)) == 1 and isinstance(d.get('goal_id'), (int, float)):
                gid = int(d['goal_id'])
                if gid not in sync_goals:
                    sync_goals[gid] = []
                sync_goals[gid].append(d)
        
        if sync_goals:
            print(f"\n   同步航点详情:")
            for gid in sorted(sync_goals.keys()):
                gdata = sync_goals[gid]
                hold_time = sum(1 for d in gdata if d.get('flight_mode') == 'HOLD') * 0.1  # 10Hz采样
                total_time = gdata[-1]['timestamp'] - gdata[0]['timestamp'] if len(gdata) > 1 else 0
                print(f"   - Goal {gid}: 总时长={total_time:.1f}s, HOLD等待={hold_time:.1f}s")


def _add_reading_guide(fig, text: str, bottom: float = 0.10):
    """在图表底部添加阅读指南说明框

    Args:
        fig: matplotlib figure
        text: 阅读指南文字（支持换行）
        bottom: 为指南预留的底部空间比例 (0-1)
    """
    fig.subplots_adjust(bottom=bottom)
    fig.text(
        0.5, 0.005, text,
        ha='center', va='bottom', fontsize=7.5,
        color='#444444', linespacing=1.6,
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#FFFDE7',
                  alpha=0.95, edgecolor='#E0E0E0'),
    )


def plot_orca_analysis(data: list, output_path: Path, header_info: dict = None):
    """绘制 ORCA/APF 避障分析图 (v14+)"""
    if not HAS_MATPLOTLIB:
        return
    
    if 'orca_active' not in data[0]:
        return
    
    # 检查是否有 ORCA 触发
    any_active = any(d.get('orca_active') == 1 for d in data)
    if not any_active:
        return
    
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    
    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(f'ORCA/APF Avoidance Analysis - {usv_id}', fontsize=14, fontweight='bold')
    
    # 1. 最近邻居距离 + ORCA 激活状态背景
    ax = axes[0]
    orca_active = [d.get('orca_active', 0) for d in data]
    closest_dist = [d.get('orca_closest_distance', -1) for d in data]
    # 将 -1 替换为 NaN 以避免绘制
    closest_dist_plot = [d if d > 0 else float('nan') for d in closest_dist]
    
    # ORCA 激活区域背景着色
    i = 0
    while i < len(data):
        if orca_active[i] == 1:
            j = i
            while j < len(data) and orca_active[j] == 1:
                j += 1
            ax.axvspan(t[i], t[min(j-1, len(t)-1)], alpha=0.15, color='red', label='ORCA active' if i == 0 or not any(orca_active[k] == 1 for k in range(i)) else '')
            i = j
        else:
            i += 1
    
    ax.plot(t, closest_dist_plot, 'b-', linewidth=1.2, label='Closest Distance')
    if 'apf_orca_min_separation' in str(header_info):
        ax.axhline(y=1.0, color='red', linestyle='--', linewidth=0.8, label='Min Separation')
    ax.axhline(y=4.0, color='orange', linestyle=':', linewidth=0.8, label='Influence Distance')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Nearest Neighbor Distance')
    ax.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)
    
    # 2. 遭遇类型 (分类散点)
    ax = axes[1]
    encounter_map = {'head_on': 3, 'crossing': 2, 'overtaking': 1, 'other': 0.5, 'none': 0}
    encounter_colors = {'head_on': 'red', 'crossing': 'orange', 'overtaking': 'blue', 'other': 'gray', 'none': 'lightgray'}
    
    for etype, yval in encounter_map.items():
        indices = [idx for idx, d in enumerate(data) if d.get('orca_encounter_type') == etype and d.get('orca_active') == 1]
        if indices:
            ax.scatter([t[i] for i in indices], [yval] * len(indices),
                      c=encounter_colors.get(etype, 'gray'), s=3, alpha=0.6, label=etype)
    
    ax.set_ylabel('Encounter Type')
    ax.set_yticks(list(encounter_map.values()))
    ax.set_yticklabels(list(encounter_map.keys()), fontsize=8)
    ax.set_title('COLREGS Encounter Classification')
    ax.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3, axis='x')
    
    # 3. 选边方向 + 硬刹车标记
    ax = axes[2]
    commit_side = [d.get('orca_commit_side', 0) for d in data]
    hard_brake = [d.get('orca_hard_brake', 0) for d in data]
    
    # 只绘制 ORCA 激活时的选边
    active_t = [t[i] for i in range(len(data)) if orca_active[i] == 1]
    active_side = [commit_side[i] for i in range(len(data)) if orca_active[i] == 1]
    if active_t:
        colors = ['green' if s > 0 else 'red' if s < 0 else 'gray' for s in active_side]
        ax.scatter(active_t, active_side, c=colors, s=4, alpha=0.5)
    
    # 硬刹车标记
    brake_t = [t[i] for i in range(len(data)) if hard_brake[i] == 1]
    if brake_t:
        ax.scatter(brake_t, [0] * len(brake_t), c='black', s=20, marker='x', zorder=5, label='Hard Brake')
    
    ax.set_ylabel('Side')
    ax.set_yticks([-1, 0, 1])
    ax.set_yticklabels(['Right(-1)', 'None', 'Left(+1)'])
    ax.set_title('Commit Side & Hard Brake Events')
    ax.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)
    
    # 4. 线速度修正量
    ax = axes[3]
    lin_corr = [d.get('orca_linear_correction', 0) for d in data]
    ang_corr = [d.get('orca_angular_correction', 0) for d in data]
    
    ax.plot(t, lin_corr, 'r-', linewidth=0.8, alpha=0.8, label='Linear Correction (m/s)')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.fill_between(t, lin_corr, alpha=0.2, color='red')
    ax.set_ylabel('Linear Corr (m/s)')
    ax.set_title('Speed Correction (negative = deceleration)')
    ax.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)
    
    # 5. 角速度修正量
    ax = axes[4]
    ax.plot(t, ang_corr, 'b-', linewidth=0.8, alpha=0.8, label='Angular Correction (rad/s)')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.fill_between(t, ang_corr, alpha=0.2, color='blue')
    ax.set_ylabel('Angular Corr (rad/s)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Heading Correction')
    ax.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)
    
    _add_reading_guide(fig,
        '【阅读指南】第1行: 蓝=最近邻居距离，红色背景=ORCA避障激活中; 距离>4m安全，<1m危险。\n'
        '第2行: 遇见类型分类—head_on(红)=对向、crossing(橙)=交叉、overtaking(蓝)=超越。\n'
        '第3行: 选边+1=左避，-1=右避，X=紧急制动。 第4行: 红填充=线速度修正(负值=减速)。 第5行: 蓝填充=角速度修正(避让转向)。',
        bottom=0.06)
    plt.savefig(output_path / 'orca_analysis.png', dpi=150)
    plt.close()
    print(f"   📈 ORCA 避障分析图: {output_path / 'orca_analysis.png'}")


def plot_wifi_signal(data: list, output_path: Path, header_info: dict = None):
    """绘制 WiFi 信号强度分析图"""
    if not HAS_MATPLOTLIB:
        return
    
    if 'wifi_rssi_dbm' not in data[0]:
        return
    
    # 检查是否有有效 WiFi 数据
    wifi_valid = [d for d in data if isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100]
    if not wifi_valid:
        return
    
    usv_id = header_info.get('usv_id', 'unknown') if header_info else 'unknown'
    t = [d['timestamp'] - data[0]['timestamp'] for d in data]
    t0 = data[0]['timestamp']
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=False)
    fig.suptitle(f'WiFi Signal Analysis - {usv_id}', fontsize=14, fontweight='bold')
    
    # ─── 1. WiFi RSSI 时序图 + 模式背景 ───
    ax = axes[0]
    # 模式着色背景
    i = 0
    while i < len(data):
        mode = data[i].get('flight_mode', '')
        j = i
        while j < len(data) and data[j].get('flight_mode', '') == mode:
            j += 1
        if mode == 'HOLD':
            ax.axvspan(t[i], t[min(j-1, len(t)-1)], alpha=0.08, color='red')
        i = j
    
    rssi = [d.get('wifi_rssi_dbm', -100) for d in data]
    rssi_plot = [r if r > -100 else float('nan') for r in rssi]
    ax.plot(t, rssi_plot, 'b-', linewidth=1.0, alpha=0.8, label='WiFi RSSI')
    
    # 信号质量参考线
    ax.axhline(y=-50, color='green', linestyle='--', linewidth=0.8, alpha=0.6, label='优秀 (-50dBm)')
    ax.axhline(y=-60, color='#FFC107', linestyle='--', linewidth=0.8, alpha=0.6, label='良好 (-60dBm)')
    ax.axhline(y=-70, color='orange', linestyle='--', linewidth=0.8, alpha=0.6, label='一般 (-70dBm)')
    ax.axhline(y=-80, color='red', linestyle='--', linewidth=0.8, alpha=0.6, label='较弱 (-80dBm)')
    
    # 标注目标点切换时刻
    prev_gid = None
    for idx, d in enumerate(data):
        gid = d.get('goal_id')
        if isinstance(gid, (int, float)) and gid != prev_gid and prev_gid is not None:
            ax.axvline(x=t[idx], color='gray', linestyle=':', linewidth=0.8, alpha=0.5)
            ax.text(t[idx], ax.get_ylim()[1], f'G{int(gid)}', fontsize=7, ha='left', va='top', color='gray')
        prev_gid = gid
    
    ax.set_ylabel('RSSI (dBm)')
    ax.set_xlabel('Time (s)')
    ax.set_title('WiFi Signal Strength Over Time')
    ax.legend(loc='lower left', fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    
    # ─── 2. WiFi RSSI 分布直方图 ───
    ax = axes[1]
    valid_rssi = [r for r in rssi if r > -100]
    if valid_rssi:
        n, bins, patches_hist = ax.hist(valid_rssi, bins=30, color='#42A5F5', alpha=0.7, edgecolor='white')
        # 按信号质量着色
        for patch, left_edge in zip(patches_hist, bins[:-1]):
            if left_edge >= -50:
                patch.set_facecolor('#4CAF50')  # 绿色-优秀
            elif left_edge >= -60:
                patch.set_facecolor('#8BC34A')  # 浅绿-良好
            elif left_edge >= -70:
                patch.set_facecolor('#FFC107')  # 黄色-一般
            else:
                patch.set_facecolor('#F44336')  # 红色-较弱
        
        avg_rssi = sum(valid_rssi) / len(valid_rssi)
        ax.axvline(x=avg_rssi, color='blue', linestyle='-', linewidth=2, label=f'均值 {avg_rssi:.1f}dBm')
    
    ax.set_xlabel('RSSI (dBm)')
    ax.set_ylabel('Count')
    ax.set_title('WiFi Signal Distribution')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # ─── 3. 每航点 WiFi 信号柱状图 ───
    ax = axes[2]
    goals = {}
    for d in data:
        gid = d.get('goal_id')
        if isinstance(gid, (int, float)) and isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100:
            gid = int(gid)
            if gid not in goals:
                goals[gid] = []
            goals[gid].append(d['wifi_rssi_dbm'])
    
    if len(goals) >= 2:
        gids = sorted(goals.keys())
        avg_rssis = [sum(goals[g]) / len(goals[g]) for g in gids]
        min_rssis = [min(goals[g]) for g in gids]
        max_rssis = [max(goals[g]) for g in gids]
        
        x_pos = range(len(gids))
        labels = [f'G{g}' for g in gids]
        
        # 绘制柱状图 + 误差线
        errors_low = [avg - mn for avg, mn in zip(avg_rssis, min_rssis)]
        errors_high = [mx - avg for avg, mx in zip(avg_rssis, max_rssis)]
        bars = ax.bar(x_pos, avg_rssis, color='#42A5F5', alpha=0.8,
                      yerr=[errors_low, errors_high], capsize=4, error_kw={'linewidth': 1})
        
        # 按信号质量着色
        for bar, v in zip(bars, avg_rssis):
            if v >= -50:
                bar.set_facecolor('#4CAF50')
            elif v >= -60:
                bar.set_facecolor('#8BC34A')
            elif v >= -70:
                bar.set_facecolor('#FFC107')
            else:
                bar.set_facecolor('#F44336')
        
        ax.axhline(y=-50, color='green', linestyle='--', linewidth=0.8, alpha=0.5)
        ax.axhline(y=-70, color='orange', linestyle='--', linewidth=0.8, alpha=0.5)
        ax.set_xticks(x_pos)
        ax.set_xticklabels(labels, rotation=45, fontsize=8)
        ax.set_ylabel('RSSI (dBm)')
        ax.set_xlabel('Goal ID')
        ax.set_title('WiFi Signal per Waypoint (mean ± min/max)')
        ax.grid(True, alpha=0.3, axis='y')
    else:
        ax.text(0.5, 0.5, 'Single waypoint - no per-goal comparison',
                transform=ax.transAxes, ha='center', va='center', fontsize=12, color='gray')
        ax.axis('off')
    
    _add_reading_guide(fig,
        '【阅读指南】上图: 蓝线=WiFi信号强度(RSSI)随时间变化，绿线≥-50dBm优秀，黄≥-60良好，橙≥-70一般，红<-80较弱。\n'
        '中图: 信号分布直方图，绿色占比越高信号越好，蓝罄1线=均值。\n'
        '下图: 每航点WiFi均值±极值，黄/红柱体的航点可能存在信号盲区。')
    plt.savefig(output_path / 'wifi_signal.png', dpi=150)
    plt.close()
    print(f"   📈 WiFi 信号图: {output_path / 'wifi_signal.png'}")


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

    _add_reading_guide(fig,
        '【阅读指南】上图: 紫线=到目标距离，降至红虚线(1.5m)以下表示到达航点。\n'
        '中图: 蓝线=航向误差(度)，接近0°表示航向对准，长期偏大说明转弯困难。\n'
        '下图: 绿线=横向偏差 CTE(m)，接近0表示紧贴航线，偏大说明路径跟踪精度不足。')
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
            plot_orca_analysis(data, output_path, header_info)
            plot_wifi_signal(data, output_path, header_info)
            plot_dashboard(data, output_path, header_info)
            plot_per_goal_stats(data, output_path, header_info)
            print(f"\n✅ 图表已保存到: {output_path}")

    report_path.write_text(report_buffer.getvalue(), encoding='utf-8')
    print(f"   📝 说明文件: {report_path}")

    # 提取 HTML 报告数据 (单文件模式下直接生成, 多文件模式下由 main 统一生成)
    report_data = None
    if HAS_HTML_REPORT:
        try:
            report_data = extract_report_data(data, header_info)
            report_data['csv_name'] = log_file.name
            if not batch_mode:
                # 单文件模式: 直接生成独立 HTML 报告
                html_path = generate_html_report(report_data, output_path, csv_name=log_file.name)
                print(f"   🌐 HTML报告: {html_path}")
                print(f"   💡 浏览器打开: file://{html_path.resolve()}")
        except Exception as e:
            print(f"   ⚠️  HTML报告数据提取失败: {e}")
            report_data = None

    return True, report_data


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
    
    _add_reading_guide(fig,
        '【阅读指南】蓝色线=自动导航(GUIDED)轨迹，灰色线=悬停等待(HOLD)阶段；\n'
        '绿●=起点，红■=终点，橙★=目标航点(G1,G2...)，蓝色箭头=USV航向方向。\n'
        '理想状态：轨迹平滑且紧贴各目标点连线，弯道处无大幅振荡。', bottom=0.08)
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
    
    _add_reading_guide(fig,
        '【阅读指南】上图: 蓝线=实际速度，红虚线=指令速度，两者越重合说明速度跟踪越准确。\n'
        '中图: 红Vx=东向分量，绿Vy=北向分量，用于分析运动方向。\n'
        '下图: 紫线=到目标距离，降至红虚线(1.5m)以下=已到达。 淡红背景=HOLD悬停模式。')
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
    
    _add_reading_guide(fig,
        '【阅读指南】上图: 蓝线=速度方向推算航向，红线=磁力计航向，运动时两者应一致。\n'
        '下图: 蓝点=运动中的航向差(可信)，灰点=低速时航向差(速度航向不可靠)。\n'
        '差值长期偏离0°说明磁力计可能需要校准。')
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
    
    _add_reading_guide(fig,
        '【阅读指南】第1行: 红=速度指令Cmd Vx，蓝=实际速度，重合度反映速度执行精度。\n'
        '第2行: 横向速度指令Cmd Vy，纯路径跟踪时应接近0。\n'
        '第3行: 角速度指令Cmd Omega，正=左转，负=右转。\n'
        '第4行(若有): 红=角速度指令ω_cmd，蓝=实际角速度ω_actual，跟踪越紧密转弯控制越好。')
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
    
    _add_reading_guide(fig,
        '【阅读指南】上图: 蓝线=MPC求解时间(ms)，超过红虚线(50ms)警戒线说明计算负荷过高。\n'
        '中图: 绿线=优化代价函数，值越低控制效果越优，突然飙升可能表示控制困难。\n'
        '下图: 黑=实际航向，红虚线=MPC参考航向，重合度反映航向预测准确性。')
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
    
    _add_reading_guide(fig,
        '【阅读指南】上图: 蓝线=当前tau_omega(转弯响应时间常数)，绿线=速度; tau随速度自动调节，虚线=配置阈值。\n'
        '中图: 红=角速度指令，蓝=实际角速度，跟踪延迟越小转弯控制越好。\n'
        '下图: 紫色填充=横向偏差(CTE)，填充面积越小表示航迹越精确，负值=偏左，正值=偏右。')
    plt.savefig(output_path / 'adaptive_tau.png', dpi=150)
    plt.close()
    print(f"   📈 自适应 Tau 图: {output_path / 'adaptive_tau.png'}")


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
    
    _add_reading_guide(fig,
        '【阅读指南】第1行: 蓝=RLS在线估计的τ，红虚=MPC实际使用的τ，绿=置信度(0~1)，"converged"标记=参数收敛。\n'
        '第2行: 红=角速度指令，蓝虚=MPC状态，黑=观测器测量值。 第3行: 橙填充=饱和率，超红线(35%)则执行器长期饱和，紫=重建次数。\n'
        '第4行: 橙=航向噪声，虚线=基线水平(0.05)，紫=横向偏差rCTE。 整体趋势平稳且τ收敛为最佳。',
        bottom=0.07)
    plt.savefig(output_path / 'ampc_analysis.png', dpi=150)
    plt.close()
    print(f"   📈 AMPC 分析图: {output_path / 'ampc_analysis.png'}")


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
    grade = "A+" if total >= 80 else "A" if total >= 70 else "B" if total >= 60 else "C" if total >= 40 else "D"
    
    # ORCA 统计
    orca_active_count = sum(1 for d in data if d.get('orca_active') == 1)
    orca_pct = orca_active_count / len(data) * 100 if data else 0
    orca_min_dist = -1.0
    if orca_active_count > 0:
        orca_dists = [d.get('orca_closest_distance', -1) for d in data
                      if d.get('orca_active') == 1 and isinstance(d.get('orca_closest_distance'), (int, float)) and d['orca_closest_distance'] > 0]
        orca_min_dist = min(orca_dists) if orca_dists else -1.0

    orca_line = f"ORCA:      {orca_pct:.0f}% (min {orca_min_dist:.2f}m)" if orca_active_count > 0 else "ORCA:      off"

    # WiFi 信号摘要
    wifi_line = "WiFi:      N/A"
    if 'wifi_rssi_dbm' in data[0]:
        wifi_vals = [d.get('wifi_rssi_dbm', -100) for d in data
                     if isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100]
        if wifi_vals:
            wifi_avg = sum(wifi_vals) / len(wifi_vals)
            wifi_min = min(wifi_vals)
            wifi_grade = "Excellent" if wifi_avg >= -50 else "Good" if wifi_avg >= -60 else "Fair" if wifi_avg >= -70 else "Weak"
            wifi_line = f"WiFi:      {wifi_avg:.0f}dBm [{wifi_grade}]"
    
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
        f"{orca_line}\n"
        f"{wifi_line}\n"
        f"\n━━━ Score ━━━\n"
        f"Total: {total:.0f}/100 [{grade}]"
    )
    ax.text(0.1, 0.95, stats_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    
    fig.text(
        0.5, 0.005,
        '【阅读指南】仪表盘一页总览: 左上=轨迹 | 中上=速度 | 右上=横向偏差 | 左中=到目标距离 | 中中=航向误差 | 右中=MPC耗时 | '
        '左下=角速度 | 中下=Tau | 右下=综合评分(A+≥80分,A≥70,B≥60,C≥40,D<40)。',
        ha='center', va='bottom', fontsize=7, color='#555555',
        bbox=dict(boxstyle='round,pad=0.4', facecolor='#FFFDE7', alpha=0.9, edgecolor='#E0E0E0'))
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
    
    # 4. 速度 或 WiFi 信号
    has_wifi = 'wifi_rssi_dbm' in data[0]
    if has_wifi:
        ax = axes[1, 1]
        avg_rssis = []
        for gid in gids:
            gdata = goals[gid]
            wifi_vals = [d.get('wifi_rssi_dbm', -100) for d in gdata
                         if isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100]
            avg_rssis.append(sum(wifi_vals) / len(wifi_vals) if wifi_vals else -100)
        
        bars = ax.bar(x, avg_rssis, color='#42A5F5', alpha=0.8)
        for bar, v in zip(bars, avg_rssis):
            if v >= -50:
                bar.set_facecolor('#4CAF50')
            elif v >= -60:
                bar.set_facecolor('#8BC34A')
            elif v >= -70:
                bar.set_facecolor('#FFC107')
            else:
                bar.set_facecolor('#F44336')
        ax.axhline(y=-50, color='green', linestyle='--', linewidth=0.8, alpha=0.5)
        ax.axhline(y=-70, color='orange', linestyle='--', linewidth=0.8, alpha=0.5)
        ax.set_ylabel('Avg WiFi RSSI (dBm)')
        ax.set_title('WiFi Signal per Goal')
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45, fontsize=8)
        ax.set_xlabel('Goal ID')
        ax.grid(True, alpha=0.3, axis='y')
    else:
        ax = axes[1, 1]
        ax.bar(x, avg_speeds, color='#FFA726', alpha=0.8)
        ax.set_ylabel('Avg Speed (m/s)')
        ax.set_title('Average Speed (GUIDED)')
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45, fontsize=8)
        ax.set_xlabel('Goal ID')
        ax.grid(True, alpha=0.3, axis='y')
    
    _add_reading_guide(fig,
        '【阅读指南】左上: 各航点平均横向偏差|CTE|，橙虚线=良好阈值0.3m，红色柱=超标(>0.5m)。\n'
        '右上: 各航点平均航向误差，橙虚线=良好阈值10°，红色柱=超标(>30°)。\n'
        '左下: 最近到达距离，红虚线=到达阈值1.5m，柱体低于红线表示成功到达。\n'
        '右下: 各航点WiFi信号或平均速度，用于发现特定航点的异常。')
    plt.savefig(output_path / 'per_goal_stats.png', dpi=150)
    plt.close()
    print(f"   📈 每航点统计: {output_path / 'per_goal_stats.png'}")


def main():
    # 确定日志路径 — 支持多个参数
    log_files = []

    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            p = Path(arg)
            if not p.exists():
                print(f"⚠️  路径不存在，已跳过: {p}")
                continue
            if p.is_dir():
                found = sorted(p.rglob('nav_log_*.csv'))
                if found:
                    log_files.extend(found)
                else:
                    print(f"⚠️  目录下未找到日志: {p}")
            else:
                log_files.append(p)
    else:
        log_dir = Path.home() / 'usv_logs'
        if not log_dir.exists():
            print("❌ 未找到日志目录: ~/usv_logs")
            sys.exit(1)
        log_files = sorted(log_dir.rglob('nav_log_*.csv'))
        if not log_files:
            print("❌ 未找到日志文件")
            sys.exit(1)
        print(f"📂 自动发现 {len(log_files)} 个日志文件")

    if not log_files:
        print("❌ 未找到任何可用日志文件")
        sys.exit(1)

    batch_mode = len(log_files) > 1
    all_report_data = []

    for idx, log_file in enumerate(log_files, start=1):
        if batch_mode:
            print("\n" + "="*60)
            print(f"[{idx}/{len(log_files)}] {log_file}")
            print("="*60)
        result = analyze_log_file(log_file, batch_mode=batch_mode)
        # result 可能是 (bool, data) 或旧版的 bool
        if isinstance(result, tuple):
            success, report_data = result
        else:
            success, report_data = result, None
        if report_data:
            all_report_data.append(report_data)

    # 多文件模式: 生成合并 HTML 报告
    if batch_mode and HAS_HTML_REPORT and len(all_report_data) >= 1:
        try:
            # 输出到第一个文件的上级目录
            multi_output = log_files[0].parent.parent if log_files[0].parent.name.startswith('nav_log_') else log_files[0].parent
            multi_output = multi_output / 'multi_report'
            multi_output.mkdir(parents=True, exist_ok=True)
            title = f'USV 多航次分析报告 ({len(all_report_data)} 条记录)'
            html_path = generate_multi_html_report(all_report_data, multi_output, title=title)
            print(f"\n🌐 多USV合并HTML报告: {html_path}")
            print(f"💡 浏览器打开: file://{html_path.resolve()}")
        except Exception as e:
            print(f"⚠️  多USV HTML报告生成失败: {e}")
            import traceback; traceback.print_exc()

    print("\n" + "="*60)
    print(f"分析完成! ({len(all_report_data)}/{len(log_files)} 文件成功)")
    print("="*60)


if __name__ == '__main__':
    main()
