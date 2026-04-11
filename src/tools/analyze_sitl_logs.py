#!/usr/bin/env python3
"""fresh57 SITL 日志综合诊断分析 — 3 USV 导航仿真数据"""

import csv
import math
import sys
import os
import numpy as np
from pathlib import Path
from collections import defaultdict

# ─────────────────────────────────────────────────────────────
# 1. 数据加载
# ─────────────────────────────────────────────────────────────
def load_nav_log(filepath):
    """加载 v18/v19 nav log CSV, 跳过注释行."""
    rows = []
    header = None
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            if header is None:
                header = line.strip().split(',')
                continue
            vals = line.strip().split(',')
            if len(vals) != len(header):
                continue
            row = {}
            for h, v in zip(header, vals):
                try:
                    row[h] = float(v)
                except ValueError:
                    row[h] = v
            rows.append(row)
    return rows, header


def extract_usv_id(filepath):
    """从文件名提取 usv_XX."""
    import re
    name = Path(filepath).name
    m = re.search(r'(usv_\d+)', name)
    if m:
        return m.group(1)
    return name


# ─────────────────────────────────────────────────────────────
# 2. 单 USV 分析
# ─────────────────────────────────────────────────────────────
def analyze_single_usv(rows, usv_id):
    """单艇指标分析."""
    result = {'usv_id': usv_id}
    if not rows:
        return result

    ts = np.array([r['timestamp'] for r in rows])
    px = np.array([r['pose_x'] for r in rows])
    py = np.array([r['pose_y'] for r in rows])
    yaw = np.array([r['pose_yaw_deg'] for r in rows])
    speed = np.array([r.get('velocity_speed', 0.0) for r in rows])
    dist_goal = np.array([r.get('distance_to_goal', -1) for r in rows])

    # RL 命令
    rl_vx = np.array([r.get('rl_cmd_vx', 0.0) for r in rows])
    rl_omega = np.array([r.get('rl_cmd_omega', 0.0) for r in rows])
    rl_active = np.array([r.get('rl_cmd_active', 0) for r in rows])

    # 最终命令
    cmd_vx = np.array([r.get('cmd_vx', 0.0) for r in rows])
    cmd_omega = np.array([r.get('cmd_omega', 0.0) for r in rows])

    # 原始命令 (MPC/APF)
    raw_vx = np.array([r.get('raw_cmd_vx', 0.0) for r in rows])
    raw_omega = np.array([r.get('raw_cmd_omega', 0.0) for r in rows])

    # 实际 omega
    omega_actual = np.array([r.get('omega_actual', 0.0) for r in rows])

    heading_err = np.array([r.get('heading_error_deg', 0.0) for r in rows])

    # ── 基本统计 ──
    duration = ts[-1] - ts[0]
    dt = np.diff(ts)
    dt_mean = np.mean(dt) if len(dt) > 0 else 0.1

    # 行驶距离
    dx = np.diff(px)
    dy = np.diff(py)
    dist_traveled = np.sum(np.sqrt(dx**2 + dy**2))

    # 目标
    goal_x = rows[-1].get('target_x', 0)
    goal_y = rows[-1].get('target_y', 0)
    start_dist = dist_goal[0] if dist_goal[0] > 0 else np.sqrt((px[0]-goal_x)**2 + (py[0]-goal_y)**2)
    final_dist = dist_goal[-1] if dist_goal[-1] > 0 else np.sqrt((px[-1]-goal_x)**2 + (py[-1]-goal_y)**2)
    progress = max(0, 1.0 - final_dist / start_dist) if start_dist > 0.01 else 0

    # 目标 ID 变化（判断是否到达过中间目标）
    goal_ids = [r.get('goal_id', -1) for r in rows]
    unique_goals = list(dict.fromkeys(goal_ids))  # 保留顺序去重

    result['duration_s'] = duration
    result['dist_traveled_m'] = dist_traveled
    result['start_pos'] = (px[0], py[0])
    result['end_pos'] = (px[-1], py[-1])
    result['goal_pos'] = (goal_x, goal_y)
    result['start_dist_to_goal'] = start_dist
    result['final_dist_to_goal'] = final_dist
    result['progress_ratio'] = progress
    result['goal_sequence'] = unique_goals
    result['reached_final_goal'] = final_dist < 0.5

    # ── 速度分析 ──
    rl_mask = rl_active > 0.5
    rl_ratio = np.mean(rl_mask)

    result['speed_mean'] = np.mean(speed)
    result['speed_max'] = np.max(speed)
    result['speed_std'] = np.std(speed)
    result['rl_active_ratio'] = rl_ratio
    result['rl_cmd_vx_mean'] = np.mean(rl_vx[rl_mask]) if np.any(rl_mask) else 0
    result['rl_cmd_omega_mean'] = np.mean(np.abs(rl_omega[rl_mask])) if np.any(rl_mask) else 0
    result['cmd_vx_mean'] = np.mean(cmd_vx)
    result['cmd_omega_mean_abs'] = np.mean(np.abs(cmd_omega))

    # 速度为0或接近0的比率（停滞）
    stall_mask = speed < 0.02
    result['stall_ratio'] = np.mean(stall_mask)

    # ── Omega 震荡分析 (关键) ──
    # omega flip: 连续两步 omega 符号翻转
    omega_sign = np.sign(cmd_omega)
    omega_flips = 0
    for i in range(1, len(omega_sign)):
        if omega_sign[i] != 0 and omega_sign[i-1] != 0 and omega_sign[i] != omega_sign[i-1]:
            omega_flips += 1

    result['omega_flip_count'] = omega_flips
    result['omega_flip_rate_per_s'] = omega_flips / duration if duration > 0 else 0

    # 角加速度 (angular jerk indicator)
    omega_dot = np.diff(cmd_omega) / np.clip(dt, 0.01, None)
    result['omega_dot_mean_abs'] = np.mean(np.abs(omega_dot))
    result['omega_dot_max'] = np.max(np.abs(omega_dot))
    result['omega_dot_p95'] = np.percentile(np.abs(omega_dot), 95)

    # RL omega 震荡
    if np.any(rl_mask):
        rl_omega_active = rl_omega[rl_mask]
        rl_sign = np.sign(rl_omega_active)
        rl_flips = sum(1 for i in range(1, len(rl_sign))
                       if rl_sign[i] != 0 and rl_sign[i-1] != 0 and rl_sign[i] != rl_sign[i-1])
        result['rl_omega_flip_count'] = rl_flips
        result['rl_omega_std'] = np.std(rl_omega_active)
        result['rl_omega_max'] = np.max(np.abs(rl_omega_active))
    else:
        result['rl_omega_flip_count'] = 0
        result['rl_omega_std'] = 0
        result['rl_omega_max'] = 0

    # ── RL vs MPC 分歧分析 ──
    if np.any(rl_mask):
        delta_vx = rl_vx[rl_mask] - raw_vx[rl_mask]
        delta_omega = rl_omega[rl_mask] - raw_omega[rl_mask]
        result['rl_raw_delta_vx_mean'] = np.mean(delta_vx)
        result['rl_raw_delta_vx_std'] = np.std(delta_vx)
        result['rl_raw_delta_omega_mean'] = np.mean(delta_omega)
        result['rl_raw_delta_omega_std'] = np.std(delta_omega)
        # RL 完全覆盖 raw 的比率
        dominant = np.abs(rl_vx[rl_mask]) > np.abs(raw_vx[rl_mask])
        result['rl_dominates_vx_ratio'] = np.mean(dominant)
    else:
        result['rl_raw_delta_vx_mean'] = 0
        result['rl_raw_delta_vx_std'] = 0
        result['rl_raw_delta_omega_mean'] = 0
        result['rl_raw_delta_omega_std'] = 0
        result['rl_dominates_vx_ratio'] = 0

    # ── 航向误差分析 ──
    valid_he = heading_err[np.abs(heading_err) < 360]
    result['heading_err_mean'] = np.mean(np.abs(valid_he)) if len(valid_he) > 0 else 0
    result['heading_err_max'] = np.max(np.abs(valid_he)) if len(valid_he) > 0 else 0

    # ── tau (动态延迟) 分析 ──
    tau_omega = np.array([r.get('current_tau_omega', 0.0) for r in rows])
    result['tau_omega_mean'] = np.mean(tau_omega[tau_omega > 0]) if np.any(tau_omega > 0) else 0
    result['tau_omega_min'] = np.min(tau_omega[tau_omega > 0]) if np.any(tau_omega > 0) else 0

    # ── 时间序列数据 (用于后续多艇分析) ──
    result['_ts'] = ts
    result['_px'] = px
    result['_py'] = py
    result['_yaw'] = yaw
    result['_speed'] = speed
    result['_cmd_omega'] = cmd_omega
    result['_rl_omega'] = rl_omega
    result['_rl_active'] = rl_active
    result['_dist_goal'] = dist_goal

    return result


# ─────────────────────────────────────────────────────────────
# 3. 多艇安全分析
# ─────────────────────────────────────────────────────────────
def analyze_inter_usv_safety(analyses):
    """计算所有 USV 对之间的最小距离时间序列."""
    usv_ids = list(analyses.keys())
    n = len(usv_ids)
    safety_results = {}

    for i in range(n):
        for j in range(i+1, n):
            a, b = analyses[usv_ids[i]], analyses[usv_ids[j]]
            ts_a, px_a, py_a = a['_ts'], a['_px'], a['_py']
            ts_b, px_b, py_b = b['_ts'], b['_px'], b['_py']

            # 对齐时间: 用最近邻插值
            t_start = max(ts_a[0], ts_b[0])
            t_end = min(ts_a[-1], ts_b[-1])
            if t_end <= t_start:
                continue

            # 统一时间网格 (10Hz)
            t_grid = np.arange(t_start, t_end, 0.1)
            xa = np.interp(t_grid, ts_a, px_a)
            ya = np.interp(t_grid, ts_a, py_a)
            xb = np.interp(t_grid, ts_b, px_b)
            yb = np.interp(t_grid, ts_b, py_b)

            dist = np.sqrt((xa - xb)**2 + (ya - yb)**2)
            min_dist = np.min(dist)
            min_idx = np.argmin(dist)
            min_time = t_grid[min_idx]

            # 近距离时间段
            close_1m = np.sum(dist < 1.0) * 0.1  # seconds below 1m
            close_2m = np.sum(dist < 2.0) * 0.1
            close_3m = np.sum(dist < 3.0) * 0.1
            close_5m = np.sum(dist < 5.0) * 0.1

            # 接近速率 (距离最小时刻前后)
            if min_idx > 5 and min_idx < len(dist) - 5:
                approach_rate = (dist[min_idx-5] - dist[min_idx]) / (5 * 0.1)  # m/s
            else:
                approach_rate = 0

            pair_id = f"{usv_ids[i]}-{usv_ids[j]}"
            safety_results[pair_id] = {
                'min_distance_m': min_dist,
                'min_dist_time_s': min_time - t_start,
                'min_dist_positions': {
                    usv_ids[i]: (xa[min_idx], ya[min_idx]),
                    usv_ids[j]: (xb[min_idx], yb[min_idx]),
                },
                'approach_rate_mps': approach_rate,
                'time_below_1m_s': close_1m,
                'time_below_2m_s': close_2m,
                'time_below_3m_s': close_3m,
                'time_below_5m_s': close_5m,
                'collision': min_dist < 0.75,
                'near_miss': min_dist < 1.5,
                '_dist_series': dist,
                '_t_grid': t_grid,
            }

    return safety_results


# ─────────────────────────────────────────────────────────────
# 4. 碰撞/近距离事件时间线重建
# ─────────────────────────────────────────────────────────────
def reconstruct_close_encounters(safety_results, analyses):
    """提取最小距离前后的详细行为时间线."""
    encounters = []
    for pair_id, sr in safety_results.items():
        if sr['min_distance_m'] >= 3.0:
            continue

        usv_a, usv_b = pair_id.split('-')
        t_grid = sr['_t_grid']
        dist = sr['_dist_series']
        min_idx = np.argmin(dist)
        min_t = t_grid[min_idx]

        # 找到距离最小点前后 15 秒窗口
        window_start = min_t - 15
        window_end = min_t + 15
        mask = (t_grid >= window_start) & (t_grid <= window_end)

        encounter = {
            'pair': pair_id,
            'min_dist': sr['min_distance_m'],
            'min_time_offset_s': sr['min_dist_time_s'],
            'approach_rate': sr['approach_rate_mps'],
            'phases': [],
        }

        # 在接近阶段分析每个 USV 的行为
        for uid in [usv_a, usv_b]:
            a = analyses[uid]
            ts_a = a['_ts']
            # 找最近时间戳
            idx_near = np.argmin(np.abs(ts_a - min_t))

            # 窗口 [-15s, +15s]
            wstart = max(0, idx_near - 150)  # ~15s at 10Hz
            wend = min(len(ts_a), idx_near + 150)

            omega_window = a['_cmd_omega'][wstart:wend]
            rl_omega_window = a['_rl_omega'][wstart:wend]
            speed_window = a['_speed'][wstart:wend]
            rl_active_window = a['_rl_active'][wstart:wend]

            # omega 行为在接近阶段
            sign_changes = 0
            for k in range(1, len(omega_window)):
                if omega_window[k] * omega_window[k-1] < 0:
                    sign_changes += 1

            encounter['phases'].append({
                'usv': uid,
                'omega_flips_in_window': sign_changes,
                'mean_speed': np.mean(speed_window),
                'min_speed': np.min(speed_window),
                'rl_active_ratio': np.mean(rl_active_window > 0.5),
                'rl_omega_mean_abs': np.mean(np.abs(rl_omega_window)),
                'rl_omega_max': np.max(np.abs(rl_omega_window)),
                'cmd_omega_mean_abs': np.mean(np.abs(omega_window)),
            })

        encounters.append(encounter)
    return encounters


# ─────────────────────────────────────────────────────────────
# 5. 报告生成
# ─────────────────────────────────────────────────────────────
def print_report(analyses, safety_results, encounters):
    """打印完整诊断报告."""
    print()
    print("=" * 78)
    print("  fresh57 SITL 3-USV 仿真诊断报告")
    print("  模型: fresh57_step_0200000.pt")
    print("=" * 78)

    # ── A. 总览 ──
    print("\n" + "─" * 78)
    print("  A. 任务完成总览")
    print("─" * 78)
    fmt = "  {:<10} {:>8} {:>8} {:>10} {:>10} {:>10} {:>8}"
    print(fmt.format("USV", "时长(s)", "距离(m)", "起始距离", "终止距离", "进度比", "到达"))
    print("  " + "-" * 72)
    for uid, a in sorted(analyses.items()):
        print(fmt.format(
            uid,
            f"{a['duration_s']:.1f}",
            f"{a['dist_traveled_m']:.2f}",
            f"{a['start_dist_to_goal']:.2f}m",
            f"{a['final_dist_to_goal']:.2f}m",
            f"{a['progress_ratio']:.1%}",
            "✅" if a['reached_final_goal'] else "❌",
        ))
    for uid, a in sorted(analyses.items()):
        print(f"  {uid} 目标序列: {a['goal_sequence']}")

    # ── B. 速度与控制 ──
    print("\n" + "─" * 78)
    print("  B. 速度与控制分析")
    print("─" * 78)
    fmt = "  {:<10} {:>8} {:>8} {:>8} {:>10} {:>10} {:>10}"
    print(fmt.format("USV", "均速", "峰速", "停滞%", "RL激活%", "RL_vx均", "RL_ω均|"))
    print("  " + "-" * 72)
    for uid, a in sorted(analyses.items()):
        print(fmt.format(
            uid,
            f"{a['speed_mean']:.3f}",
            f"{a['speed_max']:.3f}",
            f"{a['stall_ratio']:.1%}",
            f"{a['rl_active_ratio']:.1%}",
            f"{a['rl_cmd_vx_mean']:.3f}",
            f"{a['rl_cmd_omega_mean']:.3f}",
        ))

    # ── C. Omega 震荡 (核心问题) ──
    print("\n" + "─" * 78)
    print("  C. ⚠️ Omega 震荡分析 (SITL 门控核心失败指标)")
    print("─" * 78)
    fmt = "  {:<10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}"
    print(fmt.format("USV", "翻转次数", "翻转Hz", "RL翻转", "ω̇均|", "ω̇p95", "ω̇max"))
    print("  " + "-" * 72)
    for uid, a in sorted(analyses.items()):
        print(fmt.format(
            uid,
            f"{a['omega_flip_count']}",
            f"{a['omega_flip_rate_per_s']:.2f}",
            f"{a['rl_omega_flip_count']}",
            f"{a['omega_dot_mean_abs']:.3f}",
            f"{a['omega_dot_p95']:.3f}",
            f"{a['omega_dot_max']:.3f}",
        ))

    # 对比 RL omega 与 final omega 的统计
    print("\n  RL omega 统计:")
    for uid, a in sorted(analyses.items()):
        print(f"    {uid}: rl_ω_std={a['rl_omega_std']:.4f}, "
              f"rl_ω_max={a['rl_omega_max']:.4f}, "
              f"tau_ω={a['tau_omega_mean']:.3f}")

    # ── D. RL vs MPC 分歧 ──
    print("\n" + "─" * 78)
    print("  D. RL 策略 vs MPC 基线分歧")
    print("─" * 78)
    fmt = "  {:<10} {:>12} {:>12} {:>12} {:>12} {:>12}"
    print(fmt.format("USV", "Δvx_mean", "Δvx_std", "Δω_mean", "Δω_std", "RL主导vx%"))
    print("  " + "-" * 72)
    for uid, a in sorted(analyses.items()):
        print(fmt.format(
            uid,
            f"{a['rl_raw_delta_vx_mean']:.4f}",
            f"{a['rl_raw_delta_vx_std']:.4f}",
            f"{a['rl_raw_delta_omega_mean']:.4f}",
            f"{a['rl_raw_delta_omega_std']:.4f}",
            f"{a['rl_dominates_vx_ratio']:.1%}",
        ))

    # ── E. 航向误差 ──
    print("\n" + "─" * 78)
    print("  E. 航向误差")
    print("─" * 78)
    for uid, a in sorted(analyses.items()):
        print(f"  {uid}: 均值={a['heading_err_mean']:.1f}°, 最大={a['heading_err_max']:.1f}°")

    # ── F. 多艇安全 (核心) ──
    print("\n" + "─" * 78)
    print("  F. ⚠️ 多艇安全距离分析")
    print("─" * 78)
    collision_count = 0
    near_miss_count = 0
    for pair_id, sr in sorted(safety_results.items()):
        status = "✅ 安全"
        if sr['collision']:
            status = "💥 碰撞"
            collision_count += 1
        elif sr['near_miss']:
            status = "⚠️ 近距离"
            near_miss_count += 1

        print(f"\n  {pair_id}: {status}")
        print(f"    最小距离: {sr['min_distance_m']:.3f}m (@ +{sr['min_dist_time_s']:.1f}s)")
        print(f"    接近速率: {sr['approach_rate_mps']:.3f} m/s")
        print(f"    <1m: {sr['time_below_1m_s']:.1f}s, "
              f"<2m: {sr['time_below_2m_s']:.1f}s, "
              f"<3m: {sr['time_below_3m_s']:.1f}s, "
              f"<5m: {sr['time_below_5m_s']:.1f}s")

    print(f"\n  总计: {collision_count} 碰撞, {near_miss_count} 近距离事件")

    # ── G. 碰撞/近距离时间线 ──
    if encounters:
        print("\n" + "─" * 78)
        print("  G. 近距离事件行为分析 (最小距离 ±15s 窗口)")
        print("─" * 78)
        for enc in encounters:
            print(f"\n  事件: {enc['pair']} — 最小距离 {enc['min_dist']:.3f}m, "
                  f"接近速率 {enc['approach_rate']:.3f}m/s")
            for phase in enc['phases']:
                print(f"    {phase['usv']}:")
                print(f"      omega翻转: {phase['omega_flips_in_window']}次, "
                      f"rl_ω_max: {phase['rl_omega_max']:.4f}, "
                      f"cmd_ω_均: {phase['cmd_omega_mean_abs']:.4f}")
                print(f"      均速: {phase['mean_speed']:.3f}, "
                      f"最低速: {phase['min_speed']:.3f}, "
                      f"RL激活: {phase['rl_active_ratio']:.1%}")

    # ── H. 综合诊断 ──
    print("\n" + "=" * 78)
    print("  H. 综合诊断与问题清单")
    print("=" * 78)

    issues = []
    # 检查碰撞
    for pair_id, sr in safety_results.items():
        if sr['collision']:
            issues.append(f"🔴 致命: {pair_id} 发生碰撞 (min={sr['min_distance_m']:.3f}m < 0.75m)")
        elif sr['near_miss']:
            issues.append(f"🟠 严重: {pair_id} 近距离接触 (min={sr['min_distance_m']:.3f}m < 1.5m)")
        elif sr['min_distance_m'] < 3.0:
            issues.append(f"🟡 警告: {pair_id} 安全余量不足 (min={sr['min_distance_m']:.3f}m < 3.0m)")

    # 检查 omega 震荡
    for uid, a in analyses.items():
        if a['omega_flip_rate_per_s'] > 0.1:
            issues.append(f"🟠 严重: {uid} 角速度震荡剧烈 "
                         f"(翻转 {a['omega_flip_count']}次, {a['omega_flip_rate_per_s']:.2f}Hz)")
        if a['omega_dot_p95'] > 2.0:
            issues.append(f"🟡 警告: {uid} 角加速度过大 (p95={a['omega_dot_p95']:.3f} rad/s²)")

    # 检查停滞
    for uid, a in analyses.items():
        if a['stall_ratio'] > 0.3:
            issues.append(f"🟡 警告: {uid} 长时间停滞 ({a['stall_ratio']:.1%} 时间速度<0.02)")

    # 检查目标达成
    for uid, a in analyses.items():
        if not a['reached_final_goal']:
            if a['progress_ratio'] < 0.5:
                issues.append(f"🟠 严重: {uid} 进度极低 ({a['progress_ratio']:.1%})")
            else:
                issues.append(f"🟡 警告: {uid} 未到达目标 (进度 {a['progress_ratio']:.1%}, "
                             f"剩余 {a['final_dist_to_goal']:.2f}m)")

    # 检查 RL 活跃度
    for uid, a in analyses.items():
        if a['rl_active_ratio'] < 0.5:
            issues.append(f"🟡 警告: {uid} RL 策略激活率低 ({a['rl_active_ratio']:.1%})")

    if not issues:
        print("\n  ✅ 未发现严重问题")
    else:
        for i, issue in enumerate(issues, 1):
            print(f"\n  [{i}] {issue}")

    print("\n" + "=" * 78)
    return issues


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
def main():
    log_dir = Path(os.path.expanduser("~/usv_logs"))
    csv_files = sorted(log_dir.glob("nav_log_*.csv"))
    if not csv_files:
        print(f"ERROR: No nav_log CSV files found in {log_dir}")
        sys.exit(1)

    print(f"Loading {len(csv_files)} log files from {log_dir} ...")

    analyses = {}
    for f in csv_files:
        uid = extract_usv_id(f)
        rows, _ = load_nav_log(f)
        print(f"  {uid}: {len(rows)} rows ({f.name})")
        analyses[uid] = analyze_single_usv(rows, uid)

    # 多艇安全分析
    safety_results = analyze_inter_usv_safety(analyses)

    # 近距离事件重建
    encounters = reconstruct_close_encounters(safety_results, analyses)

    # 报告
    issues = print_report(analyses, safety_results, encounters)

    return len(issues)


if __name__ == '__main__':
    sys.exit(0 if main() == 0 else 1)
