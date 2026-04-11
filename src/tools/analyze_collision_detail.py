#!/usr/bin/env python3
"""深入分析碰撞时刻的 RL 策略行为和邻居感知."""

import csv
import math
import sys
import os
import re
import numpy as np
from pathlib import Path


def load_nav_log(filepath):
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


def main():
    log_dir = Path(os.path.expanduser("~/usv_logs"))
    csv_files = sorted(log_dir.glob("nav_log_*.csv"))

    data = {}
    for f in csv_files:
        m = re.search(r'(usv_\d+)', f.name)
        uid = m.group(1) if m else f.stem
        rows, _ = load_nav_log(f)
        data[uid] = rows

    # 将数据按时间对齐
    all_ts = {}
    for uid, rows in data.items():
        ts = np.array([r['timestamp'] for r in rows])
        px = np.array([r['pose_x'] for r in rows])
        py = np.array([r['pose_y'] for r in rows])
        all_ts[uid] = (ts, px, py, rows)

    uids = sorted(data.keys())
    pairs = [(uids[i], uids[j]) for i in range(len(uids)) for j in range(i+1, len(uids))]

    for uid_a, uid_b in pairs:
        ts_a, px_a, py_a, rows_a = all_ts[uid_a]
        ts_b, px_b, py_b, rows_b = all_ts[uid_b]

        t_start = max(ts_a[0], ts_b[0])
        t_end = min(ts_a[-1], ts_b[-1])
        t_grid = np.arange(t_start, t_end, 0.1)

        xa = np.interp(t_grid, ts_a, px_a)
        ya = np.interp(t_grid, ts_a, py_a)
        xb = np.interp(t_grid, ts_b, px_b)
        yb = np.interp(t_grid, ts_b, py_b)
        dist = np.sqrt((xa - xb)**2 + (ya - yb)**2)

        min_idx = np.argmin(dist)
        min_dist = dist[min_idx]
        min_t = t_grid[min_idx]

        if min_dist > 3.0:
            continue

        print(f"\n{'='*78}")
        print(f"  碰撞/近距离事件: {uid_a} - {uid_b}")
        print(f"  最小距离: {min_dist:.3f}m @ t={min_t:.1f}s")
        print(f"{'='*78}")

        # 对每个 USV 提取碰撞时刻前后 30 秒的详细数据
        for uid in [uid_a, uid_b]:
            ts_u, px_u, py_u, rows_u = all_ts[uid]
            # 找到最近的行
            idx_center = np.argmin(np.abs(ts_u - min_t))

            print(f"\n  ── {uid} 碰撞时刻详情 (行 {idx_center}) ──")
            r = rows_u[idx_center]

            print(f"    位置: ({r['pose_x']:.3f}, {r['pose_y']:.3f}), 航向: {r['pose_yaw_deg']:.1f}°")
            print(f"    速度: vx={r['velocity_vx']:.3f}, vy={r['velocity_vy']:.3f}, speed={r['velocity_speed']:.3f}")
            print(f"    目标: ({r['target_x']:.2f}, {r['target_y']:.2f}), 距离={r['distance_to_goal']:.2f}m, 航向误差={r['heading_error_deg']:.1f}°")
            print(f"    RL 命令: vx={r['rl_cmd_vx']:.4f}, omega={r['rl_cmd_omega']:.4f}, active={r['rl_cmd_active']}")
            print(f"    RL delta: dvx={r['rl_delta_vx']:.4f}, domega={r['rl_delta_omega']:.4f}")
            print(f"    MPC 命令: vx={r['raw_cmd_vx']:.4f}, omega={r['raw_cmd_omega']:.4f}")
            print(f"    最终命令: vx={r['cmd_vx']:.4f}, omega={r['cmd_omega']:.4f}")
            print(f"    遭遇类型: rl_encounter={r.get('rl_encounter_type_index', '?')}")
            print(f"    ORCA: active={r.get('orca_active', '?')}, encounter={r.get('orca_encounter_type', '?')}, "
                  f"closest={r.get('orca_closest_distance', '?')}")
            print(f"    tau_omega: {r.get('current_tau_omega', '?')}, ampc_tau={r.get('ampc_tau_estimated', '?')}")

            # 邻居信息
            for nb in range(1, 6):
                nb_id = r.get(f'neighbor_{nb}_id', 0)
                try:
                    nb_id_f = float(nb_id)
                except (ValueError, TypeError):
                    continue
                if nb_id_f == 0:
                    continue
                try:
                    nb_x = float(r.get(f'neighbor_{nb}_x', 0))
                    nb_y = float(r.get(f'neighbor_{nb}_y', 0))
                except (ValueError, TypeError):
                    continue
                nb_dist = math.sqrt((nb_x - r['pose_x'])**2 + (nb_y - r['pose_y'])**2)
                print(f"    邻居{nb}: id={nb_id_f:.0f}, pos=({nb_x:.2f}, {nb_y:.2f}), 距离={nb_dist:.3f}m")

        # 碰撞前后轨迹走势 (采样关键时间点)
        print(f"\n  ── 碰撞前后距离变化 ──")
        offsets = [-30, -20, -10, -5, -3, -1, 0, 1, 3, 5, 10, 20, 30]
        print(f"    {'偏移(s)':>8} {'距离(m)':>10} {uid_a+'速度':>12} {uid_b+'速度':>12} "
              f"{uid_a+'rl_ω':>12} {uid_b+'rl_ω':>12}")
        print(f"    {'-'*68}")
        for off in offsets:
            t_sample = min_t + off
            idx_s = np.argmin(np.abs(t_grid - t_sample))
            if idx_s >= len(dist):
                continue
            d = dist[idx_s]

            # 找每个 USV 此时刻的详情
            vals = {}
            for uid_key in [uid_a, uid_b]:
                ts_u, _, _, rows_u = all_ts[uid_key]
                idx_u = np.argmin(np.abs(ts_u - t_sample))
                vals[uid_key] = rows_u[idx_u]

            print(f"    {off:>+8.0f} {d:>10.3f} "
                  f"{vals[uid_a]['velocity_speed']:>12.3f} {vals[uid_b]['velocity_speed']:>12.3f} "
                  f"{vals[uid_a]['rl_cmd_omega']:>12.4f} {vals[uid_b]['rl_cmd_omega']:>12.4f}")

    # ── RL omega 饱和分析 ──
    print(f"\n{'='*78}")
    print(f"  RL Omega 饱和分析 (是否触顶 action space)")
    print(f"{'='*78}")
    for uid in uids:
        rows_u = data[uid]
        rl_omega = np.array([r['rl_cmd_omega'] for r in rows_u])
        rl_vx = np.array([r['rl_cmd_vx'] for r in rows_u])
        active = np.array([r['rl_cmd_active'] for r in rows_u])
        mask = active > 0.5

        if not np.any(mask):
            continue

        rl_o = rl_omega[mask]
        rl_v = rl_vx[mask]

        # 检测动作空间边界
        omega_max = np.max(np.abs(rl_o))
        omega_near_max = np.sum(np.abs(rl_o) > omega_max * 0.95) / len(rl_o)
        vx_max = np.max(rl_v)
        vx_min = np.min(rl_v)

        # 直方图
        omega_bins = np.linspace(-omega_max, omega_max, 21)
        hist, _ = np.histogram(rl_o, bins=omega_bins)
        hist_pct = hist / len(rl_o) * 100

        print(f"\n  {uid}:")
        print(f"    rl_omega 范围: [{np.min(rl_o):.4f}, {np.max(rl_o):.4f}]")
        print(f"    rl_vx 范围:    [{vx_min:.4f}, {vx_max:.4f}]")
        print(f"    omega 接近极值(>95%max)的比例: {omega_near_max:.1%}")
        print(f"    omega 分布 (直方图):")
        for i in range(len(hist)):
            bar = '█' * int(hist_pct[i] / 2)
            low = omega_bins[i]
            high = omega_bins[i+1]
            print(f"      [{low:+.3f}, {high:+.3f}): {hist_pct[i]:5.1f}% {bar}")

    # ── RL 遭遇类型分析 ──
    print(f"\n{'='*78}")
    print(f"  RL 遭遇类型 (encounter_type_index) 分析")
    print(f"{'='*78}")
    for uid in uids:
        rows_u = data[uid]
        enc_types = [r.get('rl_encounter_type_index', -1) for r in rows_u]
        from collections import Counter
        cnt = Counter(enc_types)
        total = len(enc_types)
        print(f"\n  {uid}:")
        for k, v in sorted(cnt.items()):
            pct = v / total * 100
            print(f"    type={k:.0f}: {v} ({pct:.1f}%)")

    # ── 接近速度 vs 分开速度分析 ──
    print(f"\n{'='*78}")
    print(f"  碰撞过程分析: 接近速度 vs 分开速度")
    print(f"{'='*78}")
    for uid_a, uid_b in pairs:
        ts_a, px_a, py_a, _ = all_ts[uid_a]
        ts_b, px_b, py_b, _ = all_ts[uid_b]

        t_start = max(ts_a[0], ts_b[0])
        t_end = min(ts_a[-1], ts_b[-1])
        t_grid = np.arange(t_start, t_end, 0.1)

        xa = np.interp(t_grid, ts_a, px_a)
        ya = np.interp(t_grid, ts_a, py_a)
        xb = np.interp(t_grid, ts_b, px_b)
        yb = np.interp(t_grid, ts_b, py_b)
        dist = np.sqrt((xa - xb)**2 + (ya - yb)**2)
        min_idx = np.argmin(dist)
        if dist[min_idx] > 3.0:
            continue

        # 接近阶段: 最小距离前 30s
        ap_start = max(0, min_idx - 300)
        approach_dist = dist[ap_start:min_idx+1]
        if len(approach_dist) > 10:
            # 距离变化率
            d_rate = np.diff(approach_dist) / 0.1  # m/s
            max_approach_rate = np.min(d_rate)  # 负值表示接近
            mean_approach_rate = np.mean(d_rate[d_rate < 0])

        # 分开阶段: 最小距离后 30s
        sep_end = min(len(dist), min_idx + 300)
        separate_dist = dist[min_idx:sep_end]
        if len(separate_dist) > 10:
            d_rate_sep = np.diff(separate_dist) / 0.1
            max_sep_rate = np.max(d_rate_sep)
            mean_sep_rate = np.mean(d_rate_sep[d_rate_sep > 0]) if np.any(d_rate_sep > 0) else 0

        print(f"\n  {uid_a}-{uid_b}: min_dist={dist[min_idx]:.3f}m")
        print(f"    接近阶段: 最大接近速率={max_approach_rate:.3f} m/s, 均接近速率={mean_approach_rate:.3f} m/s")
        print(f"    分开阶段: 最大分开速率={max_sep_rate:.3f} m/s, 均分开速率={mean_sep_rate:.3f} m/s")

        # 在什么距离开始密集转向?
        # 找到距离首次 < 5m 的时刻
        below_5m = np.where(dist < 5.0)[0]
        if len(below_5m) > 0:
            first_5m_idx = below_5m[0]
            first_5m_t = t_grid[first_5m_idx]
            min_t = t_grid[min_idx]
            reaction_time = (min_t - first_5m_t)
            print(f"    首次进入5m: t={first_5m_t:.1f}s, 距碰撞时刻={reaction_time:.1f}s")

            # 检查 RL 是否在进入5m时开始改变行为
            for uid_key in [uid_a, uid_b]:
                ts_u, _, _, rows_u = all_ts[uid_key]
                idx_5m = np.argmin(np.abs(ts_u - first_5m_t))
                idx_min = np.argmin(np.abs(ts_u - min_t))
                if idx_5m < idx_min:
                    rl_omega_before = np.abs(rows_u[max(0,idx_5m-50)].get('rl_cmd_omega', 0))
                    rl_omega_at5m = np.abs(rows_u[idx_5m].get('rl_cmd_omega', 0))
                    rl_omega_at_min = np.abs(rows_u[idx_min].get('rl_cmd_omega', 0))
                    enc_at5m = rows_u[idx_5m].get('rl_encounter_type_index', -1)
                    enc_at_min = rows_u[idx_min].get('rl_encounter_type_index', -1)
                    print(f"    {uid_key}: 5m前rl_ω={rl_omega_before:.4f}, "
                          f"5m时rl_ω={rl_omega_at5m:.4f}, "
                          f"碰撞时rl_ω={rl_omega_at_min:.4f}")
                    print(f"    {uid_key}: 5m时遭遇类型={enc_at5m:.0f}, 碰撞时={enc_at_min:.0f}")


if __name__ == '__main__':
    main()
