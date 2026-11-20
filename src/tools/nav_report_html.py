#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USV 导航日志 HTML 交互式报告生成器 (v2 — 多USV + 路径重放 + 声效)

功能:
- 支持多文件/多USV同时分析 (10+)
- USV选择器: 勾选不同USV查看独立数据
- 路径重放动画: 船形模型 + 目标点, 播放/暂停/加速/减速
- 声效系统: Web Audio API, 不同区域不同音效
- Plotly.js 交互图表 (缩放/悬停/框选)
- 深色科技感主题 + 毛玻璃卡片 + CSS 动画

作者: chenhangwei
日期: 2026-03-03
"""

import json
import math
from pathlib import Path
from datetime import datetime
from typing import Any

# ─── 数据提取：从原始 data list 中提取结构化统计 ──────────────────

def extract_report_data(data: list, header_info: dict) -> dict:
    """从导航日志数据中提取全部分析指标，返回结构化字典。"""
    if not data:
        return {}

    t0 = data[0]['timestamp']
    duration = data[-1]['timestamp'] - t0
    t = [d['timestamp'] - t0 for d in data]

    guided = [d for d in data if d.get('flight_mode') == 'GUIDED']
    hold = [d for d in data if d.get('flight_mode') == 'HOLD']

    info = {
        'usv_id': header_info.get('usv_id', 'unknown'),
      'version': header_info.get('version_display', header_info.get('version', '?')),
      'schema_name': header_info.get('schema_name', ''),
      'schema_description': header_info.get('schema_description', ''),
        'params': header_info.get('params', {}),
        'duration_s': duration,
        'start_ts': t0,
        'record_count': len(data),
        'guided_count': len(guided),
        'hold_count': len(hold),
        'guided_pct': len(guided) / len(data) * 100 if data else 0,
    }

    sampling = {}
    if len(data) >= 3:
        dts = sorted(data[i]['timestamp'] - data[i-1]['timestamp'] for i in range(1, len(data)))
        sampling = {
            'dt_median': dts[len(dts)//2],
            'dt_p95': dts[int(len(dts)*0.95)],
            'dt_max': dts[-1],
            'hz': 1.0 / dts[len(dts)//2] if dts[len(dts)//2] > 1e-9 else 0,
        }

    speeds = [d['velocity_speed'] for d in data]
    g_speeds = [d['velocity_speed'] for d in guided] if guided else []
    velocity = {
        'avg': sum(speeds) / len(speeds),
        'max': max(speeds),
        'guided_avg': sum(g_speeds) / len(g_speeds) if g_speeds else 0,
        'guided_max': max(g_speeds) if g_speeds else 0,
    }

    targets = []
    seen = set()
    for d in data:
        gid = d.get('goal_id')
        if isinstance(gid, (int, float)) and gid > 0:
            tidx = d.get('_task_idx', 0)
            key = (round(d.get('target_x', 0), 2), round(d.get('target_y', 0), 2), int(gid), tidx)
            if key not in seen:
                seen.add(key)
                targets.append({'x': key[0], 'y': key[1], 'id': key[2], 'task_idx': tidx})
    goal_seq = []
    prev_g = None
    for d in data:
        gid = d.get('goal_id')
        if isinstance(gid, (int, float)):
            gid = int(gid)
            if gid != prev_g:
                goal_seq.append(gid)
                prev_g = gid

    distances = []
    if all(k in data[0] for k in ('pose_x', 'pose_y', 'target_x', 'target_y')):
        distances = [math.hypot(d['target_x']-d['pose_x'], d['target_y']-d['pose_y'])
                     for d in data
                     if all(isinstance(d.get(k), (int, float)) for k in ('pose_x','pose_y','target_x','target_y'))]
    if not distances:
        distances = [d.get('distance_to_goal', 0) for d in data
                     if isinstance(d.get('distance_to_goal'), (int, float))]
    min_dist = min(distances) if distances else None
    threshold = 1.5
    reach_t = None
    for i, dist in enumerate(distances):
        if dist <= threshold:
            reach_t = t[i]
            break

    ctes_guided = [abs(d.get('cross_track_error', 0)) for d in guided
                   if isinstance(d.get('cross_track_error'), (int, float))]
    cte_rms = math.sqrt(sum(x*x for x in ctes_guided)/len(ctes_guided)) if ctes_guided else None
    cte_avg = sum(ctes_guided)/len(ctes_guided) if ctes_guided else None
    cte_max = max(ctes_guided) if ctes_guided else None

    # ── 计算相对原始规划航线 (W_{k-1} -> W_k) 的真实几何横向偏差 true_cte 与出弯过冲 ──
    planned_legs = {}  # {(task_idx, goal_id): (sx, sy, gx, gy, gl)}
    last_wp_by_task = {}
    first_pose_by_task = {}
    for d in data:
        tidx = d.get('_task_idx', 0)
        if tidx not in first_pose_by_task:
            first_pose_by_task[tidx] = (float(d.get('pose_x', 0)), float(d.get('pose_y', 0)))
        gid = d.get('goal_id')
        if isinstance(gid, (int, float)) and int(gid) > 0:
            gid = int(gid)
            gkey = (tidx, gid)
            if gkey not in planned_legs:
                gx_p = float(d.get('target_x', 0))
                gy_p = float(d.get('target_y', 0))
                sx_p, sy_p = last_wp_by_task.get(tidx, first_pose_by_task[tidx])
                gl_p = math.hypot(gx_p - sx_p, gy_p - sy_p)
                planned_legs[gkey] = (sx_p, sy_p, gx_p, gy_p, gl_p)
                last_wp_by_task[tidx] = (gx_p, gy_p)

    true_cte_series = []
    min_nb_dist_series = []
    min_nb_id_series = []
    min_nb_by_peer = {}  # {nid: min_dist}
    min_nb_overall = None
    min_nb_peer_id = ''
    min_nb_time_s = None
    for idx_d, d in enumerate(data):
        tidx = d.get('_task_idx', 0)
        gid = int(d.get('goal_id', 0)) if isinstance(d.get('goal_id'), (int, float)) else 0
        px, py = float(d.get('pose_x', 0)), float(d.get('pose_y', 0))
        leg = planned_legs.get((tidx, gid))
        if leg and leg[4] > 0.25:
            sx_p, sy_p, gx_p, gy_p, gl_p = leg
            # 正值=偏航线右侧, 负值=偏航线左侧 (与 PolicyInferenceNode / cross_track_error 符号严格一致)
            t_cte = ((px - sx_p) * (gy_p - sy_p) - (py - sy_p) * (gx_p - sx_p)) / gl_p
        else:
            t_cte = float(d.get('cross_track_error', 0) or 0.0)
        true_cte_series.append(t_cte)

        best_d = None
        best_id = ''
        for slot in range(1, 6):
            nid = d.get(f'neighbor_{slot}_id')
            if not nid or (isinstance(nid, str) and not nid.strip()):
                continue
            if isinstance(nid, (int, float)) and nid == 0:
                continue
            nid_s = str(nid).strip()
            nx = d.get(f'neighbor_{slot}_x', 0)
            ny = d.get(f'neighbor_{slot}_y', 0)
            if isinstance(nx, (int, float)) and isinstance(ny, (int, float)):
                dd = math.hypot(px - nx, py - ny)
                if dd > 0.01:
                    if nid_s not in min_nb_by_peer or dd < min_nb_by_peer[nid_s]:
                        min_nb_by_peer[nid_s] = dd
                    if best_d is None or dd < best_d:
                        best_d = dd
                        best_id = nid_s
                    if d.get('flight_mode') == 'GUIDED' and (min_nb_overall is None or dd < min_nb_overall):
                        min_nb_overall = dd
                        min_nb_peer_id = nid_s
                        min_nb_time_s = t[idx_d]
        min_nb_dist_series.append(best_d)
        min_nb_id_series.append(best_id)

    # 仅在 GUIDED 且无近距避让 (min_nb_dist >= 2.8m 或无邻船) 时统计纯路径贴线精度，避免合理避让让行被误判为偏航
    unobstructed_true_ctes = [
        abs(true_cte_series[i]) for i, d in enumerate(data)
        if d.get('flight_mode') == 'GUIDED' and (min_nb_dist_series[i] is None or min_nb_dist_series[i] >= 2.8)
    ]
    all_guided_true_ctes = [
        abs(true_cte_series[i]) for i, d in enumerate(data) if d.get('flight_mode') == 'GUIDED'
    ]
    eff_ctes = unobstructed_true_ctes if len(unobstructed_true_ctes) >= 20 else (all_guided_true_ctes or ctes_guided)
    if eff_ctes:
        s_eff = sorted(eff_ctes)
        cte_true_avg = sum(s_eff) / len(s_eff)
        cte_true_p90 = s_eff[int(len(s_eff) * 0.90)]
    else:
        cte_true_avg = cte_avg
        cte_true_p90 = cte_max

    # 统计弯道圆润度与出弯过冲 (Post-turn overshoot & Turn radius / Angular jerk)
    turn_radii = []
    ang_jerks = []
    prev_om_g = None
    prev_t_g = None
    for i, d in enumerate(data):
        if d.get('flight_mode') != 'GUIDED':
            prev_om_g = None
            continue
        spd_i = float(d.get('velocity_speed', 0) or 0)
        om_act_i = float(d.get('omega_actual', d.get('cmd_omega', 0)) or 0)
        om_cmd_i = float(d.get('cmd_omega', 0) or 0)
        if spd_i > 0.10 and abs(om_act_i) > 0.14:
            turn_radii.append(spd_i / abs(om_act_i))
        if prev_om_g is not None and prev_t_g is not None:
            dt_g = d['timestamp'] - prev_t_g
            if 0.02 <= dt_g <= 0.5:
                ang_jerks.append(abs(om_cmd_i - prev_om_g) / dt_g)
        prev_om_g = om_cmd_i
        prev_t_g = d['timestamp']

    turn_r_med = sorted(turn_radii)[len(turn_radii)//2] if turn_radii else None
    turn_r_p10 = sorted(turn_radii)[int(len(turn_radii)*0.10)] if turn_radii else None
    dom_dt_p90 = sorted(ang_jerks)[int(len(ang_jerks)*0.90)] if ang_jerks else 0.0

    # 检测每次航点切换后 (无近船干扰时) 是否冲过新航线到外侧 (出弯外飘量 corner_overshoot)
    corner_overshoots = []
    cur_gkey = None
    init_cte_sgn = 0.0
    leg_max_ov = 0.0
    leg_min_nb = 99.0
    leg_idx_count = 0
    for i, d in enumerate(data):
        if d.get('flight_mode') != 'GUIDED':
            continue
        gkey = (d.get('_task_idx', 0), int(d.get('goal_id', 0)) if isinstance(d.get('goal_id'), (int, float)) else 0)
        leg = planned_legs.get(gkey)
        if gkey != cur_gkey:
            if cur_gkey is not None and leg_idx_count > 1 and init_cte_sgn != 0.0 and leg_min_nb > 2.8:
                corner_overshoots.append(leg_max_ov)
            cur_gkey = gkey
            leg_idx_count += 1
            tc0 = true_cte_series[i]
            init_cte_sgn = (1.0 if tc0 > 0.20 else (-1.0 if tc0 < -0.20 else 0.0)) if (leg and leg[4] > 2.5) else 0.0
            leg_max_ov = 0.0
            leg_min_nb = 99.0
        else:
            if min_nb_dist_series[i] is not None:
                leg_min_nb = min(leg_min_nb, min_nb_dist_series[i])
            if init_cte_sgn != 0.0 and leg:
                px, py = float(d.get('pose_x', 0)), float(d.get('pose_y', 0))
                along_i = ((px - leg[0]) * (leg[2] - leg[0]) + (py - leg[1]) * (leg[3] - leg[1])) / max(leg[4], 1e-3)
                if along_i < min(5.5, leg[4] * 0.75):
                    ov_i = -init_cte_sgn * true_cte_series[i]
                    if ov_i > leg_max_ov:
                        leg_max_ov = ov_i
    if cur_gkey is not None and leg_idx_count > 1 and init_cte_sgn != 0.0 and leg_min_nb > 2.8:
        corner_overshoots.append(leg_max_ov)
    max_corner_overshoot = max(corner_overshoots) if corner_overshoots else 0.0
    avg_corner_overshoot = (sum(corner_overshoots) / len(corner_overshoots)) if corner_overshoots else 0.0

    hes_guided = [abs(d.get('heading_error_deg', 0)) for d in guided
                  if isinstance(d.get('heading_error_deg'), (int, float))]
    he_avg = sum(hes_guided)/len(hes_guided) if hes_guided else None
    he_max = max(hes_guided) if hes_guided else None
    # 直线段航向误差 (与 analyze_nav_log 同口径: |he|<45° 视为直线段)
    hes_straight = [h for h in hes_guided if h < 45.0]
    he_straight_avg = sum(hes_straight)/len(hes_straight) if hes_straight else None

    mpc = {}
    if 'mpc_solve_time_ms' in data[0]:
        st = sorted(d.get('mpc_solve_time_ms', 0) for d in data)
        mpc = {
            'avg': sum(st)/len(st),
            'p95': st[int(len(st)*0.95)],
            'max': st[-1],
        }

    om_cmds = [d.get('cmd_omega', 0) for d in data if isinstance(d.get('cmd_omega'), (int, float))]
    _om_avg = sum(abs(o) for o in om_cmds)/len(om_cmds) if om_cmds else 0
    omega = {
        'avg': _om_avg,
        'max': max(abs(o) for o in om_cmds) if om_cmds else 0,
        'std': (math.sqrt(sum((abs(o) - _om_avg)**2 for o in om_cmds)/len(om_cmds)) if om_cmds else 0.4),
    }
    zero_crossings = sum(1 for i in range(1, len(om_cmds)) if om_cmds[i-1]*om_cmds[i] < 0) if len(om_cmds) > 20 else 0
    osc_freq = zero_crossings / (2 * duration) if duration > 0 and zero_crossings > 0 else 0

    ampc = {}
    if 'ampc_tau_estimated' in data[0]:
        ampc_on = [d for d in data if d.get('ampc_enabled', 0) == 1]
        if ampc_on:
            tau_ests = [d['ampc_tau_estimated'] for d in ampc_on
                        if isinstance(d.get('ampc_tau_estimated'), (int, float)) and d['ampc_tau_estimated'] > 0]
            confs = [d.get('ampc_tau_confidence', 0) for d in ampc_on
                     if isinstance(d.get('ampc_tau_confidence'), (int, float))]
            sat_r = [d.get('ampc_saturation_ratio', 0) for d in ampc_on
                     if isinstance(d.get('ampc_saturation_ratio'), (int, float))]
            noise = [d.get('ampc_heading_noise', 0) for d in ampc_on
                     if isinstance(d.get('ampc_heading_noise'), (int, float))]
            converged = [d for d in ampc_on if d.get('ampc_converged', 0) == 1]
            ampc = {
                'tau_min': min(tau_ests) if tau_ests else None,
                'tau_max': max(tau_ests) if tau_ests else None,
                'tau_final': tau_ests[-1] if tau_ests else None,
                'confidence_final': confs[-1] if confs else None,
                'converge_time': converged[0]['timestamp'] - t0 if converged else None,
                'sat_avg': sum(sat_r)/len(sat_r) if sat_r else 0,
                'sat_max': max(sat_r) if sat_r else 0,
                'noise_avg': sum(noise)/len(noise) if noise else 0,
                'rebuild_count': int(max(d.get('ampc_rebuild_count', 0) for d in ampc_on)),
            }

    rl = {}
    if 'raw_cmd_vx' in data[0] or 'rl_cmd_vx' in data[0] or 'rl_delta_vx' in data[0]:
        active_samples = []
        delta_vx_vals = []
        delta_omega_vals = []
        neighbor_counts = []
        for d in data:
            active = int(d.get('rl_cmd_active', 0)) == 1 if isinstance(d.get('rl_cmd_active'), (int, float)) else False
            if not active:
                active = abs(float(d.get('rl_cmd_vx', 0) or 0)) > 1e-4 or abs(float(d.get('rl_cmd_omega', 0) or 0)) > 1e-4
            if active:
                active_samples.append(d)
            delta_vx_vals.append(float(d.get('rl_delta_vx', d.get('cmd_vx', 0) - d.get('raw_cmd_vx', 0)) or 0))
            delta_omega_vals.append(float(d.get('rl_delta_omega', d.get('cmd_omega', 0) - d.get('raw_cmd_omega', 0)) or 0))
            neighbor_count = 0
            for slot in range(1, 6):
                nid = d.get(f'neighbor_{slot}_id')
                if isinstance(nid, str) and nid.strip():
                    neighbor_count += 1
                elif isinstance(nid, (int, float)) and nid != 0:
                    neighbor_count += 1
            neighbor_counts.append(neighbor_count)

        active_count = len(active_samples)
        rl_age = [
            float(d.get('rl_cmd_age_s'))
            for d in data
            if isinstance(d.get('rl_cmd_age_s'), (int, float)) and d.get('rl_cmd_age_s', -1) >= 0
        ]
        rl = {
            'active_count': active_count,
            'active_pct': active_count / len(data) * 100 if data else 0,
            'mean_abs_delta_vx': sum(abs(v) for v in delta_vx_vals) / len(delta_vx_vals) if delta_vx_vals else 0,
            'mean_abs_delta_omega': sum(abs(v) for v in delta_omega_vals) / len(delta_omega_vals) if delta_omega_vals else 0,
            'max_abs_delta_vx': max((abs(v) for v in delta_vx_vals), default=0),
            'max_abs_delta_omega': max((abs(v) for v in delta_omega_vals), default=0),
            'neighbor_nonzero_pct': sum(1 for count in neighbor_counts if count > 0) / len(neighbor_counts) * 100 if neighbor_counts else 0,
            'age_p95': sorted(rl_age)[int(len(rl_age) * 0.95)] if rl_age else None,
        }

    wifi = {}
    if 'wifi_rssi_dbm' in data[0]:
        vals = [d.get('wifi_rssi_dbm', -100) for d in data
                if isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100]
        if vals:
            avg_rssi = sum(vals)/len(vals)
            wifi = {
                'avg': avg_rssi, 'min': min(vals), 'max': max(vals),
                'std': math.sqrt(sum((x - avg_rssi)**2 for x in vals)/len(vals)),
                'grade': 'Excellent' if avg_rssi >= -50 else 'Good' if avg_rssi >= -60 else 'Fair' if avg_rssi >= -70 else 'Weak',
            }

    per_goal = []
    goals_map = {}  # key: (task_idx, goal_id)
    for d in data:
        gid = d.get('goal_id')
        if isinstance(gid, (int, float)):
            gid = int(gid)
            tidx = d.get('_task_idx', 0)
            gkey = (tidx, gid)
            if gkey not in goals_map:
                goals_map[gkey] = []
            goals_map[gkey].append(d)
    sorted_gids = sorted(goals_map.keys())

    # === 构建全局到达事件映射 ===
    # nav_event 格式: "passed:GOAL_ID:消息" 或 "arrived:GOAL_ID:消息"
    # 事件记录在下一个 goal 的第一行数据中，按 goal_id 分组会归属到错误的 goal。
    # 因此先遍历所有数据，提取事件并按其引用的 goal_id 建立映射。
    import re as _re
    _arrival_event_map = {}  # {(task_idx, referenced_goal_id): event_string}
    for d in data:
        evt = d.get('nav_event', '')
        if not evt:
            continue
        evt_s = str(evt)
        m = _re.match(r'"?(passed|arrived):(\d+):', evt_s)
        if m:
            ref_gid = int(m.group(2))
            tidx = d.get('_task_idx', 0)
            _arrival_event_map[(tidx, ref_gid)] = evt_s

    for gi, gkey in enumerate(sorted_gids):
        tidx_g, gid = gkey
        gdata = goals_map[gkey]
        dur = gdata[-1]['timestamp'] - gdata[0]['timestamp']
        gg = [d for d in gdata if d.get('flight_mode') == 'GUIDED']
        dists_g = [d.get('distance_to_goal', 999) for d in gdata if isinstance(d.get('distance_to_goal'), (int, float))]
        ctes_g = [abs(d.get('cross_track_error', 0)) for d in gg if isinstance(d.get('cross_track_error'), (int, float))]
        hes_g = [abs(d.get('heading_error_deg', 0)) for d in gg if isinstance(d.get('heading_error_deg'), (int, float))]
        spds_g = [d.get('velocity_speed', 0) for d in gg if isinstance(d.get('velocity_speed'), (int, float))]
        min_d = min(dists_g) if dists_g else None

        # === 到达判断 (数据驱动) ===
        # 使用全局事件映射: 按事件中引用的 goal_id 匹配，而非按 goal 分组内的行
        own_event = _arrival_event_map.get((tidx_g, gid), '')
        has_arrived_event = 'arrived:' in own_event
        has_passed_event = 'passed:' in own_event

        if has_arrived_event:
            # V15+: nav_event 明确标记为 arrived
            arrival_status = 'arrived'
        elif has_passed_event:
            # V15+: nav_event 标记为 passed (中间航点通过)
            # 检查通过时的 message 内容区分"平滑切换"vs"偏离补救"
            # 注意: CSV 简单逗号分割可能截断引号内的事件文本，
            #       导致 "偏离" 关键词丢失，需用 min_d 距离兜底
            arrival_threshold = 1.5
            if '偏离' in own_event:
                arrival_status = 'diverged'
            elif min_d is not None and min_d > arrival_threshold:
                # 事件文本被截断但距离表明未真正抵达
                arrival_status = 'diverged'
            else:
                arrival_status = 'passed'
        elif own_event:
            # 有其他事件但非 arrived/passed
            arrival_status = 'unknown'
        else:
            # 无事件: V14及以前日志 或 第一个/最后一个 goal 无明确事件
            arrival_threshold = 1.5
            if min_d is not None and min_d <= arrival_threshold:
                arrival_status = 'arrived'
            elif gi < len(sorted_gids) - 1:
                # 非最后一个goal: 控制器切换了但USV未抵近
                arrival_status = 'diverged'
            else:
                # 最后一个goal: 检查最终 distance
                if min_d is not None and min_d <= arrival_threshold:
                    arrival_status = 'arrived'
                else:
                    arrival_status = 'not_reached'

        pg = {
            'id': gid, 'task_idx': tidx_g, 'duration': dur,
            'min_dist': min_d,
            'arrival': arrival_status,
            'avg_cte': sum(ctes_g)/len(ctes_g) if ctes_g else None,
            'avg_he': sum(hes_g)/len(hes_g) if hes_g else None,
            'avg_speed': sum(spds_g)/len(spds_g) if spds_g else None,
            'guided_pct': len(gg)/len(gdata)*100 if gdata else 0,
        }

        # === 每航段几何指标 (直线贴线优先判定, 2026-07-08) ===
        # 降采样去高频噪声 (10Hz -> 1Hz)
        _ds = gdata[::10] + ([gdata[-1]] if (len(gdata) - 1) % 10 else [])
        _xs = [d.get('pose_x') for d in _ds if isinstance(d.get('pose_x'), (int, float))]
        _ys = [d.get('pose_y') for d in _ds if isinstance(d.get('pose_y'), (int, float))]
        _yaws = [d.get('pose_yaw_deg') for d in _ds if isinstance(d.get('pose_yaw_deg'), (int, float))]
        if len(_xs) >= 3 and len(_xs) == len(_ys):
            _path = sum(math.hypot(_xs[i+1]-_xs[i], _ys[i+1]-_ys[i]) for i in range(len(_xs)-1))
            _tx = gdata[0].get('target_x'); _ty = gdata[0].get('target_y')
            if isinstance(_tx, (int, float)) and isinstance(_ty, (int, float)):
                _dg_end = gdata[-1].get('distance_to_goal')
                _dg_end = _dg_end if isinstance(_dg_end, (int, float)) else 0.0
                _straight = max(1.0, math.hypot(_tx-_xs[0], _ty-_ys[0]) - _dg_end)
                pg['detour'] = _path / _straight
        if len(_yaws) >= 3:
            def _unw(p, c):
                dd = c - p
                while dd > 180: dd -= 360
                while dd < -180: dd += 360
                return dd
            _acc = 0.0; _loops = 0; _net = 0.0
            _peak = 0.0; _sacc = 0.0
            _t0 = _ds[0].get('timestamp', 0)
            for _i in range(len(_yaws)-1):
                _d = _unw(_yaws[_i], _yaws[_i+1])
                _net += abs(_d)
                _acc += _d
                if abs(_acc) >= 360.0:
                    _loops += 1; _acc = 0.0
                # 切换后 15s 内的累计回转峰值 (顶点切换瞬态)
                if _ds[_i+1].get('timestamp', 0) - _t0 <= 15.0:
                    _sacc += _d
                    _peak = max(_peak, abs(_sacc))
            pg['net_turn'] = _net
            pg['loops'] = _loops
            pg['switch_peak'] = _peak

        wv = [d.get('wifi_rssi_dbm', -100) for d in gdata
              if isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100]
        if wv:
            pg['wifi_avg'] = sum(wv)/len(wv)
        per_goal.append(pg)

    # ── 老司机八维综合评估体系 (0-100, 适配 0.60m×0.30m 纯 RL 多船/单船导航) ──
    # 1) 航线贴合 (22%): 无遮挡 GUIDED 真实几何 |CTE| 均值与 P90 (反映直道贴线与转弯后是否偏离原航线)
    _c_avg = cte_true_avg if cte_true_avg is not None else 0.65
    _c_p90 = cte_true_p90 if cte_true_p90 is not None else 1.10
    _s_cte_avg = max(0.0, min(100.0, (1.0 - (_c_avg - 0.06) / (0.60 - 0.06)) * 100.0))
    _s_cte_p90 = max(0.0, min(100.0, (1.0 - (_c_p90 - 0.18) / (1.05 - 0.18)) * 100.0))
    radar_tracking = 0.65 * _s_cte_avg + 0.35 * _s_cte_p90

    # 2) 转向圆润 (16%): 出弯零外飘 + 转弯半径饱满度(拒绝原地拧尖角) + 角加速度平滑度
    _s_ov = max(0.0, min(100.0, (1.0 - (max_corner_overshoot - 0.10) / (0.85 - 0.10)) * 100.0))
    if turn_r_p10 is not None:
        # 0.60m 船模: R_p10 >= 0.48m 圆润满分, <= 0.18m 原地急折 0 分
        _s_rad = max(0.0, min(100.0, (turn_r_p10 - 0.18) / (0.48 - 0.18) * 100.0))
    else:
        _s_rad = 95.0
    _s_jerk = max(0.0, min(100.0, (1.0 - (dom_dt_p90 - 0.30) / (1.80 - 0.30)) * 100.0))
    radar_roundness = 0.45 * _s_ov + 0.30 * _s_rad + 0.25 * _s_jerk

    # 3) 安全避让 (20%): 0.60×0.30m 船模真实邻船最小质心距 (无邻船单船任务记 100)
    if min_nb_overall is None:
        radar_safety = 100.0
    elif min_nb_overall >= 0.80:
        radar_safety = 100.0
    elif min_nb_overall >= 0.60:
        radar_safety = 82.0 + (min_nb_overall - 0.60) / 0.20 * 18.0
    elif min_nb_overall >= 0.35:
        radar_safety = 20.0 + (min_nb_overall - 0.35) / 0.25 * 62.0
    else:
        radar_safety = max(0.0, (min_nb_overall - 0.22) / 0.13 * 20.0)

    # 4) 航点达成 (16%): 中途航点通过率×50% + 终点到达×50%
    if per_goal:
        _mid = per_goal[:-1]
        _mid_ok = sum(1 for g in _mid if g.get('arrival') in ('arrived', 'passed'))
        _mid_score = (_mid_ok / len(_mid) * 100.0) if _mid else 100.0
        _final_ok = per_goal[-1].get('arrival') in ('arrived', 'passed')
        radar_progress = _mid_score * 0.5 + (100.0 if _final_ok else 0.0) * 0.5
        goals_ok_count = _mid_ok + (1 if _final_ok else 0)
    else:
        radar_progress = 0.0
        goals_ok_count = 0

    # 5) 绕行效率 (10%): 无死锁绕圈 + 紧凑绕行弧长比 (detour <= 1.12 满分)
    _loops_total = sum(g.get('loops', 0) for g in per_goal)
    _detours = [g['detour'] for g in per_goal if isinstance(g.get('detour'), (int, float))]
    _detour_excess = (sum(max(0.0, dv - 1.12) for dv in _detours) / len(_detours)) if _detours else 0.0
    _peaks = [g['switch_peak'] for g in per_goal if isinstance(g.get('switch_peak'), (int, float))]
    _worst_peak = max(_peaks) if _peaks else 0.0
    _peak_pen = max(0.0, (_worst_peak - 185.0) / 8.0)
    radar_efficiency = max(0.0, min(100.0, 100.0 - _loops_total * 35.0 - _detour_excess * 95.0 - _peak_pen))

    # 6) 航向稳定 (6%): 直道航向误差 + 低过零摆动频率
    _he_st = he_straight_avg if he_straight_avg is not None else 30.0
    _s_he = max(0.0, min(100.0, (1.0 - (_he_st - 4.0) / (28.0 - 4.0)) * 100.0))
    _s_osc = max(0.0, min(100.0, (1.0 - max(0.0, osc_freq - 0.10) / 0.40) * 100.0))
    radar_heading = 0.70 * _s_he + 0.30 * _s_osc

    # 7) 巡航效率 (5%): 自适应对比指令巡航速度 (cmd_vx P85 或默认 0.50m/s)
    _g_cmd_vx = sorted(float(d.get('cmd_vx', 0) or 0) for d in guided if float(d.get('cmd_vx', 0) or 0) > 0.12)
    _target_cruise = _g_cmd_vx[int(len(_g_cmd_vx) * 0.85)] if _g_cmd_vx else 0.50
    _target_cruise = max(0.30, min(0.65, _target_cruise))
    radar_speed = max(0.0, min(100.0, (velocity['guided_avg'] / (_target_cruise * 0.90)) * 100.0)) if velocity['guided_avg'] > 0 else 0.0

    # 8) 控制平顺 (5%): 无饱和正负跳变 + 角速度平滑度
    _bb_flips = sum(
        1 for i in range(1, len(om_cmds))
        if abs(om_cmds[i]) >= 0.52 and abs(om_cmds[i-1]) >= 0.52 and om_cmds[i] * om_cmds[i-1] < 0
    )
    _s_bb = max(0.0, 100.0 - _bb_flips * 12.0)
    radar_smooth = 0.55 * _s_jerk + 0.45 * _s_bb

    total_score = (
        radar_tracking * 0.22
        + radar_safety * 0.20
        + radar_roundness * 0.16
        + radar_progress * 0.16
        + radar_efficiency * 0.10
        + radar_heading * 0.06
        + radar_speed * 0.05
        + radar_smooth * 0.05
    )
    # 碰撞硬约束: 若最小质心距 < 0.35m (船壳擦碰风险), 总分封顶 68 分 (不高于 B)
    if min_nb_overall is not None and min_nb_overall < 0.35:
        total_score = min(total_score, 68.0)

    grade = 'A+' if total_score >= 85 else 'A' if total_score >= 75 else 'B' if total_score >= 62 else 'C' if total_score >= 45 else 'D'

    scoring = {
        'cte': radar_tracking, 'he': radar_heading, 'mpc': 100.0, 'spd': radar_speed,
        'total': total_score, 'grade': grade,
        'cte_label': '优秀' if _c_avg < 0.12 else '良好' if _c_avg < 0.25 else '一般' if _c_avg < 0.45 else '偏高',
        'he_label': '优秀' if _he_st < 5 else '良好' if _he_st < 10 else '一般' if _he_st < 18 else '偏大',
        'mpc_label': '优秀' if mpc.get('avg', 0) < 15 else '正常' if mpc.get('avg', 0) < 30 else '偏高',
        'radar': {
            'tracking': round(radar_tracking, 1),
            'roundness': round(radar_roundness, 1),
            'safety': round(radar_safety, 1),
            'progress': round(radar_progress, 1),
            'efficiency': round(radar_efficiency, 1),
            'heading': round(radar_heading, 1),
            'speed': round(radar_speed, 1),
            'smooth': round(radar_smooth, 1),
        },
    }

    # 构建每任务规划航线折线列表 [(sx,sy), (g1x,g1y), ...] 供前端轨迹图与回放绘制基准线
    planned_routes = {}  # {str(task_idx): [[x, y], ...]}
    for (tidx_r, gid_r) in sorted(planned_legs.keys()):
        sx_p, sy_p, gx_p, gy_p, _ = planned_legs[(tidx_r, gid_r)]
        pts_r = planned_routes.setdefault(str(tidx_r), [[round(sx_p, 3), round(sy_p, 3)]])
        pts_r.append([round(gx_p, 3), round(gy_p, 3)])

    # ── 时间序列 (降采样) ──
    step = max(1, len(data) // 2000)
    ts_data = {
        'time': [t[i] for i in range(0, len(data), step)],
        'pose_x': [data[i]['pose_x'] for i in range(0, len(data), step)],
        'pose_y': [data[i]['pose_y'] for i in range(0, len(data), step)],
        'speed': [data[i]['velocity_speed'] for i in range(0, len(data), step)],
        'cmd_vx': [data[i].get('cmd_vx', 0) for i in range(0, len(data), step)],
        'heading_error': [data[i].get('heading_error_deg', 0) for i in range(0, len(data), step)],
        'cte': [data[i].get('cross_track_error', 0) for i in range(0, len(data), step)],
        'true_cte': [round(true_cte_series[i], 4) for i in range(0, len(data), step)],
        'min_nb_dist': [round(min_nb_dist_series[i], 3) if min_nb_dist_series[i] is not None else None for i in range(0, len(data), step)],
        'min_nb_id': [min_nb_id_series[i] for i in range(0, len(data), step)],
        'planned_routes': planned_routes,
        'distance': [],
        'flight_mode': [data[i].get('flight_mode', '') for i in range(0, len(data), step)],
        'yaw_deg': [data[i].get('pose_yaw_deg', 0) for i in range(0, len(data), step)],
        'goal_id': [data[i].get('goal_id', 0) for i in range(0, len(data), step)],
        'task_idx': [data[i].get('_task_idx', 0) for i in range(0, len(data), step)],
    }
    if distances and len(distances) == len(data):
        ts_data['distance'] = [distances[i] for i in range(0, len(data), step)]
    elif distances:
        ts_data['distance'] = [data[i].get('distance_to_goal', 0) for i in range(0, len(data), step)]
    if 'mpc_solve_time_ms' in data[0]:
        ts_data['mpc_time'] = [data[i].get('mpc_solve_time_ms', 0) for i in range(0, len(data), step)]
    if 'cmd_omega' in data[0]:
        ts_data['cmd_omega'] = [data[i].get('cmd_omega', 0) for i in range(0, len(data), step)]
    if 'raw_cmd_vx' in data[0]:
      ts_data['raw_cmd_vx'] = [data[i].get('raw_cmd_vx', 0) for i in range(0, len(data), step)]
      ts_data['raw_cmd_vy'] = [data[i].get('raw_cmd_vy', 0) for i in range(0, len(data), step)]
      ts_data['raw_cmd_omega'] = [data[i].get('raw_cmd_omega', 0) for i in range(0, len(data), step)]
      ts_data['rl_delta_vx'] = [data[i].get('rl_delta_vx', data[i].get('cmd_vx', 0) - data[i].get('raw_cmd_vx', 0)) for i in range(0, len(data), step)]
      ts_data['rl_delta_vy'] = [data[i].get('rl_delta_vy', data[i].get('cmd_vy', 0) - data[i].get('raw_cmd_vy', 0)) for i in range(0, len(data), step)]
      ts_data['rl_delta_omega'] = [data[i].get('rl_delta_omega', data[i].get('cmd_omega', 0) - data[i].get('raw_cmd_omega', 0)) for i in range(0, len(data), step)]
    if 'rl_cmd_vx' in data[0]:
      ts_data['rl_cmd_vx'] = [data[i].get('rl_cmd_vx', 0) for i in range(0, len(data), step)]
      ts_data['rl_cmd_vy'] = [data[i].get('rl_cmd_vy', 0) for i in range(0, len(data), step)]
      ts_data['rl_cmd_omega'] = [data[i].get('rl_cmd_omega', 0) for i in range(0, len(data), step)]
    if 'rl_cmd_active' in data[0]:
      ts_data['rl_active'] = [data[i].get('rl_cmd_active', 0) for i in range(0, len(data), step)]
    if 'rl_cmd_age_s' in data[0]:
      ts_data['rl_cmd_age_s'] = [data[i].get('rl_cmd_age_s', -1) for i in range(0, len(data), step)]
    if 'rl_encounter_type_index' in data[0]:
      ts_data['rl_encounter_type'] = [int(data[i].get('rl_encounter_type_index', -1)) for i in range(0, len(data), step)]
    if 'omega_actual' in data[0]:
        ts_data['omega_actual'] = [data[i].get('omega_actual', 0) for i in range(0, len(data), step)]
    if 'current_tau_omega' in data[0]:
        ts_data['tau'] = [data[i].get('current_tau_omega', 0) for i in range(0, len(data), step)]
    if 'ampc_tau_estimated' in data[0]:
        ts_data['tau_estimated'] = [data[i].get('ampc_tau_estimated', 0) for i in range(0, len(data), step)]
        ts_data['tau_confidence'] = [data[i].get('ampc_tau_confidence', 0) for i in range(0, len(data), step)]
    if 'wifi_rssi_dbm' in data[0]:
        ts_data['wifi_rssi'] = [data[i].get('wifi_rssi_dbm', -100) for i in range(0, len(data), step)]
    # v16: 提取邻居USV位置时间序列 (用于回放中显示其他USV的实时位置)
    if 'neighbor_1_id' in data[0]:
        neighbor_ts = {}  # {neighbor_usv_id: {time: [], x: [], y: [], yaw: []}}
        for i in range(0, len(data), step):
            d = data[i]
            ti = t[i]
            for slot in range(1, 6):
                nid = d.get(f'neighbor_{slot}_id')
                if not nid or (isinstance(nid, str) and not nid.strip()):
                    continue
                if isinstance(nid, (int, float)) and nid == 0:
                    continue
                nid_str = str(nid) if not isinstance(nid, str) else nid
                if nid_str not in neighbor_ts:
                    neighbor_ts[nid_str] = {'time': [], 'x': [], 'y': [], 'yaw': []}
                nb = neighbor_ts[nid_str]
                nb['time'].append(ti)
                nb['x'].append(d.get(f'neighbor_{slot}_x', 0))
                nb['y'].append(d.get(f'neighbor_{slot}_y', 0))
                nb['yaw'].append(d.get(f'neighbor_{slot}_yaw', 0))
        if neighbor_ts:
            ts_data['neighbors'] = neighbor_ts

    return {
        'info': info, 'sampling': sampling, 'velocity': velocity,
        'targets': targets, 'goal_seq': goal_seq,
        'distance': {'min': min_dist, 'end': distances[-1] if distances else None, 'reach_t': reach_t},
        'cte': {'avg': cte_avg, 'max': cte_max, 'rms': cte_rms, 'true_avg': cte_true_avg, 'true_p90': cte_true_p90},
        'heading_error': {'avg': he_avg, 'max': he_max, 'straight_avg': he_straight_avg},
        'corner': {'max_overshoot': max_corner_overshoot, 'avg_overshoot': avg_corner_overshoot,
                   'r_med': turn_r_med, 'r_p10': turn_r_p10, 'dom_dt_p90': dom_dt_p90},
        'safety': {'min_dist': min_nb_overall, 'peer_id': min_nb_peer_id, 'time_s': min_nb_time_s,
                   'by_peer': {k: round(v, 3) for k, v in sorted(min_nb_by_peer.items())}},
        'goals_ok': goals_ok_count, 'goals_total': len(per_goal),
        'mpc': mpc, 'omega': omega, 'osc_freq': osc_freq,
        'ampc': ampc, 'rl': rl, 'wifi': wifi,
        'per_goal': per_goal, 'scoring': scoring,
        'ts': ts_data,
    }


# ─── 工具函数 ──────────────────────────────────────────────────

def _safe_json(obj):
    """安全 JSON 序列化"""
    def default(o):
        if o is None:
            return None
        if isinstance(o, float):
            if math.isnan(o) or math.isinf(o):
                return None
        raise TypeError(f"Object of type {type(o)} is not JSON serializable")
    return json.dumps(obj, default=default, ensure_ascii=False)


def _fmt(val, fmt_spec='.2f', suffix='', na='N/A'):
    if val is None:
        return na
    try:
        return f"{val:{fmt_spec}}{suffix}"
    except (ValueError, TypeError):
        return str(val)


USV_COLORS = [
    '#3498db', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6',
    '#1abc9c', '#e67e22', '#34495e', '#e91e63', '#00bcd4',
    '#8bc34a', '#ff5722',
]


def _shift_ts(ts: dict, offset: float) -> dict:
    """把一艘船的时间序列平移到公共时间轴 (邻船序列的时间以观察者自身 t0 为基准, 同样平移)。"""
    if not ts or abs(offset) < 1e-9:
        return ts
    shifted = dict(ts)
    shifted['time'] = [x + offset for x in ts['time']]
    if ts.get('neighbors'):
        shifted['neighbors'] = {
            nid: {**nb, 'time': [x + offset for x in nb['time']]}
            for nid, nb in ts['neighbors'].items()
        }
    return shifted


# ─── 公开接口 ──────────────────────────────────────────────────

def generate_multi_html_report(all_report_data: list, output_path: Path, title: str = ''):
    """生成多USV对比交互式 HTML 报告"""
    if not all_report_data:
        return None

    for i, rd in enumerate(all_report_data):
        rd['_color'] = USV_COLORS[i % len(USV_COLORS)]
        rd['_idx'] = i

    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    n_usv = len(all_report_data)

    usv_list = []
    for rd in all_report_data:
        info = rd['info']
        sc = rd['scoring']
        saf = rd.get('safety', {})
        cor = rd.get('corner', {})
        mpc_val = rd['mpc'].get('avg', 0) if rd.get('mpc') else 0
        usv_list.append({
            'idx': rd['_idx'],
            'id': info['usv_id'],
            'color': rd['_color'],
            'csv': rd.get('csv_name', ''),
            'version': info['version'],
            'grade': sc['grade'],
            'score': round(sc['total'], 1),
            'duration': round(info['duration_s'], 1),
            'records': info['record_count'],
            'targets_n': rd.get('goals_total', len(rd['targets'])),
            'goals_ok': rd.get('goals_ok', len(rd['targets'])),
            'guided_pct': round(info['guided_pct'], 1),
            'cte_avg': rd['cte'].get('true_avg', rd['cte'].get('avg')),
            'cte_p90': rd['cte'].get('true_p90'),
            'he_avg': rd['heading_error'].get('straight_avg', rd['heading_error'].get('avg')),
            'speed_avg': round(rd['velocity']['guided_avg'], 3),
            'min_nb': round(saf['min_dist'], 3) if saf.get('min_dist') is not None else None,
            'min_nb_peer': saf.get('peer_id', ''),
            'min_nb_t': round(saf['time_s'], 1) if saf.get('time_s') is not None else None,
            'max_ov': round(cor.get('max_overshoot', 0), 3),
            'r_med': round(cor['r_med'], 2) if cor.get('r_med') is not None else None,
            'r_p10': round(cor['r_p10'], 2) if cor.get('r_p10') is not None else None,
            'mpc_avg': round(mpc_val, 1) if mpc_val and mpc_val > 0.05 else None,
            'rl_active_pct': round(rd['rl'].get('active_pct', 0), 1) if rd.get('rl') else 0,
            'wifi_avg': round(rd['wifi'].get('avg', -100), 1) if rd['wifi'] else None,
            'wifi_grade': rd['wifi'].get('grade', '') if rd['wifi'] else '',
        })

    # 各船日志起始时刻不同 (如缺少第一步日志), 统一到最早起始时刻
    _starts = [rd['info'].get('start_ts') for rd in all_report_data]
    _valid_starts = [s for s in _starts if isinstance(s, (int, float))]
    global_t0 = min(_valid_starts) if _valid_starts else 0.0

    all_ts, all_targets, all_per_goal, all_summaries = {}, {}, {}, {}
    for rd in all_report_data:
        key = str(rd['_idx'])
        _s = rd['info'].get('start_ts')
        offset_s = (_s - global_t0) if isinstance(_s, (int, float)) else 0.0
        all_ts[key] = _shift_ts(rd['ts'], offset_s)
        all_targets[key] = rd['targets']
        all_per_goal[key] = rd['per_goal']
        saf_shifted = dict(rd.get('safety', {}))
        if saf_shifted.get('time_s') is not None:
            saf_shifted['time_s'] = saf_shifted['time_s'] + offset_s
        all_summaries[key] = {
            'scoring': rd['scoring'],
            'velocity': rd['velocity'],
            'cte': rd['cte'],
            'heading_error': rd['heading_error'],
            'corner': rd.get('corner', {}),
            'safety': saf_shifted,
            'goals_ok': rd.get('goals_ok', 0),
            'goals_total': rd.get('goals_total', 0),
            'mpc': rd['mpc'],
            'omega': rd['omega'],
            'osc_freq': rd['osc_freq'],
            'ampc': rd['ampc'],
            'rl': rd['rl'],
            'wifi': rd['wifi'],
            'distance': rd['distance'],
            'sampling': rd['sampling'],
            'info': rd['info'],
        }

    report_title = title or f'USV 多航次分析报告 ({n_usv} 条记录)'

    html = _build_html(
        report_title, now, n_usv,
        _safe_json(usv_list),
        _safe_json(all_ts),
        _safe_json(all_targets),
        _safe_json(all_per_goal),
        _safe_json(all_summaries),
    )

    html_path = output_path / 'report.html'
    html_path.write_text(html, encoding='utf-8')
    return html_path


def generate_html_report(report_data: dict, output_path: Path, csv_name: str = ''):
    """兼容单文件模式"""
    report_data['csv_name'] = csv_name
    return generate_multi_html_report(
        [report_data], output_path,
        title=f"USV 导航分析报告 - {report_data['info']['usv_id']}")


# ─── HTML 模板构建 ─────────────────────────────────────────────

def _build_html(report_title, now, n_usv, usv_list_json,
                all_ts_json, all_targets_json, all_per_goal_json,
                all_summaries_json):
    # 使用原始字符串拼接避免 f-string 中 JS 大括号冲突
    # 所有 { } 在 f-string 中需要 {{ }}
    return f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{report_title}</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
:root {{
  --bg:#0a0e17; --bg2:#111827; --bg-card:rgba(17,24,39,0.75);
  --accent:#64FFDA; --accent2:#82B1FF; --text:#e0e0e0; --text2:#9e9e9e;
  --border:rgba(100,255,218,0.15); --glow:rgba(100,255,218,0.25);
}}
*{{ margin:0; padding:0; box-sizing:border-box; }}
body {{
  font-family:'Inter','Segoe UI','PingFang SC','Microsoft YaHei',system-ui,sans-serif;
  background:var(--bg); color:var(--text); line-height:1.6;
  background-image:
    radial-gradient(ellipse at 20% 50%,rgba(100,255,218,0.04) 0%,transparent 50%),
    radial-gradient(ellipse at 80% 20%,rgba(130,177,255,0.04) 0%,transparent 50%);
}}
.container {{ max-width:1400px; margin:0 auto; padding:20px; }}
.banner {{
  text-align:center; padding:36px 20px 24px;
  background:linear-gradient(135deg,rgba(100,255,218,0.08) 0%,rgba(130,177,255,0.08) 100%);
  border-bottom:1px solid var(--border); position:relative; overflow:hidden;
}}
.banner::before {{
  content:''; position:absolute; top:-50%; left:-50%; width:200%; height:200%;
  background:conic-gradient(from 0deg,transparent,rgba(100,255,218,0.03),transparent 30%);
  animation:bannerRotate 20s linear infinite;
}}
@keyframes bannerRotate {{ to {{ transform:rotate(360deg); }} }}
.banner h1 {{ font-size:1.7rem; font-weight:700; color:var(--accent); position:relative; }}
.banner .subtitle {{ color:var(--text2); font-size:0.82rem; margin-top:6px; position:relative; }}
.section {{
  margin:24px 0; opacity:0; transform:translateY(30px);
  transition:opacity 0.6s ease,transform 0.6s ease;
}}
.section.visible {{ opacity:1; transform:translateY(0); }}
.section-title {{
  font-size:1.15rem; font-weight:700; color:var(--accent);
  padding-bottom:8px; margin-bottom:14px;
  border-bottom:2px solid var(--border);
  display:flex; align-items:center; gap:10px;
}}
.section-title .num {{
  display:inline-flex; align-items:center; justify-content:center;
  width:24px; height:24px; border-radius:50%; background:var(--accent);
  color:var(--bg); font-size:0.75rem; font-weight:700;
}}
.usv-selector {{
  display:flex; flex-wrap:wrap; gap:8px; margin:12px 0; justify-content:center;
}}
.usv-chip {{
  display:inline-flex; align-items:center; gap:5px;
  padding:7px 14px; border-radius:18px;
  background:var(--bg-card); border:2px solid var(--border);
  cursor:pointer; transition:all 0.3s; font-size:0.82rem;
  backdrop-filter:blur(8px); user-select:none;
}}
.usv-chip:hover {{ transform:translateY(-2px); box-shadow:0 4px 16px rgba(0,0,0,0.3); }}
.usv-chip.active {{ border-color:var(--chip-color,var(--accent)); box-shadow:0 0 14px var(--chip-glow,rgba(100,255,218,0.3)); }}
.usv-chip .dot {{ width:9px; height:9px; border-radius:50%; }}
.usv-chip .grade {{ font-weight:700; font-size:0.85rem; }}
.btn-toggle-all {{
  padding:5px 12px; border-radius:14px; border:1px solid var(--accent);
  background:transparent; color:var(--accent); cursor:pointer;
  font-size:0.78rem; transition:all 0.2s;
}}
.btn-toggle-all:hover {{ background:rgba(100,255,218,0.15); }}
.compare-table {{ width:100%; border-collapse:collapse; font-size:0.8rem; }}
.compare-table th {{
  background:rgba(100,255,218,0.1); color:var(--accent);
  padding:8px 6px; text-align:center; border-bottom:2px solid var(--border);
  position:sticky; top:0; z-index:2; cursor:pointer; user-select:none; white-space:nowrap;
  transition:background 0.2s;
}}
.compare-table th:hover {{ background:rgba(100,255,218,0.2); }}
.compare-table td {{
  padding:6px 5px; text-align:center; border-bottom:1px solid rgba(255,255,255,0.05); white-space:nowrap;
}}
.compare-table tr:hover td {{ background:rgba(100,255,218,0.05); }}
.compare-table .best {{ color:#69F0AE; font-weight:700; }}
.compare-table .warn {{ color:#FFB300; font-weight:600; }}
.compare-table .danger {{ color:#FF5252; font-weight:700; }}
.top-nav {{
  position:sticky; top:0; z-index:900;
  background:rgba(10,14,23,0.88); backdrop-filter:blur(14px);
  border-bottom:1px solid var(--border); padding:8px 16px;
  display:flex; align-items:center; justify-content:center; gap:8px; flex-wrap:wrap;
}}
.top-nav a {{
  color:var(--text2); text-decoration:none; font-size:0.76rem;
  padding:4px 11px; border-radius:14px; border:1px solid rgba(255,255,255,0.08);
  transition:all 0.2s;
}}
.top-nav a:hover {{ color:var(--accent); border-color:var(--accent); background:rgba(100,255,218,0.08); }}
.kpi-grid {{
  display:grid; grid-template-columns:repeat(auto-fit,minmax(195px,1fr));
  gap:12px; margin:14px 0 18px;
}}
.kpi-card {{
  background:linear-gradient(145deg,rgba(21,30,48,0.85),rgba(13,19,32,0.85));
  border:1px solid var(--border); border-radius:12px; padding:14px 16px;
  position:relative; overflow:hidden; backdrop-filter:blur(10px);
  transition:transform 0.25s,box-shadow 0.25s;
}}
.kpi-card:hover {{ transform:translateY(-2px); box-shadow:0 6px 20px rgba(0,0,0,0.35); }}
.kpi-card .kpi-label {{ font-size:0.73rem; color:var(--text2); letter-spacing:0.3px; }}
.kpi-card .kpi-val {{ font-size:1.45rem; font-weight:800; color:var(--accent); margin:4px 0 2px; line-height:1.2; }}
.kpi-card .kpi-sub {{ font-size:0.71rem; color:var(--text2); }}
.insight-grid {{
  display:grid; grid-template-columns:repeat(auto-fit,minmax(320px,1fr));
  gap:10px; margin-bottom:14px;
}}
.insight-card {{
  background:var(--bg-card); border:1px solid var(--border); border-left:4px solid var(--accent);
  border-radius:10px; padding:10px 14px; font-size:0.77rem;
}}
.insight-tags {{ display:flex; flex-wrap:wrap; gap:6px; margin-top:6px; }}
.itag {{
  padding:2px 8px; border-radius:10px; font-size:0.71rem;
  background:rgba(255,255,255,0.06); border:1px solid rgba(255,255,255,0.1);
}}
.itag.ok {{ color:#69F0AE; border-color:rgba(105,240,174,0.35); background:rgba(105,240,174,0.08); }}
.itag.warn {{ color:#FFD740; border-color:rgba(255,215,64,0.35); background:rgba(255,215,64,0.08); }}
.itag.err {{ color:#FF5252; border-color:rgba(255,82,82,0.4); background:rgba(255,82,82,0.1); }}
.radar-wrap {{
  display:grid; grid-template-columns:1.25fr 1fr; gap:14px; align-items:center;
}}
@media (max-width:960px) {{ .radar-wrap {{ grid-template-columns:1fr; }} }}
.chart-box {{
  background:var(--bg-card); backdrop-filter:blur(12px);
  border:1px solid var(--border); border-radius:12px;
  padding:12px; margin-bottom:14px;
}}
.chart-box .chart-guide {{
  font-size:0.72rem; color:var(--text2); margin-top:6px;
  padding:7px 10px; background:rgba(100,255,218,0.06);
  border-radius:8px; border-left:3px solid var(--accent);
}}
.replay-container {{
  position:relative; border-radius:12px; overflow:hidden;
  background:var(--bg-card); border:1px solid var(--border);
}}
.replay-canvas-wrap {{
  position:relative; width:100%; padding-bottom:55%; background:#0d1117;
}}
.replay-canvas-wrap canvas {{
  position:absolute; top:0; left:0; width:100%; height:100%;
}}
.replay-toggle-corner {{
  position:absolute; top:10px; right:10px; z-index:5;
  display:flex; flex-direction:column; gap:5px; align-items:flex-end;
  max-width:60%;
}}
.replay-toggle-chip {{
  display:inline-flex; align-items:center; gap:5px;
  padding:3px 9px; border-radius:12px;
  background:rgba(13,17,23,0.75); border:1.5px solid var(--border);
  cursor:pointer; transition:all 0.2s; font-size:0.72rem;
  backdrop-filter:blur(6px); user-select:none; color:var(--text2);
  white-space:nowrap;
}}
.replay-toggle-chip:hover {{ background:rgba(13,17,23,0.92); }}
.replay-toggle-chip.active {{
  color:#e0e0e0; border-color:var(--chip-color,var(--accent));
  box-shadow:0 0 8px var(--chip-glow,rgba(100,255,218,0.25));
}}
.replay-toggle-chip .dot {{ width:8px; height:8px; border-radius:50%; flex:none; }}
.replay-toggle-chip.active .dot {{ box-shadow:0 0 4px currentColor; }}
.replay-controls {{
  display:flex; align-items:center; gap:8px; padding:10px 14px;
  background:rgba(0,0,0,0.4); border-top:1px solid var(--border); flex-wrap:wrap;
}}
.replay-controls button {{
  padding:5px 12px; border:1px solid var(--accent); border-radius:8px;
  background:transparent; color:var(--accent); cursor:pointer;
  font-size:0.82rem; transition:all 0.2s; min-width:34px;
}}
.replay-controls button:hover {{ background:rgba(100,255,218,0.15); }}
.replay-controls button.active {{ background:rgba(100,255,218,0.25); }}
.replay-controls .speed-display {{
  font-size:0.82rem; color:var(--accent); min-width:46px; text-align:center; font-weight:700;
}}
.replay-controls .time-display {{ font-size:0.78rem; color:var(--text2); }}
.replay-progress {{
  flex:1; min-width:100px; height:6px; border-radius:3px;
  background:rgba(255,255,255,0.1); cursor:pointer; position:relative;
}}
.replay-progress .fill {{
  height:100%; border-radius:3px; background:var(--accent); transition:width 0.1s;
}}
.replay-info {{
  font-size:0.7rem; color:var(--text2); padding:4px 14px 6px;
  display:flex; gap:14px; flex-wrap:wrap;
}}
.replay-info .ri-item {{ display:flex; align-items:center; gap:3px; }}
.replay-info .ri-dot {{ width:7px; height:7px; border-radius:50%; }}
.sound-toggle {{
  position:fixed; bottom:18px; right:18px; z-index:1000;
  width:44px; height:44px; border-radius:50%;
  background:var(--bg-card); border:2px solid var(--accent);
  color:var(--accent); font-size:1.2rem; cursor:pointer;
  display:flex; align-items:center; justify-content:center;
  backdrop-filter:blur(12px); transition:all 0.3s;
  box-shadow:0 4px 20px rgba(0,0,0,0.4);
}}
.sound-toggle:hover {{ transform:scale(1.1); box-shadow:0 0 20px var(--glow); }}
.sound-toggle.muted {{ border-color:#666; color:#666; }}
.reading-guide {{
  background:rgba(255,253,231,0.06); border:1px solid rgba(255,215,64,0.2);
  border-radius:10px; padding:12px 16px; margin:14px 0; font-size:0.75rem;
  color:var(--text2); line-height:1.8;
}}
.reading-guide strong {{ color:#FFD740; }}
.footer {{
  text-align:center; padding:20px; color:var(--text2); font-size:0.7rem;
  border-top:1px solid var(--border); margin-top:28px;
}}
@keyframes fadeInUp {{
  from {{ opacity:0; transform:translateY(20px); }}
  to {{ opacity:1; transform:translateY(0); }}
}}
.js-plotly-plot .plotly .modebar {{ top:4px !important; }}
@media (max-width:768px) {{
  .banner h1 {{ font-size:1.2rem; }}
  .replay-canvas-wrap {{ padding-bottom:75%; }}
}}
</style>
</head>
<body>

<div class="banner">
  <h1>🚢 {report_title}</h1>
  <div class="subtitle">{n_usv} 艘参演船只 · 0.60m × 0.30m 水密单体船 · 生成时间 {now}</div>
</div>

<nav class="top-nav">
  <a href="#s1">🎛️ 船只筛选</a>
  <a href="#s2">📊 核心看板 & 排行</a>
  <a href="#s3">🕸️ 八维雷达</a>
  <a href="#s4">🎬 动态路径回放</a>
  <a href="#s5">🗺️ 轨迹 & 原始航线</a>
  <a href="#s5b" id="nav-nb-link">🛡️ 邻船安全距</a>
  <a href="#s6">🚀 速度 & 控制</a>
  <a href="#s7">📐 航线偏差 & 航向</a>
  <a href="#s10">📋 分航点明细</a>
</nav>

<div class="container">

<div class="section" id="s1">
  <div class="section-title"><span class="num">1</span> 参演船只筛选与全局核心看板</div>
  <div style="text-align:center;margin-bottom:8px;">
    <button class="btn-toggle-all" onclick="toggleAllUSV()">全选 / 反选全部 USV</button>
  </div>
  <div class="usv-selector" id="usv-selector"></div>
  <div class="kpi-grid" id="kpi-strip"></div>
</div>

<div class="section" id="s2">
  <div class="section-title"><span class="num">2</span> 智能诊断摘要与多船量化对比表</div>
  <div id="ai-insights"></div>
  <div id="compare-area"></div>
  <div class="reading-guide">
    <strong>📊 量化对比表说明（点击任意表头可升/降序排列）：</strong><br>
    • <strong>综合评级</strong>：八维加权总分（航线贴合22% + 安全避让20% + 转向圆润16% + 航点达成16% + 绕行效率10% + 航向稳定6% + 巡航效率5% + 控制平顺5%；若发生船壳擦碰风险 &lt;0.35m 则总分封顶 68 分）。A+≥85, A≥75, B≥62, C≥45<br>
    • <strong>最小船距</strong>：GUIDED 航行期间与动船/静止船的最小质心距（0.60×0.30m 船模：≥0.75m 舒适安全，0.45–0.75m 紧凑贴身，&lt;0.35m 擦碰风险）<br>
    • <strong>航线 |CTE| (均/P90)</strong>：相对原始规划航线（W<sub>k-1</sub>→W<sub>k</sub>）无避让遮挡段的真实几何横向偏差；<strong>出弯外飘</strong>衡量航点大转弯后是否冲出原始航线外侧
  </div>
</div>

<div class="section" id="s3">
  <div class="section-title"><span class="num">3</span> “老司机”八维综合评估雷达图</div>
  <div class="chart-box">
    <div class="radar-wrap">
      <div id="radar-chart" style="width:100%;height:460px;"></div>
      <div id="radar-breakdown"></div>
    </div>
  </div>
  <div class="reading-guide">
    <strong>🕸️ 八维雷达评估口径（全部基于真实几何轨迹与动力学实测，0–100 分）：</strong><br>
    • <strong>航线贴合 (22%)</strong>：无近船干扰段相对原始航线真实 |CTE| 均值（≤0.06m 满分）与 P90（≤0.18m 满分），检验直道贴线与出弯归线精度<br>
    • <strong>安全避让 (20%)</strong>：全程最小邻船质心距 d<sub>min</sub>（≥0.80m 满分，0.60–0.80m 紧凑优良，≤0.30m 归零；单船无会遇记满分）<br>
    • <strong>转向圆润 (16%)</strong>：出弯零外飘（≤0.10m 满分）+ 转弯半径饱满度（P10 半径 ≥0.48m 满分，杜绝原地拧尖角）+ 角加速度平滑度 |dω/dt|<sub>P90</sub><br>
    • <strong>航点达成 (16%)</strong>：中途航点通过率×50% + 最终航点精准到达×50%<br>
    • <strong>绕行效率 (10%)</strong>：零绕圈死锁 + 实际航程/弦长比紧凑度（顺势择优左/右绕行，不走多余大 S 弯）<br>
    • <strong>航向稳定 (6%)</strong>：直道航向对准度 + 零蛇形过零摆动 &nbsp;|&nbsp; <strong>巡航效率 (5%)</strong>：实际均速与目标巡航速匹配度 &nbsp;|&nbsp; <strong>控制平顺 (5%)</strong>：无饱和正负反转跳变
  </div>
</div>

<div class="section" id="s4">
  <div class="section-title"><span class="num">4</span> 2D 动态路径回放（含原始航线基准 & 实时近距标尺）</div>
  <div class="replay-container">
    <div class="replay-canvas-wrap">
      <canvas id="replay-canvas"></canvas>
      <div class="replay-toggle-corner" id="replay-toggle-corner"></div>
    </div>
    <div class="replay-info" id="replay-info"></div>
    <div class="replay-controls">
      <button onclick="replayRestart()" title="重新开始">⏮</button>
      <button id="btn-play" onclick="replayToggle()" title="播放/暂停 (空格键)">▶</button>
      <button onclick="replaySlower()" title="减速">⏪</button>
      <span class="speed-display" id="speed-display">1.0x</span>
      <button onclick="replayFaster()" title="加速">⏩</button>
      <div class="replay-progress" id="replay-progress" onclick="replaySeek(event)">
        <div class="fill" id="replay-fill" style="width:0%"></div>
      </div>
      <span class="time-display" id="replay-time">0.0s / 0.0s</span>
      <button id="btn-plan-route" class="active" onclick="togglePlannedRoute()" title="显示/隐藏原始规划航线基准" style="margin-left:6px;font-size:12px;padding:3px 9px;">📐 原始航线</button>
      <button id="btn-goal-labels" onclick="toggleGoalLabels()" title="显示/隐藏目标点ID" style="font-size:12px;padding:3px 9px;">🏷️ 航点ID</button>
    </div>
  </div>
  <div class="reading-guide">
    <strong>🎬 动态回放增强功能：</strong>支持键盘 <strong>空格</strong> 播放/暂停、<strong>← / →</strong> 快退/快进 2 秒。<br>
    • <strong>📐 原始航线</strong>（默认开启）：以半透明长虚线绘制规划航点间的理想基准弦线，直观检验转弯弧度是否圆润、出弯是否偏离原始航线<br>
    • <strong>近距实时标尺</strong>：当任意两船间距进入 <strong>&lt;1.6m</strong> 时自动连线并实时显示精确米数（绿 ≥0.75m / 黄 0.45–0.75m / 红 &lt;0.45m），水面无独立日志的静止船亦自动显影
  </div>
</div>

<div class="section" id="s5">
  <div class="section-title"><span class="num">5</span> 航行轨迹与原始规划航线对比</div>
  <div class="chart-box">
    <div id="chart-traj" style="height:540px;"></div>
  </div>
  <div class="reading-guide">
    <strong>🗺️ 轨迹图说明：</strong>实线=GUIDED 实际航迹 | 同色细点划线=原始规划航线基准（点击图例可单独开关）| ★=目标航点 | 灰虚线=场内静止/未选中邻船。
  </div>
</div>

<div class="section" id="s5b">
  <div class="section-title"><span class="num">6</span> 邻船实时安全间距包络</div>
  <div class="chart-box">
    <div id="chart-nb-dist" style="height:340px;"></div>
    <div class="chart-guide">
      🛡️ 实时显示每艘 USV 与最近邻船（含动船与静止船）的质心距离，悬停可查看最近邻船 ID。绿虚线 0.80m=舒适安全线（净水 ≥0.50m），红虚线 0.35m=0.60×0.30m 船壳安全红线。
    </div>
  </div>
</div>

<div class="section" id="s6">
  <div class="section-title"><span class="num">7</span> 速度与转向控制通道分析</div>
  <div class="chart-box">
    <div id="chart-velocity" style="height:340px;"></div>
  </div>
  <div class="chart-box">
    <div id="chart-control" style="height:360px;"></div>
    <div class="chart-guide">
      ⚙️ 实线=最终角速度指令 cmd_omega，圆点线=实际角速度 ω_actual。弯道与避让段曲线呈平滑钟形无毛刺代表转向圆润丝滑。
    </div>
  </div>
</div>

<div class="section" id="s7">
  <div class="section-title"><span class="num">8</span> 真实航线横向偏差 (CTE)、航向误差与到点距离</div>
  <div class="chart-box">
    <div id="chart-cte" style="height:330px;"></div>
    <div class="chart-guide">
      📐 实线=相对原始规划航线 W<sub>k-1</sub>→W<sub>k</sub> 的真实几何横向偏差（正=偏右，负=偏左）；出弯后迅速归零说明无偏航外飘。
    </div>
  </div>
  <div class="chart-box">
    <div id="chart-he" style="height:310px;"></div>
  </div>
  <div class="chart-box">
    <div id="chart-distance" style="height:290px;"></div>
  </div>
</div>

<div class="section" id="s9" style="display:none;">
  <div class="section-title"><span class="num">9</span> 传统控制器 / 链路附加指标 (MPC · AMPC · WiFi)</div>
  <div class="chart-box" id="chart-mpc-box" style="display:none;">
    <div id="chart-mpc" style="height:260px;"></div>
  </div>
  <div class="chart-box" id="chart-tau-box" style="display:none;">
    <div id="chart-tau" style="height:280px;"></div>
  </div>
  <div class="chart-box" id="chart-wifi-box" style="display:none;">
    <div id="chart-wifi" style="height:260px;"></div>
  </div>
</div>

<div class="section" id="s10">
  <div class="section-title"><span class="num">9</span> 分航点导航明细统计</div>
  <div id="per-goal-area"></div>
</div>

</div>

<button class="sound-toggle" id="sound-toggle" onclick="toggleSound()" title="声效开关">🔊</button>

<div class="footer">
  USV Navigation Log Analysis Report (Multi-USV v2) | {now}
</div>

<script>
// ═══ 全局数据 ═══
const USV_LIST = {usv_list_json};
const ALL_TS   = {all_ts_json};
const ALL_TGTS = {all_targets_json};
const ALL_PG   = {all_per_goal_json};
const ALL_SUM  = {all_summaries_json};

let activeUSVs = new Set(USV_LIST.map(u=>u.idx));

const darkLayout = {{
  paper_bgcolor:'rgba(0,0,0,0)',
  plot_bgcolor:'rgba(17,24,39,0.5)',
  font:{{ color:'#e0e0e0',family:'Inter,system-ui,sans-serif' }},
  margin:{{ t:36,r:28,b:36,l:50 }},
  xaxis:{{ gridcolor:'rgba(255,255,255,0.06)',zerolinecolor:'rgba(255,255,255,0.1)' }},
  yaxis:{{ gridcolor:'rgba(255,255,255,0.06)',zerolinecolor:'rgba(255,255,255,0.1)' }},
  legend:{{ bgcolor:'rgba(0,0,0,0.3)',font:{{ size:10 }} }},
}};
const cfg = {{ responsive:true,displayModeBar:true,
  modeBarButtonsToRemove:['select2d','lasso2d','autoScale2d'],displaylogo:false }};

// ═══ 声效系统 ═══
let audioCtx=null, soundEnabled=true, lastSoundSection='';

function initAudio() {{
  if(!audioCtx) {{
    try {{ audioCtx=new(window.AudioContext||window.webkitAudioContext)(); }}
    catch(e) {{ soundEnabled=false; }}
  }}
}}
function playTone(freq,dur,type,vol) {{
  if(!soundEnabled||!audioCtx) return;
  try {{
    const o=audioCtx.createOscillator(), g=audioCtx.createGain();
    o.type=type||'sine'; o.frequency.setValueAtTime(freq,audioCtx.currentTime);
    g.gain.setValueAtTime(vol||0.08,audioCtx.currentTime);
    g.gain.exponentialRampToValueAtTime(0.001,audioCtx.currentTime+(dur||0.15));
    o.connect(g); g.connect(audioCtx.destination);
    o.start(); o.stop(audioCtx.currentTime+(dur||0.15));
  }} catch(e) {{}}
}}
function sfxClick()    {{ playTone(880,0.06,'sine',0.06); }}
function sfxHover()    {{ playTone(1200,0.03,'sine',0.03); }}
function sfxSection(n) {{
  if(lastSoundSection===n) return; lastSoundSection=n;
  const b=400+n*60;
  playTone(b,0.08,'sine',0.05);
  setTimeout(()=>playTone(b+200,0.08,'sine',0.04),60);
  setTimeout(()=>playTone(b+400,0.12,'triangle',0.03),120);
}}
function sfxReplayStart() {{
  playTone(523,0.08,'triangle',0.06);
  setTimeout(()=>playTone(659,0.08,'triangle',0.05),80);
  setTimeout(()=>playTone(784,0.15,'triangle',0.04),160);
}}
function sfxReplayPause() {{
  playTone(784,0.08,'triangle',0.06);
  setTimeout(()=>playTone(523,0.12,'triangle',0.05),80);
}}
function sfxGoalReached() {{
  playTone(784,0.1,'sine',0.07);
  setTimeout(()=>playTone(988,0.1,'sine',0.06),100);
  setTimeout(()=>playTone(1175,0.2,'triangle',0.05),200);
}}
function sfxGoalDiverged() {{
  // 偏离到达：低沉双音提示，与正常到达的高扬三音区分
  playTone(440,0.12,'triangle',0.07);
  setTimeout(()=>playTone(523,0.18,'triangle',0.06),120);
}}
function sfxSpeed() {{ playTone(660,0.05,'square',0.03); }}

function toggleSound() {{
  initAudio(); soundEnabled=!soundEnabled;
  const btn=document.getElementById('sound-toggle');
  btn.textContent=soundEnabled?'🔊':'🔇';
  btn.classList.toggle('muted',!soundEnabled);
  if(soundEnabled) sfxClick();
}}

// ═══ USV 选择器 ═══
function buildSelector() {{
  const el=document.getElementById('usv-selector');
  el.innerHTML='';
  USV_LIST.forEach(u=>{{
    const chip=document.createElement('div');
    chip.className='usv-chip'+(activeUSVs.has(u.idx)?' active':'');
    chip.style.setProperty('--chip-color',u.color);
    chip.style.setProperty('--chip-glow',u.color+'44');
    chip.innerHTML=`<span class="dot" style="background:${{u.color}}"></span>
      <span>${{u.id}}</span>
      <span class="grade" style="color:${{u.color}}">${{u.grade}}</span>
      <span style="font-size:0.68rem;color:var(--text2)">${{u.score}}</span>`;
    chip.onclick=()=>{{
      initAudio(); sfxClick();
      if(activeUSVs.has(u.idx)) activeUSVs.delete(u.idx);
      else activeUSVs.add(u.idx);
      chip.classList.toggle('active');
      buildReplayToggle();
      refreshAll();
    }};
    chip.onmouseenter=()=>sfxHover();
    el.appendChild(chip);
  }});
}}

function toggleAllUSV() {{
  initAudio(); sfxClick();
  if(activeUSVs.size===USV_LIST.length) activeUSVs.clear();
  else USV_LIST.forEach(u=>activeUSVs.add(u.idx));
  buildSelector(); buildReplayToggle(); refreshAll();
}}

// ═══ 重放面板右上角USV开关 (与上方选择器共享 activeUSVs 状态) ═══
function buildReplayToggle() {{
  const el=document.getElementById('replay-toggle-corner');
  if(!el) return;
  el.innerHTML='';
  USV_LIST.forEach(u=>{{
    const chip=document.createElement('div');
    chip.className='replay-toggle-chip'+(activeUSVs.has(u.idx)?' active':'');
    chip.style.setProperty('--chip-color',u.color);
    chip.style.setProperty('--chip-glow',u.color+'44');
    chip.innerHTML=`<span class="dot" style="background:${{u.color}}"></span><span>${{u.id}}</span>`;
    chip.title='显示/隐藏 '+u.id+' 的重放轨迹';
    chip.onclick=()=>{{
      initAudio(); sfxClick();
      if(activeUSVs.has(u.idx)) activeUSVs.delete(u.idx);
      else activeUSVs.add(u.idx);
      chip.classList.toggle('active');
      buildSelector();
      refreshAll();
    }};
    chip.onmouseenter=()=>sfxHover();
    el.appendChild(chip);
  }});
}}

let sortState = {{ key:'idx', asc:true }};

function refreshAll() {{
  buildKpiStrip(); buildSummaryCards(); buildCompareTable(); drawRadar();
  drawTrajectory(); drawNeighborDist(); drawVelocity();
  drawCTE(); drawHE(); drawDistance(); drawControl(); drawMPC(); drawTau(); drawWifi();
  updateLegacySectionVisibility();
  buildPerGoal(); updateReplayInfo(); renderReplayFrame();
}}

function buildKpiStrip() {{
  const el=document.getElementById('kpi-strip');
  if(!el) return;
  const active=getActive();
  if(!active.length) {{ el.innerHTML=''; return; }}
  const totGoals=active.reduce((s,u)=>s+(u.targets_n||0),0);
  const okGoals=active.reduce((s,u)=>s+(u.goals_ok||0),0);
  const goalPct=totGoals>0?(okGoals/totGoals*100):100;
  const avgScore=active.reduce((s,u)=>s+u.score,0)/active.length;
  const avgGrade=avgScore>=85?'A+':avgScore>=75?'A':avgScore>=62?'B':avgScore>=45?'C':'D';

  const nbShips=active.filter(u=>u.min_nb!==null&&u.min_nb!==undefined);
  let minNbHtml='单船无会遇', minNbSub='全程独立航行', minNbColor='#69F0AE';
  if(nbShips.length) {{
    const worst=nbShips.reduce((a,b)=>a.min_nb<=b.min_nb?a:b);
    minNbHtml=worst.min_nb.toFixed(2)+' m';
    minNbSub=worst.id+' ↔ '+(worst.min_nb_peer||'邻船')+(worst.min_nb_t!==null?' (t='+worst.min_nb_t+'s)':'');
    minNbColor=worst.min_nb>=0.75?'#69F0AE':worst.min_nb>=0.45?'#FFD740':'#FF5252';
  }}

  const ctes=active.filter(u=>u.cte_avg!==null).map(u=>u.cte_avg);
  const avgCte=ctes.length?(ctes.reduce((a,b)=>a+b,0)/ctes.length):0;
  const maxOv=Math.max(0,...active.map(u=>u.max_ov||0));
  const avgSpd=active.reduce((s,u)=>s+(u.speed_avg||0),0)/active.length;
  const rMeds=active.filter(u=>u.r_med!==null).map(u=>u.r_med);
  const avgR=rMeds.length?(rMeds.reduce((a,b)=>a+b,0)/rMeds.length):null;

  el.innerHTML=`
    <div class="kpi-card">
      <div class="kpi-label">团队综合评级</div>
      <div class="kpi-val">${{avgGrade}} <span style="font-size:1rem;font-weight:600;color:var(--text)">(${{avgScore.toFixed(1)}}分)</span></div>
      <div class="kpi-sub">${{active.length}} 艘活跃船只八维均值</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">航点达成率</div>
      <div class="kpi-val" style="color:${{goalPct>=99.9?'#69F0AE':'#FFD740'}}">${{okGoals}} / ${{totGoals}} <span style="font-size:0.95rem">(${{goalPct.toFixed(0)}}%)</span></div>
      <div class="kpi-sub">含中途切换与终点精准到达</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">全场最小邻船质心距</div>
      <div class="kpi-val" style="color:${{minNbColor}}">${{minNbHtml}}</div>
      <div class="kpi-sub">${{minNbSub}}</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">纯航线贴合 |CTE|</div>
      <div class="kpi-val" style="color:${{avgCte<=0.20?'#69F0AE':avgCte<=0.40?'#82B1FF':'#FFD740'}}">${{avgCte.toFixed(3)}} m</div>
      <div class="kpi-sub">排除近距避让段的原始航线偏差</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">最大出弯外飘量</div>
      <div class="kpi-val" style="color:${{maxOv<=0.20?'#69F0AE':maxOv<=0.45?'#FFD740':'#FF5252'}}">${{maxOv.toFixed(2)}} m</div>
      <div class="kpi-sub">衡量转弯后是否冲出原始航线</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">巡航均速 · 中位转弯半径</div>
      <div class="kpi-val" style="color:var(--accent2)">${{avgSpd.toFixed(2)}} <span style="font-size:0.9rem">m/s</span> · ${{avgR!==null?avgR.toFixed(2)+'m':'—'}}</div>
      <div class="kpi-sub">GUIDED 实测均速与弯道曲率半径</div>
    </div>`;
}}

function sortCompareBy(key) {{
  initAudio(); sfxClick();
  if(sortState.key===key) sortState.asc=!sortState.asc;
  else {{ sortState.key=key; sortState.asc=(key==='id'||key==='cte_avg'||key==='max_ov'||key==='he_avg'); }}
  buildCompareTable();
}}

// ═══ 对比表 (支持点击表头排序) ═══
function buildCompareTable() {{
  const el=document.getElementById('compare-area');
  const active=USV_LIST.filter(u=>activeUSVs.has(u.idx));
  if(!active.length) {{ el.innerHTML='<p style="color:var(--text2);text-align:center">请选择至少一个USV</p>'; return; }}
  const hasMpc=active.some(u=>u.mpc_avg!==null);
  const hasWifi=active.some(u=>u.wifi_avg!==null);
  const sorted=[...active].sort((a,b)=>{{
    const k=sortState.key;
    let va=a[k], vb=b[k];
    if(va===null||va===undefined) va=sortState.asc?1e9:-1e9;
    if(vb===null||vb===undefined) vb=sortState.asc?1e9:-1e9;
    if(typeof va==='string') return sortState.asc?va.localeCompare(vb):vb.localeCompare(va);
    return sortState.asc?(va-vb):(vb-va);
  }});
  const bestCte=Math.min(...active.map(u=>u.cte_avg!==null?u.cte_avg:999));
  const bestNb=Math.max(...active.map(u=>u.min_nb!==null?u.min_nb:-1));
  const bestScore=Math.max(...active.map(u=>u.score));
  const arrow=(k)=>sortState.key===k?(sortState.asc?' ▲':' ▼'):'';

  let rows=sorted.map(u=>{{
    const bc=u.cte_avg!==null&&u.cte_avg===bestCte&&active.length>1;
    const bs=u.score===bestScore&&active.length>1;
    const bnb=u.min_nb!==null&&u.min_nb===bestNb&&active.length>1;
    const nbCls=u.min_nb===null?'':(u.min_nb<0.35?'danger':u.min_nb<0.55?'warn':(bnb?'best':''));
    const nbTxt=u.min_nb!==null?`${{u.min_nb.toFixed(2)}}m <span style="font-size:0.7rem;color:var(--text2)">(${{u.min_nb_peer||'-'}})</span>`:'—';
    const ovCls=u.max_ov>0.45?'warn':(u.max_ov<=0.18?'best':'');
    return `<tr>
      <td><span style="color:${{u.color}};font-weight:700">●</span> <strong>${{u.id}}</strong></td>
      <td style="color:${{u.color}};font-weight:700">${{u.grade}} (${{u.score.toFixed(1)}}${{bs?' 🏆':''}})</td>
      <td>${{u.goals_ok}}/${{u.targets_n}}</td>
      <td class="${{nbCls}}">${{nbTxt}}</td>
      <td class="${{bc?'best':''}}">${{u.cte_avg!==null?u.cte_avg.toFixed(3)+'m':'N/A'}} <span style="font-size:0.7rem;color:var(--text2)">/ ${{u.cte_p90!==null&&u.cte_p90!==undefined?u.cte_p90.toFixed(2)+'m':'-'}}</span></td>
      <td class="${{ovCls}}">${{u.max_ov!==undefined?u.max_ov.toFixed(2)+'m':'—'}}</td>
      <td>${{u.r_med!==null?u.r_med.toFixed(2)+'m':'—'}} <span style="font-size:0.7rem;color:var(--text2)">(P10:${{u.r_p10!==null?u.r_p10.toFixed(2)+'m':'-'}})</span></td>
      <td>${{u.he_avg!==null?u.he_avg.toFixed(1)+'°':'N/A'}}</td>
      <td>${{u.speed_avg.toFixed(2)}} m/s</td>
      <td>${{u.duration}}s <span style="font-size:0.7rem;color:var(--text2)">(G:${{u.guided_pct}}%)</span></td>
      ${{hasMpc?`<td>${{u.mpc_avg!==null?u.mpc_avg+'ms':'—'}}</td>`:''}}
      ${{hasWifi?`<td>${{u.wifi_avg!==null?u.wifi_avg+'dBm':'—'}}</td>`:''}}</tr>`;
  }}).join('');
  el.innerHTML=`<div class="chart-box" style="overflow-x:auto">
    <table class="compare-table"><thead><tr>
      <th onclick="sortCompareBy('id')">USV${{arrow('id')}}</th>
      <th onclick="sortCompareBy('score')">综合评级${{arrow('score')}}</th>
      <th onclick="sortCompareBy('goals_ok')">航点完成${{arrow('goals_ok')}}</th>
      <th onclick="sortCompareBy('min_nb')">最小邻船距${{arrow('min_nb')}}</th>
      <th onclick="sortCompareBy('cte_avg')">航线 |CTE| (均/P90)${{arrow('cte_avg')}}</th>
      <th onclick="sortCompareBy('max_ov')">出弯外飘${{arrow('max_ov')}}</th>
      <th onclick="sortCompareBy('r_med')">转弯半径 R${{arrow('r_med')}}</th>
      <th onclick="sortCompareBy('he_avg')">直道航向差${{arrow('he_avg')}}</th>
      <th onclick="sortCompareBy('speed_avg')">GUIDED均速${{arrow('speed_avg')}}</th>
      <th onclick="sortCompareBy('duration')">时长${{arrow('duration')}}</th>
      ${{hasMpc?`<th onclick="sortCompareBy('mpc_avg')">MPC${{arrow('mpc_avg')}}</th>`:''}}
      ${{hasWifi?`<th onclick="sortCompareBy('wifi_avg')">WiFi${{arrow('wifi_avg')}}</th>`:''}}
    </tr></thead><tbody>${{rows}}</tbody></table></div>`;
}}

// ═══ 图表 ═══
function getActive() {{ return USV_LIST.filter(u=>activeUSVs.has(u.idx)); }}
function emptyTrace() {{ return [{{x:[],y:[]}}]; }}

function drawRadar() {{
  const el=document.getElementById('radar-chart');
  const bd=document.getElementById('radar-breakdown');
  if(!el) return;
  const active=getActive();
  if(!active.length) {{ Plotly.purge(el); if(bd) bd.innerHTML=''; return; }}
  const dims=[
    ['tracking','航线贴合 (22%)'],
    ['safety','安全避让 (20%)'],
    ['roundness','转向圆润 (16%)'],
    ['progress','航点达成 (16%)'],
    ['efficiency','绕行效率 (10%)'],
    ['heading','航向稳定 (6%)'],
    ['speed','巡航效率 (5%)'],
    ['smooth','控制平顺 (5%)'],
  ];
  const cats=dims.map(d=>d[1]);
  const traces=[];
  active.forEach(u=>{{
    const sc=ALL_SUM[u.idx];
    if(!sc||!sc.scoring||!sc.scoring.radar) return;
    const r=sc.scoring.radar;
    const vals=dims.map(d=>r[d[0]]!==undefined?r[d[0]]:0);
    traces.push({{
      type:'scatterpolar',
      r:[...vals, vals[0]],
      theta:[...cats, cats[0]],
      fill:'toself',
      fillcolor:(function(h){{const rv=parseInt(h.slice(1,3),16),gv=parseInt(h.slice(3,5),16),bv=parseInt(h.slice(5,7),16);return 'rgba('+rv+','+gv+','+bv+',0.14)';}})(u.color),
      line:{{color:u.color, width:2.5}},
      marker:{{size:6, color:u.color}},
      name:u.id+' ['+sc.scoring.grade+' '+sc.scoring.total.toFixed(1)+']',
      hovertemplate:'%{{theta}}: <b>%{{r:.1f}}</b> 分<extra>'+u.id+'</extra>',
    }});
  }});
  if(!traces.length) {{ Plotly.purge(el); return; }}
  const layout={{
    polar:{{
      radialaxis:{{
        visible:true, range:[0,100], tickvals:[20,40,60,80,100],
        tickfont:{{size:10,color:'#90A4AE'}},
        gridcolor:'rgba(255,255,255,0.09)',
      }},
      angularaxis:{{
        tickfont:{{size:12,color:'#ECEFF1'}},
        gridcolor:'rgba(255,255,255,0.1)',
      }},
      bgcolor:'transparent',
    }},
    showlegend:true,
    legend:{{font:{{size:11,color:'#CFD8DC'}}, orientation:'h', x:0.5, xanchor:'center', y:-0.08}},
    paper_bgcolor:'transparent', plot_bgcolor:'transparent',
    margin:{{l:65,r:65,t:30,b:55}},
    font:{{color:'#CFD8DC'}},
  }};
  Plotly.react(el,traces,layout,{{responsive:true,displayModeBar:false}});

  if(bd) {{
    let ths=active.map(u=>`<th style="color:${{u.color}}">${{u.id}}</th>`).join('');
    let trs=dims.map(d=>{{
      let tds=active.map(u=>{{
        const rv=(ALL_SUM[u.idx]&&ALL_SUM[u.idx].scoring&&ALL_SUM[u.idx].scoring.radar)?ALL_SUM[u.idx].scoring.radar[d[0]]:0;
        const c=rv>=85?'#69F0AE':rv>=68?'#E0E0E0':rv>=45?'#FFD740':'#FF5252';
        return `<td style="color:${{c}};font-weight:600">${{rv.toFixed(1)}}</td>`;
      }}).join('');
      return `<tr><td style="text-align:left;color:var(--text2)">${{d[1]}}</td>${{tds}}</tr>`;
    }}).join('');
    bd.innerHTML=`<div style="overflow-x:auto"><table class="compare-table" style="font-size:0.76rem">
      <thead><tr><th style="text-align:left">评估维度 (权重)</th>${{ths}}</tr></thead>
      <tbody>${{trs}}</tbody></table></div>`;
  }}
}}

function drawTrajectory() {{
  const traces=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts) return;
    // 绘制原始规划航线基准折线
    if(ts.planned_routes) {{
      Object.keys(ts.planned_routes).forEach((tk,ti)=>{{
        const pts=ts.planned_routes[tk];
        if(!pts||pts.length<2) return;
        traces.push({{
          x:pts.map(p=>p[0]), y:pts.map(p=>p[1]), mode:'lines',
          name:u.id+(Object.keys(ts.planned_routes).length>1?' 规划T'+(ti+1):' 规划航线'),
          line:{{color:u.color,width:1.3,dash:'dashdot'}}, opacity:0.42,
          hoverinfo:'skip'
        }});
      }});
    }}
    let gx=[],gy=[],hx=[],hy=[];
    for(let i=0;i<ts.pose_x.length;i++) {{
      if(ts.flight_mode[i]==='GUIDED') {{ gx.push(ts.pose_x[i]);gy.push(ts.pose_y[i]); }}
      else {{ hx.push(ts.pose_x[i]);hy.push(ts.pose_y[i]); }}
    }}
    traces.push({{x:gx,y:gy,mode:'lines',name:u.id+' 实际航迹',
      line:{{color:u.color,width:2.3}},hovertemplate:u.id+' X:%{{x:.2f}}m Y:%{{y:.2f}}m'}});
    if(hx.length) traces.push({{x:hx,y:hy,mode:'markers',name:u.id+' HOLD',
      marker:{{color:u.color,size:3,opacity:0.3}},showlegend:false}});
    traces.push({{x:[ts.pose_x[0]],y:[ts.pose_y[0]],mode:'markers',name:u.id+' Start',
      marker:{{color:'#4CAF50',size:11,symbol:'circle',line:{{width:2,color:u.color}}}},showlegend:false}});
    traces.push({{x:[ts.pose_x[ts.pose_x.length-1]],y:[ts.pose_y[ts.pose_y.length-1]],
      mode:'markers',name:u.id+' End',
      marker:{{color:'#F44336',size:11,symbol:'square',line:{{width:2,color:u.color}}}},showlegend:false}});
    const tg=ALL_TGTS[u.idx]||[];
    if(tg.length) traces.push({{x:tg.map(t=>t.x),y:tg.map(t=>t.y),
      mode:'markers+text',name:u.id+' Goals',
      marker:{{color:u.color,size:13,symbol:'star',line:{{width:1,color:'#fff'}}}},
      text:tg.map(t=>u.id+' G'+t.id),textposition:'top center',
      textfont:{{color:u.color,size:8}},showlegend:false}});
    // v16: 场内未勾选/静止邻船位置或轨迹
    if(ts.neighbors) {{
      Object.keys(ts.neighbors).forEach(nid=>{{
        const nb=ts.neighbors[nid];
        if(!nb||nb.x.length<3) return;
        const alreadyActive=getActive().some(a=>a.id===nid);
        if(alreadyActive) return;
        const dxSpan=Math.max(...nb.x)-Math.min(...nb.x), dySpan=Math.max(...nb.y)-Math.min(...nb.y);
        if(Math.hypot(dxSpan,dySpan)<0.45) {{
          traces.push({{x:[nb.x[0]],y:[nb.y[0]],mode:'markers+text',name:nid+' (静止船)',
            marker:{{color:'#90A4AE',size:11,symbol:'diamond',line:{{width:1.5,color:'#CFD8DC'}}}},
            text:[nid+'(静)'],textposition:'bottom center',textfont:{{color:'#B0BEC5',size:9}}}});
        }} else {{
          traces.push({{x:nb.x,y:nb.y,mode:'lines',name:nid+' (邻船)',
            line:{{color:'#90A4AE',width:1.2,dash:'dot'}},opacity:0.5,showlegend:true}});
        }}
      }});
    }}
  }});
  Plotly.react('chart-traj',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'XY 实际航迹 vs 原始规划航线基准',
    xaxis:{{...darkLayout.xaxis,title:'X (m)',scaleanchor:'y'}},
    yaxis:{{...darkLayout.yaxis,title:'Y (m)'}},
  }},cfg);
}}

function drawNeighborDist() {{
  const sec=document.getElementById('s5b');
  const navLink=document.getElementById('nav-nb-link');
  const active=getActive();
  const traces=[];
  active.forEach(u=>{{
    const ts=ALL_TS[u.idx];
    if(!ts||!ts.min_nb_dist||!ts.min_nb_dist.some(v=>v!==null)) return;
    traces.push({{
      x:ts.time, y:ts.min_nb_dist, customdata:ts.min_nb_id||[],
      name:u.id+' 最近邻船距', mode:'lines',
      line:{{color:u.color,width:1.7}},
      hovertemplate:u.id+' ↔ %{{customdata}}: <b>%{{y:.2f}}m</b> (t=%{{x:.1f}}s)<extra></extra>'
    }});
    const saf=ALL_SUM[u.idx]&&ALL_SUM[u.idx].safety;
    if(saf&&saf.min_dist!==null&&saf.time_s!==null) {{
      traces.push({{
        x:[saf.time_s], y:[saf.min_dist], mode:'markers+text',
        name:u.id+' CPA', showlegend:false,
        marker:{{color:saf.min_dist<0.45?'#FF5252':'#FFD740',size:9,symbol:'diamond',line:{{width:1,color:'#fff'}}}},
        text:[saf.min_dist.toFixed(2)+'m('+saf.peer_id+')'], textposition:'top center',
        textfont:{{size:9,color:u.color}}, hoverinfo:'skip'
      }});
    }}
  }});
  const show=traces.length>0;
  if(sec) sec.style.display=show?'block':'none';
  if(navLink) navLink.style.display=show?'inline-block':'none';
  if(!show) return;
  Plotly.react('chart-nb-dist',traces,{{
    ...darkLayout, title:'最近邻船质心距随时间变化 (CPA 监控)',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},
    yaxis:{{...darkLayout.yaxis,title:'Min Neighbor Distance (m)',rangemode:'tozero'}},
    shapes:[
      {{type:'line',x0:0,x1:1,xref:'paper',y0:0.80,y1:0.80,line:{{color:'#69F0AE',width:1.2,dash:'dash'}}}},
      {{type:'line',x0:0,x1:1,xref:'paper',y0:0.35,y1:0.35,line:{{color:'#FF5252',width:1.5,dash:'dot'}}}},
    ],
  }},cfg);
}}

function drawVelocity() {{
  const traces=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts) return;
    traces.push({{x:ts.time,y:ts.speed,name:u.id+' 实际',line:{{color:u.color,width:1.5}}}});
    if(ts.cmd_vx) traces.push({{x:ts.time,y:ts.cmd_vx,name:u.id+' 指令',
      line:{{color:u.color,width:1,dash:'dash'}},opacity:0.5,showlegend:false}});
  }});
  Plotly.react('chart-velocity',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'速度对比',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Speed (m/s)'}},
  }},cfg);
}}

function drawCTE() {{
  const traces=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts) return;
    const yCte=ts.true_cte||ts.cte;
    traces.push({{x:ts.time,y:yCte,name:u.id+' 规划航线CTE',line:{{color:u.color,width:1.4}},
      fill:'tozeroy',fillcolor:u.color+'15'}});
    if(ts.true_cte&&ts.cte) {{
      traces.push({{x:ts.time,y:ts.cte,name:u.id+' 局部跟踪器CTE',line:{{color:u.color,width:1,dash:'dot'}},
        opacity:0.45,visible:'legendonly'}});
    }}
  }});
  Plotly.react('chart-cte',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'真实规划航线横向偏差 CTE (m)',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'True Route CTE (m)'}},
  }},cfg);
}}

function drawHE() {{
  const traces=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts) return;
    traces.push({{x:ts.time,y:ts.heading_error,name:u.id,line:{{color:u.color,width:1.2}}}});
  }});
  Plotly.react('chart-he',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'航向误差对比',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Heading Error (°)'}},
  }},cfg);
}}

function drawControl() {{
  const traces=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts||!ts.cmd_omega) return;
    traces.push({{x:ts.time,y:ts.cmd_omega,name:u.id+' final ω',line:{{color:u.color,width:1.4}}}});
    if(ts.raw_cmd_omega) traces.push({{x:ts.time,y:ts.raw_cmd_omega,name:u.id+' raw ω',line:{{color:u.color,width:1,dash:'dash'}},opacity:0.65}});
    if(ts.rl_delta_omega) traces.push({{x:ts.time,y:ts.rl_delta_omega,name:u.id+' RL Δω',line:{{color:u.color,width:1,dash:'dot'}},opacity:0.85,showlegend:false}});
    if(ts.omega_actual) traces.push({{x:ts.time,y:ts.omega_actual,name:u.id+' ω_act',
      line:{{color:u.color,width:1.5,dash:'dot'}},showlegend:false}});
    if(ts.raw_cmd_vx) traces.push({{x:ts.time,y:ts.raw_cmd_vx,name:u.id+' raw vx',line:{{color:u.color,width:1,dash:'dash'}},opacity:0.35,visible:'legendonly'}});
    if(ts.cmd_vx) traces.push({{x:ts.time,y:ts.cmd_vx,name:u.id+' final vx',line:{{color:u.color,width:1}},opacity:0.35,visible:'legendonly'}});
    if(ts.rl_delta_vx) traces.push({{x:ts.time,y:ts.rl_delta_vx,name:u.id+' RL Δv',line:{{color:u.color,width:1,dash:'dot'}},opacity:0.35,visible:'legendonly'}});
  }});
  Plotly.react('chart-control',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'控制通道对比 (raw / RL / final)',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Command'}},
  }},cfg);
}}

function drawMPC() {{
  const active=getActive();
  const hasMpc=active.some(u=>ALL_TS[u.idx]&&ALL_TS[u.idx].mpc_time&&ALL_TS[u.idx].mpc_time.some(v=>v>0.05));
  const box=document.getElementById('chart-mpc-box');
  if(box) box.style.display=hasMpc?'block':'none';
  if(!hasMpc) return;
  const traces=[];
  active.forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts||!ts.mpc_time) return;
    traces.push({{x:ts.time,y:ts.mpc_time,name:u.id,line:{{color:u.color,width:1.2}}}});
  }});
  Plotly.react('chart-mpc',traces,{{
    ...darkLayout,title:'MPC 求解时间',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Time (ms)'}},
    shapes:[{{type:'line',x0:0,x1:1,xref:'paper',y0:50,y1:50,
      line:{{color:'#EF5350',width:1.5,dash:'dash'}}}}],
  }},cfg);
}}

function updateLegacySectionVisibility() {{
  const s9=document.getElementById('s9');
  if(!s9) return;
  const m=document.getElementById('chart-mpc-box');
  const t=document.getElementById('chart-tau-box');
  const w=document.getElementById('chart-wifi-box');
  const anyVisible=[m,t,w].some(b=>b&&b.style.display!=='none');
  s9.style.display=anyVisible?'block':'none';
}}

function drawTau() {{
  const active=getActive();
  const hasTau=active.some(u=>ALL_TS[u.idx]&&ALL_TS[u.idx].tau);
  document.getElementById('chart-tau-box').style.display=hasTau?'block':'none';
  if(!hasTau) return;
  const traces=[];
  active.forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts||!ts.tau) return;
    traces.push({{x:ts.time,y:ts.tau,name:u.id+' τ',line:{{color:u.color,width:2}}}});
    if(ts.tau_estimated) traces.push({{x:ts.time,y:ts.tau_estimated,name:u.id+' τ_est',
      line:{{color:u.color,width:1,dash:'dash'}},showlegend:false}});
  }});
  Plotly.react('chart-tau',traces,{{
    ...darkLayout,title:'Tau / AMPC',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'τ (s)'}},
  }},cfg);
}}

function drawWifi() {{
  const active=getActive();
  const hasW=active.some(u=>ALL_TS[u.idx]&&ALL_TS[u.idx].wifi_rssi);
  document.getElementById('chart-wifi-box').style.display=hasW?'block':'none';
  if(!hasW) return;
  const traces=[];
  active.forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts||!ts.wifi_rssi) return;
    traces.push({{x:ts.time,y:ts.wifi_rssi,name:u.id,line:{{color:u.color,width:1.2}}}});
  }});
  Plotly.react('chart-wifi',traces,{{
    ...darkLayout,title:'WiFi 信号对比',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'RSSI (dBm)'}},
    shapes:[
      {{type:'line',x0:0,x1:1,xref:'paper',y0:-50,y1:-50,line:{{color:'#4CAF50',width:0.8,dash:'dot'}}}},
      {{type:'line',x0:0,x1:1,xref:'paper',y0:-70,y1:-70,line:{{color:'#FF9800',width:0.8,dash:'dot'}}}},
    ],
  }},cfg);
}}

function buildPerGoal() {{
  const el=document.getElementById('per-goal-area');
  const active=getActive();
  if(!active.length) {{ el.innerHTML=''; return; }}
  let html='';
  active.forEach(u=>{{
    const pg=ALL_PG[u.idx]||[];
    if(pg.length<1) return;
    let rows=pg.map(g=>{{
      const arrSt=g.arrival||'unknown';
      const arr=arrSt==='arrived'||arrSt==='passed'?'✅':arrSt==='diverged'?'☑️':'❌';
      const detTxt=(g.detour&&g.detour>0)?g.detour.toFixed(2)+'×':'—';
      return `<tr><td>G${{g.id}}</td><td>${{g.duration.toFixed(1)}}s</td>
        <td>${{arr}} ${{g.min_dist!==null?g.min_dist.toFixed(2)+'m':'N/A'}}</td>
        <td>${{g.avg_cte!==null?g.avg_cte.toFixed(3)+'m':'N/A'}}</td>
        <td>${{detTxt}}</td>
        <td>${{g.avg_he!==null?g.avg_he.toFixed(1)+'°':'N/A'}}</td>
        <td>${{g.avg_speed!==null?g.avg_speed.toFixed(3)+' m/s':'N/A'}}</td>
        <td>${{g.guided_pct.toFixed(0)}}%</td></tr>`;
    }}).join('');
    html+=`<div class="chart-box">
      <h4 style="color:${{u.color}};margin-bottom:6px">● ${{u.id}}</h4>
      <table class="compare-table"><thead><tr>
        <th>航点</th><th>时长</th><th>最近距离</th><th>均CTE</th><th>绕行比</th><th>均航向差</th><th>均速</th><th>GUIDED%</th>
      </tr></thead><tbody>${{rows}}</tbody></table></div>`;
  }});
  el.innerHTML=html;
}}

// ═══ 智能诊断卡片 + 距离图表 ═══
function buildSummaryCards() {{
  const active=getActive();
  const el=document.getElementById('ai-insights');
  if(!el) return;
  if(!active.length) {{ el.innerHTML=''; return; }}
  let html='<div class="insight-grid">';
  active.forEach(u=>{{
    const sum=ALL_SUM[u.idx]; if(!sum) return;
    const sc=sum.scoring;
    let tags=[];
    tags.push(`<span class="itag ${{u.goals_ok===u.targets_n?'ok':'warn'}}">🎯 航点 ${{u.goals_ok}}/${{u.targets_n}}</span>`);
    if(sum.safety&&sum.safety.min_dist!==null&&sum.safety.min_dist!==undefined) {{
      const md=sum.safety.min_dist;
      const cls=md>=0.75?'ok':md>=0.42?'warn':'err';
      tags.push(`<span class="itag ${{cls}}">🛡️ 最小距 ${{md.toFixed(2)}}m (${{sum.safety.peer_id}})</span>`);
    }}
    if(u.cte_avg!==null) {{
      const cls=u.cte_avg<=0.18?'ok':u.cte_avg<=0.38?'':'warn';
      tags.push(`<span class="itag ${{cls}}">📐 贴线 |CTE| ${{u.cte_avg.toFixed(3)}}m</span>`);
    }}
    if(u.max_ov!==undefined) {{
      const cls=u.max_ov<=0.20?'ok':u.max_ov<=0.45?'':'warn';
      tags.push(`<span class="itag ${{cls}}">↪️ 出弯外飘 ${{u.max_ov.toFixed(2)}}m</span>`);
    }}
    if(u.r_med!==null) {{
      tags.push(`<span class="itag">⭕ 中位R ${{u.r_med.toFixed(2)}}m</span>`);
    }}
    if(sum.velocity) tags.push(`<span class="itag">🚀 均速 ${{sum.velocity.guided_avg.toFixed(2)}}m/s</span>`);
    if(sum.rl&&sum.rl.active_count>0) tags.push(`<span class="itag ok">🤖 纯RL ${{sum.rl.active_pct.toFixed(0)}}%</span>`);
    if(sum.osc_freq&&sum.osc_freq>0.3) tags.push(`<span class="itag warn">⚠️ ω振荡 ${{sum.osc_freq.toFixed(2)}}Hz</span>`);

    html+=`<div class="insight-card" style="border-left-color:${{u.color}}">
      <div style="display:flex;justify-content:space-between;align-items:center">
        <span style="color:${{u.color}};font-weight:700;font-size:0.88rem">● ${{u.id}}</span>
        <span style="font-weight:800;color:${{u.color}}">${{sc.grade}} · ${{sc.total.toFixed(1)}} 分</span>
      </div>
      <div class="insight-tags">${{tags.join('')}}</div>
    </div>`;
  }});
  html+='</div>';
  el.innerHTML=html;
}}

function drawDistance() {{
  const traces=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts||!ts.distance||!ts.distance.length) return;
    traces.push({{x:ts.time,y:ts.distance,name:u.id,line:{{color:u.color,width:1.2}}}});
  }});
  Plotly.react('chart-distance',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'到达距离',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Distance (m)'}},
    shapes:[{{type:'line',x0:0,x1:1,xref:'paper',y0:1.5,y1:1.5,
      line:{{color:'#4CAF50',width:1.5,dash:'dash'}}}}],
  }},cfg);
}}

// ═══ 路径重放引擎 ═══
let replayState = {{
  playing:false, speed:1.0, currentTime:0, maxTime:0,
  lastFrame:0, animId:null, goalReachedFlags:{{}}, goalReachedTime:{{}},
  _prevGoals:{{}}, showGoalLabels:false, showPlannedRoute:true
}};

// Catmull-Rom 样条插值: 四点 p0,p1,p2,p3, 参数 t∈[0,1]
function catmull(p0,p1,p2,p3,t) {{
  const t2=t*t, t3=t2*t;
  return 0.5*(2*p1+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t2+(-p0+3*p1-3*p2+p3)*t3);
}}

// GS导航预览箭头: 0.60m×0.30m, 归一化后 width/length=0.50
const BOAT_VERTS = [[0.5,0],[-0.5,0.25],[-0.167,0],[-0.5,-0.25]];
const BOAT_SCALE = 22;

// 避让类型颜色映射: -1=unset, 0=head_on, 1=crossing, 2=overtaking
const ENC_COLORS = {{'-1':'#BDBDBD','0':'#F44336','1':'#FF9800','2':'#2196F3'}};
const ARC_RADIUS = BOAT_SCALE * 1.6;   // 圆弧半径
const ARC_HALF_DEG = 18 * Math.PI/180; // 单侧半角 18°

function drawAvoidanceArcs(ctx, canX, canY, ts, idx, t, d) {{
  const nbs = ts.neighbors;
  if(!nbs) return;
  const cx = ts.pose_x[idx], cy = ts.pose_y[idx];
  const encType = (ts.rl_encounter_type && ts.rl_encounter_type[idx] !== undefined)
                  ? ts.rl_encounter_type[idx] : -1;
  const encColor = ENC_COLORS[String(encType)] || ENC_COLORS['-1'];
  const rlOmega = (ts.rl_cmd_omega && ts.rl_cmd_omega[idx] !== undefined)
                  ? Math.abs(ts.rl_cmd_omega[idx]) : 0;
  for(const nid in nbs) {{
    const pos = sampleNeighborPos(nbs[nid], t);
    if(!pos) continue;
    const dx = pos.x - cx, dy = pos.y - cy;
    const dist = Math.sqrt(dx*dx + dy*dy);
    if(dist < 0.1 || dist > 50) continue;
    // 实际方位角 (Canvas Y轴翻转: 世界y正→canvas y负)
    const bearing = Math.atan2(-dy, dx);
    // 紧迫度: 距离越近 & RL omega 越大 → 越紧迫
    const urgency = Math.min(1.0, Math.max(0, (1 - dist/30) * 0.7 + Math.min(rlOmega/0.6, 1)*0.3));
    if(urgency < 0.05) continue;
    // 闪烁: 紧迫度高时闪烁快
    const blinkPeriod = urgency > 0.7 ? 8 : urgency > 0.4 ? 16 : 32;
    const frameCount = replayState._arcFrame || 0;
    const blinkOn = (Math.floor(frameCount / blinkPeriod) % 2) === 0;
    if(!blinkOn && urgency < 0.85) continue;
    const alpha = 0.3 + urgency * 0.6;
    ctx.save();
    ctx.translate(canX, canY);
    ctx.beginPath();
    ctx.arc(0, 0, ARC_RADIUS, bearing - ARC_HALF_DEG, bearing + ARC_HALF_DEG);
    ctx.strokeStyle = encColor;
    ctx.lineWidth = 3 + urgency * 3;
    ctx.globalAlpha = alpha;
    ctx.shadowColor = encColor;
    ctx.shadowBlur = 6 + urgency * 8;
    ctx.stroke();
    // 高紧迫度双层弧
    if(urgency > 0.7) {{
      ctx.beginPath();
      ctx.arc(0, 0, ARC_RADIUS + 5, bearing - ARC_HALF_DEG*0.7, bearing + ARC_HALF_DEG*0.7);
      ctx.lineWidth = 2;
      ctx.globalAlpha = alpha * 0.5;
      ctx.stroke();
    }}
    ctx.restore();
  }}
}}

function drawBoat(ctx,x,y,yaw,color,s) {{
  ctx.save(); ctx.translate(x,y); ctx.rotate(-yaw);
  ctx.beginPath(); s=s||BOAT_SCALE;
  BOAT_VERTS.forEach((v,i)=>{{
    const px=v[0]*s, py=v[1]*s;
    if(i===0) ctx.moveTo(px,py); else ctx.lineTo(px,py);
  }});
  ctx.closePath(); ctx.fillStyle=color; ctx.globalAlpha=0.9; ctx.fill();
  ctx.strokeStyle='#fff'; ctx.lineWidth=1.5; ctx.stroke();
  ctx.globalAlpha=1.0; ctx.restore();
}}

function drawStar(ctx,cx,cy,r,color) {{
  ctx.save(); ctx.beginPath();
  for(let i=0;i<5;i++) {{
    const a=Math.PI/2+i*2*Math.PI/5;
    const x=cx+Math.cos(a)*r, y=cy-Math.sin(a)*r;
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    const a2=a+Math.PI/5;
    ctx.lineTo(cx+Math.cos(a2)*r*0.4,cy-Math.sin(a2)*r*0.4);
  }}
  ctx.closePath(); ctx.fillStyle=color; ctx.fill();
  ctx.strokeStyle='#fff'; ctx.lineWidth=0.8; ctx.stroke(); ctx.restore();
}}

function getReplayData() {{
  const data=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx];
    if(!ts||!ts.time||ts.time.length<2) return;
    if(!ts._smoothPath) ts._smoothPath=buildSmoothPath(ts);
    const pg=ALL_PG[u.idx]||[];
    // 构建每个goal的到达状态映射 (按 task_idx + goal_id 区分多任务)
    const arrivalMap={{}};
    pg.forEach(g=>{{ arrivalMap[(g.task_idx||0)+'_'+g.id]=g.arrival||'unknown'; }});
    data.push({{usv:u,ts:ts,targets:ALL_TGTS[u.idx]||[],sp:ts._smoothPath,arrivalMap:arrivalMap}});
  }});
  return data;
}}

// 去除GPS量化重复点，只保留位置有变化的关键帧
// 返回 {{t:[],x:[],y:[],yaw:[]}} — 用于重放插值
function buildSmoothPath(ts) {{
  const EPS=0.0005; // 位移阈值(m)
  const pt=[{{t:ts.time[0],x:ts.pose_x[0],y:ts.pose_y[0],yaw:ts.yaw_deg[0]||0}}];
  for(let i=1;i<ts.time.length;i++) {{
    const dx=ts.pose_x[i]-pt[pt.length-1].x;
    const dy=ts.pose_y[i]-pt[pt.length-1].y;
    if(dx*dx+dy*dy>EPS*EPS) {{
      pt.push({{t:ts.time[i],x:ts.pose_x[i],y:ts.pose_y[i],yaw:ts.yaw_deg[i]||0}});
    }}
  }}
  // 确保末尾点
  const last=ts.time.length-1;
  if(pt[pt.length-1].t<ts.time[last]) {{
    pt.push({{t:ts.time[last],x:ts.pose_x[last],y:ts.pose_y[last],yaw:ts.yaw_deg[last]||0}});
  }}
  return {{t:pt.map(p=>p.t),x:pt.map(p=>p.x),y:pt.map(p=>p.y),yaw:pt.map(p=>p.yaw)}};
}}

// 在平滑路径上查找时间t对应的位置 (Catmull-Rom 插值)
function sampleSmoothPos(sp,t) {{
  if(!sp||sp.t.length<2) return null;
  // 二分查找
  let lo=0,hi=sp.t.length-1;
  while(lo<hi-1) {{ const m=(lo+hi)>>1; if(sp.t[m]<=t) lo=m; else hi=m; }}
  const idx=lo;
  if(idx>=sp.t.length-1) return {{x:sp.x[idx],y:sp.y[idx],yaw:sp.yaw[idx]*Math.PI/180}};
  const dt=sp.t[idx+1]-sp.t[idx];
  if(dt<1e-6) return {{x:sp.x[idx],y:sp.y[idx],yaw:sp.yaw[idx]*Math.PI/180}};
  const f=Math.max(0,Math.min(1,(t-sp.t[idx])/dt));
  // Catmull-Rom 4点
  const i0=Math.max(0,idx-1), i2=idx+1, i3=Math.min(sp.t.length-1,idx+2);
  const cx=catmull(sp.x[i0],sp.x[idx],sp.x[i2],sp.x[i3],f);
  const cy=catmull(sp.y[i0],sp.y[idx],sp.y[i2],sp.y[i3],f);
  // yaw 环绕插值
  let y0=sp.yaw[idx], y1=sp.yaw[i2], dy=y1-y0;
  if(dy>180) dy-=360; if(dy<-180) dy+=360;
  const yaw=(y0+dy*f)*Math.PI/180;
  return {{x:cx,y:cy,yaw:yaw}};
}}

// 从邻居观测数据中采样给定时间t的位置 (线性插值)
function sampleNeighborPos(nb,t) {{
  if(!nb||!nb.time||nb.time.length<1) return null;
  if(t<=nb.time[0]) return {{x:nb.x[0],y:nb.y[0],yaw:nb.yaw[0]*Math.PI/180}};
  if(t>=nb.time[nb.time.length-1]) {{
    const li=nb.time.length-1;
    return {{x:nb.x[li],y:nb.y[li],yaw:nb.yaw[li]*Math.PI/180}};
  }}
  let lo=0,hi=nb.time.length-1;
  while(lo<hi-1){{ const m=(lo+hi)>>1; if(nb.time[m]<=t)lo=m; else hi=m; }}
  const dt=nb.time[lo+1]-nb.time[lo];
  if(dt<1e-6) return {{x:nb.x[lo],y:nb.y[lo],yaw:nb.yaw[lo]*Math.PI/180}};
  const f=(t-nb.time[lo])/dt;
  const x=nb.x[lo]+(nb.x[lo+1]-nb.x[lo])*f;
  const y=nb.y[lo]+(nb.y[lo+1]-nb.y[lo])*f;
  let y0=nb.yaw[lo],y1=nb.yaw[lo+1],dy=y1-y0;
  if(dy>180)dy-=360; if(dy<-180)dy+=360;
  const yaw=(y0+dy*f)*Math.PI/180;
  return {{x:x,y:y,yaw:yaw}};
}}

// 在其他USV的邻船观测中查找某USV在时刻t的位置, 优先使用观测时间覆盖t的观察者
function findPeerPos(data,selfD,t) {{
  const selfId=selfD.usv.id;
  let best=null, fallback=null;
  data.forEach(o=>{{
    if(o.usv.idx===selfD.usv.idx) return;
    const nb=o.ts.neighbors&&o.ts.neighbors[selfId];
    if(!nb||!nb.time||!nb.time.length) return;
    const pos=sampleNeighborPos(nb,t);
    if(!pos) return;
    fallback=pos;
    if(t>=nb.time[0]-2&&t<=nb.time[nb.time.length-1]+2) best=pos;
  }});
  return best||fallback;
}}

function updateReplayInfo() {{
  const el=document.getElementById('replay-info');
  el.innerHTML=getActive().map(u=>{{
    const sum=ALL_SUM[u.idx]||{{}};
    const rl=sum.rl&&sum.rl.active_count>0?` | RL ${{sum.rl.active_pct.toFixed(1)}}%`:'';
    return `<span class="ri-item"><span class="ri-dot" style="background:${{u.color}}"></span>${{u.id}}${{rl}}</span>`;
  }}).join('')||'<span style="color:var(--text2)">请选择USV</span>';
  const data=getReplayData();
  replayState.maxTime=Math.max(1,...data.map(d=>d.ts.time[d.ts.time.length-1]));
  updateTimeDisplay();
}}

function updateTimeDisplay() {{
  document.getElementById('replay-time').textContent=
    replayState.currentTime.toFixed(1)+'s / '+replayState.maxTime.toFixed(1)+'s';
  const pct=replayState.maxTime>0?(replayState.currentTime/replayState.maxTime*100):0;
  document.getElementById('replay-fill').style.width=Math.min(100,pct)+'%';
}}

function replayToggle() {{
  initAudio();
  replayState.playing=!replayState.playing;
  document.getElementById('btn-play').textContent=replayState.playing?'⏸':'▶';
  document.getElementById('btn-play').classList.toggle('active',replayState.playing);
  if(replayState.playing) {{
    sfxReplayStart(); replayState.lastFrame=performance.now();
    replayState.goalReachedFlags={{}};
    replayState.animId=requestAnimationFrame(replayLoop);
  }} else {{
    sfxReplayPause();
    if(replayState.animId) cancelAnimationFrame(replayState.animId);
  }}
}}

function replayRestart() {{ initAudio(); sfxClick(); replayState.currentTime=0; replayState.goalReachedFlags={{}}; replayState.goalReachedTime={{}}; replayState._prevGoals={{}}; replayState._taskFade={{}}; replayState._globalFadeTime=null; replayState._vp=null; renderReplayFrame(); updateTimeDisplay(); }}
function replayFaster() {{ initAudio(); sfxSpeed(); replayState.speed=Math.min(16,replayState.speed*2); document.getElementById('speed-display').textContent=replayState.speed+'x'; }}
function replaySlower() {{ initAudio(); sfxSpeed(); replayState.speed=Math.max(0.25,replayState.speed/2); document.getElementById('speed-display').textContent=replayState.speed+'x'; }}
function togglePlannedRoute() {{
  replayState.showPlannedRoute=!replayState.showPlannedRoute;
  const btn=document.getElementById('btn-plan-route');
  if(btn) btn.classList.toggle('active',replayState.showPlannedRoute);
  renderReplayFrame();
}}
function toggleGoalLabels() {{ replayState.showGoalLabels=!replayState.showGoalLabels; const btn=document.getElementById('btn-goal-labels'); if(btn) btn.classList.toggle('active',replayState.showGoalLabels); renderReplayFrame(); }}
function replaySeek(evt) {{
  initAudio(); sfxClick();
  const bar=document.getElementById('replay-progress');
  const rect=bar.getBoundingClientRect();
  replayState.currentTime=Math.max(0,Math.min(1,(evt.clientX-rect.left)/rect.width))*replayState.maxTime;
  replayState.goalReachedFlags={{}}; replayState.goalReachedTime={{}}; replayState._prevGoals={{}}; replayState._vp=null;
  // seek后重建任务切换状态
  replayState._taskFade={{}}; replayState._globalFadeTime=null;
  const seekT=replayState.currentTime;
  getReplayData().forEach(d=>{{
    const ts=d.ts;
    if(!ts.task_idx) return;
    // 找到当前时间对应的任务
    let sIdx=0;
    for(let i=0;i<ts.time.length;i++) {{ if(ts.time[i]<=seekT) sIdx=i; else break; }}
    const curTask=ts.task_idx[sIdx];
    // 检查是否存在更早的任务 (即已经发生过切换)
    const firstTask=ts.task_idx[0];
    if(curTask!==firstTask) {{
      // 找到切换时刻
      let switchTime=seekT;
      for(let i=1;i<ts.time.length;i++) {{ if(ts.task_idx[i]!==firstTask) {{ switchTime=ts.time[i]; break; }} }}
      const tfKey='u'+d.usv.idx;
      replayState._taskFade[tfKey]={{task:curTask,prevTask:firstTask,fadeStart:switchTime}};
      if(replayState._globalFadeTime===null) replayState._globalFadeTime=switchTime;
      else replayState._globalFadeTime=Math.min(replayState._globalFadeTime,switchTime);
    }}
  }});
  renderReplayFrame(); updateTimeDisplay();
}}

function replayLoop(now) {{
  if(!replayState.playing) return;
  const dt=(now-replayState.lastFrame)/1000*replayState.speed;
  replayState.lastFrame=now;
  replayState.currentTime+=dt;
  if(replayState.currentTime>=replayState.maxTime) {{
    replayState.currentTime=replayState.maxTime;
    replayState.playing=false;
    document.getElementById('btn-play').textContent='▶';
    document.getElementById('btn-play').classList.remove('active');
    sfxGoalReached();
    renderReplayFrame(); updateTimeDisplay(); return;
  }}
  renderReplayFrame();
  // 节流 DOM 更新：仅每 ~100ms 更新一次时间文本
  if(!replayState._lastDom||now-replayState._lastDom>100) {{
    updateTimeDisplay(); replayState._lastDom=now;
  }}
  replayState.animId=requestAnimationFrame(replayLoop);
}}

function renderReplayFrame() {{
  const canvas=document.getElementById('replay-canvas');
  const wrap=canvas.parentElement;
  const dpr=window.devicePixelRatio||1;
  const w=wrap.clientWidth, h=wrap.clientHeight;
  const nw=w*dpr, nh=h*dpr;
  // 仅在尺寸变化时重设 canvas（避免每帧清空闪烁）
  if(canvas.width!==nw||canvas.height!==nh) {{ canvas.width=nw; canvas.height=nh; }}
  const ctx=canvas.getContext('2d');
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.fillStyle='#0d1117'; ctx.fillRect(0,0,w,h);

  const data=getReplayData();
  if(!data.length) {{
    ctx.fillStyle='#9e9e9e'; ctx.font='14px Inter,system-ui,sans-serif';
    ctx.textAlign='center'; ctx.fillText('请选择USV加入重放',w/2,h/2); return;
  }}

  const t=replayState.currentTime;

  // 初始化任务切换渐隐追踪
  if(!replayState._taskFade) replayState._taskFade={{}};

  // 第一遍: 更新所有USV的任务状态,检测切换
  data.forEach(d=>{{
    const ts=d.ts;
    let dIdx=0;
    for(let i=0;i<ts.time.length;i++) {{ if(ts.time[i]<=t) dIdx=i; else break; }}
    const dTaskIdx=ts.task_idx?ts.task_idx[dIdx]:0;
    const tfKey='u'+d.usv.idx;
    if(!(tfKey in replayState._taskFade)) replayState._taskFade[tfKey]={{task:dTaskIdx}};
    const tf=replayState._taskFade[tfKey];
    if(tf.task!==dTaskIdx) {{
      tf.prevTask=tf.task; tf.fadeStart=t; tf.task=dTaskIdx;
      // 记录全局切换时间 (持久化,不会被清除)
      if(replayState._globalFadeTime===null||replayState._globalFadeTime===undefined) replayState._globalFadeTime=t;
    }}
  }});
  const globalFadeTime=replayState._globalFadeTime;
  const fadeDur=3.0;

  // 第二遍: 计算视口范围
  let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
  data.forEach(d=>{{
    const ts=d.ts;
    let dIdx=0;
    for(let i=0;i<ts.time.length;i++) {{ if(ts.time[i]<=t) dIdx=i; else break; }}
    const dTaskIdx=ts.task_idx?ts.task_idx[dIdx]:0;
    const tfKey='u'+d.usv.idx;
    const tf=replayState._taskFade[tfKey];
    const hasOwnFade=(tf.prevTask!==undefined);
    const ownFading=(hasOwnFade&&tf.fadeStart!==undefined&&t-tf.fadeStart<fadeDur);
    // 单任务USV: 仅当自身数据已结束(非仍在并发进行中)且全局渐隐已完成时才不纳入视口
    const selfEnded=t>ts.time[ts.time.length-1];
    if(!hasOwnFade&&selfEnded&&globalFadeTime!==null&&globalFadeTime!==undefined&&t-globalFadeTime>=fadeDur) return;
    // 只将当前任务(及渐隐中的旧任务)的轨迹点纳入视口范围
    for(let i=0;i<ts.pose_x.length;i++) {{
      const ti=ts.task_idx?ts.task_idx[i]:0;
      if(ti===dTaskIdx||(ownFading&&ti===tf.prevTask)) {{
        const vx=ts.pose_x[i],vy=ts.pose_y[i];
        if(vx<minX)minX=vx; if(vx>maxX)maxX=vx;
        if(vy<minY)minY=vy; if(vy>maxY)maxY=vy;
      }}
    }}
    // 只将当前任务(及渐隐中的旧任务)的目标点纳入视口范围
    d.targets.forEach(tg=>{{
      const ti=tg.task_idx||0;
      if(ti===dTaskIdx||(ownFading&&ti===tf.prevTask)) {{
        if(tg.x<minX)minX=tg.x; if(tg.x>maxX)maxX=tg.x;
        if(tg.y<minY)minY=tg.y; if(tg.y>maxY)maxY=tg.y;
      }}
    }});
  }});
  // v16: 将邻居观测位置纳入视口 (自身日志尚未开始/已结束的USV，以及无独立日志的静止邻船)
  const dataIds=new Set(data.map(d=>d.usv.id));
  const extPeerPos={{}};
  data.forEach(o=>{{
    const nbs=o.ts.neighbors; if(!nbs) return;
    for(const nid in nbs) {{
      if(dataIds.has(nid)) continue;
      const nb=nbs[nid];
      if(!nb||!nb.time||!nb.time.length) continue;
      const pos=sampleNeighborPos(nb,t);
      if(pos&&( !(nid in extPeerPos) || (t>=nb.time[0]-2&&t<=nb.time[nb.time.length-1]+2) )) extPeerPos[nid]=pos;
    }}
  }});
  data.forEach(dSelf=>{{
    const sT=dSelf.ts.time;
    if(t>=sT[0]-0.5&&t<=sT[sT.length-1]) return;
    const pos=findPeerPos(data,dSelf,t);
    if(pos) {{
      if(pos.x<minX)minX=pos.x; if(pos.x>maxX)maxX=pos.x;
      if(pos.y<minY)minY=pos.y; if(pos.y>maxY)maxY=pos.y;
    }}
  }});
  for(const nid in extPeerPos) {{
    const pos=extPeerPos[nid];
    if(pos.x<minX)minX=pos.x; if(pos.x>maxX)maxX=pos.x;
    if(pos.y<minY)minY=pos.y; if(pos.y>maxY)maxY=pos.y;
  }}

  let rx=(maxX-minX)||1, ry=(maxY-minY)||1;
  const pad=0.15;
  minX-=rx*pad; maxX+=rx*pad; minY-=ry*pad; maxY+=ry*pad;
  // 视口平滑插值 — 避免任务切换时画面跳变
  if(replayState._vp) {{
    const k=0.08;
    minX=replayState._vp.minX+(minX-replayState._vp.minX)*k;
    maxX=replayState._vp.maxX+(maxX-replayState._vp.maxX)*k;
    minY=replayState._vp.minY+(minY-replayState._vp.minY)*k;
    maxY=replayState._vp.maxY+(maxY-replayState._vp.maxY)*k;
  }}
  replayState._vp={{minX,maxX,minY,maxY}};
  rx=(maxX-minX)||1; ry=(maxY-minY)||1;
  const scX=(w-40)/rx, scY=(h-40)/ry;
  const sc=Math.min(scX,scY);
  const offX=(w-rx*sc)/2, offY=(h-ry*sc)/2;
  function toC(px,py) {{ return [offX+(px-minX)*sc, h-offY-(py-minY)*sc]; }}

  // 网格
  ctx.strokeStyle='rgba(255,255,255,0.05)'; ctx.lineWidth=0.5;
  const gs=Math.pow(10,Math.floor(Math.log10(rx)))/2||1;
  ctx.font='10px Inter,system-ui,sans-serif';
  // X轴网格 + 底部标注
  ctx.textAlign='center';
  for(let gx=Math.ceil(minX/gs)*gs;gx<=maxX;gx+=gs) {{
    const[cx]=toC(gx,0); ctx.beginPath();ctx.moveTo(cx,0);ctx.lineTo(cx,h);ctx.stroke();
    const isZero=Math.abs(gx)<1e-6;
    ctx.fillStyle=isZero?'rgba(100,255,218,0.85)':'rgba(255,255,255,0.35)';
    ctx.font=isZero?'bold 11px Inter,system-ui,sans-serif':'10px Inter,system-ui,sans-serif';
    ctx.fillText(gx.toFixed(1),cx,h-4);
  }}
  // Y轴网格 + 左侧标注
  ctx.textAlign='left';
  for(let gy=Math.ceil(minY/gs)*gs;gy<=maxY;gy+=gs) {{
    const[,cy]=toC(0,gy); ctx.beginPath();ctx.moveTo(0,cy);ctx.lineTo(w,cy);ctx.stroke();
    const isZero=Math.abs(gy)<1e-6;
    ctx.fillStyle=isZero?'rgba(100,255,218,0.85)':'rgba(255,255,255,0.35)';
    ctx.font=isZero?'bold 11px Inter,system-ui,sans-serif':'10px Inter,system-ui,sans-serif';
    ctx.fillText(gy.toFixed(1),4,cy-2);
  }}
  // X/Y 轴标识
  ctx.fillStyle='rgba(100,255,218,0.5)'; ctx.font='bold 11px Inter,system-ui,sans-serif';
  ctx.textAlign='center'; ctx.fillText('X (m)',w/2,h-14);
  ctx.save(); ctx.translate(14,h/2); ctx.rotate(-Math.PI/2);
  ctx.fillText('Y (m)',0,0); ctx.restore();

  data.forEach(d=>{{
    const ts=d.ts, color=d.usv.color, uK=d.usv.idx, sp=d.sp;
    // 自身日志尚未开始或已结束超过渐隐期: 若邻船观测里有它的位置, 交给下方幽灵船绘制, 避免残留上一轮旧轨迹与双影船
    if(t<ts.time[0]-0.5&&findPeerPos(data,d,t)) return;
    if(t>ts.time[ts.time.length-1]+fadeDur&&findPeerPos(data,d,t)) return;
    let idx=0;
    for(let i=0;i<ts.time.length;i++) {{ if(ts.time[i]<=t) idx=i; else break; }}

    // 用平滑路径计算当前位置 (连贯无跳变)
    const pos=sampleSmoothPos(sp,t)||{{x:ts.pose_x[idx],y:ts.pose_y[idx],yaw:(ts.yaw_deg[idx]||0)*Math.PI/180}};
    const cx=pos.x, cy=pos.y, yaw=pos.yaw;

    // 当前任务索引 (多任务合并时区分不同任务的目标和轨迹)
    const curTaskIdx=ts.task_idx?ts.task_idx[idx]:0;
    const curGoalId=ts.goal_id?ts.goal_id[idx]:0;

    // 计算渐隐/淡入系数
    const tfKey='u'+uK;
    const tf=replayState._taskFade[tfKey];
    const hasOwnTransition=(tf&&tf.prevTask!==undefined);
    // 只有当该USV自身数据已经结束(不再是并发进行中的伙伴)时,才因为"其他USV切换任务"而渐隐——
    // 避免仍在实时航行的USV因为另一艘USV内部切换目标段而被误判为"过时旧数据"淡出。
    const selfEndedForFade=t>ts.time[ts.time.length-1];
    let baseA=1.0;
    if(!hasOwnTransition&&selfEndedForFade&&globalFadeTime!==null&&globalFadeTime!==undefined) {{
      const gfe=t-globalFadeTime;
      baseA=gfe>=fadeDur?0:1.0-gfe/fadeDur;
    }}
    // 多任务USV新任务内容淡入
    let fadeInA=1.0;
    if(hasOwnTransition&&tf.fadeStart!==undefined) {{
      const fia=t-tf.fadeStart;
      fadeInA=Math.min(1.0,fia/fadeDur);
    }}

    // === 渐隐上一个任务的轨迹和目标点 (多任务USV) ===
    if(hasOwnTransition&&tf.fadeStart!==undefined) {{
      const fadeElapsed=t-tf.fadeStart;
      if(fadeElapsed<fadeDur) {{
        const fadeAlpha=1.0-fadeElapsed/fadeDur;
        const oldTask=tf.prevTask;
        // 计算旧任务的轨迹边界
        let oStart=0,oEnd=ts.time.length-1;
        if(ts.task_idx) {{
          for(let i=0;i<ts.time.length;i++) {{ if(ts.task_idx[i]===oldTask) {{ oStart=i; break; }} }}
          for(let i=ts.time.length-1;i>=0;i--) {{ if(ts.task_idx[i]===oldTask) {{ oEnd=i; break; }} }}
        }}
        const oST=ts.time[oStart],oET=ts.time[oEnd];
        let spOS=0,spOE=sp.t.length-1;
        for(let i=0;i<sp.t.length;i++) {{ if(sp.t[i]>=oST) {{ spOS=i; break; }} }}
        for(let i=sp.t.length-1;i>=0;i--) {{ if(sp.t[i]<=oET) {{ spOE=i; break; }} }}
        // 渐隐旧轨迹
        ctx.beginPath(); ctx.strokeStyle=color; ctx.lineWidth=2; ctx.globalAlpha=0.6*fadeAlpha;
        for(let i=spOS;i<=spOE;i++) {{
          const[px,py]=toC(sp.x[i],sp.y[i]);
          if(i===spOS)ctx.moveTo(px,py); else ctx.lineTo(px,py);
        }}
        ctx.stroke(); ctx.globalAlpha=1.0;
        // 渐隐旧目标点
        const oldTargets=d.targets.filter(tgt=>(tgt.task_idx||0)===oldTask);
        oldTargets.forEach(tgt=>{{
          const[tx,ty]=toC(tgt.x,tgt.y);
          ctx.globalAlpha=fadeAlpha;
          drawStar(ctx,tx,ty,8,color);
        }});
        ctx.globalAlpha=1.0;
        // 渐隐旧起点
        const[osx,osy]=toC(ts.pose_x[oStart],ts.pose_y[oStart]);
        ctx.globalAlpha=fadeAlpha;
        ctx.beginPath(); ctx.arc(osx,osy,5,0,Math.PI*2);
        ctx.fillStyle='#4CAF50'; ctx.fill();
        ctx.strokeStyle='#fff'; ctx.lineWidth=1.5; ctx.stroke();
        ctx.globalAlpha=1.0;
      }}
    }}

    // 当前任务段起始索引 — 从任务切换点开始
    let taskStartIdx=0;
    if(ts.task_idx) {{
      taskStartIdx=idx;
      while(taskStartIdx>0 && ts.task_idx[taskStartIdx-1]===curTaskIdx) taskStartIdx--;
    }}
    // 当前任务段结束索引 — 到任务切换点结束
    let taskEndIdx=ts.time.length-1;
    if(ts.task_idx) {{
      taskEndIdx=idx;
      while(taskEndIdx<ts.time.length-1 && ts.task_idx[taskEndIdx+1]===curTaskIdx) taskEndIdx++;
    }}
    const taskStartTime=ts.time[taskStartIdx];
    const taskEndTime=ts.time[taskEndIdx];
    let spTaskStart=0;
    for(let i=0;i<sp.t.length;i++) {{ if(sp.t[i]>=taskStartTime) {{ spTaskStart=i; break; }} }}
    let spTaskEnd=sp.t.length-1;
    for(let i=sp.t.length-1;i>=0;i--) {{ if(sp.t[i]<=taskEndTime) {{ spTaskEnd=i; break; }} }}

    // 原始规划航线基准折线 — 直观对比转弯弧度与航线贴合度
    const curA=baseA*fadeInA;
    if(replayState.showPlannedRoute && ts.planned_routes && ts.planned_routes[String(curTaskIdx)]) {{
      const pPts=ts.planned_routes[String(curTaskIdx)];
      if(pPts.length>=2) {{
        ctx.save();
        ctx.beginPath(); ctx.strokeStyle=color; ctx.lineWidth=1.3; ctx.globalAlpha=0.32*curA;
        ctx.setLineDash([8,5]);
        pPts.forEach((pt,pi)=>{{
          const[rxp,ryp]=toC(pt[0],pt[1]);
          if(pi===0) ctx.moveTo(rxp,ryp); else ctx.lineTo(rxp,ryp);
        }});
        ctx.stroke(); ctx.restore();
      }}
    }}

    // 已走轨迹 — 只显示当前任务段 (baseA=单任务渐隐, fadeInA=新任务淡入)
    ctx.beginPath(); ctx.strokeStyle=color; ctx.lineWidth=2.3; ctx.globalAlpha=0.72*curA;
    let spIdx=0;
    for(let i=0;i<sp.t.length;i++) {{ if(sp.t[i]<=t) spIdx=i; else break; }}
    for(let i=spTaskStart;i<=spIdx;i++) {{
      const[px,py]=toC(sp.x[i],sp.y[i]);
      if(i===spTaskStart)ctx.moveTo(px,py); else ctx.lineTo(px,py);
    }}
    const[interpX,interpY]=toC(cx,cy);
    ctx.lineTo(interpX,interpY);
    ctx.stroke(); ctx.globalAlpha=1.0;

    // 未来轨迹 — 只显示到当前任务结束
    if(spIdx<spTaskEnd) {{
      ctx.beginPath(); ctx.strokeStyle=color; ctx.lineWidth=1; ctx.globalAlpha=0.15*curA;
      ctx.setLineDash([4,4]);
      ctx.moveTo(interpX,interpY);
      for(let i=spIdx+1;i<=spTaskEnd;i++) {{
        const[px,py]=toC(sp.x[i],sp.y[i]);ctx.lineTo(px,py);
      }}
      ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha=1.0;
    }}

    // 目标航点 — 只显示当前任务的目标点
    const taskTargets=d.targets.filter(tgt=>(tgt.task_idx||0)===curTaskIdx);
    taskTargets.forEach(tgt=>{{
      const[tx,ty]=toC(tgt.x,tgt.y);
      const gfk=uK+'_'+curTaskIdx+'_'+tgt.id;
      const reached=!!replayState.goalReachedFlags[gfk];
      const reachedAt=replayState.goalReachedTime[gfk]||0;
      const highlightAge=t-reachedAt;
      const arrStatus=d.arrivalMap[curTaskIdx+'_'+tgt.id]||'unknown';
      const arrColor=(arrStatus==='arrived'||arrStatus==='passed')?'#4CAF50':arrStatus==='diverged'?'#FFB300':'#F44336';
      if(reached) {{
        // 到达动画 (2秒扩散环), 动画结束后保持已完成样式
        if(highlightAge<2.0) {{
          const progress=highlightAge/2.0;
          const fadeAlpha=(1.0-progress*0.5)*curA;
          ctx.globalAlpha=fadeAlpha;
          drawStar(ctx,tx,ty,10,arrColor);
          const ringR=10+progress*18;
          ctx.beginPath(); ctx.arc(tx,ty,ringR,0,Math.PI*2);
          ctx.strokeStyle=arrColor; ctx.lineWidth=2.5;
          ctx.globalAlpha=0.6*fadeAlpha;
          ctx.stroke(); ctx.globalAlpha=1.0;
        }} else {{
          // 动画结束: 以半透明已完成样式保留
          ctx.globalAlpha=0.5*curA;
          drawStar(ctx,tx,ty,7,arrColor);
          ctx.globalAlpha=1.0;
        }}
        ctx.fillStyle=arrColor;
      }} else {{
        ctx.globalAlpha=curA;
        drawStar(ctx,tx,ty,8,color);
        ctx.globalAlpha=1.0;
        ctx.fillStyle=color;
      }}
      if(replayState.showGoalLabels) {{
        ctx.font='9px Inter,system-ui,sans-serif';
        ctx.textAlign='center';
        ctx.fillText(String(tgt.id),tx,ty-16);
      }}
    }});

    // USV到下一个目标的连接虚线
    if(curGoalId>0) {{
      const nextTgt=taskTargets.find(g=>g.id===curGoalId);
      if(nextTgt) {{
        const[tx,ty]=toC(nextTgt.x,nextTgt.y);
        ctx.beginPath(); ctx.moveTo(interpX,interpY); ctx.lineTo(tx,ty);
        ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.globalAlpha=0.5*curA;
        ctx.setLineDash([6,4]); ctx.stroke();
        ctx.setLineDash([]); ctx.globalAlpha=1.0;
        ctx.beginPath(); ctx.arc(tx,ty,12,0,Math.PI*2);
        ctx.strokeStyle=color; ctx.lineWidth=1; ctx.globalAlpha=0.35*curA;
        ctx.stroke(); ctx.globalAlpha=1.0;
      }}
    }}

    // 当前任务段起点
    ctx.globalAlpha=curA;
    const[sx,sy]=toC(ts.pose_x[taskStartIdx],ts.pose_y[taskStartIdx]);
    ctx.beginPath(); ctx.arc(sx,sy,5,0,Math.PI*2);
    ctx.fillStyle='#4CAF50'; ctx.fill();
    ctx.strokeStyle='#fff'; ctx.lineWidth=1.5; ctx.stroke();
    ctx.globalAlpha=1.0;

    // 船 (始终显示,不随任务切换隐藏)
    const[canX,canY]=toC(cx,cy);
    if(ts.rl_active&&ts.rl_active[idx]===1) {{
      ctx.save();
      ctx.beginPath();
      ctx.arc(canX,canY,BOAT_SCALE*0.9,0,Math.PI*2);
      ctx.strokeStyle='rgba(0,188,212,0.9)';
      ctx.lineWidth=2.5;
      ctx.shadowColor='rgba(0,188,212,0.65)';
      ctx.shadowBlur=12;
      ctx.stroke();
      ctx.restore();
    }}
    drawBoat(ctx,canX,canY,yaw,color,BOAT_SCALE);
    // 邻船实际方位避让圆弧
    if(ts.rl_active&&ts.rl_active[idx]===1) {{
      if(!replayState._arcFrame) replayState._arcFrame=0;
      replayState._arcFrame++;
      drawAvoidanceArcs(ctx,canX,canY,ts,idx,t,d);
    }}

    // 标签
    ctx.fillStyle=color; ctx.font='bold 11px Inter,system-ui,sans-serif';
    ctx.textAlign='center'; ctx.fillText(d.usv.id,canX,canY-BOAT_SCALE-4);
    if(ts.rl_active&&ts.rl_active[idx]===1) {{
      ctx.fillStyle='#00BCD4';
      ctx.font='bold 9px Inter,system-ui,sans-serif';
      ctx.fillText('RL',canX,canY+BOAT_SCALE+10);
    }}

    // 到达检测 (使用 taskIdx_goalId 复合键)
    if(ts.goal_id) {{
      const cg=ts.goal_id[idx];
      const cti=ts.task_idx?ts.task_idx[idx]:0;
      const prevKey=uK+'_prev';
      if(!(prevKey in replayState._prevGoals)) replayState._prevGoals[prevKey]={{g:cg,t:cti}};
      const prev=replayState._prevGoals[prevKey];
      if(cg!==prev.g||cti!==prev.t) {{
        const prevFk=uK+'_'+prev.t+'_'+prev.g;
        const arrStatus=d.arrivalMap[prev.t+'_'+prev.g]||'unknown';
        if(!replayState.goalReachedFlags[prevFk] && arrStatus!=='not_reached' && arrStatus!=='unknown') {{
          replayState.goalReachedFlags[prevFk]=true;
          replayState.goalReachedTime[prevFk]=t;
          if(arrStatus==='arrived'||arrStatus==='passed') sfxGoalReached();
          else if(arrStatus==='diverged') sfxGoalDiverged();
        }}
        replayState._prevGoals[prevKey]={{g:cg,t:cti}};
      }}
      if(cg>0&&ts.distance&&ts.distance[idx]<=1.5) {{
        const fk=uK+'_'+cti+'_'+cg;
        const arrStatus=d.arrivalMap[cti+'_'+cg]||'unknown';
        if(!replayState.goalReachedFlags[fk] && (arrStatus==='arrived'||arrStatus==='passed')) {{
          replayState.goalReachedFlags[fk]=true;
          replayState.goalReachedTime[fk]=t;
          sfxGoalReached();
        }}
      }}
      if(idx>=ts.time.length-2&&cg>0) {{
        const fk=uK+'_'+cti+'_'+cg;
        const arrStatus=d.arrivalMap[cti+'_'+cg]||'unknown';
        if(!replayState.goalReachedFlags[fk] && arrStatus!=='not_reached') {{
          replayState.goalReachedFlags[fk]=true;
          replayState.goalReachedTime[fk]=t;
          if(arrStatus==='arrived'||arrStatus==='passed') sfxGoalReached();
          else if(arrStatus==='diverged') sfxGoalDiverged();
        }}
      }}
    }}
  }});

  // v16: 自身日志未开始/已结束的USV通过其他USV的邻居观测数据继续显示位置
  data.forEach(dSelf=>{{
    const ts=dSelf.ts;
    const selfEndTime=ts.time[ts.time.length-1];
    const beforeStart=t<ts.time[0]-0.5;
    if(!beforeStart&&t<=selfEndTime+fadeDur) return; // 自身数据有效或仍在渐隐期，已在上面绘制
    const bestPos=findPeerPos(data,dSelf,t);
    if(bestPos) {{
      const[canX,canY]=toC(bestPos.x,bestPos.y);
      ctx.globalAlpha=0.55;
      drawBoat(ctx,canX,canY,bestPos.yaw,dSelf.usv.color,BOAT_SCALE);
      ctx.globalAlpha=0.55;
      ctx.fillStyle=dSelf.usv.color; ctx.font='bold 11px Inter,system-ui,sans-serif';
      ctx.textAlign='center'; ctx.fillText(dSelf.usv.id+(beforeStart?'(无日志)':'(idle)'),canX,canY-BOAT_SCALE-4);
      ctx.globalAlpha=1.0;
    }}
  }});
  // 无独立日志文件的场内邻船/静止船补绘
  const EXT_COLORS={{'usv_01':'#2196F3','usv_02':'#F44336','usv_03':'#4CAF50','usv_04':'#FF9800','usv_05':'#9C27B0'}};
  for(const nid in extPeerPos) {{
    const pos=extPeerPos[nid];
    const[canX,canY]=toC(pos.x,pos.y);
    const ec=EXT_COLORS[nid]||'#90A4AE';
    ctx.globalAlpha=0.65;
    drawBoat(ctx,canX,canY,pos.yaw,ec,BOAT_SCALE);
    ctx.globalAlpha=0.75;
    ctx.fillStyle=ec; ctx.font='bold 11px Inter,system-ui,sans-serif';
    ctx.textAlign='center'; ctx.fillText(nid+'(静止)',canX,canY-BOAT_SCALE-4);
    ctx.globalAlpha=1.0;
  }}

  // 实时近距标尺 (< 1.6m): 自动连线活跃船与所有邻船并标注实时间距
  const liveHulls=[];
  data.forEach(d=>{{
    const ts=d.ts;
    if(t<ts.time[0]-0.5&&findPeerPos(data,d,t)) {{
      const pp=findPeerPos(data,d,t); if(pp) liveHulls.push({{id:d.usv.id,x:pp.x,y:pp.y}});
      return;
    }}
    if(t>ts.time[ts.time.length-1]+fadeDur&&findPeerPos(data,d,t)) {{
      const pp=findPeerPos(data,d,t); if(pp) liveHulls.push({{id:d.usv.id,x:pp.x,y:pp.y}});
      return;
    }}
    const pos=sampleSmoothPos(d.sp,t);
    if(pos) liveHulls.push({{id:d.usv.id,x:pos.x,y:pos.y}});
  }});
  for(const nid in extPeerPos) liveHulls.push({{id:nid,x:extPeerPos[nid].x,y:extPeerPos[nid].y}});
  for(let i=0;i<liveHulls.length;i++) {{
    for(let j=i+1;j<liveHulls.length;j++) {{
      const a=liveHulls[i], b=liveHulls[j];
      const dd=Math.hypot(a.x-b.x,a.y-b.y);
      if(dd>0.02 && dd<1.60) {{
        const[ax,ay]=toC(a.x,a.y), [bx,by]=toC(b.x,b.y);
        const lc=dd>=0.75?'#69F0AE':dd>=0.45?'#FFD740':'#FF5252';
        ctx.save();
        ctx.beginPath(); ctx.moveTo(ax,ay); ctx.lineTo(bx,by);
        ctx.strokeStyle=lc; ctx.lineWidth=1.6; ctx.setLineDash([3,3]); ctx.globalAlpha=0.85;
        ctx.stroke(); ctx.setLineDash([]);
        const mx=(ax+bx)/2, my=(ay+by)/2;
        ctx.fillStyle='rgba(10,14,23,0.82)';
        ctx.fillRect(mx-21,my-9,42,15);
        ctx.fillStyle=lc; ctx.font='bold 10px Inter,system-ui,sans-serif';
        ctx.textAlign='center'; ctx.fillText(dd.toFixed(2)+'m',mx,my+2);
        ctx.restore();
      }}
    }}
  }}

  // 时间文字
  ctx.fillStyle='rgba(100,255,218,0.75)';
  ctx.font='12px Inter,system-ui,sans-serif';
  ctx.textAlign='left';
  ctx.fillText('t = '+t.toFixed(1)+'s  |  '+replayState.speed+'x',12,20);
}}

// ═══ 初始化 ═══
function init() {{
  buildSelector(); buildReplayToggle(); refreshAll(); renderReplayFrame();

  document.addEventListener('keydown',(ev)=>{{
    if(ev.target && (ev.target.tagName==='INPUT'||ev.target.tagName==='TEXTAREA')) return;
    if(ev.code==='Space') {{ ev.preventDefault(); replayToggle(); }}
    else if(ev.code==='ArrowRight') {{
      ev.preventDefault();
      replayState.currentTime=Math.min(replayState.maxTime,replayState.currentTime+2.0);
      renderReplayFrame(); updateTimeDisplay();
    }} else if(ev.code==='ArrowLeft') {{
      ev.preventDefault();
      replayState.currentTime=Math.max(0,replayState.currentTime-2.0);
      replayState.goalReachedFlags={{}};
      renderReplayFrame(); updateTimeDisplay();
    }}
  }});

  const observer=new IntersectionObserver((entries)=>{{
    entries.forEach(e=>{{
      if(e.isIntersecting) {{
        e.target.classList.add('visible');
        const sn=e.target.id;
        if(sn&&sn.startsWith('s')) sfxSection(parseInt(sn.substring(1))||0);
      }}
    }});
  }},{{threshold:0.1}});
  document.querySelectorAll('.section').forEach(s=>observer.observe(s));
  setTimeout(()=>{{ document.querySelectorAll('#s1,#s2').forEach(s=>s.classList.add('visible')); }},100);

  setTimeout(()=>{{
    initAudio();
    playTone(523,0.1,'sine',0.04);
    setTimeout(()=>playTone(659,0.1,'sine',0.03),100);
    setTimeout(()=>playTone(784,0.15,'triangle',0.03),200);
  }},500);
}}

if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',init);
else init();
</script>
</body>
</html>'''
