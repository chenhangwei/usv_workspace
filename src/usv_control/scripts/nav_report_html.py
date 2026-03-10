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

    def _valid_primary_neighbor(value: Any) -> str | None:
        if isinstance(value, str):
            value = value.strip()
            if value and value != '0':
                return value
            return None
        if isinstance(value, (int, float)) and value != 0:
            return str(int(value)) if float(value).is_integer() else str(value)
        return None

    def _count_active_episodes(field: str) -> tuple[int, float]:
        total_duration = 0.0
        episode_count = 0
        index = 0
        while index < len(data):
            if data[index].get(field) == 1:
                episode_start = data[index]['timestamp']
                end_index = index
                while end_index < len(data) and data[end_index].get(field) == 1:
                    end_index += 1
                episode_end = data[end_index - 1]['timestamp']
                total_duration += max(0.0, episode_end - episode_start)
                episode_count += 1
                index = end_index
            else:
                index += 1
        return episode_count, total_duration

    t0 = data[0]['timestamp']
    duration = data[-1]['timestamp'] - t0
    t = [d['timestamp'] - t0 for d in data]

    guided = [d for d in data if d.get('flight_mode') == 'GUIDED']
    hold = [d for d in data if d.get('flight_mode') == 'HOLD']

    info = {
        'usv_id': header_info.get('usv_id', 'unknown'),
        'version': header_info.get('version', '?'),
        'params': header_info.get('params', {}),
        'duration_s': duration,
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

    hes_guided = [abs(d.get('heading_error_deg', 0)) for d in guided
                  if isinstance(d.get('heading_error_deg'), (int, float))]
    he_avg = sum(hes_guided)/len(hes_guided) if hes_guided else None
    he_max = max(hes_guided) if hes_guided else None

    mpc = {}
    if 'mpc_solve_time_ms' in data[0]:
        st = sorted(d.get('mpc_solve_time_ms', 0) for d in data)
        mpc = {
            'avg': sum(st)/len(st),
            'p95': st[int(len(st)*0.95)],
            'max': st[-1],
        }

    om_cmds = [d.get('cmd_omega', 0) for d in data if isinstance(d.get('cmd_omega'), (int, float))]
    omega = {
        'avg': sum(abs(o) for o in om_cmds)/len(om_cmds) if om_cmds else 0,
        'max': max(abs(o) for o in om_cmds) if om_cmds else 0,
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

    orca = {}
    if 'orca_active' in data[0]:
        active_samples = [d for d in data if d.get('orca_active') == 1]
        active_cnt = len(active_samples)
        orca['active_count'] = active_cnt
        orca['active_pct'] = active_cnt / len(data) * 100 if data else 0
        if active_cnt > 0:
            dists_o = [d.get('orca_closest_distance', -1) for d in active_samples
                     if isinstance(d.get('orca_closest_distance'), (int, float)) and d['orca_closest_distance'] > 0]
            orca['min_dist'] = min(dists_o) if dists_o else None
            orca['avg_dist'] = sum(dists_o)/len(dists_o) if dists_o else None
            hard_brakes = sum(1 for d in data if d.get('orca_hard_brake') == 1)
            orca['hard_brake_count'] = hard_brakes
            encounters = {}
            for d in active_samples:
                et = d.get('orca_encounter_type', 'none')
                if isinstance(et, str) and et != 'none':
                    encounters[et] = encounters.get(et, 0) + 1
            orca['encounters'] = encounters

            primary_counts = {}
            for d in active_samples:
                primary_id = _valid_primary_neighbor(d.get('orca_primary_neighbor_id'))
                if primary_id is not None:
                    primary_counts[primary_id] = primary_counts.get(primary_id, 0) + 1
            if primary_counts:
                top_primary_neighbor = max(primary_counts.items(), key=lambda item: item[1])
                orca['primary_neighbors'] = primary_counts
                orca['top_primary_neighbor'] = top_primary_neighbor[0]
                orca['top_primary_neighbor_pct'] = top_primary_neighbor[1] / active_cnt * 100

        if 'orca_escape_active' in data[0]:
            escape_samples = [d for d in data if d.get('orca_escape_active') == 1]
            escape_sample_count = len(escape_samples)
            orca['escape_sample_count'] = escape_sample_count
            orca['escape_active_pct'] = escape_sample_count / len(data) * 100 if data else 0
            if escape_sample_count > 0:
                escape_episode_count, escape_total_duration = _count_active_episodes('orca_escape_active')
                phase_counts = {}
                direction_counts = {}
                for d in escape_samples:
                    phase = d.get('orca_escape_phase', 0)
                    if isinstance(phase, (int, float)):
                        phase_num = int(phase)
                        if phase_num > 0:
                            phase_label = 'phase1_reverse_turn' if phase_num == 1 else 'phase2_weakened_orca' if phase_num == 2 else f'phase{phase_num}'
                            phase_counts[phase_label] = phase_counts.get(phase_label, 0) + 1
                    direction = d.get('orca_escape_direction', 0)
                    if isinstance(direction, (int, float)):
                        direction_num = int(direction)
                        if direction_num != 0:
                            direction_label = 'left' if direction_num > 0 else 'right'
                            direction_counts[direction_label] = direction_counts.get(direction_label, 0) + 1

                orca['escape_episode_count'] = escape_episode_count
                orca['escape_total_duration'] = escape_total_duration
                orca['escape_phases'] = phase_counts
                orca['escape_directions'] = direction_counts
                orca['max_escape_count'] = max(
                    int(d.get('orca_escape_count', 0))
                    for d in data
                    if isinstance(d.get('orca_escape_count'), (int, float))
                )

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
        wv = [d.get('wifi_rssi_dbm', -100) for d in gdata
              if isinstance(d.get('wifi_rssi_dbm'), (int, float)) and d['wifi_rssi_dbm'] > -100]
        if wv:
            pg['wifi_avg'] = sum(wv)/len(wv)
        per_goal.append(pg)

    score_cte = max(0, 100 - (cte_avg or 0.5) * 200)
    score_he = max(0, 100 - (he_avg or 40) * 2.5)
    score_mpc = max(0, 100 - mpc.get('avg', 66) * 1.5)
    score_spd = min(100, velocity['guided_avg'] / 0.3 * 100) if velocity['guided_avg'] > 0 else 0
    total_score = score_cte * 0.4 + score_he * 0.4 + score_mpc * 0.1 + score_spd * 0.1
    grade = 'A+' if total_score >= 80 else 'A' if total_score >= 70 else 'B' if total_score >= 60 else 'C' if total_score >= 40 else 'D'

    scoring = {
        'cte': score_cte, 'he': score_he, 'mpc': score_mpc, 'spd': score_spd,
        'total': total_score, 'grade': grade,
        'cte_label': '优秀' if (cte_avg or 999) < 0.1 else '良好' if (cte_avg or 999) < 0.3 else '一般' if (cte_avg or 999) < 0.5 else '较差',
        'he_label': '优秀' if (he_avg or 999) < 5 else '良好' if (he_avg or 999) < 10 else '一般' if (he_avg or 999) < 20 else '较差',
        'mpc_label': '优秀' if mpc.get('avg', 999) < 15 else '正常' if mpc.get('avg', 999) < 30 else '偏高' if mpc.get('avg', 999) < 50 else '过高',
    }

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
    if 'omega_actual' in data[0]:
        ts_data['omega_actual'] = [data[i].get('omega_actual', 0) for i in range(0, len(data), step)]
    if 'current_tau_omega' in data[0]:
        ts_data['tau'] = [data[i].get('current_tau_omega', 0) for i in range(0, len(data), step)]
    if 'ampc_tau_estimated' in data[0]:
        ts_data['tau_estimated'] = [data[i].get('ampc_tau_estimated', 0) for i in range(0, len(data), step)]
        ts_data['tau_confidence'] = [data[i].get('ampc_tau_confidence', 0) for i in range(0, len(data), step)]
    if 'wifi_rssi_dbm' in data[0]:
        ts_data['wifi_rssi'] = [data[i].get('wifi_rssi_dbm', -100) for i in range(0, len(data), step)]
    if 'orca_active' in data[0]:
        ts_data['orca_active'] = [data[i].get('orca_active', 0) for i in range(0, len(data), step)]
        ts_data['orca_closest'] = [data[i].get('orca_closest_distance', -1) for i in range(0, len(data), step)]
        ts_data['orca_linear_corr'] = [data[i].get('orca_linear_correction', 0) for i in range(0, len(data), step)]
        ts_data['orca_angular_corr'] = [data[i].get('orca_angular_correction', 0) for i in range(0, len(data), step)]
        ts_data['orca_hard_brake'] = [data[i].get('orca_hard_brake', 0) for i in range(0, len(data), step)]
    if 'orca_primary_neighbor_id' in data[0]:
      ts_data['orca_primary_neighbor'] = [
        _valid_primary_neighbor(data[i].get('orca_primary_neighbor_id')) or ''
        for i in range(0, len(data), step)
      ]
    if 'orca_escape_active' in data[0]:
      ts_data['orca_escape_active'] = [data[i].get('orca_escape_active', 0) for i in range(0, len(data), step)]
      ts_data['orca_escape_phase'] = [
        int(data[i].get('orca_escape_phase', 0)) if data[i].get('orca_escape_active', 0) == 1 else 0
        for i in range(0, len(data), step)
      ]
      ts_data['orca_escape_direction'] = [
        int(data[i].get('orca_escape_direction', 0)) if data[i].get('orca_escape_active', 0) == 1 else 0
        for i in range(0, len(data), step)
      ]
      ts_data['orca_escape_count'] = [data[i].get('orca_escape_count', 0) for i in range(0, len(data), step)]

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
        'cte': {'avg': cte_avg, 'max': cte_max, 'rms': cte_rms},
        'heading_error': {'avg': he_avg, 'max': he_max},
        'mpc': mpc, 'omega': omega, 'osc_freq': osc_freq,
        'ampc': ampc, 'orca': orca, 'wifi': wifi,
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
            'targets_n': len(rd['targets']),
            'guided_pct': round(info['guided_pct'], 1),
            'cte_avg': rd['cte'].get('avg'),
            'he_avg': rd['heading_error'].get('avg'),
            'speed_avg': round(rd['velocity']['guided_avg'], 3),
            'mpc_avg': round(rd['mpc'].get('avg', 0), 1) if rd['mpc'] else None,
            'orca_active_pct': round(rd['orca'].get('active_pct', 0), 1) if rd['orca'] else 0,
            'orca_hard_brakes': rd['orca'].get('hard_brake_count', 0) if rd['orca'] else 0,
            'orca_primary': rd['orca'].get('top_primary_neighbor', '') if rd['orca'] else '',
            'escape_count': rd['orca'].get('escape_episode_count', 0) if rd['orca'] else 0,
            'escape_duration': round(rd['orca'].get('escape_total_duration', 0), 1) if rd['orca'] else 0,
            'wifi_avg': round(rd['wifi'].get('avg', -100), 1) if rd['wifi'] else None,
            'wifi_grade': rd['wifi'].get('grade', '') if rd['wifi'] else '',
        })

    all_ts, all_targets, all_per_goal, all_summaries = {}, {}, {}, {}
    for rd in all_report_data:
        key = str(rd['_idx'])
        all_ts[key] = rd['ts']
        all_targets[key] = rd['targets']
        all_per_goal[key] = rd['per_goal']
        all_summaries[key] = {
            'scoring': rd['scoring'],
            'velocity': rd['velocity'],
            'cte': rd['cte'],
            'heading_error': rd['heading_error'],
            'mpc': rd['mpc'],
            'omega': rd['omega'],
            'osc_freq': rd['osc_freq'],
            'ampc': rd['ampc'],
            'orca': rd['orca'],
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
  padding:7px 5px; text-align:center; border-bottom:2px solid var(--border);
  position:sticky; top:0; z-index:2;
}}
.compare-table td {{
  padding:5px; text-align:center; border-bottom:1px solid rgba(255,255,255,0.05);
}}
.compare-table tr:hover td {{ background:rgba(100,255,218,0.05); }}
.compare-table .best {{ color:#69F0AE; font-weight:700; }}
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
.orca-summary-grid {{
  display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr));
  gap:14px; margin-bottom:14px;
}}
.orca-summary-card {{ margin-bottom:0; }}
.orca-summary-card h4 {{
  display:flex; align-items:center; gap:8px; margin-bottom:10px;
  font-size:0.95rem; color:var(--text);
}}
.orca-summary-card .orca-line {{
  font-size:0.76rem; color:var(--text2); margin-top:6px; line-height:1.65;
}}
.orca-summary-card .orca-line strong {{ color:var(--accent); font-weight:700; }}
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
  <div class="subtitle">{n_usv} 条航行记录 | {now}</div>
</div>

<div class="container">

<div class="section" id="s1">
  <div class="section-title"><span class="num">1</span> USV 选择器</div>
  <div style="text-align:center;margin-bottom:6px;">
    <button class="btn-toggle-all" onclick="toggleAllUSV()">全选 / 反选</button>
  </div>
  <div class="usv-selector" id="usv-selector"></div>
  <div class="reading-guide">
    <strong>操作说明：</strong>点击USV标签可显示/隐藏该USV的数据。所有图表和轨迹重放都会联动更新。
    多选模式下可对比不同USV的航行表现。
  </div>
</div>

<div class="section" id="s2">
  <div class="section-title"><span class="num">2</span> 对比总览</div>
  <div id="ai-insights"></div>
  <div id="compare-area"></div>
  <div class="reading-guide">
    <strong>📊 对比总览阅读指南：</strong><br>
    • <strong>评分</strong>：综合评分(0-100)，权重: CTE 40% + 航向误差 40% + MPC 10% + 速度 10%。A+≥80, A≥70, B≥60, C≥40, D&lt;40<br>
    • <strong>均CTE</strong>：横向跟踪偏差均值，&lt;0.1m=优秀，0.1-0.3m=良好，0.3-0.5m=一般，&gt;0.5m=较差<br>
    • <strong>均航向误差</strong>：&lt;5°=优秀，5-10°=良好，10-20°=一般，&gt;20°=较差<br>
    • <strong>GUIDED%</strong>：自动导航模式占比，越高越稳定，低值意味着频繁切为HOLD模式<br>
    • <strong>ORCA%</strong>：避障系统激活时间占比，反映编队密集程度<br>
    • 🏆标记=该指标最优者 | 绿色=最佳值 | 点击表头可以排序<br>
    <strong>🤖 AI分析摘要</strong>自动评估每个USV的导航质量，用✅❗❌标记关键发现，帮助快速定位问题。
  </div>
</div>

<div class="section" id="s3">
  <div class="section-title"><span class="num">3</span> 路径重放</div>
  <div class="replay-container">
    <div class="replay-canvas-wrap">
      <canvas id="replay-canvas"></canvas>
    </div>
    <div class="replay-info" id="replay-info"></div>
    <div class="replay-controls">
      <button onclick="replayRestart()" title="重新开始">⏮</button>
      <button id="btn-play" onclick="replayToggle()" title="播放/暂停">▶</button>
      <button onclick="replaySlower()" title="减速">⏪</button>
      <span class="speed-display" id="speed-display">1.0x</span>
      <button onclick="replayFaster()" title="加速">⏩</button>
      <div class="replay-progress" id="replay-progress" onclick="replaySeek(event)">
        <div class="fill" id="replay-fill" style="width:0%"></div>
      </div>
      <span class="time-display" id="replay-time">0.0s / 0.0s</span>
      <button id="btn-goal-labels" onclick="toggleGoalLabels()" title="显示/隐藏目标点ID" style="margin-left:8px;font-size:12px;padding:2px 8px;background:rgba(255,255,255,0.08);border:1px solid rgba(255,255,255,0.15);color:#9e9e9e;border-radius:4px;cursor:pointer;">🏷️ 航点ID</button>
    </div>
  </div>
  <div class="reading-guide">
    <strong>路径重放：</strong>实时回放各USV的航行轨迹。船形图标显示位置和朝向（参考GS导航预览风格）。<br>
    ▶ 播放/暂停 | ⏪⏩ 0.25x~16x变速 | 点击进度条跳转 | ⏮ 重新开始 |
    🟢起点 🔴终点 ⭐目标航点 | 🏷️ 显示/隐藏航点ID | 勾选USV即加入重放<br>
    到达标识(数据驱动)：🟢绿色=确认到达(distance≤1.5m) | 🟡黄色=偏离到达(距离>1.5m，偏离检测判定已到达) | 🔴红色=未到达
  </div>
</div>

<div class="section" id="s4">
  <div class="section-title"><span class="num">4</span> 航行轨迹对比</div>
  <div class="chart-box">
    <div id="chart-traj" style="height:500px;"></div>
  </div>
  <div class="reading-guide">
    <strong>🗺️ 航行轨迹阅读指南：</strong>各USV以不同颜色显示XY平面轨迹。<br>
    <strong>图例：</strong>实线=GUIDED(自动导航) | 灰点=HOLD(悬停) | ●绿=起点 | ■红=终点 | ★=目标航点<br>
    <strong>分析方法：</strong><br>
    • <strong>理想状态</strong>：轨迹平滑且紧贴航点连线，弯道处无大幅振荡<br>
    • <strong>蛇形振荡</strong>：弯道处的左右摆动 → 角速度控制器增益过高或tau参数不匹配<br>
    • <strong>系统性偏移</strong>：直道段持续偏离航线 → CTE控制不足或外部干扰(风/水流)<br>
    • <strong>对比多USV</strong>：相同路线上不同USV的轨迹分散程度反映一致性<br>
    💡 鼠标悬停查看坐标 | 滚轮缩放 | 拖拽平移 | 双击重置
  </div>
</div>

<div class="section" id="s5">
  <div class="section-title"><span class="num">5</span> 速度对比</div>
  <div class="chart-box">
    <div id="chart-velocity" style="height:380px;"></div>
  </div>
  <div class="reading-guide">
    <strong>🚀 速度分析阅读指南：</strong>实线=实际速度，虚线=速度指令(cmd_vx)。<br>
    <strong>分析方法：</strong><br>
    • <strong>跟踪性能</strong>：实际速度应紧密跟随指令曲线—差距大说明推进器响应不足或过载<br>
    • <strong>HOLD期间</strong>：速度应接近0—非零说明IMU漂移或GPS跳变<br>
    • <strong>加减速过渡</strong>：急停/急加速处是否平滑—突变可能导致系统不稳定<br>
    • <strong>典型值</strong>：GUIDED期间平均速度0.2-0.5 m/s，低于0.1可能推进力不足
  </div>
</div>

<div class="section" id="s6">
  <div class="section-title"><span class="num">6</span> 误差分析</div>
  <div class="chart-box">
    <div id="chart-cte" style="height:330px;"></div>
    <div class="chart-guide">
      📐 CTE (Cross Track Error) = USV偏离理想航线的垂直距离(m)。正值=偏右，负值=偏左。
      目标: &lt;0.1m(优秀), &lt;0.3m(良好), &lt;0.5m(一般), &gt;0.5m(较差)。
      持续同一方向→系统性偏移，高频振荡→控制器需调优。
    </div>
  </div>
  <div class="chart-box">
    <div id="chart-he" style="height:330px;"></div>
    <div class="chart-guide">
      🧭 航向误差 = USV实际航向与目标航向的差值(°)。
      转向时出现大峰值正常，关键看直道段的稳态误差。
      持续大误差+低CTE → USV“蟹行”，磁力计偏差或流角问题。
    </div>
  </div>
  <div class="chart-box">
    <div id="chart-distance" style="height:300px;"></div>
    <div class="chart-guide">
      📏 到达距离 = USV与当前目标航点的实时距离(m)。
      绿色虚线=1.5m到达阈值，低于此线表示成功到达。
      距离不降反升→驶过航点未检测或被避障推离。航点切换时距离跳变属正常。
    </div>
  </div>
  <div class="reading-guide">
    <strong>📊 误差分析综合指南：</strong><br>
    三张图表分别展示横向偏差(CTE)、航向误差和到达距离，它们共同反映导航精度。<br>
    • <strong>CTE+HE同时大</strong>：控制器整体性能不佳—检查MPC权重或约束<br>
    • <strong>CTE小但HE大</strong>：USV在航线上但方向不对—磁力计校准或flow angle问题<br>
    • <strong>某航点误差突增</strong>：结合距离图确认是否因避障或环境干扰引起
  </div>
</div>

<div class="section" id="s7">
  <div class="section-title"><span class="num">7</span> 控制指令</div>
  <div class="chart-box">
    <div id="chart-control" style="height:380px;"></div>
  </div>
  <div class="reading-guide">
    <strong>⚙️ 角速度控制阅读指南：</strong>ω_cmd=控制器输出的角速度指令(rad/s)，ω_act=传感器测量的实际角速度。<br>
    <strong>分析方法：</strong><br>
    • <strong>cmd与act差距</strong>：反映舶机/推进器响应延迟(tau值)，差距越小跟踪越好<br>
    • <strong>高频振荡</strong>：cmd快速正负交替 → 控制器过激，需降低MPC权重或增大预测步长<br>
    • <strong>零交叉频率</strong>：“左右摆动”现象，频率&gt;0.5Hz需要调参<br>
    • <strong>饱和(持续最大值)</strong>：推进力不足以满足控制需求，可能需要降低目标速度
  </div>
</div>

<div class="section" id="s8">
  <div class="section-title"><span class="num">8</span> 高级分析</div>
  <div class="chart-box">
    <div id="chart-mpc" style="height:280px;"></div>
    <div class="chart-guide">
      ⏱ MPC求解时间 = 模型预测控制器每步的计算耗时(ms)。
      红色虚线=50ms警戒线。&lt;15ms=优秀，15-30ms=正常，30-50ms=偏高，&gt;50ms=过高(可能影响10Hz控制循环)。
      突增尖峰通常对应航点切换或大角度转向。
    </div>
  </div>
  <div class="chart-box" id="chart-tau-box" style="display:none;">
    <div id="chart-tau" style="height:320px;"></div>
    <div class="chart-guide">
      🔄 τ(tau) = 推进器动力学时间常数(s)，反映舶机/推进响应速度。
      实线=控制器实际使用值，虚线=AMPC在线估计值。两者收敛说明辨识成功。
      τ过小→控制过激(振荡)，τ过大→控制迟钝(轨迹偏差大)。
    </div>
  </div>
  <div class="chart-box" id="chart-wifi-box" style="display:none;">
    <div id="chart-wifi" style="height:280px;"></div>
    <div class="chart-guide">
      📶 WiFi RSSI = 与岸基站点的无线信号强度(dBm)。
      绿线(-50dBm)=优秀，橙线(-70dBm)=弱信号警告。
      信号弱时遥控/参数更新可能延迟，影响编队同步。信号与距离有关。
    </div>
  </div>
  <div class="reading-guide">
    <strong>🔬 高级分析综合指南：</strong><br>
    • <strong>MPC慢+CTE大</strong>：控制器计算不及时，导致轨迹偏差。可减少预测步数或简化约束<br>
    • <strong>τ不收敛</strong>：AMPC辨识未完成，可能数据不足或环境变化太快<br>
    • <strong>WiFi信号与航点</strong>：结合轨迹图，某些位置信号差可能影响编队协同
  </div>
</div>

<div class="section" id="s9">
  <div class="section-title"><span class="num">9</span> ORCA 避障分析</div>
  <div id="orca-summary-area"></div>
  <div class="chart-box" id="chart-orca-box" style="display:none;">
    <div id="chart-orca-activation" style="height:280px;"></div>
  </div>
  <div class="chart-box" id="chart-orca-dist-box" style="display:none;">
    <div id="chart-orca-dist" style="height:280px;"></div>
  </div>
  <div class="chart-box" id="chart-orca-corr-box" style="display:none;">
    <div id="chart-orca-corrections" style="height:280px;"></div>
  </div>
  <div class="chart-box" id="chart-orca-escape-box" style="display:none;">
    <div id="chart-orca-escape" style="height:280px;"></div>
  </div>
  <div class="reading-guide">
    <strong>🛡️ ORCA 避障分析阅读指南：</strong>ORCA (Optimal Reciprocal Collision Avoidance) 是USV的分布式避碰算法。<br>
    <strong>图表1：避障激活</strong> — 显示ORCA是否检测到邻近USV并激活避碰(1=激活, 0=未激活)。激活率高表示编队密集。<br>
    <strong>图表2：最近邻距离</strong> — 与最近邻USV的实时距离，红虚线=1.0m安全阈值。低于此线需关注。<br>
    <strong>图表3：速度修正量</strong> — ORCA对原始控制指令的修正量(线速度+角速度)，修正越大说明碰撞风险越高。<br>
    <strong>图表4：脱困阶段</strong> — v17 新增。0=未脱困，1=后退转向，2=削弱绕行；如果持续停在高位，说明控制器正在主动解除锁死。<br>
    <strong>关键指标：</strong><br>
    • <strong>激活率</strong>：0%=全程无碰撞风险，高占比=密集编队或频繁交叉<br>
    • <strong>硬刹车</strong>：紧急制动次数，≥3次建议检查路径规划是否合理<br>
    • <strong>主导邻船</strong>：避障激活时最常压制当前艇的邻船，可快速定位互锁对象<br>
    • <strong>遭遇类型</strong>：head_on=迎面, crossing_give_way/crossing_stand_on=交叉会遇, being_overtaken/overtaking=追越, stationary=近距伴随/静态压制<br>
    <strong>分析思路：</strong>先看主导邻船是否高度集中，再结合脱困统计判断是正常避障、长时间互抑，还是已经进入脱困恢复流程。
  </div>
</div>

<div class="section" id="s10">
  <div class="section-title"><span class="num">10</span> 每航点统计</div>
  <div id="per-goal-area"></div>
  <div class="reading-guide">
    <strong>📋 每航点统计阅读指南：</strong>按目标航点(G1,G2...)分别统计导航性能。<br>
    <strong>分析方法：</strong><br>
    • <strong>最近距离</strong>：✅=确认到达(distance≤1.5m)，☑️=偏离到达(distance>1.5m，偏离检测判定已到达)，❌=未到达<br>
    • <strong>时长</strong>：各航点耗时，异常长的航点可能存在问题(逆流、避障、GPS跳变)<br>
    • <strong>均CTE</strong>：与总均CTE对比，找出最差航段 → 定位具体问题区域<br>
    • <strong>GUIDED%</strong>：低于80%说明该段频繁退出自动导航<br>
    <strong>对比思路：</strong>多USV在相同航点的表现差异，可发现是环境问题(所有USV都差)还是单船问题(只有某USV差)。
  </div>
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
  buildSelector(); refreshAll();
}}

function refreshAll() {{
  buildSummaryCards(); buildCompareTable(); drawTrajectory(); drawVelocity();
  drawCTE(); drawHE(); drawDistance(); drawControl(); drawMPC(); drawTau(); drawWifi();
  buildOrcaSummary(); drawOrca(); buildPerGoal(); updateReplayInfo();
}}

function formatOrcaCounter(counter,total,limit) {{
  if(!counter) return '—';
  const entries=Object.entries(counter).sort((a,b)=>b[1]-a[1]);
  if(!entries.length) return '—';
  return entries.slice(0,limit||3).map(([key,val])=>{{
    const pct=total&&total>0 ? ` (${{(val/total*100).toFixed(1)}}%)` : '';
    return `${{key}}: ${{val}}${{pct}}`;
  }}).join(' | ');
}}

function buildOrcaSummary() {{
  const el=document.getElementById('orca-summary-area');
  if(!el) return;
  const active=getActive();
  if(!active.length) {{ el.innerHTML=''; return; }}
  const cards=[];
  active.forEach(u=>{{
    const sum=ALL_SUM[u.idx];
    const orca=sum&&sum.orca;
    if(!orca || (!orca.active_count && !orca.escape_sample_count && !orca.top_primary_neighbor)) return;
    const activeText=orca.active_count
      ? `${{orca.active_count}} 次 / ${{(orca.active_pct||0).toFixed(1)}}%`
      : '未触发';
    const distText=orca.min_dist!==null && orca.min_dist!==undefined
      ? `${{orca.min_dist.toFixed(2)}}m / 均值 ${{(orca.avg_dist||0).toFixed(2)}}m`
      : '—';
    const primaryText=orca.top_primary_neighbor
      ? `${{orca.top_primary_neighbor}} (${{(orca.top_primary_neighbor_pct||0).toFixed(1)}}%)`
      : '—';
    const primaryTopText=formatOrcaCounter(orca.primary_neighbors, orca.active_count, 3);
    const encounterText=formatOrcaCounter(orca.encounters, orca.active_count, 4);
    const escapeText=orca.escape_episode_count
      ? `${{orca.escape_episode_count}} 次 / ${{(orca.escape_total_duration||0).toFixed(1)}}s`
      : '未触发';
    const phaseText=formatOrcaCounter(orca.escape_phases, orca.escape_sample_count, 3);
    const dirText=formatOrcaCounter(orca.escape_directions, orca.escape_sample_count, 2);
    cards.push(`
      <div class="chart-box orca-summary-card">
        <h4><span style="color:${{u.color}};font-weight:700">●</span> ${{u.id}}</h4>
        <div class="orca-line"><strong>激活:</strong> ${{activeText}}</div>
        <div class="orca-line"><strong>最近邻距离:</strong> ${{distText}}</div>
        <div class="orca-line"><strong>硬刹车:</strong> ${{orca.hard_brake_count||0}} 次</div>
        <div class="orca-line"><strong>主导邻船:</strong> ${{primaryText}}</div>
        <div class="orca-line"><strong>邻船分布:</strong> ${{primaryTopText}}</div>
        <div class="orca-line"><strong>遭遇类型:</strong> ${{encounterText}}</div>
        <div class="orca-line"><strong>脱困:</strong> ${{escapeText}}${{orca.max_escape_count?` | max_count=${{orca.max_escape_count}}`:''}}</div>
        <div class="orca-line"><strong>脱困阶段:</strong> ${{phaseText}}</div>
        <div class="orca-line"><strong>脱困方向:</strong> ${{dirText}}</div>
      </div>`);
  }});
  if(!cards.length) {{
    el.innerHTML='<div class="reading-guide">当前所选 USV 没有可用的 ORCA v14/v17 数据。</div>';
    return;
  }}
  el.innerHTML=`<div class="orca-summary-grid">${{cards.join('')}}</div>`;
}}

// ═══ 对比表 ═══
function buildCompareTable() {{
  const el=document.getElementById('compare-area');
  const active=USV_LIST.filter(u=>activeUSVs.has(u.idx));
  if(!active.length) {{ el.innerHTML='<p style="color:var(--text2);text-align:center">请选择至少一个USV</p>'; return; }}
  const bestCte=Math.min(...active.map(u=>u.cte_avg||999));
  const bestHe=Math.min(...active.map(u=>u.he_avg||999));
  const bestScore=Math.max(...active.map(u=>u.score));
  let rows=active.map(u=>{{
    const bc=u.cte_avg!==null&&u.cte_avg===bestCte&&active.length>1;
    const bh=u.he_avg!==null&&u.he_avg===bestHe&&active.length>1;
    const bs=u.score===bestScore&&active.length>1;
    return `<tr>
      <td><span style="color:${{u.color}};font-weight:700">●</span> ${{u.id}}</td>
      <td style="max-width:160px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap" title="${{u.csv}}">${{u.csv||'-'}}</td>
      <td>${{u.version}}</td>
      <td style="color:${{u.color}}">${{u.grade}} (${{u.score}}${{bs?' 🏆':''}})</td>
      <td>${{u.duration}}s</td><td>${{u.records}}</td><td>${{u.targets_n}}</td>
      <td>${{u.guided_pct}}%</td>
      <td class="${{bc?'best':''}}">${{u.cte_avg!==null?u.cte_avg.toFixed(3)+'m':'N/A'}}</td>
      <td class="${{bh?'best':''}}">${{u.he_avg!==null?u.he_avg.toFixed(1)+'°':'N/A'}}</td>
      <td>${{u.speed_avg}} m/s</td>
      <td>${{u.mpc_avg!==null?u.mpc_avg+'ms':'—'}}</td>
      <td>${{u.orca_active_pct?u.orca_active_pct+'%':'—'}}</td>
      <td>${{u.wifi_avg!==null?u.wifi_avg+'dBm':'—'}}</td></tr>`;
  }}).join('');
  el.innerHTML=`<div class="chart-box" style="overflow-x:auto">
    <table class="compare-table"><thead><tr>
      <th>USV</th><th>文件</th><th>版本</th><th>评分</th>
      <th>时长</th><th>数据</th><th>航点</th><th>GUIDED%</th>
      <th>均CTE</th><th>均航向误差</th><th>均速</th>
      <th>MPC</th><th>ORCA%</th><th>WiFi</th>
    </tr></thead><tbody>${{rows}}</tbody></table></div>`;
}}

// ═══ 图表 ═══
function getActive() {{ return USV_LIST.filter(u=>activeUSVs.has(u.idx)); }}
function emptyTrace() {{ return [{{x:[],y:[]}}]; }}

function drawTrajectory() {{
  const traces=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts) return;
    let gx=[],gy=[],hx=[],hy=[];
    for(let i=0;i<ts.pose_x.length;i++) {{
      if(ts.flight_mode[i]==='GUIDED') {{ gx.push(ts.pose_x[i]);gy.push(ts.pose_y[i]); }}
      else {{ hx.push(ts.pose_x[i]);hy.push(ts.pose_y[i]); }}
    }}
    traces.push({{x:gx,y:gy,mode:'lines',name:u.id+' GUIDED',
      line:{{color:u.color,width:2}},hovertemplate:u.id+' X:%{{x:.2f}} Y:%{{y:.2f}}'}});
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
    // v16: 邻居轨迹 (虚线叠加)
    if(ts.neighbors) {{
      Object.keys(ts.neighbors).forEach(nid=>{{
        const nb=ts.neighbors[nid];
        if(!nb||nb.x.length<5) return;
        // 检查此邻居是否已经作为独立USV被绘制 (避免重复)
        const alreadyActive=getActive().some(a=>a.id===nid);
        if(alreadyActive) return;
        traces.push({{x:nb.x,y:nb.y,mode:'lines',name:nid+' (neighbor)',
          line:{{color:u.color,width:1,dash:'dot'}},opacity:0.4,showlegend:true}});
      }});
    }}
  }});
  Plotly.react('chart-traj',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'航行轨迹对比',
    xaxis:{{...darkLayout.xaxis,title:'X (m)',scaleanchor:'y'}},
    yaxis:{{...darkLayout.yaxis,title:'Y (m)'}},
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
    traces.push({{x:ts.time,y:ts.cte,name:u.id,line:{{color:u.color,width:1.2}},
      fill:'tozeroy',fillcolor:u.color+'15'}});
  }});
  Plotly.react('chart-cte',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'横向偏差 CTE 对比',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'CTE (m)'}},
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
    traces.push({{x:ts.time,y:ts.cmd_omega,name:u.id+' ω_cmd',line:{{color:u.color,width:1}}}});
    if(ts.omega_actual) traces.push({{x:ts.time,y:ts.omega_actual,name:u.id+' ω_act',
      line:{{color:u.color,width:1.5,dash:'dot'}},showlegend:false}});
  }});
  Plotly.react('chart-control',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'角速度对比',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'ω (rad/s)'}},
  }},cfg);
}}

function drawMPC() {{
  const traces=[];
  getActive().forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts||!ts.mpc_time) return;
    traces.push({{x:ts.time,y:ts.mpc_time,name:u.id,line:{{color:u.color,width:1.2}}}});
  }});
  Plotly.react('chart-mpc',traces.length?traces:emptyTrace(),{{
    ...darkLayout,title:'MPC 求解时间',
    xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Time (ms)'}},
    shapes:[{{type:'line',x0:0,x1:1,xref:'paper',y0:50,y1:50,
      line:{{color:'#EF5350',width:1.5,dash:'dash'}}}}],
  }},cfg);
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
      return `<tr><td>G${{g.id}}</td><td>${{g.duration.toFixed(1)}}s</td>
        <td>${{arr}} ${{g.min_dist!==null?g.min_dist.toFixed(2)+'m':'N/A'}}</td>
        <td>${{g.avg_cte!==null?g.avg_cte.toFixed(3)+'m':'N/A'}}</td>
        <td>${{g.avg_he!==null?g.avg_he.toFixed(1)+'°':'N/A'}}</td>
        <td>${{g.avg_speed!==null?g.avg_speed.toFixed(3)+' m/s':'N/A'}}</td>
        <td>${{g.guided_pct.toFixed(0)}}%</td></tr>`;
    }}).join('');
    html+=`<div class="chart-box">
      <h4 style="color:${{u.color}};margin-bottom:6px">● ${{u.id}}</h4>
      <table class="compare-table"><thead><tr>
        <th>航点</th><th>时长</th><th>最近距离</th><th>均CTE</th><th>均航向误差</th><th>均速</th><th>GUIDED%</th>
      </tr></thead><tbody>${{rows}}</tbody></table></div>`;
  }});
  el.innerHTML=html;
}}

// ═══ AI 分析 + 距离 + ORCA 图表 ═══
function buildSummaryCards() {{
  const active=getActive();
  const el=document.getElementById('ai-insights');
  if(!el) return;
  if(!active.length) {{ el.innerHTML=''; return; }}
  let html='<div class="reading-guide" style="margin-bottom:14px">';
  html+='<strong>🤖 AI 分析摘要</strong><br>';
  active.forEach(u=>{{
    const sum=ALL_SUM[u.idx]; if(!sum) return;
    const sc=sum.scoring;
    let ins=[];
    if(sum.cte&&sum.cte.avg!==null) {{
      if(sum.cte.avg<0.1) ins.push('✅CTE优秀('+sum.cte.avg.toFixed(3)+'m)');
      else if(sum.cte.avg<0.3) ins.push('CTE良好('+sum.cte.avg.toFixed(3)+'m)');
      else ins.push('⚠️CTE偏高('+sum.cte.avg.toFixed(3)+'m)');
    }}
    if(sum.heading_error&&sum.heading_error.avg!==null) {{
      if(sum.heading_error.avg>20) ins.push('⚠️航向误差大('+sum.heading_error.avg.toFixed(1)+'°)');
      else if(sum.heading_error.avg>10) ins.push('航向误差一般('+sum.heading_error.avg.toFixed(1)+'°)');
      else ins.push('✅航向控制良好('+sum.heading_error.avg.toFixed(1)+'°)');
    }}
    if(sum.mpc&&sum.mpc.avg) {{
      if(sum.mpc.avg>50) ins.push('❌MPC过慢('+sum.mpc.avg.toFixed(1)+'ms)');
      else if(sum.mpc.avg>30) ins.push('⚠️MPC偏高('+sum.mpc.avg.toFixed(1)+'ms)');
      else ins.push('MPC正常('+sum.mpc.avg.toFixed(1)+'ms)');
    }}
    if(sum.orca&&sum.orca.active_count>0) {{
      ins.push('ORCA触发'+sum.orca.active_count+'次('+sum.orca.active_pct.toFixed(1)+'%)');
      if(sum.orca.hard_brake_count>0) ins.push('⚠️硬刹车'+sum.orca.hard_brake_count+'次');
      if(sum.orca.min_dist!==null&&sum.orca.min_dist<0.5) ins.push('❌最近距离仅'+sum.orca.min_dist.toFixed(2)+'m');
      if(sum.orca.top_primary_neighbor) ins.push('主导邻船'+sum.orca.top_primary_neighbor);
      if(sum.orca.escape_episode_count>0) ins.push('🆘脱困'+sum.orca.escape_episode_count+'次/'+sum.orca.escape_total_duration.toFixed(1)+'s');
    }}
    if(sum.wifi&&sum.wifi.avg) ins.push('WiFi '+sum.wifi.grade+'('+sum.wifi.avg.toFixed(0)+'dBm)');
    if(sum.velocity) ins.push('均速'+sum.velocity.guided_avg.toFixed(3)+'m/s');
    if(sum.ampc&&sum.ampc.tau_final) ins.push('τ→'+sum.ampc.tau_final.toFixed(3)+'s');
    if(sum.osc_freq&&sum.osc_freq>0.3) ins.push('⚠️ω振荡'+sum.osc_freq.toFixed(2)+'Hz');
    if(sum.distance&&sum.distance.min!==null) ins.push('最近达标距'+sum.distance.min.toFixed(2)+'m'+(sum.distance.min<=1.5?'✅':'❌'));
    html+='<span style="color:'+u.color+';font-weight:700">● '+u.id+'</span> ['+sc.grade+' '+sc.total.toFixed(0)+'分] '+ins.join(' | ')+'<br>';
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

function drawOrca() {{
  const active=getActive();
  const hasOrca=active.some(u=>ALL_TS[u.idx]&&Array.isArray(ALL_TS[u.idx].orca_active));
  const hasEscape=active.some(u=>ALL_TS[u.idx]&&Array.isArray(ALL_TS[u.idx].orca_escape_active));
  ['chart-orca-box','chart-orca-dist-box','chart-orca-corr-box'].forEach(id=>{{
    const el=document.getElementById(id); if(el) el.style.display=hasOrca?'block':'none';
  }});
  const escapeBox=document.getElementById('chart-orca-escape-box');
  if(escapeBox) escapeBox.style.display=hasEscape?'block':'none';
  if(!hasOrca && !hasEscape) return;
  const t1=[],t2=[],t3=[],t4=[];
  active.forEach(u=>{{
    const ts=ALL_TS[u.idx]; if(!ts||!ts.orca_active) return;
    t1.push({{x:ts.time,y:ts.orca_active,name:u.id+' 激活',line:{{color:u.color,width:1.5}},fill:'tozeroy',fillcolor:u.color+'20'}});
    const vd=ts.orca_closest.map(v=>v>0?v:null);
    t2.push({{x:ts.time,y:vd,name:u.id,line:{{color:u.color,width:1.5}},connectgaps:false}});
    if(ts.orca_linear_corr) {{
      t3.push({{x:ts.time,y:ts.orca_linear_corr,name:u.id+' Δv',line:{{color:u.color,width:1}}}});
      if(ts.orca_angular_corr) t3.push({{x:ts.time,y:ts.orca_angular_corr,name:u.id+' Δω',line:{{color:u.color,width:1,dash:'dash'}},showlegend:false}});
    }}
    if(Array.isArray(ts.orca_escape_active)) {{
      const phase=(ts.orca_escape_phase||[]).map((value,index)=>ts.orca_escape_active[index]===1?value:0);
      t4.push({{x:ts.time,y:phase,name:u.id+' 脱困阶段',line:{{color:u.color,width:1.8,shape:'hv'}}}});
    }}
  }});
  if(hasOrca) {{
    Plotly.react('chart-orca-activation',t1.length?t1:emptyTrace(),{{
      ...darkLayout,title:'ORCA 避障激活',
      xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Active',range:[-0.1,1.3]}},
    }},cfg);
    Plotly.react('chart-orca-dist',t2.length?t2:emptyTrace(),{{
      ...darkLayout,title:'最近邻USV距离',
      xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Distance (m)'}},
      shapes:[{{type:'line',x0:0,x1:1,xref:'paper',y0:1.0,y1:1.0,line:{{color:'#EF5350',width:1.5,dash:'dash'}}}}],
    }},cfg);
    Plotly.react('chart-orca-corrections',t3.length?t3:emptyTrace(),{{
      ...darkLayout,title:'ORCA 速度修正量',
      xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},yaxis:{{...darkLayout.yaxis,title:'Correction'}},
    }},cfg);
  }}
  if(hasEscape) {{
    Plotly.react('chart-orca-escape',t4.length?t4:emptyTrace(),{{
      ...darkLayout,title:'ORCA 脱困阶段 (v17)',
      xaxis:{{...darkLayout.xaxis,title:'Time (s)'}},
      yaxis:{{...darkLayout.yaxis,title:'Escape Phase',tickmode:'array',tickvals:[0,1,2],ticktext:['Off','P1','P2'],range:[-0.2,2.3]}},
    }},cfg);
  }}
}}

// ═══ 路径重放引擎 ═══
let replayState = {{
  playing:false, speed:1.0, currentTime:0, maxTime:0,
  lastFrame:0, animId:null, goalReachedFlags:{{}}, goalReachedTime:{{}},
  _prevGoals:{{}}, showGoalLabels:false
}};

// Catmull-Rom 样条插值: 四点 p0,p1,p2,p3, 参数 t∈[0,1]
function catmull(p0,p1,p2,p3,t) {{
  const t2=t*t, t3=t2*t;
  return 0.5*(2*p1+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t2+(-p0+3*p1-3*p2+p3)*t3);
}}

// GS导航预览箭头: 0.6m×0.35m, 归一化后 width/length=0.583
const BOAT_VERTS = [[0.5,0],[-0.5,0.292],[-0.167,0],[-0.5,-0.292]];
const BOAT_SCALE = 22;

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

function updateReplayInfo() {{
  const el=document.getElementById('replay-info');
  el.innerHTML=getActive().map(u=>
    `<span class="ri-item"><span class="ri-dot" style="background:${{u.color}}"></span>${{u.id}}</span>`
  ).join('')||'<span style="color:var(--text2)">请选择USV</span>';
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
function toggleGoalLabels() {{ replayState.showGoalLabels=!replayState.showGoalLabels; const btn=document.getElementById('btn-goal-labels'); btn.style.background=replayState.showGoalLabels?'rgba(100,255,218,0.15)':'rgba(255,255,255,0.08)'; btn.style.color=replayState.showGoalLabels?'#64ffda':'#9e9e9e'; renderReplayFrame(); }}
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
    // 单任务USV: 全局渐隐已完成则不纳入视口
    if(!hasOwnFade&&globalFadeTime!==null&&globalFadeTime!==undefined&&t-globalFadeTime>=fadeDur) return;
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
  // v16: 将邻居观测位置纳入视口 (已结束USV可能漂移到新位置)
  data.forEach(dSelf=>{{
    const selfEndTime=dSelf.ts.time[dSelf.ts.time.length-1];
    if(t<=selfEndTime) return;
    const selfId=dSelf.usv.id;
    data.forEach(dOther=>{{
      if(dOther.usv.idx===dSelf.usv.idx) return;
      const nbs=dOther.ts.neighbors;
      if(!nbs||!nbs[selfId]) return;
      const pos=sampleNeighborPos(nbs[selfId],t);
      if(pos) {{
        if(pos.x<minX)minX=pos.x; if(pos.x>maxX)maxX=pos.x;
        if(pos.y<minY)minY=pos.y; if(pos.y>maxY)maxY=pos.y;
      }}
    }});
  }});

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
    let baseA=1.0;
    if(!hasOwnTransition&&globalFadeTime!==null&&globalFadeTime!==undefined) {{
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

    // 已走轨迹 — 只显示当前任务段 (baseA=单任务渐隐, fadeInA=新任务淡入)
    const curA=baseA*fadeInA;
    ctx.beginPath(); ctx.strokeStyle=color; ctx.lineWidth=2; ctx.globalAlpha=0.6*curA;
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
    drawBoat(ctx,canX,canY,yaw,color,BOAT_SCALE);

    // 标签
    ctx.fillStyle=color; ctx.font='bold 11px Inter,system-ui,sans-serif';
    ctx.textAlign='center'; ctx.fillText(d.usv.id,canX,canY-BOAT_SCALE-4);

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

  // v16: 已结束的USV通过邻居观测数据继续显示位置
  // 当一个USV自身数据结束后，从其他仍在记录的USV的邻居数据中获取其位置
  data.forEach(dSelf=>{{
    const ts=dSelf.ts;
    const selfEndTime=ts.time[ts.time.length-1];
    if(t<=selfEndTime) return; // 自身数据未结束，已在上面绘制
    const selfId=dSelf.usv.id;
    // 在所有其他USV的邻居数据中查找此USV
    let bestPos=null;
    data.forEach(dOther=>{{
      if(dOther.usv.idx===dSelf.usv.idx) return;
      const nbs=dOther.ts.neighbors;
      if(!nbs||!nbs[selfId]) return;
      const pos=sampleNeighborPos(nbs[selfId],t);
      if(pos) bestPos=pos;
    }});
    if(bestPos) {{
      const[canX,canY]=toC(bestPos.x,bestPos.y);
      ctx.globalAlpha=0.55;
      drawBoat(ctx,canX,canY,bestPos.yaw,dSelf.usv.color,BOAT_SCALE);
      ctx.globalAlpha=0.55;
      ctx.fillStyle=dSelf.usv.color; ctx.font='bold 11px Inter,system-ui,sans-serif';
      ctx.textAlign='center'; ctx.fillText(dSelf.usv.id+'(idle)',canX,canY-BOAT_SCALE-4);
      ctx.globalAlpha=1.0;
    }}
  }});

  // 时间文字
  ctx.fillStyle='rgba(100,255,218,0.6)';
  ctx.font='12px Inter,system-ui,sans-serif';
  ctx.textAlign='left';
  ctx.fillText('t = '+t.toFixed(1)+'s  |  '+replayState.speed+'x',10,20);
}}

// ═══ 初始化 ═══
function init() {{
  buildSelector(); refreshAll(); renderReplayFrame();

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
