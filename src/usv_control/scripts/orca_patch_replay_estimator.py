#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于 nav_log 的 ORCA 补丁近似回放评估器。

用途:
1. 不依赖 rosbag，仅用历史 nav_log CSV 估算新 ORCA 补丁会更早在哪些时刻介入。
2. 对比旧日志中的硬刹车次数、最小距离、选边切换次数，与“新策略近似回放”下的
   选边锁定效果和提前脱困时机。
3. 给出“潜在可减少的硬刹车样本上界”，帮助判断修改方向是否值得继续实测验证。

重要限制:
- 该脚本不会重新求解完整控制器，也不会生成真实的新 cmd_vx/cmd_omega。
- 它只对新增的三类策略做近似回放:
  1) 近距选边锁定
  2) 快速逼近提前升级
  3) 持续近距耦合提前脱困
- 因此它适合做“方向是否对”的离线判断，不替代真实复测。

示例:
  /bin/python3 usv_control/scripts/orca_patch_replay_estimator.py --preset v17-problematic

  /bin/python3 usv_control/scripts/orca_patch_replay_estimator.py \
    --log ~/usv_logs/V17/usv_02/nav_log_usv_02_20260310_162843_goal_200001.csv \
    --log ~/usv_logs/V17/usv_03/nav_log_usv_03_20260310_162843_goal_300001.csv
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import analyze_nav_log as anl


@dataclass
class EarlyClosingEvent:
    time_s: float
    distance_m: float
    closing_speed: float
    neighbor_id: str


@dataclass
class PersistentCloseEscape:
    trigger_time_s: float
    distance_m: float
    neighbor_id: str


@dataclass
class ReplaySummary:
    log_name: str
    task_name: str
    duration_s: float
    old_min_distance_m: float | None
    old_min_distance_time_s: float | None
    old_hard_brake_count: int
    old_first_hard_brake_time_s: float | None
    old_first_hard_brake_distance_m: float | None
    old_commit_changes: int
    replay_commit_changes: int
    first_early_closing: EarlyClosingEvent | None
    persistent_escape: PersistentCloseEscape | None
    persistent_escape_companion_exemptions: int
    reducible_hard_brake_upper_bound: int
    reducible_hard_brake_ratio: float


def _valid_neighbor_id(value: Any) -> str:
    if isinstance(value, str):
        value = value.strip()
        if value and value != '0':
            return value
    return ''


def _to_float(value: Any) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return None
        try:
            return float(text)
        except ValueError:
            return None
    return None


def _to_int(value: Any) -> int | None:
    if isinstance(value, (int, float)):
        return int(value)
    return None


def _relative_time(row: dict[str, Any], t0: float) -> float:
    return float(row['timestamp']) - t0


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _neighbor_state(row: dict[str, Any], neighbor_id: str) -> tuple[float, float, float, float] | None:
    if not neighbor_id:
        return None

    for index in range(1, 6):
        row_neighbor_id = row.get(f'neighbor_{index}_id')
        if isinstance(row_neighbor_id, str) and row_neighbor_id.strip() == neighbor_id:
            nx = _to_float(row.get(f'neighbor_{index}_x'))
            ny = _to_float(row.get(f'neighbor_{index}_y'))
            nvx = _to_float(row.get(f'neighbor_{index}_vx'))
            nvy = _to_float(row.get(f'neighbor_{index}_vy'))
            if None not in (nx, ny, nvx, nvy):
                assert nx is not None and ny is not None and nvx is not None and nvy is not None
                return nx, ny, nvx, nvy
    return None


def _own_velocity(row: dict[str, Any]) -> tuple[float, float] | None:
    vx = _to_float(row.get('velocity_vx'))
    vy = _to_float(row.get('velocity_vy'))
    if vx is not None and vy is not None:
        return vx, vy

    speed = _to_float(row.get('velocity_speed'))
    yaw_deg = _to_float(row.get('velocity_yaw_deg'))
    if speed is None or yaw_deg is None:
        return None

    yaw = math.radians(yaw_deg)
    return speed * math.cos(yaw), speed * math.sin(yaw)


def _path_heading(row: dict[str, Any]) -> float | None:
    path_theta_deg = _to_float(row.get('path_theta_deg'))
    if path_theta_deg is not None:
        return math.radians(path_theta_deg)

    pose_x = _to_float(row.get('pose_x'))
    pose_y = _to_float(row.get('pose_y'))
    target_x = _to_float(row.get('target_x'))
    target_y = _to_float(row.get('target_y'))
    if None in (pose_x, pose_y, target_x, target_y):
        return None

    assert pose_x is not None and pose_y is not None and target_x is not None and target_y is not None

    dx = target_x - pose_x
    dy = target_y - pose_y
    if math.hypot(dx, dy) < 1e-3:
        return None
    return math.atan2(dy, dx)


def _is_path_companion_row(
    row: dict[str, Any],
    neighbor_x: float,
    neighbor_y: float,
    neighbor_vx: float,
    neighbor_vy: float,
    distance: float,
) -> bool:
    pose_x = _to_float(row.get('pose_x'))
    pose_y = _to_float(row.get('pose_y'))
    target_x = _to_float(row.get('target_x'))
    target_y = _to_float(row.get('target_y'))
    own_velocity = _own_velocity(row)
    path_theta = _path_heading(row)
    if None in (pose_x, pose_y, target_x, target_y) or own_velocity is None or path_theta is None:
        return False

    assert pose_x is not None and pose_y is not None and target_x is not None and target_y is not None

    own_vx, own_vy = own_velocity
    own_goal_dist = math.hypot(target_x - pose_x, target_y - pose_y)
    if own_goal_dist > max(4.0, 2.5 * distance):
        return False

    own_goal_heading = math.atan2(target_y - pose_y, target_x - pose_x)
    own_speed = math.hypot(own_vx, own_vy)
    neighbor_speed = math.hypot(neighbor_vx, neighbor_vy)
    rel_speed = math.hypot(own_vx - neighbor_vx, own_vy - neighbor_vy)
    if rel_speed > 0.18:
        return False

    own_course = math.atan2(own_vy, own_vx) if own_speed > 0.12 else math.radians(_to_float(row.get('pose_yaw_deg')) or 0.0)
    neighbor_course = math.atan2(neighbor_vy, neighbor_vx) if neighbor_speed > 0.12 else None
    neighbor_goal_heading = math.atan2(target_y - neighbor_y, target_x - neighbor_x)

    if abs(_wrap_angle(own_goal_heading - path_theta)) > math.radians(35.0):
        return False
    if abs(_wrap_angle(own_course - path_theta)) > math.radians(70.0):
        return False
    if abs(_wrap_angle(neighbor_goal_heading - path_theta)) > math.radians(35.0):
        return False
    if neighbor_course is not None and abs(_wrap_angle(neighbor_course - path_theta)) > math.radians(35.0):
        return False

    ux = math.cos(path_theta)
    uy = math.sin(path_theta)
    rel_x = neighbor_x - pose_x
    rel_y = neighbor_y - pose_y
    along_gap = rel_x * ux + rel_y * uy
    lateral_gap = abs(-uy * rel_x + ux * rel_y)

    if lateral_gap > max(1.0, 0.8 * distance):
        return False
    if along_gap < -1.0:
        return False

    neighbor_goal_dist = math.hypot(target_x - neighbor_x, target_y - neighbor_y)
    if neighbor_goal_dist > own_goal_dist + max(1.0, 0.8 * distance):
        return False

    tcpa = _to_float(row.get('orca_tcpa'))
    dcpa = _to_float(row.get('orca_dcpa'))
    if tcpa is not None and dcpa is not None and 0.0 <= tcpa < 3.0 and dcpa < max(0.5, 0.9 * distance):
        return False

    return True


def _is_goal_companion_fallback_row(row: dict[str, Any], distance: float) -> bool:
    pose_x = _to_float(row.get('pose_x'))
    pose_y = _to_float(row.get('pose_y'))
    target_x = _to_float(row.get('target_x'))
    target_y = _to_float(row.get('target_y'))
    path_theta = _path_heading(row)
    rel_speed = _to_float(row.get('orca_rel_speed'))
    rel_bearing_deg = _to_float(row.get('orca_rel_bearing_deg'))
    tcpa = _to_float(row.get('orca_tcpa'))
    dcpa = _to_float(row.get('orca_dcpa'))
    if None in (pose_x, pose_y, target_x, target_y, path_theta, rel_speed, rel_bearing_deg):
        return False

    assert pose_x is not None and pose_y is not None and target_x is not None and target_y is not None and path_theta is not None
    assert rel_speed is not None and rel_bearing_deg is not None

    own_goal_dist = math.hypot(target_x - pose_x, target_y - pose_y)
    if own_goal_dist > max(4.0, 2.5 * distance):
        return False

    own_goal_heading = math.atan2(target_y - pose_y, target_x - pose_x)
    if abs(_wrap_angle(own_goal_heading - path_theta)) > math.radians(35.0):
        return False

    if rel_speed > 0.18:
        return False

    if abs(rel_bearing_deg) < 70.0:
        return False

    if tcpa is not None and dcpa is not None and 0.0 <= tcpa < 3.0 and dcpa < max(0.5, 0.9 * distance):
        return False

    return True


def _count_nonzero_side_changes(values: list[int]) -> int:
    filtered = [value for value in values if value != 0]
    return sum(1 for index in range(1, len(filtered)) if filtered[index] != filtered[index - 1])


def _simulate_close_commit_lock(
    data: list[dict[str, Any]],
    base_hold_time: float = 2.5,
    close_lock_distance: float = 3.0,
    close_hold_time: float = 6.0,
) -> list[int]:
    simulated: list[int] = []
    committed_side = 0
    commit_deadline = 0.0
    commit_neighbor_id = ''

    for row in data:
        ts = float(row['timestamp'])
        proposed_side = _to_int(row.get('orca_commit_side')) or 0
        neighbor_id = _valid_neighbor_id(row.get('orca_primary_neighbor_id'))
        distance = _to_float(row.get('orca_closest_distance'))
        within_close_lock = bool(neighbor_id and distance is not None and 0.0 < distance <= close_lock_distance)

        if proposed_side == 0:
            if committed_side != 0 and within_close_lock and neighbor_id == commit_neighbor_id:
                current_side = committed_side
                commit_deadline = max(commit_deadline, ts + close_hold_time)
            elif committed_side != 0 and ts < commit_deadline:
                current_side = committed_side
            else:
                if ts >= commit_deadline:
                    committed_side = 0
                    commit_neighbor_id = ''
                current_side = 0
        else:
            if committed_side != 0 and within_close_lock and neighbor_id == commit_neighbor_id:
                current_side = committed_side
                commit_deadline = max(commit_deadline, ts + close_hold_time)
            elif committed_side != 0 and ts < commit_deadline and neighbor_id == commit_neighbor_id:
                current_side = committed_side
            else:
                current_side = proposed_side
                committed_side = current_side
                commit_neighbor_id = neighbor_id
                hold_time = close_hold_time if within_close_lock else base_hold_time
                commit_deadline = ts + hold_time

        simulated.append(current_side)

    return simulated


def _detect_early_closing(
    data: list[dict[str, Any]],
    t0: float,
    close_distance: float = 3.5,
    closing_speed_threshold: float = 0.18,
) -> list[EarlyClosingEvent]:
    events: list[EarlyClosingEvent] = []
    active_window = False

    for row in data:
        distance = _to_float(row.get('orca_closest_distance'))
        neighbor_id = _valid_neighbor_id(row.get('orca_primary_neighbor_id'))
        if distance is None or distance <= 0.0 or distance > close_distance or not neighbor_id:
            active_window = False
            continue

        own_velocity = _own_velocity(row)
        neighbor_state = _neighbor_state(row, neighbor_id)
        pose_x = _to_float(row.get('pose_x'))
        pose_y = _to_float(row.get('pose_y'))
        if own_velocity is None or neighbor_state is None or pose_x is None or pose_y is None:
            active_window = False
            continue

        neighbor_x, neighbor_y, neighbor_vx, neighbor_vy = neighbor_state
        own_vx, own_vy = own_velocity
        rx = pose_x - neighbor_x
        ry = pose_y - neighbor_y
        rvx = own_vx - neighbor_vx
        rvy = own_vy - neighbor_vy
        radial_rate = (rx * rvx + ry * rvy) / max(distance, 1e-6)
        closing_speed = max(0.0, -radial_rate)

        if closing_speed >= closing_speed_threshold:
            if not active_window:
                events.append(
                    EarlyClosingEvent(
                        time_s=_relative_time(row, t0),
                        distance_m=distance,
                        closing_speed=closing_speed,
                        neighbor_id=neighbor_id,
                    )
                )
                active_window = True
        else:
            active_window = False

    return events


def _detect_persistent_close_escape(
    data: list[dict[str, Any]],
    t0: float,
    distance_threshold: float = 1.8,
    timeout_s: float = 30.0,
) -> tuple[PersistentCloseEscape | None, int]:
    start_time = 0.0
    active_neighbor = ''
    companion_exemptions = 0

    for row in data:
        ts = float(row['timestamp'])
        neighbor_id = _valid_neighbor_id(row.get('orca_primary_neighbor_id'))
        distance = _to_float(row.get('orca_closest_distance'))

        if neighbor_id and distance is not None and 0.0 < distance <= distance_threshold:
            neighbor_state = _neighbor_state(row, neighbor_id)
            companion_context = False
            if neighbor_state is not None:
                companion_context = _is_path_companion_row(row, *neighbor_state, distance)
            if not companion_context:
                companion_context = _is_goal_companion_fallback_row(row, distance)

            if companion_context:
                start_time = 0.0
                active_neighbor = ''
                companion_exemptions += 1
                continue
            if start_time == 0.0 or neighbor_id != active_neighbor:
                start_time = ts
                active_neighbor = neighbor_id
            elif ts - start_time >= timeout_s:
                return (
                    PersistentCloseEscape(
                        trigger_time_s=ts - t0,
                        distance_m=distance,
                        neighbor_id=neighbor_id,
                    ),
                    companion_exemptions,
                )
        else:
            start_time = 0.0
            active_neighbor = ''

    return None, companion_exemptions


def analyze_log(
    log_path: Path,
    base_hold_time: float = 2.5,
    close_lock_distance: float = 3.0,
    close_hold_time: float = 6.0,
    close_distance: float = 3.5,
    closing_speed_threshold: float = 0.18,
    persistent_close_distance: float = 1.8,
    persistent_close_timeout: float = 30.0,
) -> ReplaySummary:
    data, header = anl.load_csv(str(log_path))
    if not data:
        raise ValueError(f'空日志: {log_path}')

    t0 = float(data[0]['timestamp'])
    old_commit_values = [(_to_int(row.get('orca_commit_side')) or 0) for row in data]
    replay_commit_values = _simulate_close_commit_lock(
        data,
        base_hold_time=base_hold_time,
        close_lock_distance=close_lock_distance,
        close_hold_time=close_hold_time,
    )

    hard_brake_rows = [row for row in data if (_to_int(row.get('orca_hard_brake')) or 0) == 1]
    old_first_hb = hard_brake_rows[0] if hard_brake_rows else None

    distance_samples: list[tuple[float, float]] = []
    for row in data:
        distance_value = _to_float(row.get('orca_closest_distance'))
        if distance_value is not None and distance_value > 0.0:
            distance_samples.append((_relative_time(row, t0), distance_value))

    if distance_samples:
        old_min_distance_time_s, old_min_distance_m = min(distance_samples, key=lambda item: item[1])
    else:
        old_min_distance_time_s, old_min_distance_m = None, None

    early_events = _detect_early_closing(
        data,
        t0=t0,
        close_distance=close_distance,
        closing_speed_threshold=closing_speed_threshold,
    )
    persistent_escape, companion_exemptions = _detect_persistent_close_escape(
        data,
        t0=t0,
        distance_threshold=persistent_close_distance,
        timeout_s=persistent_close_timeout,
    )

    reducible_upper_bound = 0
    reducible_ratio = 0.0
    if persistent_escape is not None and hard_brake_rows:
        reducible_upper_bound = sum(
            1
            for row in hard_brake_rows
            if _relative_time(row, t0) >= persistent_escape.trigger_time_s
        )
        reducible_ratio = reducible_upper_bound / max(1, len(hard_brake_rows))

    return ReplaySummary(
        log_name=log_path.name,
        task_name=header.get('task_name', ''),
        duration_s=float(data[-1]['timestamp']) - t0,
        old_min_distance_m=old_min_distance_m,
        old_min_distance_time_s=old_min_distance_time_s,
        old_hard_brake_count=len(hard_brake_rows),
        old_first_hard_brake_time_s=_relative_time(old_first_hb, t0) if old_first_hb is not None else None,
        old_first_hard_brake_distance_m=_to_float(old_first_hb.get('orca_closest_distance')) if old_first_hb is not None else None,
        old_commit_changes=_count_nonzero_side_changes(old_commit_values),
        replay_commit_changes=_count_nonzero_side_changes(replay_commit_values),
        first_early_closing=early_events[0] if early_events else None,
        persistent_escape=persistent_escape,
        persistent_escape_companion_exemptions=companion_exemptions,
        reducible_hard_brake_upper_bound=reducible_upper_bound,
        reducible_hard_brake_ratio=reducible_ratio,
    )


def _format_num(value: float | None, digits: int = 2) -> str:
    if value is None:
        return 'NA'
    return f'{value:.{digits}f}'


def _default_problematic_logs() -> list[Path]:
    root = Path.home() / 'usv_logs' / 'V17'
    return [
        root / 'usv_02' / 'nav_log_usv_02_20260310_162843_goal_200001.csv',
        root / 'usv_03' / 'nav_log_usv_03_20260310_162843_goal_300001.csv',
        root / 'usv_02' / 'nav_log_usv_02_20260310_165358_Z字形-8mx24m-白天.csv',
        root / 'usv_03' / 'nav_log_usv_03_20260310_165358_Z字形-8mx24m-白天.csv',
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='ORCA 补丁近似回放评估器')
    parser.add_argument('--log', action='append', default=[], help='待评估的 nav_log CSV，可重复传入')
    parser.add_argument(
        '--preset',
        choices=['v17-problematic'],
        help='使用内置问题日志集合。当前支持: v17-problematic',
    )
    parser.add_argument('--base-hold-time', type=float, default=2.5)
    parser.add_argument('--close-lock-distance', type=float, default=3.0)
    parser.add_argument('--close-hold-time', type=float, default=6.0)
    parser.add_argument('--close-distance', type=float, default=3.5)
    parser.add_argument('--closing-speed-threshold', type=float, default=0.18)
    parser.add_argument('--persistent-close-distance', type=float, default=1.8)
    parser.add_argument('--persistent-close-timeout', type=float, default=30.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    log_paths: list[Path] = [Path(path).expanduser() for path in args.log]
    if args.preset == 'v17-problematic':
        log_paths.extend(_default_problematic_logs())

    unique_paths: list[Path] = []
    seen: set[Path] = set()
    for path in log_paths:
        if path not in seen:
            unique_paths.append(path)
            seen.add(path)

    if not unique_paths:
        raise SystemExit('未提供日志。可使用 --preset v17-problematic 或 --log <file>.')

    summaries: list[ReplaySummary] = []
    for path in unique_paths:
        if not path.exists():
            print(f'⚠️ 跳过不存在的日志: {path}')
            continue
        summaries.append(
            analyze_log(
                path,
                base_hold_time=args.base_hold_time,
                close_lock_distance=args.close_lock_distance,
                close_hold_time=args.close_hold_time,
                close_distance=args.close_distance,
                closing_speed_threshold=args.closing_speed_threshold,
                persistent_close_distance=args.persistent_close_distance,
                persistent_close_timeout=args.persistent_close_timeout,
            )
        )

    if not summaries:
        raise SystemExit('没有可分析的日志。')

    print('=' * 96)
    print('ORCA 补丁近似回放评估')
    print('=' * 96)
    print('说明: 这是 CSV 近似回放，不是完整控制器重算。硬刹车减少量仅为“潜在上界”。')

    for summary in summaries:
        print('-' * 96)
        print(f'日志: {summary.log_name}')
        if summary.task_name:
            print(f'任务: {summary.task_name}')
        print(
            f'旧指标: duration={summary.duration_s:.1f}s, '
            f'min_dist={_format_num(summary.old_min_distance_m)}m@{_format_num(summary.old_min_distance_time_s, 1)}s, '
            f'hard_brake={summary.old_hard_brake_count}, '
            f'commit_changes={summary.old_commit_changes}'
        )
        if summary.old_first_hard_brake_time_s is not None:
            print(
                f'旧首个硬刹车: t={summary.old_first_hard_brake_time_s:.1f}s, '
                f'd={_format_num(summary.old_first_hard_brake_distance_m)}m'
            )
        else:
            print('旧首个硬刹车: none')

        print(f'近距锁定回放: commit_changes {summary.old_commit_changes} -> {summary.replay_commit_changes}')

        if summary.first_early_closing is not None:
            lead_to_hb = None
            if summary.old_first_hard_brake_time_s is not None:
                lead_to_hb = summary.old_first_hard_brake_time_s - summary.first_early_closing.time_s
            lead_to_min = None
            if summary.old_min_distance_time_s is not None:
                lead_to_min = summary.old_min_distance_time_s - summary.first_early_closing.time_s
            print(
                f'快速逼近提前升级: 首次会在 t={summary.first_early_closing.time_s:.1f}s 介入, '
                f'd={summary.first_early_closing.distance_m:.2f}m, '
                f'closing={summary.first_early_closing.closing_speed:.2f}m/s, '
                f'到旧首个硬刹车还剩 {_format_num(lead_to_hb, 1)}s, '
                f'到旧最小距离还剩 {_format_num(lead_to_min, 1)}s'
            )
        else:
            print('快速逼近提前升级: 未触发')

        if summary.persistent_escape is not None:
            lead_to_hb = None
            if summary.old_first_hard_brake_time_s is not None:
                lead_to_hb = summary.old_first_hard_brake_time_s - summary.persistent_escape.trigger_time_s
            lead_to_min = None
            if summary.old_min_distance_time_s is not None:
                lead_to_min = summary.old_min_distance_time_s - summary.persistent_escape.trigger_time_s
            print(
                f'持续近距提前脱困: t={summary.persistent_escape.trigger_time_s:.1f}s, '
                f'd={summary.persistent_escape.distance_m:.2f}m, '
                f'neighbor={summary.persistent_escape.neighbor_id}, '
                f'到旧首个硬刹车还剩 {_format_num(lead_to_hb, 1)}s, '
                f'到旧最小距离还剩 {_format_num(lead_to_min, 1)}s'
            )
            print(
                f'潜在可减少硬刹车上界: {summary.reducible_hard_brake_upper_bound}/'
                f'{summary.old_hard_brake_count} ({summary.reducible_hard_brake_ratio * 100:.1f}%)'
            )
        else:
            print(
                f'持续近距提前脱困: 未触发'
                f' (伴行豁免样本={summary.persistent_escape_companion_exemptions})'
            )


if __name__ == '__main__':
    main()