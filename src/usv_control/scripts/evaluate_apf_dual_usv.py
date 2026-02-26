#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
APF 双船联调评估脚本（usv_02 + usv_03）

功能：
1) 计算两船最小间距（按时间最近邻对齐）
2) 评估两船到点成功率与首次到点时间
3) 评估振荡代理指标（角速度方向反转频率）
4) 输出参数调优建议

用法示例：
  python3 usv_control/scripts/evaluate_apf_dual_usv.py \
    --log-a /path/to/usv_02/nav_log_xxx.csv \
    --log-b /path/to/usv_03/nav_log_xxx.csv

  # 自动查找最新日志（按 usv id）
  python3 usv_control/scripts/evaluate_apf_dual_usv.py \
    --auto-find --usv-a usv_02 --usv-b usv_03
"""

from __future__ import annotations

import argparse
import bisect
import csv
import glob
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, cast


@dataclass
class Row:
    timestamp: float
    pose_x: Optional[float] = None
    pose_y: Optional[float] = None
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    distance_to_goal: Optional[float] = None
    cmd_omega: Optional[float] = None
    heading_error_deg: Optional[float] = None


@dataclass
class GoalMetrics:
    success: bool
    min_distance_to_goal: Optional[float]
    first_reach_time: Optional[float]


@dataclass
class OscillationMetrics:
    sign_changes: int
    sign_change_rate_hz: float
    heading_error_std_deg: Optional[float]


def _to_float(value) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if text == "":
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _find_latest_log(usv_id: str) -> Optional[str]:
    home_logs = os.path.expanduser(f"~/usv_logs/{usv_id}/nav_log_*.csv")
    home_logs_nested = os.path.expanduser(f"~/usv_logs/**/{usv_id}/nav_log_*.csv")
    ws_logs = os.path.expanduser(f"~/usv_workspace/src/**/{usv_id}/nav_log_*.csv")
    direct_ws = os.path.expanduser("~/usv_workspace/src/**/nav_log_*.csv")
    direct_home = os.path.expanduser("~/usv_logs/**/nav_log_*.csv")

    candidates = []
    candidates.extend(glob.glob(home_logs))
    candidates.extend(glob.glob(home_logs_nested, recursive=True))
    candidates.extend(glob.glob(ws_logs, recursive=True))

    # 兜底：如果日志没有按 usv_id 分目录，则从全局按文件名过滤
    for path in glob.glob(direct_ws, recursive=True):
        lower = os.path.basename(path).lower()
        if usv_id.lower() in lower:
            candidates.append(path)

    for path in glob.glob(direct_home, recursive=True):
        lower = os.path.basename(path).lower()
        if usv_id.lower() in lower:
            candidates.append(path)

    if not candidates:
        return None
    candidates = sorted(set(candidates), key=os.path.getmtime)
    return candidates[-1]


def load_log(path: str) -> List[Row]:
    rows: List[Row] = []
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as handle:
        # 跳过注释头
        lines = handle.readlines()
    data_start = 0
    for index, line in enumerate(lines):
        text = line.strip()
        if text and not text.startswith("#"):
            data_start = index
            break

    reader = csv.DictReader(lines[data_start:])
    for raw in reader:
        timestamp = _to_float(raw.get("timestamp"))
        if timestamp is None:
            continue

        rows.append(
            Row(
                timestamp=timestamp,
                pose_x=_to_float(raw.get("pose_x")),
                pose_y=_to_float(raw.get("pose_y")),
                target_x=_to_float(raw.get("target_x")),
                target_y=_to_float(raw.get("target_y")),
                distance_to_goal=_to_float(raw.get("distance_to_goal")),
                cmd_omega=_to_float(raw.get("cmd_omega")),
                heading_error_deg=_to_float(raw.get("heading_error_deg")),
            )
        )

    rows.sort(key=lambda item: item.timestamp)
    return rows


def _distance_to_goal(row: Row) -> Optional[float]:
    if row.distance_to_goal is not None:
        return row.distance_to_goal
    if None not in (row.pose_x, row.pose_y, row.target_x, row.target_y):
        pose_x = row.pose_x
        pose_y = row.pose_y
        target_x = row.target_x
        target_y = row.target_y
        if None in (pose_x, pose_y, target_x, target_y):
            return None
        dx = cast(float, target_x) - cast(float, pose_x)
        dy = cast(float, target_y) - cast(float, pose_y)
        return math.hypot(dx, dy)
    return None


def evaluate_goal(rows: List[Row], threshold: float) -> GoalMetrics:
    distances: List[Tuple[float, float]] = []
    for row in rows:
        distance = _distance_to_goal(row)
        if distance is not None:
            distances.append((row.timestamp, distance))

    if not distances:
        return GoalMetrics(False, None, None)

    min_distance = min(item[1] for item in distances)
    t0 = distances[0][0]
    first_reach_time = None
    for ts, distance in distances:
        if distance <= threshold:
            first_reach_time = ts - t0
            break

    success = first_reach_time is not None
    return GoalMetrics(success, min_distance, first_reach_time)


def evaluate_oscillation(rows: List[Row], omega_epsilon: float) -> OscillationMetrics:
    # 角速度方向反转统计（忽略接近零的抖动噪声）
    filtered: List[Tuple[float, int]] = []
    for row in rows:
        omega = row.cmd_omega
        if omega is None or abs(omega) < omega_epsilon:
            continue
        sign = 1 if omega > 0 else -1
        filtered.append((row.timestamp, sign))

    sign_changes = 0
    for idx in range(1, len(filtered)):
        if filtered[idx][1] != filtered[idx - 1][1]:
            sign_changes += 1

    if len(filtered) >= 2:
        duration = max(1e-9, filtered[-1][0] - filtered[0][0])
        sign_change_rate_hz = sign_changes / duration
    else:
        sign_change_rate_hz = 0.0

    heading_values = [r.heading_error_deg for r in rows if r.heading_error_deg is not None]
    heading_std = None
    if heading_values:
        mean_val = sum(heading_values) / len(heading_values)
        var = sum((x - mean_val) ** 2 for x in heading_values) / len(heading_values)
        heading_std = math.sqrt(var)

    return OscillationMetrics(sign_changes, sign_change_rate_hz, heading_std)


def evaluate_min_intervessel_distance(rows_a: List[Row], rows_b: List[Row]) -> Optional[float]:
    samples_b = [(r.timestamp, r.pose_x, r.pose_y) for r in rows_b if r.pose_x is not None and r.pose_y is not None]
    if not samples_b:
        return None

    times_b = [item[0] for item in samples_b]
    min_distance = None

    for row_a in rows_a:
        if row_a.pose_x is None or row_a.pose_y is None:
            continue

        index = bisect.bisect_left(times_b, row_a.timestamp)
        candidate_indices = []
        if index < len(samples_b):
            candidate_indices.append(index)
        if index - 1 >= 0:
            candidate_indices.append(index - 1)

        for idx in candidate_indices:
            _, bx, by = samples_b[idx]
            distance = math.hypot(float(row_a.pose_x) - float(bx), float(row_a.pose_y) - float(by))
            if min_distance is None or distance < min_distance:
                min_distance = distance

    return min_distance


def evaluate_separation_timeliness(
    rows_a: List[Row],
    rows_b: List[Row],
    enter_threshold: float = 1.0,
    recover_thresholds: Tuple[float, float] = (1.2, 1.5),
) -> Dict[str, Optional[float]]:
    """评估进入近距后恢复到更安全距离的时效性。"""
    samples_b = [(r.timestamp, r.pose_x, r.pose_y) for r in rows_b if r.pose_x is not None and r.pose_y is not None]
    if not samples_b:
        return {
            "enter_time": None,
            "recover_to_1p2_s": None,
            "recover_to_1p5_s": None,
        }

    times_b = [item[0] for item in samples_b]
    aligned: List[Tuple[float, float]] = []

    for row_a in rows_a:
        if row_a.pose_x is None or row_a.pose_y is None:
            continue

        index = bisect.bisect_left(times_b, row_a.timestamp)
        candidate_indices = []
        if index < len(samples_b):
            candidate_indices.append(index)
        if index - 1 >= 0:
            candidate_indices.append(index - 1)
        if not candidate_indices:
            continue

        nearest = min(candidate_indices, key=lambda idx: abs(samples_b[idx][0] - row_a.timestamp))
        _, bx, by = samples_b[nearest]
        distance = math.hypot(float(row_a.pose_x) - float(bx), float(row_a.pose_y) - float(by))
        aligned.append((row_a.timestamp, distance))

    if not aligned:
        return {
            "enter_time": None,
            "recover_to_1p2_s": None,
            "recover_to_1p5_s": None,
        }

    enter_time = None
    for ts, distance in aligned:
        if distance <= enter_threshold:
            enter_time = ts
            break

    if enter_time is None:
        return {
            "enter_time": None,
            "recover_to_1p2_s": 0.0,
            "recover_to_1p5_s": 0.0,
        }

    recover_1 = None
    recover_2 = None
    target_1, target_2 = recover_thresholds

    for ts, distance in aligned:
        if ts < enter_time:
            continue
        if recover_1 is None and distance > target_1:
            recover_1 = ts - enter_time
        if recover_2 is None and distance > target_2:
            recover_2 = ts - enter_time
        if recover_1 is not None and recover_2 is not None:
            break

    return {
        "enter_time": enter_time,
        "recover_to_1p2_s": recover_1,
        "recover_to_1p5_s": recover_2,
    }


def print_report(
    log_a: str,
    log_b: str,
    usv_a: str,
    usv_b: str,
    goal_threshold: float,
    min_distance: Optional[float],
    goal_a: GoalMetrics,
    goal_b: GoalMetrics,
    osc_a: OscillationMetrics,
    osc_b: OscillationMetrics,
    separation_timeliness: Dict[str, Optional[float]],
):
    success_rate = (int(goal_a.success) + int(goal_b.success)) / 2.0 * 100.0

    print("\n" + "=" * 72)
    print("APF 双船联调评估报告")
    print("=" * 72)
    print(f"日志A: {log_a}")
    print(f"日志B: {log_b}")
    print(f"判定阈值: goal <= {goal_threshold:.2f} m")

    print("\n[1] 安全性")
    if min_distance is None:
        print("- 最小船间距: N/A（缺少 pose_x/pose_y）")
    else:
        print(f"- 最小船间距: {min_distance:.3f} m")

    print("\n[2] 到点能力")
    print(f"- {usv_a}: success={goal_a.success}, min_goal_dist={goal_a.min_distance_to_goal}, first_reach_s={goal_a.first_reach_time}")
    print(f"- {usv_b}: success={goal_b.success}, min_goal_dist={goal_b.min_distance_to_goal}, first_reach_s={goal_b.first_reach_time}")
    print(f"- 双船到点成功率: {success_rate:.1f}%")

    print("\n[3] 振荡代理指标")
    print(
        f"- {usv_a}: omega_sign_changes={osc_a.sign_changes}, "
        f"sign_change_rate={osc_a.sign_change_rate_hz:.4f} Hz, heading_err_std={osc_a.heading_error_std_deg}"
    )
    print(
        f"- {usv_b}: omega_sign_changes={osc_b.sign_changes}, "
        f"sign_change_rate={osc_b.sign_change_rate_hz:.4f} Hz, heading_err_std={osc_b.heading_error_std_deg}"
    )

    print("\n[4] 调参建议")
    if min_distance is not None and min_distance < 1.5:
        print("- 船间距偏小：建议 ↑apf_safe_distance 或 ↑apf_repulsion_gain")
    if not goal_a.success or not goal_b.success:
        print("- 存在不到点：建议 ↑apf_goal_slow_speed_threshold 或 ↓apf_goal_relax_min_scale")

    osc_rate = max(osc_a.sign_change_rate_hz, osc_b.sign_change_rate_hz)
    if osc_rate > 0.25:
        print("- 摆头较频繁：建议 ↓apf_lateral_to_yaw_gain 或 ↓apf_max_angular_correction")

    if (min_distance is not None and min_distance >= 1.5) and goal_a.success and goal_b.success and osc_rate <= 0.25:
        print("- 当前参数处于可用区间，可进行更复杂场景回归测试")

    print("\n[5] 分离时效")
    enter_time = separation_timeliness.get("enter_time")
    recover_1 = separation_timeliness.get("recover_to_1p2_s")
    recover_2 = separation_timeliness.get("recover_to_1p5_s")
    if enter_time is None:
        print("- 本次未进入 d<=1.0m 近距区间")
    else:
        print(f"- 首次进入 d<=1.0m 时间戳: {enter_time}")
        if recover_1 is None:
            print("- 恢复到 d>1.2m: 未恢复")
        else:
            print(f"- 恢复到 d>1.2m 用时: {recover_1:.2f} s")
        if recover_2 is None:
            print("- 恢复到 d>1.5m: 未恢复")
        else:
            print(f"- 恢复到 d>1.5m 用时: {recover_2:.2f} s")

    print("=" * 72 + "\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate APF performance for two USV nav logs")
    parser.add_argument("--log-a", type=str, default="", help="CSV log path for USV A")
    parser.add_argument("--log-b", type=str, default="", help="CSV log path for USV B")
    parser.add_argument("--usv-a", type=str, default="usv_02", help="USV id A (used in auto-find/report)")
    parser.add_argument("--usv-b", type=str, default="usv_03", help="USV id B (used in auto-find/report)")
    parser.add_argument("--goal-threshold", type=float, default=1.5, help="Goal reached threshold in meters")
    parser.add_argument("--omega-epsilon", type=float, default=0.05, help="Ignore |cmd_omega| below this threshold")
    parser.add_argument("--auto-find", action="store_true", help="Auto find latest logs for usv-a/usv-b")
    return parser.parse_args()


def main():
    args = parse_args()

    log_a = args.log_a
    log_b = args.log_b

    if args.auto_find:
        log_a = _find_latest_log(args.usv_a) or log_a
        log_b = _find_latest_log(args.usv_b) or log_b

    if not log_a or not os.path.exists(log_a):
        raise FileNotFoundError(f"未找到日志A: {log_a}. 可使用 --auto-find")
    if not log_b or not os.path.exists(log_b):
        raise FileNotFoundError(f"未找到日志B: {log_b}. 可使用 --auto-find")

    rows_a = load_log(log_a)
    rows_b = load_log(log_b)

    if not rows_a:
        raise RuntimeError(f"日志A无有效数据: {log_a}")
    if not rows_b:
        raise RuntimeError(f"日志B无有效数据: {log_b}")

    goal_a = evaluate_goal(rows_a, args.goal_threshold)
    goal_b = evaluate_goal(rows_b, args.goal_threshold)

    osc_a = evaluate_oscillation(rows_a, args.omega_epsilon)
    osc_b = evaluate_oscillation(rows_b, args.omega_epsilon)

    min_distance = evaluate_min_intervessel_distance(rows_a, rows_b)
    separation_timeliness = evaluate_separation_timeliness(rows_a, rows_b)

    print_report(
        log_a=log_a,
        log_b=log_b,
        usv_a=args.usv_a,
        usv_b=args.usv_b,
        goal_threshold=args.goal_threshold,
        min_distance=min_distance,
        goal_a=goal_a,
        goal_b=goal_b,
        osc_a=osc_a,
        osc_b=osc_b,
        separation_timeliness=separation_timeliness,
    )


if __name__ == "__main__":
    main()
