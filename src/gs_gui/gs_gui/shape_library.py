#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图形库 — 为集群表演编排提供路径和队形坐标计算。

支持两类图形：
1. 路径图形 (path): 生成一条由多个点组成的轨迹，USV 逐点跟踪
2. 队形图形 (formation): 生成一组同时到达的目标点，USV 同步到位

所有坐标基于本地 NED 坐标系 (x=北, y=东)。
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class Waypoint:
    """单个航点。"""
    x: float
    y: float
    z: float = 0.0


@dataclass
class StepAssignment:
    """一个 step 中所有 USV 的目标位置。"""
    positions: dict  # {usv_id: Waypoint}
    nav_mode: str = "async"
    velocity: float = 0.5
    led: str = "0"
    sync_timeout: float = 10.0
    yaw_mode: str = "auto"
    yaw_value: Optional[float] = None
    maneuver_type: Optional[str] = None   # "rotate" / "spin"
    maneuver_circles: Optional[float] = None
    maneuver_direction: Optional[str] = None  # "clockwise" / "ccw"


# ────────────────────────── 基本图形生成 ──────────────────────────

def circle(center: Tuple[float, float], radius: float,
           num_points: int = 16, start_angle: float = 0.0) -> List[Waypoint]:
    """生成圆形路径点。"""
    cx, cy = center
    pts = []
    for i in range(num_points):
        angle = start_angle + 2 * math.pi * i / num_points
        pts.append(Waypoint(cx + radius * math.cos(angle),
                            cy + radius * math.sin(angle)))
    return pts


def ellipse(center: Tuple[float, float], a: float, b: float,
            num_points: int = 16, start_angle: float = 0.0) -> List[Waypoint]:
    """生成椭圆路径点，a=x半轴, b=y半轴。"""
    cx, cy = center
    pts = []
    for i in range(num_points):
        angle = start_angle + 2 * math.pi * i / num_points
        pts.append(Waypoint(cx + a * math.cos(angle),
                            cy + b * math.sin(angle)))
    return pts


def star(center: Tuple[float, float], radius: float,
         num_points: int = 5, inner_ratio: float = 0.382) -> List[Waypoint]:
    """
    生成星形路径点（外顶点和内顶点交替）。
    inner_ratio: 内径与外径的比例，五角星默认 ≈ 0.382。
    """
    cx, cy = center
    pts = []
    inner_radius = radius * inner_ratio
    total = num_points * 2
    for i in range(total):
        angle = -math.pi / 2 + 2 * math.pi * i / total
        r = radius if i % 2 == 0 else inner_radius
        pts.append(Waypoint(cx + r * math.cos(angle),
                            cy + r * math.sin(angle)))
    return pts


def polygon(center: Tuple[float, float], radius: float,
            num_sides: int = 6) -> List[Waypoint]:
    """生成正多边形路径点。"""
    cx, cy = center
    pts = []
    for i in range(num_sides):
        angle = -math.pi / 2 + 2 * math.pi * i / num_sides
        pts.append(Waypoint(cx + radius * math.cos(angle),
                            cy + radius * math.sin(angle)))
    return pts


def figure_eight(center: Tuple[float, float], radius: float,
                 num_points: int = 24) -> List[Waypoint]:
    """
    生成 8 字形路径（双圆/Lemniscate 参数化）。
    上半圆和下半圆用 offset 分开。
    """
    cx, cy = center
    pts = []
    for i in range(num_points):
        t = 2 * math.pi * i / num_points
        # Lemniscate of Bernoulli 参数化
        denom = 1 + math.sin(t) ** 2
        x = radius * math.cos(t) / denom
        y = radius * math.sin(t) * math.cos(t) / denom
        pts.append(Waypoint(cx + x, cy + y))
    return pts


def heart(center: Tuple[float, float], size: float,
          num_points: int = 24) -> List[Waypoint]:
    """生成心形路径点。"""
    cx, cy = center
    pts = []
    for i in range(num_points):
        t = 2 * math.pi * i / num_points
        x = size * 16 * math.sin(t) ** 3 / 16
        y = size * (13 * math.cos(t) - 5 * math.cos(2 * t)
                    - 2 * math.cos(3 * t) - math.cos(4 * t)) / 16
        pts.append(Waypoint(cx + x, cy + y))
    return pts


def zigzag(start: Tuple[float, float], end: Tuple[float, float],
           amplitude: float, num_zigs: int = 4) -> List[Waypoint]:
    """
    生成 Z 字形路径。
    从 start 到 end 方向行进，沿垂直方向交替偏移 amplitude。
    """
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return [Waypoint(sx, sy)]
    # 单位方向和法向量
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    pts = []
    total_points = num_zigs * 2 + 1
    for i in range(total_points):
        frac = i / (total_points - 1)
        bx = sx + dx * frac
        by = sy + dy * frac
        if i % 2 == 1:
            sign = 1 if (i // 2) % 2 == 0 else -1
            bx += nx * amplitude * sign
            by += ny * amplitude * sign
        pts.append(Waypoint(bx, by))
    return pts


def spiral(center: Tuple[float, float], start_radius: float,
           end_radius: float, num_turns: float = 3.0,
           num_points: int = 36) -> List[Waypoint]:
    """生成螺旋路径点。"""
    cx, cy = center
    pts = []
    for i in range(num_points):
        frac = i / (num_points - 1) if num_points > 1 else 0
        angle = 2 * math.pi * num_turns * frac
        r = start_radius + (end_radius - start_radius) * frac
        pts.append(Waypoint(cx + r * math.cos(angle),
                            cy + r * math.sin(angle)))
    return pts


def line(start: Tuple[float, float], end: Tuple[float, float],
         num_points: int = 2) -> List[Waypoint]:
    """生成直线路径点。"""
    sx, sy = start
    ex, ey = end
    pts = []
    for i in range(num_points):
        frac = i / (num_points - 1) if num_points > 1 else 0
        pts.append(Waypoint(sx + (ex - sx) * frac,
                            sy + (ey - sy) * frac))
    return pts


def wave(start: Tuple[float, float], end: Tuple[float, float],
         amplitude: float, wavelength: float = 4.0,
         num_points: int = 24) -> List[Waypoint]:
    """生成正弦波路径。"""
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return [Waypoint(sx, sy)]
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    pts = []
    for i in range(num_points):
        frac = i / (num_points - 1) if num_points > 1 else 0
        dist = length * frac
        offset = amplitude * math.sin(2 * math.pi * dist / wavelength)
        bx = sx + dx * frac + nx * offset
        by = sy + dy * frac + ny * offset
        pts.append(Waypoint(bx, by))
    return pts


# ────────────────────── 队形点位生成 ──────────────────────

def formation_circle(center: Tuple[float, float], radius: float,
                     count: int) -> List[Waypoint]:
    """count 个点均匀分布在圆上。"""
    return circle(center, radius, num_points=count)


def formation_star(center: Tuple[float, float], radius: float,
                   count: int, num_points: int = 5) -> List[Waypoint]:
    """在星形外顶点上均匀分布 count 个点（如不够则插值）。"""
    cx, cy = center
    pts = []
    for i in range(count):
        angle = -math.pi / 2 + 2 * math.pi * i / count
        pts.append(Waypoint(cx + radius * math.cos(angle),
                            cy + radius * math.sin(angle)))
    return pts


def formation_line(center: Tuple[float, float], spacing: float,
                   count: int, heading: float = 0.0) -> List[Waypoint]:
    """
    一字排列，heading 为排列方向角度（弧度，0=x正方向）。
    """
    cx, cy = center
    pts = []
    total_width = spacing * (count - 1)
    ux = math.cos(heading)
    uy = math.sin(heading)
    for i in range(count):
        offset = -total_width / 2 + spacing * i
        pts.append(Waypoint(cx + ux * offset, cy + uy * offset))
    return pts


def formation_v(center: Tuple[float, float], spacing_along: float,
                spacing_cross: float, count: int,
                heading: float = 0.0) -> List[Waypoint]:
    """V 字形队列。"""
    cx, cy = center
    # heading 方向为前方
    fwd_x = math.cos(heading)
    fwd_y = math.sin(heading)
    right_x = -fwd_y
    right_y = fwd_x

    pts = [Waypoint(cx, cy)]  # leader at center
    for i in range(1, count):
        row = (i + 1) // 2
        side = 1 if i % 2 == 1 else -1
        along_offset = -row * spacing_along
        cross_offset = side * row * spacing_cross
        px = cx + fwd_x * along_offset + right_x * cross_offset
        py = cy + fwd_y * along_offset + right_y * cross_offset
        pts.append(Waypoint(px, py))
    return pts


def formation_grid(center: Tuple[float, float], spacing: float,
                   cols: int, rows: int) -> List[Waypoint]:
    """网格排列。"""
    cx, cy = center
    total_x = spacing * (cols - 1)
    total_y = spacing * (rows - 1)
    pts = []
    for r in range(rows):
        for c in range(cols):
            pts.append(Waypoint(
                cx - total_x / 2 + spacing * c,
                cy - total_y / 2 + spacing * r
            ))
    return pts


# ────────────────────── 编排辅助 ──────────────────────

def distribute_paths_to_usvs(
    paths: List[List[Waypoint]],
    usv_ids: List[str]
) -> List[StepAssignment]:
    """
    将多条并行路径分配给 USV，生成逐步的 StepAssignment 列表。
    paths[i] 是第 i 条路径的航点列表，paths[i][j] 是第 j 步的目标。
    所有路径必须等长。
    """
    if not paths or not usv_ids:
        return []
    num_steps = len(paths[0])
    num_usvs = min(len(usv_ids), len(paths))
    steps = []
    for j in range(num_steps):
        positions = {}
        for i in range(num_usvs):
            positions[usv_ids[i]] = paths[i][j]
        steps.append(StepAssignment(positions=positions))
    return steps


def formation_to_steps(
    formations: List[List[Waypoint]],
    usv_ids: List[str],
    nav_mode: str = "sync",
    velocity: float = 0.5,
    led: str = "0",
    sync_timeout: float = 10.0
) -> List[StepAssignment]:
    """
    将一系列队形点位转换为 StepAssignment。
    formations[j] 是第 j 步的队形 (长度 >= len(usv_ids))。
    """
    steps = []
    for formation in formations:
        positions = {}
        for i, usv_id in enumerate(usv_ids):
            if i < len(formation):
                positions[usv_id] = formation[i]
        steps.append(StepAssignment(
            positions=positions,
            nav_mode=nav_mode,
            velocity=velocity,
            led=led,
            sync_timeout=sync_timeout
        ))
    return steps


def rotate_points(points: List[Waypoint], center: Tuple[float, float],
                  angle: float) -> List[Waypoint]:
    """将一组点绕 center 旋转 angle 弧度。"""
    cx, cy = center
    rotated = []
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    for p in points:
        dx = p.x - cx
        dy = p.y - cy
        rotated.append(Waypoint(
            cx + dx * cos_a - dy * sin_a,
            cy + dx * sin_a + dy * cos_a,
            p.z
        ))
    return rotated


def scale_points(points: List[Waypoint], center: Tuple[float, float],
                 factor: float) -> List[Waypoint]:
    """将一组点相对 center 缩放。"""
    cx, cy = center
    return [Waypoint(cx + (p.x - cx) * factor,
                     cy + (p.y - cy) * factor, p.z)
            for p in points]


def translate_points(points: List[Waypoint],
                     dx: float, dy: float) -> List[Waypoint]:
    """平移一组点。"""
    return [Waypoint(p.x + dx, p.y + dy, p.z) for p in points]


# ────────────────────── 注册表 ──────────────────────

SHAPE_REGISTRY = {
    "circle": {
        "func": circle,
        "label": "圆形",
        "params": {"center": (0, 0), "radius": 6.0, "num_points": 16},
    },
    "ellipse": {
        "func": ellipse,
        "label": "椭圆",
        "params": {"center": (0, 0), "a": 8.0, "b": 4.0, "num_points": 16},
    },
    "star": {
        "func": star,
        "label": "五角星",
        "params": {"center": (0, 0), "radius": 6.0, "num_points": 5},
    },
    "polygon": {
        "func": polygon,
        "label": "多边形",
        "params": {"center": (0, 0), "radius": 6.0, "num_sides": 6},
    },
    "figure_eight": {
        "func": figure_eight,
        "label": "8字形",
        "params": {"center": (0, 0), "radius": 6.0, "num_points": 24},
    },
    "heart": {
        "func": heart,
        "label": "心形",
        "params": {"center": (0, 0), "size": 6.0, "num_points": 24},
    },
    "zigzag": {
        "func": zigzag,
        "label": "Z字形",
        "params": {"start": (0, -12), "end": (0, 12), "amplitude": 4.0, "num_zigs": 4},
    },
    "spiral": {
        "func": spiral,
        "label": "螺旋",
        "params": {"center": (0, 0), "start_radius": 1.0, "end_radius": 6.0},
    },
    "line": {
        "func": line,
        "label": "直线",
        "params": {"start": (0, -6), "end": (0, 6)},
    },
    "wave": {
        "func": wave,
        "label": "波浪",
        "params": {"start": (0, -12), "end": (0, 12), "amplitude": 3.0},
    },
}

FORMATION_REGISTRY = {
    "circle": {"func": formation_circle, "label": "圆形队列"},
    "star": {"func": formation_star, "label": "星形队列"},
    "line": {"func": formation_line, "label": "一字排列"},
    "v_shape": {"func": formation_v, "label": "V字形队列"},
    "grid": {"func": formation_grid, "label": "网格排列"},
}
