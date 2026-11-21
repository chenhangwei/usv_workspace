#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of usv plot window.
#
# Author: chenhangwei
# Date: 2026-01-26
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QCheckBox, QHBoxLayout,
    QPushButton, QLabel, QSlider, QGroupBox, QMessageBox, QWidget, QMenu, QAction,
    QStackedWidget, QDoubleSpinBox, QComboBox, QFormLayout, QDialogButtonBox,
    QListWidget, QListWidgetItem, QFrame, QSizePolicy
)
from PyQt5.QtCore import QTimer, Qt, QPointF, QRectF
from PyQt5.QtCore import Qt as QtCore
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QPolygonF, QFont, QLinearGradient, QPainterPath
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from matplotlib.figure import Figure
from matplotlib.markers import MarkerStyle
from matplotlib.path import Path as MplPath
from math import cos, sin, pi, atan2, sqrt, radians, degrees, tan
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.transforms import offset_copy
import numpy as np
import time
import datetime

# Configure matplotlib font
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS', 'WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False


class ThreeDTargetDialog(QDialog):
    """在 3D / 2D 态势界面快速发送三维目标点的非侵入式对话框"""

    def __init__(self, usv_list, default_xyz=(0.0, 0.0, 0.0), preselected_id=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("🎯 发送三维目标点 (3D Goal)")
        self.setMinimumWidth(360)

        layout = QVBoxLayout(self)
        form = QFormLayout()

        self.usv_list_widget = QListWidget()
        self.usv_list_widget.setSelectionMode(QListWidget.MultiSelection)
        self.usv_list_widget.setMaximumHeight(140)

        online_items = [u for u in (usv_list or []) if u.get('connected', False)]
        if not online_items:
            online_items = list(usv_list or [])

        default_z = float(default_xyz[2])
        for u in online_items:
            uid = str(u.get('namespace', '')).lstrip('/')
            if not uid:
                continue
            vt = str(u.get('vehicle_type', '') or 'usv_diff')
            pm = str(u.get('platform_mode', '') or ('2d' if vt in ('usv', 'usv_diff', 'usv_boat') else '3d'))
            pos = u.get('position', {}) or {}
            text = f"{uid}  [{vt} | {pm.upper()}]  ({pos.get('x', 0.0):.1f}, {pos.get('y', 0.0):.1f}, {pos.get('z', 0.0):.1f})"
            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, u)
            self.usv_list_widget.addItem(item)
            if preselected_id and uid == preselected_id:
                item.setSelected(True)
                if abs(default_z) < 1e-3 and pm == '3d':
                    default_z = float(u.get('default_altitude') or max(float(pos.get('z', 0.0)), 2.5))

        if self.usv_list_widget.count() > 0 and not self.usv_list_widget.selectedItems():
            self.usv_list_widget.item(0).setSelected(True)

        form.addRow("目标个体 (可多选):", self.usv_list_widget)

        self.coord_combo = QComboBox()
        self.coord_combo.addItem("局部 ENU 坐标 (地面站 3D 场景直指)", "local")
        self.coord_combo.addItem("任务区域坐标 (叠加 Area Center 偏移)", "area")
        form.addRow("参考坐标系:", self.coord_combo)

        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-5000.0, 5000.0)
        self.spin_x.setDecimals(2)
        self.spin_x.setSingleStep(0.5)
        self.spin_x.setSuffix(" m")
        self.spin_x.setValue(float(default_xyz[0]))
        form.addRow("目标 X (东):", self.spin_x)

        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-5000.0, 5000.0)
        self.spin_y.setDecimals(2)
        self.spin_y.setSingleStep(0.5)
        self.spin_y.setSuffix(" m")
        self.spin_y.setValue(float(default_xyz[1]))
        form.addRow("目标 Y (北):", self.spin_y)

        self.spin_z = QDoubleSpinBox()
        self.spin_z.setRange(-500.0, 2000.0)
        self.spin_z.setDecimals(2)
        self.spin_z.setSingleStep(0.5)
        self.spin_z.setSuffix(" m")
        self.spin_z.setValue(default_z)
        form.addRow("目标 Z (高/深):", self.spin_z)

        yaw_row = QHBoxLayout()
        self.chk_use_yaw = QCheckBox("指定到达航向")
        self.spin_yaw_deg = QDoubleSpinBox()
        self.spin_yaw_deg.setRange(-180.0, 180.0)
        self.spin_yaw_deg.setDecimals(1)
        self.spin_yaw_deg.setSuffix(" °")
        self.spin_yaw_deg.setEnabled(False)
        self.chk_use_yaw.toggled.connect(self.spin_yaw_deg.setEnabled)
        yaw_row.addWidget(self.chk_use_yaw)
        yaw_row.addWidget(self.spin_yaw_deg)
        form.addRow("航向约束:", yaw_row)

        layout.addLayout(form)

        tip = QLabel("提示: 水面差速无人船 (2D) 将自动贴合 Z=0m 水面；飞艇/无人机/海豚支持任意三维高度/深度。")
        tip.setWordWrap(True)
        tip.setStyleSheet("color: #7f8c8d; font-size: 12px;")
        layout.addWidget(tip)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Ok).setText("🚀 立即发送")
        buttons.button(QDialogButtonBox.Cancel).setText("取消")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_payload(self) -> dict:
        usv_ids = []
        for item in self.usv_list_widget.selectedItems():
            u = item.data(Qt.UserRole) or {}
            uid = str(u.get('namespace', '')).lstrip('/')
            if uid:
                usv_ids.append(uid)
        return {
            'usv_ids': usv_ids,
            'x': float(self.spin_x.value()),
            'y': float(self.spin_y.value()),
            'z': float(self.spin_z.value()),
            'yaw': radians(float(self.spin_yaw_deg.value())),
            'use_yaw': bool(self.chk_use_yaw.isChecked()),
            'coord_frame': str(self.coord_combo.currentData() or 'local'),
        }


class Usv3DCanvas(QWidget):
    """
    轻量硬件抗锯齿 3D 态势反馈画布 (ENU: X=East, Y=North, Z=Up)。
    支持：
    - 左键拖拽旋转视角、右键/中键/Shift+左键平移焦点、滚轮缩放、双击自适应全景
    - 水面网格平面 (Z=0)、高度标尺立柱、空中/水下载具高度投影垂线与水面阴影
    - 异构三维机体绘制：差速无人船 (0.60x0.30m 双推进)、仿生飞艇、仿生海豚、飞行球
    - 三维预览航线、三维实时尾迹、三维目标连线与坐标标签
    - 左键点选载具联动详情/导航面板，右键直发任意三维目标点
    """

    def __init__(self, host_plot_window, parent=None):
        super().__init__(parent)
        self.host = host_plot_window
        self.setMinimumSize(360, 260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

        # 3D 相机状态
        self.azimuth_deg = -48.0      # 水平方位角 (°)
        self.elevation_deg = 32.0     # 俯仰角 (°)
        self.distance = 32.0          # 相机距焦点距离 (m)
        self.center = np.array([0.0, 0.0, 0.8], dtype=float)
        self.fov_deg = 46.0
        self.ortho_mode = False       # 正交俯视/三视图开关
        self.follow_selected = False  # 跟随选中载具

        # 交互状态
        self._drag_mode = None        # 'orbit' | 'pan'
        self._press_pos = None
        self._last_pos = None
        self._moved_px = 0.0
        self.selected_usv_id = None
        self.hover_world_xy = (0.0, 0.0)
        self._projected_usvs = []     # [{'id', 'sx', 'sy', 'depth', 'usv'}]
        self._is_dark = True
        self._auto_fitted_once = False

    def set_dark_mode(self, is_dark: bool):
        self._is_dark = bool(is_dark)
        self.update()

    # -------------------- 相机矩阵与投影 --------------------
    def _camera_basis(self):
        az = radians(self.azimuth_deg)
        el = radians(self.elevation_deg)
        # 相机位置相对 center 的方向（球坐标，Z 轴向上）
        cam_dir = np.array([
            cos(el) * cos(az),
            cos(el) * sin(az),
            sin(el),
        ], dtype=float)
        cam_pos = self.center + self.distance * cam_dir
        forward = -cam_dir  # 从相机指向 center
        world_up = np.array([0.0, 0.0, 1.0], dtype=float)
        right = np.cross(forward, world_up)
        rn = np.linalg.norm(right)
        if rn < 1e-6:
            # 俯视接近 ±90° 时使用 ENU +X/+Y 作为屏幕参考（东向右，北向上）
            right = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            right = right / rn
        up = np.cross(right, forward)
        un = np.linalg.norm(up)
        if un > 1e-6:
            up = up / un
        return cam_pos, forward, right, up

    def project_point(self, pt3, cam_basis=None):
        """将三维世界点 (x, y, z) 投影到屏幕像素 (sx, sy, depth)"""
        w = max(1, self.width())
        h = max(1, self.height())
        if cam_basis is None:
            cam_basis = self._camera_basis()
        cam_pos, forward, right, up = cam_basis

        p = np.asarray(pt3, dtype=float)
        rel = p - cam_pos
        depth = float(np.dot(rel, forward))
        rx = float(np.dot(rel, right))
        ry = float(np.dot(rel, up))

        scale_base = min(w, h) * 0.5
        if self.ortho_mode:
            ortho_half = max(2.0, self.distance * 0.45)
            sx = w * 0.5 + (rx / ortho_half) * scale_base
            sy = h * 0.5 - (ry / ortho_half) * scale_base
            return sx, sy, max(0.1, depth)

        if depth <= 0.15:
            return None
        focal = scale_base / tan(radians(self.fov_deg * 0.5))
        sx = w * 0.5 + (rx / depth) * focal
        sy = h * 0.5 - (ry / depth) * focal
        return sx, sy, depth

    def screen_to_water_plane(self, sx: float, sy: float, plane_z: float = 0.0):
        """将屏幕像素反投射到水平高度平面 Z = plane_z，返回 (x, y, plane_z)"""
        w = max(1, self.width())
        h = max(1, self.height())
        cam_pos, forward, right, up = self._camera_basis()
        scale_base = min(w, h) * 0.5

        if self.ortho_mode:
            ortho_half = max(2.0, self.distance * 0.45)
            dx = ((sx - w * 0.5) / scale_base) * ortho_half
            dy = ((h * 0.5 - sy) / scale_base) * ortho_half
            origin = cam_pos + right * dx + up * dy
            ray_dir = forward
        else:
            focal = scale_base / tan(radians(self.fov_deg * 0.5))
            nx = (sx - w * 0.5) / focal
            ny = (h * 0.5 - sy) / focal
            ray_dir = forward + right * nx + up * ny
            norm = np.linalg.norm(ray_dir)
            if norm < 1e-8:
                return float(self.center[0]), float(self.center[1]), float(plane_z)
            ray_dir = ray_dir / norm
            origin = cam_pos

        if abs(ray_dir[2]) < 1e-4:
            return float(self.center[0]), float(self.center[1]), float(plane_z)
        t = (plane_z - origin[2]) / ray_dir[2]
        if t < 0 and not self.ortho_mode:
            return float(self.center[0]), float(self.center[1]), float(plane_z)
        hit = origin + t * ray_dir
        return float(hit[0]), float(hit[1]), float(plane_z)

    # -------------------- 预设视角与自适应 --------------------
    def set_view_preset(self, preset: str):
        if preset == '3d':
            self.ortho_mode = False
            self.azimuth_deg = -48.0
            self.elevation_deg = 32.0
        elif preset == 'top':
            self.ortho_mode = True
            self.azimuth_deg = -90.0
            self.elevation_deg = 89.9
        elif preset == 'front_xz':
            self.ortho_mode = False
            self.azimuth_deg = -90.0
            self.elevation_deg = 8.0
        elif preset == 'side_yz':
            self.ortho_mode = False
            self.azimuth_deg = 0.0
            self.elevation_deg = 8.0
        self.update()

    def fit_scene(self):
        pts = self._collect_scene_points()
        if not pts:
            self.center = np.array([0.0, 0.0, 0.8], dtype=float)
            self.distance = 30.0
            self.update()
            return
        arr = np.asarray(pts, dtype=float)
        mins = arr.min(axis=0)
        maxs = arr.max(axis=0)
        self.center = 0.5 * (mins + maxs)
        span = float(np.linalg.norm(maxs - mins))
        self.distance = max(14.0, min(800.0, span * 1.35 + 8.0))
        self.update()

    def _collect_scene_points(self):
        pts = []
        if getattr(self.host, 'home_pos', None):
            hp = self.host.home_pos
            pts.append((hp[0], hp[1], 0.0))
        if getattr(self.host, 'area_center_pos', None):
            ac = self.host.area_center_pos
            pts.append((ac[0], ac[1], 0.0))
        for items in (getattr(self.host, 'preview_trails', {}) or {}).values():
            for it in items:
                p = it.get('pos', (0.0, 0.0, 0.0))
                pts.append((p[0], p[1], p[2] if len(p) > 2 else 0.0))
        for u in (self.host.get_usv_list_func() or []):
            if not u.get('connected', False):
                continue
            pos = u.get('position', {}) or {}
            pts.append((float(pos.get('x', 0.0)), float(pos.get('y', 0.0)), float(pos.get('z', 0.0))))
            nt = u.get('nav_target_cache')
            if isinstance(nt, dict) and nt.get('x') is not None and nt.get('y') is not None:
                pts.append((float(nt.get('x', 0.0)), float(nt.get('y', 0.0)), float(nt.get('z', 0.0))))
        return pts

    # -------------------- 异构三维机体线框/面片绘制 --------------------
    def _draw_poly3d(self, painter: QPainter, verts_world, fill_color: QColor, edge_color: QColor, edge_w: float = 1.3, cam_basis=None):
        proj = []
        for v in verts_world:
            p = self.project_point(v, cam_basis)
            if p is None:
                return
            proj.append(QPointF(p[0], p[1]))
        poly = QPolygonF(proj)
        painter.setPen(QPen(edge_color, edge_w))
        painter.setBrush(QBrush(fill_color))
        painter.drawPolygon(poly)

    def _transform_body(self, local_pts, pos_xyz, yaw_rad: float, pitch_rad: float = 0.0):
        cy, sy = cos(yaw_rad), sin(yaw_rad)
        cp, sp = cos(pitch_rad), sin(pitch_rad)
        # Ry(-pitch) then Rz(yaw) in ENU
        rot = np.array([
            [cy * cp, -sy, cy * sp],
            [sy * cp,  cy, sy * sp],
            [-sp,     0.0, cp],
        ], dtype=float)
        arr = np.asarray(local_pts, dtype=float) @ rot.T + np.asarray(pos_xyz, dtype=float)
        return arr

    def _draw_vehicle_3d(self, painter: QPainter, usv: dict, base_hex: str, is_selected: bool, is_leader: bool, cam_basis):
        pos = usv.get('position', {}) or {}
        x = float(pos.get('x', 0.0))
        y = float(pos.get('y', 0.0))
        z = float(pos.get('z', 0.0))
        yaw = float(usv.get('yaw', 0.0))
        pitch = float(usv.get('pitch', 0.0) or 0.0)
        vt = str(usv.get('vehicle_type', '') or '').strip().lower()
        if not vt:
            pm = str(usv.get('platform_mode', '') or '').strip().lower()
            vt = 'bionic_airship' if (pm == '3d' or abs(z) > 0.35) else 'usv_diff'

        # 远距离视觉放大系数，保证大场景下 0.60m 船体依然清晰可辨，近景恢复真实比例
        vis_scale = max(1.0, min(4.2, self.distance / 14.0))
        c_main = QColor(base_hex)
        c_top = QColor(c_main).lighter(122)
        c_side = QColor(c_main).darker(125)
        edge_c = QColor("#FFD700") if is_leader else (QColor("#00E5FF") if is_selected else QColor("#ECF0F1"))
        edge_w = 2.4 if (is_selected or is_leader) else 1.2

        # 高度/深度投影线与水面投影圆（当 |z| > 0.08m 时显示立体高度感）
        if abs(z) > 0.08:
            p_veh = self.project_point((x, y, z), cam_basis)
            p_wat = self.project_point((x, y, 0.0), cam_basis)
            if p_veh and p_wat:
                stalk_c = QColor("#00E5FF" if z > 0 else "#3498DB")
                stalk_c.setAlpha(155)
                painter.setPen(QPen(stalk_c, 1.2, Qt.DashLine))
                painter.drawLine(QPointF(p_veh[0], p_veh[1]), QPointF(p_wat[0], p_wat[1]))
                shadow_c = QColor(0, 0, 0, 105) if self._is_dark else QColor(44, 62, 80, 85)
                painter.setPen(QPen(stalk_c, 1.0))
                painter.setBrush(QBrush(shadow_c))
                painter.drawEllipse(QPointF(p_wat[0], p_wat[1]), 4.5, 2.8)

        center_xyz = np.array([x, y, z], dtype=float)

        if vt in ('bionic_airship', 'airship'):
            # 仿生飞艇：流线椭球囊体 + 十字尾翼 + 吊舱
            L, R = 1.35 * vis_scale, 0.34 * vis_scale
            ring_x = [-0.5 * L, -0.22 * L, 0.15 * L, 0.5 * L]
            ring_r = [0.0, 0.85 * R, 1.0 * R, 0.0]
            angles = [0.0, pi * 0.5, pi, pi * 1.5]
            for i in range(len(ring_x) - 1):
                for k in range(4):
                    a0, a1 = angles[k], angles[(k + 1) % 4]
                    quad = [
                        [ring_x[i],     ring_r[i] * cos(a0),     ring_r[i] * sin(a0)],
                        [ring_x[i + 1], ring_r[i + 1] * cos(a0), ring_r[i + 1] * sin(a0)],
                        [ring_x[i + 1], ring_r[i + 1] * cos(a1), ring_r[i + 1] * sin(a1)],
                        [ring_x[i],     ring_r[i] * cos(a1),     ring_r[i] * sin(a1)],
                    ]
                    w_quad = self._transform_body(quad, center_xyz, yaw, pitch * 0.4)
                    fc = c_top if k in (0, 3) else c_side
                    fc.setAlpha(215)
                    self._draw_poly3d(painter, w_quad, fc, edge_c, edge_w * 0.75, cam_basis)
            # 尾翼
            fin_v = self._transform_body([
                [-0.28 * L, 0.0, 0.0], [-0.52 * L, 0.0, 0.52 * R * 1.5], [-0.50 * L, 0.0, 0.0]
            ], center_xyz, yaw, pitch * 0.4)
            self._draw_poly3d(painter, fin_v, c_side, edge_c, edge_w * 0.8, cam_basis)

        elif vt in ('bionic_dolphin', 'dolphin'):
            # 仿生海豚：流线梭形 + 背鳍 + 水平尾鳍
            L, W, H = 1.15 * vis_scale, 0.28 * vis_scale, 0.24 * vis_scale
            deck = self._transform_body([
                [0.55 * L, 0.0, 0.0],
                [0.10 * L,  0.5 * W, 0.2 * H],
                [-0.48 * L, 0.12 * W, 0.0],
                [-0.62 * L, 0.58 * W, 0.0],
                [-0.52 * L, 0.0, 0.0],
                [-0.62 * L, -0.58 * W, 0.0],
                [-0.48 * L, -0.12 * W, 0.0],
                [0.10 * L, -0.5 * W, 0.2 * H],
            ], center_xyz, yaw, pitch)
            self._draw_poly3d(painter, deck, c_top, edge_c, edge_w, cam_basis)
            dorsal = self._transform_body([
                [0.08 * L, 0.0, 0.18 * H], [-0.14 * L, 0.0, 0.95 * H], [-0.22 * L, 0.0, 0.12 * H]
            ], center_xyz, yaw, pitch)
            self._draw_poly3d(painter, dorsal, c_side, edge_c, edge_w * 0.85, cam_basis)

        elif vt in ('flying_sphere', 'sphere', 'uav'):
            # 飞行球：立体球体 + 正交护环 + 航向指示锥
            p_c = self.project_point(center_xyz, cam_basis)
            if p_c:
                p_r = self.project_point(center_xyz + np.array([0.32 * vis_scale, 0.0, 0.0]), cam_basis)
                rad_px = max(7.0, abs(p_r[0] - p_c[0]) if p_r else 10.0)
                grad_c = QColor(c_top)
                grad_c.setAlpha(210)
                painter.setPen(QPen(edge_c, edge_w))
                painter.setBrush(QBrush(grad_c))
                painter.drawEllipse(QPointF(p_c[0], p_c[1]), rad_px, rad_px)
                # 赤道环
                ring_pts = [
                    self._transform_body([[0.36 * vis_scale * cos(a), 0.36 * vis_scale * sin(a), 0.0]], center_xyz, yaw)[0]
                    for a in np.linspace(0, 2 * pi, 17)
                ]
                ring_2d = [self.project_point(pt, cam_basis) for pt in ring_pts]
                if all(ring_2d):
                    painter.setPen(QPen(edge_c, 1.3))
                    for i in range(len(ring_2d) - 1):
                        painter.drawLine(QPointF(ring_2d[i][0], ring_2d[i][1]), QPointF(ring_2d[i + 1][0], ring_2d[i + 1][1]))

        else:
            # 差速无人船 (usv_diff / usv): 严格 0.60m × 0.30m 尖头单体船体 + 左右双推进器
            L, W, H = 0.60 * vis_scale, 0.30 * vis_scale, 0.11 * vis_scale
            deck_local = [
                [ 0.50 * L,  0.00 * W, H],   # 船艏尖端
                [ 0.16 * L,  0.50 * W, H],   # 左舷前肩
                [-0.46 * L,  0.48 * W, H],   # 左艉角
                [-0.36 * L,  0.00 * W, H],   # 艉部推进凹槽
                [-0.46 * L, -0.48 * W, H],   # 右艉角
                [ 0.16 * L, -0.50 * W, H],   # 右舷前肩
            ]
            keel_local = [
                [ 0.44 * L,  0.00 * W, 0.0],
                [ 0.14 * L,  0.42 * W, 0.0],
                [-0.44 * L,  0.42 * W, 0.0],
                [-0.44 * L, -0.42 * W, 0.0],
                [ 0.14 * L, -0.42 * W, 0.0],
            ]
            deck_w = self._transform_body(deck_local, center_xyz, yaw)
            keel_w = self._transform_body(keel_local, center_xyz, yaw)
            # 左舷与右舷立面
            self._draw_poly3d(painter, [deck_w[0], deck_w[1], keel_w[1], keel_w[0]], c_side, edge_c, 0.9, cam_basis)
            self._draw_poly3d(painter, [deck_w[1], deck_w[2], keel_w[2], keel_w[1]], c_side, edge_c, 0.9, cam_basis)
            self._draw_poly3d(painter, [deck_w[0], deck_w[5], keel_w[4], keel_w[0]], c_side, edge_c, 0.9, cam_basis)
            self._draw_poly3d(painter, [deck_w[5], deck_w[4], keel_w[3], keel_w[4]], c_side, edge_c, 0.9, cam_basis)
            # 甲板顶面
            self._draw_poly3d(painter, deck_w, c_top, edge_c, edge_w, cam_basis)
            # 差速双推进器标识点 (y = ±0.35W)
            for py_sign in (-0.35, 0.35):
                thr_w = self._transform_body([[-0.48 * L, py_sign * W, 0.45 * H]], center_xyz, yaw)[0]
                pt_thr = self.project_point(thr_w, cam_basis)
                if pt_thr:
                    painter.setPen(Qt.NoPen)
                    painter.setBrush(QBrush(QColor("#F39C12")))
                    painter.drawEllipse(QPointF(pt_thr[0], pt_thr[1]), 2.8, 2.8)

        # 航向箭头
        nose_a = self._transform_body([[0.42 * vis_scale, 0.0, 0.14 * vis_scale]], center_xyz, yaw, pitch * 0.3)[0]
        nose_b = self._transform_body([[0.95 * vis_scale, 0.0, 0.14 * vis_scale]], center_xyz, yaw, pitch * 0.3)[0]
        pa = self.project_point(nose_a, cam_basis)
        pb = self.project_point(nose_b, cam_basis)
        if pa and pb:
            painter.setPen(QPen(edge_c, 1.8))
            painter.drawLine(QPointF(pa[0], pa[1]), QPointF(pb[0], pb[1]))

    # -------------------- 主渲染循环 --------------------
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.TextAntialiasing, True)

        w, h = self.width(), self.height()
        # 1. 立体深海/天空渐变背景
        grad = QLinearGradient(0, 0, 0, h)
        if self._is_dark:
            grad.setColorAt(0.0, QColor("#0d1b2a"))
            grad.setColorAt(0.55, QColor("#142638"))
            grad.setColorAt(1.0, QColor("#0b1520"))
            grid_major_c = QColor(90, 140, 180, 95)
            grid_minor_c = QColor(65, 105, 140, 50)
            water_fill = QColor(20, 75, 115, 38)
            text_fg = QColor("#ECF0F1")
        else:
            grad.setColorAt(0.0, QColor("#dceefb"))
            grad.setColorAt(0.55, QColor("#edf6fc"))
            grad.setColorAt(1.0, QColor("#d4e9f7"))
            grid_major_c = QColor(70, 110, 145, 110)
            grid_minor_c = QColor(100, 140, 175, 60)
            water_fill = QColor(52, 152, 219, 32)
            text_fg = QColor("#1F2D3D")
        painter.fillRect(0, 0, w, h, grad)

        usv_list = self.host.get_usv_list_func() or []
        online_usvs = [u for u in usv_list if u.get('connected', False)]

        if not self._auto_fitted_once and (online_usvs or getattr(self.host, 'preview_trails', None)):
            self._auto_fitted_once = True
            self.fit_scene()

        # 跟随选中载具
        if self.follow_selected and self.selected_usv_id:
            for u in online_usvs:
                if str(u.get('namespace', '')).lstrip('/') == self.selected_usv_id:
                    pos = u.get('position', {}) or {}
                    self.center = np.array([
                        float(pos.get('x', 0.0)),
                        float(pos.get('y', 0.0)),
                        float(pos.get('z', 0.0)),
                    ], dtype=float)
                    break

        cam_basis = self._camera_basis()

        # 2. 绘制水面网格 (Z = 0)
        if getattr(self.host, 'show_grid', True):
            raw_step = max(1.0, self.distance / 8.0)
            if raw_step <= 2.0:
                step = 2.0
            elif raw_step <= 5.0:
                step = 5.0
            elif raw_step <= 10.0:
                step = 10.0
            else:
                step = 20.0
            half_extent = step * 8.0
            gx0 = round(self.center[0] / step) * step - half_extent
            gx1 = gx0 + 2.0 * half_extent
            gy0 = round(self.center[1] / step) * step - half_extent
            gy1 = gy0 + 2.0 * half_extent

            # 水面半透明四边形
            corners = [(gx0, gy0, 0.0), (gx1, gy0, 0.0), (gx1, gy1, 0.0), (gx0, gy1, 0.0)]
            c_proj = [self.project_point(c, cam_basis) for c in corners]
            if all(c_proj):
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(water_fill))
                painter.drawPolygon(QPolygonF([QPointF(p[0], p[1]) for p in c_proj]))

            n_lines = int(round((gx1 - gx0) / step)) + 1
            for i in range(n_lines):
                xv = gx0 + i * step
                is_major = (abs(xv) < 1e-4) or (i % 2 == 0)
                painter.setPen(QPen(grid_major_c if is_major else grid_minor_c, 1.1 if is_major else 0.7))
                self._draw_segment_3d(painter, (xv, gy0, 0.0), (xv, gy1, 0.0), cam_basis)

                yv = gy0 + i * step
                is_major_y = (abs(yv) < 1e-4) or (i % 2 == 0)
                painter.setPen(QPen(grid_major_c if is_major_y else grid_minor_c, 1.1 if is_major_y else 0.7))
                self._draw_segment_3d(painter, (gx0, yv, 0.0), (gx1, yv, 0.0), cam_basis)

        # 3. 绘制三维 ENU 坐标轴与高度标尺 (原点 0,0,0)
        axis_len = max(3.0, min(12.0, self.distance * 0.16))
        self._draw_axis_line(painter, (0, 0, 0), (axis_len, 0, 0), QColor("#E74C3C"), "X (东)", cam_basis)
        self._draw_axis_line(painter, (0, 0, 0), (0, axis_len, 0), QColor("#2ECC71"), "Y (北)", cam_basis)
        self._draw_axis_line(painter, (0, 0, 0), (0, 0, axis_len * 0.85), QColor("#00E5FF"), "Z (高)", cam_basis)

        # Home 点与 Area Center 标记
        if getattr(self.host, 'home_pos', None):
            hx, hy = self.host.home_pos[0], self.host.home_pos[1]
            ph = self.project_point((hx, hy, 0.0), cam_basis)
            if ph:
                painter.setPen(QPen(QColor("#F1C40F"), 2.0))
                painter.setBrush(QBrush(QColor(241, 196, 15, 170)))
                painter.drawEllipse(QPointF(ph[0], ph[1]), 5.0, 5.0)
                painter.setFont(QFont("DejaVu Sans", 8, QFont.Bold))
                painter.drawText(QPointF(ph[0] + 7, ph[1] - 5), "🏠 Home")

        if getattr(self.host, 'area_center_pos', None):
            ax_x, ax_y = self.host.area_center_pos[0], self.host.area_center_pos[1]
            pa = self.project_point((ax_x, ax_y, 0.0), cam_basis)
            if pa:
                painter.setPen(QPen(QColor("#FF5252"), 2.0))
                painter.drawLine(QPointF(pa[0] - 7, pa[1]), QPointF(pa[0] + 7, pa[1]))
                painter.drawLine(QPointF(pa[0], pa[1] - 7), QPointF(pa[0], pa[1] + 7))
                painter.setFont(QFont("DejaVu Sans", 8))
                painter.drawText(QPointF(pa[0] + 8, pa[1] + 12), "AreaCenter")

        # 4. 绘制三维预览航线 (Preview Trails)
        if getattr(self.host, 'show_preview', True) and getattr(self.host, 'preview_trails', None):
            for uid, points in self.host.preview_trails.items():
                if not points:
                    continue
                preview_led = points[0].get('led') if isinstance(points[0], dict) else None
                p_col_str = self.host._resolve_led_color(preview_led) or "#5DADE2"
                p_col = QColor(p_col_str)
                line_pen = QPen(p_col, 1.6, Qt.DashLine)
                painter.setPen(line_pen)

                coords3 = []
                for p in points:
                    pos_t = p.get('pos', (0.0, 0.0, 0.0))
                    pz = float(pos_t[2]) if len(pos_t) > 2 else 0.0
                    coords3.append((float(pos_t[0]), float(pos_t[1]), pz))

                for i in range(len(coords3) - 1):
                    self._draw_segment_3d(painter, coords3[i], coords3[i + 1], cam_basis)

                # 航点立柱（当 z != 0 时）及起终点标记
                for idx_pt, pt3 in enumerate(coords3):
                    if abs(pt3[2]) > 0.08:
                        stalk_c = QColor(p_col)
                        stalk_c.setAlpha(90)
                        painter.setPen(QPen(stalk_c, 0.9, Qt.DotLine))
                        self._draw_segment_3d(painter, pt3, (pt3[0], pt3[1], 0.0), cam_basis)
                    sp = self.project_point(pt3, cam_basis)
                    if not sp:
                        continue
                    r = 4.2 if idx_pt in (0, len(coords3) - 1) else 2.4
                    painter.setPen(QPen(QColor("#FFFFFF"), 0.8))
                    painter.setBrush(QBrush(p_col))
                    painter.drawEllipse(QPointF(sp[0], sp[1]), r, r)
                    if idx_pt == 0:
                        painter.setPen(p_col)
                        painter.setFont(QFont("DejaVu Sans", 8, QFont.Bold))
                        painter.drawText(QPointF(sp[0] + 6, sp[1] - 4), f"{uid}")

        # 5. 绘制实时尾迹、三维目标向量与按深度排序的异构载具
        default_colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6', '#1abc9c', '#e67e22']
        render_queue = []
        for idx, usv in enumerate(online_usvs):
            uid = str(usv.get('namespace', 'unknown')).lstrip('/')
            pos = usv.get('position', {}) or {}
            ux, uy, uz = float(pos.get('x', 0.0)), float(pos.get('y', 0.0)), float(pos.get('z', 0.0))
            color_hex = self.host._get_led_color_hex(usv.get('led_status')) or default_colors[idx % len(default_colors)]

            # 3D 历史尾迹
            if getattr(self.host, 'show_trails', True):
                trail = self.host.usv_trails.get(uid) or self.host.usv_trails.get(usv.get('namespace', '')) or []
                if len(trail) > 1:
                    tc = QColor(color_hex)
                    tc.setAlpha(155)
                    painter.setPen(QPen(tc, 1.6))
                    step_stride = max(1, len(trail) // 160)
                    sampled = trail[::step_stride]
                    if sampled[-1] != trail[-1]:
                        sampled = list(sampled) + [trail[-1]]
                    for i in range(len(sampled) - 1):
                        p0 = sampled[i]
                        p1 = sampled[i + 1]
                        c0 = (p0[0], p0[1], p0[2] if len(p0) > 2 else 0.0)
                        c1 = (p1[0], p1[1], p1[2] if len(p1) > 2 else 0.0)
                        self._draw_segment_3d(painter, c0, c1, cam_basis)

            # 3D 导航目标与连线
            nav_target = usv.get('nav_target_cache')
            if getattr(self.host, 'show_nav_task', True) and isinstance(nav_target, dict):
                tx, ty = nav_target.get('x'), nav_target.get('y')
                tz = float(nav_target.get('z', 0.0) or 0.0)
                if tx is not None and ty is not None:
                    tx, ty = float(tx), float(ty)
                    reached = bool(nav_target.get('reached', False))
                    t_col = QColor("#2ECC71" if reached else "#FF5252")
                    line_c = QColor(color_hex)
                    line_c.setAlpha(200)
                    painter.setPen(QPen(line_c, 1.5, Qt.DashDotLine))
                    self._draw_segment_3d(painter, (ux, uy, uz), (tx, ty, tz), cam_basis)
                    if abs(tz) > 0.08:
                        stalk_t = QColor(t_col)
                        stalk_t.setAlpha(115)
                        painter.setPen(QPen(stalk_t, 1.0, Qt.DotLine))
                        self._draw_segment_3d(painter, (tx, ty, tz), (tx, ty, 0.0), cam_basis)
                    pt_t = self.project_point((tx, ty, tz), cam_basis)
                    if pt_t:
                        # 菱形三维目标靶标
                        d_r = 6.0
                        diamond = QPolygonF([
                            QPointF(pt_t[0], pt_t[1] - d_r),
                            QPointF(pt_t[0] + d_r, pt_t[1]),
                            QPointF(pt_t[0], pt_t[1] + d_r),
                            QPointF(pt_t[0] - d_r, pt_t[1]),
                        ])
                        painter.setPen(QPen(QColor("#FFFFFF"), 1.2))
                        painter.setBrush(QBrush(t_col))
                        painter.drawPolygon(diamond)
                        painter.setFont(QFont("DejaVu Sans", 8))
                        painter.setPen(t_col.lighter(120))
                        z_str = f", {tz:.1f}m" if abs(tz) > 0.05 else ""
                        painter.drawText(QPointF(pt_t[0] + 8, pt_t[1] + 4), f"🎯({tx:.1f}, {ty:.1f}{z_str})")

            p_u = self.project_point((ux, uy, uz), cam_basis)
            depth = p_u[2] if p_u else 1e6
            render_queue.append((depth, uid, usv, color_hex, p_u))

        # 画家算法：由远及近绘制载具本体与标签
        render_queue.sort(key=lambda item: item[0], reverse=True)
        self._projected_usvs = []
        for depth, uid, usv, color_hex, p_u in render_queue:
            is_sel = (uid == self.selected_usv_id)
            is_leader = (uid in self.host._formation_leader_ids) or (usv.get('namespace') in self.host._formation_leader_ids)
            self._draw_vehicle_3d(painter, usv, color_hex, is_sel, is_leader, cam_basis)
            if p_u:
                self._projected_usvs.append({
                    'id': uid,
                    'sx': p_u[0],
                    'sy': p_u[1],
                    'depth': depth,
                    'usv': usv,
                })
                if getattr(self.host, 'show_labels', True):
                    pos = usv.get('position', {}) or {}
                    uz = float(pos.get('z', 0.0))
                    vt = str(usv.get('vehicle_type', '') or '').strip().lower()
                    vt_icon = {
                        'bionic_airship': '🛸',
                        'bionic_dolphin': '🐬',
                        'flying_sphere': '🔮',
                        'usv': '🚤',
                        'usv_diff': '🚤',
                    }.get(vt, '🚤')
                    alt_tag = f" Z:{uz:+.1f}m" if abs(uz) > 0.08 or vt in ('bionic_airship', 'bionic_dolphin', 'flying_sphere') else ""
                    tag_text = f"{vt_icon} {uid}{alt_tag}"
                    fm = painter.fontMetrics()
                    painter.setFont(QFont("DejaVu Sans", 9, QFont.Bold))
                    tw = fm.horizontalAdvance(tag_text) + 12
                    th = 20
                    bx = p_u[0] - tw * 0.5
                    by = p_u[1] - 28
                    bg_c = QColor(12, 34, 56, 210) if is_sel else QColor(24, 36, 48, 175)
                    bd_c = QColor("#00E5FF") if is_sel else QColor(color_hex)
                    painter.setPen(QPen(bd_c, 1.4 if is_sel else 0.9))
                    painter.setBrush(QBrush(bg_c))
                    painter.drawRoundedRect(QRectF(bx, by, tw, th), 4, 4)
                    painter.setPen(QColor("#FFD700") if is_leader else QColor("#ECF0F1"))
                    painter.drawText(QRectF(bx, by, tw, th), Qt.AlignCenter, tag_text)

        # 6. 左下角紧凑视角与光标 HUD 叠加提示
        hud_bg = QColor(15, 25, 36, 175) if self._is_dark else QColor(255, 255, 255, 195)
        painter.setPen(QPen(QColor(80, 130, 170, 110), 1.0))
        painter.setBrush(QBrush(hud_bg))
        hud_rect = QRectF(10, h - 34, min(w - 20, 560), 26)
        painter.drawRoundedRect(hud_rect, 5, 5)
        painter.setPen(text_fg)
        painter.setFont(QFont("DejaVu Sans", 8))
        hx, hy = self.hover_world_xy
        proj_tag = "正交俯视" if self.ortho_mode else f"3D透视 Az:{self.azimuth_deg:.0f}° El:{self.elevation_deg:.0f}° D:{self.distance:.1f}m"
        hud_str = f" {proj_tag}  |  水面光标 X:{hx:.1f}m Y:{hy:.1f}m  |  左键旋转/点选 · 中键平移 · 滚轮缩放 · 右键下发三维目标"
        painter.drawText(hud_rect, Qt.AlignVCenter | Qt.AlignLeft, hud_str)
        painter.end()

    def _draw_segment_3d(self, painter: QPainter, p0, p1, cam_basis):
        s0 = self.project_point(p0, cam_basis)
        s1 = self.project_point(p1, cam_basis)
        if s0 and s1:
            painter.drawLine(QPointF(s0[0], s0[1]), QPointF(s1[0], s1[1]))

    def _draw_axis_line(self, painter: QPainter, p0, p1, color: QColor, label: str, cam_basis):
        painter.setPen(QPen(color, 2.2))
        s0 = self.project_point(p0, cam_basis)
        s1 = self.project_point(p1, cam_basis)
        if s0 and s1:
            painter.drawLine(QPointF(s0[0], s0[1]), QPointF(s1[0], s1[1]))
            painter.setFont(QFont("DejaVu Sans", 8, QFont.Bold))
            painter.drawText(QPointF(s1[0] + 4, s1[1] - 3), label)

    # -------------------- 鼠标交互：旋转 / 平移 / 缩放 / 点选 / 右键三维目标 --------------------
    def mousePressEvent(self, event):
        self._press_pos = event.pos()
        self._last_pos = event.pos()
        self._moved_px = 0.0
        if event.button() == Qt.LeftButton:
            if event.modifiers() & Qt.ShiftModifier:
                self._drag_mode = 'pan'
            else:
                self._drag_mode = 'orbit'
        elif event.button() == Qt.MiddleButton:
            self._drag_mode = 'pan'
        elif event.button() == Qt.RightButton:
            self._drag_mode = 'pan_or_menu'

    def mouseMoveEvent(self, event):
        pos = event.pos()
        wx, wy, _ = self.screen_to_water_plane(pos.x(), pos.y(), 0.0)
        self.hover_world_xy = (wx, wy)

        if self._drag_mode and self._last_pos is not None:
            dx = pos.x() - self._last_pos.x()
            dy = pos.y() - self._last_pos.y()
            self._moved_px += sqrt(dx * dx + dy * dy)
            self._last_pos = pos

            if self._drag_mode == 'orbit':
                self.ortho_mode = False
                self.azimuth_deg = (self.azimuth_deg - dx * 0.42) % 360.0
                if self.azimuth_deg > 180.0:
                    self.azimuth_deg -= 360.0
                self.elevation_deg = max(-75.0, min(89.5, self.elevation_deg + dy * 0.35))
                self.update()
                return

            if self._drag_mode in ('pan', 'pan_or_menu') and self._moved_px > 4.0:
                self.follow_selected = False
                _, _, right, up = self._camera_basis()
                pan_speed = self.distance * 0.0016
                self.center = self.center - right * (dx * pan_speed) + up * (dy * pan_speed)
                self.update()
                return

        self.update()

    def mouseReleaseEvent(self, event):
        pos = event.pos()
        was_click = self._moved_px <= 5.0
        mode = self._drag_mode
        self._drag_mode = None
        self._last_pos = None

        if was_click and event.button() == Qt.LeftButton:
            hit = self._pick_usv(pos.x(), pos.y())
            if hit:
                self.selected_usv_id = hit['id']
                if callable(getattr(self.host, 'usv_selected_callback', None)):
                    self.host.usv_selected_callback(hit['id'], hit['usv'])
            self.update()
        elif was_click and (event.button() == Qt.RightButton or mode == 'pan_or_menu'):
            self._open_3d_context_menu(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            hit = self._pick_usv(event.pos().x(), event.pos().y())
            if hit:
                self.host.show_usv_details(hit['usv'])
            else:
                self.follow_selected = False
                self.fit_scene()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = 0.86 if delta > 0 else 1.16
        self.distance = max(3.5, min(1500.0, self.distance * factor))
        self.update()

    def _pick_usv(self, sx: float, sy: float):
        best = None
        best_d = 28.0
        for item in self._projected_usvs:
            d = sqrt((sx - item['sx']) ** 2 + (sy - item['sy']) ** 2)
            if d < best_d:
                best_d = d
                best = item
        return best

    def _open_3d_context_menu(self, event):
        wx, wy, _ = self.screen_to_water_plane(event.pos().x(), event.pos().y(), 0.0)
        hit = self._pick_usv(event.pos().x(), event.pos().y())
        if hit:
            self.selected_usv_id = hit['id']
            if callable(getattr(self.host, 'usv_selected_callback', None)):
                self.host.usv_selected_callback(hit['id'], hit['usv'])

        menu = QMenu(self)
        act_send_3d = QAction(f"🎯 发送三维目标点到此位置 ({wx:.1f}, {wy:.1f}, Z...)", self)
        act_send_3d.triggered.connect(lambda: self.host.open_send_3d_target_dialog((wx, wy, 0.0), self.selected_usv_id))
        menu.addAction(act_send_3d)

        act_fill_dep = QAction(f"📍 填入离群三维坐标框 ({wx:.1f}, {wy:.1f})", self)
        act_fill_dep.triggered.connect(lambda: self.host.fill_departed_spinboxes(wx, wy, 0.0))
        menu.addAction(act_fill_dep)

        if hit:
            menu.addSeparator()
            act_detail = QAction(f"📋 查看 {hit['id']} 三维状态详情", self)
            act_detail.triggered.connect(lambda: self.host.show_usv_details(hit['usv']))
            menu.addAction(act_detail)

            act_follow = QAction(f"{'✅ ' if self.follow_selected else ''}🎥 视角跟随 {hit['id']}", self)
            def _toggle_follow():
                self.follow_selected = not self.follow_selected
                self.update()
            act_follow.triggered.connect(_toggle_follow)
            menu.addAction(act_follow)

        menu.addSeparator()
        act_home = QAction(f"🏠 设置此处为 Home 点 ({wx:.1f}, {wy:.1f})", self)
        act_home.triggered.connect(lambda: self.host.set_home_position(wx, wy))
        menu.addAction(act_home)

        act_fit = QAction("🔍 自适应全局视野", self)
        act_fit.triggered.connect(self.fit_scene)
        menu.addAction(act_fit)

        menu.exec_(event.globalPos())


class UsvPlotWindow(QWidget):
    def __init__(self, get_usv_list_func, parent=None):
        super().__init__(parent)
        # self.setWindowTitle("USV 2D Position & Navigation Display")
        # self.resize(1000, 750)
        
        # Explicitly set to NonModal to prevent blocking the main window
        # self.setWindowModality(Qt.NonModal)
        
        # Enable Min/Max buttons and resize
        # self.setWindowFlags(self.windowFlags() | Qt.WindowMinMaxButtonsHint)
        
        # Styles
        # Initial call to set default theme (Dark)
        self.set_theme('modern_dark')
        
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(0)
        main_layout.setContentsMargins(0, 0, 0, 0)

        # ========== Visibility Flags (Controls removed from UI) ==========
        self.show_labels = True
        self.show_trails = True
        self.show_nav_task = True
        self.show_preview = True
        self.show_grid = True
        self.auto_scale = True
        
        # Position Trackers
        self.home_pos = (0.0, 0.0)
        self.area_center_pos = None
        self.get_usv_list_func = get_usv_list_func
        self.usv_trails = {}      # {usv_id: [(x, y, z), ...]}
        self.preview_trails = {}  # {usv_id: [{'pos': (x, y, z), ...}, ...]}
        self.max_trail_length = 500

        # Callbacks for MainWindow integration
        self.request_set_home_callback = None
        self.request_send_3d_target_callback = None
        self.request_fill_departed_callback = None
        self.usv_selected_callback = None

        # ========== Top 3D / 2D Situational Control Bar ==========
        ctrl_frame = QFrame(self)
        ctrl_layout = QHBoxLayout(ctrl_frame)
        ctrl_layout.setContentsMargins(8, 4, 8, 4)
        ctrl_layout.setSpacing(6)

        self.btn_mode_3d = QPushButton("🌐 3D 立体态势")
        self.btn_mode_3d.setCheckable(True)
        self.btn_mode_3d.setChecked(True)
        self.btn_mode_3d.clicked.connect(lambda: self.set_view_mode('3d'))
        ctrl_layout.addWidget(self.btn_mode_3d)

        self.btn_mode_2d = QPushButton("🗺️ 2D 平面视图")
        self.btn_mode_2d.setCheckable(True)
        self.btn_mode_2d.clicked.connect(lambda: self.set_view_mode('2d'))
        ctrl_layout.addWidget(self.btn_mode_2d)

        # 3D 预设相机按钮组
        self._cam_btn_container = QWidget(self)
        cam_layout = QHBoxLayout(self._cam_btn_container)
        cam_layout.setContentsMargins(4, 0, 4, 0)
        cam_layout.setSpacing(4)

        self.btn_preset_iso = QPushButton("斜视")
        self.btn_preset_iso.setToolTip("恢复标准 3D 斜视透视角")
        self.btn_preset_iso.clicked.connect(lambda: self.canvas_3d.set_view_preset('3d'))
        cam_layout.addWidget(self.btn_preset_iso)

        self.btn_preset_top = QPushButton("俯视XY")
        self.btn_preset_top.setToolTip("正交俯视水面 (XY)")
        self.btn_preset_top.clicked.connect(lambda: self.canvas_3d.set_view_preset('top'))
        cam_layout.addWidget(self.btn_preset_top)

        self.btn_preset_xz = QPushButton("正视XZ")
        self.btn_preset_xz.setToolTip("观察东西-高度分层剖面 (XZ)")
        self.btn_preset_xz.clicked.connect(lambda: self.canvas_3d.set_view_preset('front_xz'))
        cam_layout.addWidget(self.btn_preset_xz)

        self.btn_preset_fit = QPushButton("🔍 全景")
        self.btn_preset_fit.setToolTip("自适应包含所有在线载具与三维航线")
        self.btn_preset_fit.clicked.connect(lambda: self.canvas_3d.fit_scene())
        cam_layout.addWidget(self.btn_preset_fit)

        ctrl_layout.addWidget(self._cam_btn_container)

        self.btn_quick_3d_goal = QPushButton("🎯 下发三维目标")
        self.btn_quick_3d_goal.setToolTip("直接向选中或指定个体发送 (X, Y, Z, Yaw) 三维导航目标")
        self.btn_quick_3d_goal.clicked.connect(lambda: self.open_send_3d_target_dialog())
        ctrl_layout.addWidget(self.btn_quick_3d_goal)

        ctrl_layout.addStretch(1)

        self.info_label = QLabel("Waiting for data...")
        ctrl_layout.addWidget(self.info_label)
        main_layout.addWidget(ctrl_frame)

        # ========== Stacked View: [0] 3D Situational Canvas, [1] 2D Matplotlib ==========
        self.view_stack = QStackedWidget(self)
        self.canvas_3d = Usv3DCanvas(self, self)
        self.view_stack.addWidget(self.canvas_3d)

        page_2d = QWidget(self)
        layout_2d = QVBoxLayout(page_2d)
        layout_2d.setContentsMargins(0, 0, 0, 0)
        layout_2d.setSpacing(0)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.toolbar = NavigationToolbar2QT(self.canvas, page_2d)
        self.toolbar.setMaximumHeight(35)
        layout_2d.addWidget(self.toolbar)
        layout_2d.addWidget(self.canvas, stretch=1)
        self.view_stack.addWidget(page_2d)

        main_layout.addWidget(self.view_stack, stretch=1)
        self.view_mode = '3d'

        # ========== Graphics Objects Cache (Optimization) ==========
        self.ax = self.figure.add_subplot(111)
        self._updating_limits = False  # Flag to distinguish programmatic updates
        self.ax.callbacks.connect('xlim_changed', self.on_limits_changed)
        self.ax.callbacks.connect('ylim_changed', self.on_limits_changed)
        
        self.artists = {
            'usv_polys': {},    # {usv_id: Polygon (数据坐标1:1, grid≤2m时使用)}
            'usv_scatters': {}, # {usv_id: PathCollection (scatter, grid>2m时使用)}
            'usv_labels': {},   # {usv_id: Text}
            'usv_trails': {},   # {usv_id: Line2D}
            'nav_lines': {},    # {usv_id: Line2D}
            'nav_markers': {},  # {usv_id: Line2D (marker)}
            'nav_labels': {},   # {usv_id: Text (target coordinates)}
            'nav_sync_markers': {}, # {usv_id: Text ('=' for sync/wait)}
            'preview_lines': {}, # {usv_id: Line2D}
            'preview_markers_start': {}, # {usv_id: PathCollection}
            'preview_markers_end': {},   # {usv_id: PathCollection}
            'preview_markers_mid': {},   # {usv_id: PathCollection}
            'preview_markers_maneuver': {}, # {usv_id: PathCollection} (Rings for maneuvers)
            'preview_labels_start': {},  # {usv_id: Text (USV ID at start)}
            'preview_sync_texts': {},    # {usv_id: [Text, ...] for '=' at sync points}
            'home_marker': None, # Home marker aritst
            'area_center_marker': None # Area Center marker (Red Cross)
        }

        # Ensure 1:1 plot scaling is active from startup
        self._init_axes()

        # Keep enough room for XY ticks/labels while maximizing plot area
        self.figure.subplots_adjust(left=0.09, right=0.995, bottom=0.10, top=0.95)
        
        # 创建自定义USV船形顶点 (数据坐标, 1:1 真实尺寸)
        # USV 实际尺寸: 长度 0.6m, 宽度 0.3m (与 Gazebo 0.60m x 0.30m 单体船严格一致)
        # 船形比例: 船头尖端 + 后部凹陷, 指向 x 正方向 (yaw=0)
        self._usv_length = 0.6   # 米
        self._usv_width = 0.3    # 米
        # 基础顶点 (以重心为原点, 单位: 米)
        # 船头在 +x 方向, 左舷在 +y 方向
        L, W = self._usv_length, self._usv_width
        self._usv_base_verts = np.array([
            [ L * 0.5,      0.0],         # 船头
            [-L * 0.35,  W * 0.5],        # 左后角
            [-L * 0.15,     0.0],         # 尾部凹陷
            [-L * 0.35, -W * 0.5],        # 右后角
        ])  # shape: (4, 2)
        # 旧箭头路径 (用于 scatter 固定像素模式, grid>2m时使用)
        arrow_verts = [
            (1.0, 0.0), (-0.8, 0.6), (-0.4, 0.0), (-0.8, -0.6), (1.0, 0.0),
        ]
        arrow_codes = [MplPath.MOVETO, MplPath.LINETO, MplPath.LINETO, MplPath.LINETO, MplPath.CLOSEPOLY]
        self._usv_arrow_path = MplPath(arrow_verts, arrow_codes)
        # 渲染模式: 'polygon' (1:1真实尺寸) 或 'scatter' (固定像素)
        self._usv_render_mode = None  # 初始未定, 首帧时决定
        
        # Apply theme to axes
        self.set_theme('modern_dark')

        # ========== 编队信息 ==========
        self._formation_leader_ids = set()    # 编队领队 ID 集合
        self._formation_follower_ids = set()  # 编队跟随者 ID 集合
        self._formation_rings = {}  # {usv_id: artist} 领队外圈artist缓存

        # ========== Timer ==========
        # Update at 5Hz to prevent blocking main thread
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(200) # 200ms

        # Interaction State
        self.is_panning = False
        self.last_mouse_pos = None

        # Signals
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.canvas.mpl_connect('button_release_event', self.on_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_move)

        self._center_on_screen()
        # Track last data update to avoid excessive plotting
        self.last_update_time = time.time()

    def set_theme(self, theme_name):
        """Set the theme for the plot window (Dark/Light)"""
        is_dark = (theme_name == 'modern_dark')
        
        if is_dark:
            # Colors for Dark Mode
            bg_color = "#2b2b2b" # Window BG
            fg_color = "#ecf0f1" # Text Color
            group_bg = "#3a3a3a"
            button_bg = "#3498db"
            button_fg = "#ffffff"
            
            # Matplotlib Colors
            mpl_fig_bg = '#2b2b2b'
            mpl_ax_bg = '#1e1e1e'
            mpl_text = '#ecf0f1'
            mpl_grid = '#7f8c8d'
            mpl_toolbar_bg = "#34495e"
            home_color = '#ffffff'  # White home in dark mode
            home_edge = '#000000'
        else:
            # Colors for Light Mode
            bg_color = "#f5f5f5" # Window BG
            fg_color = "#2c3e50" # Text Color
            group_bg = "#ffffff"
            button_bg = "#3498db"
            button_fg = "#ffffff"
            
            # Matplotlib Colors
            mpl_fig_bg = '#f5f5f5'
            mpl_ax_bg = '#ffffff'
            mpl_text = '#2c3e50'
            mpl_grid = '#bdc3c7'
            mpl_toolbar_bg = "#e0e0e0"
            home_color = '#333333' # Dark grey home in light mode
            home_edge = '#ffffff'

        self.home_marker_style = {'facecolor': home_color, 'edgecolor': home_edge, 'linewidth': 1}
        
        # 1. Qt Widget Stylesheet
        style_sheet = f"""
            QWidget {{
                background-color: {bg_color};
                color: {fg_color};
            }}
            QGroupBox {{
                background-color: {group_bg};
                border: 2px solid {mpl_grid};
                border-radius: 6px;
                margin-top: 15px;
                padding: 15px;
                font-weight: bold;
                color: {fg_color};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0px 8px 0px 8px;
                background-color: {bg_color};
                color: {fg_color};
            }}
            QCheckBox {{
                background-color: transparent;
                color: {fg_color};
                font-size: 14px;
                spacing: 5px;
            }}
            QPushButton {{
                background-color: {button_bg};
                color: {button_fg};
                border: none;
                padding: 8px 15px;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #2980b9;
            }}
            QLabel {{
                background-color: transparent;
                color: {fg_color};
                font-size: 14px;
            }}
        """
        self.setStyleSheet(style_sheet)
        
        # Update Info Label specifically if needed
        if hasattr(self, 'info_label'):
             self.info_label.setStyleSheet(f"color: {fg_color}; font-weight: bold;")

        # Update Toolbar
        if hasattr(self, 'toolbar'):
             self.toolbar.setStyleSheet(f"background-color: {mpl_toolbar_bg}; border: none; color: {fg_color};")

        if hasattr(self, 'canvas_3d'):
             self.canvas_3d.set_dark_mode(is_dark)
             self._refresh_mode_btn_styles(is_dark)
        
        # 2. Matplotlib Figure & Axes
        if hasattr(self, 'figure'):
            self.figure.patch.set_facecolor(mpl_fig_bg)
        
        if hasattr(self, 'ax'):
            self.ax.set_facecolor(mpl_ax_bg)
            self.ax.set_title("USV Fleet Overview (Live)", color=mpl_text)
            self.ax.set_xlabel('X (m)', color=mpl_text)
            self.ax.set_ylabel('Y (m)', color=mpl_text)
            
            # Grid
            self.ax.grid(True, linestyle='--', alpha=0.3, color=mpl_grid)
            
            # Spines
            for spine in self.ax.spines.values():
                spine.set_color(mpl_text)
            
            # Ticks
            self.ax.tick_params(axis='x', colors=mpl_text)
            self.ax.tick_params(axis='y', colors=mpl_text)
            
            # Redraw only when 2D canvas is active and has non-zero geometry
            if hasattr(self, 'canvas'):
                self._draw_home_marker()
                if getattr(self, 'view_mode', '3d') == '2d' and self.canvas.width() > 10 and self.canvas.height() > 10:
                    self.canvas.draw_idle()

    def _refresh_mode_btn_styles(self, is_dark: bool = True):
        active_style = (
            "QPushButton { background-color: #16a085; color: #ffffff; border: 1px solid #1abc9c; "
            "padding: 4px 10px; border-radius: 4px; font-size: 13px; font-weight: bold; }"
        )
        inactive_bg = "#34495e" if is_dark else "#dce4ec"
        inactive_fg = "#ecf0f1" if is_dark else "#2c3e50"
        inactive_style = (
            f"QPushButton {{ background-color: {inactive_bg}; color: {inactive_fg}; border: none; "
            f"padding: 4px 10px; border-radius: 4px; font-size: 13px; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #2980b9; color: #ffffff; }}"
        )
        sub_style = (
            f"QPushButton {{ background-color: {inactive_bg}; color: {inactive_fg}; border: none; "
            f"padding: 3px 8px; border-radius: 4px; font-size: 12px; }}"
            f"QPushButton:hover {{ background-color: #3498db; color: #ffffff; }}"
        )
        goal_style = (
            "QPushButton { background-color: #e67e22; color: #ffffff; border: none; "
            "padding: 4px 10px; border-radius: 4px; font-size: 13px; font-weight: bold; }"
            "QPushButton:hover { background-color: #d35400; }"
        )
        is_3d = getattr(self, 'view_mode', '3d') == '3d'
        if hasattr(self, 'btn_mode_3d'):
            self.btn_mode_3d.setStyleSheet(active_style if is_3d else inactive_style)
        if hasattr(self, 'btn_mode_2d'):
            self.btn_mode_2d.setStyleSheet(inactive_style if is_3d else active_style)
        for attr in ('btn_preset_iso', 'btn_preset_top', 'btn_preset_xz', 'btn_preset_fit'):
            btn = getattr(self, attr, None)
            if btn:
                btn.setStyleSheet(sub_style)
        if hasattr(self, 'btn_quick_3d_goal'):
            self.btn_quick_3d_goal.setStyleSheet(goal_style)

    def set_view_mode(self, mode: str):
        self.view_mode = '2d' if str(mode).lower() == '2d' else '3d'
        is_3d = (self.view_mode == '3d')
        if hasattr(self, 'btn_mode_3d'):
            self.btn_mode_3d.setChecked(is_3d)
        if hasattr(self, 'btn_mode_2d'):
            self.btn_mode_2d.setChecked(not is_3d)
        if hasattr(self, '_cam_btn_container'):
            self._cam_btn_container.setVisible(is_3d)
        if hasattr(self, 'view_stack'):
            self.view_stack.setCurrentIndex(0 if is_3d else 1)
        self._refresh_mode_btn_styles(getattr(self.canvas_3d, '_is_dark', True))
        self.update_plot()

    def open_send_3d_target_dialog(self, default_xyz=(0.0, 0.0, 0.0), preselected_id=None):
        usv_list = self.get_usv_list_func() or []
        sel_id = preselected_id or getattr(self.canvas_3d, 'selected_usv_id', None)
        dlg = ThreeDTargetDialog(usv_list, default_xyz=default_xyz, preselected_id=sel_id, parent=self)
        if dlg.exec_() == QDialog.Accepted:
            payload = dlg.get_payload()
            if payload.get('usv_ids') and callable(self.request_send_3d_target_callback):
                self.request_send_3d_target_callback(payload)

    def fill_departed_spinboxes(self, x: float, y: float, z: float = 0.0):
        if callable(self.request_fill_departed_callback):
            self.request_fill_departed_callback(float(x), float(y), float(z))

    def _init_axes(self):
        """Initialize fixed axes properties."""
        # Keep 1:1 metric scale (no shape distortion)
        self.ax.set_aspect('equal', adjustable='datalim')
        # Detailed styling is now handled in set_theme()
        
        # Draw initial home marker
        self._draw_home_marker()

    def _enforce_equal_aspect_fill(self):
        """Keep x/y metric scale at 1:1 while filling the available axes box."""
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        data_w = float(xlim[1] - xlim[0])
        data_h = float(ylim[1] - ylim[0])
        if data_w <= 0.0 or data_h <= 0.0:
            return

        bbox = self.ax.get_window_extent()
        view_w = float(bbox.width)
        view_h = float(bbox.height)
        if view_w <= 0.0 or view_h <= 0.0:
            return

        target_ratio = view_w / view_h
        current_ratio = data_w / data_h

        cx = 0.5 * (xlim[0] + xlim[1])
        cy = 0.5 * (ylim[0] + ylim[1])

        self._updating_limits = True
        if current_ratio < target_ratio:
            new_w = data_h * target_ratio
            half_w = 0.5 * new_w
            self.ax.set_xlim(cx - half_w, cx + half_w)
        elif current_ratio > target_ratio:
            new_h = data_w / target_ratio
            half_h = 0.5 * new_h
            self.ax.set_ylim(cy - half_h, cy + half_h)
        self._updating_limits = False
        
    def _draw_home_marker(self):
        """Draw or update the Home marker at current home position"""
        # Define House Icon Path (Triangle on Top, Square on bottom)
        # Vertices for a simple house shape centered at (0,0)
        # Scale can be adjusted with 's' parameter in scatter, but path vertices are relative
        vertices = [
            (-1, -1), (1, -1), (1, 0.5), (0, 1.5), (-1, 0.5), (0,0) # Close polygon technically not needed if filled, but good practice
        ]
        codes = [
            patches.Path.MOVETO,
            patches.Path.LINETO,
            patches.Path.LINETO,
            patches.Path.LINETO,
            patches.Path.LINETO,
            patches.Path.CLOSEPOLY
        ]
        house_path = patches.Path(vertices, codes)
        
        # Remove old marker if exists
        if self.artists['home_marker']:
            self.artists['home_marker'].remove()
            self.artists['home_marker'] = None
            
        style = getattr(self, 'home_marker_style', {'facecolor': 'white', 'edgecolor': 'black', 'linewidth': 1})
        
        # Draw new marker using Scatter with custom path
        # s=150 is the size area in points^2
        self.artists['home_marker'] = self.ax.scatter(
            [self.home_pos[0]], [self.home_pos[1]], 
            marker=house_path, 
            s=200, 
            facecolor=style['facecolor'],
            edgecolor=style['edgecolor'],
            linewidth=style['linewidth'],
            zorder=3, # Below USVs but above grid
            label='Home'
        )

    def set_formation_info(self, leader_ids: list, follower_ids: list):
        """
        设置编队信息，用于在绘图中区分领队和跟随者
        
        领队标记会比普通USV大一圈，并带有金色外环。
        
        Args:
            leader_ids: 领队 USV ID 列表（空列表表示清除编队信息）
            follower_ids: 跟随者 USV ID 列表
        """
        self._formation_leader_ids = set(leader_ids) if leader_ids else set()
        self._formation_follower_ids = set(follower_ids) if follower_ids else set()
        # 清除旧的外圈标记
        for uid, ring in self._formation_rings.items():
            try:
                ring.remove()
            except Exception:
                pass
        self._formation_rings.clear()

    def draw_area_center_marker(self, x, y):
        """Draw or update the Area Center marker (Red Cross)"""
        # Update stored position for auto-scale logic
        self.area_center_pos = (x, y)

        # Remove old marker if exists
        if self.artists['area_center_marker']:
            self.artists['area_center_marker'].remove()
            self.artists['area_center_marker'] = None
            
        # Draw new marker (Red Cross)
        # Using zorder=4 to be above grid/home but below USVs
        self.artists['area_center_marker'] = self.ax.scatter(
            [x], [y], 
            marker='+', 
            s=150, 
            c='red', 
            linewidth=2.0, 
            zorder=4, 
            label='Area Center'
        )
        self.canvas.draw_idle()

    def set_preview_path(self, task_data_list, offset=(0.0, 0.0), angle=0.0):
        """Set task data for preview
        Args:
            task_data_list: List of task steps
            offset: (x, y) offset to add to task coordinates (e.g. area_center)
            angle: Rotation angle in degrees (0-359), rotating around the Area Center origin (before offset translation if using local coords assumption, or after?)
                   Usually: Global = R * Local + Offset
        """
        # print(f"DEBUG: set_preview_path called with {len(task_data_list)} items, offset={offset}, angle={angle}")
        self.preview_trails = {}
        # Clear previous preview artists
        for key in ['preview_lines', 'preview_markers_start', 'preview_markers_end', 'preview_markers_mid', 'preview_markers_maneuver', 'preview_labels_start', 'preview_sync_texts']:
            # preview_sync_texts may store lists
            val = self.artists.get(key, {})
            if isinstance(val, dict):
                for artist in val.values():
                    # some values may be lists of artists
                    if isinstance(artist, list):
                        for a in artist:
                            try:
                                a.remove()
                            except Exception:
                                pass
                    else:
                        try:
                            artist.remove()
                        except Exception:
                            pass
                val.clear()
            
        if not task_data_list:
            self.update_plot()
            return

        # Prepare rotation
        theta = radians(angle)
        cos_t = cos(theta)
        sin_t = sin(theta)

        # Ensure autoscaling is active on new task load
        # self.auto_scale = True 
        # (Already done in main block below, but ensuring reset is good)

        # Group by USV
        temp = {}
        for item in task_data_list:
            uid = item.get('usv_id')
            step = item.get('step', 0)
            
            # Extract position (handle nested dictionary structure from ClusterTaskManager)
            x_local = 0.0
            y_local = 0.0
            z_local = 0.0
            
            if 'position' in item and isinstance(item['position'], dict):
                x_local = float(item['position'].get('x', 0.0))
                y_local = float(item['position'].get('y', 0.0))
                z_local = float(item['position'].get('z', 0.0))
            else:
                x_local = float(item.get('x', 0.0))
                y_local = float(item.get('y', 0.0))
                z_local = float(item.get('z', 0.0))
            
            # Apply Rotation (around local 0,0)
            x_rot = x_local * cos_t - y_local * sin_t
            y_rot = x_local * sin_t + y_local * cos_t
            
            # Apply Offset (Translate to Area Center global position, preserving Z)
            ox = float(offset[0]) if len(offset) > 0 else 0.0
            oy = float(offset[1]) if len(offset) > 1 else 0.0
            oz = float(offset[2]) if len(offset) > 2 else 0.0
            x_final = x_rot + ox
            y_final = y_rot + oy
            z_final = z_local + oz
            
            # Extract Maneuver info
            m_type = item.get('maneuver_type', 0)
            
            if uid not in temp:
                temp[uid] = []
            # Preserve relevant metadata such as led color and nav_mode for preview rendering
            temp[uid].append({
                'step': step,
                'pos': (x_final, y_final, z_final),
                'maneuver_type': m_type,
                'led': item.get('led') if isinstance(item, dict) else None,
                'nav_mode': (item.get('nav_mode') or 0) if isinstance(item, dict) else 0,
                'usv_id': uid
            })
            
        # Sort and store
        for uid, items in temp.items():
            items.sort(key=lambda x: x['step'])
            # Store full item dict to preserve maneuver info
            self.preview_trails[uid] = items 
        
        if hasattr(self, 'canvas_3d'):
            self.canvas_3d.fit_scene()
        self.update_plot()

    def request_redraw(self):
        # Update grid visibility
        self.ax.grid(self.show_grid)
        self.update_plot()

    def _get_led_color_hex(self, led_info):
        """Convert LED info {'color': [r,g,b]} to hex string."""
        if not led_info or 'color' not in led_info:
            return None
        try:
            rgb = led_info['color']
            
            # 视觉优化：避免纯黑色在深色背景下隐身
            # 如果亮度过低（接近黑色），强制提亮为深灰色
            if len(rgb) == 3:
                if sum(rgb) < 30: # 阈值判定
                    rgb = [80, 80, 80] # 提亮为灰色
                    
                return '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        except:
            pass
        return None

    def _resolve_led_color(self, led_val):
        """Resolve various LED value formats to a matplotlib-friendly color string.
        Handles dict {'color':[r,g,b]}, '#rrggbb', 'r,g,b', 'color_select|r,g,b',
        and treats 'off'/'black' as contrast color against axes background.
        """
        if not led_val:
            return None

        def _contrast_color_against_axes():
            try:
                rgba = self.ax.get_facecolor()
                r, g, b = rgba[0], rgba[1], rgba[2]
                lum = 0.299 * r + 0.587 * g + 0.114 * b
                return '#ffffff' if lum < 0.5 else '#000000'
            except Exception:
                return '#ffffff'

        # dict format
        if isinstance(led_val, dict) and 'color' in led_val:
            parsed = self._get_led_color_hex(led_val)
            if not parsed:
                return _contrast_color_against_axes()
            try:
                rr = int(parsed[1:3], 16) / 255.0
                gg = int(parsed[3:5], 16) / 255.0
                bb = int(parsed[5:7], 16) / 255.0
                lum = 0.299 * rr + 0.587 * gg + 0.114 * bb
                if lum < 0.05:
                    return _contrast_color_against_axes()
            except Exception:
                pass
            return parsed

        try:
            s = str(led_val).strip()
            if '|' in s:
                s = s.split('|')[-1].strip()
            if s.lower() in ('off', 'none', '0', 'false', 'disable', 'disabled'):
                return _contrast_color_against_axes()
            if s.startswith('#'):
                if s.lower() in ('#000000', '#000'):
                    return _contrast_color_against_axes()
                return s
            if ',' in s:
                parts = [p.strip() for p in s.split(',') if p.strip()]
                if len(parts) >= 3:
                    try:
                        r, g, b = int(parts[0]), int(parts[1]), int(parts[2])
                        if r == 0 and g == 0 and b == 0:
                            return _contrast_color_against_axes()
                        return '#{:02x}{:02x}{:02x}'.format(max(0,min(255,r)), max(0,min(255,g)), max(0,min(255,b)))
                    except Exception:
                        pass
            if s.lower() == 'black':
                return _contrast_color_against_axes()
            return s
        except Exception:
            return None

    def on_limits_changed(self, event=None):
        """Callback for axis limit changes (zoom/pan)"""
        if not self._updating_limits and self.auto_scale:
            self.auto_scale = False
            
    def update_plot(self):
        # 如果控件被隐藏或其父窗口被隐藏，则不刷新（节省资源）
        if not self.isVisible() and not self.parentWidget():
           return
        # 嵌入式模式下，如果所在 Tab 没有显示，也可以跳过（可选优化）
        
        usv_list = self.get_usv_list_func()
        
        # Blink state (0 or 1)
        blink_on = (int(time.time() * 2) % 2) == 0  # 2Hz blink

        # === NO CLEARING FIGURE ===
        # We reuse 'self.ax' and update artists

        all_x = []
        all_y = []
        
        # Include Fixed Points in Auto Scale (Home & Area Center)
        if hasattr(self, 'home_pos') and self.home_pos:
            all_x.append(self.home_pos[0])
            all_y.append(self.home_pos[1])
            
        if hasattr(self, 'area_center_pos') and self.area_center_pos:
            all_x.append(self.area_center_pos[0])
            all_y.append(self.area_center_pos[1])
        
        # Default colors
        default_colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6']

        self.active_usv_points = [] # For click detection

        # --- Draw Preview Trails (Plan) First ---
        if self.show_preview and hasattr(self, 'preview_trails'):
            for uid, points in self.preview_trails.items():
                if len(points) > 0:
                    # points is list of dicts: {'pos': (x,y), ...}
                    px = [p['pos'][0] for p in points]
                    py = [p['pos'][1] for p in points]
                    
                    plain_x = list(px)
                    plain_y = list(py)
                    all_x.extend(plain_x)
                    all_y.extend(plain_y)

                    # 1. Dashed Line
                    if uid not in self.artists['preview_lines']:
                        # Zorder 5 (above grid/home/area_center)
                        line, = self.ax.plot(px, py, color='#aaaaaa', alpha=0.6, linestyle='--', linewidth=1.0, zorder=5)
                        self.artists['preview_lines'][uid] = line
                    else:
                        self.artists['preview_lines'][uid].set_data(px, py)
                        # Ensure visibility and style
                        self.artists['preview_lines'][uid].set_color('#aaaaaa')
                        self.artists['preview_lines'][uid].set_linestyle('--')
                        self.artists['preview_lines'][uid].set_linewidth(1.0)
                        self.artists['preview_lines'][uid].set_zorder(5)
                        self.artists['preview_lines'][uid].set_visible(True)
                    
                    # 2. Start/End Points
                    # If a led color is provided in the preview data, use it for the markers
                    preview_led = points[0].get('led') if isinstance(points[0], dict) else None
                    preview_color = self._resolve_led_color(preview_led) or '#2ecc71'
                    end_color = preview_color or '#e74c3c'

                    if uid not in self.artists['preview_markers_start']:
                        # Zorder increased to 6
                        start = self.ax.scatter([px[0]], [py[0]], c=preview_color, s=30, marker='o', zorder=6, edgecolors='white', linewidth=0.5, label='Start')
                        end = self.ax.scatter([px[-1]], [py[-1]], c=preview_color, s=30, marker='o', zorder=6, edgecolors='white', linewidth=0.5, label='End')
                        self.artists['preview_markers_start'][uid] = start
                        self.artists['preview_markers_end'][uid] = end
                        
                        # Add USV ID Label at Start
                        text_artist = self.ax.text(px[0], py[0], f" {uid}", fontsize=9, color=preview_color, verticalalignment='bottom', horizontalalignment='left', zorder=7, fontweight='bold')
                        self.artists['preview_labels_start'][uid] = text_artist
                        
                        if len(points) > 2:
                            mid = self.ax.scatter(px[1:-1], py[1:-1], c=preview_color, s=10, marker='.', zorder=6, edgecolors='none')
                            self.artists['preview_markers_mid'][uid] = mid
                        # Create '=' markers for any sync nav_mode points in preview
                        sync_points = [p for p in points if int(p.get('nav_mode') or 0) == 1]
                        sync_texts = []
                        # create transform with 3 pixel offset to the right-top
                        base_transform = offset_copy(self.ax.transData, fig=self.figure, x=5, y=5, units='dots')
                        for sp in sync_points:
                            sx, sy = sp['pos'][0], sp['pos'][1]
                            txt = self.ax.text(sx, sy, '=', fontsize=12, color=preview_color if preview_color else '#f1c40f', ha='center', va='center', zorder=8, fontweight='bold', transform=base_transform)
                            sync_texts.append(txt)
                        if sync_texts:
                            self.artists['preview_sync_texts'][uid] = sync_texts
                    else:
                        self.artists['preview_markers_start'][uid].set_visible(True)
                        self.artists['preview_markers_end'][uid].set_visible(True)
                        self.artists['preview_markers_start'][uid].set_zorder(6)
                        self.artists['preview_markers_end'][uid].set_zorder(6)
                        
                        # Update Start Label
                        if uid in self.artists['preview_labels_start']:
                            self.artists['preview_labels_start'][uid].set_position((px[0], py[0]))
                            self.artists['preview_labels_start'][uid].set_visible(True)
                            self.artists['preview_labels_start'][uid].set_color(preview_color)
                        else:
                            # Create if missing (defensive)
                            text_artist = self.ax.text(px[0], py[0], f" {uid}", fontsize=9, color=preview_color, verticalalignment='bottom', horizontalalignment='left', zorder=7, fontweight='bold')
                            self.artists['preview_labels_start'][uid] = text_artist
                        
                        if uid in self.artists['preview_markers_mid']:
                            self.artists['preview_markers_mid'][uid].set_visible(True)
                            self.artists['preview_markers_mid'][uid].set_zorder(6)
                            # Update mid points offsets
                            if len(points) > 2:
                                self.artists['preview_markers_mid'][uid].set_offsets(np.c_[px[1:-1], py[1:-1]])
                                try:
                                    self.artists['preview_markers_mid'][uid].set_facecolor(preview_color)
                                except Exception:
                                    pass
                            else:
                                self.artists['preview_markers_mid'][uid].set_visible(False)
                        # Update or create sync texts for preview
                        sync_points = [p for p in points if int(p.get('nav_mode') or 0) == 1]
                        if sync_points:
                            existing = self.artists.get('preview_sync_texts', {}).get(uid)
                            if existing and isinstance(existing, list) and len(existing) == len(sync_points):
                                # update positions
                                for idx_sp, sp in enumerate(sync_points):
                                    sx, sy = sp['pos'][0], sp['pos'][1]
                                    existing[idx_sp].set_position((sx, sy))
                                    existing[idx_sp].set_visible(True)
                            else:
                                # remove old if any
                                if uid in self.artists['preview_sync_texts']:
                                    old = self.artists['preview_sync_texts'].pop(uid)
                                    for o in old:
                                        try:
                                            o.remove()
                                        except Exception:
                                            pass
                                sync_texts = []
                                base_transform = offset_copy(self.ax.transData, fig=self.figure, x=5, y=5, units='dots')
                                for sp in sync_points:
                                    sx, sy = sp['pos'][0], sp['pos'][1]
                                    txt = self.ax.text(sx, sy, '=', fontsize=12, color=preview_color if preview_color else '#f1c40f', ha='center', va='center', zorder=8, fontweight='bold', transform=base_transform)
                                    sync_texts.append(txt)
                                if sync_texts:
                                    self.artists['preview_sync_texts'][uid] = sync_texts
                        else:
                            # hide any existing
                            if uid in self.artists.get('preview_sync_texts', {}):
                                for t in self.artists['preview_sync_texts'][uid]:
                                    t.set_visible(False)
                                
                        # Update start/end positions and colors
                        self.artists['preview_markers_start'][uid].set_offsets([[px[0], py[0]]])
                        self.artists['preview_markers_end'][uid].set_offsets([[px[-1], py[-1]]])
                        try:
                            self.artists['preview_markers_start'][uid].set_facecolor(preview_color)
                            self.artists['preview_markers_end'][uid].set_facecolor(preview_color)
                        except Exception:
                            pass

                    # 3. Maneuver Rings
                    maneuver_points = [(p['pos'][0], p['pos'][1]) for p in points if p.get('maneuver_type', 0) == 1]
                    if maneuver_points:
                        mx, my = zip(*maneuver_points)
                        if uid not in self.artists['preview_markers_maneuver']:
                            # s=300 for larger ring, facecolor='none'
                            rings = self.ax.scatter(mx, my, s=300, marker='o', 
                                                  facecolor='none', edgecolor='#f39c12', 
                                                  linewidth=2.5, zorder=2, label='Maneuver')
                            self.artists['preview_markers_maneuver'][uid] = rings
                        else:
                            self.artists['preview_markers_maneuver'][uid].set_offsets(np.c_[mx, my])
                            self.artists['preview_markers_maneuver'][uid].set_visible(True)
                    elif uid in self.artists['preview_markers_maneuver']:
                         self.artists['preview_markers_maneuver'][uid].set_visible(False)

        else:
            # Hide preview artists
            for cat in ['preview_lines', 'preview_markers_start', 'preview_markers_end', 'preview_markers_mid', 'preview_markers_maneuver']:
                 for artist in self.artists[cat].values():
                     artist.set_visible(False)

        # --- 判断渲染模式: 根据Y轴网格间距 ---
        yticks = self.ax.get_yticks()
        if len(yticks) >= 2:
            grid_spacing = abs(yticks[1] - yticks[0])
        else:
            ylim = self.ax.get_ylim()
            grid_spacing = abs(ylim[1] - ylim[0]) / 5.0  # 估算
        new_mode = 'polygon' if grid_spacing <= 2.0 else 'scatter'
        
        # 切换模式时, 移除所有旧的 USV 标记
        if self._usv_render_mode is not None and new_mode != self._usv_render_mode:
            old_key = 'usv_polys' if self._usv_render_mode == 'polygon' else 'usv_scatters'
            for uid, artist in list(self.artists[old_key].items()):
                artist.remove()
            self.artists[old_key].clear()
        self._usv_render_mode = new_mode

        # --- Track current USVs to handle removals ---
        active_usv_ids = set()

        # --- Draw Live USVs ---
        for idx, usv in enumerate(usv_list):
            usv_id = usv.get('namespace', 'unknown')
            
            # Skip disconnected USVs
            if not usv.get('connected', False):
                continue

            active_usv_ids.add(usv_id)
            
            pos = usv.get('position', {})
            x = pos.get('x', 0.0)
            y = pos.get('y', 0.0)
            yaw = usv.get('yaw', 0.0) # radians
            
            all_x.append(x)
            all_y.append(y)

            # --- 1. Determine Color ---
            led_info = usv.get('led_status')
            color_hex = self._get_led_color_hex(led_info)
            if not color_hex:
                color_hex = default_colors[idx % len(default_colors)]
            
            # --- 2. Determine Blinking ---
            vel = usv.get('velocity', {}).get('linear', {})
            speed = sqrt(vel.get('x', 0)**2 + vel.get('y', 0)**2)
            is_moving = speed > 0.1
            display_alpha = 1.0 if (not is_moving or blink_on) else 0.4
            
            # --- 3. Update/Create USV Marker ---
            is_leader = usv_id in self._formation_leader_ids
            edge_color = '#FFD700' if is_leader else '#ecf0f1'  # 金色边框表示领队
            edge_width = 2.5 if is_leader else 1.2
            
            if self._usv_render_mode == 'polygon':
                # ===== 模式A: Polygon (1:1真实尺寸, grid≤2m) =====
                cos_y, sin_y = cos(yaw), sin(yaw)
                rot_matrix = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
                rotated_verts = self._usv_base_verts @ rot_matrix.T + np.array([x, y])
                
                if usv_id not in self.artists['usv_polys']:
                    poly = MplPolygon(rotated_verts, closed=True,
                                      facecolor=color_hex, edgecolor=edge_color,
                                      linewidth=edge_width, alpha=display_alpha, zorder=10)
                    self.ax.add_patch(poly)
                    self.artists['usv_polys'][usv_id] = poly
                else:
                    poly = self.artists['usv_polys'][usv_id]
                    poly.set_xy(rotated_verts)
                    poly.set_facecolor(color_hex)
                    poly.set_alpha(display_alpha)
                    poly.set_edgecolor(edge_color)
                    poly.set_linewidth(edge_width)
                    poly.set_visible(True)
            else:
                # ===== 模式B: Scatter (固定像素大小, grid>2m) =====
                rotated_marker = MarkerStyle(self._usv_arrow_path).rotated(rad=yaw)
                marker_size = 700 if is_leader else 400
                
                if usv_id not in self.artists['usv_scatters']:
                    scatter = self.ax.scatter([x], [y], s=marker_size, c=[color_hex],
                                             marker=rotated_marker,
                                             edgecolors=edge_color, linewidths=edge_width,
                                             alpha=display_alpha, zorder=10)
                    self.artists['usv_scatters'][usv_id] = scatter
                else:
                    scatter = self.artists['usv_scatters'][usv_id]
                    scatter.set_offsets([[x, y]])
                    scatter.set_facecolors([color_hex])
                    scatter.set_alpha(display_alpha)
                    scatter.set_sizes([marker_size])
                    scatter.set_edgecolors([edge_color])
                    scatter.set_linewidths([edge_width])
                    scatter.set_paths([rotated_marker.get_path().transformed(rotated_marker.get_transform())])
                    scatter.set_visible(True)
            
            # --- 3b. 领队外圈光环 ---
            if is_leader:
                if usv_id not in self._formation_rings:
                    ring = self.ax.scatter([x], [y], s=1200, c='none',
                                          marker='o', edgecolors='#FFD700',
                                          linewidths=1.5, alpha=0.6, zorder=9)
                    self._formation_rings[usv_id] = ring
                else:
                    ring = self._formation_rings[usv_id]
                    ring.set_offsets([[x, y]])
                    ring.set_visible(True)
            elif usv_id in self._formation_rings:
                self._formation_rings[usv_id].set_visible(False)

            # --- 4. Update/Create 3D Trail ---
            z = float(pos.get('z', 0.0))
            clean_id = str(usv_id).lstrip('/')
            if clean_id not in self.usv_trails:
                self.usv_trails[clean_id] = []
            self.usv_trails[clean_id].append((x, y, z))
            if len(self.usv_trails[clean_id]) > self.max_trail_length:
                self.usv_trails[clean_id].pop(0)

            if self.show_trails and len(self.usv_trails[clean_id]) > 1:
                tx = [pt[0] for pt in self.usv_trails[clean_id]]
                ty = [pt[1] for pt in self.usv_trails[clean_id]]
                if usv_id not in self.artists['usv_trails']:
                    line, = self.ax.plot(tx, ty, color=color_hex, alpha=0.3, linestyle='-', linewidth=1)
                    self.artists['usv_trails'][usv_id] = line
                else:
                    line = self.artists['usv_trails'][usv_id]
                    line.set_data(tx, ty)
                    line.set_color(color_hex)
                    line.set_visible(True)
            elif usv_id in self.artists['usv_trails']:
                 self.artists['usv_trails'][usv_id].set_visible(False)

            # --- 5. Navigation Task ---
            nav_target = usv.get('nav_target_cache')
            if self.show_nav_task and nav_target:
                tx, ty = nav_target.get('x'), nav_target.get('y')
                tz = float(nav_target.get('z', 0.0) or 0.0)
                target_reached = nav_target.get('reached', False)
                # 根据是否到达选择颜色：未到达用红色，已到达用绿色
                marker_color = '#2ecc71' if target_reached else '#e74c3c'  # 绿色 / 红色
                if tx is not None and ty is not None:
                    # Line
                    if usv_id not in self.artists['nav_lines']:
                        line, = self.ax.plot([x, tx], [y, ty], color=color_hex, linestyle=':', alpha=0.8, linewidth=1.5)
                        self.artists['nav_lines'][usv_id] = line
                    else:
                        line = self.artists['nav_lines'][usv_id]
                        line.set_data([x, tx], [y, ty])
                        line.set_color(color_hex)
                        line.set_visible(True)

                    # Marker - prefer using led color for target if provided
                    nav_led_val = None
                    if isinstance(nav_target, dict):
                        nav_led_val = nav_target.get('led')
                    # fall back to usv-level led fields
                    if not nav_led_val:
                        nav_led_val = usv.get('led') or usv.get('led_status')
                    resolved_led = self._resolve_led_color(nav_led_val)
                    marker_c = resolved_led or marker_color
                    if usv_id not in self.artists['nav_markers']:
                        mark, = self.ax.plot(tx, ty, marker='x', markersize=8, color=marker_c, markeredgewidth=2, linestyle='None')
                        self.artists['nav_markers'][usv_id] = mark
                    else:
                        mark = self.artists['nav_markers'][usv_id]
                        mark.set_data([tx], [ty])
                        mark.set_color(marker_c)
                        mark.set_visible(True)

                    # 如果该目标为同步模式 (nav_mode == 1)，在目标点附近绘制“=”符号
                    is_sync = bool(nav_target.get('nav_mode') == 1) if isinstance(nav_target, dict) else False
                    if is_sync:
                        sync_color = resolved_led or '#f1c40f'
                        if usv_id not in self.artists['nav_sync_markers']:
                            eq_transform = offset_copy(self.ax.transData, fig=self.figure, x=5, y=5, units='dots')
                            eq = self.ax.text(tx, ty, '=', fontsize=18, color=sync_color, ha='center', va='center', zorder=12, fontweight='bold', transform=eq_transform)
                            self.artists['nav_sync_markers'][usv_id] = eq
                        else:
                            eq = self.artists['nav_sync_markers'][usv_id]
                            eq.set_position((tx, ty))
                            eq.set_color(sync_color)
                            eq.set_visible(True)
                    else:
                        # 隐藏可能存在的同步标记
                        if usv_id in self.artists['nav_sync_markers']:
                            self.artists['nav_sync_markers'][usv_id].set_visible(False)

                    # Nav Target Label (XYZ Coordinates)
                    z_lbl = f", {tz:.1f}" if abs(tz) > 0.05 else ""
                    label_text = f"({tx:.1f}, {ty:.1f}{z_lbl})"
                    if usv_id not in self.artists['nav_labels']:
                        text = self.ax.text(tx, ty + 1.0, label_text,
                                           color=color_hex, fontsize=8, ha='center',
                                           bbox=dict(facecolor='#2c3e50', alpha=0.3, edgecolor='none', pad=1))
                        self.artists['nav_labels'][usv_id] = text
                    else:
                        text = self.artists['nav_labels'][usv_id]
                        text.set_position((tx, ty + 1.0))
                        text.set_text(label_text)
                        text.set_color(color_hex)
                        text.set_visible(True)

                    all_x.append(tx)
                    all_y.append(ty)
            else:
                if usv_id in self.artists['nav_lines']: self.artists['nav_lines'][usv_id].set_visible(False)
                if usv_id in self.artists['nav_markers']: self.artists['nav_markers'][usv_id].set_visible(False)
                if usv_id in self.artists['nav_labels']: self.artists['nav_labels'][usv_id].set_visible(False)
                if usv_id in self.artists['nav_sync_markers']: self.artists['nav_sync_markers'][usv_id].set_visible(False)

            # --- 6. Label ---
            if self.show_labels:
                # 编队角色标注
                role_prefix = ""
                label_color = '#ecf0f1'
                if usv_id in self._formation_leader_ids:
                    role_prefix = "👑 "
                    label_color = '#FFD700'  # 金色
                elif usv_id in self._formation_follower_ids:
                    role_prefix = "↝ "
                
                label_str = f"{role_prefix}{usv_id}"
                if usv_id not in self.artists['usv_labels']:
                    text = self.ax.text(x, y + 1.5, label_str, 
                            color=label_color, fontsize=9, ha='center',
                            bbox=dict(facecolor='#2c3e50', alpha=0.5, edgecolor='none', pad=2))
                    self.artists['usv_labels'][usv_id] = text
                else:
                    text = self.artists['usv_labels'][usv_id]
                    text.set_position((x, y + 1.5))
                    text.set_text(label_str)
                    text.set_color(label_color)
                    text.set_visible(True)
            elif usv_id in self.artists['usv_labels']:
                self.artists['usv_labels'][usv_id].set_visible(False)
            
            self.active_usv_points.append({
                'x': x, 'y': y, 'radius': 2.0, 'usv': usv, 'id': usv_id
            })

        # --- Cleanup stale artists ---
        active_key = 'usv_polys' if self._usv_render_mode == 'polygon' else 'usv_scatters'
        for uid in list(self.artists[active_key].keys()):
            if uid not in active_usv_ids:
                self.artists[active_key][uid].set_visible(False)
                if uid in self.artists['usv_labels']: self.artists['usv_labels'][uid].set_visible(False)
                if uid in self.artists['nav_labels']: self.artists['nav_labels'][uid].set_visible(False)
                if uid in self.artists['nav_sync_markers']: self.artists['nav_sync_markers'][uid].set_visible(False)
                if uid in self._formation_rings: self._formation_rings[uid].set_visible(False)
                # Trails persist usually, but maybe hide them? Keeping them for now.

        # --- 7. Auto Scale ---
        if self.auto_scale:
            if all_x:
                margin = 5.0
                min_x, max_x = min(all_x), max(all_x)
                min_y, max_y = min(all_y), max(all_y)
                
                width = max_x - min_x
                height = max_y - min_y
                
                # Ensure not zero range
                if width < 10: width = 10; mid_x = (min_x+max_x)/2; min_x = mid_x - 5; max_x = mid_x + 5
                if height < 10: height = 10; mid_y = (min_y+max_y)/2; min_y = mid_y - 5; max_y = mid_y + 5
                
                self._updating_limits = True
                self.ax.set_xlim(min_x - margin, max_x + margin)
                self.ax.set_ylim(min_y - margin, max_y + margin)
                self._updating_limits = False
            else:
                 self._updating_limits = True
                 self.ax.set_xlim(-10, 10)
                 self.ax.set_ylim(-10, 10)
                 self._updating_limits = False

        preview_count = sum(len(pts) for pts in self.preview_trails.values()) if hasattr(self, 'preview_trails') else 0
        mode_tag = "3D" if getattr(self, 'view_mode', '3d') == '3d' else f"2D ({'Auto' if self.auto_scale else 'Manual'})"
        self.info_label.setText(f"[{mode_tag}] 在线: {len(active_usv_ids)} | 预览点: {preview_count} | {datetime.datetime.now().strftime('%H:%M:%S')}")

        if getattr(self, 'view_mode', '3d') == '3d' and hasattr(self, 'canvas_3d'):
            self.canvas_3d.update()
        else:
            if self.canvas.width() > 10 and self.canvas.height() > 10:
                self._enforce_equal_aspect_fill()
                self.canvas.draw_idle()

    def on_scroll(self, event):
        """Zoom with mouse wheel at cursor position"""
        if not event.inaxes: return
        
        # Disable auto-scale on manual interaction
        self.auto_scale = False

        zoom_factor = 1.2
        if event.button == 'up':
            scale = 1 / zoom_factor
        elif event.button == 'down':
            scale = zoom_factor
        else:
            scale = 1

        xdata = event.xdata
        ydata = event.ydata

        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        new_width = (xlim[1] - xlim[0]) * scale
        new_height = (ylim[1] - ylim[0]) * scale

        relx = (xlim[1] - xdata) / (xlim[1] - xlim[0])
        rely = (ylim[1] - ydata) / (ylim[1] - ylim[0])

        self.ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
        self.ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * rely])
        
        self.canvas.draw_idle()

    def on_click(self, event):
        if not event.inaxes:
            return

        click_x, click_y = event.xdata, event.ydata

        # Middle Click (2): Start Dragging (Pan)
        if event.button == 2:
            self.is_panning = True
            self.last_mouse_pos = (event.x, event.y)
            self.auto_scale = False # Disable auto scale on interaction
            return

        # Left Click (1): Select USV or Double Click for Fit
        if event.button == 1:
            if event.dblclick:
                # Double Click: Fit All and Re-enable Auto Scale
                self.auto_scale = True
                self.update_plot() # Force update to re-calc limits
                return
            
            # specific simple hit detection
            closest = None
            min_dist = 999.0
            
            for item in self.active_usv_points:
                dist = sqrt((click_x - item['x'])**2 + (click_y - item['y'])**2)
                if dist < item['radius'] and dist < min_dist:
                    min_dist = dist
                    closest = item
            
            if closest:
                self.show_usv_details(closest['usv'])
        
        # Right Click (3): Context Menu
        elif event.button == 3:
            self._show_context_menu(event)

    def on_release(self, event):
        """Handle mouse button release"""
        if event.button == 2: # Middle Click
            self.is_panning = False
            self.last_mouse_pos = None

    def on_move(self, event):
        """Handle mouse movement for panning"""
        if self.is_panning:
            if self.last_mouse_pos is None:
                self.last_mouse_pos = (event.x, event.y)
                return
            
            if event.x is None or event.y is None:
                return
            
            # Calculate shift in pixels
            dx_pix = event.x - self.last_mouse_pos[0]
            dy_pix = event.y - self.last_mouse_pos[1]
            
            # Convert to data coordinates
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
            
            # Get pixel width/height of axes
            bbox = self.ax.get_window_extent().transformed(self.figure.dpi_scale_trans.inverted())
            width_pix = bbox.width * self.figure.dpi
            height_pix = bbox.height * self.figure.dpi
            
            # Calculate data shift
            dx_data = (xlim[1] - xlim[0]) * dx_pix / width_pix
            dy_data = (ylim[1] - ylim[0]) * dy_pix / height_pix
            
            # Apply shift (inverted because dragging map means moving view opposite)
            self.ax.set_xlim(xlim[0] - dx_data, xlim[1] - dx_data)
            self.ax.set_ylim(ylim[0] - dy_data, ylim[1] - dy_data)
            
            self.last_mouse_pos = (event.x, event.y)
            self.canvas.draw_idle()

    def _show_context_menu(self, event):
        """Show context menu for plot interactions"""
        menu = QMenu(self)

        act_3d = QAction(f"🎯 发送三维目标点到此位置 ({event.xdata:.1f}, {event.ydata:.1f}, Z...)", self)
        act_3d.triggered.connect(lambda: self.open_send_3d_target_dialog((event.xdata, event.ydata, 0.0)))
        menu.addAction(act_3d)

        act_dep = QAction(f"📍 填入离群三维坐标框 ({event.xdata:.1f}, {event.ydata:.1f})", self)
        act_dep.triggered.connect(lambda: self.fill_departed_spinboxes(event.xdata, event.ydata, 0.0))
        menu.addAction(act_dep)
        menu.addSeparator()

        # Action: Set Home Here
        set_home_action = QAction("🏠 设置此处为 Home 点", self)
        set_home_action.triggered.connect(lambda: self.set_home_position(event.xdata, event.ydata))
        menu.addAction(set_home_action)
        
        # Convert matplotlib event position to global screen position for QMenu
        # event.guiEvent is a QMouseEvent
        cursor_pos = event.guiEvent.globalPos()
        menu.exec_(cursor_pos)

    def set_home_position(self, x, y):
        """Update Home position and redraw marker"""
        self.home_pos = (x, y)
        self._draw_home_marker()
        self._enforce_equal_aspect_fill()
        self.canvas.draw_idle()
        if hasattr(self, 'canvas_3d'):
            self.canvas_3d.update()
        # Emit signal to notify parent (MainGuiApp) to send ROS command
        if hasattr(self, 'request_set_home_callback') and self.request_set_home_callback:
            self.request_set_home_callback(x, y)
        print(f"Home position set to: ({x:.2f}, {y:.2f})")

    def show_usv_details(self, usv_data):
        usv_id = usv_data.get('namespace', 'Unknown')
        vt = usv_data.get('vehicle_type', 'usv_diff')
        pm = usv_data.get('platform_mode', '2d')
        pos = usv_data.get('position', {}) or {}
        vel = (usv_data.get('velocity', {}) or {}).get('linear', {}) or {}
        led = usv_data.get('led_status', {})
        nav = usv_data.get('nav_target_cache', {})
        
        info = f"<h3>USV/UAV: {usv_id} <small>[{vt} | {str(pm).upper()}]</small></h3>"
        info += f"<b>State:</b> {usv_data.get('mode', 'N/A')} | Armed={usv_data.get('armed', False)}<br>"
        info += f"<b>3D Position:</b> X={pos.get('x',0):.2f}m, Y={pos.get('y',0):.2f}m, Z={pos.get('z',0):.2f}m<br>"
        info += f"<b>Heading:</b> {usv_data.get('heading', degrees(float(usv_data.get('yaw', 0.0)))):.1f}°<br>"
        info += f"<b>3D Speed:</b> {sqrt(vel.get('x',0)**2 + vel.get('y',0)**2 + vel.get('z',0)**2):.2f} m/s (Vz={vel.get('z',0):.2f} m/s)<br>"
        info += f"<b>Battery:</b> {usv_data.get('battery_percentage', 0):.1f}% ({usv_data.get('battery_voltage',0):.2f}V)<br>"
        
        if led:
            color = led.get('color', [])
            mode = led.get('mode', 'unknown')
            info += f"<b>LED:</b> Mode={mode}, Color={color}<br>"
            
        if nav:
            info += f"<br><b>3D Nav Target:</b> X={nav.get('x',0):.2f}m, Y={nav.get('y',0):.2f}m, Z={nav.get('z',0):.2f}m"
            
        msg = QMessageBox(self)
        msg.setWindowTitle(f"Details: {usv_id}")
        msg.setTextFormat(Qt.RichText)
        msg.setText(info)
        msg.setIcon(QMessageBox.Information)
        msg.exec_()

    def _center_on_screen(self):
        from PyQt5.QtWidgets import QApplication
        screen = QApplication.desktop().screenGeometry()
        size = self.geometry()
        self.move(
            (screen.width() - size.width()) // 2,
            (screen.height() - size.height()) // 2
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._enforce_equal_aspect_fill()
        self.canvas.draw_idle()
