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
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QCheckBox, QHBoxLayout, 
                             QPushButton, QLabel, QSlider, QGroupBox, QMessageBox, QWidget, QMenu, QAction)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtCore import Qt as QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from matplotlib.figure import Figure
from matplotlib.markers import MarkerStyle
from matplotlib.path import Path as MplPath
from math import cos, sin, pi, atan2, sqrt, radians
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import offset_copy
import numpy as np
import time
import datetime

# Configure matplotlib font
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS', 'WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False 

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
        main_layout.setSpacing(5)
        main_layout.setContentsMargins(10, 10, 10, 10)

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

        # ========== Info Bar ==========
        info_layout = QHBoxLayout()
        self.info_label = QLabel("Waiting for data...")
        # Style will be set in set_theme
        info_layout.addWidget(self.info_label)
        info_layout.addStretch()
        main_layout.addLayout(info_layout)

        # ========== Matplotlib Canvas ==========
        # Initial colors will be set in set_theme
        self.figure = Figure() 
        self.canvas = FigureCanvas(self.figure)
        
        from PyQt5.QtWidgets import QSizePolicy
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.toolbar.setMaximumHeight(35)
        
        main_layout.addWidget(self.toolbar)
        main_layout.addWidget(self.canvas, stretch=1)

        self.get_usv_list_func = get_usv_list_func
        self.usv_trails = {}  # {usv_id: [(x,y), ...]}
        self.preview_trails = {} # {usv_id: [(x,y), ...]}
        self.max_trail_length = 500
        
        # Default Home Position
        self.home_pos = (0.0, 0.0)

        # ========== Graphics Objects Cache (Optimization) ==========
        self.ax = self.figure.add_subplot(111)
        self._updating_limits = False  # Flag to distinguish programmatic updates
        self.ax.callbacks.connect('xlim_changed', self.on_limits_changed)
        self.ax.callbacks.connect('ylim_changed', self.on_limits_changed)
        
        self.artists = {
            'usv_polys': {},    # {usv_id: PathCollection (scatter)}
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
        
        # 创建自定义USV箭头标记 (固定像素大小)
        # 箭头形状: 指向右侧的船形
        arrow_verts = [
            (1.0, 0.0),    # 船头
            (-0.8, 0.6),   # 左后角
            (-0.4, 0.0),   # 中心凹陷
            (-0.8, -0.6),  # 右后角
            (1.0, 0.0),    # 闭合回船头
        ]
        arrow_codes = [MplPath.MOVETO, MplPath.LINETO, MplPath.LINETO, MplPath.LINETO, MplPath.CLOSEPOLY]
        self._usv_arrow_path = MplPath(arrow_verts, arrow_codes)
        
        # Apply theme to axes
        self.set_theme('modern_dark')

        # Callback for Set Home Request
        self.request_set_home_callback = None

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
            
            # Redraw
            if hasattr(self, 'canvas'):
                # Force redraw home marker if it exists
                self._draw_home_marker()
                self.canvas.draw_idle()

    def _init_axes(self):
        """Initialize fixed axes properties."""
        self.ax.set_aspect('equal')
        # Detailed styling is now handled in set_theme()
        
        # Draw initial home marker
        self._draw_home_marker()
        
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
            
            if 'position' in item and isinstance(item['position'], dict):
                x_local = float(item['position'].get('x', 0.0))
                y_local = float(item['position'].get('y', 0.0))
            else:
                x_local = float(item.get('x', 0.0))
                y_local = float(item.get('y', 0.0))
            
            # Apply Rotation (around local 0,0)
            x_rot = x_local * cos_t - y_local * sin_t
            y_rot = x_local * sin_t + y_local * cos_t
            
            # Apply Offset (Translate to Area Center global position)
            x_final = x_rot + offset[0]
            y_final = y_rot + offset[1]
            
            # Extract Maneuver info
            m_type = item.get('maneuver_type', 0)
            
            if uid not in temp:
                temp[uid] = []
            # Preserve relevant metadata such as led color and nav_mode for preview rendering
            temp[uid].append({
                'step': step,
                'pos': (x_final, y_final),
                'maneuver_type': m_type,
                'led': item.get('led') if isinstance(item, dict) else None,
                'nav_mode': item.get('nav_mode') if isinstance(item, dict) else None,
                'usv_id': uid
            })
            
        # Sort and store
        for uid, items in temp.items():
            items.sort(key=lambda x: x['step'])
            # Store full item dict to preserve maneuver info
            self.preview_trails[uid] = items 
        
        # print(f"DEBUG: Processed preview trails: {len(self.preview_trails)} USVs")
            
        # Force re-enable auto scale to make sure user sees the new path
        # self.auto_scale = True
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
                        sync_points = [p for p in points if int(p.get('nav_mode', 0)) == 1]
                        sync_texts = []
                        # create transform with 3 pixel offset to the right-top
                        base_transform = offset_copy(self.ax.transData, fig=self.figure, x=5, y=5, units='dots')
                        for sp in sync_points:
                            sx, sy = sp['pos']
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
                        sync_points = [p for p in points if int(p.get('nav_mode', 0)) == 1]
                        if sync_points:
                            existing = self.artists.get('preview_sync_texts', {}).get(uid)
                            if existing and isinstance(existing, list) and len(existing) == len(sync_points):
                                # update positions
                                for idx_sp, sp in enumerate(sync_points):
                                    sx, sy = sp['pos']
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
                                    sx, sy = sp['pos']
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
                    maneuver_points = [p['pos'] for p in points if p.get('maneuver_type', 0) == 1]
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
            
            # --- 3. Update/Create USV Marker (固定像素大小，不随缩放变化) ---
            # 创建旋转后的箭头标记
            rotated_marker = MarkerStyle(self._usv_arrow_path).rotated(rad=yaw)
            
            # 编队领队使用更大的标记 (s=700 vs s=400)
            is_leader = usv_id in self._formation_leader_ids
            marker_size = 700 if is_leader else 400
            edge_color = '#FFD700' if is_leader else '#ecf0f1'  # 金色边框表示领队
            edge_width = 2.5 if is_leader else 1.2
            
            if usv_id not in self.artists['usv_polys']:
                # 使用 scatter 创建固定像素大小的标记
                scatter = self.ax.scatter([x], [y], s=marker_size, c=[color_hex], 
                                         marker=rotated_marker, 
                                         edgecolors=edge_color, linewidths=edge_width,
                                         alpha=display_alpha, zorder=10)
                self.artists['usv_polys'][usv_id] = scatter
            else:
                scatter = self.artists['usv_polys'][usv_id]
                # 更新位置和外观
                scatter.set_offsets([[x, y]])
                scatter.set_facecolors([color_hex])
                scatter.set_alpha(display_alpha)
                scatter.set_sizes([marker_size])
                scatter.set_edgecolors([edge_color])
                scatter.set_linewidths([edge_width])
                # 更新旋转标记 - 需要重新设置 paths
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

            # --- 4. Update/Create Trail ---
            if usv_id not in self.usv_trails:
                self.usv_trails[usv_id] = []
            self.usv_trails[usv_id].append((x, y))
            if len(self.usv_trails[usv_id]) > self.max_trail_length:
                self.usv_trails[usv_id].pop(0)

            if self.show_trails and len(self.usv_trails[usv_id]) > 1:
                tx, ty = zip(*self.usv_trails[usv_id])
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

                    # Nav Target Label (XY Coordinates)
                    label_text = f"({tx:.1f}, {ty:.1f})"
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
        for uid in list(self.artists['usv_polys'].keys()):
            if uid not in active_usv_ids:
                self.artists['usv_polys'][uid].set_visible(False)
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

        # Update Info
        # Change: Display actual online USVs count and Preview count
        preview_count = sum(len(pts) for pts in self.preview_trails.values()) if hasattr(self, 'preview_trails') else 0
        self.info_label.setText(f"Active: {len(active_usv_ids)} | Preview Pts: {preview_count} | Scale: {'Auto' if self.auto_scale else 'Manual'} | {datetime.datetime.now().strftime('%H:%M:%S')}")

        # Optimize draw call
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
        
        # Action: Set Home Here
        set_home_action = QAction("Set Home Here", self)
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
        self.canvas.draw_idle()
        # Emit signal to notify parent (MainGuiApp) to send ROS command
        if hasattr(self, 'request_set_home_callback') and self.request_set_home_callback:
            self.request_set_home_callback(x, y)
        # Optional: Emit signal or log
        print(f"Home position set to: ({x:.2f}, {y:.2f})")

    def show_usv_details(self, usv_data):
        usv_id = usv_data.get('namespace', 'Unknown')
        pos = usv_data.get('position', {})
        vel = usv_data.get('velocity', {}).get('linear', {})
        led = usv_data.get('led_status', {})
        nav = usv_data.get('nav_target_cache', {})
        
        info = f"<h3>USV: {usv_id}</h3>"
        info += f"<b>State:</b> {usv_data.get('mode', 'N/A')}<br>"
        info += f"<b>Position:</b> X={pos.get('x',0):.2f}, Y={pos.get('y',0):.2f}<br>"
        info += f"<b>Heading:</b> {usv_data.get('heading', 0):.1f}°<br>"
        info += f"<b>Speed:</b> {sqrt(vel.get('x',0)**2 + vel.get('y',0)**2):.2f} m/s<br>"
        info += f"<b>Battery:</b> {usv_data.get('battery_percentage', 0):.1f}% ({usv_data.get('battery_voltage',0):.2f}V)<br>"
        
        if led:
            color = led.get('color', [])
            mode = led.get('mode', 'unknown')
            info += f"<b>LED:</b> Mode={mode}, Color={color}<br>"
            
        if nav:
            info += f"<br><b>Nav Target:</b> X={nav.get('x',0):.2f}, Y={nav.get('y',0):.2f}"
            
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
