from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QCheckBox, QHBoxLayout, 
                             QPushButton, QLabel, QSlider, QGroupBox, QMessageBox, QWidget, QMenu, QAction)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtCore import Qt as QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from matplotlib.figure import Figure
from math import cos, sin, pi, atan2, sqrt
import matplotlib.pyplot as plt
import matplotlib.patches as patches
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
        self.artists = {
            'usv_polys': {},    # {usv_id: Polygon}
            'usv_labels': {},   # {usv_id: Text}
            'usv_trails': {},   # {usv_id: Line2D}
            'nav_lines': {},    # {usv_id: Line2D}
            'nav_markers': {},  # {usv_id: Line2D (marker)}
            'preview_lines': {}, # {usv_id: Line2D}
            'preview_markers_start': {}, # {usv_id: PathCollection}
            'preview_markers_end': {},   # {usv_id: PathCollection}
            'preview_markers_mid': {},   # {usv_id: PathCollection}
            'preview_markers_maneuver': {}, # {usv_id: PathCollection} (Rings for maneuvers)
            'home_marker': None # Home marker aritst
        }
        
        # Apply theme to axes
        self.set_theme('modern_dark')

        # Callback for Set Home Request
        self.request_set_home_callback = None

        # ========== Timer ==========
        # Update at 5Hz to prevent blocking main thread
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(200) # 200ms

        # Signals
        self.canvas.mpl_connect('button_press_event', self.on_click)

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

    def set_preview_path(self, task_data_list):
        """Set task data for preview"""
        self.preview_trails = {}
        # Clear previous preview artists
        for key in ['preview_lines', 'preview_markers_start', 'preview_markers_end', 'preview_markers_mid', 'preview_markers_maneuver']:
            for artist in self.artists[key].values():
                artist.remove()
            self.artists[key].clear()
            
        if not task_data_list:
            self.update_plot()
            return
            
        # Group by USV
        temp = {}
        for item in task_data_list:
            uid = item.get('usv_id')
            step = item.get('step', 0)
            
            # Extract position (handle nested dictionary structure from ClusterTaskManager)
            x = 0.0
            y = 0.0
            
            if 'position' in item and isinstance(item['position'], dict):
                x = float(item['position'].get('x', 0.0))
                y = float(item['position'].get('y', 0.0))
            else:
                x = float(item.get('x', 0.0))
                y = float(item.get('y', 0.0))
            
            # Extract Maneuver info
            m_type = item.get('maneuver_type', 0)
            
            if uid not in temp:
                temp[uid] = []
            temp[uid].append({
                'step': step, 
                'pos': (x, y),
                'maneuver_type': m_type
            })
            
        # Sort and store
        for uid, items in temp.items():
            items.sort(key=lambda x: x['step'])
            # Store full item dict to preserve maneuver info
            self.preview_trails[uid] = items 
            
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
                        line, = self.ax.plot(px, py, color='#7f8c8d', alpha=0.6, linestyle='-.', linewidth=1.5, zorder=1)
                        self.artists['preview_lines'][uid] = line
                    else:
                        self.artists['preview_lines'][uid].set_data(px, py)
                        self.artists['preview_lines'][uid].set_visible(True)
                    
                    # 2. Start/End Points
                    if uid not in self.artists['preview_markers_start']:
                        start = self.ax.scatter([px[0]], [py[0]], c='#2ecc71', s=20, marker='o', zorder=2, edgecolors='none', label='Start')
                        end = self.ax.scatter([px[-1]], [py[-1]], c='#e74c3c', s=20, marker='o', zorder=2, edgecolors='none', label='End')
                        self.artists['preview_markers_start'][uid] = start
                        self.artists['preview_markers_end'][uid] = end
                        
                        if len(points) > 2:
                            mid = self.ax.scatter(px[1:-1], py[1:-1], c='#7f8c8d', s=4, marker='o', zorder=2, edgecolors='none')
                            self.artists['preview_markers_mid'][uid] = mid
                    else:
                        self.artists['preview_markers_start'][uid].set_visible(True)
                        self.artists['preview_markers_end'][uid].set_visible(True)
                        if uid in self.artists['preview_markers_mid']:
                            self.artists['preview_markers_mid'][uid].set_visible(True)

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
            
            # --- 3. Update/Create USV Polygon ---
            radius = 0.6
            pts = np.array([[1.0, 0.0], [-0.8, 0.6], [-0.4, 0.0], [-0.8, -0.6]]) * radius
            R = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
            pts_final = np.dot(pts, R.T) + np.array([x, y])

            if usv_id not in self.artists['usv_polys']:
                # 视觉优化：在深色背景下使用亮色边框 + 加粗线条，确保黑色USV也能看到轮廓
                poly = patches.Polygon(pts_final, closed=True, facecolor=color_hex, 
                                      edgecolor='#ecf0f1', alpha=display_alpha, lw=1.2, zorder=10)
                self.ax.add_patch(poly)
                self.artists['usv_polys'][usv_id] = poly
            else:
                poly = self.artists['usv_polys'][usv_id]
                poly.set_xy(pts_final)
                poly.set_facecolor(color_hex)
                poly.set_alpha(display_alpha)
                poly.set_visible(True)

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
                     # Marker
                     if usv_id not in self.artists['nav_markers']:
                         mark, = self.ax.plot(tx, ty, marker='x', markersize=8, color=color_hex, markeredgewidth=2, linestyle='None')
                         self.artists['nav_markers'][usv_id] = mark
                     else:
                         mark = self.artists['nav_markers'][usv_id]
                         mark.set_data([tx], [ty])
                         mark.set_color(color_hex)
                         mark.set_visible(True)
                     all_x.append(tx)
                     all_y.append(ty)
            else:
                 if usv_id in self.artists['nav_lines']: self.artists['nav_lines'][usv_id].set_visible(False)
                 if usv_id in self.artists['nav_markers']: self.artists['nav_markers'][usv_id].set_visible(False)

            # --- 6. Label ---
            if self.show_labels:
                if usv_id not in self.artists['usv_labels']:
                    text = self.ax.text(x, y + 1.5, f"{usv_id}", 
                            color='#ecf0f1', fontsize=9, ha='center',
                            bbox=dict(facecolor='#2c3e50', alpha=0.5, edgecolor='none', pad=2))
                    self.artists['usv_labels'][usv_id] = text
                else:
                    text = self.artists['usv_labels'][usv_id]
                    text.set_position((x, y + 1.5))
                    text.set_text(f"{usv_id}")
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
                # Trails persist usually, but maybe hide them? Keeping them for now.

        # --- 7. Auto Scale ---
        if self.auto_scale and all_x:
            margin = 5.0
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            
            width = max_x - min_x
            height = max_y - min_y
            
            # Ensure not zero range
            if width < 10: width = 10; mid_x = (min_x+max_x)/2; min_x = mid_x - 5; max_x = mid_x + 5
            if height < 10: height = 10; mid_y = (min_y+max_y)/2; min_y = mid_y - 5; max_y = mid_y + 5
            
            self.ax.set_xlim(min_x - margin, max_x + margin)
            self.ax.set_ylim(min_y - margin, max_y + margin)
        elif not all_x:
             self.ax.set_xlim(-10, 10)
             self.ax.set_ylim(-10, 10)

        # Update Info
        # Change: Display actual online USVs count instead of total list length
        self.info_label.setText(f"Active USVs: {len(active_usv_ids)} | {datetime.datetime.now().strftime('%H:%M:%S')}")

        # Optimize draw call
        self.canvas.draw_idle()

    def on_click(self, event):
        if not event.inaxes:
            return

        click_x, click_y = event.xdata, event.ydata

        # Left Click: Select USV
        if event.button == 1:
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
        
        # Right Click: Context Menu
        elif event.button == 3:
            self._show_context_menu(event)

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
