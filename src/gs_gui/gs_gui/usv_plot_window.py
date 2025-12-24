from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QCheckBox, QHBoxLayout, 
                             QPushButton, QLabel, QSlider, QGroupBox, QMessageBox)
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from matplotlib.figure import Figure
from math import cos, sin, sqrt
import matplotlib.pyplot as plt

# 配置 matplotlib 中文显示
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS', 'WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

class UsvPlotWindow(QDialog):
    def __init__(self, get_usv_list_func, parent=None):
        super().__init__(parent)
        self.setWindowTitle("USV 3D Position Display")
        self.resize(1000, 750)
        
        # 设置窗口样式
        self.setStyleSheet("""
            QDialog {
                background-color: #f5f5f5;
            }
            QGroupBox {
                background-color: #ffffff;
                border: 2px solid #e0e0e0;
                border-radius: 6px;
                margin-top: 15px;
                padding: 15px;
                font-weight: bold;
                color: #2c3e50;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0px 8px 0px 8px;
                background-color: #ffffff;
                color: #2c3e50;
            }
            QCheckBox {
                background-color: transparent;
                color: #2c3e50;
                font-size: 16px;
                spacing: 8px;
                padding: 4px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border: 2px solid #bdc3c7;
                border-radius: 3px;
                background-color: #ffffff;
            }
            QCheckBox::indicator:checked {
                background-color: #3498db;
                border: 2px solid #3498db;
                image: url(none);
            }
            QCheckBox::indicator:unchecked {
                background-color: #ffffff;
                border: 2px solid #bdc3c7;
            }
            QPushButton {
                background-color: #3498db;
                color: #ffffff;
                border: none;
                padding: 10px 20px;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
                min-width: 100px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #21618c;
            }
            QLabel {
                background-color: transparent;
                color: #2c3e50;
                font-size: 16px;
                padding: 4px;
            }
            QSlider::groove:horizontal {
                background: #ecf0f1;
                height: 8px;
                border-radius: 4px;
                border: 1px solid #bdc3c7;
            }
            QSlider::handle:horizontal {
                background: #3498db;
                width: 18px;
                height: 18px;
                margin: -6px 0;
                border-radius: 9px;
                border: 2px solid #2980b9;
            }
            QSlider::handle:horizontal:hover {
                background: #2980b9;
            }
        """)
        
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(5)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # ========== Control Panel (紧凑布局) ==========
        control_group = QGroupBox("Display Controls")
        control_group.setMaximumHeight(120)  # 限制控制面板高度
        control_layout = QVBoxLayout()
        control_layout.setSpacing(5)
        
        # First row: Display options
        display_layout = QHBoxLayout()
        
        self.show_label_checkbox = QCheckBox("Show Labels")
        self.show_label_checkbox.setChecked(True)
        display_layout.addWidget(self.show_label_checkbox)
        
        self.show_arrow_checkbox = QCheckBox("Show Arrows")
        self.show_arrow_checkbox.setChecked(True)
        display_layout.addWidget(self.show_arrow_checkbox)
        
        self.show_trail_checkbox = QCheckBox("Show Trails")
        self.show_trail_checkbox.setChecked(False)
        display_layout.addWidget(self.show_trail_checkbox)
        
        self.show_grid_checkbox = QCheckBox("Show Grid")
        self.show_grid_checkbox.setChecked(True)
        display_layout.addWidget(self.show_grid_checkbox)
        
        display_layout.addStretch()
        control_layout.addLayout(display_layout)
        
        # Second row: Refresh controls
        refresh_layout = QHBoxLayout()
        
        self.auto_refresh_checkbox = QCheckBox("Auto Refresh")
        self.auto_refresh_checkbox.setChecked(True)
        refresh_layout.addWidget(self.auto_refresh_checkbox)
        
        refresh_layout.addWidget(QLabel("Interval (s):"))
        self.refresh_slider = QSlider(Qt.Horizontal)
        self.refresh_slider.setMinimum(1)
        self.refresh_slider.setMaximum(10)
        self.refresh_slider.setValue(2)
        self.refresh_slider.setMaximumWidth(150)
        refresh_layout.addWidget(self.refresh_slider)
        
        self.refresh_label = QLabel("2s")
        self.refresh_label.setMinimumWidth(30)
        refresh_layout.addWidget(self.refresh_label)
        
        self.refresh_btn = QPushButton("🔄 Refresh Now")
        refresh_layout.addWidget(self.refresh_btn)
        
        self.reset_view_btn = QPushButton("🎯 Reset View")
        refresh_layout.addWidget(self.reset_view_btn)
        
        refresh_layout.addStretch()
        control_layout.addLayout(refresh_layout)
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)

        # ========== Info Bar (紧凑布局) ==========
        info_layout = QHBoxLayout()
        self.info_label = QLabel("USV Count: 0 | Range: - | Last Update: -")
        self.info_label.setStyleSheet("color: #34495e; font-size: 16px; padding: 3px; background-color: transparent;")
        self.info_label.setMaximumHeight(25)
        info_layout.addWidget(self.info_label)
        info_layout.addStretch()
        main_layout.addLayout(info_layout)

        # ========== matplotlib 画布和导航工具栏 ==========
        self.figure = Figure(facecolor='#ffffff')
        self.canvas = FigureCanvas(self.figure)
        
        # 设置画布大小策略：可扩展
        from PyQt5.QtWidgets import QSizePolicy
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.toolbar.setMaximumHeight(45)  # 限制工具栏高度
        
        # 设置工具栏样式 - 修复黑色按钮问题
        self.toolbar.setStyleSheet("""
            QToolBar {
                background-color: #ffffff;
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                padding: 5px;
                spacing: 3px;
            }
            QToolButton {
                background-color: #ecf0f1;
                color: #2c3e50;
                border: 1px solid #bdc3c7;
                border-radius: 3px;
                padding: 5px;
                margin: 2px;
                min-width: 30px;
                min-height: 30px;
            }
            QToolButton:hover {
                background-color: #3498db;
                color: white;
                border: 1px solid #2980b9;
            }
            QToolButton:pressed {
                background-color: #2980b9;
                border: 1px solid #21618c;
            }
            QToolButton:checked {
                background-color: #3498db;
                color: white;
                border: 1px solid #2980b9;
            }
        """)
        
        # 添加工具栏和画布，画布设置为扩展（占据最大空间）
        main_layout.addWidget(self.toolbar)
        main_layout.addWidget(self.canvas, stretch=1)  # stretch=1 使画布占据所有剩余空间

        self.get_usv_list_func = get_usv_list_func

        # ========== 数据存储 ==========
        self.usv_points = []  # 当前点的坐标和usv_id
        self.usv_trails = {}  # 存储每个USV的历史轨迹 {usv_id: [(x,y,z), ...]}
        self.max_trail_length = 50  # 最大轨迹点数

        # ========== 定时器 ==========
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.update_refresh_timer()

        # ========== 信号连接 ==========
        self.show_label_checkbox.stateChanged.connect(self.update_plot)
        self.show_arrow_checkbox.stateChanged.connect(self.update_plot)
        self.show_trail_checkbox.stateChanged.connect(self.update_plot)
        self.show_grid_checkbox.stateChanged.connect(self.update_plot)
        self.auto_refresh_checkbox.stateChanged.connect(self.on_auto_refresh_changed)
        self.refresh_slider.valueChanged.connect(self.on_refresh_interval_changed)
        self.refresh_btn.clicked.connect(self.update_plot)
        self.reset_view_btn.clicked.connect(self.reset_view)
        self.canvas.mpl_connect('button_press_event', self.on_click)

        self.update_plot()
        
        # 窗口居中显示
        self._center_on_screen()

    def update_refresh_timer(self):
        """更新定时器间隔"""
        interval = self.refresh_slider.value() * 1000  # 转换为毫秒
        self.timer.setInterval(interval)
        
    def on_auto_refresh_changed(self, state):
        """自动刷新开关"""
        if state == Qt.Checked:
            self.timer.start()
        else:
            self.timer.stop()
            
    def on_refresh_interval_changed(self, value):
        """刷新间隔改变"""
        self.refresh_label.setText(f"{value}s")
        self.update_refresh_timer()
        
    def reset_view(self):
        """重置3D视角"""
        self.update_plot()
        # 设置默认视角
        if hasattr(self.figure, 'axes') and len(self.figure.axes) > 0:
            ax = self.figure.axes[0]
            ax.view_init(elev=30, azim=45)
            self.canvas.draw()

    def update_plot(self):
        """更新3D绘图"""
        import datetime
        
        usv_list = self.get_usv_list_func()
        self.figure.clear()
        
        # 创建3D子图
        ax = self.figure.add_subplot(111, projection='3d')
        ax.set_facecolor('#fafafa')
        # Note: figure.patch is dynamic attribute in matplotlib
        try:
            self.figure.patch.set_facecolor('#ffffff')  # type: ignore
        except AttributeError:
            pass
        
        # 设置坐标轴标签
        ax.set_xlabel('X (m)', fontsize=10, color='#2c3e50', labelpad=10)
        ax.set_ylabel('Y (m)', fontsize=10, color='#2c3e50', labelpad=10)
        ax.set_zlabel('Z (m)', fontsize=10, color='#2c3e50', labelpad=10)
        ax.set_title('USV 3D Position & Heading', fontsize=12, color='#2c3e50', pad=15)
        
        # 设置网格
        if self.show_grid_checkbox.isChecked():
            ax.grid(True, linestyle='--', alpha=0.3, color='#7f8c8d')
        else:
            ax.grid(False)
        
        self.usv_points = []
        arrow_len = 1.0  # 箭头长度
        
        # 定义USV颜色方案（使用不同颜色区分不同USV）
        colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6', 
                  '#1abc9c', '#e67e22', '#34495e', '#16a085', '#c0392b']
        
        for idx, usv in enumerate(usv_list):
            pos = usv.get('position', {})
            x = pos.get('x', 0.0)
            y = pos.get('y', 0.0)
            z = pos.get('z', 0.0)
            usv_id = usv.get('usv_id', usv.get('namespace', ''))
            yaw = usv.get('yaw', 0.0)
            
            # 选择颜色
            color = colors[idx % len(colors)]
            
            # 更新轨迹数据
            if usv_id not in self.usv_trails:
                self.usv_trails[usv_id] = []
            self.usv_trails[usv_id].append((x, y, z))
            # 限制轨迹长度
            if len(self.usv_trails[usv_id]) > self.max_trail_length:
                self.usv_trails[usv_id] = self.usv_trails[usv_id][-self.max_trail_length:]
            
            # 绘制轨迹线
            if self.show_trail_checkbox.isChecked() and len(self.usv_trails[usv_id]) > 1:
                trail = self.usv_trails[usv_id]
                xs = [p[0] for p in trail]
                ys = [p[1] for p in trail]
                zs = [p[2] for p in trail]
                ax.plot(xs, ys, zs, color=color, alpha=0.3, linewidth=1.5, linestyle='--')
            
            # 绘制USV位置点（更大更醒目）
            ax.scatter(x, y, z, marker='o', color=color, s=150, 
                      edgecolors='white', linewidths=2, alpha=0.9, 
                      label=usv_id)
            
            # 绘制航向箭头
            if self.show_arrow_checkbox.isChecked():
                dx = arrow_len * cos(yaw)
                dy = arrow_len * sin(yaw)
                ax.quiver(x, y, z, dx, dy, 0, color=color, 
                         length=arrow_len, arrow_length_ratio=0.3, 
                         linewidth=2, alpha=0.8)
            
            # 绘制标注
            if self.show_label_checkbox.isChecked():
                label = f"{usv_id}\n({x:.2f}, {y:.2f}, {z:.2f})"
                ax.text(x, y, z + 0.3, label, fontsize=9, 
                       color='#2c3e50', weight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', 
                                facecolor='white', 
                                edgecolor=color, 
                                alpha=0.8))
                
            self.usv_points.append({'x': x, 'y': y, 'z': z, 'usv_id': usv_id, 'usv': usv})
        
        # 自动缩放到所有点
        if self.usv_points:
            xs = [p['x'] for p in self.usv_points]
            ys = [p['y'] for p in self.usv_points]
            zs = [p['z'] for p in self.usv_points]
            
            if xs and ys and zs:
                # 添加边距
                margin = 2.0
                x_range = max(xs) - min(xs) if len(set(xs)) > 1 else 10
                y_range = max(ys) - min(ys) if len(set(ys)) > 1 else 10
                z_range = max(zs) - min(zs) if len(set(zs)) > 1 else 5
                
                ax.set_xlim(min(xs) - margin, max(xs) + margin)
                ax.set_ylim(min(ys) - margin, max(ys) + margin)
                ax.set_zlim(min(zs) - margin/2, max(zs) + margin)
                
                # Display range info
                range_info = (f"X:[{min(xs):.1f}, {max(xs):.1f}] "
                            f"Y:[{min(ys):.1f}, {max(ys):.1f}] "
                            f"Z:[{min(zs):.1f}, {max(zs):.1f}]")
            else:
                range_info = "No data"
        else:
            # Set default range
            ax.set_xlim(-10, 10)
            ax.set_ylim(-10, 10)
            ax.set_zlim(-2, 5)
            range_info = "No USV"
        
        # Add legend (if data exists)
        if self.usv_points:
            ax.legend(loc='upper left', fontsize=9, framealpha=0.9)
        
        # Update info label
        current_time = datetime.datetime.now().strftime('%H:%M:%S')
        self.info_label.setText(
            f"USV Count: {len(self.usv_points)} | "
            f"Range: {range_info} | "
            f"Last Update: {current_time}"
        )
        
        # 设置视角
        ax.view_init(elev=25, azim=45)
        
        self.canvas.draw()

    def on_click(self, event):
        """处理点击事件（3D增强版）"""
        if event.button == 1 and event.inaxes:
            min_dist = float('inf')
            clicked_usv = None
            for point in self.usv_points:
                # 3D距离计算（降低z轴权重）
                dist = sqrt((event.xdata - point['x']) ** 2 + 
                           (event.ydata - point['y']) ** 2 +
                           (point['z'] ** 2) * 0.1)
                if dist < min_dist and dist < 3.0:  # 增大容差到3.0
                    min_dist = dist
                    clicked_usv = point
            
            if clicked_usv:
                usv_data = clicked_usv.get('usv', {})
                usv_id = clicked_usv.get('usv_id', '')
                pos = usv_data.get('position', {})
                
                # Build detailed info
                info_text = f"=== {usv_id} ===\n"
                info_text += f"Position: ({pos.get('x', 0):.3f}, {pos.get('y', 0):.3f}, {pos.get('z', 0):.3f})\n"
                try:
                    import math
                    yaw_rad = float(usv_data.get('yaw', 0.0))
                    yaw_deg = math.degrees(yaw_rad)
                    info_text += f"Heading: {yaw_deg:.2f}°\n"
                except Exception:
                    info_text += "Heading: Unknown\n"
                info_text += f"Mode: {usv_data.get('mode', 'N/A')}\n"
                info_text += f"Armed: {usv_data.get('armed', 'N/A')}\n"
                info_text += f"Battery: {usv_data.get('battery_voltage', 0):.2f}V ({usv_data.get('battery_percentage', 0):.1f}%)\n"
                info_text += f"Speed: {usv_data.get('speed', 0):.2f} m/s\n"
                
                # Show trail statistics if available
                if usv_id in self.usv_trails:
                    trail_len = len(self.usv_trails[usv_id])
                    total_dist = 0.0
                    trail = self.usv_trails[usv_id]
                    for i in range(1, len(trail)):
                        dx = trail[i][0] - trail[i-1][0]
                        dy = trail[i][1] - trail[i-1][1]
                        dz = trail[i][2] - trail[i-1][2]
                        total_dist += sqrt(dx*dx + dy*dy + dz*dz)
                    info_text += f"Trail Points: {trail_len}\n"
                    info_text += f"Total Distance: {total_dist:.2f}m"
                
                # Show info dialog
                from PyQt5.QtWidgets import QMessageBox
                msg_box = QMessageBox(self)
                msg_box.setWindowTitle(f"USV Details - {usv_id}")
                msg_box.setText(info_text)
                msg_box.setIcon(QMessageBox.Information)
                
                # 应用样式
                msg_box.setStyleSheet("""
                    QMessageBox {
                        background-color: #ffffff;
                    }
                    QLabel {
                        color: #2c3e50;
                        font-size: 16px;
                        padding: 10px;
                    }
                    QPushButton {
                        background-color: #3498db;
                        color: white;
                        border: none;
                        padding: 8px 20px;
                        border-radius: 4px;
                        font-size: 16px;
                        min-width: 80px;
                    }
                    QPushButton:hover {
                        background-color: #2980b9;
                    }
                """)
                
                msg_box.exec_()


    def _center_on_screen(self):
        """将窗口居中显示在屏幕上"""
        from PyQt5.QtWidgets import QApplication
        screen = QApplication.desktop().screenGeometry()
        size = self.geometry()
        self.move(
            (screen.width() - size.width()) // 2,
            (screen.height() - size.height()) // 2
        )
