"""
USV 3D 位置显示窗口模块。

使用 Matplotlib 提供 USV 集群的实时 3D 位置、航向及轨迹可视化。
"""

import math
import datetime
from typing import List, Dict, Any, Callable, Optional, Tuple

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QCheckBox, QHBoxLayout, 
    QPushButton, QLabel, QSlider, QGroupBox, QMessageBox,
    QSizePolicy, QApplication
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QIcon

# Matplotlib 集成
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# 配置 matplotlib 中文显示
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS', 'WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


class UsvPlotWindow(QDialog):
    """
    USV 3D 绘图窗口类。
    
    用于实时展示所有在线 USV 的 3D 坐标、航向箭头以及历史轨迹。
    """

    def __init__(self, get_usv_list_func: Callable[[], List[Dict[str, Any]]], parent: Optional[Any] = None):
        """
        初始化 3D 绘图窗口。
        
        Args:
            get_usv_list_func: 获取当前所有 USV 状态列表的回调函数。
            parent: 父窗口。
        """
        super().__init__(parent)
        self.setWindowTitle("USV 3D 集群监控")
        self.resize(1000, 750)
        
        self.get_usv_list_func = get_usv_list_func
        
        # 数据存储
        self.usv_points: List[Dict[str, Any]] = []  # 当前点的坐标和 usv_id
        self.usv_trails: Dict[str, List[Tuple[float, float, float]]] = {}  # 存储每个 USV 的历史轨迹
        self.max_trail_length = 100  # 最大轨迹点数
        
        # 初始化 UI
        self._init_ui()
        
        # 定时器
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self._update_refresh_timer()
        
        # 初始绘图
        self.update_plot()
        
        # 窗口居中
        self._center_on_screen()

    def _init_ui(self) -> None:
        """初始化用户界面。"""
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
                font-size: 14px;
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
            }
            QPushButton {
                background-color: #3498db;
                color: #ffffff;
                border: none;
                padding: 8px 15px;
                border-radius: 5px;
                font-size: 14px;
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
                font-size: 14px;
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
        """)
        
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(5)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # ========== 控制面板 ==========
        control_group = QGroupBox("显示控制")
        control_group.setMaximumHeight(130)
        control_layout = QVBoxLayout()
        control_layout.setSpacing(5)
        
        # 第一行：显示选项
        display_layout = QHBoxLayout()
        
        self.show_label_checkbox = QCheckBox("显示标签")
        self.show_label_checkbox.setChecked(True)
        display_layout.addWidget(self.show_label_checkbox)
        
        self.show_arrow_checkbox = QCheckBox("显示航向")
        self.show_arrow_checkbox.setChecked(True)
        display_layout.addWidget(self.show_arrow_checkbox)
        
        self.show_trail_checkbox = QCheckBox("显示轨迹")
        self.show_trail_checkbox.setChecked(False)
        display_layout.addWidget(self.show_trail_checkbox)
        
        self.show_grid_checkbox = QCheckBox("显示网格")
        self.show_grid_checkbox.setChecked(True)
        display_layout.addWidget(self.show_grid_checkbox)
        
        display_layout.addStretch()
        control_layout.addLayout(display_layout)
        
        # 第二行：刷新控制
        refresh_layout = QHBoxLayout()
        
        self.auto_refresh_checkbox = QCheckBox("自动刷新")
        self.auto_refresh_checkbox.setChecked(True)
        refresh_layout.addWidget(self.auto_refresh_checkbox)
        
        refresh_layout.addWidget(QLabel("间隔 (s):"))
        self.refresh_slider = QSlider(Qt.Horizontal)
        self.refresh_slider.setMinimum(1)
        self.refresh_slider.setMaximum(10)
        self.refresh_slider.setValue(2)
        self.refresh_slider.setMaximumWidth(150)
        refresh_layout.addWidget(self.refresh_slider)
        
        self.refresh_label = QLabel("2s")
        self.refresh_label.setMinimumWidth(30)
        refresh_layout.addWidget(self.refresh_label)
        
        self.refresh_btn = QPushButton("🔄 立即刷新")
        refresh_layout.addWidget(self.refresh_btn)
        
        self.reset_view_btn = QPushButton("🎯 重置视角")
        refresh_layout.addWidget(self.reset_view_btn)
        
        refresh_layout.addStretch()
        control_layout.addLayout(refresh_layout)
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)

        # ========== 信息栏 ==========
        info_layout = QHBoxLayout()
        self.info_label = QLabel("USV 数量: 0 | 范围: - | 最后更新: -")
        self.info_label.setStyleSheet("color: #34495e; font-weight: bold;")
        info_layout.addWidget(self.info_label)
        info_layout.addStretch()
        main_layout.addLayout(info_layout)

        # ========== Matplotlib 画布 ==========
        self.figure = Figure(facecolor='#ffffff')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.toolbar.setMaximumHeight(45)
        self.toolbar.setStyleSheet("""
            QToolBar {
                background-color: #ffffff;
                border: 1px solid #e0e0e0;
                border-radius: 4px;
                padding: 2px;
            }
            QToolButton {
                padding: 5px;
                margin: 2px;
            }
        """)
        
        main_layout.addWidget(self.toolbar)
        main_layout.addWidget(self.canvas, stretch=1)

        # 信号连接
        self.show_label_checkbox.stateChanged.connect(self.update_plot)
        self.show_arrow_checkbox.stateChanged.connect(self.update_plot)
        self.show_trail_checkbox.stateChanged.connect(self.update_plot)
        self.show_grid_checkbox.stateChanged.connect(self.update_plot)
        self.auto_refresh_checkbox.stateChanged.connect(self._on_auto_refresh_changed)
        self.refresh_slider.valueChanged.connect(self._on_refresh_interval_changed)
        self.refresh_btn.clicked.connect(self.update_plot)
        self.reset_view_btn.clicked.connect(self.reset_view)
        self.canvas.mpl_connect('button_press_event', self._on_click)

    def _update_refresh_timer(self) -> None:
        """更新定时器间隔。"""
        interval = self.refresh_slider.value() * 1000
        self.timer.setInterval(interval)
        if self.auto_refresh_checkbox.isChecked():
            self.timer.start()
        
    def _on_auto_refresh_changed(self, state: int) -> None:
        """
        处理自动刷新开关状态改变。
        
        Args:
            state: Qt.Checked 或 Qt.Unchecked。
        """
        if state == Qt.Checked:
            self.timer.start()
        else:
            self.timer.stop()
            
    def _on_refresh_interval_changed(self, value: int) -> None:
        """
        处理刷新间隔改变。
        
        Args:
            value: 新的间隔秒数。
        """
        self.refresh_label.setText(f"{value}s")
        self._update_refresh_timer()
        
    def reset_view(self) -> None:
        """重置 3D 视角到默认值。"""
        if hasattr(self.figure, 'axes') and len(self.figure.axes) > 0:
            ax = self.figure.axes[0]
            ax.view_init(elev=30, azim=45)
            self.canvas.draw()
        else:
            self.update_plot()

    def update_plot(self) -> None:
        """执行 3D 绘图更新。"""
        usv_list = self.get_usv_list_func()
        self.figure.clear()
        
        # 创建 3D 子图
        ax = self.figure.add_subplot(111, projection='3d')
        ax.set_facecolor('#fafafa')
        
        # 设置坐标轴标签
        ax.set_xlabel('X (m)', fontsize=10, color='#2c3e50', labelpad=10)
        ax.set_ylabel('Y (m)', fontsize=10, color='#2c3e50', labelpad=10)
        ax.set_zlabel('Z (m)', fontsize=10, color='#2c3e50', labelpad=10)
        ax.set_title('USV 集群 3D 位置与航向', fontsize=12, color='#2c3e50', pad=15)
        
        # 设置网格
        ax.grid(self.show_grid_checkbox.isChecked(), linestyle='--', alpha=0.3)
        
        self.usv_points = []
        arrow_len = 1.2
        
        # 颜色方案
        colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6', 
                  '#1abc9c', '#e67e22', '#34495e', '#16a085', '#c0392b']
        
        for idx, usv in enumerate(usv_list):
            pos = usv.get('position', {})
            x = float(pos.get('x', 0.0))
            y = float(pos.get('y', 0.0))
            z = float(pos.get('z', 0.0))
            usv_id = str(usv.get('usv_id', usv.get('namespace', 'Unknown')))
            yaw = float(usv.get('yaw', 0.0))
            
            color = colors[idx % len(colors)]
            
            # 更新轨迹
            if usv_id not in self.usv_trails:
                self.usv_trails[usv_id] = []
            self.usv_trails[usv_id].append((x, y, z))
            if len(self.usv_trails[usv_id]) > self.max_trail_length:
                self.usv_trails[usv_id].pop(0)
            
            # 绘制轨迹
            if self.show_trail_checkbox.isChecked() and len(self.usv_trails[usv_id]) > 1:
                trail = self.usv_trails[usv_id]
                xs_t = [p[0] for p in trail]
                ys_t = [p[1] for p in trail]
                zs_t = [p[2] for p in trail]
                ax.plot(xs_t, ys_t, zs_t, color=color, alpha=0.4, linewidth=1.5, linestyle='--')
            
            # 绘制位置点
            ax.scatter(x, y, z, marker='o', color=color, s=150, 
                      edgecolors='white', linewidths=2, alpha=0.9, 
                      label=usv_id)
            
            # 绘制航向箭头
            if self.show_arrow_checkbox.isChecked():
                dx = arrow_len * math.cos(yaw)
                dy = arrow_len * math.sin(yaw)
                ax.quiver(x, y, z, dx, dy, 0, color=color, 
                         length=arrow_len, arrow_length_ratio=0.3, 
                         linewidth=2, alpha=0.8)
            
            # 绘制标签
            if self.show_label_checkbox.isChecked():
                label_text = f"{usv_id}\n({x:.1f}, {y:.1f}, {z:.1f})"
                ax.text(x, y, z + 0.4, label_text, fontsize=9, 
                       color='#2c3e50', weight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', 
                                facecolor='white', 
                                edgecolor=color, 
                                alpha=0.7))
                
            self.usv_points.append({'x': x, 'y': y, 'z': z, 'usv_id': usv_id, 'usv': usv})
        
        # 自动缩放
        if self.usv_points:
            xs = [p['x'] for p in self.usv_points]
            ys = [p['y'] for p in self.usv_points]
            zs = [p['z'] for p in self.usv_points]
            
            margin = 3.0
            ax.set_xlim(min(xs) - margin, max(xs) + margin)
            ax.set_ylim(min(ys) - margin, max(ys) + margin)
            ax.set_zlim(min(zs) - 1.0, max(zs) + margin)
            
            range_info = (f"X:[{min(xs):.1f}, {max(xs):.1f}] "
                        f"Y:[{min(ys):.1f}, {max(ys):.1f}] "
                        f"Z:[{min(zs):.1f}, {max(zs):.1f}]")
            ax.legend(loc='upper left', fontsize=9, framealpha=0.8)
        else:
            ax.set_xlim(-10, 10)
            ax.set_ylim(-10, 10)
            ax.set_zlim(-2, 5)
            range_info = "无数据"
        
        # 更新信息栏
        current_time = datetime.datetime.now().strftime('%H:%M:%S')
        self.info_label.setText(
            f"USV 数量: {len(self.usv_points)} | "
            f"范围: {range_info} | "
            f"最后更新: {current_time}"
        )
        
        self.canvas.draw()

    def _on_click(self, event: Any) -> None:
        """
        处理画布点击事件，显示 USV 详细信息。
        
        Args:
            event: Matplotlib 点击事件。
        """
        if event.button == 1 and event.inaxes:
            # 寻找最近的 USV
            min_dist = float('inf')
            clicked_usv = None
            for point in self.usv_points:
                # 简单的 2D 投影距离判断
                dist = math.sqrt((event.xdata - point['x']) ** 2 + 
                                (event.ydata - point['y']) ** 2)
                if dist < min_dist and dist < 2.5:
                    min_dist = dist
                    clicked_usv = point
            
            if clicked_usv:
                self._show_usv_details(clicked_usv)

    def _show_usv_details(self, point_data: Dict[str, Any]) -> None:
        """
        弹出对话框显示 USV 详细状态。
        
        Args:
            point_data: 包含 USV 数据的字典。
        """
        usv_data = point_data.get('usv', {})
        usv_id = point_data.get('usv_id', 'Unknown')
        pos = usv_data.get('position', {})
        
        # 构建详细信息文本
        info_text = f"<b>=== {usv_id} 详细状态 ===</b><br><br>"
        info_text += f"位置: ({pos.get('x', 0):.3f}, {pos.get('y', 0):.3f}, {pos.get('z', 0):.3f}) m<br>"
        
        yaw_rad = float(usv_data.get('yaw', 0.0))
        yaw_deg = math.degrees(yaw_rad)
        info_text += f"航向: {yaw_deg:.2f}°<br>"
        
        info_text += f"模式: {usv_data.get('mode', '未知')}<br>"
        info_text += f"解锁: {'已解锁' if usv_data.get('armed') else '未解锁'}<br>"
        info_text += f"电池: {usv_data.get('battery_voltage', 0):.2f}V ({usv_data.get('battery_percentage', 0):.1f}%)<br>"
        info_text += f"速度: {usv_data.get('speed', 0):.2f} m/s<br>"
        
        if usv_id in self.usv_trails:
            trail = self.usv_trails[usv_id]
            total_dist = 0.0
            for i in range(1, len(trail)):
                total_dist += math.sqrt(
                    (trail[i][0] - trail[i-1][0])**2 + 
                    (trail[i][1] - trail[i-1][1])**2 + 
                    (trail[i][2] - trail[i-1][2])**2
                )
            info_text += f"<br>轨迹点数: {len(trail)}<br>"
            info_text += f"累计里程: {total_dist:.2f} m"
        
        # 显示消息框
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle(f"USV 详情 - {usv_id}")
        msg_box.setTextFormat(Qt.TextFormat.RichText)
        msg_box.setText(info_text)
        msg_box.setIcon(QMessageBox.Icon.Information)
        
        msg_box.setStyleSheet("""
            QMessageBox {
                background-color: #ffffff;
            }
            QLabel {
                color: #2c3e50;
                font-size: 15px;
                padding: 10px;
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                padding: 8px 20px;
                border-radius: 4px;
                font-size: 14px;
                min-width: 80px;
            }
        """)
        msg_box.exec_()

    def _center_on_screen(self) -> None:
        """将窗口居中显示在屏幕上。"""
        frame_gm = self.frameGeometry()
        screen = QApplication.desktop().screenNumber(QApplication.desktop().cursor().pos())
        center_point = QApplication.desktop().screenGeometry(screen).center()
        frame_gm.moveCenter(center_point)
        self.move(frame_gm.topLeft())
