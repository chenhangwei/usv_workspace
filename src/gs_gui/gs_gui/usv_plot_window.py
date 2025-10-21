from PyQt5.QtWidgets import QDialog, QVBoxLayout, QCheckBox, QHBoxLayout, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from matplotlib.figure import Figure
from PyQt5.QtCore import QTimer
from math import cos, sin
import mpl_toolkits.mplot3d.art3d  # 用于3D绘图

class UsvPlotWindow(QDialog):
    def __init__(self, get_usv_list_func, parent=None):
        super().__init__(parent)
        self.setWindowTitle("USV 3D 坐标显示")
        self.resize(800, 650)
        main_layout = QVBoxLayout(self)

        # 工具栏
        tool_layout = QHBoxLayout()
        self.show_label_checkbox = QCheckBox("显示编号和坐标")
        self.show_label_checkbox.setChecked(True)
        tool_layout.addWidget(self.show_label_checkbox)
        self.refresh_btn = QPushButton("刷新")
        tool_layout.addWidget(self.refresh_btn)
        tool_layout.addStretch()
        main_layout.addLayout(tool_layout)

        # matplotlib 画布和导航工具栏
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        main_layout.addWidget(self.toolbar)
        main_layout.addWidget(self.canvas)

        self.get_usv_list_func = get_usv_list_func

        # 定时器定期刷新
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(2000)  # 2秒刷新一次，减少刷新频率

        self.show_label_checkbox.stateChanged.connect(self.update_plot)
        self.refresh_btn.clicked.connect(self.update_plot)

        self.canvas.mpl_connect('button_press_event', self.on_click)

        self.usv_points = []  # 存储点的坐标和usv_id

        self.update_plot()

    def update_plot(self):
        usv_list = self.get_usv_list_func()
        self.figure.clear()
        ax = self.figure.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('USV 3D')
        self.usv_points = []
        arrow_len = 0.5  # 箭头长度，可调整
        for usv in usv_list:
            pos = usv.get('position', {})
            x = pos.get('x', 0.0)
            y = pos.get('y', 0.0)
            z = pos.get('z', 0.0)  # 添加z坐标支持
            usv_id = usv.get('usv_id', usv.get('namespace', ''))
            yaw = usv.get('yaw', 0.0)
            
            # 画原点
            ax.scatter(x, y, z, marker='o', color='b', s=50)  # 使用scatter绘制3D点
            
            # 绘制方向箭头（简化版本，仅显示yaw方向）
            arrow_len = 0.5
            dx = arrow_len * cos(yaw)
            dy = arrow_len * sin(yaw)
            # 在3D中绘制箭头
            ax.quiver(x, y, z, dx, dy, 0, color='r', length=arrow_len, arrow_length_ratio=0.3)
            
            # 标注
            if self.show_label_checkbox.isChecked():
                label = f"{usv_id}\n({x:.2f}, {y:.2f}, {z:.2f})"
                ax.text(x, y, z, label, fontsize=8)
                
            self.usv_points.append({'x': x, 'y': y, 'z': z, 'usv_id': usv_id, 'usv': usv})
        # 自动缩放到所有点
        if self.usv_points:
            xs = [p['x'] for p in self.usv_points]
            ys = [p['y'] for p in self.usv_points]
            zs = [p['z'] for p in self.usv_points]
            if xs and ys and zs:
                ax.set_xlim(min(xs), max(xs))
                ax.set_ylim(min(ys), max(ys))
                ax.set_zlim(min(zs), max(zs))
        self.canvas.draw()

    def on_click(self, event):
        # 鼠标左键点击点
        if event.button == 1 and event.inaxes:
            min_dist = float('inf')
            clicked_usv = None
            for point in self.usv_points:
                dist = ((event.xdata - point['x']) ** 2 + (event.ydata - point['y']) ** 2) ** 0.5
                if dist < min_dist and dist < 0.5:  # 0.5为点击容差，可调整
                    min_dist = dist
                    clicked_usv = point
            if clicked_usv:
                usv_id = clicked_usv['usv_id']
                x = clicked_usv['x']
                y = clicked_usv['y']
                # 你可以自定义弹窗或其他操作
                from PyQt5.QtWidgets import QMessageBox
                QMessageBox.information(self, "USV信息", f"USV: {usv_id}\n坐标: ({x:.2f}, {y:.2f})")