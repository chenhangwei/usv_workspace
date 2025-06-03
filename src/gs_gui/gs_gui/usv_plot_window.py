from PyQt5.QtWidgets import QDialog, QVBoxLayout, QCheckBox, QHBoxLayout, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT
from matplotlib.figure import Figure
from PyQt5.QtCore import QTimer
from math import cos, sin

class UsvPlotWindow(QDialog):
    def __init__(self, get_usv_list_func, parent=None):
        super().__init__(parent)
        self.setWindowTitle("USV 2D 坐标显示")
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
        self.timer.start(1000)  # 1秒刷新一次

        self.show_label_checkbox.stateChanged.connect(self.update_plot)
        self.refresh_btn.clicked.connect(self.update_plot)

        self.canvas.mpl_connect('button_press_event', self.on_click)

        self.usv_points = []  # 存储点的坐标和usv_id

        self.update_plot()

    def update_plot(self):
        usv_list = self.get_usv_list_func()
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('USV 2D')
        ax.grid(True)
        self.usv_points = []
        arrow_len = 0.5  # 箭头长度，可调整
        for usv in usv_list:
            pos = usv.get('position', {})
            x = pos.get('x', 0.0)
            y = pos.get('y', 0.0)
            usv_id = usv.get('usv_id', usv.get('namespace', ''))
            yaw = usv.get('yaw', 0.0)
            # 计算yaw方向的x轴箭头
            dx_x = arrow_len * cos(yaw)
            dy_x = arrow_len * sin(yaw)
            # y轴箭头比x轴多90度
            dx_y = arrow_len * cos(yaw + 1.5708)
            dy_y = arrow_len * sin(yaw + 1.5708)
            # 画原点
            ax.plot(x, y, marker='o', color='b')
            # 画x轴箭头（红色）
            ax.arrow(x, y, dx_x, dy_x, head_width=0.08, head_length=0.12, fc='r', ec='r', length_includes_head=True)
            # 画y轴箭头（绿色）
            ax.arrow(x, y, dx_y, dy_y, head_width=0.08, head_length=0.12, fc='g', ec='g', length_includes_head=True)
            # 标注
            if self.show_label_checkbox.isChecked():
                label = f"{usv_id}\n({x:.2f},{y:.2f})"
                ax.text(x, y-0.15, label, fontsize=9, ha='center', va='top')
            self.usv_points.append({'x': x, 'y': y, 'usv_id': usv_id, 'usv': usv})
        # 自动缩放到所有点
        xs = [p['x'] for p in self.usv_points]
        ys = [p['y'] for p in self.usv_points]
        if xs and ys:
            ax.relim()
            ax.autoscale_view()
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