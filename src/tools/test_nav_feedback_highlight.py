#!/usr/bin/env python3
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import sys

import importlib.util
import os
import sys

# Load local copies of modules by path to avoid picking installed packages
root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ros_signal_path = os.path.join(root, 'gs_gui', 'gs_gui', 'ros_signal.py')
main_app_path = os.path.join(root, 'gs_gui', 'gs_gui', 'main_gui_app.py')

def load_module_from_path(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

ros_mod = load_module_from_path('local_ros_signal', ros_signal_path)
main_mod = load_module_from_path('local_main_gui_app', main_app_path)

ROSSignal = ros_mod.ROSSignal
MainWindow = main_mod.MainWindow
print(f"[TEST] loaded MainWindow from: {main_app_path}")

class FakeFeedback:
    def __init__(self, dist, heading_error=0.0, step=0, goal_id=1, estimated_time=5):
        self.distance_to_goal = dist
        self.heading_error = heading_error
        self.step = step
        self.goal_id = goal_id
        self.estimated_time = estimated_time


def main():
    app = QApplication(sys.argv)
    rs = ROSSignal()
    win = MainWindow(rs)
    win.show()

    # immediately send feedback once for debug (synchronous)
    fb = FakeFeedback(dist=0.5, heading_error=1.0, step=2, goal_id=3, estimated_time=2)
    print("[TEST] sending immediate feedback for usv_test")
    win.handle_navigation_feedback('usv_test', fb)
    # schedule a later feedbacks to test retrigger
    QTimer.singleShot(1500, lambda: (print("[TEST] sending feedback usv_test2"), win.handle_navigation_feedback('usv_test2', FakeFeedback(dist=0.4))))
    QTimer.singleShot(3000, lambda: (print("[TEST] sending feedback usv_test3"), win.handle_navigation_feedback('usv_test3', FakeFeedback(dist=0.2))))
    # Quit after 6 seconds
    QTimer.singleShot(6000, app.quit)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
