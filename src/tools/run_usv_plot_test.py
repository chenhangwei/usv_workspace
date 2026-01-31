#!/usr/bin/env python3
import sys
import time
# Ensure package path
sys.path.insert(0, '/home/chenhangwei/usv_workspace/src')

from PyQt5.QtWidgets import QApplication
import importlib.util, os
# Load the local copy of usv_plot_window directly to avoid picking up an installed package
spec = importlib.util.spec_from_file_location(
    "usv_plot_window",
    os.path.join('/home/chenhangwei/usv_workspace/src', 'gs_gui', 'gs_gui', 'usv_plot_window.py')
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
UsvPlotWindow = module.UsvPlotWindow

# Shared data structures
usv_list = [{
    'namespace': 'usv1',
    'connected': True,
    'position': {'x': 0.0, 'y': 0.0},
    'yaw': 0.0,
    'led_status': {'color': [0, 255, 128]},
    'velocity': {'linear': {'x': 0.0, 'y': 0.0}},
    # live nav target (will use led color if present)
    'nav_target_cache': {'x': 10.0, 'y': 0.0, 'reached': False, 'nav_mode': 1, 'led': 'color_select|0,255,128'}
}]

def get_usv_list():
    return usv_list

# Example preview task data containing led and nav_mode for each step
task_data = [
    {'usv_id': 'usv1', 'position': {'x': 5.0, 'y': -5.0}, 'nav_mode': 1, 'led': 'color_select|0,255,128', 'step': 1},
    {'usv_id': 'usv1', 'position': {'x': 10.0, 'y': 0.0}, 'nav_mode': 0, 'led': '#ff00ff', 'step': 2},
    {'usv_id': 'usv1', 'position': {'x': 15.0, 'y': 5.0}, 'nav_mode': 1, 'led': '0,0,0', 'step': 3},
]

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = UsvPlotWindow(get_usv_list)
    win.resize(900, 700)
    win.show()

    # Set preview path (will draw start/end/mid and '=' for sync points)
    win.set_preview_path(task_data, offset=(0.0, 0.0), angle=0.0)

    # Run the app
    sys.exit(app.exec_())
