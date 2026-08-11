#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电子围栏对话框测试。"""

import importlib.util
import pathlib
import sys
import types

from PyQt5.QtWidgets import QApplication, QLabel


if not QApplication.instance():
    app = QApplication(sys.argv)


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1] / 'gs_gui'


def _load_geofence_dialog_class():
    """在不触发 gs_gui.__init__ 副作用的情况下加载对话框模块。"""
    package = types.ModuleType('gs_gui')
    package.__path__ = [str(PACKAGE_DIR)]
    sys.modules['gs_gui'] = package

    style_spec = importlib.util.spec_from_file_location('gs_gui.style_manager', PACKAGE_DIR / 'style_manager.py')
    style_module = importlib.util.module_from_spec(style_spec)
    sys.modules['gs_gui.style_manager'] = style_module
    style_spec.loader.exec_module(style_module)

    dialog_spec = importlib.util.spec_from_file_location('gs_gui.geofence_dialog', PACKAGE_DIR / 'geofence_dialog.py')
    dialog_module = importlib.util.module_from_spec(dialog_spec)
    sys.modules['gs_gui.geofence_dialog'] = dialog_module
    dialog_spec.loader.exec_module(dialog_module)
    return dialog_module.GeofenceDialog


def test_geofence_dialog_shows_purpose_and_usage_text():
    """对话框应直接展示中文作用说明和使用步骤。"""
    geofence_dialog_class = _load_geofence_dialog_class()
    dialog = geofence_dialog_class(
        current_bounds={'x_min': -10.0, 'x_max': 10.0, 'y_min': -20.0, 'y_max': 20.0},
        current_enabled=True,
    )

    labels = dialog.findChildren(QLabel)
    info_text = "\n".join(label.text() for label in labels)

    assert "作用：设置 USV 在本地坐标系中的矩形安全活动范围。" in info_text
    assert "使用方法：" in info_text
    assert "勾选“启用电子围栏监控”打开围栏保护" in info_text
    assert "X 最小值:" in info_text
    assert "Y 最大值:" in info_text
