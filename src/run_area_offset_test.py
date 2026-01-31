#!/usr/bin/env python3
"""
Simple runner to manually test AreaOffsetDialog UI and DB integration.
Run from the repo's `src` directory: `python3 run_area_offset_test.py`
"""
import sys
import traceback

from PyQt5.QtWidgets import QApplication

def main():
    try:
        # ensure package path
        app = QApplication(sys.argv)

        # import dialog by file path to avoid package import issues
        import os
        import importlib.util

        base_pkg_dir = os.path.join(os.path.dirname(__file__), 'gs_gui', 'gs_gui')

        # ensure package modules exist in sys.modules so that relative imports work
        import types
        pkg_root = os.path.join(os.path.dirname(__file__), 'gs_gui')
        inner_pkg_dir = os.path.join(pkg_root, 'gs_gui')
        pkg_mod = types.ModuleType('gs_gui')
        pkg_mod.__path__ = [pkg_root]
        sys.modules['gs_gui'] = pkg_mod
        inner_mod = types.ModuleType('gs_gui.gs_gui')
        inner_mod.__path__ = [inner_pkg_dir]
        sys.modules['gs_gui.gs_gui'] = inner_mod

        # load location_db module
        loc_path = os.path.join(base_pkg_dir, 'location_db.py')
        spec_loc = importlib.util.spec_from_file_location('gs_gui.gs_gui.location_db', loc_path)
        loc_mod = importlib.util.module_from_spec(spec_loc)
        spec_loc.loader.exec_module(loc_mod)
        sys.modules['gs_gui.gs_gui.location_db'] = loc_mod

        dlg_path = os.path.join(base_pkg_dir, 'area_offset_dialog.py')
        spec = importlib.util.spec_from_file_location('gs_gui.gs_gui.area_offset_dialog', dlg_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        AreaOffsetDialog = mod.AreaOffsetDialog

        class DummyParent:
            def get_selected_usv_position(self):
                return {'usv_id': 'USV_TEST', 'x': 12.34, 'y': -5.67, 'z': 0.0}

        dlg = AreaOffsetDialog(parent=None)
        dlg.parent_window = DummyParent()
        res = dlg.exec_()
        if res == 1:
            print("Dialog accepted. Offset:", dlg.get_offset())
        else:
            print("Dialog cancelled.")

    except Exception:
        traceback.print_exc()

if __name__ == '__main__':
    main()
