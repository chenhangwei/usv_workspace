"""
UI工具模块
负责UI辅助功能，包括信息显示、绘图窗口等
"""
import weakref
from collections import deque
from PyQt5.QtCore import QTimer, QProcess, QMetaObject, Qt, Q_ARG
from gs_gui.usv_plot_window import UsvPlotWindow


class UIUtils:
    """UI工具类，提供各种UI辅助功能"""
    
    def __init__(self, ui, parent_widget):
        """
        初始化UI工具
        
        Args:
            ui: UI对象
            parent_widget: 父窗口部件
        """
        self.ui = ui
        self.parent_widget = parent_widget
        
        # 日志缓冲：用于限制 info_textEdit 的写入速率和最大行数
        self._info_buffer = deque()
        self._info_max_lines = 500
        self._info_flush_interval_ms = 500
        
        # 警告缓冲：用于线程安全的警告显示
        self._warning_buffer = deque()
        self._warning_max_lines = 200
        
        # info定时器
        self._info_timer = QTimer()
        self._info_timer.setInterval(self._info_flush_interval_ms)
        self._info_timer.timeout.connect(self._flush_info_buffer)
        self._info_timer.start()
        
        # warning定时器（同一频率刷新）
        self._warning_timer = QTimer()
        self._warning_timer.setInterval(self._info_flush_interval_ms)
        self._warning_timer.timeout.connect(self._flush_warning_buffer)
        self._warning_timer.start()
        
        # USV绘图窗口的弱引用
        self.usv_plot_window_ref = None
        
        # RViz进程
        self._rviz_process = None
    
    def append_info(self, text: str):
        """
        将文本放入 info 缓冲，由定时器周期性刷新到 UI
        
        Args:
            text: 要显示的信息文本
        """
        try:
            self._info_buffer.append(str(text))
        except Exception:
            # 如果缓冲出错，作为回退直接写入
            try:
                self.ui.info_textEdit.append(str(text))
            except Exception:
                pass
    
    def append_warning(self, text: str):
        """
        将警告文本放入缓冲，由定时器周期性刷新到 UI（线程安全）
        
        Args:
            text: 警告文本
        """
        try:
            self._warning_buffer.append(str(text))
        except Exception:
            pass
    
    def _flush_warning_buffer(self):
        """定时器回调：把缓冲中的若干行追加到 warning_textEdit"""
        try:
            if not self._warning_buffer:
                return
            
            # 合并若干条消息一次性写入
            to_write = []
            max_batch = 20
            for _ in range(min(max_batch, len(self._warning_buffer))):
                to_write.append(self._warning_buffer.popleft())
            
            for line in to_write:
                try:
                    self.ui.warning_textEdit.append(line)
                except Exception:
                    pass
            
            # 控制最大行数
            try:
                doc = self.ui.warning_textEdit.document()
                if doc is not None:
                    block_count = doc.blockCount()
                    if block_count > self._warning_max_lines:
                        cursor = self.ui.warning_textEdit.textCursor()
                        cursor.movePosition(cursor.Start)
                        remove_count = block_count - self._warning_max_lines
                        for _ in range(remove_count):
                            cursor.select(cursor.LineUnderCursor)
                            cursor.removeSelectedText()
                            cursor.deleteChar()
            except Exception:
                pass
        except Exception:
            try:
                self._warning_buffer.clear()
            except Exception:
                pass
    
    def _flush_info_buffer(self):
        """定时器回调：把缓冲中的若干行追加到 info_textEdit"""
        try:
            if not self._info_buffer:
                return
            
            # 合并若干条消息一次性写入，减少 UI 更新次数
            to_write = []
            max_batch = 50
            for _ in range(min(max_batch, len(self._info_buffer))):
                to_write.append(self._info_buffer.popleft())
            
            for line in to_write:
                try:
                    self.ui.info_textEdit.append(line)
                except Exception:
                    pass
            
            # 控制最大行数
            try:
                doc = self.ui.info_textEdit.document()
                if doc is not None:
                    block_count = doc.blockCount()
                    if block_count > self._info_max_lines:
                        cursor = self.ui.info_textEdit.textCursor()
                        cursor.movePosition(cursor.Start)
                        remove_count = block_count - self._info_max_lines
                        for _ in range(remove_count):
                            cursor.select(cursor.LineUnderCursor)
                            cursor.removeSelectedText()
                            cursor.deleteChar()
            except Exception:
                pass
        
        except Exception:
            try:
                self._info_buffer.clear()
            except Exception:
                pass
    
    def get_or_create_plot_window(self, usv_online_list_getter):
        """
        获取或创建 USV 绘图窗口实例
        
        Args:
            usv_online_list_getter: 获取在线USV列表的回调函数
            
        Returns:
            UsvPlotWindow: 窗口实例
        """
        if self.usv_plot_window_ref and self.usv_plot_window_ref():
            return self.usv_plot_window_ref()
        
        # 如果不存在，创建新窗口并保存弱引用
        usv_plot_window = UsvPlotWindow(usv_online_list_getter, self.parent_widget)
        self.usv_plot_window_ref = weakref.ref(usv_plot_window)
        return usv_plot_window

    def show_usv_plot_window(self, usv_online_list_getter):
        """
        显示 USV 绘图窗口
        
        Args:
            usv_online_list_getter: 获取在线USV列表的回调函数
            
        Returns:
            UsvPlotWindow: 窗口实例
        """
        window = self.get_or_create_plot_window(usv_online_list_getter)
        
        if not window.isVisible():
            window.show()
        
        if window.isMinimized():
            window.showNormal()
            
        window.raise_()
        window.activateWindow()
        return window
    
    def start_rviz(self):
        """启动 RViz2"""
        self._rviz_process = QProcess(self.parent_widget)
        self._rviz_process.start("rviz2")
        self.append_info("启动RViz2")
    
    def update_selected_table_row(self, table_manager, state_handler):
        """
        更新选中行数据
        
        Args:
            table_manager: 表格管理器
            state_handler: 状态处理器
        """
        try:
            # 获取选中的行
            selected_indexes = self.ui.cluster_tableView.selectedIndexes()
            if not selected_indexes:
                return
            
            selected_row = selected_indexes[0].row()
            model = self.ui.cluster_tableView.model()
            if model is None:
                return
            
            index0 = model.index(selected_row, 0)
            namespace = model.data(index0) if index0.isValid() else None
            if not namespace:
                return
            
            # 获取最新状态
            state = state_handler.get_usv_state(namespace)
            if state is None:
                # 无状态数据时清空显示
                self.ui.usv_id_label.setText("Unknown")
                self.ui.usv_x_label.setText("Unknown")
                self.ui.usv_y_label.setText("Unknown")
                self.ui.usv_z_label.setText("Unknown")
                self.ui.usv_yaw_label.setText("Unknown")
                return
            
            # 更新显示
            self.ui.usv_id_label.setText(str(state.get('namespace', 'Unknown')))
            pos = state.get('position', {}) or {}
            
            try:
                x = float(pos.get('x', 0.0))
            except Exception:
                x = None
            try:
                y = float(pos.get('y', 0.0))
            except Exception:
                y = None
            try:
                z = float(pos.get('z', 0.0))
            except Exception:
                z = None
            
            yaw_val = state.get('yaw')
            try:
                yaw_val = float(yaw_val) if yaw_val is not None else None
            except Exception:
                yaw_val = None
            
            self.ui.usv_x_label.setText(f"{x:.2f}" if x is not None else "Unknown")
            self.ui.usv_y_label.setText(f"{y:.2f}" if y is not None else "Unknown")
            self.ui.usv_z_label.setText(f"{z:.2f}" if z is not None else "Unknown")
            self.ui.usv_yaw_label.setText(f"{yaw_val:.2f}" if yaw_val is not None else "Unknown")
        
        except Exception as e:
            try:
                self.append_info(f"错误：获取选中行数据失败 - {str(e)}")
            except Exception:
                pass
            self.ui.usv_id_label.setText("Unknown")
            self.ui.usv_x_label.setText("Unknown")
            self.ui.usv_y_label.setText("Unknown")
            self.ui.usv_z_label.setText("Unknown")
            self.ui.usv_yaw_label.setText("Unknown")
    
    def stop_timers(self):
        """停止所有定时器"""
        if self._info_timer:
            self._info_timer.stop()
