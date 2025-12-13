"""
表格管理模块
负责集群和离群USV表格的显示和更新
"""

import logging
from PyQt5.QtGui import QStandardItemModel, QStandardItem

_logger = logging.getLogger("gs_gui.table")



class TableManager:
    """表格管理器，负责USV表格的创建和更新"""
    
    # 表格列标题
    TABLE_HEADERS = ["编号", "当前模式", "连接状态", "武装状态", "引导模式状态", '电压V', '电量%', 
                     '电池状态', '坐标', '速度', '偏角', '导航状态', '温度']
    
    # 导航状态常量
    NAV_STATUS_UNKNOWN = "未知"
    NAV_STATUS_IDLE = "空闲"
    NAV_STATUS_ACTIVE = "执行中"
    NAV_STATUS_SUCCEEDED = "成功"
    NAV_STATUS_FAILED = "失败"
    
    def __init__(self, cluster_table_view, departed_table_view):
        """
        初始化表格管理器
        
        Args:
            cluster_table_view: 集群表格视图
            departed_table_view: 离群表格视图
        """
        self.cluster_table_view = cluster_table_view
        self.departed_table_view = departed_table_view
        
        # 初始化表格模型
        self.cluster_table_model = QStandardItemModel()
        self.departed_table_model = QStandardItemModel()
        
        # 设置表格模型
        self.cluster_table_view.setModel(self.cluster_table_model)
        self.departed_table_view.setModel(self.departed_table_model)
        
        # 设置表头
        self.refresh_table_header()
        
        # 启用表格排序功能
        self.cluster_table_view.setSortingEnabled(True)
        self.departed_table_view.setSortingEnabled(True)
    
    @staticmethod
    def map_power_supply_status(status):
        """
        将电池状态数字映射成文字
        0:未知 1:充电 2:放电 3:充满 4:未充电
        
        Args:
            status: 电池状态数字
            
        Returns:
            str: 对应的文字描述
        """
        status_map = {
            0: "未知",
            1: "充电",
            2: "放电",
            3: "充满",
            4: "未充电"
        }
        try:
            return status_map.get(int(status), "未知")
        except (ValueError, TypeError):
            return "未知"
    
    def refresh_table_header(self):
        """刷新表格表头"""
        self.cluster_table_model.setHorizontalHeaderLabels(self.TABLE_HEADERS)
        self.departed_table_model.setHorizontalHeaderLabels(self.TABLE_HEADERS)
    
    def update_cluster_table(self, state_list, usv_nav_status):
        """
        更新集群表格
        
        Args:
            state_list: 状态列表
            usv_nav_status: USV导航状态字典
        """
        try:
            model = self.cluster_table_model

            # 过滤掉离线的 USV（只显示 connected=True 的）
            online_state_list = [
                s for s in state_list
                if isinstance(s, dict) and s.get('connected', False)
            ]

            # 构建当前表中 namespace -> row 的映射
            current_ns_to_row = {}
            for row in range(model.rowCount()):
                item = model.item(row, 0)
                ns = item.text() if item is not None else ''
                current_ns_to_row[ns] = row

            # 新状态的 namespace 列表与集合
            new_ns_list = [s.get('namespace') for s in online_state_list if isinstance(s, dict) and s.get('namespace')]
            new_ns_set = set(new_ns_list)

            # 删除在当前表中但不在新状态集合的行（从后往前删除以保持索引正确）
            rows_to_remove = [r for ns, r in current_ns_to_row.items() if ns not in new_ns_set]
            for r in sorted(rows_to_remove, reverse=True):
                model.removeRow(r)

            # 重新构建映射（因为删除可能改变索引）
            current_ns_to_row = {}
            for row in range(model.rowCount()):
                item = model.item(row, 0)
                ns = item.text() if item is not None else ''
                current_ns_to_row[ns] = row

            # 更新已有行或追加新行
            for state in online_state_list:
                if not isinstance(state, dict):
                    continue
                ns = state.get('namespace', 'Unknown')
                
                # 格式化各列数据
                cells = self._format_table_cells(state, usv_nav_status)

                if ns in current_ns_to_row:
                    row = current_ns_to_row[ns]
                    # 只更新变化的列，减少重绘
                    for col, text in enumerate(cells):
                        existing_item = model.item(row, col)
                        existing_text = existing_item.text() if existing_item is not None else None
                        if existing_text != str(text):
                            model.setItem(row, col, QStandardItem(str(text)))
                else:
                    # 追加新行
                    row = model.rowCount()
                    model.insertRow(row)
                    for col, text in enumerate(cells):
                        model.setItem(row, col, QStandardItem(str(text)))

        except Exception as e:
            _logger.error(f"更新集群表格失败: {e}")
    
    def update_departed_table(self, state_list, usv_nav_status):
        """
        更新离群表格
        
        Args:
            state_list: 状态列表
            usv_nav_status: USV导航状态字典
        """
        try:
            model = self.departed_table_model

            # 过滤掉离线的 USV（只显示 connected=True 的）
            online_state_list = [
                s for s in state_list
                if isinstance(s, dict) and s.get('connected', False)
            ]

            # 当前表中 namespace -> row 映射
            current_ns_to_row = {}
            for row in range(model.rowCount()):
                item = model.item(row, 0)
                ns = item.text() if item is not None else ''
                current_ns_to_row[ns] = row

            # 新状态 ns 列表与集合
            new_ns_list = [s.get('namespace') for s in online_state_list if isinstance(s, dict) and s.get('namespace')]
            new_ns_set = set(new_ns_list)

            # 删除在表中但不在新数据中的行（倒序删除）
            rows_to_remove = [r for ns, r in current_ns_to_row.items() if ns not in new_ns_set]
            for r in sorted(rows_to_remove, reverse=True):
                model.removeRow(r)

            # 重新构建映射
            current_ns_to_row = {}
            for row in range(model.rowCount()):
                item = model.item(row, 0)
                ns = item.text() if item is not None else ''
                current_ns_to_row[ns] = row

            # 更新或追加
            for state in online_state_list:
                if not isinstance(state, dict):
                    continue
                ns = state.get('namespace', 'Unknown')

                # 格式化各列数据
                cells = self._format_table_cells(state, usv_nav_status)

                if ns in current_ns_to_row:
                    row = current_ns_to_row[ns]
                    for col, text in enumerate(cells):
                        existing_item = model.item(row, col)
                        existing_text = existing_item.text() if existing_item is not None else None
                        if existing_text != str(text):
                            model.setItem(row, col, QStandardItem(str(text)))
                else:
                    row = model.rowCount()
                    model.insertRow(row)
                    for col, text in enumerate(cells):
                        model.setItem(row, col, QStandardItem(str(text)))

        except Exception as e:
            _logger.error(f"更新离群表格失败: {e}")
    
    def _format_table_cells(self, state, usv_nav_status):
        """
        格式化表格单元格数据
        
        Args:
            state: USV状态字典
            usv_nav_status: 导航状态字典
            
        Returns:
            list: 格式化后的单元格数据列表
        """
        ns = state.get('namespace', 'Unknown')
        
        # 格式化各列文本，使用安全转换以避免异常
        try:
            bp = state.get('battery_percentage', 0)
            bp_text = f"{float(bp):.1f}%"
            # 如果处于低电压模式，添加警告标志
            if state.get('low_voltage_mode', False):
                bp_text = f"[!] {bp_text}"
        except Exception:
            bp_text = "Unknown"

        voltage = state.get('battery_voltage', 'Unknown')
        try:
            voltage_text = f"{float(voltage):.1f}"
            # 如果处于低电压模式，添加警告标志
            if state.get('low_voltage_mode', False):
                voltage_text = f"[!] {voltage_text}"
        except Exception:
            voltage_text = str(voltage) if voltage is not None else "Unknown"

        try:
            px = float(state.get('position', {}).get('x', 0.0))
            py = float(state.get('position', {}).get('y', 0.0))
            pz = float(state.get('position', {}).get('z', 0.0))
            pos_text = f"({px:.2f}, {py:.2f}, {pz:.2f})"
        except Exception:
            pos_text = "(Unknown, Unknown, Unknown)"

        try:
            vx = float(state.get('velocity', {}).get('linear', {}).get('x', 0.0))
            vy = float(state.get('velocity', {}).get('linear', {}).get('y', 0.0))
            vz = float(state.get('velocity', {}).get('linear', {}).get('z', 0.0))
            vel_text = f"({vx:.2f}, {vy:.2f}, {vz:.2f})"
        except Exception:
            vel_text = "(Unknown, Unknown, Unknown)"

        try:
            # 将偏航角从弧度转换为度数显示
            import math
            yaw_rad = float(state.get('yaw', 0.0))
            yaw_deg = math.degrees(yaw_rad)
            yaw_text = f"{yaw_deg:.1f}°"
        except Exception:
            yaw_text = "Unknown"

        nav_text = usv_nav_status.get(ns, self.NAV_STATUS_IDLE)

        try:
            import math
            temp_raw = state.get('temperature', None)
            temp_celsius = float(temp_raw) if temp_raw is not None else math.nan
            temp_text = "Unknown" if math.isnan(temp_celsius) else f"{temp_celsius:.1f}"
        except Exception:
            temp_text = "Unknown"

        # 获取电池状态文字描述
        power_status = state.get('power_supply_status', 0)
        power_status_text = self.map_power_supply_status(power_status)

        # 使用飞控连接状态 (fc_connected) 而不是网络连接状态 (connected)
        fc_connected = state.get('fc_connected', False)

        cells = [
            ns,
            state.get('mode', 'Unknown'),
            str(fc_connected),  # 飞控连接状态
            str(state.get('armed', 'Unknown')),
            str(state.get('guided', 'Unknown')),  # 引导模式状态
            voltage_text,  # 电压V
            bp_text,  # 电量%
            power_status_text,  # 电池状态
            pos_text,  # 坐标
            vel_text,  # 速度
            yaw_text,  # 偏角
            nav_text,  # 导航状态
            temp_text,  # 温度
        ]
        
        return cells
    
    def get_selected_usv_info(self, is_cluster=True):
        """
        获取选中行的USV信息
        
        Args:
            is_cluster: 是否是集群表格
            
        Returns:
            dict: USV信息字典，包含namespace等信息，如果没有选中则返回None
        """
        table_view = self.cluster_table_view if is_cluster else self.departed_table_view
        if table_view is None:
            return None
        selection_model = table_view.selectionModel()
        if selection_model is None:
            return None

        current_index = selection_model.currentIndex()
        if not current_index.isValid():
            # 回退到首个选中项
            selected_indexes = table_view.selectedIndexes()
            if not selected_indexes:
                return None
            current_index = selected_indexes[0]

        selected_row = current_index.row()
        model = table_view.model()
        if model is None:
            return None
        
        # 获取整行状态数据
        state = {}
        for col, key in enumerate(self.TABLE_HEADERS):
            index = model.index(selected_row, col)
            state[key] = model.data(index) or "Unknown"
        
        usv_id = state.get("编号")
        if not usv_id:
            return None
        
        return {'namespace': usv_id, 'state': state}
