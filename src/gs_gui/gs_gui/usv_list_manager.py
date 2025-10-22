"""
USV列表管理模块
负责管理集群和离群USV列表
"""
from PyQt5.QtWidgets import QMessageBox


class USVListManager:
    """USV列表管理器，负责USV在集群和离群列表间的移动"""
    
    def __init__(self, info_callback):
        """
        初始化USV列表管理器
        
        Args:
            info_callback: 信息输出回调函数
        """
        self.append_info = info_callback
        
        # USV列表
        self.usv_cluster_list = []      # 集群USV列表
        self.usv_departed_list = []     # 离群USV列表
        self.usv_online_list = []       # 在线USV列表
    
    @staticmethod
    def extract_namespaces(usv_list):
        """
        从USV列表中提取命名空间列表
        
        Args:
            usv_list: USV列表，每个元素是一个包含'namespace'键的字典
            
        Returns:
            list: 命名空间列表
        """
        namespaces = []
        for item in usv_list:
            if isinstance(item, dict) and 'namespace' in item:
                namespaces.append(item['namespace'])
        return namespaces
    
    def add_to_cluster(self, usv_info):
        """
        将USV添加到集群列表
        
        Args:
            usv_info: USV信息字典，包含namespace和state
            
        Returns:
            bool: 添加是否成功
        """
        if not usv_info:
            self.append_info("错误：USV信息为空")
            return False
        
        usv_id = usv_info.get('namespace')
        if not usv_id:
            self.append_info("错误：无法获取 USV ID")
            return False
        
        try:
            # 将状态添加到集群列表
            state = usv_info.get('state', {})
            state['namespace'] = usv_id
            self.usv_cluster_list.append(state)
            
            # 从离群列表中移除
            for item in self.usv_departed_list[:]:
                if item.get('namespace') == usv_id:
                    self.usv_departed_list.remove(item)
            
            self.append_info(f"添加设备 {usv_id} 到集群列表")
            return True
        except Exception as e:
            error_msg = f"错误：添加设备失败 - {str(e)}"
            self.append_info(error_msg)
            return False
    
    def remove_from_cluster(self, usv_info):
        """
        将USV从集群列表移到离群列表
        
        Args:
            usv_info: USV信息字典，包含namespace和state
            
        Returns:
            bool: 移除是否成功
        """
        if not usv_info:
            self.append_info("错误：USV信息为空")
            return False
        
        usv_id = usv_info.get('namespace')
        if not usv_id:
            self.append_info("错误：无法获取 USV ID")
            return False
        
        try:
            # 从集群列表移除并添加到离群列表
            for item in self.usv_cluster_list[:]:
                if item.get('namespace') == usv_id:
                    self.usv_cluster_list.remove(item)
                    self.usv_departed_list.append(item)
                    break
            
            self.append_info(f"添加设备 {usv_id} 到离群列表")
            return True
        except Exception as e:
            error_msg = f"错误：移除设备失败 - {str(e)}"
            self.append_info(error_msg)
            return False
    
    def update_departed_list_status(self):
        """更新离群USV列表状态信息"""
        for i, departed in enumerate(self.usv_departed_list):
            departed_id = departed.get('namespace') if isinstance(departed, dict) else departed
            state = next((ns for ns in self.usv_online_list if ns.get('namespace') == departed_id), None)
            if state:
                self.usv_departed_list[i] = state
            else:
                # 如果离群 USV 已下线，保留现有数据
                if not isinstance(departed, dict):
                    self.usv_departed_list[i] = {'namespace': departed_id}
                self.append_info(f"警告：离群设备 {departed_id} 未在在线列表中")
    
    def update_cluster_list(self):
        """更新集群USV列表，移除离群USV并保留手动添加的USV"""
        # 创建 temp_list，移除离群 USV
        temp_list = self.usv_online_list.copy()
        departed_ids = {departed.get('namespace') for departed in self.usv_departed_list 
                       if isinstance(departed, dict)}
        
        # 移除离群USV
        temp_list = [ns for ns in temp_list if ns.get('namespace') not in departed_ids]
        
        # 保留现有 usv_cluster_list 的手动添加 USV
        current_cluster = self.usv_cluster_list.copy()
        current_cluster_ids = {item.get('namespace') for item in current_cluster 
                              if isinstance(item, dict) and item.get('namespace')}
        
        # 更新 usv_cluster_list
        self.usv_cluster_list = []
        for item in current_cluster:
            if isinstance(item, dict) and item.get('namespace'):
                # 仅在仍在线时保留该 USV（下线则从集群列表移除）
                state = next((ns for ns in self.usv_online_list 
                             if ns.get('namespace') == item.get('namespace')), None)
                if state is not None:
                    self.usv_cluster_list.append(state)
        
        for ns in temp_list:
            if ns.get('namespace') not in current_cluster_ids:
                self.usv_cluster_list.append(ns)
    
    def update_online_list(self, state_list):
        """
        更新在线USV列表
        
        Args:
            state_list: 状态列表
        """
        self.usv_online_list = state_list
        self.update_departed_list_status()
        self.update_cluster_list()
