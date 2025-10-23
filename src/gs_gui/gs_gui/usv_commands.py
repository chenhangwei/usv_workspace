"""
USV命令模块
负责处理所有USV控制命令的发送
"""
from PyQt5.QtWidgets import QColorDialog


class USVCommandHandler:
    """USV命令处理器，负责各种USV控制命令"""
    
    def __init__(self, ros_signal, info_callback):
        """
        初始化命令处理器
        
        Args:
            ros_signal: ROS信号对象
            info_callback: 信息输出回调函数
        """
        self.ros_signal = ros_signal
        self.append_info = info_callback
        self._color_dialog = None
    
    def _extract_namespaces(self, usv_list):
        """
        从USV列表中提取命名空间列表
        
        Args:
            usv_list: USV列表
            
        Returns:
            list: 命名空间列表
        """
        namespaces = []
        for item in usv_list:
            if isinstance(item, dict) and 'namespace' in item:
                namespaces.append(item['namespace'])
        return namespaces
    
    # ============== 集群命令 ==============
    def set_cluster_arming(self, usv_cluster_list):
        """发送集群解锁命令"""
        try:
            namespace_list = self._extract_namespaces(usv_cluster_list)
            self.ros_signal.arm_command.emit(namespace_list)
            self.append_info(f"集群解锁命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送集群解锁命令失败: {e}")
    
    def cluster_disarming(self, usv_cluster_list):
        """发送集群加锁命令"""
        try:
            namespace_list = self._extract_namespaces(usv_cluster_list)
            self.ros_signal.disarm_command.emit(namespace_list)
            self.append_info(f"集群加锁命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送集群加锁命令失败: {e}")
    
    def set_cluster_guided(self, usv_cluster_list):
        """发送集群设置guided模式命令"""
        try:
            namespace_list = self._extract_namespaces(usv_cluster_list)
            self.ros_signal.guided_command.emit(namespace_list)
            self.append_info(f"集群设置guided模式命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送集群guided模式命令失败: {e}")
    
    def set_cluster_manual(self, usv_cluster_list):
        """发送集群设置manual模式命令"""
        try:
            namespace_list = self._extract_namespaces(usv_cluster_list)
            self.ros_signal.manual_command.emit(namespace_list)
            self.append_info(f"集群设置manual模式命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送集群manual模式命令失败: {e}")
    
    # ============== 离群命令 ==============
    def departed_arming(self, usv_departed_list):
        """发送离群解锁命令"""
        try:
            namespace_list = self._extract_namespaces(usv_departed_list)
            self.ros_signal.arm_command.emit(namespace_list)
            self.append_info(f"离群解锁命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群解锁命令失败: {e}")
    
    def departed_disarming(self, usv_departed_list):
        """发送离群加锁命令"""
        try:
            namespace_list = self._extract_namespaces(usv_departed_list)
            self.ros_signal.disarm_command.emit(namespace_list)
            self.append_info(f"离群加锁命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群加锁命令失败: {e}")
    
    def set_departed_guided(self, usv_departed_list):
        """发送离群设置guided模式命令"""
        try:
            namespace_list = self._extract_namespaces(usv_departed_list)
            self.ros_signal.guided_command.emit(namespace_list)
            self.append_info(f"离群设置guided模式命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群guided模式命令失败: {e}")
    
    def set_departed_manual(self, usv_departed_list):
        """发送离群设置manual模式命令"""
        try:
            namespace_list = self._extract_namespaces(usv_departed_list)
            self.ros_signal.manual_command.emit(namespace_list)
            self.append_info(f"离群设置manual模式命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群manual模式命令失败: {e}")
    
    def set_departed_arco(self, usv_departed_list):
        """发送离群设置ARCO模式命令"""
        try:
            namespace_list = self._extract_namespaces(usv_departed_list)
            self.ros_signal.arco_command.emit(namespace_list)
            self.append_info(f"离群设置ARCO模式命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群ARCO模式命令失败: {e}")
    
    def set_departed_steering(self, usv_departed_list):
        """发送离群设置Steering模式命令"""
        try:
            namespace_list = self._extract_namespaces(usv_departed_list)
            self.ros_signal.steering_command.emit(namespace_list)
            self.append_info(f"离群设置Steering模式命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送离群Steering模式命令失败: {e}")
    
    # ============== 声音命令 ==============
    def sound_start(self):
        """发送声音开始命令"""
        self.ros_signal.str_command.emit('sound_start')
        self.append_info("发送命令: sound_start")
    
    def sound_stop(self):
        """发送声音停止命令"""
        self.ros_signal.str_command.emit('sound_stop')
        self.append_info("发送命令: sound_stop")
    
    # ============== 颈部命令 ==============
    def neck_swinging(self):
        """发送颈部摆动命令"""
        self.ros_signal.str_command.emit('neck_swinging')
        self.append_info("发送命令: neck_swinging")
    
    def neck_stop(self):
        """发送颈部停止命令"""
        self.ros_signal.str_command.emit('neck_stop')
        self.append_info("发送命令: neck_stop")
    
    # ============== LED命令 ==============
    def led_color_switching(self):
        """LED颜色切换命令"""
        self.ros_signal.str_command.emit('color_switching')
        self.append_info("发送命令: color_switching")
    
    def led_random_color(self):
        """LED随机颜色命令"""
        self.ros_signal.str_command.emit('random_color_change')
        self.append_info("发送命令: random_color_change")
    
    def led_select_color(self, parent_widget=None):
        """
        LED选择颜色命令
        
        Args:
            parent_widget: 父窗口部件
        """
        try:
            # 使用非阻塞的 QColorDialog 实例
            dlg = QColorDialog(parent_widget)
            self._color_dialog = dlg
            dlg.setOption(QColorDialog.ShowAlphaChannel, False)
            dlg.colorSelected.connect(self._color_selected_handler)
            dlg.open()
        except Exception:
            # 回退到同步调用
            color = QColorDialog.getColor()
            if not color.isValid():
                self.append_info("颜色选择无效")
                return
            r = color.red()
            g = color.green()
            b = color.blue()
            color_str = f"color_select|{r},{g},{b}"
            self.ros_signal.str_command.emit(color_str)
            self.append_info(f"发送led3命令: {color_str}")
    
    def _color_selected_handler(self, color):
        """颜色对话框回调"""
        try:
            if not color.isValid():
                self.append_info("颜色选择无效")
                return
            r = color.red()
            g = color.green()
            b = color.blue()
            color_str = f"color_select|{r},{g},{b}"
            self.ros_signal.str_command.emit(color_str)
            self.append_info(f"发送led3命令: {color_str}")
        except Exception as e:
            self.append_info(f"处理颜色选择时出错: {e}")
    
    def led_off(self):
        """停止LED灯光命令"""
        self.ros_signal.str_command.emit('led_off')
        self.append_info("发送命令: led_off")
