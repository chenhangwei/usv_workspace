"""
系统命令处理模块

负责处理系统级命令：
- 飞控重启
- 机载计算机重启
- 设置 Home Position
- 优雅关闭 USV 节点
"""

import os
import subprocess
import yaml
from std_srvs.srv import Trigger

from .px4_command_interface import Px4CommandInterface


class SystemCommandHandler:
    """系统命令处理器类"""
    
    def __init__(self, node, ros_signal):
        """
        初始化系统命令处理器
        
        Args:
            node: ROS 节点实例
            ros_signal: ROS 信号对象
        """
        self.node = node
        self.ros_signal = ros_signal
        self.logger = node.get_logger()
    
    def reboot_autopilot(self, usv_namespace):
        """
        飞控重启
        
        通过 PX4 VehicleCommand 发送 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 命令重启飞控
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        try:
            # 使用 PX4 命令接口发送重启命令
            px4_cmd = Px4CommandInterface(self.node, usv_namespace)
            
            success = px4_cmd.reboot_autopilot()
            
            if success:
                self.logger.info(f'[OK] 已向 {usv_namespace} 发送飞控重启命令 (VehicleCommand)')
                self._emit_info(f'[OK] 已向 {usv_namespace} 发送飞控重启命令，请等待 10-20 秒')
            else:
                self.logger.error(f'[X] {usv_namespace} 飞控重启命令发送失败')
                self._emit_info(f'[X] {usv_namespace} 飞控重启命令发送失败')
            
        except Exception as e:
            self.logger.error(f'[X] 发送重启命令失败: {e}')
            self._emit_info(f'[X] 发送重启命令失败: {e}')
    
    def reboot_companion(self, usv_namespace):
        """
        机载计算机重启
        
        通过 SSH 直接重启机载计算机（更可靠的方式）
        备选方案：MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN（某些飞控可能不支持）
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        try:
            # 方法 1: 通过 SSH 直接重启（推荐，更可靠）
            workspace_path = os.path.expanduser('~/usv_workspace')
            config_file = os.path.join(
                workspace_path,
                'install/gs_bringup/share/gs_bringup/config/usv_fleet.yaml'
            )
            
            if os.path.exists(config_file):
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                    fleet_config = config.get('usv_fleet', {})
                    
                    if usv_namespace in fleet_config:
                        usv_config = fleet_config[usv_namespace]
                        hostname = usv_config.get('hostname')
                        username = usv_config.get('username')
                        
                        if hostname and username:
                            # 构建 SSH 重启命令
                            ssh_cmd = [
                                'ssh',
                                '-o', 'StrictHostKeyChecking=no',
                                '-o', 'ConnectTimeout=5',
                                f'{username}@{hostname}',
                                'systemctl reboot || sudo reboot'
                            ]
                            
                            # 异步执行 SSH 命令
                            subprocess.Popen(
                                ssh_cmd,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL
                            )
                            
                            self.logger.info(f'[OK] 已向 {usv_namespace} ({hostname}) 发送 SSH 重启命令')
                            self._emit_info(
                                f'[OK] 已向 {usv_namespace} 发送重启命令，系统将在 30-60 秒后重新上线'
                            )
                            return
                        else:
                            self.logger.error(f'[X] {usv_namespace} 配置缺少 hostname 或 username')
                    else:
                        self.logger.error(f'[X] 未找到 {usv_namespace} 的配置')
            else:
                self.logger.error(f'[X] 配置文件不存在: {config_file}')
            
            # 方法 2: 备选 - 通过 MAVLink 命令
            self.logger.warn(f'[!] SSH 重启失败，尝试 MAVLink 命令（可能不被支持）')
            self._reboot_companion_via_mavlink(usv_namespace)
            
        except Exception as e:
            self.logger.error(f'[X] 机载计算机重启失败: {e}')
            self._emit_info(f'[X] 机载计算机重启失败: {e}')
    
    def _reboot_companion_via_mavlink(self, usv_namespace):
        """
        通过 PX4 VehicleCommand 重启机载计算机（备选方案）
        
        注意：某些飞控固件可能不支持此命令
        """
        try:
            px4_cmd = Px4CommandInterface(self.node, usv_namespace)
            
            success = px4_cmd.reboot_companion()
            
            if success:
                self.logger.info(f'[OK] 已发送机载计算机重启命令到 {usv_namespace}')
                self._emit_info(
                    f'[OK] {usv_namespace} 机载计算机重启命令已发送，系统将在 30-60 秒后重新上线'
                )
            else:
                self.logger.warn(f'[!] {usv_namespace} 机载计算机重启命令发送失败')
            
        except Exception as e:
            self.logger.error(f'[X] 机载计算机重启命令失败: {e}')
    
    def set_home_position(self, usv_namespace, use_current, coords=None):
        """
        设置 Home Position
        
        通过 PX4 HomePosition 消息设置 Home Position（局部坐标系）
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
            use_current: 是否使用当前位置
            coords: 坐标字典 {'x': float, 'y': float, 'z': float}（仅当 use_current=False 时使用）
        """
        try:
            px4_cmd = Px4CommandInterface(self.node, usv_namespace)
            
            if use_current:
                success = px4_cmd.set_home_position(use_current=True)
                self.logger.info(f'[OK] 设置 {usv_namespace} Home Position 为当前位置')
            else:
                x = float(coords.get('x', 0.0)) if coords else 0.0
                y = float(coords.get('y', 0.0)) if coords else 0.0
                z = float(coords.get('z', 0.0)) if coords else 0.0
                success = px4_cmd.set_home_position(use_current=False, x=x, y=y, z=z)
                self.logger.info(
                    f'[OK] 设置 {usv_namespace} Home Position 为局部坐标: '
                    f'X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m'
                )
            
            if success:
                if use_current:
                    self._emit_info(f'[OK] 已向 {usv_namespace} 发送设置 Home Position 命令（使用当前位置）')
                else:
                    self._emit_info(
                        f'[OK] 已向 {usv_namespace} 发送设置 Home Position 命令\n'
                        f'    局部坐标: X={coords.get("x", 0):.2f}m, Y={coords.get("y", 0):.2f}m, Z={coords.get("z", 0):.2f}m'
                    )
            else:
                self.logger.error(f'[X] {usv_namespace} 设置 Home Position 命令发送失败')
                self._emit_info(f'[X] {usv_namespace} 设置 Home Position 命令发送失败')
            
        except Exception as e:
            self.logger.error(f'[X] 发送设置 Home Position 命令失败: {e}')
            self._emit_info(f'[X] 发送设置 Home Position 命令失败: {e}')
    
    def shutdown_usv(self, usv_namespace):
        """
        优雅关闭 USV 节点（通过 ROS 服务）
        
        调用 USV 端的 shutdown_service 来优雅关闭所有节点
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        try:
            # 创建服务客户端
            service_name = f'/{usv_namespace}/shutdown_all'
            client = self.node.create_client(Trigger, service_name)
            
            # 等待服务可用（3秒超时）
            if not client.wait_for_service(timeout_sec=3.0):
                self.logger.error(f'[X] 关闭服务不可用: {service_name}')
                self._emit_info(f'[X] {usv_namespace} 关闭失败：服务不可用（USV可能已离线）')
                return
            
            # 构建请求
            request = Trigger.Request()
            
            self.logger.info(f'[->] 正在关闭 {usv_namespace} 的所有节点...')
            self._emit_info(f'[->] 正在关闭 {usv_namespace} 的所有节点...')
            
            # 异步调用服务
            future = client.call_async(request)
            future.add_done_callback(
                lambda f: self._handle_shutdown_response(f, usv_namespace)
            )
            
        except Exception as e:
            self.logger.error(f'[X] 发送关闭命令失败: {e}')
            self._emit_info(f'[X] {usv_namespace} 发送关闭命令失败: {e}')
    
    def _handle_shutdown_response(self, future, usv_namespace):
        """处理 USV 关闭服务响应"""
        try:
            response = future.result()
            if response.success:
                msg = f'[OK] {usv_namespace} 节点关闭成功: {response.message}'
                self.logger.info(msg)
                self._emit_info(msg)
            else:
                msg = f'[!] {usv_namespace} 节点关闭失败: {response.message}'
                self.logger.warn(msg)
                self._emit_warning(msg)
        except Exception as e:
            self.logger.error(f'[X] 处理关闭命令响应失败: {e}')
            self._emit_info(f'[X] {usv_namespace} 关闭命令响应处理失败: {e}')
    
    def _emit_info(self, message):
        """发送信息到 GUI"""
        try:
            self.ros_signal.node_info.emit(message)
        except Exception:
            pass
    
    def _emit_warning(self, message):
        """发送警告到 GUI"""
        try:
            self.ros_signal.node_warning.emit(message)
        except Exception:
            pass
