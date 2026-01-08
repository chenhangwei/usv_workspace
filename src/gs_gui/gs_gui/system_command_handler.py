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
                            # 获取可选的密码字段
                            password = usv_config.get('password')
                            
                            # 在独立线程中执行，以便捕获输出日志而不阻塞 GUI
                            import threading
                            def run_reboot_ssh():
                                try:
                                    # 构建 SSH 重启命令
                                    if password:
                                        # 如果配置了密码，使用 sudo -S 并传入密码
                                        # 注意: 这种方式会在进程列表中暴露密码，但在内网开发环境中通常可接受
                                        remote_cmd = f"echo '{password}' | sudo -S reboot"
                                        log_cmd = "echo '******' | sudo -S reboot"
                                    else:
                                        # 尝试顺序：
                                        # 1. sudo -n reboot (非交互式 sudo，如果您配置了免密)
                                        # 2. systemctl reboot (现代 Linux)
                                        # 3. sudo reboot (可能会卡在密码输入，但在某些系统可行)
                                        remote_cmd = "sudo -n reboot || systemctl reboot || sudo reboot"
                                        log_cmd = remote_cmd
                                    
                                    ssh_cmd = [
                                        'ssh',
                                        '-o', 'StrictHostKeyChecking=no',
                                        '-o', 'ConnectTimeout=5',
                                        f'{username}@{hostname}',
                                        remote_cmd
                                    ]
                                    
                                    self.logger.info(f'[->] 正在连接 {hostname} 执行重启: {log_cmd}')
                                    
                                    # 使用 run 并捕获输出
                                    result = subprocess.run(
                                        ssh_cmd,
                                        capture_output=True,
                                        text=True,
                                        timeout=10
                                    )
                                    
                                    if result.returncode == 0:
                                        msg = f'[OK] {usv_namespace} ({hostname}) 重启命令发送成功'
                                        self.logger.info(msg)
                                        self._emit_info(msg)
                                    else:
                                        err_msg = result.stderr.strip() or result.stdout.strip() or "未知错误"
                                        # 特殊处理 sudo 需要密码的情况
                                        if "password" in err_msg.lower() or "sudo: a terminal is required" in err_msg:
                                            if not password:
                                                err_msg += " (提示: 请在 usv_fleet.yaml 中添加 'password' 字段，或在 USV 端配置 sudo 免密)"
                                        
                                        self.logger.error(f'[X] {usv_namespace} 重启失败 (Code {result.returncode}): {err_msg}')
                                        self._emit_info(f'[X] {usv_namespace} 重启失败: {err_msg}')
                                        
                                except subprocess.TimeoutExpired:
                                    self.logger.error(f'[X] 连接 {hostname} 超时')
                                    self._emit_info(f'[X] {usv_namespace} 重启失败: SSH 连接超时')
                                except Exception as e:
                                    self.logger.error(f'[X] SSH 执行异常: {e}')
                                    self._emit_info(f'[X] {usv_namespace} 重启异常: {e}')

                            # 启动线程
                            threading.Thread(target=run_reboot_ssh, daemon=True).start()
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
        强制停止 USV 节点（SSH 方式）
        
        不再依赖 ROS 服务，而是通过 SSH 发送 pkill 命令直接结束 launch 进程。
        这在节点卡死或服务不可用时更加可靠。
        
        Args:
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        try:
            self.logger.info(f'[->] 正在通过 SSH 强制停止 {usv_namespace}...')
            
            # 1. 获取目标主机信息 (复用 reboot_companion 的逻辑)
            workspace_path = os.path.expanduser('~/usv_workspace')
            config_file = os.path.join(
                workspace_path,
                'install/gs_bringup/share/gs_bringup/config/usv_fleet.yaml'
            )
            
            hostname = None
            username = None
            
            if os.path.exists(config_file):
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                    fleet_config = config.get('usv_fleet', {})
                    if usv_namespace in fleet_config:
                        usv_config = fleet_config[usv_namespace]
                        hostname = usv_config.get('hostname')
                        username = usv_config.get('username')
            
            if not hostname or not username:
                self.logger.error(f'[X] 无法停止 {usv_namespace}: 未在配置文件中找到该节点')
                self._emit_info(f'[X] {usv_namespace} 停止失败：未找到配置')
                return

            # 2. 构建 SSH 停止命令
            # 针对性杀掉 usv_launch.py 进程
            # 补充：也可以视情况加入 pkill -f ros2 (但这可能会误杀其他东西，目前杀掉launch文件通常足够)
            remote_cmd = "pkill -f usv_launch.py"
            
            ssh_cmd = [
                'ssh',
                '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=3',
                f'{username}@{hostname}',
                remote_cmd
            ]
            
            # 3. 异步执行
            subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            msg = f'[OK] 已向 {usv_namespace} ({hostname}) 发送 SSH 停止命令'
            self.logger.info(msg)
            self._emit_info(msg)
            
        except Exception as e:
            self.logger.error(f'[X] SSH 停止命令执行失败: {e}')
            self._emit_info(f'[X] {usv_namespace} 停止失败: {e}')
    
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
