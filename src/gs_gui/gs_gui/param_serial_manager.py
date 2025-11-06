"""
基于串口的 MAVLink 参数管理器

直接通过串口与飞控通信，不依赖 MAVROS。
使用 pymavlink 库实现 MAVLink 协议通信。
"""

import serial
import threading
import time
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum
import queue

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:
    PYMAVLINK_AVAILABLE = False
    print("⚠️  pymavlink 未安装，参数功能需要安装：pip3 install pymavlink")

from .param_metadata import get_param_metadata


class ParamType(Enum):
    """参数类型枚举"""
    INTEGER = "integer"
    REAL = "real"
    UNKNOWN = "unknown"


@dataclass
class ParamInfo:
    """飞控参数信息"""
    name: str
    value: float
    original_value: float
    param_type: ParamType
    description: str = ""
    unit: str = ""
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    increment: Optional[float] = None
    
    @property
    def group(self) -> str:
        """从参数名称提取分组"""
        parts = self.name.split('_')
        return parts[0] if parts else "其他"
    
    @property
    def is_modified(self) -> bool:
        """是否已修改"""
        return abs(self.value - self.original_value) > 1e-6
    
    def reset(self):
        """重置为原始值"""
        self.value = self.original_value


class ParamSerialManager:
    """
    基于串口的参数管理器
    
    直接通过串口与飞控通信，使用 MAVLink 协议。
    """
    
    def __init__(self):
        self.mavlink_conn = None
        self.connected = False
        self.params: Dict[str, ParamInfo] = {}
        
        # 线程安全
        self._lock = threading.Lock()
        self._heartbeat_thread = None
        self._heartbeat_running = False
        
        # 回调函数
        self._progress_callback: Optional[Callable[[int, int, str], None]] = None
        self._param_loaded_callback: Optional[Callable[[], None]] = None
    
    def connect(self, port: str, baudrate: int = 115200, 
                target_system: int = 1, target_component: int = 1) -> bool:
        """
        连接飞控
        
        Args:
            port: 串口设备（如 /dev/ttyACM0）
            baudrate: 波特率（默认 115200）
            target_system: 目标系统 ID（默认 1）
            target_component: 目标组件 ID（默认 1）
            
        Returns:
            是否连接成功
        """
        if not PYMAVLINK_AVAILABLE:
            raise RuntimeError("pymavlink 未安装，请运行: pip3 install pymavlink")
        
        try:
            # 创建 MAVLink 连接（pymavlink 格式：设备路径，波特率通过 baud 参数）
            connection_str = f'{port}'
            self.mavlink_conn = mavutil.mavlink_connection(
                connection_str,
                baud=baudrate,
                source_system=255,  # 地面站 ID
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
            )
            
            # 等待心跳包（超时 5 秒）
            print(f"⏳ 等待飞控心跳包...")
            self.mavlink_conn.wait_heartbeat(timeout=5)
            
            # 设置目标系统
            self.mavlink_conn.target_system = target_system
            self.mavlink_conn.target_component = target_component
            
            self.connected = True
            print(f"✅ 已连接到飞控 (System {target_system}.{target_component})")
            
            # 启动心跳线程
            self._start_heartbeat_thread()
            
            return True
            
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开连接"""
        self._stop_heartbeat_thread()
        
        if self.mavlink_conn:
            self.mavlink_conn.close()
            self.mavlink_conn = None
        
        self.connected = False
        print("± 已断开飞控连接")
    
    def _start_heartbeat_thread(self):
        """启动心跳线程（保持连接活跃）"""
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            return
        
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()
    
    def _stop_heartbeat_thread(self):
        """停止心跳线程"""
        self._heartbeat_running = False
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=2)
    
    def _heartbeat_loop(self):
        """心跳循环（每秒发送一次）"""
        while self._heartbeat_running and self.mavlink_conn:
            try:
                self.mavlink_conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                time.sleep(1)
            except Exception as e:
                print(f"⚠️  心跳发送失败: {e}")
                break
    
    def fetch_all_params(self, progress_callback: Optional[Callable[[int, int, str], None]] = None) -> Dict[str, ParamInfo]:
        """
        获取所有参数
        
        Args:
            progress_callback: 进度回调 (current, total, param_name)
            
        Returns:
            参数字典 {param_name: ParamInfo}
        """
        if not self.connected or not self.mavlink_conn:
            raise RuntimeError("未连接到飞控")
        
        self._progress_callback = progress_callback
        params_dict = {}
        
        try:
            print("📥 请求参数列表...")
            
            # 发送 PARAM_REQUEST_LIST
            self.mavlink_conn.mav.param_request_list_send(
                self.mavlink_conn.target_system,
                self.mavlink_conn.target_component
            )
            
            # 接收所有参数
            param_count = None
            received_params = set()
            timeout = time.time() + 30  # 30 秒超时
            
            while time.time() < timeout:
                msg = self.mavlink_conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                
                if msg:
                    # 处理 param_id 可能是 bytes 或 str
                    if isinstance(msg.param_id, bytes):
                        param_name = msg.param_id.decode('utf-8').rstrip('\x00')
                    else:
                        param_name = str(msg.param_id).rstrip('\x00')
                    
                    param_value = msg.param_value
                    param_index = msg.param_index
                    param_count = msg.param_count
                    
                    # 避免重复
                    if param_name in received_params:
                        continue
                    
                    received_params.add(param_name)
                    
                    # 确定参数类型
                    param_type = ParamType.REAL if msg.param_type in [
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                    ] else ParamType.INTEGER
                    
                    # 从元数据获取描述
                    metadata = get_param_metadata(param_name)
                    
                    # 创建参数对象（metadata 可能为 None）
                    params_dict[param_name] = ParamInfo(
                        name=param_name,
                        value=param_value,
                        original_value=param_value,
                        param_type=param_type,
                        description=metadata.description if metadata else '',
                        unit=metadata.unit if metadata else '',
                        min_value=metadata.min_value if metadata else None,
                        max_value=metadata.max_value if metadata else None,
                        increment=metadata.increment if metadata else None
                    )
                    
                    # 进度回调
                    if progress_callback and param_count:
                        progress_callback(len(received_params), param_count, param_name)
                    
                    # 检查是否全部接收
                    if param_count and len(received_params) >= param_count:
                        break
            
            # 检查是否接收完整
            if param_count is None:
                raise RuntimeError("未收到参数列表")
            
            if len(received_params) < param_count:
                missing = param_count - len(received_params)
                print(f"⚠️  部分参数丢失 ({missing}/{param_count})")
            
            print(f"✅ 成功接收 {len(params_dict)}/{param_count} 个参数")
            
            with self._lock:
                self.params = params_dict
            
            return params_dict
            
        except Exception as e:
            print(f"❌ 获取参数失败: {e}")
            raise
        finally:
            self._progress_callback = None
    
    def set_param(self, param_name: str, param_value: float) -> bool:
        """
        设置单个参数
        
        Args:
            param_name: 参数名称
            param_value: 参数值
            
        Returns:
            是否设置成功
        """
        if not self.connected or not self.mavlink_conn:
            raise RuntimeError("未连接到飞控")
        
        try:
            # 确定参数类型
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            if param_name in self.params:
                if self.params[param_name].param_type == ParamType.INTEGER:
                    param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
                    param_value = int(param_value)
            
            # 发送 PARAM_SET
            self.mavlink_conn.mav.param_set_send(
                self.mavlink_conn.target_system,
                self.mavlink_conn.target_component,
                param_name.encode('utf-8'),
                param_value,
                param_type
            )
            
            # 等待确认（PARAM_VALUE 回复）
            timeout = time.time() + 3
            while time.time() < timeout:
                msg = self.mavlink_conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if msg:
                    # 处理 param_id 可能是 bytes 或 str
                    if isinstance(msg.param_id, bytes):
                        received_name = msg.param_id.decode('utf-8').rstrip('\x00')
                    else:
                        received_name = str(msg.param_id).rstrip('\x00')
                    
                    if received_name == param_name:
                        # 验证值是否正确
                        if abs(msg.param_value - param_value) < 1e-6:
                            print(f"✅ 参数 {param_name} 已设置为 {param_value}")
                            
                            # 更新本地缓存
                            if param_name in self.params:
                                self.params[param_name].value = param_value
                                self.params[param_name].original_value = param_value
                            
                            return True
                        else:
                            print(f"⚠️  参数 {param_name} 设置后值不匹配: {msg.param_value} != {param_value}")
                            return False
            
            print(f"❌ 参数 {param_name} 设置超时")
            return False
            
        except Exception as e:
            print(f"❌ 设置参数失败: {e}")
            return False
    
    def get_param(self, param_name: str) -> Optional[float]:
        """
        获取单个参数的当前值
        
        Args:
            param_name: 参数名称
            
        Returns:
            参数值，失败返回 None
        """
        if not self.connected or not self.mavlink_conn:
            raise RuntimeError("未连接到飞控")
        
        try:
            # 发送 PARAM_REQUEST_READ
            self.mavlink_conn.mav.param_request_read_send(
                self.mavlink_conn.target_system,
                self.mavlink_conn.target_component,
                param_name.encode('utf-8'),
                -1  # -1 表示通过名称查询
            )
            
            # 等待响应
            timeout = time.time() + 3
            while time.time() < timeout:
                msg = self.mavlink_conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if msg:
                    received_name = msg.param_id.decode('utf-8').rstrip('\x00')
                    if received_name == param_name:
                        return msg.param_value
            
            print(f"⚠️  获取参数 {param_name} 超时")
            return None
            
        except Exception as e:
            print(f"❌ 获取参数失败: {e}")
            return None
    
    def get_all_params(self) -> Dict[str, ParamInfo]:
        """获取所有已缓存的参数"""
        with self._lock:
            return self.params.copy()
    
    def save_modified_params(self, progress_callback: Optional[Callable[[int, int, str], None]] = None) -> int:
        """
        保存所有修改的参数到飞控
        
        Args:
            progress_callback: 进度回调 (current, total, param_name)
            
        Returns:
            成功保存的参数数量
        """
        modified_params = {name: info for name, info in self.params.items() if info.is_modified}
        
        if not modified_params:
            return 0
        
        success_count = 0
        total = len(modified_params)
        
        for i, (name, info) in enumerate(modified_params.items(), 1):
            if progress_callback:
                progress_callback(i, total, name)
            
            if self.set_param(name, info.value):
                success_count += 1
            else:
                print(f"⚠️  保存参数 {name} 失败")
        
        return success_count
    
    def reboot_autopilot(self) -> bool:
        """
        重启飞控
        
        Returns:
            是否发送重启命令成功
        """
        if not self.connected or not self.mavlink_conn:
            raise RuntimeError("未连接到飞控")
        
        try:
            # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246)
            # param1=1: 重启自驾仪
            self.mavlink_conn.mav.command_long_send(
                self.mavlink_conn.target_system,
                self.mavlink_conn.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0,  # confirmation
                1,  # param1: 重启自驾仪
                0, 0, 0, 0, 0, 0
            )
            
            print("🔄 已发送重启命令")
            return True
            
        except Exception as e:
            print(f"❌ 发送重启命令失败: {e}")
            return False
