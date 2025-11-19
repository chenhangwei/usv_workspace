"""
串口资源管理器

提供串口资源的生命周期管理、自动清理和错误处理,
解决项目中串口资源泄漏的问题。
"""

import serial
from contextlib import contextmanager
from typing import Optional
import logging


class SerialResourceManager:
    """串口资源管理器,确保资源正确释放"""
    
    def __init__(self, logger=None):
        """
        初始化串口资源管理器
        
        Args:
            logger: 日志记录器(ROS logger 或 Python logger)
        """
        self.logger = logger or logging.getLogger(__name__)
        self.serial_port: Optional[serial.Serial] = None
        self._is_open = False
    
    def open(
        self,
        port: str,
        baudrate: int,
        timeout: float = 1.0,
        **kwargs
    ) -> bool:
        """
        打开串口
        
        Args:
            port: 串口设备路径 (例如 '/dev/ttyUSB0')
            baudrate: 波特率
            timeout: 读取超时时间(秒)
            **kwargs: serial.Serial 的其他参数
            
        Returns:
            是否成功打开
            
        Example:
            manager = SerialResourceManager(self.get_logger())
            if manager.open('/dev/ttyUSB0', 115200):
                data = manager.read(10)
                manager.close()
        """
        if self._is_open:
            self.logger.warn(f"串口 {port} 已打开,先关闭后重新打开")
            self.close()
        
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                **kwargs
            )
            self._is_open = True
            self.logger.info(f"✓ 串口打开成功: {port} @ {baudrate} baud")
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"✗ 打开串口失败 {port}: {e}")
            self.serial_port = None
            self._is_open = False
            return False
        except Exception as e:
            self.logger.error(f"✗ 打开串口时发生未知错误: {e}")
            self.serial_port = None
            self._is_open = False
            return False
    
    def close(self) -> bool:
        """
        关闭串口
        
        Returns:
            是否成功关闭
        """
        if not self._is_open or self.serial_port is None:
            return True
        
        try:
            if self.serial_port.is_open:
                self.serial_port.close()
            self._is_open = False
            self.logger.info("✓ 串口已关闭")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 关闭串口失败: {e}")
            return False
    
    def read(self, size: int = 1) -> Optional[bytes]:
        """安全读取数据"""
        if not self._is_open or self.serial_port is None:
            self.logger.warn("串口未打开,无法读取")
            return None
        
        try:
            return self.serial_port.read(size)
        except serial.SerialException as e:
            self.logger.error(f"读取串口数据失败: {e}")
            return None
    
    def write(self, data: bytes) -> bool:
        """安全写入数据"""
        if not self._is_open or self.serial_port is None:
            self.logger.warn("串口未打开,无法写入")
            return False
        
        try:
            self.serial_port.write(data)
            return True
        except serial.SerialException as e:
            self.logger.error(f"写入串口数据失败: {e}")
            return False
    
    def readline(self) -> Optional[bytes]:
        """安全读取一行"""
        if not self._is_open or self.serial_port is None:
            self.logger.warn("串口未打开,无法读取")
            return None
        
        try:
            return self.serial_port.readline()
        except serial.SerialException as e:
            self.logger.error(f"读取串口行失败: {e}")
            return None
    
    @property
    def is_open(self) -> bool:
        """检查串口是否打开"""
        return self._is_open and self.serial_port is not None and self.serial_port.is_open
    
    @contextmanager
    def managed_serial(self, port: str, baudrate: int, **kwargs):
        """
        上下文管理器,自动管理串口生命周期
        
        Example:
            manager = SerialResourceManager(self.get_logger())
            with manager.managed_serial('/dev/ttyUSB0', 115200) as ser:
                if ser:
                    data = manager.read(10)
            # 自动关闭串口
        """
        success = self.open(port, baudrate, **kwargs)
        try:
            yield self.serial_port if success else None
        finally:
            self.close()
    
    def __del__(self):
        """析构时确保串口关闭"""
        try:
            if self._is_open:
                self.close()
        except Exception:
            pass  # 析构时忽略异常


# 使用示例
"""
from common_utils import SerialResourceManager

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        
        # 方法1: 手动管理
        self.serial_manager = SerialResourceManager(self.get_logger())
        
        if self.serial_manager.open('/dev/ttyUSB0', 115200):
            self.timer = self.create_timer(0.1, self.read_data)
        else:
            self.get_logger().error('初始化串口失败,节点退出')
            raise RuntimeError('Serial port initialization failed')
    
    def read_data(self):
        data = self.serial_manager.readline()
        if data:
            # 处理数据
            pass
    
    def destroy_node(self):
        self.serial_manager.close()
        super().destroy_node()
    
    # 方法2: 使用上下文管理器(适合临时操作)
    def configure_sensor(self):
        with self.serial_manager.managed_serial('/dev/ttyUSB0', 115200) as ser:
            if ser:
                self.serial_manager.write(b'CONFIG\\r\\n')
                response = self.serial_manager.readline()
        # 自动关闭
"""
