"""
飞控参数管理器模块

负责与 MAVROS 参数服务通信，实现参数的读取、写入和管理功能。
类似于 QGroundControl 的参数管理功能。
"""

from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Callable
from enum import Enum
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import ParamGet, ParamSet, ParamPull, ParamPush
from mavros_msgs.msg import Param
from rcl_interfaces.srv import GetParameters
import threading
import time
import json
import os
from pathlib import Path
from datetime import datetime, timedelta
from .param_metadata import get_param_metadata, load_all_metadata


class ParamType(Enum):
    """参数类型枚举"""
    INTEGER = "integer"
    REAL = "real"
    UNKNOWN = "unknown"


@dataclass
class ParamInfo:
    """飞控参数信息"""
    name: str                    # 参数名称（如 ARMING_CHECK）
    value: float                 # 当前值
    original_value: float        # 原始值（用于检测修改）
    param_type: ParamType        # 参数类型
    description: str = ""        # 参数描述
    unit: str = ""               # 单位
    min_value: Optional[float] = None    # 最小值
    max_value: Optional[float] = None    # 最大值
    increment: Optional[float] = None    # 步进值
    
    @property
    def group(self) -> str:
        """从参数名称提取分组"""
        # 例如: ARMING_CHECK → ARMING, GPS_TYPE → GPS
        parts = self.name.split('_')
        return parts[0] if parts else "其他"
    
    @property
    def is_modified(self) -> bool:
        """是否已修改"""
        return abs(self.value - self.original_value) > 1e-6
    
    def reset(self):
        """重置为原始值"""
        self.value = self.original_value
    
    def to_dict(self) -> dict:
        """转换为字典（用于 JSON 序列化）"""
        return {
            'name': self.name,
            'value': self.value,
            'original_value': self.original_value,
            'param_type': self.param_type.value,
            'description': self.description,
            'unit': self.unit,
            'min_value': self.min_value,
            'max_value': self.max_value,
            'increment': self.increment
        }
    
    @staticmethod
    def from_dict(data: dict) -> 'ParamInfo':
        """从字典创建（用于 JSON 反序列化）"""
        return ParamInfo(
            name=data['name'],
            value=data['value'],
            original_value=data['original_value'],
            param_type=ParamType(data['param_type']),
            description=data.get('description', ''),
            unit=data.get('unit', ''),
            min_value=data.get('min_value'),
            max_value=data.get('max_value'),
            increment=data.get('increment')
        )


class ParamManager:
    """
    飞控参数管理器
    
    负责与 MAVROS 参数服务通信，管理飞控参数的读取、写入和缓存。
    
    注意：需要在 usv_launch.py 中启用 'param' 插件才能使用。
    """
    
    def __init__(self, node: Node, usv_namespace: str):
        """
        初始化参数管理器
        
        Args:
            node: ROS 2 节点实例
            usv_namespace: USV 命名空间（如 'usv_01'）
        """
        self.node = node
        self.usv_namespace = usv_namespace
        self.logger = node.get_logger()
        
        # 参数缓存：{参数名: ParamInfo}
        self._params: Dict[str, ParamInfo] = {}
        
        # 参数加载状态
        self._is_loading = False
        self._load_start_time = 0.0
        self._expected_param_count = 0  # 预期参数总数（从 ParamPull 响应获取）
        self._received_param_count = 0  # 已接收参数数量
        
        # 缓存配置
        self._cache_dir = Path.home() / '.cache' / 'usv_params'
        self._cache_file = self._cache_dir / f'{usv_namespace}.json'
        self._cache_expiry_hours = 24  # 缓存过期时间（小时）
        
        # 创建缓存目录
        self._cache_dir.mkdir(parents=True, exist_ok=True)
        
        # 加载参数元数据
        load_all_metadata()
        self.logger.info("参数元数据已加载")
        
        # 创建服务客户端
        self._create_service_clients()
        
        # 创建 topic 订阅（用于接收参数）
        self._create_param_subscriber()
        
        # 回调函数
        self._on_progress: Optional[Callable[[int, int], None]] = None
        self._on_complete: Optional[Callable[[bool, str], None]] = None
    
    def _create_service_clients(self):
        """创建 MAVROS 参数服务客户端"""
        base_namespace = f'/{self.usv_namespace}/param'
        
        # ParamPull: 从飞控拉取所有参数到 ROS 参数服务器
        self.pull_client = self.node.create_client(
            ParamPull, 
            f'{base_namespace}/pull'
        )
        
        # ParamPush: 推送 ROS 参数到飞控
        self.push_client = self.node.create_client(
            ParamPush,
            f'{base_namespace}/push'
        )
        
        # ParamGet: 获取单个参数
        self.get_client = self.node.create_client(
            ParamGet,
            f'{base_namespace}/get'
        )
        
        # ParamSet: 设置单个参数
        self.set_client = self.node.create_client(
            ParamSet,
            f'{base_namespace}/set'
        )
        
        # ROS 2 参数服务（用于从 ROS 参数服务器读取）
        self.ros_param_client = self.node.create_client(
            GetParameters,
            f'/{self.usv_namespace}/get_parameters'
        )
    
    def _create_param_subscriber(self):
        """创建参数值 topic 订阅器"""
        param_topic = f'/{self.usv_namespace}/mavros/param/param_value'
        
        self.param_sub = self.node.create_subscription(
            Param,
            param_topic,
            self._param_value_callback,
            10  # QoS depth
        )
        
        self.logger.info(f"已订阅参数 topic: {param_topic}")
    
    def set_callbacks(self, 
                     on_progress: Optional[Callable[[int, int], None]] = None,
                     on_complete: Optional[Callable[[bool, str], None]] = None):
        """
        设置回调函数
        
        Args:
            on_progress: 进度回调 (current, total)
            on_complete: 完成回调 (success, message)
        """
        self._on_progress = on_progress
        self._on_complete = on_complete
    
    def _param_value_callback(self, msg: Param):
        """
        参数值 topic 回调
        
        当 MAVROS 从飞控接收到参数时，会发布到这个 topic。
        
        Args:
            msg: Param 消息
        """
        if not self._is_loading:
            # 不在加载状态，忽略
            return
        
        param_name = msg.header.frame_id  # 参数名在 frame_id 字段
        
        # 判断参数类型（MAVROS 会设置 integer 或 real 字段）
        if msg.value.integer != 0:
            param_type = ParamType.INTEGER
            param_value = float(msg.value.integer)
        else:
            param_type = ParamType.REAL
            param_value = msg.value.real
        
        # 尝试获取参数元数据
        metadata = get_param_metadata(param_name)
        
        # 创建 ParamInfo 对象（合并元数据）
        param_info = ParamInfo(
            name=param_name,
            value=param_value,
            original_value=param_value,
            param_type=param_type,
            description=metadata.description if metadata else "",
            unit=metadata.unit if metadata else "",
            min_value=metadata.min_value if metadata else None,
            max_value=metadata.max_value if metadata else None,
            increment=metadata.increment if metadata else None
        )
        
        # 存入缓存
        self._params[param_name] = param_info
        self._received_param_count += 1
        
        # 调用进度回调
        if self._on_progress:
            self._on_progress(self._received_param_count, self._expected_param_count)
        
        # 记录日志（每 50 个参数记录一次）
        if self._received_param_count % 50 == 0:
            self.logger.info(
                f"已接收 {self._received_param_count}/{self._expected_param_count} 个参数"
            )
    
    def pull_all_params(self, timeout_sec: float = 60.0) -> bool:
        """
        从飞控拉取所有参数（非阻塞异步版本）
        
        工作流程：
        1. 清空当前参数缓存
        2. 开始监听 /mavros/param/param_value topic
        3. 调用 ParamPull 服务（不等待完成）
        4. MAVROS 会逐个发布参数到 topic
        5. 通过回调接收参数并更新进度
        6. 通过定时器检查是否完成
        
        Args:
            timeout_sec: 超时时间（秒）
        
        Returns:
            bool: 是否成功启动（不代表加载完成）
        """
        try:
            # 检查是否已在加载
            if self._is_loading:
                self.logger.warn("参数加载已在进行中")
                return False
            
            # 检查服务是否可用（增加超时到 10 秒）
            if not self.pull_client.wait_for_service(timeout_sec=10.0):
                error_msg = (
                    "ParamPull 服务不可用。\n\n"
                    "可能原因：\n"
                    "1. MAVROS param 插件未启用\n"
                    "2. USV 未连接或离线\n"
                    "3. 飞控通信链路异常\n\n"
                    "解决方案：\n"
                    "- 检查 usv_launch.py 中是否启用了 'param' 插件\n"
                    "- 确认 USV 在线并连接正常\n"
                    "- 等待 MAVROS 完成参数同步（可能需要 1-2 分钟）\n"
                    "- 检查串口通信质量（波特率 921600）"
                )
                self.logger.error(error_msg)
                if self._on_complete:
                    self._on_complete(False, error_msg)
                return False
            
            # 重置状态
            self._params.clear()
            self._is_loading = True
            self._load_start_time = time.time()
            self._received_param_count = 0
            self._expected_param_count = 0
            
            # 调用 ParamPull 服务（异步，不等待）
            request = ParamPull.Request()
            future = self.pull_client.call_async(request)
            
            # 添加回调处理响应
            future.add_done_callback(self._on_pull_response)
            
            self.logger.info(f"开始从 {self.usv_namespace} 拉取参数...")
            
            # 启动完成检查定时器（每秒检查一次）
            self._start_completion_timer(timeout_sec)
            
            return True
            
        except Exception as e:
            error_msg = f"启动参数拉取失败: {e}"
            self.logger.error(error_msg)
            self._is_loading = False
            if self._on_complete:
                self._on_complete(False, error_msg)
            return False
    
    def _on_pull_response(self, future):
        """
        ParamPull 服务响应回调
        
        Args:
            future: 服务调用的 future 对象
        """
        try:
            response = future.result()
            
            # 检查响应中的参数数量
            # 注意：ParamPull.Response 中有 param_received 字段表示接收到的参数数量
            self._expected_param_count = response.param_received
            
            self.logger.info(f"ParamPull 响应: 预期接收 {self._expected_param_count} 个参数")
            
            # 如果参数数为 0，可能是失败
            if self._expected_param_count == 0:
                self.logger.warn("ParamPull 响应参数数为 0，可能拉取失败")
                # 设置默认预期值（ArduPilot 通常有 400-600 个参数）
                self._expected_param_count = 500
                
        except Exception as e:
            self.logger.error(f"处理 ParamPull 响应失败: {e}")
            # 设置默认预期值
            self._expected_param_count = 500
    
    def _start_completion_timer(self, timeout_sec: float):
        """
        启动完成检查定时器
        
        定期检查参数加载是否完成或超时
        
        Args:
            timeout_sec: 超时时间（秒）
        """
        def check_completion():
            if not self._is_loading:
                return  # 已经停止加载
            
            elapsed_time = time.time() - self._load_start_time
            
            # 检查是否超时
            if elapsed_time > timeout_sec:
                self._is_loading = False
                error_msg = f"参数加载超时（{timeout_sec}秒）。已接收 {self._received_param_count} 个参数。"
                self.logger.error(error_msg)
                if self._on_complete:
                    self._on_complete(False, error_msg)
                return
            
            # 检查是否完成（接收数量达到预期）
            # 由于 MAVROS 可能不准确报告总数，我们设置一个容错值
            # 如果接收数量 >= 预期的 95%，且超过 3 秒没有新参数，则认为完成
            if self._expected_param_count > 0:
                progress = self._received_param_count / self._expected_param_count
                if progress >= 0.95:
                    # 等待 3 秒确认没有新参数
                    time.sleep(3)
                    
                    # 再次检查数量是否变化
                    final_count = self._received_param_count
                    if final_count >= self._expected_param_count * 0.95:
                        self._is_loading = False
                        success_msg = f"成功加载 {final_count} 个参数"
                        self.logger.info(success_msg)
                        
                        # 保存到缓存
                        self.save_cache()
                        
                        if self._on_complete:
                            self._on_complete(True, success_msg)
                        return
            
            # 继续检查（1 秒后）
            timer = threading.Timer(1.0, check_completion)
            timer.daemon = True
            timer.start()
        
        # 启动第一次检查
        timer = threading.Timer(1.0, check_completion)
        timer.daemon = True
        timer.start()
    
    def _read_params_from_ros(self) -> bool:
        """
        从 ROS 参数服务器读取参数列表
        
        Returns:
            bool: 是否成功
        """
        try:
            # 等待服务可用
            if not self.ros_param_client.wait_for_service(timeout_sec=3.0):
                self.logger.warn("ROS 参数服务不可用，尝试直接读取参数")
                # 降级方案：使用节点自带的参数接口
                return self._read_params_direct()
            
            # 调用 get_parameters 服务
            # 注意：需要先知道参数名称列表
            # 这里使用降级方案直接读取
            return self._read_params_direct()
            
        except Exception as e:
            self.logger.error(f"从 ROS 读取参数异常: {e}")
            return False
    
    def _read_params_direct(self) -> bool:
        """
        直接从节点读取参数（降级方案）
        
        由于 MAVROS param 插件将参数存储在节点内部，
        我们需要通过单独的 ParamGet 调用逐个读取。
        
        Returns:
            bool: 是否成功
        """
        # 这里需要知道参数名称列表
        # 由于 ParamPull 只是将参数存储到内部，
        # 我们需要使用其他方式获取参数列表
        
        # 临时方案：使用常见参数名称列表进行测试
        # TODO: 实现参数列表获取（可能需要订阅 /mavros/param/param_value topic）
        
        self.logger.warn("直接参数读取功能待实现，当前返回空列表")
        return False
    
    def get_param(self, param_name: str) -> Optional[ParamInfo]:
        """
        获取单个参数
        
        Args:
            param_name: 参数名称
        
        Returns:
            ParamInfo 或 None
        """
        return self._params.get(param_name)
    
    def set_param(self, param_name: str, value: float) -> bool:
        """
        设置单个参数值
        
        Args:
            param_name: 参数名称
            value: 新值
        
        Returns:
            bool: 是否成功
        """
        try:
            if param_name not in self._params:
                self.logger.error(f"参数不存在: {param_name}")
                return False
            
            param_info = self._params[param_name]
            
            # 等待服务可用
            if not self.set_client.wait_for_service(timeout_sec=3.0):
                self.logger.error("ParamSet 服务不可用")
                return False
            
            # 构造请求
            request = ParamSet.Request()
            request.param_id = param_name
            
            # 根据参数类型设置值
            if param_info.param_type == ParamType.INTEGER:
                request.value.integer = int(value)
                request.value.real = 0.0
            else:
                request.value.integer = 0
                request.value.real = float(value)
            
            # 调用服务
            future = self.set_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            
            if not future.done():
                self.logger.error(f"设置参数 {param_name} 超时")
                return False
            
            response = future.result()
            if response and response.success:
                # 更新缓存
                param_info.value = value
                self.logger.info(f"成功设置参数 {param_name} = {value}")
                return True
            else:
                self.logger.error(f"设置参数 {param_name} 失败")
                return False
                
        except Exception as e:
            self.logger.error(f"设置参数异常: {e}")
            return False
    
    def get_all_params(self) -> Dict[str, ParamInfo]:
        """
        获取所有参数
        
        Returns:
            Dict[str, ParamInfo]: 参数字典
        """
        return self._params.copy()
    
    def get_param_groups(self) -> List[str]:
        """
        获取参数分组列表
        
        Returns:
            List[str]: 分组名称列表（排序）
        """
        groups = set(param.group for param in self._params.values())
        return sorted(groups)
    
    def get_params_by_group(self, group: str) -> List[ParamInfo]:
        """
        获取指定分组的参数
        
        Args:
            group: 分组名称
        
        Returns:
            List[ParamInfo]: 参数列表
        """
        return [
            param for param in self._params.values()
            if param.group == group
        ]
    
    def get_modified_params(self) -> List[ParamInfo]:
        """
        获取已修改的参数
        
        Returns:
            List[ParamInfo]: 已修改参数列表
        """
        return [
            param for param in self._params.values()
            if param.is_modified
        ]
    
    def reset_all_modified(self):
        """重置所有修改"""
        for param in self._params.values():
            if param.is_modified:
                param.reset()
    
    def save_all_modified(self) -> bool:
        """
        保存所有修改的参数到飞控
        
        Returns:
            bool: 是否全部成功
        """
        modified = self.get_modified_params()
        if not modified:
            self.logger.info("没有需要保存的修改")
            return True
        
        self.logger.info(f"开始保存 {len(modified)} 个修改的参数...")
        
        success_count = 0
        for param in modified:
            if self.set_param(param.name, param.value):
                success_count += 1
                param.original_value = param.value  # 更新原始值
        
        if success_count == len(modified):
            self.logger.info(f"成功保存所有 {success_count} 个参数")
            return True
        else:
            self.logger.warn(
                f"部分参数保存失败: {success_count}/{len(modified)}"
            )
            return False
    
    # ==================== 缓存功能 ====================
    
    def save_cache(self) -> bool:
        """
        保存参数到缓存文件
        
        Returns:
            bool: 是否成功
        """
        try:
            # 准备缓存数据
            cache_data = {
                'timestamp': datetime.now().isoformat(),
                'usv_namespace': self.usv_namespace,
                'param_count': len(self._params),
                'params': {
                    name: param.to_dict() 
                    for name, param in self._params.items()
                }
            }
            
            # 写入文件
            with open(self._cache_file, 'w', encoding='utf-8') as f:
                json.dump(cache_data, f, indent=2, ensure_ascii=False)
            
            self.logger.info(f"参数已缓存到 {self._cache_file}")
            return True
            
        except Exception as e:
            self.logger.error(f"保存缓存失败: {e}")
            return False
    
    def load_cache(self) -> bool:
        """
        从缓存文件加载参数
        
        Returns:
            bool: 是否成功
        """
        try:
            # 检查缓存文件是否存在
            if not self._cache_file.exists():
                self.logger.info("缓存文件不存在")
                return False
            
            # 读取缓存
            with open(self._cache_file, 'r', encoding='utf-8') as f:
                cache_data = json.load(f)
            
            # 验证缓存
            if cache_data.get('usv_namespace') != self.usv_namespace:
                self.logger.warn("缓存文件命名空间不匹配")
                return False
            
            # 检查缓存是否过期
            cache_time = datetime.fromisoformat(cache_data['timestamp'])
            if datetime.now() - cache_time > timedelta(hours=self._cache_expiry_hours):
                self.logger.info(
                    f"缓存已过期（超过 {self._cache_expiry_hours} 小时）"
                )
                return False
            
            # 加载参数
            self._params.clear()
            for name, param_dict in cache_data['params'].items():
                self._params[name] = ParamInfo.from_dict(param_dict)
            
            cache_age = datetime.now() - cache_time
            hours = int(cache_age.total_seconds() / 3600)
            minutes = int((cache_age.total_seconds() % 3600) / 60)
            
            self.logger.info(
                f"从缓存加载了 {len(self._params)} 个参数 "
                f"（缓存时间: {hours}小时{minutes}分钟前）"
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"加载缓存失败: {e}")
            return False
    
    def is_cache_valid(self) -> bool:
        """
        检查缓存是否有效
        
        Returns:
            bool: 缓存是否存在且未过期
        """
        if not self._cache_file.exists():
            return False
        
        try:
            with open(self._cache_file, 'r', encoding='utf-8') as f:
                cache_data = json.load(f)
            
            cache_time = datetime.fromisoformat(cache_data['timestamp'])
            return datetime.now() - cache_time <= timedelta(hours=self._cache_expiry_hours)
            
        except Exception:
            return False
    
    def get_cache_info(self) -> Optional[dict]:
        """
        获取缓存信息
        
        Returns:
            dict: 缓存信息（时间戳、参数数量等）或 None
        """
        if not self._cache_file.exists():
            return None
        
        try:
            with open(self._cache_file, 'r', encoding='utf-8') as f:
                cache_data = json.load(f)
            
            cache_time = datetime.fromisoformat(cache_data['timestamp'])
            cache_age = datetime.now() - cache_time
            
            return {
                'timestamp': cache_time,
                'age_hours': cache_age.total_seconds() / 3600,
                'param_count': cache_data['param_count'],
                'is_valid': self.is_cache_valid(),
                'file_path': str(self._cache_file)
            }
            
        except Exception as e:
            self.logger.error(f"获取缓存信息失败: {e}")
            return None
    
    def clear_cache(self) -> bool:
        """
        清除缓存文件
        
        Returns:
            bool: 是否成功
        """
        try:
            if self._cache_file.exists():
                self._cache_file.unlink()
                self.logger.info("缓存已清除")
            return True
        except Exception as e:
            self.logger.error(f"清除缓存失败: {e}")
            return False


class ParamManagerAsync:
    """
    异步参数管理器
    
    由于新的 pull_all_params 实现已经是异步的（基于 topic 订阅和定时器），
    这个类主要用于保持接口兼容性。
    """
    
    def __init__(self, node: Node, usv_namespace: str):
        self.manager = ParamManager(node, usv_namespace)
    
    def pull_all_params_async(self, 
                              on_progress: Optional[Callable[[int, int], None]] = None,
                              on_complete: Optional[Callable[[bool, str], None]] = None):
        """
        异步拉取所有参数
        
        注意：新的实现中，pull_all_params 本身就是异步的，
        所以这里直接调用即可，不需要额外的线程。
        
        Args:
            on_progress: 进度回调
            on_complete: 完成回调
        """
        self.manager.set_callbacks(on_progress, on_complete)
        return self.manager.pull_all_params()
    
    def __getattr__(self, name):
        """代理其他方法到同步管理器"""
        return getattr(self.manager, name)
