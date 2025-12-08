"""
PX4 参数管理器 - PX4 uXRCE-DDS 版本

该模块提供 PX4 飞控参数的读取和设置功能。

注意：PX4 uXRCE-DDS 的参数管理与 MAVROS 不同：
1. PX4 通过 /fmu/out/parameter_update 话题发布参数更新
2. 参数设置需要通过 MAVLink 参数协议或 QGroundControl
3. 对于 uXRCE-DDS，通常使用 QGC 或直接配置 PX4 参数文件

本模块提供：
- 参数缓存和查询
- 通过 VehicleCommand 发送参数相关命令
- 与原 MAVROS 接口兼容的适配层
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from typing import Dict, Optional, Callable, Any
from dataclasses import dataclass
from enum import Enum
import threading

# PX4 消息类型
from px4_msgs.msg import VehicleCommand, ParameterUpdate


class ParamType(Enum):
    """参数类型枚举"""
    INT = 1
    FLOAT = 2
    UNKNOWN = 0


@dataclass
class ParamValue:
    """参数值数据类"""
    name: str
    value: Any
    param_type: ParamType
    timestamp: float = 0.0


@dataclass
class ParamInfo:
    """参数信息数据类（兼容旧接口）"""
    name: str
    value: Any = None
    param_type: ParamType = ParamType.UNKNOWN
    description: str = ""
    default_value: Any = None
    min_value: Any = None
    max_value: Any = None
    unit: str = ""


class ParamManager:
    """
    PX4 参数管理器
    
    提供与 MAVROS ParamManager 兼容的接口，用于 PX4 uXRCE-DDS 环境。
    
    注意：由于 uXRCE-DDS 的限制，完整的参数读写需要通过其他方式实现：
    1. 使用 QGroundControl 设置参数
    2. 使用 PX4 的 parameter 微服务
    3. 直接修改 PX4 参数文件
    
    本类主要用于：
    - 缓存和查询已知参数
    - 订阅参数更新通知
    - 提供兼容的接口给上层应用
    """

    def __init__(self, node: Node, usv_namespace: str = ''):
        """
        初始化参数管理器
        
        Args:
            node: ROS 2 节点实例
            usv_namespace: USV 命名空间
        """
        self.node = node
        self.usv_namespace = usv_namespace
        self.logger = node.get_logger()
        
        # QoS 配置
        self.qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 参数缓存
        self._params: Dict[str, ParamValue] = {}
        self._params_lock = threading.Lock()
        
        # 回调函数
        self._on_param_update: Optional[Callable[[str, Any], None]] = None
        self._on_progress: Optional[Callable[[int, int], None]] = None
        self._on_complete: Optional[Callable[[bool, str], None]] = None
        
        # 是否初始化完成
        self._initialized = False
        
        # 创建订阅器
        self._create_subscribers()
        
        # 创建发布器
        self._create_publishers()
        
        self.logger.info(f'✅ PX4 参数管理器已初始化 (namespace: {usv_namespace})')

    def _create_subscribers(self):
        """创建订阅器"""
        # 订阅参数更新（如果 PX4 支持）
        try:
            topic = 'fmu/out/parameter_update'
            if self.usv_namespace:
                topic = f'{self.usv_namespace}/{topic}'
            
            self._param_update_sub = self.node.create_subscription(
                ParameterUpdate,
                topic,
                self._param_update_callback,
                self.qos_px4
            )
        except Exception as e:
            self.logger.warning(f'无法订阅 ParameterUpdate: {e}')

    def _create_publishers(self):
        """创建发布器"""
        topic = 'fmu/in/vehicle_command'
        if self.usv_namespace:
            topic = f'{self.usv_namespace}/{topic}'
        
        self._command_pub = self.node.create_publisher(
            VehicleCommand,
            topic,
            self.qos_px4
        )

    def _param_update_callback(self, msg: ParameterUpdate):
        """参数更新回调"""
        try:
            # ParameterUpdate 消息包含已更新参数的索引
            # 但不包含具体的参数名和值
            # 需要通过其他方式获取参数详情
            
            if self._on_param_update:
                self._on_param_update('parameter_update', msg.instance)
                
        except Exception as e:
            self.logger.error(f'参数更新回调错误: {e}')

    def set_callbacks(self,
                     on_progress: Optional[Callable[[int, int], None]] = None,
                     on_complete: Optional[Callable[[bool, str], None]] = None,
                     on_param_update: Optional[Callable[[str, Any], None]] = None):
        """
        设置回调函数
        
        Args:
            on_progress: 进度回调 (current, total)
            on_complete: 完成回调 (success, message)
            on_param_update: 参数更新回调 (name, value)
        """
        self._on_progress = on_progress
        self._on_complete = on_complete
        self._on_param_update = on_param_update

    def get_param(self, name: str) -> Optional[Any]:
        """
        获取缓存的参数值
        
        Args:
            name: 参数名称
            
        Returns:
            参数值，如果不存在返回 None
        """
        with self._params_lock:
            if name in self._params:
                return self._params[name].value
        return None

    def get_all_params(self) -> Dict[str, Any]:
        """
        获取所有缓存的参数
        
        Returns:
            参数字典 {name: value}
        """
        with self._params_lock:
            return {name: p.value for name, p in self._params.items()}

    def set_param(self, name: str, value: Any) -> bool:
        """
        设置参数（本地缓存）
        
        注意：由于 uXRCE-DDS 限制，此方法只更新本地缓存。
        实际设置飞控参数需要使用 QGroundControl 或其他方式。
        
        Args:
            name: 参数名称
            value: 参数值
            
        Returns:
            是否成功
        """
        try:
            with self._params_lock:
                param_type = ParamType.FLOAT if isinstance(value, float) else ParamType.INT
                self._params[name] = ParamValue(
                    name=name,
                    value=value,
                    param_type=param_type,
                    timestamp=self.node.get_clock().now().nanoseconds / 1e9
                )
            
            self.logger.info(f'📝 参数已缓存: {name} = {value}')
            self.logger.warning('⚠️ 注意: PX4 uXRCE-DDS 不支持直接设置飞控参数，请使用 QGC')
            
            return True
            
        except Exception as e:
            self.logger.error(f'设置参数失败: {e}')
            return False

    def pull_all_params(self, timeout_sec: float = 60.0) -> bool:
        """
        拉取所有参数（兼容接口）
        
        注意：PX4 uXRCE-DDS 不支持直接拉取参数。
        建议使用 QGroundControl 或 PX4 Shell 获取参数。
        
        Args:
            timeout_sec: 超时时间
            
        Returns:
            是否成功
        """
        self.logger.warning(
            '⚠️ PX4 uXRCE-DDS 不支持直接拉取参数\n'
            '请使用以下方式获取 PX4 参数：\n'
            '1. QGroundControl: 参数页面\n'
            '2. PX4 Shell: param show\n'
            '3. 日志文件: flight_log'
        )
        
        if self._on_complete:
            self._on_complete(False, 'PX4 uXRCE-DDS 不支持参数拉取')
        
        return False

    def load_preset_params(self, preset: Dict[str, Any]):
        """
        加载预设参数到缓存
        
        用于加载已知的参数配置，便于 GUI 显示。
        
        Args:
            preset: 预设参数字典 {name: value}
        """
        with self._params_lock:
            for name, value in preset.items():
                param_type = ParamType.FLOAT if isinstance(value, float) else ParamType.INT
                self._params[name] = ParamValue(
                    name=name,
                    value=value,
                    param_type=param_type
                )
        
        self.logger.info(f'📚 已加载 {len(preset)} 个预设参数')

    def get_common_px4_params(self) -> Dict[str, str]:
        """
        获取常用 PX4 参数描述
        
        Returns:
            参数描述字典 {name: description}
        """
        return {
            'SYS_AUTOSTART': '自动启动机架 ID',
            'MAV_SYS_ID': 'MAVLink 系统 ID',
            'MAV_COMP_ID': 'MAVLink 组件 ID',
            'COM_ARM_SWISBTN': '解锁按钮模式',
            'COM_RC_ARM_HYST': 'RC 解锁滞后时间',
            'COM_DISARM_LAND': '着陆后自动上锁时间',
            'COM_DISARM_PRFLT': '预飞行自动上锁时间',
            'EKF2_AID_MASK': 'EKF2 辅助模式',
            'EKF2_HGT_MODE': 'EKF2 高度模式',
            'EKF2_MAG_TYPE': 'EKF2 磁力计类型',
            'MPC_XY_VEL_MAX': '最大水平速度',
            'MPC_Z_VEL_MAX_DN': '最大下降速度',
            'MPC_Z_VEL_MAX_UP': '最大上升速度',
            'MPC_TILTMAX_AIR': '最大倾斜角',
            'NAV_ACC_RAD': '航点接受半径',
            'RTL_RETURN_ALT': '返航高度',
            'RTL_DESCEND_ALT': '返航下降高度',
            'RTL_LAND_DELAY': '返航着陆延迟',
            'BAT_V_CHARGED': '满电电压',
            'BAT_V_EMPTY': '空电电压',
            'BAT_N_CELLS': '电池串数',
            'BAT_LOW_THR': '低电压阈值',
            'BAT_CRIT_THR': '严重低电压阈值',
        }

    def is_connected(self) -> bool:
        """
        检查是否连接（始终返回 True，因为 uXRCE-DDS 是被动接收）
        """
        return True

    def cleanup(self):
        """清理资源"""
        self._params.clear()
        self.logger.info('PX4 参数管理器已清理')
