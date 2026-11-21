"""
PX4 命令接口模块 - PX4 uXRCE-DDS 版本

该模块提供与 MAVROS CommandLong 服务兼容的接口，
通过 PX4 VehicleCommand 消息实现飞控命令发送。

用于替代 ground_station_node 中的 MAVROS 命令调用。
"""

from typing import Optional, Callable
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# PX4 消息类型
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, HomePosition


class Px4CommandInterface:
    """
    PX4 命令接口类
    
    提供与 MAVROS CommandLong 服务类似的接口，
    通过 PX4 VehicleCommand 话题发送命令。
    """
    
    # MAVLink 命令 ID
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
    MAV_CMD_DO_SET_HOME = 179
    MAV_CMD_DO_SET_MODE = 176
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_CMD_NAV_LAND = 21
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    MAV_CMD_DO_REPOSITION = 192
    
    def __init__(self, node: Node, usv_namespace: str = ''):
        """
        初始化命令接口
        
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
        
        # 命令计数器
        self._command_counter = 0
        
        # 创建发布器和订阅器
        self._create_publishers()
        self._create_subscribers()
        
        # 等待 ACK 的回调
        self._pending_callbacks = {}
        
        self.logger.info(f'✅ PX4 命令接口已初始化 (namespace: {usv_namespace})')

    def _create_publishers(self):
        """创建发布器"""
        topic = 'fmu/in/vehicle_command'
        if self.usv_namespace:
            topic = f'/{self.usv_namespace}/{topic}'
        
        self._command_pub = self.node.create_publisher(
            VehicleCommand,
            topic,
            self.qos_px4
        )

    def _create_subscribers(self):
        """创建订阅器"""
        topic = 'fmu/out/vehicle_command_ack'
        if self.usv_namespace:
            topic = f'/{self.usv_namespace}/{topic}'
        
        try:
            self._ack_sub = self.node.create_subscription(
                VehicleCommandAck,
                topic,
                self._command_ack_callback,
                self.qos_px4
            )
        except Exception as e:
            self.logger.warning(f'无法订阅 VehicleCommandAck: {e}')

    def _command_ack_callback(self, msg: VehicleCommandAck):
        """命令确认回调"""
        cmd_id = msg.command
        result = msg.result
        
        # 检查是否有等待的回调
        if cmd_id in self._pending_callbacks:
            callback = self._pending_callbacks.pop(cmd_id)
            success = (result == 0)  # VEHICLE_CMD_RESULT_ACCEPTED
            callback(success, result)

    def send_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        target_system: int = 0,
        target_component: int = 1,
        callback: Optional[Callable[[bool, int], None]] = None
    ) -> bool:
        """
        发送 VehicleCommand
        
        Args:
            command: MAVLink 命令 ID
            param1-7: 命令参数
            target_system: 目标系统 ID
            target_component: 目标组件 ID
            callback: 完成回调 (success, result_code)
            
        Returns:
            是否成功发送
        """
        try:
            msg = VehicleCommand()
            msg.command = command
            msg.param1 = float(param1)
            msg.param2 = float(param2)
            msg.param3 = float(param3)
            msg.param4 = float(param4)
            msg.param5 = float(param5)
            msg.param6 = float(param6)
            msg.param7 = float(param7)
            msg.target_system = target_system
            msg.target_component = target_component
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
            
            self._command_pub.publish(msg)
            
            # 注册回调
            if callback:
                self._pending_callbacks[command] = callback
            
            self._command_counter += 1
            return True
            
        except Exception as e:
            self.logger.error(f'发送命令失败: {e}')
            return False

    def reboot_autopilot(self, callback: Optional[Callable[[bool, int], None]] = None) -> bool:
        """
        重启飞控
        
        Args:
            callback: 完成回调
            
        Returns:
            是否成功发送命令
        """
        self.logger.info('🔄 发送飞控重启命令...')
        return self.send_command(
            command=self.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            param1=1.0,  # 重启飞控
            param2=0.0,  # 不重启机载计算机
            callback=callback
        )

    def reboot_companion(self, callback: Optional[Callable[[bool, int], None]] = None) -> bool:
        """
        重启机载计算机（通过 MAVLink）
        
        注意：某些飞控可能不支持此命令
        
        Args:
            callback: 完成回调
            
        Returns:
            是否成功发送命令
        """
        self.logger.info('🔄 发送机载计算机重启命令...')
        return self.send_command(
            command=self.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            param1=0.0,  # 不重启飞控
            param2=3.0,  # 重启机载计算机
            callback=callback
        )

    def set_home_position(
        self,
        use_current: bool = True,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        callback: Optional[Callable[[bool, int], None]] = None
    ) -> bool:
        """
        设置 Home 位置（局部坐标系）
        
        Args:
            use_current: 是否使用当前位置
            x, y, z: 局部坐标（use_current=False 时使用，ENU坐标系，单位：米）
            callback: 完成回调
            
        Returns:
            是否成功发送命令
        """
        if use_current:
            self.logger.info('📍 设置 Home 为当前位置...')
            return self.send_command(
                command=self.MAV_CMD_DO_SET_HOME,
                param1=1.0,  # 使用当前位置
                callback=callback
            )
        else:
            self.logger.info(f'📍 设置 Home 为局部坐标: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m')
            
            # 发布 HomePosition 消息来设置局部坐标系的 Home
            try:
                # 创建发布器
                topic_name = f'/{self.usv_namespace}/fmu/in/home_position' if self.usv_namespace else '/fmu/in/home_position'
                home_pub = self.node.create_publisher(HomePosition, topic_name, 10)
                
                # 创建消息
                home_msg = HomePosition()
                home_msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)  # 微秒
                home_msg.x = float(x)
                home_msg.y = float(y)
                home_msg.z = float(z)
                home_msg.yaw = 0.0
                home_msg.valid_lpos = True
                home_msg.valid_hpos = False  # 不使用 GPS 坐标
                home_msg.valid_alt = True
                home_msg.manual_home = True
                
                # 发布消息
                home_pub.publish(home_msg)
                self.logger.info(f'✅ 已发布 HomePosition 消息到 {topic_name}')
                
                # 销毁发布器
                self.node.destroy_publisher(home_pub)
                
                if callback:
                    callback(True, 0)
                return True
            except Exception as e:
                self.logger.error(f'❌ 发布 HomePosition 失败: {e}')
                if callback:
                    callback(False, -1)
                return False

    def arm(self, callback: Optional[Callable[[bool, int], None]] = None) -> bool:
        """
        解锁飞控
        
        Returns:
            是否成功发送命令
        """
        self.logger.info('🔓 发送解锁命令...')
        return self.send_command(
            command=self.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,  # 1=解锁
            param2=0.0,  # 0=正常解锁, 21196=强制解锁
            callback=callback
        )

    def disarm(self, force: bool = False, callback: Optional[Callable[[bool, int], None]] = None) -> bool:
        """
        上锁飞控
        
        Args:
            force: 是否强制上锁
            callback: 完成回调
            
        Returns:
            是否成功发送命令
        """
        self.logger.info('🔒 发送上锁命令...')
        return self.send_command(
            command=self.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,  # 0=上锁
            param2=21196.0 if force else 0.0,  # 21196=强制上锁
            callback=callback
        )

    def takeoff(self, altitude: float = 2.0, callback: Optional[Callable[[bool, int], None]] = None) -> bool:
        """
        起飞
        
        Args:
            altitude: 目标高度
            callback: 完成回调
            
        Returns:
            是否成功发送命令
        """
        self.logger.info(f'🛫 发送起飞命令 (目标高度: {altitude}m)...')
        return self.send_command(
            command=self.MAV_CMD_NAV_TAKEOFF,
            param7=altitude,
            callback=callback
        )

    def land(self, callback: Optional[Callable[[bool, int], None]] = None) -> bool:
        """
        降落
        
        Returns:
            是否成功发送命令
        """
        self.logger.info('🛬 发送降落命令...')
        return self.send_command(
            command=self.MAV_CMD_NAV_LAND,
            callback=callback
        )

    def return_to_launch(self, callback: Optional[Callable[[bool, int], None]] = None) -> bool:
        """
        返航
        
        Returns:
            是否成功发送命令
        """
        self.logger.info('🏠 发送返航命令...')
        return self.send_command(
            command=self.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            callback=callback
        )

    def goto_gps(
        self,
        lat: float,
        lon: float,
        alt: float,
        ground_speed: float = -1.0,
        callback: Optional[Callable[[bool, int], None]] = None
    ) -> bool:
        """
        飞往指定 GPS 坐标
        
        Args:
            lat, lon, alt: 目标 GPS 坐标
            ground_speed: 地速，-1 使用默认值
            callback: 完成回调
            
        Returns:
            是否成功发送命令
        """
        self.logger.info(f'📍 飞往 GPS 坐标: ({lat:.7f}, {lon:.7f}, {alt:.2f})')
        return self.send_command(
            command=self.MAV_CMD_DO_REPOSITION,
            param1=ground_speed,
            param4=float('nan'),  # Yaw 保持不变
            param5=lat,
            param6=lon,
            param7=alt,
            callback=callback
        )

    def cleanup(self):
        """清理资源"""
        self._pending_callbacks.clear()
        self.logger.info('PX4 命令接口已清理')
