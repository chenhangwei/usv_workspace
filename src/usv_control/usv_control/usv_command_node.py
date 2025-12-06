"""
无人船命令控制节点

该节点负责处理来自地面站的命令，包括模式切换和解锁/上锁操作。
通过MAVROS服务与飞控通信，执行相应的控制命令。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class UsvCommandNode(Node):
    """
    无人船命令控制节点类
    
    该节点订阅地面站发送的模式切换和解锁/上锁命令，
    通过MAVROS服务与飞控通信，执行相应的控制操作。
    """

    def __init__(self):
        """初始化无人船命令控制节点"""
        super().__init__('usv_command_node')

        # 创建 QoS 配置
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 声明参数
        self.declare_parameter('supported_modes', ['OFFBOARD', 'GUIDED', 'MANUAL', 'AUTO', 'HOLD','ARCO','STEERING'])
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('arming_command_timeout_sec', 10.0)

        # 获取参数值
        self.supported_modes = self.get_parameter('supported_modes').get_parameter_value().string_array_value
        self.service_timeout_sec = self.get_parameter('service_timeout_sec').get_parameter_value().double_value
        self.arming_command_timeout_sec = self.get_parameter('arming_command_timeout_sec').get_parameter_value().double_value

        # 防抖机制: 记录最后一次模式切换的时间和模式
        self.last_mode_command = None
        self.last_mode_time = 0.0
        self.mode_debounce_sec = 0.5  # 0.5秒防抖时间
        
        # 模式切换中标志,防止并发切换
        self.mode_switching = False
        
        # 订阅MAVROS状态,记录当前实际模式
        self.current_mavros_mode = None
        self.state_sub = self.create_subscription(
            State,
            'state',
            self.mavros_state_callback,
            qos_reliable
        )

        self.get_logger().info(f'支持的模式: {", ".join(self.supported_modes)}')
        self.get_logger().info(f'服务超时时间: {self.service_timeout_sec}秒')
        self.get_logger().info(f'解锁命令超时时间: {self.arming_command_timeout_sec}秒')

        # 订阅地面站的模式切换命令
        self.sub_mode = self.create_subscription(
            String,
            'set_usv_mode',
            self.set_mode_callback,
            qos_reliable
        )

        # 订阅地面站的解锁命令
        self.sub_arming = self.create_subscription(
            String,
            'set_usv_arming',
            self.set_arming_callback,
            qos_reliable
        )

        # 创建服务客户端
        self.arming_client = self.create_client(
            CommandBool,
            'cmd/arming'
        )

        self.mode_client = self.create_client(
            SetMode,
            'set_mode'
        )

        self.get_logger().info('USV 命令控制节点已启动')

    def mavros_state_callback(self, msg):
        """
        MAVROS状态回调,更新当前实际模式
        
        Args:
            msg (State): MAVROS状态消息
        """
        self.current_mavros_mode = msg.mode

    def set_mode_callback(self, msg):
        """
        处理模式切换命令回调函数
        
        Args:
            msg (String): 包含目标模式的字符串消息
        """
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的模式消息类型')
            return

        # 检查模式是否受支持
        if msg.data not in self.supported_modes:
            self.get_logger().error(f'不支持的模式: {msg.data}，支持的模式: {", ".join(self.supported_modes)}')
            return

        # 防抖: 0.5秒内相同模式只处理一次(优先检查,避免日志刷屏)
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (self.last_mode_command == msg.data and 
            current_time - self.last_mode_time < self.mode_debounce_sec):
            # 静默忽略重复命令,不打印日志避免刷屏
            return

        # 如果当前模式已经是目标模式,静默忽略
        if self.current_mavros_mode == msg.data:
            # 更新防抖时间,避免频繁检查
            self.last_mode_command = msg.data
            self.last_mode_time = current_time
            return

        # 如果已经在切换中,静默拒绝新请求
        if self.mode_switching:
            return
        
        self.get_logger().info(f'收到模式消息: {msg.data}')
        self.last_mode_command = msg.data
        self.last_mode_time = current_time
        self.mode_switching = True  # 标记切换中

        # 等待服务可用
        try:
            if not self.mode_client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.get_logger().error('模式服务不可用')
                return
        except Exception as e:
            self.get_logger().error(f'等待模式服务时发生异常: {e}')
            return

        # 切换模式
        mode_req = SetMode.Request()
        mode_req.custom_mode = msg.data
        future = self.mode_client.call_async(mode_req)
        future.add_done_callback(lambda future: self.mode_done_callback(future, msg.data))

    def mode_done_callback(self, future, mode):
        """
        模式切换完成回调函数
        
        Args:
            future: 异步调用的future对象
            mode (str): 目标模式名称
        """
        self.mode_switching = False  # 清除切换标志
        try:
            response = future.result()
            if response and response.mode_sent:
                self.get_logger().info(f'[OK] 成功切换到模式: {mode}')
            else:
                self.get_logger().warn(f'[!] 切换模式 {mode} 失败')
        except Exception as e:
            self.get_logger().error(f'[X] 切换模式 {mode} 时发生异常: {e}')

    def set_arming_callback(self, msg):
        """
        处理解锁/上锁命令回调函数
        
        Args:
            msg (String): 包含解锁/上锁命令的字符串消息
        """
        self.get_logger().info(f'收到解锁消息: {msg.data}')
        
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的解除武装消息类型')
            return

        # 验证命令类型
        if msg.data not in ['ARMING', 'DISARMING']:
            self.get_logger().error(f'无效的解锁命令: {msg.data}，仅支持 ARMING/DISARMING')
            return

        # 等待服务可用
        try:
            if not self.arming_client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.get_logger().error('武装服务不可用')
                return
        except Exception as e:
            self.get_logger().error(f'等待武装服务时发生异常: {e}')
            return

        # 设置解锁或上锁
        arm_req = CommandBool.Request()
        arm_req.value = (msg.data == 'ARMING')
        
        future = self.arming_client.call_async(arm_req)
        future.add_done_callback(lambda future: self.arming_done_callback(future, msg.data))

    def arming_done_callback(self, future, command):
        """
        解锁/上锁完成回调函数
        
        Args:
            future: 异步调用的future对象
            command (str): 命令类型 (ARMING/DISARMING)
        """
        try:
            response = future.result()
            state = 'armed' if command == 'ARMING' else 'disarmed'
            if response and response.success:
                self.get_logger().info(f'[OK] Vehicle {state} successfully')
            else:
                self.get_logger().warn(f'[!] 设置 {state} 失败')
        except Exception as e:
            self.get_logger().error(f'[X] 设置 {state} 时发生异常: {e}')

    def destroy_node(self):
        """节点销毁时的资源清理"""
        # 该节点没有 timer,只需调用父类方法
        super().destroy_node()


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = UsvCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()