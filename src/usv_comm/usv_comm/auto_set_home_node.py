"""
自动设置Home点节点

该节点用于自动设置无人船的EKF原点。当无人船启动并获取到第一个位置信息后，
经过指定延迟时间后自动设置EKF原点，确保系统定位正确初始化。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandHome
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class AutoSetHomeNode(Node):
    """
    自动设置Home点节点类
    
    该节点订阅无人船的本地位置信息，当接收到第一个位置消息后，
    经过指定延迟时间后自动调用MAVROS服务设置EKF原点。
    """

    def __init__(self):
        """初始化自动设置Home点节点"""
        super().__init__('auto_set_home_node')

        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 声明并获取参数
        self.declare_parameter('set_delay_sec', 3.0)
        self.set_delay_sec = self.get_parameter('set_delay_sec').get_parameter_value().double_value
        self.set_home_sent = False
        
        # 重试相关状态变量
        self.retry_count = 0
        self.max_retries = 30
        self.retry_interval = 1.0  # 秒
        self.retry_timer = None
        
        # GPS 和系统状态
        self.gps_fix_type = 0  # 0=无定位, 2=2D, 3=3D
        self.system_connected = False
        self.first_pose_received = False

        # 订阅本地位置信息
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',
            self.local_position_callback,
            qos_best_effort
        )
        
        # 订阅 GPS 状态
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'global_position/global',
            self.gps_callback,
            qos_best_effort
        )
        
        # 订阅 MAVROS 状态
        self.state_sub = self.create_subscription(
            State,
            'state',
            self.state_callback,
            qos_best_effort
        )

        # 创建设置Home点的服务客户端
        self.set_home_cli = self.create_client(CommandHome, 'cmd/set_home')

        self.get_logger().info(
            f'AutoSetHomeNode initialized with {self.set_delay_sec}s delay, waiting for GPS fix...'
        )
    
    def gps_callback(self, msg):
        """GPS 状态回调"""
        self.gps_fix_type = msg.status.status
        # status: -1=无服务, 0=无定位, 1=GPS定位, 2=DGPS定位
        # 我们需要至少 GPS 定位 (status >= 0)
    
    def state_callback(self, msg):
        """MAVROS 状态回调"""
        self.system_connected = msg.connected

    def local_position_callback(self, msg):
        """
        本地位置回调函数
        
        当接收到第一个本地位置消息时，标记状态并等待 GPS 就绪。
        
        Args:
            msg (PoseStamped): 包含本地位置信息的消息
        """
        if not self.first_pose_received:
            self.first_pose_received = True
            self.get_logger().info('Received first local_position, waiting for GPS fix...')
            # 启动定期检查定时器
            self.create_timer(1.0, self.check_and_set_home)
    
    def check_and_set_home(self):
        """检查 GPS 和系统状态，满足条件后设置 Home 点"""
        if self.set_home_sent:
            return  # 已经设置过，不再重复
        
        # 检查系统是否连接
        if not self.system_connected:
            self.get_logger().debug('Waiting for MAVROS connection...')
            return
        
        # 检查 GPS 定位状态
        # NavSatFix.status.status: -1=无服务, 0=无定位, 1=GPS定位, 2=DGPS
        if self.gps_fix_type < 0:
            self.get_logger().info(
                f'Waiting for GPS fix (current: {self.gps_fix_type}, need >= 0)...'
            )
            return
        
        # 所有条件满足，设置 Home 点
        self.get_logger().info(
            f'✅ GPS fix obtained (type: {self.gps_fix_type}), system connected. '
            f'Setting Home point in {self.set_delay_sec:.1f}s...'
        )
        self.set_home_sent = True
        self.create_timer(self.set_delay_sec, self.set_home)

    def set_home(self):
        """设置EKF原点的入口函数,启动非阻塞重试定时器"""
        self.retry_count = 0
        self._try_set_home()
    
    def _try_set_home(self):
        """尝试设置EKF原点,使用非阻塞定时器实现重试"""
        # 检查服务是否就绪
        if self.set_home_cli.service_is_ready():
            self.get_logger().info(
                f'Service /mavros/cmd/set_home is ready (attempt {self.retry_count + 1}/{self.max_retries})'
            )
            self._send_set_home_request()
            return
        
        # 服务未就绪,记录并准备重试
        self.retry_count += 1
        self.get_logger().warn(
            f'Service /mavros/cmd/set_home not ready, will retry... ({self.retry_count}/{self.max_retries})'
        )
        
        # 检查是否达到最大重试次数
        if self.retry_count >= self.max_retries:
            self.get_logger().error(
                f'❌ Service /mavros/cmd/set_home not available after {self.max_retries} attempts. '
                'Please check MAVROS connection.'
            )
            return
        
        # 创建定时器在指定间隔后重试
        if self.retry_timer is not None:
            self.retry_timer.cancel()
        
        self.retry_timer = self.create_timer(self.retry_interval, self._retry_callback)
    
    def _retry_callback(self):
        """重试定时器回调"""
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        
        self._try_set_home()
    
    def _send_set_home_request(self):
        """发送设置Home点的服务请求"""
        # 创建服务请求
        req = CommandHome.Request()
        req.current_gps = True
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        # 异步调用服务
        future = self.set_home_cli.call_async(req)
        future.add_done_callback(self._handle_set_home_response)
    
    def _handle_set_home_response(self, future):
        """处理设置Home点的服务响应"""
        try:
            result = future.result()
            if result is not None and getattr(result, 'success', False):
                self.get_logger().info('✅ EKF origin set successfully!')
            else:
                self.get_logger().error('❌ Failed to set EKF origin!')
        except Exception as e:
            self.get_logger().error(f'Exception occurred while setting EKF origin: {e}')


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = AutoSetHomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()