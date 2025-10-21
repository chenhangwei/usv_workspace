"""
自动设置Home点节点

该节点用于自动设置无人船的EKF原点。当无人船启动并获取到第一个位置信息后，
经过指定延迟时间后自动设置EKF原点，确保系统定位正确初始化。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandHome
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

        # 订阅本地位置信息
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',
            self.local_position_callback,
            qos_best_effort
        )

        # 创建设置Home点的服务客户端
        self.set_home_cli = self.create_client(CommandHome, 'cmd/set_home')

        self.get_logger().info(
            f'AutoSetHomeNode initialized with {self.set_delay_sec}s delay, waiting for vision pose...'
        )

    def local_position_callback(self, msg):
        """
        本地位置回调函数
        
        当接收到第一个本地位置消息时，启动定时器在指定延迟后设置EKF原点。
        
        Args:
            msg (PoseStamped): 包含本地位置信息的消息
        """
        if not self.set_home_sent:
            self.set_home_sent = True
            self.get_logger().info(
                f'Received first local_position, will set EKF origin in {self.set_delay_sec:.1f} sec...'
            )
            self.create_timer(self.set_delay_sec, self.set_home)

    def set_home(self):
        """设置EKF原点"""
        # 检查服务是否可用
        if not self.set_home_cli.service_is_ready():
            self.get_logger().warn('Service /mavros/cmd/set_home not ready, retrying...')
            self.set_home_sent = False
            return

        # 创建服务请求
        req = CommandHome.Request()
        req.current_gps = True
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        # 异步调用服务
        future = self.set_home_cli.call_async(req)
        
        # 等待服务响应
        rclpy.spin_until_future_complete(self, future)

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