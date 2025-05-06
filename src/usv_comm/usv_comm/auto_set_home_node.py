# 文件名：auto_set_home.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandHome
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class AutoSetHomeNode(Node):
    def __init__(self):
        super().__init__('auto_set_home_node')

        qos = QoSProfile(
                    depth=10,  # 队列大小
                    reliability=QoSReliabilityPolicy.BEST_EFFORT# 可靠性策略                   
                )
           # 创建 QoS 配置
        qos_a = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.RELIABLE # 可靠性策略          
        )

        self.declare_parameter('set_delay_sec', 3.0)
        param_value = self.get_parameter('set_delay_sec').value
        self.set_delay_sec = float(param_value) if param_value is not None else 3.0
        self.set_home_sent = False

        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',
            self.local_position_callback,
            qos 
        )

        self.cli = self.create_client(CommandHome, 'cmd/set_home')

        self.get_logger().info('AutoSetHomeNode initialized, waiting for vision pose...')

    def local_position_callback(self, msg):
        if not self.set_home_sent:
            self.set_home_sent = True
            self.get_logger().info('Received first local_position, will set EKF origin in %.1f sec...' % self.set_delay_sec)
            self.create_timer(self.set_delay_sec, self.set_home)

    def set_home(self):
        if not self.cli.service_is_ready():
            self.get_logger().warn('Service /mavros/cmd/set_home not ready, retrying...')
            self.set_home_sent = False
            return

        req = CommandHome.Request()
        req.current_gps = True
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is not None and getattr(result, 'success', False):
            self.get_logger().info('✅ EKF origin set successfully!')
        else:
            self.get_logger().error('❌ Failed to set EKF origin!')
            
def main(args=None):
    rclpy.init(args=args)
    node = AutoSetHomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()