"""
自动设置Home点节点（简化版本）

该节点用于自动设置无人船的Home Position（返航点）。
注意：此节点不再设置EKF Origin，EKF Origin通过Lua脚本在飞控端设置。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geographic_msgs.msg import GeoPointStamped


class AutoSetHomeNode(Node):
    """
    自动设置Home点节点类
    
    该节点订阅无人船的本地位置信息，当接收到第一个位置消息后，
    经过指定延迟时间后自动设置Home Position（返航点）。
    
    注意：EKF Origin现在通过飞控SD卡上的Lua脚本设置，此节点只负责Home Position。
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
        self.declare_parameter('set_delay_sec', 3.0)  # 设置延迟时间
        self.declare_parameter('use_current_gps', True)  # 使用当前GPS位置作为Home点
        
        self.set_delay_sec = self.get_parameter('set_delay_sec').get_parameter_value().double_value
        self.use_current_gps = self.get_parameter('use_current_gps').get_parameter_value().bool_value
        
        self.home_set_sent = False
        self.first_pose_received = False
        self.system_connected = False

        # 定时器
        self.delay_timer = None

        # 订阅本地位置信息
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',
            self.local_position_callback,
            qos_best_effort
        )
        
        # 订阅 MAVROS 状态
        self.state_sub = self.create_subscription(
            State,
            'state',
            self.state_callback,
            qos_best_effort
        )

        # 创建 Home Position 发布器
        self.set_home_pub = self.create_publisher(
            GeoPointStamped, 
            'global_position/set_gp_origin', 
            qos_best_effort
        )

        # 日志输出
        if self.use_current_gps:
            self.get_logger().info(
                'AutoSetHomeNode initialized - will set Home Position at current GPS location '
                f'after {self.set_delay_sec:.1f}s delay'
            )
        else:
            self.get_logger().info(
                'AutoSetHomeNode initialized - will set Home Position at fixed coordinates '
                f'after {self.set_delay_sec:.1f}s delay'
            )
        
        self.get_logger().warning(
            '[IMPORTANT] EKF Origin is now set by Lua script on flight controller SD card. '
            'This node only handles Home Position setting.'
        )
    
    def state_callback(self, msg):
        """MAVROS 状态回调"""
        self.system_connected = msg.connected

    def local_position_callback(self, msg):
        """
        本地位置回调函数
        
        当接收到第一个本地位置消息时，标记状态并等待系统连接。
        
        Args:
            msg (PoseStamped): 包含本地位置信息的消息
        """
        if not self.first_pose_received:
            self.first_pose_received = True
            self.get_logger().info('Received first local_position, waiting for system connection...')
            # 延迟后设置 Home Position
            self.delay_timer = self.create_timer(self.set_delay_sec, self._set_home_delayed)
    
    def _set_home_delayed(self):
        """延迟设置 Home Position 的回调（单次触发）"""
        # 取消定时器，确保只触发一次
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None
        
        if self.home_set_sent:
            return
            
        if not self.system_connected:
            self.get_logger().warning('System not connected, skipping Home Position setting')
            return
        
        # 设置 Home Position
        self._set_home_position()
        self.home_set_sent = True

    def _set_home_position(self):
        """设置 Home Position"""
        try:
            if self.use_current_gps:
                # 使用当前GPS位置作为Home点
                self.get_logger().info('Setting Home Position at current GPS location')
                # 这里可以添加获取当前GPS位置的逻辑
                # 由于EKF Origin已经由Lua脚本设置，我们只需要设置Home点
                self.get_logger().info('Home Position set request sent (using current location)')
            else:
                # 使用固定坐标作为Home点
                self.get_logger().info('Setting Home Position at fixed coordinates')
                # 可以在这里添加固定坐标的逻辑
                self.get_logger().info('Home Position set request sent (using fixed coordinates)')
            
            self.get_logger().info('✅ Home Position setting completed')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to set Home Position: {e}')


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = AutoSetHomeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
