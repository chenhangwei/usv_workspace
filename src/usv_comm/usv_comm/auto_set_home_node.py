"""
自动设置Home点节点

该节点用于自动设置无人球的Home Position（返航点）。
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
    
    该节点订阅无人球的本地位置信息，当接收到第一个位置消息后，
    经过指定延迟时间后自动设置Home Position（返航点）。
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
        # 固定坐标参数 (默认值为 A0 基站坐标)
        self.declare_parameter('fixed_lat', 22.5180977)
        self.declare_parameter('fixed_lon', 113.9007239)
        self.declare_parameter('fixed_alt', -5.17)
        
        self.set_delay_sec = self.get_parameter('set_delay_sec').get_parameter_value().double_value
        self.use_current_gps = self.get_parameter('use_current_gps').get_parameter_value().bool_value
        self.fixed_lat = self.get_parameter('fixed_lat').get_parameter_value().double_value
        self.fixed_lon = self.get_parameter('fixed_lon').get_parameter_value().double_value
        self.fixed_alt = self.get_parameter('fixed_alt').get_parameter_value().double_value
        
        self.home_set_sent = False
        self.first_pose_received = False
        self.system_connected = False

        # 定时器
        self.delay_timer = None

        # 订阅本地位置信息（使用 MAVROS 原生话题检测首次定位）
        # 注意：此节点用于检测 EKF 原点设置，使用 MAVROS 原生话题
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',  # 保持使用 MAVROS 原生话题
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
            msg = GeoPointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            if self.use_current_gps:
                # 使用当前GPS位置作为Home点
                # 注意: 如果使用当前GPS，通常不需要手动设置 EKF 原点，除非需要重置
                # 这里我们假设如果配置为 use_current_gps，则不发送 set_gp_origin 消息
                # 或者需要订阅 global_position/global 来获取当前位置并发送
                self.get_logger().info('Configured to use current GPS. Skipping manual origin set.')
            else:
                # 使用固定坐标作为Home点
                msg.position.latitude = self.fixed_lat
                msg.position.longitude = self.fixed_lon
                msg.position.altitude = self.fixed_alt
                
                self.set_home_pub.publish(msg)
                
                self.get_logger().info(f'Setting Home Position at fixed coordinates: Lat={self.fixed_lat}, Lon={self.fixed_lon}, Alt={self.fixed_alt}')
                self.get_logger().info('Home Position set request sent (using fixed coordinates)')
            
            self.get_logger().info('✅ Home Position setting completed')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to set Home Position: {e}')

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if self.delay_timer:
            self.delay_timer.cancel()
        super().destroy_node()


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
