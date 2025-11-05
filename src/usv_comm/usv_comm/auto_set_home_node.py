"""
自动设置Home点节点

该节点用于自动设置无人船的EKF原点。当无人船启动并获取到第一个位置信息后，
经过指定延迟时间后自动设置EKF原点，确保系统定位正确初始化。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandLong
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
        self.declare_parameter('use_current_gps', False)
        
        # ⚠️ 重要：以下坐标应该是定位基站A0的GPS原点坐标，不是USV上电位置！
        # 这个Home点将作为所有USV共享的全局坐标系原点
        # 所有USV应该使用相同的Home点坐标（定位基站A0的位置）
        self.declare_parameter('home_latitude', 22.5180977)   # A0基站纬度
        self.declare_parameter('home_longitude', 113.9007239)  # A0基站经度
        self.declare_parameter('home_altitude', -4.8)          # A0基站海拔高度（米）
        
        self.declare_parameter('retry_interval', 1.0)
        self.declare_parameter('max_retries', 30)
        self.set_delay_sec = self.get_parameter('set_delay_sec').get_parameter_value().double_value
        self.use_current_gps = self.get_parameter('use_current_gps').get_parameter_value().bool_value
        self.home_latitude = self.get_parameter('home_latitude').get_parameter_value().double_value
        self.home_longitude = self.get_parameter('home_longitude').get_parameter_value().double_value
        self.home_altitude = self.get_parameter('home_altitude').get_parameter_value().double_value
        self.retry_interval = self.get_parameter('retry_interval').get_parameter_value().double_value
        self.max_retries = int(self.get_parameter('max_retries').get_parameter_value().integer_value)
        self.set_home_sent = False

        # 重试相关状态变量
        self.retry_attempt = 0
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

        # 创建 MAV_CMD_DO_SET_HOME 服务客户端（通过 COMMAND_LONG 发送）
        self.set_home_cli = self.create_client(CommandLong, 'cmd/command')

        # 日志输出Home点设置模式
        if self.use_current_gps:
            home_mode = 'current GPS position (当前GPS位置)'
            self.get_logger().warning(
                '⚠️ 使用当前GPS位置作为Home点！'
                '如果你的系统使用定位基站A0，请将 use_current_gps 设为 false'
            )
        else:
            home_mode = (
                f'fixed coordinates (定位基站A0): '
                f'({self.home_latitude:.7f}, {self.home_longitude:.7f}, {self.home_altitude:.2f}m)'
            )
        
        self.get_logger().info(
            f'AutoSetHomeNode initialized with {self.set_delay_sec}s delay, '
            f'target origin: {home_mode}. Waiting for GPS fix...'
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
        # ArduPilot 要求至少有 GPS 定位（>= 1）才能设置 EKF 原点
        if self.gps_fix_type < 1:
            self.get_logger().info(
                f'⏳ Waiting for GPS fix (current type: {self.gps_fix_type}, need >= 1)...'
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
        self.retry_attempt = 0
        self._try_set_home()
    
    def _try_set_home(self):
        """尝试设置EKF原点,使用非阻塞定时器实现重试"""
        if self.retry_attempt >= self.max_retries:
            self.get_logger().error(
                f'❌ Failed to set EKF origin after {self.max_retries} attempts. Giving up.'
            )
            return

        self.retry_attempt += 1
        attempt = self.retry_attempt

        # 检查服务是否就绪
        if self.set_home_cli.service_is_ready():
            self.get_logger().info(
                f'Service /mavros/cmd/command is ready (attempt {attempt}/{self.max_retries})'
            )
            self._send_set_home_request()
            return
        
        self._schedule_retry(
            f'Service /mavros/cmd/command not ready, will retry (attempt {attempt}/{self.max_retries})'
        )
    
    def _retry_callback(self):
        """重试定时器回调"""
        self._clear_retry_timer()
        self._try_set_home()
    
    def _send_set_home_request(self):
        """发送设置Home点的服务请求"""
        # 创建服务请求 (MAV_CMD_DO_SET_HOME = 179)
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 179
        req.confirmation = 0
        req.param1 = 1.0 if self.use_current_gps else 0.0
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = float(self.home_latitude)
        req.param6 = float(self.home_longitude)
        req.param7 = float(self.home_altitude)

        # 异步调用服务
        future = self.set_home_cli.call_async(req)
        future.add_done_callback(self._handle_set_home_response)
    
    def _handle_set_home_response(self, future):
        """处理设置Home点的服务响应"""
        try:
            result = future.result()
            if result is not None and getattr(result, 'success', False):
                self._clear_retry_timer()
                self.get_logger().info('✅ EKF origin set successfully via MAV_CMD_DO_SET_HOME!')
            else:
                result_code = getattr(result, 'result', None)
                if self.retry_attempt >= self.max_retries:
                    self.get_logger().error(
                        f'❌ Failed to set EKF origin after {self.retry_attempt} attempts '
                        f'(last result={result_code}).'
                    )
                    return

                self._schedule_retry(
                    f'❌ Failed to set EKF origin (success={getattr(result, "success", None)}, '
                    f'result={result_code})'
                )
        except Exception as e:
            if self.retry_attempt >= self.max_retries:
                self.get_logger().error(f'Exception occurred while setting EKF origin: {e}')
                return
            self._schedule_retry(f'Exception occurred while setting EKF origin: {e}')

    def _schedule_retry(self, reason: str):
        """安排下一次重试"""
        if self.retry_attempt >= self.max_retries:
            self.get_logger().error(
                f'{reason}. Reached retry limit ({self.max_retries}).'
            )
            return

        self._clear_retry_timer()
        self.get_logger().warn(
            f'{reason}. Retrying in {self.retry_interval:.1f}s '
            f'(next attempt {self.retry_attempt + 1}/{self.max_retries}).'
        )
        self.retry_timer = self.create_timer(self.retry_interval, self._retry_callback)

    def _clear_retry_timer(self):
        """清理已存在的重试定时器"""
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None


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