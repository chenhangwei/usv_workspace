import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class UsvCommandNode(Node):
    def __init__(self):
        super().__init__('usv_command_node')

         # 初始化 QoS 策略（根据需要调整）
        self.qos_a = QoSProfile(depth=10,reliability= QoSReliabilityPolicy.RELIABLE)

        # 订阅地面站的模式切换命令
        self.sub_mode = self.create_subscription(
            String,  # 模式命令使用 String 类型
            'set_usv_mode',  # 话题名称，添加前缀斜杠
            self.set_mode_callback,
            self.qos_a
        )

        # 订阅地面站的解锁命令
        self.sub_arming = self.create_subscription(
            Bool,  # 解锁命令使用 Bool 类型
            'set_usv_arming',  # 话题名称
            self.set_arming_callback,
            self.qos_a
        )

        # 创建服务客户端
        self.arming_client = self.create_client(
            CommandBool,
            'cmd/arming'  # MAVROS 默认服务名称
        )

        self.mode_client = self.create_client(
            SetMode,
            'set_mode'  # MAVROS 默认服务名称
        )

    def set_mode_callback(self, msg):
        
        """处理模式切换命令"""
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的模式消息类型')
            return

        # 等待服务可用
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('模式服务不可用')
            return

        # 切换模式
        mode_req = SetMode.Request()
        mode_req.custom_mode = msg.data  # String 类型的消息使用 .data 获取内容
        future = self.mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)  # 等待服务响应
        if future.result() is not None:
            self.get_logger().info(f'切换到模式：: {msg.data}')
        else:
            self.get_logger().error('切换模式失败')

    def set_arming_callback(self, msg):
        
        """处理解锁/上锁命令"""
        if not isinstance(msg, Bool):
            self.get_logger().error('收到无效的解除武装消息类型')
            return

        # 等待服务可用
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('武装服务不可用')
            return

        # 设置解锁或上锁
        arm_req = CommandBool.Request()
        arm_req.value = msg.data  # Bool 类型的消息使用 .data 获取内容
        future = self.arming_client.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)  # 等待服务响应
        if future.result() is not None:
            state = 'armed' if msg.data else 'disarmed'
            self.get_logger().info(f'Vehicle {state}')
        else:
            self.get_logger().error('设置警戒状态失败')

def main(args=None):
    rclpy.init(args=args)
    node = UsvCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()