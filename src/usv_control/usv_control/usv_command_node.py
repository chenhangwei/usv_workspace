import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from mavros_msgs.srv import CommandBool, SetMode

class UsvCommandNode(Node):
    def __init__(self):
        super().__init__('usv_command_node')

        # 订阅地面站的模式切换命令
        self.sub_mode = self.create_subscription(
            String,  # 模式命令使用 String 类型
            'usv_set_mode',  # 话题名称，添加前缀斜杠
            self.set_mode_callback,
            10
        )

        # 订阅地面站的解锁命令
        self.sub_arming = self.create_subscription(
            Bool,  # 解锁命令使用 Bool 类型
            'usv_set_arming',  # 话题名称
            self.set_arming_callback,
            10
        )

        # 创建服务客户端
        self.arming_client = self.create_client(
            CommandBool,
            'mavros/cmd/arming'  # MAVROS 默认服务名称
        )

        self.mode_client = self.create_client(
            SetMode,
            'mavros/set_mode'  # MAVROS 默认服务名称
        )

    def set_mode_callback(self, msg):
        
        """处理模式切换命令"""
        if not isinstance(msg, String):
            self.get_logger().error('Received invalid mode message type')
            return

        # 等待服务可用
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Mode service not available')
            return

        # 切换模式
        mode_req = SetMode.Request()
        mode_req.custom_mode = msg.data  # String 类型的消息使用 .data 获取内容
        future = self.mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)  # 等待服务响应
        if future.result() is not None:
            self.get_logger().info(f'Switched to mode: {msg.data}')
        else:
            self.get_logger().error('Failed to switch mode')

    def set_arming_callback(self, msg):
        
        """处理解锁/上锁命令"""
        if not isinstance(msg, Bool):
            self.get_logger().error('Received invalid arming message type')
            return

        # 等待服务可用
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service not available')
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
            self.get_logger().error('Failed to set arming state')

def main(args=None):
    rclpy.init(args=args)
    node = UsvCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()