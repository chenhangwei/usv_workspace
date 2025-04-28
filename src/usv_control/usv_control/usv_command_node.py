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
           String,  # 解锁命令使用 String 类型
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
        self.get_logger().info(f'收到模式消息: {msg}')
        """处理模式切换命令"""
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的模式消息类型')
            return

        # 等待服务可用
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('模式服务不可用')
            return
        
        if msg.data not in ['GUIDED', 'MANUAL', 'AUTO', 'HOLD']:
            self.get_logger().error(f'不支持的模式: {msg.data}')
            return

        # 切换模式
        mode_req = SetMode.Request()
        mode_req.custom_mode = msg.data  # String 类型的消息使用 .data 获取内容
        future = self.mode_client.call_async(mode_req)

        future.add_done_callback(lambda future: self.mode_done_callback(future, msg.data))

    def mode_done_callback(self, future, mode):
        try:
            response = future.result()
            if response and response.mode_sent:
                self.get_logger().info(f'成功切换到模式: {mode}')
            else:
                self.get_logger().warn(f'切换模式 {mode} 失败')
        except Exception as e:
            self.get_logger().error(f'切换模式失败: {e}')


    def set_arming_callback(self, msg):
        self.get_logger().info(f'收到解锁消息: {msg.data}')
        """处理解锁/上锁命令"""
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的解除武装消息类型')
            return

        # 等待服务可用
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('武装服务不可用')
            return

        # 设置解锁或上锁
        arm_req = CommandBool.Request()
        if msg.data == 'ARMING':
            arm_req.value = True
        elif msg.data == 'DISARMING':
            arm_req.value = False       
        future = self.arming_client.call_async(arm_req)
        future.add_done_callback(lambda future: self.arming_done_callback(future, msg.data))
    def arming_done_callback(self, future, command):
        try:
            response = future.result()
            state = 'armed' if command == 'ARMING' else 'disarmed'
            if response and response.success:
                self.get_logger().info(f'Vehicle {state}')
            else:
                self.get_logger().warn(f'设置 {state} 失败')
        except Exception as e:
            self.get_logger().error(f'设置警戒状态失败: {e}')     

def main(args=None):
    rclpy.init(args=args)
    node = UsvCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()