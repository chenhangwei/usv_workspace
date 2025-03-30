import rclpy
from rclpy.node import Node

from std_msgs.msg import String
class UsvControlNode(Node):

    def __init__(self):
        super().__init__('usv_control_node')
        self.publisher_ = self.create_publisher(String, 'usv_control_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
def main(args=None):
    rclpy.init(args=args)
    usv_control_node = UsvControlNode()
    rclpy.spin(usv_control_node)
    usv_control_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()        