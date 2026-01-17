import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('unicast_tester')
        self.publisher_ = self.create_publisher(String, 'unicast_test_topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Unicast tester started. Domain: {os.environ.get('ROS_DOMAIN_ID')}")

    def timer_callback(self):
        msg = String()
        msg.data = 'Unicast Test Message'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
