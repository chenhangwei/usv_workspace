import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster # 静态坐标发布器
from geometry_msgs.msg import TransformStamped # 消息接口
from tf_transformations import quaternion_from_euler # 欧拉角转四元数函数
import math # 角度转弧度函数

class StaticTfLaserNode(Node):
    def __init__(self):
        super().__init__('static_tf_laser_node')
        self.static_laser_=StaticTransformBroadcaster(self)
        self.declare_parameter('namespace', 'usv_01')
        self.ns = self.get_parameter('namespace').get_parameter_value().string_value

        self.publish_static_tf()

    def publish_static_tf(self):
        """
        发布静态TF 从base_link到laser_frame
        """
        transform=TransformStamped()
        transform.header.stamp=self.get_clock().now().to_msg()
        transform.header.frame_id=f'base_link_{self.ns}'
        transform.child_frame_id=f'laser_frame_{self.ns}'
        transform.transform.translation.x=0.1
        transform.transform.translation.y=0.0
        transform.transform.translation.z=0.3

        #欧拉角转四元数
        q=quaternion_from_euler(math.radians(180),0,0)
        transform.transform.rotation.x=q[0]
        transform.transform.rotation.y=q[1]
        transform.transform.rotation.z=q[2]
        transform.transform.rotation.w=q[3]

        self.static_laser_.sendTransform(transform)
        self.get_logger().info(f'发布静态TF:{transform}')


def main(args=None):
    rclpy.init(args=args)
    node =StaticTfLaserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()       

        
    
