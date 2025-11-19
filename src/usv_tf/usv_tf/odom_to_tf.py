import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from common_utils import ParamLoader

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        qos = QoSProfile(
            depth=10,  # 队列大小
            reliability=QoSReliabilityPolicy.BEST_EFFORT # 可靠性策略
        )
        self.tf_broadcaster=TransformBroadcaster(self)
        
        # 使用ParamLoader统一加载参数
        loader = ParamLoader(self)
        ns = loader.load_param('namespace', 'usv_01')
        self.base_link_frame = f'base_link_{ns}'
        
        # [OK] 使用相对路径订阅（节点已在命名空间中启动）
        # 相对路径会自动添加节点的命名空间前缀
        self.subscription_=self.create_subscription(
            Odometry,
            'global_position/local',  # 相对路径（推荐）
            self.odom_callback,
            qos
        )

    def odom_callback(self,msg):
        if not isinstance(msg,Odometry):
            return
        transform=TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id  # "map"
        transform.child_frame_id = self.base_link_frame  # 例如 "base_link_usv_01"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()