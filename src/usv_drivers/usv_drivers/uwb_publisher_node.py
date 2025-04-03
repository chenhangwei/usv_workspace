import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class UwbPublisherNode(Node):
    def __init__(self):
        super().__init__('uwb_publisher_node')



        self.uwb_publisher=self.create_publisher(PoseStamped,f'mavros/local_position/pose',10)

        self.uwb_timer=self.create_timer(0.1,self.publish_uwb_pose)
        
        self.x=1.0
        self.y=1.0
        self.z=0.0

    def publish_uwb_pose(self):
        uwb_pose=PoseStamped()
        uwb_pose.header.stamp=self.get_clock().now().to_msg()
        uwb_pose.header.frame_id="base_link"
        uwb_pose.pose.position.x=self.x
        uwb_pose.pose.position.y=self.y
        uwb_pose.pose.position.z=self.z
        uwb_pose.pose.orientation.x=0.0
        uwb_pose.pose.orientation.y=0.0
        uwb_pose.pose.orientation.z=0.0
        uwb_pose.pose.orientation.w=1.0

        self.uwb_publisher.publish(uwb_pose)
        self.get_logger().info(f'发布到{self.uwb_publisher.topic}:x={self.x},y={self.y},z={self.z}')

def main(args=None):
    rclpy.init(args=args)    
    node=UwbPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__' :
    main()   