import serial 
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
class UsvUwbNode(Node): 
    def __init__(self): 
        super().__init__('usv_uwb_node') 
        self.uwb_pub = self.create_publisher(PoseStamped, 'local_position/pose', 10) 
    #     self.serial_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=1) # 假设串口为 "/dev/ttyUSB0"，波特率为 57600
        self.timer = self.create_timer(1, self.read_and_publish) # 10 Hz 


        self.ax=1.0
        self.by=2.0
    def read_and_publish(self): 
            try: 
                # line = self.serial_port.readline().decode('utf-8').strip() # 假设 UWB 输出为 "x,y,z"（单位：米） 
                # x, y, z = map(float, line.split(',')) 

                x, y, z = self.ax, self.by, 0.0 # 示例数据
                pose = PoseStamped() 
                pose.header.stamp = self.get_clock().now().to_msg() 
                pose.header.frame_id = 'map' 
                pose.pose.position.x = x 
                pose.pose.position.y = y 
                pose.pose.position.z = z 
                pose.pose.orientation.w = 1.0 # 默认朝向 
                self.uwb_pub.publish(pose) 
                self.get_logger().info(f'Published: x={x}, y={y}, z={z}') 
                self.ax+=0.1
                self.by+=0.1



            except Exception as e: 
                self.get_logger().error(f'Error: {e}') 
    
def main(): 
        rclpy.init() 
        node = UsvUwbNode() 
        rclpy.spin(node) 
        node.destroy_node() 
        rclpy.shutdown() 
        
if __name__ == '__main__': 
     main()