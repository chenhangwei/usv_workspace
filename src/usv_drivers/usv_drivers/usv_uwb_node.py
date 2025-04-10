import serial 
import rclpy 
import random
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
class UsvUwbNode(Node): 
    def __init__(self): 
        super().__init__('usv_uwb_node') 
        self.uwb_pub = self.create_publisher(PoseStamped, 'local_position/pose', 10) 
    #     self.serial_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=1) # 假设串口为 "/dev/ttyUSB0"，波特率为 57600
        self.timer = self.create_timer(0.1, self.read_and_publish) # 10 Hz 


        self.ax=1.0
        self.by=2.0
        self.cz=3.0
    def read_and_publish(self): 
            try: 
                # line = self.serial_port.readline().decode('utf-8').strip() # 假设 UWB 输出为 "x,y,z"（单位：米） 
                # x, y, z = map(float, line.split(',')) 
                random_int = random.randint(0,10)
                randint=random_int/100


                x, y, z = self.ax, self.by,self.cz # 示例数据
                pose = PoseStamped() 
                pose.header.stamp = self.get_clock().now().to_msg() 
                pose.header.frame_id = 'base_link' 
                pose.pose.position.x = x 
                pose.pose.position.y = y 
                pose.pose.position.z = z 
                pose.pose.orientation.w = 1.0 # 默认朝向 
                self.uwb_pub.publish(pose) 

                self.ax+=(0.1+randint)
                self.by+=(0.1+randint)
                self.cz+=(0.1+randint)



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