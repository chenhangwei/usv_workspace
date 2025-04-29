import serial 
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
import re
class UsvUwbNode(Node): 
    def __init__(self): 
        super().__init__('usv_uwb_node') 
        self.uwb_pub = self.create_publisher(PoseStamped, 'vision_pose/pose', 10) 
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) # 假设串口为 "/dev/ttyUSB0"，波特率为 57600
        self.timer = self.create_timer(0.02, self.read_and_publish) # 10 Hz 
        
        self.uwb_msg=PoseStamped()

    def read_and_publish(self): 
      
        try:  
            # 读取一行数据
            data = self.serial_port.readline().decode('ASCII').strip()
            if not data:
                self.get_logger().warn("没有接收到数据")
                return
            # self.get_logger().info(f"Raw data: {data}")

            # 查找 LO=[...]
            ma=re.search(r'LO=\[([^]]+)\]',data)

            if ma:
                    coords_str = ma.group(1)  # 提取括号内的内容，例如 "1.41,4.13,0.26" 或 "no solution"
                     # 检查是否为 "no solution"
                    if coords_str == 'no solution':
                        self.get_logger().warn("No valid position solution from $KT0")
                        return
                    try:
                    
                    
                        values=list(map(float,ma.group(1).split(',')))
                        if len(values)==3:
                            x,y,z=values
                           # self.get_logger().info(f'x:{x},y:{y},z:{z}')
                            self.uwb_msg.header.stamp=self.get_clock().now().to_msg()
                            self.uwb_msg.header.frame_id='map'
                            self.uwb_msg.pose.position.x=x
                            self.uwb_msg.pose.position.y=y
                            self.uwb_msg.pose.position.z=z
                            self.uwb_pub.publish(self.uwb_msg)
                        else:
                            pass
                            #self.get_logger().warning(f'括号内数据不符合三个数值的要求')  
                    except ValueError as e:
                            self.get_logger().warning(f'{e}')
                                 
            else:
                 pass
                  #self.get_logger().warning(f'括号内数据不符合三个数值的要求')  
        except Exception as e :
                self.get_logger().warn(f'{e}')
           

    
def main(): 
        rclpy.init() 
        node = UsvUwbNode() 
        rclpy.spin(node) 
        node.destroy_node() 
        rclpy.shutdown() 
        
if __name__ == '__main__': 
     main()