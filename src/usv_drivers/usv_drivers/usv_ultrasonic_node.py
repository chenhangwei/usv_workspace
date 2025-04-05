import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import serial
import time

class UsvUltrasonicNode(Node):
    def __init__(self):
        super().__init__('usv_ultrasonic_node')
        
        # 创建 LaserScan 消息发布者，发布到 'ultrasonic_scan' 话题，队列大小为 10
        self.scan_pub = self.create_publisher(LaserScan, 'radar_scan111', 10)
        
        # 串口参数（根据实际超声雷达设备调整）
        self.serial_port = '/dev/ttyUSB0'  # 串口设备路径
        self.baud_rate = 115200            # 波特率
        self.timeout = 1                   # 超时时间（秒）
        
        # # 尝试打开串口
        # try:
        #     self.ser = serial.Serial(
        #         port=self.serial_port,
        #         baudrate=self.baud_rate,
        #         timeout=self.timeout
        #     )
        #     self.get_logger().info(f'串口 {self.serial_port} 打开成功')
        # except serial.SerialException as e:
        #     self.get_logger().error(f'打开串口失败: {e}')
        #     raise
        
        # # 定向超声雷达参数设置
        # # 假设超声雷达的视场角为15度，即 -7.5° ~ 7.5°（转换为弧度）
        # beam_width_deg = 15.0
        # beam_width_rad = beam_width_deg * 3.14159 / 180.0
        # self.angle_min = -beam_width_rad / 2   # 最小角度
        # self.angle_max = beam_width_rad / 2    # 最大角度
        # self.angle_increment = 0.0             # 只有一个测量值，无角度间隔
        # self.range_min = 0.05                  # 最小距离（米），根据实际传感器调整
        # self.range_max = 4.0                   # 最大距离（米），根据实际传感器调整
        # self.num_readings = 1                  # 定向超声雷达只有一个测量点
        
        # # 定时器，10Hz 定时发布超声雷达数据（每 0.1 秒调用一次）
        # self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        """读取串口数据并发布 LaserScan 消息"""
        try:
            # 调用读取超声雷达数据函数，返回列表（包含一个距离值）
            ranges = self.read_ultrasonic_frame()
            if ranges is None:
                return

            # 创建 LaserScan 消息
            scan = LaserScan()
            scan.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='ultrasonic_radar_link')
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_increment
            scan.range_min = self.range_min
            scan.range_max = self.range_max
            scan.ranges = ranges  # 距离数组，只有一个测量值

            # 对于单个测量点，time_increment 可设置为 scan_time
            scan.scan_time = 0.1  # 一帧总时间（秒）
            scan.time_increment = scan.scan_time

            # 发布消息
            self.scan_pub.publish(scan)
            self.get_logger().info('已发布超声雷达扫描数据')
        except Exception as e:
            self.get_logger().error(f'发布超声雷达数据出错: {e}')

    def read_ultrasonic_frame(self):
        """
        从串口读取一帧定向超声雷达数据
        假设传感器每行只输出一个距离值，单位为米
        """
        try:
            # 清空输入缓冲区，避免读取到旧数据
            self.ser.reset_input_buffer()
            # 读取一行数据
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                # 将读取的字符串转换为浮点数
                distance = float(line)
                # 判断距离是否在有效范围内
                if self.range_min <= distance <= self.range_max:
                    return [distance]
                else:
                    return [float('inf')]
            else:
                return None
        except Exception as e:
            self.get_logger().error(f'读取超声雷达数据出错: {e}')
            return None

    def __del__(self):
        """关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('串口已关闭')

def main(args=None):
    rclpy.init(args=args)
    node = UsvUltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
