
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import serial
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class UsvLedNode(Node):
    def __init__(self):
        super().__init__('usv_led_node')  

                 # 初始化 QoS 策略（根据需要调整）
        self.qos = QoSProfile(depth=10,reliability= QoSReliabilityPolicy.BEST_EFFORT)

        # 声明并获取串口参数 
        self.declare_parameter('port', '/dev/ttyUSB1') # 默认串口
        self.declare_parameter('baudrate', 9600) # 默认波特率 
        port=self.get_parameter('port').get_parameter_value().string_value
        baud=self.get_parameter('baudrate').get_parameter_value().integer_value
        try:
            self.ser=serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'串口 {port} 打开成功')
        except serial.SerialException as e:
            self.get_logger().error(f'打开串口失败: {e}')
            return        
        # 订阅地面站的 LED 控制命令
        self.gs_led_sub = self.create_subscription(String, 'gs_led_command', self.gs_led_callback, self.qos)
  

        self.usv_batterystate_sub = self.create_subscription(BatteryState, 'battery', self.usv_batterystate_callback, self.qos)


        # 创建定时器
        self.timer = self.create_timer(1.0, self.timer_callback)


        self.is_low_battery_level = False  # 是否处于低电量状态
        self.gs_command_str= None  # 地面站的命令字符串 



    #   构建串口指令
    def build_command(self, group_addr, device_addr, port_num, led_type, data, extend_times):
            """
            构建串口指令
            """
            header = bytes.fromhex('DD55EE')
            group_addr_bytes = group_addr.to_bytes(2, 'big')
            device_addr_bytes = device_addr.to_bytes(2, 'big')
            port_num_byte = port_num.to_bytes(1, 'big')
            func_code = bytes.fromhex('99')  #  显示颜色数据
            led_type_byte = led_type.to_bytes(1, 'big')
            reserved = bytes.fromhex('0000')
            data_len = len(data).to_bytes(2, 'big')
            extend_times_bytes = extend_times.to_bytes(2, 'big')
            data_bytes = bytes(data)
            tail = bytes.fromhex('AABB')

            command = header + group_addr_bytes + device_addr_bytes + port_num_byte + \
                    func_code + led_type_byte + reserved + data_len + extend_times_bytes + \
                    data_bytes + tail
            return command


    def gs_led_callback(self, msg):
        if isinstance(msg, String):
            self.gs_command_str = msg.data
            self.get_logger().info(f'收到地面站的 LED 控制命令: {self.gs_command_str}')

    def usv_batterystate_callback(self, msg):
        if isinstance(msg, BatteryState):
            self.usv_state = msg
            if self.usv_state.voltage<=11.1:
                self.is_low_battery_level= True
                # self.get_logger().info('电池电量过低')
            if self.usv_state.voltage>11.1:
                self.is_low_battery_level= False
                # self.get_logger().info('电池电量正常')
    def timer_callback(self):
        if self.is_low_battery_level:
            # 发送低电量指令
            command = self.build_command(0x0001, 0x0001, 0x01, 0x01, [0x00, 0x00, 0x00], 0)
            self.ser.write(command)
            self.get_logger().info('发送低电量指令')
        else:
            if self.gs_command_str is not None:
               match self.gs_command_str:
                    case 'led_yellow':
                        command = self.build_command(0x0001, 0x0001, 0x01, 0x01, [0xFF, 0xFF, 0x00], 0)
                        self.ser.write(command)
                        self.get_logger().info('发送黄色指令')
                    case 'led_green':
                        command = self.build_command(0x0001, 0x0001, 0x01, 0x01, [0x00, 0xFF, 0x00], 0)
                        self.ser.write(command)
                        self.get_logger().info('发送绿色指令')
                    case 'led_blue':
                        command = self.build_command(0x0001, 0x0001, 0x01, 0x01, [0x00, 0x00, 0xFF], 0)
                        self.ser.write(command)
                        self.get_logger().info('发送蓝色指令')
                    case 'led_off':
                        command = self.build_command(0x0001, 0x0001, 0x01, 0x01, [0x00, 0x00, 0x00], 0)
                        self.ser.write(command)
                        self.get_logger().info('发送关闭指令')
                    case _:
                        self.get_logger().error('无效的 LED 控制命令')
        # 接收并处理返回值
        response = self.ser.read_all()
        
        if response:
             self.process_response(response)

    def process_response(self, response):
        pass
        # """
        # 处理控制器返回的数据
        # """
        # self.get_logger().info(f'收到控制器返回: {response.hex()}')
        #  根据协议解析返回值，例如确认信息或错误代码                    

def main(args=None):
    rclpy.init(args=args)
    node = UsvLedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()        

              
