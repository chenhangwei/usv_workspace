import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import serial
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time
import random

class UsvLedNode(Node):
    def __init__(self):
        super().__init__('usv_led_node')  

                 # 初始化 QoS 策略（根据需要调整）
        self.qos = QoSProfile(depth=10,reliability= QoSReliabilityPolicy.BEST_EFFORT)

        # 声明并获取串口参数 
        self.declare_parameter('port', '/dev/ttyUSB0') # 默认串口
        self.declare_parameter('baudrate',115200) # 默认波特率 
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
        self.timer_period = 1.0   # 当前定时器周期
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.usv_state = BatteryState()  # 初始化 USV 电池状态
        self.is_low_battery_level = False  # 是否处于低电量状态
        self.gs_command_str= None  # 地面站的命令字符串 

        self.breath_index = 0
        self.breath_up = True
        self.breath_values = self.generate_breath_values()
        self.last_breath_time = time.time()


        self.mode = None
        self.color_list = [
            [255, 0, 0],    # 红
            [255, 127, 0],  # 橙
            [255, 255, 0],  # 黄
            [0, 255, 0],    # 绿
            [0, 255, 255],  # 青
            [0, 0, 255],    # 蓝
            [139, 0, 255],  # 紫
        ]
        self.color_index = 0
        self.current_color = [255, 0, 0]
        self.target_color = [255, 0, 0]
        self.last_switch_time = time.time()
        self.in_transition = False
        self.transition_start_color = [255, 0, 0]
        self.transition_duration = 3.0  # 渐变3秒
        self.hold_duration = 10.0       # 停留10秒

        self.last_sent_rgb = None  # 缓存上一次发送的RGB，避免重复写串口
        self.last_mode_before_low_battery = None  # 低电压前的模式
        self.timer_period_high = 0.03  # 高频定时器周期
        self.timer_period_low = 0.5    # 低频定时器周期
        self.current_timer_period = self.timer_period_low
        self.timer = self.create_timer(self.current_timer_period, self.timer_callback)


    #   构建串口指令
    def build_command(self, rgb_data, extend_times=60):
        """
        构建灯带控制串口指令，格式严格按照
        DD 55 EE 00 00 00 01 00 99 01 00 00 00 [数据长度2字节] [扩展次数2字节] [RGB数组] AA BB
        :param rgb_data: list 或 bytes，RGB数据（如 [R, G, B] 或 [R1, G1, B1, R2, G2, B2, ...]）
        :param extend_times: int，扩展次数（2字节）
        """
        header = bytes.fromhex('DD55EE')
        fixed1 = bytes.fromhex('000000010099')
        port = bytes.fromhex('01')
        fixed2 = bytes.fromhex('00000003')
        # 数据长度（2字节，大端）
        data_len = len(rgb_data).to_bytes(2, 'big')
        # 扩展次数（2字节，大端）
        extend_bytes = extend_times.to_bytes(2, 'big')
        data_bytes = bytes(rgb_data)
        tail = bytes.fromhex('AABB')
        command = header + fixed1 + port + fixed2 + data_len + extend_bytes + data_bytes + tail
        return command


    def gs_led_callback(self, msg):
        if isinstance(msg, String):
            if msg.data.startswith('color_select'):
                # color_select|r,g,b
                parts = msg.data.split('|')
                if len(parts) == 2:
                    rgb = [int(x) for x in parts[1].split(',')]
                    self.mode = 'color_select'
                    self.current_color = rgb
                    self.target_color = rgb
                    self.in_transition = False
            else:
                self.mode = msg.data
                self.last_switch_time = time.time()
                self.color_index = 0
                self.current_color = self.color_list[0]
                self.target_color = self.color_list[0]
                self.in_transition = False


    def usv_batterystate_callback(self, msg):
        if isinstance(msg, BatteryState):
            self.usv_state = msg
            if self.usv_state.voltage < 11.1:
                if not self.is_low_battery_level:
                    self.last_mode_before_low_battery = self.mode
                    self.mode = 'low_battery_breath'
                    self.get_logger().warn('USV 电池电压低于 11.1V，进入低电量呼吸灯状态')
                self.is_low_battery_level = True
            else:
                if self.is_low_battery_level:
                    # 恢复原有模式
                    self.mode = self.last_mode_before_low_battery or 'color_switching'
                    self.get_logger().info('USV 电池电压恢复，退出低电量呼吸灯')
                self.is_low_battery_level = False

    def generate_breath_values(self):
        # 生成红色呼吸亮度序列（0~255~0，步进可调）
        up = list(range(0, 256, 6))
        down = list(range(255, -1, -6))
        return up + down[1:]  # 去掉重复的255
   
    def set_timer_period(self, period):
        # 动态调整定时器周期，避免频繁重建定时器
        if abs(period - self.current_timer_period) > 1e-3:
            self.timer.cancel()
            self.timer = self.create_timer(period, self.timer_callback)
            self.current_timer_period = period

    def timer_callback(self):
        now = time.time()
        rgb_to_send = None
        # 低电压呼吸灯，强制高频
        if self.mode == 'low_battery_breath':
            self.set_timer_period(self.timer_period_high)
            voltage = self.usv_state.voltage if hasattr(self.usv_state, 'voltage') else 12.0
            if voltage < 11.1:
                voltage = 10.6
            base_interval = 0.1
            min_interval = 0.03
            delta_v = max(0, (11.1 - voltage))
            interval = max(min_interval, base_interval - int(delta_v * 10) * 0.015)
            if now - self.last_breath_time >= interval:
                r = self.breath_values[self.breath_index % len(self.breath_values)]
                g = 0
                b = 0
                rgb_to_send = [r, g, b]
                self.breath_index = (self.breath_index + 1) % len(self.breath_values)
                self.last_breath_time = now
        elif self.mode == 'color_switching':
            self.set_timer_period(self.timer_period_high)
            if not self.in_transition:
                if now - self.last_switch_time >= self.hold_duration:
                    self.in_transition = True
                    self.transition_start_color = self.current_color[:]
                    self.color_index = (self.color_index + 1) % len(self.color_list)
                    self.target_color = self.color_list[self.color_index]
                    self.last_switch_time = now
            else:
                t = min(1.0, (now - self.last_switch_time) / self.transition_duration)
                self.current_color = [
                    int(self.transition_start_color[i] + (self.target_color[i] - self.transition_start_color[i]) * t)
                    for i in range(3)
                ]
                if t >= 1.0:
                    self.in_transition = False
                    self.last_switch_time = now
            rgb_to_send = self.current_color
        elif self.mode == 'random_color_change':
            self.set_timer_period(self.timer_period_high)
            if not self.in_transition:
                if now - self.last_switch_time >= self.hold_duration:
                    self.in_transition = True
                    self.transition_start_color = self.current_color[:]
                    self.target_color = [random.randint(0, 255) for _ in range(3)]
                    self.last_switch_time = now
            else:
                t = min(1.0, (now - self.last_switch_time) / self.transition_duration)
                self.current_color = [
                    int(self.transition_start_color[i] + (self.target_color[i] - self.transition_start_color[i]) * t)
                    for i in range(3)
                ]
                if t >= 1.0:
                    self.in_transition = False
                    self.last_switch_time = now
            rgb_to_send = self.current_color
        elif self.mode == 'color_select':
            self.set_timer_period(self.timer_period_low)
            rgb_to_send = self.current_color
        elif self.mode == 'led_off':
            self.set_timer_period(self.timer_period_low)
            rgb_to_send = [0, 0, 0]
        else:
            self.set_timer_period(self.timer_period_low)
            rgb_to_send = [0, 0, 0]
        # 只有RGB变化时才写串口
        if rgb_to_send is not None and (self.last_sent_rgb != rgb_to_send):
            command = self.build_command(rgb_to_send, 60)
            self.ser.write(command)
            self.last_sent_rgb = rgb_to_send[:]
              
def main(args=None):
    rclpy.init(args=args)
    node = UsvLedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()


