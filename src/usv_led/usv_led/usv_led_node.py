"""
无人船LED控制节点

该节点负责控制无人船的LED灯带，支持多种灯光效果。
包括颜色切换、随机颜色变化、低电量呼吸灯等模式。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import serial
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time
import random
import math


class UsvLedNode(Node):
    """
    无人船LED控制节点类
    
    该节点实现LED灯带的控制功能，支持多种灯光效果模式。
    通过订阅地面站命令和电池状态，自动调整LED显示效果。
    """

    def __init__(self):
        """初始化无人船LED控制节点"""
        super().__init__('usv_led_node')

        # 初始化 QoS 策略
        self.qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 声明并获取串口参数 
        self.declare_parameter('port', '/dev/ttyUSB1')  # 默认串口
        self.declare_parameter('baudrate', 115200)      # 默认波特率 
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # 声明并获取LED效果参数
        self.declare_parameter('color_list', [255, 0, 0, 255, 127, 0, 255, 255, 0, 0, 255, 0, 0, 255, 255, 0, 0, 255, 139, 0, 255])
        self.declare_parameter('timer_period_high', 0.03)  # 高频定时器周期
        self.declare_parameter('timer_period_low', 0.5)    # 低频定时器周期
        self.declare_parameter('transition_duration', 3.0)  # 渐变持续时间（秒）
        self.declare_parameter('hold_duration', 10.0)       # 颜色停留时间（秒）
        self.declare_parameter('low_battery_threshold', 11.1)  # 低电量阈值（伏特）
        self.declare_parameter('recover_battery_threshold', 11.6)  # 恢复电量阈值（伏特）
        self.declare_parameter('breath_step', 6)  # 呼吸灯步进值
        
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'串口 {port} 打开成功')
        except serial.SerialException as e:
            self.get_logger().error(f'打开串口失败: {e}')
            self.ser = None
            return        
        
        # 订阅地面站的 LED 控制命令
        self.gs_led_sub = self.create_subscription(
            String, 
            'gs_led_command', 
            self.gs_led_callback, 
            self.qos
        )

        self.usv_batterystate_sub = self.create_subscription(
            BatteryState, 
            'battery', 
            self.usv_batterystate_callback, 
            self.qos
        )

        # 创建定时器
        self.timer_period = 1.0   # 当前定时器周期
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.usv_state = BatteryState()  # 初始化 USV 电池状态
        self.is_low_battery_level = False  # 是否处于低电量状态
        self.gs_command_str = None  # 地面站的命令字符串 

        self.breath_index = 0
        self.breath_up = True
        self.breath_values = self.generate_breath_values()
        self.last_breath_time = time.time()

        self.mode = None
        
        # 获取颜色列表参数并转换为二维数组
        color_list_param = self.get_parameter('color_list').get_parameter_value().integer_array_value
        self.color_list = []
        for i in range(0, len(color_list_param), 3):
            if i + 2 < len(color_list_param):
                self.color_list.append([color_list_param[i], color_list_param[i+1], color_list_param[i+2]])
        
        self.color_index = 0
        self.current_color = [255, 0, 0]
        self.target_color = [255, 0, 0]
        self.last_switch_time = time.time()
        self.in_transition = False
        self.transition_start_color = [255, 0, 0]
        
        # 获取定时器和过渡参数
        self.transition_duration = self.get_parameter('transition_duration').get_parameter_value().double_value
        self.hold_duration = self.get_parameter('hold_duration').get_parameter_value().double_value
        self.timer_period_high = self.get_parameter('timer_period_high').get_parameter_value().double_value
        self.timer_period_low = self.get_parameter('timer_period_low').get_parameter_value().double_value

        self.last_sent_rgb = None  # 缓存上一次发送的RGB，避免重复写串口
        self.last_mode_before_low_battery = None  # 低电压前的模式
        self.current_timer_period = self.timer_period_low
        
        # 重新创建定时器
        self.timer.cancel()
        self.timer = self.create_timer(self.current_timer_period, self.timer_callback)

        self.get_logger().info('USV LED 节点启动完成')
        self.get_logger().info(f'支持的颜色数量: {len(self.color_list)}')
        self.get_logger().info(f'高频定时器周期: {self.timer_period_high}s')
        self.get_logger().info(f'低频定时器周期: {self.timer_period_low}s')

    def build_command(self, rgb_data, extend_times=60):
        """
        构建灯带控制串口指令，严格按照：
        DD 55 EE 00 00 00 01 00 99 01 00 00 [数据长度2字节大端] [扩展次数2字节大端] [RBG数据] AA BB
        
        Args:
            rgb_data: list 或 bytes，RBG数据（如 [R, B, G] 或 [R1, B1, G1, ...]）
            extend_times: int，扩展次数（2字节，大端）
            
        Returns:
            bytes: 构建好的串口指令
        """
        try:
            header = bytes.fromhex('DD55EE')  # 帧头
            group_addr = bytes.fromhex('0000')  # 组地址（全0表示单个设备）
            device_addr = bytes.fromhex('0001')  # 设备地址（全1表示广播）
            port = bytes.fromhex('00')  # 端口号（0x00表示默认端口）
            func = bytes.fromhex('99')  # 功能码（0x99表示LED控制）
            led_type = bytes.fromhex('01')  # LED类型（0x01表示普通LED）
            reserved = bytes.fromhex('0000')  # 保留字节（2字节，填充为0）
            
            # 数据长度（2字节，大端）
            data_len = len(rgb_data).to_bytes(2, 'big')
            # 扩展次数（2字节，大端）
            extend_bytes = extend_times.to_bytes(2, 'big')
            
            # 直接使用RBG顺序
            rbg_data = []
            # 支持多组颜色
            for i in range(0, len(rgb_data), 3):
                if i + 2 < len(rgb_data):  # 确保有足够的数据
                    r = rgb_data[i]
                    g = rgb_data[i+1]
                    b = rgb_data[i+2]
                    rbg_data.extend([r, b, g])
                    
            rgb_bytes = bytes(rbg_data)
            tail = bytes.fromhex('AABB')
            command = (header + group_addr + device_addr + port + func + led_type + reserved +
                       data_len + extend_bytes + rgb_bytes + tail)
            return command
        except Exception as e:
            self.get_logger().error(f'构建串口指令时发生错误: {e}')
            return bytes()

    def gs_led_callback(self, msg):
        """
        地面站LED命令回调函数
        
        Args:
            msg (String): 包含LED控制命令的消息
        """
        try:
            if not isinstance(msg, String):
                self.get_logger().warn('收到无效的LED命令类型')
                return
                
            # 低电压优先，禁止切换其他模式
            if self.is_low_battery_level:
                self.get_logger().info('低电压状态下，忽略LED模式切换指令')
                return
                
            allowed_modes = [
                'color_switching',
                'random_color_change',
                'led_off',
                'color_infect',
            ]
            
            if msg.data.startswith('color_select'):
                # color_select|r,g,b
                parts = msg.data.split('|')
                if len(parts) == 2:
                    rgb = [int(x) for x in parts[1].split(',')]
                    self.mode = 'color_select'
                    self.current_color = rgb
                    self.target_color = rgb
                    self.in_transition = False
            elif msg.data.startswith('color_infect'):
                # color_infect|r,g,b
                parts = msg.data.split('|')
                if len(parts) == 2:
                    rgb = [int(x) for x in parts[1].split(',')]
                    # 记录原有模式和颜色，便于恢复
                    if not hasattr(self, '_infect_backup'):
                        self._infect_backup = {
                            'mode': self.mode,
                            'current_color': self.current_color[:],
                            'target_color': self.target_color[:],
                            'in_transition': self.in_transition
                        }
                    self.mode = 'color_infect'
                    self.target_color = rgb
                    self.in_transition = True
                    self.transition_start_color = self.current_color[:]
                    self.transition_duration = 2.0  # 传染渐变2秒
                    self.last_switch_time = time.time()
            elif msg.data in allowed_modes:
                # 其它允许模式
                # 若之前处于传染，恢复备份
                if hasattr(self, '_infect_backup'):
                    self.mode = self._infect_backup['mode']
                    self.current_color = self._infect_backup['current_color'][:]
                    self.target_color = self._infect_backup['target_color'][:]
                    self.in_transition = self._infect_backup['in_transition']
                    del self._infect_backup
                else:
                    self.mode = msg.data
                    self.last_switch_time = time.time()
                    self.color_index = 0
                    self.current_color = self.color_list[0]
                    self.target_color = self.color_list[0]
                    self.in_transition = False
            else:
                # 其它无关消息内容，忽略，不影响LED
                self.get_logger().info(f'忽略无关LED指令: {msg.data}')
                return
        except Exception as e:
            self.get_logger().error(f'处理LED命令时发生错误: {e}')

    def usv_batterystate_callback(self, msg):
        """
        电池状态回调函数
        
        Args:
            msg (BatteryState): 包含电池状态信息的消息
        """
        try:
            if isinstance(msg, BatteryState):
                self.usv_state = msg
                voltage = getattr(self.usv_state, 'voltage', None)
                # self.get_logger().info(f"收到电池电压: {voltage}")
                low_threshold = self.get_parameter('low_battery_threshold').get_parameter_value().double_value
                recover_threshold = self.get_parameter('recover_battery_threshold').get_parameter_value().double_value
                
                # 过滤无效电压
                if voltage is None or voltage <= 0 or (isinstance(voltage, float) and math.isnan(voltage)):
                    # self.get_logger().warn("收到无效电池电压，忽略本次数据")
                    return
                    
                if voltage < low_threshold:
                    if not self.is_low_battery_level:
                        self.last_mode_before_low_battery = self.mode
                        self.mode = 'low_battery_breath'
                        self.get_logger().warn(f'USV 电池电压低于 {low_threshold}V，进入低电量呼吸灯状态')
                    self.is_low_battery_level = True
                elif voltage > recover_threshold:
                    if self.is_low_battery_level:
                        # 恢复原有模式
                        self.mode = self.last_mode_before_low_battery or 'color_switching'
                        self.get_logger().info(f'USV 电池电压恢复，退出低电量呼吸灯')
                    self.is_low_battery_level = False
        except Exception as e:
            self.get_logger().error(f'处理电池状态时发生错误: {e}')

    def generate_breath_values(self):
        """
        生成呼吸灯亮度序列
        
        Returns:
            list: 包含呼吸灯亮度值的列表
        """
        # 生成红色呼吸亮度序列（0~255~0，步进可调）
        breath_step = self.get_parameter('breath_step').get_parameter_value().integer_value
        up = list(range(0, 256, breath_step))
        down = list(range(255, -1, -breath_step))
        return up + down[1:]  # 去掉重复的255

    def set_timer_period(self, period):
        """
        动态调整定时器周期
        
        Args:
            period (float): 新的定时器周期
        """
        try:
            # 动态调整定时器周期，避免频繁重建定时器
            if abs(period - self.current_timer_period) > 1e-3:
                self.timer.cancel()
                self.timer = self.create_timer(period, self.timer_callback)
                self.current_timer_period = period
        except Exception as e:
            self.get_logger().error(f'调整定时器周期时发生错误: {e}')

    def timer_callback(self):
        """定时器回调函数，处理LED控制逻辑"""
        try:
            now = time.time()
            rgb_to_send = None
            
            if self.mode == 'color_infect':
                self.set_timer_period(self.timer_period_high)
                # 渐变到 target_color
                t = min(1.0, (now - self.last_switch_time) / self.transition_duration)
                self.current_color = [
                    int(self.transition_start_color[i] + (self.target_color[i] - self.transition_start_color[i]) * t)
                    for i in range(3)
                ]
                if t >= 1.0:
                    self.in_transition = False
                    self.last_switch_time = now  # 渐变结束后更新时间，保证停留
                rgb_to_send = self.current_color
                
            # 低电压呼吸灯，强制高频
            elif self.mode == 'low_battery_breath':
                self.set_timer_period(self.timer_period_high)
                voltage = self.usv_state.voltage if hasattr(self.usv_state, 'voltage') else 12.0
                low_threshold = self.get_parameter('low_battery_threshold').get_parameter_value().double_value
                if voltage < low_threshold:
                    voltage = 10.6
                base_interval = 0.1
                min_interval = 0.03
                delta_v = max(0, (low_threshold - voltage))
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
                    # 渐变结束后等待 hold_duration 再切换
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
                        self.last_switch_time = now  # 渐变结束后更新时间
                rgb_to_send = self.current_color
                
            elif self.mode == 'random_color_change':
                self.set_timer_period(self.timer_period_high)
                if not self.in_transition:
                    # 渐变结束后等待 hold_duration 再切换
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
                        self.last_switch_time = now  # 渐变结束后更新时间
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
                # 转换为RGB顺序
                rbg_to_send = []
                for i in range(0, len(rgb_to_send), 3):
                    if i + 2 < len(rgb_to_send):  # 确保有足够的数据
                        r = rgb_to_send[i]
                        g = rgb_to_send[i+1]
                        b = rgb_to_send[i+2]
                        rbg_to_send.extend([r, g, b])
                        
                command = self.build_command(rbg_to_send, 60)
                if command and self.ser and self.ser.is_open:
                    self.ser.write(command)
                    self.last_sent_rgb = rgb_to_send[:]  # 及时缓存，防止重复发送
                    # self.get_logger().info(f'发送LED命令：{command.hex()}')
                elif not self.ser or not self.ser.is_open:
                    self.get_logger().warn('串口未打开，无法发送LED命令')
                    
        except Exception as e:
            self.get_logger().error(f'定时器回调处理时发生错误: {e}')

    def __del__(self):
        """清理资源"""
        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info('串口已关闭')
        except Exception as e:
            self.get_logger().warn(f'关闭串口时发生错误: {e}')


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    try:
        node = UsvLedNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'节点运行时发生错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()