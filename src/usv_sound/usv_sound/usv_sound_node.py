"""
无人船声音播放节点

该节点负责播放声音文件，支持循环播放和根据电池电压状态选择不同声音。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import pyaudio
import wave
import os
import ament_index_python.packages
import threading
import random
import time


class UsvSoundNode(Node):
    """
    无人船声音播放节点类
    
    该节点实现声音播放功能，支持循环播放和根据电池电压状态选择不同声音。
    通过订阅地面站命令和电池状态，自动调整声音播放策略。
    """

    def __init__(self):
        """初始化无人船声音播放节点"""
        super().__init__('usv_sound_node')

        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 订阅地面站的声音控制命令
        self.subscription = self.create_subscription(
            String, 
            'gs_sound_command', 
            self.gs_sound_callback, 
            qos_reliable
        )
        
        # 订阅 MAVROS 的电池状态主题
        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery',
            self.voltage_callback,
            qos_best_effort
        )
        
        self.get_logger().info('声音播放节点已启动')
        
        # 声明参数
        # ⚠️ 低电量判断 - 基于飞控百分比（需要在 QGroundControl 中配置 BATT_CAPACITY）
        self.declare_parameter('low_battery_percentage', 10.0)        # 低电量阈值（百分比）
        self.declare_parameter('sound_types', ['gaga101', 'gaga102', 'gaga103', 'gaga104'])
        self.declare_parameter('moon_type', 'moon101')
        self.declare_parameter('min_play_interval', 2)
        self.declare_parameter('max_play_interval', 10)
        self.declare_parameter('min_play_count', 1)
        self.declare_parameter('max_play_count', 3)
        
        # 初始化音频相关变量
        try:
            self.audio = pyaudio.PyAudio()
        except Exception as e:
            self.get_logger().error(f'初始化PyAudio失败: {e}')
            self.audio = None
            
        self.loop_thread = None
        self.loop_stop_event = threading.Event()
        self.low_voltage = False
        
        # 从参数读取配置
        self.sound_types = self.get_parameter('sound_types').get_parameter_value().string_array_value
        self.moon_type = self.get_parameter('moon_type').get_parameter_value().string_value
        self.voltage = 12.0
        self.battery_percentage = 100.0

    def gs_sound_callback(self, msg):
        """
        地面站声音命令回调函数
        
        Args:
            msg (String): 包含声音控制命令的消息
        """
        try:
            if not isinstance(msg, String):
                self.get_logger().error('收到无效的声音消息类型')
                return
                
            if msg.data == 'sound_start':
                self.get_logger().info('收到sound_start，启动循环')
                self.start_sound_loop()
            elif msg.data == 'sound_stop':
                self.get_logger().info('收到sound_stop，停止循环')
                self.stop_sound_loop()
            else:
                self.get_logger().warn(f'未知的声音控制命令: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'处理声音命令时发生错误: {e}')

    def voltage_callback(self, msg):
        """
        电池状态回调函数 - 基于飞控百分比判断低电量
        
        Args:
            msg (BatteryState): 包含电池状态信息的消息
        """
        try:
            if not isinstance(msg, BatteryState):
                self.get_logger().warn('收到无效的电池状态消息类型')
                return
                
            # 获取飞控百分比（0.0-1.0）
            self.voltage = msg.voltage if hasattr(msg, 'voltage') else 12.0
            percentage = getattr(msg, 'percentage', -1.0)
            
            if self.voltage == 0.0:
                return
            
            # 检查百分比是否有效
            if not (0.0 <= percentage <= 1.0):
                # 飞控未配置 BATT_CAPACITY，无法判断低电量
                return
            
            # 转换为百分比（0-100）
            self.battery_percentage = percentage * 100.0
            
            # 获取低电量阈值
            low_threshold = self.get_parameter('low_battery_percentage').get_parameter_value().double_value
            
            # 检查是否为低电量状态（带状态变化日志）
            if self.battery_percentage < low_threshold:
                if not self.low_voltage:
                    self.get_logger().warn(
                        f'⚠️ 电池电量低: {self.battery_percentage:.1f}%，切换到低电量声音'
                    )
                self.low_voltage = True
            else:
                if self.low_voltage:
                    self.get_logger().info(
                        f'✅ 电池电量恢复正常: {self.battery_percentage:.1f}%'
                    )
                self.low_voltage = False
                
        except Exception as e:
            self.get_logger().error(f'处理电池状态时发生错误: {e}')

    def start_sound_loop(self):
        """启动声音循环播放"""
        try:
            # 如果循环已在运行则不重复启动
            if self.loop_thread and self.loop_thread.is_alive():
                self.get_logger().info('循环已在运行，忽略重复启动')
                return
                
            self.loop_stop_event.clear()
            self.loop_thread = threading.Thread(target=self.sound_loop, daemon=True)
            self.loop_thread.start()
            self.get_logger().info('声音循环播放已启动')
        except Exception as e:
            self.get_logger().error(f'启动声音循环时发生错误: {e}')

    def stop_sound_loop(self):
        """停止声音循环播放"""
        try:
            self.loop_stop_event.set()
            if self.loop_thread and self.loop_thread.is_alive():
                self.loop_thread.join(timeout=1)
            self.loop_thread = None
            self.get_logger().info('声音循环播放已停止')
        except Exception as e:
            self.get_logger().error(f'停止声音循环时发生错误: {e}')

    def destroy_node(self):
        """节点销毁时确保循环线程安全退出"""
        try:
            self.stop_sound_loop()
            super().destroy_node()
        except Exception as e:
            self.get_logger().error(f'销毁节点时发生错误: {e}')

    def sound_loop(self):
        """声音循环播放主逻辑"""
        try:
            self.get_logger().info('开始声音循环播放')
            while not self.loop_stop_event.is_set():
                sec = random.randint(2, 10)
                num = random.randint(1, 3)
                
                # 根据电压状态选择声音类型
                if self.low_voltage:
                    sound_type = self.moon_type
                    self.get_logger().debug(f'低电压状态，选择声音: {sound_type}')
                else:
                    sound_type = random.choice(self.sound_types)
                    self.get_logger().debug(f'正常电压状态，随机选择声音: {sound_type}')
                    
                self.get_logger().info(f'循环播放: {sound_type}.wav, 延时: {sec}s, 次数: {num}')
                
                for i in range(num):
                    if self.loop_stop_event.is_set():
                        break
                        
                    self.sound_play(sound_type)
                    
                    # 等待指定的延时
                    for _ in range(sec * 10):
                        if self.loop_stop_event.is_set():
                            break
                        time.sleep(0.1)
                        
            self.get_logger().info('声音循环播放结束')
        except Exception as e:
            self.get_logger().error(f'声音循环播放时发生错误: {e}')

    def sound_play(self, sound_type, chunk_size=1024):
        """
        播放指定声音文件
        
        Args:
            sound_type (str): 声音文件类型
            chunk_size (int): 音频数据块大小
        """
        try:
            # 检查音频系统是否已初始化
            if not self.audio:
                self.get_logger().error('音频系统未初始化')
                return
                
            package_name = 'usv_sound'
            package_path = ament_index_python.packages.get_package_share_directory(package_name)
            filename = os.path.join(package_path, 'resource', f'{sound_type}.wav')
            
            # 检查声音文件是否存在
            if not os.path.exists(filename):
                self.get_logger().warn(f'声音文件不存在: {filename}')
                return
                
            self.get_logger().debug(f'准备播放声音文件: {filename}')
            
            # 获取默认音频输出设备
            try:
                default_index = self.audio.get_default_output_device_info()['index']
            except Exception as e:
                self.get_logger().error(f'获取默认音频输出设备失败: {e}')
                return
                
            # 打开并播放音频文件
            with wave.open(filename, 'rb') as wf:
                try:
                    stream = self.audio.open(
                        format=self.audio.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True,
                        output_device_index=default_index,
                        frames_per_buffer=chunk_size
                    )
                except OSError as e:
                    self.get_logger().error(f'无法打开音频流: {e}')
                    return
                    
                # 读取并播放音频数据
                data = wf.readframes(chunk_size)
                while data and not self.loop_stop_event.is_set():
                    stream.write(data)
                    data = wf.readframes(chunk_size)
                    
                # 清理音频流
                stream.stop_stream()
                stream.close()
                self.get_logger().debug(f'声音播放完成: {sound_type}.wav')
                
        except Exception as e:
            self.get_logger().error(f'播放音频时出错: {e}')

    def __del__(self):
        """清理音频资源"""
        try:
            if hasattr(self, 'audio') and self.audio:
                self.audio.terminate()
                self.get_logger().info('音频资源已清理')
        except Exception as e:
            self.get_logger().warn(f'清理音频资源时发生错误: {e}')


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    try:
        node = UsvSoundNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'节点运行时发生错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()