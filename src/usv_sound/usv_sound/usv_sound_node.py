import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState
import pyaudio
import wave
import os
import ament_index_python.packages
import threading
import random

class UsvSoundNode(Node):
    def __init__(self):
        super().__init__('usv_sound_node')
        self.subscription = self.create_subscription(
            String, 'gs_sound_command', self.gs_sound_callback, 10)
        # 订阅 MAVROS 的电池状态主题
        self.battery_sub = self.create_subscription(
            BatteryState,
            f'battery',  # 使用命名空间
            self.voltage_callback,
           10
        )
        self.get_logger().info('声音播放节点已启动')
        self.audio = pyaudio.PyAudio()
        self.loop_thread = None
        self.loop_stop_event = threading.Event()
        self.low_voltage = False
        self.sound_types = ['gaga1', 'gaga2', 'gaga3', 'gaga4']
        self.moon_type = 'moon'
        self.voltage = 12.0

    def gs_sound_callback(self, msg):
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的声音消息类型')
            return
        if msg.data == 'sound_start':
            self.get_logger().info('收到sound_start，启动循环')
            self.start_sound_loop()
        elif msg.data == 'sound_stop':
            self.get_logger().info('收到sound_stop，停止循环')
            self.stop_sound_loop()

    def voltage_callback(self, msg):
        if not isinstance(msg, BatteryState):
            return
        self.voltage = msg.voltage
        if self.voltage < 11.1:
            self.low_voltage = True
        else:
            self.low_voltage = False

    def start_sound_loop(self):
        # 如果循环已在运行则不重复启动
        if self.loop_thread and self.loop_thread.is_alive():
            self.get_logger().info('循环已在运行，忽略重复启动')
            return
        self.loop_stop_event.clear()
        self.loop_thread = threading.Thread(target=self.sound_loop, daemon=True)
        self.loop_thread.start()

    def stop_sound_loop(self):
        self.loop_stop_event.set()
        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=1)
        self.loop_thread = None

    def destroy_node(self):
        # 节点销毁时确保循环线程安全退出
        self.stop_sound_loop()
        super().destroy_node()

    def sound_loop(self):
        while not self.loop_stop_event.is_set():
            sec = random.randint(2, 10)
            num = random.randint(1, 3)
            if self.low_voltage:
                sound_type = self.moon_type
            else:
                sound_type = random.choice(self.sound_types)
            self.get_logger().info(f'循环播放: {sound_type}.wav, 延时: {sec}s, 次数: {num}')
            for i in range(num):
                if self.loop_stop_event.is_set():
                    break
                self.sound_play(sound_type)
                for _ in range(sec * 10):
                    if self.loop_stop_event.is_set():
                        break
                    import time
                    time.sleep(0.1)
            # 一次循环结束后自动选择下一个sound_type

    def sound_play(self, sound_type, chunk_size=1024):
        package_name = 'usv_sound'
        package_path = ament_index_python.packages.get_package_share_directory(package_name)
        filename = os.path.join(package_path, 'resource', f'{sound_type}.wav')
        if not os.path.exists(filename):
            self.get_logger().warn(f'文件不存在: {filename}')
            return
        try:
            with wave.open(filename, 'rb') as wf:
                try:
                    stream = self.audio.open(
                        # format=pyaudio.paInt16,  # 强制 16-bit PCM
                        format=self.audio.get_format_from_width(wf.getsampwidth()),                    
                        # channels=2,              # 立体声 
                        channels=wf.getnchannels(),
                        # rate=44100,              # 44.1 kHz
                        rate=wf.getframerate(),
                        output=True,
                        output_device_index=None,  # 不指定索引，使用 default
                        frames_per_buffer=1024  # 添加此参数

                    )
                except OSError as e:
                    self.get_logger().error(f'无法打开音频流: {e}')
                    return
                data = wf.readframes(chunk_size)
                while data:
                    stream.write(data)
                    data = wf.readframes(chunk_size)
                stream.stop_stream()
                stream.close()
        except Exception as e:
            self.get_logger().error(f'播放音频时出错: {e}')

    def __del__(self):
        self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = UsvSoundNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()