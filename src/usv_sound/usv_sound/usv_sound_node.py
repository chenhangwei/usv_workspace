import rclpy
from rclpy.node import Node
from std_msgs.msg import String  #  或者你自定义的音频消息类型
import pyaudio
import wave
import os
import ament_index_python.packages  # Add this import
import threading

class UsvSoundNode(Node):
    def __init__(self):
        super().__init__('usv_sound_node')

        self.subscription = self.create_subscription(
            String,  #  替换为你的音频消息类型
            'gs_sound_command',  #  替换为你的topic名称
            self.gs_sound_callback,
            10)
        self.get_logger().info('声音播放节点已启动')

        self.timer_loop = self.create_timer(10, self.timer_callback)

        self.audio = pyaudio.PyAudio()
        self.sound_command=''
        self.is_playing = False  # 防止音频重叠
        self.current_iteration = 0  # 当前循环次数
        self.max_iterations = 0  # 最大循环次数
        self.delay_sec = 0  # 延时秒数
        self.timer_ = None  # 定时器对象
        self.sound_type_moon = 'moon_44100'  # 默认音频类型

    # 接收地面站的音频播放命令
    def gs_sound_callback(self, msg):
        self.get_logger().info(f'收到声音播放命令: {msg.data}')
        if not isinstance(msg, String):
            self.get_logger().error('收到无效的声音消息类型')
            return
        self.sound_command=msg.data

    # 定时播放声音
    def timer_callback(self):

        match self.sound_command:
            case 'gaga1':
                self.sound_loop(2, 1,self.sound_type_moon)  # 延时2秒，循环3次,播放 moon_44100.wav
            case 'gaga2':
                self.sound_loop(2, 2,self.sound_type_moon)  # 延时2秒，循环2次，播放 moon_44100.wav
            case 'gaga3':
               self.sound_loop(2, 3,self.sound_type_moon)  # 延时2秒，循环3次，播放 moon_44100.wav
            case 'gaga4':
                self.sound_loop(2, 4,self.sound_type_moon)  # 延时2秒，循环4次，播放 moon_44100.wav
            case 'gaga-stop':
                self.sound_loop(0, 0,self.sound_type_moon)  # 停止循环

                
       
    def sound_loop(self,sec,num, sound_type):
        if num <= 0:
            # self.get_logger().warn('循环次数必须大于 0')
            return
        if sec < 0:
            # self.get_logger().warn('延时秒数不能为负')
            return
        if self.current_iteration > 0:
            # self.get_logger().warn('已有循环在运行，忽略本次请求')
            return

        self.current_iteration = 0
        self.max_iterations = num
        self.delay_sec = sec
        # self.get_logger().info(f'开始循环播放 {num} 次，每次延时 {sec} 秒')
        self.sound_moon_play(sound_type)  # 启动第一次播放
 
    def post_play_callback(self):
        # self.get_logger().info('音频播放后延时 {} 秒执行的任务'.format(self.delay_sec))
        if self.timer_ is not None:
            self.timer_.cancel()  # 取消当前定时器

        self.current_iteration += 1
        if self.current_iteration >= self.max_iterations:
            # self.get_logger().info('循环播放完成')
            self.current_iteration = 0  # 重置状态
            return

        # 触发下一次播放
        # self.get_logger().info(f'第 {self.current_iteration + 1} 次播放')
        self.sound_moon_play(self.sound_type_moon)

    def sound_moon_play(self, sound_type, chunk_size=1024):
        if self.is_playing:
            # self.get_logger().warn('另一个音频正在播放，忽略本次请求')
            return
        package_name = 'usv_sound'
        package_path = ament_index_python.packages.get_package_share_directory(package_name)
        filename = os.path.join(package_path, 'resource', f'{sound_type}.wav')
        if not os.path.exists(filename):
            self.get_logger().warn(f'文件不存在: {filename}')
            return

        self.is_playing = True
        def _play():
            try:
                with wave.open(filename, 'rb') as wf:
                    duration = wf.getnframes() / wf.getframerate()
                    try:
                        device_index = self.get_output_device_index()
                        self.get_logger().info(f'使用设备索引: {device_index}')
                        stream = self.audio.open(
                                format=pyaudio.paInt16,  # 强制 16-bit PCM
                                channels=2,              # 立体声
                                rate=44100,              # 44.1 kHz
                                output=True,
                                output_device_index=None,  # 不指定索引，使用 default
                                frames_per_buffer=1024  # 添加此参数
                            # format=self.audio.get_format_from_width(wf.getsampwidth()),
                            # channels=wf.getnchannels(),
                            # rate=wf.getframerate(),
                            # output=True,
                            # output_device_index=device_index if device_index is not None else 0
                        )
                    except OSError as e:
                        self.get_logger().error(f'无法打开音频流: {e}')
                        self.is_playing = False
                        return

                    try:
                        data = wf.readframes(chunk_size)
                        while data:
                            stream.write(data)
                            data = wf.readframes(chunk_size)
                    except Exception as e:
                        self.get_logger().error(f'播放音频时出错: {e}')
                    finally:
                        stream.stop_stream()
                        stream.close()
                        self.get_logger().info('音频流已关闭')
                        self.is_playing = False
                        self.timer_ = self.create_timer(self.delay_sec, self.post_play_callback)

            except wave.Error as e:
                self.get_logger().error(f'无法打开 WAV 文件: {e}')
            except Exception as e:
                self.get_logger().error(f'未知错误: {e}')

        threading.Thread(target=_play, daemon=True).start()
        # self.get_logger().info(f'开始播放音频: {filename}')
    def __del__(self):
        self.audio.terminate()

    def get_output_device_index(self):
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            name = str(info.get('name', ''))
            max_output_channels = info.get('maxOutputChannels')
            # 优先选 HDMI
            if isinstance(max_output_channels, (int, float)) and max_output_channels > 0 and ('HDMI' in name or 'hdmi' in name):
                self.get_logger().info(f"检测到HDMI音响: index={i}, name={info['name']}")
                return i
        # 退而求其次，选第一个有输出通道的设备
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            max_output_channels = info.get('maxOutputChannels')
            if isinstance(max_output_channels, (int, float)) and max_output_channels > 0:
                self.get_logger().info(f"检测到可用音响: index={i}, name={info['name']}")
                return i
        self.get_logger().warn('未检测到可用音响，将使用默认输出设备')
        return None
def main(args=None):
    rclpy.init(args=args)
    node =UsvSoundNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()