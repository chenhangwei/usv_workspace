#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# ROS 2 Node implementation: Usv Sound Node.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
无人船声音播放节点

该节点负责播放声音文件，支持循环播放和根据电池电压状态选择不同声音。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, Bool
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
        
        # 订阅专门的低电压模式话题（RELIABLE QoS 确保送达）
        self.low_voltage_mode_sub = self.create_subscription(
            Bool,
            'low_voltage_mode',
            self.low_voltage_mode_callback,
            qos_reliable
        )
        
        self.get_logger().info('声音播放节点已启动')
        
        # 使用ParamLoader统一加载参数
        from common_utils import ParamLoader
        loader = ParamLoader(self)
        self.sound_types = loader.load_param('sound_types', ['gaga101', 'gaga102', 'gaga103', 'gaga104'])
        self.moon_type = loader.load_param('moon_type', 'moon101')
        self.min_play_interval = loader.load_param('min_play_interval', 2)
        self.max_play_interval = loader.load_param('max_play_interval', 10)
        self.min_play_count = loader.load_param('min_play_count', 1)
        self.max_play_count = loader.load_param('max_play_count', 3)
        
        # 初始化音频相关变量
        try:
            self.audio = pyaudio.PyAudio()
        except Exception as e:
            self.get_logger().error(f'初始化PyAudio失败: {e}')
            self.audio = None
            
        self.loop_thread = None
        self.loop_stop_event = threading.Event()
        self.low_voltage = False
        
        # 用户意图标志：记录用户是否主动停止声音
        self.user_stopped_sound = False

    def low_voltage_mode_callback(self, msg):
        """
        低电压模式专用回调函数 - 立即响应低电量状态
        
        此回调优先级高于 usv_status_callback，确保快速响应
        
        Args:
            msg (Bool): 低电压模式标志（True=进入低电量，False=退出低电量）
        """
        try:
            if not isinstance(msg, Bool):
                self.get_logger().warn('收到无效的低电压模式消息类型')
                return
            
            if msg.data and not self.low_voltage:
                # 进入低电量模式
                self.low_voltage = True
                self.get_logger().error('[!][!][!] 低电压模式触发！')
                
                # 🚨 低电量警告是安全关键功能，必须无条件触发！
                # 清除用户停止标志，强制播放低电量警告声音
                if self.user_stopped_sound:
                    self.get_logger().warn('[!] 低电量触发，覆盖用户停止指令，强制播放警告声音')
                    self.user_stopped_sound = False
                
                # 启动或重启声音循环
                if not (self.loop_thread and self.loop_thread.is_alive()):
                    self.get_logger().error('[!] 自动启动低电量警告声音播放')
                    self.start_sound_loop()
                else:
                    self.get_logger().info('[!] 声音循环已在运行，将切换到低电量音效')
                
            elif not msg.data and self.low_voltage:
                # 退出低电量模式
                self.low_voltage = False
                self.get_logger().info('[OK] 退出低电压模式')
                
        except Exception as e:
            self.get_logger().error(f'处理低电压模式回调时发生错误: {e}')
    
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
                self.user_stopped_sound = False  # 清除用户停止标志
                self.start_sound_loop()
            elif msg.data == 'sound_stop':
                self.get_logger().info('收到sound_stop，停止循环')
                self.user_stopped_sound = True  # 记录用户主动停止意图
                self.stop_sound_loop()
            else:
                self.get_logger().warn(f'未知的声音控制命令: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'处理声音命令时发生错误: {e}')

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
        rclpy.logging.get_logger('usv_sound_node').error(f'节点运行时发生错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()