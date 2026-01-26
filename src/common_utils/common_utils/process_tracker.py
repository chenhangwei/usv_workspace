#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of process tracker.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
进程追踪器

管理 subprocess 子进程,防止僵尸进程和资源泄漏。
"""

import subprocess
import atexit
import signal
from typing import Dict, Optional, List
import logging


class ProcessTracker:
    """
    全局进程追踪器,管理所有子进程的生命周期
    
    单例模式,确保所有进程被统一管理
    """
    
    _instance = None
    _processes: Dict[int, subprocess.Popen] = {}
    _logger = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._logger = logging.getLogger(__name__)
            # 注册退出清理
            atexit.register(cls.cleanup_all)
            # 注册信号处理
            signal.signal(signal.SIGTERM, cls._signal_handler)
            signal.signal(signal.SIGINT, cls._signal_handler)
        return cls._instance
    
    @classmethod
    def track(cls, process: subprocess.Popen, description: str = "") -> int:
        """
        追踪子进程
        
        Args:
            process: 子进程对象
            description: 进程描述(用于日志)
            
        Returns:
            进程 PID
            
        Example:
            proc = subprocess.Popen(['aplay', 'sound.wav'])
            ProcessTracker.track(proc, '播放声音')
        """
        cls._processes[process.pid] = {
            'process': process,
            'description': description
        }
        if cls._logger:
            cls._logger.debug(f"追踪进程 PID={process.pid}: {description}")
        return process.pid
    
    @classmethod
    def untrack(cls, pid: int) -> bool:
        """
        停止追踪进程(进程已正常结束)
        
        Args:
            pid: 进程 PID
            
        Returns:
            是否成功移除追踪
        """
        if pid in cls._processes:
            desc = cls._processes[pid]['description']
            del cls._processes[pid]
            if cls._logger:
                cls._logger.debug(f"停止追踪进程 PID={pid}: {desc}")
            return True
        return False
    
    @classmethod
    def terminate(cls, pid: int, timeout: float = 5.0) -> bool:
        """
        终止指定进程
        
        Args:
            pid: 进程 PID
            timeout: 等待超时时间
            
        Returns:
            是否成功终止
        """
        if pid not in cls._processes:
            return False
        
        proc_info = cls._processes[pid]
        process = proc_info['process']
        desc = proc_info['description']
        
        try:
            # 检查进程是否还在运行
            if process.poll() is None:
                if cls._logger:
                    cls._logger.info(f"终止进程 PID={pid}: {desc}")
                
                # 先尝试温和终止
                process.terminate()
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    # 强制杀死
                    if cls._logger:
                        cls._logger.warn(f"进程 {pid} 未响应 SIGTERM,发送 SIGKILL")
                    process.kill()
                    process.wait()
            
            # 移除追踪
            del cls._processes[pid]
            return True
            
        except Exception as e:
            if cls._logger:
                cls._logger.error(f"终止进程 {pid} 失败: {e}")
            return False
    
    @classmethod
    def cleanup_all(cls):
        """清理所有追踪的进程"""
        if not cls._processes:
            return
        
        if cls._logger:
            cls._logger.info(f"清理 {len(cls._processes)} 个子进程...")
        
        pids = list(cls._processes.keys())
        for pid in pids:
            cls.terminate(pid, timeout=3.0)
        
        if cls._logger:
            cls._logger.info("所有子进程已清理")
    
    @classmethod
    def get_running_processes(cls) -> List[dict]:
        """
        获取所有运行中的进程信息
        
        Returns:
            进程信息列表 [{'pid': int, 'description': str, 'running': bool}, ...]
        """
        result = []
        for pid, info in cls._processes.items():
            process = info['process']
            result.append({
                'pid': pid,
                'description': info['description'],
                'running': process.poll() is None
            })
        return result
    
    @classmethod
    def _signal_handler(cls, signum, frame):
        """信号处理器"""
        if cls._logger:
            cls._logger.info(f"接收到信号 {signum},清理子进程...")
        cls.cleanup_all()
    
    @classmethod
    def run_and_track(
        cls,
        args: List[str],
        description: str = "",
        wait: bool = False,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Optional[subprocess.Popen]:
        """
        运行并追踪子进程的便捷方法
        
        Args:
            args: 命令参数列表
            description: 进程描述
            wait: 是否等待进程结束
            timeout: 等待超时时间
            **kwargs: Popen 的其他参数
            
        Returns:
            进程对象(wait=False 时) 或 None
            
        Example:
            # 后台播放声音
            ProcessTracker.run_and_track(
                ['aplay', 'sound.wav'],
                description='播放声音',
                wait=False
            )
            
            # 同步执行命令
            ProcessTracker.run_and_track(
                ['ls', '-l'],
                description='列出文件',
                wait=True,
                timeout=5.0
            )
        """
        try:
            process = subprocess.Popen(args, **kwargs)
            cls.track(process, description)
            
            if wait:
                try:
                    process.wait(timeout=timeout)
                    cls.untrack(process.pid)
                except subprocess.TimeoutExpired:
                    if cls._logger:
                        cls._logger.warn(f"进程超时: {description}")
                    cls.terminate(process.pid)
                return None
            else:
                return process
                
        except Exception as e:
            if cls._logger:
                cls._logger.error(f"启动进程失败 {description}: {e}")
            return None


# 使用示例
"""
from common_utils import ProcessTracker

class SoundNode(Node):
    def __init__(self):
        super().__init__('sound_node')
        self.process_tracker = ProcessTracker()
    
    def play_sound(self, sound_file: str):
        # 方法1: 手动追踪
        proc = subprocess.Popen(['aplay', sound_file])
        ProcessTracker.track(proc, f'播放 {sound_file}')
        
        # 方法2: 自动追踪(推荐)
        ProcessTracker.run_and_track(
            ['aplay', sound_file],
            description=f'播放 {sound_file}',
            wait=False  # 后台播放
        )
    
    def run_command_sync(self, command: List[str]):
        # 同步执行,带超时
        ProcessTracker.run_and_track(
            command,
            description=' '.join(command),
            wait=True,
            timeout=10.0
        )
    
    def destroy_node(self):
        # 清理所有子进程
        ProcessTracker.cleanup_all()
        super().destroy_node()
"""
