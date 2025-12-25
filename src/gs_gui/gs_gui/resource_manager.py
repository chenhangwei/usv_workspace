"""
资源管理模块。
提供统一的资源生命周期管理、自动清理和泄漏检测。
"""

import weakref
import threading
import queue
import atexit
from typing import Dict, List, Callable, Any, Optional
from contextlib import contextmanager
import logging


class ResourceManager:
    """
    资源管理器类，跟踪和管理系统资源。
    """
    
    def __init__(self, node_logger: Optional[logging.Logger] = None):
        """
        初始化资源管理器。
        
        Args:
            node_logger: 日志记录器。
        """
        self.logger: logging.Logger = node_logger or logging.getLogger(__name__)
        self._resources: Dict[str, Dict[str, Any]] = {}  # 资源注册表
        self._cleanup_handlers: List[Callable] = []  # 清理函数列表
        self._lock: threading.Lock = threading.Lock()  # 线程锁
        self._is_shutdown: bool = False  # 关闭标志
        
        # 注册程序退出时的清理
        atexit.register(self.cleanup_all)
    
    def register_resource(self, name: str, resource: Any, 
                          cleanup_func: Optional[Callable[[Any], None]] = None) -> bool:
        """
        注册需要管理的资源。
        
        Args:
            name: 资源名称。
            resource: 资源对象。
            cleanup_func: 清理函数。
            
        Returns:
            bool: 注册是否成功。
        """
        with self._lock:
            if name in self._resources:
                self.logger.warning(f"资源 {name} 已存在，将被覆盖")
            
            self._resources[name] = {
                'resource': resource,
                'cleanup': cleanup_func,
                'ref': weakref.ref(resource, lambda x: self._on_resource_deleted(name))
            }
            self.logger.debug(f"注册资源: {name}")
            return True
    
    def unregister_resource(self, name: str) -> bool:
        """
        注销资源（手动清理）。
        
        Args:
            name: 资源名称。
            
        Returns:
            bool: 注销是否成功。
        """
        with self._lock:
            if name not in self._resources:
                self.logger.warning(f"资源 {name} 不存在")
                return False
            
            resource_info = self._resources[name]
            
            # 执行清理函数
            if resource_info['cleanup']:
                try:
                    resource_info['cleanup'](resource_info['resource'])
                    self.logger.debug(f"清理资源: {name}")
                except Exception as e:
                    self.logger.error(f"清理资源 {name} 失败: {e}")
            
            del self._resources[name]
            return True
    
    def get_resource(self, name: str) -> Any:
        """
        获取资源对象。
        
        Args:
            name: 资源名称。
            
        Returns:
            Any: 资源对象或 None。
        """
        with self._lock:
            if name in self._resources:
                return self._resources[name]['resource']
            return None
    
    def _on_resource_deleted(self, name: str) -> None:
        """
        资源被垃圾回收时的回调。
        
        Args:
            name: 资源名称。
        """
        self.logger.warning(f"资源 {name} 被垃圾回收但未显式清理")
    
    def cleanup_all(self) -> None:
        """
        清理所有注册的资源。
        """
        if self._is_shutdown:
            return
        
        self._is_shutdown = True
        self.logger.info("开始清理所有资源...")
        
        with self._lock:
            # 按注册顺序反向清理
            for name in list(self._resources.keys()):
                try:
                    self.unregister_resource(name)
                except Exception as e:
                    self.logger.error(f"清理资源 {name} 时发生异常: {e}")
        
        self.logger.info("所有资源清理完成")
    
    @contextmanager
    def managed_resource(self, name: str, resource: Any, 
                         cleanup_func: Optional[Callable[[Any], None]] = None):
        """
        上下文管理器，自动管理资源生命周期。
        
        Args:
            name: 资源名称。
            resource: 资源对象。
            cleanup_func: 清理函数。
        """
        try:
            self.register_resource(name, resource, cleanup_func)
            yield resource
        finally:
            self.unregister_resource(name)


class QueueManager:
    """
    队列管理器类，安全管理消息队列。
    """
    
    def __init__(self, max_size: int = 100, block_on_full: bool = False):
        """
        初始化队列管理器。
        
        Args:
            max_size: 队列最大容量。
            block_on_full: 队列满时是否阻塞。
        """
        self.queue: queue.Queue = queue.Queue(maxsize=max_size)
        self.block_on_full: bool = block_on_full
        self._total_enqueued: int = 0
        self._total_dequeued: int = 0
        self._dropped_count: int = 0
    
    def put(self, item: Any, timeout: Optional[float] = None) -> bool:
        """
        安全入队。
        
        Args:
            item: 要入队的项。
            timeout: 超时时间。
            
        Returns:
            bool: 是否成功入队。
        """
        try:
            if self.block_on_full:
                self.queue.put(item, block=True, timeout=timeout)
            else:
                self.queue.put_nowait(item)
            
            self._total_enqueued += 1
            return True
        except queue.Full:
            self._dropped_count += 1
            return False
    
    def get(self, timeout: float = 1.0) -> Any:
        """
        安全出队。
        
        Args:
            timeout: 超时时间。
            
        Returns:
            Any: 出队的项或 None。
        """
        try:
            item = self.queue.get(timeout=timeout)
            self._total_dequeued += 1
            self.queue.task_done()
            return item
        except queue.Empty:
            return None
    
    def get_stats(self) -> Dict[str, int]:
        """
        获取队列统计信息。
        
        Returns:
            Dict[str, int]: 统计信息字典。
        """
        return {
            'size': self.queue.qsize(),
            'total_enqueued': self._total_enqueued,
            'total_dequeued': self._total_dequeued,
            'dropped': self._dropped_count,
            'pending': self._total_enqueued - self._total_dequeued
        }
    
    def clear(self) -> None:
        """
        清空队列。
        """
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except queue.Empty:
                break


class ThreadManager:
    """
    线程管理器类，统一管理后台线程。
    """
    
    def __init__(self, node_logger: Optional[logging.Logger] = None):
        """
        初始化线程管理器。
        
        Args:
            node_logger: 日志记录器。
        """
        self.logger: logging.Logger = node_logger or logging.getLogger(__name__)
        self._threads: Dict[str, threading.Thread] = {}
        self._stop_events: Dict[str, threading.Event] = {}
        self._lock: threading.Lock = threading.Lock()
    
    def start_thread(self, name: str, target: Callable[[threading.Event], None], 
                     daemon: bool = True, **kwargs) -> bool:
        """
        启动并注册线程。
        
        Args:
            name: 线程名称。
            target: 线程目标函数。
            daemon: 是否为守护线程。
            **kwargs: 传递给线程的参数。
            
        Returns:
            bool: 是否成功启动。
        """
        with self._lock:
            if name in self._threads and self._threads[name].is_alive():
                self.logger.warning(f"线程 {name} 已在运行")
                return False
            
            stop_event = threading.Event()
            self._stop_events[name] = stop_event
            
            # 包装目标函数，传入 stop_event
            def wrapped_target():
                try:
                    target(stop_event, **kwargs)
                except Exception as e:
                    self.logger.error(f"线程 {name} 异常退出: {e}")
            
            thread = threading.Thread(
                target=wrapped_target,
                name=name,
                daemon=daemon
            )
            thread.start()
            self._threads[name] = thread
            
            self.logger.info(f"启动线程: {name}")
            return True
    
    def stop_thread(self, name: str, timeout: float = 2.0) -> bool:
        """
        停止指定线程。
        
        Args:
            name: 线程名称。
            timeout: 等待超时时间。
            
        Returns:
            bool: 是否成功停止。
        """
        with self._lock:
            if name not in self._threads:
                self.logger.warning(f"线程 {name} 不存在")
                return False
            
            thread = self._threads[name]
            stop_event = self._stop_events[name]
            
            # 发送停止信号
            stop_event.set()
            
            # 等待线程结束
            thread.join(timeout=timeout)
            
            if thread.is_alive():
                self.logger.warning(f"线程 {name} 未在超时时间内停止")
                return False
            
            del self._threads[name]
            del self._stop_events[name]
            
            self.logger.info(f"停止线程: {name}")
            return True
    
    def stop_all(self, timeout: float = 5.0) -> None:
        """
        停止所有线程。
        
        Args:
            timeout: 每个线程等待的超时时间。
        """
        self.logger.info("停止所有线程...")
        
        with self._lock:
            thread_names = list(self._threads.keys())
        
        for name in thread_names:
            self.stop_thread(name, timeout=timeout)
