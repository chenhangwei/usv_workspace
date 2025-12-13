"""
线程安全工具

提供线程安全装饰器和数据结构,解决多线程访问共享资源的问题。
"""

import threading
from functools import wraps
from typing import TypeVar, Generic, Dict, Optional, Callable

K = TypeVar('K')
V = TypeVar('V')


def thread_safe(func: Callable) -> Callable:
    """
    线程安全装饰器,为方法添加自动锁保护
    
    要求类中有 _lock 属性(threading.Lock 或 RLock)
    
    Example:
        class MyClass:
            def __init__(self):
                self._lock = threading.RLock()
                self.counter = 0
            
            @thread_safe
            def increment(self):
                self.counter += 1
            
            @thread_safe
            def get_counter(self):
                return self.counter
    """
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not hasattr(self, '_lock'):
            raise AttributeError(
                f"{self.__class__.__name__} 必须有 _lock 属性才能使用 @thread_safe"
            )
        with self._lock:
            return func(self, *args, **kwargs)
    return wrapper


class ThreadSafeDict(Generic[K, V]):
    """
    线程安全字典
    
    提供基本字典操作的线程安全版本,适合多线程环境下的共享数据。
    
    Example:
        # 替换普通字典
        # self._usv_states = {}  # 不安全
        self._usv_states = ThreadSafeDict()  # 安全
        
        # 使用方式相同
        self._usv_states['usv_01'] = state
        state = self._usv_states.get('usv_01')
    """
    
    def __init__(self):
        self._dict: Dict[K, V] = {}
        self._lock = threading.RLock()
    
    def __setitem__(self, key: K, value: V):
        """设置键值"""
        with self._lock:
            self._dict[key] = value
    
    def __getitem__(self, key: K) -> V:
        """获取键值"""
        with self._lock:
            return self._dict[key]
    
    def __delitem__(self, key: K):
        """删除键值"""
        with self._lock:
            del self._dict[key]
    
    def __contains__(self, key: K) -> bool:
        """检查键是否存在"""
        with self._lock:
            return key in self._dict
    
    def __len__(self) -> int:
        """获取长度"""
        with self._lock:
            return len(self._dict)
    
    def get(self, key: K, default: Optional[V] = None) -> Optional[V]:
        """安全获取,不存在返回默认值"""
        with self._lock:
            return self._dict.get(key, default)
    
    def pop(self, key: K, default: Optional[V] = None) -> Optional[V]:
        """移除并返回值"""
        with self._lock:
            return self._dict.pop(key, default)
    
    def keys(self):
        """获取所有键(快照)"""
        with self._lock:
            return list(self._dict.keys())
    
    def values(self):
        """获取所有值(快照)"""
        with self._lock:
            return list(self._dict.values())
    
    def items(self):
        """获取所有键值对(快照)"""
        with self._lock:
            return list(self._dict.items())
    
    def clear(self):
        """清空字典"""
        with self._lock:
            self._dict.clear()
    
    def update(self, other: Dict[K, V]):
        """批量更新"""
        with self._lock:
            self._dict.update(other)
    
    def setdefault(self, key: K, default: V) -> V:
        """如果键不存在则设置默认值"""
        with self._lock:
            return self._dict.setdefault(key, default)


class ThreadSafeCounter:
    """
    线程安全计数器
    
    Example:
        counter = ThreadSafeCounter()
        counter.increment()
        counter.increment()
        print(counter.value)  # 2
    """
    
    def __init__(self, initial: int = 0):
        self._value = initial
        self._lock = threading.Lock()
    
    def increment(self, delta: int = 1) -> int:
        """递增并返回新值"""
        with self._lock:
            self._value += delta
            return self._value
    
    def decrement(self, delta: int = 1) -> int:
        """递减并返回新值"""
        with self._lock:
            self._value -= delta
            return self._value
    
    def reset(self, value: int = 0):
        """重置为指定值"""
        with self._lock:
            self._value = value
    
    @property
    def value(self) -> int:
        """获取当前值"""
        with self._lock:
            return self._value


class ReadWriteLock:
    """
    读写锁
    
    允许多个读者同时读,但写者独占。
    适合读多写少的场景。
    
    Example:
        lock = ReadWriteLock()
        
        # 读操作
        with lock.read():
            data = shared_data.copy()
        
        # 写操作
        with lock.write():
            shared_data['key'] = 'value'
    """
    
    def __init__(self):
        self._readers = 0
        self._writers = 0
        self._read_ready = threading.Condition(threading.Lock())
        self._write_ready = threading.Condition(threading.Lock())
    
    def acquire_read(self):
        """获取读锁"""
        self._read_ready.acquire()
        try:
            while self._writers > 0:
                self._read_ready.wait()
            self._readers += 1
        finally:
            self._read_ready.release()
    
    def release_read(self):
        """释放读锁"""
        self._read_ready.acquire()
        try:
            self._readers -= 1
            if self._readers == 0:
                self._read_ready.notifyAll()
        finally:
            self._read_ready.release()
    
    def acquire_write(self):
        """获取写锁"""
        self._write_ready.acquire()
        self._writers += 1
        self._write_ready.release()
        
        self._read_ready.acquire()
        while self._readers > 0:
            self._read_ready.wait()
    
    def release_write(self):
        """释放写锁"""
        self._writers -= 1
        self._read_ready.notifyAll()
        self._read_ready.release()
    
    def read(self):
        """读锁上下文管理器"""
        return _ReadLockContext(self)
    
    def write(self):
        """写锁上下文管理器"""
        return _WriteLockContext(self)


class _ReadLockContext:
    def __init__(self, lock: ReadWriteLock):
        self.lock = lock
    
    def __enter__(self):
        self.lock.acquire_read()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock.release_read()
        return False


class _WriteLockContext:
    def __init__(self, lock: ReadWriteLock):
        self.lock = lock
    
    def __enter__(self):
        self.lock.acquire_write()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock.release_write()
        return False


# 使用示例
"""
from common_utils import thread_safe, ThreadSafeDict, ThreadSafeCounter, ReadWriteLock

class GroundStationNode(Node):
    def __init__(self):
        super().__init__('ground_station')
        
        # 方法1: 使用线程安全字典(推荐)
        self._usv_states = ThreadSafeDict()  # 替换普通字典
        self._usv_nav_target_cache = ThreadSafeDict()
        
        # 方法2: 使用装饰器保护方法
        self._lock = threading.RLock()  # 必须有这个属性
        
        # 方法3: 使用读写锁(读多写少场景)
        self._config_lock = ReadWriteLock()
        self._config = {}
    
    @thread_safe
    def send_nav_goal_via_topic(self, usv_id: str, x: float, y: float):
        '''线程安全方法,自动加锁'''
        self._usv_nav_target_cache[usv_id] = {'x': x, 'y': y}
    
    @thread_safe
    def get_usv_state(self, usv_id: str):
        '''线程安全方法'''
        return self._usv_states.get(usv_id)
    
    def read_config(self, key: str):
        '''读配置(多线程可同时读)'''
        with self._config_lock.read():
            return self._config.get(key)
    
    def update_config(self, key: str, value: Any):
        '''写配置(独占访问)'''
        with self._config_lock.write():
            self._config[key] = value
"""
