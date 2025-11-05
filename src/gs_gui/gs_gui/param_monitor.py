"""
参数监控模块

提供参数变化监控、日志记录和历史追踪功能
"""

from datetime import datetime
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from .param_manager import ParamInfo


@dataclass
class ParamChangeEvent:
    """参数变化事件"""
    timestamp: datetime
    param_name: str
    old_value: float
    new_value: float
    source: str = "manual"  # "manual", "import", "sync"
    
    @property
    def change_delta(self) -> float:
        """变化量"""
        return self.new_value - self.old_value
    
    @property
    def change_percent(self) -> Optional[float]:
        """变化百分比"""
        if self.old_value == 0:
            return None
        return (self.change_delta / self.old_value) * 100


class ParamMonitor:
    """
    参数监控器
    
    监控参数变化并记录历史
    """
    
    def __init__(self, max_history: int = 1000):
        """
        初始化监控器
        
        Args:
            max_history: 最大历史记录数
        """
        self.max_history = max_history
        self.change_history: List[ParamChangeEvent] = []
        self.change_callbacks: List[Callable[[ParamChangeEvent], None]] = []
        self._watched_params: Dict[str, float] = {}  # {param_name: last_value}
    
    def register_callback(self, callback: Callable[[ParamChangeEvent], None]):
        """注册变化回调"""
        self.change_callbacks.append(callback)
    
    def unregister_callback(self, callback: Callable[[ParamChangeEvent], None]):
        """取消注册回调"""
        if callback in self.change_callbacks:
            self.change_callbacks.remove(callback)
    
    def watch_param(self, param_name: str, current_value: float):
        """开始监控参数"""
        self._watched_params[param_name] = current_value
    
    def unwatch_param(self, param_name: str):
        """停止监控参数"""
        if param_name in self._watched_params:
            del self._watched_params[param_name]
    
    def check_changes(self, params: Dict[str, ParamInfo], source: str = "manual"):
        """
        检查参数变化
        
        Args:
            params: 当前参数字典
            source: 变化来源
        """
        for param_name, old_value in self._watched_params.items():
            if param_name not in params:
                continue
            
            new_value = params[param_name].value
            
            # 检测变化
            if abs(new_value - old_value) > 1e-9:
                event = ParamChangeEvent(
                    timestamp=datetime.now(),
                    param_name=param_name,
                    old_value=old_value,
                    new_value=new_value,
                    source=source
                )
                
                # 记录事件
                self._record_event(event)
                
                # 触发回调
                self._notify_callbacks(event)
                
                # 更新监控值
                self._watched_params[param_name] = new_value
    
    def _record_event(self, event: ParamChangeEvent):
        """记录事件"""
        self.change_history.append(event)
        
        # 限制历史记录数量
        if len(self.change_history) > self.max_history:
            self.change_history = self.change_history[-self.max_history:]
    
    def _notify_callbacks(self, event: ParamChangeEvent):
        """通知回调"""
        for callback in self.change_callbacks:
            try:
                callback(event)
            except Exception as e:
                print(f"回调执行失败: {e}")
    
    def get_param_history(self, param_name: str) -> List[ParamChangeEvent]:
        """获取参数变化历史"""
        return [e for e in self.change_history if e.param_name == param_name]
    
    def get_recent_changes(self, limit: int = 10) -> List[ParamChangeEvent]:
        """获取最近的变化"""
        return self.change_history[-limit:]
    
    def get_change_count(self, param_name: Optional[str] = None) -> int:
        """获取变化次数"""
        if param_name is None:
            return len(self.change_history)
        return len([e for e in self.change_history if e.param_name == param_name])
    
    def clear_history(self):
        """清除历史记录"""
        self.change_history.clear()
    
    def export_log(self, filepath: str):
        """导出日志到文件"""
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write("# 参数变化日志\n")
                f.write(f"# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 总变化次数: {len(self.change_history)}\n")
                f.write("#\n")
                f.write("时间戳,参数名,旧值,新值,变化量,变化百分比,来源\n")
                
                for event in self.change_history:
                    percent = f"{event.change_percent:.2f}%" if event.change_percent is not None else "N/A"
                    f.write(
                        f"{event.timestamp.strftime('%Y-%m-%d %H:%M:%S')},"
                        f"{event.param_name},"
                        f"{event.old_value:.6g},"
                        f"{event.new_value:.6g},"
                        f"{event.change_delta:.6g},"
                        f"{percent},"
                        f"{event.source}\n"
                    )
            return True
        except Exception as e:
            print(f"导出日志失败: {e}")
            return False
