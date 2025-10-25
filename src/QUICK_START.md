# 🚀 优化快速开始指南

> **适用场景**: 立即应用最关键的优化，快速提升系统稳定性  
> **预计时间**: 2-4 小时  
> **前置要求**: 熟悉 Python、ROS 2 基础

---

## 第一步：集成错误处理模块（30分钟）

### 1.1 在 GroundStationNode 中集成

```bash
# 编辑文件
cd ~/usv_workspace/src/gs_gui/gs_gui
nano ground_station_node.py
```

在文件开头添加导入：
```python
from .error_handler import (
    RobustErrorHandler, 
    ErrorCategory, 
    ErrorSeverity,
    safe_execute
)
```

在 `__init__` 方法中初始化：
```python
def __init__(self, signal):
    super().__init__('groundstationnode')
    # ... 现有代码 ...
    
    # 添加错误处理器
    self.error_handler = RobustErrorHandler(self.get_logger())
```

### 1.2 替换关键路径的异常处理

**示例 1: send_nav_goal_via_action**
```python
# 找到方法（约第340行）
def send_nav_goal_via_action(self, usv_id, x, y, z=0.0, yaw=0.0, timeout=300.0):
    # 检查USV是否存在
    if usv_id not in self.usv_manager.navigate_to_point_clients:
        # 旧代码：
        # self.get_logger().error(f"未找到USV {usv_id} 的导航客户端")
        
        # 新代码：
        self.error_handler.handle_error(
            ValueError(f"导航客户端不存在: {usv_id}"),
            context=f"发送导航目标到 {usv_id}",
            severity=ErrorSeverity.ERROR,
            category=ErrorCategory.STATE
        )
        self.ros_signal.nav_status_update.emit(usv_id, "失败")
        return False
```

**示例 2: usv_state_callback（在 usv_manager.py）**
```python
@safe_execute(
    error_handler,  # 需要传入
    context="处理 USV 状态回调",
    category=ErrorCategory.NETWORK,
    severity=ErrorSeverity.WARNING
)
def usv_state_callback(self, msg, usv_id):
    # 原有逻辑不变
    pass
```

### 1.3 测试
```bash
# 重新构建
cd ~/usv_workspace
colcon build --packages-select gs_gui

# 启动并观察日志
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

---

## 第二步：改进资源清理（30分钟）

### 2.1 修改 GroundStationNode

在 `__init__` 中添加资源管理器：
```python
from .resource_manager import ResourceManager, ThreadManager

def __init__(self, signal):
    super().__init__('groundstationnode')
    # ... 现有代码 ...
    
    # 添加资源管理器
    self.resource_manager = ResourceManager(self.get_logger())
    self.thread_manager = ThreadManager(self.get_logger())
    
    # 注册发布队列
    self.resource_manager.register_resource(
        "publish_queue",
        self.publish_queue,
        cleanup_func=lambda q: self._cleanup_queue(q)
    )
    
    # 重构启动线程的代码
    # 旧代码：
    # self.publish_thread = threading.Thread(target=self.process_publish_queue, daemon=True)
    # self.publish_thread.start()
    
    # 新代码：
    def publish_worker(stop_event):
        while not stop_event.is_set():
            try:
                pub, msg = self.publish_queue.get(timeout=1.0)
                if pub and msg:
                    pub.publish(msg)
                    self.publish_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.error_handler.handle_error(
                    e, 
                    "发布队列处理",
                    ErrorSeverity.ERROR,
                    ErrorCategory.UNKNOWN
                )
    
    self.thread_manager.start_thread(
        "publish_worker",
        target=publish_worker,
        daemon=True
    )
```

添加队列清理方法：
```python
def _cleanup_queue(self, q):
    """清空队列"""
    try:
        while not q.empty():
            q.get_nowait()
            q.task_done()
        self.get_logger().info("发布队列已清空")
    except Exception as e:
        self.get_logger().error(f"清空队列失败: {e}")
```

### 2.2 改进 shutdown 方法

```python
def shutdown(self):
    """优雅停止节点"""
    self.get_logger().info('GroundStationNode 正在关闭...')
    
    try:
        # 1. 停止所有线程
        if hasattr(self, 'thread_manager'):
            self.thread_manager.stop_all(timeout=3.0)
        
        # 2. 清理所有资源
        if hasattr(self, 'resource_manager'):
            self.resource_manager.cleanup_all()
        
        # 3. 其他清理逻辑保持不变
        # ...
        
    except Exception as e:
        self.get_logger().error(f"关闭过程中发生错误: {e}")
```

### 2.3 在 main() 中调用 shutdown

在 `main_gui_app.py` 中：
```python
def main(argv=None):
    # ... 现有代码 ...
    
    try:
        exit_code = app.exec_()
    finally:
        # 确保调用 shutdown
        try:
            if hasattr(node, 'shutdown'):
                node.shutdown()
        except Exception as e:
            print(f"调用 shutdown() 时出错: {e}")
        
        # 原有的销毁逻辑
        try:
            node.destroy_node()
        except Exception as e:
            print(f"销毁节点时出错: {e}")
        # ...
```

---

## 第三步：配置统一日志（20分钟）

### 3.1 在包初始化时设置日志

编辑 `gs_gui/__init__.py`：
```python
"""Ground Station GUI 包初始化"""

from .logger_config import LoggerConfig
import logging
import os

# 配置日志
_log_dir = os.path.abspath(os.path.join(os.getcwd(), '.logs'))
logger = LoggerConfig.setup_logger(
    name='gs_gui',
    log_dir=_log_dir,
    level=logging.INFO,
    console_output=True,
    file_output=True,
    max_bytes=10 * 1024 * 1024,  # 10MB
    backup_count=5,
    json_format=False
)

__all__ = ['logger']
```

### 3.2 在模块中使用日志

在所有模块中替换日志调用：
```python
# 旧代码
self.get_logger().info("...")

# 新代码（推荐同时保留ROS日志）
import logging
logger = logging.getLogger('gs_gui')

self.get_logger().info("...")  # ROS日志（仍然保留）
logger.info("...", extra={'usv_id': usv_id})  # 统一日志
```

### 3.3 使用性能监控上下文

```python
from gs_gui.logger_config import LogContext
import logging

logger = logging.getLogger('gs_gui')

def send_nav_goal(self, usv_id, x, y, z):
    with LogContext(logger, f"发送导航目标到 {usv_id}", level=logging.INFO):
        # 原有逻辑
        result = self._do_send_goal(usv_id, x, y, z)
        return result
    # 自动记录耗时
```

---

## 第四步：测试和验证（40分钟）

### 4.1 单元测试

创建测试文件 `gs_gui/test/test_error_handler.py`:
```python
import pytest
from gs_gui.error_handler import RobustErrorHandler, ErrorCategory, ErrorSeverity

def test_error_handling():
    """测试错误处理基本功能"""
    handler = RobustErrorHandler()
    
    # 模拟错误
    try:
        raise ConnectionError("测试连接错误")
    except Exception as e:
        result = handler.handle_error(
            e,
            context="测试上下文",
            severity=ErrorSeverity.ERROR,
            category=ErrorCategory.NETWORK
        )
        assert not result  # 无恢复动作返回 False

def test_error_counting():
    """测试错误计数"""
    handler = RobustErrorHandler()
    
    # 连续记录多个错误
    for i in range(12):
        try:
            raise ValueError(f"错误 {i}")
        except Exception as e:
            handler.handle_error(
                e,
                context="测试计数",
                severity=ErrorSeverity.WARNING,
                category=ErrorCategory.PARAMETER
            )
    
    # 验证错误计数
    error_key = "parameter:测试计数"
    assert error_key in handler._error_counts
    assert handler._error_counts[error_key] == 12
```

运行测试：
```bash
cd ~/usv_workspace/src/gs_gui
python3 -m pytest test/test_error_handler.py -v
```

### 4.2 集成测试

启动系统并观察日志：
```bash
# 终端1: 启动地面站
source ~/usv_workspace/install/setup.bash
ros2 launch gs_bringup gs_launch.py

# 终端2: 观察日志文件
tail -f ~/.logs/gs_gui.log

# 终端3: 模拟USV（可选）
ros2 launch usv_bringup usv_launch.py namespace:=usv_01
```

### 4.3 验证检查清单

- [ ] **错误日志格式统一**  
  检查 `.logs/gs_gui.log` 文件，确认格式为：
  ```
  2025-10-24 14:30:15 [    INFO] [gs_gui] [ground_station_node.py:100] - 消息内容
  ```

- [ ] **错误分类正确**  
  触发一个错误（如断开网络），检查日志中是否有：
  ```
  [ERROR] [network] 发送导航目标到 usv_01
  错误: ConnectionError: ...
  ```

- [ ] **资源清理生效**  
  - 关闭地面站（Ctrl+C）
  - 检查是否有 "GroundStationNode 正在关闭..." 日志
  - 检查是否有 "停止线程: publish_worker" 日志
  - 检查是否有 "所有资源清理完成" 日志

- [ ] **日志轮转工作**  
  检查 `.logs` 目录，应该有：
  ```
  gs_gui.log
  gs_gui_error.log
  gs_gui.log.1 (如果日志超过10MB)
  ```

- [ ] **性能监控生效**  
  在日志中查找带 `[SLOW]` 标记的操作（如果有慢操作）

---

## 第五步：性能优化（60分钟，可选）

### 5.1 添加性能监控

创建 `gs_gui/gs_gui/metrics_collector.py`:
```python
"""性能指标收集模块"""

import time
from collections import deque
from typing import Dict
import statistics


class MetricsCollector:
    """性能指标收集器"""
    
    def __init__(self, window_size: int = 100):
        self._metrics: Dict[str, deque] = {}
        self._window_size = window_size
    
    def record(self, metric_name: str, value: float):
        """记录指标值"""
        if metric_name not in self._metrics:
            self._metrics[metric_name] = deque(maxlen=self._window_size)
        self._metrics[metric_name].append(value)
    
    def get_stats(self, metric_name: str) -> Dict[str, float]:
        """获取指标统计"""
        if metric_name not in self._metrics or not self._metrics[metric_name]:
            return {}
        
        values = list(self._metrics[metric_name])
        return {
            'count': len(values),
            'mean': statistics.mean(values),
            'median': statistics.median(values),
            'min': min(values),
            'max': max(values)
        }


class PerformanceTimer:
    """性能计时器"""
    
    def __init__(self, metrics_collector: MetricsCollector, metric_name: str):
        self.metrics_collector = metrics_collector
        self.metric_name = metric_name
        self.start_time = None
    
    def __enter__(self):
        self.start_time = time.time()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.start_time:
            duration_ms = (time.time() - self.start_time) * 1000
            self.metrics_collector.record(self.metric_name, duration_ms)
        return False
```

### 5.2 在关键路径中使用

在 `ground_station_node.py` 中：
```python
from .metrics_collector import MetricsCollector, PerformanceTimer

def __init__(self, signal):
    # ...
    self.metrics = MetricsCollector(window_size=100)
    
    # 添加定时器定期报告
    self.create_timer(60.0, self._report_metrics)

def send_nav_goal_via_action(self, usv_id, x, y, z=0.0, yaw=0.0, timeout=300.0):
    with PerformanceTimer(self.metrics, "nav_goal_send"):
        # 原有逻辑
        pass

def _report_metrics(self):
    """定期报告性能指标"""
    stats = self.metrics.get_stats("nav_goal_send")
    if stats:
        self.get_logger().info(
            f"导航目标发送性能: "
            f"平均={stats['mean']:.2f}ms, "
            f"中位数={stats['median']:.2f}ms, "
            f"最大={stats['max']:.2f}ms"
        )
```

---

## 常见问题

### Q1: 编译时出现导入错误？
```bash
# 确保已安装所有依赖
pip3 install -r requirements.txt

# 重新构建
cd ~/usv_workspace
colcon build --packages-select gs_gui --cmake-clean-cache
source install/setup.bash
```

### Q2: 日志文件在哪里？
```bash
# 默认位置
ls -lh ~/.logs/

# 或在工作空间根目录
ls -lh ~/usv_workspace/.logs/
```

### Q3: 如何调整日志级别？
编辑 `gs_gui/__init__.py`:
```python
logger = LoggerConfig.setup_logger(
    name='gs_gui',
    level=logging.DEBUG,  # 改为 DEBUG 级别
    # ...
)
```

### Q4: 如何禁用某个优化？
```python
# 如果不想使用错误处理器，简单注释掉初始化即可
# self.error_handler = RobustErrorHandler(self.get_logger())

# 如果不想使用资源管理器
# self.resource_manager = ResourceManager(self.get_logger())
```

---

## 下一步

完成以上步骤后，您的系统应该已经具备：
- ✅ 健壮的错误处理和恢复机制
- ✅ 完善的资源管理和清理
- ✅ 统一的日志系统
- ✅ 基础性能监控

**建议继续阅读：**
1. `OPTIMIZATION_GUIDE.md` - 完整优化指南
2. `OPTIMIZATION_SUMMARY.md` - 详细分析报告
3. `.github/copilot-instructions.md` - 项目架构文档

**进一步优化：**
1. 添加更多单元测试（目标70%覆盖率）
2. 实施配置管理服务
3. 优化 GUI 性能（差量更新）
4. 集成代码质量工具（pylint, mypy）

---

**遇到问题？**
- 查看日志文件：`~/.logs/gs_gui.log`
- 查看错误日志：`~/.logs/gs_gui_error.log`
- 提交 Issue：GitHub Issues
- 查看文档：项目 docs/ 目录

**祝您优化顺利！** 🚀
