# USV项目优化实施指南

**版本**: 1.0  
**日期**: 2025-11-19  
**目标**: 系统化提升项目健壮性、可维护性和可靠性

---

## 📋 目录

1. [工具包概览](#1-工具包概览)
2. [快速开始](#2-快速开始)
3. [逐步迁移指南](#3-逐步迁移指南)
4. [代码审查检查清单](#4-代码审查检查清单)
5. [最佳实践](#5-最佳实践)
6. [常见问题FAQ](#6-常见问题faq)

---

## 1. 工具包概览

### 📦 common_utils 包

新创建的 `common_utils` 包提供四个核心工具类:

| 工具类 | 解决问题 | 典型用途 |
|--------|----------|----------|
| **ParamLoader** | 参数加载不一致、异常吞噬 | 统一参数加载、验证、日志 |
| **SerialResourceManager** | 串口资源泄漏 | 串口生命周期管理 |
| **ProcessTracker** | subprocess僵尸进程 | 子进程追踪和清理 |
| **ThreadSafeDict/thread_safe** | 多线程竞态条件 | 线程安全数据结构和装饰器 |

### 安装和导入

```bash
# 编译包
cd ~/usv_workspace
colcon build --packages-select common_utils
source install/setup.bash
```

```python
# 在节点中导入
from common_utils import (
    ParamLoader,
    SerialResourceManager,
    ProcessTracker,
    thread_safe,
    ThreadSafeDict
)
```

---

## 2. 快速开始

### 示例1: 参数加载改造

**改造前** (问题: 异常吞噬、无日志、无验证):
```python
try:
    self.declare_parameter('publish_rate', 20.0)
    publish_rate = self.get_parameter('publish_rate').value
except Exception:
    publish_rate = 20.0  # ❌ 静默失败
```

**改造后**:
```python
from common_utils import ParamLoader, ParamValidator

loader = ParamLoader(self)
self.publish_rate = loader.load_param(
    'publish_rate',
    20.0,
    ParamValidator.frequency,
    '发布频率(Hz)'
)
# ✅ 自动验证、记录日志、处理异常
```

### 示例2: 串口资源管理

**改造前** (问题: 资源泄漏、异常后节点继续运行):
```python
try:
    self.ser = serial.Serial('/dev/ttyUSB0', 115200)
except serial.SerialException as e:
    self.get_logger().error(f'打开串口失败: {e}')
    # ❌ 异常后节点仍运行但无功能
```

**改造后**:
```python
from common_utils import SerialResourceManager

self.serial_manager = SerialResourceManager(self.get_logger())

if not self.serial_manager.open('/dev/ttyUSB0', 115200):
    self.get_logger().error('初始化串口失败，节点退出')
    raise RuntimeError('Serial port init failed')

# 在 destroy_node() 中
def destroy_node(self):
    self.serial_manager.close()  # ✅ 确保资源释放
    super().destroy_node()
```

### 示例3: 进程管理

**改造前** (问题: 僵尸进程、无追踪):
```python
subprocess.Popen(['aplay', 'sound.wav'])
# ❌ 进程泄漏
```

**改造后**:
```python
from common_utils import ProcessTracker

ProcessTracker.run_and_track(
    ['aplay', 'sound.wav'],
    description='播放声音',
    wait=False
)
# ✅ 自动追踪、程序退出时清理
```

### 示例4: 线程安全

**改造前** (问题: 竞态条件):
```python
class MyNode(Node):
    def __init__(self):
        self._usv_states = {}  # ❌ 多线程访问不安全
    
    def callback1(self, msg):
        self._usv_states[msg.id] = msg.state
    
    def callback2(self):
        for usv_id in self._usv_states.keys():  # ❌ 可能崩溃
            ...
```

**改造后**:
```python
from common_utils import ThreadSafeDict

class MyNode(Node):
    def __init__(self):
        self._usv_states = ThreadSafeDict()  # ✅ 线程安全
    
    def callback1(self, msg):
        self._usv_states[msg.id] = msg.state  # ✅ 自动加锁
    
    def callback2(self):
        for usv_id in self._usv_states.keys():  # ✅ 安全遍历
            ...
```

---

## 3. 逐步迁移指南

### 阶段1: 修复串口资源泄漏 (优先级: 🔴 紧急)

**影响文件**:
- `usv_drivers/usv_ultrasonic_node.py` ✅ 已完成
- `usv_drivers/usv_uwb_node.py`
- `usv_drivers/usv_lidar_node.py`
- `usv_led/usv_led_node.py`

**迁移步骤**:

1. **添加依赖**:
   在 `package.xml` 中添加:
   ```xml
   <depend>common_utils</depend>
   ```

2. **导入工具**:
   ```python
   from common_utils import SerialResourceManager, ParamLoader, ParamValidator
   ```

3. **替换串口初始化**:
   ```python
   # 旧代码
   try:
       self.ser = serial.Serial(port, baudrate)
   except Exception as e:
       self.get_logger().error(...)
       raise
   
   # 新代码
   self.serial_manager = SerialResourceManager(self.get_logger())
   if not self.serial_manager.open(port, baudrate):
       raise RuntimeError('Serial init failed')
   ```

4. **替换读写操作**:
   ```python
   # self.ser.read(10) → self.serial_manager.read(10)
   # self.ser.write(data) → self.serial_manager.write(data)
   # self.ser.readline() → self.serial_manager.readline()
   ```

5. **添加清理**:
   ```python
   def destroy_node(self):
       self.serial_manager.close()
       super().destroy_node()
   ```

**验证**:
```bash
# 编译并测试
colcon build --packages-select usv_drivers
ros2 run usv_drivers usv_ultrasonic_node
# Ctrl+C 退出,检查日志确认串口关闭
```

---

### 阶段2: 统一参数加载 (优先级: 🟠 重要)

**影响文件**: 所有节点

**迁移步骤**:

1. **识别参数加载代码**:
   搜索模式: `declare_parameter.*try.*except`

2. **使用 ParamLoader 替换**:
   ```python
   loader = ParamLoader(self)
   
   # 单个参数
   self.rate = loader.load_param('rate', 20.0, ParamValidator.positive)
   
   # 批量参数
   config = {
       'timeout': {'default': 5.0, 'validator': ParamValidator.timeout},
       'port': {'default': 8080, 'validator': ParamValidator.port_number}
   }
   params = loader.load_params(config)
   
   # GPS原点(专用方法)
   gps = loader.load_gps_origin(22.5180977, 113.9007239, -5.17)
   ```

3. **删除重复的GPS原点定义**:
   - 保留 `usv_params.yaml` 中的定义
   - 所有节点通过 `load_gps_origin()` 加载
   - 删除硬编码值

---

### 阶段3: 进程管理 (优先级: 🟠 重要)

**影响文件**:
- `usv_sound/usv_sound_node.py`
- `gs_gui/main_gui_app.py`
- `gs_gui/usv_fleet_launcher*.py`

**迁移步骤**:

1. **替换 subprocess 调用**:
   ```python
   # 旧代码
   proc = subprocess.Popen(['aplay', 'sound.wav'])
   
   # 新代码
   from common_utils import ProcessTracker
   ProcessTracker.run_and_track(
       ['aplay', 'sound.wav'],
       description='播放声音',
       wait=False
   )
   ```

2. **节点关闭时清理**:
   ```python
   def destroy_node(self):
       ProcessTracker.cleanup_all()
       super().destroy_node()
   ```

---

### 阶段4: 线程安全 (优先级: 🟠 重要)

**影响文件**:
- `gs_gui/ground_station_node.py`
- `gs_gui/cluster_controller.py`

**迁移步骤**:

1. **识别共享状态**:
   - 字典: `_usv_states`, `_usv_nav_target_cache`, `_goal_to_usv`
   - 列表: `_usv_list`
   - 计数器: `_goal_id_counter`

2. **替换为线程安全版本**:
   ```python
   from common_utils import ThreadSafeDict, ThreadSafeCounter
   
   # 旧代码
   self._usv_states = {}
   self._goal_id_counter = 0
   
   # 新代码
   self._usv_states = ThreadSafeDict()
   self._goal_id_counter = ThreadSafeCounter()
   ```

3. **或使用装饰器**:
   ```python
   from common_utils import thread_safe
   
   class MyNode(Node):
       def __init__(self):
           self._lock = threading.RLock()
           self._data = {}
       
       @thread_safe
       def update_data(self, key, value):
           self._data[key] = value  # 自动加锁
   ```

---

## 4. 代码审查检查清单

### ✅ 参数加载

- [ ] 所有 `declare_parameter` 有对应的异常处理
- [ ] 参数加载失败时有日志记录
- [ ] 关键参数有验证(范围、类型)
- [ ] GPS原点配置来自统一位置

### ✅ 资源管理

- [ ] 串口使用 `SerialResourceManager`
- [ ] 所有资源在 `destroy_node()` 中释放
- [ ] 文件句柄使用 `with` 语句或确保关闭
- [ ] subprocess 通过 `ProcessTracker` 管理

### ✅ 线程安全

- [ ] 多线程访问的字典/列表使用线程安全版本
- [ ] 共享状态有明确的锁保护
- [ ] GUI更新通过信号/槽机制

### ✅ 错误处理

- [ ] 避免裸 `except Exception: pass`
- [ ] 异常有明确的类型(`ConnectionError`, `ValueError`)
- [ ] 错误日志包含上下文信息
- [ ] 关键路径有恢复策略

### ✅ 日志规范

- [ ] 使用正确的日志级别(`debug/info/warn/error`)
- [ ] 避免使用 `print()` (测试除外)
- [ ] 日志格式统一
- [ ] 关键操作有日志记录

---

## 5. 最佳实践

### 原则1: 参数化而非硬编码

```python
# ❌ 避免
TIMEOUT = 300.0
GPS_ORIGIN_LAT = 22.5180977

# ✅ 推荐
self.timeout = loader.load_param('timeout', 300.0)
gps = loader.load_gps_origin()
```

### 原则2: 资源必须释放

```python
# ❌ 避免
def __init__(self):
    self.ser = serial.Serial(...)

# ✅ 推荐
def __init__(self):
    self.serial_manager = SerialResourceManager(...)
    self.serial_manager.open(...)

def destroy_node(self):
    self.serial_manager.close()
    super().destroy_node()
```

### 原则3: 线程安全优先

```python
# ❌ 避免
self._cache = {}  # 多线程不安全

# ✅ 推荐
self._cache = ThreadSafeDict()  # 自动线程安全
```

### 原则4: 异常要分类

```python
# ❌ 避免
except Exception:
    pass

# ✅ 推荐
except ConnectionError as e:
    logger.error(f"网络错误: {e}")
    self.reconnect()
except ValueError as e:
    logger.warn(f"数据格式错误: {e}")
except Exception as e:
    logger.critical(f"未知错误: {e}", exc_info=True)
    raise
```

### 原则5: 日志要有意义

```python
# ❌ 避免
self.get_logger().debug('发送完成')

# ✅ 推荐
self.get_logger().info(f'发送导航目标到 {usv_id}: ({x:.2f}, {y:.2f})')
```

---

## 6. 常见问题FAQ

### Q1: 如何判断哪些节点需要优先修复?

**A**: 按以下优先级:
1. 🔴 **P0-紧急**: 资源泄漏节点(所有驱动、LED、声音)
2. 🟠 **P1-重要**: 核心功能节点(control, comm, ground_station)
3. 🟡 **P2-中等**: 辅助节点(tf, fan)

### Q2: 修改后如何验证?

**A**: 三步验证:
```bash
# 1. 编译检查
colcon build --packages-select <package_name>

# 2. 静态检查
ros2 run ament_flake8 <package_name>

# 3. 运行测试
ros2 run <package_name> <node_name>
# 观察日志,检查资源正常加载和释放
```

### Q3: ThreadSafeDict 性能如何?

**A**: 
- 读写都有锁开销,但对于 ROS 2 回调频率(通常<100Hz)影响极小
- 如果是高频操作(>1kHz),考虑使用 `ReadWriteLock`

### Q4: 是否需要修改所有节点?

**A**: 
- **必须修复**: 资源泄漏问题(串口、进程)
- **强烈建议**: 参数加载、线程安全
- **可选**: 其他节点按需逐步迁移

### Q5: 如何处理现有的 error_handler.py?

**A**: 
- `common_utils` 与现有 `error_handler.py` 互补
- `error_handler.py` 专注错误分类和恢复策略
- `common_utils` 专注资源和线程管理
- 可以组合使用

---

## 附录A: 完整迁移示例

### 迁移前后对比: usv_ultrasonic_node.py

**迁移前**问题:
- ❌ 参数硬编码
- ❌ 串口异常后节点继续运行
- ❌ 资源泄漏风险
- ❌ `__del__` 不可靠

**迁移后**优势:
- ✅ 参数化配置
- ✅ 初始化失败时节点退出
- ✅ 确保资源释放
- ✅ 日志完整

详见: `/usv_drivers/usv_drivers/usv_ultrasonic_node.py` (已完成)

---

## 附录B: 工具API参考

### ParamLoader API

```python
loader = ParamLoader(node)

# 基础加载
value = loader.load_param(name, default, validator, description)

# 批量加载
params = loader.load_params(config_dict)

# GPS专用
gps = loader.load_gps_origin(lat, lon, alt)
```

### SerialResourceManager API

```python
manager = SerialResourceManager(logger)

# 打开串口
success = manager.open(port, baudrate, timeout)

# 读写操作
data = manager.read(size)
success = manager.write(data)
line = manager.readline()

# 检查状态
if manager.is_open: ...

# 关闭
manager.close()

# 上下文管理器
with manager.managed_serial(port, baudrate) as ser:
    if ser:
        manager.read(10)
```

### ProcessTracker API

```python
# 追踪进程
proc = subprocess.Popen(['cmd'])
ProcessTracker.track(proc, 'description')

# 便捷方法
ProcessTracker.run_and_track(
    ['cmd', 'arg'],
    description='desc',
    wait=False
)

# 终止进程
ProcessTracker.terminate(pid, timeout=5.0)

# 清理所有
ProcessTracker.cleanup_all()
```

### ThreadSafeDict API

```python
cache = ThreadSafeDict()

# 基础操作(自动加锁)
cache['key'] = value
value = cache['key']
value = cache.get('key', default)
del cache['key']

# 迭代(快照)
for key in cache.keys(): ...
for value in cache.values(): ...
for k, v in cache.items(): ...
```

---

**文档版本**: 1.0  
**最后更新**: 2025-11-19  
**维护者**: USV项目团队
