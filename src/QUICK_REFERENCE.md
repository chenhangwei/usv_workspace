# USV项目优化快速参考

## 🚀 立即修复的问题

### 1. 串口资源泄漏 🔴
```python
# ❌ 问题代码
try:
    self.ser = serial.Serial(port, baud)
except Exception as e:
    self.get_logger().error(...)  # 节点继续运行但无功能

# ✅ 修复方案
from common_utils import SerialResourceManager

self.serial_manager = SerialResourceManager(self.get_logger())
if not self.serial_manager.open(port, baud):
    raise RuntimeError('Serial init failed')

def destroy_node(self):
    self.serial_manager.close()
    super().destroy_node()
```

**影响文件**: `usv_drivers/*_node.py`, `usv_led_node.py`

---

### 2. 参数加载异常吞噬 🟠
```python
# ❌ 问题代码
try:
    self.declare_parameter('rate', 20.0)
    rate = self.get_parameter('rate').value
except Exception:
    rate = 20.0  # 静默失败,无日志

# ✅ 修复方案
from common_utils import ParamLoader, ParamValidator

loader = ParamLoader(self)
self.rate = loader.load_param('rate', 20.0, ParamValidator.positive)
```

**影响文件**: 所有节点

---

### 3. subprocess 僵尸进程 🟠
```python
# ❌ 问题代码
subprocess.Popen(['aplay', 'sound.wav'])  # 进程泄漏

# ✅ 修复方案
from common_utils import ProcessTracker

ProcessTracker.run_and_track(
    ['aplay', 'sound.wav'],
    description='播放声音',
    wait=False
)

def destroy_node(self):
    ProcessTracker.cleanup_all()
    super().destroy_node()
```

**影响文件**: `usv_sound_node.py`, `main_gui_app.py`, `usv_fleet_launcher*.py`

---

### 4. 多线程竞态条件 🟠
```python
# ❌ 问题代码
self._usv_states = {}  # 多线程不安全
def callback(self):
    self._usv_states[id] = state  # 竞态条件

# ✅ 修复方案
from common_utils import ThreadSafeDict

self._usv_states = ThreadSafeDict()  # 自动线程安全
def callback(self):
    self._usv_states[id] = state  # 自动加锁
```

**影响文件**: `ground_station_node.py`, `cluster_controller.py`

---

## 📦 工具包使用

### 安装
```bash
cd ~/usv_workspace
colcon build --packages-select common_utils
source install/setup.bash
```

### 在package.xml中添加依赖
```xml
<depend>common_utils</depend>
```

### 导入
```python
from common_utils import (
    ParamLoader,
    ParamValidator,
    SerialResourceManager,
    ProcessTracker,
    ThreadSafeDict,
    thread_safe
)
```

---

## 🔍 代码审查检查点

- [ ] 所有串口使用 `SerialResourceManager`
- [ ] 所有参数加载使用 `ParamLoader`
- [ ] subprocess 通过 `ProcessTracker` 管理
- [ ] 多线程共享数据使用 `ThreadSafeDict`
- [ ] 没有裸 `except Exception: pass`
- [ ] 资源在 `destroy_node()` 中释放
- [ ] GPS原点统一加载(不硬编码)
- [ ] 日志使用正确级别(不用print)

---

## 📊 优先级矩阵

| 问题 | 严重性 | 影响范围 | 修复成本 | 优先级 |
|------|--------|---------|---------|--------|
| 串口资源泄漏 | 高 | 6个文件 | 低 | 🔴 P0 |
| 进程僵尸 | 中 | 3个文件 | 低 | 🟠 P1 |
| 线程安全 | 中 | 2个文件 | 中 | 🟠 P1 |
| 参数加载 | 低 | 所有 | 低 | 🟡 P2 |

---

## 🎯 本周目标

### Day 1-2: 修复所有驱动节点
- [x] usv_ultrasonic_node.py ✅
- [ ] usv_uwb_node.py
- [ ] usv_lidar_node.py  
- [ ] usv_led_node.py

### Day 3: 修复进程管理
- [ ] usv_sound_node.py
- [ ] usv_fleet_launcher*.py

### Day 4-5: 增强线程安全
- [ ] ground_station_node.py
- [ ] cluster_controller.py

---

## 📚 完整文档

详见: `/home/chenhangwei/usv_workspace/src/OPTIMIZATION_GUIDE.md`

---

**快速参考版本**: 1.0  
**最后更新**: 2025-11-19
