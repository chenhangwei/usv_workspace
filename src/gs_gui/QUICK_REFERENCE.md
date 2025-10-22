# Ground Station GUI 重构 - 快速参考指南

## 📁 文件清单

| 文件名 | 大小 | 行数 | 功能 |
|--------|------|------|------|
| `main_gui_app.py` | 19KB | ~400 | 主窗口，整合所有模块 |
| `table_manager.py` | 11KB | ~250 | 表格管理（集群/离群） |
| `usv_commands.py` | 9.3KB | ~200 | USV命令处理 |
| `cluster_task_manager.py` | 13KB | ~220 | 集群任务管理 |
| `usv_list_manager.py` | 5.8KB | ~150 | USV列表管理 |
| `state_handler.py` | 4.0KB | ~120 | 状态接收和缓存 |
| `ui_utils.py` | 7.3KB | ~170 | UI辅助工具 |
| **总计** | **~69KB** | **~1510** | **原文件70KB/1200行** |

## 🔧 常用操作速查

### 添加新的USV命令

**位置**: `usv_commands.py`

```python
def your_new_command(self, usv_list):
    """新命令说明"""
    try:
        namespace_list = self._extract_namespaces(usv_list)
        self.ros_signal.your_command.emit(namespace_list)
        self.append_info(f"命令已发送: {namespace_list}")
    except Exception as e:
        self.append_info(f"发送命令失败: {e}")
```

**然后在** `main_gui_app.py`:
```python
def your_new_command_wrapper(self):
    """命令包装"""
    self.command_handler.your_new_command(self.list_manager.usv_cluster_list)
```

**最后连接按钮**:
```python
self.ui.your_button.clicked.connect(self.your_new_command_wrapper)
```

---

### 修改表格显示

**位置**: `table_manager.py`

修改 `TABLE_HEADERS` 常量：
```python
TABLE_HEADERS = ["编号", "模式", "状态", ...]  # 添加或删除列
```

修改 `_format_table_cells()` 方法：
```python
cells = [
    ns,
    state.get('mode'),
    # 添加新的列数据
    your_new_data,
]
```

---

### 添加新的任务类型

1. **创建新管理器** (参考 `cluster_task_manager.py`):
```python
class YourTaskManager:
    def __init__(self, ros_signal, info_callback):
        self.ros_signal = ros_signal
        self.append_info = info_callback
        
    def start_task(self):
        # 任务逻辑
        pass
```

2. **在主窗口中集成**:
```python
# __init__ 中
self.your_task_manager = YourTaskManager(
    self.ros_signal,
    self.ui_utils.append_info
)
```

---

### 修改状态更新频率

**位置**: `state_handler.py`

```python
self._ui_refresh_timer.setInterval(200)  # 毫秒，默认200
```

**位置**: `ui_utils.py`

```python
self._info_flush_interval_ms = 500  # 毫秒，默认500
```

---

### 调整日志最大行数

**位置**: `ui_utils.py`

```python
self._info_max_lines = 500  # 默认500行
```

---

## 🐛 常见问题排查

### 问题1: 表格不更新

**检查点**:
1. `StateHandler` 的定时器是否启动
2. `receive_state_callback` 是否被调用
3. `_usv_state_dirty` 标志是否设置

**调试代码**:
```python
# 在 state_handler.py 的 receive_state_callback 中添加
print(f"收到 {len(msg)} 个USV状态")

# 在 _flush_state_cache_to_ui 中添加
print(f"刷新UI: {len(self._usv_state_cache)} 个USV")
```

---

### 问题2: 命令无响应

**检查点**:
1. ROS信号是否正确连接
2. 命名空间列表是否为空
3. ROS节点是否接收到信号

**调试代码**:
```python
# 在 usv_commands.py 的命令方法中添加
print(f"发送命令到: {namespace_list}")
```

---

### 问题3: 任务启动失败

**检查点**:
1. XML文件格式是否正确
2. `cluster_position_list` 是否为空
3. 离群列表过滤是否正确

**调试代码**:
```python
# 在 cluster_task_manager.py 的 start_task 中添加
print(f"任务数据: {len(filtered_list)} 个目标点")
print(f"离群列表: {departed_ids}")
```

---

### 问题4: UI卡顿

**可能原因**:
1. 定时器间隔太短
2. 日志输出太频繁
3. 表格更新范围太大

**解决方案**:
```python
# 增加定时器间隔
self._ui_refresh_timer.setInterval(300)  # 从200改为300

# 减少日志批量大小
max_batch = 30  # 从50改为30

# 使用差量更新（已实现）
```

---

## 📊 模块职责快速对照

| 功能 | 负责模块 | 主要方法 |
|------|---------|---------|
| 显示USV列表 | TableManager | `update_cluster_table()` |
| 发送控制命令 | USVCommandHandler | `set_cluster_arming()` 等 |
| 管理USV分组 | USVListManager | `add_to_cluster()` |
| 接收状态更新 | StateHandler | `receive_state_callback()` |
| 执行集群任务 | ClusterTaskManager | `start_task()` |
| 显示日志信息 | UIUtils | `append_info()` |
| 协调所有模块 | MainWindow | 各种包装方法 |

---

## 🔄 数据流向图

### 状态更新
```
ROS → StateHandler → (缓存) → (200ms) → ListManager → TableManager → UI
```

### 命令发送
```
UI按钮 → MainWindow → CommandHandler → ROS信号 → ROS节点
```

### 任务执行
```
XML文件 → TaskManager → (解析) → (确认) → ROS信号 → ROS节点
                              ↓
                         进度更新 → UI
```

---

## 🛠️ 开发工作流

### 添加新功能
1. **确定模块**: 判断新功能属于哪个模块
2. **实现功能**: 在对应模块中添加方法
3. **集成到主窗口**: 在 `main_gui_app.py` 中添加包装
4. **连接UI**: 连接按钮或菜单项
5. **测试**: 单元测试 + 集成测试

### 修改现有功能
1. **定位模块**: 找到对应的模块文件
2. **修改代码**: 只修改该模块
3. **检查依赖**: 确认其他模块是否受影响
4. **测试**: 回归测试

### 调试问题
1. **确定层级**: 判断问题在哪一层（UI/逻辑/ROS）
2. **添加日志**: 在关键位置添加调试输出
3. **逐步跟踪**: 跟踪数据流向
4. **修复问题**: 在正确的模块中修复

---

## 📝 代码规范

### 命名约定
- **类名**: `PascalCase` (如 `TableManager`)
- **方法名**: `snake_case` (如 `update_cluster_table`)
- **私有方法**: `_snake_case` (如 `_flush_state_cache`)
- **常量**: `UPPER_SNAKE_CASE` (如 `TABLE_HEADERS`)

### 文档字符串
```python
def method_name(self, param1, param2):
    """
    方法简短描述
    
    Args:
        param1: 参数1说明
        param2: 参数2说明
        
    Returns:
        返回值说明
    """
    pass
```

### 异常处理
```python
try:
    # 操作
    pass
except SpecificException as e:
    # 具体异常处理
    self.append_info(f"操作失败: {e}")
except Exception as e:
    # 通用异常处理
    self.append_info(f"未知错误: {e}")
```

---

## 🚀 性能优化检查清单

- [ ] 避免在循环中频繁更新UI
- [ ] 使用缓冲机制批量处理
- [ ] 只更新变化的数据（差量更新）
- [ ] 合理设置定时器间隔
- [ ] 限制日志输出数量
- [ ] 避免不必要的数据复制
- [ ] 使用弱引用避免内存泄漏

---

## 📞 快速联系

**如有问题，检查以下文件**:
- `REFACTOR_README.md` - 重构说明
- `MODULE_ARCHITECTURE.md` - 架构详解
- `QUICK_REFERENCE.md` - 本文件

**原始备份**: `main_gui_app_backup.py`
