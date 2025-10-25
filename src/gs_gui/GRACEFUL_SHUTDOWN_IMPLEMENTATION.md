# 地面站优雅关闭功能 - 实现总结

## 需求

用户要求："关闭地面站，应该先关闭声音、扭头、灯光"

## 实现方案

### 核心思路

在 `MainWindow` 类中重写 `closeEvent` 方法，拦截窗口关闭事件，在关闭前向所有在线USV发送外设关闭命令。

### 实现细节

#### 1. 添加关闭标志（避免重复发送）

在 `__init__` 方法中添加：
```python
self._shutdown_commands_sent = False
```

#### 2. 实现 closeEvent 方法

```python
def closeEvent(self, event):
    # 检查是否已发送命令（避免重复）
    if self._shutdown_commands_sent:
        event.accept()
        return
    
    # 获取在线USV列表
    online_usvs = self.list_manager.usv_online_list
    
    if online_usvs:
        # 发送三个关闭命令
        self.ros_signal.str_command.emit('led_off')
        self.ros_signal.str_command.emit('sound_stop')
        self.ros_signal.str_command.emit('neck_stop')
        
        # 标记已发送
        self._shutdown_commands_sent = True
        
        # 延迟500ms后关闭
        QTimer.singleShot(500, lambda: self.close())
        event.ignore()
    else:
        event.accept()
```

#### 3. 命令传递流程

```
MainWindow.closeEvent()
    ↓ (PyQt Signal)
ros_signal.str_command.emit('led_off')
    ↓ (Signal-Slot)
GroundStationNode.str_command_callback()
    ↓ (ROS Topic Publish)
/{namespace}/gs_led_command
    ↓ (ROS Topic Subscribe)
usv_led_node.py 接收并执行
```

## 关键技术点

### 1. 避免重复发送

使用 `_shutdown_commands_sent` 标志，确保关闭命令只发送一次。

**Why**: `event.ignore()` 后会再次触发 `closeEvent`，不加标志会重复发送命令。

### 2. 非阻塞延迟

使用 `QTimer.singleShot()` 而非 `time.sleep()`：

```python
QTimer.singleShot(500, lambda: self.close())
event.ignore()
```

**Why**: 
- `time.sleep()` 会阻塞GUI主线程，界面会卡死
- `QTimer.singleShot()` 是异步的，不阻塞界面

### 3. 错误容忍

每个命令发送都有独立的 try-except，确保单个失败不影响其他。

### 4. 命令广播

`str_command` 信号会自动广播到所有在线USV（在 `GroundStationNode` 中实现）。

## 修改文件

| 文件 | 修改内容 | 行号 |
|------|---------|------|
| `gs_gui/main_gui_app.py` | 添加 `_shutdown_commands_sent` 标志 | ~42 |
| `gs_gui/main_gui_app.py` | 实现 `closeEvent` 方法 | ~424-487 |

## 测试方法

### 快速测试

1. 启动地面站和USV
2. 激活LED、声音、扭头
3. 关闭地面站窗口
4. 观察USV外设是否关闭

### 详细测试

使用提供的测试脚本：

```bash
# 监听USV命令
./src/gs_gui/scripts/test_graceful_shutdown.sh usv_01

# 关闭地面站
# 应该看到三个命令：led_off, sound_stop, neck_stop
```

## 用户体验

### 正常流程

1. 用户点击窗口关闭按钮
2. 地面站日志显示：
   ```
   正在关闭所有USV外设（LED、声音、扭头）...
   已发送外设关闭命令
   ```
3. 等待500ms
4. 窗口关闭

### 无USV在线

如果没有USV在线，跳过命令发送，直接关闭（0ms延迟）。

### 错误处理

如果命令发送失败，记录错误但仍允许窗口关闭。

## 性能影响

- **关闭延迟**: 500ms（仅当有USV在线时）
- **网络开销**: 3个字符串消息（~30字节）
- **CPU影响**: 可忽略不计

## 潜在问题和解决方案

### 问题1: USV未响应命令

**原因**: ROS网络延迟或USV节点未运行

**解决**: 
- 500ms延迟足够覆盖局域网延迟
- 机载节点应该设置为自动启动

### 问题2: 重复发送命令

**原因**: `event.ignore()` 会再次触发 `closeEvent`

**解决**: 使用 `_shutdown_commands_sent` 标志

### 问题3: 界面卡顿

**原因**: 使用 `time.sleep()` 阻塞主线程

**解决**: 使用 `QTimer.singleShot()` 异步延迟

## 文档

- **详细说明**: `GRACEFUL_SHUTDOWN.md`
- **快速参考**: `GRACEFUL_SHUTDOWN_QUICK_REF.md`
- **测试脚本**: `scripts/test_graceful_shutdown.sh`

## 构建

```bash
cd ~/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash
```

## 未来改进

1. **可配置延迟**: 从参数文件读取延迟时间
2. **关闭确认**: 添加可选的确认对话框
3. **进度显示**: 显示关闭进度条
4. **日志记录**: 记录关闭命令到文件
5. **状态反馈**: 等待USV确认收到命令

## 相关issue/PR

- 用户需求: "关闭地面站，应该先关闭声音、扭头、灯光"
- 实现日期: 2025-10

## 验收标准

- [x] 关闭地面站时自动发送关闭命令
- [x] 命令发送到所有在线USV
- [x] 不阻塞界面（使用异步延迟）
- [x] 避免重复发送命令
- [x] 错误容忍（失败时仍可关闭）
- [x] 无USV时直接关闭（0延迟）
- [x] 提供测试脚本
- [x] 编写文档

---

**实现完成** ✅

**构建成功** ✅

**待测试** 🔄（需要实际硬件或仿真环境）
