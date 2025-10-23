# 修复：离群 Steering 模式按钮功能

## 问题描述

离群表格中的 **Steering** 按钮在 UI 中存在，但点击后没有任何反应。

## 原因分析

1. ✅ UI 按钮已定义：`set_departed_Steering_pushButton`
2. ✅ 命令处理方法已实现：`USVCommandHandler.set_departed_steering()`
3. ✅ ROS 信号已定义：`steering_command`
4. ✅ 回调函数已实现：`GroundStationNode.set_steering_callback()`
5. ❌ **缺失**：主窗口未连接按钮点击信号到处理方法

## 修复方案

### 修改文件：`gs_gui/main_gui_app.py`

#### 1. 连接按钮信号（第 130 行）

```python
# 在 _connect_ui_signals() 方法中添加
self.ui.set_departed_Steering_pushButton.clicked.connect(self.set_departed_steering_command)
```

#### 2. 添加包装方法（第 202 行后）

```python
def set_departed_steering_command(self):
    """离群设置Steering模式"""
    self.command_handler.set_departed_steering(self.list_manager.usv_departed_list)
```

## 完整调用链

```
UI按钮点击
    ↓
MainWindow.set_departed_steering_command()
    ↓
USVCommandHandler.set_departed_steering()
    ↓
发送信号: steering_command.emit(namespace_list)
    ↓
GroundStationNode.set_steering_callback()
    ↓
CommandProcessor.set_steering_callback()
    ↓
发布ROS消息: set_usv_mode (mode="STEERING")
    ↓
机载接收并执行
```

## 测试验证

### 1. 编译

```bash
cd ~/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash
```

### 2. 运行测试

```bash
# 启动地面站
ros2 launch gs_bringup gs_launch.py

# 操作步骤：
# 1. 等待 USV 上线
# 2. 在离群列表中选择 USV
# 3. 点击 "Steering" 按钮
# 4. 查看日志输出是否有 "离群设置Steering模式命令已发送"
```

### 3. 监控消息

```bash
# 在另一个终端监控模式设置命令
ros2 topic echo /usv_01/set_usv_mode
```

预期输出：
```
data: 'STEERING'
```

## 相关命令对比

| 按钮 | 包装方法 | 命令处理器方法 | ROS 信号 |
|------|---------|---------------|---------|
| Arming | `departed_arming_command` | `departed_arming` | `arming_command` |
| Disarming | `departed_disarming_command` | `departed_disarming` | `disarming_command` |
| Guided | `set_departed_guided_command` | `set_departed_guided` | `guided_command` |
| Manual | `set_departed_manual_command` | `set_departed_manual` | `manual_command` |
| ARCO | `set_departed_arco_command` | `set_departed_arco` | `arco_command` |
| **Steering** | **`set_departed_steering_command`** ✅ | `set_departed_steering` | `steering_command` |

## 修复状态

- ✅ 按钮信号已连接
- ✅ 包装方法已添加
- ✅ 编译成功
- 🔄 待端到端测试验证

## 修复日期

2025-10-23

---

**注意**：此修复遵循现有代码模式，与其他离群模式按钮（Guided、Manual、ARCO）保持一致。
