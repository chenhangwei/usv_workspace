# 地面站优雅关闭功能说明

## 概述

地面站关闭时会自动向所有在线USV发送外设关闭命令，确保USV处于安全状态，避免遗留外设处于激活状态。

## 实现原理

### 1. closeEvent 拦截

在 `MainWindow` 类中重写 `closeEvent` 方法，在窗口关闭之前执行清理操作：

```python
def closeEvent(self, event):
    """窗口关闭事件处理器"""
    # 获取所有在线USV
    online_usvs = self.list_manager.usv_online_list
    
    if online_usvs:
        # 发送关闭命令
        self.ros_signal.str_command.emit('led_off')      # 关闭LED
        self.ros_signal.str_command.emit('sound_stop')   # 停止声音
        self.ros_signal.str_command.emit('neck_stop')    # 停止扭头
        
        # 等待500ms确保命令发送
        QTimer.singleShot(500, lambda: event.accept())
        event.ignore()  # 暂时忽略关闭事件
    else:
        event.accept()  # 直接关闭
```

### 2. 关闭序列

1. **检测在线USV**：从 `list_manager.usv_online_list` 获取所有在线USV列表
2. **发送关闭命令**：
   - `led_off`: 关闭LED灯光
   - `sound_stop`: 停止声音播放
   - `neck_stop`: 停止扭头动作
3. **延迟关闭**：等待500ms确保命令通过ROS网络发送到机载节点
4. **完成关闭**：接受关闭事件，销毁窗口

### 3. 命令传递路径

```
MainWindow.closeEvent()
    ↓
ros_signal.str_command.emit('led_off')
    ↓
GroundStationNode.str_command_callback()
    ↓
发布到 /{namespace}/gs_led_command
    ↓
USV机载节点接收并执行
```

## 关闭的外设

### 1. LED灯光（LED）

- **命令**: `led_off`
- **目标节点**: `usv_led/usv_led_node.py`
- **效果**: 关闭所有LED灯光，包括呼吸灯、颜色切换等模式

### 2. 声音（Sound）

- **命令**: `sound_stop`
- **目标节点**: `usv_sound/usv_sound_node.py`
- **效果**: 停止声音播放

### 3. 扭头动作（Neck/Head Action）

- **命令**: `neck_stop`
- **目标节点**: `usv_action/usv_head_action_node.py`
- **效果**: 停止舵机摆动，保持当前位置

## 用户体验

### 正常关闭流程

1. 用户点击窗口关闭按钮（X）或按 `Alt+F4`
2. 地面站日志显示：
   ```
   正在关闭所有USV外设（LED、声音、扭头）...
   已发送外设关闭命令
   ```
3. 等待500ms
4. 窗口关闭，ROS节点销毁

### 无USV在线时

如果没有USV在线，跳过命令发送，直接关闭窗口。

### 异常处理

如果发送命令时出现异常：
- 记录错误日志
- 仍然允许窗口关闭（避免卡死）

## 技术细节

### 1. 延迟关闭机制

使用 `QTimer.singleShot()` 实现非阻塞延迟：

```python
QTimer.singleShot(500, lambda: event.accept())
event.ignore()  # 先忽略，500ms后再accept
```

**Why**: 避免阻塞GUI主线程，保持界面响应性。

### 2. 命令广播

`str_command` 信号会广播到**所有在线USV**：

```python
# ground_station_node.py
def str_command_callback(self, command_str):
    """字符串命令回调"""
    for namespace, pub in self.usv_manager.str_command_pubs.items():
        pub.publish(String(data=command_str))
```

### 3. 错误容忍

每个命令发送都包含独立的 try-except：

```python
try:
    self.ros_signal.str_command.emit('led_off')
except Exception as e:
    print(f"发送LED关闭命令失败: {e}")
```

确保单个命令失败不影响其他命令执行。

## 测试方法

### 1. 正常关闭测试

```bash
# 终端1: 启动地面站
ros2 launch gs_bringup gs_launch.py

# 终端2-4: 启动3艘USV
ros2 launch usv_bringup usv_launch.py namespace:=usv_01 fcu_url:=...
ros2 launch usv_bringup usv_launch.py namespace:=usv_02 fcu_url:=...
ros2 launch usv_bringup usv_launch.py namespace:=usv_03 fcu_url:=...

# 在GUI中激活LED、声音、扭头
# 点击集群LED按钮、声音按钮、扭头按钮

# 关闭地面站窗口（点击X）
# 观察USV：LED应该关闭、声音停止、扭头停止
```

### 2. 监控命令发送

在USV机载终端监听命令topic：

```bash
# 监听LED命令
ros2 topic echo /usv_01/gs_led_command

# 监听声音命令
ros2 topic echo /usv_01/gs_sound_command

# 监听扭头命令
ros2 topic echo /usv_01/gs_action_command
```

关闭地面站时应该看到：
```
data: 'led_off'
---
data: 'sound_stop'
---
data: 'neck_stop'
```

### 3. 无USV在线测试

```bash
# 只启动地面站，不启动USV
ros2 launch gs_bringup gs_launch.py

# 关闭地面站
# 应该直接关闭，不等待500ms
```

## 修改记录

| 日期 | 修改内容 | 文件 |
|------|---------|------|
| 2025-10 | 添加 closeEvent 方法实现优雅关闭 | `gs_gui/main_gui_app.py` |

## 相关文件

- `gs_gui/gs_gui/main_gui_app.py`: MainWindow.closeEvent() 实现
- `gs_gui/gs_gui/usv_commands.py`: 命令处理器（led_off, sound_stop, neck_stop）
- `gs_gui/gs_gui/ground_station_node.py`: 命令广播到所有USV
- `usv_led/usv_led/usv_led_node.py`: LED关闭处理
- `usv_sound/usv_sound/usv_sound_node.py`: 声音停止处理
- `usv_action/usv_action/usv_head_action_node.py`: 扭头停止处理

## 注意事项

1. **500ms延迟**：足够命令通过ROS网络传输，但不会让用户感觉卡顿
2. **容错设计**：即使命令发送失败，也允许窗口关闭
3. **广播机制**：命令发送到所有在线USV，无需单独处理每艘USV
4. **ROS生命周期**：closeEvent在ROS节点销毁之前执行，确保命令能发送

## 未来改进

- [ ] 添加关闭前确认对话框（可选）
- [ ] 记录关闭命令日志到文件
- [ ] 支持自定义关闭延迟时间
- [ ] 添加关闭状态进度条显示
