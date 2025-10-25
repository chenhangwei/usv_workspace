# 优雅关闭功能 - 快速参考

## 功能说明

关闭地面站时自动关闭所有在线USV的LED、声音和扭头动作，避免USV处于激活状态。

## 工作原理

```
用户点击关闭按钮
    ↓
检测在线USV列表
    ↓
发送三个关闭命令：
  - led_off (关闭LED)
  - sound_stop (停止声音)
  - neck_stop (停止扭头)
    ↓
等待500ms确保命令发送
    ↓
关闭窗口
```

## 代码位置

**文件**: `gs_gui/gs_gui/main_gui_app.py`

**方法**: `MainWindow.closeEvent(self, event)`

**关键代码**:
```python
def closeEvent(self, event):
    online_usvs = self.list_manager.usv_online_list
    if online_usvs:
        # 发送关闭命令
        self.ros_signal.str_command.emit('led_off')
        self.ros_signal.str_command.emit('sound_stop')
        self.ros_signal.str_command.emit('neck_stop')
        
        # 延迟500ms后关闭
        QTimer.singleShot(500, lambda: event.accept())
        event.ignore()
    else:
        event.accept()
```

## 测试验证

### 方法1: 监控ROS话题

```bash
# 监听命令（在USV机载终端）
ros2 topic echo /usv_01/gs_led_command
ros2 topic echo /usv_01/gs_sound_command
ros2 topic echo /usv_01/gs_action_command
```

关闭地面站时应看到三个命令。

### 方法2: 观察USV外设

1. 启动地面站和USV
2. 激活LED、声音、扭头
3. 关闭地面站窗口
4. 观察USV：LED应关闭、声音停止、扭头停止

## 关闭延迟

- **有USV在线**: 500ms延迟（发送命令）
- **无USV在线**: 0ms延迟（直接关闭）

## 错误处理

即使命令发送失败，窗口仍会关闭，避免卡死。

## 相关文档

详细说明见 `GRACEFUL_SHUTDOWN.md`
