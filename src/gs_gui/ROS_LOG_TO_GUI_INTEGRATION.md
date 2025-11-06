# ROS 日志信息集成到 GUI 窗口

## 📋 修改概述

将 ROS 节点的关键日志信息（StatusText 消息和命令处理）自动输出到 GUI 的 info/warning 窗口，提升用户体验。

**修改日期**: 2025-11-06  
**影响包**: `gs_gui`

---

## 🎯 修改目标

之前这些信息只输出到终端（ROS logger），用户在 GUI 中看不到：
- ❌ USV 飞控的 StatusText 消息（PreArm 检查、错误提示等）
- ❌ 命令处理状态（LED 颜色、声音、扭头等）

现在这些信息会**智能分流**到 GUI 的两个窗口：
- ✅ **Info 窗口**：普通信息（NOTICE/INFO/DEBUG，severity ≥ 5）
- ✅ **Warning 窗口**：警告和错误（EMERGENCY/ALERT/CRITICAL/ERROR/WARNING，severity ≤ 4）

---

## 📝 修改详情

### 1. GroundStationNode 初始化修改

**文件**: `gs_gui/ground_station_node.py`

```python
def __init__(self, signal, append_info=None, append_warning=None):
    """
    初始化地面站节点
    
    Args:
        signal: ROS信号对象，用于与GUI界面通信
        append_info: GUI 信息输出回调函数（可选）
        append_warning: GUI 警告输出回调函数（可选）
    """
    super().__init__('groundstationnode')
    self.ros_signal = signal
    self.append_info = append_info if append_info else lambda x: None
    self.append_warning = append_warning if append_warning else lambda x: None
    # ...
```

**变化**：
- ➕ 新增 `append_info` 参数（GUI info 窗口回调）
- ➕ 新增 `append_warning` 参数（GUI warning 窗口回调）
- ➕ 默认值为空函数，确保向后兼容

### 2. StatusText 消息处理修改

**文件**: `gs_gui/ground_station_node.py` - `handle_status_text()` 方法

```python
# 根据 severity 输出到不同窗口
# MAVLink Severity 定义：
# 0: EMERGENCY   → warning 窗口
# 1: ALERT       → warning 窗口
# 2: CRITICAL    → warning 窗口
# 3: ERROR       → warning 窗口
# 4: WARNING     → warning 窗口
# 5: NOTICE      → info 窗口
# 6: INFO        → info 窗口
# 7: DEBUG       → info 窗口

if severity <= 4:  # 错误和警告
    self.append_warning(f"⚠️ [{usv_id}] {text}")
else:  # 普通信息
    self.append_info(f"📡 [{usv_id}] {text}")
```

**示例输出**：
```
Warning 窗口:
⚠️ [usv_03] PreArm: Hardware safety switch
⚠️ [usv_03] PreArm: Check mag field: 1238, max 875, min 185
⚠️ [usv_03] PreArm: Radio failsafe on

Info 窗口:
📡 [usv_01] EKF3 IMU0 is using GPS
📡 [usv_02] Reached destination
```

### 3. 命令处理消息修改

**文件**: `gs_gui/command_processor.py` - `process_incoming_str_commands()` 方法

```python
# 输出到 GUI info 窗口和 ROS logger
if hasattr(self.node, 'append_info'):
    self.node.append_info(f"📤 处理命令: {msg}")
self.node.get_logger().info(f"处理入队命令: {msg}")
```

**示例输出**：
```
Info 窗口:
📤 处理命令: color_select|255,0,0
📤 处理命令: sound_start
📤 处理命令: neck_mode|tracking
```

### 4. 主窗口初始化修改

**文件**: `gs_gui/main_gui_app.py` - `main()` 函数

```python
# 初始化ROS节点（传入 append_info 和 append_warning 回调以输出到 GUI）
rclpy.init(args=None)
node = GroundStationNode(
    ros_signal, 
    append_info=main_window.ui_utils.append_info,
    append_warning=main_window.ui_utils.append_warning
)
```

**变化**：
- ➕ 传入 `append_info` 回调（连接到 `UIUtils.append_info()`）
- ➕ 传入 `append_warning` 回调（连接到 `UIUtils.append_warning()`）

---

## 📊 Severity 级别映射表

| Severity | 标签 | 窗口 | Emoji | 示例消息 |
|----------|------|------|-------|---------|
| 0 | EMERGENCY | Warning | ⚠️ | System failure |
| 1 | ALERT | Warning | ⚠️ | Battery critical |
| 2 | CRITICAL | Warning | ⚠️ | GPS lost |
| 3 | ERROR | Warning | ⚠️ | Sensor error |
| 4 | WARNING | Warning | ⚠️ | PreArm: Hardware safety switch |
| 5 | NOTICE | Info | 📡 | Mode changed to GUIDED |
| 6 | INFO | Info | 📡 | EKF3 IMU0 is using GPS |
| 7 | DEBUG | Info | 📡 | Received heartbeat |

---

## 🎨 Emoji 使用规范

| Emoji | 含义 | 使用场景 |
|-------|------|---------|
| ⚠️ | 警告/错误 | Severity ≤ 4 的 StatusText 消息 |
| 📡 | 消息接收 | Severity ≥ 5 的 StatusText 消息 |
| 📤 | 发送命令 | 命令处理（LED/声音/扭头等） |

---

## ✅ 功能验证

启动系统后，应在 GUI 窗口看到：

**Info 窗口示例**：
```
📤 处理命令: color_select|255,0,0
📤 处理命令: color_select|255,127,0
📤 处理命令: color_select|255,255,0
📡 [usv_01] EKF3 IMU0 is using GPS
📡 [usv_02] Reached destination
```

**Warning 窗口示例**：
```
⚠️ [usv_03] PreArm: Hardware safety switch
⚠️ [usv_03] PreArm: Check mag field: 1238, max 875, min 185
⚠️ [usv_03] PreArm: Radio failsafe on
⚠️ [usv_01] GPS glitch detected
```

---

## 🔧 向后兼容性

- ✅ `append_info` 和 `append_warning` 参数为可选，默认值为空函数
- ✅ 旧代码不传入回调时，日志仍输出到 ROS logger（终端）
- ✅ 不影响其他功能模块

---

## 📚 相关文件

- `gs_gui/ground_station_node.py` - ROS 节点初始化和 StatusText 处理
- `gs_gui/command_processor.py` - 命令处理和日志输出
- `gs_gui/main_gui_app.py` - 主窗口初始化和回调传递
- `gs_gui/ui_utils.py` - GUI 工具类（提供 `append_info()` 和 `append_warning()`）

---

## 🚀 使用效果

**之前**：
- 用户只能在终端看到 ROS logger 输出
- GUI 中无法实时监控 USV 状态和命令执行情况
- 需要频繁切换终端和 GUI 窗口

**之后**：
- ✅ GUI 中实时显示所有关键信息
- ✅ 警告和错误自动高亮显示（warning 窗口）
- ✅ 普通信息不干扰用户（info 窗口）
- ✅ 一键查看所有历史日志（窗口自带滚动和缓冲）

---

**最后更新**: 2025-11-06  
**编译状态**: ✅ 成功  
**测试状态**: ⏳ 待用户验证
