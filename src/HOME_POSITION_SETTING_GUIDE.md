# Home Position 设置功能实现总结

## 概述

本次实现在地面站 GUI 中添加了 **Home Position 设置功能**，允许用户在系统运行时动态设置 USV 的 Home Position（返航点）。

## 背景

在之前的优化中，我们将 **EKF Origin** 和 **Home Position** 进行了概念分离：

- **EKF Origin (GPS_GLOBAL_ORIGIN)**:
  - 在 USV 启动时自动设置为 A0 基站坐标 (22.5180977°N, 113.9007239°E, -4.8m)
  - 通过 `auto_set_home_node` 自动完成，无需手动干预
  - 使用 `SET_GPS_GLOBAL_ORIGIN` MAVLink 消息（通过 GeoPointStamped topic）
  - 作用：为 EKF 提供全局坐标系原点，确保位置估计的稳定性

- **Home Position (DO_SET_HOME)**:
  - 现在可以通过地面站 GUI 菜单手动设置
  - 使用 `MAV_CMD_DO_SET_HOME` MAVLink 命令（command ID: 179）
  - 作用：定义 RTL（返航）和 Failsafe（失联保护）的目标位置
  - 可以在任务执行过程中动态修改

## 实现内容

### 1. ROS 信号定义（`ros_signal.py`）

添加了新的 PyQt Signal 用于 GUI 和 ROS 节点之间的通信：

```python
# Home Position 设置信号
set_home_position = pyqtSignal(str, bool, dict)
# 参数：USV命名空间, 是否使用当前位置, 坐标字典{'lat':float,'lon':float,'alt':float}
```

### 2. ROS 节点实现（`ground_station_node.py`）

#### 新增方法

**`set_home_position_callback(usv_namespace, use_current, coords)`**:
- 发送 `MAV_CMD_DO_SET_HOME` 命令到指定 USV
- 支持两种模式：
  - `use_current=True`: 使用 USV 当前 GPS 位置作为 Home Position
  - `use_current=False`: 使用指定坐标（latitude, longitude, altitude）
- 通过 MAVROS 的 `/cmd/command` 服务发送命令
- 异步调用，通过回调函数处理响应

**`_handle_set_home_response(future, usv_namespace, use_current, coords)`**:
- 处理 MAVLink 命令响应
- 显示成功/失败消息在 GUI 日志窗口
- 记录日志到 ROS logger

#### MAVLink 命令参数

```python
request.command = 179  # MAV_CMD_DO_SET_HOME
request.param1 = 1.0 if use_current else 0.0  # 1=当前位置, 0=指定坐标
request.param5 = latitude   # 纬度（°）
request.param6 = longitude  # 经度（°）
request.param7 = altitude   # 高度（米，相对海平面）
```

### 3. 设置对话框（`set_home_dialog.py`）

创建了新的 `SetHomeDialog` 类，提供友好的用户界面：

#### UI 组件

1. **说明文本**：解释 Home Position 的作用和与 EKF Origin 的区别
2. **USV 选择器**：下拉列表选择要设置 Home Position 的 USV
3. **坐标来源**：
   - 单选按钮："使用 USV 当前位置" / "指定坐标"
   - 切换时自动启用/禁用坐标输入框
4. **坐标输入**：
   - 纬度输入框（°）
   - 经度输入框（°）
   - 高度输入框（米）
   - 快捷按钮："使用 A0 基站坐标"（自动填入 22.5180977, 113.9007239, -4.8）
5. **确定/取消按钮**

#### 数据验证

- 检查是否有在线 USV
- 验证坐标格式（是否为有效数字）
- 验证坐标范围：
  - 纬度：-90° 到 90°
  - 经度：-180° 到 180°

### 4. GUI 菜单集成（`main_gui_app.py`）

#### 菜单项

在 "工具(&T)" 菜单中添加：
- 🏠 **设置 Home Position** (快捷键: Ctrl+H)
- 提示文本："设置 USV 的 Home Position（RTL 返航点）"

#### 对话框调用

**`open_set_home_dialog()`**:
- 检查是否有在线 USV（如果没有，显示警告消息）
- 创建并显示 `SetHomeDialog`
- 处理对话框结果：
  - 用户点击 "确定"：发送 `set_home_position` 信号到 ROS 节点
  - 用户点击 "取消"：关闭对话框，不执行任何操作
- 在 GUI 日志窗口显示操作反馈

#### 信号连接

在 `main()` 函数中连接信号：
```python
ros_signal.set_home_position.connect(node.set_home_position_callback)
```

## 使用方法

### 通过 GUI 菜单

1. 启动地面站 GUI：
   ```bash
   ros2 launch gs_bringup gs_launch.py
   ```

2. 确保至少有一艘 USV 在线

3. 打开菜单：**工具 → 设置 Home Position** (或按 `Ctrl+H`)

4. 在对话框中：
   - 选择要设置的 USV
   - 选择坐标来源：
     - **使用 USV 当前位置**：将 Home Position 设置为 USV 当前 GPS 位置
     - **指定坐标**：手动输入坐标，或点击 "使用 A0 基站坐标" 快速填入
   - 点击 "确定"

5. 查看日志窗口确认操作结果

### 命令示例

#### 使用当前位置
```
[OK] 已向 usv_01 发送设置 Home Position 命令（使用当前位置）
[OK] usv_01 Home Position 已设置为当前位置
```

#### 使用指定坐标
```
[OK] 已向 usv_01 发送设置 Home Position 命令
    坐标: 22.5180977, 113.9007239, -4.80m
[OK] usv_01 Home Position 已设置为指定坐标
    坐标: 22.5180977, 113.9007239, -4.80m
```

## 技术细节

### MAVLink 命令流程

```
GUI (SetHomeDialog)
    ↓ [用户点击确定]
PyQt Signal: set_home_position(usv_namespace, use_current, coords)
    ↓
GroundStationNode.set_home_position_callback()
    ↓
创建 MAVROS CommandLong 服务客户端
    ↓
发送 MAV_CMD_DO_SET_HOME (179) 命令
    ↓
等待 MAVLink 响应（异步）
    ↓
_handle_set_home_response()
    ↓
显示成功/失败消息在 GUI
```

### 与 EKF Origin 的区别

| 特性 | EKF Origin (GPS_GLOBAL_ORIGIN) | Home Position (DO_SET_HOME) |
|------|-------------------------------|----------------------------|
| **设置时机** | 启动时自动设置 | 手动通过 GUI 设置 |
| **设置方式** | `auto_set_home_node` | GUI 菜单 "设置 Home Position" |
| **MAVLink** | `SET_GPS_GLOBAL_ORIGIN` 消息 | `MAV_CMD_DO_SET_HOME` 命令 |
| **MAVROS 接口** | `/global_position/set_gp_origin` (topic) | `/cmd/command` (service) |
| **作用** | 为 EKF 提供全局坐标系原点 | 定义 RTL/Failsafe 目标位置 |
| **可修改性** | 启动后不建议修改 | 可在任务中动态修改 |
| **默认坐标** | A0 基站 (22.5180977, 113.9007239, -4.8) | 未设置（需要手动设置） |

### 为什么需要区分？

在 ArduPilot 4.7.0-dev 中，`MAV_CMD_DO_SET_HOME` 命令实际上会**同时设置** EKF Origin 和 Home Position。这导致了以下问题：

1. **启动时设置的混淆**：如果启动时使用 `DO_SET_HOME`，会同时设置 Origin 和 Home
2. **任务中动态修改的风险**：如果任务中使用 `DO_SET_HOME` 修改 Home Position，也会意外修改 EKF Origin，导致坐标系混乱

**解决方案**：
- 启动时使用 `SET_GPS_GLOBAL_ORIGIN` **仅**设置 EKF Origin
- 任务中使用 `DO_SET_HOME` 设置 Home Position（此时 Origin 已固定，不会被影响）

## 验证方法

### 1. 检查 Home Position 是否生效

通过 MAVROS topic 监听：
```bash
ros2 topic echo /usv_01/mavros/home_position/home
```

输出示例：
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 0
  frame_id: "map"
geo:
  latitude: 22.5180977
  longitude: 113.9007239
  altitude: -4.8
position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
```

### 2. 测试 RTL 返航

1. 设置 Home Position 到某个位置
2. 将 USV 移动到其他位置
3. 切换到 RTL 模式：
   ```bash
   ros2 service call /usv_01/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'RTL'}"
   ```
4. 观察 USV 是否返航到设置的 Home Position

## 文件修改清单

### 新增文件
- `gs_gui/gs_gui/set_home_dialog.py` - Home Position 设置对话框

### 修改文件
1. `gs_gui/gs_gui/ros_signal.py`
   - 添加 `set_home_position` 信号

2. `gs_gui/gs_gui/ground_station_node.py`
   - 添加 `set_home_position_callback()` 方法
   - 添加 `_handle_set_home_response()` 方法

3. `gs_gui/gs_gui/main_gui_app.py`
   - 在 "工具" 菜单添加 "设置 Home Position" 菜单项
   - 添加 `open_set_home_dialog()` 方法
   - 连接 `set_home_position` 信号

## 已知限制

1. **飞控固件版本依赖**：需要 ArduPilot 4.7.0 或更高版本（支持 `MAV_CMD_DO_SET_HOME` 命令）
2. **GPS 定位要求**：使用 "当前位置" 模式时，USV 必须有有效的 GPS 定位（3D Fix）
3. **命名空间要求**：USV 必须在线且命名空间格式正确（如 `usv_01`）
4. **权限要求**：MAVROS 服务必须可用（通常在 USV 启动后 3-6 秒内就绪）

## 后续优化建议

1. **显示当前 Home Position**：
   - 在对话框中显示 USV 当前的 Home Position 坐标
   - 订阅 `/mavros/home_position/home` topic

2. **批量设置**：
   - 支持同时为多艘 USV 设置 Home Position
   - 添加 "全部使用 A0 基站" 快捷按钮

3. **地图交互**：
   - 在地图上点击选择 Home Position
   - 可视化显示当前 Home Position 位置

4. **历史记录**：
   - 保存常用的 Home Position 坐标
   - 提供快速选择历史坐标的功能

5. **RTL 测试**：
   - 添加 "测试 RTL" 按钮，自动切换到 RTL 模式验证 Home Position

## 参考文档

- **MAVLink 协议**: [MAV_CMD_DO_SET_HOME](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME)
- **ArduRover 文档**: [Set Home](https://ardupilot.org/rover/docs/common-rally-points.html)
- **MAVROS 服务**: [CommandLong Service](http://wiki.ros.org/mavros/CustomPlugins#Command_plugin)
- **项目文档**: 
  - `BOOT_POSE_CLEANUP_SUMMARY.md` - EKF Origin 与 Home Position 分离说明
  - `usv_comm/AUTO_SET_HOME_NODE_REDESIGN.md` - `auto_set_home_node` 重写总结

## 更新日期

2025-01-XX

---

**实现完成，功能验证通过！** ✅
