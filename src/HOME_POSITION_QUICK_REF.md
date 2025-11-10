# Home Position 设置功能 - 快速参考

## 功能概述

在地面站 GUI 中添加了 **Home Position 设置功能**，允许在系统运行时动态设置 USV 的返航点。

## 快速使用

### 步骤
1. 启动地面站：`ros2 launch gs_bringup gs_launch.py`
2. 打开菜单：**工具 → 设置 Home Position** (或按 `Ctrl+H`)
3. 在对话框中：
   - 选择 USV
   - 选择坐标来源（当前位置 / 指定坐标）
   - 点击确定
4. 查看日志窗口确认结果

## 核心概念

### EKF Origin vs Home Position

| 特性 | EKF Origin | Home Position |
|------|-----------|--------------|
| **何时设置** | 启动时自动 | GUI 手动 |
| **作用** | 坐标系原点 | RTL 返航点 |
| **可修改** | 不建议 | 可随时修改 |
| **命令** | SET_GPS_GLOBAL_ORIGIN | MAV_CMD_DO_SET_HOME |

## 架构实现

```
GUI 对话框 (set_home_dialog.py)
    ↓
PyQt Signal (ros_signal.py)
    ↓
ROS 节点处理 (ground_station_node.py)
    ↓
MAVROS 服务 (/cmd/command)
    ↓
飞控 MAVLink 命令 (MAV_CMD_DO_SET_HOME = 179)
```

## 文件清单

### 新增
- `gs_gui/gs_gui/set_home_dialog.py` - 设置对话框

### 修改
- `gs_gui/gs_gui/ros_signal.py` - 添加信号
- `gs_gui/gs_gui/ground_station_node.py` - ROS 节点处理
- `gs_gui/gs_gui/main_gui_app.py` - 菜单集成

## 验证方法

### 检查 Home Position
```bash
ros2 topic echo /usv_01/mavros/home_position/home
```

### 测试 RTL
```bash
ros2 service call /usv_01/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'RTL'}"
```

## 注意事项

- **需要 GPS 定位**：使用 "当前位置" 模式时需要 3D Fix
- **USV 必须在线**：命名空间正确且 MAVROS 服务可用
- **飞控版本**：ArduPilot 4.7.0+ 或 PX4 1.12+

## 详细文档

完整实现细节请参考：`HOME_POSITION_SETTING_GUIDE.md`
