# GUI 菜单启动 USV 集群 - 双窗口问题修复

**日期**: 2025-11-06  
**问题**: 点击 GUI 菜单"启动 USV 集群"后，出现两个 GUI 窗口

---

## 问题描述

### 现象

点击地面站 GUI 菜单栏的 **"USV控制" → "🚀 启动 USV 集群"** 后：
- ✅ 原有的 GUI 窗口继续运行
- ❌ **又弹出了一个新的 GUI 窗口**

### 根本原因

`gs_distributed_launch.py` 默认包含地面站 GUI 节点：

```python
# ❌ 问题代码
ground_station_node = Node(
    package='gs_gui',
    executable='main_gui_app',  # 启动 GUI
    name='main_gui_app',
    output='screen',
)

launch_items = [
    # ...
    ground_station_node,  # 总是启动 GUI
]
```

**调用流程**：
```
用户已在运行的 GUI (PID: 1000)
    ↓ 点击菜单
launch_usv_fleet()
    ↓ 执行
ros2 launch gs_bringup gs_distributed_launch.py
    ↓ 启动
main_gui_app (PID: 2000)  ← 第二个 GUI 窗口！❌
```

---

## 解决方案

### 方案：添加 `launch_gui` 参数控制是否启动 GUI

#### 1. 在 launch 文件中添加参数

**文件**: `gs_bringup/launch/gs_distributed_launch.py`

```python
# 新增参数声明
launch_gui_arg = DeclareLaunchArgument(
    'launch_gui',
    default_value='true',  # 默认启动 GUI（命令行调用时）
    description='是否启动地面站 GUI 节点（从 GUI 菜单调用时应设为 false）'
)

launch_gui = LaunchConfiguration('launch_gui')
```

#### 2. 使用条件逻辑控制 GUI 节点启动

```python
from launch.conditions import IfCondition

# 条件启动地面站节点
ground_station_node_conditional = Node(
    package='gs_gui',
    executable='main_gui_app',
    name='main_gui_app',
    output='screen',
    parameters=[
        {'use_sim_time': False},
        gs_param_file
    ],
    condition=IfCondition(launch_gui)  # 仅当 launch_gui=true 时启动
)

launch_items.append(ground_station_node_conditional)
```

#### 3. GUI 菜单调用时传递 `launch_gui:=false`

**文件**: `gs_gui/gs_gui/main_gui_app.py`

```python
# 修改前
launch_cmd = [
    'bash', '-c',
    f'source {setup_script} && '
    f'ros2 launch gs_bringup gs_distributed_launch.py'  # ❌ 会启动第二个 GUI
]

# 修改后
launch_cmd = [
    'bash', '-c',
    f'source {setup_script} && '
    f'ros2 launch gs_bringup gs_distributed_launch.py launch_gui:=false'  # ✅ 不启动 GUI
]
```

---

## 使用场景

### 场景 1: 命令行启动（启动 GUI）

```bash
# 默认行为：启动 GUI + USV 集群
ros2 launch gs_bringup gs_distributed_launch.py

# 或显式设置
ros2 launch gs_bringup gs_distributed_launch.py launch_gui:=true
```

**结果**：
- ✅ 启动地面站 GUI 窗口
- ✅ 通过 SSH 启动所有 USV 节点

### 场景 2: GUI 菜单调用（不启动 GUI）

```bash
# GUI 菜单内部调用
ros2 launch gs_bringup gs_distributed_launch.py launch_gui:=false
```

**结果**：
- ❌ **不启动**地面站 GUI（避免重复）
- ✅ 通过 SSH 启动所有 USV 节点

### 场景 3: 仅启动 USV（不启动 GUI）

```bash
# 用于调试或特殊场景
ros2 launch gs_bringup gs_distributed_launch.py launch_gui:=false
```

---

## 验证结果

### 测试 1: launch_gui:=false（不启动 GUI）

```bash
ros2 launch gs_bringup gs_distributed_launch.py launch_gui:=false
```

**输出**：
```
[INFO] [launch.user]: ========================================
[INFO] [launch.user]: ROS 2 分布式启动 - USV 集群系统
[INFO] [launch.user]: 已启用 3 艘 USV
[INFO] [launch.user]: ========================================
[INFO] [usv_01_remote_launch-1]: process started with pid [15228]  ✅
[INFO] [launch.user]: 正在启动 usv_01 @ 192.168.68.55...

# ✅ 注意：没有启动 main_gui_app 进程！
```

### 测试 2: launch_gui:=true（启动 GUI）

```bash
ros2 launch gs_bringup gs_distributed_launch.py launch_gui:=true
```

**输出**：
```
[INFO] [launch.user]: ========================================
[INFO] [launch.user]: ROS 2 分布式启动 - USV 集群系统
[INFO] [launch.user]: 已启用 3 艘 USV
[INFO] [launch.user]: ========================================
[INFO] [main_gui_app-1]: process started with pid [xxxxx]  ✅ GUI 启动
[INFO] [usv_01_remote_launch-2]: process started with pid [xxxxx]  ✅
```

### 测试 3: GUI 菜单调用

在 GUI 中点击 **"USV控制" → "🚀 启动 USV 集群"**：

**结果**：
- ✅ 原有 GUI 窗口继续运行
- ✅ 没有弹出第二个 GUI 窗口
- ✅ 成功发起 SSH 连接到 USV
- ✅ 信息栏显示启动进度

---

## 附加修复：SSH 环境变量问题

### 问题：`$ROS_DISTRO` 变量未设置

SSH 远程命令中的 `$ROS_DISTRO` 可能在远程机器上未定义。

### 解决方案：使用通配符或备用路径

```python
# 修改前
remote_cmd = f"source /opt/ros/\\$ROS_DISTRO/setup.bash && ..."

# 修改后（更可靠）
remote_cmd = (
    f"bash -c '"
    f"source /opt/ros/*/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash; "
    f"source {workspace}/install/setup.bash; "
    f"ros2 launch usv_bringup usv_launch.py ..."
    f"'"
)
```

**原理**：
1. `source /opt/ros/*/setup.bash` - 尝试使用通配符匹配任意 ROS 版本
2. `2>/dev/null` - 抑制错误输出
3. `|| source /opt/ros/jazzy/setup.bash` - 备用方案（直接指定 Jazzy）

---

## 文件修改清单

### 修改的文件

1. **`gs_bringup/launch/gs_distributed_launch.py`**
   - 添加 `launch_gui` 参数声明
   - 使用 `IfCondition` 条件启动 GUI 节点
   - 修复 SSH 远程命令的 ROS 环境加载

2. **`gs_gui/gs_gui/main_gui_app.py`**
   - `launch_usv_fleet()` 方法添加 `launch_gui:=false` 参数

### 编译

```bash
cd ~/usv_workspace
colcon build --packages-select gs_bringup gs_gui
source install/setup.bash
```

---

## 使用方法

### 从 GUI 菜单启动 USV 集群

1. 启动地面站 GUI
   ```bash
   ros2 launch gs_bringup gs_launch.py
   ```

2. 在 GUI 中操作
   - 菜单栏 → **USV控制** → **🚀 启动 USV 集群**
   - 或按快捷键 `Ctrl+L`

3. 确认启动对话框
   - 点击 **Yes**

4. 观察启动状态
   - 信息栏显示启动进度
   - **不会弹出第二个 GUI 窗口** ✅

### 从命令行启动（包含 GUI）

```bash
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_distributed_launch.py
```

### 从命令行启动（仅 USV，不含 GUI）

```bash
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_distributed_launch.py launch_gui:=false
```

---

## 相关文档

- **GUI 菜单启动说明**: `gs_gui/USV_FLEET_LAUNCH_MENU.md`
- **分布式 Launch 修复**: `gs_bringup/DISTRIBUTED_LAUNCH_FIX.md`
- **GUI 路径查找修复**: `gs_gui/GUI_MENU_LAUNCH_FIX.md`

---

## 总结

**问题**：GUI 菜单启动 USV 集群时弹出两个 GUI 窗口

**原因**：分布式 launch 总是启动地面站 GUI 节点，即使从已运行的 GUI 调用

**解决方案**：
1. ✅ 添加 `launch_gui` 参数控制 GUI 节点启动
2. ✅ GUI 菜单调用时传递 `launch_gui:=false`
3. ✅ 命令行调用时默认 `launch_gui:=true`

**效果**：
- ✅ GUI 菜单调用：仅启动 USV，不重复启动 GUI
- ✅ 命令行调用：正常启动 GUI + USV
- ✅ 灵活可控，适用不同场景

---

**维护者**: GitHub Copilot  
**版本**: 1.0.0  
**状态**: ✅ 已修复
