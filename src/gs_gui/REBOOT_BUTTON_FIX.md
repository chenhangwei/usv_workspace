# 飞控重启按钮修复文档

## 问题描述

**症状**：USV 详细界面中的"🔄 重启飞控"按钮点击后没有任何反应，飞控未重启。

**影响**：用户无法通过地面站 GUI 远程重启飞控，需要手动重启硬件或使用其他工具（如 QGroundControl）。

---

## 根本原因

### 服务名称错误

代码中使用的 MAVROS 命令服务名称**包含了多余的 `mavros` 子命名空间**：

**错误的服务名称**：
```python
service_name = f'/{usv_namespace}/mavros/cmd/command'
# 例如：/usv_03/mavros/cmd/command
```

**实际的服务名称**（来自 MAVROS 启动配置）：
```python
service_name = f'/{usv_namespace}/cmd/command'
# 例如：/usv_03/cmd/command
```

### 为什么会出现这个错误？

在 `usv_launch.py` 中，MAVROS 节点的配置如下：

```python
Node(
    package='mavros',
    executable='mavros_node',
    name='mavros',
    namespace=namespace,  # 例如 usv_03
    ...
)
```

MAVROS 节点运行在 `namespace` (如 `usv_03`) 下，所有服务直接发布到该命名空间下：
- `/usv_03/cmd/command` ✅
- `/usv_03/cmd/arming` ✅
- `/usv_03/set_usv_mode` ✅（这是我们自己的 topic，也在同一命名空间）

**不会**额外嵌套一层 `/mavros` 子命名空间。

### 验证方法

```bash
# 列出所有 cmd 相关服务
ros2 service list | grep usv_03 | grep cmd

# 正确输出：
/usv_03/cmd/arming
/usv_03/cmd/command          # ✅ 正确的服务名
/usv_03/cmd/command_int
/usv_03/cmd/land
...

# 如果使用错误的服务名
ros2 service call /usv_03/mavros/cmd/command ...
# 结果：Service not found
```

---

## 修复方案

### 代码修改

**文件**：`gs_gui/ground_station_node.py`（第 825 行）

**修改前**：
```python
def reboot_autopilot_callback(self, usv_namespace):
    try:
        from mavros_msgs.srv import CommandLong
        
        # ❌ 错误的服务名
        service_name = f'/{usv_namespace}/mavros/cmd/command'
        client = self.create_client(CommandLong, service_name)
        
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'❌ 服务不可用: {service_name}')
            # 实际上服务存在，但名称错误导致找不到
```

**修改后**：
```python
def reboot_autopilot_callback(self, usv_namespace):
    try:
        from mavros_msgs.srv import CommandLong
        
        # ✅ 正确的服务名（MAVROS 命令服务在节点命名空间下，不需要 mavros 子命名空间）
        service_name = f'/{usv_namespace}/cmd/command'
        client = self.create_client(CommandLong, service_name)
        
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'❌ 服务不可用: {service_name}')
            # 现在能正确找到服务
```

### 重新构建

```bash
cd ~/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash
```

---

## 测试验证

### 测试步骤

1. **启动 USV 节点**：
```bash
ros2 launch usv_bringup usv_launch.py namespace:=usv_03
```

2. **启动地面站 GUI**：
```bash
ros2 launch gs_bringup gs_launch.py
```

3. **在 GUI 中测试重启按钮**：
   - 选中一个 USV
   - 切换到 "USV 信息" 标签页
   - 点击 "🔄 重启飞控" 按钮
   - 确认重启对话框

4. **验证结果**：
   - 查看日志输出：应显示 "✅ 已向 usv_03 发送飞控重启命令"
   - 飞控应在 10-20 秒内重启
   - MAVROS 连接断开后重新连接

### 手动验证服务调用

```bash
# 手动测试重启命令（等效于按钮点击）
ros2 service call /usv_03/cmd/command mavros_msgs/srv/CommandLong \
"{
  broadcast: false,
  command: 246,
  confirmation: 0,
  param1: 1.0,
  param2: 0.0,
  param3: 0.0,
  param4: 0.0,
  param5: 0.0,
  param6: 0.0,
  param7: 0.0
}"

# 预期结果：
# success: true
# result: 0  # MAV_RESULT_ACCEPTED
```

**命令参数说明**：
- `command: 246` - MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
- `param1: 1.0` - 重启飞控（1=reboot autopilot）
- `param2: 0.0` - 不重启机载计算机

---

## 相关代码位置

### 信号流程

```
UI 按钮点击
  ↓
main_gui_app.py: on_reboot_autopilot_clicked()
  ↓
ros_signal.reboot_autopilot.emit(usv_id)
  ↓
ground_station_node.py: reboot_autopilot_callback(usv_namespace)
  ↓
创建 CommandLong 服务客户端
  ↓
调用 /usv_XX/cmd/command 服务（✅ 现已修复）
  ↓
飞控执行重启
```

### 涉及文件

1. **`gs_gui/usv_info_panel.py`** (第 272 行)
   - 创建 "🔄 重启飞控" 按钮
   - 设置提示文本和样式

2. **`gs_gui/main_gui_app.py`**
   - 第 136 行：连接按钮点击信号
   ```python
   self.usv_info_panel.reboot_button.clicked.connect(self.on_reboot_autopilot_clicked)
   ```
   - 第 512-539 行：`on_reboot_autopilot_clicked()` 方法
   - 第 800 行：连接 ROS 信号到节点回调
   ```python
   ros_signal.reboot_autopilot.connect(node.reboot_autopilot_callback)
   ```

3. **`gs_gui/ground_station_node.py`** (第 814-890 行)
   - `reboot_autopilot_callback()` - 主要修复位置
   - `_handle_reboot_response()` - 处理重启响应

4. **`gs_gui/ros_signal.py`**
   - 定义 `reboot_autopilot` 信号
   ```python
   reboot_autopilot = pyqtSignal(str)  # USV namespace
   ```

---

## 经验教训

### 1. MAVROS 命名空间规则

**MAVROS 节点配置**：
```python
Node(
    package='mavros',
    executable='mavros_node',
    name='mavros',        # 节点名
    namespace=namespace,  # 如 'usv_03'
    ...
)
```

**服务命名规则**：
- MAVROS 服务发布在 **节点命名空间** 下
- 服务路径：`/{namespace}/cmd/command`
- **不会**额外嵌套 `/mavros` 子命名空间

**对比其他系统**（容易混淆的地方）：
- 有些系统会嵌套：`/{namespace}/{node_name}/service`
- 但 MAVROS 配置中已经指定了 `namespace=usv_03`，服务直接在该命名空间下
- 节点名 `name='mavros'` 不影响服务路径

### 2. 调试服务问题的方法

```bash
# 1. 列出所有服务
ros2 service list

# 2. 查看服务类型
ros2 service type /usv_03/cmd/command
# 输出：mavros_msgs/srv/CommandLong

# 3. 查看服务接口定义
ros2 interface show mavros_msgs/srv/CommandLong

# 4. 手动调用服务测试
ros2 service call /usv_03/cmd/command mavros_msgs/srv/CommandLong "{...}"

# 5. 监控服务调用（调试用）
ros2 topic echo /rosout | grep -i "command\|reboot"
```

### 3. 代码审查要点

在引用 ROS 服务/topic 时，务必：
1. **确认实际命名空间结构**（用 `ros2 service list` 验证）
2. **检查 launch 文件配置**（namespace 参数）
3. **区分节点名和命名空间**（name vs namespace）
4. **参考现有代码**（如 arming、mode 切换等已验证可用的服务调用）

---

## 参考文档

- **MAVROS 命令服务文档**: [mavros_msgs/CommandLong](http://docs.ros.org/en/api/mavros_msgs/html/srv/CommandLong.html)
- **MAVLink 重启命令**: [MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246)](https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
- **USV 启动配置**: `usv_bringup/launch/usv_launch.py`
- **快速开始指南**: `../QUICK_START.md`

---

**修复日期**: 2025-11-04  
**影响版本**: 所有使用 MAVROS 2.x 的版本  
**修复状态**: ✅ 已修复并测试通过
