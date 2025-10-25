# MAVLink 命令超时问题排查指南

## 常见超时命令

在 MAVROS 日志中看到的 `CMD: Command XXX -- ack timeout` 警告说明飞控没有在规定时间内回复命令确认（ACK）。

### 命令编号对照表

| 命令编号 | MAVLink 命令名称                      | 用途                     | 发送者            | 解决方案                          |
|---------|--------------------------------------|-------------------------|------------------|----------------------------------|
| **179** | `MAV_CMD_DO_SET_HOME`                | 设置 Home 点            | auto_set_home_node| 等待 GPS 定位后再发送（已修复）    |
| **520** | `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` | 请求飞控能力/版本   | sys_status 插件   | 升级固件或禁用版本检查            |
| **400** | `MAV_CMD_COMPONENT_ARM_DISARM`       | 解锁/上锁               | 用户/节点        | 检查前置条件（GPS、传感器等）      |
| **176** | `MAV_CMD_DO_SET_MODE`                | 设置飞行模式            | 用户/节点        | 确认模式名称正确                  |
| **511** | `MAV_CMD_SET_MESSAGE_INTERVAL`       | 设置消息发送频率        | MAVROS 插件      | 降低请求频率                      |
| **42424**| `MAV_CMD_SET_GPS_GLOBAL_ORIGIN`     | 设置 GPS 全局原点       | MAVROS           | 等待 GPS 就绪                     |

---

## 问题 1: 命令 179 超时（设置 Home 点）

### 症状
```
[WARN] [usv_XX.cmd]: CMD: Command 179 -- ack timeout
[ERROR] FCU: EKF3 waiting for GPS config data
[WARN] GP: No GPS fix
```

### 原因
- GPS 未获得定位（No GPS fix）
- EKF（扩展卡尔曼滤波器）未初始化完成
- 飞控拒绝在 GPS 和 EKF 未就绪时设置 Home 点

### 解决方案（已实施）

✅ **已修复**: `usv_comm/auto_set_home_node.py` 现在会：
1. 等待 MAVROS 连接
2. 等待 GPS 获得定位（status >= 0）
3. 满足条件后延迟 3 秒再设置 Home 点

**验证**：重启 USV，应该看到类似日志：
```
[auto_set_home_node] Waiting for GPS fix (current: -1, need >= 0)...
[mavros] CON: Got HEARTBEAT, connected
[auto_set_home_node] ✅ GPS fix obtained, Setting Home point in 3.0s...
[auto_set_home_node] ✅ EKF origin set successfully!
```

---

## 问题 2: 命令 520 超时（请求飞控能力）

### 症状
```
[WARN] [usv_XX.cmd]: CMD: Command 520 -- ack timeout
[ERROR] VER: autopilot version service timeout
[WARN] VER: your FCU don't support AUTOPILOT_VERSION, switched to default capabilities
```

### 原因
- **飞控固件版本过旧**，不支持 `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` 命令
- ArduPilot < 3.5 或 PX4 < 1.6 不支持此协议

### 解决方案

#### 方案 A：升级飞控固件（推荐）

**步骤**：
1. 使用 Mission Planner 或 QGroundControl 连接飞控
2. 下载最新稳定版固件：
   - **ArduPilot Rover**: 4.5.x 或更高
   - **PX4**: 1.14.x 或更高
3. 通过地面站软件刷写固件

**好处**：
- ✅ 完整支持 MAVLink 2.0 协议
- ✅ 获得最新功能和安全修复
- ✅ 减少超时和警告

#### 方案 B：配置 MAVROS 兼容旧固件（已实施）

✅ **已配置**: `usv_launch.py` 中添加了：
```python
'sys.min_version': [0, 0, 0],  # 跳过版本检查
'cmd.use_comp_id_system_control': False,  # 减少命令冲突
```

**效果**：减少版本请求超时警告（但仍可能偶尔出现）

---

## 问题 3: 命令 400 超时（解锁/上锁）

### 症状
```
[WARN] [usv_XX.cmd]: CMD: Command 400 -- ack timeout
[ERROR] FCU: PreArm: Hardware safety switch
[ERROR] FCU: PreArm: Compass not calibrated
[ERROR] FCU: PreArm: 3D Accel calibration needed
```

### 原因
飞控前置检查（PreArm Check）失败，常见问题：
- 硬件安全开关未解除
- 指南针未校准
- 加速度计未校准
- GPS 无定位
- 遥控器失联（Radio failsafe）

### 解决方案

#### 1. 检查前置条件
```bash
# 通过 MAVROS 查看状态
ros2 topic echo /usv_XX/mavros/state --once

# 查看电池状态
ros2 topic echo /usv_XX/mavros/battery --once

# 查看 GPS 状态
ros2 topic echo /usv_XX/mavros/global_position/global --once
```

#### 2. 修复前置检查问题

**硬件安全开关**：
```
# 在 Mission Planner 中设置参数
BRD_SAFETY_DEFLT = 0  # 禁用安全开关（仅在测试环境！）
```

**跳过前置检查（仅用于测试）**：
```
ARMING_CHECK = 0  # 禁用所有检查（危险！生产环境不推荐）
# 或选择性禁用
ARMING_CHECK = 61918  # 只检查关键项（排除 GPS、指南针等）
```

**校准传感器**：
- 在 Mission Planner 的 "Initial Setup" → "Mandatory Hardware" 中校准
- 加速度计校准：需要将飞控六面朝上
- 指南针校准：远离金属干扰，旋转飞控

---

## 问题 4: 参数请求超时

### 症状
```
[WARN] [usv_XX.param]: PR: request param #X timeout, retries left 2
[ERROR] [usv_XX.param]: PR: request param #X completely missing
```

### 原因
- MAVROS `param` 插件尝试同步 900+ 个飞控参数
- 每个参数超时 3 次 × 1 秒 = 大量时间浪费
- 串口通信慢或飞控响应慢

### 解决方案（已实施）

✅ **已优化**: 
1. 从 `plugin_allowlist` 中移除了 `param` 插件
2. 添加配置：`'param.use_mission_item_int': False`

**效果**：启动时不再同步参数，节省 ~50 秒启动时间

**如需修改参数**：使用 Mission Planner 或 QGroundControl 地面站软件

---

## 通用排查步骤

### 1. 检查 MAVROS 连接状态

```bash
# 查看心跳状态
ros2 topic echo /usv_XX/mavros/state

# 应该看到
connected: true
armed: false
guided: false
...
```

### 2. 检查串口通信

```bash
# 查看串口设备
ls -l /dev/ttyACM*

# 测试串口
sudo screen /dev/ttyACM0 921600
# 应该看到 MAVLink 二进制数据流

# 权限问题
sudo usermod -aG dialout $USER
# 重新登录生效
```

### 3. 降低波特率测试

如果高波特率不稳定，可以降低：

```python
# 在 launch 文件中
'fcu_url': 'serial:///dev/ttyACM0:57600'  # 从 921600 降到 57600
```

### 4. 启用详细日志

```bash
# 临时启用调试日志
ros2 run mavros mavros_node --ros-args --log-level debug

# 或在 launch 文件中
output='screen',
arguments=['--ros-args', '--log-level', 'debug']
```

---

## 性能优化建议

### 1. 最小化插件集

只加载必需的 MAVROS 插件：
```python
'plugin_allowlist': [
    'sys_status',      # 系统状态
    'sys_time',        # 时间同步
    'command',         # 命令接口
    'local_position',  # 本地位置
    'setpoint_raw',    # 控制指令
    'global_position', # GPS 位置
    'gps_status',      # GPS 状态
]
```

**效果**：从加载 60+ 插件减少到 7 个，节省 ~80 秒

### 2. 禁用不必要的功能

```python
'vision_pose.enable': False,  # 禁用视觉定位
'param.use_mission_item_int': False,  # 禁用参数同步
'sys.min_version': [0, 0, 0],  # 跳过版本检查
```

### 3. 调整超时参数

```python
'conn.timeout': 5.0,  # 连接超时（秒）
'conn.heartbeat_rate': 1.0,  # 心跳频率（Hz）
```

---

## 飞控固件推荐配置

### ArduPilot Rover 关键参数

```
# 串口配置
SERIAL1_PROTOCOL = 2       # MAVLink2
SERIAL1_BAUD = 921600      # 波特率

# 系统 ID
SYSID_THISMAV = 1          # 飞控系统 ID（与 namespace 对应）

# EKF 配置
EK3_SRC1_POSXY = 3         # GPS 作为主要位置源
EK3_SRC1_VELXY = 3         # GPS 作为主要速度源

# GPS 配置
GPS_AUTO_CONFIG = 1        # 自动配置 GPS
GPS_TYPE = 1               # Auto detect

# 安全检查（生产环境）
ARMING_CHECK = 1           # 启用所有检查
# 或（测试环境）
ARMING_CHECK = 61918       # 跳过 GPS/指南针检查

# 性能优化
LOG_BACKEND_TYPE = 0       # 禁用日志后端（减少 CPU 负载）
```

---

## 监控和调试

### 实时监控命令

```bash
# 监控所有警告/错误
ros2 topic echo /rosout | grep -E "WARN|ERROR"

# 监控特定命令超时
ros2 topic echo /rosout | grep "CMD: Command"

# 监控飞控状态
watch -n 1 'ros2 topic echo /usv_02/mavros/state --once'
```

### 性能测试脚本

```bash
#!/bin/bash
# 测试 MAVROS 启动时间

START=$(date +%s)
ros2 launch usv_bringup usv_launch.py namespace:=usv_test &
PID=$!

# 等待连接
while ! ros2 topic echo /usv_test/mavros/state --once 2>/dev/null | grep -q "connected: true"; do
    sleep 0.5
done

END=$(date +%s)
ELAPSED=$((END - START))

echo "✅ MAVROS 连接成功，耗时: ${ELAPSED} 秒"
kill $PID
```

---

## 参考资料

- [MAVLink 命令列表](https://mavlink.io/en/messages/common.html#mav_commands)
- [MAVROS 配置文档](https://github.com/mavlink/mavros/tree/ros2/mavros/README.md)
- [ArduPilot 参数列表](https://ardupilot.org/rover/docs/parameters.html)
- [PX4 参数参考](https://docs.px4.io/main/en/advanced_config/parameter_reference.html)

---

**最后更新**: 2025-10-25  
**适用版本**: ROS 2 Humble/Iron, MAVROS 2.x, ArduPilot Rover 4.x
