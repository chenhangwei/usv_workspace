# MAVROS 启动优化指南

## 问题描述

USV 启动时 MAVROS 连接飞控耗时过长（约 97 秒），主要原因：
1. **插件串行加载**：MAVROS 默认加载 60+ 个插件，每个插件初始化都需要时间
2. **参数同步阻塞**：`param` 插件启动时尝试同步 900+ 个飞控参数，每个超时 3 次
3. **版本请求超时**：飞控响应慢导致多次重试
4. **GPS/EKF 未就绪**：EKF 等待 GPS 配置数据

## 已实施的优化

### 1. 插件白名单（已配置）

在 `usv_launch.py` 中配置了最小插件集：

```python
'plugin_allowlist': [
    'sys_status',      # 系统状态（必需）
    'sys_time',        # 时间同步（必需）
    'command',         # 命令接口（解锁/模式切换）
    'local_position',  # 本地位置（导航必需）
    'setpoint_raw',    # 原始设定点（控制必需）
    'global_position', # GPS 全局位置
    'gps_status',      # GPS 状态和卫星数
]
```

**效果**：从加载 60+ 个插件减少到 7 个，预计减少 80% 启动时间。

### 2. 移除 param 插件（重要）

**之前配置包含 `param` 插件**，导致启动时同步 900+ 参数。

**新配置已移除**，如需修改飞控参数，请使用地面站软件（Mission Planner/QGC）。

### 3. 连接超时优化

```python
'conn.timeout': 5.0,  # 减少超时等待时间
'conn.heartbeat_mav_type': 'MAV_TYPE_SURFACE_BOAT',  # 明确设备类型
```

## 预期效果

| 优化项             | 优化前耗时 | 优化后耗时 | 节省时间 |
|-------------------|-----------|-----------|---------|
| 插件加载           | 90 秒     | 5-8 秒    | ~82 秒  |
| 参数同步           | 不适用    | 0 秒      | N/A     |
| 版本请求           | 7 秒      | 5 秒      | 2 秒    |
| **总启动时间**     | **97 秒** | **10-15 秒** | **~82 秒** |

## 验证优化效果

### 1. 重新构建包

```bash
cd ~/usv_workspace
colcon build --packages-select usv_bringup
source install/setup.bash
```

### 2. 启动 USV 并观察日志

```bash
ros2 launch usv_bringup usv_launch.py namespace:=usv_02
```

**观察指标：**
- ✅ 插件加载日志应该只显示 7 个插件（而非 60+）
- ✅ 不应出现 `PR: request param #X timeout` 日志
- ✅ `MAVROS UAS started` 应在启动后 10-15 秒内出现

### 3. 检查插件加载情况

```bash
# 查看 MAVROS 话题（验证功能正常）
ros2 topic list | grep usv_02/mavros

# 应该看到：
# /usv_02/mavros/state           # sys_status 提供
# /usv_02/mavros/battery         # sys_status 提供
# /usv_02/mavros/local_position/pose  # local_position 提供
# /usv_02/mavros/setpoint_raw/local   # setpoint_raw 提供
# /usv_02/mavros/global_position/global  # global_position 提供
# /usv_02/mavros/gps_status/gps  # gps_status 提供
```

## 硬件相关优化

### 1. 检查串口波特率

确保飞控串口配置正确：

```bash
# 检查串口设备
ls -l /dev/ttyACM*

# 查看串口权限
sudo usermod -aG dialout $USER  # 添加当前用户到 dialout 组
# 重新登录生效
```

### 2. 飞控固件参数优化

在 Mission Planner/QGC 中设置：

```
SERIAL1_PROTOCOL = 2       # MAVLink2
SERIAL1_BAUD = 921600      # 波特率
LOG_BACKEND_TYPE = 0       # 禁用日志后端（减少 CPU 负载）
```

### 3. GPS 配置（如果 EKF 警告持续）

```
EK3_SRC1_POSXY = 3         # GPS 作为主要位置源
EK3_SRC1_VELXY = 3         # GPS 作为主要速度源
GPS_AUTO_CONFIG = 1        # 自动配置 GPS
ARMING_CHECK = 1           # 启用解锁检查（或设为 0 跳过检查，仅测试用）
```

## 故障排查

### 问题 1：仍然看到大量插件加载日志

**可能原因**：参数文件覆盖了 launch 文件配置

**解决方案**：
```bash
# 检查是否有其他参数文件
grep -r "plugin_allowlist" ~/usv_workspace/src/usv_bringup/

# 移除冲突的配置
```

### 问题 2：VER: broadcast request timeout

**可能原因**：飞控响应慢或串口通信问题

**解决方案**：
- 检查串口连接是否稳定
- 降低波特率测试（如 `57600`）
- 更新飞控固件到最新稳定版

### 问题 3：EKF3 waiting for GPS config data

**可能原因**：GPS 未正确配置或无信号

**解决方案**：
- 确保 GPS 天线放置在开阔位置
- 等待 GPS 获得定位（至少 4 颗卫星）
- 在地面站中配置 GPS 参数

### 问题 4：数据过期警告

```
[WARN] 数据过期 - Pose: 999.0s, State: 999.0s, Battery: 999.0s
```

**可能原因**：MAVROS 未完全启动或飞控未连接

**解决方案**：
- 等待 MAVROS 完全启动（看到 `CON: Got HEARTBEAT, connected`）
- 检查飞控是否正常上电
- 检查 MAVLink 系统 ID 是否匹配

## 进一步优化（可选）

### 1. 使用 MAVROS 的 `plugin_blocklist`

如果某些不需要的插件仍在加载，可以显式阻止：

```python
'plugin_blocklist': [
    'altitude', 'imu', 'rc_io', 'setpoint_position',
    'setpoint_velocity', 'waypoint', 'mission'
]
```

### 2. 延迟非关键节点启动

修改 `usv_launch.py`，使用 `TimerAction` 延迟启动传感器节点：

```python
from launch.actions import TimerAction

# 延迟 5 秒启动 LED 节点（等待 MAVROS 完成）
delayed_led_node = TimerAction(
    period=5.0,
    actions=[usv_led_node]
)
```

### 3. 并行启动多个 USV

使用脚本同时启动多艘 USV：

```bash
#!/bin/bash
# 并行启动脚本

ros2 launch usv_bringup usv_launch.py namespace:=usv_01 fcu_url:=serial:///dev/ttyACM0:921600 tgt_system:=1 &
ros2 launch usv_bringup usv_launch.py namespace:=usv_02 fcu_url:=serial:///dev/ttyACM1:921600 tgt_system:=2 &
ros2 launch usv_bringup usv_launch.py namespace:=usv_03 fcu_url:=serial:///dev/ttyACM2:921600 tgt_system:=3 &

wait  # 等待所有进程完成
```

## 性能监控

### 查看节点启动时间

```bash
# 查看节点 CPU 占用
top -p $(pgrep -d',' -f mavros_node)

# 查看 ROS 2 节点延迟
ros2 topic hz /usv_02/mavros/state
ros2 topic delay /usv_02/mavros/local_position/pose
```

### 启动时间测试脚本

```bash
#!/bin/bash
# 测试启动时间

START_TIME=$(date +%s)
ros2 launch usv_bringup usv_launch.py namespace:=usv_test &
LAUNCH_PID=$!

# 等待 HEARTBEAT 连接
while ! ros2 topic echo /usv_test/mavros/state --once | grep -q "connected: true"; do
    sleep 1
done

END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

echo "MAVROS 启动耗时: ${ELAPSED} 秒"

kill $LAUNCH_PID
```

## 参考资料

- [MAVROS 官方文档](https://github.com/mavlink/mavros/tree/ros2)
- [ArduPilot 参数列表](https://ardupilot.org/rover/docs/parameters.html)
- [ROS 2 Launch 优化](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

---

**最后更新**: 2025-10-25  
**适用版本**: ROS 2 Humble, MAVROS 2.x, ArduPilot Rover 4.x
