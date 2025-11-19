# USV 项目话题和 Action 完整分析

## 📊 分析摘要

本文档基于对 USV 项目源码的完整分析，列出了所有需要在 Domain Bridge 中配置的话题和 Action。

**分析日期**: 2025-11-18  
**项目结构**: 3艘USV (Domain 11, 12, 13) + 1个地面站 (Domain 99)

---

## 1️⃣ 核心消息类型

### 自定义消息 (common_interfaces)

| 消息类型 | 文件 | 用途 |
|---------|------|------|
| `UsvStatus` | `common_interfaces/msg/UsvStatus.msg` | USV 综合状态信息 |
| `UsvSetPoint` | `common_interfaces/msg/UsvSetPoint.msg` | USV 目标点设置 |

### Action 定义

| Action 名称 | 文件 | 用途 |
|------------|------|------|
| `NavigateToPoint` | `common_interfaces/action/NavigateToPoint.action` | 导航到目标点 |

### Service 定义

| Service 名称 | 文件 | 用途 | 状态 |
|-------------|------|------|------|
| `MissionControl` | `common_interfaces/srv/MissionControl.srv` | 任务控制 | 空文件，未实现 |

---

## 2️⃣ USV -> 地面站 (状态上报)

### A. 核心状态话题

| 话题名称 | 消息类型 | 节点 | 频率 | 说明 |
|---------|---------|------|------|------|
| `usv_XX/usv_state` | `UsvStatus` | usv_status_node | 1Hz | **最重要** 综合状态 |
| `usv_XX/usv_temperature` | `Float32` | usv_status_node | 1Hz | CPU 温度 |
| `usv_XX/low_voltage_mode` | `Bool` | usv_status_node | 事件触发 | 低电压警告 |

### B. MAVROS 状态话题

| 话题名称 | 消息类型 | 来源 | 频率 | 说明 |
|---------|---------|------|------|------|
| `usv_XX/mavros/state` | `State` | MAVROS | 10Hz | 飞控状态 |
| `usv_XX/mavros/battery` | `BatteryState` | MAVROS | 1Hz | 电池状态 |
| `usv_XX/mavros/extended_state` | `ExtendedState` | MAVROS | 1Hz | 扩展状态 |
| `usv_XX/mavros/sys_status` | `SysStatus` | MAVROS | 1Hz | 系统状态 |
| `usv_XX/mavros/statustext/recv` | `StatusText` | MAVROS | 事件触发 | 飞控消息 |

### C. 位置和导航话题

| 话题名称 | 消息类型 | 来源 | 频率 | 说明 |
|---------|---------|------|------|------|
| `usv_XX/mavros/local_position/pose` | `PoseStamped` | MAVROS | 50Hz | 本地位置 |
| `usv_XX/mavros/global_position/global` | `NavSatFix` | MAVROS | 10Hz | GPS 全球位置 |
| `usv_XX/mavros/local_position/velocity_local` | `TwistStamped` | MAVROS | 50Hz | 本地速度 |
| `usv_XX/mavros/home_position/home` | `HomePosition` | MAVROS | 0.5Hz | Home 位置 |
| `usv_XX/local_position/pose_from_gps` | `PoseStamped` | coord_transform_node | 10Hz | GPS 转换位置 |

### D. GPS 详细信息

| 话题名称 | 消息类型 | 来源 | 频率 | 说明 |
|---------|---------|------|------|------|
| `usv_XX/mavros/gpsstatus/gps1/raw` | `GPSRAW` | MAVROS | 5Hz | GPS 原始数据 |
| `usv_XX/mavros/altitude` | `Altitude` | MAVROS | 10Hz | 高度信息 |

### E. LED 状态反馈

| 话题名称 | 消息类型 | 节点 | 频率 | 说明 |
|---------|---------|------|------|------|
| `usv_XX/led_state` | `String` | usv_led_node | 事件触发 | LED 状态回传 |

### F. TF 变换 (可选)

| 话题名称 | 消息类型 | 来源 | 频率 | 说明 |
|---------|---------|------|------|------|
| `usv_XX/tf` | `TFMessage` | tf2 | 变化时 | 动态坐标变换 |
| `usv_XX/tf_static` | `TFMessage` | tf2 | 启动时 | 静态坐标变换 |

---

## 3️⃣ 地面站 -> USV (控制指令)

### A. 导航控制

| 话题名称 | 消息类型 | 节点 | 说明 |
|---------|---------|------|------|
| `usv_XX/set_usv_target_position` | `PoseStamped` | usv_control_node | 目标位置设定 |

### B. 模式和解锁控制

| 话题名称 | 消息类型 | 节点 | 说明 |
|---------|---------|------|------|
| `usv_XX/set_usv_mode` | `String` | usv_command_node | 模式切换 (GUIDED/MANUAL/AUTO/HOLD) |
| `usv_XX/set_usv_arming` | `String` | usv_command_node | 解锁/上锁 (ARM/DISARM) |

### C. 外设控制

| 话题名称 | 消息类型 | 节点 | 说明 |
|---------|---------|------|------|
| `usv_XX/gs_led_command` | `String` | usv_led_node | LED 控制命令 |
| `usv_XX/gs_sound_command` | `String` | usv_sound_node | 声音控制命令 |
| `usv_XX/gs_action_command` | `String` | (预留) | 动作控制命令 |

---

## 4️⃣ Action 服务器

### NavigateToPoint Action

**Action Server**: `usv_XX/navigate_to_point`  
**节点**: `navigate_to_point_server.py`  
**说明**: 导航到指定目标点，提供实时反馈

#### Goal (目标)
```
geometry_msgs/PoseStamped goal
float32 timeout
```

#### Result (结果)
```
bool success
uint8 error_code
string message
```

#### Feedback (反馈)
```
float32 distance_to_goal
float32 heading_error
float32 estimated_time
```

**Domain Bridge 处理**: Action 基于 topic 实现，Domain Bridge 会自动转发底层的 goal/result/feedback topics。

---

## 5️⃣ USV 内部话题 (不需要跨 Domain)

以下话题用于 USV 内部通信，**不需要**在 Domain Bridge 中配置：

| 话题名称 | 用途 | 说明 |
|---------|------|------|
| `usv_XX/mavros/setpoint_raw/local` | 发送目标点到飞控 | usv_control_node 内部使用 |
| `usv_XX/avoidance_position` | 避障目标点 | 避障系统内部 |
| `usv_XX/avoidance_flag` | 避障标志 | 避障系统内部 |
| `usv_XX/ultrasonic_radar_range` | 超声波雷达 | 传感器数据 (可选转发) |
| `usv_XX/laser_scan` | 激光扫描 | 传感器数据 (可选转发) |
| `usv_XX/vision_pose/pose` | UWB 定位 | 传感器数据 (可选转发) |

---

## 6️⃣ MAVROS 服务调用 (不需要跨 Domain)

以下服务由 `usv_command_node` 在 USV 本地调用，**不需要**跨 Domain 转发：

| 服务名称 | 服务类型 | 用途 |
|---------|---------|------|
| `usv_XX/mavros/set_mode` | `SetMode` | 设置飞控模式 |
| `usv_XX/mavros/cmd/arming` | `CommandBool` | 解锁/上锁 |

**工作流程**:
1. 地面站发布话题: `usv_XX/set_usv_mode` 或 `usv_XX/set_usv_arming`
2. Domain Bridge 转发话题到 USV
3. USV 上的 `usv_command_node` 接收话题
4. `usv_command_node` 调用本地 MAVROS 服务

---

## 7️⃣ 关键节点功能总结

| 节点 | 包 | 主要功能 |
|------|---|----------|
| `usv_status_node` | usv_comm | 聚合 USV 状态，发布 UsvStatus |
| `navigate_to_point_server` | usv_comm | Action 服务器，导航控制 |
| `usv_control_node` | usv_control | 目标点控制，发送到飞控 |
| `usv_command_node` | usv_control | 处理模式和解锁命令 |
| `coord_transform_node` | usv_control | GPS 坐标转换 |
| `usv_avoidance_node` | usv_control | 避障控制 |
| `ground_station_node` | gs_gui | 地面站主节点，GUI 界面 |
| `usv_manager` | gs_gui | USV 管理，话题订阅/发布 |

---

## 8️⃣ Domain Bridge 配置总结

### 配置统计

| 分类 | 每艘USV话题数 | 3艘USV总计 |
|------|-------------|-----------|
| 状态上报 (USV -> GS) | ~20 | ~60 |
| 控制指令 (GS -> USV) | ~6 | ~18 |
| TF 变换 (可选) | ~2 | ~6 |
| **总计** | **~28** | **~84** |

### 推荐配置策略

1. **必须配置**: 
   - `usv_state` (最重要的综合状态)
   - 控制命令 (mode, arming, target)
   
2. **高优先级**:
   - MAVROS 基础状态 (state, battery)
   - 位置信息 (local_position, global_position)
   
3. **中等优先级**:
   - 扩展状态、系统状态
   - LED 状态反馈
   
4. **低优先级/可选**:
   - TF 变换 (仅用于可视化)
   - GPS 原始数据 (调试用)
   - 传感器数据 (根据需求)

---

## 9️⃣ 使用建议

### 启动顺序

1. 启动 USV 节点 (各自的 Domain 11/12/13)
2. 启动 domain_bridge (地面站 Domain 99)
3. 启动地面站 GUI

### 验证通信

```bash
# 在地面站 (Domain 99)
export ROS_DOMAIN_ID=99

# 查看 USV 话题
ros2 topic list | grep usv_

# 监听状态
ros2 topic echo /usv_01/usv_state

# 查看 Action
ros2 action list | grep navigate

# 发送控制命令
ros2 topic pub /usv_01/set_usv_mode std_msgs/msg/String "data: 'GUIDED'" --once
```

### 性能监控

```bash
# 监控话题频率
ros2 topic hz /usv_01/usv_state

# 监控带宽
ros2 topic bw /usv_01/mavros/local_position/pose

# 检查延迟
ros2 topic echo /usv_01/usv_state/header/stamp
```

---

## 🔟 故障排查

### 常见问题

1. **看不到 USV 话题**
   - 检查 Domain ID 设置
   - 检查 domain_bridge 运行状态
   - 检查网络连通性

2. **Action 调用失败**
   - 确认 Action 服务器启动
   - 检查底层 topics 是否转发
   - 查看 domain_bridge 日志

3. **控制指令无响应**
   - 确认 USV 节点正常运行
   - 检查话题名称拼写
   - 验证消息类型匹配

---

**维护者**: chenhangwei  
**最后更新**: 2025-11-18
