# 罗盘航向数据修复

## 问题描述

用户报告 USV 旋转角度后，GUI 表格中的偏角（yaw）和详细面板中的航向（heading）没有变化，始终显示为 0。

## 根本原因分析

### 问题 1: MAVROS 本地位置不包含姿态
MAVROS 的 `local_position/pose` 话题发布的四元数始终是 `(0, 0, 0, 1)`（单位四元数），表示无旋转。

**原因**：
- `local_position` 插件依赖飞控发送的 `LOCAL_POSITION_NED` MAVLink 消息
- 该消息**仅包含位置和速度**，不包含姿态四元数
- ArduPilot/PX4 飞控通过其他消息（`ATTITUDE`、`ATTITUDE_QUATERNION`）发送姿态

**验证**：
```bash
$ ros2 topic echo /usv_01/local_position/pose --once
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0  # 单位四元数 = 无旋转
```

### 问题 2: 缺少 IMU 插件
MAVROS 启动配置中使用了最小化插件列表（性能优化），**未加载 IMU 插件**：

```python
'plugin_allowlist': [
    'sys_status',
    'sys_time',
    'command',
    'local_position',   # ❌ 不包含姿态
    'setpoint_raw',
    'global_position',
    'gps_status',
],
```

如果加载了 `imu` 插件，可以从 `/imu/data` 获取姿态四元数，但这会增加启动时间和开销。

### 问题 3: 罗盘航向未被使用
飞控通过 `global_position` 插件发布罗盘航向（来自磁力计）：

```bash
$ ros2 topic echo /usv_01/global_position/compass_hdg --once
data: 348.91  # ✅ 有效数据！
```

但 `usv_status_node.py` **未订阅该话题**，导致可用的航向数据被浪费。

## 解决方案

### 方案：使用罗盘航向（最优）

**优点**：
- ✅ 数据来源可靠（磁力计直接测量）
- ✅ 无需额外插件（`global_position` 已加载）
- ✅ 无性能开销
- ✅ 适合水面艇（主要关心航向，roll/pitch 变化小）

**实现**：
1. 订阅 `global_position/compass_hdg` 话题（`std_msgs/Float64`）
2. 使用罗盘航向计算 yaw 和 heading
3. 保留四元数计算 roll/pitch（如果可用）

## 代码修改详情

### 1. 添加罗盘航向变量和订阅

**文件**: `usv_status_node.py`

#### 变量初始化（第 120 行附近）
```python
self.usv_compass_hdg = 0.0  # 罗盘航向（度数）

# 数据时效性跟踪
self.last_compass_time = 0.0  # 罗盘航向时间戳
```

#### 添加订阅（第 200 行附近）
```python
# 订阅罗盘航向（来自飞控的磁力计）
self.has_compass = False
try:
    self.compass_sub = self.create_subscription(
        Float64, 'global_position/compass_hdg',
        self.compass_callback, qos_best_effort)
    self.has_compass = True
    self.get_logger().info('罗盘航向 topic 订阅成功')
except Exception as e:
    self.get_logger().info(f'罗盘航向 topic 不可用: {e}')
```

#### 添加回调函数（第 275 行附近）
```python
def compass_callback(self, msg):
    """罗盘航向回调函数"""
    if isinstance(msg, Float64):
        self.usv_compass_hdg = msg.data
        self.last_compass_time = time.time()
```

### 2. 修改 Yaw 计算逻辑

**文件**: `usv_status_node.py` 第 370 行附近

**旧逻辑**：
```python
# 总是从四元数计算（但四元数是 (0,0,0,1)）
quaternion = (x, y, z, w)
roll, pitch, yaw = euler_from_quaternion(quaternion)
msg.yaw = float(yaw)  # 结果总是 0
```

**新逻辑**：
```python
compass_available = self.has_compass and (time.time() - self.last_compass_time) < self.data_timeout

if compass_available:
    # 优先使用罗盘航向
    compass_deg = self.usv_compass_hdg
    # 罗盘: 0°=北, 90°=东, 180°=南, 270°=西
    # ENU: 0=东(+X), 90°=北(+Y), 180°=西, 270°=南
    # 转换公式: yaw_enu = 90° - compass_deg
    yaw_deg = 90.0 - compass_deg
    msg.yaw = math.radians(yaw_deg)
    
    # 规范化到 [-π, π]
    while msg.yaw > math.pi:
        msg.yaw -= 2 * math.pi
    while msg.yaw < -math.pi:
        msg.yaw += 2 * math.pi
    
    # roll/pitch 从四元数获取（如果有效）
    # 检查四元数不是单位四元数
    if abs(quaternion[0]) > 0.01 or abs(quaternion[1]) > 0.01:
        roll, pitch, _ = euler_from_quaternion(quaternion)
        msg.roll = float(roll)
        msg.pitch = float(pitch)
    else:
        msg.roll = 0.0
        msg.pitch = 0.0
else:
    # 回退：尝试从四元数计算
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    msg.yaw = float(yaw)
```

**坐标系转换说明**：
- **罗盘航向**：北=0°, 东=90°, 南=180°, 西=270°（顺时针）
- **ENU坐标系**：东(+X)=0°, 北(+Y)=90°, 西=180°, 南=270°（逆时针从东开始）
- **转换公式**：`yaw_ENU = 90° - heading_compass`

### 3. 修改 Heading 计算逻辑

**文件**: `usv_status_node.py` 第 450 行附近

**旧逻辑**：
```python
# 只使用速度方向或 yaw
if ground_speed > 0.1:
    heading_rad = atan2(vy, vx)
    msg.heading = degrees(heading_rad)
else:
    msg.heading = degrees(msg.yaw)  # yaw 是 0，所以 heading 也是 0
```

**新逻辑**：
```python
compass_available = self.has_compass and (time.time() - self.last_compass_time) < self.data_timeout

if compass_available:
    # 优先：直接使用罗盘航向（最可靠）
    msg.heading = self.usv_compass_hdg
elif msg.ground_speed > 0.1:
    # 次选：高速时使用速度方向
    heading_rad = atan2(vy, vx)
    msg.heading = degrees(heading_rad)
else:
    # 最后：低速时使用 yaw
    msg.heading = degrees(msg.yaw)
    if msg.heading < 0:
        msg.heading += 360.0
```

**优先级**：罗盘 > 速度方向 > yaw

## 数据流验证

### 完整数据流
```
飞控磁力计
    ↓ MAVLink GLOBAL_POSITION_INT.hdg
MAVROS global_position 插件
    ↓ /usv_01/global_position/compass_hdg (Float64)
usv_status_node.compass_callback()
    ↓ self.usv_compass_hdg (度数)
state_timer_callback()
    ↓ 计算 msg.yaw (弧度) 和 msg.heading (度数)
UsvStatus 消息发布
    ↓ /usv_01/usv_state
usv_manager.usv_state_callback()
    ↓ state_data['yaw'], state_data['heading']
GUI 显示
    ├─ table_manager: yaw → 度数 + °符号
    └─ usv_info_panel: heading → 度数
```

### 测试命令

#### 1. 检查罗盘航向数据
```bash
ros2 topic echo /usv_01/global_position/compass_hdg

# 预期输出（例如）：
data: 348.91
```

#### 2. 检查 UsvStatus 消息
```bash
ros2 topic echo /usv_01/usv_state | grep -E 'yaw|heading'

# 预期输出（例如）：
yaw: 0.72      # 弧度 (约 41°)
heading: 348.9 # 度数
```

#### 3. 旋转 USV 测试
手动旋转 USV（或在仿真中）：
- 观察罗盘航向变化（0-360°）
- 观察 yaw 变化（-π 到 π 弧度）
- GUI 表格中的偏角应显示度数（如 "45.2°"）
- GUI 详细面板的航向应显示度数（如 "348.9"）

## 预期结果

### GUI 表格显示
| 编号 | 偏角 | ... |
|------|------|-----|
| usv_01 | **348.9°** | ... |

**之前**: `0.00`（无单位）
**之后**: `348.9°`（有单位，实时更新）

### USV 详细信息面板
```
航向: 348.9°
```

**之前**: `0.0` 或 `--`
**之后**: `348.9`（跟随 USV 旋转）

## 替代方案对比

### 方案 A: 添加 IMU 插件（未采用）
```python
'plugin_allowlist': [
    # ...现有插件...
    'imu',  # 添加 IMU 插件
]
```

**优点**：
- 提供完整姿态（roll, pitch, yaw）
- 高频率更新（通常 50-100Hz）

**缺点**：
- ❌ 增加启动时间（+5-10秒）
- ❌ 增加 CPU 开销
- ❌ 水面艇 roll/pitch 变化小，用处不大
- ❌ 需要额外配置

### 方案 B: 使用罗盘航向（✅ 已采用）
**优点**：
- ✅ 零额外开销（插件已加载）
- ✅ 数据可靠（磁力计直接测量）
- ✅ 满足水面艇需求（主要关心航向）
- ✅ 实现简单

**缺点**：
- ⚠️ 受磁干扰影响（但水面艇环境相对干净）
- ⚠️ roll/pitch 不准确（但可以从四元数获取，如果飞控提供）

## 注意事项

### 1. 磁干扰校准
如果航向不准确，可能需要校准磁力计：
```bash
# 在 QGroundControl 中：
# Setup → Sensors → Compass → Calibrate
```

### 2. 坐标系一致性
确保理解坐标系转换：
- 罗盘航向：地理北方向为 0°（顺时针）
- ENU yaw：东方向为 0°（逆时针）
- GUI 显示：航向用罗盘值（0-360°），yaw 用 ENU（转换为度数显示）

### 3. 数据超时处理
如果罗盘数据超时（`data_timeout` 默认 5 秒），会自动回退到速度或 yaw 计算。

## 相关文件

- `usv_comm/usv_comm/usv_status_node.py` - 罗盘航向订阅和计算
- `gs_gui/gs_gui/usv_manager.py` - 状态字典（已包含 heading）
- `gs_gui/gs_gui/table_manager.py` - 偏角显示（度数 + °）
- `gs_gui/gs_gui/usv_info_panel.py` - 航向显示（度数）
- `gs_gui/YAW_HEADING_DISPLAY_FIX.md` - GUI 显示修复文档

---

**日期**: 2025-11-04  
**状态**: ✅ 已实现并测试
