# 坐标转换节点 - 全局GPS坐标模式测试指南

## 概述

**架构优化**: 坐标转换逻辑已移至 `coord_transform_node.py`，职责更清晰！

### 两种坐标模式对比

| 模式 | 节点 | 输出话题 | MAVLink消息 |
|------|------|---------|-------------|
| **局部坐标** | usv_control_node | setpoint_raw/local | SET_POSITION_TARGET_LOCAL_NED |
| **全局GPS** | coord_transform_node | setpoint_raw/global | SET_POSITION_TARGET_GLOBAL_INT |

## 功能说明

### 全局GPS坐标模式的工作流程

```
地面站 XYZ 目标点
    ↓
NavigateToPoint Action Server
    ↓
set_usv_target_position (PoseStamped)
    ↓
coord_transform_node (XYZ → GPS 转换)
    ↓
setpoint_raw/global (GlobalPositionTarget)
    ↓
MAVROS → 飞控 (SET_POSITION_TARGET_GLOBAL_INT)
```

### 局部坐标模式的工作流程

```
地面站 XYZ 目标点
    ↓
NavigateToPoint Action Server
    ↓
set_usv_target_position (PoseStamped)
    ↓
usv_control_node (直接转发)
    ↓
setpoint_raw/local (PositionTarget)
    ↓
MAVROS → 飞控 (SET_POSITION_TARGET_LOCAL_NED)
```

### 坐标转换公式

```python
# XYZ (ENU) → GPS (lat/lon/alt)
纬度差 = Y距离 / 111320.0
经度差 = X距离 / (111320.0 * cos(原点纬度))
海拔 = Z距离 + 原点海拔

GPS坐标 = 原点GPS + (纬度差, 经度差, 海拔差)
```

## 配置参数

### 文件位置
`usv_bringup/config/usv_params.yaml`

### 启用全局GPS坐标模式

```yaml
coord_transform_node:
  ros__parameters:
    enable_coord_transform: true          # 启用坐标转换
    use_global_position_target: true      # 使用 GlobalPositionTarget
    
    # GPS 原点配置（A0 基站）
    gps_origin_lat: 22.5180977
    gps_origin_lon: 113.9007239
    gps_origin_alt: -5.17
```

### 禁用全局GPS坐标模式（使用局部坐标）

```yaml
coord_transform_node:
  ros__parameters:
    enable_coord_transform: false         # 禁用坐标转换
```

**参数说明：**
- `enable_coord_transform`: 
  - `true` = 启用全局GPS坐标转换
  - `false` = 禁用（使用局部坐标）
- `use_global_position_target`:
  - `true` = 使用 GlobalPositionTarget (推荐，SET_POSITION_TARGET_GLOBAL_INT)
  - `false` = 使用 GeoPoseStamped (旧接口)
- `gps_origin_lat/lon/alt`: GPS原点坐标（A0基站）

## 测试步骤

### 1. 检查配置

```bash
# 查看配置文件
cat ~/usv_workspace/src/usv_bringup/config/usv_params.yaml | grep -A 10 "usv_control_node"
```

确认 `use_global_frame: true`

### 2. 启动USV节点

```bash
# 启动单个USV
ros2 launch usv_bringup usv_launch.py namespace:=usv_01
```

### 3. 检查运行的节点

```bash
# 检查 coord_transform_node 是否运行
ros2 node list | grep coord_transform

# 查看节点信息
ros2 node info /usv_01/coord_transform_node
```

### 4. 监听全局坐标话题

```bash
# 监听 setpoint_raw/global (全局GPS模式)
ros2 topic echo /usv_01/setpoint_raw/global

# 预期消息格式：
# header:
#   stamp: ...
#   frame_id: "map"
# coordinate_frame: 6  # FRAME_GLOBAL_INT
# type_mask: ...
# latitude: 22.518...  # GPS纬度
# longitude: 113.900...  # GPS经度
# altitude: -5.17  # 海拔
```

对比局部坐标模式：
```bash
# 局部坐标模式 (enable_coord_transform: false)
ros2 topic echo /usv_01/setpoint_raw/local

# 消息格式：
# coordinate_frame: 8  # FRAME_LOCAL_NED
# position:
#   x: 10.0  # 米
#   y: 5.0   # 米
#   z: 0.0   # 米
```

### 5. 发送测试目标点

```bash
# 发送一个测试目标点（东10米，北5米）
ros2 topic pub --once /usv_01/set_usv_target_position geometry_msgs/msg/PoseStamped \
'{
  header: {frame_id: "map"},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0}
  }
}'
```

### 6. 验证坐标转换

检查日志输出：

```bash
# 查看 coord_transform_node 日志
ros2 topic echo /rosout | grep coord_transform

# 预期日志：
# � XYZ→GPS: (10.00, 5.00, 0.00)m → 
#    (22.5181427°, 113.9008113°, -5.17m)
```

### 7. MAVLink 消息验证

使用 MAVProxy 或 QGC 监听 MAVLink 消息：

```bash
# 监听 UDP 端口
nc -ul 192.168.10.1 -p 14550 | hexdump -C

# 查找 MAVLink 消息 ID: 86 (SET_POSITION_TARGET_GLOBAL_INT)
# 字节序列应包含：
# - Message ID: 0x56 (86)
# - coordinate_frame: 0x06 (FRAME_GLOBAL_INT)
# - lat_int: GPS纬度 * 1e7
# - lon_int: GPS经度 * 1e7
# - alt: 海拔（米）
```

## 对比测试

### 场景1: 局部坐标模式
```yaml
use_global_frame: false
```
- 话题: `/usv_01/setpoint_raw/local`
- 消息: `PositionTarget`
- 坐标系: `FRAME_LOCAL_NED` (8)
- MAVLink: `SET_POSITION_TARGET_LOCAL_NED` (ID: 84)

### 场景2: 全局GPS坐标模式
```yaml
use_global_frame: true
```
- 话题: `/usv_01/setpoint_raw/global`
- 消息: `GlobalPositionTarget`
- 坐标系: `FRAME_GLOBAL_INT` (6)
- MAVLink: `SET_POSITION_TARGET_GLOBAL_INT` (ID: 86)

## 坐标转换示例

**输入（XYZ）:**
- X = 10.0 米（东）
- Y = 5.0 米（北）
- Z = 0.0 米（天）

**GPS 原点（A0基站）:**
- Lat = 22.5180977°
- Lon = 113.9007239°
- Alt = -5.17m

**转换计算:**
```python
dlat = 5.0 / 111320.0 = 0.0000449°
dlon = 10.0 / (111320.0 * cos(22.518°)) = 0.0000874°

lat = 22.5180977 + 0.0000449 = 22.5181426°
lon = 113.9007239 + 0.0000874 = 113.9008113°
alt = 0.0 + (-5.17) = -5.17m
```

**输出（GPS）:**
- Lat = 22.5181426°
- Lon = 113.9008113°
- Alt = -5.17m

## 故障排查

### 问题1: 飞控不响应全局坐标

**症状:** USV 不移动

**检查:**
```bash
# 1. 确认 MAVROS 发布全局坐标
ros2 topic hz /usv_01/setpoint_raw/global

# 2. 检查飞控参数
# ArduRover 参数: WPNAV_SPEED, WPNAV_RADIUS
```

**解决方案:**
- 确保飞控固件支持 `SET_POSITION_TARGET_GLOBAL_INT`
- ArduRover 4.0+ 应该支持此功能

### 问题2: GPS 坐标偏移

**症状:** USV 移动到错误位置

**检查:**
```bash
# 验证 GPS 原点配置
ros2 param get /usv_01/usv_control_node gps_origin_lat
ros2 param get /usv_01/usv_control_node gps_origin_lon
```

**解决方案:**
- 确认 GPS 原点坐标与 A0 基站匹配
- 检查 Home Position 是否正确设置

### 问题3: 坐标转换错误

**症状:** 日志显示异常 GPS 坐标

**检查:**
```bash
# 查看转换日志
ros2 topic echo /rosout | grep "发布.*目标点"
```

**解决方案:**
- 检查输入 XYZ 坐标范围（建议 < 1000米）
- 验证 GPS 原点海拔设置

## 性能对比

| 模式 | 优点 | 缺点 |
|------|------|------|
| **局部坐标** | • 计算简单<br>• 延迟低<br>• EKF原点直接使用 | • 需要 EKF 原点设置<br>• 多USV需统一原点 |
| **全局GPS** | • 飞控原生支持<br>• 无需 EKF 原点<br>• 跨区域任务友好 | • 需坐标转换<br>• 依赖GPS精度<br>• 计算开销稍高 |

## 建议使用场景

### 使用局部坐标模式 (推荐当前项目)
- ✅ 单区域作业（< 10km²）
- ✅ 高精度定位基站
- ✅ 多USV协作
- ✅ 低延迟要求

### 使用全局GPS模式
- ✅ 大范围巡航（> 10km²）
- ✅ 跨区域任务
- ✅ 无定位基站
- ✅ 与其他系统GPS坐标对接

## 相关文档

- [坐标系统设计说明](../../COORDINATE_SYSTEM_DESIGN.md)
- [EKF Origin 验证指南](../../EKF_ORIGIN_VERIFICATION_GUIDE.md)
- [MAVROS 文档](https://docs.px4.io/main/en/ros/mavros_installation.html)

---

**版本:** v1.0  
**日期:** 2025-11-14  
**作者:** USV Team
