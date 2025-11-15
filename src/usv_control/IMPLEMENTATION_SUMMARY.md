# ✅ 坐标转换功能完成总结

## 📋 修改内容

### 1. **增强 `coord_transform_node.py`** ✨

**新增功能**:
- ✅ 支持两种输出格式：
  - `GlobalPositionTarget` → `setpoint_raw/global` (推荐，MAVLink ID: 86)
  - `GeoPoseStamped` → `setpoint_position/global` (旧接口)
- ✅ XYZ → GPS 坐标转换（使用A0基站作为原点）
- ✅ 可配置启用/禁用坐标转换

**关键代码**:
```python
if self.use_global_position_target:
    # 发布 GlobalPositionTarget
    global_msg.latitude = gps_coord['lat']
    global_msg.longitude = gps_coord['lon']
    global_msg.altitude = gps_coord['alt']
    self.global_target_pub.publish(global_msg)
```

---

### 2. **保持 `usv_control_node.py` 简洁** 🎯

**职责**:
- 订阅 `set_usv_target_position`
- 处理避障逻辑
- EKF原点就绪检查
- 发布到 `setpoint_raw/local` (局部坐标)

**未修改**: 保持原有简单架构，无坐标转换逻辑

---

### 3. **更新配置文件** ⚙️

**文件**: `usv_bringup/config/usv_params.yaml`

```yaml
coord_transform_node:
  ros__parameters:
    enable_coord_transform: true          # 启用坐标转换
    use_global_position_target: true      # 使用 GlobalPositionTarget
    gps_origin_lat: 22.5180977           # A0基站纬度
    gps_origin_lon: 113.9007239          # A0基站经度
    gps_origin_alt: -5.17                # A0基站海拔
```

---

## 🏗️ 系统架构

### 全局GPS坐标模式架构

```
地面站 → NavigateToPoint Action
           ↓
     set_usv_target_position (PoseStamped: X, Y, Z)
           ↓
    coord_transform_node
    (XYZ → GPS 转换)
           ↓
    setpoint_raw/global (GlobalPositionTarget)
           ↓
         MAVROS
           ↓
    SET_POSITION_TARGET_GLOBAL_INT (MAVLink ID: 86)
           ↓
        ArduRover 飞控
```

### 局部坐标模式架构

```
地面站 → NavigateToPoint Action
           ↓
     set_usv_target_position (PoseStamped: X, Y, Z)
           ↓
     usv_control_node
           ↓
    setpoint_raw/local (PositionTarget)
           ↓
         MAVROS
           ↓
    SET_POSITION_TARGET_LOCAL_NED (MAVLink ID: 84)
           ↓
        ArduRover 飞控
```

---

## 🎯 设计优势

### ✅ 职责分离
- `coord_transform_node`: 专注坐标转换
- `usv_control_node`: 专注控制逻辑
- 各司其职，易于维护

### ✅ 灵活切换
- 通过配置文件轻松切换模式
- 无需修改代码

### ✅ 向后兼容
- `usv_control_node` 保持原有功能
- 现有系统无需修改

### ✅ 可扩展
- 未来可添加其他坐标系转换
- 可独立升级坐标转换逻辑

---

## 📊 模式对比

| 特性 | 局部坐标 | 全局GPS |
|------|---------|---------|
| **转换节点** | ❌ | ✅ coord_transform_node |
| **话题** | setpoint_raw/local | setpoint_raw/global |
| **消息类型** | PositionTarget | GlobalPositionTarget |
| **MAVLink** | LOCAL_NED (84) | GLOBAL_INT (86) |
| **EKF依赖** | 强 | 弱 |
| **适用范围** | < 10km² | > 10km² |
| **计算开销** | 低 | 中 |

---

## 🧪 测试方法

### 快速测试脚本

```bash
# 运行测试脚本
cd /home/chenhangwei/usv_workspace/src/usv_control
./test_global_frame.sh usv_01
```

### 手动测试

```bash
# 1. 启动USV
ros2 launch usv_bringup usv_launch.py namespace:=usv_01

# 2. 检查坐标转换节点
ros2 node list | grep coord_transform

# 3. 监听全局坐标输出
ros2 topic echo /usv_01/setpoint_raw/global

# 4. 发送测试目标点
ros2 topic pub --once /usv_01/set_usv_target_position \
  geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 10.0, y: 5.0, z: 0.0}}}'

# 5. 查看转换日志
ros2 topic echo /rosout | grep "XYZ→GPS"
```

---

## 📁 新增文件

1. **`COORDINATE_ARCHITECTURE.md`** - 架构说明文档
2. **`TEST_GLOBAL_FRAME.md`** - 详细测试指南
3. **`test_global_frame.sh`** - 自动化测试脚本

---

## ⚙️ 配置示例

### 启用全局GPS模式

```yaml
coord_transform_node:
  ros__parameters:
    enable_coord_transform: true
    use_global_position_target: true
```

### 禁用全局GPS模式（使用局部坐标）

```yaml
coord_transform_node:
  ros__parameters:
    enable_coord_transform: false
```

---

## 🔧 MAVLink 消息格式

### SET_POSITION_TARGET_GLOBAL_INT (ID: 86)

```
Field                Type        Description
--------------------------------------------------
time_boot_ms         uint32      系统启动时间(ms)
target_system        uint8       目标系统ID
target_component     uint8       目标组件ID
coordinate_frame     uint8       坐标系 (6=GLOBAL_INT)
type_mask            uint16      忽略字段掩码
lat_int              int32       纬度 * 1e7
lon_int              int32       经度 * 1e7
alt                  float       海拔(米)
vx, vy, vz           float       速度(m/s)
afx, afy, afz        float       加速度(m/s²)
yaw                  float       偏航角(rad)
yaw_rate             float       偏航角速率(rad/s)
```

---

## 📖 相关文档

- **架构说明**: `src/usv_control/COORDINATE_ARCHITECTURE.md`
- **测试指南**: `src/usv_control/TEST_GLOBAL_FRAME.md`
- **坐标系统设计**: `src/COORDINATE_SYSTEM_DESIGN.md`
- **EKF验证**: `src/EKF_ORIGIN_VERIFICATION_GUIDE.md`

---

## 🎉 总结

✅ **已完成**:
1. ✅ `coord_transform_node` 增强支持 `GlobalPositionTarget`
2. ✅ 保持 `usv_control_node` 简洁
3. ✅ 配置文件更新
4. ✅ 测试脚本和文档完善
5. ✅ 职责分离，架构清晰

🚀 **下一步**:
1. 实际环境测试
2. 验证飞控响应
3. 性能对比测试
4. 根据测试结果优化

---

**版本**: v1.0  
**日期**: 2025-11-14  
**状态**: ✅ 完成
