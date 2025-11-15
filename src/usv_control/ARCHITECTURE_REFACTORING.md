# 🔄 架构重构：双节点协作模式

## 📌 问题分析

之前的设计中，**两个节点同时发送控制指令到飞控**：

```
❌ 旧架构（冲突）:

地面站 → set_usv_target_position ─┬→ coord_transform_node → setpoint_raw/global
                                   │
                                   └→ usv_control_node → setpoint_raw/local

避障 → avoidance_position → usv_control_node → setpoint_raw/local
```

**冲突原因**：
- `coord_transform_node` 和 `usv_control_node` 都订阅 `set_usv_target_position`
- 两个节点同时发布到 MAVROS，飞控会混淆

---

## ✅ 解决方案：互斥控制模式

### 新架构

```
✅ 新架构（互斥）:

模式1：全局GPS模式（推荐）
┌────────────────────────────────────────────────────────────────┐
│ enable_coord_transform: true                                   │
│ enable_local_control: false                                    │
└────────────────────────────────────────────────────────────────┘

地面站 → set_usv_target_position ──→ coord_transform_node ──┐
                                                            ├→ setpoint_raw/global
避障 → avoidance_position ─────────→ coord_transform_node ──┘
                                     (XYZ→GPS转换)
                                     
usv_control_node: ⏸️  不发送控制指令（只接收数据）


模式2：局部坐标模式（传统）
┌────────────────────────────────────────────────────────────────┐
│ enable_coord_transform: false                                  │
│ enable_local_control: true                                     │
└────────────────────────────────────────────────────────────────┘

地面站 → set_usv_target_position ──→ usv_control_node ──┐
                                                        ├→ setpoint_raw/local
避障 → avoidance_position ─────────→ usv_control_node ──┘

coord_transform_node: ⏸️  禁用（不参与控制）
```

---

## 🔧 代码修改

### 1. `usv_control_node.py`

#### 新增参数
```python
self.declare_parameter('enable_local_control', True)
self.enable_local_control = bool(self.get_parameter('enable_local_control').value)
```

#### publish_target() 增加检查
```python
def publish_target(self):
    # 检查是否启用局部控制
    if not self.enable_local_control:
        return  # 如果禁用，直接返回，不发送任何控制指令
    
    # ... 原有逻辑 ...
```

#### 启动日志
```python
if not self.enable_local_control:
    self.get_logger().warning('⚠️  局部控制已禁用 - 本节点不会发送控制指令')
    self.get_logger().info('💡 坐标转换由 coord_transform_node 处理')
else:
    self.get_logger().info('✅ 局部控制已启用 - 使用 FRAME_LOCAL_NED')
```

---

### 2. `coord_transform_node.py`

#### 新增避障目标点订阅
```python
# 订阅避障 XYZ 目标点（从 usv_avoidance_node）
self.avoidance_target_sub = self.create_subscription(
    PositionTarget,
    'avoidance_position',
    self.avoidance_target_callback,
    qos_reliable
)
```

#### 新增避障回调函数
```python
def avoidance_target_callback(self, msg: PositionTarget):
    """接收避障节点的 XYZ 目标点，转换为 GPS 坐标发送给飞控"""
    x = msg.position.x
    y = msg.position.y
    z = msg.position.z
    
    gps_coord = self._xyz_to_gps(x, y, z)
    
    # 发布到 setpoint_raw/global
    # ...
```

---

### 3. `usv_params.yaml`

```yaml
coord_transform_node:
  ros__parameters:
    enable_coord_transform: true            # ✅ 启用全局GPS模式
    use_global_position_target: true
    gps_origin_lat: 22.5180977
    gps_origin_lon: 113.9007239
    gps_origin_alt: -5.17

usv_control_node:
  ros__parameters:
    enable_local_control: false             # ⛔ 禁用局部控制（避免冲突）
    publish_rate: 20.0
    frame_id: 'map'
    coordinate_frame: 8
```

---

## 🎯 配置切换

### 使用全局GPS模式（推荐）

```yaml
coord_transform_node:
  ros__parameters:
    enable_coord_transform: true

usv_control_node:
  ros__parameters:
    enable_local_control: false
```

**效果**：
- ✅ `coord_transform_node` 处理地面站 + 避障目标点
- ✅ 转换为 GPS 坐标发送到 `setpoint_raw/global`
- ⛔ `usv_control_node` 不发送控制指令

---

### 使用局部坐标模式（传统）

```yaml
coord_transform_node:
  ros__parameters:
    enable_coord_transform: false

usv_control_node:
  ros__parameters:
    enable_local_control: true
```

**效果**：
- ⛔ `coord_transform_node` 禁用
- ✅ `usv_control_node` 处理地面站 + 避障目标点
- ✅ 发送到 `setpoint_raw/local` (FRAME_LOCAL_NED)

---

## 📊 数据流对比

### 全局GPS模式

```
地面站
  │ set_usv_target_position
  │ (PoseStamped: X, Y, Z)
  ↓
coord_transform_node
  │ XYZ → GPS 转换
  │ setpoint_raw/global
  │ (GlobalPositionTarget: lat, lon, alt)
  ↓
MAVROS
  │ SET_POSITION_TARGET_GLOBAL_INT (MAVLink ID: 86)
  ↓
飞控 (ArduRover)

避障
  │ avoidance_position
  │ (PositionTarget: X, Y, Z)
  ↓
coord_transform_node
  │ XYZ → GPS 转换
  │ setpoint_raw/global
  ↓
MAVROS → 飞控
```

### 局部坐标模式

```
地面站
  │ set_usv_target_position
  │ (PoseStamped: X, Y, Z)
  ↓
usv_control_node
  │ setpoint_raw/local
  │ (PositionTarget: X, Y, Z)
  ↓
MAVROS
  │ SET_POSITION_TARGET_LOCAL_NED (MAVLink ID: 84)
  ↓
飞控 (ArduRover)

避障
  │ avoidance_position
  │ (PositionTarget: X, Y, Z)
  ↓
usv_control_node
  │ setpoint_raw/local
  ↓
MAVROS → 飞控
```

---

## 🧪 测试验证

### 1. 检查节点状态

```bash
# 启动 USV
ros2 launch usv_bringup usv_launch.py namespace:=usv_01

# 检查节点是否启动
ros2 node list | grep -E "coord_transform|usv_control"
```

### 2. 查看日志（确认模式）

```bash
# coord_transform_node 日志
ros2 topic echo /rosout | grep "coord_transform"

# 应该看到：
# ✅ XYZ→GPS 坐标转换节点已启动
# 📥 订阅: set_usv_target_position (地面站 XYZ)
# 📥 订阅: avoidance_position (避障 XYZ)
# 📤 发布: setpoint_raw/global (GlobalPositionTarget)

# usv_control_node 日志
ros2 topic echo /rosout | grep "usv_control"

# 应该看到：
# ⚠️  局部控制已禁用 - 本节点不会发送控制指令
# 💡 坐标转换由 coord_transform_node 处理
```

### 3. 监听话题

```bash
# 全局GPS模式应该有输出
ros2 topic echo /usv_01/setpoint_raw/global

# 局部坐标模式应该无输出（因为禁用了）
ros2 topic echo /usv_01/setpoint_raw/local
```

### 4. 发送测试目标点

```bash
# 测试地面站目标点
ros2 topic pub --once /usv_01/set_usv_target_position \
  geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 10.0, y: 5.0, z: 0.0}}}'

# 查看 coord_transform_node 日志（应该有 XYZ→GPS 转换）
```

---

## ⚠️ 注意事项

### 1. 配置一致性

**必须保证两个参数互斥**：

```yaml
# ✅ 正确配置1
enable_coord_transform: true
enable_local_control: false

# ✅ 正确配置2
enable_coord_transform: false
enable_local_control: true

# ❌ 错误配置（同时启用会冲突）
enable_coord_transform: true
enable_local_control: true

# ❌ 错误配置（同时禁用无控制）
enable_coord_transform: false
enable_local_control: false
```

### 2. 避障逻辑

- **全局模式**：避障目标点由 `coord_transform_node` 转换并发送
- **局部模式**：避障目标点由 `usv_control_node` 直接发送

### 3. EKF 原点

- **全局模式**：使用 A0 基站作为 GPS 原点
- **局部模式**：依赖飞控 EKF 原点（需要设置 Home Position）

---

## 📝 总结

### 优势

✅ **避免冲突**：两个节点不会同时发送控制指令  
✅ **职责清晰**：`coord_transform_node` 专注坐标转换，`usv_control_node` 专注控制逻辑  
✅ **灵活切换**：通过配置文件轻松切换模式  
✅ **避障支持**：两种模式都支持避障功能  

### 建议配置

**推荐：全局GPS模式**（适合多USV、大范围任务）
```yaml
enable_coord_transform: true
enable_local_control: false
```

**备选：局部坐标模式**（适合单USV、小范围任务）
```yaml
enable_coord_transform: false
enable_local_control: true
```

---

**版本**: v2.0  
**日期**: 2025-11-14  
**状态**: ✅ 已完成重构
