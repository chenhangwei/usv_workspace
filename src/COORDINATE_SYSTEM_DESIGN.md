# USV 坐标系统设计说明

## 核心设计理念 ⭐

本系统采用**统一坐标系设计**：
- ✅ **全局坐标系 = USV本地坐标系 = 以定位基站A0为原点**
- ✅ 所有USV共享同一个坐标系，便于集群协作
- ✅ 无需复杂的坐标转换，简化导航逻辑

---

## 坐标系统架构

```
┌─────────────────────────────────────────────────────────┐
│ 定位基站A0 (GPS原点)                                      │
│ Lat: 22.5180977°N, Lon: 113.9007239°E, Alt: -4.8m      │
└─────────────────────────────────────────────────────────┘
                         ↓ [set_home设置]
┌─────────────────────────────────────────────────────────┐
│ 全局坐标系 (Map/Global)                                   │
│ - 原点: A0基站位置                                        │
│ - X轴: 东向 (East)                                       │
│ - Y轴: 北向 (North)                                      │
│ - Z轴: 天向 (Up)                                         │
└─────────────────────────────────────────────────────────┘
                         ║ 
                         ║ (同一坐标系，无需转换！)
                         ║
┌─────────────────────────────────────────────────────────┐
│ USV本地坐标系 (Local)                                     │
│ - 原点: **也是A0基站位置**（通过set_home设置）              │
│ - 坐标轴: 与全局坐标系完全相同                             │
│ - 所有USV使用同一坐标系                                   │
└─────────────────────────────────────────────────────────┘
                         ↑ [+area_center偏移]
┌─────────────────────────────────────────────────────────┐
│ XML任务坐标系 (Area Coordinate)                          │
│ - 原点: 任务区域中心（相对坐标）                           │
│ - 用途: 便于编写可重用的任务文件                           │
└─────────────────────────────────────────────────────────┘
```

---

## 关键概念说明

### 1. Home点 (A0基站)

**定义**: 定位基站A0的GPS坐标，作为所有USV共享的坐标系原点

**配置位置**: `usv_bringup/config/usv_params.yaml`

```yaml
auto_set_home_node:
  ros__parameters:
    use_current_gps: false          # ⚠️ 必须为false！
    home_latitude: 22.5180977       # A0基站纬度
    home_longitude: 113.9007239     # A0基站经度
    home_altitude: -4.8             # A0基站海拔
```

**设置时机**: USV启动后，`auto_set_home_node` 自动调用 `MAV_CMD_DO_SET_HOME` 将飞控的EKF原点设为A0基站位置

**作用**:
- ✅ 统一所有USV的坐标系
- ✅ 飞控的本地坐标 = 全局坐标（都相对A0）
- ✅ 简化多USV协作逻辑

---

### 2. Boot Pose (USV上电位姿)

**定义**: USV上电时在全局坐标系（以A0为原点）中的位置和航向

**作用**: 
- ✅ 记录USV初始位置，用于GUI显示
- ✅ 可用于任务规划或故障恢复
- ❌ **不作为坐标系原点**（这是关键！）

**数据结构**:
```python
usv_boot_pose = {
    'usv_01': {
        'x': 10.5,      # 相对A0基站的东向偏移（米）
        'y': 15.2,      # 相对A0基站的北向偏移（米）
        'z': 0.0,       # 相对A0基站的高度偏移（米）
        'yaw': 1.57     # 航向角（弧度，0=正北）
    }
}
```

---

### 3. Area Center (任务坐标系原点)

**定义**: 任务区域中心在全局坐标系中的位置

**配置位置**: `gs_bringup/config/gs_params.yaml`

```yaml
main_gui_app:
  ros__parameters:
    # 任务区域中心相对A0基站的偏移
    area_center_x: 50.0   # 东向50米
    area_center_y: 30.0   # 北向30米
    area_center_z: 0.0    # 高度0米
```

**作用**:
- ✅ 允许XML任务文件使用相对坐标（以任务区域中心为原点）
- ✅ 同一任务文件可用于不同场地（只需修改area_center）
- ✅ 提高任务文件的可重用性

---

## 坐标转换流程

### 完整转换链

```
XML任务点 (0, 0, 0)
    ↓ [+area_center]
全局坐标 (50, 30, 0) ← 相对A0基站
    ↓ [无需转换！]
USV本地坐标 (50, 30, 0) ← 也是相对A0基站
    ↓ [发送给飞控]
飞控执行导航 → 前往A0基站东50m、北30m的位置
```

### 代码实现

**Step 1: Area → Global** (`cluster_controller.py`)

```python
def _area_to_global(self, p_area):
    """XML任务坐标 → 全局坐标"""
    return {
        'x': area_center_x + p_area['x'],
        'y': area_center_y + p_area['y'],
        'z': area_center_z + p_area['z']
    }
```

**Step 2: Global → USV Local** (`cluster_controller.py`)

```python
def _global_to_usv_local(self, usv_id, p_global):
    """全局坐标 → USV本地坐标（实际上无需转换）"""
    # 因为全局坐标系 = USV本地坐标系（都以A0为原点）
    return p_global  # 直接返回！
```

**Step 3: 发送给飞控** (`ground_station_node.py`)

```python
# 坐标转换
p_global = self.cluster_controller._area_to_global(pos)
p_local = self.cluster_controller._global_to_usv_local(usv_id, p_global)

# 发送导航目标（p_local与p_global数值相同）
self.send_nav_goal_via_action(usv_id, p_local['x'], p_local['y'], p_local['z'], yaw)
```

---

## 使用示例

### 场景1: 单个任务点导航

**任务文件** (`task.xml`):
```xml
<step id="1">
    <usv id="usv_01">
        <position x="10" y="5" z="0"/>  <!-- 任务区域中心东10m、北5m -->
        <yaw>0</yaw>
    </usv>
</step>
```

**配置** (`gs_params.yaml`):
```yaml
area_center_x: 50.0  # 任务区域中心在A0基站东50m
area_center_y: 30.0  # 任务区域中心在A0基站北30m
```

**坐标转换过程**:
1. XML坐标: (10, 5, 0)
2. 全局坐标: (50+10, 30+5, 0) = (60, 35, 0)
3. USV本地坐标: (60, 35, 0) ← 与全局坐标相同
4. 飞控执行: 前往A0基站东60m、北35m的位置

---

### 场景2: 多USV集群任务

**优势**: 所有USV在同一坐标系下，便于协同

```xml
<step id="1">
    <usv id="usv_01">
        <position x="0" y="0" z="0"/>    <!-- 任务区域中心 -->
    </usv>
    <usv id="usv_02">
        <position x="10" y="0" z="0"/>   <!-- 中心东10m -->
    </usv>
    <usv id="usv_03">
        <position x="5" y="8.66" z="0"/> <!-- 形成三角形 -->
    </usv>
</step>
```

**结果**: 三艘USV在全局坐标系中形成等边三角形（边长10m）

---

## 与其他系统的对比

### 传统ROS导航系统（不适用于本系统）

```
全局坐标系 (Map)
    ↓ [TF变换]
机器人坐标系 (Base_link) ← 以机器人为原点
    ↓ [路径规划]
控制指令
```

**问题**: 每个机器人有独立的本地坐标系，多机器人协作需要复杂的TF树

---

### 本系统设计（更简单）

```
全局坐标系 (Map) = USV本地坐标系 ← 都以A0为原点
    ↓ [无需转换]
控制指令
```

**优势**: 
- ✅ 无需TF树维护
- ✅ 多USV天然在同一坐标系
- ✅ 简化代码逻辑

---

## 常见误区

### ❌ 误区1: Boot Pose是坐标系原点

**错误理解**: 
> USV本地坐标系以USV上电位置为原点，需要减去Boot Pose进行坐标转换

**正确理解**:
> USV本地坐标系以A0基站为原点（通过set_home设置），Boot Pose只是记录USV上电时的位置

---

### ❌ 误区2: 需要旋转变换

**错误理解**:
> 需要根据USV航向角旋转坐标，将全局坐标转换为USV机体坐标系

**正确理解**:
> 导航目标点使用全局坐标（ENU），飞控内部会处理航向控制，外部无需旋转

---

### ❌ 误区3: use_current_gps应该为true

**错误理解**:
> 每个USV使用自己的当前位置作为Home点

**正确理解**:
> 所有USV必须使用同一个Home点（A0基站），`use_current_gps`必须为false

---

## 验证方法

### 测试1: 检查Home点设置

```bash
# 启动USV后，查看日志
ros2 topic echo /usv_01/mavros/home_position/home

# 应该看到所有USV的Home点都是A0基站坐标
# latitude: 22.5180977
# longitude: 113.9007239
```

### 测试2: 坐标转换验证

```python
# 在 cluster_controller.py 中启用调试日志
gs_params.yaml:
  debug_coordinates: true

# 发送导航目标后，查看日志
# 应该看到 Global坐标 = Local坐标（数值完全相同）
```

### 测试3: 实地测试

1. 在A0基站附近放置USV
2. 发送目标点：相对A0东10m、北10m
3. USV应该正确导航到该位置

---

## 故障排查

### 问题1: USV导航到错误位置

**可能原因**:
- Home点未正确设置（检查 `use_current_gps` 是否为false）
- Area Center配置错误

**解决方法**:
```bash
# 检查Home点
ros2 topic echo /usv_01/mavros/home_position/home

# 检查Area Center
ros2 param get /main_gui_app area_center_x
```

---

### 问题2: 不同USV坐标不一致

**原因**: 某些USV的Home点设置不同

**解决方法**: 确保所有USV的 `usv_params.yaml` 中Home点坐标完全一致

---

## 配置检查清单

- [ ] `usv_params.yaml` 中 `use_current_gps: false`
- [ ] `home_latitude/longitude/altitude` 设置为A0基站坐标
- [ ] 所有USV的Home点坐标完全一致
- [ ] `gs_params.yaml` 中 `area_center_x/y/z` 根据任务区域设置
- [ ] 启动USV后检查Home点是否正确设置

---

## 总结

**坐标系统设计的优雅之处**:

✅ **简单**: 全局坐标 = 本地坐标，无需复杂转换  
✅ **统一**: 所有USV共享同一坐标系  
✅ **灵活**: Area Center支持任务文件重用  
✅ **可靠**: 基于定位基站，坐标精度高  

**关键要点**:
1. A0基站 = Home点 = 坐标系原点
2. Boot Pose ≠ 坐标系原点
3. 全局坐标 = USV本地坐标（无需转换）
4. 所有USV必须使用相同的Home点

---

**文档版本**: v2.0 (2025-11-04)  
**维护者**: USV Team  
**最后更新**: 基于用户澄清重新编写，明确A0基站作为统一坐标系原点的设计
