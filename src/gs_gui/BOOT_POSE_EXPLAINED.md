# Boot Pose 机制详解

## 📌 什么是 Boot Pose？

**Boot Pose**（启动位姿/上电原点）是指 **USV 上电启动时的位置和航向**。系统会把这个位置作为该 USV 的本地坐标系原点 `(0, 0, 0)`。

## 🎯 Boot Pose 的作用

### 核心问题：为什么需要 Boot Pose？

在多 USV 集群系统中存在一个坐标系统挑战：

```
问题场景：
- USV的飞控（ArduPilot/PX4）使用自己的本地坐标系
- 飞控启动时，当前位置被设定为本地坐标原点 (0, 0, 0)
- 但任务需要在全局地图坐标系中规划
- 如何把全局任务坐标转换为每艘USV的本地坐标？
```

**Boot Pose 就是解决这个问题的关键！**

## 🔄 完整的坐标转换流程

系统使用 **三层坐标转换**：

```
┌─────────────────────────────────────────────────────────────┐
│  XML任务文件 (Area相对坐标)                                   │
│  例如：三角形巡航 (0,0) → (10,0) → (5,8.66)                  │
└─────────────────────────────────────────────────────────────┘
                           ↓
              【转换1: Area → Global】
              添加 area_center 偏移量
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  全局地图坐标 (Map全局坐标)                                   │
│  例如：(100,50) → (110,50) → (105,58.66)                    │
└─────────────────────────────────────────────────────────────┘
                           ↓
              【转换2: Global → USV Local】
              减去 boot_pose 并旋转
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  USV本地坐标 (飞控执行坐标)                                   │
│  例如：(5,3) → (15,3) → (10,11.66)                          │
│  这是飞控实际收到并执行的坐标                                  │
└─────────────────────────────────────────────────────────────┘
```

### 转换公式详解

**步骤1: Area → Global**
```python
global_x = area_center_x + area_x
global_y = area_center_y + area_y
global_z = area_center_z + area_z
```

**步骤2: Global → USV Local（使用 Boot Pose）**
```python
# 1. 计算相对于 boot pose 的偏移
dx = global_x - boot_pose_x
dy = global_y - boot_pose_y
dz = global_z - boot_pose_z

# 2. 考虑 USV 启动时的航向角旋转
theta = -boot_pose_yaw  # 负号表示逆旋转
local_x = cos(theta) * dx - sin(theta) * dy
local_y = sin(theta) * dx + cos(theta) * dy
local_z = dz
```

## 💡 实际案例说明

### 场景设置
- **任务区域**：湖面中心作为 Area Center
- **全局坐标系**：GPS 坐标或 ENU 本地坐标系
- **USV-01**：在码头 A 启动
- **USV-02**：在码头 B 启动（不同位置）

### 没有 Boot Pose 的问题

```
❌ 错误情况（不使用 Boot Pose）：
任务：巡航到点 (10, 5)
- USV-01 收到命令：去 (10, 5)
- USV-02 收到命令：去 (10, 5)

问题：两艘 USV 的飞控都认为自己启动位置是 (0,0)
结果：它们会去各自坐标系的 (10,5)，而不是全局地图的同一个点！
      → 任务失败，集群协同失效
```

### 使用 Boot Pose 的正确做法

```
✅ 正确情况（使用 Boot Pose）：

步骤1: 标记 Boot Pose
- USV-01 在 (100, 50) 启动，航向 0°  → 标记为 boot_pose_01
- USV-02 在 (120, 60) 启动，航向 90° → 标记为 boot_pose_02

步骤2: 地面站规划任务
- Area Center 设置为 (105, 55)
- 任务文件：巡航到 (10, 5)

步骤3: 自动坐标转换
全局坐标 = (105+10, 55+5) = (115, 60)

USV-01 本地坐标转换：
  dx = 115 - 100 = 15
  dy = 60 - 50 = 10
  → 飞控收到：去 (15, 10)

USV-02 本地坐标转换：
  dx = 115 - 120 = -5
  dy = 60 - 60 = 0
  考虑航向 90° 旋转：
  → 飞控收到：去 (0, -5)

结果：两艘 USV 都正确地去往全局坐标 (115, 60) 这个点！
      → 任务成功，集群协同正常
```

## 🛠️ 如何使用 Boot Pose

### 方法1：GUI 操作（推荐）

**单个 USV 标记：**
1. 启动所有 USV，确保它们已上线
2. 在地面站 GUI 的集群列表中选中一个 USV
3. 点击菜单：`Boot Pose` → `标记选中USV Boot Pose`
4. 系统会记录该 USV 当前的位置和航向

**批量标记（推荐）：**
1. 确保所有 USV 都已在集群列表中
2. 点击菜单：`Boot Pose` → `批量标记集群USV Boot Pose`
3. 系统会一次性标记所有集群 USV 的 Boot Pose

### 方法2：自动标记（高级）

在 `usv_manager.py` 中，当 USV 第一次上线时可以自动记录：

```python
# 代码片段（已在 usv_manager.py 中实现）
if usv_id not in self.node.usv_boot_pose:
    bp = {'x': position_x, 'y': position_y, 'z': position_z, 'yaw': yaw}
    self.node.usv_boot_pose[usv_id] = bp
    self.node.get_logger().info(f"自动记录 USV {usv_id} 的 boot_pose")
```

## 📂 代码实现位置

### 数据存储
**文件**: `ground_station_node.py`
```python
class GroundStationNode(Node):
    def __init__(self, signal):
        # 存储每艘 USV 的 boot pose
        self.usv_boot_pose = {}  # {'usv_01': {'x':..., 'y':..., 'z':..., 'yaw':...}}
```

### 标记功能
**文件**: `ground_station_node.py`
```python
def set_boot_pose_callback(self, usv_id: str):
    """记录 USV 当前位姿为 boot_pose"""
    st = self.usv_states.get(usv_id)  # 获取当前状态
    bp = {
        'x': st['position']['x'],
        'y': st['position']['y'],
        'z': st['position']['z'],
        'yaw': st['yaw']
    }
    self.usv_boot_pose[usv_id] = bp  # 保存
```

### 坐标转换使用
**文件**: `cluster_controller.py`
```python
def _global_to_usv_local(self, usv_id, p_global):
    """将全局坐标转换为 USV 本地坐标"""
    bp = self.node.usv_boot_pose.get(usv_id)  # 读取 boot_pose
    if bp:
        # 使用 boot_pose 进行转换
        dx = p_global['x'] - bp['x']
        dy = p_global['y'] - bp['y']
        theta = -bp['yaw']
        local_x = cos(theta) * dx - sin(theta) * dy
        local_y = sin(theta) * dx + cos(theta) * dy
        return {'x': local_x, 'y': local_y, 'z': ...}
    else:
        # 回退：使用当前位置
        return p_global
```

## ⚠️ 重要注意事项

### 1. 标记时机
- ✅ **推荐**：USV 完全启动并稳定后立即标记
- ✅ **任务开始前**：确保所有 USV 都已标记 Boot Pose
- ❌ **避免**：任务执行过程中修改 Boot Pose

### 2. 坐标系一致性
- Boot Pose 坐标必须与全局地图坐标系一致
- 如果使用 GPS，确保所有 USV 使用相同的坐标系转换
- 航向角 (yaw) 的定义要一致（通常：北为0°，顺时针为正）

### 3. Boot Pose 的生命周期
```
USV 上电 → 启动节点 → 标记 Boot Pose → 执行任务 → 关机

注意：
- Boot Pose 存储在内存中，关机后清除
- 重启后需要重新标记
- 可以考虑保存到配置文件（未实现）
```

### 4. 多场地使用
```
场景：在不同湖面测试

方案1（推荐）：
- 任务文件不变（使用相对坐标）
- 修改 Area Center 偏移量
- 重新标记 Boot Pose

方案2：
- 每个场地准备不同的配置文件
- 包含该场地的 Area Center 和参考点
```

## 🔍 故障排查

### 问题1：USV 导航到错误位置
**可能原因**：
- Boot Pose 未标记或标记错误
- 坐标系不一致

**解决方法**：
1. 检查日志：`已设置 usv_XX 的 boot_pose: {x:..., y:..., z:..., yaw:...}`
2. 重新标记 Boot Pose
3. 验证全局坐标系定义

### 问题2：Boot Pose 标记失败
**可能原因**：
- USV 状态信息未更新
- USV 不在线

**解决方法**：
```bash
# 检查 USV 是否在线
ros2 topic list | grep usv_01

# 检查状态消息
ros2 topic echo /usv_01/usv_status

# 重新标记
# GUI: Boot Pose → 批量标记集群USV Boot Pose
```

### 问题3：集群 USV 不同步
**可能原因**：
- 部分 USV 的 Boot Pose 未标记
- Boot Pose 坐标系不统一

**解决方法**：
1. 使用批量标记功能确保全部标记
2. 检查所有 USV 的 Boot Pose 是否合理
3. 确认 Area Center 设置正确

## 📊 Boot Pose vs. 其他方案对比

| 方案 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| **Boot Pose** | ✅ 灵活，支持任意位置启动<br>✅ 支持不同航向启动<br>✅ 任务文件可重用 | ⚠️ 需要标记操作<br>⚠️ 重启后需重新标记 | **多机集群**<br>不同位置启动 |
| 固定启动点 | ✅ 无需标记<br>✅ 简单 | ❌ 必须在指定位置启动<br>❌ 不灵活 | 单机测试<br>固定场地 |
| GPS 绝对坐标 | ✅ 全局一致<br>✅ 无需转换 | ❌ 依赖 GPS 精度<br>❌ 室内不可用<br>❌ 任务文件不可重用 | 户外大范围<br>GPS 环境 |

## 🎓 最佳实践

### 标准操作流程 (SOP)

```
步骤1: 系统启动
├─ 1.1 启动地面站 GUI
├─ 1.2 启动所有 USV 机载系统
└─ 1.3 确认所有 USV 在线（查看在线列表）

步骤2: 坐标系设置
├─ 2.1 设置 Area Center 偏移量（如果需要）
│      菜单 → 坐标系设置 → 设置任务坐标系偏移量
└─ 2.2 验证设置正确

步骤3: 标记 Boot Pose ⭐
├─ 3.1 将需要的 USV 添加到集群列表
├─ 3.2 点击菜单 → Boot Pose → 批量标记集群USV Boot Pose
├─ 3.3 检查日志确认标记成功
│      应该看到：已设置 usv_XX 的 boot_pose: {...}
└─ 3.4 验证所有 USV 都已标记

步骤4: 加载任务
├─ 4.1 点击 File → Open，选择任务 XML 文件
└─ 4.2 确认任务加载成功

步骤5: 执行任务
├─ 5.1 集群解锁、设置 Guided 模式
├─ 5.2 点击"cluster start"开始任务
└─ 5.3 监控执行状态

步骤6: 任务完成
├─ 6.1 确认所有步骤完成
├─ 6.2 集群加锁、设置 Manual 模式
└─ 6.3 保存日志（如需要）
```

### 调试建议

```bash
# 1. 查看某个 USV 的状态
ros2 topic echo /usv_01/usv_status

# 2. 查看 TF 树（验证 Boot Pose transform）
ros2 run tf2_tools view_frames
# 会生成 frames.pdf，查看是否有 usv_01_boot 等 frame

# 3. 手动查询 Boot Pose transform
ros2 run tf2_ros tf2_echo map usv_01_boot

# 4. 查看地面站日志
# GUI 日志区域会显示 Boot Pose 相关信息
```

## 🔗 相关文件

- **Boot Pose 标记**：`gs_gui/ground_station_node.py` - `set_boot_pose_callback()`
- **坐标转换**：`gs_gui/cluster_controller.py` - `_global_to_usv_local()`
- **GUI 菜单**：`gs_gui/main_gui_app.py` - `set_boot_pose_command()`
- **ROS 信号**：`gs_gui/ros_signal.py` - `set_boot_pose`, `set_boot_pose_all`
- **坐标系文档**：`gs_gui/AREA_OFFSET_GUIDE.md`

## 📝 总结

Boot Pose 机制是 USV 集群系统坐标转换的核心机制，它解决了以下关键问题：

1. ✅ **多机协同**：不同位置启动的 USV 能正确执行全局任务
2. ✅ **任务重用**：同一任务文件可在不同场地使用
3. ✅ **坐标统一**：统一全局坐标系和 USV 本地坐标系
4. ✅ **灵活部署**：支持任意位置、任意航向启动

**记住**：在执行任务前，**务必标记所有 USV 的 Boot Pose**！这是系统正常工作的前提。

---

**更新时间**：2025-10-23  
**文档版本**：v1.0
