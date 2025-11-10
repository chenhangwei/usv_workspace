# 如何判断 EKF Origin 是否生效

## ✅ 验证方法总结

### 快速验证（推荐）

运行验证脚本：
```bash
~/usv_workspace/scripts/verify_ekf_origin.sh
```

### 手动验证（详细）

## 方法 1: 查看启动日志 ⭐⭐⭐⭐⭐

**最简单直接的方法**

在 USV 启动日志中查找以下消息：

```bash
[INFO] [usv_01.auto_set_home_node]: ✅ EKF Origin set successfully via SET_GPS_GLOBAL_ORIGIN: (22.5180977, 113.9007239, -4.80m)
```

**判断标准**：
- ✅ 看到此消息 → EKF Origin 设置成功
- ❌ 未看到此消息 → EKF Origin 未设置或设置失败

**何时出现**：启动后约 4-5 秒

## 方法 2: 检查 Home Position Topic ⭐⭐⭐⭐

**间接验证 EKF Origin**

```bash
ros2 topic echo /usv_01/home_position/home --once
```

**预期输出**：
```yaml
geo:
  latitude: 22.5180976    # 应该接近我们设置的 22.5180977
  longitude: 113.9007232  # 应该接近我们设置的 113.9007239
  altitude: -8.33         # 可能与设置值不同（飞控自动修正）
position:
  x: -3.71  # 局部坐标（相对于某个点）
  y: -0.95
  z: -2.47
```

**判断标准**：
- ✅ latitude/longitude 接近设置值（±0.0001° 以内）→ 原点已设置
- ❌ 无数据或坐标完全不对 → 原点未设置

**注意**: 
- `altitude` 可能与设置值不同（飞控会根据气压计修正）
- `position.x/y/z` 是相对坐标，不是绝对 GPS

## 方法 3: 检查 Local Position 有效性 ⭐⭐⭐⭐

**验证坐标系是否可用**

```bash
ros2 topic echo /usv_01/local_position/pose --once
```

**预期输出**：
```yaml
header:
  frame_id: map  # ✅ 必须是 'map'
pose:
  position:
    x: 0.088      # 有数值（相对于 EKF Origin 的偏移）
    y: -0.211
    z: -3.667
  orientation:
    x: 0.0069
    y: 0.0039
    z: -0.7663
    w: -0.6424
```

**判断标准**：
- ✅ `frame_id: map` + 有位置数据 → 坐标系有效
- ❌ `frame_id: ''` 或无数据 → 坐标系未初始化

## 方法 4: 检查控制节点日志 ⭐⭐⭐

**查看 usv_control_node 的状态消息**

在启动日志中查找：

```bash
[INFO] [usv_01.usv_control_node]: ✅ Local Position 有效: (0.08, -0.23, -2.16)
[INFO] [usv_01.usv_control_node]: ✅ Home Position 已设置: (22.5180976, 113.9007232, -8.33m)
[INFO] [usv_01.usv_control_node]: 🎯 EKF Origin 完全就绪，可以安全发布目标点！
```

**判断标准**：
- ✅ 看到三条消息 → EKF Origin 完全就绪
- ⚠️ 只看到前两条 → 等待 Home Position 设置
- ❌ 未看到消息 → 控制节点未就绪

## 方法 5: 功能测试 ⭐⭐⭐⭐⭐

**最可靠的验证方法**

### 测试 1: 发送导航目标

```bash
# 发送一个相对于 EKF Origin 的目标点（向东 5 米）
ros2 topic pub --once /usv_01/setpoint_position/local geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}}}"
```

**预期行为**：
- ✅ USV 向东移动约 5 米 → 坐标系正确
- ❌ USV 不动或移动方向错误 → 坐标系有问题

### 测试 2: RTL 返航测试

1. 通过 GUI 设置 Home Position
2. 将 USV 移动到其他位置
3. 切换到 RTL 模式
4. 观察是否返航

**切换 RTL 命令**：
```bash
ros2 topic pub --once /usv_01/set_usv_mode std_msgs/msg/String "data: 'RTL'"
```

## 方法 6: 对比 GPS 和 Local 坐标 ⭐⭐

**理论验证（不推荐，误差较大）**

```bash
# 查看全局 GPS 坐标
ros2 topic echo /usv_01/global_position/global --once

# 查看局部坐标
ros2 topic echo /usv_01/local_position/pose --once
```

**计算方法**：
```python
# GPS 坐标差值
Δlat = current_lat - origin_lat  # 应该对应 local_y
Δlon = current_lon - origin_lon  # 应该对应 local_x

# 转换为米（粗略）
local_y_expected ≈ Δlat × 111320
local_x_expected ≈ Δlon × 111320 × cos(latitude)
```

**注意**：
- GPS 精度 ±1-5 米，误差较大
- EKF 融合多传感器数据，不完全等于 GPS 转换
- **不建议用此方法验证**，仅供参考

## 验证脚本

我已经创建了自动验证脚本：`scripts/verify_ekf_origin.sh`

**使用方法**：
```bash
~/usv_workspace/scripts/verify_ekf_origin.sh
```

**脚本功能**：
1. 检查 Home Position（验证原点坐标）
2. 检查 Local Position（验证坐标系有效性）
3. 检查 Global Position（对比 GPS）
4. 显示所有关键数据

## 常见问题

### Q1: Home Position 坐标与设置值略有不同？

**A**: 正常现象
- GPS 精度 ±1-5 米
- 飞控会根据气压计修正高度
- 只要差异在 0.001° 以内就是正常的

### Q2: Local Position 数值很小（接近 0）？

**A**: 正常现象
- 表示 USV 当前位置接近 EKF Origin
- 可能启动时就在 A0 基站附近
- 移动 USV 后数值会变大

### Q3: 看不到 `gp_origin` topic 数据？

**A**: 正常现象
- `SET_GPS_GLOBAL_ORIGIN` 是一次性命令
- 不是持续发布的 topic
- 通过 `home_position` 间接验证即可

### Q4: 高度值对不上？

**A**: 正常现象
- GPS 高度相对于 WGS84 椭球（海平面）
- Local Z 可能相对于地面或其他基准
- 气压计会影响高度读数

## 总结

### 推荐验证流程

1. **启动时**：查看日志是否有 "✅ EKF Origin set successfully"
2. **运行时**：运行验证脚本 `verify_ekf_origin.sh`
3. **功能测试**：发送导航目标或 RTL 测试

### 判断标准

| 检查项 | 期望结果 | 重要性 |
|--------|---------|--------|
| 启动日志 | "✅ EKF Origin set successfully" | ⭐⭐⭐⭐⭐ |
| Home Position | latitude/longitude 接近设置值 | ⭐⭐⭐⭐ |
| Local Position | frame_id='map' + 有数据 | ⭐⭐⭐⭐ |
| 控制节点日志 | "🎯 EKF Origin 完全就绪" | ⭐⭐⭐ |
| 导航功能测试 | USV 能正确导航 | ⭐⭐⭐⭐⭐ |

---

**结论**：从您之前的日志和验证脚本输出来看，**EKF Origin 已经成功生效**！✅
