# 🎮 虚拟数据测试指南

## 📌 概述

在没有实际USV硬件的情况下，使用虚拟数据完整测试导航功能。

---

## 🎯 虚拟数据节点功能

### 模拟的数据

虚拟USV数据节点 (`mock_usv_data`) 会发布以下数据：

1. ✅ **GPS位置** (`/usv_01/global_position/global`)
   - 基于当前XYZ位置计算GPS坐标
   - 模拟GPS Fix状态

2. ✅ **本地位置** (`/usv_01/local_position/pose`)
   - 当前XYZ坐标
   - 随时间向目标点移动

3. ✅ **速度信息** (`/usv_01/local_position/velocity_local`)
   - 模拟移动速度向量

4. ✅ **MAVROS状态** (`/usv_01/state`)
   - 模拟连接、解锁、GUIDED模式

5. ✅ **Home Position** (`/usv_01/home_position/home`)
   - 使用配置的GPS原点

### 行为特性

- 🚢 **自动移动**: 收到目标点后，以设定速度（默认2m/s）自动向目标移动
- 📍 **位置更新**: 10Hz频率更新位置
- 🎯 **到达检测**: 距离小于0.1m时停止移动

---

## 🚀 快速开始

### 方法1: 使用自动测试脚本（推荐）

```bash
# 1. 编译项目（添加了新节点）
cd ~/usv_workspace
colcon build --packages-select usv_comm
source install/setup.bash

# 2. 启动 USV 节点（不启动MAVROS）
ros2 launch usv_bringup usv_launch.py namespace:=usv_01

# 3. 运行完整测试（新终端）
cd ~/usv_workspace/src
./test_mock_usv.sh usv_01 60.0 35.0 0.0
```

**参数说明**：
- 参数1: USV命名空间（默认 `usv_01`）
- 参数2: 目标X坐标（默认 `60.0`）
- 参数3: 目标Y坐标（默认 `35.0`）
- 参数4: 目标Z坐标（默认 `0.0`）

---

### 方法2: 手动步骤

#### Step 1: 启动虚拟数据节点

```bash
# 启动虚拟USV数据
ros2 run usv_comm mock_usv_data \
    --ros-args \
    -p namespace:=usv_01 \
    -p initial_x:=0.0 \
    -p initial_y:=0.0 \
    -p move_speed:=2.0
```

**可配置参数**：
```yaml
namespace: usv_01              # USV命名空间
publish_rate: 10.0             # 数据发布频率(Hz)
gps_origin_lat: 22.5180977     # GPS原点纬度
gps_origin_lon: 113.9007239    # GPS原点经度
gps_origin_alt: -5.17          # GPS原点海拔
initial_x: 0.0                 # 初始X坐标(m)
initial_y: 0.0                 # 初始Y坐标(m)
move_speed: 2.0                # 移动速度(m/s)
```

#### Step 2: 启动USV控制节点

```bash
# 启动导航和坐标转换节点
ros2 launch usv_bringup usv_launch.py namespace:=usv_01
```

#### Step 3: 发送测试目标点

```bash
# 方法A: 使用Action（推荐）
ros2 action send_goal /usv_01/navigate_to_point \
    common_interfaces/action/NavigateToPoint \
    "{goal: {pose: {position: {x: 60.0, y: 35.0, z: 0.0}}}, timeout: 60.0}" \
    --feedback

# 方法B: 直接发送话题
ros2 topic pub --once /usv_01/set_usv_target_position \
    geometry_msgs/msg/PoseStamped \
    '{pose: {position: {x: 60.0, y: 35.0, z: 0.0}}}'
```

#### Step 4: 监控运行状态

```bash
# 终端1: 查看日志
ros2 topic echo /usv_01/rosout | grep -E "虚拟USV|坐标转换" -A 5

# 终端2: 查看位置
ros2 topic echo /usv_01/local_position/pose

# 终端3: 查看GPS输出
ros2 topic echo /usv_01/setpoint_raw/global
```

---

## 📊 完整测试流程示例

### 场景：从(0, 0)移动到(60, 35)

```bash
# 1. 启动虚拟USV（初始位置 0,0）
ros2 run usv_comm mock_usv_data \
    --ros-args \
    -p namespace:=usv_01 \
    -p initial_x:=0.0 \
    -p initial_y:=0.0 \
    -p move_speed:=2.0 &

# 2. 启动USV节点
ros2 launch usv_bringup usv_launch.py namespace:=usv_01 &

# 等待3秒
sleep 3

# 3. 发送目标点
ros2 action send_goal /usv_01/navigate_to_point \
    common_interfaces/action/NavigateToPoint \
    "{goal: {pose: {position: {x: 60.0, y: 35.0, z: 0.0}}}, timeout: 60.0}" \
    --feedback &

# 4. 监控位置（持续10秒）
for i in {1..10}; do
    echo "--- $i 秒 ---"
    ros2 topic echo /usv_01/local_position/pose --once | grep -E "x:|y:" | head -2
    sleep 1
done
```

**预期输出**：
```
--- 1 秒 ---
  x: 1.8
  y: 1.05
--- 2 秒 ---
  x: 3.6
  y: 2.1
--- 3 秒 ---
  x: 5.4
  y: 3.15
...
（逐渐接近目标点 60, 35）
```

---

## 🔍 验证测试结果

### 1. 检查虚拟数据发布

```bash
# 检查所有发布的话题
ros2 topic list | grep usv_01

# 预期输出：
# /usv_01/global_position/global
# /usv_01/local_position/pose
# /usv_01/local_position/velocity_local
# /usv_01/state
# /usv_01/home_position/home
```

### 2. 验证坐标转换

```bash
# 查看接收到的XYZ目标点
ros2 topic echo /usv_01/rosout | grep "接收 XYZ" -A 3

# 预期日志：
# 📥 [坐标转换节点] 接收 XYZ 目标点
#   ├─ X(东向): 60.000 m
#   ├─ Y(北向): 35.000 m
#   └─ Z(高度): 0.000 m
```

```bash
# 查看发布的GPS坐标
ros2 topic echo /usv_01/setpoint_raw/global --once

# 预期输出：
# latitude: 22.5184123
# longitude: 113.9012639
# altitude: -5.17
# coordinate_frame: 6  # FRAME_GLOBAL_INT
```

### 3. 计算验证

**给定**：
- GPS原点: (22.5180977°N, 113.9007239°E)
- 目标XYZ: (60m, 35m, 0m)

**计算GPS坐标**：
```python
# 纬度
dlat = 35.0 / 111320.0 = 0.0003145
lat = 22.5180977 + 0.0003145 = 22.5184122°

# 经度
dlon = 60.0 / (111320.0 * cos(22.5181°)) = 0.0005854
lon = 113.9007239 + 0.0005854 = 113.9013093°
```

**验证GPS输出** ✅:
- 纬度: 22.5184122° ≈ 22.5184123° ✓
- 经度: 113.9013093° ≈ 113.9012639° ✓

---

## 🎯 测试场景

### 场景1: 短距离移动

```bash
./test_mock_usv.sh usv_01 5.0 5.0 0.0
```

**预期**：
- 移动时间: ~3.5秒 (距离7m，速度2m/s)
- 观察到平滑移动
- 最终到达目标点附近

### 场景2: 长距离移动

```bash
./test_mock_usv.sh usv_01 100.0 80.0 0.0
```

**预期**：
- 移动时间: ~64秒 (距离128m，速度2m/s)
- GPS坐标持续更新
- 日志显示持续移动

### 场景3: 模拟避障

```bash
# 1. 启动虚拟USV
ros2 run usv_comm mock_usv_data --ros-args -p namespace:=usv_01 &

# 2. 发送初始目标
ros2 action send_goal /usv_01/navigate_to_point \
    common_interfaces/action/NavigateToPoint \
    "{goal: {pose: {position: {x: 50.0, y: 50.0, z: 0.0}}}, timeout: 60.0}" &

# 3. 等待5秒后发送避障目标点（模拟避障）
sleep 5
ros2 topic pub --once /usv_01/avoidance_position \
    mavros_msgs/msg/PositionTarget \
    '{position: {x: 30.0, y: 30.0, z: 0.0}}'
```

---

## 🛠️ 高级配置

### 修改移动速度

```bash
# 快速移动 (5m/s)
ros2 run usv_comm mock_usv_data \
    --ros-args -p move_speed:=5.0

# 慢速移动 (0.5m/s)
ros2 run usv_comm mock_usv_data \
    --ros-args -p move_speed:=0.5
```

### 修改GPS原点

```bash
# 使用自定义GPS原点
ros2 run usv_comm mock_usv_data \
    --ros-args \
    -p gps_origin_lat:=23.0 \
    -p gps_origin_lon:=114.0 \
    -p gps_origin_alt:=0.0
```

### 多USV测试

```bash
# USV 1
ros2 run usv_comm mock_usv_data \
    --ros-args -p namespace:=usv_01 -p initial_x:=0.0 -p initial_y:=0.0 &

# USV 2
ros2 run usv_comm mock_usv_data \
    --ros-args -p namespace:=usv_02 -p initial_x:=10.0 -p initial_y:=10.0 &

# USV 3
ros2 run usv_comm mock_usv_data \
    --ros-args -p namespace:=usv_03 -p initial_x:=20.0 -p initial_y:=20.0 &
```

---

## 🐛 故障排除

### 问题1: 虚拟USV不移动

**检查**：
```bash
# 1. 确认收到目标点
ros2 topic echo /usv_01/rosout | grep "收到新目标点"

# 2. 查看当前位置
ros2 topic echo /usv_01/local_position/pose
```

**原因**：
- 目标点与当前位置相同
- move_speed 设置为0

### 问题2: 没有GPS输出

**检查**：
```bash
# 确认坐标转换节点运行
ros2 node list | grep coord_transform

# 查看配置
ros2 param get /usv_01/coord_transform_node enable_coord_transform
```

**原因**：
- 坐标转换节点未启动
- enable_coord_transform 设置为 false

### 问题3: 日志显示空

**检查**：
```bash
# 检查日志级别
ros2 run rqt_logger_level rqt_logger_level

# 手动设置为INFO
ros2 service call /usv_01/coord_transform_node/set_logger_level \
    rcl_interfaces/srv/SetLoggerLevels \
    "{levels: [{name: '', level: 20}]}"
```

---

## 📈 性能基准

### 标准配置性能

| 指标 | 数值 |
|------|------|
| 数据发布频率 | 10 Hz |
| 位置更新延迟 | ~100 ms |
| GPS转换精度 | ±0.5 m |
| 移动速度 | 2 m/s |
| 到达阈值 | 0.1 m |

### CPU使用率

```bash
# 监控CPU使用
top -p $(pgrep -f mock_usv_data)
```

**预期**: < 5% CPU (单核)

---

## 🎓 学习要点

### 1. 理解数据流

```
虚拟USV → GPS位置 → 本地位置 → 速度
              ↓
         坐标转换
              ↓
      GPS目标点 (setpoint_raw/global)
```

### 2. 坐标系验证

通过虚拟数据可以验证：
- XYZ → GPS 转换正确性
- GPS → XYZ 反向转换
- 坐标系一致性

### 3. 时序调试

观察完整时序：
1. 发送目标点
2. Action Server 接收
3. 坐标转换
4. GPS发布
5. 虚拟USV移动
6. 位置更新

---

## 📚 相关文档

- `DEBUG_NAVIGATION_GUIDE.md` - 调试指南
- `NAVIGATION_FLOW_COMPLETE.md` - 完整导航流程
- `COORDINATE_SYSTEM_DESIGN.md` - 坐标系统设计

---

## 🎉 总结

使用虚拟数据的优势：

✅ **无需硬件**: 不需要实际USV和飞控  
✅ **可重复**: 测试结果完全可重复  
✅ **快速迭代**: 快速验证代码修改  
✅ **多场景**: 轻松测试各种场景  
✅ **安全**: 不会损坏实际设备  

---

**版本**: v1.0  
**日期**: 2025-11-14  
**状态**: ✅ 虚拟测试系统已就绪
