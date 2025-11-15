# 🎮 虚拟USV测试系统

## ✨ 快速开始

### 一键测试（完全虚拟环境）

```bash
# 1. 编译新增节点
cd ~/usv_workspace
colcon build --packages-select usv_comm
source install/setup.bash

# 2. 启动USV节点（新终端1）
ros2 launch usv_bringup usv_launch.py namespace:=usv_01

# 3. 运行完整测试（新终端2）
cd ~/usv_workspace/src
./test_mock_usv.sh usv_01 60.0 35.0 0.0
```

**仅需30秒，无需任何硬件！** 🚀

---

## 📦 新增内容

### 1. 虚拟数据节点
**文件**: `usv_comm/usv_comm/mock_usv_data.py`

**功能**:
- ✅ 模拟GPS位置
- ✅ 模拟本地位置
- ✅ 模拟MAVROS状态
- ✅ 接收目标点并自动移动

**使用**:
```bash
ros2 run usv_comm mock_usv_data \
    --ros-args \
    -p namespace:=usv_01 \
    -p move_speed:=2.0
```

---

### 2. 完整测试脚本
**文件**: `test_mock_usv.sh`

**功能**:
- ✅ 自动启动虚拟数据
- ✅ 检查节点状态
- ✅ 发送测试目标点
- ✅ 实时监控位置变化
- ✅ 显示调试日志

**使用**:
```bash
./test_mock_usv.sh [usv_namespace] [x] [y] [z]

# 示例
./test_mock_usv.sh usv_01 60.0 35.0 0.0
```

---

### 3. 详细文档
- **MOCK_USV_GUIDE.md** - 虚拟测试完整指南
- **DEBUG_LOG_SUMMARY.md** - 调试日志说明
- **DEBUG_NAVIGATION_GUIDE.md** - 调试指南

---

## 🎯 测试场景

### 场景1: 基础导航测试
```bash
./test_mock_usv.sh usv_01 10.0 10.0 0.0
```
验证：短距离移动 ~7秒

### 场景2: 长距离导航
```bash
./test_mock_usv.sh usv_01 100.0 80.0 0.0
```
验证：长距离移动 ~64秒

### 场景3: GPS坐标验证
```bash
# 发送目标点
./test_mock_usv.sh usv_01 60.0 35.0 0.0

# 验证GPS输出
ros2 topic echo /usv_01/setpoint_raw/global
```
验证：XYZ → GPS 转换正确性

---

## 📊 完整数据流

```
🎮 虚拟数据节点 (mock_usv_data)
   ├─ 发布 GPS 位置 → global_position/global
   ├─ 发布本地位置 → local_position/pose
   ├─ 发布速度 → velocity_local
   ├─ 发布状态 → state
   └─ 订阅目标点 → set_usv_target_position
           ↓
📨 Action Server (navigate_to_point_server)
   ├─ 接收 Action Goal
   └─ 转发到 set_usv_target_position
           ↓
🔄 坐标转换 (coord_transform_node)
   ├─ 接收 XYZ 坐标
   └─ 发布 GPS 坐标 → setpoint_raw/global
           ↓
🎯 虚拟USV自动移动到目标点
```

---

## 🔍 实时监控

### 查看位置变化
```bash
watch -n 0.5 "ros2 topic echo /usv_01/local_position/pose --once | grep -E 'x:|y:' | head -2"
```

### 查看调试日志
```bash
ros2 topic echo /usv_01/rosout | grep -E "📥|📤|📨|🎯" --color=always -A 5
```

### 查看GPS输出
```bash
ros2 topic echo /usv_01/setpoint_raw/global
```

---

## ⚙️ 配置参数

### 虚拟USV配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| namespace | usv_01 | USV命名空间 |
| initial_x | 0.0 | 初始X坐标(m) |
| initial_y | 0.0 | 初始Y坐标(m) |
| move_speed | 2.0 | 移动速度(m/s) |
| publish_rate | 10.0 | 数据发布频率(Hz) |
| gps_origin_lat | 22.5180977 | GPS原点纬度 |
| gps_origin_lon | 113.9007239 | GPS原点经度 |

---

## 💡 使用技巧

### 1. 快速验证坐标转换

```bash
# 发送已知XYZ坐标
ros2 topic pub --once /usv_01/set_usv_target_position \
  geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 60.0, y: 35.0, z: 0.0}}}'

# 查看GPS输出并手动计算验证
ros2 topic echo /usv_01/setpoint_raw/global
```

### 2. 测试不同速度

```bash
# 快速移动
ros2 run usv_comm mock_usv_data --ros-args -p move_speed:=5.0

# 慢速移动
ros2 run usv_comm mock_usv_data --ros-args -p move_speed:=0.5
```

### 3. 多USV同时测试

```bash
# 终端1: USV 01
ros2 run usv_comm mock_usv_data --ros-args -p namespace:=usv_01 &

# 终端2: USV 02
ros2 run usv_comm mock_usv_data --ros-args -p namespace:=usv_02 -p initial_x:=10.0 &

# 终端3: USV 03
ros2 run usv_comm mock_usv_data --ros-args -p namespace:=usv_03 -p initial_x:=20.0 &
```

---

## 🛠️ 故障排除

### 虚拟USV不移动？
```bash
# 检查日志
ros2 topic echo /usv_01/rosout | grep "虚拟USV"

# 查看当前位置
ros2 topic echo /usv_01/local_position/pose
```

### 没有GPS输出？
```bash
# 检查坐标转换节点
ros2 node list | grep coord_transform

# 检查配置
ros2 param get /usv_01/coord_transform_node enable_coord_transform
```

### 停止所有测试
```bash
# 停止虚拟数据节点
pkill -f mock_usv_data

# 停止所有USV节点
pkill -f usv_launch
```

---

## 🎓 学习价值

### 理解数据流
通过虚拟数据，你可以清楚看到：
1. 目标点如何从地面站到USV
2. XYZ坐标如何转换为GPS
3. GPS坐标如何发送到飞控
4. USV如何响应导航命令

### 验证算法
- ✅ XYZ → GPS 转换算法
- ✅ GPS → XYZ 反向转换
- ✅ 坐标系一致性
- ✅ 移动控制逻辑

### 快速调试
- ✅ 无需实际硬件
- ✅ 完全可重复测试
- ✅ 快速迭代开发
- ✅ 安全无风险

---

## 📚 相关文档

1. **MOCK_USV_GUIDE.md** - 详细使用指南
2. **DEBUG_LOG_SUMMARY.md** - 调试日志增强说明
3. **DEBUG_NAVIGATION_GUIDE.md** - 完整调试指南
4. **NAVIGATION_FLOW_COMPLETE.md** - 导航流程详解
5. **COORDINATE_SYSTEM_DESIGN.md** - 坐标系统设计

---

## ✅ 验证清单

测试虚拟系统时，确认以下内容：

- [ ] 虚拟数据节点正常运行
- [ ] GPS位置有数据输出
- [ ] 本地位置有数据输出
- [ ] MAVROS状态显示已连接、解锁、GUIDED模式
- [ ] 发送目标点后虚拟USV开始移动
- [ ] 坐标转换节点收到XYZ坐标
- [ ] GPS目标点正确发布到 setpoint_raw/global
- [ ] 调试日志清晰显示完整流程
- [ ] 虚拟USV最终到达目标点附近(±0.1m)

---

## 🎉 总结

**虚拟测试系统让你能够**：
- 🚀 无需硬件即可完整测试导航功能
- 🔍 清晰观察每个环节的数据流转
- ✅ 快速验证代码修改
- 🎯 安全地测试各种极端场景

**立即开始**: `./test_mock_usv.sh` 🎮

---

**版本**: v1.0  
**日期**: 2025-11-14  
**状态**: ✅ 就绪使用
