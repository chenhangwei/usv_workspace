# 🔍 导航目标点调试指南

## 📌 概述

本指南帮助你查看完整的导航目标点数据流转过程，从地面站发送到飞控接收的每一步都有详细日志。

---

## 🎯 调试日志输出位置

### 1️⃣ **地面站端（Ground Station）**

#### 查看集群控制器坐标转换
```bash
# 终端1：地面站日志
ros2 launch gs_bringup gs_launch.py

# 终端2：过滤集群控制器日志
ros2 topic echo /rosout | grep -A 5 "集群控制器"
```

**日志格式示例**：
```
📤 [集群控制器] Step 1 → usv_01
  ├─ Area坐标: X=10.00, Y=5.00, Z=0.00
  ├─ Global坐标: X=60.00, Y=35.00, Z=0.00
  └─ Local坐标: X=60.00, Y=35.00, Z=0.00
```

#### 查看地面站发送日志
```bash
# 过滤地面站发送日志
ros2 topic echo /rosout | grep -A 6 "地面站发送"
```

**日志格式示例**：
```
📤 [地面站发送] usv_01
  ├─ Area坐标(XML): X=10.00, Y=5.00, Z=0.00
  ├─ Global坐标: X=60.00, Y=35.00, Z=0.00
  ├─ Local坐标: X=60.00, Y=35.00, Z=0.00
  └─ Yaw: 0.00 rad
```

---

### 2️⃣ **USV 端（机载计算机）**

#### 查看 Action Server 接收日志
```bash
# 终端1：启动 USV
ros2 launch usv_bringup usv_launch.py namespace:=usv_01

# 终端2：查看 Action Server 日志
ros2 topic echo /usv_01/rosout | grep -A 5 "Action Server"
```

**日志格式示例**：
```
📥 [Action Server 接收] 导航目标点
  ├─ X坐标: 60.000 m
  ├─ Y坐标: 35.000 m
  ├─ Z坐标: 0.000 m
  └─ 超时: 300.0 s

📨 [Action Server 转发] → set_usv_target_position
  ├─ X: 60.000 m
  ├─ Y: 35.000 m
  └─ Z: 0.000 m
```

#### 查看坐标转换节点日志
```bash
# 查看 XYZ → GPS 转换
ros2 topic echo /usv_01/rosout | grep -A 10 "坐标转换节点"
```

**日志格式示例**：
```
📥 [坐标转换节点] 接收 XYZ 目标点
  ├─ X(东向): 60.000 m
  ├─ Y(北向): 35.000 m
  └─ Z(高度): 0.000 m

📤 [坐标转换节点] 发布 GlobalPositionTarget
  ├─ 纬度(Lat): 22.5184123°
  ├─ 经度(Lon): 113.9012639°
  ├─ 海拔(Alt): -5.17 m
  ├─ 话题: setpoint_raw/global
  └─ MAVLink: SET_POSITION_TARGET_GLOBAL_INT (ID:86)
```

#### 查看避障日志（如果触发避障）
```bash
# 查看避障目标点
ros2 topic echo /usv_01/rosout | grep -A 5 "避障"
```

**日志格式示例**：
```
🚨 [坐标转换节点] 接收避障 XYZ 目标点
  ├─ X(东向): 58.500 m
  ├─ Y(北向): 36.200 m
  └─ Z(高度): 0.000 m
```

---

### 3️⃣ **MAVROS 和飞控**

#### 监听 MAVROS 发布的全局目标点
```bash
# 查看发布到飞控的 GPS 目标点
ros2 topic echo /usv_01/setpoint_raw/global
```

**消息格式示例**：
```yaml
header:
  stamp:
    sec: 1731578400
    nanosec: 123456789
  frame_id: map
coordinate_frame: 6  # FRAME_GLOBAL_INT
type_mask: 4088      # 0x0FF8 (只使用位置)
latitude: 22.5184123
longitude: 113.9012639
altitude: -5.17
velocity: {x: 0.0, y: 0.0, z: 0.0}
acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
yaw_rate: 0.0
```

#### 监听 MAVLink 消息（网络抓包）
```bash
# 终端1：监听 UDP 端口
nc -ul 192.168.10.1 -p 14550 | hexdump -C | grep "86"

# 或使用 tcpdump
sudo tcpdump -i any -n udp port 14550 -X
```

**MAVLink 消息格式**：
```
Message ID: 86 (SET_POSITION_TARGET_GLOBAL_INT)
- lat_int: 225184123 (纬度 * 1e7)
- lon_int: 1139012639 (经度 * 1e7)
- alt: -5.17 (海拔)
- coordinate_frame: 6 (FRAME_GLOBAL_INT)
```

---

## 🧪 完整调试流程

### Step 1: 启动系统

```bash
# 1. 启动地面站（在地面站电脑）
ros2 launch gs_bringup gs_launch.py

# 2. 启动 USV（在 USV 机载电脑，或远程启动）
ros2 launch usv_bringup usv_launch.py namespace:=usv_01
```

### Step 2: 开启日志监控

```bash
# 地面站端（新终端）
ros2 topic echo /rosout | grep -E "地面站发送|集群控制器" -A 6

# USV 端（新终端）
ros2 topic echo /usv_01/rosout | grep -E "Action Server|坐标转换节点" -A 10
```

### Step 3: 发送测试目标点

```bash
# 方法1：通过地面站 GUI 发送
# - 加载 XML 任务文件
# - 点击 "发送集群目标点"

# 方法2：手动发送测试目标点
ros2 topic pub --once /usv_01/set_usv_target_position \
  geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, 
    pose: {position: {x: 60.0, y: 35.0, z: 0.0}}}'
```

### Step 4: 观察日志输出

**地面站应该输出**：
```
📤 [地面站发送] usv_01
  ├─ Area坐标(XML): X=10.00, Y=5.00, Z=0.00
  ├─ Global坐标: X=60.00, Y=35.00, Z=0.00
  ├─ Local坐标: X=60.00, Y=35.00, Z=0.00
  └─ Yaw: 0.00 rad
```

**USV 应该依次输出**：
```
1. Action Server 接收：
   📥 [Action Server 接收] 导航目标点
      ├─ X坐标: 60.000 m
      ├─ Y坐标: 35.000 m
      └─ Z坐标: 0.000 m

2. Action Server 转发：
   📨 [Action Server 转发] → set_usv_target_position
      ├─ X: 60.000 m
      └─ Y: 35.000 m

3. 坐标转换接收：
   📥 [坐标转换节点] 接收 XYZ 目标点
      ├─ X(东向): 60.000 m
      └─ Y(北向): 35.000 m

4. 坐标转换发布：
   📤 [坐标转换节点] 发布 GlobalPositionTarget
      ├─ 纬度(Lat): 22.5184123°
      ├─ 经度(Lon): 113.9012639°
      └─ MAVLink: SET_POSITION_TARGET_GLOBAL_INT (ID:86)
```

---

## 📊 日志过滤技巧

### 只看坐标数据
```bash
# 只看坐标值（去掉emoji和格式）
ros2 topic echo /usv_01/rosout | grep -E "X=|Y=|Z=|纬度|经度"
```

### 只看关键节点
```bash
# 地面站发送
ros2 topic echo /rosout | grep "地面站发送" -A 6

# USV 接收
ros2 topic echo /usv_01/rosout | grep "Action Server 接收" -A 5

# 坐标转换
ros2 topic echo /usv_01/rosout | grep "坐标转换节点" -A 5
```

### 保存日志到文件
```bash
# 保存完整日志
ros2 topic echo /usv_01/rosout > usv_debug_$(date +%Y%m%d_%H%M%S).log

# 只保存导航相关日志
ros2 topic echo /usv_01/rosout | grep -E "Action Server|坐标转换" -A 10 > nav_debug.log
```

---

## 🔍 常见问题调试

### 问题1：地面站发送了，USV没收到

**检查步骤**：
```bash
# 1. 检查 Action Server 是否运行
ros2 node list | grep navigate_to_point

# 2. 检查 Action 客户端连接
ros2 action list | grep navigate_to_point

# 3. 查看网络连接
ping 192.168.68.54  # USV IP
```

### 问题2：USV收到了，但没转发到坐标转换节点

**检查步骤**：
```bash
# 1. 检查话题是否有数据
ros2 topic echo /usv_01/set_usv_target_position

# 2. 检查坐标转换节点是否运行
ros2 node list | grep coord_transform

# 3. 查看节点状态
ros2 node info /usv_01/coord_transform_node
```

### 问题3：坐标转换后，飞控没反应

**检查步骤**：
```bash
# 1. 检查 MAVROS 连接
ros2 topic echo /usv_01/state

# 2. 查看全局目标点是否发布
ros2 topic echo /usv_01/setpoint_raw/global

# 3. 检查飞控模式
# 需要在 GUIDED 模式下才能接收导航命令

# 4. 查看飞控参数
# SYSID_THISMAV 是否正确
# GPS 是否 Fix
```

---

## 📈 性能监控

### 查看消息频率
```bash
# 查看各话题发布频率
ros2 topic hz /usv_01/set_usv_target_position
ros2 topic hz /usv_01/setpoint_raw/global
```

### 查看消息延迟
```bash
# 查看消息时间戳
ros2 topic echo /usv_01/setpoint_raw/global --field header.stamp
```

---

## 🎯 调试技巧总结

### 1. **分层调试法**
- 从地面站开始，逐层验证
- 每一层都确认数据正确后再检查下一层

### 2. **对比坐标值**
- Area坐标 (XML)
- Global坐标 (Area + Offset)
- Local坐标 (应该与Global相同)
- GPS坐标 (纬度/经度)

### 3. **使用多终端**
```
终端1: 地面站日志
终端2: USV日志
终端3: 话题监听
终端4: MAVLink抓包
```

### 4. **记录时间戳**
- 记录每条日志的时间
- 计算传输延迟

---

## 🛠️ 快速调试脚本

创建 `debug_nav.sh`：
```bash
#!/bin/bash

echo "🔍 导航目标点调试工具"
echo "===================="
echo ""

# 检查节点状态
echo "1️⃣ 检查关键节点..."
ros2 node list | grep -E "navigate_to_point|coord_transform|mavros"
echo ""

# 检查话题
echo "2️⃣ 检查关键话题..."
ros2 topic list | grep -E "set_usv_target|setpoint_raw|navigate_to_point"
echo ""

# 发送测试目标点
echo "3️⃣ 发送测试目标点 (60, 35, 0)..."
ros2 topic pub --once /usv_01/set_usv_target_position \
  geometry_msgs/msg/PoseStamped \
  '{pose: {position: {x: 60.0, y: 35.0, z: 0.0}}}'
echo ""

# 监听日志（持续5秒）
echo "4️⃣ 监听USV日志（5秒）..."
timeout 5 ros2 topic echo /usv_01/rosout | grep -E "Action Server|坐标转换" -A 5
echo ""

echo "✅ 调试完成！"
```

使用方法：
```bash
chmod +x debug_nav.sh
./debug_nav.sh
```

---

## 📚 相关文档

- `NAVIGATION_FLOW_COMPLETE.md` - 完整导航流程
- `COORDINATE_SYSTEM_DESIGN.md` - 坐标系统设计
- `ARCHITECTURE_REFACTORING.md` - 架构重构说明

---

**版本**: v1.0  
**日期**: 2025-11-14  
**状态**: ✅ 调试日志已添加
