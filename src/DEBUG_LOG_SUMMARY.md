# ✅ 调试日志增强完成

## 📌 修改总结

已在关键节点添加详细的控制台调试日志，让你能清晰看到导航目标点在整个系统中的流转过程。

---

## 🔧 修改的文件

### 1. **地面站端** (`gs_gui`)

#### `ground_station_node.py`
```python
# 离群目标点发送日志
📤 [地面站发送] usv_01
  ├─ Area坐标(XML): X=10.00, Y=5.00, Z=0.00
  ├─ Global坐标: X=60.00, Y=35.00, Z=0.00
  ├─ Local坐标: X=60.00, Y=35.00, Z=0.00
  └─ Yaw: 0.00 rad
```

#### `cluster_controller.py`
```python
# 集群任务目标点发送日志
📤 [集群控制器] Step 1 → usv_01
  ├─ Area坐标: X=10.00, Y=5.00, Z=0.00
  ├─ Global坐标: X=60.00, Y=35.00, Z=0.00
  └─ Local坐标: X=60.00, Y=35.00, Z=0.00
```

---

### 2. **USV Action Server** (`usv_comm`)

#### `navigate_to_point_server.py`
```python
# Action 接收日志
📥 [Action Server 接收] 导航目标点
  ├─ X坐标: 60.000 m
  ├─ Y坐标: 35.000 m
  ├─ Z坐标: 0.000 m
  └─ 超时: 300.0 s

# Action 转发日志
📨 [Action Server 转发] → set_usv_target_position
  ├─ X: 60.000 m
  ├─ Y: 35.000 m
  └─ Z: 0.000 m
```

---

### 3. **坐标转换节点** (`usv_control`)

#### `coord_transform_node.py`

**常规目标点**：
```python
# 接收 XYZ
📥 [坐标转换节点] 接收 XYZ 目标点
  ├─ X(东向): 60.000 m
  ├─ Y(北向): 35.000 m
  └─ Z(高度): 0.000 m

# 发布 GPS (GlobalPositionTarget)
📤 [坐标转换节点] 发布 GlobalPositionTarget
  ├─ 纬度(Lat): 22.5184123°
  ├─ 经度(Lon): 113.9012639°
  ├─ 海拔(Alt): -5.17 m
  ├─ 话题: setpoint_raw/global
  └─ MAVLink: SET_POSITION_TARGET_GLOBAL_INT (ID:86)
```

**避障目标点**：
```python
# 接收避障 XYZ
🚨 [坐标转换节点] 接收避障 XYZ 目标点
  ├─ X(东向): 58.500 m
  ├─ Y(北向): 36.200 m
  └─ Z(高度): 0.000 m

# 发布避障 GPS
📤 [坐标转换节点] 发布避障 GlobalPositionTarget
  ├─ 纬度: 22.5183541°
  ├─ 经度: 113.9012102°
  ├─ 海拔: -5.17 m
  └─ 话题: setpoint_raw/global
```

---

## 🧪 使用方法

### 快速调试脚本

```bash
# 运行调试脚本（默认 usv_01）
cd /home/chenhangwei/usv_workspace/src
./debug_nav.sh

# 指定其他 USV
./debug_nav.sh usv_02
```

**脚本功能**：
1. ✅ 检查关键节点运行状态
2. ✅ 检查关键话题存在性
3. ✅ 显示坐标转换配置
4. ✅ 发送测试目标点
5. ✅ 监听日志输出（10秒）
6. ✅ 检查输出话题数据

---

### 手动查看日志

#### 地面站端
```bash
# 查看集群控制器日志
ros2 topic echo /rosout | grep "集群控制器" -A 4

# 查看地面站发送日志
ros2 topic echo /rosout | grep "地面站发送" -A 5
```

#### USV 端
```bash
# 查看 Action Server 日志
ros2 topic echo /usv_01/rosout | grep "Action Server" -A 5

# 查看坐标转换日志
ros2 topic echo /usv_01/rosout | grep "坐标转换节点" -A 8

# 查看完整日志（包含emoji）
ros2 topic echo /usv_01/rosout | grep -E "📥|📤|📨|🚨" -A 5
```

#### 监听输出话题
```bash
# 查看发送到飞控的 GPS 目标点
ros2 topic echo /usv_01/setpoint_raw/global

# 只看关键字段
ros2 topic echo /usv_01/setpoint_raw/global | grep -E "latitude|longitude|altitude"
```

---

## 📊 完整数据流示例

### 输入：XML 任务文件
```xml
<waypoint id="1" x="10.0" y="5.0" z="0.0" />
```

### Area Center 配置
```yaml
area_center_x: 50.0
area_center_y: 30.0
area_center_z: 0.0
```

### 预期日志输出

#### 1️⃣ 地面站发送
```
📤 [集群控制器] Step 1 → usv_01
  ├─ Area坐标: X=10.00, Y=5.00, Z=0.00      ← XML文件坐标
  ├─ Global坐标: X=60.00, Y=35.00, Z=0.00   ← Area + Offset
  └─ Local坐标: X=60.00, Y=35.00, Z=0.00    ← 与Global相同
```

#### 2️⃣ USV 接收
```
📥 [Action Server 接收] 导航目标点
  ├─ X坐标: 60.000 m                        ← 通过网络接收
  ├─ Y坐标: 35.000 m
  ├─ Z坐标: 0.000 m
  └─ 超时: 300.0 s
```

#### 3️⃣ Action 转发
```
📨 [Action Server 转发] → set_usv_target_position
  ├─ X: 60.000 m                            ← 转发到话题
  ├─ Y: 35.000 m
  └─ Z: 0.000 m
```

#### 4️⃣ 坐标转换接收
```
📥 [坐标转换节点] 接收 XYZ 目标点
  ├─ X(东向): 60.000 m                      ← 订阅话题接收
  ├─ Y(北向): 35.000 m
  └─ Z(高度): 0.000 m
```

#### 5️⃣ GPS 转换发布
```
📤 [坐标转换节点] 发布 GlobalPositionTarget
  ├─ 纬度(Lat): 22.5184123°                 ← XYZ → GPS
  ├─ 经度(Lon): 113.9012639°
  ├─ 海拔(Alt): -5.17 m
  ├─ 话题: setpoint_raw/global              ← 发布话题
  └─ MAVLink: SET_POSITION_TARGET_GLOBAL_INT (ID:86)
```

#### 6️⃣ 飞控接收（通过 MAVROS）
```
# 监听话题
ros2 topic echo /usv_01/setpoint_raw/global

latitude: 22.5184123                        ← 飞控收到GPS坐标
longitude: 113.9012639
altitude: -5.17
coordinate_frame: 6  # FRAME_GLOBAL_INT
```

---

## 🔍 调试技巧

### 1. **分层验证**
```bash
# 第1层：地面站发送
ros2 topic echo /rosout | grep "地面站发送" -A 5

# 第2层：USV接收
ros2 topic echo /usv_01/rosout | grep "Action Server 接收" -A 4

# 第3层：坐标转换
ros2 topic echo /usv_01/rosout | grep "坐标转换节点" -A 8

# 第4层：飞控接收
ros2 topic echo /usv_01/setpoint_raw/global
```

### 2. **对比坐标值**
- Area坐标: `(10.0, 5.0, 0.0)` ← XML文件
- Global坐标: `(60.0, 35.0, 0.0)` ← Area + Offset(50, 30, 0)
- Local坐标: `(60.0, 35.0, 0.0)` ← 与Global相同
- GPS坐标: `(22.5184°N, 113.9013°E)` ← XYZ转GPS

### 3. **保存日志**
```bash
# 保存完整日志
ros2 topic echo /usv_01/rosout > nav_debug_$(date +%Y%m%d_%H%M%S).log

# 只保存导航相关
ros2 topic echo /usv_01/rosout | grep -E "Action Server|坐标转换" -A 10 > nav.log
```

### 4. **实时监控**
```bash
# 多终端监控
# 终端1: 地面站日志
ros2 topic echo /rosout | grep "地面站" -A 5

# 终端2: USV日志
ros2 topic echo /usv_01/rosout | grep -E "📥|📤" -A 5

# 终端3: GPS输出
ros2 topic echo /usv_01/setpoint_raw/global

# 终端4: MAVLink抓包
nc -ul 192.168.10.1 -p 14550 | hexdump -C | grep "86"
```

---

## 📁 新增文件

1. ✅ `DEBUG_NAVIGATION_GUIDE.md` - 详细调试指南
2. ✅ `debug_nav.sh` - 快速调试脚本
3. ✅ `DEBUG_LOG_SUMMARY.md` - 本文档

---

## 🎯 验证清单

使用以下清单验证日志是否正常：

- [ ] 地面站发送日志包含 Area/Global/Local 坐标
- [ ] USV Action Server 显示接收到目标点
- [ ] Action Server 显示转发到 set_usv_target_position
- [ ] 坐标转换节点显示接收 XYZ 坐标
- [ ] 坐标转换节点显示发布 GPS 坐标
- [ ] setpoint_raw/global 话题有数据输出
- [ ] 坐标值从 Area → GPS 逐层可追踪

---

## 💡 常见问题

### Q1: 没有看到日志输出？
```bash
# 检查日志级别
ros2 run rqt_logger_level rqt_logger_level

# 或直接设置为 INFO
ros2 service call /${USV_NS}/coord_transform_node/set_logger_level \
  rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: '', level: 20}]}"
```

### Q2: emoji 显示乱码？
```bash
# 确保终端支持 UTF-8
export LANG=en_US.UTF-8

# 或使用过滤器去掉emoji
ros2 topic echo /usv_01/rosout | sed 's/[^\x00-\x7F]//g'
```

### Q3: 日志太多看不清？
```bash
# 只看关键字
ros2 topic echo /usv_01/rosout | grep -E "X坐标|纬度" --color=always

# 或使用脚本
./debug_nav.sh
```

---

## 📚 相关文档

- `NAVIGATION_FLOW_COMPLETE.md` - 完整导航流程说明
- `DEBUG_NAVIGATION_GUIDE.md` - 详细调试指南
- `ARCHITECTURE_REFACTORING.md` - 架构重构说明
- `COORDINATE_SYSTEM_DESIGN.md` - 坐标系统设计

---

**版本**: v1.0  
**日期**: 2025-11-14  
**状态**: ✅ 调试日志已部署，可以开始调试！
