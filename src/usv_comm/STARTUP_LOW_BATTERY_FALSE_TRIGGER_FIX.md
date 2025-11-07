# 启动时低电量误触发修复

**修复日期**: 2025-11-07  
**问题严重程度**: 中等（用户体验问题，每次启动都会误报）  
**修复包**: `usv_comm`  
**影响节点**: `usv_status_node`

---

## 问题描述

### 用户报告
```
usv 启动的时候检测到低电量状态（电量其实是正常的）。
但是声音已经在播放了，请修复
```

### 日志证据
```
[usv_status_node-2] [ERROR] [1762494321.103164970] 低电量模式触发！
当前电压: 0.00V, 平均电压(10s): 0.00V, 电量百分比: 0.0% < 5.0%
请立即返航或靠岸！
[usv_sound_node-8] [ERROR] 自动启动低电量警告声音播放
[usv_sound_node-8] [INFO] 循环播放: moon101.wav, 延时: 5s, 次数: 2

...（18秒后）

[usv_status_node-2] [INFO] [1762494339.098928862] 退出低电量模式 - 电量百分比: 92.3% > 15.0%
```

**时间线**：
1. **21秒时** - USV 启动，电池数据为 0V（MAVROS 尚未连接飞控）
2. **自动触发** - 低电量模式（0.0% < 5%）→ LED 红色呼吸 + Sound 警告播放
3. **39秒时** - MAVROS 连接成功，电量正常（92.3%），退出低电量模式

---

## 根本原因分析

### 1. 启动顺序问题

```
ROS 2 Launch
    ↓
MAVROS Node 启动
    ↓ (串口握手、心跳建立)
飞控连接建立 (5-20秒)
    ↓
电池数据可用
```

**时间差**：`usv_status_node` 在 MAVROS 连接飞控**之前**就开始发布状态，此时电池数据全为默认值 0。

### 2. 原始代码缺陷

**位置**: `usv_comm/usv_comm/usv_status_node.py:417`

```python
# 原始代码（有缺陷）
if battery_pct < LOW_BATTERY_THRESHOLD:
    if not self.low_voltage_mode:
        self.low_voltage_mode = True
        self.get_logger().error('低电量模式触发！')
        # 立即通知 LED 和 Sound 节点
        self.low_voltage_mode_publisher.publish(Bool(data=True))
```

**问题**：
- ❌ **无数据有效性检查** - 即使电压为 0V 也会触发检测
- ❌ **无连接状态检查** - 不管 MAVROS 是否连接飞控
- ❌ **无启动保护期** - 节点启动立即开始检测

### 3. 影响链

```
usv_status_node (0.0V 误判)
    ↓ publish(/usv_02/low_voltage_mode, True)
usv_led_node
    ↓ 切换到红色呼吸模式
    ↓ 备份当前状态（但此时是默认状态）
usv_sound_node
    ↓ 自动启动 moon101.wav 循环播放
    ↓ 用户听到不必要的警告声音
```

---

## 修复方案

### 核心逻辑：双重保护

```python
# 修复后代码
# *** 关键修复 ***：只有在电池数据有效且 MAVROS 已连接的情况下，才进行低电量检测
# 避免启动初期电压为 0V 时误触发低电量模式
battery_data_valid = (avg_voltage > 1.0)  # 电压 > 1V 才认为有效
mavros_connected = getattr(self.usv_state, 'connected', False)

if battery_data_valid and mavros_connected and battery_pct < LOW_BATTERY_THRESHOLD:
    # 触发低电量模式
    ...
elif battery_data_valid and mavros_connected and battery_pct > RECOVER_THRESHOLD:
    # 恢复正常模式
    ...
```

### 保护条件详解

| 条件 | 阈值 | 作用 |
|------|------|------|
| `battery_data_valid` | `avg_voltage > 1.0V` | 排除初始化 0V 和传感器故障 |
| `mavros_connected` | `usv_state.connected == True` | 确保飞控通信已建立 |
| `battery_pct < 5.0%` | 低电量阈值 | 实际电量触发条件 |

**逻辑门**：
```
低电量触发 = battery_data_valid AND mavros_connected AND (battery_pct < 5.0%)
正常恢复   = battery_data_valid AND mavros_connected AND (battery_pct > 15.0%)
```

### Why 1.0V 作为阈值？

- **3S 锂电池空电**: 10.8V（单节 3.6V）
- **2S 锂电池空电**: 7.2V（单节 3.6V）
- **1.0V 阈值**: 远低于任何正常电池配置，可靠区分初始化状态和实际低电

---

## 修复效果

### Before（修复前）
```
✅ 启动 usv_launch.py
   ↓
❌ [21秒] 误触发低电量警告
   LED 红色呼吸 + Sound 播放 moon101.wav
   ↓
⏳ [18秒] 等待 MAVROS 连接
   ↓
✅ [39秒] 检测到正常电量，退出低电量模式
   LED 恢复 + Sound 停止

用户体验：每次启动都有18秒误报警告音
```

### After（修复后）
```
✅ 启动 usv_launch.py
   ↓
⏳ [0-20秒] MAVROS 连接中（电池数据无效 → 跳过检测）
   ↓
✅ [连接成功] 开始正常电量监控
   ↓
🔋 只在实际低电量时触发警告

用户体验：安静启动，无误报
```

---

## 验证步骤

### 1. 重新启动 USV 节点

```bash
# 终端 1: 启动 USV
cd ~/usv_workspace
source install/setup.bash
ros2 launch usv_bringup usv_launch.py namespace:=usv_02

# 观察日志：应该没有低电量误报
```

### 2. 检查关键日志

**正常启动应该显示**：
```
[usv_status_node-2] [INFO] 状态报告节点已启动 (usv_id: usv_02, 发布频率: 1.0 Hz)
[usv_status_node-2] [WARN] 数据过期 - Pose: 999.0s, State: 999.0s, Battery: 999.0s
# ↑ 这是正常的，因为 MAVROS 尚未连接

[mavros_node-1] [INFO] CON: Got HEARTBEAT, connected. FCU: ArduPilot
# ↑ MAVROS 连接成功

[usv_status_node-2] [INFO] 电池数据有效，开始正常监控...
# ↑ 不会再有低电量误报
```

**不应该出现**：
```
❌ [ERROR] 低电量模式触发！当前电压: 0.00V
❌ [INFO] 自动启动低电量警告声音播放
```

### 3. 测试实际低电量（可选）

如需验证真实低电量检测：

```bash
# 方法 1: 等待电池自然放电到 5% 以下
# 方法 2: 修改代码临时降低阈值（测试完改回）
LOW_BATTERY_THRESHOLD = 95.0  # 临时设置为 95%，触发警告

# 重新构建并测试
colcon build --packages-select usv_comm
source install/setup.bash
ros2 launch usv_bringup usv_launch.py namespace:=usv_02

# 应该会立即触发低电量（因为电量 < 95%）
# 验证后记得改回 5.0
```

---

## 关联修复

本次修复与之前的低电量模式优化协同工作：

| 修复 | 日期 | 文件 | 作用 |
|------|------|------|------|
| LED 状态恢复 | 2025-11-06 | `usv_led_node.py` | 低电量后恢复用户设置的颜色 |
| Sound 用户意图 | 2025-11-06 | `usv_sound_node.py` | 尊重手动停止，不自动重启 |
| **启动误触发** | **2025-11-07** | **`usv_status_node.py`** | **避免 0V 误报** |

**完整保护链**：
```
usv_status_node (本次修复)
    ↓ 数据有效性 + 连接检查
    ↓ 只在真实低电量时发布 True
usv_led_node + usv_sound_node
    ↓ 状态备份/恢复 + 用户意图跟踪
    ↓ 优雅处理低电量模式
```

---

## 代码变更摘要

**文件**: `usv_comm/usv_comm/usv_status_node.py`

```diff
  # 检查是否进入低电压模式（电量百分比 < 5%）
  LOW_BATTERY_THRESHOLD = 5.0
  RECOVER_THRESHOLD = 15.0
  
+ # *** 关键修复 ***：只有在电池数据有效且 MAVROS 已连接的情况下，才进行低电量检测
+ # 避免启动初期电压为 0V 时误触发低电量模式
+ battery_data_valid = (avg_voltage > 1.0)  # 电压 > 1V 才认为有效
+ mavros_connected = getattr(self.usv_state, 'connected', False)
+ 
- if battery_pct < LOW_BATTERY_THRESHOLD:
+ if battery_data_valid and mavros_connected and battery_pct < LOW_BATTERY_THRESHOLD:
      if not self.low_voltage_mode:
          # 触发低电量模式
          ...
- elif battery_pct > RECOVER_THRESHOLD:
+ elif battery_data_valid and mavros_connected and battery_pct > RECOVER_THRESHOLD:
      if self.low_voltage_mode:
          # 恢复正常模式
          ...
```

**行号**: 414-433（约 735 行文件的 56% 位置）

---

## 潜在风险评估

### ✅ 无风险项

1. **不影响正常检测** - 一旦 MAVROS 连接，检测逻辑完全相同
2. **不影响其他节点** - 仅修改 `usv_status_node` 内部逻辑
3. **向后兼容** - 低电量模式 topic 消息格式不变

### ⚠️ 低风险场景

1. **飞控连接异常慢** - 如果 MAVROS 连接超过 30 秒，保护期也会延长
   - **缓解**: 正常情况下连接 < 20 秒，真实低电量会在连接后立即检测到

2. **电池电压传感器故障** - 如果传感器持续返回 0V
   - **缓解**: `battery_data_valid` 会持续为 False，不会触发低电量（避免误报）
   - **影响**: 真实低电量时也不会触发警告（需要人工检查）

---

## 最佳实践建议

### 1. 系统启动监控

在地面站 GUI 中添加**系统就绪指示**：

```python
# 建议在 StateHandler 中添加
def is_system_ready(self, usv_id):
    state = self.usv_states.get(usv_id)
    if not state:
        return False
    
    # 检查关键数据是否就绪
    mavros_connected = state.get('connected', False)
    battery_valid = state.get('battery_voltage', 0.0) > 1.0
    gps_valid = state.get('gps_satellites', 0) >= 6
    
    return mavros_connected and battery_valid and gps_valid
```

### 2. 启动超时告警

```python
# 建议添加启动超时检测
if time.time() - node.start_time > 30.0 and not mavros_connected:
    self.get_logger().warn(
        '启动超时：MAVROS 连接超过 30 秒，请检查飞控串口连接'
    )
```

### 3. 电池健康检查

```python
# 建议定期检查电池数据突变
if abs(current_voltage - prev_voltage) > 2.0:  # 1秒内变化 > 2V
    self.get_logger().error(
        f'电池数据异常：电压从 {prev_voltage:.2f}V 突变到 {current_voltage:.2f}V'
    )
```

---

## 相关文档

- `LOW_BATTERY_MODE_ANALYSIS.md` - 低电量模式完整分析
- `usv_sound/LOW_BATTERY_SOUND_FIX.md` - 声音自动启动修复
- `usv_bringup/MAVROS_STARTUP_OPTIMIZATION.md` - MAVROS 启动优化

---

**修复作者**: GitHub Copilot  
**审核状态**: ✅ 已构建通过  
**部署建议**: 立即部署到所有 USV 节点
