# 电池电量百分比功能使用指南

## 📊 功能概述

系统现已完全基于**飞控实际计算的电量百分比**进行低电量判断，不再使用电压计算方式。

### 关键特性

1. **纯百分比判断**：完全依赖飞控的电量百分比（需配置 BATT_CAPACITY）
2. **准确可靠**：飞控百分比考虑电流积分、电池特性、温度补偿等因素
3. **统一阈值**：LED 和声音节点使用相同的百分比阈值
4. **强制配置**：飞控未配置 BATT_CAPACITY 时，将无法判断低电量（系统会警告）

---

## ⚙️ 飞控参数配置（必需！）

### ⚠️ 重要提示

**系统已移除电压降级模式，必须在 QGroundControl 中配置以下参数，否则低电量功能无法工作！**

### 必需参数（PM07电源模块）

```ini
# 基础监控配置
BATT_MONITOR = 4                # 电压+电流监控
BATT_CAPACITY = 5000            # 电池容量（mAh）⚠️ 必须设置！

# 电压监测
BATT_VOLT_PIN = 2               # 电压引脚
BATT_VOLT_MULT = 10.1           # 电压分压比（需校准）

# 电流监测
BATT_CURR_PIN = 3               # 电流引脚
BATT_AMP_PERVLT = 17.0          # 电流灵敏度（需校准）
BATT_AMP_OFFSET = 0.0           # 电流零点偏移

# 低电量警告阈值
BATT_LOW_VOLT = 10.8            # 低电压警告（V）
BATT_CRT_VOLT = 10.5            # 严重低电压（V）
BATT_LOW_MAH = 1000             # 剩余容量警告（mAh）
BATT_CRT_MAH = 500              # 剩余容量严重警告（mAh）
```

### 校准步骤

**电压校准：**
```
1. 用万用表测量实际电压（如 12.35V）
2. 查看 QGC 显示电压（如 12.15V）
3. 新 VOLT_MULT = 10.1 × (12.35 / 12.15) = 10.27
4. 设置 BATT_VOLT_MULT = 10.27
```

**电流校准：**
```
1. 无负载测试，电流应为 0A
2. 如果显示非零（如 0.3A），设置 BATT_AMP_OFFSET = -0.3
3. 已知负载测试（如 5A），调整 BATT_AMP_PERVLT
```

---

## 🔧 ROS 2 参数配置

### 配置文件位置

`usv_bringup/config/usv_params.yaml`

### LED 节点配置

```yaml
usv_led_node:
  ros__parameters:
    # ⚠️ 电池低电量判断 - 基于飞控百分比（需要在 QGroundControl 中配置 BATT_CAPACITY）
    low_battery_percentage: 10.0        # 低电量阈值 10%
    recover_battery_percentage: 15.0    # 恢复阈值 15%（滞后 5%）
```

### 声音节点配置

```yaml
usv_sound_node:
  ros__parameters:
    # ⚠️ 低电量判断 - 基于飞控百分比（需要在 QGroundControl 中配置 BATT_CAPACITY）
    low_battery_percentage: 10.0        # 低电量阈值 10%
```

---

## 📈 阈值建议

### 推荐配置（3S LiPo）

| 用途 | 百分比阈值 | 对应电压（参考） | 说明 |
|------|----------|----------------|------|
| **警告** | 20% | ~11.1V | 提前警告，建议返航 |
| **低电量** | 10% | ~10.8V | 触发低电量模式（红色呼吸灯+特殊声音） |
| **严重低电量** | 5% | ~10.5V | 强制返航/降落 |
| **过放保护** | 0% | ~10.2V | 电池保护阈值，禁止使用 |

### 滞后设置（防止抖动）

- **LED**: 低电量 10% → 恢复 15%（滞后 5%）
- **声音**: 低电量 10%（立即触发）
- **飞控**: 低电量 1000mAh → 恢复策略（通过充电）

---

## 🚀 使用流程

### 1. 配置飞控（首次）

```bash
# 使用 QGroundControl 连接飞控
# 进入 Parameters → Battery Monitor
# 设置以上参数
# 保存并重启飞控
```

### 2. 验证飞控数据

```bash
# 启动 USV 节点
ros2 launch usv_bringup usv_launch.py namespace:=usv_01 fcu_url:=serial:///dev/ttyACM0:921600

# 查看电池数据
ros2 topic echo /usv_01/battery --once

# 应该看到：
# voltage: 11.587          # 真实电压
# percentage: 0.27         # 27% (0.0-1.0 之间为有效值)
# current: -23.17          # 放电电流（负值）
# capacity: 5000.0         # 总容量

# ⚠️ 如果 percentage = 0.0，说明飞控未配置，需要设置 BATT_CAPACITY！
```

### 3. 查看 ROS 日志

```bash
# usv_status_node 日志（每 10 次记录一次）
[usv_status_node]: 电池: 11.59V, 27.3% (飞控百分比: 0.273)

# 如果飞控未配置，会看到警告：
[usv_status_node]: ⚠️ 飞控电量百分比无效，请在 QGroundControl 中配置 BATT_CAPACITY 参数！

# usv_led_node 日志（状态变化时）
[usv_led_node]: ⚠️ USV 电池电量低于 10.0% (当前: 8.5%)，进入低电量红色呼吸灯模式
[usv_led_node]: ✅ USV 电池电量恢复至 15.0% 以上 (当前: 16.2%)，退出低电量模式

# usv_sound_node 日志
[usv_sound_node]: ⚠️ 电池电量低: 9.2%，切换到低电量声音
[usv_sound_node]: ✅ 电池电量恢复正常: 17.5%
```

---

## 🔍 故障排查

### 问题 1: 百分比始终为 0，低电量功能不工作

**原因**: 飞控 `BATT_CAPACITY` 未设置

**解决**:
```ini
# 在 QGroundControl 中设置
BATT_CAPACITY = 5000  # 设置你的实际电池容量（mAh）
```

**⚠️ 注意**: 系统已移除电压降级模式，必须配置飞控参数才能使用低电量功能！

---

### 问题 2: 百分比不准确

**原因**: 电流传感器未校准

**解决**:
1. 校准 `BATT_VOLT_MULT`（电压）
2. 校准 `BATT_AMP_PERVLT`（电流）
3. 校准 `BATT_AMP_OFFSET`（电流零点）

---

### 问题 3: 低电量模式触发过早/过晚

**原因**: 阈值设置不合理

**解决**: 调整 `usv_params.yaml` 中的阈值
```yaml
# 提高阈值（更保守）
low_battery_percentage: 15.0

# 降低阈值（更激进，不推荐）
low_battery_percentage: 5.0
```

---

## 📊 工作原理

### 数据流

```
飞控 PM07
  ↓ (电压+电流)
飞控固件
  ↓ (计算百分比: 库仑计法 + 电压)
MAVROS /battery topic
  ↓ (BatteryState.percentage = 0.0-1.0)
usv_status_node
  ↓ (直接使用飞控百分比 × 100)
usv_state topic (百分比 0-100%)
  ↓
usv_led_node / usv_sound_node
  ↓ (低电量判断: 纯百分比)
红色呼吸灯 + 特殊声音
```

### 判断逻辑（usv_led_node）

```python
# 1. 检查飞控百分比是否有效
percentage = getattr(msg, 'percentage', -1.0)
if not (0.0 <= percentage <= 1.0):
    # 飞控未配置 BATT_CAPACITY，无法判断
    return

# 2. 转换为百分比
battery_percentage = percentage * 100.0

# 3. 低电量判断（带滞后）
if battery_percentage < low_threshold:  # 10%
    进入低电量模式()
elif battery_percentage > recover_threshold:  # 15%
    退出低电量模式()
```

---

## 🎯 最佳实践

### 必需步骤

**⚠️ 飞控必须配置以下参数，否则低电量功能无法工作：**

```ini
BATT_MONITOR = 4       # 启用电压+电流监控
BATT_CAPACITY = 5000   # 设置实际电池容量（mAh）
```

### 关键参数调优

1. **低电量阈值**: 10%（推荐，对应 3S LiPo 约 10.8V）
2. **恢复阈值**: 15%（滞后 5%，防止抖动）
3. **电池容量**: 根据实际电池设置（如 5000mAh）
4. **校准参数**: 定期用万用表验证电压和电流

### 与旧版本的区别

**v1.0 (电压模式)**:
- ❌ 基于电压范围线性插值计算百分比
- ❌ 不考虑电池特性曲线
- ❌ 不考虑放电电流

**v2.0 (双模式)**:
- ✅ 优先使用飞控百分比
- ⚠️ 飞控未配置时降级电压模式
- ⚠️ 需要手动切换模式

**v3.0 (纯百分比，当前版本)**:
- ✅ 完全基于飞控百分比
- ✅ 考虑电流积分（库仑计法）
- ✅ 考虑电池放电曲线
- ✅ 更准确可靠
- ⚠️ 强制要求配置飞控参数

---

## 📝 相关文件

- `usv_comm/usv_comm/usv_status_node.py` - 电量百分比计算
- `usv_led/usv_led/usv_led_node.py` - LED 低电量模式
- `usv_sound/usv_sound/usv_sound_node.py` - 声音低电量模式
- `usv_bringup/config/usv_params.yaml` - 参数配置文件

---

**最后更新**: 2025-11-04  
**版本**: v2.0 - 支持飞控百分比
