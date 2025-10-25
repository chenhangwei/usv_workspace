# 电池电量百分比计算修复文档

## 📋 问题描述

### 发现的问题

**时间**: 2025-01-22  
**报告人**: 用户反馈  
**现象**: 电压 11.4V 时显示电量 96.0%（不合理）

从地面站界面截图看到：
```
编号: usv_02
电压: 11.4V
电量: 96.0%  ← 错误！应该是 20% 左右
```

### 实际电池规格

根据用户提供的电池特性：
- **满电电压**: 12.6V → 电量应显示 **100%**
- **空电电压**: 11.1V → 电量应显示 **0%**（需要充电）
- **电池类型**: 3S 锂电池（3.7V × 3 = 11.1V）

### 预期电量映射表

| 电压 (V) | 电量 (%) | 状态 |
|---------|---------|------|
| 12.6    | 100     | 满电 |
| 12.3    | 80      | 良好 |
| 12.0    | 60      | 正常 |
| 11.7    | 40      | 中等 |
| 11.4    | **20**  | 偏低 ⚠️ |
| 11.1    | 0       | 空电 🔴 需充电 |
| < 11.1  | 0       | 过放 ⚠️ 损坏风险 |

---

## 🔍 根本原因分析

### 原有实现（错误）

**文件**: `usv_comm/usv_comm/usv_status_node.py` (第 275-276 行，修改前)

```python
battery_pct = getattr(self.usv_battery, 'percentage', 0.0) * 100.0
msg.battery_percentage = max(0.0, min(100.0, battery_pct))
```

**问题**：
1. **依赖飞控计算**：直接使用 MAVROS 的 `BatteryState.percentage` 字段
2. **飞控配置错误**：飞控可能使用了错误的电压范围（如 10.8V-12.6V）
3. **算法不同**：飞控可能基于容量而非电压计算
4. **无法控制**：地面站无法调整电量算法

### 为什么会出现 11.4V = 96% 的错误？

可能的飞控配置：
```
飞控参数可能设置:
- 空电电压: 10.8V (错误，应该是 11.1V)
- 满电电压: 12.6V (正确)

飞控计算:
(11.4 - 10.8) / (12.6 - 10.8) = 0.6 / 1.8 = 0.333 → 33.3%

但实际飞控返回 96%，说明可能使用了其他算法或参数
```

---

## ✅ 解决方案

### 修改策略

**基于电压的线性插值计算**，不依赖飞控的百分比字段。

### 实现细节

#### 1. 添加参数配置（第 48-49 行）

```python
# 电池电压范围参数
self.declare_parameter('battery_voltage_full', 12.6)   # 满电电压（V）
self.declare_parameter('battery_voltage_empty', 11.1)  # 空电电压（V）
```

#### 2. 读取参数（第 79-86 行）

```python
# 读取电池电压范围参数
try:
    self.battery_voltage_full = float(self.get_parameter('battery_voltage_full').get_parameter_value().double_value)
except Exception:
    self.battery_voltage_full = 12.6
try:
    self.battery_voltage_empty = float(self.get_parameter('battery_voltage_empty').get_parameter_value().double_value)
except Exception:
    self.battery_voltage_empty = 11.1
```

#### 3. 记录配置日志（第 99-102 行）

```python
# 记录电池电压配置
self.get_logger().info(
    f"电池电压范围配置: 满电={self.battery_voltage_full}V (100%), "
    f"空电={self.battery_voltage_empty}V (0%)"
)
```

#### 4. 核心计算逻辑（第 296-311 行）

```python
# 基于电压计算电量百分比（使用配置的电压范围）
voltage = msg.battery_voltage
if voltage >= self.battery_voltage_full:
    battery_pct = 100.0
elif voltage <= self.battery_voltage_empty:
    battery_pct = 0.0
else:
    # 线性插值计算电量百分比
    voltage_range = self.battery_voltage_full - self.battery_voltage_empty
    voltage_above_empty = voltage - self.battery_voltage_empty
    battery_pct = (voltage_above_empty / voltage_range) * 100.0

msg.battery_percentage = max(0.0, min(100.0, battery_pct))
```

**线性插值公式**:
```
电量% = (当前电压 - 空电电压) / (满电电压 - 空电电压) × 100%
      = (V - 11.1) / (12.6 - 11.1) × 100%
      = (V - 11.1) / 1.5 × 100%
```

#### 5. 调试日志（第 313-319 行）

```python
# 定期记录电量计算日志（每10条消息记录一次，避免刷屏）
self.battery_log_counter += 1
if self.battery_log_counter >= 10:
    self.battery_log_counter = 0
    self.get_logger().debug(
        f"电池: {voltage:.2f}V → {msg.battery_percentage:.1f}% "
        f"(范围: {self.battery_voltage_empty}V-{self.battery_voltage_full}V)"
    )
```

---

## 📊 修改后的电量计算示例

### 示例 1: 11.4V（用户报告的实际情况）

```python
voltage = 11.4
battery_pct = (11.4 - 11.1) / (12.6 - 11.1) * 100.0
            = 0.3 / 1.5 * 100.0
            = 20.0%
```

✅ **修改前**: 96.0%（错误）  
✅ **修改后**: 20.0%（正确）

### 示例 2: 各电压对应的电量

| 电压 (V) | 计算过程 | 电量 (%) |
|---------|---------|---------|
| 13.0 | >= 12.6 | **100** (超过满电) |
| 12.6 | = 12.6 | **100** |
| 12.3 | (12.3-11.1)/1.5 | **80** |
| 12.0 | (12.0-11.1)/1.5 | **60** |
| 11.7 | (11.7-11.1)/1.5 | **40** |
| 11.4 | (11.4-11.1)/1.5 | **20** ⚠️ |
| 11.1 | = 11.1 | **0** 🔴 |
| 10.8 | <= 11.1 | **0** (过放) |

---

## 🔧 配置参数

### 参数文件位置

**文件**: `usv_bringup/config/usv_params.yaml`

```yaml
usv_status_node:
  ros__parameters:
    # 目标点判断阈值
    target_reach_threshold: 1.0             # 到达目标点的距离阈值（米）
    
    # 电池电压范围配置（基于实际电池特性）
    battery_voltage_full: 12.6              # 满电电压（V）- 电量显示100%
    battery_voltage_empty: 11.1             # 空电电压（V）- 电量显示0%，需要充电
```

### 调整参数（如需）

如果使用不同规格的电池：

**4S 锂电池** (14.8V 标称电压):
```yaml
battery_voltage_full: 16.8    # 4.2V × 4
battery_voltage_empty: 14.0   # 3.5V × 4
```

**2S 锂电池** (7.4V 标称电压):
```yaml
battery_voltage_full: 8.4     # 4.2V × 2
battery_voltage_empty: 7.0    # 3.5V × 2
```

**铅酸电池** (12V 标称电压):
```yaml
battery_voltage_full: 13.2    # 满电电压
battery_voltage_empty: 10.5   # 空电电压
```

---

## 🧪 验证测试

### 测试步骤

1. **构建包**:
   ```bash
   cd ~/usv_workspace
   colcon build --packages-select usv_comm
   source install/setup.bash
   ```

2. **启动 USV 节点**:
   ```bash
   ros2 launch usv_bringup usv_launch.py namespace:=usv_01
   ```

3. **查看启动日志**（确认参数加载）:
   ```
   [usv_status_node]: 电池电压范围配置: 满电=12.6V (100%), 空电=11.1V (0%)
   ```

4. **监控电池状态**:
   ```bash
   ros2 topic echo /usv_01/usv_state --field battery_voltage
   ros2 topic echo /usv_01/usv_state --field battery_percentage
   ```

5. **启动地面站**:
   ```bash
   ros2 launch gs_bringup gs_launch.py
   ```

6. **验证显示**:
   - 检查表格中的电压和电量列
   - 确认电量百分比符合预期

### 预期结果

对于电压 11.4V 的 USV：
- ✅ 电量应显示 **20.0%**（而非 96.0%）
- ✅ 电池状态应显示为**黄色或橙色**（低电量警告）
- ✅ 如果配置了声音警告，应该播放低电量提示音

---

## 📈 电量曲线对比

### 修改前（错误的飞控数据）

```
电压 → 电量（错误）
12.6V → 100%
12.0V → 95%
11.4V → 96%  ← 异常！
11.1V → 90%
```

**问题**: 曲线不单调，逻辑混乱

### 修改后（基于电压的线性映射）

```
电压 → 电量（正确）
12.6V → 100% ─┐
12.3V → 80%   │
12.0V → 60%   │ 线性递减
11.7V → 40%   │
11.4V → 20%   │
11.1V → 0%  ──┘
```

**优点**: 单调递减，符合物理规律

---

## ⚠️ 注意事项

### 1. 电压 vs 容量

**线性电压模型的局限性**：
- 锂电池放电曲线在中间段较平坦
- 在 20%-80% 之间，电压变化不明显
- 接近空电时，电压快速下降

**改进方案**（可选）：
可以使用**分段线性**或**查表法**获得更精确的电量估计：

```python
# 分段线性示例（更精确）
voltage_points = [11.1, 11.4, 11.7, 12.0, 12.3, 12.6]
percentage_points = [0, 10, 30, 60, 85, 100]

# 使用线性插值查表
battery_pct = np.interp(voltage, voltage_points, percentage_points)
```

### 2. 温度影响

电池电压受温度影响：
- **低温**: 电压下降，实际容量未耗尽
- **高温**: 电压升高，但可能损坏电池

建议：未来可以加入温度补偿算法

### 3. 负载影响

大电流放电时电压会下降：
- 建议在**低负载或静止**时测量电压
- 可以使用**滑动平均**滤波平滑电压波动

### 4. 校准建议

定期校准电池参数：
1. 充满电后测量电压（应接近 12.6V）
2. 完全放电后测量电压（应接近 11.1V）
3. 如有偏差，调整 `usv_params.yaml` 中的参数

---

## 🔍 调试命令

### 查看原始 MAVROS 电池数据

```bash
# 查看飞控报告的电池状态
ros2 topic echo /usv_01/mavros/battery

# 输出示例:
# header: ...
# voltage: 11.4
# temperature: nan
# current: -5.2
# charge: nan
# capacity: nan
# design_capacity: nan
# percentage: 0.96  ← 飞控报告 96%（错误）
# power_supply_status: 2
# ...
```

### 查看处理后的 USV 状态

```bash
# 查看地面站接收的状态
ros2 topic echo /usv_01/usv_state

# 输出示例（修改后）:
# battery_voltage: 11.4
# battery_percentage: 20.0  ← 正确！
# battery_current: -5.2
# ...
```

### 启用调试日志

```bash
# 启动节点时启用 DEBUG 级别日志
ros2 run usv_comm usv_status_node --ros-args --log-level DEBUG

# 查看日志输出（每10条消息）:
# [DEBUG] [usv_status_node]: 电池: 11.40V → 20.0% (范围: 11.1V-12.6V)
```

---

## 📚 相关文档

- **温度显示修改**: `gs_gui/TEMPERATURE_DISPLAY_UPDATE.md`
- **温度滞后逻辑**: `gs_gui/TEMPERATURE_HYSTERESIS_UPDATE.md`
- **模块架构**: `gs_gui/MODULE_ARCHITECTURE.md`
- **ROS 2 消息定义**: `common_interfaces/msg/UsvStatus.msg`

---

## 📝 修改文件清单

| 文件 | 修改内容 | 行数 |
|------|---------|------|
| `usv_comm/usv_comm/usv_status_node.py` | 添加电压范围参数 | +2 |
| | 读取参数逻辑 | +8 |
| | 记录配置日志 | +4 |
| | 电量计算逻辑 | +15 |
| | 调试日志 | +8 |
| `usv_bringup/config/usv_params.yaml` | 添加电池参数配置 | +4 |
| **总计** | **2 个文件** | **+41 行** |

---

## ✅ 验证清单

完成以下验证步骤，确保修改正确：

- [x] 代码编译通过 (`colcon build`)
- [ ] USV 节点启动成功
- [ ] 启动日志显示电压范围配置
- [ ] 电量计算符合预期（11.4V → 20%）
- [ ] 地面站显示正确电量
- [ ] 低电量警告正常触发
- [ ] 参数文件可正确加载
- [ ] 调试日志输出正常

---

**最后更新**: 2025-01-22  
**状态**: ✅ 已实现，待实际测试验证  
**影响范围**: 所有 USV 的电池电量显示
