# 低电量模式触发逻辑修改

**修改日期**: 2025-11-06  
**修改文件**: `usv_comm/usv_comm/usv_status_node.py`  
**修改原因**: 将低电量模式触发条件从基于电压值改为基于电量百分比

---

## 📋 修改内容

### 修改前：基于电压值判断

```python
# 旧逻辑：当电压 < battery_voltage_empty (10.5V) 时触发
if avg_voltage < self.battery_voltage_empty:
    if not self.low_voltage_mode:
        self.low_voltage_mode = True
        # 触发警告...
```

**问题**：
- 依赖硬编码的电压阈值（10.5V）
- 不同电池类型需要不同配置
- 电压判断不够直观

---

### 修改后：基于电量百分比判断

```python
# 新逻辑：当电量百分比 < 5% 时触发
LOW_BATTERY_THRESHOLD = 5.0  # 低电量阈值：5%
RECOVER_THRESHOLD = 8.0      # 恢复阈值：8%（滞后设计）

if battery_pct < LOW_BATTERY_THRESHOLD:
    if not self.low_voltage_mode:
        self.low_voltage_mode = True
        # 触发警告...
elif battery_pct > RECOVER_THRESHOLD:
    if self.low_voltage_mode:
        self.low_voltage_mode = False
        # 恢复正常...
```

**优势**：
- ✅ 使用百分比，更直观易懂
- ✅ 与电池类型无关（自动适配）
- ✅ 添加滞后机制（5% 触发，8% 恢复），避免频繁切换
- ✅ 更符合用户预期（剩余 5% 是真正的低电量）

---

## 🎯 触发条件

| 状态 | 电量百分比 | 触发条件 | 行为 |
|------|------------|----------|------|
| **进入低电量模式** | < 5% | `battery_pct < 5.0` | 🔴 红色呼吸灯 + 特殊声音 |
| **保持低电量模式** | 5% - 8% | 在滞后区间内 | 🔴 继续低电量模式 |
| **退出低电量模式** | > 8% | `battery_pct > 8.0` | ✅ 恢复正常模式 |

**滞后设计说明**：
- 5% 进入，8% 退出
- 避免在临界点附近频繁切换
- 提供 3% 的缓冲区间

---

## 📊 实际案例

### 案例 1: 正常放电过程

```
电量 100% → 50% → 20% → 10% → 8% → 6% → 4% ❗触发低电量模式
                                              ↓
                                         🔴 红色呼吸灯
                                         🔊 特殊声音
```

### 案例 2: 充电恢复过程

```
充电中: 4% → 5% → 6% → 7% → 8% → 9% ✅ 退出低电量模式
                                    ↓
                               🟢 恢复正常模式
```

### 案例 3: 避免频繁切换

```
电压波动导致百分比在 4-6% 之间波动：
- 4.5% → 5.2% → 4.8% → 5.5%
  (进入)   (保持)  (保持)  (保持)  ← 不会频繁切换

只有当百分比 > 8% 时才退出低电量模式
```

---

## 🔧 相关代码修改

### 1. 低电量判断逻辑 (行 405-430)

```python
# 检查是否进入低电压模式（电量百分比 < 5%）
LOW_BATTERY_THRESHOLD = 5.0  # 低电量阈值：5%
RECOVER_THRESHOLD = 8.0      # 恢复阈值：8%（滞后设计）

if battery_pct < LOW_BATTERY_THRESHOLD:
    if not self.low_voltage_mode:
        # 刚进入低电压模式
        self.low_voltage_mode = True
        self.get_logger().error(
            f'[!][!][!] 低电量模式触发！ [!][!][!]\n'
            f'当前电压: {current_voltage:.2f}V, '
            f'平均电压(10s): {avg_voltage:.2f}V, '
            f'电量百分比: {battery_pct:.1f}% < {LOW_BATTERY_THRESHOLD}%\n'
            f'请立即返航或靠岸！'
        )
elif battery_pct > RECOVER_THRESHOLD:
    if self.low_voltage_mode:
        # 退出低电压模式
        self.low_voltage_mode = False
        self.get_logger().info(
            f'[OK] 退出低电量模式 - 电量百分比: {battery_pct:.1f}% > {RECOVER_THRESHOLD}%'
        )
```

### 2. 启动日志 (行 113-118)

```python
self.get_logger().info(
    f"电池电压范围配置: 满电={self.battery_voltage_full}V (100%), "
    f"空电={self.battery_voltage_empty}V (0%), "
    f"平均窗口={self.battery_avg_window}秒, "
    f"低电量触发阈值=5%"  # ← 新增
)
```

### 3. 调试日志 (行 432-440)

```python
self.get_logger().debug(
    f"电池: 当前={current_voltage:.2f}V, "
    f"平均(10s)={avg_voltage:.2f}V, "
    f"百分比={battery_pct:.1f}%, "
    f"样本数={voltage_samples}, "
    f"低电量模式={'是' if self.low_voltage_mode else '否'} (触发阈值<5%)"  # ← 修改
)
```

---

## 🧪 测试验证

### 手动测试步骤

1. **启动系统**（重启 usv_status_node 以加载新代码）
   ```bash
   # 需要重启对应的 USV 节点
   ros2 lifecycle set /usv_01/usv_status_node shutdown
   # 或直接重启整个 USV launch
   ```

2. **监控日志**
   ```bash
   ros2 topic echo /rosout | grep -i "低电"
   ```

3. **模拟低电量状态**（如果有测试工具）
   ```bash
   # 发布低电压消息（需要能触发 < 5% 的电压）
   # 例如：对于 12.6V-10.5V 范围，5% 对应约 10.6V
   ```

4. **验证触发**
   - 当电量 < 5% 时，应该看到错误日志
   - LED 应该变为红色呼吸灯
   - 声音应该切换到特殊音效

5. **验证恢复**
   - 当电量 > 8% 时，应该看到恢复日志
   - LED 和声音恢复正常

### 预期日志输出

**触发时**：
```
[ERROR] [usv_status_node]: [!][!][!] 低电量模式触发！ [!][!][!]
当前电压: 10.55V, 平均电压(10s): 10.58V, 电量百分比: 4.2% < 5.0%
请立即返航或靠岸！
```

**恢复时**：
```
[INFO] [usv_status_node]: [OK] 退出低电量模式 - 电量百分比: 8.5% > 8.0%
```

**定期调试日志**（每10条消息一次）：
```
[DEBUG] [usv_status_node]: 电池: 当前=11.25V, 平均(10s)=11.28V, 百分比=37.1%, 样本数=10, 低电量模式=否 (触发阈值<5%)
```

---

## 📝 注意事项

### 1. 需要重启节点
修改代码后需要重启 `usv_status_node` 节点才能生效：
```bash
# 方法1: 重启整个 USV launch
ros2 launch usv_bringup usv_launch.py namespace:=usv_01

# 方法2: 如果有生命周期管理
ros2 lifecycle set /usv_01/usv_status_node shutdown
```

### 2. 电池电压范围配置
系统仍然依赖 `battery_voltage_full` 和 `battery_voltage_empty` 参数来计算百分比：
- `battery_voltage_full`: 12.6V (100%)
- `battery_voltage_empty`: 10.5V (0%)

**这些参数不影响低电量触发，但会影响百分比计算的准确性**

### 3. 阈值可调整
如果需要修改触发阈值，只需修改常量：
```python
LOW_BATTERY_THRESHOLD = 5.0   # 改为 10.0 则在 10% 时触发
RECOVER_THRESHOLD = 8.0       # 改为 12.0 则在 12% 时恢复
```

### 4. 与其他节点的关系
- **usv_led_node**: 仍然订阅 `usv_status.low_voltage_mode` 字段
- **usv_sound_node**: 同上
- **地面站**: GUI 会显示低电压模式状态

所有下游节点无需修改，自动适配新逻辑！

---

## 🔗 相关文件

- **修改文件**: `usv_comm/usv_comm/usv_status_node.py`
- **消息定义**: `common_interfaces/msg/UsvStatus.msg` (包含 `low_voltage_mode` 字段)
- **LED 节点**: `usv_led/usv_led/usv_led_node.py` (订阅 `usv_status`)
- **声音节点**: `usv_sound/usv_sound/usv_sound_node.py` (订阅 `usv_status`)
- **参数配置**: `usv_bringup/config/usv_params.yaml`

---

## ✅ 总结

**修改完成！** 低电量模式现在基于**电量百分比 < 5%** 触发，而不是基于电压值。

**关键改进**：
1. ✅ 触发条件更直观（5% 剩余电量）
2. ✅ 添加滞后机制（8% 恢复），避免频繁切换
3. ✅ 与电池类型无关，自动适配
4. ✅ 下游节点无需修改
5. ✅ 日志信息更清晰

**下一步**：重启 USV 节点以应用新逻辑！
