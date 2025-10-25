# USV 启动差异 - 快速参考

## 核心问题

**Q: 为什么 USV_02 没有打印 "Failed to set EKF origin!" 错误？**

**A: 因为 USV_02 的 GPS 初始化更慢，还在等待 GPS 配置数据，所以根本没有尝试设置 EKF origin。**

## 三个 USV 的状态

| USV | GPS 状态 | 行为 | 结果 |
|-----|---------|------|------|
| **USV_01** | 部分就绪，EKF 未准备好 | ✅ 尝试设置 | ❌ 失败（会重试） |
| **USV_02** | **等待 GPS 配置数据** | ⏸️ **不尝试设置** | ✅ **无错误** |
| **USV_03** | 部分就绪，时钟错误 | ✅ 尝试设置 | ❌ 失败（会重试） |

## 代码逻辑

```python
# auto_set_home_node.py - check_and_set_home()

if self.gps_fix_type < 0:  # GPS 未就绪
    return  # 👈 USV_02 在这里返回，不尝试设置
    
# GPS 就绪，继续设置
self.create_timer(self.set_delay_sec, self.set_home)  # 👈 USV_01/03 执行这里
```

## GPS 初始化阶段

```
阶段1: GPS 上电
    ↓
阶段2: 接收卫星信号
    ↓
阶段3: 获取位置数据 ← USV_02 卡在这里
    ↓
阶段4: 提供配置数据给 EKF ← USV_01/03 可能到这里但 EKF 未就绪
    ↓
阶段5: EKF 初始化完成
    ↓
阶段6: 可以设置 EKF origin ✅
```

## 为什么初始化速度不同？

1. **GPS 模块性能差异**
2. **卫星信号强度不同**（位置、遮挡）
3. **飞控启动时序差异**
4. **串口通信速度**
5. **系统负载**

## 这是问题吗？

### ✅ 正常现象

- GPS 初始化本身就是异步的
- USV_02 的保护机制正常工作（等待就绪）
- 最终都会成功

### ⚠️ 需要关注

- Command 179 超时（已优化）
- Radio failsafe（遥控器未连接）
- USV_03 的时钟错误

## 如何监控？

```bash
# 查看 GPS 状态
ros2 topic echo /usv_01/mavros/global_position/global
ros2 topic echo /usv_02/mavros/global_position/global
ros2 topic echo /usv_03/mavros/global_position/global

# 查看 MAVROS 连接状态
ros2 topic echo /usv_01/mavros/state
```

## 日志解读

### USV_02 的日志
```
[mavros_node-1] FCU: EKF3 waiting for GPS config data
```
👆 这是**正常的**，说明在等待 GPS 数据

### USV_01/03 的日志
```
[auto_set_home_node-6] [ERROR] Failed to set EKF origin!
[mavros_node-1] [WARN] CMD: Command 179 -- ack timeout
```
👆 尝试设置但失败，**会自动重试**，最终会成功

## 优化建议

1. **增加延迟**: 让 GPS 有更多时间初始化
   ```python
   'set_delay_sec': 5.0  # 默认 3.0
   ```

2. **等待更多条件**: 除了 GPS fix，还检查卫星数量

3. **增加日志**: 显示 GPS 初始化进度

## 相关文档

- 详细分析: `USV_STARTUP_DIFFERENCES_ANALYSIS.md`
- MAVLink 超时: `gs_gui/MAVLINK_COMMAND_TIMEOUT_GUIDE.md`
- MAVROS 优化: `usv_bringup/MAVROS_STARTUP_OPTIMIZATION.md`

---

**结论**: USV_02 没有错误是因为它**智能地等待 GPS 就绪**，而不是过早尝试设置。这是**正确的行为**。
