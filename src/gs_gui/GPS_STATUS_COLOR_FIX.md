# GPS 状态颜色判断修复

## 问题描述

**现象：** GUI 传感器状态栏中，GPS Fix 显示为绿色，但实际卫星数量为 0 sats，HDOP 为 2.5 或 99.0，这明显是错误的状态。

**截图示例：**
```
GPS Fix: 3D Fix  (0 sats, HDOP 2.5)  [绿色背景]  ❌ 不合理
```

## 根本原因

### 旧的判断逻辑（仅基于 fix_type）

位置：`gs_gui/ground_station_node.py` 第 995-1006 行（修复前）

```python
if fix_int <= 1:
    gps_level = 'error'    # 0 (No GPS) 或 1 (No Fix) → 红色
elif fix_int == 2:
    gps_level = 'warn'     # 2 (2D Fix) → 黄色
else:
    gps_level = 'ok'       # 3+ (3D Fix 及以上) → 绿色 ❌ 问题在这里！
```

### 问题分析

1. **只依赖 fix_type**：代码仅根据 `gps_fix_type` 判断，没有考虑卫星数量和 HDOP
2. **飞控报告不准确**：ArduPilot/PX4 在某些情况下会报告 `fix_type=3 (3D Fix)`，即使卫星数为 0
   - 可能是缓存的旧状态
   - 可能是 EKF（扩展卡尔曼滤波）融合了其他传感器数据，仍认为有定位
3. **HDOP 值被忽略**：HDOP (Horizontal Dilution of Precision) 反映定位精度，值越大精度越差
   - HDOP < 2：优秀
   - HDOP 2-5：良好
   - HDOP 5-10：中等
   - HDOP > 10：差
   - HDOP = 99.0：通常表示无效值

## 修复方案

### 新的综合判断逻辑（fix_type + 卫星数 + HDOP）

```python
# 综合判断：fix_type + 卫星数 + HDOP
# 优先级：卫星数 > HDOP > fix_type
if sat_int is not None and sat_int < 4:
    # 卫星数少于4颗，无法可靠定位 → 错误
    gps_level = 'error'
elif eph_val is not None and eph_val > 10.0:
    # HDOP > 10（精度极差）→ 错误
    gps_level = 'error'
elif fix_int <= 1:
    # No GPS 或 No Fix → 错误
    gps_level = 'error'
elif fix_int == 2 or (eph_val is not None and eph_val > 5.0):
    # 2D Fix 或 HDOP > 5（精度较差）→ 警告
    gps_level = 'warn'
else:
    # 3D Fix 及以上，且卫星数≥4，且 HDOP ≤ 5 → 正常
    gps_level = 'ok'
```

### 判断优先级（从高到低）

1. **卫星数量检查**（最重要）
   - `< 4 颗` → 红色（无法可靠定位，GPS 至少需要 4 颗卫星）
   
2. **HDOP 检查**（精度指标）
   - `> 10.0` → 红色（精度极差，不可信）
   - `> 5.0` → 黄色（精度较差，谨慎使用）
   
3. **fix_type 检查**（基础状态）
   - `<= 1 (No GPS/No Fix)` → 红色
   - `== 2 (2D Fix)` → 黄色（缺少高度信息）
   - `>= 3 (3D Fix)` → 绿色（仅在前两项检查通过的情况下）

## 修复后的行为

| 场景 | fix_type | 卫星数 | HDOP | 颜色 | 说明 |
|------|----------|--------|------|------|------|
| 你的问题场景 | 3 (3D Fix) | 0 | 2.5 | **红色** ✅ | 修复后：卫星数 < 4 → 错误 |
| 正常工作 | 3 (3D Fix) | 12 | 1.2 | 绿色 | 所有条件良好 |
| 精度差 | 3 (3D Fix) | 8 | 8.5 | 黄色 | HDOP > 5 → 警告 |
| 完全无信号 | 0 (No GPS) | 0 | 99.0 | 红色 | fix_type ≤ 1 → 错误 |
| 2D 定位 | 2 (2D Fix) | 6 | 3.0 | 黄色 | 2D Fix → 警告 |
| 极差精度 | 3 (3D Fix) | 5 | 15.0 | 红色 | HDOP > 10 → 错误 |

## 代码位置

**文件：** `gs_gui/gs_gui/ground_station_node.py`

**函数：** `_build_sensor_status()` (第 972 行)

**修改范围：** 第 994-1009 行（GPS Fix 颜色判断逻辑）

## 测试建议

### 场景 1：室内测试（无 GPS）
```bash
# 启动地面站，观察 GPS Fix 状态栏
ros2 launch gs_bringup gs_launch.py

# 预期结果：
# GPS Fix: No GPS (0 sats, HDOP 99.0) [红色背景] ✅
```

### 场景 2：室外刚启动（正在搜星）
```
# 预期结果：
# GPS Fix: 3D Fix (2 sats, HDOP 8.5) [红色背景] ✅  (卫星数 < 4)
# 几秒后...
# GPS Fix: 3D Fix (5 sats, HDOP 6.2) [黄色背景] ✅  (HDOP > 5)
# 稳定后...
# GPS Fix: 3D Fix (12 sats, HDOP 1.2) [绿色背景] ✅  (全部正常)
```

### 场景 3：信号遮挡（如树荫下）
```
# 预期结果：
# GPS Fix: 3D Fix (3 sats, HDOP 12.0) [红色背景] ✅  (卫星数 < 4 或 HDOP > 10)
```

## 相关文档

- **HDOP 标准参考**：[Wikipedia - Dilution of Precision](https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation))
- **GPS 定位原理**：至少需要 4 颗卫星（3 颗确定 XY 位置，第 4 颗确定高度和时钟偏差）
- **ArduPilot GPS 文档**：https://ardupilot.org/copter/docs/common-gps-how-it-works.html

## 后续优化建议

### 1. 添加视觉提示优化

在 detail 部分明确标注问题：
```python
if sat_int is not None and sat_int < 4:
    detail_parts.append("⚠️ 卫星不足")
if eph_val is not None and eph_val > 10.0:
    detail_parts.append("⚠️ 精度差")
```

### 2. 添加日志记录

```python
if sat_int is not None and sat_int < 4 and fix_int >= 3:
    self.get_logger().warning(
        f"{usv_id}: GPS 状态异常 - fix_type={fix_int} (3D Fix) 但卫星数={sat_int} < 4"
    )
```

### 3. 历史趋势监控

记录最近 10 次 GPS 状态，避免短暂抖动导致频繁变色：
```python
# 使用滑动窗口平滑判断
if recent_sat_average < 4:
    gps_level = 'error'
```

---

**修复日期：** 2025-11-06  
**修复人员：** AI Agent (GitHub Copilot)  
**影响范围：** 地面站 GUI 传感器状态栏 GPS Fix 颜色显示  
**向后兼容性：** ✅ 完全兼容，仅修改颜色判断逻辑
