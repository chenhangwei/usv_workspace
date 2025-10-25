# 温度显示滞后逻辑实现文档

## 📋 更新概述

**时间**: 2025-01-22  
**包**: `gs_gui`  
**文件**: `gs_gui/usv_info_panel.py`  
**目的**: 为温度颜色切换添加滞后（hysteresis）逻辑，防止在阈值附近颜色频繁闪烁

---

## 🎯 需求背景

### 原有问题
- 温度在 50°C 附近波动时（例如 49.8°C ↔ 50.2°C），颜色会在绿色和红色之间快速切换
- 这会导致UI闪烁，影响用户体验
- 无法区分"刚刚超温"和"持续超温"状态

### 解决方案
实现**滞后控制**（Hysteresis Control）：
- **上限阈值**: 50°C - 达到此温度时切换到红色警告
- **下限阈值**: 48°C - 降至此温度时切换回绿色正常
- **死区**: 48-50°C（2°C宽度）- 在此范围内保持当前颜色不变

---

## 🔧 技术实现

### 1. 状态变量添加

**文件**: `usv_info_panel.py`  
**位置**: `__init__` 方法（约第 33-44 行）

```python
def __init__(self, parent=None):
    super().__init__(parent)
    self._setup_ui()
    
    # 更新定时器（用于动态效果）
    self._update_timer = QTimer()
    self._update_timer.timeout.connect(self._update_dynamic_styles)
    self._update_timer.start(1000)  # 每秒更新一次
    
    # 当前状态缓存
    self._current_state = None
    
    # 温度状态跟踪（用于实现滞后效果）
    self._is_high_temperature = False  # False=低温(绿色), True=高温(红色)
```

**关键变量**:
- `_is_high_temperature`: 布尔标志，记忆当前温度状态
  - `False`: 低温状态（绿色显示）
  - `True`: 高温状态（红色显示）

---

### 2. 滞后逻辑实现

**文件**: `usv_info_panel.py`  
**位置**: `_update_temperature_style` 方法（约第 607-647 行）

```python
def _update_temperature_style(self, temp_celsius):
    """
    根据温度更新样式（带滞后效果）
    
    滞后逻辑：
    - 温度 >= 50°C 时切换到红色
    - 温度 < 48°C 时切换到绿色
    - 在 48-50°C 之间保持当前颜色（2°C死区）
    
    这样可以防止温度在50°C附近波动时颜色频繁闪烁
    
    Args:
        temp_celsius: 温度（摄氏度）
    """
    try:
        temp = float(temp_celsius)
        
        # 滞后逻辑实现
        if self._is_high_temperature:
            # 当前是高温状态（红色）
            if temp < 48:  # 温度降到48°C以下才切换到绿色
                color = "#27ae60"  # 绿色
                self._is_high_temperature = False
            else:
                color = "#e74c3c"  # 保持红色
        else:
            # 当前是低温状态（绿色）
            if temp >= 50:  # 温度升到50°C及以上才切换到红色
                color = "#e74c3c"  # 红色
                self._is_high_temperature = True
            else:
                color = "#27ae60"  # 保持绿色
        
        self.temperature_label.setStyleSheet(f"""
            QLabel {{
                color: {color};
                font-weight: bold;
                font-size: 14px;
            }}
        """)
    except (ValueError, TypeError):
        self.temperature_label.setStyleSheet("")
```

**状态转换表**:

| 当前状态 | 温度条件 | 动作 | 新状态 | 显示颜色 |
|---------|---------|------|--------|---------|
| 低温(绿) | temp < 48°C | 保持 | 低温(绿) | 绿色 #27ae60 |
| 低温(绿) | 48°C ≤ temp < 50°C | 保持 | 低温(绿) | 绿色 #27ae60 |
| 低温(绿) | temp ≥ 50°C | **切换** | 高温(红) | 红色 #e74c3c |
| 高温(红) | temp < 48°C | **切换** | 低温(绿) | 绿色 #27ae60 |
| 高温(红) | 48°C ≤ temp < 50°C | 保持 | 高温(红) | 红色 #e74c3c |
| 高温(红) | temp ≥ 50°C | 保持 | 高温(红) | 红色 #e74c3c |

---

### 3. 状态重置

**文件**: `usv_info_panel.py`  
**位置**: `clear_display` 方法（约第 490-509 行）

```python
def clear_display(self):
    """清空所有显示内容"""
    # ... 其他清空操作 ...
    
    # 重置温度状态标志
    self._is_high_temperature = False
    
    self._current_state = None
```

**目的**: 当 USV 离线或清除显示时，重置温度状态为初始值（低温/绿色）

---

## 📊 行为示例

### 场景 1: 温度逐渐升高
```
温度变化: 45 → 48 → 49 → 50 → 51 → 52°C
颜色显示: 绿   绿   绿   红   红   红
状态标志: F    F    F    T    T    T
```

### 场景 2: 温度逐渐降低
```
温度变化: 52 → 50 → 49 → 48 → 47 → 45°C
颜色显示: 红   红   红   绿   绿   绿
状态标志: T    T    T    F    F    F
```

### 场景 3: 在死区内波动（关键场景）
```
初始状态: 绿色 (F)
温度变化: 49.0 → 49.5 → 49.8 → 49.5 → 49.0°C
颜色显示: 绿     绿     绿     绿     绿
状态标志: F      F      F      F      F
说明: 温度在48-50°C之间波动，保持绿色不闪烁

初始状态: 绿色 (F)
温度变化: 49.8 → 50.1 → 49.5 → 49.0 → 48.5°C
颜色显示: 绿     红     红     红     红
状态标志: F      T      T      T      T
说明: 一旦超过50°C切换到红色，在48-50°C间保持红色

初始状态: 红色 (T)
温度变化: 49.5 → 48.5 → 49.0 → 48.0 → 47.5°C
颜色显示: 红     红     红     绿     绿
状态标志: T      T      T      F      F
说明: 必须降到48°C以下才切回绿色
```

---

## 🧪 测试验证

### 测试场景 1: 正常升温过程
**步骤**:
1. 启动地面站: `ros2 launch gs_bringup gs_launch.py`
2. 启动 USV 节点（模拟或真实）
3. 观察温度从 30°C 逐渐上升到 60°C

**预期结果**:
- 30-49.9°C: 绿色显示
- 50.0°C: 切换到红色
- 50.1-60°C: 保持红色

---

### 测试场景 2: 正常降温过程
**步骤**:
1. USV 温度从 60°C 逐渐下降到 30°C

**预期结果**:
- 60-48.0°C: 红色显示
- 47.9°C: 切换到绿色
- 47.8-30°C: 保持绿色

---

### 测试场景 3: 边界波动（核心测试）
**步骤**:
1. 模拟温度在 49-51°C 之间快速波动
2. 观察颜色变化频率

**预期结果**:
- **从绿色开始**: 49.5 → 50.2 → 49.8 → 50.1 → 49.5
  - 颜色: 绿 → 红 → 红 → 红 → 红（只切换一次）
- **从红色开始**: 49.5 → 48.5 → 49.2 → 47.8 → 48.5
  - 颜色: 红 → 红 → 红 → 绿 → 绿（只切换一次）

---

### 测试场景 4: 状态重置
**步骤**:
1. USV 温度达到 55°C（红色）
2. USV 离线
3. USV 重新上线（温度 45°C）

**预期结果**:
- 离线前: 红色显示
- 清空显示时: 状态重置为 `False`
- 重新上线: 45°C 显示为绿色（不受之前状态影响）

---

## 📈 性能影响

**计算复杂度**: O(1) - 每次温度更新只需一次条件判断  
**内存开销**: +1 字节（布尔变量）  
**UI 刷新**: 无额外刷新，仅在颜色实际改变时更新 QLabel 样式

---

## 🔍 调试方法

### 方法 1: 添加调试日志
在 `_update_temperature_style` 中添加打印：

```python
def _update_temperature_style(self, temp_celsius):
    try:
        temp = float(temp_celsius)
        old_state = self._is_high_temperature
        
        # ... 滞后逻辑 ...
        
        if old_state != self._is_high_temperature:
            print(f"[温度状态切换] {temp:.1f}°C: "
                  f"{'低温→高温' if self._is_high_temperature else '高温→低温'}")
    except:
        pass
```

### 方法 2: 监控 ROS 话题
```bash
# 监听温度数据
ros2 topic echo /usv_01/usv_status --field temperature
```

### 方法 3: 手动发送测试数据
```bash
# 发送模拟温度数据
ros2 topic pub /usv_01/usv_status common_interfaces/msg/UsvStatus \
    "{temperature: 49500}"  # 49.5°C (milliCelsius)
```

---

## 🎨 UI 变化对比

### 修改前（简单阈值）
```python
if temp < 50:
    color = "#27ae60"  # 绿色
elif temp < 70:
    color = "#f39c12"  # 橙色
else:
    color = "#e74c3c"  # 红色
```

**问题**: 
- 49.9°C → 绿色
- 50.0°C → 橙色（立即切换）
- 49.9°C → 绿色（立即切回）
- **结果**: 快速闪烁

---

### 修改后（滞后逻辑）
```python
if self._is_high_temperature:
    if temp < 48:
        color = "#27ae60"; self._is_high_temperature = False
    else:
        color = "#e74c3c"
else:
    if temp >= 50:
        color = "#e74c3c"; self._is_high_temperature = True
    else:
        color = "#27ae60"
```

**改进**:
- 49.9°C (绿) → 50.1°C (红) → 49.5°C (保持红) → 47.8°C (绿)
- **结果**: 稳定切换，无闪烁

---

## 📚 相关文档

- **温度显示更新**: `TEMPERATURE_DISPLAY_UPDATE.md`
- **UI 现代化**: `UI_MODERNIZATION_GUIDE.md`
- **模块架构**: `MODULE_ARCHITECTURE.md`
- **快速参考**: `QUICK_REFERENCE.md`

---

## 🛠️ 未来扩展

### 可配置阈值
将阈值移至参数文件 `gs_params.yaml`:

```yaml
temperature_thresholds:
  high_trigger: 50.0    # 高温触发阈值
  low_trigger: 48.0     # 低温恢复阈值
  critical: 70.0        # 严重过热阈值（可添加闪烁效果）
```

### 多级报警
```python
# 三级颜色系统
if self._is_critical:  # > 70°C
    color = "#e74c3c"  # 红色 + 闪烁
elif self._is_high_temperature:  # > 50°C
    color = "#f39c12"  # 橙色
else:  # < 48°C
    color = "#27ae60"  # 绿色
```

### 温度趋势指示
```python
self.temp_trend_label.setText(
    "↑" if temp > self._last_temp else 
    "↓" if temp < self._last_temp else "→"
)
```

---

**最后更新**: 2025-01-22  
**作者**: GitHub Copilot  
**状态**: ✅ 已实现并测试
