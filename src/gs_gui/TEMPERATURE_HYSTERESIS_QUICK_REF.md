# 温度滞后逻辑 - 快速参考

## 🎯 核心逻辑（2 秒理解）

```
温度 ≥ 50°C  →  红色警告
温度 < 48°C  →  绿色正常
48-50°C 之间  →  保持当前颜色（死区）
```

---

## 📊 状态转换图

```
     绿色状态                          红色状态
  (温度正常)                          (温度过高)
       │                                  │
       │  温度 ≥ 50°C                     │  温度 < 48°C
       └─────────────► 切换到红色 ◄──────────┘
                                     
    48-50°C 死区：
    - 从绿色进入 → 保持绿色
    - 从红色进入 → 保持红色
```

---

## 🔧 实现位置

**文件**: `gs_gui/gs_gui/usv_info_panel.py`

**状态变量**（第 43 行）:
```python
self._is_high_temperature = False  # 温度状态标志
```

**逻辑方法**（第 607-647 行）:
```python
def _update_temperature_style(self, temp_celsius):
    if self._is_high_temperature:
        if temp < 48:
            # 切换到绿色
        else:
            # 保持红色
    else:
        if temp >= 50:
            # 切换到红色
        else:
            # 保持绿色
```

---

## ✅ 测试示例

### 场景 1: 温度上升
```
45°C → 48°C → 49°C → 50°C → 51°C
 🟢     🟢     🟢     🔴     🔴
(绿)   (绿)   (绿)   (红)   (红)
```

### 场景 2: 温度下降
```
55°C → 50°C → 49°C → 48°C → 47°C
 🔴     🔴     🔴     🔴     🟢
(红)   (红)   (红)   (红)   (绿)
```

### 场景 3: 死区波动（关键）
```
初始: 45°C 绿色
49°C → 50°C → 49.5°C → 48.5°C → 47.5°C
 🟢     🔴      🔴       🔴       🟢
(绿)   (红)    (红)     (红)     (绿)

说明：
- 50°C: 触发上限，切换到红色
- 49.5°C: 死区内，保持红色
- 48.5°C: 死区内，保持红色
- 47.5°C: 触发下限，切换到绿色
```

---

## 🧪 运行测试

```bash
# 可视化演示 + 单元测试
cd /home/chenhangwei/usv_workspace/src/gs_gui
python3 test/test_temperature_hysteresis.py

# 仅单元测试
python3 -m pytest test/test_temperature_hysteresis.py -v
```

---

## 📈 优点

1. **防止闪烁**: 温度在 50°C 附近波动时不会频繁切换颜色
2. **稳定性**: 2°C 死区提供足够的缓冲
3. **工业标准**: 滞后控制是温控系统的标准实践
4. **性能**: O(1) 复杂度，无额外开销

---

## 🔍 调试技巧

### 添加调试日志
```python
# 在 _update_temperature_style 中添加
print(f"温度: {temp:.1f}°C, 状态: {'高温' if self._is_high_temperature else '低温'}, 颜色: {color}")
```

### 监控温度话题
```bash
ros2 topic echo /usv_01/usv_status --field temperature
```

---

## 📚 相关文档

- 详细实现: `TEMPERATURE_HYSTERESIS_UPDATE.md`
- 温度显示: `TEMPERATURE_DISPLAY_UPDATE.md`
- 模块架构: `MODULE_ARCHITECTURE.md`

---

**最后更新**: 2025-01-22 | **状态**: ✅ 已实现并通过测试
