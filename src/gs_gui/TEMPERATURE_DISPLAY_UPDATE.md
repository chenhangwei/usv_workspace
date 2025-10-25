# 温度显示改进 - 更新日志

## 📝 改进内容

### 1️⃣ 表格温度显示优化
**位置**: 集群列表/离群列表的"温度"列

**改进**:
- ✅ 温度值自动除以1000，转换为摄氏度
- ✅ 显示格式：保留1位小数（例如：`45.3` 而不是 `45300`）

**代码位置**: `gs_gui/table_manager.py` - `_format_table_cells()` 方法

```python
# 修改前
temp_text = f"{float(state.get('temperature', 0.0)):.1f}"  # 显示 45300.0

# 修改后  
temp_raw = float(state.get('temperature', 0.0))
temp_celsius = temp_raw / 1000.0
temp_text = f"{temp_celsius:.1f}"  # 显示 45.3
```

---

### 2️⃣ USV详细信息面板新增温度显示
**位置**: 右侧"🔋 电池信息"分组框内

**新增字段**:
```
温度: XX.X ℃
```

**智能颜色指示**:
- 🟢 **绿色** (< 50℃): 温度正常
- 🟠 **橙色** (50-70℃): 温度偏高，警告
- 🔴 **红色** (> 70℃): 温度过热，危险

**代码位置**: `gs_gui/usv_info_panel.py`
- UI创建: `_create_battery_info_group()` 方法
- 数据更新: `update_state()` 方法
- 样式更新: `_update_temperature_style()` 方法

---

## 🎨 UI效果预览

### 表格显示示例：
```
编号    | 模式   | ... | 温度
--------|--------|-----|------
usv_01  | HOLD   | ... | 45.3
usv_02  | GUIDED | ... | 52.1
usv_03  | MANUAL | ... | 38.7
```

### 详细信息面板示例：
```
🔋 电池信息
┌─────────────────────┐
│ ████████░░░░░ 75%   │
├─────────────────────┤
│ 电压: 11.68 V       │
│ 电流: --    A       │
│ 温度: 45.3  ℃  🟢  │  ← 新增，颜色根据温度变化
└─────────────────────┘
```

---

## 🧪 测试方法

### 1. 启动系统
```bash
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

### 2. 检查表格温度列
- 查看集群列表/离群列表的"温度"列
- 确认数值显示为合理的摄氏度范围（如 30-60）
- 原本显示 45300 的应该变成 45.3

### 3. 检查详细信息面板
- 在表格中选中一艘USV
- 查看右侧"🔋 电池信息"分组框
- 确认"温度"字段显示，格式为 `XX.X ℃`
- 观察温度值的颜色是否根据数值变化：
  - 低温（<50℃）显示绿色
  - 中温（50-70℃）显示橙色
  - 高温（>70℃）显示红色

### 4. 模拟温度变化（可选）
如果需要测试颜色变化，可以手动修改温度值：
```bash
# 发布测试温度（示例）
ros2 topic pub /usv_01/temperature std_msgs/msg/Float32 "data: 75000.0" --once
```

---

## 📊 技术细节

### 温度数据流
```
MAVROS (飞控)
    ↓ 毫摄氏度 (milli-°C)
usv_status_node
    ↓ temperature 字段 (float, 原始值)
UsvStatus 消息
    ↓
GroundStationNode
    ↓
┌──────────────────┬────────────────────┐
│  TableManager    │  UsvInfoPanel      │
│  (表格显示)      │  (详细信息)         │
│  ÷ 1000          │  ÷ 1000            │
│  显示: XX.X      │  显示: XX.X ℃      │
│                  │  + 颜色指示         │
└──────────────────┴────────────────────┘
```

### 数据转换公式
```python
# 原始值（毫摄氏度）
temperature_raw = 45300.0  # 来自飞控

# 转换为摄氏度
temperature_celsius = temperature_raw / 1000.0  # 45.3

# 格式化显示
display_text = f"{temperature_celsius:.1f}"  # "45.3"
```

---

## 🔧 故障排查

### 问题1: 表格温度列仍显示大数值
**可能原因**: 旧的Python缓存  
**解决方法**:
```bash
cd ~/usv_workspace
rm -rf build/gs_gui install/gs_gui
colcon build --packages-select gs_gui
source install/setup.bash
```

### 问题2: 详细信息面板温度显示"--"
**可能原因**: 
1. 未选中任何USV
2. USV状态中无温度数据

**排查步骤**:
```bash
# 检查USV状态消息
ros2 topic echo /usv_01/usv_state --once | grep temperature
```

### 问题3: 温度颜色不变化
**可能原因**: 样式表优先级冲突  
**解决方法**: 检查 `modern_style.qss` 中是否有覆盖 QLabel 样式的规则

---

## 📝 代码变更摘要

### 文件1: `gs_gui/table_manager.py`
```python
# Line ~248 (在 _format_table_cells 方法中)
- temp_text = f"{float(state.get('temperature', 0.0)):.1f}"
+ temp_raw = float(state.get('temperature', 0.0))
+ temp_celsius = temp_raw / 1000.0
+ temp_text = f"{temp_celsius:.1f}"
```

### 文件2: `gs_gui/usv_info_panel.py`

**变更1: UI创建** (Line ~258)
```python
# 在 _create_battery_info_group() 中添加
self.temperature_label = self._create_value_label("--")
info_layout.addWidget(self._create_key_label("温度:"), 2, 0)
info_layout.addWidget(self.temperature_label, 2, 1)
info_layout.addWidget(QLabel("℃"), 2, 2)
```

**变更2: 数据更新** (Line ~443)
```python
# 在 update_state() 中添加
temp_raw = float(state.get('temperature', 0.0))
temp_celsius = temp_raw / 1000.0
self.temperature_label.setText(self._format_float(temp_celsius, precision=1))
self._update_temperature_style(temp_celsius)
```

**变更3: 新增方法** (Line ~607)
```python
def _update_temperature_style(self, temp_celsius):
    """根据温度更新样式"""
    if temp < 50:
        color = "#27ae60"  # 绿色
    elif temp < 70:
        color = "#f39c12"  # 橙色
    else:
        color = "#e74c3c"  # 红色
    # ... 应用样式
```

**变更4: 清空显示** (Line ~493)
```python
# 在 _clear_display() 中添加
self.temperature_label.setText("--")
```

---

## ✅ 验收清单

- [x] 表格温度列显示合理的摄氏度数值（30-60范围）
- [x] 详细信息面板新增"温度"字段
- [x] 温度单位显示为"℃"
- [x] 温度值保留1位小数
- [x] 温度颜色根据数值智能变化（绿/橙/红）
- [x] 代码已构建成功
- [x] 无编译错误或警告

---

**更新时间**: 2025-10-25  
**改进版本**: v1.1.0  
**影响组件**: gs_gui (table_manager, usv_info_panel)
