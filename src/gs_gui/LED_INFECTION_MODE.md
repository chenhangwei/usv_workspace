# LED传染模式功能说明

## 功能概述

LED传染模式是一个可以通过菜单开关控制的功能，允许用户启用或禁用USV之间的LED颜色传染行为。

## 功能特性

### 1. 菜单开关
- **位置**: 主窗口菜单栏 → "LED设置" → "LED传染模式"
- **类型**: 可勾选菜单项（Checkable Action）
- **默认状态**: 开启（勾选）

### 2. 传染模式行为

#### 开启状态（默认）
- USV之间距离小于2米时触发传染
- 被传染的USV的LED会**实时跟随**传染源USV的LED颜色变化
- 传染源由USV ID排序决定（ID靠前的为传染源）
- 保留被传染USV的原始LED状态，以便恢复

#### 关闭状态
- 停止检测USV之间的传染逻辑
- 自动恢复所有被传染USV的原始LED状态
- 清空所有传染相关的状态记录

### 3. 实时跟随机制

传染模式的核心特性是**实时跟随**：
- 当USV A传染USV B后，USV B的LED会持续显示USV A的当前颜色
- 如果USV A改变LED颜色（例如通过GUI控制），USV B的LED会自动更新
- 这种跟随是动态的，每2秒检查一次并更新
- 只要两个USV保持在传染距离内（2米以内），这种跟随就会持续

## 实现细节

### 1. 代码修改

#### ROSSignal (`ros_signal.py`)
```python
# 添加LED传染模式控制信号
led_infection_mode_changed = pyqtSignal(bool)  # 参数：True开启/False关闭
```

#### MainWindow (`main_gui_app.py`)
- 在 `_init_custom_menu()` 中添加"LED设置"菜单
- 添加可勾选的"LED传染模式"菜单项
- 实现 `toggle_led_infection_mode()` 方法处理开关切换
- 在 `main()` 函数中连接信号到节点回调

#### GroundStationNode (`ground_station_node.py`)
- 添加 `_led_infection_enabled` 标志（默认True）
- 实现 `set_led_infection_mode_callback()` 处理开关变化
- 修改 `check_usv_infect()` 方法，根据开关状态执行传染逻辑
- 关闭时自动恢复所有被传染USV的原始LED状态

#### LedInfectionHandler (`led_infection.py`)
- 完善文档说明传染模式的实时跟随特性
- 每次检查时都会更新被传染USV的LED颜色

### 2. 工作流程

```
用户点击菜单开关
    ↓
MainWindow.toggle_led_infection_mode()
    ↓
发送 ros_signal.led_infection_mode_changed 信号
    ↓
GroundStationNode.set_led_infection_mode_callback()
    ↓
更新 _led_infection_enabled 标志
    ↓
check_usv_infect() 定时器根据标志决定是否执行传染逻辑
```

### 3. 传染源选择规则

- 按USV ID字符串排序（字典序）
- ID靠前的USV作为传染源（例如 usv_01 传染 usv_02）
- 这确保了传染关系的一致性和可预测性

### 4. 状态管理

**本地LED状态维护** (`_usv_current_led_state`):
```python
{
    'usv_01': {'mode': 'color_select', 'color': [255, 0, 0]},
    'usv_02': {'mode': 'color_switching', 'color': [0, 255, 0]},
    ...
}
```

**原始LED状态记录** (`_usv_led_modes`):
```python
{
    'usv_02': ('color_select', [0, 255, 0]),  # 被传染前的状态
    ...
}
```

## 使用示例

### 启用传染模式（默认）
1. 启动地面站GUI
2. 传染模式默认开启（菜单项已勾选）
3. 当两个USV距离小于2米时，自动触发传染
4. 被传染的USV LED会实时跟随传染源颜色变化

### 禁用传染模式
1. 点击菜单栏"LED设置" → 取消勾选"LED传染模式"
2. 系统自动恢复所有被传染USV的原始LED状态
3. 停止传染检测

### 重新启用
1. 再次勾选"LED传染模式"
2. 系统重新开始检测并执行传染逻辑
3. 被传染的USV会根据当前距离和传染源状态更新LED

## 技术参数

- **传染距离**: 2米（INFECTION_DISTANCE_SQUARED = 4.0）
- **检查周期**: 2秒（INFECTION_CHECK_PERIOD = 2.0）
- **传染源判定**: USV ID字典序
- **状态恢复**: 自动（关闭模式时）

## 调试信息

日志级别为INFO时，可以看到：
- LED传染模式开启/关闭消息
- 恢复被传染USV状态的详细信息
- 传染命令发送记录

## 注意事项

1. **默认开启**: 传染模式默认是开启的，如不需要请手动关闭
2. **实时跟随**: 被传染USV会持续跟随传染源的颜色变化，直到离开传染范围
3. **自动恢复**: 关闭传染模式时，系统会自动恢复所有被传染USV的原始状态
4. **状态持久化**: 传染状态不会持久化，重启GUI后需要重新触发传染

## 相关文件

- `gs_gui/gs_gui/ros_signal.py` - 信号定义
- `gs_gui/gs_gui/main_gui_app.py` - GUI菜单和控制
- `gs_gui/gs_gui/ground_station_node.py` - 节点逻辑和状态管理
- `gs_gui/gs_gui/led_infection.py` - 传染逻辑实现

---

**最后更新**: 2025-10-24 | **功能版本**: v1.0
