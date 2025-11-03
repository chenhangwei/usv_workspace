# USV 信息面板 - 使用和集成指南

## 📋 概述

`usv_info_panel.py` 提供了一个全新的、美观的 USV 详细信息显示组件，用于替换原有的简单标签显示。

### 主要特性

✨ **美观的UI设计**
- 使用彩色分组框，不同信息类型有不同的颜色主题
- 使用图标（emoji）增强视觉识别
- 圆角边框和阴影效果
- 根据状态自动改变颜色（如电池低电量变红色）

📊 **丰富的信息展示**
- **基本信息**：USV ID、模式、状态、解锁状态
- **位置信息**：X、Y、Z 坐标和 Yaw 角度
- **电池信息**：电压、电流、百分比（带进度条）
- **GPS 信息**：卫星数量、精度
- **速度信息**：地速、航向

🎨 **智能样式更新**
- 模式颜色：GUIDED（绿）、MANUAL（橙）、AUTO（蓝）
- 状态颜色：ACTIVE（绿）、STANDBY（蓝）、CRITICAL（红）
- 解锁状态：已解锁（红）、已锁定（绿）
- 电池进度条：>60%（绿）、30-60%（橙）、<30%（红）
- GPS卫星数：>=10（绿）、6-9（橙）、<6（红）

---

## 🚀 快速集成（方案1：替换原有显示）

### 步骤 1：修改 UI 文件

编辑 `/home/chenhangwei/usv_workspace/src/gs_gui/resource/gs_ui.ui`

找到 `<widget class="QGroupBox" name="groupBox_3">` 部分（usv_info），将其**完全替换**为：

```xml
<widget class="QGroupBox" name="groupBox_3">
 <property name="title">
  <string>usv_info</string>
 </property>
 <layout class="QVBoxLayout" name="verticalLayout_usv_info">
  <item>
   <!-- 这里将动态插入 UsvInfoPanel -->
  </item>
 </layout>
</widget>
```

### 步骤 2：重新生成 Python UI 文件

在 `gs_gui/resource/` 目录下运行：

```bash
cd /home/chenhangwei/usv_workspace/src/gs_gui/resource
pyuic5 gs_ui.ui -o ../gs_gui/ui.py
```

### 步骤 3：修改 MainWindow 初始化

编辑 `/home/chenhangwei/usv_workspace/src/gs_gui/gs_gui/main_gui_app.py`

在导入部分添加：

```python
from gs_gui.usv_info_panel import UsvInfoPanel
```

在 `__init__` 方法中添加（在 `self.ui.setupUi(self)` 之后）：

```python
# 创建并嵌入 USV 信息面板
self.usv_info_panel = UsvInfoPanel()
# 将面板添加到 groupBox_3 的布局中
self.ui.groupBox_3.layout().addWidget(self.usv_info_panel)
```

### 步骤 4：修改 ui_utils.py 中的更新逻辑

编辑 `/home/chenhangwei/usv_workspace/src/gs_gui/gs_gui/ui_utils.py`

修改 `update_selected_table_row` 方法：

```python
def update_selected_table_row(self, table_manager, state_handler, usv_info_panel):
    """
    更新选中行数据
    
    Args:
        table_manager: 表格管理器
        state_handler: 状态处理器
        usv_info_panel: USV 信息面板实例
    """
    try:
        # 获取选中的行
        selected_indexes = self.ui.cluster_tableView.selectedIndexes()
        if not selected_indexes:
            # 没有选中时清空显示
            usv_info_panel.update_state(None)
            return
        
        selected_row = selected_indexes[0].row()
        model = self.ui.cluster_tableView.model()
        if model is None:
            usv_info_panel.update_state(None)
            return
        
        index0 = model.index(selected_row, 0)
        namespace = model.data(index0) if index0.isValid() else None
        if not namespace:
            usv_info_panel.update_state(None)
            return
        
        # 获取最新状态
        state = state_handler.get_usv_state(namespace)
        
        # 使用新的信息面板更新显示
        usv_info_panel.update_state(state)
    
    except Exception as e:
        try:
            self.append_info(f"错误：获取选中行数据失败 - {str(e)}")
        except Exception:
            pass
        usv_info_panel.update_state(None)
```

### 步骤 5：修改 state_handler.py 中的刷新逻辑

编辑 `/home/chenhangwei/usv_workspace/src/gs_gui/gs_gui/state_handler.py`

在 `StateHandler` 类的 `__init__` 方法中添加 `usv_info_panel` 参数：

```python
def __init__(self, table_manager, list_manager, append_warning, usv_info_panel=None):
    # ... 现有代码 ...
    self.usv_info_panel = usv_info_panel
```

在 `_refresh_table` 方法中调用面板更新（在刷新表格之后）：

```python
def _refresh_table(self):
    """定时刷新表格显示"""
    try:
        # ... 现有刷新逻辑 ...
        
        # 如果有选中的 USV，更新详细信息面板
        if self.usv_info_panel:
            # 获取选中的 namespace
            selected_ns = self._get_selected_namespace()
            if selected_ns:
                state = self.usv_states.get(selected_ns)
                self.usv_info_panel.update_state(state)
    except Exception as e:
        # ...
```

添加辅助方法：

```python
def _get_selected_namespace(self):
    """获取当前选中的 USV namespace"""
    try:
        # 从 table_manager 获取选中的行
        # 这里需要根据你的实际代码调整
        return None  # 暂时返回 None，需要具体实现
    except Exception:
        return None
```

### 步骤 6：更新 MainWindow 中的连接

在 `main_gui_app.py` 的 `__init__` 中，将 `usv_info_panel` 传递给 `StateHandler`：

```python
# 初始化状态处理器（在创建 usv_info_panel 之后）
self.state_handler = StateHandler(
    self.table_manager,
    self.list_manager,
    self.ui_utils.append_warning,
    self.usv_info_panel  # 添加这个参数
)
```

---

## 🎯 快速集成（方案2：独立窗口显示）

如果你希望将 USV 详细信息显示在独立的弹出窗口中（类似 `UsvPlotWindow`），可以采用以下方式：

### 步骤 1：创建包装对话框

在 `gs_gui/gs_gui/` 中创建 `usv_info_dialog.py`：

```python
from PyQt5.QtWidgets import QDialog, QVBoxLayout
from PyQt5.QtCore import Qt
from gs_gui.usv_info_panel import UsvInfoPanel


class UsvInfoDialog(QDialog):
    """USV 信息对话框"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("USV 详细信息")
        self.setMinimumSize(400, 700)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.info_panel = UsvInfoPanel()
        layout.addWidget(self.info_panel)
        
        self.setWindowFlags(
            Qt.Window | 
            Qt.WindowTitleHint | 
            Qt.WindowCloseButtonHint |
            Qt.WindowStaysOnTopHint
        )
    
    def update_state(self, state):
        """更新 USV 状态"""
        self.info_panel.update_state(state)
```

### 步骤 2：在 UI 中添加按钮

在主窗口添加一个"显示详细信息"按钮，连接到：

```python
def show_usv_detail_info(self):
    """显示选中 USV 的详细信息"""
    # 获取选中的 USV
    usv_info = self.table_manager.get_selected_usv_info()
    if not usv_info:
        self.ui_utils.append_warning("请先选中一个 USV")
        return
    
    namespace = usv_info['namespace']
    state = self.state_handler.get_usv_state(namespace)
    
    # 创建或显示对话框
    if not hasattr(self, 'usv_info_dialog'):
        self.usv_info_dialog = UsvInfoDialog(self)
    
    self.usv_info_dialog.update_state(state)
    self.usv_info_dialog.show()
    self.usv_info_dialog.raise_()
    self.usv_info_dialog.activateWindow()
```

---

## 📊 状态数据格式

新面板期望的状态数据格式（`state` 字典）：

```python
{
    # 基本信息
    'namespace': 'usv_01',          # USV ID
    'mode': 'GUIDED',                # 飞行模式
    'connected': True,               # 是否在线
    'system_status': 'ACTIVE',       # 飞控系统状态（可选）
    'armed': True,                   # 是否解锁
    
    # 位置信息
    'position': {
        'x': 10.5,                   # X 坐标 (m)
        'y': -5.2,                   # Y 坐标 (m)
        'z': 0.3                     # Z 坐标 (m)
    },
    'yaw': 0.8,                      # 航向角 (弧度)
    
    # 电池信息
    'battery_percentage': 75.0,      # 电池百分比 (%)
    'battery_voltage': 12.6,         # 电压 (V)
    'battery_current': 2.3,          # 电流 (A)
    'temperature': 42.5,             # 机载温度 (°C，可选)
    
    # GPS 信息
    'gps_satellites_visible': 12,    # 卫星数量
    'gps_eph': 0.8,                  # 水平精度 HDOP (m)
    
    # 速度信息
    'velocity': {
        'linear': {'x': 1.1, 'y': 1.0}
    },                               # 线速度向量，用于计算地速

    # 预检与传感器状态
    'prearm_ready': True,            # 预检是否通过
    'prearm_warnings': [],           # 预检警告列表
    'sensor_status': [               # 关键传感器健康度列表
        {'name': 'GPS Fix', 'status': '3D Fix', 'detail': '12 sats', 'level': 'ok'}
    ],
    'vehicle_messages': [            # 最近的飞控消息（Optional）
        {'severity': 6, 'severity_label': 'INFO', 'text': 'Mission uploaded', 'time': '12:01', 'timestamp': 0.0}
    ]
}
```

### 与现有数据的兼容性

如果 `UsvStatus` 消息中缺少部分字段（如 `battery_current`、`gps_eph`、`sensor_status` 等），面板会自动显示 `--` 或占位提示，不会报错。

---

## 🎨 自定义样式

### 修改颜色主题

编辑 `usv_info_panel.py` 中的各个 `_create_*_group` 方法，修改 `border` 颜色：

```python
# 当前颜色主题
基本信息: #3498db (蓝色)
位置信息: #27ae60 (绿色)
电池信息: #f39c12 (橙色)
GPS信息:  #9b59b6 (紫色)
速度信息: #e74c3c (红色)
```

### 修改字体大小

在 `_create_key_label` 和 `_create_value_label` 方法中修改 `font-size`。

### 添加新的信息字段

1. 在对应的 `_create_*_group` 方法中添加新的标签
2. 在 `update_state` 方法中添加更新逻辑
3. 在 `_clear_display` 方法中添加清空逻辑

---

## 🧪 测试

### 单元测试示例

创建 `test_usv_info_panel.py`：

```python
import sys
from PyQt5.QtWidgets import QApplication
from gs_gui.usv_info_panel import UsvInfoPanel


def test_usv_info_panel():
    """测试 USV 信息面板"""
    app = QApplication(sys.argv)
    
    panel = UsvInfoPanel()
    panel.setMinimumSize(400, 700)
    panel.show()
    
    # 模拟状态数据
    test_state = {
        'namespace': 'usv_01',
        'mode': 'GUIDED',
        'connected': True,
        'system_status': 'ACTIVE',
        'armed': True,
        'position': {'x': 10.5, 'y': -5.2, 'z': 0.3},
        'yaw': 0.8,
        'battery_percentage': 75.0,
        'battery_voltage': 12.6,
        'battery_current': 2.3,
        'temperature': 42.5,
        'gps_satellites_visible': 12,
        'gps_eph': 0.8,
        'velocity': {'linear': {'x': 1.1, 'y': 1.0}},
        'prearm_ready': True,
        'prearm_warnings': [],
        'sensor_status': [],
        'vehicle_messages': []
    }
    
    panel.update_state(test_state)
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    test_usv_info_panel()
```

运行测试：

```bash
cd /home/chenhangwei/usv_workspace/src/gs_gui
python3 -m pytest test/test_usv_info_panel.py -v
```

---

## 🔧 故障排查

### 问题1：面板显示空白

**原因**：可能是 Qt Designer 生成的布局有问题

**解决**：手动在代码中添加面板：

```python
# 清除原有内容
while self.ui.groupBox_3.layout().count():
    item = self.ui.groupBox_3.layout().takeAt(0)
    if item.widget():
        item.widget().deleteLater()

# 添加新面板
self.ui.groupBox_3.layout().addWidget(self.usv_info_panel)
```

### 问题2：状态不更新

**原因**：可能是没有正确连接信号

**解决**：在 `state_handler.py` 的刷新逻辑中添加调试信息：

```python
def _refresh_table(self):
    # ...
    print(f"[DEBUG] 刷新面板，当前选中: {selected_ns}")
    if self.usv_info_panel and selected_ns:
        state = self.usv_states.get(selected_ns)
        print(f"[DEBUG] 状态数据: {state}")
        self.usv_info_panel.update_state(state)
```

### 问题3：某些字段显示 "--"

**原因**：状态数据中缺少对应字段

**解决**：检查 `usv_status_node.py` 是否填充了所有字段，或者修改面板代码使用默认值。

---

## 📚 相关文件

- **核心文件**：`gs_gui/usv_info_panel.py`
- **UI 文件**：`gs_gui/resource/gs_ui.ui`
- **主窗口**：`gs_gui/main_gui_app.py`
- **UI 工具**：`gs_gui/ui_utils.py`
- **状态处理**：`gs_gui/state_handler.py`
- **数据来源**：`usv_comm/usv_status_node.py`

---

## 🎓 扩展建议

### 1. 添加历史数据图表

集成 `matplotlib` 显示电池、速度等历史曲线。

### 2. 添加报警功能

当电池低于 20%、GPS 信号差时弹出提示。

### 3. 添加导出功能

导出当前 USV 状态为 JSON 或 CSV 文件。

### 4. 添加比较功能

同时显示多个 USV 的信息进行对比。

---

## 🆕 版本历史

- **v1.0** (2025-01): 初始版本，支持基本信息、位置、电池、GPS、速度显示

---

**完成集成后，记得重新构建项目：**

```bash
cd /home/chenhangwei/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash
```
