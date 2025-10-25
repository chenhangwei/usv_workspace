# Ground Station GUI - 现代化UI设计指南

## 🎨 UI现代化改进方案

本文档说明如何在**不改变Qt Designer工作流程**的前提下，实现UI的现代化升级。

---

## ✅ 改进方案总结

### 核心原则
- **保持原有工作流程**：继续使用Qt Designer (.ui文件) → pyuic5 → ui.py
- **样式与逻辑分离**：通过QSS（Qt Style Sheets）注入现代样式
- **AI友好**：样式文件是纯CSS语法，AI可以轻松理解和修改

### 已实施的改进

1. **✅ 现代深色主题** (`modern_style.qss`)
   - 深色背景，高对比度文字
   - 圆角边框，扁平化设计
   - 悬停/点击动画效果
   - 专业的颜色配色方案

2. **✅ 样式管理器** (`style_manager.py`)
   - 自动加载QSS文件
   - 支持主题切换（future-proof）
   - 后备样式（加载失败时的保障）

3. **✅ 自适应打包配置**
   - 开发模式：从源码加载
   - 安装模式：从包资源加载

---

## 📁 文件结构

```
gs_gui/
├── gs_gui/
│   ├── main_gui_app.py          # 主窗口（已集成StyleManager）
│   ├── style_manager.py         # 样式管理器（NEW）
│   └── ...
├── resource/
│   ├── gs_ui.ui                 # Qt Designer UI文件（不变）
│   ├── modern_style.qss         # 现代深色主题（NEW）
│   ├── triangle.xml             # 任务文件
│   └── ...
└── setup.py                     # 打包配置（已更新）
```

---

## 🚀 使用方法

### 1. 日常开发（无需改变）

继续使用原有流程：

```bash
# 1. 用Qt Designer编辑 gs_ui.ui
designer resource/gs_ui.ui

# 2. 转换为Python代码（自动覆盖ui.py）
pyuic5 resource/gs_ui.ui -o gs_gui/ui.py

# 3. 构建并运行（样式自动加载）
colcon build --packages-select gs_gui
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

**关键点**：样式是**运行时注入**的，修改UI文件不会破坏样式！

---

### 2. 定制样式

#### 方案A：修改现有QSS文件（推荐）

直接编辑 `resource/modern_style.qss`：

```css
/* 示例：修改按钮颜色 */
QPushButton {
    background-color: #ff5722;  /* 改为橙色 */
    color: #ffffff;
}

QPushButton:hover {
    background-color: #ff7043;
}
```

**修改后**：
```bash
# 无需重新转换UI文件，直接构建即可
colcon build --packages-select gs_gui
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

---

#### 方案B：创建新主题

1. 复制现有QSS文件：
```bash
cd src/gs_gui/resource
cp modern_style.qss light_theme.qss
```

2. 编辑 `light_theme.qss`（改为浅色主题）：
```css
QMainWindow { background-color: #f5f5f5; }
QWidget { color: #212121; background-color: #f5f5f5; }
/* ... 其他浅色样式 */
```

3. 注册新主题（编辑 `style_manager.py`）：
```python
THEMES = {
    'modern_dark': 'modern_style.qss',
    'light': 'light_theme.qss',  # 新增
    'classic': None,
}
```

4. 切换主题（在 `main_gui_app.py` 的 `__init__` 中）：
```python
self.style_manager.load_theme('light')  # 改为浅色
```

---

### 3. 针对特定控件定制

#### 示例1：为特定按钮设置颜色

在Qt Designer中给按钮设置 **对象名称**（Object Name）：

- `add_cluster_pushButton` → 绿色（成功操作）
- `quit_cluster_pushButton` → 红色（危险操作）

然后在QSS中使用ID选择器：

```css
/* modern_style.qss 中已包含 */
QPushButton#add_cluster_pushButton {
    background-color: #388e3c;  /* 绿色 */
}

QPushButton#quit_cluster_pushButton {
    background-color: #d32f2f;  /* 红色 */
}
```

---

#### 示例2：为信息面板设置样式

```css
/* 在 modern_style.qss 中添加 */
UsvInfoPanel {
    background-color: #252525;
    border: 2px solid #4fc3f7;
    border-radius: 10px;
}

UsvInfoPanel QLabel {
    color: #e0e0e0;
    font-size: 11pt;
}

UsvInfoPanel QProgressBar {
    border: 1px solid #4fc3f7;
    background-color: #2b2b2b;
}
```

---

## 🎯 AI辅助修改样式

### 场景1：让按钮更醒目

**指令给AI**：
```
请修改 modern_style.qss 中的 QPushButton 样式，使其：
1. 增加阴影效果
2. 悬停时有缩放动画
3. 使用渐变背景
```

AI会生成类似代码：
```css
QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                stop:0 #4fc3f7, stop:1 #0288d1);
    border: none;
    border-radius: 8px;
    padding: 10px 20px;
    box-shadow: 0 4px 6px rgba(0,0,0,0.3);
}
```

---

### 场景2：适配大屏幕

**指令给AI**：
```
请修改 modern_style.qss，让字体在4K显示器上更清晰（增大字号和行距）
```

AI会修改：
```css
* {
    font-size: 12pt;  /* 从10pt增大 */
    line-height: 1.5;
}

QGroupBox {
    padding-top: 15px;  /* 增加内边距 */
}
```

---

## 🛠️ 调试技巧

### 实时预览样式变化

不重启程序即可看到样式效果：

```python
# 在Python控制台或调试器中执行
self.style_manager.load_theme('modern_dark')  # 重新加载
```

---

### 检查控件对象名称

在Qt Designer中：
1. 选中控件
2. 查看 **属性编辑器** → `objectName`
3. 在QSS中使用 `#对象名称` 定制

---

### 查看当前应用的样式

```python
# 在Python控制台中
print(self.styleSheet())
```

---

## 📊 颜色配色方案参考

当前主题（Modern Dark）使用的核心颜色：

| 用途 | 颜色代码 | 说明 |
|------|---------|------|
| 主背景 | `#1e1e1e` | 深灰黑 |
| 次背景 | `#252525` | 稍浅灰 |
| 边框 | `#3a3a3a` | 灰色边框 |
| 主文字 | `#e0e0e0` | 浅灰白 |
| 强调色 | `#4fc3f7` | 青蓝色（主题色） |
| 选中背景 | `#1976d2` | 深蓝 |
| 成功/确认 | `#4caf50` | 绿色 |
| 危险/取消 | `#f44336` | 红色 |

**配色工具推荐**：
- [Material Design Colors](https://materialui.co/colors)
- [Coolors](https://coolors.co/)

---

## 🔄 回退到原始样式

如果不喜欢新样式，随时可以禁用：

```python
# 在 main_gui_app.py 的 __init__ 中注释掉：
# self.style_manager = StyleManager(self)
# self.style_manager.load_theme('modern_dark')
```

或者切换到经典主题：
```python
self.style_manager.load_theme('classic')  # 无样式表
```

---

## 📝 总结

### 优势
✅ **不破坏现有工作流程**：Qt Designer → pyuic5 照常使用  
✅ **AI友好**：QSS是CSS语法，AI可以轻松理解和生成  
✅ **灵活定制**：修改QSS文件即可，无需重新设计UI  
✅ **主题切换**：支持多主题，一键切换  
✅ **向后兼容**：可随时禁用回到原样式  

### 注意事项
⚠️ **不要手动编辑 ui.py**：该文件会被pyuic5覆盖  
⚠️ **样式优先级**：内联样式 > QSS文件 > 默认样式  
⚠️ **性能**：复杂样式可能影响渲染性能（当前配置已优化）  

---

## 🎓 进阶学习资源

- [Qt Style Sheets 官方文档](https://doc.qt.io/qt-5/stylesheet.html)
- [Qt Style Sheets 示例](https://doc.qt.io/qt-5/stylesheet-examples.html)
- [PyQt5 样式表教程](https://www.pythonguis.com/tutorials/pyqt-qss-styling/)

---

**最后更新**: 2025-10-25  
**适用版本**: ROS 2 Jazzy, PyQt5 5.15+
