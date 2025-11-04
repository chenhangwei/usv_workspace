# 参数管理按钮点击无效问题修复

## 问题描述

点击"⚙️ 飞控参数配置"按钮没有反应。

## 问题原因

代码中存在变量名不一致的问题：

- **实际变量名**: `self.usv_info_panel` (在 `_init_usv_info_panel()` 中定义)
- **引用的变量名**: `self.info_panel` (在信号连接和其他地方使用)

导致信号连接失败，按钮点击事件没有被正确绑定。

## 修复内容

修改了 `main_gui_app.py` 中所有 `self.info_panel` 引用为 `self.usv_info_panel`：

### 1. 信号连接（第 135-140 行）
```python
# 修复前
if hasattr(self, 'info_panel') and hasattr(self.info_panel, 'param_button'):
    self.info_panel.param_button.clicked.connect(self.on_param_config_clicked)

# 修复后
if hasattr(self, 'usv_info_panel') and hasattr(self.usv_info_panel, 'param_button'):
    self.usv_info_panel.param_button.clicked.connect(self.on_param_config_clicked)
```

### 2. 重启按钮禁用（第 536-538 行）
```python
# 修复前
if hasattr(self, 'info_panel') and hasattr(self.info_panel, 'reboot_button'):
    self.info_panel.reboot_button.setEnabled(False)
    self.info_panel.reboot_button.setText("⏳ 重启中…")

# 修复后
if hasattr(self, 'usv_info_panel') and hasattr(self.usv_info_panel, 'reboot_button'):
    self.usv_info_panel.reboot_button.setEnabled(False)
    self.usv_info_panel.reboot_button.setText("⏳ 重启中…")
```

### 3. 重启按钮恢复（第 625-627 行）
```python
# 修复前
if hasattr(self, 'info_panel') and hasattr(self.info_panel, 'reboot_button'):
    self.info_panel.reboot_button.setEnabled(True)
    self.info_panel.reboot_button.setText("🔄 重启飞控")

# 修复后
if hasattr(self, 'usv_info_panel') and hasattr(self.usv_info_panel, 'reboot_button'):
    self.usv_info_panel.reboot_button.setEnabled(True)
    self.usv_info_panel.reboot_button.setText("🔄 重启飞控")
```

## 验证方法

### 1. 重新构建
```bash
cd ~/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash
```

### 2. 启动地面站
```bash
ros2 launch gs_bringup gs_launch.py
```

### 3. 测试步骤
1. 选择一个在线的 USV
2. 查看右侧详细面板
3. 点击"⚙️ 飞控参数配置"按钮
4. 应该弹出参数配置窗口

## 状态

✅ **已修复** - 2025-11-04

---

**修复文件**: `gs_gui/gs_gui/main_gui_app.py`  
**修改行数**: 4 处（共 12 行代码）
