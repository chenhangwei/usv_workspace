# USV 集群启动器 - 非模态窗口改进

## 修改日期
2025-11-07

## 问题描述
USV 集群启动器打开后会锁住主界面，无法同时操作地面站 GUI 和集群启动器。

## 解决方案

### 1. 从模态改为非模态

**修改前（模态对话框）：**
```python
launcher = UsvFleetLauncher(self, workspace_path)
launcher.exec_()  # 阻塞主窗口，必须关闭对话框才能操作主界面
```

**修改后（非模态对话框）：**
```python
# 创建并显示非模态窗口
self._usv_fleet_launcher = UsvFleetLauncher(self, workspace_path)
self._usv_fleet_launcher.show()  # 不阻塞主窗口
```

### 2. 防止重复打开

添加窗口实例检查，避免重复打开多个集群启动器：

```python
if hasattr(self, '_usv_fleet_launcher') and self._usv_fleet_launcher is not None:
    # 窗口已存在，激活并置顶
    self._usv_fleet_launcher.raise_()
    self._usv_fleet_launcher.activateWindow()
else:
    # 创建新窗口
    self._usv_fleet_launcher = UsvFleetLauncher(self, workspace_path)
    self._usv_fleet_launcher.show()
```

### 3. 关闭时清理引用

在集群启动器关闭时，清理父窗口中的引用：

```python
def closeEvent(self, event):
    # 停止定时器
    self.status_timer.stop()
    
    # 通知父窗口清理引用
    if self.parent():
        if hasattr(self.parent(), '_usv_fleet_launcher'):
            self.parent()._usv_fleet_launcher = None
    
    event.accept()
```

## 修改文件

1. **main_gui_app.py** - 主窗口
   - 修改 `launch_usv_fleet_action()` 方法
   - 使用 `show()` 替代 `exec_()`
   - 添加实例管理和窗口激活逻辑

2. **usv_fleet_launcher.py** - 集群启动器
   - 修改 `closeEvent()` 方法
   - 添加引用清理逻辑

## 使用效果

### 修改前 ❌
1. 打开 USV 集群启动器
2. 主界面被锁定，无法操作
3. 必须关闭集群启动器才能继续使用主界面

### 修改后 ✅
1. 打开 USV 集群启动器
2. **主界面仍可正常操作**
3. 可以同时：
   - 在主界面控制单个 USV
   - 在集群启动器批量管理 USV
   - 切换窗口查看不同信息

### 窗口管理
- **重复打开**：如果集群启动器已打开，再次点击菜单会激活现有窗口而不是创建新窗口
- **多窗口**：可以同时打开多个非模态窗口（集群启动器、参数配置等）
- **独立操作**：各窗口互不干扰

## 技术说明

### QDialog 显示模式对比

| 特性 | exec_()（模态） | show()（非模态） |
|------|----------------|-----------------|
| 阻塞主窗口 | ✅ 是 | ❌ 否 |
| 返回值 | 有（Accepted/Rejected） | 无 |
| 使用场景 | 必须等待用户响应的对话框 | 辅助工具窗口 |
| 示例 | 确认对话框、登录框 | 属性编辑器、工具面板 |

### Qt 窗口管理方法

```python
# 激活窗口并置顶
window.raise_()           # 将窗口置于其他窗口之上
window.activateWindow()   # 激活窗口（获得焦点）

# 检查窗口是否存在
if window is not None:
    # 窗口对象存在
    
if window.isVisible():
    # 窗口可见
```

## 其他非模态窗口

以下其他功能也适合改为非模态窗口：

1. ✅ **USV 集群启动器**（已修改）
2. 🔄 **参数配置器**（param_manager_dialog.py）- 建议修改
3. 🔄 **坐标系设置对话框**（area_offset_command）- 建议修改
4. ❌ **确认对话框**（QMessageBox）- 应保持模态

## 测试步骤

1. 重启地面站 GUI
2. 打开菜单 → USV 集群启动器
3. 尝试在主界面上操作（如发送命令、查看状态表格）
4. 确认主界面不被锁定 ✅
5. 再次点击菜单打开集群启动器
6. 确认激活现有窗口而非创建新窗口 ✅

## 相关文档

- **状态检测优化**：`gs_gui/USV_FLEET_STATUS_DETECTION.md`
- **集群启动器架构**：`gs_gui/USV_FLEET_LAUNCHER_V2.md`
- **模块架构**：`gs_gui/MODULE_ARCHITECTURE.md`

## 总结

通过将 USV 集群启动器从模态改为非模态窗口，用户现在可以：

1. ✅ 同时操作主界面和集群启动器
2. ✅ 在不关闭集群启动器的情况下控制单个 USV
3. ✅ 灵活切换窗口查看不同信息
4. ✅ 提升多任务操作效率

这大大提高了用户体验和操作灵活性！
