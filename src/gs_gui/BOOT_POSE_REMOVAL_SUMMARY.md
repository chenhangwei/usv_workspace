# Boot Pose 功能移除总结

## 修改日期
2025-10-23

## 修改原因
用户需求：删除 Boot Pose 手动标记功能，系统直接使用 USV 上电时的位置作为本地坐标系原点。

## 核心变化

### 原理变化

**之前（使用 Boot Pose）：**
```
1. USV 上电启动
2. 用户手动点击菜单标记 Boot Pose
3. 系统记录该位置为原点
4. 坐标转换：Global → USV Local（基于记录的 Boot Pose）
```

**现在（无 Boot Pose）：**
```
1. USV 上电启动
2. 飞控自动将上电位置设为本地原点 (0,0,0)
3. 坐标转换：Global → 直接发送给飞控
4. 飞控自己处理全局坐标到本地坐标的转换
```

## 修改文件清单

### 1. GUI 相关
- ✅ `gs_gui/main_gui_app.py`
  - 删除 `_init_boot_pose_menu()` 改为 `_init_custom_menu()`
  - 删除 Boot Pose 菜单项
  - 删除 `set_boot_pose_command()` 和 `set_all_boot_pose_command()` 方法
  - 删除 Boot Pose 信号连接

### 2. ROS 信号
- ✅ `gs_gui/ros_signal.py`
  - 删除 `set_boot_pose` 信号
  - 删除 `set_boot_pose_all` 信号

### 3. 命令处理器
- ✅ `gs_gui/usv_commands.py`
  - 删除 `set_boot_pose()` 方法
  - 删除 `set_boot_pose_all()` 方法

### 4. 地面站节点
- ✅ `gs_gui/ground_station_node.py`
  - 删除 `self.usv_boot_pose = {}` 初始化
  - 删除 `set_boot_pose_callback()` 方法
  - 删除 `set_boot_pose_all_callback()` 方法

### 5. 坐标转换逻辑
- ✅ `gs_gui/cluster_controller.py`
  - 简化 `_global_to_usv_local()` 方法
  - 现在直接返回全局坐标，由飞控处理本地转换

### 6. USV 管理器
- ✅ `gs_gui/usv_manager.py`
  - 删除首次上线时自动记录 boot_pose 的代码
  - 删除离线时清理 boot_pose 的代码

## 坐标系统简化

### 之前（三层转换）
```
Area 坐标 → Global 坐标 → USV Local 坐标
   ↓            ↓              ↓
相对坐标    全局地图坐标    飞控本地坐标
(XML文件)  (area_center)   (boot_pose)
```

### 现在（两层转换）
```
Area 坐标 → Global 坐标 → 直接发送给飞控
   ↓            ↓              ↓
相对坐标    全局地图坐标    飞控自动处理
(XML文件)  (area_center)
```

## 用户影响

### 操作流程变化

**之前的流程：**
1. 启动所有 USV
2. **打开 GUI，点击菜单标记 Boot Pose** ⬅️ 需要手动操作
3. 加载任务文件
4. 执行任务

**现在的流程：**
1. 启动所有 USV
2. 加载任务文件
3. 执行任务

### 优点
✅ 操作更简单，无需手动标记  
✅ 减少人为操作错误  
✅ 代码更简洁

### 注意事项
⚠️ **重要前提**：飞控必须在上电时将当前位置设为本地坐标原点 (0,0,0)  
⚠️ USV 重启后，本地坐标系会重置  
⚠️ 如果 USV 在不同位置重启，坐标系原点会改变

## 技术细节

### 坐标转换实现变化

**之前的 `_global_to_usv_local()` 方法：**
```python
def _global_to_usv_local(self, usv_id, p_global):
    # 获取 boot_pose
    bp = self.node.usv_boot_pose.get(usv_id)
    if bp:
        # 计算相对于 boot_pose 的偏移
        dx = p_global['x'] - bp['x']
        dy = p_global['y'] - bp['y']
        # 考虑航向旋转
        theta = -bp['yaw']
        local_x = cos(theta) * dx - sin(theta) * dy
        local_y = sin(theta) * dx + cos(theta) * dy
        return {'x': local_x, 'y': local_y, 'z': ...}
    else:
        return p_global
```

**现在的 `_global_to_usv_local()` 方法：**
```python
def _global_to_usv_local(self, usv_id, p_global):
    """
    直接返回全局坐标
    飞控会根据自己的上电位置自动转换为本地坐标
    """
    return p_global
```

## 兼容性说明

### MAVROS/飞控要求
- 飞控必须支持接收全局坐标
- 飞控必须能自动将全局坐标转换为本地坐标
- 大多数现代飞控（ArduPilot, PX4）都支持此功能

### 坐标系定义
- 全局坐标系：通常是 GPS 坐标或 ENU 本地坐标系
- 飞控本地坐标系：以上电位置为原点的坐标系
- Area 坐标系：任务文件中的相对坐标系（仍然保留）

## 测试建议

### 单 USV 测试
1. 启动 USV 在位置 A
2. 发送导航目标到全局坐标 B
3. 验证 USV 能正确到达 B 点

### 多 USV 集群测试
1. 在不同位置启动多艘 USV
2. 发送相同的全局坐标任务
3. 验证所有 USV 都到达相同的全局位置

### Area Center 测试
1. 设置 Area Center 偏移量
2. 加载 XML 任务文件
3. 验证坐标转换正确（Area → Global）

## 回滚方案

如果需要恢复 Boot Pose 功能，可以通过 Git 回退：

```bash
# 查看修改前的提交
git log --oneline

# 回退到删除 Boot Pose 之前的版本
git checkout <commit-hash-before-removal>

# 或者恢复特定文件
git checkout <commit-hash> -- gs_gui/gs_gui/ground_station_node.py
```

## 相关文档

以下文档已过时，建议存档：
- ❌ `BOOT_POSE_EXPLAINED.md` - Boot Pose 详细说明（已废弃）
- ✅ `AREA_OFFSET_GUIDE.md` - Area Center 偏移量说明（仍有效）

## 总结

Boot Pose 功能已完全移除，系统现在依赖飞控自身的本地坐标系处理。这简化了用户操作流程，但要求飞控必须正确处理全局坐标到本地坐标的转换。

---

**修改者**: AI Assistant  
**审核状态**: 待测试  
**版本**: v2.0-no-boot-pose
