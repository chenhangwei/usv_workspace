# Boot Pose 迁移指南

## 快速总结

**Boot Pose 功能已在 2025-10-23 完全移除。**

### 变化概览

| 项目 | 之前 | 现在 |
|------|------|------|
| 操作流程 | 启动 USV → **手动标记 Boot Pose** → 执行任务 | 启动 USV → 执行任务 |
| 坐标转换 | Area → Global → **USV Local** | Area → Global（飞控自动处理） |
| 代码行数 | ~150 行 Boot Pose 代码 | 删除 |
| 菜单项 | 设置Boot Pose / 设置所有Boot Pose | 无 |
| ROS 信号 | `set_boot_pose`, `set_boot_pose_all` | 删除 |

## 代码变化

### 删除的功能

1. **GUI 菜单**：`_init_boot_pose_menu()` 及相关菜单项
2. **命令方法**：`set_boot_pose()`, `set_all_boot_pose_command()`
3. **ROS 信号**：`set_boot_pose`, `set_boot_pose_all`
4. **数据存储**：`usv_boot_pose` 字典
5. **回调函数**：`set_boot_pose_callback()`, `set_boot_pose_all_callback()`
6. **自动记录**：首次上线时记录 boot_pose 的逻辑

### 修改的功能

**`cluster_controller.py::_global_to_usv_local()`**

```python
# 之前（复杂）
def _global_to_usv_local(self, usv_id, p_global):
    bp = self.node.usv_boot_pose.get(usv_id)
    if bp:
        # 计算相对偏移和旋转
        ...
    return local_coords

# 现在（简化）
def _global_to_usv_local(self, usv_id, p_global):
    """直接返回全局坐标，飞控自动处理本地转换"""
    return p_global
```

## 使用指南

### 系统前提条件

✅ **必须满足**：飞控在上电时将当前位置设为本地原点 (0,0,0)  
✅ **支持的飞控**：ArduPilot, PX4（都支持此功能）

### 坐标系设置

现在只需要设置 **Area Center**（任务坐标系原点）：

1. 打开 GUI
2. 点击菜单：**Settings → Set Area Center Offset**
3. 输入 Area Center 在全局地图中的坐标（X, Y, Z）
4. 点击 **OK**

### 任务执行流程

```
1. 启动所有 USV（飞控自动设置上电位置为本地原点）
2. 打开地面站 GUI
3. 配置 Area Center 偏移量（如果需要）
4. 加载 XML 任务文件
5. 执行集群任务
```

## 坐标转换示例

### 场景
- XML 任务文件中有一个目标点：`(10, 5, 0)` (Area 坐标)
- Area Center 设置为：`(100, 50, 0)` (全局坐标)
- USV 在 `(105, 52, 0)` 位置上电

### 转换流程

```
Step 1: Area → Global
    (10, 5, 0) + (100, 50, 0) = (110, 55, 0)

Step 2: 发送给飞控
    地面站发送全局坐标 (110, 55, 0) 给 USV

Step 3: 飞控自动转换（USV内部处理）
    飞控将 (110, 55, 0) 转换为本地坐标
    本地原点是 (105, 52, 0)
    最终本地目标：(5, 3, 0)
```

**关键**：Step 3 现在由飞控自动完成，地面站不再参与！

## 测试验证

### 运行测试

```bash
cd ~/usv_workspace/src/gs_gui

# 测试坐标转换
python3 test/test_coordinate_transform.py

# 运行所有测试
python3 -m pytest test/ -v
```

### 端到端测试

1. **单 USV 导航**
   ```bash
   # 启动 USV
   ros2 launch usv_bringup usv_launch.py namespace:=usv_01
   
   # 启动地面站
   ros2 launch gs_bringup gs_launch.py
   
   # 发送目标点，验证 USV 能正确到达
   ```

2. **集群任务**
   - 加载示例任务：`resource/circle_patrol_and_stop.xml`
   - 添加多艘 USV 到集群
   - 执行任务，验证所有 USV 正确导航

## 常见问题

### Q1: USV 导航到错误的位置怎么办？

**检查项**：
1. 确认 Area Center 设置正确
2. 确认飞控支持全局坐标
3. 检查飞控参数（如 EKF 原点设置）

### Q2: 需要精确控制每艘 USV 的本地坐标系吗？

**不需要**！现在飞控自动处理。如果确实需要，可以：
- 在飞控参数中设置
- 或通过 MAVROS 发送 `SET_GPS_GLOBAL_ORIGIN` 消息

### Q3: 如何回滚到旧版本？

```bash
git log --oneline --all | grep -i "boot.*pose"
git checkout <commit-hash-before-removal>
```

## 文档资源

- ✅ **BOOT_POSE_REMOVAL_SUMMARY.md** - 完整删除总结
- ✅ **AREA_OFFSET_GUIDE.md** - Area Center 设置指南
- ❌ **BOOT_POSE_EXPLAINED.md** - 已过时（描述已删除功能）

## 受影响的文件列表

```
修改的文件 (7个):
  gs_gui/main_gui_app.py              - 删除菜单和命令
  gs_gui/ros_signal.py                - 删除信号
  gs_gui/usv_commands.py              - 删除命令方法
  gs_gui/ground_station_node.py       - 删除存储和回调
  gs_gui/cluster_controller.py        - 简化坐标转换
  gs_gui/usv_manager.py               - 删除自动记录
  test/test_coordinate_transform.py   - 更新测试逻辑

删除的代码量:
  约 150 行 Boot Pose 相关代码
```

---

**生效日期**: 2025-10-23  
**建议**: 如遇问题，请先检查飞控是否正确支持全局坐标系
