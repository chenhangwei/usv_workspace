# Guided模式导航功能测试指南

## 修复内容总结

### 问题
1. **重复发送**：地面站每5秒重复发送相同的导航目标
2. **无反馈**：手动移动到目标点后仍无"到达"反馈
3. **通信不兼容**：地面站使用话题通信，USV端启动的是Action服务器

### 修复
1. ✅ 修复地面站重复发送逻辑（`cluster_controller.py`）
2. ✅ 切换USV端到话题版本导航节点（`usv_launch.py`）
3. ✅ 增强导航结果调试日志（`ground_station_node.py`）

## 测试步骤

### 1. 重新编译

```bash
cd ~/usv_workspace
colcon build --packages-select gs_gui usv_comm usv_bringup
source install/setup.bash
```

### 2. 重启系统

```bash
# 停止所有运行的节点
# 然后重新启动
```

### 3. 基础功能测试

#### 测试1: 单个USV导航

**步骤**:
1. 在GUI中选择一个USV
2. 发送导航目标点
3. 切换到GUIDED模式
4. 观察日志输出

**预期结果**:
```
[main_gui_app] [INFO] 📤 Step 1 → usv_01: Local(1.40, 6.10, 0.00) [首次发送]
[navigate_to_point_node] [INFO] 📥 收到新目标 [ID=1]: (1.40, 6.10, 0.00)
[navigate_to_point_node] [INFO] 导航中 [ID=1]: 距离=5.20m, 航向误差=12.3°
```

✅ **验证点**:
- [ ] 目标点只发送1次（看到 "[首次发送]"）
- [ ] USV端收到目标
- [ ] 定期看到导航反馈

#### 测试2: 手动到达目标点

**步骤**:
1. 发送导航目标后
2. 手动移动USV到目标点附近（< 1m）
3. 观察日志和GUI状态

**预期结果**:
```
[navigate_to_point_node] [INFO] 🎯 到达目标点! [ID=1] 最终距离=0.850m
[main_gui_app] [INFO] 🔍 [DEBUG] 收到导航结果: usv_id=usv_01, goal_id=1, success=True
[main_gui_app] [INFO] ✅ usv_01 导航完成 [ID=1]: 成功到达目标点
```

✅ **验证点**:
- [ ] USV端检测到到达（距离 < 1.0m）
- [ ] 发送成功结果
- [ ] 地面站收到结果
- [ ] GUI显示"成功"状态
- [ ] 不再重复发送目标点

#### 测试3: 自动导航到达

**步骤**:
1. 发送导航目标后
2. 让USV自动导航（GUIDED模式）
3. 等待USV自动到达目标点

**预期结果**:
- 过程中定期看到导航反馈
- 到达后自动判定成功
- GUI状态更新为"成功"

✅ **验证点**:
- [ ] 导航过程中看到反馈（距离、航向）
- [ ] 自动检测到达
- [ ] 正确发送成功结果
- [ ] GUI正确更新状态

### 4. 集群导航测试

#### 测试4: 3艘USV集群导航

**步骤**:
1. 导入集群任务XML文件
2. 点击"cluster start"
3. 切换到GUIDED模式
4. 观察3艘USV的导航情况

**预期结果**:
```
[main_gui_app] [INFO] 📤 Step 1 → usv_01: Local(1.40, 6.10, 0.00) [首次发送]
[main_gui_app] [INFO] 📤 Step 1 → usv_02: Local(1.00, 6.40, 0.00) [首次发送]
[main_gui_app] [INFO] 📤 Step 1 → usv_03: Local(0.90, 5.90, 0.00) [首次发送]
... (导航过程) ...
[main_gui_app] [INFO] ✅ usv_01 导航完成 [ID=1]: 成功到达目标点
[main_gui_app] [INFO] ✅ usv_02 导航完成 [ID=2]: 成功到达目标点
[main_gui_app] [INFO] ✅ usv_03 导航完成 [ID=3]: 成功到达目标点
[main_gui_app] [INFO] 集群任务状态 running -> completed: 全部步骤完成
```

✅ **验证点**:
- [ ] 每个USV目标点只发送1次
- [ ] 所有USV都能收到目标
- [ ] 都能正确发送反馈和结果
- [ ] 集群任务正确完成

### 5. 异常情况测试

#### 测试5: 超时测试

**步骤**:
1. 发送一个很远的目标点（无法在超时时间内到达）
2. 等待超时（默认300秒）

**预期结果**:
```
[navigate_to_point_node] [WARN] ⏱️ 导航超时! [ID=1] 耗时=300.0s, 剩余距离=10.50m
[main_gui_app] [INFO] ❌ usv_01 导航完成 [ID=1]: 导航超时...
```

✅ **验证点**:
- [ ] 正确检测超时
- [ ] 发送失败结果
- [ ] GUI更新为"失败"状态

#### 测试6: 手动切换到MANUAL模式

**步骤**:
1. 导航过程中切换到MANUAL模式
2. 观察导航任务是否停止

**预期结果**:
- 导航反馈停止
- 不再发送目标点

✅ **验证点**:
- [ ] 导航任务正确停止
- [ ] 不再有导航相关日志

## 调试技巧

### 监控导航话题

```bash
# 监控导航目标（地面站发送）
ros2 topic echo /usv_01/navigation_goal

# 监控导航反馈（USV发送）
ros2 topic echo /usv_01/navigation_feedback

# 监控导航结果（USV发送）
ros2 topic echo /usv_01/navigation_result
```

### 检查节点是否运行

```bash
# 检查navigate_to_point_node是否运行
ros2 node list | grep navigate

# 应该看到：
# /usv_01/navigate_to_point_node
# /usv_02/navigate_to_point_node
# /usv_03/navigate_to_point_node
```

### 手动发送导航目标测试

```bash
# 手动发送导航目标测试USV端是否正常
ros2 topic pub --once /usv_01/navigation_goal common_interfaces/msg/NavigationGoal "{
  goal_id: 999,
  target_pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.0, y: 2.0, z: 0.0},
      orientation: {w: 1.0}
    }
  },
  timeout: 300.0
}"

# 然后监控反馈和结果
ros2 topic echo /usv_01/navigation_feedback
ros2 topic echo /usv_01/navigation_result
```

## 问题排查

### 问题1: 没有收到导航反馈

**可能原因**:
- Domain Bridge配置错误
- 话题名称不匹配

**排查步骤**:
```bash
# 检查话题是否存在
ros2 topic list | grep navigation

# 检查Domain Bridge配置
cat ~/usv_workspace/src/gs_bringup/config/domain_bridge.yaml | grep navigation
```

### 问题2: 仍然重复发送

**可能原因**:
- 代码没有正确编译
- 仍在运行旧版本节点

**排查步骤**:
```bash
# 确认重新编译
cd ~/usv_workspace
colcon build --packages-select gs_gui --cmake-clean-first

# 确认source新版本
source install/setup.bash

# 重启所有节点
```

### 问题3: USV端没有反应

**可能原因**:
- 仍在使用Action版本服务器
- navigate_to_point_node没有启动

**排查步骤**:
```bash
# 检查运行的节点
ros2 node list | grep navigate

# 应该是 navigate_to_point_node，不是 navigate_to_point_server
# 如果看到 navigate_to_point_server，说明launch文件没有更新

# 重新编译launch文件
colcon build --packages-select usv_bringup
source install/setup.bash
```

## 测试清单

### 基础功能
- [ ] 单个USV导航（发送目标）
- [ ] 手动到达检测
- [ ] 自动导航到达
- [ ] 导航反馈正常

### 集群功能
- [ ] 3艘USV同时导航
- [ ] 所有USV都能到达
- [ ] 集群任务正确完成

### 异常处理
- [ ] 超时正确处理
- [ ] 手动切换模式正常
- [ ] 停止任务正常

### 关键验证点
- [ ] **不再重复发送**（最重要！）
- [ ] 收到导航反馈
- [ ] 收到导航结果
- [ ] GUI状态正确更新

## 成功标准

✅ 所有测试项通过，特别是：
1. 每个目标点只发送1次
2. 手动到达能正确检测
3. 自动导航能正确完成
4. 集群导航所有USV都能正常工作

---

**测试日期**: ____________  
**测试人**: ____________  
**测试结果**: ⬜ 通过 / ⬜ 失败  
**备注**: ___________________________________________
