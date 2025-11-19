# USV ARM 命令和 Ready 检查问题修复

## 修复日期
2025-11-18

## 问题描述

### 问题 1: USV 没有收到 arm 命令
- 用户点击"解锁"按钮后，USV 没有收到 arm 命令
- 原因：之前的修改中，离线的 USV (`connected=False`) 不在集群列表中，导致 arm 命令根本不会发送

### 问题 2: Ready 检查一直显示黄色
- Ready 按钮一直显示黄色"PreArm Checks Required"
- 显示"[传感器] 等待传感器数据... 异常"警告
- 原因：传感器健康检查逻辑在未收到 SYS_STATUS 消息时返回失败

## 解决方案

### 架构调整

改进状态管理的分层架构：

1. **数据层** (`ground_station_node`, `usv_manager`): 
   - 维护所有 USV 的完整状态（包括在线和离线）
   - 推送所有 USV 状态到 GUI

2. **逻辑层** (`state_handler`, `list_manager`):
   - 接收所有 USV 状态
   - 在命令处理时检查 USV 是否在线

3. **显示层** (`table_manager`):
   - 只显示在线的 USV (`connected=True`)
   - 离线的 USV 从表格中自动移除

### 修改文件

#### 1. `gs_gui/command_processor.py`

**修改**: 增强 `_set_arming_for_usvs()` 方法

**改进**:
- 支持从字典类型的 state 对象中提取 namespace
- 添加在线状态检查，只向在线的 USV 发送命令
- 添加详细的统计日志（发送成功、离线跳过、无效 USV）
- 明确的日志输出，帮助用户了解命令发送情况

**关键逻辑**:
```python
# 检查USV是否在线
is_online = False
if usv_id in self.node.usv_states:
    is_online = self.node.usv_states[usv_id].get('connected', False)

if not is_online:
    offline_count += 1
    self.node.get_logger().warn(f"⚠️  {usv_id}: USV离线，跳过 {arming_state} 命令")
    continue

# 发送命令
arming_msg = String()
arming_msg.data = arming_state
self.node.publish_queue.put((self.node.usv_manager.set_usv_arming_pubs[usv_id], arming_msg))
sent_count += 1
```

#### 2. `gs_gui/state_handler.py`

**修改**: 保留 `_flush_state_cache_to_ui()` 方法原样

**说明**: 
- 推送所有 USV 状态（包括离线的）
- 离线过滤在显示层进行

#### 3. `gs_gui/table_manager.py`

**修改**: 在 `update_cluster_table()` 和 `update_departed_table()` 中添加过滤

**新增逻辑**:
```python
# 过滤掉离线的 USV（只显示 connected=True 的）
online_state_list = [
    s for s in state_list
    if isinstance(s, dict) and s.get('connected', False)
]
```

**效果**:
- 离线的 USV 不会出现在表格中
- USV 离线后自动从表格中移除
- USV 上线后自动出现在表格中

#### 4. `gs_gui/ground_station_node.py` 和 `usv_manager.py`

**修改**: 撤销之前的状态过滤

**说明**:
- 恢复推送所有 USV 状态（包括离线的）
- 确保内部状态管理完整性
- 命令发送可以访问到所有 USV 的状态信息

## 工作流程

### ARM 命令发送流程

```
用户点击"解锁"按钮
    ↓
MainWindow.set_cluster_arming_command()
    ↓
USVCommandHandler.set_cluster_arming(usv_cluster_list)
    ↓
ros_signal.arm_command.emit(namespace_list)
    ↓
CommandProcessor.set_arming_callback(msg)
    ↓
CommandProcessor._set_arming_for_usvs(msg, "ARMING")
    ↓
【检查 1】USV 是否有发布者？
    ├─ 否 → 跳过（无效 USV）
    └─ 是 → 继续
         ↓
【检查 2】USV 是否在线？
    ├─ 否 → 跳过（离线 USV）+ 记录警告日志
    └─ 是 → 发送 ARM 命令
         ↓
发送到 /usv_XX/set_usv_arming topic
    ↓
UsvCommandNode 接收并调用 MAVROS 服务
    ↓
飞控执行解锁操作
```

### 离线检测和显示流程

```
check_usv_topics_availability() 定期检查
    ↓
超过 10 秒未收到数据
    ↓
标记 usv_states[usv_id]['connected'] = False
    ↓
推送所有状态到 GUI
    ↓
state_handler 接收状态
    ↓
list_manager.update_online_list(all_states)
    ↓
table_manager.update_cluster_table(state_list, ...)
    ↓
过滤掉 connected=False 的 USV
    ↓
表格中移除离线 USV
```

## 日志输出示例

### 成功发送 ARM 命令
```
[INFO] 接收到武装命令
[INFO] ✓ usv_01: 发送 ARMING 命令
[INFO] ✓ usv_02: 发送 ARMING 命令
[INFO] 📤 ARMING 命令已发送至 2 艘在线USV
```

### USV 离线时的输出
```
[INFO] 接收到武装命令
[WARN] ⚠️  usv_01: USV离线，跳过 ARMING 命令
[INFO] ✓ usv_02: 发送 ARMING 命令
[INFO] 📤 ARMING 命令已发送至 1 艘在线USV
[WARN] ⚠️  跳过 1 艘离线USV
```

### 无效 USV 的输出
```
[INFO] 接收到武装命令
[WARN] ⚠️  usv_99: 发布者不存在，跳过
[INFO] 📤 ARMING 命令已发送至 0 艘在线USV
[WARN] ⚠️  跳过 1 个无效USV
```

## Ready 检查说明

Ready 状态由三个条件决定：

1. **无 PreArm 警告** - 来自飞控的 STATUSTEXT 消息
2. **无严重错误** - 最近 30 秒内没有 CRITICAL/ERROR 级别的消息
3. **传感器健康** - 关键传感器（陀螺仪、加速度计、气压计、GPS）都正常

### 传感器健康检查逻辑

- 如果未收到 `SYS_STATUS` 消息，返回 `False, ["等待传感器数据..."]`
- 这会导致 Ready 检查显示黄色
- 这是正常的保护机制，确保在传感器数据可用前不允许解锁

### 解决 Ready 检查显示黄色的方法

1. **确认 MAVROS 正常运行**
   ```bash
   ros2 topic list | grep /usv_01/mavros
   ros2 topic echo /usv_01/mavros/state
   ```

2. **检查飞控连接**
   ```bash
   ros2 topic echo /usv_01/mavros/sys_status
   ```

3. **查看 PreArm 警告**
   - 在 USV 信息面板中查看"PreArm 警告"部分
   - 根据警告信息解决相应问题（GPS、传感器、校准等）

4. **检查传感器状态**
   - 在"传感器状态"部分查看各传感器的状态
   - 绿色表示正常，黄色表示警告，红色表示错误

## 测试方法

### 测试 1: 在线 USV 收到 ARM 命令

1. 确保 USV 在线（能在表格中看到）
2. 点击"解锁"按钮
3. 检查日志输出是否显示命令已发送
4. 检查 USV 是否解锁成功

### 测试 2: 离线 USV 不收到 ARM 命令

1. 停止 USV 节点或断开网络
2. 等待 10 秒（离线阈值）
3. USV 应该从表格中消失
4. 点击"解锁"按钮
5. 日志应该显示"跳过离线USV"

### 测试 3: 离线 USV 重新上线

1. 重新启动 USV 节点
2. USV 应该重新出现在表格中
3. Ready 状态应该更新（可能需要等待传感器数据）
4. 点击"解锁"按钮应该能成功发送命令

## 注意事项

1. **离线阈值**: 10 秒未收到数据认为离线
2. **内部状态保留**: `usv_states` 字典保留所有 USV 状态，只是在显示时过滤
3. **命令检查**: 所有命令（ARM、模式切换等）都会检查 USV 在线状态
4. **传感器数据**: Ready 检查需要等待传感器数据，首次启动可能需要几秒钟
5. **PreArm 警告**: 飞控的 PreArm 检查是强制性的，必须解决所有警告才能解锁

## 相关文件

- `gs_gui/command_processor.py` - 命令处理和在线检查
- `gs_gui/table_manager.py` - 表格显示过滤
- `gs_gui/state_handler.py` - 状态缓存管理
- `gs_gui/ground_station_node.py` - 状态检测和 Ready 检查
- `gs_gui/usv_manager.py` - USV 发布者管理
- `usv_control/usv_command_node.py` - 机载命令处理
