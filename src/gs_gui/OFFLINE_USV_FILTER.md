# USV 离线设备过滤功能

## 修改日期
2025-11-18

## 问题描述
USV 设备离线后（`connected=False`），仍然会出现在地面站的集群列表和离群列表中，导致用户看到已离线的设备。

## 解决方案
在所有推送 USV 状态到 GUI 的地方，添加过滤逻辑，只推送 `connected=True` 的在线 USV。

## 修改文件

### 1. `gs_gui/state_handler.py`
**位置**: `_flush_state_cache_to_ui()` 方法

**修改前**:
```python
# 用缓存构建在线列表
online_list = list(self._usv_state_cache.values())
```

**修改后**:
```python
# 用缓存构建在线列表，只包含 connected=True 的 USV
online_list = [
    state for state in self._usv_state_cache.values()
    if state.get('connected', False)
]
```

### 2. `gs_gui/ground_station_node.py`
修改了 3 处状态推送位置：

#### (1) `check_usv_topics_availability()` 方法
```python
# 只推送在线的USV
online_states = [state for state in self.usv_states.values() if state.get('connected', False)]
self.ros_signal.receive_state_list.emit(online_states)
```

#### (2) 批量状态推送
```python
# 推送完整状态列表到 GUI（只推送在线的USV）
online_states = [state for state in self.usv_states.values() if state.get('connected', False)]
self.ros_signal.receive_state_list.emit(online_states)
```

#### (3) `handle_statustext()` 方法
```python
# 只推送在线的USV
online_states = [state for state in self.usv_states.values() if state.get('connected', False)]
self.ros_signal.receive_state_list.emit(online_states)
```

### 3. `gs_gui/usv_manager.py`
修改了 2 处状态推送位置：

#### (1) `_remove_usv_state()` 方法
```python
# 只推送在线的USV
online_states = [state for state in self.node.usv_states.values() if state.get('connected', False)]
self.node.ros_signal.receive_state_list.emit(online_states)
```

#### (2) `usv_state_callback()` 方法
```python
# 只推送在线的USV
online_states = [state for state in self.node.usv_states.values() if state.get('connected', False)]
self.node.ros_signal.receive_state_list.emit(online_states)
```

## 工作原理

1. **离线检测**: `ground_station_node.py` 的 `check_usv_topics_availability()` 方法定期检查 USV topic 上是否有数据
   - 如果超过 10 秒未收到数据，将 USV 标记为离线（`connected=False`）
   - 如果重新收到数据，将 USV 标记为在线（`connected=True`）

2. **状态过滤**: 所有推送到 GUI 的状态列表现在都会被过滤
   - 只有 `connected=True` 的 USV 才会被推送到 GUI
   - GUI 的 `state_handler.py` 会将过滤后的列表传递给 `usv_list_manager`
   - `usv_list_manager` 更新集群列表和离群列表时，只会看到在线的 USV

3. **自动移除**: 当 USV 离线后
   - 它会从 `usv_online_list` 中消失
   - `update_cluster_list()` 方法会检查 USV 是否在 `usv_online_list` 中
   - 如果不在，该 USV 会从 `usv_cluster_list` 中移除
   - 同样，离群列表也会自动清理离线的 USV

## 测试方法

### 1. 启动地面站
```bash
cd /home/chenhangwei/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

### 2. 观察在线 USV
- 打开地面站 GUI
- 查看集群列表，应该能看到所有在线的 USV

### 3. 模拟 USV 离线
在 USV 机载计算机上停止节点：
```bash
# 在 USV 机载计算机上
pkill -f "ros2 launch usv_bringup"
```

或者断开 USV 的网络连接。

### 4. 验证离线移除
- 等待 10 秒（离线阈值）
- 地面站日志应该显示：`⚠️  usv_XX 已离线（XX.Xs未收到数据）`
- 该 USV 应该从集群列表中消失

### 5. 验证重新上线
重新启动 USV 节点后：
- 地面站日志应该显示：`✓ usv_XX 已上线`
- 该 USV 应该重新出现在列表中

## 注意事项

1. **离线阈值**: 当前设置为 10 秒（`offline_threshold = 10.0`）
2. **内部状态保留**: `usv_states` 字典中仍然保留离线 USV 的状态信息，只是不推送到 GUI
3. **LED 传染逻辑**: `led_infection.py` 仍然使用完整的 `usv_states`，因为需要计算所有 USV 之间的距离

## 影响范围

✅ 集群列表：离线 USV 会自动移除  
✅ 离群列表：离线 USV 会自动移除  
✅ USV 信息面板：选中的 USV 离线后会清空显示  
✅ USV 导航面板：选中的 USV 离线后会清空显示  
⚠️ LED 传染检测：仍然使用所有 USV（包括离线的）进行距离计算
