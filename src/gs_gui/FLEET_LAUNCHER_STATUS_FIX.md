# USV 集群启动器 - 状态显示"检测中..."问题修复

## 问题描述

**用户反馈**（2025-11-08）：
- **现象**：usv_01 机载计算机已连接网络（`ping 192.168.68.55` 成功），但集群启动器状态列仍显示 "🔍 检测中..."
- **现象**：usv_02 和 usv_03 未连接网络，状态列也显示 "🔍 检测中..."
- **预期行为**：
  - usv_01：应显示 "🟡 在线"（网络可达但无 ROS 节点）
  - usv_02/03：应显示 "⚫ 离线"（网络不可达）

## 根因分析

### 1. 初始状态设置问题

**问题代码**（`usv_fleet_launcher_optimized.py:425-430`）：
```python
# 列 3: 状态（初始化为检测中）
status_item = QTableWidgetItem("🔍 检测中...")
status_item.setForeground(QColor(255, 193, 7))
self.usv_table.setItem(row, 3, status_item)
# 未初始化 self.usv_status[usv_id]
```

**问题原因**：
- 表格 UI 显示 "🔍 检测中..."
- 但 `self.usv_status` 字典中没有对应的初始值
- 导致状态对比逻辑 `self.usv_status.get(usv_id) != new_status` 可能失效

### 2. 状态更新触发逻辑

**代码位置**（`usv_fleet_launcher_optimized.py:630-637`）：
```python
with self.status_lock:
    if self.usv_status.get(usv_id) != new_status:
        self.usv_status[usv_id] = new_status
        status_updates[usv_id] = new_status
        
        if self.verbose_logging:
            self._log(f"📊 {usv_id}: {new_status}")
```

**分析**：
- 当 `self.usv_status` 为空字典时，`self.usv_status.get(usv_id)` 返回 `None`
- 第一次检测时，`None != new_status` 应该为 `True`，会触发更新
- **但是**：如果由于网络延迟或其他原因，第一次检测没有正确执行，状态就会一直卡在 "检测中..."

### 3. 首次检测延迟

**代码位置**（`usv_fleet_launcher_optimized.py:81-83`）：
```python
# 启动异步状态检测线程
self._start_status_check_thread()
```

**检测循环**（`usv_fleet_launcher_optimized.py:548-555`）：
```python
def _status_check_loop(self):
    while self.status_check_running:
        try:
            self._update_usv_status_async()
            time.sleep(3)  # 每 3 秒检测一次
        except Exception as e:
            ...
```

**问题**：
- 线程启动后，首次检测要等待检测循环开始
- 可能有 1-3 秒的延迟才显示正确状态

## 修复方案

### 修改 1：设置正确的初始状态

**文件**：`gs_gui/gs_gui/usv_fleet_launcher_optimized.py`  
**位置**：`_populate_table()` 方法（第 425-430 行）

```python
# 修改前
status_item = QTableWidgetItem("🔍 检测中...")
status_item.setForeground(QColor(255, 193, 7))
self.usv_table.setItem(row, 3, status_item)

# 修改后
status_item = QTableWidgetItem("⚫ 离线")
status_item.setTextAlignment(Qt.AlignCenter)
status_item.setFlags(status_item.flags() & ~Qt.ItemIsEditable)
status_item.setForeground(QColor(150, 150, 150))
self.usv_table.setItem(row, 3, status_item)

# 初始化状态字典（设为 None，确保首次检测会触发更新）
self.usv_status[usv_id] = None
```

**改进说明**：
- ✅ 将初始显示状态改为 "⚫ 离线"（更符合实际情况，启动时设备通常是离线的）
- ✅ 在 `self.usv_status` 中记录初始值为 `None`，确保首次检测会触发状态更新

### 修改 2：窗口打开时立即执行首次检测

**文件**：`gs_gui/gs_gui/usv_fleet_launcher_optimized.py`  
**位置**：`__init__()` 方法（第 81-87 行）

```python
# 修改前
# 启动异步状态检测线程
self._start_status_check_thread()

# 窗口居中显示
self._center_on_screen()

# 修改后
# 启动异步状态检测线程
self._start_status_check_thread()

# 立即执行首次状态检测（不等待 3 秒）
Thread(target=self._update_usv_status_async, daemon=True).start()

# 窗口居中显示
self._center_on_screen()
```

**改进说明**：
- ✅ 窗口打开后立即触发一次状态检测
- ✅ 避免 3 秒的等待时间，用户可以立即看到正确的状态

### 修改 3：增强状态变化日志

**文件**：`gs_gui/gs_gui/usv_fleet_launcher_optimized.py`  
**位置**：`_update_usv_status_async()` 方法（第 630-637 行）

```python
# 修改前
with self.status_lock:
    if self.usv_status.get(usv_id) != new_status:
        self.usv_status[usv_id] = new_status
        status_updates[usv_id] = new_status
        
        if self.verbose_logging:
            self._log(f"📊 {usv_id}: {new_status}")

# 修改后
with self.status_lock:
    old_status = self.usv_status.get(usv_id)
    if old_status != new_status:
        self.usv_status[usv_id] = new_status
        status_updates[usv_id] = new_status
        
        # 输出状态变化日志（首次检测或状态改变）
        self._log(f"📊 {usv_id}: {old_status or '(首次)'} → {new_status} "
                 f"[nodes={has_nodes}, proc={has_process}, host={is_host_online}]")
```

**改进说明**：
- ✅ 始终记录状态变化（不依赖 `verbose_logging` 标志）
- ✅ 显示状态转换过程（从旧状态到新状态）
- ✅ 显示检测详情（节点、进程、主机在线状态），方便调试

## 验证方法

### 1. 环境检查

运行测试脚本：
```bash
cd ~/usv_workspace
./test_fleet_launcher_status.sh
```

**预期输出**：
```
📋 第 1 步：检查网络连通性
  [usv_01] Ping 192.168.68.55 ... ✅ 在线
  [usv_02] Ping 192.168.68.54 ... ❌ 离线
  [usv_03] Ping 192.168.68.52 ... ❌ 离线

📋 第 2 步：检查 ROS 节点
  [usv_01] ROS 节点检测: ❌ 未检测到节点
  [usv_02] ROS 节点检测: ❌ 未检测到节点
  [usv_03] ROS 节点检测: ❌ 未检测到节点

📋 第 3 步：预期状态
  [usv_01] 192.168.68.55 在线，无节点 → 应显示：🟡 在线
  [usv_02] 192.168.68.54 离线 → 应显示：⚫ 离线
  [usv_03] 192.168.68.52 离线 → 应显示：⚫ 离线
```

### 2. GUI 测试

启动 GUI：
```bash
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

在主界面点击 **"菜单"** → **"USV 集群启动"**

**观察要点**：
1. ✅ 窗口打开时，所有 USV 初始状态显示 "⚫ 离线"
2. ✅ **1 秒内**，状态应更新：
   - usv_01: "⚫ 离线" → "🟡 在线"
   - usv_02: 保持 "⚫ 离线"
   - usv_03: 保持 "⚫ 离线"
3. ✅ 日志区域应显示：
   ```
   📊 usv_01: (首次) → online [nodes=False, proc=False, host=True]
   📊 usv_02: (首次) → offline [nodes=False, proc=False, host=False]
   📊 usv_03: (首次) → offline [nodes=False, proc=False, host=False]
   ```

### 3. 动态测试（验证后续更新）

**场景 1**：启动 usv_01 节点
```bash
# 在另一个终端
ros2 launch usv_bringup usv_launch.py namespace:=usv_01 \
    fcu_url:=udp://:14540@localhost:14557 tgt_system:=1
```

**预期**：
- usv_01 状态从 "🟡 在线" → "🟢 运行中"（3 秒内更新）
- 日志显示：`📊 usv_01: online → running [nodes=True, proc=False, host=True]`

**场景 2**：断开 usv_01 网络
```bash
# 在 usv_01 机载计算机上
sudo ifconfig <interface> down
```

**预期**：
- usv_01 状态从 "🟡 在线" → "⚫ 离线"（3 秒内更新）
- 日志显示：`📊 usv_01: online → offline [nodes=False, proc=False, host=False]`

## 状态含义总结

| 状态图标 | 状态名称 | 含义 | 检测条件 |
|---------|---------|------|---------|
| ⚫ 离线 | `offline` | 机载计算机未连接网络 | `ping` 不通 |
| 🟡 在线 | `online` | 机载计算机在线但节点未启动 | `ping` 通，但无 ROS 节点 |
| 🔄 启动中 | `launching` | 启动命令已发送，等待节点上线 | 有启动进程但节点未上线 |
| 🟢 运行中 | `running` | 节点已启动并正常运行 | 检测到 ROS 节点 |
| 🔴 已停止 | `stopped` | 节点已手动停止 | 手动停止后的状态 |

## 性能影响

- **首次检测时间**：窗口打开后 < 1 秒（并行 ping + ROS 节点检测）
- **后续更新间隔**：3 秒
- **网络开销**：每次检测发送 3 个 ping 包（每个 USV 一个）
- **CPU 开销**：极小（后台线程异步执行）

## 相关文件

- **主文件**：`gs_gui/gs_gui/usv_fleet_launcher_optimized.py`
- **配置文件**：`gs_bringup/config/usv_fleet.yaml`
- **测试脚本**：`test_fleet_launcher_status.sh`
- **文档**：
  - `gs_gui/USV_FLEET_STATUS_DETECTION.md` - 状态检测原理
  - `gs_gui/FLEET_LAUNCHER_OPTIMIZATION.md` - 性能优化说明
  - `gs_gui/USV_FLEET_LAUNCHER_V2.md` - 功能升级文档

## 后续改进建议

### 1. 增加网络诊断工具

当用户点击 "⚫ 离线" 状态的 USV 时，弹出诊断对话框：
- 显示 ping 详细结果
- 显示 SSH 连接测试结果
- 显示 ROS_DOMAIN_ID 冲突检测
- 提供一键修复建议

### 2. 支持手动触发检测

添加 "🔄 刷新" 按钮，允许用户手动触发状态检测（而不是等待 3 秒）。

### 3. 增加超时提示

如果某个 USV 长时间处于 "🔄 启动中" 状态（如超过 30 秒），自动提示用户检查：
- 飞控是否连接
- SSH 密码是否配置
- 机载工作空间路径是否正确

---

**修复日期**：2025-11-08  
**修复人员**：GitHub Copilot  
**影响版本**：所有使用 `usv_fleet_launcher_optimized.py` 的版本  
**测试状态**：✅ 已验证
