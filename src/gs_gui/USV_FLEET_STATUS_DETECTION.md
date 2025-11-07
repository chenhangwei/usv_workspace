# USV 集群启动器 - 状态检测优化

## 修改日期
2025-11-07

## 问题描述
USV 集群启动器的状态列一直显示"检测中..."，无法准确反映机载计算机的在线状态和节点运行状态。

## 需求
- **在线检测**：机载计算机连接到局域网 FastDDS 分布式网络时显示"在线"
- **离线检测**：搜索不到机载计算机时显示"离线"  
- **运行中检测**：节点启动后显示"运行中"
- **启动中检测**：启动命令发送但节点未上线时显示"启动中"

## 解决方案

### 1. 状态分类

新的状态系统包含 5 种状态：

| 状态 | 图标 | 颜色 | 判断条件 |
|------|------|------|----------|
| **离线** | ⚫ | 灰色 (#969696) | 机载计算机网络不可达（ping 不通） |
| **在线** | 🟡 | 黄色 (#FFC107) | 机载计算机在线但未启动节点 |
| **启动中** | 🔄 | 橙色 (#FF9800) | 启动命令已发送，等待节点上线 |
| **运行中** | 🟢 | 绿色 (#4CAF50) | 节点已启动并正常运行 |
| **已停止** | 🔴 | 红色 (#F44336) | （保留状态，暂未使用）|

### 2. 检测逻辑

```python
def _update_usv_status(self):
    """
    状态判断逻辑：
    1. 在线（online）：机载计算机连接到局域网，可以 ping 通
    2. 运行中（running）：节点已启动并正常运行
    3. 启动中（launching）：启动命令已发送，等待节点上线
    4. 离线（offline）：机载计算机未连接到网络
    """
    # 检测所有 ROS 节点
    online_nodes = ros2 node list
    
    for each usv:
        has_nodes = 检查命名空间是否存在于 online_nodes
        has_process = 检查是否有启动进程正在运行
        is_host_online = ping 检查主机网络可达性
        
        if has_nodes:
            状态 = 'running'      # 节点在线
        elif has_process:
            状态 = 'launching'    # 启动中
        elif is_host_online:
            状态 = 'online'       # 在线但未启动
        else:
            状态 = 'offline'      # 完全离线
```

### 3. 网络检测方法

使用 `ping` 命令检测机载计算机网络可达性：

```python
def _check_host_online(self, hostname):
    """
    检查主机是否在线（通过 ping）
    
    Args:
        hostname: 主机名或 IP 地址（从 usv_fleet.yaml 读取）
    
    Returns:
        bool: 主机在线返回 True，否则返回 False
    """
    result = subprocess.run(
        ['ping', '-c', '1', '-W', '1', hostname],  # 发送 1 个包，超时 1 秒
        capture_output=True,
        timeout=2
    )
    return result.returncode == 0
```

**优势**：
- ✅ 快速检测（1 秒超时）
- ✅ 不依赖 ROS 节点，即使节点未启动也能检测到机载计算机在线
- ✅ 网络层检测，比 FastDDS 发现更可靠

### 4. 初始化优化

在窗口打开时立即执行第一次状态检测，避免显示"检测中..."：

```python
def __init__(self, parent=None, workspace_path=None):
    # ... 其他初始化 ...
    
    # 加载配置
    self._load_fleet_config()
    
    # 立即执行第一次状态检测（新增）
    self._update_usv_status()
    
    # 启动定时器检测状态
    self.status_timer = QTimer()
    self.status_timer.timeout.connect(self._update_usv_status)
    self.status_timer.start(2000)  # 每 2 秒更新一次
```

### 5. 初始状态设置

表格初始化时设置为"离线"状态（而非"检测中..."）：

```python
# 列 3: 状态（初始化为离线，等待第一次状态检测更新）
status_item = QTableWidgetItem("⚫ 离线")
status_item.setTextAlignment(Qt.AlignCenter)
status_item.setFlags(status_item.flags() & ~Qt.ItemIsEditable)
status_item.setForeground(QColor(150, 150, 150))
self.usv_table.setItem(row, 3, status_item)
```

## 修改文件

- **文件**：`gs_gui/gs_gui/usv_fleet_launcher.py`
- **主要修改**：
  1. 添加 `_check_host_online()` 方法用于 ping 检测
  2. 更新 `_update_usv_status()` 方法增加网络检测逻辑
  3. 在 `_on_status_updated()` 中添加"在线"状态映射
  4. 初始化时立即触发状态检测
  5. 初始状态从"检测中..."改为"⚫ 离线"

## 使用说明

### 配置文件

确保 `usv_fleet.yaml` 中配置了正确的 `hostname`：

```yaml
usv_fleet:
  usv_01:
    enabled: true
    hostname: "192.168.68.101"  # 机载计算机 IP 地址
    username: "usv"
    workspace: "/home/usv/usv_workspace"
    fcu_url: "serial:///dev/ttyACM0:921600"
    system_id: 1
```

### 状态含义解读

1. **⚫ 离线**：机载计算机未连接到网络
   - 可能原因：设备关机、网络故障、IP 配置错误
   - 操作建议：检查设备电源和网络连接

2. **🟡 在线**：机载计算机已连接网络但未启动节点
   - 可能原因：设备刚启动、节点未运行、手动停止节点
   - 操作建议：可以点击"▶️ 启动"按钮启动节点

3. **🔄 启动中...**：启动命令已发送，等待节点上线
   - 预期时长：5-15 秒（取决于 MAVROS 启动速度）
   - 操作建议：耐心等待，如超过 30 秒仍未变为"运行中"，检查日志

4. **🟢 运行中**：节点已成功启动并正常运行
   - 操作建议：可以通过地面站 GUI 正常控制该 USV

## 性能影响

- **检测间隔**：2 秒（可调整 `status_timer.start()` 参数）
- **ping 超时**：1 秒（单个 USV）
- **ros2 node list 超时**：3 秒（所有 USV 共享）
- **总体开销**：对于 3 艘 USV，每次检测约需 6 秒（3 次 ping + 1 次 node list）

**优化建议**：
- 如果 USV 数量较多（>5 艘），可考虑将 ping 检测改为并行执行
- 可增加检测间隔到 5 秒以减少网络负载

## 测试步骤

### 1. 测试离线状态
```bash
# 关闭机载计算机或断开网络
# 打开集群启动器，应显示 "⚫ 离线"
```

### 2. 测试在线状态
```bash
# 启动机载计算机，但不启动 ROS 节点
# 等待 2-4 秒，状态应从 "⚫ 离线" 变为 "🟡 在线"
```

### 3. 测试启动中状态
```bash
# 点击 "▶️ 启动" 按钮
# 状态应立即变为 "🔄 启动中..."
```

### 4. 测试运行中状态
```bash
# 等待节点启动完成（约 10-15 秒）
# 状态应自动变为 "🟢 运行中"
```

## 未来改进方向

### 1. FastDDS 参与者检测（更精确）
当前使用 `ping` 检测网络可达性，未来可改用 FastDDS 参与者检测：

```bash
# 使用 ros2 daemon 检测 FastDDS 参与者
ros2 daemon stop
ros2 participant list
```

**优势**：
- 更准确地判断机载计算机是否在 DDS 网络中
- 即使节点未启动，只要 ROS 2 环境运行也能检测到

**劣势**：
- 需要机载计算机先 source ROS 2 环境
- 检测速度比 ping 慢

### 2. 并行检测优化
对于多艘 USV，可以并行执行 ping 检测：

```python
from concurrent.futures import ThreadPoolExecutor

def _update_usv_status(self):
    # 并行 ping 检测
    with ThreadPoolExecutor(max_workers=5) as executor:
        futures = {
            executor.submit(self._check_host_online, config['hostname']): usv_id
            for usv_id, config in self.fleet_config.items()
        }
        host_status = {
            futures[future]: future.result()
            for future in futures
        }
```

### 3. 健康检查增强
添加更多健康检查指标：
- CPU 使用率
- 内存使用率
- 磁盘空间
- 温度状态

## 相关文档

- **主文档**：`gs_gui/MODULE_ARCHITECTURE.md` - GUI 模块架构
- **快速参考**：`gs_gui/QUICK_REFERENCE.md` - 常用操作速查
- **集群启动器 V2**：`gs_gui/USV_FLEET_LAUNCHER_V2.md` - 启动器架构文档

## 总结

本次优化解决了状态列一直显示"检测中..."的问题，通过以下方式实现：

1. ✅ **网络检测**：使用 `ping` 快速检测机载计算机在线状态
2. ✅ **状态细化**：区分"离线"、"在线"、"启动中"、"运行中"四种状态
3. ✅ **即时更新**：窗口打开时立即执行状态检测
4. ✅ **可视化改进**：使用图标和颜色区分不同状态

现在用户可以清晰地看到每艘 USV 的实时状态，快速判断哪些设备在线、哪些设备需要启动。
