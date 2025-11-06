# USV 集群启动器 - 功能升级文档

**升级日期**: 2025-11-06  
**版本**: 2.0.0  
**状态**: ✅ 完成

---

## 升级概述

将原有的简单命令行启动方式升级为功能完整、美观的图形化管理界面。

### 升级前（v1.0）
- ✅ 点击菜单 → 后台启动所有 USV
- ❌ 无法选择启动哪些 USV
- ❌ 无法单独控制单个 USV
- ❌ 无法停止已启动的 USV
- ❌ 无实时状态反馈

### 升级后（v2.0）
- ✅ 图形化启动器对话框
- ✅ 在线设备列表（自动检测）
- ✅ 单独启动/停止单个 USV
- ✅ 批量启动/停止选中的 USV
- ✅ 停止所有 USV 节点
- ✅ 实时状态监控（离线/启动中/运行中/已停止）
- ✅ 现代化美观 UI 设计
- ✅ 操作日志实时显示

---

## 新增功能详解

### 1. 📋 USV 设备列表

**功能**：
- 自动从 `usv_fleet.yaml` 加载所有配置的 USV
- 显示设备 ID、主机地址、FCU 配置等信息
- 实时更新设备状态（每 2 秒）

**状态类型**：
| 图标 | 状态 | 说明 |
|------|------|------|
| ⚫ | 离线 | USV 未启动，节点不在线 |
| 🔄 | 启动中... | SSH 连接已建立，等待节点上线 |
| 🟢 | 运行中 | USV 节点正常运行 |
| 🔴 | 已停止 | USV 已手动停止 |

### 2. ✓ 设备选择功能

**功能**：
- 每行前有复选框，可多选
- **全选** 按钮：选中所有 USV
- **取消全选** 按钮：清除所有选择

**使用场景**：
- 批量启动部分 USV（如只启动 usv_01 和 usv_03）
- 批量停止部分 USV

### 3. ▶ 单独启动/停止

**功能**：
- 每行有独立的 **▶ 启动** 和 **⏹ 停止** 按钮
- 可精确控制单个 USV

**启动流程**：
```
点击"启动" → 发送 SSH 命令 → 远程启动 ros2 launch → 状态变为"启动中"
→ (2秒后) 检测到节点上线 → 状态变为"运行中"
```

**停止流程**：
```
点击"停止" → 终止本地 SSH 进程 → 发送远程 kill 命令 → 状态变为"已停止"
```

### 4. 🎯 批量操作

**🚀 启动选中**：
- 启动所有勾选的 USV
- 间隔 2 秒依次启动（避免同时连接）
- 显示确认对话框

**⏹ 停止选中**：
- 停止所有勾选的 USV
- 显示确认对话框

**⏹ 停止所有 USV**：
- 停止所有正在运行的 USV
- 红色醒目按钮，防止误操作
- 双重确认对话框

### 5. 📝 操作日志

**功能**：
- 实时显示所有操作和状态变化
- 黑底绿字终端风格
- 自动滚动到最新日志

**日志示例**：
```
✅ 加载配置成功: 3 艘 USV
🚀 正在启动 usv_01...
✅ usv_01 启动命令已发送 (PID: 12345)
⏹ 正在停止 usv_02...
✅ usv_02 已停止
```

### 6. 🔄 刷新状态

**功能**：
- 手动刷新所有 USV 状态
- 自动刷新：每 2 秒一次

**刷新逻辑**：
```bash
# 获取所有 ROS 节点
ros2 node list

# 检查每个 USV 的节点是否在线
# 例如：/usv_01/mavros_node, /usv_01/usv_control_node 等

# 更新状态：
# - 有节点 + 进程运行 → 运行中
# - 无节点 + 进程运行 → 启动中
# - 有节点 + 进程停止 → 运行中（已启动但本地进程结束）
# - 无节点 + 进程停止 → 离线/已停止
```

---

## 界面设计

### 现代化 UI 风格

**配色方案**：
- 主色调：蓝色 (#2196F3) - 专业、科技
- 成功色：绿色 (#4CAF50) - 启动按钮
- 警告色：橙色 (#FF9800) - 启动中状态
- 危险色：红色 (#F44336) - 停止按钮
- 背景色：浅灰 (#F5F5F5) - 清爽简洁

**组件样式**：
- **表格头**：蓝色背景 + 白色粗体文字
- **表格行**：交替颜色 + 悬停高亮
- **按钮**：圆角 + 渐变 + 悬停效果
- **分组框**：圆角边框 + 阴影效果
- **日志区**：黑色终端风格

**布局设计**：
```
┌─────────────────────────────────────────┐
│     🚀 USV 集群管理                      │
│   管理和监控所有 USV 节点的启动与停止    │
├─────────────────────────────────────────┤
│  📋 USV 设备列表                         │
│  ┌───┬────────┬──────┬────┬─────┬────┐  │
│  │✓  │设备ID  │主机  │状态│操作 │详情│  │
│  ├───┼────────┼──────┼────┼─────┼────┤  │
│  │☑  │usv_01  │.55   │🟢  │▶⏹  │FCU │  │
│  │☐  │usv_02  │.54   │⚫  │▶⏹  │FCU │  │
│  │☑  │usv_03  │.52   │🔄  │▶⏹  │FCU │  │
│  └───┴────────┴──────┴────┴─────┴────┘  │
├─────────────────────────────────────────┤
│  🎯 批量操作                             │
│  [✓全选] [✗取消] [🚀启动选中] [⏹停止选中]│
├─────────────────────────────────────────┤
│  📝 操作日志                             │
│  ┌─────────────────────────────────────┐│
│  │ ✅ usv_01 启动命令已发送            ││
│  │ 🔄 手动刷新状态...                  ││
│  │ ✅ 刷新完成                         ││
│  └─────────────────────────────────────┘│
├─────────────────────────────────────────┤
│  [🔄刷新] [⏹停止所有]         [关闭]   │
└─────────────────────────────────────────┘
```

---

## 使用方法

### 启动 USV 集群启动器

**方式 1：菜单栏**
```
地面站 GUI → 菜单栏 → USV控制 → 🚀 启动 USV 集群
```

**方式 2：快捷键**
```
Ctrl + L
```

### 启动单个 USV

1. 在设备列表中找到目标 USV
2. 点击该行右侧的 **▶ 启动** 按钮
3. 观察状态列变化：⚫ → 🔄 → 🟢

### 批量启动多个 USV

1. 勾选要启动的 USV（可用"全选"按钮）
2. 点击 **🚀 启动选中** 按钮
3. 在确认对话框中点击 **Yes**
4. 等待所有 USV 依次启动（间隔 2 秒）

### 停止单个 USV

1. 在设备列表中找到目标 USV
2. 点击该行右侧的 **⏹ 停止** 按钮
3. 观察状态列变化：🟢 → 🔴

### 停止所有 USV

1. 点击底部红色的 **⏹ 停止所有 USV** 按钮
2. 在确认对话框中查看将停止的 USV 列表
3. 点击 **Yes** 确认
4. 所有运行中的 USV 将被停止

### 查看状态和日志

- **状态列**：实时显示每个 USV 的当前状态
- **日志区**：查看所有操作记录和错误信息
- **刷新按钮**：手动触发状态更新

---

## 技术实现

### 核心类：`UsvFleetLauncher`

**文件**：`gs_gui/gs_gui/usv_fleet_launcher.py`

**主要方法**：

```python
class UsvFleetLauncher(QDialog):
    # 初始化
    __init__(parent, workspace_path)
    
    # UI 构建
    _init_ui()                      # 构建界面
    _apply_styles()                 # 应用样式
    _populate_table()               # 填充设备列表
    _create_action_buttons(usv_id)  # 创建操作按钮
    
    # 配置加载
    _load_fleet_config()            # 加载 usv_fleet.yaml
    
    # 状态管理
    _update_usv_status()            # 更新所有状态（定时器）
    _on_status_updated(usv_id, status)  # 状态变化回调
    _refresh_status()               # 手动刷新
    
    # 启动操作
    _launch_single(usv_id)          # 启动单个 USV
    _launch_selected()              # 启动选中的 USV
    
    # 停止操作
    _stop_single(usv_id)            # 停止单个 USV
    _stop_selected()                # 停止选中的 USV
    _stop_all()                     # 停止所有 USV
    
    # 选择操作
    _select_all()                   # 全选
    _deselect_all()                 # 取消全选
    _get_selected_usvs()            # 获取选中列表
    
    # 日志
    _log(message)                   # 添加日志
```

### 状态检测逻辑

```python
def _update_usv_status(self):
    # 1. 获取所有 ROS 节点
    result = subprocess.run(['ros2', 'node', 'list'], ...)
    online_nodes = result.stdout.split('\n')
    
    # 2. 检查每个 USV
    for usv_id in fleet_config:
        namespace = f"/{usv_id}"
        has_nodes = any(namespace in node for node in online_nodes)
        
        # 3. 判断状态
        if usv_id in processes and process.poll() is None:
            status = 'launching' if not has_nodes else 'running'
        else:
            status = 'running' if has_nodes else 'offline'
        
        # 4. 更新 UI
        self.status_updated.emit(usv_id, status)
```

### SSH 远程启动

```python
def _launch_single(self, usv_id):
    # 构建远程命令
    remote_cmd = (
        f"bash -c '"
        f"source /opt/ros/*/setup.bash; "
        f"source {workspace}/install/setup.bash; "
        f"ros2 launch usv_bringup usv_launch.py "
        f"namespace:={usv_id} fcu_url:=... tgt_system:=..."
        f"'"
    )
    
    # SSH 命令
    ssh_cmd = [
        'ssh', '-o', 'StrictHostKeyChecking=no',
        f'{username}@{hostname}',
        remote_cmd
    ]
    
    # 启动进程
    process = subprocess.Popen(ssh_cmd, ...)
    self.usv_processes[usv_id] = process
```

### 远程停止

```python
def _stop_single(self, usv_id):
    # 1. 终止本地 SSH 进程
    if usv_id in self.usv_processes:
        process.terminate()
    
    # 2. 杀死远程 ROS 节点
    kill_cmd = [
        'ssh', f'{username}@{hostname}',
        "pkill -f 'ros2 launch usv_bringup' || killall -9 ros2"
    ]
    subprocess.run(kill_cmd, ...)
```

---

## 前置条件

### 1. SSH 免密登录配置

```bash
# 在地面站生成密钥
ssh-keygen -t rsa -b 4096

# 复制公钥到所有 USV
ssh-copy-id chenhangwei@192.168.68.55  # USV 01
ssh-copy-id chenhangwei@192.168.68.54  # USV 02  
ssh-copy-id chenhangwei@192.168.68.52  # USV 03

# 验证免密登录
ssh chenhangwei@192.168.68.55 "hostname && exit"
```

### 2. USV 集群配置

确保 `usv_fleet.yaml` 正确配置：

```yaml
usv_fleet:
  usv_01:
    enabled: true
    hostname: "192.168.68.55"
    username: "chenhangwei"
    workspace: "/home/chenhangwei/usv_workspace"
    namespace: "usv_01"
    mavlink_sys_id: 1
    system_id: 1
    fcu_url: "serial:///dev/ttyACM0:921600"
    gcs_url: "udp://:14560@192.168.68.53:14550"
```

### 3. 工作空间编译

```bash
cd ~/usv_workspace
colcon build --packages-select gs_gui
source install/setup.bash
```

---

## 故障排查

### 问题 1: 配置文件未找到

**错误**：`⚠️ 配置文件未找到: .../usv_fleet.yaml`

**解决方案**：
```bash
# 检查文件是否存在
ls ~/usv_workspace/install/gs_bringup/share/gs_bringup/config/usv_fleet.yaml

# 如果不存在，编译 gs_bringup
cd ~/usv_workspace
colcon build --packages-select gs_bringup
```

### 问题 2: SSH 连接失败

**症状**：状态一直显示"🔄 启动中..."，无法变为"🟢 运行中"

**检查步骤**：
```bash
# 1. 测试网络
ping 192.168.68.55

# 2. 测试 SSH
ssh chenhangwei@192.168.68.55 "echo OK"

# 3. 查看日志区的错误信息
```

### 问题 3: 节点检测失败

**症状**：USV 已启动但状态显示"⚫ 离线"

**解决方案**：
1. 点击 **🔄 刷新状态** 按钮
2. 检查 ROS 节点：`ros2 node list | grep usv_01`
3. 检查 ROS_DOMAIN_ID 是否一致

### 问题 4: 停止失败

**症状**：点击停止后 USV 仍在运行

**解决方案**：
```bash
# 手动 SSH 到 USV 停止
ssh chenhangwei@192.168.68.55
pkill -f ros2
killall -9 ros2
```

---

## 相关文档

- **双 GUI 问题修复**: `gs_gui/DOUBLE_GUI_FIX.md`
- **分布式 Launch 修复**: `gs_bringup/DISTRIBUTED_LAUNCH_FIX.md`
- **原版启动说明**: `gs_gui/USV_FLEET_LAUNCH_MENU.md`

---

## 升级总结

### 功能对比表

| 功能 | v1.0 | v2.0 |
|------|------|------|
| 启动所有 USV | ✅ | ✅ |
| 选择启动部分 USV | ❌ | ✅ |
| 单独启动/停止 | ❌ | ✅ |
| 停止所有 USV | ❌ | ✅ |
| 实时状态显示 | ❌ | ✅ |
| 操作日志 | ❌ | ✅ |
| 图形化界面 | ❌ | ✅ |
| 批量操作 | ❌ | ✅ |

### 用户体验提升

- **操作更直观**：图形化界面，一目了然
- **控制更精细**：可单独控制每个 USV
- **反馈更及时**：实时状态 + 操作日志
- **界面更美观**：现代化设计，符合主题风格

---

**维护者**: GitHub Copilot  
**版本**: 2.0.0  
**最后更新**: 2025-11-06  
**状态**: ✅ 生产就绪
