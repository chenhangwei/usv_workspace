# USV 集群启动器 - 机载计算机重启功能

## 功能概述

在 USV 集群启动器中添加了机载计算机重启功能，支持单个重启和批量重启操作。

**实现日期**: 2025-11-07  
**功能状态**: ✅ 已完成并构建成功

---

## 功能特性

### 1. 单个 USV 重启
- 在每个 USV 操作按钮区域新增 "🔄 重启" 按钮
- 点击后弹出确认对话框，提醒用户重启影响
- 通过 MAVLink 命令 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (param2=3.0) 重启机载计算机
- 重启后系统需要 30-60 秒恢复在线

### 2. 批量重启
- 在批量操作区域新增 "🔄 重启选中" 按钮
- 支持批量重启多艘 USV 的机载计算机
- 自动延迟 2 秒发送每个重启命令，避免网络拥塞
- 显示详细的批量操作进度日志

---

## 实现细节

### 1. ROS 信号层 (`ros_signal.py`)

添加了新的 PyQt 信号用于触发机载计算机重启：

```python
# 机载计算机重启命令信号
reboot_companion = pyqtSignal(str)  # 机载计算机重启命令，参数：USV命名空间
```

### 2. 地面站节点层 (`ground_station_node.py`)

实现了重启回调函数，**优先使用 SSH 直接重启**（更可靠），MAVLink 作为备选：

```python
def reboot_companion_callback(self, usv_namespace):
    """
    机载计算机重启回调
    
    方法 1（推荐）: 通过 SSH 执行 `sudo reboot`
    方法 2（备选）: MAVLink 命令（某些飞控可能不支持）
    """
    # 从 usv_fleet.yaml 读取机载计算机配置
    hostname = usv_config.get('hostname')
    username = usv_config.get('username')
    
    # 构建 SSH 重启命令
    ssh_cmd = [
        'ssh',
        '-o', 'StrictHostKeyChecking=no',
        '-o', 'ConnectTimeout=5',
        f'{username}@{hostname}',
        'sudo reboot'
    ]
    
    # 异步执行（不等待返回）
    subprocess.Popen(ssh_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
```

**实现说明**:
- **方法 1 - SSH 重启** (推荐):
  - 直接通过 SSH 执行 `sudo reboot` 命令
  - 从 `usv_fleet.yaml` 读取机载计算机 IP 和用户名
  - 需要配置免密 SSH 登录（`ssh-copy-id`）
  - 更可靠，不依赖飞控支持

- **方法 2 - MAVLink 命令** (备选):
  - `command = 246`: `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`
  - `param1 = 0.0`: 不重启飞控
  - `param2 = 3.0`: 重启机载计算机
  - 某些飞控固件可能不支持或需要特殊配置

### 3. 主窗口连接 (`main_gui_app.py`)

连接 ROS 信号到地面站节点回调：

```python
# 连接机载计算机重启信号
ros_signal.reboot_companion.connect(node.reboot_companion_callback)
```

### 4. 集群启动器 UI (`usv_fleet_launcher.py`)

#### 单个重启按钮
在操作按钮区域添加橙色的 "🔄 重启" 按钮：

```python
# 重启按钮 - 紧凑设计
reboot_btn = QPushButton("🔄 重启")
reboot_btn.setStyleSheet("""
    QPushButton {
        background-color: #FF9800;  # 橙色
        color: white;
        ...
    }
""")
reboot_btn.clicked.connect(lambda: self._reboot_single(usv_id))
```

#### 批量重启按钮
在批量操作区域添加 "🔄 重启选中" 按钮：

```python
self.reboot_selected_btn = QPushButton("🔄 重启选中")
self.reboot_selected_btn.setStyleSheet("""
    QPushButton {
        background-color: #FF9800;  # 橙色
        ...
    }
""")
self.reboot_selected_btn.clicked.connect(self._reboot_selected)
```

#### 重启逻辑实现

**单个重启** (`_reboot_single`):
```python
def _reboot_single(self, usv_id):
    """重启单个 USV 的机载计算机"""
    # 1. 弹出确认对话框
    reply = QMessageBox.question(...)
    
    # 2. 获取父窗口的 ROS 信号
    parent = self.parent()
    if parent and hasattr(parent, 'ros_signal'):
        # 3. 发送重启信号
        parent.ros_signal.reboot_companion.emit(usv_id)
```

**批量重启** (`_reboot_selected`):
```python
def _reboot_selected(self):
    """批量重启选中的 USV 机载计算机"""
    selected = self._get_selected_usvs()
    
    # 遍历选中的 USV
    for usv_id in selected:
        parent.ros_signal.reboot_companion.emit(usv_id)
        # 延迟 2 秒避免同时发送命令
        time.sleep(2)
```

---

## 用户界面调整

### 表格列宽度
为了容纳新的重启按钮，调整了"操作"列的宽度：

**修改前**: 180px（仅启动按钮）  
**修改后**: 240px（启动 + 重启按钮）

```python
self.usv_table.setColumnWidth(4, 240)   # 操作列增加到 240px
```

### 按钮布局
- **启动按钮**: 绿色 (#4CAF50)，表示启动操作
- **重启按钮**: 橙色 (#FF9800)，表示重启操作
- 两个按钮水平排列，左对齐
- 固定高度 38px，宽度 70-85px

---

## 安全机制

### 确认对话框
所有重启操作都需要用户确认：

**单个重启确认**:
```
确定要重启 usv_01 的机载计算机吗？

⚠️ 重启后系统需要 30-60 秒恢复在线
⚠️ 所有运行中的节点将被终止
```

**批量重启确认**:
```
确定要重启以下 3 艘 USV 的机载计算机吗？

usv_01
usv_02
usv_03

⚠️ 重启后系统需要 30-60 秒恢复在线
⚠️ 所有运行中的节点将被终止
```

### 错误处理
- ROS 信号不可用时显示警告
- 服务不可用时记录错误日志
- 命令发送失败时弹出错误对话框

---

## 测试建议

### 单元测试
1. 测试 `reboot_companion` 信号的连接
2. 测试 `reboot_companion_callback` 的 MAVLink 命令构建
3. 测试确认对话框的显示

### 集成测试
1. **单个重启测试**:
   - 启动地面站和至少一艘 USV
   - 打开集群启动器
   - 点击"🔄 重启"按钮
   - 确认对话框出现
   - 点击"Yes"确认
   - 检查日志输出和机载计算机重启状态

2. **批量重启测试**:
   - 选中多艘 USV
   - 点击"🔄 重启选中"按钮
   - 确认批量重启对话框
   - 检查所有 USV 是否成功接收重启命令
   - 监控日志输出，确认延迟发送生效

### 现场测试
1. 验证机载计算机是否成功重启
2. 检查重启后系统恢复时间（应在 30-60 秒内）
3. 确认所有节点重新上线后功能正常

---

## 构建状态

✅ **构建成功**  
```bash
cd /home/chenhangwei/usv_workspace
colcon build --packages-select gs_gui
```

**构建输出**:
```
Summary: 1 package finished [2.29s]
```

---

## 使用方法

### 单个 USV 重启
1. 打开地面站 GUI
2. 点击菜单 **USV → 集群启动器**
3. 在 USV 列表中找到目标 USV
4. 点击该行的 "🔄 重启" 按钮
5. 确认重启操作
6. 等待 30-60 秒，USV 将重新上线

### 批量重启
1. 打开集群启动器
2. 勾选需要重启的 USV（或点击"全选"）
3. 点击 "🔄 重启选中" 按钮
4. 确认批量重启
5. 系统将依次发送重启命令（间隔 2 秒）

---

## 前置条件

### SSH 免密登录配置

由于重启功能使用 SSH 直接执行 `sudo reboot`，需要配置免密登录：

```bash
# 在地面站生成 SSH 密钥（如果没有）
ssh-keygen -t rsa -b 4096

# 复制公钥到每艘 USV 机载计算机
ssh-copy-id chenhangwei@192.168.68.55  # usv_01
ssh-copy-id chenhangwei@192.168.68.54  # usv_02
ssh-copy-id chenhangwei@192.168.68.52  # usv_03

# 配置 sudo 免密重启（在每艘 USV 上执行）
ssh chenhangwei@192.168.68.55
sudo visudo

# 添加以下行（允许用户无密码执行 reboot 命令）
chenhangwei ALL=(ALL) NOPASSWD: /sbin/reboot
```

### usv_fleet.yaml 配置

确保配置文件包含正确的主机信息：

```yaml
usv_fleet:
  usv_01:
    enabled: true
    hostname: "192.168.68.55"   # 必须：机载计算机 IP
    username: "chenhangwei"      # 必须：SSH 用户名
    # ...
```

---

## 已知限制

1. **重启时间**: 机载计算机重启需要 30-60 秒，期间 USV 将离线
2. **节点终止**: 重启会终止所有运行中的 ROS 节点，需要重新启动
3. **飞控状态**: 重启机载计算机不会影响飞控，飞控将继续运行
4. **SSH 依赖**: 需要配置免密 SSH 登录和 sudo 免密重启权限
5. **MAVLink 备选**: 某些飞控固件可能不支持 MAVLink 重启命令（已改用 SSH 作为主要方式）

---

## 未来改进建议

1. **重启状态监控**: 添加重启进度显示（重启中 → 离线 → 恢复中 → 在线）
2. **自动重启节点**: 重启完成后自动重新启动 USV 节点
3. **重启历史记录**: 记录重启时间和原因
4. **定时重启**: 支持定时自动重启功能（如每天凌晨）
5. **选择性重启**: 支持只重启部分服务而不是整个系统

---

## 相关文件

**修改的文件**:
- `gs_gui/gs_gui/ros_signal.py` - 添加 `reboot_companion` 信号
- `gs_gui/gs_gui/ground_station_node.py` - 实现 `reboot_companion_callback`
- `gs_gui/gs_gui/main_gui_app.py` - 连接信号到回调
- `gs_gui/gs_gui/usv_fleet_launcher.py` - 添加重启按钮和逻辑

**新增功能**:
- 单个 USV 机载计算机重启
- 批量 USV 机载计算机重启
- 重启确认对话框
- 重启日志输出

---

## 参考资料

- **MAVLink 协议**: [MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN](https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
- **MAVROS 文档**: [CommandLong Service](http://wiki.ros.org/mavros)
- **项目架构**: `QUICK_START.md`, `MODULE_ARCHITECTURE.md`

---

**作者**: GitHub Copilot  
**最后更新**: 2025-11-07
