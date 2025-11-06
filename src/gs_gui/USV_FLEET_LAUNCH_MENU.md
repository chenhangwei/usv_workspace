# GUI 菜单栏启动 USV 集群功能说明

**更新日期**: 2025-11-06  
**功能**: 在地面站 GUI 菜单栏中添加"启动 USV 集群"选项

---

## 功能概述

在地面站 GUI 主窗口菜单栏新增 **"USV控制"** 菜单，提供一键启动整个 USV 集群的功能，替代手动在终端执行 `ros2 launch` 命令。

### 菜单位置

```
主窗口菜单栏
├── USV控制(U)            ← 新增
│   └── 🚀 启动 USV 集群   (快捷键: Ctrl+L)
├── 坐标系设置
├── LED设置
└── 工具(T)
```

---

## 使用方法

### 1. 启动地面站 GUI

```bash
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

### 2. 通过菜单启动 USV 集群

**方式一：菜单点击**
- 点击菜单栏 **"USV控制" → "🚀 启动 USV 集群"**

**方式二：快捷键**
- 按 `Ctrl+L`

### 3. 确认启动

系统会弹出确认对话框：
```
确定要启动所有 USV 节点吗？

将通过 SSH 远程启动 usv_fleet.yaml 中配置的所有 USV。
请确保已配置 SSH 免密登录。
```

点击 **"Yes"** 开始启动。

### 4. 查看启动状态

信息栏会显示启动进度：
```
==================================================
🚀 开始启动 USV 集群...
==================================================
✅ 分布式 launch 已启动 (PID: 12345)
等待 USV 节点上线，请查看终端输出...
提示: 可在终端中查看详细日志
✅ 分布式 launch 进程运行正常
请等待 USV 出现在在线列表中...
```

### 5. 验证 USV 上线

- 查看 GUI 中的 **"在线列表"**，等待 USV 出现
- 查看 **集群表格** 或 **离群表格**，确认 USV 状态更新

---

## 工作原理

### 后台执行流程

1. **读取配置**：从 `gs_bringup/config/usv_fleet.yaml` 读取 USV 列表
2. **SSH 连接**：通过免密 SSH 连接到每艘 USV 的机载电脑
3. **远程启动**：在远程机器上执行 `ros2 launch usv_bringup usv_launch.py`
4. **分阶段启动**：每艘 USV 间隔 3 秒启动，避免网络冲突

### 核心代码逻辑

```python
def launch_usv_fleet(self):
    """启动 USV 集群（通过分布式 launch）"""
    # 1. 确认对话框
    reply = QMessageBox.question(...)
    
    # 2. 查找工作空间路径
    workspace_path = self._find_workspace_path()
    
    # 3. 构建 launch 命令
    launch_cmd = [
        'bash', '-c',
        f'source {setup_script} && '
        f'ros2 launch gs_bringup gs_distributed_launch.py'
    ]
    
    # 4. 后台启动进程（非阻塞）
    process = subprocess.Popen(launch_cmd, ...)
    
    # 5. 定时检查进程状态
    QTimer.singleShot(3000, lambda: self._check_launch_process(process))
```

---

## 前置条件

### ✅ 必需配置

1. **SSH 免密登录已配置**
   ```bash
   # 在地面站生成密钥
   ssh-keygen -t rsa -b 4096
   
   # 复制公钥到所有 USV
   ssh-copy-id chenhangwei@192.168.68.55  # USV 01
   ssh-copy-id chenhangwei@192.168.68.54  # USV 02
   ssh-copy-id chenhangwei@192.168.68.52  # USV 03
   ```

2. **usv_fleet.yaml 已配置**
   ```yaml
   # gs_bringup/config/usv_fleet.yaml
   usv_01:
     enabled: true
     hostname: "192.168.68.55"
     username: "chenhangwei"
     workspace: "/home/chenhangwei/usv_workspace"
     mavlink_sys_id: 1
     fcu_url: "serial:///dev/ttyACM0:921600"
   # ... USV 02, 03 配置 ...
   ```

3. **gs_bringup 包已编译**
   ```bash
   cd ~/usv_workspace
   colcon build --packages-select gs_bringup
   source install/setup.bash
   ```

4. **USV 机载电脑在线**
   - 所有 USV 机载电脑已开机
   - 网络连接正常（能 ping 通）
   - ROS 2 环境已安装

### ⚠️ 验证清单

运行以下命令验证配置：

```bash
# 1. 测试 SSH 免密登录
ssh chenhangwei@192.168.68.55 "hostname && exit"  # 应直接返回主机名
ssh chenhangwei@192.168.68.54 "hostname && exit"
ssh chenhangwei@192.168.68.52 "hostname && exit"

# 2. 检查 fleet 配置
cat ~/usv_workspace/src/gs_bringup/config/usv_fleet.yaml

# 3. 验证 launch 文件存在
ls ~/usv_workspace/install/gs_bringup/share/gs_bringup/launch/gs_distributed_launch.py
```

---

## 故障排查

### 问题 1: "未找到 install 目录"

**原因**: gs_bringup 包未编译

**解决方案**:
```bash
cd ~/usv_workspace
colcon build --packages-select gs_bringup
source install/setup.bash
# 重新启动 GUI
```

### 问题 2: "SSH 连接超时"

**原因**: 
- USV 机载电脑未开机
- 网络不通
- IP 地址配置错误

**解决方案**:
```bash
# 测试网络连通性
ping 192.168.68.55
ping 192.168.68.54
ping 192.168.68.52

# 检查 usv_fleet.yaml 中的 IP 配置
# 确保与实际 USV 的 IP 一致
```

### 问题 3: "SSH 提示输入密码"

**原因**: 免密登录未配置

**解决方案**:
```bash
# 重新配置公钥
ssh-copy-id chenhangwei@192.168.68.55
ssh-copy-id chenhangwei@192.168.68.54
ssh-copy-id chenhangwei@192.168.68.52

# 验证免密登录
ssh chenhangwei@192.168.68.55 "echo OK"  # 应直接输出 OK
```

### 问题 4: "Launch 进程意外退出"

**原因**:
- USV 机载工作空间路径错误
- ROS 2 环境未安装或版本不匹配
- launch 文件有错误

**调试方法**:
```bash
# 手动在终端测试分布式 launch
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_distributed_launch.py

# 查看详细错误信息（在终端中）
```

### 问题 5: "USV 节点未上线"

**检查步骤**:
```bash
# 1. 在地面站检查 ROS 节点
ros2 node list | grep usv

# 2. SSH 到 USV 机载电脑检查进程
ssh chenhangwei@192.168.68.55
ps aux | grep ros2

# 3. 查看 USV 机载日志
ssh chenhangwei@192.168.68.55 "tail -n 50 ~/.ros/log/*/rosout.log"
```

---

## 手动启动对比

### 传统方式（多终端操作）

**地面站**:
```bash
# 终端 1
ros2 launch gs_bringup gs_launch.py
```

**USV 01**:
```bash
# 终端 2: SSH 到 USV 01
ssh chenhangwei@192.168.68.55
cd ~/usv_workspace
source install/setup.bash
ros2 launch usv_bringup usv_launch.py namespace:=usv_01 fcu_url:=serial:///dev/ttyACM0:921600
```

**USV 02**:
```bash
# 终端 3: SSH 到 USV 02
ssh chenhangwei@192.168.68.54
cd ~/usv_workspace
source install/setup.bash
ros2 launch usv_bringup usv_launch.py namespace:=usv_02 fcu_url:=serial:///dev/ttyACM1:921600
```

**USV 03**:
```bash
# 终端 4: SSH 到 USV 03
ssh chenhangwei@192.168.68.52
cd ~/usv_workspace
source install/setup.bash
ros2 launch usv_bringup usv_launch.py namespace:=usv_03 fcu_url:=serial:///dev/ttyACM2:921600
```

➡️ **需要 4 个终端，手动操作 4 次**

---

### 新方式（GUI 菜单栏）

1. 启动地面站 GUI
2. 点击 **"USV控制" → "🚀 启动 USV 集群"** 或按 `Ctrl+L`
3. 点击确认

➡️ **只需 1 次点击，自动完成所有 USV 启动**

---

## 技术细节

### 文件修改清单

**1. `gs_gui/main_gui_app.py`**
- 导入 `subprocess` 模块
- 添加 `_init_custom_menu()` 中的 USV 控制菜单
- 连接信号 `self.action_launch_usv_fleet.triggered.connect(self.launch_usv_fleet)`
- 实现 `launch_usv_fleet()` 方法
- 实现 `_check_launch_process()` 方法

### 依赖关系

```
MainWindow (GUI)
    ↓ 用户点击菜单
launch_usv_fleet()
    ↓ 查找工作空间
workspace_path = ~/usv_workspace
    ↓ 构建命令
subprocess.Popen(['bash', '-c', 'source ... && ros2 launch gs_bringup gs_distributed_launch.py'])
    ↓ 后台执行
gs_distributed_launch.py
    ↓ 读取配置
usv_fleet.yaml
    ↓ SSH 远程启动
ssh chenhangwei@192.168.68.55 "cd ~/usv_workspace && source ... && ros2 launch usv_bringup usv_launch.py ..."
    ↓ USV 节点上线
ROS 2 DDS 自动发现
    ↓ 地面站检测到
GUI 在线列表更新
```

---

## 进阶使用

### 自定义启动参数

如需修改启动参数（如超时时间、启动间隔），编辑：

```bash
nano ~/usv_workspace/src/gs_bringup/launch/gs_distributed_launch.py
```

关键参数：
```python
# 启动间隔（秒）
startup_delay = 3.0  # 每艘 USV 之间延迟 3 秒

# SSH 超时时间
ssh_timeout = 30  # 30 秒后认为 SSH 失败
```

### 仅启动部分 USV

编辑 `usv_fleet.yaml`，将不需要启动的 USV 的 `enabled` 设为 `false`：

```yaml
usv_01:
  enabled: true   # 启动
usv_02:
  enabled: false  # 不启动
usv_03:
  enabled: true   # 启动
```

### 查看详细日志

在地面站启动 GUI 的终端中，可以看到 `gs_distributed_launch.py` 的详细输出：

```bash
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
# GUI 启动后，点击菜单启动 USV 集群
# 终端会显示 SSH 连接和远程启动的详细信息
```

---

## 相关文档

- **分布式 Launch 实现**: `gs_bringup/DISTRIBUTED_LAUNCH_GUIDE.md`
- **Fleet 配置说明**: `gs_bringup/FLEET_CONFIG_GUIDE.md`
- **SSH 免密登录设置**: `gs_bringup/SSH_SETUP_GUIDE.md`
- **快速启动脚本**: `gs_bringup/scripts/quick_start.sh`

---

## 总结

**优势**:
- ✅ 一键启动整个集群，节省操作时间
- ✅ 无需手动 SSH 到每艘 USV
- ✅ 图形化界面，操作直观
- ✅ 实时反馈启动状态
- ✅ 快捷键支持（Ctrl+L）

**适用场景**:
- 日常测试和开发
- 演示和展示
- 快速部署多机集群

**限制**:
- 需要预先配置 SSH 免密登录
- 所有 USV 必须在网络中可达
- 首次使用需熟悉配置文件结构

---

**维护者**: GitHub Copilot  
**版本**: 1.0.0  
**最后更新**: 2025-11-06
