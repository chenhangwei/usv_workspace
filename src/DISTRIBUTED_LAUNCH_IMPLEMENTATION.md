# ROS 2 分布式启动实现总结

## 📋 实现内容

本次实现了**完全基于 ROS 2 原生机制**的分布式启动方案，允许从地面站通过 SSH 远程启动多艘 USV 的机载节点。

---

## 🎯 核心特性

### ✅ ROS 2 原生实现
- 使用标准 `launch.actions.ExecuteProcess` 执行 SSH 命令
- 无需第三方工具（Ansible、Docker Swarm 等）
- 符合 ROS 2 设计理念

### ✅ 配置文件驱动
- `usv_fleet.yaml` 集中管理所有 USV 配置
- 支持动态启用/禁用 USV
- 易于扩展和维护

### ✅ 自动化管理
- 一键启动整个集群
- 自动处理 SSH 连接
- 统一停止（Ctrl+C）

### ✅ 灵活部署
- 支持分布式部署（生产环境）
- 支持本地测试（开发环境）
- 支持混合部署

---

## 📁 新增文件清单

### 1. 核心 Launch 文件

| 文件 | 用途 | 位置 |
|------|------|------|
| `gs_distributed_launch.py` | 分布式启动主文件 | `gs_bringup/launch/` |
| `gs_local_multi_usv.py` | 本地多 USV 测试 | `gs_bringup/launch/` |

### 2. 配置文件

| 文件 | 用途 | 位置 |
|------|------|------|
| `usv_fleet.yaml` | USV 集群配置 | `gs_bringup/config/` |

### 3. 文档

| 文件 | 用途 | 位置 |
|------|------|------|
| `DISTRIBUTED_LAUNCH_GUIDE.md` | 完整使用指南 | `src/` |
| `DISTRIBUTED_LAUNCH_README.md` | 快速开始文档 | `gs_bringup/` |

### 4. 辅助脚本

| 文件 | 用途 | 位置 |
|------|------|------|
| `quick_start.sh` | 快速启动脚本 | `gs_bringup/scripts/` |

---

## 🔧 使用方法

### 快速开始（3 步）

#### Step 1: 配置 SSH 免密登录

```bash
# 在地面站执行
ssh-keygen -t rsa -b 4096
ssh-copy-id usv@192.168.68.101  # USV 01
ssh-copy-id usv@192.168.68.102  # USV 02
ssh-copy-id usv@192.168.68.103  # USV 03
```

#### Step 2: 编辑集群配置

编辑 `src/gs_bringup/config/usv_fleet.yaml`：

```yaml
usv_fleet:
  usv_01:
    enabled: true
    hostname: "192.168.68.101"
    username: "usv"
    workspace: "/home/usv/usv_workspace"
    namespace: "usv_01"
    fcu_url: "serial:///dev/ttyACM0:921600"
    system_id: 1
```

#### Step 3: 启动系统

```bash
cd ~/usv_workspace
source install/setup.bash

# 方法 1：使用快速脚本
./src/gs_bringup/scripts/quick_start.sh

# 方法 2：直接 launch
ros2 launch gs_bringup gs_distributed_launch.py
```

---

## 🏗️ 技术架构

### 工作流程

```
地面站启动
    ↓
加载 usv_fleet.yaml
    ↓
为每个启用的 USV 创建 ExecuteProcess
    ├─> SSH 连接 USV 01 (192.168.68.101)
    │   └─> ros2 launch usv_bringup usv_launch.py namespace:=usv_01
    │
    ├─> SSH 连接 USV 02 (192.168.68.102)
    │   └─> ros2 launch usv_bringup usv_launch.py namespace:=usv_02
    │
    └─> SSH 连接 USV 03 (192.168.68.103)
        └─> ros2 launch usv_bringup usv_launch.py namespace:=usv_03
    ↓
启动地面站 GUI (本地)
    ↓
ROS 2 自动节点发现（5-10秒）
    ↓
系统就绪，可以控制
```

### SSH 远程命令

实际执行的 SSH 命令（示例 USV 01）：

```bash
ssh -o StrictHostKeyChecking=no \
    -o ConnectTimeout=10 \
    -t \
    usv@192.168.68.101 \
    "source /opt/ros/$ROS_DISTRO/setup.bash && \
     source /home/usv/usv_workspace/install/setup.bash && \
     ros2 launch usv_bringup usv_launch.py \
       namespace:=usv_01 \
       fcu_url:=serial:///dev/ttyACM0:921600 \
       tgt_system:=1"
```

### ROS 2 通信机制

- **节点发现**：通过 DDS (Data Distribution Service) 自动发现
- **跨机器通信**：ROS 2 原生支持（无需额外配置）
- **命名空间隔离**：每个 USV 运行在独立命名空间

---

## 📊 与其他方案对比

| 方案 | 优点 | 缺点 | 评分 |
|------|------|------|------|
| **ROS 2 原生 Launch**（本方案） | • ROS 2 原生<br>• 无额外依赖<br>• 配置简单<br>• 统一管理 | • 需要 SSH 配置<br>• 网络要求较高 | ⭐⭐⭐⭐⭐ |
| Ansible | • 功能强大<br>• 支持复杂部署 | • 需要学习 Ansible<br>• 配置复杂<br>• 非 ROS 2 原生 | ⭐⭐⭐⭐ |
| Docker Swarm | • 容器化<br>• 资源隔离 | • 配置复杂<br>• 资源开销大<br>• 非 ROS 2 原生 | ⭐⭐⭐ |
| 手动 SSH | • 最简单 | • 无法统一管理<br>• 操作繁琐<br>• 易出错 | ⭐⭐ |

---

## 🎓 核心代码解析

### 1. 加载配置文件

```python
def load_fleet_config(config_file):
    """加载 USV 集群配置"""
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config
```

### 2. 创建 SSH 命令

```python
def create_ssh_launch_command(usv_config):
    """创建 SSH 远程启动命令"""
    remote_cmd = (
        f"source /opt/ros/$ROS_DISTRO/setup.bash && "
        f"source {workspace}/install/setup.bash && "
        f"ros2 launch usv_bringup usv_launch.py "
        f"namespace:={namespace} "
        f"fcu_url:={fcu_url} "
        f"tgt_system:={system_id}"
    )
    
    ssh_cmd = [
        'ssh', '-o', 'StrictHostKeyChecking=no',
        '-o', 'ConnectTimeout=10', '-t',
        f'{username}@{hostname}',
        remote_cmd
    ]
    return ssh_cmd
```

### 3. 创建远程进程

```python
usv_process = ExecuteProcess(
    cmd=ssh_cmd,
    name=f'{usv_id}_remote_launch',
    output='screen',  # 输出到终端
    shell=False,
)

# 添加启动延迟
usv_process = TimerAction(
    period=current_delay,
    actions=[usv_process]
)
```

### 4. 注册事件处理

```python
# 进程启动事件
start_event = RegisterEventHandler(
    OnProcessStart(
        target_action=usv_process,
        on_start=[LogInfo(msg=f"启动 {usv_id}...")]
    )
)

# 进程退出事件
exit_event = RegisterEventHandler(
    OnProcessExit(
        target_action=usv_process,
        on_exit=[LogInfo(msg=f"{usv_id} 已退出")]
    )
)
```

---

## ✅ 验证清单

### 编译和安装

```bash
cd ~/usv_workspace
colcon build --packages-select gs_bringup
source install/setup.bash
```

### 功能验证

- [ ] **配置文件加载**
  ```bash
  cat src/gs_bringup/config/usv_fleet.yaml
  ```

- [ ] **SSH 连接测试**
  ```bash
  ssh usv@192.168.68.101 "echo 'SSH OK'"
  ```

- [ ] **本地测试启动**
  ```bash
  ros2 launch gs_bringup gs_local_multi_usv.py
  # 检查节点：ros2 node list
  ```

- [ ] **分布式启动**（需要实际硬件）
  ```bash
  ros2 launch gs_bringup gs_distributed_launch.py
  # 检查远程节点：ros2 node list | grep usv
  ```

- [ ] **地面站 GUI**
  - 在线列表显示所有 USV
  - 可以选择 USV 并查看状态
  - 可以发送控制命令

---

## 🔄 后续优化建议

### 1. 错误重试机制

```python
# 在 ExecuteProcess 失败后自动重试
max_retries = 3
for i in range(max_retries):
    try:
        # SSH 启动
        break
    except:
        if i < max_retries - 1:
            time.sleep(5)
            continue
```

### 2. 健康检查

```python
# 启动后检查节点是否在线
def check_usv_health(namespace):
    nodes = get_node_names_and_namespaces()
    return any(ns == f'/{namespace}' for _, ns in nodes)
```

### 3. 日志聚合

```python
# 将远程日志保存到文件
usv_process = ExecuteProcess(
    cmd=ssh_cmd,
    output='log',
    log_file=f'/tmp/{usv_id}.log'
)
```

### 4. Web 控制面板

- 开发 Web UI 管理 USV 集群
- 实时监控各 USV 状态
- 远程启停和参数配置

---

## 📚 相关文档

- **完整使用指南**：`DISTRIBUTED_LAUNCH_GUIDE.md`
- **命名空间隔离分析**：`NAMESPACE_ISOLATION_ANALYSIS.md`
- **快速开始**：`gs_bringup/DISTRIBUTED_LAUNCH_README.md`

---

## 🎉 总结

本次实现提供了一个**完全基于 ROS 2 原生机制**的分布式启动方案，具有以下特点：

1. **✅ 符合 ROS 2 设计理念**：使用标准 launch 系统
2. **✅ 易于使用**：配置文件驱动，一键启动
3. **✅ 灵活部署**：支持本地测试和远程部署
4. **✅ 统一管理**：从地面站控制整个集群
5. **✅ 可扩展性强**：轻松添加新 USV

这是**最 ROS 2 的方案**，推荐作为生产环境的标准部署方式！

---

**实现日期：** 2025-11-06  
**实现者：** AI Agent (GitHub Copilot)  
**版本：** 1.0
