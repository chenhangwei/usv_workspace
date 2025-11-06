# ROS 2 分布式启动 - 完整指南

## 📋 概述

本指南介绍如何使用 ROS 2 原生的分布式 launch 机制，从地面站通过 SSH 远程启动多艘 USV 的机载节点。这是最符合 ROS 2 设计理念的部署方式。

**核心优势：**
- ✅ **原生 ROS 2**：使用标准 launch 系统，无需额外工具
- ✅ **集中管理**：地面站统一启动/停止所有 USV
- ✅ **自动化**：配置文件驱动，一键启动整个集群
- ✅ **灵活性**：支持动态增减 USV，支持本地/远程混合部署
- ✅ **可观测性**：远程输出可转发到地面站终端

---

## 🏗️ 架构设计

### 系统拓扑

```
地面站 (192.168.68.53)
    │
    ├─ gs_gui (本地)
    │   └─ ROS 2 节点：main_gui_app
    │
    └─ SSH 远程启动 ──┬─> USV 01 (192.168.68.101)
                      │    └─ ros2 launch usv_bringup usv_launch.py namespace:=usv_01
                      │
                      ├─> USV 02 (192.168.68.102)
                      │    └─ ros2 launch usv_bringup usv_launch.py namespace:=usv_02
                      │
                      └─> USV 03 (192.168.68.103)
                           └─ ros2 launch usv_bringup usv_launch.py namespace:=usv_03
```

### ROS 2 通信

- **网络模式**：DDS 多播（默认）或单播
- **节点发现**：自动（通过 ROS_DOMAIN_ID）
- **Topic/Service**：跨机器透明通信

---

## 🚀 快速开始

### 1. 前提条件

#### 地面站要求
- ✅ ROS 2 Humble/Iron 已安装
- ✅ usv_workspace 已编译
- ✅ 网络连接到所有 USV 机载计算机

#### USV 机载要求（每艘）
- ✅ ROS 2 Humble/Iron 已安装
- ✅ usv_workspace 已编译（与地面站版本一致）
- ✅ SSH 服务已启用
- ✅ 飞控已连接（串口或网络）

### 2. 配置免密 SSH 登录

**在地面站执行：**

```bash
# 生成 SSH 密钥（如果没有）
ssh-keygen -t rsa -b 4096

# 将公钥复制到每艘 USV
ssh-copy-id usv@192.168.68.101  # USV 01
ssh-copy-id usv@192.168.68.102  # USV 02
ssh-copy-id usv@192.168.68.103  # USV 03

# 测试免密登录
ssh usv@192.168.68.101 "echo 'SSH 连接成功'"
```

### 3. 编辑集群配置文件

**文件：** `gs_bringup/config/usv_fleet.yaml`

```yaml
usv_fleet:
  usv_01:
    enabled: true                                    # 启用 USV 01
    hostname: "192.168.68.101"                       # USV IP 地址
    username: "usv"                                  # SSH 用户名
    workspace: "/home/usv/usv_workspace"             # 工作空间路径
    namespace: "usv_01"                              # ROS 命名空间
    fcu_url: "serial:///dev/ttyACM0:921600"          # 飞控串口
    system_id: 1                                     # MAVLink ID
    gcs_url: "udp://:14540@192.168.68.53:14550"      # 地面站地址（可选）
    
  usv_02:
    enabled: true
    hostname: "192.168.68.102"
    # ... 类似配置
    
  usv_03:
    enabled: false  # 禁用 USV 03（如果暂时不用）
    # ...
```

**重要参数说明：**

| 参数 | 说明 | 示例 |
|------|------|------|
| `enabled` | 是否启用该 USV | `true` / `false` |
| `hostname` | USV 机载计算机 IP | `192.168.68.101` |
| `username` | SSH 登录用户名 | `usv` |
| `workspace` | 机载 ROS 工作空间路径 | `/home/usv/usv_workspace` |
| `namespace` | ROS 命名空间（必须唯一） | `usv_01`, `usv_02` |
| `fcu_url` | 飞控连接 URL | `serial:///dev/ttyACM0:921600` |
| `system_id` | MAVLink 系统 ID | `1`, `2`, `3` |
| `gcs_url` | 地面站 MAVLink 地址（可选） | `udp://:14540@192.168.68.53:14550` |

### 4. 启动分布式系统

#### 方法 1：一键启动（推荐）

```bash
cd ~/usv_workspace
source install/setup.bash

# 启动地面站 + 所有启用的 USV
ros2 launch gs_bringup gs_distributed_launch.py
```

**执行流程：**
1. 加载 `usv_fleet.yaml` 配置
2. 启动地面站 GUI（本地）
3. 通过 SSH 依次启动各 USV 节点（远程）
4. 等待 ROS 2 节点发现（约 5-10 秒）
5. 地面站 GUI 显示在线 USV

#### 方法 2：指定配置文件

```bash
ros2 launch gs_bringup gs_distributed_launch.py \
    fleet_config:=/path/to/custom_fleet.yaml \
    gs_param_file:=/path/to/custom_params.yaml
```

#### 方法 3：本地测试（无需 SSH）

```bash
# 在单台电脑上启动多个 USV（用于调试）
ros2 launch gs_bringup gs_local_multi_usv.py
```

---

## 📊 验证和监控

### 1. 检查节点在线状态

```bash
# 查看所有节点
ros2 node list

# 应该看到：
# /main_gui_app                    (地面站)
# /usv_01/usv_status_node          (USV 01)
# /usv_01/usv_control_node
# /usv_01/mavros
# /usv_02/usv_status_node          (USV 02)
# ...
```

### 2. 检查 Topic 通信

```bash
# 查看 USV 状态 topic
ros2 topic list | grep usv_state

# 应该看到：
# /usv_01/usv_state
# /usv_02/usv_state
# /usv_03/usv_state

# 监听某个 USV 的状态
ros2 topic echo /usv_01/usv_state
```

### 3. 检查远程进程

在地面站终端，你会看到远程 USV 的启动日志：

```
[INFO] [launch]: 正在启动 usv_01 @ 192.168.68.101...
[usv_01_remote_launch-1] [INFO] [usv_status_node]: 初始化无人船状态节点
[usv_01_remote_launch-1] [INFO] [mavros]: MAVROS started. MY ID 1.1
...
```

### 4. 地面站 GUI 验证

- 打开地面站 GUI
- 在"在线列表"中应该看到所有启用的 USV
- 可以选择 USV 并查看状态
- 可以发送控制命令测试

---

## ⚙️ 高级配置

### 1. 自定义启动参数

**场景：不同 USV 使用不同的参数文件**

修改 `gs_distributed_launch.py`：

```python
# 在 create_ssh_launch_command 中添加
remote_cmd = (
    f"source /opt/ros/$ROS_DISTRO/setup.bash && "
    f"source {workspace}/install/setup.bash && "
    f"ros2 launch usv_bringup usv_launch.py "
    f"namespace:={namespace} "
    f"fcu_url:={fcu_url} "
    f"tgt_system:={system_id} "
    f"param_file:={workspace}/config/{namespace}_params.yaml"  # ← 自定义参数
)
```

### 2. 调整启动延迟

**场景：网络带宽有限，避免同时启动多个 USV**

修改 `usv_fleet.yaml`：

```yaml
launch_options:
  launch_delay: 5.0  # 每个 USV 间隔 5 秒启动（默认 2 秒）
```

### 3. 单播 DDS（避免多播问题）

**场景：网络不支持多播或跨子网**

在每台机器的 `~/.bashrc` 中添加：

```bash
# 地面站（192.168.68.53）
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER=192.168.68.53:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_super_client.xml

# USV 01（192.168.68.101）
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER=192.168.68.53:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_client.xml
```

**创建 `fastdds_super_client.xml`（地面站）：**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="super_client_profile" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                        <discoveryServersList>
                            <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                                <metatrafficUnicastLocatorList>
                                    <locator>
                                        <udpv4>
                                            <address>192.168.68.53</address>
                                            <port>11811</port>
                                        </udpv4>
                                    </locator>
                                </metatrafficUnicastLocatorList>
                            </RemoteServer>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

### 4. 启用/禁用特定 USV

**临时禁用某艘 USV：**

```bash
# 编辑配置文件
vim ~/usv_workspace/src/gs_bringup/config/usv_fleet.yaml

# 修改
usv_03:
  enabled: false  # ← 改为 false

# 重新启动
ros2 launch gs_bringup gs_distributed_launch.py
```

---

## 🐛 故障排查

### 问题 1：SSH 连接失败

**现象：**
```
[ERROR] [usv_01_remote_launch]: ssh: connect to host 192.168.68.101 port 22: Connection refused
```

**解决方案：**
```bash
# 1. 检查网络连通性
ping 192.168.68.101

# 2. 检查 SSH 服务
ssh usv@192.168.68.101

# 3. 检查防火墙
sudo ufw status
sudo ufw allow 22/tcp

# 4. 重启 SSH 服务（在 USV 上）
sudo systemctl restart ssh
```

### 问题 2：节点未出现在地面站

**现象：** 地面站 GUI 的在线列表为空

**解决方案：**
```bash
# 1. 检查 ROS_DOMAIN_ID 是否一致
echo $ROS_DOMAIN_ID  # 地面站
ssh usv@192.168.68.101 "echo \$ROS_DOMAIN_ID"  # USV 01

# 2. 检查节点是否启动
ros2 node list

# 3. 检查网络连接
ros2 topic list  # 应该看到 /usv_XX/... topics

# 4. 手动测试通信
ros2 topic echo /usv_01/usv_state
```

### 问题 3：远程输出未显示

**现象：** 看不到远程 USV 的日志

**解决方案：**

修改 `usv_fleet.yaml`：
```yaml
launch_options:
  show_remote_output: true  # ← 确保为 true
```

或在 launch 文件中修改：
```python
usv_process = ExecuteProcess(
    cmd=ssh_cmd,
    output='screen',  # ← 改为 screen
    # ...
)
```

### 问题 4：工作空间路径错误

**现象：**
```
[usv_01] bash: ros2: command not found
```

**解决方案：**

检查 `usv_fleet.yaml` 中的 `workspace` 路径：
```yaml
usv_01:
  workspace: "/home/usv/usv_workspace"  # ← 确保路径正确
```

在 USV 上验证：
```bash
ssh usv@192.168.68.101 "ls /home/usv/usv_workspace/install"
# 应该看到 setup.bash
```

---

## 📚 与传统方式对比

| 特性 | 传统方式（手动 SSH） | ROS 2 分布式 Launch |
|------|---------------------|---------------------|
| **启动方式** | 每台机器手动启动 | 地面站一键启动 |
| **配置管理** | 分散在各机器 | 集中在地面站配置文件 |
| **进程管理** | 手动 kill | Ctrl+C 统一停止 |
| **日志查看** | 需登录各机器 | 可转发到地面站 |
| **故障恢复** | 手动重启 | 自动重试（可配置） |
| **扩展性** | 需修改多处 | 只需修改配置文件 |
| **ROS 2 原生** | ❌ | ✅ |

---

## 🎯 最佳实践

### 1. 网络规划

- **静态 IP**：为每台机器分配固定 IP
- **网络隔离**：USV 专用网络（避免与外网冲突）
- **带宽预留**：视频流、传感器数据需要足够带宽

### 2. 版本同步

- **ROS 版本**：所有机器使用相同的 ROS 2 发行版
- **代码版本**：使用 Git 同步 `usv_workspace`
- **依赖版本**：MAVROS、传感器驱动等保持一致

### 3. 配置备份

```bash
# 备份配置文件
cp gs_bringup/config/usv_fleet.yaml \
   gs_bringup/config/usv_fleet.yaml.backup

# 版本控制
git add gs_bringup/config/usv_fleet.yaml
git commit -m "Update fleet configuration"
```

### 4. 分阶段部署

1. **本地测试**：先用 `gs_local_multi_usv.py` 测试
2. **单机远程**：测试一艘 USV 的 SSH 启动
3. **多机远程**：逐步增加 USV 数量
4. **完整集群**：所有 USV 同时启动

---

## 🔄 更新和维护

### 代码更新流程

```bash
# 在地面站
cd ~/usv_workspace
git pull
colcon build
source install/setup.bash

# 同步到所有 USV
for i in 101 102 103; do
    ssh usv@192.168.68.$i "cd ~/usv_workspace && git pull && colcon build"
done

# 重启系统
ros2 launch gs_bringup gs_distributed_launch.py
```

### 添加新 USV

1. 在新机器上部署 `usv_workspace`
2. 配置 SSH 免密登录
3. 编辑 `usv_fleet.yaml` 添加配置
4. 重新启动系统

---

## 📖 参考资料

- **ROS 2 Launch 系统**：https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- **SSH 免密登录**：https://www.ssh.com/academy/ssh/copy-id
- **FastDDS Discovery Server**：https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html
- **本项目架构文档**：`NAMESPACE_ISOLATION_ANALYSIS.md`

---

**文档更新日期：** 2025-11-06  
**版本：** 1.0  
**维护者：** AI Agent (GitHub Copilot)
