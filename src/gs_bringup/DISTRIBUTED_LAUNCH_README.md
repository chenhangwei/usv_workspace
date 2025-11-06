# ROS 2 分布式启动 - 快速开始

## 🚀 一键启动

### 方法 1：使用快速启动脚本（推荐）

```bash
cd ~/usv_workspace
./src/gs_bringup/scripts/quick_start.sh
```

然后按提示选择：
1. **分布式启动** - 通过 SSH 启动远程 USV（生产环境）
2. **本地测试** - 在本机启动多个 USV（开发测试）
3. **仅地面站** - 只启动地面站 GUI

### 方法 2：直接使用 ros2 launch

```bash
cd ~/usv_workspace
source install/setup.bash

# 分布式启动（推荐）
ros2 launch gs_bringup gs_distributed_launch.py

# 本地测试
ros2 launch gs_bringup gs_local_multi_usv.py

# 仅地面站
ros2 launch gs_bringup gs_launch.py
```

## 📝 配置集群

编辑 `src/gs_bringup/config/usv_fleet.yaml`：

```yaml
usv_fleet:
  usv_01:
    enabled: true                          # 是否启用
    hostname: "192.168.68.101"             # USV IP
    username: "usv"                        # SSH 用户名
    workspace: "/home/usv/usv_workspace"   # 工作空间路径
    namespace: "usv_01"                    # ROS 命名空间
    fcu_url: "serial:///dev/ttyACM0:921600" # 飞控串口
    system_id: 1                           # MAVLink ID
```

## 🔧 前提条件

### 地面站
- ✅ ROS 2 已安装并 source
- ✅ usv_workspace 已编译
- ✅ 网络连接到所有 USV

### USV 机载（每艘）
- ✅ ROS 2 已安装
- ✅ usv_workspace 已编译
- ✅ SSH 服务已启用
- ✅ 已配置免密 SSH 登录

### 配置免密 SSH

```bash
# 在地面站执行
ssh-keygen -t rsa -b 4096
ssh-copy-id usv@192.168.68.101  # 每艘 USV
ssh usv@192.168.68.101 "echo 'SSH 成功'"  # 测试
```

## 📚 完整文档

详细配置和故障排查请参考：
- **完整指南**：[DISTRIBUTED_LAUNCH_GUIDE.md](../DISTRIBUTED_LAUNCH_GUIDE.md)
- **架构分析**：[NAMESPACE_ISOLATION_ANALYSIS.md](../NAMESPACE_ISOLATION_ANALYSIS.md)

## 🎯 工作流程

1. **配置集群** → 编辑 `usv_fleet.yaml`
2. **配置 SSH** → 免密登录到所有 USV
3. **启动系统** → 运行 `quick_start.sh` 或 `ros2 launch`
4. **验证状态** → 检查地面站 GUI 在线列表
5. **控制 USV** → 发送命令，执行任务

## ⚠️ 注意事项

1. **网络要求**：所有机器必须在同一网络或可路由
2. **时钟同步**：建议使用 NTP 同步时间
3. **版本一致**：所有机器使用相同的 ROS 2 版本和代码
4. **防火墙**：确保 ROS 2 DDS 端口（默认 7400-7999）开放

## 🐛 常见问题

### 问题：SSH 连接失败
```bash
# 检查网络
ping 192.168.68.101

# 测试 SSH
ssh usv@192.168.68.101
```

### 问题：节点未出现
```bash
# 检查 ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# 检查节点列表
ros2 node list

# 检查 topic
ros2 topic list | grep usv
```

### 问题：远程日志不显示
编辑 `usv_fleet.yaml`：
```yaml
launch_options:
  show_remote_output: true
```

---

**更新日期：** 2025-11-06  
**版本：** 1.0
