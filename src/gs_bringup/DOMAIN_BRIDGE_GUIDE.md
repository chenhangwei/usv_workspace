# Domain Bridge 使用指南

Domain Bridge 用于实现地面站（Domain 99）与多艘 USV（Domain 11, 12, 13）之间的跨 ROS Domain 通信。

## 📁 文件结构

```
gs_bringup/
├── launch/
│   ├── gs_launch.py                    # 地面站主启动文件（包含 domain_bridge）
│   └── domain_bridge.launch.py         # Domain Bridge 独立启动文件
└── scripts/
    └── domain_bridge.sh                # Domain Bridge 管理脚本

~/domain_bridge/
├── domain_bridge.yaml                  # 当前使用的配置文件
├── domain_bridge.yaml.bak              # 备份文件
└── domain_bridge_enhanced.yaml         # 增强版配置（包含更多话题）
```

## 🚀 快速启动

### 方式 1：随地面站一起启动（推荐）

```bash
# 启动地面站（自动启动 domain_bridge）
ros2 launch gs_bringup gs_launch.py
```

### 方式 2：独立启动

```bash
# 使用 launch 文件
ros2 launch gs_bringup domain_bridge.launch.py

# 指定配置文件
ros2 launch gs_bringup domain_bridge.launch.py \
    config_file:=~/domain_bridge/domain_bridge_enhanced.yaml
```

### 方式 3：使用管理脚本（最方便）

```bash
# 启动（后台运行）
./src/gs_bringup/scripts/domain_bridge.sh start

# 查看状态
./src/gs_bringup/scripts/domain_bridge.sh status

# 查看实时日志（按 Ctrl+A 然后 D 分离）
./src/gs_bringup/scripts/domain_bridge.sh attach

# 停止
./src/gs_bringup/scripts/domain_bridge.sh stop

# 重启
./src/gs_bringup/scripts/domain_bridge.sh restart
```

## ⚙️ 配置说明

### Domain ID 分配

| 设备 | Domain ID | 说明 |
|------|-----------|------|
| 地面站 | 99 | 统一使用 Domain 99 |
| USV_01 | 11 | 第一艘无人船 |
| USV_02 | 12 | 第二艘无人船 |
| USV_03 | 13 | 第三艘无人船 |

### 设置 Domain ID

在地面站运行前设置：

```bash
export ROS_DOMAIN_ID=99
```

建议加入 `~/.bashrc`：

```bash
echo "export ROS_DOMAIN_ID=99" >> ~/.bashrc
source ~/.bashrc
```

### 配置文件示例

基础配置（`domain_bridge.yaml`）：

```yaml
domains:
  - id: 11  # USV_01
  - id: 12  # USV_02
  - id: 13  # USV_03
  - id: 99  # 地面站

rules:
  # USV 状态 -> 地面站
  - topic: "usv_01/status"
    type: "common_interfaces/msg/UsvStatus"
    from_domain: 11
    to_domain: 99

  # 地面站控制 -> USV
  - topic: "usv_01/set_point"
    type: "common_interfaces/msg/UsvSetPoint"
    from_domain: 99
    to_domain: 11
```

增强版配置包含更多话题（MAVROS、TF 等），使用：

```bash
cp ~/domain_bridge/domain_bridge_enhanced.yaml ~/domain_bridge/domain_bridge.yaml
```

## 🔍 验证通信

### 1. 检查 domain_bridge 运行状态

```bash
# 方式 1：使用管理脚本
./src/gs_bringup/scripts/domain_bridge.sh status

# 方式 2：检查进程
ps aux | grep domain_bridge

# 方式 3：检查 screen 会话
screen -ls
```

### 2. 验证话题转发

在地面站（Domain 99）：

```bash
# 设置 Domain ID
export ROS_DOMAIN_ID=99

# 查看 USV 话题（应该能看到 usv_01, usv_02, usv_03 的话题）
ros2 topic list | grep usv_

# 监听 USV 状态
ros2 topic echo /usv_01/status

# 发送控制指令
ros2 topic pub /usv_01/set_point common_interfaces/msg/UsvSetPoint "{...}"
```

### 3. 检查网络连接

```bash
# 测试 USV 网络连通性
ping <usv_ip_address>

# 检查防火墙（如果需要）
sudo ufw status
sudo ufw allow 7400:7500/udp  # ROS DDS 默认端口范围
```

## 📋 后台运行方法

### 方式 1：screen（推荐）

```bash
# 启动新 screen 会话
screen -S domain_bridge

# 运行 domain_bridge
ros2 launch gs_bringup domain_bridge.launch.py

# 分离会话：按 Ctrl+A 然后 D

# 查看所有会话
screen -ls

# 重新连接
screen -r domain_bridge

# 终止会话
screen -S domain_bridge -X quit
```

### 方式 2：nohup

```bash
# 后台运行
nohup ros2 launch gs_bringup domain_bridge.launch.py > ~/domain_bridge.log 2>&1 &

# 查看日志
tail -f ~/domain_bridge.log

# 停止（查找进程 ID 并 kill）
ps aux | grep domain_bridge
kill <PID>
```

### 方式 3：systemd 服务（开机自启）

创建服务文件 `/etc/systemd/system/domain-bridge.service`：

```ini
[Unit]
Description=ROS 2 Domain Bridge
After=network.target

[Service]
Type=simple
User=chenhangwei
Environment="ROS_DOMAIN_ID=99"
WorkingDirectory=/home/chenhangwei/usv_workspace
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source /home/chenhangwei/usv_workspace/install/setup.bash && ros2 launch gs_bringup domain_bridge.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

启用服务：

```bash
sudo systemctl daemon-reload
sudo systemctl enable domain-bridge
sudo systemctl start domain-bridge
sudo systemctl status domain-bridge
```

## 🐛 故障排查

### 问题 1：看不到 USV 话题

**检查项：**
1. 确认地面站 Domain ID 是否正确：`echo $ROS_DOMAIN_ID`
2. 检查 domain_bridge 是否运行：`ps aux | grep domain_bridge`
3. 检查网络连通性：`ping <usv_ip>`
4. 检查配置文件路径是否正确
5. 查看 domain_bridge 日志

**解决方案：**
```bash
# 重启 domain_bridge
./src/gs_bringup/scripts/domain_bridge.sh restart

# 查看实时日志
./src/gs_bringup/scripts/domain_bridge.sh attach
```

### 问题 2：domain_bridge 启动失败

**可能原因：**
- 配置文件不存在或格式错误
- domain_bridge 包未安装
- 端口被占用

**解决方案：**
```bash
# 检查配置文件
cat ~/domain_bridge/domain_bridge.yaml

# 检查包是否安装
ros2 pkg list | grep domain_bridge

# 如果未安装
sudo apt-get install ros-jazzy-domain-bridge

# 检查端口占用
sudo netstat -tulpn | grep 740
```

### 问题 3：性能问题/延迟高

**优化建议：**
1. 减少转发的话题数量（只转发必要的话题）
2. 降低话题发布频率
3. 使用有线网络代替无线
4. 检查网络带宽和延迟

## 📚 参考资料

- [ROS 2 Domain Bridge 官方文档](https://github.com/ros2/domain_bridge)
- [ROS 2 DDS 配置](https://docs.ros.org/en/jazzy/How-To-Guides/DDS-tuning.html)
- [Screen 使用指南](https://www.gnu.org/software/screen/manual/screen.html)

## 🔗 相关文件

- 地面站启动配置：`gs_bringup/launch/gs_launch.py`
- Domain Bridge 启动文件：`gs_bringup/launch/domain_bridge.launch.py`
- 管理脚本：`gs_bringup/scripts/domain_bridge.sh`
- 配置文件：`~/domain_bridge/domain_bridge.yaml`

---

**最后更新：** 2025-11-18  
**维护者：** chenhangwei
