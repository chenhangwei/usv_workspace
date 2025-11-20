# USV 跨域通信快速启动指南

## 系统概览

- **地面站**: Domain 99, 运行 GUI 和 Domain Bridge
- **USV_01 (gauss01)**: Domain 11, IP 192.168.68.55
- **USV_02 (gauss02)**: Domain 12, IP 192.168.68.54
- **USV_03 (gauss03)**: Domain 13, IP 192.168.68.52

## 启动步骤

### 1. 启动地面站 (本机)

```bash
cd ~/usv_workspace
source install/setup.bash
ros2 launch gs_bringup gs_launch.py
```

**预期输出**:
```
[INFO] [main_gui_app-1]: process started with pid [xxxxx]
[INFO] [domain_bridge-2]: process started with pid [xxxxx]
```

### 2. 启动 USV (在 gauss01 上)

```bash
# SSH 到 gauss01
ssh chenhangwei@192.168.68.55

# 设置 Domain ID
export ROS_DOMAIN_ID=11

# 启动 USV
cd ~/usv_workspace
source install/setup.bash
ros2 launch usv_bringup usv_launch.py
```

**持久化配置** (可选):
```bash
echo 'export ROS_DOMAIN_ID=11' >> ~/.bashrc
```

### 3. 验证通信

在**地面站**执行:

```bash
# 检查话题
export ROS_DOMAIN_ID=99
ros2 topic list | grep usv_01

# 应该看到 13 个话题:
# /usv_01/battery
# /usv_01/global_position/compass_hdg
# /usv_01/global_position/global
# /usv_01/global_position/raw/fix
# /usv_01/global_position/raw/gps_vel
# /usv_01/home_position/home
# /usv_01/led_state
# /usv_01/local_position/pose
# /usv_01/local_position/pose_from_gps
# /usv_01/local_position/velocity_local
# /usv_01/low_voltage_mode
# /usv_01/usv_state
# /usv_01/usv_temperature

# 查看实时数据
ros2 topic echo /usv_01/usv_state

# 检查位置更新频率
ros2 topic hz /usv_01/local_position/pose
```

## 快速诊断

### 检查网络连通性
```bash
ping 192.168.68.55  # gauss01
ping 192.168.68.54  # gauss02
ping 192.168.68.52  # gauss03
```

### 检查进程状态
```bash
# 地面站
ps aux | grep -E "domain_bridge|main_gui" | grep -v grep

# USV (在 gauss01 上)
ps aux | grep -E "mavros|usv" | grep -v grep
```

### 运行诊断脚本
```bash
# 地面站
~/usv_workspace/src/gs_bringup/scripts/diagnose_domain_bridge.sh

# USV
~/usv_workspace/src/usv_bringup/scripts/verify_cross_domain.sh
```

## 常见问题

### Q: 地面站看不到 USV 话题

**检查清单**:
1. ✓ 网络通: `ping 192.168.68.55`
2. ✓ USV Domain ID: 在 gauss01 上 `echo $ROS_DOMAIN_ID` 应该是 `11`
3. ✓ GS Domain ID: 在地面站 `echo $ROS_DOMAIN_ID` 应该是 `99`
4. ✓ domain_bridge 运行: `ps aux | grep domain_bridge`
5. ✓ USV 已启动: 在 gauss01 上 `ros2 topic list | grep usv_01`

**解决方案**:
```bash
# 重启 domain_bridge
pkill -f domain_bridge
cd ~/usv_workspace && source install/setup.bash
ros2 launch gs_bringup domain_bridge.launch.py
```

### Q: MAVROS 未连接

**症状**: `/usv_01/mavros/state` 显示 `connected: false`

**解决方案**:
```bash
# 检查飞控连接 (在 gauss01 上)
ls -l /dev/ttyACM0  # 或 /dev/ttyUSB0

# 检查 MAVROS 日志
ros2 topic echo /usv_01/mavros/state --once

# 重启 USV 系统
# Ctrl+C 停止,然后重新启动
ros2 launch usv_bringup usv_launch.py
```

### Q: QoS 警告

**警告消息**:
```
[WARN] New publisher/subscription discovered on topic '/tf', offering incompatible QoS
```

**说明**: 这是正常的!不同节点对 `/tf` 话题使用不同的 QoS 设置。只要话题能正常转发就没问题。

## 进阶操作

### 发送控制命令

从地面站控制 USV:

```bash
# 设置目标位置 (本地坐标系,单位:米)
export ROS_DOMAIN_ID=99
ros2 topic pub /usv_01/set_usv_target_position common_interfaces/msg/UsvSetPoint \
  "{x: 10.0, y: 5.0, z: 0.0, yaw: 0.0}" --once

# 切换模式
ros2 topic pub /usv_01/set_usv_mode std_msgs/msg/String \
  "{data: 'GUIDED'}" --once

# 解锁
ros2 topic pub /usv_01/set_usv_arming std_msgs/msg/Bool \
  "{data: true}" --once
```

### 监控性能

```bash
# 查看所有话题频率
export ROS_DOMAIN_ID=99
ros2 topic hz /usv_01/local_position/pose &
ros2 topic hz /usv_01/usv_state &

# 查看网络延迟
ros2 topic echo /usv_01/usv_state --once | grep "stamp"

# 查看 domain_bridge 日志
tail -f /tmp/domain_bridge.log
```

### 多 USV 部署

重复上述步骤,为其他 USV 设置不同的 Domain ID:

- **gauss02**: `export ROS_DOMAIN_ID=12`
- **gauss03**: `export ROS_DOMAIN_ID=13`

然后创建对应的配置文件:
- `~/domain_bridge/domain_bridge_usv02.yaml`
- `~/domain_bridge/domain_bridge_usv03.yaml`

## 配置文件

- **Domain Bridge**: `~/domain_bridge/domain_bridge_usv01.yaml`
- **USV 参数**: `~/usv_workspace/src/usv_bringup/config/usv_params.yaml`
- **地面站参数**: `~/usv_workspace/src/gs_bringup/config/gs_params.yaml`

## 有用的命令

```bash
# 列出所有 ROS 节点
ros2 node list

# 查看节点信息
ros2 node info /usv_01/mavros_node

# 检查话题类型
ros2 topic type /usv_01/usv_state

# 查看消息定义
ros2 interface show common_interfaces/msg/UsvStatus

# 记录数据 (用于调试)
ros2 bag record /usv_01/usv_state /usv_01/local_position/pose
```

## 停止系统

```bash
# 地面站
Ctrl+C  # 停止 gs_launch.py

# USV (在 gauss01 上)
Ctrl+C  # 停止 usv_launch.py

# 强制停止所有进程
pkill -f "mavros|domain_bridge|main_gui"
```

---

**准备就绪!开始使用 USV 系统!** 🚀

如有问题,参考:
- Copilot指南: `.github/copilot-instructions.md`
- USV优雅关机: `src/USV_GRACEFUL_SHUTDOWN_GUIDE.md`
