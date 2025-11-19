# Domain Bridge 部署成功总结

## 配置完成时间
2025-11-18 13:22

## 系统配置

### 网络拓扑
- **地面站**: Domain ID = 99
- **gauss01 (USV_01)**: Domain ID = 11, IP = 192.168.68.55
- **gauss02 (USV_02)**: Domain ID = 12, IP = 192.168.68.54  
- **gauss03 (USV_03)**: Domain ID = 13, IP = 192.168.68.52

### 关键发现
之前的配置文件格式错误!ROS 2 Jazzy 的 domain_bridge 使用不同的 YAML 格式:

**错误格式 (旧版本)**:
```yaml
domains:
  - id: 11
  - id: 99
rules:
  - topic: "usv_01/usv_state"
    type: "common_interfaces/msg/UsvStatus"
    from_domain: 11
    to_domain: 99
```

**正确格式 (Jazzy)**:
```yaml
name: usv_domain_bridge
from_domain: 11
to_domain: 99
topics:
  usv_01/usv_state:
    type: common_interfaces/msg/UsvStatus
```

## 当前状态

### ✅ 成功转发的话题 (13个)
```
/usv_01/battery
/usv_01/global_position/compass_hdg
/usv_01/global_position/global
/usv_01/global_position/raw/fix
/usv_01/global_position/raw/gps_vel
/usv_01/home_position/home
/usv_01/led_state
/usv_01/local_position/pose
/usv_01/local_position/pose_from_gps
/usv_01/local_position/velocity_local
/usv_01/low_voltage_mode
/usv_01/usv_state
/usv_01/usv_temperature
```

### ⚠️ QoS 警告
```
/tf 话题存在 QoS 不兼容问题 (RELIABILITY_QOS_POLICY)
```
需要在配置中调整 TF 的 QoS 设置。

## 使用方法

### 启动地面站
```bash
# 方式 1: 后台运行 (推荐)
cd ~/usv_workspace
source install/setup.bash
export ROS_DOMAIN_ID=99
nohup ros2 run domain_bridge domain_bridge ~/domain_bridge/domain_bridge_usv01.yaml > /tmp/domain_bridge.log 2>&1 &

# 方式 2: 使用 launch 文件 (需要更新配置路径)
ros2 launch gs_bringup gs_launch.py
```

### 启动 USV (在 gauss01 上)
```bash
export ROS_DOMAIN_ID=11
ros2 launch usv_bringup usv_launch.py

# 持久化配置
echo 'export ROS_DOMAIN_ID=11' >> ~/.bashrc
```

### 验证通信
```bash
# 地面站
export ROS_DOMAIN_ID=99
ros2 topic list | grep usv_01

# 查看数据
ros2 topic echo /usv_01/usv_state --once

# 检查数据率
ros2 topic hz /usv_01/local_position/pose
```

## 配置文件位置

- **主配置**: `~/domain_bridge/domain_bridge_usv01.yaml` (USV_01专用)
- **Launch文件**: `~/usv_workspace/src/gs_bringup/launch/domain_bridge.launch.py`
- **地面站Launch**: `~/usv_workspace/src/gs_bringup/launch/gs_launch.py`

## 下一步

### 1. 添加 USV_02 和 USV_03 支持
创建多USV配置文件或使用多个 domain_bridge 实例。

### 2. 修复 TF QoS 问题
更新配置中的 TF 话题 QoS 为 best_effort。

### 3. 添加双向控制话题
当前只配置了状态转发,需要添加从地面站到 USV 的控制命令转发。

### 4. 性能优化
- 监控网络带宽usage
- 调整话题转发优先级
- 配置 QoS 策略以适应网络条件

## 故障排查

### 问题: 地面站看不到 USV 话题
1. 检查网络连通性: `ping 192.168.68.55`
2. 检查 Domain ID: `echo $ROS_DOMAIN_ID`
3. 检查 domain_bridge 进程: `ps aux | grep domain_bridge`
4. 查看日志: `cat /tmp/domain_bridge.log`

### 问题: Domain Bridge 不断重启
- 原因: 配置文件格式错误
- 解决: 使用 `domain_bridge_usv01.yaml` (新格式)

### 问题: 话题存在但无数据
- 检查 USV 端是否正在发布: `ros2 topic hz /usv_01/usv_state` (在 gauss01 上,Domain 11)
- 检查 QoS 兼容性: 查看 domain_bridge 日志中的 QoS 警告

## 参考命令

```bash
# 诊断脚本
~/usv_workspace/src/gs_bringup/scripts/diagnose_domain_bridge.sh

# 停止 domain_bridge
pkill -f domain_bridge

# 查看官方示例
cat /opt/ros/jazzy/share/domain_bridge/examples/example_bridge_config.yaml
```

## 成功标志
- ✅ 网络连通 (gauss01/02/03 可 ping)
- ✅ USV 在 Domain 11 运行
- ✅ 地面站在 Domain 99 运行
- ✅ domain_bridge 进程稳定运行
- ✅ 地面站可见 13+ 个 usv_01 话题
- ✅ 防火墙规则已配置 (gauss01)

---
**部署完成!跨域通信已建立!** 🚀
