# Domain Bridge 完整部署指南

本指南说明如何在 USV 集群系统中部署和使用 Domain Bridge 实现跨域通信。

---

## 📋 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      地面站 (Domain 99)                      │
│  ┌────────────────┐        ┌──────────────────┐            │
│  │  Ground Station│◄──────►│  Domain Bridge   │            │
│  │      GUI       │        │                  │            │
│  └────────────────┘        └──────────────────┘            │
└────────────────────────────────┬────────────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │      网络 (WiFi/以太网)   │
                    └────────────┬────────────┘
                                 │
        ┌────────────────────────┼────────────────────────┐
        │                        │                        │
┌───────▼─────────┐    ┌────────▼────────┐    ┌─────────▼───────┐
│ USV_01 (D:11)   │    │ USV_02 (D:12)   │    │ USV_03 (D:13)   │
│   gauss01       │    │   gauss02       │    │   gauss03       │
│                 │    │                 │    │                 │
│ ┌─────────────┐ │    │ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │   MAVROS    │ │    │ │   MAVROS    │ │    │ │   MAVROS    │ │
│ │   控制节点  │ │    │ │   控制节点  │ │    │ │   控制节点  │ │
│ │   状态节点  │ │    │ │   状态节点  │ │    │ │   状态节点  │ │
│ └─────────────┘ │    │ └─────────────┘ │    │ └─────────────┘ │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

**Domain ID 分配:**
- 地面站: `99`
- USV_01 (gauss01): `11`
- USV_02 (gauss02): `12`
- USV_03 (gauss03): `13`

---

## 🚀 部署步骤

### 1️⃣ 地面站配置

#### 步骤 1: 设置 Domain ID

```bash
# 临时设置
export ROS_DOMAIN_ID=99

# 永久设置（推荐）
echo 'export ROS_DOMAIN_ID=99' >> ~/.bashrc
source ~/.bashrc
```

#### 步骤 2: 验证配置文件

```bash
# 检查配置文件
cat ~/domain_bridge/domain_bridge.yaml

# 应该包含 75 条转发规则
grep -c "topic:" ~/domain_bridge/domain_bridge.yaml
```

#### 步骤 3: 启动地面站

```bash
cd ~/usv_workspace
source install/setup.bash

# 启动地面站（自动启动 domain_bridge）
ros2 launch gs_bringup gs_launch.py
```

#### 步骤 4: 验证地面站

打开新终端：

```bash
export ROS_DOMAIN_ID=99
source ~/usv_workspace/install/setup.bash

# 运行验证脚本
./src/gs_bringup/scripts/verify_domain_bridge.sh
```

**预期输出:**
```
✓ ROS_DOMAIN_ID = 99
✓ domain_bridge 正在运行
✓ 配置文件存在: 75 条话题转发规则
✓ 发现 2 个 ROS 节点 (domain_bridge, main_gui_app)
```

---

### 2️⃣ USV 配置（在每艘 USV 上执行）

#### USV_01 (gauss01)

```bash
# SSH 登录到 gauss01
ssh chenhangwei@gauss01

# 设置 Domain ID
echo 'export ROS_DOMAIN_ID=11' >> ~/.bashrc
source ~/.bashrc

# 验证
echo $ROS_DOMAIN_ID  # 应输出: 11
```

#### USV_02 (gauss02)

```bash
# SSH 登录到 gauss02
ssh chenhangwei@gauss02

# 设置 Domain ID
echo 'export ROS_DOMAIN_ID=12' >> ~/.bashrc
source ~/.bashrc

# 验证
echo $ROS_DOMAIN_ID  # 应输出: 12
```

#### USV_03 (gauss03)

```bash
# SSH 登录到 gauss03
ssh chenhangwei@gauss03

# 设置 Domain ID
echo 'export ROS_DOMAIN_ID=13' >> ~/.bashrc
source ~/.bashrc

# 验证
echo $ROS_DOMAIN_ID  # 应输出: 13
```

---

### 3️⃣ 启动 USV 节点

#### 在 USV 上启动（以 gauss01 为例）

```bash
cd ~/usv_workspace
source install/setup.bash

# 确认 Domain ID
echo $ROS_DOMAIN_ID  # 应输出: 11

# 启动 USV
ros2 launch usv_bringup usv_launch.py
```

#### 验证 USV

打开新终端：

```bash
export ROS_DOMAIN_ID=11  # 根据 USV 调整
source ~/usv_workspace/install/setup.bash

# 运行验证脚本
./src/usv_bringup/scripts/test_usv_domain.sh
```

**预期输出:**
```
✓ ROS_DOMAIN_ID = 11
✓ 发现 12 个 USV 节点
✓ /usv_01/usv_state
✓ /usv_01/mavros/state
✓ /usv_01/mavros/local_position/pose
✓ /usv_01/set_usv_target_position
```

---

## 🔍 验证跨域通信

### 在地面站验证

```bash
# 设置地面站 Domain ID
export ROS_DOMAIN_ID=99
source ~/usv_workspace/install/setup.bash

# 查看所有 USV 话题（通过 Domain Bridge 转发）
ros2 topic list | grep usv_

# 应该看到类似输出：
# /usv_01/usv_state
# /usv_01/mavros/state
# /usv_01/mavros/local_position/pose
# /usv_01/mavros/global_position/global
# /usv_01/mavros/battery
# ... (更多话题)

# 监听 USV 状态
ros2 topic echo /usv_01/usv_state

# 查看话题频率
ros2 topic hz /usv_01/usv_state
```

### 发送控制命令测试

```bash
# 地面站 (Domain 99)
export ROS_DOMAIN_ID=99

# 切换模式到 GUIDED
ros2 topic pub /usv_01/set_usv_mode std_msgs/msg/String "data: 'GUIDED'" --once

# 解锁 USV
ros2 topic pub /usv_01/set_usv_arming std_msgs/msg/String "data: 'ARM'" --once

# 发送目标位置（示例）
ros2 topic pub /usv_01/set_usv_target_position geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}" --once
```

### 在 USV 端验证（可选）

```bash
# 在 USV (Domain 11)
export ROS_DOMAIN_ID=11

# 监听来自地面站的命令
ros2 topic echo /usv_01/set_usv_mode
ros2 topic echo /usv_01/set_usv_target_position
```

---

## 🛠️ 故障排查

### 问题 1: 地面站看不到 USV 话题

**症状:**
```bash
# 地面站 (Domain 99)
ros2 topic list | grep usv_
# 无输出
```

**排查步骤:**

1. **检查 USV Domain ID**
   ```bash
   # 在 USV 上
   echo $ROS_DOMAIN_ID
   # 应输出: 11 (对于 gauss01)
   ```

2. **检查 domain_bridge 运行状态**
   ```bash
   # 在地面站
   ps aux | grep domain_bridge
   ```

3. **检查网络连通性**
   ```bash
   # 从地面站 ping USV
   ping gauss01
   ```

4. **检查防火墙**
   ```bash
   # 允许 ROS DDS 端口
   sudo ufw allow 7400:7500/udp
   ```

5. **查看 domain_bridge 日志**
   ```bash
   # 查看最近的日志
   journalctl -u domain-bridge -f
   
   # 或者查看终端输出
   ros2 launch gs_bringup domain_bridge.launch.py
   ```

**解决方案:**
- 确保 USV 设置了正确的 `ROS_DOMAIN_ID`
- 重启 domain_bridge: `./src/gs_bringup/scripts/domain_bridge.sh restart`
- 检查配置文件: `cat ~/domain_bridge/domain_bridge.yaml`

---

### 问题 2: USV 未设置 Domain ID

**症状:**
USV 启动后，domain_bridge 看不到话题。

**解决方案:**

```bash
# 在 USV 上
# 1. 停止 USV 节点
pkill -f usv_launch

# 2. 设置 Domain ID
export ROS_DOMAIN_ID=11  # 根据 USV 调整

# 3. 重新启动
ros2 launch usv_bringup usv_launch.py

# 4. 验证
./src/usv_bringup/scripts/test_usv_domain.sh
```

---

### 问题 3: 控制命令无响应

**症状:**
地面站发送命令，USV 无响应。

**排查:**

1. **检查 USV 控制节点**
   ```bash
   # 在 USV 上
   ros2 node list | grep command
   # 应显示: /usv_01/usv_command_node
   ```

2. **检查话题订阅**
   ```bash
   # 在 USV 上
   ros2 topic info /usv_01/set_usv_mode
   # 应显示订阅者
   ```

3. **手动测试**
   ```bash
   # 在 USV 本地测试
   export ROS_DOMAIN_ID=11
   ros2 topic pub /usv_01/set_usv_mode std_msgs/msg/String "data: 'GUIDED'" --once
   ```

---

### 问题 4: 延迟或丢包

**症状:**
话题更新缓慢，或数据丢失。

**优化建议:**

1. **降低高频话题频率**
   - 编辑 USV launch 文件，降低发布频率

2. **使用有线网络**
   - WiFi 延迟高，推荐使用以太网

3. **检查网络带宽**
   ```bash
   # 监控带宽使用
   iftop
   ```

4. **减少转发话题**
   - 编辑 `~/domain_bridge/domain_bridge.yaml`
   - 注释掉不需要的话题规则

---

## 📊 性能监控

### 监控命令

```bash
# 地面站 (Domain 99)
export ROS_DOMAIN_ID=99

# 1. 监控话题频率
ros2 topic hz /usv_01/usv_state
ros2 topic hz /usv_01/mavros/local_position/pose

# 2. 监控带宽
ros2 topic bw /usv_01/mavros/local_position/pose

# 3. 查看延迟
ros2 topic echo /usv_01/usv_state/header/stamp --once

# 4. 监控节点
ros2 node list
ros2 node info /domain_bridge
```

### 性能基准

| 话题 | 预期频率 | 带宽 |
|------|---------|------|
| `usv_state` | 1 Hz | ~1 KB/s |
| `mavros/state` | 10 Hz | ~2 KB/s |
| `local_position/pose` | 50 Hz | ~10 KB/s |
| `global_position/global` | 10 Hz | ~2 KB/s |

---

## 🔧 管理脚本

### 地面站管理

```bash
# 启动
ros2 launch gs_bringup gs_launch.py

# 单独启动 domain_bridge
ros2 launch gs_bringup domain_bridge.launch.py

# 使用管理脚本
./src/gs_bringup/scripts/domain_bridge.sh start
./src/gs_bringup/scripts/domain_bridge.sh status
./src/gs_bringup/scripts/domain_bridge.sh stop
./src/gs_bringup/scripts/domain_bridge.sh restart

# 验证
./src/gs_bringup/scripts/verify_domain_bridge.sh
```

### USV 管理

```bash
# 启动 USV
ros2 launch usv_bringup usv_launch.py

# 验证
./src/usv_bringup/scripts/test_usv_domain.sh
```

---

## 📚 相关文档

- **Domain Bridge 配置**: `~/domain_bridge/domain_bridge.yaml`
- **话题分析**: `src/gs_bringup/USV_TOPICS_ANALYSIS.md`
- **使用指南**: `src/gs_bringup/DOMAIN_BRIDGE_GUIDE.md`

---

## ✅ 快速检查清单

### 地面站

- [ ] `ROS_DOMAIN_ID=99` 已设置
- [ ] domain_bridge 正在运行
- [ ] 配置文件存在 (75 条规则)
- [ ] main_gui_app 正在运行
- [ ] 能看到 domain_bridge 节点

### USV

- [ ] `ROS_DOMAIN_ID=11/12/13` 已设置（根据 USV）
- [ ] USV 节点正在运行
- [ ] MAVROS 已连接飞控
- [ ] 能看到 USV 话题
- [ ] 网络连接正常

### 通信验证

- [ ] 地面站能看到 USV 话题
- [ ] 能监听 `/usv_XX/usv_state`
- [ ] 能发送控制命令
- [ ] USV 能收到地面站命令
- [ ] 延迟可接受 (<100ms)

---

**维护者**: chenhangwei  
**最后更新**: 2025-11-18
