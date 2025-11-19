# Domain 物理隔离架构说明

## 📋 架构概述

本系统采用 **Domain 物理隔离架构**，每艘 USV 运行在独立的 ROS Domain 中，不会在 DDS 层互相广播 discovery，从根源上解决了启动慢和 DDS discovery 风暴问题。

### 架构拓扑

```
┌──────────────────────────────────────────────────────────────┐
│                    地面站 (Domain 99)                         │
│  ┌────────────────┐        ┌──────────────────┐             │
│  │  Ground Station│◄──────►│  Domain Bridge   │             │
│  │   (静态配置)    │        │  (话题转发)      │             │
│  └────────────────┘        └──────────────────┘             │
└────────────────────────────────┬─────────────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │   Domain Bridge 转发     │
                    └────────────┬────────────┘
                                 │
        ┌────────────────────────┼────────────────────────┐
        │                        │                        │
┌───────▼─────────┐    ┌────────▼────────┐    ┌─────────▼───────┐
│ USV_01 (D:11)   │    │ USV_02 (D:12)   │    │ USV_03 (D:13)   │
│  完全隔离        │    │  完全隔离        │    │  完全隔离        │
│                 │    │                 │    │                 │
│  ❌ 无 DDS      │    │  ❌ 无 DDS      │    │  ❌ 无 DDS      │
│     Discovery  │    │     Discovery  │    │     Discovery  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

**关键特性:**
- ✅ 每个 USV 在独立的 Domain (11, 12, 13...) 中运行
- ✅ USV 之间完全隔离，互不感知
- ✅ 地面站在 Domain 99 运行
- ✅ Domain Bridge 负责话题转发（99 ↔ 11/12/13）
- ✅ 启动速度快，无 DDS discovery 开销

---

## 🔄 与旧架构的对比

### 旧架构（动态节点发现）

```python
def update_subscribers_and_publishers(self):
    """通过 DDS 动态发现 USV 节点"""
    # ❌ 问题：在 Domain 隔离下无法工作
    node_names_and_namespaces = self.get_node_names_and_namespaces()
    
    # 筛选 /usv_* 命名空间
    for name, ns in node_names_and_namespaces:
        if ns.startswith('/usv_'):
            # 动态创建订阅者/发布者
            ...
```

**限制:**
- ❌ 依赖 DDS discovery（USV 在不同 Domain 中无法被发现）
- ❌ 启动慢（大量节点互相 discovery）
- ❌ 动态性过高，不稳定

---

### 新架构（静态配置）

```python
def _load_fleet_config(self):
    """从配置文件加载 USV 列表"""
    # ✅ 从 usv_fleet.yaml 读取
    config = yaml.safe_load(open('usv_fleet.yaml'))
    return config.get('usv_fleet', {})

def initialize_usv_from_config(self):
    """静态初始化所有 USV 的订阅/发布者"""
    for usv_id in self._static_usv_list:
        # 为每个 USV 创建订阅者/发布者
        self.usv_manager.add_usv_namespace(f"/{usv_id}")
```

**优势:**
- ✅ 不依赖 DDS discovery，适配 Domain 隔离
- ✅ 启动快速，立即可用
- ✅ 配置明确，可预测
- ✅ 支持离线检测（通过 topic 数据流判断）

---

## ⚙️ 配置文件结构

### usv_fleet.yaml

```yaml
usv_fleet:
  usv_01:
    enabled: true                          # 是否启用
    hostname: "192.168.68.55"              # USV IP 地址
    username: "chenhangwei"                # SSH 用户名
    namespace: "usv_01"                    # ROS 命名空间
    # ... 其他配置
    
  usv_02:
    enabled: true
    hostname: "192.168.68.54"
    username: "chenhangwei"
    namespace: "usv_02"
    
  usv_03:
    enabled: false  # ⚠️ 禁用的 USV 不会被初始化
    hostname: "192.168.68.52"
    username: "chenhangwei"
    namespace: "usv_03"
```

**关键字段:**
- `enabled`: 控制是否初始化该 USV（true/false）
- `namespace`: USV 在 ROS 中的命名空间（必须与 Domain Bridge 配置一致）
- `hostname`: 用于网络连通性检查（可选）

---

## 🚀 启动流程

### 1. 地面站启动

```bash
# 设置地面站 Domain ID
export ROS_DOMAIN_ID=99

# 启动地面站（自动读取 usv_fleet.yaml）
ros2 launch gs_bringup gs_launch.py
```

**内部流程:**
1. 读取 `usv_fleet.yaml`
2. 提取所有 `enabled: true` 的 USV
3. 为每个 USV 静态创建订阅者/发布者
4. 启动 Domain Bridge 进行话题转发
5. 启动 GUI 应用

**日志输出示例:**
```
============================================================
🚀 初始化USV订阅者和发布者（静态配置模式）
============================================================
✓ 已加载fleet配置文件: /path/to/usv_fleet.yaml
  ├─ usv_01 (已启用)
  ├─ usv_02 (已启用)
  ├─ usv_03 (已禁用)
✓ 从配置文件读取到 2 艘USV: ['usv_01', 'usv_02']
✓ usv_01 初始化完成
✓ usv_02 初始化完成
============================================================
✓ 完成初始化 2 艘USV
============================================================
```

### 2. USV 启动（在各 USV 机载计算机上）

```bash
# USV_01 (Domain 11)
ssh chenhangwei@192.168.68.55
export ROS_DOMAIN_ID=11
ros2 launch usv_bringup usv_launch.py

# USV_02 (Domain 12)
ssh chenhangwei@192.168.68.54
export ROS_DOMAIN_ID=12
ros2 launch usv_bringup usv_launch.py
```

**关键点:**
- ✅ 每个 USV 使用独立的 Domain ID
- ✅ USV 之间完全隔离，无 DDS 交互
- ✅ 通过 Domain Bridge 与地面站通信

---

## 🔍 离线检测机制

由于无法使用 DDS 节点发现，新架构通过 **topic 数据流** 判断 USV 在线状态：

### 检测逻辑

```python
def check_usv_topics_availability(self):
    """定期检查 USV topic 是否有数据"""
    offline_threshold = 10.0  # 10秒未收到数据 → 离线
    
    for usv_id in self._static_usv_list:
        last_seen = self._ns_last_seen.get(usv_id, 0.0)
        elapsed = now - last_seen
        
        if elapsed > offline_threshold:
            # 标记为离线（但不删除订阅者）
            self.usv_states[usv_id]['connected'] = False
            self.get_logger().warn(f"⚠️ {usv_id} 可能已离线")
        else:
            # 标记为在线
            self.usv_states[usv_id]['connected'] = True
```

**特点:**
- ✅ 不删除订阅者（USV 重新上线后自动恢复）
- ✅ 使用 `connected` 字段标记状态
- ✅ 检测周期：5 秒
- ✅ 离线阈值：10 秒

---

## 📊 话题转发规则

### Domain Bridge 配置示例

```yaml
# ~/domain_bridge/domain_bridge.yaml
domains:
  - id: 11  # USV_01
  - id: 12  # USV_02
  - id: 13  # USV_03
  - id: 99  # 地面站

rules:
  # USV → 地面站（状态监控）
  - topic: "usv_01/usv_state"
    type: "common_interfaces/msg/UsvStatus"
    from_domain: 11
    to_domain: 99
    
  # 地面站 → USV（控制命令）
  - topic: "usv_01/set_usv_mode"
    type: "std_msgs/msg/String"
    from_domain: 99
    to_domain: 11
```

**转发方向:**
- `USV → 地面站`: 状态、位置、传感器数据
- `地面站 → USV`: 控制命令、目标点、模式切换

---

## 🛠️ 故障排查

### 问题 1: 地面站看不到 USV 数据

**检查清单:**

1. **验证配置文件**
   ```bash
   cat ~/usv_workspace/src/gs_bringup/config/usv_fleet.yaml
   # 确保 enabled: true
   ```

2. **检查地面站日志**
   ```bash
   # 应看到初始化日志
   ✓ 从配置文件读取到 X 艘USV
   ✓ usv_XX 初始化完成
   ```

3. **验证 Domain Bridge**
   ```bash
   # 地面站 (Domain 99)
   export ROS_DOMAIN_ID=99
   ros2 topic list | grep usv_
   # 应看到 /usv_01/usv_state 等话题
   ```

4. **检查 USV Domain ID**
   ```bash
   # 在 USV 上
   echo $ROS_DOMAIN_ID
   # 应输出: 11 (对应 usv_01)
   ```

---

### 问题 2: USV 显示为离线

**可能原因:**
- Domain Bridge 未运行
- USV 实际离线
- 网络不通
- Domain ID 配置错误

**调试步骤:**

1. **检查 topic 数据流**
   ```bash
   # 地面站 (Domain 99)
   export ROS_DOMAIN_ID=99
   ros2 topic echo /usv_01/usv_state --once
   ```

2. **查看离线日志**
   ```bash
   # 地面站日志
   ⚠️ usv_01 可能已离线（10.5s未收到数据）
   ```

3. **测试网络连通性**
   ```bash
   ping 192.168.68.55
   ```

4. **重启 Domain Bridge**
   ```bash
   ./src/gs_bringup/scripts/domain_bridge.sh restart
   ```

---

### 问题 3: 添加新 USV

**步骤:**

1. **编辑配置文件**
   ```yaml
   # usv_fleet.yaml
   usv_04:
     enabled: true
     hostname: "192.168.68.XX"
     username: "chenhangwei"
     namespace: "usv_04"
     # ... 其他配置
   ```

2. **更新 Domain Bridge 配置**
   ```yaml
   # ~/domain_bridge/domain_bridge.yaml
   domains:
     - id: 14  # 新 USV Domain ID
   
   rules:
     # 添加 usv_04 的转发规则
     - topic: "usv_04/usv_state"
       from_domain: 14
       to_domain: 99
   ```

3. **重启地面站和 Domain Bridge**
   ```bash
   # 重启地面站
   ros2 launch gs_bringup gs_launch.py
   
   # 重启 Domain Bridge
   ./src/gs_bringup/scripts/domain_bridge.sh restart
   ```

4. **启动新 USV**
   ```bash
   ssh chenhangwei@192.168.68.XX
   export ROS_DOMAIN_ID=14
   ros2 launch usv_bringup usv_launch.py
   ```

---

## 📚 相关文件

### 配置文件
- **USV 列表**: `gs_bringup/config/usv_fleet.yaml`
- **Domain Bridge**: `~/domain_bridge/domain_bridge.yaml`
- **地面站参数**: `gs_bringup/config/gs_params.yaml`

### 代码文件
- **节点初始化**: `gs_gui/gs_gui/ground_station_node.py`
  - `_load_fleet_config()`: 加载配置
  - `_extract_usv_list_from_config()`: 提取 USV 列表
  - `initialize_usv_from_config()`: 静态初始化
  - `check_usv_topics_availability()`: 离线检测

### Launch 文件
- **地面站启动**: `gs_bringup/launch/gs_launch.py`
- **Domain Bridge**: `gs_bringup/launch/domain_bridge.launch.py`

---

## ✅ 最佳实践

### 1. 配置文件管理

```bash
# 备份配置
cp ~/usv_workspace/src/gs_bringup/config/usv_fleet.yaml \
   ~/usv_workspace/src/gs_bringup/config/usv_fleet.yaml.bak

# 版本控制
git add src/gs_bringup/config/usv_fleet.yaml
git commit -m "Update USV fleet configuration"
```

### 2. Domain ID 规划

| 设备类型 | Domain ID 范围 | 说明 |
|---------|---------------|------|
| 地面站   | 99            | 固定使用 99 |
| USV     | 11-19         | 每艘 USV 独立 Domain |
| 测试环境 | 20-29         | 用于本地测试 |

### 3. 监控脚本

```bash
#!/bin/bash
# monitor_usv.sh - 监控 USV 连接状态

export ROS_DOMAIN_ID=99
source ~/usv_workspace/install/setup.bash

echo "监控 USV 连接状态..."
while true; do
    for usv in usv_01 usv_02 usv_03; do
        if ros2 topic echo /${usv}/usv_state --once &>/dev/null; then
            echo "✓ ${usv} 在线"
        else
            echo "✗ ${usv} 离线"
        fi
    done
    sleep 5
done
```

---

## 🎯 总结

**Domain 物理隔离架构的核心要点:**

1. **静态配置优于动态发现**
   - 从 `usv_fleet.yaml` 读取 USV 列表
   - 启动时一次性初始化所有订阅/发布者

2. **Domain Bridge 是关键**
   - 负责跨 Domain 话题转发
   - 必须正确配置 Domain ID 和转发规则

3. **离线检测改用数据流**
   - 不依赖节点发现
   - 通过 topic 数据判断在线状态

4. **配置文件统一管理**
   - `usv_fleet.yaml`: USV 列表
   - `domain_bridge.yaml`: 话题转发规则

---

**维护者**: chenhangwei  
**最后更新**: 2025-11-18  
**架构版本**: v2.0 (Domain Isolation)
