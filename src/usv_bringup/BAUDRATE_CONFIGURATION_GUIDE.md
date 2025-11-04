# 波特率配置指南

## 概述

本文档说明如何为不同使用场景选择和配置 MAVROS 串口波特率。波特率选择直接影响：
- 参数同步成功率
- 任务上传/下载稳定性
- 遥测数据实时性
- 通信链路可靠性

---

## 快速推荐

| 使用场景 | 推荐波特率 | 配置示例 |
|----------|-----------|----------|
| **参数配置** | 115200 | `fcu_url:=serial:///dev/ttyACM0:115200` |
| **任务规划** | 115200 | 同上 |
| **高速遥测** | 921600 | `fcu_url:=serial:///dev/ttyACM0:921600` |
| **长线缆（>2m）** | 57600 | `fcu_url:=serial:///dev/ttyACM0:57600` |
| **无线数传** | 57600 | 根据数传模块规格 |

---

## 波特率详细对比

### 57600 baud - 最稳定

**优点**：
- ⭐ **极高稳定性**：适合长线缆（>2 米）或低质量 USB 线
- ⭐ **无错误率**：参数同步几乎从不失败
- 适合嘈杂电磁环境

**缺点**：
- 遥测数据刷新较慢（~5-10 Hz）
- 参数同步时间较长（~3-5 分钟）

**推荐场景**：
- 长 USB 延长线连接飞控
- 户外现场调试（电磁干扰多）
- 首次参数同步或完整参数备份

**配置示例**：
```bash
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_01 \
    fcu_url:=serial:///dev/ttyACM0:57600
```

---

### 115200 baud - 推荐默认值 ⭐

**优点**：
- ⭐ **平衡速度和稳定性**：ArduPilot 官方推荐值
- ⭐ **参数同步可靠**：丢包率 <0.1%
- 遥测数据足够快（~10-20 Hz）
- 兼容大多数 USB 串口和线缆

**缺点**：
- 极长线缆（>3 米）可能偶尔出错
- 高速日志下载较慢

**推荐场景**：
- **参数配置窗口操作**（最推荐）
- 任务上传/下载
- 日常开发和调试
- 标准 USB 线缆（<2 米）

**配置示例**：
```bash
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_01 \
    fcu_url:=serial:///dev/ttyACM0:115200
```

---

### 921600 baud - 高速但不稳定

**优点**：
- 遥测数据高速（~50 Hz）
- 日志下载快

**缺点**：
- ❌ **参数同步易出错**：丢包率 ~3%（实测案例：30/900 参数丢失）
- ❌ **协议错误**：频繁出现 `idx=65535` 无效索引错误
- 对 USB 线缆质量要求极高（必须屏蔽线，<1 米）
- 长线缆或 USB Hub 下几乎无法工作

**不推荐场景**：
- ❌ 参数配置（极易超时）
- ❌ 任务规划（上传可能失败）
- ❌ 初始系统配置

**可用场景**（仅当必须高速时）：
- 短 USB 线直连（<50cm 优质屏蔽线）
- 纯遥测数据流（不涉及参数/任务）
- 高速日志下载（但不稳定）

**配置示例**（不推荐）：
```bash
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_01 \
    fcu_url:=serial:///dev/ttyACM0:921600
```

---

## 实际诊断案例

### 案例 1: USV 03 参数服务超时（921600 → 115200）

**问题描述**：
```bash
# 使用 921600 baud 时
$ ros2 service call /usv_03/mavros/param/pull mavros_msgs/srv/ParamPull
waiting for service to become available...
^C  # 无限等待
```

**MAVROS 日志**：
```
PR: request param #874 timeout, retries left 0, and 30 params still missing
PR: got an unsolicited param value idx=65535, not resetting retries count
```

**根本原因**：
- 921600 baud 导致 ~3.3% 参数丢失率（30/900）
- `idx=65535` 表明 MAVLink 协议层通信错误
- USB 串口在高速下产生 CRC 校验失败

**解决方案**：
```bash
# 改用 115200 baud 重新启动
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_03 \
    fcu_url:=serial:///dev/ttyACM0:115200

# 等待 2 分钟参数同步完成
ros2 topic echo /rosout | grep "parameters successfully synchronized"

# 现在参数服务正常工作
ros2 service call /usv_03/mavros/param/pull mavros_msgs/srv/ParamPull
# ✅ 成功返回
```

**结果**：
- 参数丢失率：3.3% → 0%
- 同步时间：未完成 → 2 分钟
- 协议错误：频繁 → 无

---

## 修改波特率方法

### 方法 1: 启动参数（推荐）

每次启动时指定：
```bash
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_01 \
    fcu_url:=serial:///dev/ttyACM0:115200
```

**优点**：不修改源码，灵活切换

### 方法 2: 修改默认值

编辑 `usv_bringup/launch/usv_launch.py`：

```python
fcu_url_arg = DeclareLaunchArgument(
    'fcu_url',
    default_value='serial:///dev/ttyACM0:115200',  # ⬅️ 修改这里
    description='MAVLink connection URL'
)
```

重新构建：
```bash
cd ~/usv_workspace
colcon build --packages-select usv_bringup
source install/setup.bash
```

**优点**：一次修改，永久生效

### 方法 3: 参数文件（不推荐）

在 `usv_params.yaml` 中添加：
```yaml
mavros:
  ros__parameters:
    fcu_url: "serial:///dev/ttyACM0:115200"
```

**缺点**：MAVROS 启动参数优先级高于配置文件，可能不生效

---

## 飞控端配置

### ArduPilot 参数

确保飞控端串口波特率匹配：

| 参数名 | 推荐值 | 说明 |
|--------|--------|------|
| `SERIAL0_BAUD` | 115 | USB 串口（数值 115 = 115200 baud） |
| `SERIAL1_BAUD` | 57 | 遥测 1（数传，57 = 57600） |
| `SERIAL2_BAUD` | 57 | 遥测 2 |

**配置方法**：
1. 使用 QGroundControl 或 Mission Planner
2. 在 Config/Parameters 中搜索 `SERIAL0_BAUD`
3. 设置为 `115`（表示 115200）
4. **Write Parameters** 并重启飞控

**注意**：ArduPilot 波特率参数是枚举值，不是实际数值：
- `57` = 57600
- `115` = 115200
- `921` = 921600

---

## 故障排查

### 问题 1: 修改波特率后无法连接

**症状**：
```bash
ros2 topic echo /usv_01/mavros/state
# connected: false
```

**可能原因**：
1. 飞控端波特率未匹配修改
2. 串口设备路径错误
3. 权限不足

**解决**：
```bash
# 1. 检查串口设备
ls -l /dev/ttyACM0

# 2. 检查权限
groups  # 应包含 dialout

# 3. 确认飞控 SERIAL0_BAUD 参数
# 使用 QGroundControl 查看

# 4. 尝试标准波特率
ros2 launch usv_bringup usv_launch.py \
    fcu_url:=serial:///dev/ttyACM0:57600
```

### 问题 2: 参数同步仍然超时

**可能原因**：
1. 波特率仍太高（试试 57600）
2. USB 线缆质量差或太长
3. 飞控繁忙（PreArm 检查等）

**解决**：
```bash
# 1. 进一步降低波特率
ros2 launch usv_bringup usv_launch.py \
    fcu_url:=serial:///dev/ttyACM0:57600

# 2. 更换短的优质 USB 线（<1 米）

# 3. 禁用 PreArm 检查（临时）
# 在飞控参数中设置 ARMING_CHECK=0

# 4. 等待更长时间（5 分钟）
```

---

## 最佳实践

### 1. 分场景使用不同波特率

**开发/调试阶段**：
```bash
# 使用 115200 获得稳定性和合理速度
ros2 launch usv_bringup usv_launch.py \
    fcu_url:=serial:///dev/ttyACM0:115200
```

**参数完整备份**：
```bash
# 使用 57600 确保 100% 可靠
ros2 launch usv_bringup usv_launch.py \
    fcu_url:=serial:///dev/ttyACM0:57600
```

**高速数据采集**（可选）：
```bash
# 仅在必须时使用 921600（不建议）
ros2 launch usv_bringup usv_launch.py \
    fcu_url:=serial:///dev/ttyACM0:921600
```

### 2. 首次系统配置流程

```bash
# Step 1: 使用 57600 首次同步参数（最稳定）
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_01 \
    fcu_url:=serial:///dev/ttyACM0:57600

# Step 2: 等待参数同步完成（3-5 分钟）
ros2 topic echo /rosout | grep "parameters successfully"

# Step 3: 打开参数窗口配置
# 在地面站 GUI 中操作

# Step 4: 日常使用切换到 115200
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_01 \
    fcu_url:=serial:///dev/ttyACM0:115200
```

### 3. 避免波特率陷阱

❌ **不要做**：
- 用 921600 进行参数配置
- 用 57600 进行高速日志下载
- 在长 USB 线（>2 米）下用高波特率
- 在 USB Hub 下用 921600

✅ **应该做**：
- 默认使用 115200（除非有特殊需求）
- 参数操作前确认波特率合适
- 长线缆场景降到 57600
- 定期检查通信错误率

---

## 相关文档

- **MAVROS 启动优化**: `MAVROS_STARTUP_OPTIMIZATION.md`
- **参数服务故障排查**: `../gs_gui/PARAM_SERVICE_TROUBLESHOOTING.md`
- **快速开始指南**: `../QUICK_START.md`

---

**最后更新**: 2025-11-04  
**实测硬件**: ArduPilot Rover 4.x + Pixhawk 4 Mini + USB 串口  
**推荐配置**: 115200 baud（参数/任务）+ 57600 baud（无线遥测）
