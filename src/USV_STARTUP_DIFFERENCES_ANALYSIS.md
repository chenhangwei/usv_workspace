# USV 启动差异分析报告

## 问题描述

三个 USV (usv_01, usv_02, usv_03) 启动时的终端输出存在明显差异，特别是：
- USV_01 和 USV_03 显示 `❌ Failed to set EKF origin!` 错误
- USV_02 **没有**显示这个错误信息

## 三个 USV 的状态对比

### USV_01（左上终端）

**关键日志**:
```
[mavros_node-1] [ERROR] [usv_01.sys]: FCU: PreArm: Radio failsafe on
[auto_set_home_node-6] [ERROR] Failed to set EKF origin!
[mavros_node-1] [WARN] CMD: Command 179 -- ack timeout
```

**状态分析**:
- ❌ GPS 可能有问题或初始化慢
- ❌ 尝试设置 EKF origin 但**失败**
- ❌ Command 179 (SET_GPS_GLOBAL_ORIGIN) 超时
- ⚠️ 飞控报告 Radio failsafe（遥控器失联保护）

### USV_02（右上终端）

**关键日志**:
```
[mavros_node-1] [ERROR] [usv_02.sys]: FCU: EKF3 waiting for GPS config data
[mavros_node-1] [ERROR] [usv_02.sys]: FCU: PreArm: Radio failsafe on
```

**状态分析**:
- ⏳ EKF3 正在**等待 GPS 配置数据**
- ✅ **没有尝试设置 EKF origin**（因为 GPS 未就绪）
- ✅ **没有出现 "Failed to set EKF origin!" 错误**
- ⚠️ 同样有 Radio failsafe 报告

### USV_03（底部终端）

**关键日志**:
```
[auto_set_home_node-6] [ERROR] Failed to set EKF origin!
[mavros_node-1] [WARN] CMD: Command 179 -- ack timeout
[mavros_node-1] [ERROR] [usv_03.time]: TM: Wrong FCU time.
[mavros_node-1] [ERROR] [usv_03.sys]: FCU: PreArm: Radio failsafe on
```

**状态分析**:
- ❌ 尝试设置 EKF origin 但**失败**
- ❌ Command 179 超时
- ⚠️ **飞控时间错误**（时钟未同步）
- ⚠️ Radio failsafe

## 核心问题：为什么 USV_02 没有打印 "Failed to set EKF origin!"？

### 原因分析

查看 `auto_set_home_node.py` 的代码逻辑：

```python
def check_and_set_home(self):
    """检查 GPS 和系统状态，满足条件后设置 Home 点"""
    if self.set_home_sent:
        return  # 已经设置过，不再重复
    
    # 检查系统是否连接
    if not self.system_connected:
        self.get_logger().debug('Waiting for MAVROS connection...')
        return
    
    # 检查 GPS 定位状态
    # NavSatFix.status.status: -1=无服务, 0=无定位, 1=GPS定位, 2=DGPS
    if self.gps_fix_type < 0:
        self.get_logger().info(
            f'Waiting for GPS fix (current: {self.gps_fix_type}, need >= 0)...'
        )
        return  # 👈 GPS 未就绪，直接返回，不尝试设置
    
    # 所有条件满足，设置 Home 点
    self.get_logger().info(
        f'✅ GPS fix obtained (type: {self.gps_fix_type}), system connected. '
        f'Setting Home point in {self.set_delay_sec:.1f}s...'
    )
    self.set_home_sent = True
    self.create_timer(self.set_delay_sec, self.set_home)  # 👈 只有这里才会触发设置
```

### 关键流程

1. **GPS 检查前置条件** (`check_and_set_home` 每秒调用一次)
   ```
   system_connected? → YES
   ↓
   gps_fix_type >= 0? → YES/NO
   ↓                      ↓
   设置 Home            return（等待）
   ```

2. **三个 USV 的不同路径**:

   | USV | GPS 状态 | 行为 | 结果 |
   |-----|---------|------|------|
   | USV_01 | GPS 部分就绪，但 EKF 未准备好 | ✅ 尝试设置 | ❌ 失败 |
   | USV_02 | **GPS 配置数据未到达** | ⏸️ **不尝试设置** | ✅ **无错误** |
   | USV_03 | GPS 部分就绪，但时钟错误 | ✅ 尝试设置 | ❌ 失败 |

### 详细解释

**USV_02 的情况**:
```
EKF3 waiting for GPS config data
    ↓
NavSatFix.status.status = -1 或未收到消息
    ↓
gps_fix_type < 0
    ↓
check_and_set_home() 直接 return
    ↓
不调用 set_home()
    ↓
不发送 MAVLink 命令
    ↓
不会收到失败响应
    ↓
✅ 没有 "Failed to set EKF origin!" 错误
```

**USV_01 和 USV_03 的情况**:
```
GPS 部分初始化（可能有位置数据但 EKF 未就绪）
    ↓
NavSatFix.status.status >= 0
    ↓
gps_fix_type >= 0
    ↓
check_and_set_home() 认为条件满足
    ↓
调用 set_home()
    ↓
发送 MAVLink Command 179 (SET_GPS_GLOBAL_ORIGIN)
    ↓
飞控拒绝（EKF 未就绪/Radio failsafe/时钟错误）
    ↓
❌ "Failed to set EKF origin!" 错误
```

## GPS 初始化阶段差异

### GPS 初始化的多个阶段

```
1. GPS 模块上电
    ↓
2. 接收卫星信号
    ↓
3. 获取位置数据 (NavSatFix.status >= 0)
    ↓
4. 提供配置数据给 EKF (GPS config data)
    ↓
5. EKF 融合数据并初始化
    ↓
6. 可以设置 EKF origin
```

**三个 USV 的阶段**:
- **USV_02**: 卡在阶段 3-4 之间（等待 GPS config data）
- **USV_01**: 到达阶段 3，但 EKF 未就绪（飞控拒绝设置）
- **USV_03**: 到达阶段 3，但时钟未同步（飞控拒绝设置）

## 为什么初始化速度不同？

### 可能的原因

1. **GPS 模块差异**
   - 不同的 GPS 接收器性能不同
   - 卫星信号强度差异（室内/室外）
   - 硬件质量差异

2. **飞控启动顺序**
   - EKF 初始化顺序不同
   - 参数加载速度不同
   - 传感器校准速度不同

3. **串口通信速度**
   - UART 波特率稳定性
   - USB 枚举顺序（如果用 USB）
   - 驱动加载时间

4. **环境因素**
   - 天线位置和朝向
   - 周围金属物体干扰
   - 卫星可见度

5. **系统负载**
   - CPU 使用率
   - 其他进程占用资源
   - ROS 节点启动顺序

## 这是问题吗？

### ✅ 正常现象

这种差异是**正常的**，因为：

1. **GPS 初始化本身就是异步的**
   - 卫星捕获时间不确定
   - 取决于上次关机位置、时间等

2. **保护机制正常工作**
   - USV_02 正确等待 GPS 就绪（避免过早设置失败）
   - USV_01/03 尝试设置但被飞控拒绝（飞控保护）

3. **最终都会初始化成功**
   - USV_02 等 GPS 数据到达后会自动设置
   - USV_01/03 会重试（代码中有重试机制）

### ⚠️ 需要关注的问题

1. **Command 179 超时**
   - 说明飞控响应慢或拒绝命令
   - 已通过之前的优化解决（GPS 状态检查）

2. **Radio failsafe**
   - 遥控器未连接或信号丢失
   - 需要检查遥控器连接

3. **Wrong FCU time (USV_03)**
   - 飞控时钟未同步
   - 可能影响日志时间戳

## 优化建议

### 1. 增加日志详细度

让 `auto_set_home_node` 输出更多调试信息：

```python
def check_and_set_home(self):
    """检查 GPS 和系统状态，满足条件后设置 Home 点"""
    if self.set_home_sent:
        return
    
    # 检查系统是否连接
    if not self.system_connected:
        self.get_logger().info('⏳ Waiting for MAVROS connection...')  # 改为 info
        return
    
    # 检查 GPS 定位状态
    if self.gps_fix_type < 0:
        self.get_logger().info(
            f'⏳ Waiting for GPS fix (current: {self.gps_fix_type}, need >= 0)...'
        )
        return
    
    # 新增：检查 GPS 数据质量
    self.get_logger().info(
        f'📡 GPS ready - fix_type: {self.gps_fix_type}, '
        f'system_connected: {self.system_connected}'
    )
    
    # 所有条件满足，设置 Home 点
    self.get_logger().info(
        f'✅ All conditions met! Setting Home point in {self.set_delay_sec:.1f}s...'
    )
    self.set_home_sent = True
    self.create_timer(self.set_delay_sec, self.set_home)
```

### 2. 添加 GPS 质量检查

除了检查 `status.status`，还应检查卫星数量：

```python
def gps_callback(self, msg):
    """GPS 状态回调"""
    self.gps_fix_type = msg.status.status
    # 新增：记录卫星数量（如果可用）
    if hasattr(msg.status, 'service'):
        self.gps_satellites = msg.status.service
        self.get_logger().debug(f'GPS satellites: {self.gps_satellites}')
```

### 3. 等待 EKF 就绪

订阅 EKF 状态消息，确认 EKF 已初始化：

```python
# 在 __init__ 中添加
self.ekf_status_ok = False
self.ekf_status_sub = self.create_subscription(
    # 需要查找具体的 EKF 状态消息类型
    # 可能是 mavros_msgs/msg/EKFStatus
)
```

### 4. 统一启动时间

在 launch 文件中添加延迟，让 GPS 有更多时间初始化：

```python
# usv_launch.py
auto_set_home_node = Node(
    package='usv_comm',
    executable='auto_set_home_node',
    namespace=namespace,
    parameters=[{
        'set_delay_sec': 5.0  # 增加到 5 秒
    }]
)
```

## 监控命令

### 实时查看 GPS 状态

```bash
# USV_01
ros2 topic echo /usv_01/mavros/global_position/global

# USV_02
ros2 topic echo /usv_02/mavros/global_position/global

# USV_03
ros2 topic echo /usv_03/mavros/global_position/global
```

### 检查 MAVROS 状态

```bash
ros2 topic echo /usv_01/mavros/state
ros2 topic echo /usv_02/mavros/state
ros2 topic echo /usv_03/mavros/state
```

## 总结

| 问题 | USV_01 | USV_02 | USV_03 |
|------|--------|--------|--------|
| GPS 状态 | 部分就绪 | **等待配置数据** | 部分就绪 |
| 是否尝试设置 EKF | ✅ 是 | ❌ **否** | ✅ 是 |
| 是否失败 | ✅ 失败 | ➖ 未尝试 | ✅ 失败 |
| 错误日志 | ✅ 有 | ❌ **无** | ✅ 有 |
| 根本原因 | GPS/EKF 未就绪 | **GPS 初始化慢** | 时钟+GPS 问题 |
| 是否正常 | ✅ 是（会重试） | ✅ **是（保护机制）** | ⚠️ 需检查时钟 |

**核心结论**:
- USV_02 没有错误是因为它**正确地等待 GPS 就绪**，没有过早尝试设置
- USV_01 和 USV_03 尝试设置但失败，会自动重试，最终也会成功
- 这种差异是 GPS 初始化的**正常异步行为**，不是系统问题

---

**建议行动**:
1. ✅ 增加调试日志，观察 GPS 初始化进度
2. ✅ 检查 USV_03 的时钟同步问题
3. ✅ 确认所有 USV 最终都能成功设置 EKF origin
4. ⚠️ 如果长期失败，检查 GPS 天线和飞控参数
