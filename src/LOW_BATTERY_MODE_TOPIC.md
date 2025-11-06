# 低电量模式话题发布机制

## 修改日期
2025-11-06

## 问题描述
usv_01 的电量已经降至 0.0%（电压 10.5V），但外设节点（LED 和 Sound）没有立即进入低电量模式。

## 原因分析
1. **检测逻辑存在但响应不够快**
   - `usv_status_node.py` 能够检测低电量（< 5%）并设置 `low_voltage_mode = True`
   - 但该标志仅通过 `UsvStatus` 消息传递，可能存在延迟

2. **外设节点依赖状态消息**
   - `usv_led_node` 和 `usv_sound_node` 通过订阅 `usv_status` 话题获取低电压模式
   - 状态消息发布频率较低（1Hz），导致响应延迟

## 解决方案

### 新增低电压模式专用话题
在 `usv_status_node` 中新增一个 **专门的低电压模式话题**，使用 `std_msgs/Bool` 消息类型：

- **话题名称**: `/low_voltage_mode`
- **消息类型**: `std_msgs/Bool`
- **QoS 策略**: `RELIABLE`（确保送达）
- **发布时机**: 
  - 检测到电量 < 5% 时立即发布 `True`
  - 检测到电量 > 8% 时立即发布 `False`（滞后恢复）

### 修改内容

#### 1. usv_comm/usv_status_node.py

**添加低电压模式发布器**：
```python
# 创建发布者（第 46 行）
self.state_publisher = self.create_publisher(UsvStatus, 'usv_state', 10)
self.temperature_publisher = self.create_publisher(Float32, 'usv_temperature', 10)
# 低电压模式话题发布器（用于通知 LED 和 Sound 节点立即响应）
self.low_voltage_mode_publisher = self.create_publisher(Bool, 'low_voltage_mode', qos_reliable)
```

**触发时发布话题**（第 418 行）：
```python
if battery_pct < LOW_BATTERY_THRESHOLD:
    if not self.low_voltage_mode:
        # 刚进入低电压模式
        self.low_voltage_mode = True
        self.get_logger().error(
            f'[!][!][!] 低电量模式触发！ [!][!][!]\n'
            f'当前电压: {current_voltage:.2f}V, '
            f'平均电压(10s): {avg_voltage:.2f}V, '
            f'电量百分比: {battery_pct:.1f}% < {LOW_BATTERY_THRESHOLD}%\n'
            f'请立即返航或靠岸！'
        )
        # 立即发布低电压模式话题，通知 LED 和 Sound 节点
        low_voltage_msg = Bool()
        low_voltage_msg.data = True
        self.low_voltage_mode_publisher.publish(low_voltage_msg)
        self.get_logger().info('已发布低电压模式话题 (True) 到外设节点')
```

**恢复时发布话题**（第 432 行）：
```python
elif battery_pct > RECOVER_THRESHOLD:
    if self.low_voltage_mode:
        # 退出低电压模式
        self.low_voltage_mode = False
        self.get_logger().info(
            f'[OK] 退出低电量模式 - 电量百分比: {battery_pct:.1f}% > {RECOVER_THRESHOLD}%'
        )
        # 发布低电压模式恢复话题
        low_voltage_msg = Bool()
        low_voltage_msg.data = False
        self.low_voltage_mode_publisher.publish(low_voltage_msg)
        self.get_logger().info('已发布低电压模式话题 (False) 到外设节点')
```

#### 2. usv_led/usv_led_node.py

**导入 Bool 消息类型**（第 11 行）：
```python
from std_msgs.msg import String, Bool
```

**添加订阅器**（第 90 行）：
```python
# 订阅专门的低电压模式话题（优先级更高，响应更快）
self.low_voltage_mode_sub = self.create_subscription(
    Bool,
    'low_voltage_mode',
    self.low_voltage_mode_callback,
    QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
)
```

**添加回调函数**（第 395 行，在 `usv_status_callback` 之后）：
```python
def low_voltage_mode_callback(self, msg):
    """
    低电压模式专用回调函数 - 立即响应低电量状态
    
    此回调优先级高于 usv_status_callback，确保快速响应
    
    Args:
        msg (Bool): 低电压模式标志（True=进入低电量，False=退出低电量）
    """
    try:
        if not isinstance(msg, Bool):
            self.get_logger().warn('收到无效的低电压模式消息类型')
            return
        
        if msg.data:  # 进入低电量模式
            if not self.is_low_battery_level:
                self.last_mode_before_low_battery = self.mode
                self.mode = 'low_battery_breath'
                self.get_logger().error(
                    f'[!][!][!] 低电压模式触发！ [!][!][!]\n'
                    f'进入低电量红色呼吸灯模式'
                )
            self._color_select_transition_active = False
            self.is_low_battery_level = True
        else:  # 退出低电量模式
            if self.is_low_battery_level:
                # 恢复原有模式
                self.mode = self.last_mode_before_low_battery or 'color_switching'
                self.get_logger().info(
                    f'[OK] 退出低电压模式，恢复正常LED模式'
                )
            self.is_low_battery_level = False
            
    except Exception as e:
        self.get_logger().error(f'处理低电压模式回调时发生错误: {e}')
```

#### 3. usv_sound/usv_sound_node.py

**导入 Bool 消息类型**（第 10 行）：
```python
from std_msgs.msg import String, Bool
```

**添加订阅器**（第 68 行）：
```python
# 订阅专门的低电压模式话题（优先级更高，响应更快）
self.low_voltage_mode_sub = self.create_subscription(
    Bool,
    'low_voltage_mode',
    self.low_voltage_mode_callback,
    qos_reliable
)
```

**添加回调函数**（第 143 行，在 `usv_status_callback` 之后）：
```python
def low_voltage_mode_callback(self, msg):
    """
    低电压模式专用回调函数 - 立即响应低电量状态
    
    此回调优先级高于 usv_status_callback，确保快速响应
    
    Args:
        msg (Bool): 低电压模式标志（True=进入低电量，False=退出低电量）
    """
    try:
        if not isinstance(msg, Bool):
            self.get_logger().warn('收到无效的低电压模式消息类型')
            return
        
        if msg.data:  # 进入低电量模式
            if not self.low_voltage:
                self.get_logger().error(
                    f'[!][!][!] 低电压模式触发！ [!][!][!]\n'
                    f'切换到低电量告警声音'
                )
            self.low_voltage = True
        else:  # 退出低电量模式
            if self.low_voltage:
                self.get_logger().info(
                    f'[OK] 退出低电压模式，恢复正常声音'
                )
            self.low_voltage = False
            
    except Exception as e:
        self.get_logger().error(f'处理低电压模式回调时发生错误: {e}')
```

## 工作流程

```
┌─────────────────────────┐
│  usv_status_node        │
│                         │
│  检测电池电量 < 5%      │
│  ↓                      │
│  设置 low_voltage_mode  │
│  = True                 │
│  ↓                      │
│  发布 Bool(True) 到     │
│  /low_voltage_mode      │
└────────┬────────────────┘
         │
         ├─────────────────┬─────────────────┐
         ↓                 ↓                 ↓
┌────────────────┐ ┌────────────────┐ ┌──────────────────┐
│ usv_led_node   │ │ usv_sound_node │ │ (其他外设节点)   │
│                │ │                │ │                  │
│ 接收 True      │ │ 接收 True      │ │ 接收 True        │
│ ↓              │ │ ↓              │ │ ↓                │
│ 进入红色呼吸灯 │ │ 播放警告声音   │ │ 执行保护动作     │
└────────────────┘ └────────────────┘ └──────────────────┘
```

## 优势

1. **快速响应**: 专用话题使用 RELIABLE QoS，确保消息可靠送达
2. **解耦设计**: 外设节点不依赖完整的 `UsvStatus` 消息，降低耦合度
3. **双重保障**: 
   - 主路径：专用话题 `/low_voltage_mode`（快速响应）
   - 备用路径：`UsvStatus.low_voltage_mode` 字段（兼容性）
4. **易于扩展**: 其他节点（如舵机、风扇）可以轻松订阅此话题实现低电量保护

## 触发阈值

- **进入低电量模式**: 电量百分比 < 5%
- **退出低电量模式**: 电量百分比 > 8%（滞后设计，避免频繁切换）
- **电压范围**:
  - 满电（100%）: 12.6V
  - 触发阈值（5%）: 10.605V
  - 恢复阈值（8%）: 10.668V
  - 空电（0%）: 10.5V

## 测试方法

### 1. 模拟低电量测试
```bash
# 终端 1: 启动 USV 节点
ros2 launch usv_bringup usv_launch.py namespace:=usv_01

# 终端 2: 监听低电压模式话题
ros2 topic echo /usv_01/low_voltage_mode

# 终端 3: 手动发布低电量模拟消息（修改飞控电池参数或等待实际低电量）
```

### 2. 查看日志输出
```bash
# usv_status_node 日志
[usv_status_node-X] [ERROR] [!][!][!] 低电量模式触发！ [!][!][!]
[usv_status_node-X] [INFO] 已发布低电压模式话题 (True) 到外设节点

# usv_led_node 日志
[usv_led_node-X] [ERROR] [!][!][!] 低电压模式触发！ [!][!][!]
[usv_led_node-X] 进入低电量红色呼吸灯模式

# usv_sound_node 日志
[usv_sound_node-X] [ERROR] [!][!][!] 低电压模式触发！ [!][!][!]
[usv_sound_node-X] 切换到低电量告警声音
```

## 构建和部署

```bash
# 构建修改的包
cd ~/usv_workspace
colcon build --packages-select usv_comm usv_led usv_sound

# Source 环境
source install/setup.bash

# 重启 USV 节点
ros2 launch usv_bringup usv_launch.py namespace:=usv_01
```

## 相关文件
- `usv_comm/usv_comm/usv_status_node.py` - 低电量检测和话题发布
- `usv_led/usv_led/usv_led_node.py` - LED 响应低电量模式
- `usv_sound/usv_sound/usv_sound_node.py` - 声音响应低电量模式
- `BATTERY_PERCENTAGE_GUIDE.md` - 电池百分比计算说明

## 注意事项

1. **飞控配置**: 确保飞控 BATT_CAPACITY 参数已正确配置（如 20000 mAh）
2. **电压校准**: 根据实际电池特性调整 `battery_voltage_full` 和 `battery_voltage_empty` 参数
3. **滞后机制**: 5%-8% 区间为滞后区，避免频繁切换模式
4. **话题命名空间**: 话题会自动添加 USV 命名空间前缀（如 `/usv_01/low_voltage_mode`）

## 未来改进

1. **自动返航**: 检测到低电量后自动切换飞控到 RTL 模式
2. **GUI 警告**: 在地面站 GUI 中弹出红色警告对话框
3. **声音告警**: 播放更明显的低电量警告音效
4. **电量预测**: 根据放电速率预测剩余飞行时间
