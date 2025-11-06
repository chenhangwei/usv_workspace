# USV 集群命名空间隔离分析报告

## 📋 概述

本报告全面分析 USV 集群系统的命名空间隔离情况，确保多艘 USV（如 `usv_01`, `usv_02`, `usv_03`）可以同时运行而不会相互干扰。

**分析日期：** 2025-11-06  
**系统版本：** ROS 2 Humble/Iron  
**命名空间格式：** `/usv_XX` (XX 为两位数字)

---

## ✅ 命名空间隔离现状总结

### 整体评估：**良好（有 1 个严重问题需修复）**

| 类别 | 状态 | 问题数 |
|------|------|--------|
| 核心通信节点 | ✅ 完全隔离 | 0 |
| 控制节点 | ✅ 完全隔离 | 0 |
| 传感器驱动 | ⚠️ 部分问题 | 1（UWB硬编码） |
| 辅助功能 | ✅ 完全隔离 | 0 |
| TF坐标变换 | ⚠️ 需检查 | 1（潜在问题） |
| 地面站集成 | ✅ 完全隔离 | 0 |

---

## 📊 详细分析

### 1. 启动文件（usv_launch.py）- ✅ 完全隔离

**文件：** `usv_bringup/launch/usv_launch.py`

所有节点都正确使用 `namespace` 参数：

```python
namespace = LaunchConfiguration('namespace')

# 示例：所有节点都有 namespace 配置
usv_status_node = Node(
    package='usv_comm',
    executable='usv_status_node',
    name='usv_status_node',
    namespace=namespace,  # ✅ 正确
    output='screen',
    parameters=[param_file]
)
```

**验证方式：**
```bash
# 启动 3 艘 USV
ros2 launch usv_bringup usv_launch.py namespace:=usv_01 &
ros2 launch usv_bringup usv_launch.py namespace:=usv_02 &
ros2 launch usv_bringup usv_launch.py namespace:=usv_03 &

# 验证节点隔离
ros2 node list | grep usv
# 应该看到：
# /usv_01/usv_status_node
# /usv_01/usv_control_node
# /usv_02/usv_status_node
# /usv_02/usv_control_node
# /usv_03/usv_status_node
# /usv_03/usv_control_node
```

---

### 2. 通信与状态管理节点 - ✅ 完全隔离

#### 2.1 usv_status_node（状态聚合）

**文件：** `usv_comm/usv_comm/usv_status_node.py`

**Topic 使用分析：**

| Topic | 类型 | 是否隔离 | 说明 |
|-------|------|----------|------|
| `usv_state` | Pub | ✅ | 相对路径，自动加命名空间 |
| `usv_temperature` | Pub | ✅ | 相对路径 |
| `state` | Sub | ✅ | MAVROS topic，自动继承命名空间 |
| `battery` | Sub | ✅ | MAVROS topic |
| `local_position/pose` | Sub | ✅ | MAVROS topic |
| `local_position/velocity_local` | Sub | ✅ | MAVROS topic |
| `setpoint_raw/local` | Sub | ✅ | MAVROS topic |
| `global_position/global` | Sub | ✅ | MAVROS topic |
| `gpsstatus/gps1/raw` | Sub | ✅ | MAVROS topic |
| `global_position/compass_hdg` | Sub | ✅ | MAVROS topic |

**代码示例：**
```python
# ✅ 所有 topic 都使用相对路径，自动隔离
self.state_publisher = self.create_publisher(UsvStatus, 'usv_state', 10)
self.state_sub = self.create_subscription(State, 'state', ...)
```

#### 2.2 auto_set_home_node（自动设置Home点）

**文件：** `usv_comm/usv_comm/auto_set_home_node.py`

**Service 使用分析：**

| Service | 类型 | 是否隔离 | 说明 |
|---------|------|----------|------|
| `cmd/command` | Client | ✅ | MAVROS service，自动继承命名空间 |

**代码示例：**
```python
# ✅ 相对路径，自动隔离
self.set_home_cli = self.create_client(CommandLong, 'cmd/command')
self.pose_sub = self.create_subscription(PoseStamped, 'local_position/pose', ...)
self.gps_sub = self.create_subscription(NavSatFix, 'global_position/global', ...)
```

#### 2.3 navigate_to_point_server（导航动作服务器）

**文件：** `usv_comm/usv_comm/navigate_to_point_server.py`

**Action 使用分析：**

| Action | 类型 | 是否隔离 | 说明 |
|--------|------|----------|------|
| `navigate_to_point` | Server | ✅ | 相对路径，自动隔离 |

**代码示例：**
```python
# ✅ 使用相对路径，每个命名空间有独立的 Action 服务器
self._action_server = ActionServer(
    self,
    NavigateToPoint,
    'navigate_to_point',  # ✅ 相对路径
    self.execute_callback
)
```

---

### 3. 控制节点 - ✅ 完全隔离

#### 3.1 usv_control_node（核心控制器）

**文件：** `usv_control/usv_control/usv_control_node.py`

**Topic 使用分析：**

| Topic | 类型 | 是否隔离 | 说明 |
|-------|------|----------|------|
| `setpoint_raw/local` | Pub | ✅ | MAVROS topic，相对路径 |
| `state` | Sub | ✅ | MAVROS topic |
| `set_usv_target_position` | Sub | ✅ | 地面站命令，相对路径 |
| `avoidance_position` | Sub | ✅ | 避障模块，相对路径 |
| `avoidance_flag` | Sub | ✅ | 避障标志，相对路径 |

```python
# ✅ 所有 topic 正确隔离
self.target_point_pub = self.create_publisher(
    PositionTarget, 'setpoint_raw/local', qos_best_effort)
self.target_point_sub = self.create_subscription(
    PoseStamped, 'set_usv_target_position', ...)
```

#### 3.2 usv_command_node（模式和解锁控制）

**文件：** `usv_control/usv_control/usv_command_node.py`

**Service 使用分析：**

| Service | 类型 | 是否隔离 | 说明 |
|---------|------|----------|------|
| `cmd/arming` | Client | ✅ | MAVROS service |
| `set_mode` | Client | ✅ | MAVROS service |

**Topic 使用分析：**

| Topic | 类型 | 是否隔离 | 说明 |
|-------|------|----------|------|
| `set_usv_mode` | Sub | ✅ | 地面站命令 |
| `set_usv_arming` | Sub | ✅ | 地面站命令 |

```python
# ✅ 正确隔离
self.arming_client = self.create_client(CommandBool, 'cmd/arming')
self.mode_client = self.create_client(SetMode, 'set_mode')
self.sub_mode = self.create_subscription(String, 'set_usv_mode', ...)
```

#### 3.3 usv_avoidance_node（避障）

**文件：** `usv_control/usv_control/usv_avoidance_node.py`

**Topic 使用分析：**

| Topic | 类型 | 是否隔离 | 说明 |
|-------|------|----------|------|
| `avoidance_position` | Pub | ✅ | 输出给控制节点 |
| `avoidance_flag` | Pub | ✅ | 避障标志 |
| `ultrasonic_radar_range` | Sub | ✅ | 超声波雷达数据 |
| `setpoint_raw/local` | Sub | ✅ | 当前目标点 |
| `local_position/pose` | Sub | ✅ | 当前位置 |

```python
# ✅ 正确隔离
self.target_pub = self.create_publisher(PositionTarget, 'avoidance_position', 10)
self.radar_sub = self.create_subscription(Range, 'ultrasonic_radar_range', ...)
```

---

### 4. 传感器驱动节点 - ⚠️ 1 个严重问题

#### 4.1 usv_uwb_node（UWB定位）- ❌ **硬编码串口路径**

**文件：** `usv_drivers/usv_drivers/usv_uwb_node.py`

**问题：**
```python
# ❌ 硬编码了串口路径 '/dev/ttyUSB0'
self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
```

**影响：**
- **多艘 USV 无法同时使用 UWB**，因为串口设备路径冲突
- 如果 USV 01 和 USV 02 都尝试打开 `/dev/ttyUSB0`，第二个会失败

**修复方案：**
```python
# ✅ 从参数读取串口路径
self.declare_parameter('uwb_port', '/dev/ttyUSB0')
port = self.get_parameter('uwb_port').value
self.serial_port = serial.Serial(port, 115200, timeout=1)
```

**配置示例（usv_params.yaml）：**
```yaml
# USV 01
usv_01:
  usv_uwb_node:
    ros__parameters:
      uwb_port: /dev/ttyUSB0

# USV 02
usv_02:
  usv_uwb_node:
    ros__parameters:
      uwb_port: /dev/ttyUSB1

# USV 03
usv_03:
  usv_uwb_node:
    ros__parameters:
      uwb_port: /dev/ttyUSB2
```

**Topic 使用：**

| Topic | 类型 | 是否隔离 | 说明 |
|-------|------|----------|------|
| `vision_pose/pose` | Pub | ✅ | MAVROS vision topic，相对路径 |

#### 4.2 其他传感器节点 - ✅ 正确隔离

**usv_laserscan_node, usv_ultrasonic_node, usv_su04_node, usv_ultrasonic_radar_node：**

所有节点都使用相对路径发布 topic，自动隔离：

```python
# ✅ 相对路径，自动隔离
self.publisher_ = self.create_publisher(Range, 'ultrasonic_radar_range', 10)
self.scan_pub = self.create_publisher(LaserScan, 'ultrasonic_scan', 10)
```

**注意：** 这些节点如果涉及硬件串口，也需要类似 UWB 的参数化配置。

---

### 5. 辅助功能节点 - ✅ 完全隔离

#### 5.1 usv_led_node（LED控制）

**文件：** `usv_led/usv_led/usv_led_node.py`

**Topic 使用分析：**

| Topic | 类型 | 是否隔离 | 说明 |
|-------|------|----------|------|
| `led_state` | Pub | ✅ | 状态回传给地面站 |
| `gs_led_command` | Sub | ✅ | 地面站命令 |
| `battery` | Sub | ✅ | MAVROS battery |
| `usv_state` | Sub | ✅ | USV状态 |

```python
# ✅ 正确隔离
self.led_state_pub = self.create_publisher(String, 'led_state', ...)
self.gs_led_sub = self.create_subscription(String, 'gs_led_command', ...)
```

#### 5.2 usv_sound_node, usv_fan_node, usv_head_action_node

**类似 LED 节点，所有 topic 都使用相对路径，自动隔离。**

---

### 6. TF 坐标变换节点 - ⚠️ 需检查

#### 6.1 static_tf_laser_node（静态TF发布）

**文件：** `usv_tf/usv_tf/static_tf_laser_node.py`

**分析：**
```python
# ✅ 使用命名空间参数化 frame_id
self.declare_parameter('namespace', 'usv_01')
self.ns = self.get_parameter('namespace').get_parameter_value().string_value

transform.header.frame_id = f'base_link_{self.ns}'  # ✅ 正确
transform.child_frame_id = f'laser_frame_{self.ns}'  # ✅ 正确
```

**评估：** ✅ **正确隔离**

TF frame 名称使用了命名空间后缀（如 `base_link_usv_01`），不会冲突。

#### 6.2 odom_to_tf（里程计TF转换）

**文件：** `usv_tf/usv_tf/odom_to_tf.py`

**分析：**
```python
# ✅ 使用命名空间参数化
self.declare_parameter('namespace', 'usv_01')
ns = self.get_parameter('namespace').get_parameter_value().string_value
self.base_link_frame = f'base_link_{ns}'  # ✅ 正确

# ⚠️ 订阅话题使用绝对路径
self.subscription_ = self.create_subscription(
    Odometry,
    f'/{ns}/global_position/local',  # ⚠️ 绝对路径
    self.odom_callback,
    qos
)
```

**潜在问题：**
- 使用绝对路径 `f'/{ns}/global_position/local'` 虽然功能正确，但不推荐
- 如果节点本身在命名空间中启动（已经是），应该使用相对路径

**建议修复：**
```python
# ✅ 推荐：使用相对路径（节点已在命名空间中）
self.subscription_ = self.create_subscription(
    Odometry,
    'global_position/local',  # 相对路径
    self.odom_callback,
    qos
)
```

**但目前代码也能工作**，因为绝对路径明确指定了命名空间。

---

### 7. MAVROS 节点 - ✅ 完全隔离

**启动配置（usv_launch.py）：**

```python
mavros_node = Node(
    package='mavros',
    executable='mavros_node',
    namespace=namespace,  # ✅ 正确，MAVROS 运行在 USV 命名空间中
    parameters=[
        {
            'fcu_url': fcu_url,  # 每个 USV 不同的串口
            'system_id': tgt_system,  # 每个 USV 不同的 MAVLink ID
            'target_system_id': tgt_system,
            # ...
        }
    ]
)
```

**关键隔离机制：**
1. **命名空间隔离**：每个 MAVROS 运行在不同命名空间（如 `/usv_01/mavros`）
2. **串口隔离**：`fcu_url` 参数化，每个 USV 使用不同串口
   - USV 01: `serial:///dev/ttyACM0:921600`
   - USV 02: `serial:///dev/ttyACM1:921600`
   - USV 03: `serial:///dev/ttyACM2:921600`
3. **MAVLink ID 隔离**：`system_id` 参数化
   - USV 01: `system_id=1`
   - USV 02: `system_id=2`
   - USV 03: `system_id=3`

**验证：**
```bash
ros2 topic list | grep mavros
# 应该看到：
# /usv_01/mavros/state
# /usv_01/mavros/battery
# /usv_02/mavros/state
# /usv_02/mavros/battery
# ...
```

---

### 8. 地面站集成 - ✅ 完全隔离

**文件：** `gs_gui/gs_gui/usv_manager.py`

**动态管理机制：**

```python
def add_usv_namespace(self, ns):
    """为每个 USV 动态创建订阅者和发布者"""
    usv_id = ns.lstrip('/')
    
    # ✅ 使用绝对路径（带命名空间）创建 topic
    topic_state = f"{ns}/usv_state"
    topic_mode = f"{ns}/set_usv_mode"
    topic_arming = f"{ns}/set_usv_arming"
    # ...
    
    # 为每个 USV 创建独立的发布者/订阅者
    self.usv_state_subs[usv_id] = self.node.create_subscription(
        UsvStatus, topic_state, ...)
    self.set_usv_mode_pubs[usv_id] = self.node.create_publisher(
        String, topic_mode, ...)
```

**评估：** ✅ **完全隔离**

地面站为每个在线 USV 动态创建独立的通信通道，不会混淆。

---

## 🚨 发现的问题总结

### 严重问题（必须修复）

#### 问题 1: UWB 串口路径硬编码

**影响：** 多艘 USV 无法同时使用 UWB 定位

**位置：** `usv_drivers/usv_drivers/usv_uwb_node.py` 第 28 行

**修复优先级：** 🔴 **高**

**详见：** 第 4.1 节

---

### 改进建议（可选）

#### 建议 1: odom_to_tf 使用相对路径

**影响：** 代码风格和可维护性

**位置：** `usv_tf/usv_tf/odom_to_tf.py` 第 20 行

**修复优先级：** 🟡 **中**

**详见：** 第 6.2 节

#### 建议 2: 其他传感器节点参数化串口

**影响：** 如果多 USV 使用相同传感器硬件，需要参数化

**相关节点：**
- `usv_laserscan_node`
- `usv_ultrasonic_node`
- `usv_su04_node`
- `usv_ultrasonic_radar_node`

**修复优先级：** 🟢 **低**（如果不使用多个传感器）

---

## ✅ 验证检查清单

### 启动前检查

- [ ] **MAVROS 串口配置**：确认每个 USV 使用不同的 `/dev/ttyACMX`
- [ ] **MAVLink System ID**：确认每个 USV 的 `system_id` 不同
- [ ] **传感器串口**：如果启用 UWB/激光雷达，确认串口路径不同
- [ ] **命名空间参数**：启动时明确指定 `namespace:=usv_XX`

### 运行时检查

```bash
# 1. 检查节点命名空间隔离
ros2 node list | grep usv
# 应该看到：/usv_01/xxx, /usv_02/xxx, /usv_03/xxx

# 2. 检查 topic 隔离
ros2 topic list | grep usv
# 应该看到：/usv_01/usv_state, /usv_02/usv_state, ...

# 3. 检查 TF 树隔离
ros2 run tf2_tools view_frames
# 应该看到：base_link_usv_01, base_link_usv_02, ...

# 4. 检查 MAVROS 连接
ros2 topic echo /usv_01/mavros/state --once
ros2 topic echo /usv_02/mavros/state --once
# 应该看到不同的状态

# 5. 检查地面站订阅
ros2 node info /groundstationnode
# 应该看到订阅了 /usv_01/usv_state, /usv_02/usv_state, ...
```

---

## 🔧 修复代码示例

### 修复 1: UWB 节点参数化

**文件：** `usv_drivers/usv_drivers/usv_uwb_node.py`

```python
class UsvUwbNode(Node):
    def __init__(self):
        super().__init__('usv_uwb_node')
        
        # ✅ 添加参数声明
        self.declare_parameter('uwb_port', '/dev/ttyUSB0')
        self.declare_parameter('uwb_baudrate', 115200)
        
        # ✅ 从参数读取配置
        port = self.get_parameter('uwb_port').value
        baudrate = self.get_parameter('uwb_baudrate').value
        
        self.uwb_pub = self.create_publisher(PoseStamped, 'vision_pose/pose', 10)
        
        try:
            # ✅ 使用参数化的串口配置
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'UWB 串口打开成功: {port}@{baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'打开UWB串口失败 ({port}): {e}')
            return
        
        # ... 其余代码不变
```

**参数文件配置（usv_params.yaml）：**

```yaml
# 每个 USV 使用不同的 UWB 串口
usv_01:
  usv_uwb_node:
    ros__parameters:
      uwb_port: /dev/ttyUSB0
      uwb_baudrate: 115200

usv_02:
  usv_uwb_node:
    ros__parameters:
      uwb_port: /dev/ttyUSB1
      uwb_baudrate: 115200

usv_03:
  usv_uwb_node:
    ros__parameters:
      uwb_port: /dev/ttyUSB2
      uwb_baudrate: 115200
```

### 修复 2: odom_to_tf 使用相对路径

**文件：** `usv_tf/usv_tf/odom_to_tf.py`

```python
class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ✅ 仍然声明参数（用于 TF frame 命名）
        self.declare_parameter('namespace', 'usv_01')
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.base_link_frame = f'base_link_{ns}'
        
        # ✅ 改用相对路径（节点已在命名空间中启动）
        self.subscription_ = self.create_subscription(
            Odometry,
            'global_position/local',  # 相对路径（推荐）
            self.odom_callback,
            qos
        )
```

---

## 📚 最佳实践总结

### DO ✅

1. **使用相对路径**：节点内部创建的 topic/service 使用相对路径
   ```python
   self.pub = self.create_publisher(String, 'my_topic', 10)  # ✅
   ```

2. **启动时指定命名空间**：在 launch 文件中明确指定 `namespace`
   ```python
   Node(package='pkg', executable='node', namespace=namespace)  # ✅
   ```

3. **参数化硬件路径**：所有硬件接口（串口、设备文件）使用参数
   ```python
   self.declare_parameter('port', '/dev/ttyUSB0')  # ✅
   ```

4. **TF frame 命名**：使用命名空间后缀区分 frame
   ```python
   frame_id = f'base_link_{self.ns}'  # ✅
   ```

5. **验证隔离**：启动后检查 `ros2 node list` 和 `ros2 topic list`

### DON'T ❌

1. **避免硬编码绝对路径**：
   ```python
   self.pub = self.create_publisher(String, '/global_topic', 10)  # ❌
   ```

2. **避免硬编码硬件路径**：
   ```python
   serial.Serial('/dev/ttyUSB0', 115200)  # ❌
   ```

3. **避免共享全局资源**：
   - 全局 topic（以 `/` 开头）
   - 共享 TF frame（没有命名空间后缀）
   - 硬编码的硬件设备

---

## 🎯 下一步行动

### 立即修复（必须）

1. **修复 UWB 节点硬编码串口**
   - [ ] 修改 `usv_uwb_node.py` 代码
   - [ ] 更新 `usv_params.yaml` 参数文件
   - [ ] 测试多 USV 同时启动

### 建议改进（可选）

2. **优化 odom_to_tf**
   - [ ] 改用相对路径订阅
   - [ ] 测试验证功能不变

3. **检查其他传感器节点**
   - [ ] 确认是否需要多 USV 同时使用激光雷达/超声波
   - [ ] 如需要，参数化串口配置

### 持续验证

4. **集成测试**
   - [ ] 同时启动 3 艘 USV
   - [ ] 验证地面站能正确识别和控制所有 USV
   - [ ] 验证集群任务执行不互相干扰

---

**分析完成日期：** 2025-11-06  
**下次复审建议：** 代码修改后或新增传感器节点时
