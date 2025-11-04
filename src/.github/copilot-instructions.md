# USV Workspace - AI Agent Instructions

## 项目概述 (Project Overview)

这是一个 **ROS 2 (Robot Operating System 2)** 工作空间，用于控制和管理多艘无人水面艇（USV - Unmanned Surface Vehicles）集群系统。系统采用分布式架构，包含地面站GUI、机载控制节点、传感器驱动和通信模块。

**核心架构模式：**
- **命名空间隔离**：每艘 USV 运行在独立命名空间（如 `/usv_01`, `/usv_02`）中，实现多机器人并行控制
- **集群 vs 离群**：USV 分为集群（协同工作）和离群（独立操作）两种状态
- **地面站-机载分离**：`gs_*` 包运行在地面站，`usv_*` 包运行在机载计算机
- **消息接口统一**：`common_interfaces` 包定义所有共享消息和动作类型
- **Manager 模式**：各功能模块封装为独立 Manager 类（如 `TableManager`, `ResourceManager`, `ErrorHandler`），通过回调注入实现解耦

## 关键架构决策

### 1. 坐标系统（Coordinate Systems）

**三层坐标变换：**
```python
# 任务坐标 (Area) → 全局坐标 (Map) → USV本地坐标 (与全局坐标相同)
def _area_to_global(pos):
    # area_center 在 gs_params.yaml 中配置
    return {
        'x': area_center_x + pos['x'],
        'y': area_center_y + pos['y'],
        'z': area_center_z + pos['z']
    }

def _global_to_usv_local(usv_id, pos_global):
    # 全局坐标系 = USV本地坐标系（都以A0基站为原点）
    # 无需转换，直接返回
    return pos_global
```

**Why**: - XML 任务文件使用相对坐标，需要转换到全局 map 坐标系
- 全局坐标系 = USV本地坐标系（都以定位基站A0为原点，通过set_home设置）
- 无需复杂的坐标变换，所有USV共享同一坐标系

### 2. 导航系统（Navigation via ROS 2 Actions）

USV 导航使用 **Action** 接口（`NavigateToPoint.action`），而非简单的 topic：
- **Goal**: 目标点 (PoseStamped)
- **Feedback**: 实时反馈（距离、航向误差、预计时间）
- **Result**: 任务完成状态（成功/失败/超时）

```python
# 地面站发送导航目标（ground_station_node.py）
self.send_nav_goal_via_action(usv_id, x, y, z, yaw, timeout)

# 机载接收并执行（通过 usv_control_node.py 接收并发布到飞控）
```

**Why**: Action 提供取消、反馈和状态管理能力，topic 无法满足导航任务的复杂生命周期管理。

### 3. 状态管理（State Management）

**分层状态处理：**
```
MAVROS (飞控) → usv_status_node → UsvStatus.msg → GroundStationNode → StateHandler → GUI
```

- `usv_status_node`：机载节点，从 MAVROS 聚合飞控状态，发布标准化的 `UsvStatus` 消息
- `StateHandler` (地面站)：200ms 定时器批量刷新 UI，避免高频更新卡顿
- **差量更新**：`TableManager` 只更新变化的单元格，减少 Qt 重绘开销

### 4. GUI 模块化设计（gs_gui 重构后架构）

**2025-10 重构**：从 70KB 单文件拆分为 13+ 个职责明确的模块：

```
MainWindow (main_gui_app.py) - 协调者
├── 核心模块
│   ├── UIUtils (ui_utils.py) - 日志缓冲、绘图窗口
│   ├── TableManager (table_manager.py) - 集群/离群表格
│   ├── USVListManager (usv_list_manager.py) - 三个列表（集群/离群/在线）
│   ├── StateHandler (state_handler.py) - 状态缓存和刷新
│   ├── USVCommandHandler (usv_commands.py) - 命令封装
│   └── ClusterTaskManager (cluster_task_manager.py) - XML任务管理
├── 基础设施模块
│   ├── ResourceManager (resource_manager.py) - 资源管理（线程、队列、清理）
│   ├── ErrorHandler (error_handler.py) - 错误处理和恢复
│   ├── StyleManager (style_manager.py) - QSS 样式管理
│   └── LoggerConfig (logger_config.py) - 统一日志配置
├── 功能扩展模块
│   ├── LedInfectionHandler (led_infection.py) - LED "感染"模式
│   └── CommandProcessor (command_processor.py) - 命令解析和验证
└── ROS 集成
    ├── GroundStationNode (ground_station_node.py) - ROS 2 节点
    ├── UsvManager (usv_manager.py) - USV 发布者/订阅者管理
    └── ClusterController (cluster_controller.py) - 集群导航控制
```

**关键模式：**
- **回调注入**：子模块通过回调函数与主窗口通信，避免循环依赖
- **信号驱动**：`ROSSignal` (PyQt Signal) 作为 ROS 线程和 GUI 线程的桥梁
- **资源生命周期管理**：`ResourceManager` 统一管理线程、队列等资源的创建和清理
- **错误分类与恢复**：`ErrorHandler` 区分错误类型（网络/参数/状态），支持自动重试和降级
- **性能优化**：日志 500ms 批量输出、状态 200ms 定时刷新、差量 UI 更新

## 开发工作流

### 1. 环境设置和构建

**首次设置：**
```bash
# 克隆代码库
cd ~/
git clone https://github.com/chenhangwei/usv_workspace.git
cd usv_workspace

# 安装 ROS 2 依赖（假设已安装 ROS 2 Humble/Iron）
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pyqt5

# 安装 MAVROS（如果未安装）
sudo apt install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras

# 安装可选依赖
sudo apt install -y python3-pytest  # 测试框架
```

**日常构建：**
```bash
# 从工作空间根目录（包含 src/ 目录）
cd ~/usv_workspace

# 完整构建（首次或修改了消息定义）
colcon build

# 仅构建特定包（推荐，更快）
colcon build --packages-select gs_gui
colcon build --packages-select usv_control

# 构建并显示详细输出
colcon build --packages-select gs_gui --event-handlers console_direct+

# 清理构建缓存后重新构建
rm -rf build/ install/ log/
colcon build

# Source 环境（每个新终端都需要！）
source install/setup.bash

# 或添加到 ~/.bashrc 自动加载
echo "source ~/usv_workspace/install/setup.bash" >> ~/.bashrc
```

**构建技巧：**
```bash
# 并行构建（加速）
colcon build --parallel-workers 4

# 只构建修改过的包及其依赖
colcon build --packages-up-to gs_gui

# 构建时忽略特定包
colcon build --packages-skip usv_drivers
```

### 2. 启动系统

**地面站启动：**
```bash
# 终端 1: 启动地面站 GUI
source install/setup.bash
ros2 launch gs_bringup gs_launch.py

# 如需修改参数文件位置
ros2 launch gs_bringup gs_launch.py \
    gs_param_file:=/path/to/custom_params.yaml
```

**机载系统启动：**
```bash
# 终端 2-4: 启动多艘 USV（每艘一个终端）
source install/setup.bash

# USV 01
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_01 \
    fcu_url:=serial:///dev/ttyACM0:921600 \
    tgt_system:=1

# USV 02（不同终端）
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_02 \
    fcu_url:=serial:///dev/ttyACM1:921600 \
    tgt_system:=2

# USV 03（不同终端）
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_03 \
    fcu_url:=serial:///dev/ttyACM2:921600 \
    tgt_system:=3
```

**启动参数说明：**
- `namespace`: USV 命名空间，格式 `usv_XX`（对应飞控 System ID）
- `fcu_url`: 飞控串口 `serial://<device>:<baudrate>`
- `tgt_system`: MAVROS 目标系统 ID（通常从 namespace 自动推断）
- `gs_param_file`: 地面站参数文件（包含 area_center 坐标系原点）
- `lidar_port`: 激光雷达串口（如启用）

**模拟/测试模式启动：**
```bash
# 无硬件模式（调试 GUI）
ros2 launch gs_bringup gs_launch.py

# 单个 USV 无飞控测试
ros2 launch usv_bringup usv_launch.py \
    namespace:=usv_01 \
    fcu_url:=udp://:14540@localhost:14557  # SITL 仿真
```

### 3. 测试工作流

**运行测试：**
```bash
# 运行所有测试
colcon test
colcon test-result --verbose

# 运行特定包的测试
colcon test --packages-select gs_gui
colcon test-result --all

# 运行特定测试文件
cd src/gs_gui
python3 -m pytest test/test_demo.py -v

# 运行特定测试方法
python3 -m pytest test/test_demo.py::TestROSSignal::test_simple_math -v

# 显示测试覆盖率
python3 -m pytest test/ --cov=gs_gui --cov-report=html
```

**代码质量检查：**
```bash
# Flake8 代码风格检查
cd src/gs_gui
python3 -m flake8 gs_gui/ --max-line-length=120

# 自动格式化代码（推荐 black）
pip3 install black
black gs_gui/ --line-length=120

# 检查文档字符串
python3 -m pydocstyle gs_gui/
```

### 4. 调试工作流

**查看系统状态：**
```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看特定命名空间的话题
ros2 topic list | grep usv_01

# 查看节点信息
ros2 node info /usv_01/usv_control_node

# 查看话题消息类型
ros2 topic info /usv_01/usv_status
```

**监控消息：**
```bash
# 实时监听 USV 状态
ros2 topic echo /usv_01/usv_status

# 监听特定字段（使用 --field）
ros2 topic echo /usv_01/usv_status --field mode

# 查看话题发布频率
ros2 topic hz /usv_01/usv_status

# 查看话题带宽
ros2 topic bw /usv_01/usv_status
```

**手动发送测试命令：**
```bash
# 发送 LED 颜色命令
ros2 topic pub /usv_01/gs_led_command std_msgs/msg/String \
    "data: 'color_select|255,0,0'" --once

# 发送导航目标点
ros2 topic pub /usv_01/set_usv_target_position geometry_msgs/msg/PoseStamped \
    "header: {frame_id: 'map'}
     pose: {position: {x: 10.0, y: 5.0, z: 0.0}}" --once

# 发送声音命令
ros2 topic pub /usv_01/gs_sound_command std_msgs/msg/String \
    "data: 'sound_start'" --once
```

**ROS 2 调试工具：**
```bash
# 查看 TF 树
ros2 run tf2_tools view_frames
# 会生成 frames.pdf

# 启动 rqt 图形界面
rqt

# 查看计算图
rqt_graph

# 查看日志
ros2 run rqt_console rqt_console
```

### 5. 开发新功能完整流程

**示例：添加新的 USV 命令（如"紧急停止"）**

**Step 1: 定义 ROS 信号** (`gs_gui/gs_gui/ros_signal.py`)
```python
class ROSSignal(QObject):
    # ... 现有信号 ...
    emergency_stop = pyqtSignal(list)  # 添加紧急停止信号
```

**Step 2: 实现命令处理器** (`gs_gui/gs_gui/usv_commands.py`)
```python
class USVCommandHandler:
    # ... 现有方法 ...
    
    def emergency_stop(self, usv_list):
        """发送紧急停止命令"""
        try:
            namespace_list = self._extract_namespaces(usv_list)
            self.ros_signal.emergency_stop.emit(namespace_list)
            self.append_info(f"紧急停止命令已发送: {namespace_list}")
        except Exception as e:
            self.append_info(f"发送紧急停止命令失败: {e}")
```

**Step 3: 主窗口包装** (`gs_gui/gs_gui/main_gui_app.py`)
```python
class MainWindow(QMainWindow):
    def __init__(self):
        # ... 现有初始化代码 ...
        
        # 连接 UI 按钮
        self.ui.emergency_stop_button.clicked.connect(self.emergency_stop_wrapper)
    
    def emergency_stop_wrapper(self):
        """紧急停止命令包装器"""
        # 可以选择集群或离群列表
        self.command_handler.emergency_stop(self.list_manager.usv_cluster_list)
```

**Step 4: 地面站节点处理** (`gs_gui/gs_gui/ground_station_node.py`)
```python
class GroundStationNode(Node):
    def __init__(self, signal):
        # ... 现有初始化代码 ...
        
        # 连接信号到回调
        self.ros_signal.emergency_stop.connect(self.emergency_stop_callback)
    
    def emergency_stop_callback(self, namespace_list):
        """紧急停止回调"""
        for ns in namespace_list:
            try:
                # 发布到对应 USV 的 topic
                if ns in self.usv_manager.set_usv_mode_pubs:
                    # 示例：切换到手动模式并加锁
                    self.usv_manager.set_usv_mode_pubs[ns].publish(
                        String(data='MANUAL'))
                    self.usv_manager.set_usv_arming_pubs[ns].publish(
                        String(data='disarm'))
                    self.get_logger().info(f"紧急停止 {ns}")
            except Exception as e:
                self.get_logger().error(f"紧急停止 {ns} 失败: {e}")
```

**Step 5: 构建和测试**
```bash
# 构建修改的包
colcon build --packages-select gs_gui

# Source 环境
source install/setup.bash

# 启动地面站测试
ros2 launch gs_bringup gs_launch.py

# 在另一个终端监控命令发送
ros2 topic echo /usv_01/set_usv_mode
ros2 topic echo /usv_01/set_usv_arming
```

**Step 6: 添加单元测试** (`gs_gui/test/test_usv_functions.py`)
```python
def test_emergency_stop_command():
    """测试紧急停止命令"""
    signal = ROSSignal()
    handler = USVCommandHandler(signal, lambda x: None)
    
    # 测试数据
    usv_list = [{'namespace': 'usv_01'}, {'namespace': 'usv_02'}]
    
    # 连接测试信号
    received = []
    signal.emergency_stop.connect(lambda ns_list: received.append(ns_list))
    
    # 执行命令
    handler.emergency_stop(usv_list)
    
    # 验证
    assert len(received) == 1
    assert received[0] == ['usv_01', 'usv_02']
```

### 6. 修改消息定义流程

**场景：向 UsvStatus 添加新字段**

**Step 1: 修改消息文件** (`common_interfaces/msg/UsvStatus.msg`)
```
# 现有字段...
float32 yaw
float32 temperature

# 新增字段
float32 battery_current         # 电池电流（安培）
uint8 gps_satellite_count       # GPS 卫星数量
```

**Step 2: 重新构建消息包**
```bash
# 必须先构建消息包
colcon build --packages-select common_interfaces

# 然后构建依赖它的包
colcon build --packages-select gs_gui usv_comm

# Source 环境
source install/setup.bash
```

**Step 3: 更新状态节点** (`usv_comm/usv_comm/usv_status_node.py`)
```python
# 在状态聚合逻辑中填充新字段
status_msg.battery_current = battery_state.current
status_msg.gps_satellite_count = gps_data.satellites_visible
```

**Step 4: 更新 GUI 显示** (`gs_gui/gs_gui/table_manager.py`)
```python
# 在 TABLE_HEADERS 中添加新列
TABLE_HEADERS = ["编号", "模式", "状态", ..., "电流", "卫星"]

# 在 _format_table_cells 中添加数据
cells = [
    ns,
    state.get('mode'),
    # ... 现有字段 ...
    f"{state.get('battery_current', 0.0):.2f}A",
    str(state.get('gps_satellite_count', 0))
]
```

### 7. 添加新硬件节点流程

**场景：添加新的传感器驱动**

**Step 1: 创建新包**
```bash
cd src/
ros2 pkg create --build-type ament_python usv_new_sensor \
    --dependencies rclpy std_msgs sensor_msgs

cd usv_new_sensor/usv_new_sensor/
```

**Step 2: 编写节点** (`usv_new_sensor/usv_new_sensor/new_sensor_node.py`)
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class NewSensorNode(Node):
    def __init__(self):
        super().__init__('new_sensor_node')
        
        # 声明参数
        self.declare_parameter('sensor_port', '/dev/ttyUSB0')
        self.declare_parameter('publish_rate', 10.0)
        
        # 发布器
        self.publisher = self.create_publisher(Range, 'new_sensor_data', 10)
        
        # 定时器
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.timer_callback)
        
        # 初始化硬件
        self.init_hardware()
    
    def init_hardware(self):
        """初始化传感器硬件"""
        port = self.get_parameter('sensor_port').value
        # 硬件初始化代码...
    
    def timer_callback(self):
        """定时发布传感器数据"""
        msg = Range()
        # 填充消息...
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NewSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 3: 配置 setup.py**
```python
setup(
    # ...
    entry_points={
        'console_scripts': [
            'new_sensor_node = usv_new_sensor.new_sensor_node:main',
        ],
    },
)
```

**Step 4: 添加到启动文件** (`usv_bringup/launch/usv_launch.py`)
```python
# 在 generate_launch_description() 中添加
new_sensor_node = Node(
    package='usv_new_sensor',
    executable='new_sensor_node',
    name='new_sensor_node',
    namespace=namespace,
    output='screen',
    parameters=[param_file]
)

# 在返回的 LaunchDescription 中添加
return LaunchDescription([
    # ... 现有节点 ...
    new_sensor_node,
])
```

**Step 5: 构建和测试**
```bash
colcon build --packages-select usv_new_sensor
source install/setup.bash

# 单独测试节点
ros2 run usv_new_sensor new_sensor_node

# 或通过 launch 测试
ros2 launch usv_bringup usv_launch.py namespace:=usv_01
```

### 8. 版本控制工作流

**日常开发：**
```bash
# 创建功能分支
git checkout -b feature/emergency-stop

# 提交修改
git add gs_gui/gs_gui/usv_commands.py
git commit -m "feat(gui): 添加紧急停止命令

- 在 USVCommandHandler 中添加 emergency_stop 方法
- 在 GroundStationNode 中添加处理逻辑
- 添加相关测试用例"

# 推送到远程
git push origin feature/emergency-stop

# 创建 Pull Request（在 GitHub 上操作）
```

**提交消息规范：**
```
<type>(<scope>): <subject>

<body>

<footer>
```

类型（type）：
- `feat`: 新功能
- `fix`: 修复 bug
- `docs`: 文档修改
- `style`: 代码格式（不影响功能）
- `refactor`: 重构
- `test`: 添加测试
- `chore`: 构建/工具链修改

范围（scope）：
- `gui`: GUI 相关
- `control`: 控制逻辑
- `comm`: 通信模块
- `drivers`: 驱动程序
- `msgs`: 消息定义

### 9. 故障排查工作流

**问题：USV 不在线**
```bash
# 1. 检查节点是否启动
ros2 node list | grep usv_01

# 2. 检查命名空间是否正确
ros2 topic list | grep usv_01

# 3. 检查网络连接
ping <usv_ip>

# 4. 查看节点日志
ros2 run rqt_console rqt_console

# 5. 检查 MAVROS 连接
ros2 topic echo /usv_01/mavros/state
```

**问题：GUI 不显示状态**
```bash
# 1. 检查状态话题是否发布
ros2 topic hz /usv_01/usv_status

# 2. 检查消息内容
ros2 topic echo /usv_01/usv_status --once

# 3. 检查地面站节点是否订阅
ros2 node info /groundstationnode

# 4. 重启 GUI
# Ctrl+C 停止，然后重新启动
```

**问题：导航不工作**
```bash
# 1. 检查 Action 服务器是否运行
ros2 action list | grep navigate

# 2. 发送测试 Action 目标
ros2 action send_goal /usv_01/navigate_to_point \
    common_interfaces/action/NavigateToPoint \
    "goal: {pose: {position: {x: 10, y: 5, z: 0}}}"

# 3. 检查飞控状态
ros2 topic echo /usv_01/mavros/state

# 4. 查看控制节点日志
ros2 topic echo /rosout | grep usv_control
```

## 项目约定

### 命名规范

- **命名空间格式**: `usv_XX` (XX 为两位数字，如 usv_01)
- **节点名称**: 遵循 ROS 2 snake_case，如 `usv_status_node`
- **Topic 名称**: 小写下划线，如 `set_usv_target_position`
- **包名称**: 带前缀区分功能域
  - `gs_*`: 地面站相关（Ground Station）
  - `usv_*`: 机载相关（USV Onboard）
  - `common_*`: 共享接口

### 文件组织

```
usv_workspace/
├── src/
│   ├── common_interfaces/      # 消息和动作定义（CMake）
│   ├── gs_gui/                  # 地面站 GUI（ament_python）
│   │   ├── gs_gui/              # Python 源码
│   │   ├── resource/            # UI 文件、XML 任务
│   │   └── launch/              # 启动文件
│   ├── usv_control/             # 控制算法（ament_python）
│   ├── usv_drivers/             # 传感器驱动（ament_python）
│   └── usv_bringup/             # 启动配置（ament_python）
├── install/                     # 编译输出
├── build/                       # 构建临时文件
└── log/                         # 日志
```

### 参数配置

- **地面站参数**: `gs_bringup/config/gs_params.yaml`
  - `area_center_x/y/z`: 任务坐标系原点（在全局 map 中的位置）
  - `step_timeout`: 集群任务步骤超时
  - `offline_grace_period`: USV 离线判定宽限期

- **机载参数**: `usv_bringup/config/usv_params.yaml`
  - 传感器配置、控制参数、硬件接口

### QoS 策略

- **状态消息**: `BEST_EFFORT` (允许丢包，优先实时性)
- **命令消息**: `RELIABLE` (确保送达)

```python
qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
```

## 常见陷阱

### 1. 命名空间检测延迟

地面站通过 `get_node_names_and_namespaces()` 检测在线 USV，实现了**稳定性检测机制**：
- 需要连续 3 次检测结果一致才确认变化
- 离线判定有 5 秒宽限期（`offline_grace_period`，已从 20 秒优化）
- **Why**: 避免网络抖动或 ROS 图暂时不可见导致误判离线

**详见**：`gs_gui/OFFLINE_DETECTION_OPTIMIZATION.md`

### 2. 坐标系混淆

- XML 任务文件使用 **Area 坐标系**（相对坐标）
- MAVROS 使用 **USV 本地坐标系**（以 boot pose 为原点）
- 导航目标必须经过 `_area_to_global` → `_global_to_usv_local` 转换

**详见**：`gs_gui/AREA_OFFSET_GUIDE.md`

### 3. GUI 线程安全

- **ROS 回调运行在 ROS spin 线程**，不能直接操作 Qt 控件
- 使用 `PyQt Signal-Slot` 跨线程通信：
```python
# ROS 线程
self.ros_signal.state_update.emit(usv_states)

# Qt 主线程
self.ros_signal.state_update.connect(self.state_handler.receive_state_callback)
```

### 4. Action 句柄管理

发送新导航目标前必须取消旧任务，避免冲突：
```python
def _cancel_active_goal(self, usv_id):
    if usv_id in self._usv_active_goals:
        goal_handle = self._usv_active_goals[usv_id]
        goal_handle.cancel_goal_async()
        del self._usv_active_goals[usv_id]
```

### 5. 资源清理和错误处理

**使用 ResourceManager 管理资源生命周期：**
```python
# 初始化资源管理器
self.resource_manager = ResourceManager(self.get_logger())

# 注册资源（线程、队列等）
self.resource_manager.register_resource(
    "publish_queue",
    self.publish_queue,
    cleanup_func=lambda q: self._cleanup_queue(q)
)

# 在 shutdown() 中自动清理
self.resource_manager.cleanup_all()
```

**使用 ErrorHandler 进行健壮的错误处理：**
```python
from .error_handler import RobustErrorHandler, ErrorCategory, ErrorSeverity

# 初始化错误处理器
self.error_handler = RobustErrorHandler(self.get_logger())

# 处理错误并尝试恢复
self.error_handler.handle_error(
    exception,
    context="发送导航目标",
    severity=ErrorSeverity.ERROR,
    category=ErrorCategory.NETWORK
)
```

**详见**：`OPTIMIZATION_GUIDE.md`, `OPTIMIZATION_SUMMARY.md`

### 6. MAVROS 启动优化

**问题**：默认 MAVROS 启动耗时 ~97 秒（加载 60+ 插件 + 同步 900+ 参数）

**解决方案**（在 `usv_launch.py` 中配置）：
```python
'plugin_allowlist': [
    'sys_status',      # 系统状态（必需）
    'sys_time',        # 时间同步（必需）
    'command',         # 命令接口（解锁/模式切换）
    'local_position',  # 本地位置（导航必需）
    'setpoint_raw',    # 原始设定点（控制必需）
    'global_position', # GPS 全局位置
    'gps_status',      # GPS 状态和卫星数
]
# 移除 param 插件，避免参数同步阻塞
```

**效果**：启动时间从 97 秒降至 10-15 秒（节省 ~82 秒）

**详见**：`usv_bringup/MAVROS_STARTUP_OPTIMIZATION.md`

### 7. 优雅关闭机制

地面站关闭时自动向所有在线 USV 发送外设关闭命令：
```python
def closeEvent(self, event):
    """窗口关闭事件处理器"""
    online_usvs = self.list_manager.usv_online_list
    if online_usvs:
        # 发送关闭命令
        self.ros_signal.str_command.emit('led_off')
        self.ros_signal.str_command.emit('sound_stop')
        self.ros_signal.str_command.emit('neck_stop')
        # 等待 500ms 确保命令发送
        QTimer.singleShot(500, lambda: event.accept())
        event.ignore()
    else:
        event.accept()
```

**Why**: 确保 USV 外设处于安全状态，避免 LED/声音/舵机遗留在激活状态

**详见**：`gs_gui/GRACEFUL_SHUTDOWN.md`

## 调试技巧

```bash
# 查看活动 topic
ros2 topic list

# 监听特定 USV 的状态
ros2 topic echo /usv_01/usv_status

# 查看节点列表
ros2 node list

# 查看节点信息
ros2 node info /usv_01/usv_control_node

# 手动发送测试命令
ros2 topic pub /usv_01/gs_led_command std_msgs/msg/String "data: 'color_select|255,0,0'"

# 查看 tf 树
ros2 run tf2_tools view_frames

# 查看系统日志（集中式日志配置后）
tail -f ~/.logs/gs_gui.log
tail -f ~/.logs/gs_gui_error.log  # 仅错误日志
```

## 文档资源

### 核心架构和快速开始
- **项目概览**: `QUICK_START.md` - 快速开始指南
- **Markdown 管理**: `MARKDOWN_FILES_MANAGEMENT.md` - 文档索引和管理

### GUI 模块（gs_gui/）
- **详细架构**: `gs_gui/MODULE_ARCHITECTURE.md` - 模块关系和数据流
- **快速参考**: `gs_gui/QUICK_REFERENCE.md` - 常用操作速查
- **测试指南**: `gs_gui/TEST_GUIDE.md` - 测试框架和实践
- **重构总结**: `gs_gui/REFACTOR_SUMMARY.md` - 重构成果和改进
- **优雅关闭**: `gs_gui/GRACEFUL_SHUTDOWN.md` - 关闭流程和外设管理
- **坐标系统**: `gs_gui/AREA_OFFSET_GUIDE.md` - Area 坐标偏移设置
- **离线检测**: `gs_gui/OFFLINE_DETECTION_OPTIMIZATION.md` - 检测机制优化
- **LED 感染**: `gs_gui/LED_INFECTION_MODE.md` - LED 感染模式实现
- **温度滞后**: `gs_gui/TEMPERATURE_HYSTERESIS_UPDATE.md` - 温度监控优化
- **UI 现代化**: `gs_gui/UI_MODERNIZATION_GUIDE.md` - UI 设计指南
- **响应式设计**: `gs_gui/UI_RESPONSIVE_DESIGN.md` - 响应式布局
- **滚动实现**: `gs_gui/UI_SCROLL_IMPLEMENTATION.md` - 滚动区域实现

### 机载系统（usv_*/）
- **MAVROS 优化**: `usv_bringup/MAVROS_STARTUP_OPTIMIZATION.md` - 启动时间优化
- **电池百分比**: `usv_comm/BATTERY_PERCENTAGE_FIX.md` - 电池显示修复

### 系统优化
- **优化指南**: `OPTIMIZATION_GUIDE.md` - 完整优化方案
- **优化总结**: `OPTIMIZATION_SUMMARY.md` - 代码质量评估
- **MAVLink 超时**: `MAVLINK_COMMAND_TIMEOUT_GUIDE.md` - 命令超时处理
- **启动差异**: `USV_STARTUP_DIFFERENCES_ANALYSIS.md` - USV 启动对比

## 关键外部依赖

- **MAVROS**: 与 ArduPilot/PX4 飞控通信（需单独安装）
- **PyQt5**: GUI 框架（`python3-pyqt5`）
- **tf2_ros**: 坐标变换库
- **rplidar_ros**: 激光雷达驱动（可选）
- **adafruit-circuitpython**: 硬件控制库（机载可选）

---

**最后更新**: 2025-10-25 | **工作空间版本**: ROS 2 Humble/Iron
