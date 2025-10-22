# USV Workspace Copilot Instructions

## 项目概述
这是一个无人水面艇(USV)集群控制系统，采用ROS2架构，支持多艇协同作业。系统分为地面站(Ground Station)和无人艇(USV)两个主要部分。

## 核心架构

### 命名空间体系
- **地面站**: 无命名空间，运行GUI和集群管理
- **无人艇**: 使用命名空间如`usv_01`, `usv_02`等区分不同艇只
- MAVROS系统ID自动从命名空间推导（如`usv_02` → `tgt_system=2`）

### 关键数据流
1. **状态上报**: USV → `usv_state`(UsvStatus) → 地面站
2. **导航控制**: 地面站 → `navigate_to_point`(Action) → USV  
3. **飞控通信**: MAVROS双向桥接MAVLink协议

## 启动和构建

### 标准启动流程
```bash
# 构建工作空间
colcon build

# 启动地面站
ros2 launch gs_bringup gs_launch.py

# 启动单个USV (需要指定串口等参数)
ros2 launch usv_bringup usv_launch.py namespace:=usv_01 fcu_url:=serial:///dev/ttyACM0:921600
```

### 参数配置
- USV参数: `usv_bringup/config/usv_params.yaml` 
- 地面站参数: `gs_bringup/config/gs_params.yaml`

## 编码约定

### 消息和接口
- 所有自定义消息定义在`common_interfaces`包中
- 使用中文注释说明字段含义和用途
- 关键消息：
  - `UsvStatus`: 无人艇完整状态信息
  - `NavigateToPoint.action`: 导航到指定点的Action接口

### 节点命名和结构
- 节点名使用下划线分隔：`usv_status_node`, `usv_control_node`
- 包名反映功能域：`usv_comm`(通信), `usv_control`(控制), `gs_gui`(地面站界面)
- 每个可执行文件都有对应的launch文件入口

### QoS配置模式
```python
# 标准QoS配置模板
qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
```

### 参数处理模式
```python
# 参数声明和获取模式
self.declare_parameter('param_name', default_value)
param_value = self.get_parameter('param_name').value
```

## 关键集成点

### MAVROS集成
- USV通过MAVROS与Pixhawk飞控通信
- 使用`/mavros/setpoint_position/local`发送位置控制指令
- 监听`/mavros/state`获取飞控状态

### PyQt5 GUI架构
- 主界面: `gs_gui/main_gui_app.py`
- ROS信号桥: `ros_signal.py`使用Qt信号连接ROS回调与GUI更新
- 表格更新模式：ROSSignal → 主线程 → 表格行更新

### Action服务器模式
导航控制使用ROS2 Action模式：
- 服务器: `navigate_to_point_server.py`
- 客户端: 地面站通过ActionClient发送导航目标
- 反馈: 实时距离、航向误差、预计时间

## 调试和故障排除

### 常见问题
1. **串口权限**: 确保用户在dialout组中 `sudo usermod -a -G dialout $USER`
2. **MAVROS连接**: 检查`fcu_url`参数和硬件连接
3. **命名空间问题**: 确保topic名称包含正确的命名空间前缀

### 日志和监控
- 使用`ros2 topic list`查看活跃话题
- `ros2 node list`查看运行节点
- GUI界面实时显示USV连接状态和电池信息

## 扩展指南

### 添加新USV节点
1. 在对应包中创建节点文件
2. 在`usv_launch.py`中添加节点配置
3. 在`usv_params.yaml`中添加参数配置
4. 遵循现有的命名空间和QoS模式

### 添加新传感器
传感器驱动放在`usv_drivers`包中，按照现有模式：
- 继承`rclpy.node.Node`
- 使用参数文件配置串口等硬件参数  
- 发布标准ROS2消息类型(sensor_msgs等)