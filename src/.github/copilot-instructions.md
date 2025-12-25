# Copilot 工作指引（本仓库）

你在一个 ROS 2 + PX4(uXRCE-DDS) 的多艇/多机集群仓库中工作。请优先遵循仓库既有结构与话题约定，避免引入新的框架或重写架构。

## 1) 架构速览
- **GS 侧 (Ground Station)**：`gs_bringup/` + `gs_gui/`
  - **核心节点**：`gs_gui/ground_station_node.py`（目标下发、反馈接收、动态发现、集群控制）。
  - **任务管理**：`gs_gui/cluster_task_manager.py`（解析 XML 任务）、`gs_gui/cluster_controller.py`（step 推进/超时重试/ack）。
  - **入口**：`gs_launch.py`（单机）与 `gs_distributed_launch.py`（SSH 批量启动 USV）。
- **USV 侧 (Unmanned Surface Vehicle)**：`usv_bringup/` + `usv_control/` + `usv_comm/` + `usv_drivers/`
  - **控制核心**：`usv_control/usv_control_node.py`（发布 `fmu/in/trajectory_setpoint` 维持 Offboard）。
  - **状态聚合**：`usv_comm/usv_status_node.py`（聚合 PX4 状态为 `UsvStatus`）。
  - **定位注入**：`usv_drivers/usv_uwb_node.py`（UWB -> `fmu/in/vehicle_visual_odometry`）。

## 2) 关键话题与通信约定
- **PX4 uXRCE-DDS**：
  - 输入：`fmu/in/trajectory_setpoint` (位置/偏航), `fmu/in/offboard_control_mode` (心跳)。
  - 输出：`fmu/out/vehicle_status_v1`, `fmu/out/vehicle_local_position`。
  - **QoS**：PX4 相关话题必须使用 **BEST_EFFORT**。
- **GS ⇄ USV 导航**：
  - 话题：`navigation/goal` (`NavigationGoal`), `navigation/feedback`, `navigation/result`。
  - **QoS**：必须使用 **RELIABLE**，确保任务指令不丢失。
- **多艇隔离**：每艇使用独立 namespace（如 `/usv_01`）。
- **跨域通信**：使用 Zenoh bridge。GS 监听 `tcp/7447`，USV 连接。**必须保证 GS 与 USV 时钟同步**（建议使用 chrony）。

## 3) 坐标系与单位
- **PX4 内部**：使用 **NED** 坐标系。
- **GS/GUI**：可能使用 **ENU**，转换应集中在 `usv_control_node.py` 或 `usv_uwb_node.py` 的边界层。
- **单位**：
  - `VehicleLocalPosition.heading`：弧度，范围 `[-pi, pi]`。
  - `NavigationGoal`：yaw 通过 `PoseStamped.pose.orientation` (四元数) 承载。
  - XML 任务文件：`yaw/value` 需明确单位（代码中通常处理为弧度）。

## 4) 开发规范与模式
- **参数加载**：优先使用 `common_utils.ParamLoader` 进行参数声明与验证。
  ```python
  loader = ParamLoader(self)
  rate = loader.load_param('publish_rate', 20.0, ParamValidator.frequency)
  ```
- **Offboard 维持**：USV 必须以高频（>2Hz，通常 20Hz）持续发布 `offboard_control_mode` 和 `trajectory_setpoint`，否则 PX4 会退出 Offboard 模式。
- **2D vs 3D 模式**：通过 `platform_mode` 参数区分。2D 模式下忽略 roll/pitch。
- **避免 Action**：跨机通信优先使用 Topic + Ack 机制（见 `NavigationAck`），少用 ROS 2 Action 以增强弱网鲁棒性。

## 5) 关键工作流
- **编译**：`colcon build --symlink-install`。
- **启动 GS**：`ros2 launch gs_bringup gs_launch.py`。
- **批量启动 USV**：修改 `usv_fleet.yaml` 后运行 `ros2 launch gs_bringup gs_distributed_launch.py`。
- **调试**：若 Zenoh 报错 `timestamp exceeding delta`，检查 `chronyc sources -v`。

## 6) 常见坑点
- **ArduPilot 遗留**：`cluster_controller.py` 中部分逻辑源自 ArduPilot，修改时以 PX4 Offboard 逻辑为准。
- **目标航向**：UI 侧“目标航向”显示依赖 `NavigationFeedback.heading_error`，必须正确计算并发布。
- **QoS 不匹配**：订阅 PX4 话题时若不设为 BEST_EFFORT，将接收不到任何数据。