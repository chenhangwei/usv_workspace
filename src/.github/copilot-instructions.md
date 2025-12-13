# Copilot 工作指引（本仓库）

你在一个 ROS 2 + PX4(uXRCE-DDS) 的多艇/多机集群仓库中工作。请优先遵循仓库既有结构与话题约定，避免引入新的框架或重写架构。

## 1) 架构速览
- GS 侧：`gs_bringup/` + `gs_gui/`
  - 入口：`gs_bringup/launch/gs_launch.py`（GS 单机）与 `gs_distributed_launch.py`（SSH 批量启动 USV）
  - 核心节点：`gs_gui/ground_station_node.py`（目标下发、反馈/结果接收、动态发现、集群 step 控制）
  - 集群任务：`gs_gui/cluster_task_manager.py`（解析 XML step 任务）、`gs_gui/cluster_controller.py`（step 推进/超时重试/ack）
- USV 侧：`usv_bringup/` + `usv_control/` + `usv_comm/` + `usv_drivers/`
  - 入口：`usv_bringup/launch/usv_launch.py`
  - 控制：`usv_control/usv_control_node.py`（发布 `fmu/in/trajectory_setpoint` 与 `fmu/in/offboard_control_mode`，并发布导航 feedback/result）
  - 状态：`usv_comm/usv_status_node.py`（聚合 PX4 状态为 `UsvStatus`）
  - UWB：`usv_drivers/usv_uwb_node.py`（LinkTrack PB → `fmu/in/vehicle_visual_odometry`）

## 2) 关键话题与约定
- PX4 uXRCE-DDS：
  - 输入：`fmu/in/trajectory_setpoint`、`fmu/in/offboard_control_mode`、`fmu/in/vehicle_command`、`fmu/in/vehicle_visual_odometry`
  - 输出：`fmu/out/vehicle_status_v1`、`fmu/out/vehicle_local_position` 等
  - QoS：PX4 相关通常为 BEST_EFFORT
- GS⇄USV 导航（话题版本，跨域更稳）：
  - `navigation/goal`（`common_interfaces/NavigationGoal`）
  - `navigation/feedback`（`NavigationFeedback`）
  - `navigation/result`（`NavigationResult`）
  - QoS：RELIABLE

## 3) 多艇/跨域通信
- 每艇使用独立 namespace（如 `/usv_01`）。
- USV 组映射 domain（A-F → 11-16），GS 默认 domain 99。
- 跨网络建议使用 Zenoh bridge：GS 监听 tcp/7447，USV 连接。

## 4) 坐标系与单位（修改代码时必须显式说明）
- PX4 本地坐标：NED。
- `VehicleLocalPosition.heading` 单位：弧度，范围 -pi..pi。
- 任务 XML 中 `yaw/value` 目前未强制单位；在任何新增/修改中请明确“弧度/度”。
- 如涉及 ENU↔NED 转换，请集中在少数边界层完成，避免多处重复转换。

## 5) 开发原则（请遵守）
- 小步修改：优先修复根因，不要无关重构。
- 避免引入新依赖；如必须引入，先解释原因并保持最小化。
- 对跨域/跨机通信：优先使用 topic 方案，少用 Action。
- 对规模化（40+）：避免高频大量日志；定时器频率、重试间隔要可参数化。

## 6) 常见坑
- `cluster_controller.py` 中存在“ArduPilot”遗留语义；修改相关逻辑时，以 PX4 Offboard 为准并同步注释。
- `NavigationGoal` 不单独携带 yaw 字段：yaw 通过 `PoseStamped.pose.orientation`（quaternion）承载。
- 导航面板的“目标航向”依赖 `NavigationFeedback.heading_error`，如果不发布/一直为 0，会导致 UI 侧目标航向无意义。
