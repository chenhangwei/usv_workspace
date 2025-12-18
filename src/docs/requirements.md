# USV/无人球集群系统需求文档（基于当前代码库梳理）

> 范围：本文件基于当前 `usv_workspace/src` 代码与配置梳理“系统应该做什么”。它既包含你描述的目标形态（40+ 集群、UWB 室内、step 任务推进），也标注了代码里已经实现/默认采用的约束与协议。

## 1. 系统概述

### 1.1 目标
- 地面站（GS）统一管理 40+ USV/无人球（以下统称 USV）。
- 室内无 GPS，使用 UWB（Nooploop LinkTrack PB）提供位置，向 PX4 注入 `VehicleOdometry`（visual odometry）。
- GS 下发分步骤（step）的任务：每步为一组 USV 的目标点（位置 + 可选航向/姿态 + 可选速度）。
- 任务推进策略：基于每艇“确认/到达”反馈（ack），按阈值或全员完成进入下一步。

### 1.2 组成
- **USV 端（每艇）**
  - `usv_bringup/launch/usv_launch.py`：启动 XRCE Agent、Zenoh bridge、控制/指令/状态/UWB 等节点。
  - `usv_control/usv_control_node.py`：PX4 Offboard 位置/偏航 setpoint 发布 + 导航反馈/结果。
  - `usv_comm/usv_status_node.py`：PX4 状态聚合发布（给 GS/UI）。
  - `usv_drivers/usv_uwb_node.py`：串口读 UWB → 坐标变换 → 发布 `fmu/in/vehicle_visual_odometry`。
- **GS 端**
  - `gs_bringup/launch/gs_launch.py` / `gs_distributed_launch.py`：启动 GUI、可选 Zenoh、并可通过 SSH 批量启动各 USV。
  - `gs_gui/ground_station_node.py`：核心 ROS2 节点；发送导航目标、接收反馈/结果、集群控制、动态发现。
  - `gs_gui/cluster_task_manager.py`：读取任务 XML（step/usv/position/yaw/velocity）。
  - `gs_gui/cluster_controller.py`：step 推进 + ack/超时/重试。
- **接口定义**
  - `common_interfaces/msg/*`：`NavigationGoal/Feedback/Result`、`UsvStatus`、`UsvSetPoint` 等。
  - `px4_msgs/`：PX4 uXRCE-DDS 消息（TrajectorySetpoint、VehicleLocalPosition 等）。

## 2. 运行与通信约束（需求）

### 2.1 命名空间与多艇隔离
- 每艇必须使用独立 ROS2 namespace（示例：`/usv_01`、`/usv_02`…）。
- GS 应能动态发现当前可用艇，并维护在线/离线状态（含宽限期）。

### 2.2 Domain/跨网通信
- 支持按“组”映射 ROS_DOMAIN_ID（USV 侧 `A-F -> 11-16`），GS 默认 domain 99。
- 跨 domain/跨网络传输优先使用 **Zenoh bridge**（GS 监听 tcp/7447；USV 连接该地址）。
- 可选使用 domain bridge（仓库已有脚本，但 launch 文件需配齐）。

### 2.3 QoS 约定
- PX4 uXRCE-DDS 话题：**BEST_EFFORT**（与 PX4 匹配）。
- GS⇄USV 的导航/控制话题：默认 **RELIABLE**，避免丢包导致 step 推进异常。

### 2.4 时钟同步（Zenoh 必需）
- 在使用 Zenoh bridge 跨机/跨网时，GS 与所有 USV **必须进行系统时钟同步**（建议 NTP/chrony）。
- 若看到类似日志：`incoming timestamp ... exceeding delta 500ms is rejected`，通常表示 **远端机器时间比本机“快”超过 ~500ms**。
  - Zenoh 可能会对样本“重打时间戳”，但会持续刷 ERROR，并可能在高负载/抖动时引入额外问题。
- 建议运维动作（按现场网络条件二选一）：
  - 有外网：所有机器启用系统 NTP（如 `timedatectl set-ntp true`）。
  - 无外网：以 GS 为局域网 NTP 服务器（chrony），USV 指向 GS，同步到同一时间源。

## 3. 任务模型与行为需求

### 3.1 任务输入（XML）
- 一个任务由多个 `<step>` 组成；每个 step 有多个 `<usv>` 目标。
- 每个 `<usv>` 至少包含：
  - `usv_id`
  - `position/x`, `position/y`（`z` 可选）
- 可选包含：
  - `roll/value`：目标翻滚角（建议明确单位：弧度/度）
  - `pitch/value`：目标俯仰角（建议明确单位：弧度/度）
  - `yaw/value`：目标偏航角（建议明确单位：弧度/度）
  - `velocity/value`：期望速度（目前更多是规划字段，需定义是否影响 PX4 setpoint）

### 3.2 任务执行（step 推进）
- GS 周期性下发“当前 step 的目标”。
- 每艇收到目标后进入执行；USV 端周期性回传：
  - `navigation/feedback`：至少包含 `distance_to_goal`、`heading_error`、`estimated_time`。
  - `navigation/result`：到达/超时/取消等结果。
- GS 应维护 per-USV 的 ack 状态（成功/失败/等待/超时重试计数），并据此推进 step。

### 3.3 失败与重试
- 对“未响应”的艇：
  - `step_timeout`（默认 25s）用于判定“未响应/未确认”，触发重发（`max_retries` 默认 3）。
- 对“导航执行”层面的超时：
  - `cluster_action_timeout`（默认 300s）用于单个目标的执行上限（由 USV 端 result 触发）。
- 任务推进策略至少支持：
  - 全员 ack 才推进；或
  - ack_rate ≥ 阈值（默认 0.8）推进（需要明确“失败艇如何处理”的语义）。

## 4. 控制与定位需求

### 4.1 PX4 Offboard 控制
- USV 端必须持续发布：
  - `fmu/in/offboard_control_mode`（维持 OFFBOARD）
  - `fmu/in/trajectory_setpoint`（位置控制 + yaw 控制）
- 平台模式限制（必须可配置）：
  - **3D 模式**：允许 roll/pitch/yaw 目标（是否启用 6DoF setpoint 由参数开关决定）。
  - **2D 模式**：忽略 roll/pitch，只保留位置 + yaw。
- 控制坐标系需一致：
  - PX4 本地坐标为 NED。
  - 如 GS/GUI 输入为 ENU，需要明确并在一个位置统一转换。

### 4.2 UWB 定位注入
- UWB 节点从串口读取 LinkTrack PB 帧并发布 `fmu/in/vehicle_visual_odometry`。
- 需支持参数化：轴交换/翻转/旋转、坐标系 ENU→NED。
- 无 GPS 场景下，系统需定义 PX4 侧 EKF2/视觉定位相关参数基线（文档化）。

## 5. 观测与诊断需求
- GS 能显示：每艇位置/速度/航向、电池、模式/解锁状态、failsafe、传感器健康。
- 导航面板应显示：距离、航向误差、目标航向（由当前航向 + 误差推导或直接显示）。
- 应能回溯 PX4 事件（`/rosout` 解码）用于定位问题。

## 6. 非功能需求
- **规模**：40+ USV 并发。
- **鲁棒性**：短时网络抖动不应导致任务错误推进；需要明确“离线/超时/失败艇”处理。
- **一致性**：坐标系与单位（yaw/速度/距离）必须在 GS/USV/PX4 端一致并可追溯。
- **可运维性**：分布式一键启动/停止；日志可定位单艇问题；配置集中管理（fleet/yaml、zenoh/json5）。
