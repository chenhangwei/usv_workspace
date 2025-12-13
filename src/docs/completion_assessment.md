# 完成度评估与建议（基于当前代码库）

> 说明：该评估以“你的目标需求”对照当前仓库实现情况，标注：✅已实现 / 🟡部分实现 / ❌缺失，并给出优先级建议。

## 1. 关键结论（先读这个）
- **step 任务框架已具备**（XML→GS→按 step 下发→USV result→GS ack→推进）。
- **“目标航向/航向误差”显示缺失的根因在 USV 端反馈**：`heading_error` 之前是常数（TODO）。我已在 [usv_control/usv_control_node.py](usv_control/usv_control_node.py) 补齐了基于方位角的 `heading_error` 与简单 ETA。
- **“姿态/yaw 作为任务输入”目前未贯通**：XML 里有 yaw，但 `cluster_controller` 下发目标时固定传 `yaw=0.0`（并且注释仍写 ArduPilot），这与 PX4 体系与“按 yaw 控制”目标不一致。
- **Domain bridge 脚本与实际 launch 文件不一致**：脚本引用的 `domain_bridge.launch.py` 在仓库中未找到，需要补齐或调整方案（建议统一用 Zenoh 或把 domain bridge 方案补完整）。

## 2. 需求对照矩阵

### 2.1 任务/编队
- ✅ XML 任务导入：`gs_gui/cluster_task_manager.py` 读取 `<step>` / `<usv>` / 位置 / yaw / velocity。
- ✅ 姿态字段贯通（消息层）：XML 的 roll/pitch/yaw 可被解析并通过 `NavigationGoal.target_pose.pose.orientation` 下发到 USV。
- ✅ step 下发与推进：`gs_gui/cluster_controller.py` 管理 `run_step`、ack 状态、超时重发、推进下一步。
- 🟡 ack 语义与阈值：
  - 已实现：基于 `navigation/result` 标记 ack。
  - 待明确：`MIN_ACK_RATE_FOR_PROCEED` 与“失败艇”策略（目前逻辑倾向“必须全部成功 ack 才进入下一步”，但又定义了 ack_rate 阈值，需要统一）。
- ❌ velocity 贯通：XML 中 `velocity` 尚未影响 PX4 setpoint（TrajectorySetpoint 当前 velocity 置 NaN）。

### 2.2 控制（PX4 Offboard）
- ✅ Offboard 必需心跳：`OffboardControlMode` 连续发布。
- ✅ 位置 setpoint：`TrajectorySetpoint.position` 发布。
- ✅ yaw 控制链路：GS 下发 yaw → USV 从 quaternion 取 yaw → `TrajectorySetpoint.yaw`。
- 🟡 roll/pitch 执行：USV 侧新增 2D/3D 模式（2D 忽略 roll/pitch），并可选发布 `fmu/in/trajectory_setpoint6dof` 携带 quaternion；PX4 侧是否消费/生效需在目标固件与控制器配置上验证。

### 2.3 定位（UWB）
- ✅ LinkTrack PB 串口解析 + 位置注入：`usv_drivers/usv_uwb_node.py` → `fmu/in/vehicle_visual_odometry`。
- 🟡 姿态/速度注入：当前 odometry orientation/velocity 设 NaN（可接受，但需确认 EKF 侧配置与预期一致）。
- ❌ EKF 参数基线文档：仓库未见对 PX4 EKF2/视觉定位参数的明确说明与版本约束。

### 2.4 通信与部署
- ✅ Zenoh bridge：GS/USV launch 均可启动 peer 模式（GS 监听 tcp/7447）。
- ✅ 分布式 SSH 启动：`gs_bringup/launch/gs_distributed_launch.py` + `config/usv_fleet.yaml`。
- 🟡 domain bridge：存在管理脚本，但缺少匹配的 launch 文件/配置闭环。

### 2.5 GUI/观测
- ✅ 状态聚合：`usv_comm/usv_status_node.py` 输出丰富状态（`UsvStatus.msg`）。
- 🟡 导航面板“目标航向”：之前依赖 `heading_error`（为 0），导致显示无意义；已通过 USV 端反馈修复。

## 3. 风险点（建议优先处理）

1) **PX4 vs ArduPilot 语义混用**
- `cluster_controller.py` 中多处注释与行为（例如“ArduPilot auto yaw”）与 PX4 Offboard 模式不一致，容易导致需求理解偏差。

2) **yaw 单位/坐标系不清晰**
- XML `yaw/value` 没有单位说明；GS→USV 代码默认按弧度转四元数；一旦用度输入会直接错。

3) **step 推进策略需要统一**
- 目前代码既有“all ack 才推进”的路径，也有“ack_rate 达阈值推进”的概念，实际行为容易出现你不期望的提前/延后推进。

## 4. 建议（按优先级）

### P0（立刻做，能明显提升可用性）
- 明确并文档化：坐标系（GS 输入是 NED 还是 ENU）、yaw 单位（弧度/度）。
- 统一 step 推进策略：
  - 选其一：全员成功才推进；或 ack_rate + 失败艇隔离策略（失败艇退出后续 step / 进入 HOLD / 人工介入）。

### P1（让“姿态目标点”真正落地）
- 在 `cluster_controller` 下发目标时，支持按任务 yaw 下发（至少提供一个开关参数，避免破坏现有行为）。
- 如果需要 roll/pitch：需要新增接口与 PX4 setpoint 类型/OffboardControlMode 配置（这属于一条独立设计线）。

### P2（规模化与运维）
- 补齐 domain bridge 方案：要么删除脚本并统一 Zenoh，要么把缺失的 launch/config 补齐并写清楚“什么时候用它”。
- 给 40+ 规模加“节流/分组策略”说明：例如 step 下发周期、重试间隔、GUI 刷新频率、日志级别等。

