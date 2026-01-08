# Copilot Instructions (USV Workspace)

You are assisting with a **ROS 2 + PX4 (uXRCE-DDS)** multi-USV/UAV cluster project.
Prioritize existing patterns, `common_interfaces` contracts, and `common_utils` libraries.

## 1. Architecture & Service Boundaries
- **Ground Station (GS)**: `gs_bringup`, `gs_gui`
  - **Core**: `gs_gui/ground_station_node.py` manages cluster state and task dispatch.
  - **Task Logic**: `cluster_controller.py` handles step logic; `cluster_task_manager.py` parses XML missions.
  - **Communication**: Zenoh bridge on `tcp/7447` for GS-USV link. Clock sync (chrony) is mandatory.
- **USV/UAV Implementation**: `usv_control`, `usv_drivers`
  - **Control**: `usv_control_node.py` maps `NavigationGoal` -> PX4 `TrajectorySetpoint`.
  - **Bridge**: `usv_comm` aggregates status (`UsvStatus`) from PX4.
- **Contracts**: `common_interfaces` contains ALL custom Msgs/Srvs (`NavigationGoal`, `UsvStatus`, etc.).

## 2. Critical Communication & QoS Rules
- **PX4 Interaction (uXRCE-DDS)**:
  - **QoS**: MUST be **BEST_EFFORT** (Reliability) / **VOLATILE** (Durability) for `fmu/*` topics.
  - **Control Loop**: Publish to `/fmu/in/trajectory_setpoint` & `/fmu/in/offboard_control_mode` > 2Hz or mode fails.
- **GS <-> USV**:
  - **QoS**: MUST be **RELIABLE** for command integrity (`navigation/goal`, `navigation/result`).
  - **Namespace**: `/usv_XX` prefix used for multi-agent isolation.

## 3. Coordinate Systems & Units
- **PX4**: Uses **NED** (North-East-Down) internally.
- **ROS 2/UI**: Uses **ENU** generally; conversion occurs at driver boundaries (`usv_control`, `usv_uwb`).
- **Heading/Yaw**:
  - Code: Radians `[-pi, pi]`.
  - Messages: Quaternions (`PoseStamped.orientation`).
- **Platform Modes**:
  - `platform_mode='2d'`: Ignored roll/pitch, Z is fixed height (Surface Vessel).
  - `platform_mode='3d'`: Full 6DoF control (UAV).

## 4. Coding Conventions & Patterns
- **Parameter Loading**: ALWAYS use `common_utils.ParamLoader`.
  ```python
  from common_utils import ParamLoader, ParamValidator
  loader = ParamLoader(self)
  rate = loader.load_param('rate', 20.0, ParamValidator.frequency, "Hz")
  ```
- **Logging**: Use `self.get_logger().info()` with structured prefixes if applicable.
- **Messages**: Import standard messages from `common_interfaces.msg` first.

## 5. Workflows & Commands
- **Build**: `colcon build --symlink-install` (source `install/setup.bash` after).
- **Launch GS**: `ros2 launch gs_bringup gs_launch.py`.
- **Launch USV Fleet**: `ros2 launch gs_bringup gs_distributed_launch.py` (reads `usv_fleet.yaml`).
- **Debugging**:
  - Check Zenoh: `journalctl -u zenoh-bridge` (if service) or logical logs.
  - Check Time Sync: `chronyc tracking` (critical for distributed stability).

## 6. Common Pitfalls
- **QoS Mismatch**: Subscribing to PX4 topics with default (Reliable) QoS will yield NO data.
- **Legacy Code**: `cluster_controller.py` contains ArduPilot legacy logic; strictly follow PX4 Offboard logic for new features.
- **Zero Quaternions**: Never initialize a quaternion as `0,0,0,0`; use `0,0,0,1` (w=1).
