# USV Workspace

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-green.svg)](https://docs.ros.org/en/humble/)

A ROS 2 workspace for Unmanned Surface Vehicle (USV) fleet management, including autonomous navigation, formation control, obstacle avoidance, and ground station GUI.

## Overview

This workspace contains a collection of ROS 2 packages for operating single or multiple USVs. It covers the full stack from low-level hardware drivers to high-level mission planning and a graphical ground station interface.

### Architecture

```
Ground Station (gs_*)          USV Onboard (usv_*)
┌──────────────────┐           ┌──────────────────────────┐
│  gs_gui          │           │  usv_control             │
│  gs_bringup      │◄─────────►│  usv_comm                │
│                  │  ROS 2    │  usv_drivers             │
│                  │  DDS      │  usv_bringup             │
└──────────────────┘           │  usv_action              │
                               │  usv_tf / usv_sim        │
                               │  usv_fan / usv_led /     │
                               │  usv_sound               │
                               └──────────────────────────┘
                                        │
                               ┌────────┴────────┐
                               │ common_utils     │
                               │ common_interfaces│
                               └─────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| **common_interfaces** | Custom ROS 2 message definitions for USV systems |
| **common_utils** | Shared utility classes (parameter loading, resource management) |
| **gs_bringup** | Ground station launch files and configurations |
| **gs_gui** | PyQt5-based graphical ground station interface |
| **usv_action** | Custom action interfaces for multi-step missions |
| **usv_bringup** | USV onboard system launch files and configurations |
| **usv_comm** | Communication modules (status, GPS, command relay) |
| **usv_control** | Control algorithms (path planning, avoidance, formation) |
| **usv_drivers** | Hardware drivers (LiDAR, UWB, ultrasonic sensors) |
| **usv_fan** | Fan control for onboard temperature regulation |
| **usv_led** | LED indicator control |
| **usv_sim** | Simulation launch files for SITL testing |
| **usv_sound** | Audio alert and feedback system |
| **usv_tf** | TF coordinate transformations |

## Prerequisites

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: >= 3.8
- **Additional Dependencies**:
  - [MAVROS](https://github.com/mavlink/mavros) (for autopilot communication)
  - [rplidar_ros](https://github.com/Slamtec/rplidar_ros) (for LiDAR driver)
  - PyQt5 (for ground station GUI)

## Installation

### 1. Clone the Repository

```bash
mkdir -p ~/usv_workspace/src
cd ~/usv_workspace/src
git clone https://github.com/chenhangwei/usv_workspace.git .
```

### 2. Install Dependencies

```bash
cd ~/usv_workspace
rosdep install --from-paths src --ignore-src -r -y
pip3 install PyQt5
```

### 3. Build

```bash
cd ~/usv_workspace
colcon build --symlink-install
```

### 4. Source the Workspace

```bash
source ~/usv_workspace/install/setup.bash
```

## Quick Start

### Launch Ground Station

```bash
ros2 launch gs_bringup gs_launch.py
```

### Launch USV Onboard System

```bash
ros2 launch usv_bringup usv_launch.py
```

### Launch Simulation (SITL)

```bash
ros2 launch usv_sim usv_sim_launch.py
```

## Custom Messages

The `common_interfaces` package defines the following message types:

| Message | Description |
|---------|-------------|
| `UsvStatus` | Complete USV state information |
| `UsvSetPoint` | Target setpoint for control |
| `NavigationGoal` | Navigation mission goal |
| `NavigationFeedback` | Real-time navigation progress |
| `NavigationResult` | Mission completion result |
| `MpcDebug` | Model Predictive Control debug data |
| `FormationConfig` | Fleet formation configuration |
| `NeighborPose` | Neighbor USV position |
| `FleetNeighborPoses` | All neighbors' positions |

## Project Structure

```
usv_workspace/src/
├── LICENSE                    # Apache License 2.0
├── README.md                  # This file
├── CONTRIBUTING.md            # Contribution guidelines
├── CHANGELOG.md               # Version history
├── common_interfaces/         # Custom ROS 2 messages (ament_cmake)
├── common_utils/              # Shared utilities (ament_python)
├── gs_bringup/                # Ground station launch (ament_python)
├── gs_gui/                    # Ground station GUI (ament_python)
├── usv_action/                # Action interfaces (ament_python)
├── usv_bringup/               # USV launch files (ament_python)
├── usv_comm/                  # Communication modules (ament_python)
├── usv_control/               # Control algorithms (ament_python)
├── usv_drivers/               # Hardware drivers (ament_python)
├── usv_fan/                   # Fan control (ament_python)
├── usv_led/                   # LED control (ament_python)
├── usv_sim/                   # Simulation (ament_python)
├── usv_sound/                 # Sound system (ament_python)
├── usv_tf/                    # TF transforms (ament_python)
├── tools/                     # Development and analysis scripts
└── docs/                      # Documentation
```

## Testing

Run all tests:

```bash
cd ~/usv_workspace
colcon test
colcon test-result --verbose
```

Run tests for a specific package:

```bash
colcon test --packages-select usv_control
```

## Contributing

Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on how to contribute to this project.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

Copyright (c) 2026 Chen Hangwei
