# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Project-wide standardization to ROS 2 conventions
- Apache 2.0 license headers on all source files
- Top-level README.md with architecture overview
- CONTRIBUTING.md with development guidelines
- .gitignore for ROS 2 workspace
- CHANGELOG.md (this file)

### Changed
- Unified `package.xml` format across all packages (package format 3)
- Unified `setup.py` metadata (license identifier, classifiers, Python version)
- Standardized copyright headers to Apache 2.0 boilerplate
- Fixed email consistency across all packages
- Fixed version consistency (all packages start at 0.0.0)

### Fixed
- Typo in `usv_bringup/setup.cfg` (`usv_bingup` → `usv_bringup`)
- Missing `setup.cfg` for `usv_sim` package
- Inconsistent maintainer email in `common_utils`

## [0.0.0] - 2026-01-26

### Added
- Initial release of USV Workspace
- `common_interfaces` - Custom ROS 2 message definitions
- `common_utils` - Shared utility classes
- `gs_bringup` - Ground station launch management
- `gs_gui` - PyQt5-based ground station GUI
- `usv_action` - Custom action interfaces
- `usv_bringup` - USV onboard launch management
- `usv_comm` - Communication modules
- `usv_control` - Control algorithms (path planning, avoidance, formation)
- `usv_drivers` - Hardware drivers (LiDAR, UWB, ultrasonic)
- `usv_fan` - Fan temperature control
- `usv_led` - LED indicator control
- `usv_sim` - Simulation (SITL) launch files
- `usv_sound` - Audio alert system
- `usv_tf` - TF coordinate transformations

[Unreleased]: https://github.com/chenhangwei/usv_workspace/compare/v0.0.0...HEAD
[0.0.0]: https://github.com/chenhangwei/usv_workspace/releases/tag/v0.0.0
