# Contributing to USV Workspace

Thank you for your interest in contributing to this project! This document provides guidelines and instructions for contributing.

## Code of Conduct

Please be respectful and constructive in all interactions. We are committed to providing a welcoming and inclusive experience for everyone.

## How to Contribute

### Reporting Issues

1. Check existing [issues](https://github.com/chenhangwei/usv_workspace/issues) to avoid duplicates.
2. Use a clear and descriptive title.
3. Provide detailed steps to reproduce the issue.
4. Include your environment details (OS, ROS 2 version, Python version).

### Submitting Changes

1. Fork the repository.
2. Create a feature branch from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Make your changes following the coding standards below.
4. Test your changes thoroughly.
5. Commit with clear, descriptive messages:
   ```bash
   git commit -m "feat(usv_control): add PID tuning module"
   ```
6. Push to your fork and submit a Pull Request.

### Commit Message Format

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

**Types**: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`, `ci`

**Scope**: Package name (e.g., `usv_control`, `gs_gui`, `common_utils`)

## Coding Standards

### Python

- Follow [PEP 8](https://peps.python.org/pep-0008/) style guide.
- Maximum line length: 120 characters.
- Use type hints where practical.
- All public functions and classes must have docstrings ([PEP 257](https://peps.python.org/pep-0257/)).
- All source files must include the Apache 2.0 license header:

```python
# Copyright (c) 2026 Chen Hangwei
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```

### ROS 2 Conventions

- Node names: `snake_case` (e.g., `usv_control_node`)
- Topic names: `snake_case` with namespace (e.g., `/usv_01/status`)
- Service names: `snake_case` (e.g., `set_mode`)
- Parameter names: `snake_case` (e.g., `max_speed`)
- Package names: `snake_case` (e.g., `usv_control`)

### Testing

- Write unit tests for all new functionality.
- Use `pytest` for Python tests.
- Tests must pass before submitting a PR:
  ```bash
  colcon test --packages-select <package_name>
  colcon test-result --verbose
  ```

### ROS 2 Linting

Ensure your code passes standard ROS 2 linters:

```bash
ament_copyright <package_path>
ament_flake8 <package_path>
ament_pep257 <package_path>
```

## Development Setup

1. Set up the workspace following the [README](README.md).
2. Install development tools:
   ```bash
   pip3 install flake8 pytest mypy
   ```
3. Use `colcon build --symlink-install` for development builds.

## License

By contributing to this project, you agree that your contributions will be licensed under the Apache License 2.0.
