import argparse
import importlib.util
import json
import os
import shutil
from pathlib import Path


def _command_exists(name: str) -> bool:
    return shutil.which(name) is not None


def _module_info(name: str) -> dict:
    spec = importlib.util.find_spec(name)
    if spec is None:
        return {'found': False, 'version': None, 'error': 'module_not_found'}

    try:
        module = __import__(name)
        version = getattr(module, '__version__', None)
        if version is None and name == 'rclpy':
            version = 'installed'
        return {'found': True, 'version': str(version) if version is not None else None, 'error': None}
    except Exception as exc:
        return {'found': True, 'version': None, 'error': str(exc)}


def _torch_cuda_info() -> dict:
    info = _module_info('torch')
    if not info['found'] or info['error']:
        return {'available': False, 'device_count': 0}

    try:
        import torch
        return {
            'available': bool(torch.cuda.is_available()),
            'device_count': int(torch.cuda.device_count()),
        }
    except Exception:
        return {'available': False, 'device_count': 0}


def _path_status(path: str) -> dict:
    target = Path(path)
    return {
        'path': str(target),
        'exists': target.exists(),
        'is_dir': target.is_dir(),
        'writable': os.access(target, os.W_OK) if target.exists() else False,
    }


def collect_status() -> dict:
    modules = {
        name: _module_info(name)
        for name in ('numpy', 'torch', 'gymnasium', 'stable_baselines3', 'rclpy', 'casadi', 'mavros_msgs', 'serial')
    }

    ros_setup_path = Path('/opt/ros/jazzy/setup.bash')
    paths = {
        'workspace': _path_status('/mnt/workspace/usv_workspace'),
        'datasets': _path_status('/mnt/data/datasets/usv_rl'),
        'checkpoints': _path_status('/mnt/data/checkpoints/usv_rl'),
        'evals': _path_status('/mnt/data/evals/usv_rl'),
        'logs': _path_status('/mnt/data/logs/usv_rl'),
    }

    commands = {
        'python3': _command_exists('python3'),
        'pip3': _command_exists('pip3'),
        'ros2': _command_exists('ros2'),
        'colcon': _command_exists('colcon'),
    }

    readiness = {
        'ros_runtime_ready': bool(
            ros_setup_path.exists()
            and commands['ros2']
            and modules['rclpy']['found']
            and not modules['rclpy']['error']
        ),
        'mappo_train_ready': bool(
            commands['python3']
            and modules['numpy']['found']
            and not modules['numpy']['error']
            and modules['torch']['found']
            and not modules['torch']['error']
            and modules['casadi']['found']
            and not modules['casadi']['error']
            and ros_setup_path.exists()
            and commands['ros2']
            and commands['colcon']
            and modules['rclpy']['found']
            and not modules['rclpy']['error']
            and modules['mavros_msgs']['found']
            and not modules['mavros_msgs']['error']
        ),
        'ppo_optional_ready': bool(
            modules['gymnasium']['found']
            and not modules['gymnasium']['error']
            and modules['stable_baselines3']['found']
            and not modules['stable_baselines3']['error']
        ),
        'artifact_dirs_ready': all(item['exists'] and item['is_dir'] and item['writable'] for item in paths.values() if item['path'] != '/mnt/workspace/usv_workspace'),
    }

    missing = []
    if not ros_setup_path.exists():
        missing.append('/opt/ros/jazzy/setup.bash')
    if not commands['ros2']:
        missing.append('ros2 command')
    if not commands['colcon']:
        missing.append('colcon command')
    if not modules['rclpy']['found'] or modules['rclpy']['error']:
        missing.append('python module rclpy')
    if not modules['casadi']['found'] or modules['casadi']['error']:
        missing.append('python module casadi')
    if not modules['mavros_msgs']['found'] or modules['mavros_msgs']['error']:
        missing.append('python module mavros_msgs')
    if not modules['serial']['found'] or modules['serial']['error']:
        missing.append('python module serial (pyserial)')
    if not modules['torch']['found'] or modules['torch']['error']:
        missing.append('python module torch')
    if not modules['numpy']['found'] or modules['numpy']['error']:
        missing.append('python module numpy')

    return {
        'python_executable': shutil.which('python3'),
        'pip_executable': shutil.which('pip3'),
        'ros_setup_path': str(ros_setup_path),
        'ros_setup_exists': ros_setup_path.exists(),
        'commands': commands,
        'modules': modules,
        'torch_cuda': _torch_cuda_info(),
        'paths': paths,
        'readiness': readiness,
        'missing_required_items': missing,
    }


def _print_summary(status: dict) -> None:
    print('USV RL preflight summary')
    print(f"- ros_runtime_ready: {status['readiness']['ros_runtime_ready']}")
    print(f"- mappo_train_ready: {status['readiness']['mappo_train_ready']}")
    print(f"- ppo_optional_ready: {status['readiness']['ppo_optional_ready']}")
    print(f"- artifact_dirs_ready: {status['readiness']['artifact_dirs_ready']}")
    print('- commands:')
    for name, available in status['commands'].items():
        print(f'  - {name}: {available}')
    print('- modules:')
    for name, info in status['modules'].items():
        version = info['version'] if info['version'] is not None else 'unknown'
        error = f", error={info['error']}" if info['error'] else ''
        print(f"  - {name}: found={info['found']}, version={version}{error}")
    print('- missing_required_items:')
    if status['missing_required_items']:
        for item in status['missing_required_items']:
            print(f'  - {item}')
    else:
        print('  - none')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Check whether the current DSW environment is ready for USV RL training.')
    parser.add_argument('--output-json', help='Optional JSON output path.')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    status = collect_status()
    _print_summary(status)

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved preflight summary to {output_path}')

    raise SystemExit(0 if status['readiness']['mappo_train_ready'] else 1)


if __name__ == '__main__':
    main()