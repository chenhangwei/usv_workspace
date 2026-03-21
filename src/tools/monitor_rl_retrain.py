#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Monitor a running RL retraining job and its checkpoint progress.')
    parser.add_argument('--pid', type=int, help='Training worker PID to inspect.')
    parser.add_argument('--output', help='Optional output checkpoint path. Inferred from --pid when omitted.')
    parser.add_argument('--resume-from', dest='resume_from', help='Optional resume checkpoint path. Inferred from --pid when omitted.')
    parser.add_argument('--checkpoint-interval', type=int, default=None, help='Periodic checkpoint interval in timesteps. Inferred from --pid when omitted.')
    parser.add_argument('--total-timesteps', type=int, default=None, help='Target total timesteps. Inferred from --pid when omitted.')
    parser.add_argument('--interval', type=float, default=30.0, help='Polling interval in seconds when following.')
    parser.add_argument('--follow', action='store_true', help='Poll repeatedly until interrupted.')
    return parser.parse_args()


def _read_pid_cmdline(pid: int) -> list[str]:
    cmdline_path = Path(f'/proc/{pid}/cmdline')
    if not cmdline_path.exists():
        raise FileNotFoundError(f'PID does not exist: {pid}')
    raw = cmdline_path.read_bytes()
    return [part for part in raw.decode(errors='replace').split('\x00') if part]


def _extract_cli_value(arguments: list[str], option: str) -> str | None:
    for index, token in enumerate(arguments):
        if token == option and index + 1 < len(arguments):
            return arguments[index + 1]
        prefix = option + '='
        if token.startswith(prefix):
            return token[len(prefix):]
    return None


def _bool_process_exists(pid: int | None) -> bool:
    return pid is not None and Path(f'/proc/{pid}').exists()


def _run_ps(pid: int) -> str:
    command = ['ps', '-p', str(pid), '-o', 'pid=,etime=,%cpu=,%mem=,args=']
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return result.stdout.strip()


def _stdout_target(pid: int) -> str:
    target = Path(f'/proc/{pid}/fd/1')
    try:
        return os.readlink(target)
    except OSError as exc:
        return f'unavailable: {exc}'


def _load_payload(payload_path: Path) -> dict | None:
    if not payload_path.exists():
        return None
    try:
        import torch
    except ImportError:
        return None
    try:
        return torch.load(payload_path, map_location='cpu', weights_only=False)
    except Exception as exc:
        return {'_load_error': str(exc)}


def _payload_completed_timesteps(payload: dict | None) -> int:
    if not isinstance(payload, dict) or payload.get('_load_error'):
        return -1
    try:
        return int(payload.get('completed_timesteps', -1))
    except (TypeError, ValueError):
        return -1


def _latest_checkpoint(checkpoint_dir: Path) -> Path | None:
    if not checkpoint_dir.exists():
        return None
    checkpoints = sorted(checkpoint_dir.glob('*.pt'), key=lambda path: path.stat().st_mtime)
    return checkpoints[-1] if checkpoints else None


def _format_timestamp(epoch_seconds: float | None) -> str:
    if epoch_seconds is None:
        return 'n/a'
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(epoch_seconds))


def _print_snapshot(
    *,
    pid: int | None,
    output_path: Path | None,
    resume_path: Path | None,
    checkpoint_interval: int | None,
    total_timesteps: int | None,
) -> None:
    print('=' * 80, flush=True)
    print(f'timestamp: {_format_timestamp(time.time())}', flush=True)
    if pid is not None:
        print(f'pid: {pid}', flush=True)
        print(f'process_alive: {_bool_process_exists(pid)}', flush=True)
        if _bool_process_exists(pid):
            try:
                print(f'process: {_run_ps(pid)}', flush=True)
            except subprocess.SubprocessError as exc:
                print(f'process: unavailable ({exc})', flush=True)
            print(f'stdout: {_stdout_target(pid)}', flush=True)

    print(f'output_path: {output_path if output_path else "n/a"}', flush=True)
    print(f'resume_path: {resume_path if resume_path else "n/a"}', flush=True)
    print(f'checkpoint_interval: {checkpoint_interval if checkpoint_interval is not None else "n/a"}', flush=True)
    print(f'total_timesteps_target: {total_timesteps if total_timesteps is not None else "n/a"}', flush=True)
    if output_path is not None and output_path.exists():
        output_stat = output_path.stat()
        print(f'output_checkpoint_mtime: {_format_timestamp(output_stat.st_mtime)}', flush=True)
        print(f'output_checkpoint_size_bytes: {output_stat.st_size}', flush=True)

    latest_payload_path = None
    latest_payload = None
    output_payload = _load_payload(output_path) if output_path is not None and output_path.exists() else None
    checkpoint_dir = None
    if output_path is not None and checkpoint_interval and checkpoint_interval > 0:
        checkpoint_dir = output_path.with_name(f'{output_path.stem}_checkpoints')
        latest_payload_path = _latest_checkpoint(checkpoint_dir)
        if latest_payload_path is not None:
            latest_payload = _load_payload(latest_payload_path)

    resume_payload = _load_payload(resume_path) if resume_path is not None else None
    payload_candidates = [
        ('output', output_payload),
        ('latest_checkpoint', latest_payload),
        ('resume', resume_payload),
    ]
    reference_source, reference_payload = max(
        payload_candidates,
        key=lambda item: _payload_completed_timesteps(item[1]),
    )

    print(f'checkpoint_dir: {checkpoint_dir if checkpoint_dir else "n/a"}', flush=True)
    if checkpoint_dir is not None:
        print(f'checkpoint_dir_exists: {checkpoint_dir.exists()}', flush=True)
    print(f'latest_checkpoint: {latest_payload_path if latest_payload_path else "n/a"}', flush=True)
    if latest_payload_path is not None:
        stat = latest_payload_path.stat()
        print(f'latest_checkpoint_mtime: {_format_timestamp(stat.st_mtime)}', flush=True)
        print(f'latest_checkpoint_size_bytes: {stat.st_size}', flush=True)

    if latest_payload is not None and isinstance(latest_payload, dict) and latest_payload.get('_load_error'):
        print(f'latest_checkpoint_load_error: {latest_payload["_load_error"]}', flush=True)
    if output_payload is not None and isinstance(output_payload, dict) and output_payload.get('_load_error'):
        print(f'output_checkpoint_load_error: {output_payload["_load_error"]}', flush=True)
    if resume_payload is not None and isinstance(resume_payload, dict) and resume_payload.get('_load_error'):
        print(f'resume_checkpoint_load_error: {resume_payload["_load_error"]}', flush=True)

    if isinstance(reference_payload, dict) and '_load_error' not in reference_payload:
        completed_timesteps = int(reference_payload.get('completed_timesteps', 0))
        print(f'reference_payload: {reference_source}', flush=True)
        payload_total = reference_payload.get('total_timesteps')
        effective_total = total_timesteps if total_timesteps is not None else (int(payload_total) if payload_total is not None else None)
        next_checkpoint_step = reference_payload.get('next_checkpoint_step')
        if next_checkpoint_step is None and checkpoint_interval and checkpoint_interval > 0:
            next_checkpoint_step = ((completed_timesteps // checkpoint_interval) + 1) * checkpoint_interval

        print(f'completed_timesteps: {completed_timesteps}', flush=True)
        print(f'update_index: {reference_payload.get("update_index", "n/a")}', flush=True)
        print(f'next_checkpoint_step: {next_checkpoint_step if next_checkpoint_step is not None else "n/a"}', flush=True)
        if next_checkpoint_step is not None:
            print(f'steps_until_next_checkpoint: {max(0, int(next_checkpoint_step) - completed_timesteps)}', flush=True)
        if effective_total is not None and effective_total > 0:
            progress_ratio = completed_timesteps / effective_total
            print(f'progress_ratio: {progress_ratio:.2%}', flush=True)
            print(f'steps_remaining_to_target: {max(0, int(effective_total) - completed_timesteps)}', flush=True)
        scenarios = reference_payload.get('scenarios')
        if scenarios is not None:
            print(f'scenarios: {tuple(scenarios)}', flush=True)
    else:
        print('progress: unavailable (checkpoint metadata not readable yet)', flush=True)


def main() -> int:
    args = parse_args()
    pid = args.pid

    inferred_output = args.output
    inferred_resume = args.resume_from
    inferred_checkpoint_interval = args.checkpoint_interval
    inferred_total_timesteps = args.total_timesteps

    if pid is not None:
        cmdline = _read_pid_cmdline(pid)
        inferred_output = inferred_output or _extract_cli_value(cmdline, '--output')
        inferred_resume = inferred_resume or _extract_cli_value(cmdline, '--resume-from')
        checkpoint_interval_value = _extract_cli_value(cmdline, '--checkpoint-interval')
        total_timesteps_value = _extract_cli_value(cmdline, '--total-timesteps')
        if inferred_checkpoint_interval is None and checkpoint_interval_value is not None:
            inferred_checkpoint_interval = int(checkpoint_interval_value)
        if inferred_total_timesteps is None and total_timesteps_value is not None:
            inferred_total_timesteps = int(total_timesteps_value)

    output_path = Path(inferred_output).expanduser() if inferred_output else None
    resume_path = Path(inferred_resume).expanduser() if inferred_resume else None

    _print_snapshot(
        pid=pid,
        output_path=output_path,
        resume_path=resume_path,
        checkpoint_interval=inferred_checkpoint_interval,
        total_timesteps=inferred_total_timesteps,
    )

    if not args.follow:
        return 0

    try:
        while True:
            time.sleep(max(1.0, float(args.interval)))
            _print_snapshot(
                pid=pid,
                output_path=output_path,
                resume_path=resume_path,
                checkpoint_interval=inferred_checkpoint_interval,
                total_timesteps=inferred_total_timesteps,
            )
    except KeyboardInterrupt:
        print('monitor stopped', flush=True)
        return 0


if __name__ == '__main__':
    sys.exit(main())
