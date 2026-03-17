import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class ControllerSession:
    namespace: str
    process: Optional[subprocess.Popen] = None
    log_path: Optional[Path] = None
    _log_handle: Optional[object] = None

    def start(self):
        if self.process is not None:
            return

        self.log_path = Path('/tmp') / f'usv_rl_{self.namespace}_controller.log'
        self._log_handle = self.log_path.open('w', encoding='utf-8')

        cmd = [
            'ros2',
            'run',
            'usv_control',
            'velocity_controller_node',
            '--ros-args',
            '-r',
            f'__ns:=/{self.namespace}',
            '-p',
            'apf_enabled:=true',
            '-p',
            'apf_orca_enabled:=false',
            '-p',
            'require_guided_mode:=true',
            '-p',
            'require_armed:=true',
        ]
        self.process = subprocess.Popen(
            cmd,
            stdout=self._log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        time.sleep(1.5)

    def read_log_tail(self, max_lines: int = 80) -> str:
        if self.log_path is None or not self.log_path.exists():
            return ''
        try:
            lines = self.log_path.read_text(encoding='utf-8', errors='replace').splitlines()
        except OSError:
            return ''
        return '\n'.join(lines[-max_lines:])

    def stop(self):
        if self.process is None:
            return

        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            self.process = None
            return

        try:
            self.process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.wait(timeout=5.0)
        finally:
            if self._log_handle is not None:
                self._log_handle.close()
                self._log_handle = None
            self.process = None