import subprocess
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class SitlSession:
    namespace: str
    launch_file: str = 'sitl_launch.py'
    package: str = 'usv_sim'
    process: Optional[subprocess.Popen] = None

    def start(self):
        if self.process is not None:
            return
        cmd = [
            'ros2',
            'launch',
            self.package,
            self.launch_file,
            f'namespace:={self.namespace}',
        ]
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(2.0)

    def stop(self):
        if self.process is None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5.0)
        finally:
            self.process = None