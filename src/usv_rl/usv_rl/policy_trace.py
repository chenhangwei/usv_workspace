from datetime import datetime
import json
import os
from pathlib import Path


class PolicyTrace:
    def __init__(self, directory, namespace, metadata, on_error):
        self._stream = None
        self._on_error = on_error
        self.path = None
        try:
            folder = Path(directory).expanduser()
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            name = namespace.strip('/').replace('/', '_')
            self.path = folder / f'policy_trace_{name}_{stamp}_{os.getpid()}.jsonl'
            self._stream = self.path.open('x', encoding='utf-8', buffering=1)
            self.write({'type': 'metadata', 'schema': 'policy_trace_v1', **metadata})
        except OSError as exc:
            self._disable(exc)

    @property
    def enabled(self):
        return self._stream is not None

    def write(self, record):
        if self._stream is None:
            return
        try:
            self._stream.write(json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n')
        except (OSError, ValueError, TypeError) as exc:
            self._disable(exc)

    def _disable(self, error):
        self.close()
        self._on_error(f'Policy trace disabled: {error}')

    def close(self):
        stream, self._stream = self._stream, None
        if stream is not None:
            try:
                stream.close()
            except OSError:
                pass