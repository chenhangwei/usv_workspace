import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from usv_rl.policy_trace import PolicyTrace


class PolicyTraceTests(unittest.TestCase):
    def test_metadata_and_action_are_readable_before_close(self):
        with tempfile.TemporaryDirectory() as directory:
            warning = Mock()
            trace = PolicyTrace(directory, '/fleet/usv_03', {'action_mode': 'speed_scale'}, warning)
            trace.write({'type': 'action', 'timestamp': 12.0, 'actor_action': [0.5, 0.1]})
            records = [json.loads(line) for line in trace.path.read_text().splitlines()]
            self.assertEqual(records[0]['schema'], 'policy_trace_v1')
            self.assertEqual(records[0]['action_mode'], 'speed_scale')
            self.assertEqual(records[1]['actor_action'], [0.5, 0.1])
            trace.close()
            trace.close()
            self.assertFalse(trace.enabled)
            warning.assert_not_called()

    def test_directory_failure_disables_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            blocker = Path(directory) / 'not_a_directory'
            blocker.touch()
            warning = Mock()
            trace = PolicyTrace(blocker, 'usv_03', {}, warning)
            trace.write({'type': 'action'})
            self.assertFalse(trace.enabled)
            warning.assert_called_once()

    def test_write_failure_does_not_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            warning = Mock()
            trace = PolicyTrace(directory, 'usv_03', {}, warning)
            trace.close()
            broken_stream = Mock()
            broken_stream.write.side_effect = OSError('disk full')
            broken_stream.close.side_effect = OSError('disk full')
            trace._stream = broken_stream
            trace.write({'type': 'action'})
            trace.write({'type': 'action'})
            self.assertFalse(trace.enabled)
            warning.assert_called_once()


if __name__ == '__main__':
    unittest.main()