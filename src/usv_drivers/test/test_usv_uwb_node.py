import unittest
from unittest.mock import MagicMock, patch
import sys
import os

# Ensure we import the local source, not the installed package
# Insert at index 0
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../common_utils')))

# Mock rclpy and its submodules BEFORE importing the module under test
mock_rclpy = MagicMock()
sys.modules['rclpy'] = mock_rclpy

mock_node_module = MagicMock()
sys.modules['rclpy.node'] = mock_node_module

mock_parameter_module = MagicMock()
sys.modules['rclpy.parameter'] = mock_parameter_module

mock_logging_module = MagicMock()
sys.modules['rclpy.logging'] = mock_logging_module

# Define a Mock Node class that UsvUwbNode will inherit from
class MockNode:
    def __init__(self, node_name, **kwargs):
        self.node_name = node_name
        self.logger = MagicMock()
        self.logger.warn = print
        self.logger.error = print
        self.logger.info = print
        self.logger.debug = print
    
    def create_timer(self, period, callback):
        return MagicMock()
    
    def create_publisher(self, msg_type, topic, qos_profile):
        return MagicMock()
    
    def get_logger(self):
        return self.logger
    
    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.to_msg.return_value = "time"
        return clock

mock_node_module.Node = MockNode

# Now import the module under test
import usv_drivers.usv_uwb_node
import importlib
importlib.reload(usv_drivers.usv_uwb_node)

from usv_drivers.usv_uwb_node import UsvUwbNode
from geometry_msgs.msg import PoseStamped

class TestUsvUwbNode(unittest.TestCase):
    def setUp(self):
        # Mock SerialResourceManager
        self.serial_patcher = patch('usv_drivers.usv_uwb_node.SerialResourceManager')
        self.mock_serial_manager_cls = self.serial_patcher.start()
        self.mock_serial_manager = self.mock_serial_manager_cls.return_value
        self.mock_serial_manager.open.return_value = True
        self.mock_serial_manager.is_open = True
        self.mock_serial_manager.serial_port = MagicMock()
        
        # Mock ParamLoader
        self.param_loader_patcher = patch('usv_drivers.usv_uwb_node.ParamLoader')
        self.mock_param_loader_cls = self.param_loader_patcher.start()
        self.mock_param_loader = self.mock_param_loader_cls.return_value
        # Setup default return values for params
        def load_param_side_effect(name, default, validator, desc):
            return default
        self.mock_param_loader.load_param.side_effect = load_param_side_effect

        # Instantiate the node
        self.node = UsvUwbNode()
        
        # Mock the publisher created in __init__
        self.node.uwb_pub = MagicMock()

    def tearDown(self):
        self.serial_patcher.stop()
        self.param_loader_patcher.stop()

    def test_parse_int24(self):
        # Test positive value: 0x0003E8 (1000)
        data = b'\xE8\x03\x00'
        self.assertEqual(self.node.parse_int24(data), 1000)
        
        # Test negative value: 0xFFFC18 (-1000) -> 0x1000000 - 1000 = 0xFFFC18
        # Little endian: 18 FC FF
        data = b'\x18\xFC\xFF'
        self.assertEqual(self.node.parse_int24(data), -1000)

    def test_read_and_publish_valid_frame(self):
        # Construct a valid frame
        # Header: 0x55, Func: 0x01
        frame = bytearray([0] * 128)
        frame[0] = 0x55
        frame[1] = 0x01
        
        # Position (x, y, z) * 1000
        # x = 1000mm (1m) -> 0x0003E8 -> E8 03 00
        frame[4] = 0xE8; frame[5] = 0x03; frame[6] = 0x00
        # y = 2000mm (2m) -> 0x0007D0 -> D0 07 00
        frame[7] = 0xD0; frame[8] = 0x07; frame[9] = 0x00
        # z = 3000mm (3m) -> 0x000BB8 -> B8 0B 00
        frame[10] = 0xB8; frame[11] = 0x0B; frame[12] = 0x00
        
        # Quaternion (q0, q1, q2, q3)
        # Just use 0.0 for simplicity, bytes 88-103 are already 0
        
        # Checksum
        checksum = sum(frame[:-1]) & 0xFF
        frame[127] = checksum
        
        # Mock serial reading
        self.mock_serial_manager.serial_port.in_waiting = 128
        self.mock_serial_manager.serial_port.read.return_value = bytes(frame)
        
        # Run the method
        self.node.read_and_publish()
        
        # Verify publisher was called
        self.assertTrue(self.node.uwb_pub.publish.called)
        msg = self.node.uwb_pub.publish.call_args[0][0]
        self.assertIsInstance(msg, PoseStamped)
        self.assertAlmostEqual(msg.pose.position.x, 1.0)
        self.assertAlmostEqual(msg.pose.position.y, 2.0)
        self.assertAlmostEqual(msg.pose.position.z, 3.0)

    def test_read_and_publish_invalid_checksum(self):
        # Construct a frame with invalid checksum
        frame = bytearray([0] * 128)
        frame[0] = 0x55
        frame[1] = 0x01
        frame[127] = 0x00 # Invalid checksum (should be something else)
        
        # Ensure the calculated checksum is not 0x00
        # With all zeros except header and func, sum is 0x56. 0x56 != 0x00.
        
        self.mock_serial_manager.serial_port.in_waiting = 128
        self.mock_serial_manager.serial_port.read.return_value = bytes(frame)
        
        self.node.read_and_publish()
        
        # Verify publisher was NOT called
        self.assertFalse(self.node.uwb_pub.publish.called)

if __name__ == '__main__':
    unittest.main()
