"""
Common utilities package for USV project.

This package provides reusable utility classes for:
- Parameter loading with validation
- Resource management (serial ports, processes)
- Thread safety decorators
- Performance monitoring
"""

__version__ = '1.0.0'

from .param_loader import ParamLoader, ParamValidator
from .serial_manager import SerialResourceManager
from .process_tracker import ProcessTracker
from .thread_safety import thread_safe, ThreadSafeDict
from .geo_utils import GeoUtils

__all__ = [
    'ParamLoader',
    'ParamValidator',
    'SerialResourceManager',
    'ProcessTracker',
    'thread_safe',
    'ThreadSafeDict',
    'GeoUtils'
]
