import re


_BENIGN_SHUTDOWN_TRACEBACK_PATTERNS = (
    r'Traceback \(most recent call last\):.*?KeyboardInterrupt',
    r'Traceback \(most recent call last\):.*?rclpy\._rclpy_pybind11\.RCLError: failed to shutdown: rcl_shutdown already called on the given context[^\n]*',
    r'Traceback \(most recent call last\):.*?rclpy\._rclpy_pybind11\.RCLError: Failed to publish: publisher\'s context is invalid[^\n]*',
    r"Traceback \(most recent call last\):.*?RuntimeError: Unable to convert call argument '0' to Python object[^\n]*",
)

_BENIGN_SHUTDOWN_LINE_PATTERNS = (
    r'The following exception was never retrieved: cannot use Destroyable because destruction was requested',
    r'terminate called after throwing an instance of \'rclcpp::exceptions::RCLError\'',
    r'\s*what\(\):\s+could not create publisher: rcl node\'s context is invalid[^\n]*',
    r'\[ERROR\] \[[^\n]+process has died[^\n]*',
    r'\[ERROR\] \[[^\n]+open failed: DeviceError:tcp:connect: Interrupted system call[^\n]*',
)


def scrub_benign_shutdown_output(output: str) -> str:
    scrubbed = output
    for pattern in _BENIGN_SHUTDOWN_TRACEBACK_PATTERNS:
        scrubbed = re.sub(pattern, '', scrubbed, flags=re.S)

    for pattern in _BENIGN_SHUTDOWN_LINE_PATTERNS:
        scrubbed = re.sub(pattern, '', scrubbed)

    return scrubbed


def has_unexpected_traceback(output: str) -> bool:
    if 'Traceback' not in output and 'terminate called after throwing an instance' not in output:
        return False

    if 'user interrupted with ctrl-c (SIGINT)' not in output:
        return True

    scrubbed = scrub_benign_shutdown_output(output)
    return 'Traceback' in scrubbed or 'terminate called after throwing an instance' in scrubbed