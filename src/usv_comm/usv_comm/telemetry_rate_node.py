"""Request bounded SITL telemetry intervals and monitor source freshness."""

from collections import deque
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandLong
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class IntervalRequests:
    """Advance one asynchronous request at a time, without blocking spin."""

    def __init__(self, clients, rate_hz=15.0, timeout=3.0, retries=3, ready_timeout=60.0):
        self.clients = clients
        self.rate_hz = rate_hz
        self.timeout = timeout
        self.retries = retries
        self.ready_timeout = ready_timeout
        self.message_ids = (33, 32, 30, 24)
        self.results = {}
        self.attempts = {}
        self.pending = None
        self.client = None
        self.started = None
        self.deadline = 0.0
        self.done = False

    def tick(self, now):
        if self.done:
            return None
        if self.started is None:
            self.started = now
        message_id = self.message_ids[len(self.results)]
        if self.pending is not None:
            if self.pending.done():
                try:
                    response = self.pending.result()
                    accepted = response is not None and response.success and response.result == 0
                    reason = f'ACK success={getattr(response, "success", None)} result={getattr(response, "result", None)}'
                except Exception as exc:
                    accepted, reason = False, f'response error: {exc}'
            elif now >= self.deadline:
                self.client.remove_pending_request(self.pending)
                self.pending.cancel()
                accepted, reason = False, 'response timeout'
            else:
                return None
            self.pending = None
            if accepted or self.attempts[message_id] >= self.retries:
                self.results[message_id] = bool(accepted)
                self.done = len(self.results) == len(self.message_ids)
            return f'message={message_id} attempt={self.attempts[message_id]} {reason}'

        ready = [client for client in self.clients if client.service_is_ready()]
        if not ready:
            if now - self.started >= self.ready_timeout:
                self.results.update({mid: False for mid in self.message_ids if mid not in self.results})
                self.done = True
                return 'service discovery timed out'
            return None
        attempt = self.attempts.get(message_id, 0)
        self.client = ready[attempt % len(ready)]
        self.attempts[message_id] = attempt + 1
        request = CommandLong.Request()
        request.command = 511
        request.param1 = float(message_id)
        request.param2 = 1e6 / self.rate_hz
        try:
            self.pending = self.client.call_async(request)
            self.deadline = now + self.timeout
            return f'message={message_id} attempt={attempt + 1} requested {self.rate_hz:g}Hz'
        except Exception as exc:
            if attempt + 1 >= self.retries:
                self.results[message_id] = False
                self.done = len(self.results) == len(self.message_ids)
            return f'message={message_id} request error: {exc}'


class SourceFreshness:
    """Measure unique source stamps separately from callbacks and value changes."""

    def __init__(self, window=10.0):
        self.window = window
        self.receipts = deque(maxlen=2000)
        self.updates = deque(maxlen=2000)
        self.changes = deque(maxlen=2000)
        self.stamp = None
        self.value = None
        self.last_update = None

    def observe(self, stamp, value, now):
        self.receipts.append(now)
        if stamp > 0 and (self.stamp is None or stamp > self.stamp):
            self.updates.append(now)
            self.last_update = now
            self.stamp = stamp
        if self.value is None or value != self.value:
            self.changes.append(now)
            self.value = value

    def snapshot(self, now, ros_now, min_hz, max_age):
        for events in (self.receipts, self.updates, self.changes):
            while events and events[0] < now - self.window:
                events.popleft()

        def rate(events):
            if len(events) < 2:
                return 0.0
            return (len(events) - 1) / max(now - events[0], 1e-6)

        receipt_age = None if not self.receipts else now - self.receipts[-1]
        update_age = None if self.last_update is None else now - self.last_update
        stamp_age = None if self.stamp is None else ros_now - self.stamp
        source_hz = rate(self.updates)
        warmed = len(self.updates) >= 2 and now - self.updates[0] >= 2.0
        healthy = (warmed and source_hz >= min_hz and receipt_age is not None
                   and receipt_age <= max_age and update_age <= max_age
                   and stamp_age is not None and -0.05 <= stamp_age <= max_age)
        return {
            'callback_hz': rate(self.receipts), 'source_hz': source_hz,
            'value_change_hz': rate(self.changes), 'receipt_age_s': receipt_age,
            'source_update_age_s': update_age, 'source_stamp_age_s': stamp_age,
            'healthy': bool(healthy),
        }


class TelemetryRateNode(Node):
    """Keep telemetry request acknowledgments and measured health distinct."""

    def __init__(self):
        super().__init__('telemetry_rate_node')
        defaults = {
            'requested_rate_hz': 15.0, 'request_timeout_s': 3.0,
            'request_retries': 3, 'service_timeout_s': 60.0,
            'minimum_source_hz': 10.0, 'maximum_source_age_s': 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        if any(not math.isfinite(float(value)) or float(value) <= 0 for value in values.values()):
            raise ValueError('Telemetry rates, timeouts and retries must be positive and finite')
        clients = [self.create_client(CommandLong, name) for name in ('cmd/command', 'mavros/cmd/command')]
        self.requests = IntervalRequests(
            clients, values['requested_rate_hz'], values['request_timeout_s'],
            values['request_retries'], values['service_timeout_s'],
        )
        self.minimum_hz = values['minimum_source_hz']
        self.maximum_age = values['maximum_source_age_s']
        self.sources = {name: SourceFreshness() for name in ('pose', 'velocity')}
        self.create_subscription(PoseStamped, 'local_position/pose', self._pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'local_position/velocity_local', self._velocity, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, 'telemetry/status', 10)
        self.create_timer(0.1, self._request_tick)
        self.create_timer(1.0, self._report)
        self._last_health = None
        self._last_log = -math.inf

    def _observe(self, name, msg, value):
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.sources[name].observe(stamp, value, time.monotonic())

    def _pose(self, msg):
        orientation = msg.pose.orientation
        self._observe('pose', msg, (orientation.x, orientation.y, orientation.z, orientation.w))

    def _velocity(self, msg):
        linear = msg.twist.linear
        self._observe('velocity', msg, (linear.x, linear.y, linear.z))

    def _request_tick(self):
        event = self.requests.tick(time.monotonic())
        if event:
            self.get_logger().info(f'[rate-request] {event}')

    def _report(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        sources = {name: source.snapshot(now, ros_now, self.minimum_hz, self.maximum_age)
                   for name, source in self.sources.items()}
        healthy = all(source['healthy'] for source in sources.values())
        status = {
            'timestamp': ros_now, 'namespace': self.get_namespace(),
            'requests_done': self.requests.done,
            'requests_accepted': self.requests.done and all(self.requests.results.values()),
            'message_results': self.requests.results,
            'minimum_source_hz': self.minimum_hz, 'maximum_source_age_s': self.maximum_age,
            'healthy': healthy, 'sources': sources,
        }
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(message)
        if healthy != self._last_health or now - self._last_log >= 10.0:
            if healthy:
                self.get_logger().info(f'[telemetry] {message.data}')
            else:
                self.get_logger().warning(f'[telemetry] {message.data}')
            self._last_health, self._last_log = healthy, now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TelemetryRateNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()