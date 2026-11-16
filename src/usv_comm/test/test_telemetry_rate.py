"""Tests for bounded requests and actual source freshness."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from usv_comm.telemetry_rate_node import IntervalRequests, SourceFreshness, needs_rerequest


def test_slow_source_triggers_rerequest_only_after_done_and_cooldown():
    slow = {'pose': {'source_hz': 0.4}, 'velocity': {'source_hz': 12.0}}
    fast = {'pose': {'source_hz': 12.0}, 'velocity': {'source_hz': 12.0}}
    assert needs_rerequest(True, slow, 10.0, 30.0, 20.0)
    assert not needs_rerequest(True, slow, 10.0, 10.0, 20.0)
    assert not needs_rerequest(False, slow, 10.0, 30.0, 20.0)
    assert not needs_rerequest(True, fast, 10.0, 30.0, 20.0)


def client_with_response(success=True, result=0):
    client = Mock()
    client.service_is_ready.return_value = True

    def call(request):
        future = Future()
        future.set_result(SimpleNamespace(success=success, result=result))
        return future

    client.call_async.side_effect = call
    return client


def test_all_message_ids_require_accepted_ack():
    client = client_with_response()
    requests = IntervalRequests([client])
    for tick in range(8):
        requests.tick(float(tick))
    assert requests.done
    assert requests.results == {33: True, 32: True, 30: True, 24: True}
    assert [call.args[0].param1 for call in client.call_async.call_args_list] == [33, 32, 30, 24]
    assert abs(client.call_async.call_args.args[0].param2 - 66666.666667) < 0.001


def test_rejected_ack_has_bounded_retries():
    client = client_with_response(True, 2)
    requests = IntervalRequests([client], retries=2)
    for tick in range(16):
        requests.tick(float(tick))
    assert requests.done and not any(requests.results.values())
    assert client.call_async.call_count == 8


def test_timeout_cancels_pending_and_continues_other_messages():
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.side_effect = lambda request: Future()
    requests = IntervalRequests([client], timeout=1.0, retries=1)
    for tick in range(8):
        requests.tick(float(tick))
    assert requests.done and not any(requests.results.values())
    assert client.remove_pending_request.call_count == 4


def test_unavailable_service_does_not_block_forever():
    client = Mock()
    client.service_is_ready.return_value = False
    requests = IntervalRequests([client], ready_timeout=5.0)
    requests.tick(0.0)
    requests.tick(5.0)
    assert requests.done and not any(requests.results.values())
    client.call_async.assert_not_called()


def test_retry_can_use_alternative_service():
    rejected = client_with_response(False, 0)
    accepted = client_with_response()
    requests = IntervalRequests([rejected, accepted])
    for tick in range(4):
        requests.tick(float(tick))
    assert requests.results[33]
    assert rejected.call_async.call_count == accepted.call_async.call_count == 1


def test_fresh_high_rate_static_values_are_healthy():
    source = SourceFreshness()
    for index in range(151):
        source.observe(100.0 + index / 15.0, (0.0, 0.0), index / 15.0)
    status = source.snapshot(10.0, 110.0, 10.0, 0.25)
    assert status['healthy']
    assert status['source_hz'] == 15.0
    assert status['value_change_hz'] == 0.0


def test_repeated_stamp_cannot_fake_freshness():
    source = SourceFreshness()
    for index in range(101):
        source.observe(100.0, (0.0,), index / 10.0)
    status = source.snapshot(10.0, 110.0, 10.0, 0.25)
    assert status['callback_hz'] == 10.0
    assert status['source_hz'] == 0.0
    assert not status['healthy']


def test_slow_sources_and_stopped_sources_fail():
    source = SourceFreshness()
    for index in range(6):
        source.observe(100.0 + index * 1.9, (float(index),), index * 1.9)
    status = source.snapshot(9.5, 109.5, 10.0, 0.25)
    assert 0.52 < status['source_hz'] < 0.54
    assert not status['healthy']
    assert not source.snapshot(25.0, 125.0, 10.0, 0.25)['healthy']


def test_old_or_future_stamps_fail_even_with_fast_callbacks():
    for offset in (-2.0, 2.0):
        source = SourceFreshness()
        for index in range(151):
            source.observe(100.0 + offset + index / 15.0, (0.0,), index / 15.0)
        assert not source.snapshot(10.0, 110.0, 10.0, 0.25)['healthy']


def test_call_exception_is_bounded():
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.side_effect = RuntimeError('client unavailable')
    requests = IntervalRequests([client], retries=1)
    for tick in range(4):
        requests.tick(float(tick))
    assert requests.done and not any(requests.results.values())


def test_ros_service_and_source_topics_end_to_end():
    import time

    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.srv import CommandLong
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    import json

    from usv_comm.telemetry_rate_node import TelemetryRateNode

    rclpy.init(args=[
        '--ros-args', '-r', '__ns:=/telemetry_regression',
        '-p', 'requested_rate_hz:=30.0',
    ])
    peer = Node('fake_fcu')
    monitor = TelemetryRateNode()
    executor = SingleThreadedExecutor()
    executor.add_node(peer)
    executor.add_node(monitor)
    requested = []
    statuses = []

    def acknowledge(request, response):
        assert request.command == 511
        assert abs(request.param2 - 1e6 / 30.0) < 0.01
        requested.append(request.param1)
        response.success = True
        response.result = 0
        return response

    peer.create_service(CommandLong, 'cmd/command', acknowledge)
    pose_pub = peer.create_publisher(PoseStamped, 'local_position/pose', qos_profile_sensor_data)
    velocity_pub = peer.create_publisher(TwistStamped, 'local_position/velocity_local', qos_profile_sensor_data)
    peer.create_subscription(String, 'telemetry/status', lambda msg: statuses.append(json.loads(msg.data)), 10)

    def publish_sources():
        pose = PoseStamped()
        velocity = TwistStamped()
        pose.header.stamp = peer.get_clock().now().to_msg()
        velocity.header.stamp = pose.header.stamp
        pose.pose.orientation.w = 1.0
        pose_pub.publish(pose)
        velocity_pub.publish(velocity)

    publisher_timer = peer.create_timer(1.0 / 15.0, publish_sources)
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if statuses and statuses[-1]['healthy'] and statuses[-1]['requests_accepted']:
                break
        assert statuses and statuses[-1]['healthy']
        assert statuses[-1]['requests_accepted']
        assert requested == [33.0, 32.0, 30.0, 24.0]
        assert statuses[-1]['sources']['pose']['value_change_hz'] == 0.0
        publisher_timer.cancel()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and statuses[-1]['healthy']:
            executor.spin_once(timeout_sec=0.02)
        assert not statuses[-1]['healthy']
        assert statuses[-1]['requests_accepted']
    finally:
        executor.shutdown()
        monitor.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()