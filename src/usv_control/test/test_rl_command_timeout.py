from types import SimpleNamespace
from unittest.mock import Mock, patch

from usv_control.velocity_controller_node import VelocityControllerNode
from usv_control.velocity_path_tracker import VelocityCommand


def controller(fallback=False):
    return SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=fallback),
        _rl_policy_last_action_time=10.0,
        _rl_policy_action_timeout=0.5,
        _rl_policy_last_action=VelocityCommand(linear_x=0.2, linear_y=0.0, angular_z=0.3),
        _rl_policy_last_shaped_action=VelocityCommand(linear_x=0.2, linear_y=0.0, angular_z=0.3),
        _sync_rl_command_shaper=Mock(),
        _shape_rl_policy_command=Mock(return_value=VelocityCommand(linear_x=0.2, linear_y=0.0, angular_z=0.3)),
    )


def test_expired_pure_command_stops_and_resets_shaper():
    node = controller()
    with patch('usv_control.velocity_controller_node.time.time', return_value=10.6):
        result = VelocityControllerNode._apply_rl_policy_to_command(node, VelocityCommand(0.34, 0.0, 0.1))
    assert result.linear_x == result.angular_z == 0.0
    stopped, timestamp = node._sync_rl_command_shaper.call_args.args
    assert stopped.linear_x == stopped.angular_z == 0.0
    assert timestamp == 10.6
    node._shape_rl_policy_command.assert_not_called()


def test_explicit_raw_fallback_preserved():
    node = controller(fallback=True)
    raw = VelocityCommand(0.34, 0.0, 0.1)
    with patch('usv_control.velocity_controller_node.time.time', return_value=10.6):
        result = VelocityControllerNode._apply_rl_policy_to_command(node, raw)
    assert result is raw


def test_fresh_pure_command_still_applied():
    node = controller()
    with patch('usv_control.velocity_controller_node.time.time', return_value=10.1):
        result = VelocityControllerNode._apply_rl_policy_to_command(node, VelocityCommand.stop())
    assert result.linear_x == 0.2
    assert result.angular_z == 0.3
    node._sync_rl_command_shaper.assert_not_called()


def test_guard_zero_survives_real_shaper_and_speed_floor():
    node = controller()
    node.get_parameter = lambda name: SimpleNamespace(value=name == 'rl_policy_enabled')
    node._rl_policy_last_action = VelocityCommand.stop()
    node._sync_rl_command_shaper = lambda command, now: VelocityControllerNode._sync_rl_command_shaper(node, command, now)
    node._shape_rl_policy_command = lambda command, now: VelocityControllerNode._shape_rl_policy_command(node, command, now)
    with patch('usv_control.velocity_controller_node.time.time', return_value=10.1):
        result = VelocityControllerNode._apply_rl_policy_to_command(node, VelocityCommand(0.34, 0.0, 0.1))
    result = VelocityControllerNode._enforce_avoidance_min_forward_speed(node, result)
    assert result.linear_x == result.linear_y == result.angular_z == 0.0
    assert node._rl_policy_last_shaped_action.linear_x == 0.0