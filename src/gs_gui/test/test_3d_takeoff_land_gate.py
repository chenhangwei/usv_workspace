#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
#
# This file is part of the USV Workspace project.
#
# Unit tests for 3D takeoff/land actions, motor protection lock, and cluster takeoff altitude gate.

"""Unit tests for 3D vehicle takeoff, land motor protection lock, and cluster altitude gate."""

import json
import types
import pytest

import rclpy
from std_msgs.msg import Bool, Float32, String
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

from usv_control.usv_control_node import UsvControlPx4Node
from gs_gui.cluster_controller import ClusterController, ClusterTaskState
from gs_gui.command_processor import CommandProcessor
from gs_gui.ground_station_node import GroundStationNode


@pytest.fixture(scope='module')
def rclpy_context():
    """Initialize rclpy once for this test module."""
    already = rclpy.ok()
    if not already:
        rclpy.init()
    yield
    if not already and rclpy.ok():
        rclpy.shutdown()


def _make_local_pos(ned_x=0.0, ned_y=0.0, enu_z=0.0, heading=0.0):
    msg = VehicleLocalPosition()
    msg.xy_valid = True
    msg.z_valid = True
    msg.x = float(ned_x)
    msg.y = float(ned_y)
    msg.z = -float(enu_z)  # NED Down = -ENU Z
    msg.heading = float(heading)
    return msg


def _make_vehicle_status(armed=True, nav_state=14):
    st = VehicleStatus()
    st.arming_state = (
        VehicleStatus.ARMING_STATE_ARMED if armed else VehicleStatus.ARMING_STATE_DISARMED
    )
    st.nav_state = int(nav_state)
    st.system_id = 1
    st.component_id = 1
    return st


@pytest.mark.parametrize(
    ('vehicle_type', 'expected_takeoff_z', 'expected_land_z'),
    [
        ('flying_sphere', 2.0, 0.25),
        ('bionic_dolphin', 6.0, 0.30),
        ('bionic_airship', 15.0, 0.50),
    ],
)
def test_3d_vehicles_takeoff_climb_and_land_protection_lock(
    vehicle_type, expected_takeoff_z, expected_land_z
):
    """Verify flying_sphere, bionic_dolphin, bionic_airship takeoff climb & land motor lock."""
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(
        args=[
            '--ros-args',
            '-p',
            'platform_mode:=3d',
            '-p',
            f'vehicle_type:={vehicle_type}',
        ]
    )
    node = UsvControlPx4Node()
    try:
        assert pytest.approx(node.takeoff_altitude, abs=1e-3) == expected_takeoff_z
        assert pytest.approx(node.landing_altitude, abs=1e-3) == expected_land_z

        published_sp = []
        published_cmd = []
        published_fs = []
        node.setpoint_pub.publish = lambda m: published_sp.append(m)
        node.command_pub.publish = lambda m: published_cmd.append(m)
        node.flight_state_pub.publish = lambda m: published_fs.append(json.loads(m.data))

        # 1) Disarmed: takeoff must be rejected
        node.vehicle_status = _make_vehicle_status(armed=False)
        node.current_position = _make_local_pos(enu_z=0.0)
        node.takeoff_command_callback(Float32(data=-1.0))
        assert node._flight_action is None
        assert node._takeoff_ready is False

        # 2) Armed at ground Z=0: takeoff climbs along +Z to preset height
        node.vehicle_status = _make_vehicle_status(armed=True)
        node.takeoff_command_callback(Float32(data=-1.0))
        assert node._flight_action == 'takeoff'

        node.publish_setpoint()
        assert len(published_sp) >= 1
        # PX4 NED Z must be negative of preset takeoff_altitude
        assert pytest.approx(float(published_sp[-1].position[2]), abs=1e-3) == -expected_takeoff_z

        # Simulate reaching preset takeoff altitude
        node.current_position = _make_local_pos(enu_z=expected_takeoff_z)
        node.publish_setpoint()
        assert node._flight_action is None
        assert node._takeoff_ready is True
        assert published_fs[-1]['takeoff_ready'] is True

        # 3) Land command: descends along -Z toward landing_altitude, then locks motors
        node.land_command_callback(Float32(data=-1.0))
        assert node._flight_action == 'land'
        assert node._takeoff_ready is False

        # Mid-descent above landing_altitude -> should NOT disarm yet
        published_cmd.clear()
        node.current_position = _make_local_pos(enu_z=expected_land_z + 0.5)
        node.publish_setpoint()
        assert node._protection_locked is False
        assert not any(c.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM for c in published_cmd)

        # At/below landing_altitude -> immediately force disarm & latch _protection_locked
        node.current_position = _make_local_pos(enu_z=expected_land_z)
        node.publish_setpoint()
        assert node._protection_locked is True
        assert node._flight_action is None
        disarm_cmds = [
            c for c in published_cmd
            if c.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM and c.param1 == 0.0
        ]
        assert len(disarm_cmds) == 1
        assert pytest.approx(float(disarm_cmds[0].param2), abs=1e-1) == 21196.0

        # 4) While _protection_locked is True, takeoff & nav goals are strictly blocked
        #    even if PX4 status were momentarily still reporting armed
        node.takeoff_command_callback(Float32(data=-1.0))
        assert node._flight_action is None
        from common_interfaces.msg import NavigationGoal
        goal = NavigationGoal()
        goal.goal_id = 100001
        node.nav_goal_callback(goal)
        assert node.target_active is False

        # 5) Manual Arming unlocks _protection_locked and allows takeoff again
        node._manual_arming_str_callback(String(data='arm'))
        assert node._protection_locked is False
        node.vehicle_status = _make_vehicle_status(armed=True)
        node.current_position = _make_local_pos(enu_z=0.1)
        node.takeoff_command_callback(Float32(data=-1.0))
        assert node._flight_action == 'takeoff'
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class _DummySignal:
    def __init__(self):
        self.emitted = []

    def emit(self, *args):
        self.emitted.append(args)


def test_cluster_start_gate_requires_3d_takeoff_preset_height():
    """Verify cluster path program blocks 3D vehicles below preset Z and passes once at preset Z."""
    fake_node = types.SimpleNamespace()
    fake_node._usv_vehicle_type = {
        'usv_01': 'flying_sphere',
        'usv_02': 'bionic_dolphin',
        'usv_03': 'bionic_airship',
        'usv_04': 'usv_diff',
    }
    fake_node._usv_platform_mode = {
        'usv_01': '3d',
        'usv_02': '3d',
        'usv_03': '3d',
        'usv_04': '2d',
    }
    fake_node._usv_default_altitude = {
        'usv_01': 2.0,
        'usv_02': 6.0,
        'usv_03': 15.0,
        'usv_04': 0.0,
    }
    fake_node._usv_landing_altitude = {}
    fake_node._usv_reach_z = {'usv_01': 0.5, 'usv_02': 0.8, 'usv_03': 1.5, 'usv_04': 0.5}
    fake_node.usv_states = {
        'usv_01': {'connected': True, 'armed': True, 'protection_locked': False, 'position': {'z': 0.2}},
        'usv_02': {'connected': True, 'armed': True, 'protection_locked': False, 'position': {'z': 6.0}},
        'usv_03': {'connected': True, 'armed': True, 'protection_locked': False, 'position': {'z': 15.0}},
        'usv_04': {'connected': True, 'armed': True, 'protection_locked': False, 'position': {'z': 0.0}},
    }
    fake_node._usv_nav_target_cache = {}
    fake_node.current_targets = []
    fake_node.run_step = 0
    fake_node.usv_target_number = 0
    fake_node.max_step = 0
    fake_node.MIN_ACK_RATE_FOR_PROCEED = 0.8
    fake_node.ros_signal = types.SimpleNamespace(
        node_warning=_DummySignal(),
        node_info=_DummySignal(),
        cluster_progress_update=_DummySignal(),
        nav_status_update=_DummySignal(),
    )
    fake_node.get_logger = lambda: types.SimpleNamespace(
        info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None, debug=lambda *_: None
    )
    fake_node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=1_000_000_000)
    )

    fake_node.command_processor = CommandProcessor(fake_node)
    fake_node.is_3d_vehicle = types.MethodType(GroundStationNode.is_3d_vehicle, fake_node)
    fake_node.get_vehicle_required_takeoff_z = types.MethodType(
        GroundStationNode.get_vehicle_required_takeoff_z, fake_node
    )
    fake_node.validate_3d_cluster_takeoff_ready = types.MethodType(
        GroundStationNode.validate_3d_cluster_takeoff_ready, fake_node
    )

    cc = ClusterController(fake_node)
    targets = [
        {'usv_id': 'usv_01', 'step': 1, 'position': {'x': 2.0, 'y': 0.0, 'z': 2.0}},
        {'usv_id': 'usv_02', 'step': 1, 'position': {'x': 4.0, 'y': 0.0, 'z': 6.0}},
        {'usv_id': 'usv_03', 'step': 1, 'position': {'x': 6.0, 'y': 0.0, 'z': 15.0}},
        {'usv_id': 'usv_04', 'step': 1, 'position': {'x': 8.0, 'y': 0.0, 'z': 0.0}},
    ]

    # 1) usv_01 (flying_sphere) still at Z=0.2m < 2.0m -> cluster start MUST be blocked!
    cc.set_cluster_target_point_callback(targets)
    assert cc._state == ClusterTaskState.IDLE
    assert len(fake_node.ros_signal.node_warning.emitted) == 1
    assert 'usv_01' in fake_node.ros_signal.node_warning.emitted[-1][0]

    # 2) 2D boat alone (usv_04 at Z=0.0m) -> unaffected by 3D takeoff gate, starts immediately
    cc.set_cluster_target_point_callback([targets[3]])
    assert cc._state == ClusterTaskState.RUNNING
    cc.stop_cluster_task('reset')

    # 3) Once usv_01 climbs to Z=2.0m, full heterogeneous 3D+2D cluster starts!
    fake_node.usv_states['usv_01']['position']['z'] = 2.0
    cc.set_cluster_target_point_callback(targets)
    assert cc._state == ClusterTaskState.RUNNING
    cc.stop_cluster_task('reset')

    # 4) If usv_03 (airship) lands and enters protection_locked, cluster start is blocked again
    fake_node.usv_states['usv_03']['protection_locked'] = True
    fake_node.usv_states['usv_03']['armed'] = False
    fake_node.usv_states['usv_03']['position']['z'] = 0.4
    cc.set_cluster_target_point_callback(targets)
    assert cc._state == ClusterTaskState.IDLE
    assert '保护锁止' in fake_node.ros_signal.node_warning.emitted[-1][0]
