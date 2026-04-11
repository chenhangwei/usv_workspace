import argparse
import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .multi_agent_scenarios import FleetScenario


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass
class _SimAgentState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    target_v: float = 0.0
    target_omega: float = 0.0
    last_cmd_time: float = 0.0


@dataclass
class _DomainRandomizationConfig:
    """Domain randomization noise parameters for sim-to-real robustness.

    Process noise affects the actual dynamics (simulating wind/current).
    Observation noise affects published sensor readings (simulating GPS/compass noise).
    """
    enabled: bool = False
    # Observation noise (applied to published values only)
    position_noise_std: float = 0.10       # GPS jitter (m)
    heading_noise_std: float = 0.02        # Compass jitter (rad, ~1.1°)
    velocity_noise_ratio: float = 0.03     # Velocity measurement noise (fraction)
    # Process noise: water current (Ornstein-Uhlenbeck drift)
    current_speed_max: float = 0.04        # Maximum current magnitude (m/s)
    current_theta: float = 0.15            # OU mean-reversion rate
    current_sigma: float = 0.02            # OU volatility
    # Process noise: velocity execution noise
    velocity_exec_noise: float = 0.05      # Fraction of commanded speed


class MultiUsvSimNode(Node):
    def __init__(self, agent_namespaces: tuple[str, ...], update_rate: float = 20.0,
                 tau_linear: float = 0.45, tau_angular: float = 0.25):
        super().__init__('multi_usv_sim')
        self._agent_namespaces = tuple(agent_namespaces)
        self._dt = 1.0 / max(update_rate, 1.0)
        self._tau_linear = tau_linear
        self._tau_angular = tau_angular
        self._command_timeout = 0.6
        self._dr = _DomainRandomizationConfig()
        self._rng = np.random.default_rng()
        self._current_x: float = 0.0
        self._current_y: float = 0.0

        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        self._agent_states = {namespace: _SimAgentState() for namespace in self._agent_namespaces}
        self._pose_publishers = {}
        self._mavros_pose_publishers = {}
        self._velocity_publishers = {}
        self._state_publishers = {}

        for namespace in self._agent_namespaces:
            self._pose_publishers[namespace] = self.create_publisher(
                PoseStamped,
                f'/{namespace}/local_position/pose_from_gps',
                qos_best_effort,
            )
            self._mavros_pose_publishers[namespace] = self.create_publisher(
                PoseStamped,
                f'/{namespace}/local_position/pose',
                qos_best_effort,
            )
            self._velocity_publishers[namespace] = self.create_publisher(
                TwistStamped,
                f'/{namespace}/local_position/velocity_local',
                qos_best_effort,
            )
            self._state_publishers[namespace] = self.create_publisher(
                State,
                f'/{namespace}/state',
                qos_reliable,
            )
            self.create_subscription(
                PositionTarget,
                f'/{namespace}/setpoint_raw/local',
                self._make_cmd_callback(namespace),
                qos_best_effort,
            )

        self._update_timer = self.create_timer(self._dt, self._on_timer)

    def _make_cmd_callback(self, namespace: str):
        def _callback(msg: PositionTarget):
            state = self._agent_states[namespace]
            state.target_v = float(msg.velocity.x)
            state.target_omega = float(msg.yaw_rate)
            state.last_cmd_time = time.monotonic()
        return _callback

    def reset_agents(self, scenario: FleetScenario):
        now = time.monotonic()
        for namespace, state in self._agent_states.items():
            spawn = scenario.agent_spawns[namespace]
            state.x = float(spawn.x)
            state.y = float(spawn.y)
            state.yaw = float(spawn.yaw)
            state.v = 0.0
            state.omega = 0.0
            state.target_v = 0.0
            state.target_omega = 0.0
            state.last_cmd_time = now
            self._publish_agent_state(namespace, state)

    def get_agent_snapshot(self, namespace: str) -> tuple[float, float, float, float]:
        state = self._agent_states[namespace]
        return state.x, state.y, state.yaw, state.v

    def randomize_tau(
        self,
        tau_linear_low: float,
        tau_linear_high: float,
        tau_angular_low: float,
        tau_angular_high: float,
    ) -> None:
        """Randomize dynamics time constants (call once per episode reset)."""
        self._tau_linear = float(self._rng.uniform(tau_linear_low, tau_linear_high))
        self._tau_angular = float(self._rng.uniform(tau_angular_low, tau_angular_high))

    def set_domain_randomization(
        self,
        enabled: bool = False,
        position_noise_std: float = 0.10,
        heading_noise_std: float = 0.02,
        velocity_noise_ratio: float = 0.03,
        current_speed_max: float = 0.04,
        current_theta: float = 0.15,
        current_sigma: float = 0.02,
        velocity_exec_noise: float = 0.05,
    ) -> None:
        self._dr = _DomainRandomizationConfig(
            enabled=enabled,
            position_noise_std=position_noise_std,
            heading_noise_std=heading_noise_std,
            velocity_noise_ratio=velocity_noise_ratio,
            current_speed_max=current_speed_max,
            current_theta=current_theta,
            current_sigma=current_sigma,
            velocity_exec_noise=velocity_exec_noise,
        )
        self._current_x = 0.0
        self._current_y = 0.0

    def _step_current_drift(self) -> tuple[float, float]:
        """Evolve water current using an Ornstein-Uhlenbeck process."""
        dt = self._dt
        theta = self._dr.current_theta
        sigma = self._dr.current_sigma
        cap = self._dr.current_speed_max
        sqrt_dt = math.sqrt(dt)
        self._current_x += theta * (-self._current_x) * dt + sigma * float(self._rng.standard_normal()) * sqrt_dt
        self._current_y += theta * (-self._current_y) * dt + sigma * float(self._rng.standard_normal()) * sqrt_dt
        self._current_x = max(-cap, min(cap, self._current_x))
        self._current_y = max(-cap, min(cap, self._current_y))
        return self._current_x, self._current_y

    def _on_timer(self):
        now = time.monotonic()
        dr = self._dr
        if dr.enabled:
            current_x, current_y = self._step_current_drift()
        else:
            current_x, current_y = 0.0, 0.0

        for namespace, state in self._agent_states.items():
            if (now - state.last_cmd_time) > self._command_timeout:
                state.target_v = 0.0
                state.target_omega = 0.0

            linear_alpha = min(1.0, self._dt / max(self._tau_linear, 1e-3))
            angular_alpha = min(1.0, self._dt / max(self._tau_angular, 1e-3))
            state.v += linear_alpha * (state.target_v - state.v)
            state.omega += angular_alpha * (state.target_omega - state.omega)

            # Velocity execution noise (actuator imperfection)
            effective_v = state.v
            effective_omega = state.omega
            if dr.enabled and dr.velocity_exec_noise > 0.0:
                effective_v *= 1.0 + dr.velocity_exec_noise * float(self._rng.standard_normal())
                effective_omega *= 1.0 + dr.velocity_exec_noise * float(self._rng.standard_normal())

            state.yaw += effective_omega * self._dt
            while state.yaw > math.pi:
                state.yaw -= 2.0 * math.pi
            while state.yaw < -math.pi:
                state.yaw += 2.0 * math.pi

            vx_world = effective_v * math.cos(state.yaw)
            vy_world = effective_v * math.sin(state.yaw)
            state.x += (vx_world + current_x) * self._dt
            state.y += (vy_world + current_y) * self._dt
            self._publish_agent_state(namespace, state)

    def _publish_agent_state(self, namespace: str, state: _SimAgentState):
        stamp = self.get_clock().now().to_msg()
        dr = self._dr

        # Apply observation noise (sensor imperfection) to published values only
        pub_x = state.x
        pub_y = state.y
        pub_yaw = state.yaw
        pub_v = state.v
        pub_omega = state.omega
        if dr.enabled:
            if dr.position_noise_std > 0.0:
                pub_x += dr.position_noise_std * float(self._rng.standard_normal())
                pub_y += dr.position_noise_std * float(self._rng.standard_normal())
            if dr.heading_noise_std > 0.0:
                pub_yaw += dr.heading_noise_std * float(self._rng.standard_normal())
            if dr.velocity_noise_ratio > 0.0:
                pub_v *= 1.0 + dr.velocity_noise_ratio * float(self._rng.standard_normal())
                pub_omega *= 1.0 + dr.velocity_noise_ratio * float(self._rng.standard_normal())

        qx, qy, qz, qw = _yaw_to_quaternion(pub_yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = pub_x
        pose_msg.pose.position.y = pub_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self._pose_publishers[namespace].publish(pose_msg)
        self._mavros_pose_publishers[namespace].publish(pose_msg)

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = stamp
        velocity_msg.header.frame_id = 'map'
        velocity_msg.twist.linear.x = pub_v * math.cos(pub_yaw)
        velocity_msg.twist.linear.y = pub_v * math.sin(pub_yaw)
        velocity_msg.twist.angular.z = pub_omega
        self._velocity_publishers[namespace].publish(velocity_msg)

        state_msg = State()
        state_msg.connected = True
        state_msg.armed = True
        state_msg.guided = True
        state_msg.manual_input = False
        state_msg.mode = 'GUIDED'
        self._state_publishers[namespace].publish(state_msg)

    def prepare_for_shutdown(self):
        if self._update_timer is not None:
            self._update_timer.cancel()
            self.destroy_timer(self._update_timer)
            self._update_timer = None
        for state in self._agent_states.values():
            state.target_v = 0.0
            state.target_omega = 0.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Run the lightweight multi-USV simulator used for MAPPO migration.')
    parser.add_argument('--namespaces', nargs='+', default=['usv_01', 'usv_02'], help='Simulated USV namespaces.')
    parser.add_argument('--update-rate', type=float, default=20.0, help='Simulation update rate in Hz.')
    return parser.parse_args(argv)


def main(argv=None):
    cli_args = rclpy.utilities.remove_ros_args(args=argv)
    args = parse_args(cli_args[1:])
    rclpy.init(args=argv)
    node = None
    try:
        node = MultiUsvSimNode(tuple(args.namespaces), update_rate=args.update_rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.prepare_for_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()