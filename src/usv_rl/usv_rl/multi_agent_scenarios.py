import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .types import NeighborState


@dataclass
class AgentSpawnConfig:
    x: float
    y: float
    yaw: float


@dataclass
class AgentGoalConfig:
    x: float
    y: float


@dataclass
class BackgroundTrack:
    track_id: str
    start_x: float
    start_y: float
    vx: float
    vy: float
    yaw: float

    def state_at(self, elapsed: float) -> NeighborState:
        return NeighborState(
            usv_id=self.track_id,
            x=self.start_x + self.vx * elapsed,
            y=self.start_y + self.vy * elapsed,
            yaw=self.yaw,
            vx=self.vx,
            vy=self.vy,
        )


@dataclass
class FleetScenario:
    name: str
    duration: float
    agent_spawns: Dict[str, AgentSpawnConfig]
    agent_goals: Dict[str, AgentGoalConfig]
    active_agent_ids: tuple[str, ...] = field(default_factory=tuple)
    background_tracks: List[BackgroundTrack] = field(default_factory=list)

    def __post_init__(self):
        if not self.active_agent_ids:
            self.active_agent_ids = tuple(self.agent_spawns)

    @property
    def inactive_agent_ids(self) -> tuple[str, ...]:
        return tuple(agent_id for agent_id in self.agent_spawns if agent_id not in self.active_agent_ids)

    def states_at(self, elapsed: float) -> List[NeighborState]:
        return [track.state_at(elapsed) for track in self.background_tracks]

    def apply_randomization(
        self,
        rng: np.random.Generator,
        *,
        spawn_position_std: float = 0.5,
        spawn_heading_std: float = 0.15,
        goal_position_std: float = 0.3,
    ) -> 'FleetScenario':
        """Return a copy with Gaussian jitter on spawn/goal positions and headings."""
        new_spawns = {}
        for agent_id, spawn in self.agent_spawns.items():
            new_spawns[agent_id] = AgentSpawnConfig(
                x=spawn.x + spawn_position_std * float(rng.standard_normal()),
                y=spawn.y + spawn_position_std * float(rng.standard_normal()),
                yaw=spawn.yaw + spawn_heading_std * float(rng.standard_normal()),
            )
        new_goals = {}
        for agent_id, goal in self.agent_goals.items():
            new_goals[agent_id] = AgentGoalConfig(
                x=goal.x + goal_position_std * float(rng.standard_normal()),
                y=goal.y + goal_position_std * float(rng.standard_normal()),
            )
        return FleetScenario(
            name=self.name,
            duration=self.duration,
            agent_spawns=new_spawns,
            agent_goals=new_goals,
            active_agent_ids=self.active_agent_ids,
            background_tracks=list(self.background_tracks),
        )


class MultiAgentScenarioFactory:
    _REQUIRED_AGENT_COUNTS = {
        'solo_navigation': 1,
        'single_usv_overtaking': 1,
        'waypoint_turn': 1,
        'two_usv_head_on': 2,
        'two_usv_crossing': 2,
        'two_usv_overtaking': 2,
        'three_usv_crossing': 3,
        'three_usv_clear_route': 3,
        'three_usv_overtaking': 3,
        'two_usv_random_encounter': 2,
        'three_usv_random_encounter': 3,
        'pentagram_convergence': 3,
        'five_usv_dense_head_on': 5,
        'five_usv_dense_crossing': 5,
        'five_usv_dense_overtaking': 5,
        'five_usv_dense_overtaking_tight': 5,
        'five_usv_dense_overtaking_merge': 5,
    }

    @staticmethod
    def _add_spectator_agents(
        agent_ids: tuple[str, ...],
        agent_spawns: Dict[str, AgentSpawnConfig],
        agent_goals: Dict[str, AgentGoalConfig],
    ) -> None:
        for offset, agent_id in enumerate(agent_ids):
            if agent_id in agent_spawns:
                continue
            spectator_y = 8.0 + (offset * 2.5)
            agent_spawns[agent_id] = AgentSpawnConfig(x=-2.0, y=spectator_y, yaw=0.0)
            agent_goals[agent_id] = AgentGoalConfig(x=0.0, y=spectator_y)

    @staticmethod
    def available() -> tuple[str, ...]:
        return (
            'solo_navigation',
            'single_usv_overtaking',
            'waypoint_turn',
            'three_usv_clear_route',
            'two_usv_head_on',
            'two_usv_crossing',
            'two_usv_overtaking',
            'three_usv_crossing',
            'three_usv_overtaking',
            'two_usv_random_encounter',
            'three_usv_random_encounter',
            'pentagram_convergence',
        )

    @staticmethod
    def cluster_available() -> tuple[str, ...]:
        return ('five_usv_dense_head_on', 'five_usv_dense_crossing', 'five_usv_dense_overtaking')

    @staticmethod
    def all_available() -> tuple[str, ...]:
        return MultiAgentScenarioFactory.available() + MultiAgentScenarioFactory.cluster_available()

    @staticmethod
    def required_agent_count(kind: str) -> int:
        try:
            return MultiAgentScenarioFactory._REQUIRED_AGENT_COUNTS[kind]
        except KeyError as exc:
            raise ValueError(f'Unsupported multi-agent scenario kind: {kind}') from exc

    @staticmethod
    def compatible_scenarios(agent_count: int, scenarios: tuple[str, ...]) -> tuple[str, ...]:
        return tuple(
            scenario
            for scenario in scenarios
            if MultiAgentScenarioFactory.required_agent_count(scenario) <= agent_count
        )

    @staticmethod
    def scenario_set(kind: str, agent_count: int) -> tuple[str, ...]:
        if kind == 'smoke':
            scenarios = MultiAgentScenarioFactory.available()
        elif kind == 'dense':
            scenarios = MultiAgentScenarioFactory.cluster_available()
        elif kind == 'all':
            scenarios = MultiAgentScenarioFactory.all_available()
        elif kind == 'auto':
            scenarios = (
                MultiAgentScenarioFactory.all_available()
                if agent_count >= 5
                else MultiAgentScenarioFactory.available()
            )
        else:
            raise ValueError(f'Unsupported multi-agent scenario set: {kind}')

        compatible = MultiAgentScenarioFactory.compatible_scenarios(agent_count, scenarios)
        if not compatible:
            raise ValueError(
                f'No scenarios from set {kind!r} are compatible with agent_count={agent_count}.'
            )
        return compatible

    @staticmethod
    def create(
        kind: str,
        agent_ids: tuple[str, ...],
        *,
        goal_distance: float,
        neighbor_speed: float,
        rng: Optional[np.random.Generator] = None,
        spawn_position_std: float = 0.0,
        spawn_heading_std: float = 0.0,
        goal_position_std: float = 0.0,
    ) -> FleetScenario:
        scenario = MultiAgentScenarioFactory._create_base(
            kind, agent_ids, goal_distance=goal_distance, neighbor_speed=neighbor_speed, rng=rng,
        )
        return MultiAgentScenarioFactory._maybe_randomize(
            scenario, rng, spawn_position_std, spawn_heading_std, goal_position_std,
        )

    @staticmethod
    def _create_base(
        kind: str,
        agent_ids: tuple[str, ...],
        *,
        goal_distance: float,
        neighbor_speed: float,
        rng: Optional[np.random.Generator] = None,
    ) -> FleetScenario:
        if kind == 'solo_navigation':
            first = agent_ids[0]
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=40.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first,),
            )

        if kind == 'waypoint_turn':
            # Single agent moving east (yaw=0), as if it has just passed a
            # waypoint.  The goal is placed off to one side at a large bearing so
            # the policy must execute a big heading change -- exactly the
            # transient that happens right after a waypoint switch.  Paired with
            # arc-turn rewards (pure_spin_penalty / pure_cruise_reward /
            # min_forward_speed_floor>0 / straight_line_omega_penalty) this teaches
            # a smooth >1m-radius arc instead of a pivot-in-place.
            first = agent_ids[0]
            local_rng = rng if rng is not None else np.random.default_rng()
            # Turn magnitude ~45deg..135deg, random port/starboard.
            turn = float(local_rng.uniform(0.8, 2.4))
            if local_rng.uniform() < 0.5:
                turn = -turn
            gx = float(goal_distance * math.cos(turn))
            gy = float(goal_distance * math.sin(turn))
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
            }
            agent_goals = {
                first: AgentGoalConfig(x=gx, y=gy),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=40.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first,),
            )

        if kind == 'single_usv_overtaking':
            first = agent_ids[0]
            lead_speed = max(0.08, min(0.18, neighbor_speed * 0.42))
            agent_spawns = {
                # Active agent is the rear/overtaking vessel.
                first: AgentSpawnConfig(x=0.0, y=-0.6, yaw=0.0),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance + 1.5, y=-0.6),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=45.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first,),
                background_tracks=[
                    BackgroundTrack(
                        track_id='scripted_slow_lead',
                        start_x=3.0,
                        start_y=0.0,
                        vx=lead_speed,
                        vy=0.0,
                        yaw=0.0,
                    ),
                ],
            )

        if kind == 'two_usv_head_on':
            if len(agent_ids) < 2:
                raise ValueError('two_usv_head_on requires at least 2 agents.')
            first, second = agent_ids[:2]
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                second: AgentSpawnConfig(x=goal_distance, y=0.0, yaw=math.pi),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
                second: AgentGoalConfig(x=0.0, y=0.0),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=45.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first, second),
            )

        if kind == 'two_usv_crossing':
            if len(agent_ids) < 2:
                raise ValueError('two_usv_crossing requires at least 2 agents.')
            first, second = agent_ids[:2]
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                # Starboard crossing for the first agent: second approaches
                # from negative Y and crosses the first agent's route near x=5.
                second: AgentSpawnConfig(x=5.0, y=-5.0, yaw=math.pi / 2.0),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
                second: AgentGoalConfig(x=5.0, y=7.0),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=45.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first, second),
            )

        if kind == 'two_usv_overtaking':
            if len(agent_ids) < 2:
                raise ValueError('two_usv_overtaking requires at least 2 agents.')
            first, second = agent_ids[:2]
            agent_spawns = {
                # First agent is ahead in-lane; second starts astern and
                # slightly to port, forcing a controlled pass rather than
                # rear-end following.
                first: AgentSpawnConfig(x=3.0, y=0.0, yaw=0.0),
                second: AgentSpawnConfig(x=0.0, y=-0.6, yaw=0.0),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance + 3.0, y=0.0),
                second: AgentGoalConfig(x=goal_distance + 1.5, y=-0.6),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=45.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first, second),
            )

        if kind == 'three_usv_clear_route':
            if len(agent_ids) < 3:
                raise ValueError('three_usv_clear_route requires at least 3 agents.')
            first, second, third = agent_ids[:3]
            agent_spawns = {
                # Three non-conflicting routes with side/rear neighbors present.
                # All agents travel generally east; lateral offsets keep the
                # goal-direction forward cones clear while neighbors remain
                # visible in observation.
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                second: AgentSpawnConfig(x=-3.0, y=-3.5, yaw=0.0),
                third: AgentSpawnConfig(x=3.0, y=3.5, yaw=0.0),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
                second: AgentGoalConfig(x=goal_distance - 3.0, y=-3.5),
                third: AgentGoalConfig(x=goal_distance + 3.0, y=3.5),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=45.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first, second, third),
            )

        if kind == 'three_usv_crossing':
            if len(agent_ids) < 3:
                raise ValueError('three_usv_crossing requires at least 3 agents.')
            first, second, third = agent_ids[:3]
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                second: AgentSpawnConfig(x=6.0, y=-6.0, yaw=math.pi / 2.0),
                third: AgentSpawnConfig(x=12.0, y=0.0, yaw=math.pi),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
                second: AgentGoalConfig(x=6.0, y=6.0),
                third: AgentGoalConfig(x=0.0, y=0.0),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=50.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first, second, third),
            )

        if kind == 'three_usv_overtaking':
            if len(agent_ids) < 3:
                raise ValueError('three_usv_overtaking requires at least 3 agents.')
            first, second, third = agent_ids[:3]
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                second: AgentSpawnConfig(x=6.0, y=-3.0, yaw=0.0),
                third: AgentSpawnConfig(x=3.0, y=3.0, yaw=-0.15),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
                second: AgentGoalConfig(x=goal_distance, y=-3.0),
                third: AgentGoalConfig(x=goal_distance, y=2.5),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=50.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first, second, third),
                background_tracks=[
                    BackgroundTrack(
                        track_id='background_slow_ahead',
                        start_x=4.0,
                        start_y=0.0,
                        vx=neighbor_speed * 0.35,
                        vy=0.0,
                        yaw=0.0,
                    ),
                ],
            )

        if kind == 'two_usv_random_encounter':
            if len(agent_ids) < 2:
                raise ValueError('two_usv_random_encounter requires at least 2 agents.')
            first, second = agent_ids[:2]
            # Agent 1 always traverses along X-axis.
            # Agent 2 approaches from a uniformly random angle theta
            # around an encounter center shifted toward Agent 1 (0.35*goal)
            # so the orbit wraps behind Agent 1, giving true 360° bearing.
            _rng = rng if rng is not None else np.random.default_rng()
            theta = float(_rng.uniform(0.0, 2.0 * math.pi))
            encounter_x = goal_distance * 0.35
            radius = goal_distance * 0.45
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                second: AgentSpawnConfig(
                    x=encounter_x + radius * math.cos(theta),
                    y=radius * math.sin(theta),
                    yaw=theta + math.pi,  # heading toward encounter center
                ),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
                second: AgentGoalConfig(
                    x=encounter_x - radius * math.cos(theta),
                    y=-radius * math.sin(theta),
                ),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=50.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first, second),
            )

        if kind == 'three_usv_random_encounter':
            if len(agent_ids) < 3:
                raise ValueError('three_usv_random_encounter requires at least 3 agents.')
            first, second, third = agent_ids[:3]
            _rng = rng if rng is not None else np.random.default_rng()
            encounter_x = goal_distance * 0.35
            radius = goal_distance * 0.45
            theta1 = float(_rng.uniform(0.0, 2.0 * math.pi))
            # Ensure agent 3 is at least 60 degrees away from agent 2
            theta2 = theta1 + float(_rng.uniform(math.pi / 3.0, 5.0 * math.pi / 3.0))
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                second: AgentSpawnConfig(
                    x=encounter_x + radius * math.cos(theta1),
                    y=radius * math.sin(theta1),
                    yaw=theta1 + math.pi,
                ),
                third: AgentSpawnConfig(
                    x=encounter_x + radius * math.cos(theta2),
                    y=radius * math.sin(theta2),
                    yaw=theta2 + math.pi,
                ),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
                second: AgentGoalConfig(
                    x=encounter_x - radius * math.cos(theta1),
                    y=-radius * math.sin(theta1),
                ),
                third: AgentGoalConfig(
                    x=encounter_x - radius * math.cos(theta2),
                    y=-radius * math.sin(theta2),
                ),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=55.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=(first, second, third),
            )

        if kind == 'pentagram_convergence':
            # Deployment-replica scenario (5-USV pentagram mission, 2026-06-11
            # SITL analysis): agents sit on the vertices of a regular polygon
            # and each transits to a vertex two steps away, so every route
            # crosses the arena centre simultaneously — the exact congestion
            # pattern that collapsed fresh520 (78% of time all 5 USVs inside
            # r<8m).  Randomized: global rotation, star direction (±2 skip),
            # and the number of active agents (density 3..n); inactive agents
            # hold their vertex as static obstacles, mirroring vessels that
            # already finished their goal in deployment.
            if len(agent_ids) < 3:
                raise ValueError('pentagram_convergence requires at least 3 agents.')
            _rng = rng if rng is not None else np.random.default_rng()
            n = min(5, len(agent_ids))
            vertex_agents = list(agent_ids[:n])
            radius = goal_distance * 0.5
            rotation = float(_rng.uniform(0.0, 2.0 * math.pi))
            skip = 2 if float(_rng.uniform()) < 0.5 else -2
            vertices = [
                (
                    radius * math.cos(rotation + 2.0 * math.pi * k / n),
                    radius * math.sin(rotation + 2.0 * math.pi * k / n),
                )
                for k in range(n)
            ]
            agent_spawns = {}
            agent_goals = {}
            for k, agent_id in enumerate(vertex_agents):
                sx, sy = vertices[k]
                gx, gy = vertices[(k + skip) % n]
                agent_spawns[agent_id] = AgentSpawnConfig(
                    x=sx, y=sy, yaw=math.atan2(gy - sy, gx - sx),
                )
                agent_goals[agent_id] = AgentGoalConfig(x=gx, y=gy)
            # Density randomization: 3..n active agents per episode so the
            # policy stays robust to varying neighbor counts.
            active_count = int(_rng.integers(3, n + 1))
            order = list(_rng.permutation(n))
            active_ids = tuple(vertex_agents[i] for i in sorted(order[:active_count]))
            # Push inactive vertex agents radially outward so they never sit
            # exactly on an active agent's goal vertex (which would make the
            # goal unreachable within collision distance).  They stay visible
            # as static near-route obstacles.
            for k, agent_id in enumerate(vertex_agents):
                if agent_id in active_ids:
                    continue
                vx, vy = vertices[k]
                norm = max(math.hypot(vx, vy), 1e-6)
                push = 2.0
                agent_spawns[agent_id] = AgentSpawnConfig(
                    x=vx * (1.0 + push / norm),
                    y=vy * (1.0 + push / norm),
                    yaw=agent_spawns[agent_id].yaw,
                )
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=60.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                active_agent_ids=active_ids,
            )

        if kind == 'five_usv_dense_head_on':
            if len(agent_ids) < 5:
                raise ValueError('five_usv_dense_head_on requires at least 5 agents.')
            first, second, third, fourth, fifth = agent_ids[:5]
            return FleetScenario(
                name=kind,
                duration=55.0,
                agent_spawns={
                    first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                    second: AgentSpawnConfig(x=12.0, y=0.0, yaw=math.pi),
                    third: AgentSpawnConfig(x=4.0, y=-5.0, yaw=math.pi / 2.0),
                    fourth: AgentSpawnConfig(x=6.0, y=5.0, yaw=-math.pi / 2.0),
                    fifth: AgentSpawnConfig(x=2.5, y=-1.0, yaw=0.0),
                },
                agent_goals={
                    first: AgentGoalConfig(x=12.0, y=0.0),
                    second: AgentGoalConfig(x=0.0, y=0.0),
                    third: AgentGoalConfig(x=4.0, y=7.0),
                    fourth: AgentGoalConfig(x=6.0, y=-7.0),
                    fifth: AgentGoalConfig(x=14.0, y=-1.0),
                },
            )

        if kind == 'five_usv_dense_crossing':
            if len(agent_ids) < 5:
                raise ValueError('five_usv_dense_crossing requires at least 5 agents.')
            first, second, third, fourth, fifth = agent_ids[:5]
            return FleetScenario(
                name=kind,
                duration=55.0,
                agent_spawns={
                    first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                    second: AgentSpawnConfig(x=5.0, y=-6.0, yaw=math.pi / 2.0),
                    third: AgentSpawnConfig(x=9.0, y=-4.0, yaw=math.pi / 2.0),
                    fourth: AgentSpawnConfig(x=12.0, y=1.2, yaw=math.pi),
                    fifth: AgentSpawnConfig(x=7.0, y=5.5, yaw=-math.pi / 2.0),
                },
                agent_goals={
                    first: AgentGoalConfig(x=12.0, y=0.0),
                    second: AgentGoalConfig(x=5.0, y=7.0),
                    third: AgentGoalConfig(x=9.0, y=7.0),
                    fourth: AgentGoalConfig(x=0.0, y=1.2),
                    fifth: AgentGoalConfig(x=7.0, y=-7.0),
                },
            )

        if kind == 'five_usv_dense_overtaking':
            if len(agent_ids) < 5:
                raise ValueError('five_usv_dense_overtaking requires at least 5 agents.')
            first, second, third, fourth, fifth = agent_ids[:5]
            return FleetScenario(
                name=kind,
                duration=55.0,
                agent_spawns={
                    first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                    second: AgentSpawnConfig(x=2.4, y=0.0, yaw=0.0),
                    third: AgentSpawnConfig(x=3.2, y=-1.4, yaw=0.0),
                    fourth: AgentSpawnConfig(x=6.0, y=-5.0, yaw=math.pi / 2.0),
                    fifth: AgentSpawnConfig(x=12.0, y=1.0, yaw=math.pi),
                },
                agent_goals={
                    first: AgentGoalConfig(x=12.0, y=0.0),
                    second: AgentGoalConfig(x=13.5, y=0.0),
                    third: AgentGoalConfig(x=13.0, y=-1.4),
                    fourth: AgentGoalConfig(x=6.0, y=6.0),
                    fifth: AgentGoalConfig(x=0.0, y=1.0),
                },
            )

        if kind == 'five_usv_dense_overtaking_tight':
            if len(agent_ids) < 5:
                raise ValueError('five_usv_dense_overtaking_tight requires at least 5 agents.')
            first, second, third, fourth, fifth = agent_ids[:5]
            return FleetScenario(
                name=kind,
                duration=55.0,
                agent_spawns={
                    first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                    second: AgentSpawnConfig(x=1.9, y=0.05, yaw=0.0),
                    third: AgentSpawnConfig(x=2.8, y=-0.85, yaw=0.0),
                    fourth: AgentSpawnConfig(x=5.4, y=-4.2, yaw=math.pi / 2.0),
                    fifth: AgentSpawnConfig(x=10.5, y=0.7, yaw=math.pi),
                },
                agent_goals={
                    first: AgentGoalConfig(x=12.0, y=0.0),
                    second: AgentGoalConfig(x=13.2, y=0.15),
                    third: AgentGoalConfig(x=12.8, y=-0.9),
                    fourth: AgentGoalConfig(x=5.4, y=6.2),
                    fifth: AgentGoalConfig(x=0.0, y=0.7),
                },
            )

        if kind == 'five_usv_dense_overtaking_merge':
            if len(agent_ids) < 5:
                raise ValueError('five_usv_dense_overtaking_merge requires at least 5 agents.')
            first, second, third, fourth, fifth = agent_ids[:5]
            return FleetScenario(
                name=kind,
                duration=55.0,
                agent_spawns={
                    first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                    second: AgentSpawnConfig(x=2.2, y=0.45, yaw=0.0),
                    third: AgentSpawnConfig(x=3.6, y=-1.2, yaw=0.0),
                    fourth: AgentSpawnConfig(x=4.8, y=-3.5, yaw=math.pi / 2.0),
                    fifth: AgentSpawnConfig(x=9.5, y=1.4, yaw=math.pi),
                },
                agent_goals={
                    first: AgentGoalConfig(x=12.0, y=0.0),
                    second: AgentGoalConfig(x=13.4, y=0.0),
                    third: AgentGoalConfig(x=13.0, y=-0.8),
                    fourth: AgentGoalConfig(x=6.5, y=6.5),
                    fifth: AgentGoalConfig(x=0.0, y=1.0),
                },
            )

        raise ValueError(f'Unsupported multi-agent scenario kind: {kind}')

    @staticmethod
    def _maybe_randomize(
        scenario: FleetScenario,
        rng: Optional[np.random.Generator],
        spawn_position_std: float,
        spawn_heading_std: float,
        goal_position_std: float,
    ) -> FleetScenario:
        if rng is None or (spawn_position_std <= 0 and spawn_heading_std <= 0 and goal_position_std <= 0):
            return scenario
        return scenario.apply_randomization(
            rng,
            spawn_position_std=spawn_position_std,
            spawn_heading_std=spawn_heading_std,
            goal_position_std=goal_position_std,
        )