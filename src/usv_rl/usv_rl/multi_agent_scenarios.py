import math
from dataclasses import dataclass, field
from typing import Dict, List

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
    background_tracks: List[BackgroundTrack] = field(default_factory=list)

    def states_at(self, elapsed: float) -> List[NeighborState]:
        return [track.state_at(elapsed) for track in self.background_tracks]


class MultiAgentScenarioFactory:
    _REQUIRED_AGENT_COUNTS = {
        'two_usv_head_on': 2,
        'three_usv_crossing': 3,
        'three_usv_overtaking': 3,
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
        return ('two_usv_head_on', 'three_usv_crossing', 'three_usv_overtaking')

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
    ) -> FleetScenario:
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
            )

        if kind == 'three_usv_overtaking':
            if len(agent_ids) < 3:
                raise ValueError('three_usv_overtaking requires at least 3 agents.')
            first, second, third = agent_ids[:3]
            agent_spawns = {
                first: AgentSpawnConfig(x=0.0, y=0.0, yaw=0.0),
                second: AgentSpawnConfig(x=2.5, y=0.0, yaw=0.0),
                third: AgentSpawnConfig(x=4.0, y=-3.0, yaw=math.pi / 2.0),
            }
            agent_goals = {
                first: AgentGoalConfig(x=goal_distance, y=0.0),
                second: AgentGoalConfig(x=goal_distance + 1.5, y=0.0),
                third: AgentGoalConfig(x=4.0, y=goal_distance * 0.5),
            }
            MultiAgentScenarioFactory._add_spectator_agents(agent_ids, agent_spawns, agent_goals)
            return FleetScenario(
                name=kind,
                duration=50.0,
                agent_spawns=agent_spawns,
                agent_goals=agent_goals,
                background_tracks=[
                    BackgroundTrack(
                        track_id='background_head_on',
                        start_x=goal_distance * 0.75,
                        start_y=1.5,
                        vx=-neighbor_speed * 0.7,
                        vy=0.0,
                        yaw=math.pi,
                    ),
                ],
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