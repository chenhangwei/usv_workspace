import json
import math
from dataclasses import replace
from pathlib import Path

from .multi_agent_types import AgentLocalObservation


def load_runtime_metadata(path):
    with Path(path).open() as stream:
        metadata = json.loads(stream.readline())
    if metadata.get('type') != 'metadata' or metadata.get('runtime_action_contract_version') != 1:
        raise ValueError('expected a policy trace metadata header with action contract version 1')
    return metadata


def collection_config_from_metadata(config, metadata):
    if len(config.agent_namespaces) != 1:
        raise ValueError('runtime projection profile is currently verified for true-solo collection only')
    if metadata.get('runtime_action_contract_version') != 1:
        raise ValueError('unsupported runtime action contract version')
    if config.action_mode != 'speed_scale' or metadata.get('action_mode') != 'speed_scale':
        raise ValueError('runtime collection profile requires speed_scale mode')
    if metadata.get('obs_dim') != AgentLocalObservation.vector_size(config.max_neighbors):
        raise ValueError('runtime observation width differs from collection layout')
    contract = metadata['action_contract']
    bounds = metadata['action_bounds']
    projection = metadata['projection']
    expected = {
        'scale_min': (config.action_speed_scale_min, contract['scale_min']),
        'cruise_speed': (config.cruise_speed, contract['cruise_speed']),
        'linear_bound': (max(config.cruise_speed, config.action_bounds.linear_delta), bounds['linear']),
        'angular_bound': (max(config.max_angular_velocity, config.action_bounds.angular_delta), bounds['angular']),
    }
    for name, (actual, required) in expected.items():
        if not math.isfinite(float(required)) or not math.isclose(float(actual), float(required), rel_tol=1e-6, abs_tol=1e-7):
            raise ValueError(f'collection {name}={actual} differs from runtime {required}')
    names = (
        'control_dt', 'heading_omega_deadband', 'heading_omega_reference', 'angular_authority_power',
        'angular_accel_limit', 'angular_decel_limit', 'conflict_turn_relief',
        'angular_authority_floor', 'min_forward_speed_floor',
    )
    values = {name: float(projection[name]) for name in names}
    values.update(goal_tolerance=float(contract['goal_tolerance']),
                  goal_proximity_relief_distance=float(contract['relief_distance']),
                  goal_proximity_speed_relief=float(contract['speed_relief']))
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError('runtime projection and speed parameters must be finite')
    if any(values[name] <= 0 for name in ('control_dt', 'angular_authority_power', 'angular_accel_limit', 'angular_decel_limit')):
        raise ValueError('runtime timing and angular rate limits must be positive')
    if not 0 <= values['heading_omega_deadband'] < values['heading_omega_reference']:
        raise ValueError('runtime heading thresholds must be ordered and nonnegative')
    if any(not 0 <= values[name] <= 1 for name in ('conflict_turn_relief', 'angular_authority_floor', 'goal_proximity_speed_relief')):
        raise ValueError('runtime authority and relief values must be in [0, 1]')
    if not 0 <= values['min_forward_speed_floor'] <= config.cruise_speed:
        raise ValueError('runtime forward floor exceeds collection speed bounds')
    if not 0 <= values['goal_tolerance'] < values['goal_proximity_relief_distance']:
        raise ValueError('runtime goal relief distance must exceed goal tolerance')
    minimum_speed = float(contract['minimum_speed'])
    if not math.isfinite(minimum_speed) or not .08 <= minimum_speed <= config.cruise_speed:
        raise ValueError('runtime navigation minimum speed is outside collection bounds')
    reward = replace(config.reward, desired_conflict_speed=minimum_speed, conflict_speed_floor=minimum_speed)
    return replace(config, reward=reward, **values)