from pathlib import Path

from .online_launch_parsing import has_unexpected_traceback


def inspect_benchmark_log(log_path: Path | None, scenario: str) -> dict:
    if log_path is None:
        return {
            'path': None,
            'exists': False,
            'scenario_started': False,
            'observation_ready': False,
            'first_policy_action_logged': False,
            'unexpected_traceback': False,
            'content_valid': False,
        }

    if not log_path.exists():
        return {
            'path': str(log_path),
            'exists': False,
            'scenario_started': False,
            'observation_ready': False,
            'first_policy_action_logged': False,
            'unexpected_traceback': False,
            'content_valid': False,
        }

    content = log_path.read_text(encoding='utf-8', errors='replace')
    scenario_started = f'Started synthetic neighbor publisher scenario={scenario} ' in content
    observation_ready = 'Observation stream ready; pure RL policy inference is active.' in content
    first_policy_action_logged = 'Publishing first pure RL action:' in content
    unexpected_traceback = has_unexpected_traceback(content)
    content_valid = scenario_started and observation_ready and first_policy_action_logged and not unexpected_traceback

    return {
        'path': str(log_path),
        'exists': True,
        'scenario_started': scenario_started,
        'observation_ready': observation_ready,
        'first_policy_action_logged': first_policy_action_logged,
        'unexpected_traceback': unexpected_traceback,
        'content_valid': content_valid,
    }


def inspect_smoke_log(log_path: Path | None) -> dict:
    if log_path is None:
        return {
            'path': None,
            'exists': False,
            'effective_config_logged': False,
            'policy_process_started': False,
            'controller_param_enabled': False,
            'unexpected_traceback': False,
            'content_valid': False,
        }

    if not log_path.exists():
        return {
            'path': str(log_path),
            'exists': False,
            'effective_config_logged': False,
            'policy_process_started': False,
            'controller_param_enabled': False,
            'unexpected_traceback': False,
            'content_valid': False,
        }

    content = log_path.read_text(encoding='utf-8', errors='replace')
    effective_config_logged = '[USV Launch] 生效配置:' in content
    policy_process_started = 'policy_inference_node' in content
    controller_param_enabled = 'Enabled rl_policy_enabled on velocity_controller_node.' in content
    unexpected_traceback = has_unexpected_traceback(content)
    content_valid = effective_config_logged and policy_process_started and controller_param_enabled and not unexpected_traceback

    return {
        'path': str(log_path),
        'exists': True,
        'effective_config_logged': effective_config_logged,
        'policy_process_started': policy_process_started,
        'controller_param_enabled': controller_param_enabled,
        'unexpected_traceback': unexpected_traceback,
        'content_valid': content_valid,
    }