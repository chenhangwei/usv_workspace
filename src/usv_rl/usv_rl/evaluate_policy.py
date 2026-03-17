import argparse
import json
from pathlib import Path

import numpy as np

from .config import EnvConfig
from .env import UsvRlEnv
from .policies import load_residual_policy
from .scenarios import ScenarioFactory


class ZeroPolicy:
    def predict(self, observation: np.ndarray) -> np.ndarray:
        return np.zeros(2, dtype=np.float32)

    @property
    def action_dim(self) -> int:
        return 2


class PpoResidualPolicy:
    def __init__(self, model_path: str, device: str = 'cpu'):
        try:
            from stable_baselines3 import PPO
        except ImportError as exc:
            raise RuntimeError(
                'stable_baselines3 is not installed. Install gymnasium and stable-baselines3 before PPO evaluation.'
            ) from exc
        self._model = PPO.load(model_path, device=device)

    @property
    def action_dim(self) -> int:
        shape = getattr(self._model.action_space, 'shape', None)
        if not shape:
            return 2
        return int(shape[0])

    @property
    def obs_dim(self) -> int:
        shape = getattr(self._model.observation_space, 'shape', None)
        if not shape:
            return 28
        return int(shape[0])

    def predict(self, observation: np.ndarray) -> np.ndarray:
        action, _ = self._model.predict(observation, deterministic=True)
        return np.asarray(action, dtype=np.float32)


def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate a residual policy in the lightweight USV RL environment.')
    parser.add_argument('--policy', choices=['auto', 'zero', 'bc', 'ppo'], default='auto', help='Policy backend.')
    parser.add_argument('--model', help='Model path for bc (.npz) or ppo (.zip). Not required for zero policy.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--episodes', type=int, default=9, help='Number of evaluation episodes.')
    parser.add_argument('--steps-per-episode', type=int, default=240, help='Maximum control steps per episode.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name to include. Repeatable.')
    parser.add_argument('--external-stack', action='store_true', help='Use an already-running stack.')
    parser.add_argument('--device', default='cpu', help='Torch device for PPO evaluation.')
    parser.add_argument('--action-mode', choices=['auto', 'full', 'angular_only'], default='auto', help='Residual action representation used during evaluation.')
    parser.add_argument('--max-neighbors', default='auto', help='Neighbor slots encoded during evaluation. Use auto to infer from the model, or an integer such as 4.')
    parser.add_argument('--min-separation-threshold', type=float, default=0.5, help='Hard acceptance threshold for episode minimum neighbor distance.')
    parser.add_argument('--output-json', help='Optional path to save evaluation metrics as JSON.')
    return parser.parse_args()


def _resolve_action_mode(args, policy) -> str:
    if args.action_mode != 'auto':
        return args.action_mode
    action_dim = getattr(policy, 'action_dim', None)
    if action_dim is None:
        action_high = getattr(policy, 'action_high', None)
        if action_high is not None:
            action_dim = int(np.asarray(action_high).shape[0])
    if action_dim == 1:
        return 'angular_only'
    return 'full'


def _resolve_max_neighbors(args, policy) -> int:
    if str(args.max_neighbors).lower() != 'auto':
        return max(1, int(args.max_neighbors))

    obs_dim = getattr(policy, 'obs_dim', None)
    if obs_dim is None:
        obs_mean = getattr(policy, 'obs_mean', None)
        if obs_mean is not None:
            obs_dim = int(np.asarray(obs_mean).shape[0])

    if obs_dim is None or obs_dim < 10:
        return 4

    inferred = (int(obs_dim) - 10) // 6
    return max(1, inferred)


def _detect_policy_kind(policy_kind: str, model_path: str | None) -> str:
    if policy_kind != 'auto':
        return policy_kind
    if not model_path:
        return 'zero'
    suffix = Path(model_path).suffix.lower()
    if suffix == '.npz':
        return 'bc'
    if suffix == '.zip':
        return 'ppo'
    raise RuntimeError(f'Unable to infer policy type from model path: {model_path}')


def _load_policy(policy_kind: str, model_path: str | None, device: str):
    if policy_kind == 'zero':
        return ZeroPolicy()
    if model_path is None:
        raise RuntimeError(f'Policy type {policy_kind} requires --model.')
    if policy_kind == 'bc':
        return load_residual_policy(model_path)
    if policy_kind == 'ppo':
        return PpoResidualPolicy(model_path, device=device)
    raise RuntimeError(f'Unsupported policy type: {policy_kind}')


def _final_distance(observation: np.ndarray) -> float:
    return float(observation[4])


def _build_scenario_summaries(episode_metrics: list[dict]) -> dict[str, dict[str, float | int]]:
    grouped: dict[str, list[dict]] = {}
    for metric in episode_metrics:
        grouped.setdefault(metric['scenario'], []).append(metric)

    summaries: dict[str, dict[str, float | int]] = {}
    for scenario, items in grouped.items():
        count = max(1, len(items))
        summaries[scenario] = {
            'episodes': len(items),
            'success_rate': sum(item['success'] for item in items) / count,
            'collision_rate': sum(item['collision'] for item in items) / count,
            'timeout_rate': sum(item['timeout'] for item in items) / count,
            'min_separation_pass_rate': sum(item['separation_ok'] for item in items) / count,
            'mean_final_distance': float(np.mean([item['final_distance'] for item in items])),
            'mean_min_neighbor_distance': float(np.mean([item['min_neighbor_distance'] for item in items])),
            'worst_case_min_neighbor_distance': float(np.min([item['min_neighbor_distance'] for item in items])),
        }
    return summaries


def main():
    args = parse_args()
    scenarios = tuple(args.scenarios) if args.scenarios else ScenarioFactory.cluster_standard_available()
    policy_kind = _detect_policy_kind(args.policy, args.model)
    policy = _load_policy(policy_kind, args.model, args.device)
    action_mode = _resolve_action_mode(args, policy)
    max_neighbors = _resolve_max_neighbors(args, policy)

    env = UsvRlEnv(
        EnvConfig(
            namespace=args.namespace,
            launch_sitl=not args.external_stack,
            enable_rl_backend=True,
            action_mode=action_mode,
            max_neighbors=max_neighbors,
            default_scenarios=scenarios,
        )
    )

    episode_metrics = []
    try:
        for episode in range(args.episodes):
            scenario_name = scenarios[episode % len(scenarios)]
            observation, info = env.reset(options={'scenario_kind': scenario_name})
            episode_return = 0.0
            episode_min_neighbor_distance = float('inf')
            terminated = False
            truncated = False
            steps = 0

            for step in range(args.steps_per_episode):
                action = policy.predict(observation)
                observation, reward, terminated, truncated, step_info = env.step(action)
                episode_return += float(reward)
                episode_min_neighbor_distance = min(
                    episode_min_neighbor_distance,
                    float(step_info['min_neighbor_distance']),
                )
                steps = step + 1
                if terminated or truncated:
                    break

            final_distance = _final_distance(observation)
            success = terminated and final_distance <= env.config.goal_tolerance
            collision = terminated and episode_min_neighbor_distance < env.config.collision_distance
            timeout = truncated and not success and not collision
            separation_ok = episode_min_neighbor_distance >= args.min_separation_threshold

            episode_metrics.append(
                {
                    'episode': episode,
                    'scenario': info['scenario'],
                    'steps': steps,
                    'return': episode_return,
                    'success': success,
                    'collision': collision,
                    'timeout': timeout,
                    'separation_ok': separation_ok,
                    'final_distance': final_distance,
                    'min_neighbor_distance': episode_min_neighbor_distance,
                }
            )
    finally:
        env.close()

    success_rate = sum(item['success'] for item in episode_metrics) / max(1, len(episode_metrics))
    collision_rate = sum(item['collision'] for item in episode_metrics) / max(1, len(episode_metrics))
    timeout_rate = sum(item['timeout'] for item in episode_metrics) / max(1, len(episode_metrics))
    mean_return = float(np.mean([item['return'] for item in episode_metrics]))
    mean_steps = float(np.mean([item['steps'] for item in episode_metrics]))
    mean_final_distance = float(np.mean([item['final_distance'] for item in episode_metrics]))
    mean_min_neighbor_distance = float(np.mean([item['min_neighbor_distance'] for item in episode_metrics]))
    min_separation_pass_rate = sum(item['separation_ok'] for item in episode_metrics) / max(1, len(episode_metrics))
    worst_case_min_neighbor_distance = float(np.min([item['min_neighbor_distance'] for item in episode_metrics]))

    summary = {
        'policy': policy_kind,
        'model': args.model,
        'action_mode': action_mode,
        'max_neighbors': max_neighbors,
        'collision_distance': env.config.collision_distance,
        'min_separation_threshold': args.min_separation_threshold,
        'episodes': len(episode_metrics),
        'scenarios': list(scenarios),
        'success_rate': success_rate,
        'collision_rate': collision_rate,
        'timeout_rate': timeout_rate,
        'min_separation_pass_rate': min_separation_pass_rate,
        'mean_return': mean_return,
        'mean_steps': mean_steps,
        'mean_final_distance': mean_final_distance,
        'mean_min_neighbor_distance': mean_min_neighbor_distance,
        'worst_case_min_neighbor_distance': worst_case_min_neighbor_distance,
        'scenario_summaries': _build_scenario_summaries(episode_metrics),
        'episode_metrics': episode_metrics,
    }

    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if args.output_json:
        output_path = Path(args.output_json)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved evaluation summary to {output_path}')


if __name__ == '__main__':
    main()