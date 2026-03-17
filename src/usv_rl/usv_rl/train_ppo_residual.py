import argparse
from pathlib import Path

import numpy as np

from .config import EnvConfig
from .env import UsvRlEnv
from .scenarios import ScenarioFactory


def _create_ppo_model(PPO, env, args, n_steps: int, batch_size: int):
    return PPO('MlpPolicy', env, verbose=1, device=args.device, n_steps=n_steps, batch_size=batch_size)


def _load_or_initialize_model(PPO, env, args, n_steps: int, batch_size: int):
    if not args.load_model:
        return _create_ppo_model(PPO, env, args, n_steps, batch_size)

    try:
        return PPO.load(args.load_model, env=env, device=args.device)
    except ValueError as exc:
        if 'Action spaces do not match' not in str(exc):
            raise

        print('Detected action-space mismatch while loading PPO checkpoint; transferring policy weights into a fresh model.')
        source_model = PPO.load(args.load_model, device=args.device)
        target_model = _create_ppo_model(PPO, env, args, n_steps, batch_size)
        target_model.set_parameters(source_model.get_parameters(), exact_match=False)
        return target_model


def _pretrain_policy_from_dataset(model, dataset_path: str, *, epochs: int, batch_size: int, learning_rate: float):
    if epochs <= 0:
        return

    try:
        import torch
        import torch.nn.functional as F
    except ImportError as exc:
        raise RuntimeError('torch is required for PPO policy pretraining.') from exc

    dataset = np.load(dataset_path)
    observations = dataset['observations'].astype(np.float32)
    actions = dataset['actions'].astype(np.float32)
    if observations.shape[0] == 0:
        raise RuntimeError(f'Pretraining dataset is empty: {dataset_path}')

    optimizer = model.policy.optimizer
    for param_group in optimizer.param_groups:
        param_group['lr'] = learning_rate

    device = model.policy.device
    rng = np.random.default_rng(0)
    model.policy.set_training_mode(True)

    effective_batch_size = max(1, batch_size)
    for epoch in range(epochs):
        indices = rng.permutation(observations.shape[0])
        batch_losses = []
        for start in range(0, observations.shape[0], effective_batch_size):
            batch_indices = indices[start:start + effective_batch_size]
            obs_tensor = torch.as_tensor(observations[batch_indices], dtype=torch.float32, device=device)
            action_tensor = torch.as_tensor(actions[batch_indices], dtype=torch.float32, device=device)

            distribution = model.policy.get_distribution(obs_tensor)
            predicted_action = distribution.distribution.mean
            loss = F.mse_loss(predicted_action, action_tensor)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            batch_losses.append(float(loss.detach().cpu().item()))

        mean_loss = float(np.mean(batch_losses)) if batch_losses else 0.0
        print(f'Pretrain epoch {epoch + 1}/{epochs}: action_mse={mean_loss:.6f}')

    model.policy.set_training_mode(False)
    print(f'Completed PPO policy pretraining from {Path(dataset_path)} using {observations.shape[0]} samples.')


def parse_args():
    parser = argparse.ArgumentParser(description='Optional PPO fine-tuning entry for the residual RL environment.')
    parser.add_argument('--namespace', default='usv_03', help='Target USV namespace.')
    parser.add_argument('--total-timesteps', type=int, default=20000, help='Total PPO timesteps.')
    parser.add_argument('--output', required=True, help='Output model path prefix.')
    parser.add_argument('--external-stack', action='store_true', help='Use an already-running SITL stack.')
    parser.add_argument('--device', default='cpu', help='Torch device for PPO. Default keeps training on CPU.')
    parser.add_argument('--n-steps', type=int, default=None, help='Rollout steps per PPO update. Defaults to a small value for smoke runs.')
    parser.add_argument('--batch-size', type=int, default=None, help='Mini-batch size. Defaults to min(64, n_steps).')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name to train on. Repeatable.')
    parser.add_argument('--action-mode', choices=['full', 'angular_only'], default='angular_only', help='Residual action representation used during PPO training.')
    parser.add_argument('--max-neighbors', type=int, default=4, help='Number of neighbors encoded for retraining. Use 4 for 5-USV standard scenarios.')
    parser.add_argument('--load-model', help='Optional PPO checkpoint (.zip) to continue training from.')
    parser.add_argument('--pretrain-dataset', help='Optional .npz dataset with observations/actions for supervised policy pretraining.')
    parser.add_argument('--pretrain-epochs', type=int, default=0, help='Supervised pretraining epochs before PPO learning.')
    parser.add_argument('--pretrain-batch-size', type=int, default=128, help='Mini-batch size for supervised pretraining.')
    parser.add_argument('--pretrain-learning-rate', type=float, default=3e-4, help='Learning rate for supervised pretraining.')
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        from stable_baselines3 import PPO
    except ImportError as exc:
        raise RuntimeError(
            'stable_baselines3 is not installed. Install gymnasium and stable-baselines3 before PPO fine-tuning.'
        ) from exc

    config = EnvConfig(
        namespace=args.namespace,
        launch_sitl=not args.external_stack,
        enable_rl_backend=True,
        action_mode=args.action_mode,
        max_neighbors=max(1, args.max_neighbors),
        default_scenarios=tuple(args.scenarios) if args.scenarios else ScenarioFactory.cluster_standard_available(),
    )
    env = UsvRlEnv(config)
    try:
        n_steps = args.n_steps or max(8, min(256, args.total_timesteps))
        batch_size = args.batch_size or min(64, n_steps)
        model = _load_or_initialize_model(PPO, env, args, n_steps, batch_size)
        if args.pretrain_dataset:
            _pretrain_policy_from_dataset(
                model,
                args.pretrain_dataset,
                epochs=max(0, args.pretrain_epochs),
                batch_size=max(1, args.pretrain_batch_size),
                learning_rate=float(args.pretrain_learning_rate),
            )
        if args.total_timesteps > 0:
            model.learn(total_timesteps=args.total_timesteps)
        model.save(args.output)
        print(f'Saved PPO model to {args.output}')
    finally:
        env.close()


if __name__ == '__main__':
    main()