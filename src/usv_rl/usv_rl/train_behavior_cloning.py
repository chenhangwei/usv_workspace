import argparse
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description='Train a behavior cloning policy baseline.')
    parser.add_argument('--dataset', required=True, help='Collected .npz dataset path.')
    parser.add_argument('--output', required=True, help='Output model .npz path.')
    parser.add_argument('--model-type', choices=['linear', 'mlp'], default='linear', help='Behavior cloning model type.')
    parser.add_argument('--reg', type=float, default=1e-3, help='Ridge regularization strength for linear models.')
    parser.add_argument('--val-ratio', type=float, default=0.1, help='Validation split ratio.')
    parser.add_argument('--hidden-size', action='append', dest='hidden_sizes', type=int, default=None, help='Hidden layer size for MLP models. Repeatable.')
    parser.add_argument('--epochs', type=int, default=80, help='Training epochs for MLP models.')
    parser.add_argument('--batch-size', type=int, default=128, help='Batch size for MLP models.')
    parser.add_argument('--learning-rate', type=float, default=3e-4, help='Learning rate for MLP models.')
    parser.add_argument('--activation', choices=['tanh', 'relu'], default='tanh', help='Activation for MLP models.')
    return parser.parse_args()


def _mse(prediction: np.ndarray, target: np.ndarray) -> float:
    return float(np.mean(np.square(prediction - target)))


def _train_linear_model(args, train_x_norm, train_y, val_x_norm):
    train_x_aug = np.concatenate([train_x_norm, np.ones((train_x_norm.shape[0], 1), dtype=np.float32)], axis=1)
    identity = np.eye(train_x_aug.shape[1], dtype=np.float32)
    identity[-1, -1] = 0.0
    solution = np.linalg.solve(
        train_x_aug.T @ train_x_aug + args.reg * identity,
        train_x_aug.T @ train_y,
    )

    weights = solution[:-1]
    bias = solution[-1]
    train_prediction = train_x_norm @ weights + bias
    val_prediction = val_x_norm @ weights + bias
    payload = {
        'model_type': np.asarray('linear'),
        'weights': weights.astype(np.float32),
        'bias': bias.astype(np.float32),
    }
    return payload, train_prediction, val_prediction


def _train_mlp_model(args, train_x_norm, train_y, val_x_norm):
    try:
        import torch
        from torch import nn
    except ImportError as exc:
        raise RuntimeError('torch is required for MLP behavior cloning.') from exc

    hidden_sizes = tuple(args.hidden_sizes or [128, 128])
    activation_layer = nn.Tanh if args.activation == 'tanh' else nn.ReLU

    layers = []
    input_dim = train_x_norm.shape[1]
    for hidden_size in hidden_sizes:
        layers.append(nn.Linear(input_dim, hidden_size))
        layers.append(activation_layer())
        input_dim = hidden_size
    layers.append(nn.Linear(input_dim, train_y.shape[1]))
    model = nn.Sequential(*layers)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
    loss_fn = nn.MSELoss()
    batch_size = max(1, args.batch_size)
    rng = np.random.default_rng(0)

    train_x_tensor = torch.as_tensor(train_x_norm, dtype=torch.float32)
    train_y_tensor = torch.as_tensor(train_y, dtype=torch.float32)
    val_x_tensor = torch.as_tensor(val_x_norm, dtype=torch.float32)

    model.train()
    for _ in range(max(1, args.epochs)):
        permutation = rng.permutation(train_x_norm.shape[0])
        for start in range(0, train_x_norm.shape[0], batch_size):
            batch_indices = permutation[start:start + batch_size]
            prediction = model(train_x_tensor[batch_indices])
            loss = loss_fn(prediction, train_y_tensor[batch_indices])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

    model.eval()
    with torch.no_grad():
        train_prediction = model(train_x_tensor).cpu().numpy().astype(np.float32)
        val_prediction = model(val_x_tensor).cpu().numpy().astype(np.float32)

    linear_layers = [layer for layer in model if isinstance(layer, nn.Linear)]
    payload = {
        'model_type': np.asarray('mlp'),
        'hidden_sizes': np.asarray(hidden_sizes, dtype=np.int32),
        'action_dim': np.asarray(train_y.shape[1], dtype=np.int32),
        'activation': np.asarray(args.activation),
    }
    for layer_index, layer in enumerate(linear_layers):
        payload[f'layer_{layer_index}_weight'] = layer.weight.detach().cpu().numpy().astype(np.float32)
        payload[f'layer_{layer_index}_bias'] = layer.bias.detach().cpu().numpy().astype(np.float32)
    return payload, train_prediction, val_prediction


def main():
    args = parse_args()
    dataset = np.load(args.dataset)
    observations = dataset['observations'].astype(np.float32)
    actions = dataset['actions'].astype(np.float32)

    if observations.shape[0] < 10:
        raise RuntimeError('Dataset is too small for behavior cloning. Collect more episodes first.')

    val_size = max(1, int(observations.shape[0] * args.val_ratio))
    train_x = observations[:-val_size]
    val_x = observations[-val_size:]
    train_y = actions[:-val_size]
    val_y = actions[-val_size:]

    obs_mean = train_x.mean(axis=0)
    obs_std = train_x.std(axis=0)
    obs_std = np.where(obs_std < 1e-6, 1.0, obs_std)

    train_x_norm = (train_x - obs_mean) / obs_std
    val_x_norm = (val_x - obs_mean) / obs_std

    if args.model_type == 'linear':
        model_payload, train_prediction, val_prediction = _train_linear_model(args, train_x_norm, train_y, val_x_norm)
    else:
        model_payload, train_prediction, val_prediction = _train_mlp_model(args, train_x_norm, train_y, val_x_norm)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        **model_payload,
        obs_mean=obs_mean.astype(np.float32),
        obs_std=obs_std.astype(np.float32),
        action_low=actions.min(axis=0).astype(np.float32),
        action_high=actions.max(axis=0).astype(np.float32),
        train_mse=np.asarray(_mse(train_prediction, train_y), dtype=np.float32),
        val_mse=np.asarray(_mse(val_prediction, val_y), dtype=np.float32),
    )

    print(f'Train MSE: {_mse(train_prediction, train_y):.6f}')
    print(f'Val MSE: {_mse(val_prediction, val_y):.6f}')
    print(f'Saved model to {output_path}')


if __name__ == '__main__':
    main()