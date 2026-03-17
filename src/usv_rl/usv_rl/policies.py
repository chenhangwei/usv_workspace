from dataclasses import dataclass

import numpy as np


@dataclass
class LinearResidualPolicy:
    weights: np.ndarray
    bias: np.ndarray
    obs_mean: np.ndarray
    obs_std: np.ndarray
    action_low: np.ndarray
    action_high: np.ndarray

    @classmethod
    def load(cls, path: str) -> 'LinearResidualPolicy':
        data = np.load(path)
        return cls(
            weights=data['weights'],
            bias=data['bias'],
            obs_mean=data['obs_mean'],
            obs_std=data['obs_std'],
            action_low=data['action_low'],
            action_high=data['action_high'],
        )

    @property
    def action_dim(self) -> int:
        return int(np.asarray(self.action_high).shape[0])

    @property
    def obs_dim(self) -> int:
        return int(np.asarray(self.obs_mean).shape[0])

    def predict(self, observation: np.ndarray) -> np.ndarray:
        normalized = (observation - self.obs_mean) / self.obs_std
        action = normalized @ self.weights + self.bias
        return np.clip(action, self.action_low, self.action_high)


@dataclass
class MlpResidualPolicy:
    layer_weights: list[np.ndarray]
    layer_biases: list[np.ndarray]
    obs_mean: np.ndarray
    obs_std: np.ndarray
    action_low: np.ndarray
    action_high: np.ndarray
    activation: str = 'tanh'

    @classmethod
    def load(cls, path: str) -> 'MlpResidualPolicy':
        data = np.load(path)
        hidden_sizes = tuple(int(v) for v in np.asarray(data['hidden_sizes']).tolist())
        layer_sizes = hidden_sizes + (int(np.asarray(data['action_dim']).item()),)
        layer_weights = []
        layer_biases = []
        for layer_index, _ in enumerate(layer_sizes):
            layer_weights.append(data[f'layer_{layer_index}_weight'])
            layer_biases.append(data[f'layer_{layer_index}_bias'])

        activation = 'tanh'
        if 'activation' in data:
            activation = str(np.asarray(data['activation']).item())

        return cls(
            layer_weights=layer_weights,
            layer_biases=layer_biases,
            obs_mean=data['obs_mean'],
            obs_std=data['obs_std'],
            action_low=data['action_low'],
            action_high=data['action_high'],
            activation=activation,
        )

    def _activate(self, value: np.ndarray) -> np.ndarray:
        if self.activation == 'relu':
            return np.maximum(value, 0.0)
        return np.tanh(value)

    @property
    def action_dim(self) -> int:
        return int(np.asarray(self.action_high).shape[0])

    @property
    def obs_dim(self) -> int:
        return int(np.asarray(self.obs_mean).shape[0])

    def predict(self, observation: np.ndarray) -> np.ndarray:
        hidden = (observation - self.obs_mean) / self.obs_std
        for layer_index, (weight, bias) in enumerate(zip(self.layer_weights, self.layer_biases)):
            hidden = hidden @ weight.T + bias
            if layer_index < (len(self.layer_weights) - 1):
                hidden = self._activate(hidden)
        return np.clip(hidden, self.action_low, self.action_high).astype(np.float32)


def load_residual_policy(path: str):
    data = np.load(path)
    model_type = str(np.asarray(data['model_type']).item()) if 'model_type' in data else 'linear'
    if model_type == 'linear':
        return LinearResidualPolicy.load(path)
    if model_type == 'mlp':
        return MlpResidualPolicy.load(path)
    raise RuntimeError(f'Unsupported residual policy type in {path}: {model_type}')