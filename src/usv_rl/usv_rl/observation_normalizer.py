"""Running observation normalizer for MAPPO training.

Uses Welford's online algorithm to maintain running mean and variance
statistics across all agents and episodes. Observations are normalized
to approximately zero-mean, unit-variance, then clipped to prevent
extreme values.

This is critical for stable RL training — raw observations in this
project span 3+ orders of magnitude (pose 0-14 m vs speed 0-0.4 m/s
vs angles ±π), which causes severe gradient imbalance without
normalization.
"""

import numpy as np


class RunningMeanStd:
    """Tracks running mean and variance using Welford's online algorithm."""

    __slots__ = ('mean', 'var', 'count')

    def __init__(self, shape: tuple[int, ...]):
        self.mean = np.zeros(shape, dtype=np.float64)
        self.var = np.ones(shape, dtype=np.float64)
        self.count: float = 1e-4

    def update(self, batch: np.ndarray) -> None:
        batch = np.asarray(batch, dtype=np.float64)
        if batch.ndim == 1:
            batch = batch[np.newaxis, :]
        batch_mean = batch.mean(axis=0)
        batch_var = batch.var(axis=0)
        batch_count = float(batch.shape[0])

        delta = batch_mean - self.mean
        total_count = self.count + batch_count
        new_mean = self.mean + delta * batch_count / total_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + (delta ** 2) * self.count * batch_count / total_count

        self.mean = new_mean
        self.var = m2 / total_count
        self.count = total_count

    def state_dict(self) -> dict:
        return {
            'mean': self.mean.copy(),
            'var': self.var.copy(),
            'count': float(self.count),
        }

    def load_state_dict(self, state: dict) -> None:
        self.mean = np.asarray(state['mean'], dtype=np.float64)
        self.var = np.asarray(state['var'], dtype=np.float64)
        self.count = float(state['count'])


class ObservationNormalizer:
    """Normalizes observations using running mean/variance statistics.

    During training, call ``update()`` with each batch of raw
    observations, then ``normalize()`` before feeding to the network.
    Save ``state_dict()`` into the checkpoint so inference can
    reproduce the same normalization.
    """

    def __init__(self, obs_dim: int, *, clip: float = 10.0, epsilon: float = 1e-8):
        self.rms = RunningMeanStd(shape=(obs_dim,))
        self.clip = clip
        self.epsilon = epsilon

    def update(self, obs_batch: np.ndarray) -> None:
        self.rms.update(obs_batch)

    def normalize(self, obs: np.ndarray) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float32)
        mean = self.rms.mean.astype(np.float32)
        std = np.sqrt(self.rms.var.astype(np.float32) + self.epsilon)
        return np.clip((obs - mean) / std, -self.clip, self.clip).astype(np.float32)

    def state_dict(self) -> dict:
        d = self.rms.state_dict()
        d['clip'] = float(self.clip)
        d['epsilon'] = float(self.epsilon)
        return d

    def load_state_dict(self, state: dict) -> None:
        self.rms.load_state_dict(state)
        self.clip = float(state.get('clip', 10.0))
        self.epsilon = float(state.get('epsilon', 1e-8))
