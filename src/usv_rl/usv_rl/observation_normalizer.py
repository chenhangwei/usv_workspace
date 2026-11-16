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
        obs_arr = np.asarray(obs, dtype=np.float32)
        mean = self.rms.mean.astype(np.float32)
        std = np.sqrt(self.rms.var.astype(np.float32) + self.epsilon)
        out = np.clip((obs_arr - mean) / std, -self.clip, self.clip).astype(np.float32)
        # Preserve the exact neighbor-mask invariant expected by _parse_local_obs (distance > 1e-6 iff slot active):
        # shift each neighbor slot's normalized distance by its empty-slot (raw=0) baseline + 1e-3 so active
        # neighbors (raw distance > 0) are strictly positive and empty padded slots (raw distance == 0) are 0.0.
        dim = obs_arr.shape[-1]
        for ego_dim in (19, 17, 12, 11, 10):
            for nb_dim in (15, 10, 6):
                rem = dim - ego_dim
                if rem > 0 and (rem % nb_dim == 0 or (rem >= 3 and (rem - 3) % nb_dim == 0)):
                    n_slots = (rem if rem % nb_dim == 0 else rem - 3) // nb_dim
                    for k in range(n_slots):
                        d_idx = ego_dim + k * nb_dim + 4
                        raw_d = obs_arr[..., d_idx]
                        z0 = float(-mean[d_idx] / std[d_idx])
                        shifted = np.maximum(out[..., d_idx] - z0 + 1e-3, 1e-3)
                        out[..., d_idx] = np.where(raw_d > 1e-6, shifted, 0.0)
                    return out
        return out

    def state_dict(self) -> dict:
        d = self.rms.state_dict()
        d['clip'] = float(self.clip)
        d['epsilon'] = float(self.epsilon)
        return d

    def load_state_dict(self, state: dict) -> None:
        self.rms.load_state_dict(state)
        self.clip = float(state.get('clip', 10.0))
        self.epsilon = float(state.get('epsilon', 1e-8))
