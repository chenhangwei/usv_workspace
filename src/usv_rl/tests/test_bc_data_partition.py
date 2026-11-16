import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

import numpy as np
import torch

from usv_rl.bc_pretrain_attention_actor import (
    _balance_recovery_weights, _load_dataset, _partition_dataset, _prepare_normalizer, main,
)
from usv_rl.neighbor_attention import AttentionActor
from usv_rl.observation_normalizer import ObservationNormalizer


class BCDataPartitionTests(unittest.TestCase):
    def dataset(self):
        return {
            'observations': np.array([[1., 2.], [3., 4.], [1000., 2000.], [3000., 4000.]]),
            'actions': np.zeros((4, 2)),
            'episode_seeds': np.array([177, 177, 178, 178]),
            'pair_ids': np.array([0, 0, 1, 1]),
            'split': np.array(['train', 'train', 'validation', 'validation']),
        }

    def test_normalizer_never_fits_validation_observations(self):
        train, validation = _partition_dataset(self.dataset(), None, .1, 9)
        normalizer = _prepare_normalizer(2, train['observations'], 'fit_train', {})
        before = normalizer.state_dict()
        normalizer.normalize(validation['observations'])
        np.testing.assert_allclose(before['mean'], [2., 3.], atol=.0002)
        self.assertAlmostEqual(before['count'], 2.0001)
        np.testing.assert_array_equal(normalizer.state_dict()['mean'], before['mean'])

    def test_warmstart_normalizer_is_frozen(self):
        source = ObservationNormalizer(2)
        source.update(np.array([[10., 20.], [20., 30.]]))
        state = source.state_dict()
        normalizer = _prepare_normalizer(2, np.array([[1000., 2000.]]), 'warmstart', {'obs_normalizer': state})
        for key, value in state.items():
            np.testing.assert_array_equal(normalizer.state_dict()[key], value)
        with self.assertRaises(ValueError):
            _prepare_normalizer(2, np.zeros((1, 2)), 'warmstart', {})

    def test_rejects_trajectory_leakage_and_wrong_split(self):
        train, validation = _partition_dataset(self.dataset(), None, .1, 9)
        for key in ('episode_seeds', 'pair_ids'):
            invalid = {**validation, key: train[key].copy()}
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'overlap'):
                _partition_dataset(train, invalid, .1, 9)
        with self.assertRaisesRegex(ValueError, 'another split'):
            _partition_dataset(self.dataset(), validation, .1, 9)

    def test_legacy_split_has_no_empty_partition(self):
        data = {'observations': np.ones((2, 2)), 'actions': np.zeros((2, 2))}
        train, validation = _partition_dataset(data, None, .9, 9)
        self.assertEqual(len(train['observations']), 1)
        self.assertEqual(len(validation['observations']), 1)
        with self.assertRaises(ValueError):
            _partition_dataset({key: value[:1] for key, value in data.items()}, None, .1, 9)

    def test_side_balance_preserves_mass_and_other_samples(self):
        observations = np.zeros((5, 82), dtype=np.float32)
        observations[:, 3] = .12
        heading = np.radians([-10., 10., 10., 10., 10.])
        observations[:, 5], observations[:, 6] = np.sin(heading), np.cos(heading)
        observations[:, 17] = [-1., 1., 1., 1., 1.]
        observations[-1, 23] = 2.0
        original = np.array([2., 1., 1., 1., 3.], dtype=np.float32)
        balanced = _balance_recovery_weights(observations, original)
        self.assertAlmostEqual(balanced[0], balanced[1:4].sum())
        self.assertAlmostEqual(balanced.sum(), original.sum())
        self.assertEqual(balanced[-1], original[-1])
        np.testing.assert_array_equal(original, [2., 1., 1., 1., 3.])
        with self.assertRaises(ValueError):
            _balance_recovery_weights(observations[1:], original[1:])

    def test_loader_rejects_invalid_weights_and_nonfinite_observations(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'data.npz'
            for weights in (np.array([0., 1.]), np.array([float('nan'), 1.])):
                np.savez_compressed(path, observations=np.ones((2, 2)), actions=np.ones((2, 2)), sample_weights=weights)
                with self.assertRaises(ValueError):
                    _load_dataset(path, 2, 2)
            np.savez_compressed(path, observations=np.full((2, 2), float('inf')), actions=np.ones((2, 2)))
            with self.assertRaises(ValueError):
                _load_dataset(path, 2, 2)

    def test_loader_rejects_mismatched_observation_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            legacy, canonical = Path(directory) / 'legacy.npz', Path(directory) / 'canonical.npz'
            np.savez_compressed(legacy, observations=np.ones((2, 2)), actions=np.ones((2, 2)))
            np.savez_compressed(canonical, observations=np.ones((2, 2)), actions=np.ones((2, 2)),
                                observation_frame=np.asarray('route_canonical_v1'))
            self.assertEqual(len(_load_dataset(legacy, 2, 2)['observations']), 2)
            self.assertEqual(len(_load_dataset(canonical, 2, 2, 'route_canonical_v1')['observations']), 2)
            for path, frame in ((legacy, 'route_canonical_v1'), (canonical, 'legacy')):
                with self.subTest(path=path.name), self.assertRaises(ValueError):
                    _load_dataset(path, 2, 2, frame)

    def test_training_roundtrip_uses_all_train_and_only_external_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            base, output = folder / 'base.pt', folder / 'candidate.pt'
            train_path, val_path = folder / 'train.npz', folder / 'validation.npz'
            actor = AttentionActor(max_neighbors=4, encounter_dim=3, hidden_sizes=(8, 8),
                                   action_dim=2, embed_dim=8, num_heads=1)
            state = ObservationNormalizer(82).state_dict()
            checkpoint = {
                'actor_state_dict': actor.state_dict(), 'critic_state_dict': {},
                'actor_log_std': torch.zeros(2), 'action_dim': 2,
                'action_low': np.array([.3, -.7]), 'action_high': np.array([1., .7]),
                'local_observation_size': 82, 'neighbor_attention': True,
                'max_neighbors': 4, 'hidden_sizes': [8, 8], 'attention_embed_dim': 8,
                'attention_num_heads': 1, 'squash_actions': True, 'action_mode': 'speed_scale',
                'obs_normalizer': state,
            }
            torch.save(checkpoint, base)
            for path, label, seed in ((train_path, 'train', 177), (val_path, 'validation', 179)):
                obs = np.zeros((4, 82), dtype=np.float32)
                obs[:, 3] = .12
                obs[:, 5] = np.sin(np.radians([-10, 10, 10, 10]))
                obs[:, 6] = np.cos(np.radians(10))
                obs[:, 11] = obs[:, 17] = [-1, 1, 1, 1]
                np.savez_compressed(path, observations=obs, actions=np.array([[1, -.7], [1, .7], [1, .7], [1, .7]]),
                                    episode_seeds=np.full(4, seed), split=np.full(4, label))
            argv = ['bc', '--warmstart', str(base), '--dataset', str(train_path),
                    '--validation-dataset', str(val_path), '--normalizer-mode', 'warmstart',
                    '--balance-recovery-sides', '--epochs', '1', '--batch-size', '4', '--output', str(output)]
            with patch('sys.argv', argv):
                main()
            saved = torch.load(output, map_location='cpu', weights_only=False)
            self.assertEqual(saved['bc_pretrain_metadata']['samples'], 4)
            self.assertEqual(saved['bc_pretrain_metadata']['validation_samples'], 4)
            self.assertEqual(saved['bc_pretrain_metadata']['epochs_ran'], 1)
            self.assertLessEqual(saved['bc_pretrain_metadata']['best_val_mse'],
                                 saved['bc_pretrain_metadata']['baseline_val_mse'])
            for key, value in state.items():
                np.testing.assert_array_equal(saved['obs_normalizer'][key], value)
            actor.load_state_dict(saved['actor_state_dict'])


if __name__ == '__main__':
    unittest.main()