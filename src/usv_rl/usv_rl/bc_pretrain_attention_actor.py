"""Behaviour-cloning pretrain of the attention actor from a teacher dataset (M2-3).

Produces a MAPPO-compatible checkpoint whose attention actor has been supervised
to imitate the COLREGS rule teacher, so the fresh602 PPO run can warm-start from
it via ``--load-weights-from`` exactly like any other MAPPO checkpoint.

Why a standalone script (not train_distill_policy): that distiller builds a plain
MLP student, but the production actor is an AttentionActor -- the state_dicts are
incompatible. Here we rebuild the *same* AttentionActor from the warm-start
checkpoint's metadata, so the BC weights drop straight into PPO with zero
architecture transfer risk.

Key correctness points:
    * The student sees NORMALIZED observations, using training-only statistics
        or a frozen warm-start normalizer. Its state is baked into the output
        checkpoint (``--load-weights-from`` restores it).
  * The squash mapping (tanh -> [action_low, action_high]) mirrors the trainer
    exactly, so the MSE target (teacher action) lives in the same space as the
    squashed actor output.
  * The output checkpoint is a full copy of the warm-start checkpoint with only
    ``actor_state_dict`` and ``obs_normalizer`` replaced, keeping a valid
    critic / log_std / metadata payload.

Example:
  python3 -m usv_rl.bc_pretrain_attention_actor \
    --warmstart /mnt/data/checkpoints/usv_rl/fresh601_speedscale_nogate.pt \
    --dataset   /mnt/data/checkpoints/usv_rl/teacher_3usv.npz \
    --output    /mnt/data/checkpoints/usv_rl/fresh602_bc_init.pt \
    --epochs 200 --lr 1e-3 --batch-size 512
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

import numpy as np
import torch

from .neighbor_attention import AttentionActor
from .multi_agent_types import ENCOUNTER_TYPE_COUNT
from .observation_normalizer import ObservationNormalizer


def parse_args():
    p = argparse.ArgumentParser(description='BC-pretrain the attention actor from a teacher dataset.')
    p.add_argument('--warmstart', required=True, help='Base MAPPO checkpoint (.pt) providing actor architecture + metadata.')
    p.add_argument('--dataset', required=True, help='Teacher dataset (.npz) with observations + actions.')
    p.add_argument('--output', required=True, help='Output BC-initialised checkpoint (.pt).')
    p.add_argument('--epochs', type=int, default=200)
    p.add_argument('--lr', type=float, default=1e-3)
    p.add_argument('--batch-size', type=int, default=512)
    p.add_argument('--validation-split', type=float, default=0.1)
    p.add_argument('--validation-dataset', help='Independent held-out NPZ; disables random frame splitting.')
    p.add_argument('--normalizer-mode', choices=['fit_train', 'warmstart'], default='fit_train',
                   help='Fit on training observations only, or freeze the warm-start normalizer.')
    p.add_argument('--balance-recovery-sides', action='store_true',
                   help='Equalize total training weight of left/right true-solo low-speed recovery samples.')
    p.add_argument('--early-stopping-patience', type=int, default=20)
    p.add_argument('--engaged-weight', type=float, default=1.0,
                   help='MSE weight for samples flagged engaged in the dataset (dense-encounter emphasis).')
    p.add_argument('--max-grad-norm', type=float, default=0.5)
    p.add_argument('--input-dropout-slots', default='',
                   help='Comma-separated obs slot indices randomly zeroed (normalized space) during training. '
                        'Breaks causal confusion: e.g. 8,10 stops the student from copying raw/final omega '
                        'commands instead of steering from heading/CTE geometry (fresh643).')
    p.add_argument('--input-dropout-prob', type=float, default=0.6)
    p.add_argument('--device', default='cpu')
    p.add_argument('--observation-frame', choices=['legacy', 'route_canonical_v1'], default=None,
                   help='Observation frame of the training data, recorded in the checkpoint (default: keep warmstart).')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--scratch-embed-dim', type=int, default=0,
                   help='When > 0, train a freshly initialised actor with this attention width '
                        '(warmstart then only supplies metadata and the normalizer).')
    p.add_argument('--scratch-num-heads', type=int, default=1)
    args = p.parse_args()
    if args.epochs < 1 or args.batch_size < 1:
        p.error('epochs and batch size must be positive')
    if not 0 < args.validation_split < 1:
        p.error('validation split must be between zero and one')
    if not np.isfinite(args.engaged_weight) or args.engaged_weight <= 0:
        p.error('engaged weight must be finite and positive')
    if Path(args.output).resolve() == Path(args.warmstart).resolve():
        p.error('output must not overwrite the warm-start checkpoint')
    if args.validation_dataset and Path(args.validation_dataset).resolve() == Path(args.dataset).resolve():
        p.error('explicit validation must use a separate dataset')
    return args


def _load_dataset(path, obs_dim, action_dim, observation_frame='legacy'):
    with np.load(path, allow_pickle=False) as archive:
        recorded_frame = str(archive['observation_frame']) if 'observation_frame' in archive.files else 'legacy'
        keys = ('observations', 'actions', 'engaged', 'sample_weights', 'split',
                'episode_ids', 'episode_seeds', 'pair_ids')
        data = {key: archive[key] for key in keys if key in archive.files}
    if recorded_frame != observation_frame:
        raise ValueError(f'{path}: observation_frame {recorded_frame!r} differs from target {observation_frame!r}')
    for key, width in (('observations', obs_dim), ('actions', action_dim)):
        values = np.asarray(data[key], dtype=np.float32)
        if values.ndim != 2 or values.shape[1] != width or not np.isfinite(values).all():
            raise ValueError(f'{path}: invalid {key} shape or nonfinite values')
        data[key] = values
    size = len(data['observations'])
    if size == 0 or any(len(value) != size for value in data.values()):
        raise ValueError(f'{path}: empty or inconsistent sample counts')
    for key in data.keys() - {'observations', 'actions'}:
        if data[key].ndim != 1:
            raise ValueError(f'{path}: {key} must be a sample vector')
    weights = data.get('sample_weights', np.ones(size, dtype=np.float32))
    if not np.isfinite(weights).all() or np.any(weights <= 0):
        raise ValueError(f'{path}: sample weights must be finite and positive')
    return data


def _partition_dataset(data, validation, fraction, seed):
    if validation is None:
        if 'split' in data:
            if not np.isin(data['split'], ['train', 'validation']).all():
                raise ValueError('unknown dataset split label')
            train_indices = np.flatnonzero(data['split'] == 'train')
            val_indices = np.flatnonzero(data['split'] == 'validation')
        else:
            if any(key in data for key in ('episode_ids', 'episode_seeds', 'pair_ids')):
                raise ValueError('trajectory data requires explicit held-out data or split labels')
            size = len(data['observations'])
            permutation = np.random.default_rng(seed).permutation(size)
            val_size = min(size - 1, max(1, int(size * fraction)))
            val_indices, train_indices = permutation[:val_size], permutation[val_size:]
        validation = {key: value[val_indices] for key, value in data.items()}
        data = {key: value[train_indices] for key, value in data.items()}
    if not len(data['observations']) or not len(validation['observations']):
        raise ValueError('training and validation must both be nonempty')
    for dataset, label in ((data, 'train'), (validation, 'validation')):
        if 'split' in dataset and not np.all(dataset['split'] == label):
            raise ValueError(f'{label} input contains samples from another split')
    for key in ('episode_ids', 'episode_seeds', 'pair_ids'):
        if key in data and key in validation and np.intersect1d(data[key], validation[key]).size:
            raise ValueError(f'training/validation overlap in {key}')
    return data, validation


def _prepare_normalizer(obs_dim, train_observations, mode, checkpoint):
    normalizer = ObservationNormalizer(obs_dim)
    if mode == 'warmstart':
        state = checkpoint.get('obs_normalizer')
        if not state or np.asarray(state['mean']).shape != (obs_dim,):
            raise ValueError('warm-start normalizer missing or incompatible')
        normalizer.load_state_dict(copy.deepcopy(state))
    else:
        normalizer.update(train_observations)
    return normalizer


def _balance_recovery_weights(observations, weights):
    weights = np.asarray(weights, dtype=np.float32).copy()
    heading = np.degrees(np.arctan2(observations[:, 5], observations[:, 6]))
    cte = observations[:, 17]
    recovery = ((observations[:, 3] > .03) & (observations[:, 3] < .15)
                & (abs(heading) > 3) & (abs(heading) < 20) & (abs(cte) > .5)
                & (heading * cte > 0) & np.all(observations[:, 19:-3] == 0, axis=1))
    left, right = recovery & (cte < 0), recovery & (cte > 0)
    if not left.any() or not right.any():
        raise ValueError('recovery balancing requires both left and right training samples')
    total = float(weights[recovery].sum())
    for mask in (left, right):
        weights[mask] *= total / (2 * float(weights[mask].sum()))
    return weights


def _squash(action_mean, action_low, action_high):
    """Mirror the trainer's tanh squash into [action_low, action_high]."""
    action_mean = action_mean.clamp(-3.0, 3.0)
    half = (action_high - action_low) / 2.0
    mid = (action_high + action_low) / 2.0
    return torch.tanh(action_mean) * half + mid


def main():
    args = parse_args()
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = torch.device(args.device)

    ckpt = torch.load(args.warmstart, map_location=device, weights_only=False)
    for key in ('actor_state_dict', 'critic_state_dict', 'actor_log_std', 'action_dim',
                'action_low', 'action_high', 'local_observation_size'):
        if key not in ckpt:
            raise KeyError(f'warm-start checkpoint missing required key: {key}')

    if not bool(ckpt.get('neighbor_attention', False)):
        raise ValueError('This script only supports attention-actor checkpoints (neighbor_attention=True).')

    obs_dim = int(ckpt['local_observation_size'])
    action_dim = int(ckpt['action_dim'])
    hidden_sizes = tuple(ckpt.get('hidden_sizes', [256, 256]))
    max_neighbors = int(ckpt.get('max_neighbors', 4))
    embed_dim = int(ckpt.get('attention_embed_dim', 32))
    num_heads = int(ckpt.get('attention_num_heads', 1))
    if args.scratch_embed_dim > 0:
        embed_dim, num_heads = args.scratch_embed_dim, args.scratch_num_heads
        torch.manual_seed(args.seed)
    squash = bool(ckpt.get('squash_actions', False))
    action_mode = str(ckpt.get('action_mode', 'full'))

    # Rebuild the SAME actor architecture and seed it from the warm-start weights.
    actor = AttentionActor(
        max_neighbors=max_neighbors,
        encounter_dim=ENCOUNTER_TYPE_COUNT,
        hidden_sizes=hidden_sizes,
        action_dim=action_dim,
        embed_dim=embed_dim,
        num_heads=num_heads,
    ).to(device)
    if args.scratch_embed_dim <= 0:
        actor.load_state_dict(ckpt['actor_state_dict'])

    action_low = torch.as_tensor(ckpt['action_low'], dtype=torch.float32, device=device)
    action_high = torch.as_tensor(ckpt['action_high'], dtype=torch.float32, device=device)

    frame = args.observation_frame or str(ckpt.get('observation_frame', 'legacy'))
    data = _load_dataset(args.dataset, obs_dim, action_dim, frame)
    validation = _load_dataset(args.validation_dataset, obs_dim, action_dim, frame) if args.validation_dataset else None
    data, validation = _partition_dataset(data, validation, args.validation_split, args.seed)
    obs_normalizer = _prepare_normalizer(obs_dim, data['observations'], args.normalizer_mode, ckpt)
    train_obs = torch.as_tensor(obs_normalizer.normalize(data['observations']), dtype=torch.float32, device=device)
    train_act = torch.as_tensor(data['actions'], dtype=torch.float32, device=device)
    val_obs = torch.as_tensor(obs_normalizer.normalize(validation['observations']), dtype=torch.float32, device=device)
    val_act = torch.as_tensor(validation['actions'], dtype=torch.float32, device=device)
    w_np = np.asarray(data.get('sample_weights', np.ones(len(train_obs))), dtype=np.float32).copy()
    if 'engaged' in data:
        w_np[np.asarray(data['engaged']).astype(bool)] *= args.engaged_weight
    if args.balance_recovery_sides:
        w_np = _balance_recovery_weights(data['observations'], w_np)
    train_w = torch.as_tensor(w_np, dtype=torch.float32, device=device)
    print(f'Loaded train={len(train_obs)} validation={len(val_obs)} action_mode={action_mode} '
          f'squash={squash} normalizer={args.normalizer_mode}', flush=True)

    optimizer = torch.optim.Adam(actor.parameters(), lr=args.lr)
    actor_params = [p for p in actor.parameters() if p.requires_grad]

    def predict(obs_batch):
        out = actor(obs_batch)
        return _squash(out, action_low, action_high) if squash else out

    actor.eval()
    with torch.no_grad():
        baseline_val = float(torch.nn.functional.mse_loss(predict(val_obs), val_act).item())
    best_val = baseline_val
    best_state = {k: v.detach().clone() for k, v in actor.state_dict().items()}
    patience = 0
    bs = max(1, args.batch_size)
    n_train = train_obs.shape[0]
    dropout_slots = [int(s) for s in str(args.input_dropout_slots).split(',') if s.strip() != '']
    if dropout_slots:
        print(f'input dropout: slots={dropout_slots} prob={args.input_dropout_prob}', flush=True)
    for epoch in range(args.epochs):
        actor.train()
        ep = torch.randperm(n_train, device=device)
        total = 0.0
        for start in range(0, n_train, bs):
            bidx = ep[start:start + bs]
            obs_batch = train_obs[bidx]
            if dropout_slots:
                obs_batch = obs_batch.clone()
                drop = torch.rand(obs_batch.shape[0], len(dropout_slots), device=device) < args.input_dropout_prob
                for j, sl in enumerate(dropout_slots):
                    obs_batch[drop[:, j], sl] = 0.0  # normalized space: 0 = dataset mean (uninformative)
            pred = predict(obs_batch)
            wb = train_w[bidx]
            loss = (((pred - train_act[bidx]) ** 2).mean(dim=1) * wb).sum() / wb.sum()
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(actor_params, args.max_grad_norm)
            optimizer.step()
            total += float(loss.item()) * len(bidx)
        train_loss = total / max(1, n_train)

        actor.eval()
        with torch.no_grad():
            val_loss = float(torch.nn.functional.mse_loss(predict(val_obs), val_act).item())
        if val_loss < best_val - 1e-6:
            best_val = val_loss
            best_state = {k: v.detach().clone() for k, v in actor.state_dict().items()}
            patience = 0
        else:
            patience += 1
        if epoch % 10 == 0 or epoch == args.epochs - 1:
            print(f'epoch {epoch:3d}  train_mse={train_loss:.5f}  val_mse={val_loss:.5f}  best={best_val:.5f}', flush=True)
        if patience >= args.early_stopping_patience:
            print(f'early stop at epoch {epoch} (best val_mse={best_val:.5f})', flush=True)
            break

    actor.load_state_dict(best_state)

    # ---- Save full MAPPO-compatible checkpoint ----
    out_ckpt = copy.deepcopy(ckpt)
    out_ckpt['actor_state_dict'] = {k: v.detach().cpu() for k, v in actor.state_dict().items()}
    out_ckpt['attention_embed_dim'] = embed_dim
    out_ckpt['attention_num_heads'] = num_heads
    out_ckpt['obs_normalizer'] = obs_normalizer.state_dict()
    if args.observation_frame:
        out_ckpt['observation_frame'] = args.observation_frame
    out_ckpt['bc_pretrain_metadata'] = {
        'dataset': str(args.dataset),
        'warmstart': str(args.warmstart),
        'samples': int(n_train),
        'validation_dataset': str(args.validation_dataset) if args.validation_dataset else None,
        'validation_samples': int(len(val_obs)),
        'normalizer_mode': args.normalizer_mode,
        'balance_recovery_sides': bool(args.balance_recovery_sides),
        'baseline_val_mse': baseline_val,
        'best_val_mse': float(best_val),
        'epochs_ran': int(epoch + 1),
        'input_dropout_slots': dropout_slots,
        'input_dropout_prob': float(args.input_dropout_prob) if dropout_slots else 0.0,
    }
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(out_ckpt, out_path)
    print(f'Saved BC-initialised checkpoint to {out_path} (best val_mse={best_val:.5f})', flush=True)


if __name__ == '__main__':
    main()
