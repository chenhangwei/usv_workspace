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
  * The student sees NORMALIZED observations, so we calibrate an
    ObservationNormalizer from the teacher dataset and bake its state into the
    output checkpoint (``--load-weights-from`` restores it).
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
    p.add_argument('--early-stopping-patience', type=int, default=20)
    p.add_argument('--max-grad-norm', type=float, default=0.5)
    p.add_argument('--device', default='cpu')
    p.add_argument('--seed', type=int, default=0)
    return p.parse_args()


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
    actor.load_state_dict(ckpt['actor_state_dict'])

    action_low = torch.as_tensor(ckpt['action_low'], dtype=torch.float32, device=device)
    action_high = torch.as_tensor(ckpt['action_high'], dtype=torch.float32, device=device)

    # ---- Load teacher dataset ----
    data = np.load(args.dataset, allow_pickle=True)
    obs_np = np.asarray(data['observations'], dtype=np.float32)
    act_np = np.asarray(data['actions'], dtype=np.float32)
    if obs_np.shape[1] != obs_dim:
        raise ValueError(f'dataset obs dim {obs_np.shape[1]} != checkpoint obs dim {obs_dim}')
    if act_np.shape[1] != action_dim:
        raise ValueError(f'dataset action dim {act_np.shape[1]} != checkpoint action dim {action_dim}')
    print(f'Loaded {obs_np.shape[0]} samples (obs={obs_np.shape[1]}, act={act_np.shape[1]}) '
          f'action_mode={action_mode} squash={squash}', flush=True)

    # ---- Calibrate observation normalizer from the dataset ----
    obs_normalizer = ObservationNormalizer(obs_dim)
    obs_normalizer.update(obs_np)
    norm_obs_np = obs_normalizer.normalize(obs_np)

    obs_t = torch.as_tensor(norm_obs_np, dtype=torch.float32, device=device)
    act_t = torch.as_tensor(act_np, dtype=torch.float32, device=device)

    n = obs_t.shape[0]
    perm = torch.randperm(n)
    n_val = max(1, int(n * args.validation_split))
    val_idx, train_idx = perm[:n_val], perm[n_val:]
    train_obs, train_act = obs_t[train_idx], act_t[train_idx]
    val_obs, val_act = obs_t[val_idx], act_t[val_idx]

    optimizer = torch.optim.Adam(actor.parameters(), lr=args.lr)
    actor_params = [p for p in actor.parameters() if p.requires_grad]

    def predict(obs_batch):
        out = actor(obs_batch)
        return _squash(out, action_low, action_high) if squash else out

    best_val = float('inf')
    best_state = {k: v.detach().clone() for k, v in actor.state_dict().items()}
    patience = 0
    bs = max(1, args.batch_size)
    n_train = train_obs.shape[0]
    for epoch in range(args.epochs):
        actor.train()
        ep = torch.randperm(n_train, device=device)
        total = 0.0
        for start in range(0, n_train, bs):
            bidx = ep[start:start + bs]
            pred = predict(train_obs[bidx])
            loss = torch.nn.functional.mse_loss(pred, train_act[bidx])
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
    out_ckpt['obs_normalizer'] = obs_normalizer.state_dict()
    out_ckpt['bc_pretrain_metadata'] = {
        'dataset': str(args.dataset),
        'warmstart': str(args.warmstart),
        'samples': int(n),
        'best_val_mse': float(best_val),
        'epochs_ran': int(epoch + 1),
    }
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(out_ckpt, out_path)
    print(f'Saved BC-initialised checkpoint to {out_path} (best val_mse={best_val:.5f})', flush=True)


if __name__ == '__main__':
    main()
