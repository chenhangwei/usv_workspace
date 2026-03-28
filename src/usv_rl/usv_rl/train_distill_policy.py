"""Offline teacher-student distillation (BC + optional DAgger).

Usage::

    # Phase 2 — pure BC from collected trajectories
    python3 -m usv_rl.train_distill_policy \
        --teacher /path/to/a10_step_358400.pt \
        --trajectories /mnt/data/distillation/teacher_trajectories.npz \
        --output /mnt/data/distillation/student_bc_v1.pt \
        --epochs 100 --lr 1e-3

    # Phase 3 — DAgger from an existing student
    python3 -m usv_rl.train_distill_policy \
        --teacher /path/to/a10_step_358400.pt \
        --student /mnt/data/distillation/student_bc_v1.pt \
        --dagger-rounds 3 --dagger-episodes 100 \
        --output /mnt/data/distillation/student_dagger_v1.pt
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Train a student policy via distillation from a frozen MAPPO teacher.')
    parser.add_argument('--teacher', required=True, help='Teacher checkpoint (.pt).')
    parser.add_argument('--trajectories', help='Pre-collected trajectory dataset (.npz). Required for pure BC mode.')
    parser.add_argument('--student', help='Optional student checkpoint to resume from (.pt).')
    parser.add_argument('--output', required=True, help='Output student model path (.pt).')
    parser.add_argument('--epochs', type=int, default=100, help='Training epochs for BC phase.')
    parser.add_argument('--lr', type=float, default=1e-3, help='Learning rate.')
    parser.add_argument('--batch-size', type=int, default=512, help='Mini-batch size.')
    parser.add_argument('--validation-split', type=float, default=0.1, help='Fraction of data reserved for validation.')
    parser.add_argument('--early-stopping-patience', type=int, default=10, help='Stop training if val loss does not improve for this many epochs.')
    parser.add_argument('--clip-actions', action='store_true', help='Clip target actions to checkpoint action_bounds before training (avoids bang-bang behavior).')
    parser.add_argument('--device', default='cpu', help='Torch device.')
    parser.add_argument('--dagger-rounds', type=int, default=0, help='Number of DAgger rounds. 0 = pure BC only.')
    parser.add_argument('--dagger-episodes', type=int, default=100, help='New episodes collected per DAgger round.')
    parser.add_argument('--dagger-steps', type=int, default=180, help='Steps per DAgger episode.')
    parser.add_argument('--dagger-epochs', type=int, default=50, help='Training epochs per DAgger round.')
    parser.add_argument('--scenario', action='append', dest='scenarios', default=None, help='Scenario name. Repeatable.')
    parser.add_argument('--summary-json', help='Optional JSON path for training summary.')
    return parser.parse_args(argv)


def _build_mlp(nn, input_dim: int, hidden_sizes: tuple[int, ...], output_dim: int):
    layers = []
    current_dim = input_dim
    for hidden_size in hidden_sizes:
        layers.append(nn.Linear(current_dim, hidden_size))
        layers.append(nn.Tanh())
        current_dim = hidden_size
    layers.append(nn.Linear(current_dim, output_dim))
    return nn.Sequential(*layers)


def _train_bc(
    student,
    train_obs,
    train_actions,
    val_obs,
    val_actions,
    *,
    epochs: int,
    batch_size: int,
    lr: float,
    patience: int,
    device,
    torch,
) -> dict:
    """Train student via MSE behavior cloning. Returns training log."""
    optimizer = torch.optim.Adam(student.parameters(), lr=lr)
    best_val_loss = float('inf')
    best_state = None
    patience_counter = 0
    log = {'train_losses': [], 'val_losses': [], 'best_epoch': 0}

    n_train = train_obs.shape[0]

    for epoch in range(epochs):
        student.train()
        perm = torch.randperm(n_train, device=device)
        epoch_loss = 0.0
        n_batches = 0

        for start in range(0, n_train, batch_size):
            idx = perm[start:start + batch_size]
            obs_batch = train_obs[idx]
            act_batch = train_actions[idx]

            pred = student(obs_batch)
            loss = torch.nn.functional.mse_loss(pred, act_batch)

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(student.parameters(), 1.0)
            optimizer.step()

            epoch_loss += loss.item()
            n_batches += 1

        avg_train_loss = epoch_loss / max(1, n_batches)
        log['train_losses'].append(avg_train_loss)

        # Validation
        student.eval()
        with torch.no_grad():
            val_pred = student(val_obs)
            val_loss = torch.nn.functional.mse_loss(val_pred, val_actions).item()
        log['val_losses'].append(val_loss)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_state = {k: v.clone() for k, v in student.state_dict().items()}
            patience_counter = 0
            log['best_epoch'] = epoch
        else:
            patience_counter += 1

        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(
                f'  epoch {epoch + 1}/{epochs}: '
                f'train_loss={avg_train_loss:.6f}, val_loss={val_loss:.6f}, '
                f'best_val={best_val_loss:.6f}',
                flush=True,
            )

        if patience_counter >= patience:
            print(f'  Early stopping at epoch {epoch + 1} (patience={patience}).', flush=True)
            break

    if best_state is not None:
        student.load_state_dict(best_state)
    log['best_val_loss'] = best_val_loss
    return log


def _save_student_checkpoint(
    student,
    teacher_checkpoint: dict,
    output_path: str,
    *,
    distillation_metadata: dict,
):
    """Save student in MAPPO-compatible checkpoint format."""
    import torch

    checkpoint = {
        'actor_state_dict': student.state_dict(),
        'hidden_sizes': list(teacher_checkpoint.get('hidden_sizes', [256, 256])),
        'local_observation_size': int(teacher_checkpoint['local_observation_size']),
        'action_dim': int(teacher_checkpoint['action_dim']),
        'action_mode': str(teacher_checkpoint.get('action_mode', 'full')),
        'action_bounds': dict(teacher_checkpoint.get('action_bounds', {'linear_delta': 0.7, 'angular_delta': 0.6})),
        'agent_namespaces': list(teacher_checkpoint['agent_namespaces']),
        'max_neighbors': int(teacher_checkpoint.get('max_neighbors', 4)),
        'max_agents': int(teacher_checkpoint.get('max_agents', 5)),
        'cruise_speed': float(teacher_checkpoint.get('cruise_speed', 0.5)),
        'max_angular_velocity': float(teacher_checkpoint.get('max_angular_velocity', 0.5)),
        'episode_timeout': float(teacher_checkpoint.get('episode_timeout', 45.0)),
        'no_progress_timeout': float(teacher_checkpoint.get('no_progress_timeout', 10.0)),
        'min_progress_delta': float(teacher_checkpoint.get('min_progress_delta', 0.3)),
        'collision_distance': float(teacher_checkpoint.get('collision_distance', 0.5)),
        'near_miss_distance': float(teacher_checkpoint.get('near_miss_distance', 1.5)),
        'scenario_neighbor_speed': float(teacher_checkpoint.get('scenario_neighbor_speed', 0.45)),
        'distillation_metadata': distillation_metadata,
    }
    if 'reward_config' in teacher_checkpoint:
        checkpoint['reward_config'] = dict(teacher_checkpoint['reward_config'])
    if 'scenarios' in teacher_checkpoint:
        checkpoint['scenarios'] = list(teacher_checkpoint['scenarios'])
    for key in (
        'goal_proximity_reward_weight', 'goal_proximity_relief_distance',
        'goal_proximity_heading_relief', 'goal_proximity_smoothness_relief',
        'goal_proximity_conflict_relief', 'goal_proximity_speed_relief',
        'team_reward_weight', 'team_progress_weight',
        'team_goal_proximity_weight', 'team_regression_penalty_weight',
        'team_dispersion_penalty_weight', 'team_dispersion_margin',
        'coordination_reward_weight', 'team_completion_bonus', 'deadlock_penalty_weight',
    ):
        if key in teacher_checkpoint:
            checkpoint[key] = teacher_checkpoint[key]

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, output_path)
    print(f'Student checkpoint saved to {output_path}', flush=True)


def _collect_dagger_data(
    student,
    teacher_policy,
    env,
    env_kwargs: dict,
    *,
    episodes: int,
    steps_per_episode: int,
    scenarios: tuple[str, ...],
    torch,
    device,
    clip_bounds: tuple[float, float] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Run student in env, label with teacher actions."""
    from .evaluate_mappo_policy import _create_env

    all_obs = []
    all_teacher_actions = []

    for episode in range(episodes):
        scenario_name = scenarios[episode % len(scenarios)]
        try:
            observations, info = env.reset(options={'scenario_kind': scenario_name})
        except RuntimeError:
            env.close()
            env = _create_env(env_kwargs)
            observations, info = env.reset(options={'scenario_kind': scenario_name})

        for step in range(steps_per_episode):
            exec_actions = {}
            for agent_id in env.agent_ids:
                obs = observations[agent_id]
                # Student controls execution
                obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
                with torch.no_grad():
                    student_action = student(obs_tensor).cpu().numpy()[0]
                # Teacher provides labels
                teacher_action = teacher_policy.predict(obs)

                exec_actions[agent_id] = student_action
                all_obs.append(obs.copy())
                all_teacher_actions.append(teacher_action.copy())

            observations, _, terminated_dict, truncated_dict, info = env.step(exec_actions)
            if bool(terminated_dict['__all__']) or bool(truncated_dict['__all__']):
                break

        if (episode + 1) % 20 == 0:
            print(f'  DAgger collect [{episode + 1}/{episodes}] samples={len(all_obs)}', flush=True)

    obs_arr = np.array(all_obs, dtype=np.float32)
    act_arr = np.array(all_teacher_actions, dtype=np.float32)
    if clip_bounds is not None:
        lin_b, ang_b = clip_bounds
        act_arr[:, 0] = np.clip(act_arr[:, 0], 0.0, lin_b)
        act_arr[:, 1] = np.clip(act_arr[:, 1], -ang_b, ang_b)
    return obs_arr, act_arr


def main():
    args = parse_args()

    import torch
    from torch import nn

    device = torch.device(args.device)

    # Load teacher
    teacher_checkpoint = torch.load(args.teacher, map_location=device, weights_only=False)
    hidden_sizes = tuple(int(v) for v in teacher_checkpoint.get('hidden_sizes', [256, 256]))
    obs_dim = int(teacher_checkpoint['local_observation_size'])
    action_dim = int(teacher_checkpoint['action_dim'])

    print(f'Teacher: obs_dim={obs_dim}, action_dim={action_dim}, hidden={hidden_sizes}', flush=True)

    # Build teacher policy for DAgger (reuse eval code)
    from .evaluate_mappo_policy import MappoActorPolicy
    teacher_policy = MappoActorPolicy(teacher_checkpoint, device=str(device))

    # Build student (same architecture as teacher)
    student = _build_mlp(nn, obs_dim, hidden_sizes, action_dim).to(device)

    # If resuming from existing student
    if args.student:
        student_ckpt = torch.load(args.student, map_location=device, weights_only=False)
        student.load_state_dict(student_ckpt['actor_state_dict'])
        print(f'Resumed student from {args.student}', flush=True)

    # Phase 2: BC from collected trajectories
    if args.trajectories:
        print(f'Loading trajectories from {args.trajectories}...', flush=True)
        data = np.load(args.trajectories)
        obs_all = data['observations']
        actions_all = data['actions']
        print(f'Dataset: {obs_all.shape[0]} samples, obs_dim={obs_all.shape[1]}, action_dim={actions_all.shape[1]}', flush=True)

        if args.clip_actions:
            action_bounds = teacher_checkpoint.get('action_bounds', {'linear_delta': 0.3, 'angular_delta': 0.4})
            lin_bound = max(float(action_bounds.get('linear_delta', 0.3)), float(teacher_checkpoint.get('cruise_speed', 0.0)))
            ang_bound = max(float(action_bounds.get('angular_delta', 0.4)), float(teacher_checkpoint.get('max_angular_velocity', 0.0)))
            print(f'Clipping actions to linear=[0, {lin_bound}], angular=[{-ang_bound}, {ang_bound}]', flush=True)
            actions_all[:, 0] = np.clip(actions_all[:, 0], 0.0, lin_bound)
            actions_all[:, 1] = np.clip(actions_all[:, 1], -ang_bound, ang_bound)
            raw_sat = np.mean(np.abs(data['actions'][:, 1]) >= ang_bound * 0.95)
            print(f'  (original angular saturation rate: {raw_sat*100:.1f}%)', flush=True)

        # Train/val split
        n = obs_all.shape[0]
        n_val = max(1, int(n * args.validation_split))
        indices = np.random.permutation(n)
        val_idx = indices[:n_val]
        train_idx = indices[n_val:]

        train_obs = torch.as_tensor(obs_all[train_idx], dtype=torch.float32, device=device)
        train_actions = torch.as_tensor(actions_all[train_idx], dtype=torch.float32, device=device)
        val_obs = torch.as_tensor(obs_all[val_idx], dtype=torch.float32, device=device)
        val_actions = torch.as_tensor(actions_all[val_idx], dtype=torch.float32, device=device)

        print(f'BC training: {len(train_idx)} train, {len(val_idx)} val, epochs={args.epochs}', flush=True)
        t0 = time.monotonic()
        bc_log = _train_bc(
            student, train_obs, train_actions, val_obs, val_actions,
            epochs=args.epochs,
            batch_size=args.batch_size,
            lr=args.lr,
            patience=args.early_stopping_patience,
            device=device,
            torch=torch,
        )
        bc_elapsed = time.monotonic() - t0
        print(f'BC training done in {bc_elapsed:.0f}s, best_val_loss={bc_log["best_val_loss"]:.6f}', flush=True)
    else:
        bc_log = None
        bc_elapsed = 0.0
        if not args.student:
            print('Warning: No --trajectories and no --student provided. Student is randomly initialized.', flush=True)

    # Phase 3: DAgger
    dagger_logs = []
    if args.dagger_rounds > 0:
        from .evaluate_mappo_policy import _create_env
        from .collect_teacher_trajectories import _build_env_kwargs_from_checkpoint
        from .multi_agent_scenarios import MultiAgentScenarioFactory

        scenarios = (
            tuple(args.scenarios) if args.scenarios
            else tuple(teacher_checkpoint.get('scenarios', MultiAgentScenarioFactory.cluster_available()))
        )
        env_kwargs = _build_env_kwargs_from_checkpoint(teacher_checkpoint, scenarios)
        env = _create_env(env_kwargs)

        # Compute clip bounds if requested
        dagger_clip_bounds = None
        if args.clip_actions:
            _ab = teacher_checkpoint.get('action_bounds', {'linear_delta': 0.3, 'angular_delta': 0.4})
            _lb = max(float(_ab.get('linear_delta', 0.3)), float(teacher_checkpoint.get('cruise_speed', 0.0)))
            _ab2 = max(float(_ab.get('angular_delta', 0.4)), float(teacher_checkpoint.get('max_angular_velocity', 0.0)))
            dagger_clip_bounds = (_lb, _ab2)

        # Accumulate all data across rounds
        if args.trajectories:
            data = np.load(args.trajectories)
            cumulative_obs = list(data['observations'])
            cumulative_actions = list(data['actions'])
        else:
            cumulative_obs = []
            cumulative_actions = []

        try:
            for round_idx in range(args.dagger_rounds):
                print(f'\n=== DAgger round {round_idx + 1}/{args.dagger_rounds} ===', flush=True)

                # Collect new data with student execution + teacher labels
                new_obs, new_actions = _collect_dagger_data(
                    student, teacher_policy, env, env_kwargs,
                    episodes=args.dagger_episodes,
                    steps_per_episode=args.dagger_steps,
                    scenarios=scenarios,
                    torch=torch,
                    device=device,
                    clip_bounds=dagger_clip_bounds,
                )
                print(f'  Collected {new_obs.shape[0]} new samples', flush=True)

                cumulative_obs.extend(new_obs)
                cumulative_actions.extend(new_actions)

                # Retrain on full accumulated dataset
                all_obs_np = np.array(cumulative_obs, dtype=np.float32)
                all_act_np = np.array(cumulative_actions, dtype=np.float32)
                n = all_obs_np.shape[0]
                n_val = max(1, int(n * args.validation_split))
                perm = np.random.permutation(n)

                train_obs = torch.as_tensor(all_obs_np[perm[n_val:]], dtype=torch.float32, device=device)
                train_actions = torch.as_tensor(all_act_np[perm[n_val:]], dtype=torch.float32, device=device)
                val_obs = torch.as_tensor(all_obs_np[perm[:n_val]], dtype=torch.float32, device=device)
                val_actions = torch.as_tensor(all_act_np[perm[:n_val]], dtype=torch.float32, device=device)

                print(f'  Training on {n - n_val} samples ({n} total accumulated)', flush=True)
                t_round = time.monotonic()
                round_log = _train_bc(
                    student, train_obs, train_actions, val_obs, val_actions,
                    epochs=args.dagger_epochs,
                    batch_size=args.batch_size,
                    lr=args.lr,
                    patience=args.early_stopping_patience,
                    device=device,
                    torch=torch,
                )
                round_elapsed = time.monotonic() - t_round
                print(
                    f'  DAgger round {round_idx + 1} done in {round_elapsed:.0f}s, '
                    f'best_val_loss={round_log["best_val_loss"]:.6f}',
                    flush=True,
                )
                dagger_logs.append({
                    'round': round_idx + 1,
                    'new_samples': int(new_obs.shape[0]),
                    'total_samples': n,
                    'best_val_loss': round_log['best_val_loss'],
                    'elapsed_seconds': round(round_elapsed, 1),
                })
        finally:
            env.close()

    # Save student
    distillation_metadata = {
        'teacher_checkpoint': args.teacher,
        'phase': 'dagger' if args.dagger_rounds > 0 else 'bc',
        'dagger_rounds': args.dagger_rounds,
    }
    if bc_log is not None:
        distillation_metadata['bc_best_val_loss'] = bc_log['best_val_loss']
        distillation_metadata['bc_best_epoch'] = bc_log['best_epoch']
    if dagger_logs:
        distillation_metadata['dagger_logs'] = dagger_logs

    _save_student_checkpoint(student, teacher_checkpoint, args.output, distillation_metadata=distillation_metadata)

    # Summary
    summary = {
        'teacher': args.teacher,
        'output': args.output,
        'obs_dim': obs_dim,
        'action_dim': action_dim,
        'hidden_sizes': list(hidden_sizes),
        'bc_epochs': args.epochs if args.trajectories else 0,
        'bc_best_val_loss': bc_log['best_val_loss'] if bc_log else None,
        'dagger_rounds': args.dagger_rounds,
        'dagger_logs': dagger_logs,
    }
    print(f'\nDistillation complete → {args.output}', flush=True)

    if args.summary_json:
        Path(args.summary_json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.summary_json).write_text(
            json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8'
        )
        print(f'Summary saved to {args.summary_json}')

    return summary


if __name__ == '__main__':
    main()
