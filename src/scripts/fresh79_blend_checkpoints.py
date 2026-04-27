#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch


BLEND_PAYLOAD_KEYS = (
    'actor_state_dict',
    'critic_state_dict',
    'actor_log_std',
    'obs_normalizer',
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Blend compatible MAPPO checkpoints.')
    parser.add_argument(
        '--checkpoint',
        action='append',
        required=True,
        help='Checkpoint spec in the form /path/to/model.pt:weight',
    )
    parser.add_argument('--template', help='Optional metadata template checkpoint. Defaults to first checkpoint.')
    parser.add_argument('--output', required=True, help='Output blended checkpoint path.')
    parser.add_argument('--label', default='', help='Optional provenance label stored in the checkpoint.')
    return parser.parse_args()


def _parse_checkpoint_specs(specs: list[str]) -> list[tuple[Path, float]]:
    parsed: list[tuple[Path, float]] = []
    for spec in specs:
        if ':' not in spec:
            raise ValueError(f'Invalid checkpoint spec (missing ":"): {spec}')
        path_str, weight_str = spec.rsplit(':', 1)
        path = Path(path_str).expanduser().resolve()
        weight = float(weight_str)
        if weight < 0:
            raise ValueError(f'Checkpoint weight must be >= 0: {spec}')
        if not path.exists():
            raise FileNotFoundError(path)
        parsed.append((path, weight))
    total = sum(weight for _, weight in parsed)
    if total <= 0:
        raise ValueError('At least one checkpoint weight must be > 0.')
    return [(path, weight / total) for path, weight in parsed]


def _load_payload(path: Path) -> dict:
    payload = torch.load(path, map_location='cpu', weights_only=False)
    if not isinstance(payload, dict):
        raise ValueError(f'Checkpoint is not a dict payload: {path}')
    return payload


def _assert_compatible(reference: dict, candidate: dict, *, ref_path: Path, cand_path: Path) -> None:
    scalar_keys = (
        'action_dim',
        'local_observation_size',
        'global_state_size',
        'hidden_sizes',
        'max_agents',
        'max_neighbors',
        'neighbor_attention',
        'attention_embed_dim',
        'attention_num_heads',
        'normalize_observations',
        'agent_namespaces',
    )
    for key in scalar_keys:
        if reference.get(key) != candidate.get(key):
            raise ValueError(
                f'Incompatible checkpoint metadata for key {key}: {ref_path}={reference.get(key)!r}, '
                f'{cand_path}={candidate.get(key)!r}'
            )

    for state_key in ('actor_state_dict', 'critic_state_dict'):
        ref_sd = reference[state_key]
        cand_sd = candidate[state_key]
        if ref_sd.keys() != cand_sd.keys():
            raise ValueError(f'State dict keys differ for {state_key}: {ref_path} vs {cand_path}')
        for tensor_key in ref_sd:
            if tuple(ref_sd[tensor_key].shape) != tuple(cand_sd[tensor_key].shape):
                raise ValueError(
                    f'Shape mismatch in {state_key}[{tensor_key}]: '
                    f'{ref_sd[tensor_key].shape} vs {cand_sd[tensor_key].shape}'
                )

    if tuple(reference['actor_log_std'].shape) != tuple(candidate['actor_log_std'].shape):
        raise ValueError(f'actor_log_std shape mismatch: {ref_path} vs {cand_path}')

    ref_norm = reference.get('obs_normalizer')
    cand_norm = candidate.get('obs_normalizer')
    if (ref_norm is None) != (cand_norm is None):
        raise ValueError(f'obs_normalizer presence mismatch: {ref_path} vs {cand_path}')
    if ref_norm is not None:
        for key in ('mean', 'var'):
            if np.asarray(ref_norm[key]).shape != np.asarray(cand_norm[key]).shape:
                raise ValueError(f'obs_normalizer[{key}] shape mismatch: {ref_path} vs {cand_path}')


def _blend_values(values: list[object], weights: list[float]) -> object:
    first = values[0]
    if torch.is_tensor(first):
        acc = torch.zeros_like(first, dtype=torch.float32)
        for value, weight in zip(values, weights):
            acc = acc + value.to(dtype=torch.float32) * float(weight)
        return acc.to(dtype=first.dtype)
    if isinstance(first, np.ndarray):
        acc = np.zeros_like(first, dtype=np.float64)
        for value, weight in zip(values, weights):
            acc += np.asarray(value, dtype=np.float64) * float(weight)
        return acc.astype(first.dtype)
    if isinstance(first, dict):
        return {key: _blend_values([value[key] for value in values], weights) for key in first}
    if isinstance(first, (float, int, np.floating, np.integer)):
        return type(first)(sum(float(value) * float(weight) for value, weight in zip(values, weights)))
    raise TypeError(f'Unsupported value type for blending: {type(first).__name__}')


def main() -> None:
    args = parse_args()
    specs = _parse_checkpoint_specs(args.checkpoint)
    template_path = Path(args.template).expanduser().resolve() if args.template else specs[0][0]

    payloads = [(_load_payload(path), path, weight) for path, weight in specs]
    reference_payload, reference_path, _ = payloads[0]
    for payload, path, _ in payloads[1:]:
        _assert_compatible(reference_payload, payload, ref_path=reference_path, cand_path=path)

    template_payload = _load_payload(template_path)
    _assert_compatible(reference_payload, template_payload, ref_path=reference_path, cand_path=template_path)

    output_payload = dict(template_payload)
    weights = [weight for _, _, weight in payloads]
    for key in BLEND_PAYLOAD_KEYS:
        values = [payload[key] for payload, _, _ in payloads]
        output_payload[key] = _blend_values(values, weights)

    output_payload.pop('optimizer_state_dict', None)
    output_payload.pop('grad_scaler_state_dict', None)
    output_payload['completed_timesteps'] = 0
    output_payload['update_index'] = 0
    output_payload['next_checkpoint_step'] = None
    output_payload['blend_sources'] = [
        {'path': str(path), 'weight': float(weight)} for _, path, weight in payloads
    ]
    if args.label:
        output_payload['blend_label'] = args.label

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(output_payload, output_path)
    print(json.dumps({'output': str(output_path), 'sources': output_payload['blend_sources']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
