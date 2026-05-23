import argparse
from pathlib import Path

import torch


def parse_args():
    parser = argparse.ArgumentParser(description='Interpolate MAPPO actor weights between two checkpoints.')
    parser.add_argument('--base', required=True, help='Base checkpoint path. Metadata and non-actor tensors are copied from this checkpoint.')
    parser.add_argument('--target', required=True, help='Target checkpoint path that provides the interpolation endpoint.')
    parser.add_argument('--output', required=True, help='Output checkpoint path.')
    parser.add_argument('--alpha', type=float, required=True, help='Blend factor: 0 keeps base actor, 1 uses target actor.')
    parser.add_argument('--device', default='cpu')
    return parser.parse_args()


def main():
    args = parse_args()
    alpha = min(1.0, max(0.0, float(args.alpha)))
    base = torch.load(args.base, map_location=args.device, weights_only=False)
    target = torch.load(args.target, map_location=args.device, weights_only=False)
    base_actor = base.get('actor_state_dict')
    target_actor = target.get('actor_state_dict')
    if not isinstance(base_actor, dict) or not isinstance(target_actor, dict):
        raise RuntimeError('Both checkpoints must contain actor_state_dict.')
    if set(base_actor) != set(target_actor):
        missing = sorted(set(base_actor).symmetric_difference(set(target_actor)))[:8]
        raise RuntimeError(f'Actor state dict keys differ: {missing}')

    blended_actor = {}
    for key, base_value in base_actor.items():
        target_value = target_actor[key]
        if torch.is_floating_point(base_value):
            blended_actor[key] = (1.0 - alpha) * base_value + alpha * target_value.to(base_value.device)
        else:
            blended_actor[key] = base_value.clone()

    output = dict(base)
    output['actor_state_dict'] = blended_actor
    output['interpolated_actor'] = {
        'base': str(args.base),
        'target': str(args.target),
        'alpha': alpha,
    }
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(output, output_path)
    print(f'interpolated actor saved {output_path} alpha={alpha:.4f}')


if __name__ == '__main__':
    main()