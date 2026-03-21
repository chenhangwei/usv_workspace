import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate multiple MAPPO checkpoints and select the best dense candidate.')
    parser.add_argument('--checkpoint-dir', required=True, help='Directory containing periodic MAPPO checkpoints.')
    parser.add_argument('--episodes', type=int, default=15, help='Evaluation episodes per checkpoint. Default to a larger sample because 3-episode rankings are too noisy for close dense checkpoints.')
    parser.add_argument('--steps-per-episode', type=int, default=90, help='Maximum steps per evaluation episode.')
    parser.add_argument('--device', default='cpu', help='Torch device.')
    parser.add_argument('--scenario', action='append', dest='scenarios', required=True, help='Scenario name. Repeatable.')
    parser.add_argument('--episode-timeout', type=float, help='Optional override for environment episode timeout.')
    parser.add_argument('--no-progress-timeout', type=float, help='Optional override for no-progress timeout.')
    parser.add_argument('--summary-json', help='Optional JSON output path for the full ranking summary.')
    parser.add_argument('--per-checkpoint-json-dir', help='Optional directory where each checkpoint summary is written.')
    return parser.parse_args()


def _sorted_checkpoints(checkpoint_dir: Path) -> list[Path]:
    return sorted(path for path in checkpoint_dir.glob('*.pt') if path.is_file())


def _ranking_record(summary: dict) -> dict:
    scenario_summaries = summary.get('scenario_summaries', {})
    crossing = scenario_summaries.get('five_usv_dense_crossing', {})
    progress_values = [
        float(item.get('mean_team_goal_distance_delta', 0.0))
        for item in scenario_summaries.values()
    ]
    balanced_progress = min(progress_values) if progress_values else float('-inf')
    return {
        'model': summary['model'],
        'collision_rate': float(summary['collision_rate']),
        'timeout_rate': float(summary['timeout_rate']),
        'mean_team_goal_distance_delta': float(summary['mean_team_goal_distance_delta']),
        'mean_team_goal_progress_ratio': float(summary['mean_team_goal_progress_ratio']),
        'crossing_team_goal_distance_delta': float(crossing.get('mean_team_goal_distance_delta', float('-inf'))),
        'crossing_team_goal_progress_ratio': float(crossing.get('mean_team_goal_progress_ratio', float('-inf'))),
        'balanced_team_goal_distance_delta': balanced_progress,
    }


def _best_by(records: list[dict], key: str) -> dict | None:
    if not records:
        return None
    return max(records, key=lambda item: (item[key], -item['collision_rate'], -item['timeout_rate']))


def main():
    args = parse_args()
    checkpoint_dir = Path(args.checkpoint_dir)
    checkpoints = _sorted_checkpoints(checkpoint_dir)
    if not checkpoints:
        raise FileNotFoundError(f'No checkpoints found in {checkpoint_dir}')

    ranking = []
    per_checkpoint_json_dir = Path(args.per_checkpoint_json_dir) if args.per_checkpoint_json_dir else None
    if per_checkpoint_json_dir is not None:
        per_checkpoint_json_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix='mappo_checkpoint_eval_') as temporary_dir:
        temporary_dir_path = Path(temporary_dir)
        for checkpoint_path in checkpoints:
            output_path = (
                per_checkpoint_json_dir / f'{checkpoint_path.stem}.json'
                if per_checkpoint_json_dir is not None
                else temporary_dir_path / f'{checkpoint_path.stem}.json'
            )
            command = [
                sys.executable,
                '-m',
                'usv_rl.evaluate_mappo_policy',
                '--model',
                str(checkpoint_path),
                '--episodes',
                str(args.episodes),
                '--steps-per-episode',
                str(args.steps_per_episode),
                '--device',
                args.device,
                '--output-json',
                str(output_path),
            ]
            for scenario in args.scenarios:
                command.extend(['--scenario', scenario])
            if args.episode_timeout is not None:
                command.extend(['--episode-timeout', str(args.episode_timeout)])
            if args.no_progress_timeout is not None:
                command.extend(['--no-progress-timeout', str(args.no_progress_timeout)])

            subprocess.run(command, check=True)
            summary = json.loads(output_path.read_text(encoding='utf-8'))
            ranking.append(_ranking_record(summary))

    ranking.sort(key=lambda item: item['model'])
    result = {
        'checkpoint_dir': str(checkpoint_dir),
        'scenarios': list(args.scenarios),
        'episodes': args.episodes,
        'steps_per_episode': args.steps_per_episode,
        'evaluated_checkpoints': ranking,
        'best_overall': _best_by(ranking, 'mean_team_goal_distance_delta'),
        'best_crossing': _best_by(ranking, 'crossing_team_goal_distance_delta'),
        'best_balanced': _best_by(ranking, 'balanced_team_goal_distance_delta'),
    }

    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.summary_json:
        summary_path = Path(args.summary_json)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Saved MAPPO checkpoint ranking to {summary_path}')


if __name__ == '__main__':
    main()