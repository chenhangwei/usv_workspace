#!/usr/bin/env python3
"""Rank checkpoint evaluation summaries with a gate-aligned heuristic."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

SOLO_SCENARIO = "solo_navigation"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--eval-json-dir", required=True, help="Directory containing evaluation summary JSON files.")
    parser.add_argument("--top-k", type=int, default=3, help="Number of candidates to emit.")
    parser.add_argument(
        "--format",
        choices=("json", "paths", "table"),
        default="json",
        help="Output format.",
    )
    parser.add_argument(
        "--existing-only",
        action="store_true",
        help="Skip summaries whose model path no longer exists.",
    )
    return parser.parse_args()


def _max0(value: float) -> float:
    return value if value > 0.0 else 0.0


def gate_pass(scenario: str, summary: dict) -> bool:
    collision = float(summary.get("collision_rate", 1.0))
    progress = float(summary.get("mean_team_goal_progress_ratio", 0.0))
    if scenario == SOLO_SCENARIO:
        heading = float(summary.get("mean_heading_error", math.inf))
        cte = float(summary.get("mean_cross_track_error", math.inf))
        return collision <= 0.0 and progress >= 0.40 and heading <= 0.50 and cte <= 1.0

    separation = float(summary.get("worst_episode_min_separation", 0.0))
    omega_flip = float(summary.get("mean_omega_flip_count", math.inf))
    return collision <= 0.0 and progress >= 0.40 and separation >= 1.5 and omega_flip <= 5.0


def scenario_score(scenario: str, summary: dict) -> tuple[float, dict]:
    progress = float(summary.get("mean_team_goal_progress_ratio", 0.0))
    collision = float(summary.get("collision_rate", 1.0))
    separation = float(summary.get("worst_episode_min_separation", 0.0))
    omega_flip = float(summary.get("mean_omega_flip_count", 999.0))
    heading = float(summary.get("mean_heading_error", 0.0))
    cte = float(summary.get("mean_cross_track_error", 0.0))
    colregs = float(summary.get("mean_colregs_compliance_ratio", 0.0))
    entanglement = float(summary.get("mean_entanglement_ratio", 0.0))

    score = 0.0
    if scenario == SOLO_SCENARIO:
        score += 260.0 * progress
        score -= 900.0 * collision
        score -= 180.0 * _max0(heading - 0.50)
        score -= 140.0 * _max0(cte - 1.0)
        if collision <= 0.0:
            score += 20.0
        if progress >= 0.40:
            score += 25.0
        if heading <= 0.50:
            score += 15.0
        if cte <= 1.0:
            score += 15.0
    else:
        score += 220.0 * progress
        score -= 1100.0 * collision
        score -= 150.0 * _max0(1.5 - separation)
        score -= 8.0 * omega_flip
        score += 35.0 * max(0.0, min(colregs, 1.0))
        score -= 60.0 * max(0.0, entanglement)
        if collision <= 0.0:
            score += 25.0
        if progress >= 0.40:
            score += 20.0
        if separation >= 1.5:
            score += 20.0
        if omega_flip <= 5.0:
            score += 15.0

        if scenario == "two_usv_head_on":
            score -= 24.0 * _max0(separation - 4.0)
            score -= 120.0 * _max0(heading - 0.70)
        elif scenario in {"three_usv_crossing", "three_usv_random_encounter"}:
            score -= 25.0 * max(0.0, entanglement)

    metrics = {
        "progress": progress,
        "collision_rate": collision,
        "worst_min_sep": separation,
        "omega_flip": omega_flip,
        "heading_error": heading,
        "cross_track_error": cte,
        "colregs": colregs,
        "entanglement_ratio": entanglement,
        "passed_gate": gate_pass(scenario, summary),
    }
    return score, metrics


def extract_scenario_summaries(payload: dict) -> dict[str, dict]:
    scenario_summaries = payload.get("scenario_summaries")
    if isinstance(scenario_summaries, dict) and scenario_summaries:
        return scenario_summaries

    scenarios = payload.get("scenarios")
    if isinstance(scenarios, list) and len(scenarios) == 1:
        return {str(scenarios[0]): payload}
    return {}


def load_record(summary_path: Path) -> dict | None:
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    scenario_summaries = extract_scenario_summaries(payload)
    if not scenario_summaries:
        return None

    model = str(payload.get("model", "")).strip()
    if not model:
        return None

    total_score = 0.0
    pass_count = 0
    breakdown: dict[str, dict] = {}
    for scenario, scenario_payload in scenario_summaries.items():
        score, metrics = scenario_score(scenario, scenario_payload)
        breakdown[scenario] = {"score": score, **metrics}
        total_score += score
        if metrics["passed_gate"]:
            pass_count += 1

    total_score += 12.0 * pass_count
    return {
        "model": model,
        "source_json": str(summary_path),
        "scenario_count": len(scenario_summaries),
        "pass_count": pass_count,
        "score": total_score,
        "collision_rate": float(payload.get("collision_rate", 1.0)),
        "mean_progress": float(payload.get("mean_team_goal_progress_ratio", 0.0)),
        "scenario_breakdown": breakdown,
    }


def better_record(candidate: dict, incumbent: dict) -> bool:
    candidate_key = (
        candidate["pass_count"],
        candidate["score"],
        -candidate["collision_rate"],
        candidate["mean_progress"],
    )
    incumbent_key = (
        incumbent["pass_count"],
        incumbent["score"],
        -incumbent["collision_rate"],
        incumbent["mean_progress"],
    )
    return candidate_key > incumbent_key


def load_records(eval_dir: Path, existing_only: bool) -> list[dict]:
    best_by_model: dict[str, dict] = {}
    for summary_path in sorted(eval_dir.glob("*.json")):
        record = load_record(summary_path)
        if record is None:
            continue
        model_path = Path(record["model"])
        if existing_only and not model_path.is_file():
            continue
        key = str(model_path)
        incumbent = best_by_model.get(key)
        if incumbent is None or better_record(record, incumbent):
            best_by_model[key] = record

    records = list(best_by_model.values())
    records.sort(
        key=lambda item: (
            item["pass_count"],
            item["score"],
            -item["collision_rate"],
            item["mean_progress"],
        ),
        reverse=True,
    )
    return records


def render_table(records: list[dict]) -> str:
    lines = []
    for record in records:
        lines.append(
            f"pass={record['pass_count']}/{record['scenario_count']} "
            f"score={record['score']:.2f} "
            f"coll={record['collision_rate']:.3f} "
            f"progress={record['mean_progress']:.3f} "
            f"model={record['model']}"
        )
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    eval_dir = Path(args.eval_json_dir)
    if not eval_dir.is_dir():
        raise SystemExit(f"Evaluation JSON directory not found: {eval_dir}")

    records = load_records(eval_dir, existing_only=bool(args.existing_only))
    records = records[: max(0, int(args.top_k))]

    if args.format == "paths":
        for record in records:
            print(record["model"])
        return 0

    if args.format == "table":
        if records:
            print(render_table(records))
        return 0

    json.dump({"candidates": records}, sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
