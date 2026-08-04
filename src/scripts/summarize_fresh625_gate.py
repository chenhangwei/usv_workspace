#!/usr/bin/env python3
"""Aggregate fresh625 offline gate results (logs/eval_fresh625_gate).

fresh625 arbiter (2026-07-08 user priorities): collisions are REPORTED ONLY,
not a hard fail. Hard checks: cluster reach 9/9, crossing success held,
no entangled pile-up. Route-first metrics (CTE / entanglement / omega
saturation / heading flips) reported for straightness comparison vs 622/624.
"""
import json
import os
import sys
from pathlib import Path

OUT = Path(os.environ.get("OUT", "/home/chenhangwei/usv_workspace/src/logs/eval_fresh625_gate"))
MODELS = os.environ.get("MODEL_NAMES", "fresh622 fresh624 fresh625").split()
SCENARIOS = os.environ.get(
    "SCENARIOS",
    "cluster_escape two_usv_head_on three_usv_crossing three_usv_overtaking",
).split()
SEEDS = [int(s) for s in os.environ.get("SEEDS", "5241 5242 5243").split()]


def summarize_episode(em):
    ts = em.get("trace_samples", [])
    fam = em.get("final_agent_metrics", {})
    reached = sum(1 for a in fam.values() if a.get("reached_goal"))
    mind = {}
    for sm in ts:
        for aid, a in sm["agents"].items():
            mind[aid] = min(mind.get(aid, 9e9), a["distance_to_goal"])
    return dict(
        steps=em["steps"],
        collision=em["collision"],
        success=em["success"],
        minsep=em["episode_min_separation"],
        goal=em["goal_completion_ratio"],
        reached=reached,
        n_agents=len(fam),
        min_dist_goal=min(mind.values()) if mind else 9e9,
    )


def main():
    rows = []
    straight = []  # (model, scen, seed, top-level route metrics)
    missing = []
    for model in MODELS:
        for scen in SCENARIOS:
            for seed in SEEDS:
                tag = f"{model}_{scen}_s{seed}"
                path = OUT / f"{tag}.json"
                if not path.exists():
                    missing.append(tag)
                    continue
                data = json.loads(path.read_text())
                em = data["episode_metrics"][0]
                rows.append((model, scen, seed, summarize_episode(em)))
                straight.append(
                    (
                        model,
                        scen,
                        seed,
                        dict(
                            cte=data.get("mean_cross_track_error"),
                            ent=data.get("mean_entanglement_ratio"),
                            osat=data.get("mean_omega_saturation_ratio"),
                            hflip=data.get("mean_heading_sign_flip_count"),
                            peff=data.get("mean_progress_efficiency"),
                        ),
                    )
                )

    total = len(MODELS) * len(SCENARIOS) * len(SEEDS)
    print(f"Loaded {len(rows)}/{total} episodes")
    if missing:
        print(f"Missing ({len(missing)}): {', '.join(missing[:8])}{'...' if len(missing) > 8 else ''}")

    print("\n=== Per-scenario collision / reach summary ===")
    print(f"{'model':<10} {'scenario':<22} {'coll':>4} {'succ':>4} {'reach':>6} {'<=0.5m':>6} {'minsep':>7}")
    for model in MODELS:
        for scen in SCENARIOS:
            rs = [r for r in rows if r[0] == model and r[1] == scen]
            if not rs:
                continue
            coll = sum(1 for r in rs if r[3]["collision"])
            succ = sum(1 for r in rs if r[3]["success"])
            reach = sum(r[3]["reached"] for r in rs)
            tight = sum(1 for r in rs if r[3]["min_dist_goal"] <= 0.5)
            minsep = min(r[3]["minsep"] for r in rs)
            print(
                f"{model:<10} {scen:<22} {coll}/{len(rs):>1} "
                f"{succ}/{len(rs):>1} {reach:>2}/{len(rs)*3:>2} "
                f"{tight:>2}/{len(rs):>1} {minsep:>7.2f}"
            )

    print("\n=== Route-first straightness metrics (mean over seeds) ===")
    print(f"{'model':<10} {'scenario':<22} {'CTE':>6} {'entangle':>9} {'omega_sat':>9} {'hdg_flip':>8} {'prog_eff':>8}")
    for model in MODELS:
        for scen in SCENARIOS:
            ss = [s[3] for s in straight if s[0] == model and s[1] == scen]
            if not ss:
                continue

            def avg(key):
                vals = [x[key] for x in ss if x[key] is not None]
                return sum(vals) / len(vals) if vals else float("nan")

            print(
                f"{model:<10} {scen:<22} {avg('cte'):>6.2f} {avg('ent'):>9.3f} "
                f"{avg('osat'):>9.3f} {avg('hflip'):>8.1f} {avg('peff'):>8.3f}"
            )

    print("\n=== Gate verdict (fresh625 arbiter: collision REPORT-ONLY) ===")
    for model in MODELS:
        cluster_reach = cluster_n = 0
        crossing_succ = crossing_n = 0
        encounter_coll = encounter_n = 0
        pileup = 0
        for scen in SCENARIOS:
            rs = [r for r in rows if r[0] == model and r[1] == scen]
            if scen == "cluster_escape":
                cluster_reach = sum(r[3]["reached"] for r in rs)
                cluster_n = len(rs) * 3
            else:
                encounter_coll += sum(1 for r in rs if r[3]["collision"])
                encounter_n += len(rs)
            if scen == "three_usv_crossing":
                crossing_succ = sum(1 for r in rs if r[3]["success"])
                crossing_n = len(rs)
            # entangled pile-up proxy: collision + very low minsep + nobody reached
            pileup += sum(
                1 for r in rs if r[3]["collision"] and r[3]["reached"] == 0 and r[3]["minsep"] < 0.4
            )
        ok = cluster_n > 0 and cluster_reach == cluster_n and pileup == 0
        print(
            f"  {model}: cluster_reach={cluster_reach}/{cluster_n}, "
            f"crossing_succ={crossing_succ}/{crossing_n}, "
            f"encounter_coll={encounter_coll}/{encounter_n} (report-only), "
            f"pileup={pileup} -> {'PASS' if ok else 'FAIL'}"
        )


if __name__ == "__main__":
    sys.exit(main())
