#!/usr/bin/env python3
"""Leg-wise SITL metrics for the pentagram route (fresh624/625 arbiter).

Per leg: detour ratio (path length / straight distance), net turn (sum of
unwrapped heading change), time; plus loop events (cumulative |turn| >= 360deg
within a sliding window), CTE p90 overall and in the 2-5m neighbor band.
Usage: python3 sitl_leg_metrics.py <nav_log_*.csv.gz> [...]
"""
import gzip
import math
import sys
import csv
from collections import defaultdict


def load(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", errors="ignore") as f:
        rows = [r for r in csv.DictReader(x for x in f if not x.startswith("#"))]
    return rows


def fnum(r, k, default=float("nan")):
    try:
        return float(r[k])
    except (KeyError, TypeError, ValueError):
        return default


def unwrap_deg(prev, cur):
    d = cur - prev
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def analyze(path, stride=10):
    rows = load(path)
    name = path.split("/")[-1].split("_2026")[0].replace("nav_log_", "")
    legs = defaultdict(list)
    for r in rows:
        gid = r.get("goal_id", "")
        if gid:
            legs[gid].append(r)

    print(f"\n===== {name} ({len(rows)} rows, stride={stride}) =====")
    print(f"{'goal':>8} {'time(s)':>8} {'straight':>9} {'path':>7} {'detour':>7} {'netturn°':>9} {'loops':>5} {'CTEp90':>7}")
    tot_loops = 0
    detours = []
    all_cte = []
    band_cte = []
    for gid in sorted(legs):
        rs = legs[gid]
        if len(rs) < 5:
            continue
        ds = rs[::stride] + ([rs[-1]] if (len(rs) - 1) % stride else [])
        xs = [fnum(r, "pose_x") for r in ds]
        ys = [fnum(r, "pose_y") for r in ds]
        ts = [fnum(r, "timestamp") for r in ds]
        yaws = [fnum(r, "pose_yaw_deg") for r in ds]
        path_len = sum(
            math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) for i in range(len(xs) - 1)
        )
        # straight distance: start position -> closest-to-goal position (leg ends at pass threshold)
        tx, ty = fnum(rs[0], "target_x"), fnum(rs[0], "target_y")
        straight = math.hypot(tx - xs[0], ty - ys[0]) - fnum(rs[-1], "distance_to_goal", 0.0)
        straight = max(straight, 1.0)
        detour = path_len / straight
        # net turn: sum of |unwrapped delta yaw|
        net = sum(abs(unwrap_deg(yaws[i], yaws[i + 1])) for i in range(len(yaws) - 1))
        # loop events: cumulative signed turn crossing +-360 (reset after each)
        loops = 0
        acc = 0.0
        for i in range(len(yaws) - 1):
            acc += unwrap_deg(yaws[i], yaws[i + 1])
            if abs(acc) >= 360.0:
                loops += 1
                acc = 0.0
        tot_loops += loops
        detours.append(detour)
        # CTE
        for r in rs:
            c = abs(fnum(r, "cross_track_error"))
            if not math.isnan(c):
                all_cte.append(c)
                nd = fnum(r, "orca_closest_distance")
                if 2.0 <= nd <= 5.0:
                    band_cte.append(c)
        dur = ts[-1] - ts[0]
        print(f"{gid:>8} {dur:>8.1f} {straight:>9.2f} {path_len:>7.2f} {detour:>7.2f} {net:>9.0f} {loops:>5} "
              f"{p90([abs(fnum(r,'cross_track_error')) for r in rs]):>7.2f}")
    print(f"  legs>1.3x: {sum(1 for d in detours if d > 1.3)}/{len(detours)}   "
          f">1.4x: {sum(1 for d in detours if d > 1.4)}/{len(detours)}   "
          f"loop_events: {tot_loops}   "
          f"CTE p90 all: {p90(all_cte):.2f}   CTE p90 (2-5m band): {p90(band_cte):.2f} (n={len(band_cte)})")
    return detours, tot_loops


def p90(vals):
    vals = sorted(v for v in vals if not math.isnan(v))
    if not vals:
        return float("nan")
    return vals[min(len(vals) - 1, int(0.9 * len(vals)))]


def main():
    all_det = []
    all_loops = 0
    for path in sys.argv[1:]:
        d, l = analyze(path)
        all_det.extend(d)
        all_loops += l
    print(f"\n=== FLEET: legs {len(all_det)}, detour>1.3x {sum(1 for d in all_det if d>1.3)}, "
          f">1.4x {sum(1 for d in all_det if d>1.4)}, loop_events {all_loops} ===")


if __name__ == "__main__":
    main()
