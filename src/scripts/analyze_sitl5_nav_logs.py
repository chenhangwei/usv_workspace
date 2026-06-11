#!/usr/bin/env python3
"""Analyze 5-USV SITL nav logs: straight-line tracking, separation, goal progress."""
import gzip, sys, math, glob, os
import numpy as np

LOG_DIR = os.path.expanduser('~/usv_logs')
PATTERN = sys.argv[1] if len(sys.argv) > 1 else '20260610_191347'

def load(fname):
    cols = None
    rows = []
    with gzip.open(fname, 'rt') as f:
        for line in f:
            if line.startswith('#'):
                continue
            if cols is None:
                cols = line.strip().split(',')
                continue
            parts = line.rstrip('\n').split(',')
            if len(parts) != len(cols):
                continue
            rows.append(parts)
    return cols, rows

def fcol(cols, rows, name, default=np.nan):
    i = cols.index(name)
    out = []
    for r in rows:
        try:
            out.append(float(r[i]))
        except ValueError:
            out.append(default)
    return np.asarray(out)

def scol(cols, rows, name):
    i = cols.index(name)
    return [r[i] for r in rows]

usvs = {}
for f in sorted(glob.glob(f'{LOG_DIR}/nav_log_usv_0*_{PATTERN}*.csv.gz')):
    uid = os.path.basename(f).split('_')[3]
    cols, rows = load(f)
    usvs['usv_' + uid] = (cols, rows)

print(f'Loaded: {list(usvs.keys())}')

# ---------- per-USV trajectory analysis ----------
traj = {}
for uid, (cols, rows) in usvs.items():
    t = fcol(cols, rows, 'timestamp')
    x = fcol(cols, rows, 'pose_x')
    y = fcol(cols, rows, 'pose_y')
    tx = fcol(cols, rows, 'target_x')
    ty = fcol(cols, rows, 'target_y')
    gid = scol(cols, rows, 'goal_id')
    d2g = fcol(cols, rows, 'distance_to_goal')
    spd = fcol(cols, rows, 'velocity_speed')
    cte = fcol(cols, rows, 'cross_track_error')
    rl_act = fcol(cols, rows, 'rl_cmd_active')
    rl_vx = fcol(cols, rows, 'rl_cmd_vx')
    rl_om = fcol(cols, rows, 'rl_cmd_omega')
    enc = fcol(cols, rows, 'rl_encounter_type_index')
    mode = scol(cols, rows, 'flight_mode')
    traj[uid] = dict(t=t, x=x, y=y, tx=tx, ty=ty, gid=gid, d2g=d2g, spd=spd,
                     cte=cte, rl_act=rl_act, rl_vx=rl_vx, rl_om=rl_om, enc=enc, mode=mode)

print('\n===== 1. GOAL SEGMENTS & PROGRESS =====')
for uid, d in traj.items():
    # segment by goal_id
    gids = d['gid']
    segs = []
    cur = None
    for i, g in enumerate(gids):
        if g != cur:
            segs.append([g, i, i])
            cur = g
        else:
            segs[-1][2] = i
    print(f'\n--- {uid} ---')
    for g, i0, i1 in segs:
        dur = d['t'][i1] - d['t'][i0]
        x0, y0 = d['x'][i0], d['y'][i0]
        x1, y1 = d['x'][i1], d['y'][i1]
        tx, ty = d['tx'][i1], d['ty'][i1]
        d2g_start = d['d2g'][i0]
        d2g_end = d['d2g'][i1]
        d2g_min = np.nanmin(d['d2g'][i0:i1+1]) if i1 > i0 else d2g_end
        # straight-line dist start->target vs actual path length
        seg_x = d['x'][i0:i1+1]; seg_y = d['y'][i0:i1+1]
        path_len = float(np.sum(np.hypot(np.diff(seg_x), np.diff(seg_y))))
        straight = math.hypot(tx - x0, ty - y0)
        eff = straight / path_len if path_len > 0.5 else float('nan')
        mean_spd = float(np.nanmean(d['spd'][i0:i1+1]))
        print(f'  goal={g} dur={dur:7.1f}s  start=({x0:6.2f},{y0:6.2f}) end=({x1:6.2f},{y1:6.2f}) target=({tx:6.2f},{ty:6.2f})')
        print(f'      d2g: start={d2g_start:5.2f} end={d2g_end:5.2f} min={d2g_min:5.2f} | path={path_len:6.1f}m straight={straight:5.1f}m eff={eff:4.2f} | mean_spd={mean_spd:4.2f}')

print('\n===== 2. CROSS-TRACK ERROR (straightness) =====')
for uid, d in traj.items():
    cte = np.abs(d['cte'][~np.isnan(d['cte'])])
    cte = cte[cte > 0]  # 0 often means not computed
    if len(cte):
        print(f'{uid}: |CTE| mean={np.mean(cte):5.2f} p50={np.percentile(cte,50):5.2f} p90={np.percentile(cte,90):5.2f} max={np.max(cte):5.2f}  (n={len(cte)})')

print('\n===== 3. PAIRWISE SEPARATION (clamping/entanglement) =====')
# interpolate all to common time base
t0 = max(d['t'][0] for d in traj.values())
t1 = min(d['t'][-1] for d in traj.values())
tt = np.arange(t0, t1, 0.5)
pos = {}
for uid, d in traj.items():
    pos[uid] = (np.interp(tt, d['t'], d['x']), np.interp(tt, d['t'], d['y']))
ids = sorted(pos)
import itertools
sep_stats = []
for a, b in itertools.combinations(ids, 2):
    dist = np.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
    frac_close = float(np.mean(dist < 1.5))
    frac_vclose = float(np.mean(dist < 1.06))
    frac_coll = float(np.mean(dist < 0.75))
    sep_stats.append((a, b, np.min(dist), frac_coll, frac_vclose, frac_close))
    print(f'{a}-{b}: min={np.min(dist):5.2f}m  t<0.75m={frac_coll*100:5.1f}%  t<1.06m={frac_vclose*100:5.1f}%  t<1.5m={frac_close*100:5.1f}%')

print('\n===== 4. RL COMMAND BEHAVIOR =====')
for uid, d in traj.items():
    act = d['rl_act'] > 0.5
    if act.sum() == 0:
        print(f'{uid}: RL never active!')
        continue
    vx = d['rl_vx'][act]; om = d['rl_om'][act]
    spin = np.abs(om) > 0.4
    slow = vx < 0.08
    print(f'{uid}: rl_active={act.mean()*100:5.1f}%  vx mean={np.nanmean(vx):4.2f} p10={np.percentile(vx,10):4.2f}  |omega| mean={np.nanmean(np.abs(om)):4.2f} sat(>0.4)={spin.mean()*100:4.1f}%  vx<0.08={slow.mean()*100:4.1f}%')

print('\n===== 5. ENCOUNTER TYPE DISTRIBUTION (rl_encounter_type_index) =====')
for uid, d in traj.items():
    act = d['rl_act'] > 0.5
    enc = d['enc'][act]
    n = len(enc)
    if n == 0: continue
    vals, counts = np.unique(enc, return_counts=True)
    s = '  '.join(f'{int(v)}:{c/n*100:4.1f}%' for v, c in zip(vals, counts))
    print(f'{uid}: {s}   (-1=none 0=head_on 1=crossing 2=overtaking)')

print('\n===== 6. STALL / OSCILLATION DETECTION =====')
for uid, d in traj.items():
    act = d['rl_act'] > 0.5
    t = d['t'][act]
    if len(t) < 100: continue
    x = d['x'][act]; y = d['y'][act]; d2g = d['d2g'][act]
    # progress over 30s windows
    win = 30.0
    stall_time = 0.0
    i = 0
    n = len(t)
    while i < n:
        j = i
        while j < n and t[j] - t[i] < win:
            j += 1
        if j >= n: break
        prog = d2g[i] - d2g[j]
        if prog < 0.3:
            stall_time += t[j] - t[i]
        i = j
    total = t[-1] - t[0]
    print(f'{uid}: rl-active span={total:7.1f}s  stalled(>30s win, <0.3m d2g progress)={stall_time:7.1f}s ({stall_time/total*100:4.1f}%)')
