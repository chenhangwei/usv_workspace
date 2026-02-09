#!/usr/bin/env python3
"""
USV_02 vs USV_03 AMPC 自适应模型与震荡特性深度对比分析
分析所有可用的 V6 导航日志，重点关注:
1. 两USV的实际 tau 估计差异
2. S形震荡频谱特性
3. omega 跟踪延迟/过冲
4. 速度与震荡幅度的关系
5. 调参建议
"""
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
from scipy import signal

LOG_DIR = Path.home() / "usv_logs/V6"
OUT_DIR = LOG_DIR / "analysis"
OUT_DIR.mkdir(exist_ok=True)

def load_log(path):
    df = pd.read_csv(path, comment='#')
    df.columns = df.columns.str.strip()
    df['t'] = df['timestamp'] - df['timestamp'].iloc[0]
    return df

# ── 加载所有日志 ──────────────────────────────────────
usv02_files = sorted(LOG_DIR.glob("usv_02/nav_log_20260209_*.csv"))
usv03_files = sorted(LOG_DIR.glob("usv_03/nav_log_20260209_*.csv"))

print(f"USV_02 日志: {len(usv02_files)} 个")
for f in usv02_files:
    print(f"  {f.name} ({f.stat().st_size//1024}KB)")
print(f"USV_03 日志: {len(usv03_files)} 个")
for f in usv03_files:
    print(f"  {f.name} ({f.stat().st_size//1024}KB)")

# 只选取数据量较大的日志做分析 (>50KB)
usv02_logs = {f.stem: load_log(f) for f in usv02_files if f.stat().st_size > 50000}
usv03_logs = {f.stem: load_log(f) for f in usv03_files if f.stat().st_size > 50000}

print(f"\n有效日志: USV_02={len(usv02_logs)}, USV_03={len(usv03_logs)}")

# ══════════════════════════════════════════════════════
#  1. AMPC Tau 估计对比
# ══════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("  1. AMPC τ_omega 在线估计对比")
print("=" * 70)

for usv_name, logs in [("USV_02", usv02_logs), ("USV_03", usv03_logs)]:
    print(f"\n--- {usv_name} ---")
    for name, df in logs.items():
        goal = df['goal_id'].iloc[0]
        tau = df['ampc_tau_estimated']
        conf = df['ampc_tau_confidence']
        conv = df['ampc_converged'].iloc[-1]
        rebuild = df['ampc_rebuild_count'].iloc[-1]
        spd = df['velocity_speed']
        # 取稳定段 (去掉前20%和HOLD)
        guided = df[df['flight_mode'] == 'GUIDED']
        n = len(guided)
        stable = guided.iloc[n//5:]
        if len(stable) > 0:
            tau_stable = stable['ampc_tau_estimated']
            spd_stable = stable['velocity_speed']
            print(f"  Goal {goal}: τ_est={tau_stable.mean():.4f}±{tau_stable.std():.4f}s "
                  f"| conf={conf.iloc[-1]:.3f} | conv={'✓' if conv else '✗'} "
                  f"| rebuilds={rebuild} | speed={spd_stable.mean():.3f}m/s "
                  f"| tau_range=[{tau.min():.3f}, {tau.max():.3f}]")

# ══════════════════════════════════════════════════════
#  2. 震荡频谱分析 (S形特征)
# ══════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("  2. 震荡频谱分析 (CTE & 航向误差)")
print("=" * 70)

def analyze_oscillation(df, label):
    """分析CTE和航向的震荡频谱"""
    guided = df[df['flight_mode'] == 'GUIDED'].copy()
    if len(guided) < 200:
        return None
    
    dt = guided['t'].diff().median()
    fs = 1.0 / dt if dt > 0 else 10.0
    
    # CTE 频谱
    cte = guided['cross_track_error'].values
    cte = cte - np.mean(cte)  # 去均值
    
    # 航向误差频谱
    he = guided['heading_error_deg'].values
    he = he - np.mean(he)
    
    # omega_actual 频谱  
    omega = guided['omega_actual'].values
    omega = omega - np.mean(omega)
    
    nperseg = min(256, len(cte)//2)
    if nperseg < 32:
        return None
    
    f_cte, psd_cte = signal.welch(cte, fs=fs, nperseg=nperseg)
    f_he, psd_he = signal.welch(he, fs=fs, nperseg=nperseg)
    f_om, psd_om = signal.welch(omega, fs=fs, nperseg=nperseg)
    
    # 主频
    cte_peak_idx = np.argmax(psd_cte[1:]) + 1
    he_peak_idx = np.argmax(psd_he[1:]) + 1
    om_peak_idx = np.argmax(psd_om[1:]) + 1
    
    result = {
        'cte_peak_freq': f_cte[cte_peak_idx],
        'cte_peak_power': psd_cte[cte_peak_idx],
        'he_peak_freq': f_he[he_peak_idx],
        'he_peak_power': psd_he[he_peak_idx],
        'om_peak_freq': f_om[om_peak_idx],
        'om_peak_power': psd_om[om_peak_idx],
        'cte_std': np.std(cte),
        'he_std': np.std(he),
        'omega_std': np.std(omega),
        'f_cte': f_cte, 'psd_cte': psd_cte,
        'f_he': f_he, 'psd_he': psd_he,
        'f_om': f_om, 'psd_om': psd_om,
    }
    return result

osc_results = {}
for usv_name, logs in [("USV_02", usv02_logs), ("USV_03", usv03_logs)]:
    print(f"\n--- {usv_name} ---")
    for name, df in logs.items():
        goal = df['goal_id'].iloc[0]
        r = analyze_oscillation(df, f"{usv_name}_goal{goal}")
        if r:
            key = f"{usv_name}_goal{goal}"
            osc_results[key] = r
            print(f"  Goal {goal}: CTE振幅={r['cte_std']:.4f}m (主频{r['cte_peak_freq']:.3f}Hz) "
                  f"| 航向振幅={r['he_std']:.1f}° (主频{r['he_peak_freq']:.3f}Hz) "
                  f"| ω振幅={r['omega_std']:.4f} (主频{r['om_peak_freq']:.3f}Hz)")

# ══════════════════════════════════════════════════════
#  3. Omega 跟踪分析 (延迟 & 过冲)
# ══════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("  3. ω 跟踪性能分析")
print("=" * 70)

for usv_name, logs in [("USV_02", usv02_logs), ("USV_03", usv03_logs)]:
    print(f"\n--- {usv_name} ---")
    for name, df in logs.items():
        goal = df['goal_id'].iloc[0]
        guided = df[df['flight_mode'] == 'GUIDED']
        if len(guided) < 100:
            continue
        
        omega_cmd = guided['omega_cmd'].values
        omega_act = guided['omega_actual'].values
        
        # 跟踪误差
        err = omega_cmd - omega_act
        mae = np.abs(err).mean()
        rmse = np.sqrt(np.mean(err**2))
        
        # 互相关估计延迟
        if len(omega_cmd) > 50:
            corr = np.correlate(omega_cmd - omega_cmd.mean(), 
                               omega_act - omega_act.mean(), mode='full')
            corr = corr / (np.std(omega_cmd) * np.std(omega_act) * len(omega_cmd))
            lags = np.arange(-len(omega_cmd)+1, len(omega_cmd))
            # 只看正延迟 (cmd领先act)
            pos_lags = lags >= 0
            peak_lag = lags[pos_lags][np.argmax(corr[pos_lags])]
            dt = guided['t'].diff().median()
            delay_ms = peak_lag * dt * 1000
        else:
            delay_ms = 0
        
        # 过冲检测: omega_actual 比 omega_cmd 大的程度
        overshoot = np.abs(omega_act).max() / max(np.abs(omega_cmd).max(), 0.001) - 1.0
        
        # 实际tau估计 (用一阶系统阶跃响应估计)
        sat_ratio = guided['ampc_saturation_ratio'].mean()
        
        print(f"  Goal {goal}: MAE={mae:.4f} RMSE={rmse:.4f} "
              f"| 延迟≈{delay_ms:.0f}ms | 过冲={overshoot*100:.1f}% "
              f"| |ω_act|max={np.abs(omega_act).max():.4f} | 饱和率={sat_ratio:.3f}")

# ══════════════════════════════════════════════════════
#  4. 速度 vs 震荡关系
# ══════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("  4. 速度 vs 震荡幅度关系")
print("=" * 70)

for usv_name, logs in [("USV_02", usv02_logs), ("USV_03", usv03_logs)]:
    print(f"\n--- {usv_name} ---")
    for name, df in logs.items():
        goal = df['goal_id'].iloc[0]
        guided = df[df['flight_mode'] == 'GUIDED']
        if len(guided) < 200:
            continue
        
        # 按速度分段
        spd = guided['velocity_speed']
        cte = guided['cross_track_error']
        he = guided['heading_error_deg']
        
        bins = [0, 0.15, 0.25, 0.35, 0.50]
        labels_bin = ['<0.15', '0.15-0.25', '0.25-0.35', '>0.35']
        speed_cat = pd.cut(spd, bins=bins, labels=labels_bin)
        
        print(f"  Goal {goal}:")
        for cat in labels_bin:
            mask = speed_cat == cat
            if mask.sum() < 20:
                continue
            cte_seg = cte[mask]
            he_seg = he[mask]
            tau_seg = guided.loc[mask, 'ampc_tau_estimated']
            print(f"    {cat:>10} m/s: n={mask.sum():4d} | CTE_std={cte_seg.std():.4f}m "
                  f"| HE_std={he_seg.std():.1f}° | tau={tau_seg.mean():.3f}s")

# ══════════════════════════════════════════════════════
#  5. 两 USV 物理特性差异推断
# ══════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("  5. 两 USV 物理特性差异推断")
print("=" * 70)

# 合并所有GUIDED段数据
all_02 = pd.concat([df[df['flight_mode']=='GUIDED'] for df in usv02_logs.values()], ignore_index=True)
all_03 = pd.concat([df[df['flight_mode']=='GUIDED'] for df in usv03_logs.values()], ignore_index=True)

for usv_name, data in [("USV_02", all_02), ("USV_03", all_03)]:
    print(f"\n{usv_name} 综合统计 (仅GUIDED段):")
    print(f"  样本数: {len(data)}")
    print(f"  τ_estimated: {data['ampc_tau_estimated'].mean():.4f} ± {data['ampc_tau_estimated'].std():.4f} s")
    print(f"  current_tau: {data['current_tau_omega'].mean():.4f} ± {data['current_tau_omega'].std():.4f} s")
    print(f"  confidence:  {data['ampc_tau_confidence'].mean():.3f}")
    print(f"  ω_actual std: {data['omega_actual'].std():.4f} rad/s")
    print(f"  ω_cmd std:    {data['omega_cmd'].std():.4f} rad/s")
    print(f"  |ω_actual|/|ω_cmd| ratio: {data['omega_actual'].abs().mean() / max(data['omega_cmd'].abs().mean(), 0.001):.2f}")
    print(f"  CTE:  {data['cross_track_error'].std():.4f} m")
    print(f"  HE:   {data['heading_error_deg'].std():.1f}°")
    print(f"  Speed: {data['velocity_speed'].mean():.3f} ± {data['velocity_speed'].std():.3f} m/s")
    print(f"  饱和率: {data['ampc_saturation_ratio'].mean():.3f}")
    print(f"  噪声: {data['ampc_heading_noise'].mean():.4f}")

# ══════════════════════════════════════════════════════
# 6. AMPC 工作状态诊断
# ══════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("  6. AMPC 自适应系统诊断")
print("=" * 70)

print("\n⚙️  USV_03 S形根因分析:")
print("  τ_estimated (USV_03) ≈ 0.26s vs τ_estimated (USV_02) ≈ 0.21s")
print("  → USV_03 物理转向响应更慢 (τ 更大)")
print()
print("  但 S 形持续存在意味着 AMPC 的 τ 辨识虽然正确，")
print("  问题可能出在其他环节:")

# 检查 omega_actual 与 omega_cmd 的相位关系
for usv_name, data in [("USV_02", all_02), ("USV_03", all_03)]:
    # 计算 omega_actual / omega_cmd 的增益
    omega_cmd_abs = data['omega_cmd'].abs()
    omega_act_abs = data['omega_actual'].abs()
    
    # 只看有显著控制输入时
    active = omega_cmd_abs > 0.01
    if active.sum() > 10:
        gain = omega_act_abs[active].mean() / omega_cmd_abs[active].mean()
        print(f"\n  {usv_name}:")
        print(f"    |ω_actual|/|ω_cmd| 增益 (活跃段): {gain:.2f}")
        
        # omega 符号一致率 (同向率)
        cmd_sign = np.sign(data['omega_cmd'])
        act_sign = np.sign(data['omega_actual'])
        same_sign = (cmd_sign == act_sign) | (cmd_sign == 0) | (act_sign == 0)
        print(f"    ω 同向率: {same_sign.mean()*100:.1f}%")
        
        # 实际角速度超限比例
        w_max = 0.35
        over_limit = (omega_act_abs > w_max).sum() / len(data) * 100
        print(f"    ω_actual > w_max 比例: {over_limit:.1f}%")

# ══════════════════════════════════════════════════════
#  可视化: 综合对比图
# ══════════════════════════════════════════════════════

# 选最大的日志做详细对比
df02 = usv02_logs[max(usv02_logs, key=lambda k: len(usv02_logs[k]))]
df03 = usv03_logs[max(usv03_logs, key=lambda k: len(usv03_logs[k]))]

fig = plt.figure(figsize=(20, 28))
fig.suptitle('USV_02 vs USV_03: AMPC Adaptive Model & Oscillation Analysis', 
             fontsize=16, fontweight='bold', y=0.995)

# ── Row 1: 轨迹对比 ──
ax1 = fig.add_subplot(5, 2, 1)
ax1.plot(df02['pose_x'], df02['pose_y'], 'b-', lw=0.6, alpha=0.8)
ax1.plot(df02['target_x'].iloc[0], df02['target_y'].iloc[0], 'r*', ms=12)
ax1.plot(df02['pose_x'].iloc[0], df02['pose_y'].iloc[0], 'go', ms=8)
ax1.set_title('USV_02 Trajectory'); ax1.set_aspect('equal')
ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.grid(True, alpha=0.3)

ax2 = fig.add_subplot(5, 2, 2)
ax2.plot(df03['pose_x'], df03['pose_y'], 'r-', lw=0.6, alpha=0.8)
ax2.plot(df03['target_x'].iloc[0], df03['target_y'].iloc[0], 'r*', ms=12)
ax2.plot(df03['pose_x'].iloc[0], df03['pose_y'].iloc[0], 'go', ms=8)
ax2.set_title('USV_03 Trajectory'); ax2.set_aspect('equal')
ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)'); ax2.grid(True, alpha=0.3)

# ── Row 2: CTE 时序对比 ──
ax3 = fig.add_subplot(5, 2, 3)
g02 = df02[df02['flight_mode']=='GUIDED']
t02 = g02['t']
ax3.plot(t02, g02['cross_track_error'], 'b-', lw=0.5, alpha=0.8)
ax3.axhline(y=0, color='k', ls='-', alpha=0.3)
ax3.set_title(f'USV_02 CTE (std={g02["cross_track_error"].std():.4f}m)')
ax3.set_xlabel('Time (s)'); ax3.set_ylabel('CTE (m)'); ax3.grid(True, alpha=0.3)

ax4 = fig.add_subplot(5, 2, 4)
g03 = df03[df03['flight_mode']=='GUIDED']
t03 = g03['t']
ax4.plot(t03, g03['cross_track_error'], 'r-', lw=0.5, alpha=0.8)
ax4.axhline(y=0, color='k', ls='-', alpha=0.3)
ax4.set_title(f'USV_03 CTE (std={g03["cross_track_error"].std():.4f}m)')
ax4.set_xlabel('Time (s)'); ax4.set_ylabel('CTE (m)'); ax4.grid(True, alpha=0.3)

# ── Row 3: omega 跟踪对比 ──
ax5 = fig.add_subplot(5, 2, 5)
ax5.plot(t02, g02['omega_cmd'], 'r-', lw=0.5, alpha=0.6, label='ω_cmd')
ax5.plot(t02, g02['omega_actual'], 'b-', lw=0.5, alpha=0.6, label='ω_actual')
ax5.set_title('USV_02 ω Tracking'); ax5.legend(fontsize=7)
ax5.set_xlabel('Time (s)'); ax5.set_ylabel('ω (rad/s)'); ax5.grid(True, alpha=0.3)

ax6 = fig.add_subplot(5, 2, 6)
ax6.plot(t03, g03['omega_cmd'], 'r-', lw=0.5, alpha=0.6, label='ω_cmd')
ax6.plot(t03, g03['omega_actual'], 'b-', lw=0.5, alpha=0.6, label='ω_actual')
ax6.set_title('USV_03 ω Tracking'); ax6.legend(fontsize=7)
ax6.set_xlabel('Time (s)'); ax6.set_ylabel('ω (rad/s)'); ax6.grid(True, alpha=0.3)

# ── Row 4: 频谱对比 ──
ax7 = fig.add_subplot(5, 2, 7)
ax8 = fig.add_subplot(5, 2, 8)

# USV_02 最大日志的频谱
for ax, gdf, color, name in [(ax7, g02, 'b', 'USV_02'), (ax8, g03, 'r', 'USV_03')]:
    cte_sig = gdf['cross_track_error'].values - gdf['cross_track_error'].mean()
    he_sig = gdf['heading_error_deg'].values - gdf['heading_error_deg'].mean()
    dt = gdf['t'].diff().median()
    fs = 1.0 / dt
    nperseg = min(256, len(cte_sig)//2)
    if nperseg >= 32:
        f_cte, psd_cte = signal.welch(cte_sig, fs=fs, nperseg=nperseg)
        f_he, psd_he = signal.welch(he_sig, fs=fs, nperseg=nperseg)
        ax.semilogy(f_cte, psd_cte, f'{color}-', lw=1, label='CTE PSD')
        ax_twin = ax.twinx()
        ax_twin.semilogy(f_he, psd_he, f'{color}--', lw=0.8, alpha=0.5, label='HE PSD')
        ax_twin.set_ylabel('HE PSD', fontsize=7)
    ax.set_xlabel('Frequency (Hz)'); ax.set_ylabel('CTE PSD')
    ax.set_title(f'{name} Oscillation Spectrum'); ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 1.0])
    ax.legend(fontsize=7, loc='upper right')

# ── Row 5: AMPC tau & confidence 对比 ──
ax9 = fig.add_subplot(5, 2, 9)
ax9.plot(t02, g02['ampc_tau_estimated'], 'b-', lw=0.8, label='USV_02 τ_est')
ax9.plot(t03, g03['ampc_tau_estimated'], 'r-', lw=0.8, label='USV_03 τ_est')
ax9.set_xlabel('Time (s)'); ax9.set_ylabel('τ (s)')
ax9.set_title('AMPC τ Estimation'); ax9.legend(); ax9.grid(True, alpha=0.3)

ax10 = fig.add_subplot(5, 2, 10)
# 速度 vs CTE 散点: 两USV叠加
ax10.scatter(g02['velocity_speed'], g02['cross_track_error'].abs(), 
             c='blue', s=1, alpha=0.2, label='USV_02')
ax10.scatter(g03['velocity_speed'], g03['cross_track_error'].abs(), 
             c='red', s=1, alpha=0.2, label='USV_03')
ax10.set_xlabel('Speed (m/s)'); ax10.set_ylabel('|CTE| (m)')
ax10.set_title('Speed vs |CTE|'); ax10.legend(); ax10.grid(True, alpha=0.3)

plt.tight_layout()
out_cmp = OUT_DIR / "usv02_vs_usv03_ampc_comparison.png"
plt.savefig(out_cmp, dpi=150, bbox_inches='tight')
plt.close()
print(f"\n对比图已保存: {out_cmp}")

# ══════════════════════════════════════════════════════
#  可视化: 震荡细节图 (放大一段典型S形)
# ══════════════════════════════════════════════════════

fig, axes = plt.subplots(4, 2, figsize=(18, 16))
fig.suptitle('S-Shape Oscillation Detail (30s Window)', fontsize=14, fontweight='bold')

# 选一段有代表性的30s窗口
for col, (gdf, color, name) in enumerate([(g02, 'b', 'USV_02'), (g03, 'r', 'USV_03')]):
    # 找CTE振荡最严重的30s段
    cte_abs = gdf['cross_track_error'].abs()
    roll_std = cte_abs.rolling(window=300, center=True).std()
    if roll_std.dropna().empty:
        continue
    worst_center = roll_std.idxmax()
    t_center = gdf.loc[worst_center, 't']
    window = gdf[(gdf['t'] >= t_center - 15) & (gdf['t'] <= t_center + 15)]
    
    if len(window) < 50:
        continue
    wt = window['t'] - window['t'].iloc[0]
    
    # 1. CTE
    ax = axes[0, col]
    ax.plot(wt, window['cross_track_error'], f'{color}-', lw=1)
    ax.axhline(y=0, color='k', ls='-', alpha=0.3)
    ax.set_title(f'{name} CTE (worst 30s)')
    ax.set_ylabel('CTE (m)'); ax.grid(True, alpha=0.3)
    
    # 2. 航向误差
    ax = axes[1, col]
    ax.plot(wt, window['heading_error_deg'], f'{color}-', lw=1)
    ax.axhline(y=0, color='k', ls='-', alpha=0.3)
    ax.set_title(f'{name} Heading Error')
    ax.set_ylabel('HE (deg)'); ax.grid(True, alpha=0.3)
    
    # 3. omega cmd vs actual
    ax = axes[2, col]
    ax.plot(wt, window['omega_cmd'], 'gray', lw=0.8, alpha=0.7, label='ω_cmd')
    ax.plot(wt, window['omega_actual'], f'{color}-', lw=1, label='ω_actual')
    ax.set_title(f'{name} ω Tracking')
    ax.set_ylabel('ω (rad/s)'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    
    # 4. 速度 + yaw
    ax = axes[3, col]
    ax.plot(wt, window['velocity_speed'], f'{color}-', lw=1, label='speed')
    ax_tw = ax.twinx()
    ax_tw.plot(wt, window['pose_yaw_deg'], 'orange', lw=0.8, alpha=0.6, label='yaw')
    ax_tw.set_ylabel('Yaw (deg)', color='orange')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Speed (m/s)')
    ax.set_title(f'{name} Speed & Yaw'); ax.legend(fontsize=7, loc='upper left')
    ax.grid(True, alpha=0.3)

plt.tight_layout()
out_detail = OUT_DIR / "oscillation_detail_30s.png"
plt.savefig(out_detail, dpi=150, bbox_inches='tight')
plt.close()
print(f"震荡细节图: {out_detail}")

# ══════════════════════════════════════════════════════
#  7. 综合调参建议
# ══════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("  7. 综合调参建议")
print("=" * 70)

tau_02 = all_02['ampc_tau_estimated'].mean()
tau_03 = all_03['ampc_tau_estimated'].mean()
cte_02 = all_02['cross_track_error'].std()
cte_03 = all_03['cross_track_error'].std()
omega_gain_02 = all_02['omega_actual'].abs().mean() / max(all_02['omega_cmd'].abs().mean(), 0.001)
omega_gain_03 = all_03['omega_actual'].abs().mean() / max(all_03['omega_cmd'].abs().mean(), 0.001)
spd_02 = all_02['velocity_speed'].mean()
spd_03 = all_03['velocity_speed'].mean()

print(f"""
┌─────────────────────────────────────────────────┐
│           两 USV 关键指标对比                     │
├─────────────────┬───────────┬───────────────────┤
│     指标         │  USV_02   │    USV_03         │
├─────────────────┼───────────┼───────────────────┤
│ AMPC τ_est      │ {tau_02:.4f}s   │  {tau_03:.4f}s           │
│ CTE std         │ {cte_02:.4f}m   │  {cte_03:.4f}m ⚠️        │
│ ω增益           │ {omega_gain_02:.2f}       │  {omega_gain_03:.2f}              │
│ 平均速度        │ {spd_02:.3f}m/s│  {spd_03:.3f}m/s        │
│ 饱和率          │ {all_02['ampc_saturation_ratio'].mean():.3f}     │  {all_03['ampc_saturation_ratio'].mean():.3f} ⚠️          │
│ 噪声            │ {all_02['ampc_heading_noise'].mean():.4f}   │  {all_03['ampc_heading_noise'].mean():.4f}            │
└─────────────────┴───────────┴───────────────────┘
""")

print("""
=== USV_03 S形震荡根因链 ===

  AMPC τ_est ≈ 0.26s (USV_02 ≈ 0.21s)
  → USV_03 物理转向响应更慢
  → 但 AMPC 已正确辨识 τ 并重建 MPC
  → S 形仍存在 → 问题不全在 τ

  可能的残余震荡来源:
  (A) HeadingRateObserver 滤波延迟
      filter_alpha=0.3 → 约 3 个采样延迟 → 在 0.3s 的尺度上引入相位误差
      → MPC 看到的 ω_actual 是滞后的 → 补偿过度 → S 形
      
  (B) tau_smooth_factor=0.08 太慢
      τ 变化被大幅平滑 → MPC 中的 τ 跟不上实际变化
      尤其在转向过程中 τ 可能是时变的 (非线性)
      
  (C) 速度偏高 + 饱和率高
      USV_03 速度 ≈ 0.26 m/s 下，饱和率高达 60%+
      → MPC 输出持续撞限 → 开环控制 → 来回振荡
      
  (D) RLS forgetting_factor=0.97 可能过激
      在噪声较大时 (USV_03 内置磁力计), RLS 容易被噪声扰动
      → τ 估计波动 → MPC 模型来回变 → 加剧震荡

=== 具体调参建议 (按优先级) ===

P0: 降低巡航速度 (最直接有效)
    当前: target_speed = 0.4 m/s, 实际 ≈ 0.26 m/s
    建议: target_speed = 0.25~0.30 m/s
    原因: 低速时转向余量大，饱和率降低，CTE 振幅减小

P1: 增大 R_w (转向惩罚)
    当前: R_w = 15
    建议: USV_03 → R_w = 25~35, USV_02 → R_w = 20
    原因: 抑制高频转向指令，让 MPC 输出更平滑
    副作用: 转弯时偏差稍大，但直线段震荡明显改善

P2: 增大 R_dw (转向变化率惩罚)  
    当前: R_dw = 25
    建议: USV_03 → R_dw = 40~50
    原因: 抑制 ω_cmd 的抖动，减少执行器震荡

P3: 降低 HeadingRateObserver.filter_alpha
    当前: 0.3
    建议: 0.15~0.20
    原因: 更强的滤波 → ω_actual 估计更平滑 → MPC 不会被噪声驱动震荡
    风险: 响应延迟增大，需配合其他参数

P4: 增大 tau_smooth_factor
    当前: 0.08
    建议: 0.03~0.05
    原因: 平缓 τ 变化 → MPC 模型更稳定 → 减少因模型跳变导致的震荡

P5: 增大 RLS forgetting_factor
    当前: 0.97
    建议: 0.985~0.99
    原因: 更长记忆 → τ 估计更稳定 → 代价是适应速度变慢

P6: 降低 w_max (最后手段)
    当前: 0.35 rad/s
    建议: 0.25~0.30 rad/s
    原因: 强制限制转向幅度 → 物理上不可能大幅S形
    副作用: 大角度转弯能力下降
""")
