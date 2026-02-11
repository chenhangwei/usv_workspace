#!/usr/bin/env python3
"""
GAZEBO仿真数据对比分析 - USV_02 vs USV_03
分析日期: 2026-02-10
数据来源:
  - usv_03: nav_log_20260210_121106_goal_1.csv
  - usv_02: nav_log_20260210_121710_goal_1.csv
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path
import os

# ============================================================
# 配置
# ============================================================
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'WenQuanYi Micro Hei']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 120

LOG_DIR = os.path.expanduser("~/usv_logs")
FILE_USV03 = os.path.join(LOG_DIR, "nav_log_20260210_121106_goal_1.csv")
FILE_USV02 = os.path.join(LOG_DIR, "nav_log_20260210_121710_goal_1.csv")
OUTPUT_DIR = os.path.expanduser("~/usv_logs/analysis_output")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def load_data(filepath, usv_id):
    """加载CSV数据，跳过注释行"""
    df = pd.read_csv(filepath, comment='#')
    # 过滤GUIDED模式数据（实际导航阶段）
    df_guided = df[df['flight_mode'] == 'GUIDED'].copy()
    # 归一化时间轴（从0开始，秒）
    if len(df_guided) > 0:
        t0 = df_guided['timestamp'].iloc[0]
        df_guided['time_s'] = df_guided['timestamp'] - t0
    df_guided['usv_id'] = usv_id
    return df, df_guided


def compute_metrics(df_guided, usv_id):
    """计算各项性能指标"""
    metrics = {}
    metrics['USV'] = usv_id
    
    # 任务时长
    metrics['任务时长(s)'] = df_guided['time_s'].iloc[-1] - df_guided['time_s'].iloc[0]
    
    # 航程（累计位移）
    dx = df_guided['pose_x'].diff()
    dy = df_guided['pose_y'].diff()
    dist_steps = np.sqrt(dx**2 + dy**2)
    metrics['总航程(m)'] = dist_steps.sum()
    
    # 速度统计
    metrics['平均速度(m/s)'] = df_guided['velocity_speed'].mean()
    metrics['最大速度(m/s)'] = df_guided['velocity_speed'].max()
    metrics['速度标准差(m/s)'] = df_guided['velocity_speed'].std()
    
    # 航向误差（heading_error_deg）
    metrics['平均航向误差(deg)'] = df_guided['heading_error_deg'].abs().mean()
    metrics['最大航向误差(deg)'] = df_guided['heading_error_deg'].abs().max()
    metrics['航向误差标准差(deg)'] = df_guided['heading_error_deg'].std()
    
    # 横向跟踪误差（cross_track_error）
    metrics['平均CTE(m)'] = df_guided['cross_track_error'].abs().mean()
    metrics['最大CTE(m)'] = df_guided['cross_track_error'].abs().max()
    metrics['CTE标准差(m)'] = df_guided['cross_track_error'].std()
    
    # 到目标距离
    metrics['最终到目标距离(m)'] = df_guided['distance_to_goal'].iloc[-1]
    metrics['最小到目标距离(m)'] = df_guided['distance_to_goal'].min()
    
    # MPC求解时间
    mpc_valid = df_guided[df_guided['mpc_solve_time_ms'] > 0]['mpc_solve_time_ms']
    if len(mpc_valid) > 0:
        metrics['MPC平均求解时间(ms)'] = mpc_valid.mean()
        metrics['MPC最大求解时间(ms)'] = mpc_valid.max()
    
    # 角速度命令统计
    metrics['平均cmd_omega(rad/s)'] = df_guided['cmd_omega'].abs().mean()
    metrics['最大cmd_omega(rad/s)'] = df_guided['cmd_omega'].abs().max()
    
    # AMPC统计
    ampc_data = df_guided[df_guided['ampc_enabled'] == 1]
    if len(ampc_data) > 0:
        metrics['AMPC_tau估计均值'] = ampc_data['ampc_tau_estimated'].mean()
        metrics['AMPC置信度均值'] = ampc_data['ampc_tau_confidence'].mean()
        converged = ampc_data[ampc_data['ampc_converged'] == 1]
        metrics['AMPC收敛率(%)'] = len(converged) / len(ampc_data) * 100
    
    # 航点数量
    metrics['航点数量'] = df_guided['goal_id'].nunique()
    
    return metrics


def plot_trajectory(ax, df03, df02):
    """绘制2D轨迹对比"""
    ax.plot(df03['pose_x'], df03['pose_y'], 'b-', linewidth=1.2, alpha=0.8, label='USV_03')
    ax.plot(df02['pose_x'], df02['pose_y'], 'r-', linewidth=1.2, alpha=0.8, label='USV_02')
    
    # 目标点标注
    for df, color, marker in [(df03, 'blue', 's'), (df02, 'red', '^')]:
        goals = df.groupby('goal_id').first()
        ax.scatter(goals['target_x'], goals['target_y'], c=color, marker=marker, 
                   s=80, zorder=5, edgecolors='k', linewidths=0.5)
    
    # 起止点
    ax.plot(df03['pose_x'].iloc[0], df03['pose_y'].iloc[0], 'b*', markersize=12, label='USV_03 起点')
    ax.plot(df02['pose_x'].iloc[0], df02['pose_y'].iloc[0], 'r*', markersize=12, label='USV_02 起点')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory Comparison (GAZEBO Sim)')
    ax.legend(loc='best', fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)


def plot_speed(ax, df03, df02):
    """速度对比"""
    ax.plot(df03['time_s'], df03['velocity_speed'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03')
    ax.plot(df02['time_s'], df02['velocity_speed'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Speed Profile')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_heading_error(ax, df03, df02):
    """航向误差对比"""
    ax.plot(df03['time_s'], df03['heading_error_deg'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03')
    ax.plot(df02['time_s'], df02['heading_error_deg'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02')
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading Error (deg)')
    ax.set_title('Heading Error')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_cross_track_error(ax, df03, df02):
    """横向跟踪误差对比"""
    ax.plot(df03['time_s'], df03['cross_track_error'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03')
    ax.plot(df02['time_s'], df02['cross_track_error'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02')
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cross-Track Error (m)')
    ax.set_title('Cross-Track Error (CTE)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_distance_to_goal(ax, df03, df02):
    """到目标距离对比"""
    ax.plot(df03['time_s'], df03['distance_to_goal'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03')
    ax.plot(df02['time_s'], df02['distance_to_goal'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance to Goal (m)')
    ax.set_title('Distance to Goal')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_cmd_omega(ax, df03, df02):
    """角速度命令对比"""
    ax.plot(df03['time_s'], df03['cmd_omega'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03')
    ax.plot(df02['time_s'], df02['cmd_omega'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02')
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('cmd_omega (rad/s)')
    ax.set_title('Steering Command (omega)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_mpc_solve_time(ax, df03, df02):
    """MPC求解时间对比"""
    m03 = df03[df03['mpc_solve_time_ms'] > 0]
    m02 = df02[df02['mpc_solve_time_ms'] > 0]
    ax.plot(m03['time_s'], m03['mpc_solve_time_ms'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03')
    ax.plot(m02['time_s'], m02['mpc_solve_time_ms'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('MPC Solve Time (ms)')
    ax.set_title('MPC Solver Performance')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_ampc_tau(ax, df03, df02):
    """AMPC Tau估计对比"""
    a03 = df03[df03['ampc_enabled'] == 1]
    a02 = df02[df02['ampc_enabled'] == 1]
    if len(a03) > 0:
        ax.plot(a03['time_s'], a03['ampc_tau_estimated'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03 tau_est')
        ax.plot(a03['time_s'], a03['current_tau_omega'], 'b--', linewidth=0.8, alpha=0.5, label='USV_03 tau_cur')
    if len(a02) > 0:
        ax.plot(a02['time_s'], a02['ampc_tau_estimated'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02 tau_est')
        ax.plot(a02['time_s'], a02['current_tau_omega'], 'r--', linewidth=0.8, alpha=0.5, label='USV_02 tau_cur')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Tau Omega (s)')
    ax.set_title('AMPC Tau Estimation')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)


def plot_cmd_vx(ax, df03, df02):
    """纵向速度命令对比"""
    ax.plot(df03['time_s'], df03['cmd_vx'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03')
    ax.plot(df02['time_s'], df02['cmd_vx'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('cmd_vx (m/s)')
    ax.set_title('Forward Speed Command')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_yaw(ax, df03, df02):
    """航向角对比"""
    ax.plot(df03['time_s'], df03['pose_yaw_deg'], 'b-', linewidth=0.8, alpha=0.7, label='USV_03')
    ax.plot(df02['time_s'], df02['pose_yaw_deg'], 'r-', linewidth=0.8, alpha=0.7, label='USV_02')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (deg)')
    ax.set_title('Heading Angle (Yaw)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)


def main():
    print("=" * 60)
    print("  GAZEBO仿真数据对比分析: USV_02 vs USV_03")
    print("=" * 60)
    
    # 加载数据
    df03_all, df03 = load_data(FILE_USV03, 'usv_03')
    df02_all, df02 = load_data(FILE_USV02, 'usv_02')
    
    print(f"\n[数据概要]")
    print(f"  USV_03: 总行数={len(df03_all)}, GUIDED模式行数={len(df03)}")
    print(f"  USV_02: 总行数={len(df02_all)}, GUIDED模式行数={len(df02)}")
    
    # 计算性能指标
    m03 = compute_metrics(df03, 'USV_03')
    m02 = compute_metrics(df02, 'USV_02')
    
    # 打印指标对比表
    print(f"\n{'='*60}")
    print(f"{'指标':<25} {'USV_03':>15} {'USV_02':>15}")
    print(f"{'='*60}")
    all_keys = [k for k in m03 if k != 'USV']
    for key in all_keys:
        v03 = m03.get(key, 'N/A')
        v02 = m02.get(key, 'N/A')
        if isinstance(v03, float):
            print(f"  {key:<25} {v03:>15.4f} {v02 if isinstance(v02, str) else v02:>15.4f}")
        else:
            print(f"  {key:<25} {str(v03):>15} {str(v02):>15}")
    print(f"{'='*60}")
    
    # ============================================================
    # 图 1: 综合仪表板 (3x4 布局)
    # ============================================================
    fig = plt.figure(figsize=(22, 18))
    fig.suptitle('GAZEBO Simulation Analysis: USV_02 vs USV_03\n(2026-02-10)', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    gs = gridspec.GridSpec(3, 4, hspace=0.35, wspace=0.3,
                           left=0.05, right=0.97, top=0.93, bottom=0.05)
    
    # 轨迹 (占2格)
    ax_traj = fig.add_subplot(gs[0, 0:2])
    plot_trajectory(ax_traj, df03, df02)
    
    # 速度
    ax_spd = fig.add_subplot(gs[0, 2])
    plot_speed(ax_spd, df03, df02)
    
    # 速度命令
    ax_cmdvx = fig.add_subplot(gs[0, 3])
    plot_cmd_vx(ax_cmdvx, df03, df02)
    
    # 航向角
    ax_yaw = fig.add_subplot(gs[1, 0])
    plot_yaw(ax_yaw, df03, df02)
    
    # 航向误差
    ax_herr = fig.add_subplot(gs[1, 1])
    plot_heading_error(ax_herr, df03, df02)
    
    # 横向跟踪误差
    ax_cte = fig.add_subplot(gs[1, 2])
    plot_cross_track_error(ax_cte, df03, df02)
    
    # 到目标距离
    ax_d2g = fig.add_subplot(gs[1, 3])
    plot_distance_to_goal(ax_d2g, df03, df02)
    
    # 角速度命令
    ax_omega = fig.add_subplot(gs[2, 0])
    plot_cmd_omega(ax_omega, df03, df02)
    
    # MPC求解时间
    ax_mpc = fig.add_subplot(gs[2, 1])
    plot_mpc_solve_time(ax_mpc, df03, df02)
    
    # AMPC Tau
    ax_tau = fig.add_subplot(gs[2, 2])
    plot_ampc_tau(ax_tau, df03, df02)
    
    # 指标汇总表
    ax_table = fig.add_subplot(gs[2, 3])
    ax_table.axis('off')
    key_metrics = [
        ('任务时长(s)', m03.get('任务时长(s)', 0), m02.get('任务时长(s)', 0)),
        ('总航程(m)', m03.get('总航程(m)', 0), m02.get('总航程(m)', 0)),
        ('平均速度(m/s)', m03.get('平均速度(m/s)', 0), m02.get('平均速度(m/s)', 0)),
        ('平均CTE(m)', m03.get('平均CTE(m)', 0), m02.get('平均CTE(m)', 0)),
        ('平均航向误差(°)', m03.get('平均航向误差(deg)', 0), m02.get('平均航向误差(deg)', 0)),
        ('MPC均值(ms)', m03.get('MPC平均求解时间(ms)', 0), m02.get('MPC平均求解时间(ms)', 0)),
    ]
    cell_text = [[f"{v03:.3f}", f"{v02:.3f}"] for _, v03, v02 in key_metrics]
    row_labels = [k for k, _, _ in key_metrics]
    table = ax_table.table(cellText=cell_text, rowLabels=row_labels,
                           colLabels=['USV_03', 'USV_02'],
                           cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.4)
    ax_table.set_title('Key Metrics Summary', fontsize=10, fontweight='bold')
    
    output_path = os.path.join(OUTPUT_DIR, "gazebo_sim_comparison.png")
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n[+] 综合对比图已保存: {output_path}")
    
    # ============================================================
    # 图 2: 航向误差 & CTE 分布直方图
    # ============================================================
    fig2, axes2 = plt.subplots(1, 3, figsize=(18, 5))
    fig2.suptitle('Error Distribution Comparison', fontsize=14, fontweight='bold')
    
    # 航向误差分布
    axes2[0].hist(df03['heading_error_deg'].dropna(), bins=50, alpha=0.6, 
                  color='blue', label='USV_03', density=True)
    axes2[0].hist(df02['heading_error_deg'].dropna(), bins=50, alpha=0.6, 
                  color='red', label='USV_02', density=True)
    axes2[0].set_xlabel('Heading Error (deg)')
    axes2[0].set_ylabel('Density')
    axes2[0].set_title('Heading Error Distribution')
    axes2[0].legend()
    axes2[0].grid(True, alpha=0.3)
    
    # CTE分布
    axes2[1].hist(df03['cross_track_error'].dropna(), bins=50, alpha=0.6, 
                  color='blue', label='USV_03', density=True)
    axes2[1].hist(df02['cross_track_error'].dropna(), bins=50, alpha=0.6, 
                  color='red', label='USV_02', density=True)
    axes2[1].set_xlabel('Cross-Track Error (m)')
    axes2[1].set_ylabel('Density')
    axes2[1].set_title('CTE Distribution')
    axes2[1].legend()
    axes2[1].grid(True, alpha=0.3)
    
    # 速度分布
    axes2[2].hist(df03['velocity_speed'].dropna(), bins=50, alpha=0.6, 
                  color='blue', label='USV_03', density=True)
    axes2[2].hist(df02['velocity_speed'].dropna(), bins=50, alpha=0.6, 
                  color='red', label='USV_02', density=True)
    axes2[2].set_xlabel('Speed (m/s)')
    axes2[2].set_ylabel('Density')
    axes2[2].set_title('Speed Distribution')
    axes2[2].legend()
    axes2[2].grid(True, alpha=0.3)
    
    fig2.tight_layout()
    output_path2 = os.path.join(OUTPUT_DIR, "error_distribution.png")
    fig2.savefig(output_path2, dpi=150, bbox_inches='tight')
    print(f"[+] 误差分布图已保存: {output_path2}")
    
    # ============================================================
    # 图 3: 分航点性能分析
    # ============================================================
    goal_ids_03 = sorted(df03['goal_id'].unique())
    goal_ids_02 = sorted(df02['goal_id'].unique())
    all_goals = sorted(set(goal_ids_03) | set(goal_ids_02))
    
    if len(all_goals) > 1:
        fig3, axes3 = plt.subplots(2, 2, figsize=(14, 10))
        fig3.suptitle('Per-Waypoint Performance', fontsize=14, fontweight='bold')
        
        # 每个航点的平均CTE
        cte_03 = [df03[df03['goal_id'] == g]['cross_track_error'].abs().mean() for g in all_goals]
        cte_02 = [df02[df02['goal_id'] == g]['cross_track_error'].abs().mean() for g in all_goals]
        x = np.arange(len(all_goals))
        w = 0.35
        axes3[0, 0].bar(x - w/2, cte_03, w, label='USV_03', color='blue', alpha=0.7)
        axes3[0, 0].bar(x + w/2, cte_02, w, label='USV_02', color='red', alpha=0.7)
        axes3[0, 0].set_xlabel('Goal ID')
        axes3[0, 0].set_ylabel('Mean |CTE| (m)')
        axes3[0, 0].set_title('Mean CTE per Waypoint')
        axes3[0, 0].set_xticks(x)
        axes3[0, 0].set_xticklabels([str(g) for g in all_goals])
        axes3[0, 0].legend()
        axes3[0, 0].grid(True, alpha=0.3)
        
        # 每个航点的平均航向误差
        herr_03 = [df03[df03['goal_id'] == g]['heading_error_deg'].abs().mean() for g in all_goals]
        herr_02 = [df02[df02['goal_id'] == g]['heading_error_deg'].abs().mean() for g in all_goals]
        axes3[0, 1].bar(x - w/2, herr_03, w, label='USV_03', color='blue', alpha=0.7)
        axes3[0, 1].bar(x + w/2, herr_02, w, label='USV_02', color='red', alpha=0.7)
        axes3[0, 1].set_xlabel('Goal ID')
        axes3[0, 1].set_ylabel('Mean |Heading Error| (deg)')
        axes3[0, 1].set_title('Mean Heading Error per Waypoint')
        axes3[0, 1].set_xticks(x)
        axes3[0, 1].set_xticklabels([str(g) for g in all_goals])
        axes3[0, 1].legend()
        axes3[0, 1].grid(True, alpha=0.3)
        
        # 每个航点的平均速度
        spd_03 = [df03[df03['goal_id'] == g]['velocity_speed'].mean() for g in all_goals]
        spd_02 = [df02[df02['goal_id'] == g]['velocity_speed'].mean() for g in all_goals]
        axes3[1, 0].bar(x - w/2, spd_03, w, label='USV_03', color='blue', alpha=0.7)
        axes3[1, 0].bar(x + w/2, spd_02, w, label='USV_02', color='red', alpha=0.7)
        axes3[1, 0].set_xlabel('Goal ID')
        axes3[1, 0].set_ylabel('Mean Speed (m/s)')
        axes3[1, 0].set_title('Mean Speed per Waypoint')
        axes3[1, 0].set_xticks(x)
        axes3[1, 0].set_xticklabels([str(g) for g in all_goals])
        axes3[1, 0].legend()
        axes3[1, 0].grid(True, alpha=0.3)
        
        # 每个航点的用时
        time_03 = []
        time_02 = []
        for g in all_goals:
            seg03 = df03[df03['goal_id'] == g]
            seg02 = df02[df02['goal_id'] == g]
            time_03.append(seg03['time_s'].iloc[-1] - seg03['time_s'].iloc[0] if len(seg03) > 1 else 0)
            time_02.append(seg02['time_s'].iloc[-1] - seg02['time_s'].iloc[0] if len(seg02) > 1 else 0)
        axes3[1, 1].bar(x - w/2, time_03, w, label='USV_03', color='blue', alpha=0.7)
        axes3[1, 1].bar(x + w/2, time_02, w, label='USV_02', color='red', alpha=0.7)
        axes3[1, 1].set_xlabel('Goal ID')
        axes3[1, 1].set_ylabel('Duration (s)')
        axes3[1, 1].set_title('Time per Waypoint')
        axes3[1, 1].set_xticks(x)
        axes3[1, 1].set_xticklabels([str(g) for g in all_goals])
        axes3[1, 1].legend()
        axes3[1, 1].grid(True, alpha=0.3)
        
        fig3.tight_layout()
        output_path3 = os.path.join(OUTPUT_DIR, "per_waypoint_analysis.png")
        fig3.savefig(output_path3, dpi=150, bbox_inches='tight')
        print(f"[+] 分航点分析图已保存: {output_path3}")
    
    # ============================================================
    # 输出分析总结
    # ============================================================
    print(f"\n{'='*60}")
    print("  分析结论")
    print(f"{'='*60}")
    
    # 比较关键指标
    dt03 = m03['任务时长(s)']
    dt02 = m02['任务时长(s)']
    faster = 'USV_03' if dt03 < dt02 else 'USV_02'
    print(f"  [任务效率] {faster} 更快完成任务 "
          f"(USV_03: {dt03:.1f}s, USV_02: {dt02:.1f}s, 差异: {abs(dt03-dt02):.1f}s)")
    
    cte03 = m03['平均CTE(m)']
    cte02 = m02['平均CTE(m)']
    better_cte = 'USV_03' if cte03 < cte02 else 'USV_02'
    print(f"  [路径跟踪] {better_cte} 横向跟踪更精确 "
          f"(USV_03: {cte03:.4f}m, USV_02: {cte02:.4f}m)")
    
    he03 = m03['平均航向误差(deg)']
    he02 = m02['平均航向误差(deg)']
    better_he = 'USV_03' if he03 < he02 else 'USV_02'
    print(f"  [航向控制] {better_he} 航向控制更精确 "
          f"(USV_03: {he03:.2f}°, USV_02: {he02:.2f}°)")
    
    spd03 = m03['平均速度(m/s)']
    spd02 = m02['平均速度(m/s)']
    print(f"  [速度表现] USV_03: {spd03:.4f}m/s, USV_02: {spd02:.4f}m/s")
    
    mpc03 = m03.get('MPC平均求解时间(ms)', 0)
    mpc02 = m02.get('MPC平均求解时间(ms)', 0)
    if mpc03 > 0 and mpc02 > 0:
        better_mpc = 'USV_03' if mpc03 < mpc02 else 'USV_02'
        print(f"  [MPC性能] {better_mpc} MPC求解更快 "
              f"(USV_03: {mpc03:.2f}ms, USV_02: {mpc02:.2f}ms)")
    
    ampc03 = m03.get('AMPC收敛率(%)', None)
    ampc02 = m02.get('AMPC收敛率(%)', None)
    if ampc03 is not None and ampc02 is not None:
        print(f"  [AMPC] USV_03收敛率: {ampc03:.1f}%, USV_02收敛率: {ampc02:.1f}%")
    
    print(f"\n[+] 所有分析图表已保存至: {OUTPUT_DIR}")
    print("=" * 60)
    
    plt.show()


if __name__ == '__main__':
    main()
