#!/bin/bash
# headon27 训练监控脚本
# 用法: bash scripts/monitor_h27.sh
set -euo pipefail

echo "=== headon27 训练监控 ==="

# 检查训练进程
if pgrep -la python3 | grep -q train_mappo; then
  echo "状态: 🟢 训练进行中"
else
  echo "状态: 🔴 训练已停止"
fi

# 合并两份日志
python3 << 'EOF'
import re, os

records = []
for logpath in ['/tmp/headon27_train.log', '/tmp/headon27_train_r.log']:
    try:
        with open(logpath) as f:
            for line in f:
                m = re.search(r'\[MAPPO\]\[update=(\d+)\].*total_steps=(\d+).*loss=([\d.]+).*mean_reward=([-\d.]+).*episodes=(\d+)', line)
                if m:
                    upd, steps, loss, reward, eps = int(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4)), int(m.group(5))
                    if not records or upd > records[-1][0]:
                        records.append((upd, steps, loss, reward, eps))
    except FileNotFoundError:
        pass

if not records:
    print("未找到训练日志")
    exit(0)

print(f"更新数: {len(records)}  |  步数: {records[-1][1]:,} / 300,000 ({records[-1][1]*100/300000:.1f}%)")

# 10K step blocks
step_blocks = {}
for r in records:
    block = (r[1] // 10000) * 10
    if block not in step_blocks:
        step_blocks[block] = []
    step_blocks[block].append(r)

print(f"\n{'步数段':>12} | {'更新':>4} | {'平均reward':>10} | {'范围':>16} | {'平均loss':>8}")
print("-" * 65)
for block_k in sorted(step_blocks):
    br = step_blocks[block_k]
    rewards = [r[3] for r in br]
    losses = [r[2] for r in br]
    label = f"{block_k}K-{block_k+10}K"
    print(f"{label:>12} | {len(br):4d} | {sum(rewards)/len(rewards):+10.3f} | [{min(rewards):+.1f},{max(rewards):+.1f}] | {sum(losses)/len(losses):8.0f}")

# Moving averages
for w in [10, 20]:
    if len(records) >= w:
        last = [r[3] for r in records[-w:]]
        print(f"\n最近{w}次更新平均reward: {sum(last)/len(last):+.3f}")

# Checkpoints
ckpt_dir = '/mnt/data/checkpoints/usv_rl/headon27_checkpoints'
if os.path.isdir(ckpt_dir):
    ckpts = sorted([f for f in os.listdir(ckpt_dir) if f.endswith('.pt')])
    print(f"\n检查点: {len(ckpts)} 个")
    for c in ckpts:
        print(f"  {c}")

# ETA
avg_sps = 22
remaining = 300000 - records[-1][1]
hours = remaining / avg_sps / 3600
print(f"\n预估剩余: ~{hours:.1f}小时 (按{avg_sps} sps)")
EOF
