import json
import sys

for seed in ["1460", "1461"]:
    path = f"/tmp/fresh443_1460_1461_r1/repeat1_seed{seed}/seed{seed}.json"
    try:
        with open(path) as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"File not found: {path}")
        continue
    
    print(f"--- Seed {seed} Final Agent Metrics ---")
    metrics = data.get("final_agent_metrics", {})
    for agent, m in metrics.items():
        p = m.get("progress", 0)
        dg = m.get("dgoal", 0)
        vx = m.get("vx", 0)
        om = m.get("omega", 0)
        print(f"{agent}: progress={p:.4f}, dgoal={dg:.4f}, vx={vx:.4f}, omega={om:.4f}")
    
    if seed == "1461":
        print(f"\n--- Seed 1461 usv_02 Trace (step >= 700) ---")
        trace = data.get("trace", [])
        for entry in trace:
            if entry.get("agent_id") == "usv_02" and entry.get("step", 0) >= 700:
                s = entry.get("step")
                vx = entry.get("vx", 0)
                om = entry.get("omega", 0)
                pr = entry.get("progress", 0)
                x = entry.get("x", 0)
                y = entry.get("y", 0)
                print(f"step={s}: x={x:.2f}, y={y:.2f}, prog={pr:.4f}, vx={vx:.4f}, omega={om:.4f}")
