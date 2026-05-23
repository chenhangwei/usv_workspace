import json
import os

seeds = ["1460", "1461"]
base_dir = "/tmp/fresh444_1460_1461_r1"

for seed in seeds:
    json_path = os.path.join(base_dir, f"repeat1_seed{seed}", f"seed{seed}.json")
    if not os.path.exists(json_path):
        print(f"File {json_path} not found")
        continue
    
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    scenario_data = data['scenario_summaries']['three_usv_random_encounter']
    metrics = scenario_data.get('final_agent_metrics', {})
    
    print(f"\n--- Seed {seed} Final Agent Metrics ---")
    for agent, m in metrics.items():
        print(f"{agent}: progress={m.get('route_progress', 0):.4f}, dgoal={m.get('distance_to_goal', 0):.4f}, vx={m.get('final_linear_x', 0):.4f}, omega={m.get('final_angular_z', 0):.4f}")

    if seed == "1461":
        print(f"\n--- Seed 1461 usv_02 Trace (step >= 700) ---")
        trace_samples = scenario_data.get('trace_samples', [])
        for sample in trace_samples:
            if sample.get('step', 0) >= 700:
                step = sample.get('step')
                u2 = sample.get('agents', {}).get('usv_02', {})
                prog = u2.get('route_progress', 0)
                dg = u2.get('distance_to_goal', 0)
                vx = u2.get('final_linear_x', 0)
                omega = u2.get('final_angular_z', 0)
                print(f"step={step}: progress={prog:.4f}, dgoal={dg:.4f}, vx={vx:.4f}, omega={omega:.4f}")

