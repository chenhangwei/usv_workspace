import json
import os

def analyze_seed1460(file_path):
    print(f"--- Analysis for {os.path.basename(file_path)} ---")
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    # Assuming one episode per file
    samples = data['episode_metrics'][0]['trace_samples']
    
    agents = sorted(samples[0]['agents'].keys())
    
    # 1) Max abs raw CTE for each agent
    print("\n1) Max abs raw CTE for each agent:")
    for agent_id in agents:
        max_abs_cte = -1
        best_sample = None
        for s in samples:
            if agent_id in s['agents']:
                val = abs(s['agents'][agent_id]['raw_cross_track_error'])
                if val > max_abs_cte:
                    max_abs_cte = val
                    best_sample = s
        
        if best_sample:
            a_data = best_sample['agents'][agent_id]
            print(f"Agent {agent_id}: Max Abs CTE {max_abs_cte:.4f} at Step {best_sample['step']}, "
                  f"Linear: {a_data['final_linear_x']:.4f}, Omega: {a_data['final_angular_z']:.4f}, "
                  f"Progress: {a_data['route_progress']:.4f}")

    # 2) usv_02 filtered table
    print("\n2) usv_02 filter (step >= 560 or abs(raw_cte) > 4.5):")
    print(f"{'Step':>6} | {'Progress':>8} | {'Raw CTE':>8} | {'Overflow':>8} | {'Linear':>8} | {'Omega':>8} | {'NearDist':>8}")
    count = 0
    for s in samples:
        if 'usv_02' in s['agents']:
            a_data = s['agents']['usv_02']
            raw_cte = a_data['raw_cross_track_error']
            if s['step'] >= 560 or abs(raw_cte) > 4.5:
                print(f"{s['step']:6d} | {a_data['route_progress']:8.4f} | {raw_cte:8.4f} | "
                      f"{a_data.get('cross_track_overflow', 0):8.4f} | {a_data['final_linear_x']:8.4f} | "
                      f"{a_data['final_angular_z']:8.4f} | {a_data['nearest_distance']:8.4f}")
                count += 1
                if count >= 25:
                    break

def analyze_seed1461(file_path):
    print(f"\n--- Analysis for {os.path.basename(file_path)} ---")
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    samples = data['episode_metrics'][0]['trace_samples']
    
    # Find global min nearest_distance
    min_dist = float('inf')
    min_idx = -1
    min_agent = None
    
    for i, s in enumerate(samples):
        for agent_id, a_data in s['agents'].items():
            if a_data['nearest_distance'] < min_dist:
                min_dist = a_data['nearest_distance']
                min_idx = i
                min_agent = agent_id
    
    print(f"\n3) Global minimum nearest_distance: {min_dist:.4f} (Agent {min_agent} at Step {samples[min_idx]['step']})")
    print(f"{'Step':>6} | {'Agent':>8} | {'NearDist':>8} | {'Raw CTE':>8} | {'Linear':>8} | {'Omega':>8} | {'Progress':>8}")
    
    start = max(0, min_idx - 3)
    end = min(len(samples), min_idx + 4)
    
    for i in range(start, end):
        s = samples[i]
        # The request asks to list step, agent, nearest_distance, etc. 
        # Since each step has multiple agents, I'll show the data for the 'min_agent' at those steps.
        # Or maybe all agents? Usually "near minimum" refers to the event. I'll stick to min_agent.
        if min_agent in s['agents']:
            a_data = s['agents'][min_agent]
            print(f"{s['step']:6d} | {min_agent:8} | {a_data['nearest_distance']:8.4f} | "
                  f"{a_data['raw_cross_track_error']:8.4f} | {a_data['final_linear_x']:8.4f} | "
                  f"{a_data['final_angular_z']:8.4f} | {a_data['route_progress']:8.4f}")

if __name__ == '__main__':
    analyze_seed1460('/tmp/fresh489_1460_1461_r1/repeat1_seed1460/seed1460.json')
    analyze_seed1461('/tmp/fresh489_1460_1461_r1/repeat1_seed1461/seed1461.json')
