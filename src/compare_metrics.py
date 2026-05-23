import json
import numpy as np
import os

def analyze_json(file_path):
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return

    with open(file_path, 'r') as f:
        data = json.load(f)

    metrics_list = data.get("episode_metrics", [])
    if not metrics_list:
        print(f"No episode_metrics in {file_path}")
        return
    
    metrics = metrics_list[0]
    trace_samples = metrics.get("trace_samples", [])

    print(f"--- Analysis for {file_path} ---")
    print(f"Collision: {metrics.get('collision')}")
    print(f"Success: {metrics.get('success')}")
    print(f"Timeout: {metrics.get('timeout')}")
    print(f"Team Goal Progress Ratio: {metrics.get('team_goal_progress_ratio')}")
    print(f"Min Team Separation: {metrics.get('min_team_separation', metrics.get('episode_min_separation'))}")
    print(f"Steps: {metrics.get('steps')}")

    if not trace_samples:
        print("No trace_samples found in this file.")
        print("\n")
        return

    # Aggregation per 100 steps
    intervals = range(0, metrics.get('steps', 0) + 100, 100)
    for i in range(len(intervals) - 1):
        start, end = intervals[i], intervals[i+1]
        samples = [s for s in trace_samples if start <= s['step'] < end]
        if not samples:
            continue
        
        all_agents = []
        for s in samples:
            if 'agents' in s and isinstance(s['agents'], dict):
                for aid, a_data in s['agents'].items():
                    all_agents.append(a_data)
        
        if not all_agents:
            continue

        avg_linear_x = np.mean([a.get('final_linear_x', 0) for a in all_agents])
        avg_angular_z = np.mean([a.get('final_angular_z', 0) for a in all_agents])
        avg_dist = np.mean([a.get('distance_to_goal', 0) for a in all_agents])
        avg_cte = np.mean([a.get('cross_track_error', 0) for a in all_agents])
        avg_prog = np.mean([a.get('route_progress', 0) for a in all_agents])
        
        min_sep = min([s.get('pairwise_min_separation', float('inf')) for s in samples if s.get('pairwise_min_separation') is not None] or [float('nan')])

        print(f"Steps {start}-{end}: linear_x={avg_linear_x:.3f}, angular_z={avg_angular_z:.3f}, dist={avg_dist:.3f}, cte={avg_cte:.3f}, progress={avg_prog:.3f}, max_threat=0, min_sep={min_sep}")

    print("\nLast 10 Trace Samples (per agent):")
    if trace_samples:
        latest_sample = None
        for s in reversed(trace_samples):
            if 'agents' in s and s['agents']:
                latest_sample = s
                break
        
        if latest_sample:
            agent_ids = sorted(latest_sample['agents'].keys())
            for aid in agent_ids:
                print(f"Agent {aid}:")
                count = 0
                for s in reversed(trace_samples):
                    if 'agents' in s and aid in s['agents']:
                        a = s['agents'][aid]
                        yaw_err = a.get('raw_route_yaw_error', a.get('route_yaw_error', 'N/A'))
                        sep = s.get('pairwise_min_separation', 'N/A')
                        print(f"  Step {s['step']}: x={a.get('final_linear_x', 0):.3f}, z={a.get('final_angular_z', 0):.3f}, dist={a.get('distance_to_goal', 0):.3f}, cte={a.get('cross_track_error', 0):.3f}, prog={a.get('route_progress', 0):.3f}, yaw_err={yaw_err}, threat=N/A, sep={sep}")
                        count += 1
                    if count >= 10:
                        break
    print("\n")

analyze_json("/tmp/fresh367_guard1463_stride20_700/repeat1_seed1463/seed1463.json")
analyze_json("/tmp/fresh368_guard1463_r1/repeat1_seed1463/seed1463.json")
